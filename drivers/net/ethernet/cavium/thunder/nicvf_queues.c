/*
 * Copyright (C) 2015 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/ip.h>
#include <linux/etherdevice.h>

#include "nic_reg.h"
#include "nic.h"
#include "q_struct.h"
#include "nicvf_queues.h"

struct rbuf_info {
	void	*data;
	u64	offset;
};

#define GET_RBUF_INFO(x) ((struct rbuf_info *)(x - NICVF_RCV_BUF_ALIGN_BYTES))

/* Poll a register for a specific value */
static int nicvf_poll_reg(struct nicvf *nic, int qidx,
			  u64 reg, int bit_pos, int bits, int val)
{
	u64 bit_mask;
	u64 reg_val;
	int timeout = 10;

	bit_mask = (1ULL << bits) - 1;
	bit_mask = (bit_mask << bit_pos);

	while (timeout) {
		reg_val = nicvf_queue_reg_read(nic, reg, qidx);
		if (((reg_val & bit_mask) >> bit_pos) == val)
			return 0;
		usleep_range(1000, 2000);
		timeout--;
	}
	netdev_err(nic->netdev, "Poll on reg 0x%llx failed\n", reg);
	return 1;
}

/* Allocate memory for a queue's descriptors */
static int nicvf_alloc_q_desc_mem(struct nicvf *nic, struct q_desc_mem *dmem,
				  int q_len, int desc_size, int align_bytes)
{
	dmem->q_len = q_len;
	dmem->size = (desc_size * q_len) + align_bytes;
	/* Save address, need it while freeing */
	dmem->unalign_base = dma_zalloc_coherent(&nic->pdev->dev, dmem->size,
						&dmem->dma, GFP_KERNEL);
	if (!dmem->unalign_base)
		return -1;

	/* Align memory address for 'align_bytes' */
	dmem->phys_base = NICVF_ALIGNED_ADDR((u64)dmem->dma, align_bytes);
	dmem->base = (void *)((u8 *)dmem->unalign_base +
			      (dmem->phys_base - dmem->dma));
	return 0;
}

/* Free queue's descriptor memory */
static void nicvf_free_q_desc_mem(struct nicvf *nic, struct q_desc_mem *dmem)
{
	if (!dmem)
		return;

	dma_free_coherent(&nic->pdev->dev, dmem->size,
			  dmem->unalign_base, dmem->dma);
	dmem->unalign_base = NULL;
	dmem->base = NULL;
}

/* Allocate buffer for packet reception
 * HW returns memory address where packet is DMA'ed but not a pointer
 * into RBDR ring, so save buffer address at the start of fragment and
 * align the start address to a cache aligned address
 */
static inline int nicvf_alloc_rcv_buffer(struct nicvf *nic,
					 u64 buf_len, u64 **rbuf)
{
	u64 data;
	struct rbuf_info *rinfo;

	/* SKB is built after packet is received */
	data = (u64)netdev_alloc_frag(buf_len);
	if (!data) {
		netdev_err(nic->netdev, "Failed to allocate new rcv buffer\n");
		return -ENOMEM;
	}

	/* Align buffer addr to cache line i.e 128 bytes */
	rinfo = (struct rbuf_info *)(data + NICVF_RCV_BUF_ALIGN_LEN(data));
	/* Store start address for later retrieval */
	rinfo->data = (void *)data;
	/* Store alignment offset */
	rinfo->offset = NICVF_RCV_BUF_ALIGN_LEN(data);

	data += rinfo->offset;

	/* Give next aligned address to hw for DMA */
	*rbuf = (u64 *)(data + NICVF_RCV_BUF_ALIGN_BYTES);
	return 0;
}

/* Retrieve actual buffer start address and build skb for received packet */
static struct sk_buff *nicvf_rb_ptr_to_skb(struct nicvf *nic, u64 rb_ptr)
{
	struct sk_buff *skb;
	struct rbuf_info *rinfo;

	rb_ptr = (u64)phys_to_virt(rb_ptr);
	/* Get buffer start address and alignment offset */
	rinfo = GET_RBUF_INFO(rb_ptr);

	/* Now build an skb to give to stack */
	skb = build_skb(rinfo->data, RCV_FRAG_LEN);
	if (!skb) {
		put_page(virt_to_head_page(rinfo->data));
		return NULL;
	}
	/* Set correct skb->data */
	skb_reserve(skb, rinfo->offset + NICVF_RCV_BUF_ALIGN_BYTES);

	prefetch((void *)rb_ptr);
	return skb;
}

/* Allocate RBDR ring and populate receive buffers */
static int  nicvf_init_rbdr(struct nicvf *nic, struct rbdr *rbdr,
			    int ring_len, int buf_size)
{
	int idx;
	u64 *rbuf;
	struct rbdr_entry_t *desc;

	if (nicvf_alloc_q_desc_mem(nic, &rbdr->dmem, ring_len,
				   sizeof(struct rbdr_entry_t),
				   NICVF_RCV_BUF_ALIGN_BYTES)) {
		netdev_err(nic->netdev,
			   "Unable to allocate memory for rcv buffer ring\n");
		return -ENOMEM;
	}

	rbdr->desc = rbdr->dmem.base;
	/* Buffer size has to be in multiples of 128 bytes */
	rbdr->dma_size = buf_size;
	rbdr->enable = true;
	rbdr->thresh = RBDR_THRESH;

	for (idx = 0; idx < ring_len; idx++) {
		if (nicvf_alloc_rcv_buffer(nic, RCV_FRAG_LEN, &rbuf))
			return -ENOMEM;

		desc = GET_RBDR_DESC(rbdr, idx);
		desc->buf_addr = virt_to_phys(rbuf) >> NICVF_RCV_BUF_ALIGN;
	}
	return 0;
}

/* Free RBDR ring and its receive buffers */
static void nicvf_free_rbdr(struct nicvf *nic, struct rbdr *rbdr)
{
	int head, tail;
	u64 buf_addr;
	struct rbdr_entry_t *desc;
	struct rbuf_info *rinfo;

	if (!rbdr)
		return;

	rbdr->enable = false;
	if (!rbdr->dmem.base)
		return;

	head = rbdr->head;
	tail = rbdr->tail;

	/* Free SKBs */
	while (head != tail) {
		desc = GET_RBDR_DESC(rbdr, head);
		buf_addr = desc->buf_addr << NICVF_RCV_BUF_ALIGN;
		rinfo = GET_RBUF_INFO((u64)phys_to_virt(buf_addr));
		put_page(virt_to_head_page(rinfo->data));
		head++;
		head &= (rbdr->dmem.q_len - 1);
	}
	/* Free SKB of tail desc */
	desc = GET_RBDR_DESC(rbdr, tail);
	buf_addr = desc->buf_addr << NICVF_RCV_BUF_ALIGN;
	rinfo = GET_RBUF_INFO((u64)phys_to_virt(buf_addr));
	put_page(virt_to_head_page(rinfo->data));

	/* Free RBDR ring */
	nicvf_free_q_desc_mem(nic, &rbdr->dmem);
}

/* Refill receive buffer descriptors with new buffers.
 * This runs in softirq context .
 */
void nicvf_refill_rbdr(unsigned long data)
{
	struct nicvf *nic = (struct nicvf *)data;
	struct queue_set *qs = nic->qs;
	int rbdr_idx = qs->rbdr_cnt;
	int tail, qcount;
	int refill_rb_cnt;
	struct rbdr *rbdr;
	struct rbdr_entry_t *desc;
	u64 *rbuf;

refill:
	if (!rbdr_idx)
		return;
	rbdr_idx--;
	rbdr = &qs->rbdr[rbdr_idx];
	/* Check if it's enabled */
	if (!rbdr->enable)
		goto next_rbdr;

	/* Get no of desc's to be refilled */
	qcount = nicvf_queue_reg_read(nic, NIC_QSET_RBDR_0_1_STATUS0, rbdr_idx);
	qcount &= 0x7FFFF;

	/* Get no of desc's to be refilled */
	refill_rb_cnt = qs->rbdr_len - qcount - 1;

	/* Start filling descs from tail */
	tail = nicvf_queue_reg_read(nic, NIC_QSET_RBDR_0_1_TAIL, rbdr_idx) >> 3;
	while (refill_rb_cnt) {
		tail++;
		tail &= (rbdr->dmem.q_len - 1);

		if (nicvf_alloc_rcv_buffer(nic, RCV_FRAG_LEN, &rbuf))
			break;

		desc = GET_RBDR_DESC(rbdr, tail);
		desc->buf_addr = virt_to_phys(rbuf) >> NICVF_RCV_BUF_ALIGN;
		refill_rb_cnt--;
	}
	/* Notify HW */
	nicvf_queue_reg_write(nic, NIC_QSET_RBDR_0_1_DOOR,
			      rbdr_idx, qs->rbdr_len - qcount - 1);
next_rbdr:
	if (rbdr_idx)
		goto refill;

	/* Re-enable RBDR interrupts */
	for (rbdr_idx = 0; rbdr_idx < qs->rbdr_cnt; rbdr_idx++)
		nicvf_enable_intr(nic, NICVF_INTR_RBDR, rbdr_idx);
}

/* Initialize completion queue */
static int nicvf_init_cmp_queue(struct nicvf *nic,
				struct cmp_queue *cq, int q_len)
{
	if (nicvf_alloc_q_desc_mem(nic, &cq->dmem, q_len,
				   CMP_QUEUE_DESC_SIZE,
				   NICVF_CQ_BASE_ALIGN_BYTES)) {
		netdev_err(nic->netdev,
			   "Unable to allocate memory for completion queue\n");
		return -ENOMEM;
	}
	cq->desc = cq->dmem.base;
	cq->thresh = CMP_QUEUE_CQE_THRESH;
	nic->cq_coalesce_usecs = (CMP_QUEUE_TIMER_THRESH * 0.05) - 1;

	return 0;
}

static void nicvf_free_cmp_queue(struct nicvf *nic, struct cmp_queue *cq)
{
	if (!cq)
		return;
	if (!cq->dmem.base)
		return;

	nicvf_free_q_desc_mem(nic, &cq->dmem);
}

/* Initialize transmit queue */
static int nicvf_init_snd_queue(struct nicvf *nic,
				struct snd_queue *sq, int q_len)
{
	if (nicvf_alloc_q_desc_mem(nic, &sq->dmem, q_len,
				   SND_QUEUE_DESC_SIZE,
				   NICVF_SQ_BASE_ALIGN_BYTES)) {
		netdev_err(nic->netdev,
			   "Unable to allocate memory for send queue\n");
		return -ENOMEM;
	}

	sq->desc = sq->dmem.base;
	sq->skbuff = kcalloc(q_len, sizeof(u64), GFP_ATOMIC);
	sq->head = 0;
	sq->tail = 0;
	atomic_set(&sq->free_cnt, q_len - 1);
	sq->thresh = SND_QUEUE_THRESH;

	return 0;
}

static void nicvf_free_snd_queue(struct nicvf *nic, struct snd_queue *sq)
{
	if (!sq)
		return;
	if (!sq->dmem.base)
		return;

	kfree(sq->skbuff);
	nicvf_free_q_desc_mem(nic, &sq->dmem);
}

static void nicvf_reclaim_snd_queue(struct nicvf *nic,
				    struct queue_set *qs, int qidx)
{
	/* Disable send queue */
	nicvf_queue_reg_write(nic, NIC_QSET_SQ_0_7_CFG, qidx, 0);
	/* Check if SQ is stopped */
	if (nicvf_poll_reg(nic, qidx, NIC_QSET_SQ_0_7_STATUS, 21, 1, 0x01))
		return;
	/* Reset send queue */
	nicvf_queue_reg_write(nic, NIC_QSET_SQ_0_7_CFG, qidx, NICVF_SQ_RESET);
}

static void nicvf_reclaim_rcv_queue(struct nicvf *nic,
				    struct queue_set *qs, int qidx)
{
	struct nic_mbx mbx = {};

	/* Make sure all packets in the pipeline are written back into mem */
	mbx.msg = NIC_MBOX_MSG_RQ_SW_SYNC;
	mbx.data.rq.cfg = 0;
	nicvf_send_msg_to_pf(nic, &mbx);
}

static void nicvf_reclaim_cmp_queue(struct nicvf *nic,
				    struct queue_set *qs, int qidx)
{
	/* Disable timer threshold (doesn't get reset upon CQ reset */
	nicvf_queue_reg_write(nic, NIC_QSET_CQ_0_7_CFG2, qidx, 0);
	/* Disable completion queue */
	nicvf_queue_reg_write(nic, NIC_QSET_CQ_0_7_CFG, qidx, 0);
	/* Reset completion queue */
	nicvf_queue_reg_write(nic, NIC_QSET_CQ_0_7_CFG, qidx, NICVF_CQ_RESET);
}

static void nicvf_reclaim_rbdr(struct nicvf *nic,
			       struct rbdr *rbdr, int qidx)
{
	u64 tmp;
	int timeout = 10;

	/* Save head and tail pointers for feeing up buffers */
	rbdr->head = nicvf_queue_reg_read(nic,
					  NIC_QSET_RBDR_0_1_HEAD,
					  qidx) >> 3;
	rbdr->tail = nicvf_queue_reg_read(nic,
					  NIC_QSET_RBDR_0_1_TAIL,
					  qidx) >> 3;

	/* Disable RBDR */
	nicvf_queue_reg_write(nic, NIC_QSET_RBDR_0_1_CFG, qidx, 0);
	if (nicvf_poll_reg(nic, qidx, NIC_QSET_RBDR_0_1_STATUS0, 62, 2, 0x00))
		return;
	while (1) {
		tmp = nicvf_queue_reg_read(nic,
					   NIC_QSET_RBDR_0_1_PREFETCH_STATUS,
					   qidx);
		if ((tmp & 0xFFFFFFFF) == ((tmp >> 32) & 0xFFFFFFFF))
			break;
		usleep_range(1000, 2000);
		timeout--;
		if (!timeout) {
			netdev_err(nic->netdev,
				   "Failed polling on prefetch status\n");
			return;
		}
	}
	nicvf_queue_reg_write(nic, NIC_QSET_RBDR_0_1_CFG,
			      qidx, NICVF_RBDR_RESET);

	if (nicvf_poll_reg(nic, qidx, NIC_QSET_RBDR_0_1_STATUS0, 62, 2, 0x02))
		return;
	nicvf_queue_reg_write(nic, NIC_QSET_RBDR_0_1_CFG, qidx, 0x00);
	if (nicvf_poll_reg(nic, qidx, NIC_QSET_RBDR_0_1_STATUS0, 62, 2, 0x00))
		return;
}

/* Configures receive queue */
static void nicvf_rcv_queue_config(struct nicvf *nic, struct queue_set *qs,
				   int qidx, bool enable)
{
	struct nic_mbx mbx = {};
	struct rcv_queue *rq;
	struct cmp_queue *cq;
	struct rq_cfg rq_cfg;
#ifdef CONFIG_RPS
	struct netdev_rx_queue *rxqueue;
	struct rps_map *map, *oldmap;
	cpumask_var_t rps_mask;
	int i, cpu;
#endif

	rq = &qs->rq[qidx];
	rq->enable = enable;

	/* Disable receive queue */
	nicvf_queue_reg_write(nic, NIC_QSET_RQ_0_7_CFG, qidx, 0);

	if (!rq->enable) {
		nicvf_reclaim_rcv_queue(nic, qs, qidx);
		return;
	}

	rq->cq_qs = qs->vnic_id;
	rq->cq_idx = qidx;
	rq->start_rbdr_qs = qs->vnic_id;
	rq->start_qs_rbdr_idx = qs->rbdr_cnt - 1;
	rq->cont_rbdr_qs = qs->vnic_id;
	rq->cont_qs_rbdr_idx = qs->rbdr_cnt - 1;
	/* all writes of RBDR data to be loaded into L2 Cache as well*/
	rq->caching = 1;

	/* Send a mailbox msg to PF to config RQ */
	mbx.msg = NIC_MBOX_MSG_RQ_CFG;
	mbx.data.rq.qs_num = qs->vnic_id;
	mbx.data.rq.rq_num = qidx;
	mbx.data.rq.cfg = (rq->caching << 26) | (rq->cq_qs << 19) |
			  (rq->cq_idx << 16) | (rq->cont_rbdr_qs << 9) |
			  (rq->cont_qs_rbdr_idx << 8) |
			  (rq->start_rbdr_qs << 1) | (rq->start_qs_rbdr_idx);
	nicvf_send_msg_to_pf(nic, &mbx);

	mbx.msg = NIC_MBOX_MSG_RQ_BP_CFG;
	mbx.data.rq.cfg = (1ULL << 63) | (1ULL << 62) | (qs->vnic_id << 0);
	nicvf_send_msg_to_pf(nic, &mbx);

	/* RQ drop config
	 * Enable CQ drop to reserve sufficient CQEs for all tx packets
	 */
	mbx.msg = NIC_MBOX_MSG_RQ_DROP_CFG;
	mbx.data.rq.cfg = (1ULL << 62) | (RQ_CQ_DROP << 8);
	nicvf_send_msg_to_pf(nic, &mbx);

	nicvf_queue_reg_write(nic, NIC_QSET_RQ_GEN_CFG, qidx, 0x00);

	/* Enable Receive queue */
	rq_cfg.ena = 1;
	rq_cfg.tcp_ena = 0;
	nicvf_queue_reg_write(nic, NIC_QSET_RQ_0_7_CFG, qidx, *(u64 *)&rq_cfg);

#ifdef CONFIG_RPS
	cq = &qs->cq[qidx];
	/* Set RPS CPU map */
	cpumask_copy(rps_mask, cpu_online_mask);
	cpumask_clear_cpu(cpumask_first(&cq->affinity_mask), rps_mask);

	rxqueue = nic->netdev->_rx + qidx;
	oldmap = rcu_dereference(rxqueue->rps_map);

	map = kzalloc(max_t(unsigned int,
			    RPS_MAP_SIZE(cpumask_weight(rps_mask)),
			    L1_CACHE_BYTES), GFP_KERNEL);
	if (!map)
		return;

	i = 0;
	for_each_cpu_and(cpu, rps_mask, cpu_online_mask)
		map->cpus[i++] = cpu;
	map->len = i;

	static_key_slow_inc(&rps_needed);
	rcu_assign_pointer(rxqueue->rps_map, map);

	if (oldmap) {
		kfree_rcu(oldmap, rcu);
		static_key_slow_dec(&rps_needed);
	}
#endif
}

/* Configures completion queue */
void nicvf_cmp_queue_config(struct nicvf *nic, struct queue_set *qs,
			    int qidx, bool enable)
{
	struct cmp_queue *cq;
	struct cq_cfg cq_cfg;

	cq = &qs->cq[qidx];
	cq->enable = enable;

	if (!cq->enable) {
		nicvf_reclaim_cmp_queue(nic, qs, qidx);
		return;
	}

	/* Reset completion queue */
	nicvf_queue_reg_write(nic, NIC_QSET_CQ_0_7_CFG, qidx, NICVF_CQ_RESET);

	if (!cq->enable)
		return;

	spin_lock_init(&cq->lock);
	/* Set completion queue base address */
	nicvf_queue_reg_write(nic, NIC_QSET_CQ_0_7_BASE,
			      qidx, (u64)(cq->dmem.phys_base));

	/* Enable Completion queue */
	cq_cfg.ena = 1;
	cq_cfg.reset = 0;
	cq_cfg.caching = 0;
	cq_cfg.qsize = CMP_QSIZE;
	cq_cfg.avg_con = 0;
	nicvf_queue_reg_write(nic, NIC_QSET_CQ_0_7_CFG, qidx, *(u64 *)&cq_cfg);

	/* Set threshold value for interrupt generation */
	nicvf_queue_reg_write(nic, NIC_QSET_CQ_0_7_THRESH, qidx, cq->thresh);
	nicvf_queue_reg_write(nic, NIC_QSET_CQ_0_7_CFG2,
			      qidx, nic->cq_coalesce_usecs);
}

/* Configures transmit queue */
static void nicvf_snd_queue_config(struct nicvf *nic, struct queue_set *qs,
				   int qidx, bool enable)
{
	struct nic_mbx mbx = {};
	struct snd_queue *sq;
	struct sq_cfg sq_cfg;

	sq = &qs->sq[qidx];
	sq->enable = enable;

	if (!sq->enable) {
		nicvf_reclaim_snd_queue(nic, qs, qidx);
		return;
	}

	/* Reset send queue */
	nicvf_queue_reg_write(nic, NIC_QSET_SQ_0_7_CFG, qidx, NICVF_SQ_RESET);

	sq->cq_qs = qs->vnic_id;
	sq->cq_idx = qidx;

	/* Send a mailbox msg to PF to config SQ */
	mbx.msg = NIC_MBOX_MSG_SQ_CFG;
	mbx.data.sq.qs_num = qs->vnic_id;
	mbx.data.sq.sq_num = qidx;
	mbx.data.sq.cfg = (sq->cq_qs << 3) | sq->cq_idx;
	nicvf_send_msg_to_pf(nic, &mbx);

	/* Set queue base address */
	nicvf_queue_reg_write(nic, NIC_QSET_SQ_0_7_BASE,
			      qidx, (u64)(sq->dmem.phys_base));

	/* Enable send queue  & set queue size */
	sq_cfg.ena = 1;
	sq_cfg.reset = 0;
	sq_cfg.ldwb = 0;
	sq_cfg.qsize = SND_QSIZE;
	sq_cfg.tstmp_bgx_intf = 0;
	nicvf_queue_reg_write(nic, NIC_QSET_SQ_0_7_CFG, qidx, *(u64 *)&sq_cfg);

	/* Set threshold value for interrupt generation */
	nicvf_queue_reg_write(nic, NIC_QSET_SQ_0_7_THRESH, qidx, sq->thresh);

	/* Set queue:cpu affinity for better load distribution */
	if (cpu_online(qidx)) {
		cpumask_set_cpu(qidx, &sq->affinity_mask);
		netif_set_xps_queue(nic->netdev,
				    &sq->affinity_mask, qidx);
	}
}

/* Configures receive buffer descriptor ring */
static void nicvf_rbdr_config(struct nicvf *nic, struct queue_set *qs,
			      int qidx, bool enable)
{
	struct rbdr *rbdr;
	struct rbdr_cfg rbdr_cfg;

	rbdr = &qs->rbdr[qidx];
	nicvf_reclaim_rbdr(nic, rbdr, qidx);
	if (!enable)
		return;

	/* Set descriptor base address */
	nicvf_queue_reg_write(nic, NIC_QSET_RBDR_0_1_BASE,
			      qidx, (u64)(rbdr->dmem.phys_base));

	/* Enable RBDR  & set queue size */
	/* Buffer size should be in multiples of 128 bytes */
	rbdr_cfg.ena = 1;
	rbdr_cfg.reset = 0;
	rbdr_cfg.ldwb = 0;
	rbdr_cfg.qsize = RBDR_SIZE;
	rbdr_cfg.avg_con = 0;
	rbdr_cfg.lines = rbdr->dma_size / 128;
	nicvf_queue_reg_write(nic, NIC_QSET_RBDR_0_1_CFG,
			      qidx, *(u64 *)&rbdr_cfg);

	/* Notify HW */
	nicvf_queue_reg_write(nic, NIC_QSET_RBDR_0_1_DOOR,
			      qidx, qs->rbdr_len - 1);

	/* Set threshold value for interrupt generation */
	nicvf_queue_reg_write(nic, NIC_QSET_RBDR_0_1_THRESH,
			      qidx, rbdr->thresh - 1);
}

/* Requests PF to assign and enable Qset */
void nicvf_qset_config(struct nicvf *nic, bool enable)
{
	struct  nic_mbx mbx = {};
	struct queue_set *qs = nic->qs;
	struct qs_cfg *qs_cfg;

	if (!qs) {
		netdev_warn(nic->netdev,
			    "Qset is still not allocated, don't init queues\n");
		return;
	}

	qs->enable = enable;
	qs->vnic_id = nic->vf_id;

	/* Send a mailbox msg to PF to config Qset */
	mbx.msg = NIC_MBOX_MSG_QS_CFG;
	mbx.data.qs.num = qs->vnic_id;

	mbx.data.qs.cfg = 0;
	qs_cfg = (struct qs_cfg *)&mbx.data.qs.cfg;
	if (qs->enable) {
		qs_cfg->ena = 1;
#ifdef __BIG_ENDIAN
		qs_cfg->be = 1;
#endif
		qs_cfg->vnic = qs->vnic_id;
	}
	nicvf_send_msg_to_pf(nic, &mbx);
}

static void nicvf_free_resources(struct nicvf *nic)
{
	int qidx;
	struct queue_set *qs = nic->qs;

	/* Free receive buffer descriptor ring */
	for (qidx = 0; qidx < qs->rbdr_cnt; qidx++)
		nicvf_free_rbdr(nic, &qs->rbdr[qidx]);

	/* Free completion queue */
	for (qidx = 0; qidx < qs->cq_cnt; qidx++)
		nicvf_free_cmp_queue(nic, &qs->cq[qidx]);

	/* Free send queue */
	for (qidx = 0; qidx < qs->sq_cnt; qidx++)
		nicvf_free_snd_queue(nic, &qs->sq[qidx]);
}

static int nicvf_alloc_resources(struct nicvf *nic)
{
	int qidx;
	struct queue_set *qs = nic->qs;

	/* Alloc receive buffer descriptor ring */
	for (qidx = 0; qidx < qs->rbdr_cnt; qidx++) {
		if (nicvf_init_rbdr(nic, &qs->rbdr[qidx], qs->rbdr_len,
				    DMA_BUFFER_LEN))
			goto alloc_fail;
	}

	/* Alloc send queue */
	for (qidx = 0; qidx < qs->sq_cnt; qidx++) {
		if (nicvf_init_snd_queue(nic, &qs->sq[qidx], qs->sq_len))
			goto alloc_fail;
	}

	/* Alloc completion queue */
	for (qidx = 0; qidx < qs->cq_cnt; qidx++) {
		if (nicvf_init_cmp_queue(nic, &qs->cq[qidx], qs->cq_len))
			goto alloc_fail;
	}

	return 0;
alloc_fail:
	nicvf_free_resources(nic);
	return -ENOMEM;
}

int nicvf_set_qset_resources(struct nicvf *nic)
{
	struct queue_set *qs;

	qs = kzalloc(sizeof(*qs), GFP_ATOMIC);
	if (!qs)
		return -ENOMEM;
	nic->qs = qs;

	/* Set count of each queue */
	qs->rbdr_cnt = RBDR_CNT;
#ifdef VNIC_RSS_SUPPORT
	qs->rq_cnt = RCV_QUEUE_CNT;
#else
	qs->rq_cnt = 1;
#endif
	qs->sq_cnt = SND_QUEUE_CNT;
	qs->cq_cnt = CMP_QUEUE_CNT;

	/* Set queue lengths */
	qs->rbdr_len = RCV_BUF_COUNT;
	qs->sq_len = SND_QUEUE_LEN;
	qs->cq_len = CMP_QUEUE_LEN;
	return 0;
}

int nicvf_config_data_transfer(struct nicvf *nic, bool enable)
{
	bool disable = false;
	struct queue_set *qs = nic->qs;
	int qidx;

	if (!qs)
		return 0;

	if (enable) {
		if (nicvf_alloc_resources(nic))
			return -ENOMEM;

		for (qidx = 0; qidx < qs->sq_cnt; qidx++)
			nicvf_snd_queue_config(nic, qs, qidx, enable);
		for (qidx = 0; qidx < qs->cq_cnt; qidx++)
			nicvf_cmp_queue_config(nic, qs, qidx, enable);
		for (qidx = 0; qidx < qs->rbdr_cnt; qidx++)
			nicvf_rbdr_config(nic, qs, qidx, enable);
		for (qidx = 0; qidx < qs->rq_cnt; qidx++)
			nicvf_rcv_queue_config(nic, qs, qidx, enable);
	} else {
		for (qidx = 0; qidx < qs->rq_cnt; qidx++)
			nicvf_rcv_queue_config(nic, qs, qidx, disable);
		for (qidx = 0; qidx < qs->rbdr_cnt; qidx++)
			nicvf_rbdr_config(nic, qs, qidx, disable);
		for (qidx = 0; qidx < qs->sq_cnt; qidx++)
			nicvf_snd_queue_config(nic, qs, qidx, disable);
		for (qidx = 0; qidx < qs->cq_cnt; qidx++)
			nicvf_cmp_queue_config(nic, qs, qidx, disable);

		nicvf_free_resources(nic);
	}

	return 0;
}

/* Get a free desc from SQ
 * returns descriptor ponter & descriptor number
 */
static inline int nicvf_get_sq_desc(struct queue_set *qs, int qnum, void **desc)
{
	int qentry;
	struct snd_queue *sq = &qs->sq[qnum];

	if (!atomic_read(&sq->free_cnt))
		return 0;

	qentry = sq->tail++;
	atomic_dec(&sq->free_cnt);

	sq->tail &= (sq->dmem.q_len - 1);
	*desc = GET_SQ_DESC(sq, qentry);
	return qentry;
}

/* Free descriptor back to SQ for future use */
void nicvf_put_sq_desc(struct snd_queue *sq, int desc_cnt)
{
	while (desc_cnt--) {
		atomic_inc(&sq->free_cnt);
		sq->head++;
		sq->head &= (sq->dmem.q_len - 1);
	}
}

void nicvf_sq_enable(struct nicvf *nic, struct snd_queue *sq, int qidx)
{
	u64 sq_cfg;

	sq_cfg = nicvf_queue_reg_read(nic, NIC_QSET_SQ_0_7_CFG, qidx);
	sq_cfg |= NICVF_SQ_EN;
	nicvf_queue_reg_write(nic, NIC_QSET_SQ_0_7_CFG, qidx, sq_cfg);
	/* Ring doorbell so that H/W restarts processing SQEs */
	nicvf_queue_reg_write(nic, NIC_QSET_SQ_0_7_DOOR, qidx, 0);
}

void nicvf_sq_disable(struct nicvf *nic, int qidx)
{
	u64 sq_cfg;

	sq_cfg = nicvf_queue_reg_read(nic, NIC_QSET_SQ_0_7_CFG, qidx);
	sq_cfg &= ~NICVF_SQ_EN;
	nicvf_queue_reg_write(nic, NIC_QSET_SQ_0_7_CFG, qidx, sq_cfg);
}

void nicvf_sq_free_used_descs(struct net_device *netdev, struct snd_queue *sq,
			      int qidx)
{
	u64 head, tail;
	struct sk_buff *skb;
	struct nicvf *nic = netdev_priv(netdev);
	struct sq_hdr_subdesc *hdr;

	head = nicvf_queue_reg_read(nic, NIC_QSET_SQ_0_7_HEAD, qidx) >> 4;
	tail = nicvf_queue_reg_read(nic, NIC_QSET_SQ_0_7_TAIL, qidx) >> 4;
	while (sq->head != head) {
		hdr = (struct sq_hdr_subdesc *)GET_SQ_DESC(sq, sq->head);
		if (hdr->subdesc_type != SQ_DESC_TYPE_HEADER) {
			nicvf_put_sq_desc(sq, 1);
			continue;
		}
		skb = (struct sk_buff *)sq->skbuff[sq->head];
		atomic64_add(1, (atomic64_t *)&netdev->stats.tx_packets);
		atomic64_add(hdr->tot_len,
			     (atomic64_t *)&netdev->stats.tx_bytes);
		dev_kfree_skb_any(skb);
		nicvf_put_sq_desc(sq, hdr->subdesc_cnt + 1);
	}
}

/* Get the number of SQ descriptors needed to xmit this skb */
static int nicvf_sq_subdesc_required(struct nicvf *nic, struct sk_buff *skb)
{
	int subdesc_cnt = MIN_SQ_DESC_PER_PKT_XMIT;

	if (skb_shinfo(skb)->gso_size) {
		subdesc_cnt = skb_shinfo(skb)->gso_size / nic->mtu;
		subdesc_cnt *= MIN_SQ_DESC_PER_PKT_XMIT;
		return subdesc_cnt;
	}

	if (skb_shinfo(skb)->nr_frags)
		subdesc_cnt += skb_shinfo(skb)->nr_frags;

	return subdesc_cnt;
}

/* Add SQ HEADER subdescriptor.
 * First subdescriptor for every send descriptor.
 */
struct sq_hdr_subdesc *
nicvf_sq_add_hdr_subdesc(struct queue_set *qs, int sq_num,
			 int subdesc_cnt, struct sk_buff *skb)
{
	int qentry, proto;
	void *desc = NULL;
	struct snd_queue *sq;
	struct sq_hdr_subdesc *hdr;

	sq = &qs->sq[sq_num];
	qentry = nicvf_get_sq_desc(qs, sq_num, &desc);
	sq->skbuff[qentry] = (u64)skb;

	hdr = (struct sq_hdr_subdesc *)desc;

	memset(hdr, 0, SND_QUEUE_DESC_SIZE);
	hdr->subdesc_type = SQ_DESC_TYPE_HEADER;
	/* Enable notification via CQE after processing SQE */
	hdr->post_cqe = 1;
	/* No of subdescriptors following this */
	hdr->subdesc_cnt = subdesc_cnt;
	hdr->tot_len = skb->len;

	/* Offload checksum calculation to HW */
	if (skb->ip_summed == CHECKSUM_PARTIAL) {
		if (skb->protocol != htons(ETH_P_IP))
			return hdr;
		hdr->csum_l3 = 0;
		hdr->l3_offset = skb_network_offset(skb);
		hdr->l4_offset = skb_transport_offset(skb);
		proto = ip_hdr(skb)->protocol;
		switch (proto) {
		case IPPROTO_TCP:
			hdr->csum_l4 = SEND_L4_CSUM_TCP;
			break;
		case IPPROTO_UDP:
			hdr->csum_l4 = SEND_L4_CSUM_UDP;
			break;
		case IPPROTO_SCTP:
			hdr->csum_l4 = SEND_L4_CSUM_SCTP;
			break;
		}
	}

	return hdr;
}

/* SQ GATHER subdescriptor
 * Must follow HDR descriptor
 */
static void nicvf_sq_add_gather_subdesc(struct nicvf *nic, struct queue_set *qs,
					int sq_num, struct sk_buff *skb)
{
	int i;
	void *desc = NULL;
	struct sq_gather_subdesc *gather;

	nicvf_get_sq_desc(qs, sq_num, &desc);
	gather = (struct sq_gather_subdesc *)desc;

	memset(gather, 0, SND_QUEUE_DESC_SIZE);
	gather->subdesc_type = SQ_DESC_TYPE_GATHER;
	gather->ld_type = NIC_SEND_LD_TYPE_E_LDD;
	gather->size = skb_is_nonlinear(skb) ? skb_headlen(skb) : skb->len;
	gather->addr = virt_to_phys(skb->data);

	/* Check for scattered buffer */
	if (!skb_is_nonlinear(skb))
		return;

	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		const struct skb_frag_struct *frag;

		frag = &skb_shinfo(skb)->frags[i];

		nicvf_get_sq_desc(qs, sq_num, &desc);
		gather = (struct sq_gather_subdesc *)desc;

		memset(gather, 0, SND_QUEUE_DESC_SIZE);
		gather->subdesc_type = SQ_DESC_TYPE_GATHER;
		gather->ld_type = NIC_SEND_LD_TYPE_E_LDD;
		gather->size = skb_frag_size(frag);
		gather->addr = virt_to_phys(skb_frag_address(frag));
	}
}

/* Append an skb to a SQ for packet transfer. */
int nicvf_sq_append_skb(struct nicvf *nic, struct sk_buff *skb)
{
	int subdesc_cnt;
	int sq_num;
	struct queue_set *qs = nic->qs;
	struct snd_queue *sq;
	struct sq_hdr_subdesc *hdr_desc;

	sq_num = skb_get_queue_mapping(skb);
	sq = &qs->sq[sq_num];

	subdesc_cnt = nicvf_sq_subdesc_required(nic, skb);
	if (subdesc_cnt > atomic_read(&sq->free_cnt))
		goto append_fail;

	/* Add SQ header subdesc */
	hdr_desc = nicvf_sq_add_hdr_subdesc(qs, sq_num, subdesc_cnt - 1, skb);

	/* Add SQ gather subdesc */
	nicvf_sq_add_gather_subdesc(nic, qs, sq_num, skb);

	/* make sure all memory stores are done before ringing doorbell */
	smp_wmb();

	/* Inform HW to xmit new packet */
	nicvf_queue_reg_write(nic, NIC_QSET_SQ_0_7_DOOR,
			      sq_num, subdesc_cnt);
	return 1;

append_fail:
	nic_dbg(&nic->pdev->dev, "Not enough SQ descriptors to xmit pkt\n");
	return 0;
}

static inline unsigned frag_num(unsigned i)
{
#ifdef __BIG_ENDIAN
	return (i & ~3) + 3 - (i & 3);
#else
	return i;
#endif
}

/* Returns SKB for a received packet */
struct sk_buff *nicvf_get_rcv_skb(struct nicvf *nic, struct cqe_rx_t *cqe_rx)
{
	int frag;
	int payload_len = 0;
	struct sk_buff *skb = NULL;
	struct sk_buff *skb_frag = NULL;
	struct sk_buff *prev_frag = NULL;
	u16 *rb_lens = NULL;
	u64 *rb_ptrs = NULL;

	rb_lens = (void *)cqe_rx + (3 * sizeof(u64));
	rb_ptrs = (void *)cqe_rx + (6 * sizeof(u64));

	nic_dbg(&nic->pdev->dev, "%s rb_cnt %d rb0_ptr %llx rb0_sz %d\n",
		__func__, cqe_rx->rb_cnt, cqe_rx->rb0_ptr, cqe_rx->rb0_sz);

	for (frag = 0; frag < cqe_rx->rb_cnt; frag++) {
		payload_len = rb_lens[frag_num(frag)];
		if (!frag) {
			/* First fragment */
			skb = nicvf_rb_ptr_to_skb(nic,
						  *rb_ptrs - cqe_rx->align_pad);
			if (!skb)
				return NULL;
			skb_reserve(skb, cqe_rx->align_pad);
			skb_put(skb, payload_len);
		} else {
			/* Add fragments */
			skb_frag = nicvf_rb_ptr_to_skb(nic, *rb_ptrs);
			if (!skb_frag) {
				dev_kfree_skb(skb);
				return NULL;
			}

			if (!skb_shinfo(skb)->frag_list)
				skb_shinfo(skb)->frag_list = skb_frag;
			else
				prev_frag->next = skb_frag;

			prev_frag = skb_frag;
			skb->len += payload_len;
			skb->data_len += payload_len;
			skb_frag->len = payload_len;
		}
		/* Next buffer pointer */
		rb_ptrs++;
	}
	return skb;
}

/* Enable interrupt */
void nicvf_enable_intr(struct nicvf *nic, int int_type, int q_idx)
{
	u64 reg_val;

	reg_val = nicvf_reg_read(nic, NIC_VF_ENA_W1S);

	switch (int_type) {
	case NICVF_INTR_CQ:
		reg_val |= ((1ULL << q_idx) << NICVF_INTR_CQ_SHIFT);
	break;
	case NICVF_INTR_SQ:
		reg_val |= ((1ULL << q_idx) << NICVF_INTR_SQ_SHIFT);
	break;
	case NICVF_INTR_RBDR:
		reg_val |= ((1ULL << q_idx) << NICVF_INTR_RBDR_SHIFT);
	break;
	case NICVF_INTR_PKT_DROP:
		reg_val |= (1ULL << NICVF_INTR_PKT_DROP_SHIFT);
	break;
	case NICVF_INTR_TCP_TIMER:
		reg_val |= (1ULL << NICVF_INTR_TCP_TIMER_SHIFT);
	break;
	case NICVF_INTR_MBOX:
		reg_val |= (1ULL << NICVF_INTR_MBOX_SHIFT);
	break;
	case NICVF_INTR_QS_ERR:
		reg_val |= (1ULL << NICVF_INTR_QS_ERR_SHIFT);
	break;
	default:
		netdev_err(nic->netdev,
			   "Failed to enable interrupt: unknown type\n");
	break;
	}

	nicvf_reg_write(nic, NIC_VF_ENA_W1S, reg_val);
}

/* Disable interrupt */
void nicvf_disable_intr(struct nicvf *nic, int int_type, int q_idx)
{
	u64 reg_val = 0;

	switch (int_type) {
	case NICVF_INTR_CQ:
		reg_val |= ((1ULL << q_idx) << NICVF_INTR_CQ_SHIFT);
	break;
	case NICVF_INTR_SQ:
		reg_val |= ((1ULL << q_idx) << NICVF_INTR_SQ_SHIFT);
	break;
	case NICVF_INTR_RBDR:
		reg_val |= ((1ULL << q_idx) << NICVF_INTR_RBDR_SHIFT);
	break;
	case NICVF_INTR_PKT_DROP:
		reg_val |= (1ULL << NICVF_INTR_PKT_DROP_SHIFT);
	break;
	case NICVF_INTR_TCP_TIMER:
		reg_val |= (1ULL << NICVF_INTR_TCP_TIMER_SHIFT);
	break;
	case NICVF_INTR_MBOX:
		reg_val |= (1ULL << NICVF_INTR_MBOX_SHIFT);
	break;
	case NICVF_INTR_QS_ERR:
		reg_val |= (1ULL << NICVF_INTR_QS_ERR_SHIFT);
	break;
	default:
		netdev_err(nic->netdev,
			   "Failed to disable interrupt: unknown type\n");
	break;
	}

	nicvf_reg_write(nic, NIC_VF_ENA_W1C, reg_val);
}

/* Clear interrupt */
void nicvf_clear_intr(struct nicvf *nic, int int_type, int q_idx)
{
	u64 reg_val = 0;

	switch (int_type) {
	case NICVF_INTR_CQ:
		reg_val = ((1ULL << q_idx) << NICVF_INTR_CQ_SHIFT);
	break;
	case NICVF_INTR_SQ:
		reg_val = ((1ULL << q_idx) << NICVF_INTR_SQ_SHIFT);
	break;
	case NICVF_INTR_RBDR:
		reg_val = ((1ULL << q_idx) << NICVF_INTR_RBDR_SHIFT);
	break;
	case NICVF_INTR_PKT_DROP:
		reg_val = (1ULL << NICVF_INTR_PKT_DROP_SHIFT);
	break;
	case NICVF_INTR_TCP_TIMER:
		reg_val = (1ULL << NICVF_INTR_TCP_TIMER_SHIFT);
	break;
	case NICVF_INTR_MBOX:
		reg_val = (1ULL << NICVF_INTR_MBOX_SHIFT);
	break;
	case NICVF_INTR_QS_ERR:
		reg_val |= (1ULL << NICVF_INTR_QS_ERR_SHIFT);
	break;
	default:
		netdev_err(nic->netdev,
			   "Failed to clear interrupt: unknown type\n");
	break;
	}

	nicvf_reg_write(nic, NIC_VF_INT, reg_val);
}

/* Check if interrupt is enabled */
int nicvf_is_intr_enabled(struct nicvf *nic, int int_type, int q_idx)
{
	u64 reg_val;
	u64 mask = 0xff;

	reg_val = nicvf_reg_read(nic, NIC_VF_ENA_W1S);

	switch (int_type) {
	case NICVF_INTR_CQ:
		mask = ((1ULL << q_idx) << NICVF_INTR_CQ_SHIFT);
	break;
	case NICVF_INTR_SQ:
		mask = ((1ULL << q_idx) << NICVF_INTR_SQ_SHIFT);
	break;
	case NICVF_INTR_RBDR:
		mask = ((1ULL << q_idx) << NICVF_INTR_RBDR_SHIFT);
	break;
	case NICVF_INTR_PKT_DROP:
		mask = NICVF_INTR_PKT_DROP_MASK;
	break;
	case NICVF_INTR_TCP_TIMER:
		mask = NICVF_INTR_TCP_TIMER_MASK;
	break;
	case NICVF_INTR_MBOX:
		mask = NICVF_INTR_MBOX_MASK;
	break;
	case NICVF_INTR_QS_ERR:
		mask = NICVF_INTR_QS_ERR_MASK;
	break;
	default:
		netdev_err(nic->netdev,
			   "Failed to check interrupt enable: unknown type\n");
	break;
	}

	return (reg_val & mask);
}

void nicvf_update_rq_stats(struct nicvf *nic, int rq_idx)
{
	struct rcv_queue *rq;

#define GET_RQ_STATS(reg) \
	nicvf_reg_read(nic, NIC_QSET_RQ_0_7_STAT_0_1 |\
			    (rq_idx << NIC_Q_NUM_SHIFT) | (reg << 3))

	rq = &nic->qs->rq[rq_idx];
	rq->stats.bytes = GET_RQ_STATS(RQ_SQ_STATS_OCTS);
	rq->stats.pkts = GET_RQ_STATS(RQ_SQ_STATS_PKTS);
}

void nicvf_update_sq_stats(struct nicvf *nic, int sq_idx)
{
	struct snd_queue *sq;

#define GET_SQ_STATS(reg) \
	nicvf_reg_read(nic, NIC_QSET_SQ_0_7_STAT_0_1 |\
			    (sq_idx << NIC_Q_NUM_SHIFT) | (reg << 3))

	sq = &nic->qs->sq[sq_idx];
	sq->stats.bytes = GET_SQ_STATS(RQ_SQ_STATS_OCTS);
	sq->stats.pkts = GET_SQ_STATS(RQ_SQ_STATS_PKTS);
}

/* Check for errors in the receive cmp.queue entry */
int nicvf_check_cqe_rx_errs(struct nicvf *nic,
			    struct cmp_queue *cq, struct cqe_rx_t *cqe_rx)
{
	struct cmp_queue_stats *stats = &cq->stats;

	if (!cqe_rx->err_level && !cqe_rx->err_opcode) {
		stats->rx.errop.good++;
		return 0;
	}

	if (netif_msg_rx_err(nic))
		netdev_err(nic->netdev,
			   "%s: RX error CQE err_level 0x%x err_opcode 0x%x\n",
			   nic->netdev->name,
			   cqe_rx->err_level, cqe_rx->err_opcode);

	switch (cqe_rx->err_level) {
	case CQ_ERRLVL_MAC:
		stats->rx.errlvl.mac_errs++;
	break;
	case CQ_ERRLVL_L2:
		stats->rx.errlvl.l2_errs++;
	break;
	case CQ_ERRLVL_L3:
		stats->rx.errlvl.l3_errs++;
	break;
	case CQ_ERRLVL_L4:
		stats->rx.errlvl.l4_errs++;
	break;
	}

	switch (cqe_rx->err_opcode) {
	case CQ_RX_ERROP_RE_PARTIAL:
		stats->rx.errop.partial_pkts++;
	break;
	case CQ_RX_ERROP_RE_JABBER:
		stats->rx.errop.jabber_errs++;
	break;
	case CQ_RX_ERROP_RE_FCS:
		stats->rx.errop.fcs_errs++;
	break;
	case CQ_RX_ERROP_RE_TERMINATE:
		stats->rx.errop.terminate_errs++;
	break;
	case CQ_RX_ERROP_RE_RX_CTL:
		stats->rx.errop.bgx_rx_errs++;
	break;
	case CQ_RX_ERROP_PREL2_ERR:
		stats->rx.errop.prel2_errs++;
	break;
	case CQ_RX_ERROP_L2_FRAGMENT:
		stats->rx.errop.l2_frags++;
	break;
	case CQ_RX_ERROP_L2_OVERRUN:
		stats->rx.errop.l2_overruns++;
	break;
	case CQ_RX_ERROP_L2_PFCS:
		stats->rx.errop.l2_pfcs++;
	break;
	case CQ_RX_ERROP_L2_PUNY:
		stats->rx.errop.l2_puny++;
	break;
	case CQ_RX_ERROP_L2_MAL:
		stats->rx.errop.l2_hdr_malformed++;
	break;
	case CQ_RX_ERROP_L2_OVERSIZE:
		stats->rx.errop.l2_oversize++;
	break;
	case CQ_RX_ERROP_L2_UNDERSIZE:
		stats->rx.errop.l2_undersize++;
	break;
	case CQ_RX_ERROP_L2_LENMISM:
		stats->rx.errop.l2_len_mismatch++;
	break;
	case CQ_RX_ERROP_L2_PCLP:
		stats->rx.errop.l2_pclp++;
	break;
	case CQ_RX_ERROP_IP_NOT:
		stats->rx.errop.non_ip++;
	break;
	case CQ_RX_ERROP_IP_CSUM_ERR:
		stats->rx.errop.ip_csum_err++;
	break;
	case CQ_RX_ERROP_IP_MAL:
		stats->rx.errop.ip_hdr_malformed++;
	break;
	case CQ_RX_ERROP_IP_MALD:
		stats->rx.errop.ip_payload_malformed++;
	break;
	case CQ_RX_ERROP_IP_HOP:
		stats->rx.errop.ip_hop_errs++;
	break;
	case CQ_RX_ERROP_L3_ICRC:
		stats->rx.errop.l3_icrc_errs++;
	break;
	case CQ_RX_ERROP_L3_PCLP:
		stats->rx.errop.l3_pclp++;
	break;
	case CQ_RX_ERROP_L4_MAL:
		stats->rx.errop.l4_malformed++;
	break;
	case CQ_RX_ERROP_L4_CHK:
		stats->rx.errop.l4_csum_errs++;
	break;
	case CQ_RX_ERROP_UDP_LEN:
		stats->rx.errop.udp_len_err++;
	break;
	case CQ_RX_ERROP_L4_PORT:
		stats->rx.errop.bad_l4_port++;
	break;
	case CQ_RX_ERROP_TCP_FLAG:
		stats->rx.errop.bad_tcp_flag++;
	break;
	case CQ_RX_ERROP_TCP_OFFSET:
		stats->rx.errop.tcp_offset_errs++;
	break;
	case CQ_RX_ERROP_L4_PCLP:
		stats->rx.errop.l4_pclp++;
	break;
	case CQ_RX_ERROP_RBDR_TRUNC:
		stats->rx.errop.pkt_truncated++;
	break;
	}

	return 1;
}

/* Check for errors in the send cmp.queue entry */
int nicvf_check_cqe_tx_errs(struct nicvf *nic,
			    struct cmp_queue *cq, struct cqe_send_t *cqe_tx)
{
	struct cmp_queue_stats *stats = &cq->stats;

	switch (cqe_tx->send_status) {
	case CQ_TX_ERROP_GOOD:
		stats->tx.good++;
		return 0;
	break;
	case CQ_TX_ERROP_DESC_FAULT:
		stats->tx.desc_fault++;
	break;
	case CQ_TX_ERROP_HDR_CONS_ERR:
		stats->tx.hdr_cons_err++;
	break;
	case CQ_TX_ERROP_SUBDC_ERR:
		stats->tx.subdesc_err++;
	break;
	case CQ_TX_ERROP_IMM_SIZE_OFLOW:
		stats->tx.imm_size_oflow++;
	break;
	case CQ_TX_ERROP_DATA_SEQUENCE_ERR:
		stats->tx.data_seq_err++;
	break;
	case CQ_TX_ERROP_MEM_SEQUENCE_ERR:
		stats->tx.mem_seq_err++;
	break;
	case CQ_TX_ERROP_LOCK_VIOL:
		stats->tx.lock_viol++;
	break;
	case CQ_TX_ERROP_DATA_FAULT:
		stats->tx.data_fault++;
	break;
	case CQ_TX_ERROP_TSTMP_CONFLICT:
		stats->tx.tstmp_conflict++;
	break;
	case CQ_TX_ERROP_TSTMP_TIMEOUT:
		stats->tx.tstmp_timeout++;
	break;
	case CQ_TX_ERROP_MEM_FAULT:
		stats->tx.mem_fault++;
	break;
	case CQ_TX_ERROP_CK_OVERLAP:
		stats->tx.csum_overlap++;
	break;
	case CQ_TX_ERROP_CK_OFLOW:
		stats->tx.csum_overflow++;
	break;
	}

	return 1;
}

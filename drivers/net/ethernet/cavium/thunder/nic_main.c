/*
 * Copyright (C) 2015 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/of.h>

#include "nic_reg.h"
#include "nic.h"
#include "q_struct.h"
#include "thunder_bgx.h"

#define DRV_NAME	"thunder-nic"
#define DRV_VERSION	"1.0"

static void nic_config_cpi(struct nicpf *nic, struct cpi_cfg_msg *cfg);
#ifdef VNIC_RSS_SUPPORT
static void nic_send_rss_size(struct nicpf *nic, int vf);
static void nic_config_rss(struct nicpf *nic, struct rss_cfg_msg *cfg);
#endif
static void nic_tx_channel_cfg(struct nicpf *nic, u8 vnic, u8 sq_idx);
static int nic_update_hw_frs(struct nicpf *nic, int new_frs, int vf);
static int nic_rcv_queue_sw_sync(struct nicpf *nic);
static void nic_get_bgx_stats(struct nicpf *nic, struct bgx_stats_msg *bgx);

/* Supported devices */
static const struct pci_device_id nic_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_CAVIUM, PCI_DEVICE_ID_THUNDER_NIC_PF) },
	{ 0, }  /* end of table */
};

MODULE_AUTHOR("Sunil Goutham");
MODULE_DESCRIPTION("Cavium Thunder NIC Physical Function Driver");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRV_VERSION);
MODULE_DEVICE_TABLE(pci, nic_id_table);

/* Register read/write APIs */
static void nic_reg_write(struct nicpf *nic, u64 offset, u64 val)
{
	u64 addr = nic->reg_base + offset;

	writeq_relaxed(val, (void *)addr);
}

static u64 nic_reg_read(struct nicpf *nic, u64 offset)
{
	u64 addr = nic->reg_base + offset;

	return readq_relaxed((void *)addr);
}

/* PF -> VF mailbox communication APIs */
static void nic_enable_mbx_intr(struct nicpf *nic)
{
	/* Enable mailbox interrupt for all 128 VFs */
	nic_reg_write(nic, NIC_PF_MAILBOX_ENA_W1S, ~0x00ull);
	nic_reg_write(nic, NIC_PF_MAILBOX_ENA_W1S + sizeof(u64), ~0x00ull);
}

static void nic_clear_mbx_intr(struct nicpf *nic, int vf, int mbx_reg)
{
	nic_reg_write(nic, NIC_PF_MAILBOX_INT + (mbx_reg << 3), (1ULL << vf));
}

static u64 nic_get_mbx_addr(int vf)
{
	return NIC_PF_VF_0_127_MAILBOX_0_1 + (vf << NIC_VF_NUM_SHIFT);
}

/* Send a mailbox message to VF
 * @vf: vf to which this message to be sent
 * @mbx: Message to be sent
 */
static int nic_send_msg_to_vf(struct nicpf *nic, int vf, struct nic_mbx *mbx)
{
	u64 *msg;
	u64 mbx_addr;

	msg = (u64 *)mbx;
	mbx_addr = nic->reg_base + nic_get_mbx_addr(vf);

	/* In first revision HW, mbox interrupt is triggerred
	 * when PF writes to MBOX(1), in next revisions when
	 * PF writes to MBOX(0)
	 */
	if (nic->rev_id == 0) {
		writeq_relaxed(*(msg), (void *)mbx_addr);
		writeq_relaxed(*(msg + 1), (void *)(mbx_addr + 8));
	} else {
		writeq_relaxed(*(msg + 1), (void *)(mbx_addr + 8));
		writeq_relaxed(*(msg), (void *)mbx_addr);
	}

	return 0;
}

/* Function to read MAC addresses from DTS.
 * If not found random MACs will be used by VF interface
 */
static int nic_get_mac_addresses(struct nicpf *nic)
{
	struct device_node *np, *np_child;
	u32  nplen;
	u8  *np_ptr = NULL, node[10], next_range_off = 10;
	u32  i, mac_count = 0, range, mac_range;
	u64  start_mac = 0, next_mac  = 0;

	/* Check if MAC ID list is present in DTS */
	np = of_find_node_by_name(NULL, "ethernet-macs");
	if (!np)
		return 0;

	/* Check if MAC ID list is available for this node (NUMA) */
	sprintf(node, "node%d", nic->node);
	np_child = of_get_child_by_name(np, node);
	if (!np_child)
		return 0;

	np_ptr = (u8 *)of_get_property(np_child, "mac", &nplen);
	if (!np_ptr)
		return 0;

	for (range = 0; range < nplen; range = range + next_range_off) {
		/* get first mac into cpu format */
		memcpy(&start_mac, np_ptr + range, 8);
#ifdef __BIG_ENDIAN
		start_mac = start_mac >> 16; /* MACADDR is only 48bits */
#else
		start_mac = start_mac << 16;
#endif
		start_mac = be64_to_cpu(start_mac);
		next_mac  = start_mac;
		/* get range length for given range */
		mac_range = be32_to_cpup((u32 *)(np_ptr + range +
					ETH_ALEN));
		for (i = 0; i < mac_range; i++) {
			if (mac_count >= MAX_NUM_VFS_SUPPORTED) {
				dev_info(&nic->pdev->dev,
					 "OF mac address count exceeded\n");
				goto out;
			}

#ifdef __BIG_ENDIAN
			nic->mac[mac_count++] =
				cpu_to_be64(next_mac) << 16;
#else
			nic->mac[mac_count++] =
				cpu_to_be64(next_mac) >> 16;
#endif
			next_mac++;
		}
	}
out:
	return 1;
}

/* Responds to VF's READY message with VF's
 * ID, node, MAC address e.t.c
 * @vf: VF which sent READY message
 */
static void nic_mbx_send_ready(struct nicpf *nic, int vf)
{
	struct nic_mbx mbx = {};

	mbx.msg = NIC_MBOX_MSG_READY;
	mbx.data.nic_cfg.vf_id = vf;

	if (nic->flags & NIC_TNS_ENABLED)
		mbx.data.nic_cfg.tns_mode = NIC_TNS_MODE;
	else
		mbx.data.nic_cfg.tns_mode = NIC_TNS_BYPASS_MODE;
	ether_addr_copy((u8 *)&mbx.data.nic_cfg.mac_addr,
			(u8 *)&nic->mac[vf]);
	mbx.data.nic_cfg.node_id = nic->node;
	nic_send_msg_to_vf(nic, vf, &mbx);
}

/* ACKs VF's mailbox message
 * @vf: VF to which ACK to be sent
 */
static void nic_mbx_send_ack(struct nicpf *nic, int vf)
{
	struct nic_mbx mbx = {};

	mbx.msg = NIC_MBOX_MSG_ACK;
	nic_send_msg_to_vf(nic, vf, &mbx);
}

/* NACKs VF's mailbox message that PF is not able to
 * complete the action
 * @vf: VF to which ACK to be sent
 */
static void nic_mbx_send_nack(struct nicpf *nic, int vf)
{
	struct nic_mbx mbx = {};

	mbx.msg = NIC_MBOX_MSG_NACK;
	nic_send_msg_to_vf(nic, vf, &mbx);
}

/* Interrupt handler to handle mailbox messages from VFs */
static void nic_handle_mbx_intr(struct nicpf *nic, int vf)
{
	struct nic_mbx mbx = {};
	u64 *mbx_data;
	u64 mbx_addr;
	u64 reg_addr;
	u64 mac_addr;
	int bgx, lmac;
	int i;
	int ret = 0;

	nic->mbx_lock[vf] = true;

	mbx_addr = nic_get_mbx_addr(vf);
	mbx_data = (u64 *)&mbx;

	for (i = 0; i < NIC_PF_VF_MAILBOX_SIZE; i++) {
		*mbx_data = nic_reg_read(nic, mbx_addr);
		mbx_data++;
		mbx_addr += sizeof(u64);
	}

	nic_dbg(&nic->pdev->dev, "%s: Mailbox msg %d from VF%d\n",
		__func__, mbx.msg, vf);
	switch (mbx.msg) {
	case NIC_MBOX_MSG_READY:
		nic_mbx_send_ready(nic, vf);
		nic->link[vf] = 0;
		nic->duplex[vf] = 0;
		nic->speed[vf] = 0;
		ret = 1;
		break;
	case NIC_MBOX_MSG_QS_CFG:
		reg_addr = NIC_PF_QSET_0_127_CFG |
			   (mbx.data.qs.num << NIC_QS_ID_SHIFT);
		nic_reg_write(nic, reg_addr, mbx.data.qs.cfg);
		break;
	case NIC_MBOX_MSG_RQ_CFG:
		reg_addr = NIC_PF_QSET_0_127_RQ_0_7_CFG |
			   (mbx.data.rq.qs_num << NIC_QS_ID_SHIFT) |
			   (mbx.data.rq.rq_num << NIC_Q_NUM_SHIFT);
		nic_reg_write(nic, reg_addr, mbx.data.rq.cfg);
		break;
	case NIC_MBOX_MSG_RQ_BP_CFG:
		reg_addr = NIC_PF_QSET_0_127_RQ_0_7_BP_CFG |
			   (mbx.data.rq.qs_num << NIC_QS_ID_SHIFT) |
			   (mbx.data.rq.rq_num << NIC_Q_NUM_SHIFT);
		nic_reg_write(nic, reg_addr, mbx.data.rq.cfg);
		break;
	case NIC_MBOX_MSG_RQ_SW_SYNC:
		ret = nic_rcv_queue_sw_sync(nic);
		/* Last message of VF teardown msg sequence */
		nic->vf_enabled[vf] = false;
		break;
	case NIC_MBOX_MSG_RQ_DROP_CFG:
		reg_addr = NIC_PF_QSET_0_127_RQ_0_7_DROP_CFG |
			   (mbx.data.rq.qs_num << NIC_QS_ID_SHIFT) |
			   (mbx.data.rq.rq_num << NIC_Q_NUM_SHIFT);
		nic_reg_write(nic, reg_addr, mbx.data.rq.cfg);
		/* Last message of VF config msg sequence */
		nic->vf_enabled[vf] = true;
		break;
	case NIC_MBOX_MSG_SQ_CFG:
		reg_addr = NIC_PF_QSET_0_127_SQ_0_7_CFG |
			   (mbx.data.sq.qs_num << NIC_QS_ID_SHIFT) |
			   (mbx.data.sq.sq_num << NIC_Q_NUM_SHIFT);
		nic_reg_write(nic, reg_addr, mbx.data.sq.cfg);
		nic_tx_channel_cfg(nic, mbx.data.qs.num, mbx.data.sq.sq_num);
		break;
	case NIC_MBOX_MSG_SET_MAC:
		lmac = mbx.data.mac.vf_id;
		bgx = NIC_GET_BGX_FROM_VF_LMAC_MAP(nic->vf_lmac_map[lmac]);
		lmac = NIC_GET_LMAC_FROM_VF_LMAC_MAP(nic->vf_lmac_map[lmac]);
#ifdef __BIG_ENDIAN
		mac_addr = cpu_to_be64(mbx.data.nic_cfg.mac_addr) << 16;
#else
		mac_addr = cpu_to_be64(mbx.data.nic_cfg.mac_addr) >> 16;
#endif
		ether_addr_copy((u8 *)&nic->mac[vf],
				(u8 *)&mac_addr);
		break;
	case NIC_MBOX_MSG_SET_MAX_FRS:
		ret = nic_update_hw_frs(nic, mbx.data.frs.max_frs,
					mbx.data.frs.vf_id);
		break;
	case NIC_MBOX_MSG_CPI_CFG:
		nic_config_cpi(nic, &mbx.data.cpi_cfg);
		break;
#ifdef VNIC_RSS_SUPPORT
	case NIC_MBOX_MSG_RSS_SIZE:
		nic_send_rss_size(nic, vf);
		goto unlock;
	case NIC_MBOX_MSG_RSS_CFG:
	case NIC_MBOX_MSG_RSS_CFG_CONT:
		nic_config_rss(nic, &mbx.data.rss_cfg);
		break;
#endif
	case NIC_MBOX_MSG_BGX_STATS:
		nic_get_bgx_stats(nic, &mbx.data.bgx_stats);
		goto unlock;
	default:
		netdev_err(nic->netdev,
			   "Invalid msg from VF%d, msg 0x%x\n", vf, mbx.msg);
		break;
	}

	if (!ret)
		nic_mbx_send_ack(nic, vf);
	else if (mbx.msg != NIC_MBOX_MSG_READY)
		nic_mbx_send_nack(nic, vf);
unlock:
	nic->mbx_lock[vf] = false;
}

/* Flush all in flight receive packets to memory and
 * bring down an active RQ
 */
static int nic_rcv_queue_sw_sync(struct nicpf *nic)
{
	u16 timeout = ~0x00;

	nic_reg_write(nic, NIC_PF_SW_SYNC_RX, 0x01);
	/* Wait till sync cycle is finished */
	while (timeout) {
		if (nic_reg_read(nic, NIC_PF_SW_SYNC_RX_DONE) & 0x1)
			break;
		timeout--;
	}
	nic_reg_write(nic, NIC_PF_SW_SYNC_RX, 0x00);
	if (!timeout) {
		netdev_err(nic->netdev, "Recevie queue software sync failed");
		return 1;
	}
	return 0;
}

/* Get BGX Rx/Tx stats and respond to VF's request */
static void nic_get_bgx_stats(struct nicpf *nic, struct bgx_stats_msg *bgx)
{
	int bgx_idx, lmac;
	struct nic_mbx mbx = {};

	bgx_idx = NIC_GET_BGX_FROM_VF_LMAC_MAP(nic->vf_lmac_map[bgx->vf_id]);
	lmac = NIC_GET_LMAC_FROM_VF_LMAC_MAP(nic->vf_lmac_map[bgx->vf_id]);

	mbx.msg = NIC_MBOX_MSG_BGX_STATS;
	mbx.data.bgx_stats.vf_id = bgx->vf_id;
	mbx.data.bgx_stats.rx = bgx->rx;
	mbx.data.bgx_stats.idx = bgx->idx;
	if (bgx->rx)
		mbx.data.bgx_stats.stats = bgx_get_rx_stats(nic->node, bgx_idx,
							    lmac, bgx->idx);
	else
		mbx.data.bgx_stats.stats = bgx_get_tx_stats(nic->node, bgx_idx,
							    lmac, bgx->idx);
	nic_send_msg_to_vf(nic, bgx->vf_id, &mbx);
}

/* Update hardware min/max frame size */
static int nic_update_hw_frs(struct nicpf *nic, int new_frs, int vf)
{
	if ((new_frs > NIC_HW_MAX_FRS) || (new_frs < NIC_HW_MIN_FRS)) {
		netdev_err(nic->netdev,
			   "Invalid MTU setting from VF%d rejected, should be between %d and %d\n",
			   vf, NIC_HW_MIN_FRS, NIC_HW_MAX_FRS);
		return 1;
	}
	new_frs += ETH_HLEN;
	if (new_frs <= nic->pkind.maxlen)
		return 0;

	nic->pkind.maxlen = new_frs;
	nic_reg_write(nic, NIC_PF_PKIND_0_15_CFG, *(u64 *)&nic->pkind);
	return 0;
}

/* Set minimum transmit packet size */
static void nic_set_tx_pkt_pad(struct nicpf *nic, int size)
{
	int lmac;
	u64 lmac_cfg;

	/* Max value that can be set is 60 */
	if (size > 60)
		size = 60;

	for (lmac = 0; lmac < (MAX_BGX_PER_CN88XX * MAX_LMAC_PER_BGX); lmac++) {
		lmac_cfg = nic_reg_read(nic, NIC_PF_LMAC_0_7_CFG | (lmac << 3));
		lmac_cfg &= ~(0xF << 2);
		lmac_cfg |= ((size / 4) << 2);
		nic_reg_write(nic, NIC_PF_LMAC_0_7_CFG | (lmac << 3), lmac_cfg);
	}
}

/* Function to check number of LMACs present and set VF::LMAC mapping.
 * Mapping will be used while initializing channels.
 */
static void nic_set_lmac_vf_mapping(struct nicpf *nic)
{
	int bgx, bgx_count, next_bgx_lmac = 0;
	int lmac, lmac_cnt = 0;
	u64 lmac_credit;

	nic->num_vf_en = 0;
	if (nic->flags & NIC_TNS_ENABLED) {
		nic->num_vf_en = DEFAULT_NUM_VF_ENABLED;
		return;
	}

	bgx_get_count(nic->node, &bgx_count);
	for (bgx = 0; bgx < NIC_MAX_BGX; bgx++) {
		if (!(bgx_count & (1 << bgx)))
			continue;
		nic->bgx_cnt++;
		lmac_cnt = bgx_get_lmac_count(nic->node, bgx);
		for (lmac = 0; lmac < lmac_cnt; lmac++)
			nic->vf_lmac_map[next_bgx_lmac++] =
						NIC_SET_VF_LMAC_MAP(bgx, lmac);
		nic->num_vf_en += lmac_cnt;

		/* Program LMAC credits */
		lmac_credit = (1ull << 1); /* channel credit enable */
		lmac_credit |= (0x1ff << 2); /* Max outstanding pkt count */
		/* 48KB BGX Tx buffer size, each unit is of size 16bytes */
		lmac_credit |= (((((48 * 1024) / lmac_cnt) -
				NIC_HW_MAX_FRS) / 16) << 12);
		lmac = bgx * MAX_LMAC_PER_BGX;
		for (; lmac < lmac_cnt + (bgx * MAX_LMAC_PER_BGX); lmac++)
			nic_reg_write(nic,
				      NIC_PF_LMAC_0_7_CREDIT + (lmac * 8),
				      lmac_credit);
	}
}

static void nic_init_hw(struct nicpf *nic)
{
	int i;
	u64 reg;

	/* Reset NIC, incase if driver is repeatedly inserted and removed */
	nic_reg_write(nic, NIC_PF_SOFT_RESET, 1);

	/* Enable NIC HW block */
	nic_reg_write(nic, NIC_PF_CFG, 0x3);

	/* Enable backpressure */
	nic_reg_write(nic, NIC_PF_BP_CFG, (1ULL << 6) | 0x03);
	nic_reg_write(nic, NIC_PF_INTF_0_1_BP_CFG, (1ULL << 63) | 0x08);
	nic_reg_write(nic,
		      NIC_PF_INTF_0_1_BP_CFG + (1 << 8), (1ULL << 63) | 0x09);

	if (nic->flags & NIC_TNS_ENABLED) {
		reg = NIC_TNS_MODE << 7;
		reg |= 0x06;
		nic_reg_write(nic, NIC_PF_INTF_0_1_SEND_CFG, reg);
		reg &= ~0xFull;
		reg |= 0x07;
		nic_reg_write(nic, NIC_PF_INTF_0_1_SEND_CFG | (1 << 8), reg);
	} else {
		/* Disable TNS mode on both interfaces */
		reg = NIC_TNS_BYPASS_MODE << 7;
		reg |= 0x08; /* Block identifier */
		nic_reg_write(nic, NIC_PF_INTF_0_1_SEND_CFG, reg);
		reg &= ~0xFull;
		reg |= 0x09;
		nic_reg_write(nic, NIC_PF_INTF_0_1_SEND_CFG | (1 << 8), reg);
	}

	/* PKIND configuration */
	nic->pkind.minlen = 0;
	nic->pkind.maxlen = NIC_HW_MAX_FRS + ETH_HLEN;
	nic->pkind.lenerr_en = 1;
	nic->pkind.rx_hdr = 0;
	nic->pkind.hdr_sl = 0;

	for (i = 0; i < NIC_MAX_PKIND; i++)
		nic_reg_write(nic, NIC_PF_PKIND_0_15_CFG | (i << 3),
			      *(u64 *)&nic->pkind);

	nic_set_tx_pkt_pad(nic, NIC_HW_MIN_FRS);

	/* Timer config */
	nic_reg_write(nic, NIC_PF_INTR_TIMER_CFG, NICPF_CLK_PER_INT_TICK);
}

/* Channel parse index configuration */
static void nic_config_cpi(struct nicpf *nic, struct cpi_cfg_msg *cfg)
{
	u32 vnic, bgx, lmac, chan;
	u32 padd, cpi_count = 0;
	u64 cpi_base, cpi, rssi_base, rssi;
	u8  qset, rq_idx = 0;

	vnic = cfg->vf_id;
	bgx = NIC_GET_BGX_FROM_VF_LMAC_MAP(nic->vf_lmac_map[vnic]);
	lmac = NIC_GET_LMAC_FROM_VF_LMAC_MAP(nic->vf_lmac_map[vnic]);

	chan = (lmac * MAX_BGX_CHANS_PER_LMAC) + (bgx * NIC_CHANS_PER_INF);
	cpi_base = (lmac * NIC_MAX_CPI_PER_LMAC) + (bgx * NIC_CPI_PER_BGX);
	rssi_base = (lmac * nic->rss_ind_tbl_size) + (bgx * NIC_RSSI_PER_BGX);

	/* Rx channel configuration */
	nic_reg_write(nic, NIC_PF_CHAN_0_255_RX_BP_CFG | (chan << 3),
		      (1ull << 63) | (vnic << 0));
	nic_reg_write(nic, NIC_PF_CHAN_0_255_RX_CFG | (chan << 3),
		      ((u64)cfg->cpi_alg << 62) | (cpi_base << 48));

	if (cfg->cpi_alg == CPI_ALG_NONE)
		cpi_count = 1;
	else if (cfg->cpi_alg == CPI_ALG_VLAN) /* 3 bits of PCP */
		cpi_count = 8;
	else if (cfg->cpi_alg == CPI_ALG_VLAN16) /* 3 bits PCP + DEI */
		cpi_count = 16;
	else if (cfg->cpi_alg == CPI_ALG_DIFF) /* 6bits DSCP */
		cpi_count = NIC_MAX_CPI_PER_LMAC;

	/* RSS Qset, Qidx mapping */
	qset = cfg->vf_id;
	rssi = rssi_base;
	for (; rssi < (rssi_base + cfg->rq_cnt); rssi++) {
		nic_reg_write(nic, NIC_PF_RSSI_0_4097_RQ | (rssi << 3),
			      (qset << 3) | rq_idx);
		rq_idx++;
	}

	rssi = 0;
	cpi = cpi_base;
	for (; cpi < (cpi_base + cpi_count); cpi++) {
		/* Determine port to channel adder */
		if (cfg->cpi_alg != CPI_ALG_DIFF)
			padd = cpi % cpi_count;
		else
			padd = cpi % 8; /* 3 bits CS out of 6bits DSCP */

		/* Leave RSS_SIZE as '0' to disable RSS */
		nic_reg_write(nic, NIC_PF_CPI_0_2047_CFG | (cpi << 3),
			      (vnic << 24) | (padd << 16) | (rssi_base + rssi));

		if ((rssi + 1) >= cfg->rq_cnt)
			continue;

		if (cfg->cpi_alg == CPI_ALG_VLAN)
			rssi++;
		else if (cfg->cpi_alg == CPI_ALG_VLAN16)
			rssi = ((cpi - cpi_base) & 0xe) >> 1;
		else if (cfg->cpi_alg == CPI_ALG_DIFF)
			rssi = ((cpi - cpi_base) & 0x38) >> 3;
	}
	nic->cpi_base[cfg->vf_id] = cpi_base;
}

#ifdef VNIC_RSS_SUPPORT
/* Responsds to VF with its RSS indirection table size */
static void nic_send_rss_size(struct nicpf *nic, int vf)
{
	struct nic_mbx mbx = {};
	u64  *msg;

	msg = (u64 *)&mbx;

	mbx.msg = NIC_MBOX_MSG_RSS_SIZE;
	mbx.data.rss_size.ind_tbl_size = nic->rss_ind_tbl_size;
	nic_send_msg_to_vf(nic, vf, &mbx);
}

/* Receive side scaling configuration
 * configure:
 * - RSS index
 * - indir table i.e hash::RQ mapping
 * - no of hash bits to consider
 */
static void nic_config_rss(struct nicpf *nic, struct rss_cfg_msg *cfg)
{
	u8  qset, idx = 0;
	u64 cpi_cfg, cpi_base, rssi_base, rssi;

	cpi_base = nic->cpi_base[cfg->vf_id];
	cpi_cfg = nic_reg_read(nic, NIC_PF_CPI_0_2047_CFG | (cpi_base << 3));
	rssi_base = (cpi_cfg & 0x0FFF) + cfg->tbl_offset;

	rssi = rssi_base;
	qset = cfg->vf_id;

	for (; rssi < (rssi_base + cfg->tbl_len); rssi++) {
		nic_reg_write(nic, NIC_PF_RSSI_0_4097_RQ | (rssi << 3),
			      (qset << 3) | (cfg->ind_tbl[idx] & 0x7));
		idx++;
	}

	cpi_cfg &= ~(0xFULL << 20);
	cpi_cfg |= (cfg->hash_bits << 20);
	nic_reg_write(nic, NIC_PF_CPI_0_2047_CFG | (cpi_base << 3), cpi_cfg);
}
#endif

/* 4 level transmit side scheduler configutation
 * for TNS bypass mode
 *
 * Sample configuration for SQ0
 * VNIC0-SQ0 -> TL4(0)   -> TL3[0]   -> TL2[0]  -> TL1[0] -> BGX0
 * VNIC1-SQ0 -> TL4(8)   -> TL3[2]   -> TL2[0]  -> TL1[0] -> BGX0
 * VNIC2-SQ0 -> TL4(16)  -> TL3[4]   -> TL2[1]  -> TL1[0] -> BGX0
 * VNIC3-SQ0 -> TL4(24)  -> TL3[6]   -> TL2[1]  -> TL1[0] -> BGX0
 * VNIC4-SQ0 -> TL4(512) -> TL3[128] -> TL2[32] -> TL1[1] -> BGX1
 * VNIC5-SQ0 -> TL4(520) -> TL3[130] -> TL2[32] -> TL1[1] -> BGX1
 * VNIC6-SQ0 -> TL4(528) -> TL3[132] -> TL2[33] -> TL1[1] -> BGX1
 * VNIC7-SQ0 -> TL4(536) -> TL3[134] -> TL2[33] -> TL1[1] -> BGX1
 */
static void nic_tx_channel_cfg(struct nicpf *nic, u8 vnic, u8 sq_idx)
{
	u32 bgx, lmac, chan;
	u32 tl2, tl3, tl4;
	u32 rr_quantum;

	bgx = NIC_GET_BGX_FROM_VF_LMAC_MAP(nic->vf_lmac_map[vnic]);
	lmac = NIC_GET_LMAC_FROM_VF_LMAC_MAP(nic->vf_lmac_map[vnic]);
	/* 24 bytes for FCS, IPG and preamble */
	rr_quantum = ((NIC_HW_MAX_FRS + 24) / 4);

	tl4 = (lmac * NIC_TL4_PER_LMAC) + (bgx * NIC_TL4_PER_BGX);
	tl4 += sq_idx;
	tl3 = tl4 / (NIC_MAX_TL4 / NIC_MAX_TL3);
	nic_reg_write(nic, NIC_PF_QSET_0_127_SQ_0_7_CFG2 |
		      ((u64)vnic << NIC_QS_ID_SHIFT) |
		      ((u32)sq_idx << NIC_Q_NUM_SHIFT), tl4);
	nic_reg_write(nic, NIC_PF_TL4_0_1023_CFG | (tl4 << 3),
		      ((u64)vnic << 27) | ((u32)sq_idx << 24) | rr_quantum);

	nic_reg_write(nic, NIC_PF_TL3_0_255_CFG | (tl3 << 3), rr_quantum);
	chan = (lmac * MAX_BGX_CHANS_PER_LMAC) + (bgx * NIC_CHANS_PER_INF);
	nic_reg_write(nic, NIC_PF_TL3_0_255_CHAN | (tl3 << 3), chan);
	/* Enable backpressure on the channel */
	nic_reg_write(nic, NIC_PF_CHAN_0_255_TX_CFG | (chan << 3), 1);

	tl2 = tl3 >> 2;
	nic_reg_write(nic, NIC_PF_TL3A_0_63_CFG | (tl2 << 3), tl2);
	nic_reg_write(nic, NIC_PF_TL2_0_63_CFG | (tl2 << 3), rr_quantum);
	/* No priorities as of now */
	nic_reg_write(nic, NIC_PF_TL2_0_63_PRI | (tl2 << 3), 0x00);
}

static void nic_mbx_intr_handler (struct nicpf *nic, int mbx)
{
	u64 intr;
	u8  vf, vf_per_mbx_reg = 64;

	intr = nic_reg_read(nic, NIC_PF_MAILBOX_INT + (mbx << 3));
	nic_dbg(&nic->pdev->dev, "PF interrupt Mbox%d 0x%llx\n", mbx, intr);
	for (vf = 0; vf < vf_per_mbx_reg; vf++) {
		if (intr & (1ULL << vf)) {
			nic_dbg(&nic->pdev->dev, "Intr from VF %d\n",
				vf + (mbx * vf_per_mbx_reg));
			if ((vf + (mbx * vf_per_mbx_reg)) > nic->num_vf_en)
				break;
			nic_handle_mbx_intr(nic, vf + (mbx * vf_per_mbx_reg));
			nic_clear_mbx_intr(nic, vf, mbx);
		}
	}
}

static irqreturn_t nic_mbx0_intr_handler (int irq, void *nic_irq)
{
	struct nicpf *nic = (struct nicpf *)nic_irq;

	nic_mbx_intr_handler(nic, 0);

	return IRQ_HANDLED;
}

static irqreturn_t nic_mbx1_intr_handler (int irq, void *nic_irq)
{
	struct nicpf *nic = (struct nicpf *)nic_irq;

	nic_mbx_intr_handler(nic, 1);

	return IRQ_HANDLED;
}

static int nic_enable_msix(struct nicpf *nic)
{
	int i, ret;

	nic->num_vec = NIC_PF_MSIX_VECTORS;

	for (i = 0; i < nic->num_vec; i++)
		nic->msix_entries[i].entry = i;

	ret = pci_enable_msix(nic->pdev, nic->msix_entries, nic->num_vec);
	if (ret) {
		netdev_err(nic->netdev,
			   "Request for #%d msix vectors failed\n",
			   nic->num_vec);
		return 0;
	}

	nic->msix_enabled = 1;
	return 1;
}

static void nic_disable_msix(struct nicpf *nic)
{
	if (nic->msix_enabled) {
		pci_disable_msix(nic->pdev);
		nic->msix_enabled = 0;
		nic->num_vec = 0;
	}
}

static int nic_register_interrupts(struct nicpf *nic)
{
	int irq, ret = 0;

	/* Enable MSI-X */
	if (!nic_enable_msix(nic))
		return 1;

	/* Register mailbox interrupt handlers */
	ret = request_irq(nic->msix_entries[NIC_PF_INTR_ID_MBOX0].vector,
			  nic_mbx0_intr_handler, 0 , "NIC Mbox0", nic);
	if (ret)
		goto fail;
	else
		nic->irq_allocated[NIC_PF_INTR_ID_MBOX0] = true;

	ret = request_irq(nic->msix_entries[NIC_PF_INTR_ID_MBOX1].vector,
			  nic_mbx1_intr_handler, 0 , "NIC Mbox1", nic);
	if (ret)
		goto fail;
	else
		nic->irq_allocated[NIC_PF_INTR_ID_MBOX1] = true;

	/* Enable mailbox interrupt */
	nic_enable_mbx_intr(nic);
	return 0;
fail:
	netdev_err(nic->netdev, "Request irq failed\n");
	for (irq = 0; irq < nic->num_vec; irq++) {
		if (nic->irq_allocated[irq])
			free_irq(nic->msix_entries[irq].vector, nic);
		nic->irq_allocated[irq] = false;
	}
	return 1;
}

static void nic_unregister_interrupts(struct nicpf *nic)
{
	int irq;

	/* Free registered interrupts */
	for (irq = 0; irq < nic->num_vec; irq++) {
		if (nic->irq_allocated[irq])
			free_irq(nic->msix_entries[irq].vector, nic);
		nic->irq_allocated[irq] = false;
	}

	/* Disable MSI-X */
	nic_disable_msix(nic);
}

int nic_sriov_configure(struct pci_dev *pdev, int num_vfs_requested)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct nicpf *nic = netdev_priv(netdev);
	int err;

	if (nic->num_vf_en == num_vfs_requested)
		return num_vfs_requested;

	if (nic->flags & NIC_SRIOV_ENABLED) {
		pci_disable_sriov(pdev);
		nic->flags &= ~NIC_SRIOV_ENABLED;
	}

	nic->num_vf_en = 0;
	if (num_vfs_requested > MAX_NUM_VFS_SUPPORTED)
		return -EPERM;

	if (num_vfs_requested) {
		err = pci_enable_sriov(pdev, num_vfs_requested);
		if (err) {
			dev_err(&pdev->dev, "SRIOV, Failed to enable %d VFs\n",
				num_vfs_requested);
			return err;
		}
		nic->num_vf_en = num_vfs_requested;
		nic->flags |= NIC_SRIOV_ENABLED;
	}

	return num_vfs_requested;
}

static int nic_sriov_init(struct pci_dev *pdev, struct nicpf *nic)
{
	int    pos = 0;

	pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_SRIOV);
	if (!pos) {
		dev_err(&pdev->dev, "SRIOV capability is not found in PCIe config space\n");
		return 0;
	}

	pci_read_config_word(pdev, (pos + PCI_SRIOV_TOTAL_VF),
			     &nic->total_vf_cnt);
	if (nic->total_vf_cnt < nic->num_vf_en)
		nic->num_vf_en = nic->total_vf_cnt;

	if (nic->total_vf_cnt && pci_enable_sriov(pdev, nic->num_vf_en)) {
		dev_err(&pdev->dev, "SRIOV enable failed, num VF is %d\n",
			nic->num_vf_en);
		nic->num_vf_en = 0;
		return 0;
	}
	dev_info(&pdev->dev, "SRIOV enabled, numer of VF available %d\n",
		 nic->num_vf_en);

	nic->flags |= NIC_SRIOV_ENABLED;
	return 1;
}

/* Poll for BGX LMAC link status and update corresponding VF
 * if there is a change, valid only if internal L2 switch
 * is not present otherwise VF link is always treated as up
 */
static void nic_poll_for_link(struct work_struct *work)
{
	struct nic_mbx mbx = {};
	struct nicpf *nic;
	struct bgx_link_status link;
	u8 vf, bgx, lmac;

	nic = container_of(work, struct nicpf, dwork.work);

	mbx.msg = NIC_MBOX_MSG_BGX_LINK_CHANGE;

	for (vf = 0; vf < nic->num_vf_en; vf++) {
		/* Poll only if VF is UP */
		if (!nic->vf_enabled[vf])
			continue;

		/* Get BGX, LMAC indices for the VF */
		bgx = NIC_GET_BGX_FROM_VF_LMAC_MAP(nic->vf_lmac_map[vf]);
		lmac = NIC_GET_LMAC_FROM_VF_LMAC_MAP(nic->vf_lmac_map[vf]);
		/* Get interface link status */
		bgx_get_lmac_link_state(nic->node, bgx, lmac, &link);

		/* Inform VF only if link status changed */
		if (nic->link[vf] == link.link_up)
			continue;

		if (!nic->mbx_lock[vf]) {
			nic->link[vf] = link.link_up;
			nic->duplex[vf] = link.duplex;
			nic->speed[vf] = link.speed;

			/* Send a mbox message to VF with current link status */
			mbx.data.link_status.link_up = link.link_up;
			mbx.data.link_status.duplex = link.duplex;
			mbx.data.link_status.speed = link.speed;
			nic_send_msg_to_vf(nic, vf, &mbx);
		}
	}
	queue_delayed_work(nic->check_link, &nic->dwork, HZ * 2);
}

static int nic_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	struct net_device *netdev;
	struct nicpf *nic;
	int    err;

	netdev = alloc_etherdev(sizeof(struct nicpf));
	if (!netdev)
		return -ENOMEM;

	pci_set_drvdata(pdev, netdev);

	SET_NETDEV_DEV(netdev, &pdev->dev);

	nic = netdev_priv(netdev);
	nic->netdev = netdev;
	nic->pdev = pdev;

	err = pci_enable_device(pdev);
	if (err) {
		dev_err(dev, "Failed to enable PCI device\n");
		goto exit;
	}

	err = pci_request_regions(pdev, DRV_NAME);
	if (err) {
		dev_err(dev, "PCI request regions failed 0x%x\n", err);
		goto err_disable_device;
	}

	err = pci_set_dma_mask(pdev, DMA_BIT_MASK(48));
	if (err) {
		dev_err(dev, "Unable to get usable DMA configuration\n");
		goto err_release_regions;
	}

	err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(48));
	if (err) {
		dev_err(dev, "unable to get 48-bit DMA for consistent allocations\n");
		goto err_release_regions;
	}

	/* MAP PF's configuration registers */
	nic->reg_base = (u64)pci_ioremap_bar(pdev, PCI_CFG_REG_BAR_NUM);
	if (!nic->reg_base) {
		dev_err(dev, "Cannot map config register space, aborting\n");
		err = -ENOMEM;
		goto err_release_regions;
	}

	pci_read_config_byte(pdev, PCI_REVISION_ID, &nic->rev_id);

	nic->node = NIC_NODE_ID(pci_resource_start(pdev, PCI_CFG_REG_BAR_NUM));

	/* By default set NIC in TNS bypass mode */
	nic->flags &= ~NIC_TNS_ENABLED;
	nic_set_lmac_vf_mapping(nic);

	/* Initialize hardware */
	nic_init_hw(nic);

	/* Set RSS TBL size for each VF */
	nic->rss_ind_tbl_size = NIC_MAX_RSS_IDR_TBL_SIZE;

	/* Register interrupts */
	if (nic_register_interrupts(nic))
		goto err_unmap_resources;

	/* Configure SRIOV */
	if (!nic_sriov_init(pdev, nic))
		goto err_unmap_resources;

	if (!nic_get_mac_addresses(nic))
		dev_info(&pdev->dev, " Mac node not present in dts\n");

	if (nic->flags & NIC_TNS_ENABLED)
		goto exit;

	/* Register a physical link status poll fn() */
	nic->check_link = alloc_workqueue("check_link_status",
					  WQ_UNBOUND | WQ_MEM_RECLAIM, 1);
	if (!nic->check_link)
		return -ENOMEM;
	INIT_DELAYED_WORK(&nic->dwork, nic_poll_for_link);
	queue_delayed_work(nic->check_link, &nic->dwork, 0);

	goto exit;

err_unmap_resources:
	if (nic->reg_base)
		iounmap((void *)nic->reg_base);
err_release_regions:
	pci_release_regions(pdev);
err_disable_device:
	pci_disable_device(pdev);
exit:
	return err;
}

static void nic_remove(struct pci_dev *pdev)
{
	struct net_device *netdev = pci_get_drvdata(pdev);
	struct nicpf *nic;

	if (!netdev)
		return;

	nic = netdev_priv(netdev);

	nic_unregister_interrupts(nic);

	if (nic->flags & NIC_SRIOV_ENABLED)
		pci_disable_sriov(pdev);

	pci_set_drvdata(pdev, NULL);

	if (nic->reg_base)
		iounmap((void *)nic->reg_base);

	pci_release_regions(pdev);
	pci_disable_device(pdev);
	free_netdev(netdev);
}

static struct pci_driver nic_driver = {
	.name = DRV_NAME,
	.id_table = nic_id_table,
	.probe = nic_probe,
	.remove = nic_remove,
	.sriov_configure = nic_sriov_configure,
};

static int __init nic_init_module(void)
{
	pr_info("%s, ver %s\n", DRV_NAME, DRV_VERSION);

	return pci_register_driver(&nic_driver);
}

static void __exit nic_cleanup_module(void)
{
	pci_unregister_driver(&nic_driver);
}

module_init(nic_init_module);
module_exit(nic_cleanup_module);

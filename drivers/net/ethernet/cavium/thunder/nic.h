/*
 * Copyright (C) 2015 Cavium, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */

#ifndef NIC_H
#define	NIC_H

#include <linux/netdevice.h>
#include <linux/interrupt.h>
#include "thunder_bgx.h"

/* PCI device IDs */
#define	PCI_DEVICE_ID_THUNDER_NIC_PF		0xA01E
#define	PCI_DEVICE_ID_THUNDER_PASS1_NIC_VF	0x0011
#define	PCI_DEVICE_ID_THUNDER_NIC_VF		0xA034
#define	PCI_DEVICE_ID_THUNDER_BGX		0xA026

/* PCI BAR nos */
#define	PCI_CFG_REG_BAR_NUM		0
#define	PCI_MSIX_REG_BAR_NUM		4

/* NIC SRIOV VF count */
#define	MAX_NUM_VFS_SUPPORTED		128
#define	DEFAULT_NUM_VF_ENABLED		8

#define	NIC_TNS_BYPASS_MODE		0
#define	NIC_TNS_MODE			1

/* NIC priv flags */
#define	NIC_SRIOV_ENABLED		(1 << 0)
#define	NIC_TNS_ENABLED			(1 << 1)

/* VNIC HW optimiation features */
#undef VNIC_RSS_SUPPORT /* 3.10 doesn't support it */

/* Min/Max packet size */
#define	NIC_HW_MIN_FRS			64
#define	NIC_HW_MAX_FRS			9200 /* 9216 max packet including FCS */

/* Max pkinds */
#define	NIC_MAX_PKIND			16

/* Rx Channels */
/* Receive channel configuration in TNS bypass mode
 * Below is configuration in TNS bypass mode
 * BGX0-LMAC0-CHAN0 - VNIC CHAN0
 * BGX0-LMAC1-CHAN0 - VNIC CHAN16
 * ...
 * BGX1-LMAC0-CHAN0 - VNIC CHAN128
 * ...
 * BGX1-LMAC3-CHAN0 - VNIC CHAN174
 */
#define	NIC_INTF_COUNT			2  /* Interfaces btw VNIC and TNS/BGX */
#define	NIC_CHANS_PER_INF		128
#define	NIC_MAX_CHANS			(NIC_INTF_COUNT * NIC_CHANS_PER_INF)
#define	NIC_CPI_COUNT			2048 /* No of channel parse indices */

/* TNS bypass mode: 1-1 mapping between VNIC and BGX:LMAC */
#define NIC_MAX_BGX			MAX_BGX_PER_CN88XX
#define	NIC_CPI_PER_BGX			(NIC_CPI_COUNT / NIC_MAX_BGX)
#define	NIC_MAX_CPI_PER_LMAC		64 /* Max when CPI_ALG is IP diffserv */
#define	NIC_RSSI_PER_BGX		(NIC_RSSI_COUNT / NIC_MAX_BGX)

/* Tx scheduling */
#define	NIC_MAX_TL4			1024
#define	NIC_MAX_TL4_SHAPERS		256 /* 1 shaper for 4 TL4s */
#define	NIC_MAX_TL3			256
#define	NIC_MAX_TL3_SHAPERS		64  /* 1 shaper for 4 TL3s */
#define	NIC_MAX_TL2			64
#define	NIC_MAX_TL2_SHAPERS		2  /* 1 shaper for 32 TL2s */
#define	NIC_MAX_TL1			2

/* TNS bypass mode */
#define	NIC_TL2_PER_BGX			32
#define	NIC_TL4_PER_BGX			(NIC_MAX_TL4 / NIC_MAX_BGX)
#define	NIC_TL4_PER_LMAC		(NIC_MAX_TL4 / NIC_CHANS_PER_INF)

/* NIC VF Interrupts */
#define	NICVF_INTR_CQ			0
#define	NICVF_INTR_SQ			1
#define	NICVF_INTR_RBDR			2
#define	NICVF_INTR_PKT_DROP		3
#define	NICVF_INTR_TCP_TIMER		4
#define	NICVF_INTR_MBOX			5
#define	NICVF_INTR_QS_ERR		6

#define	NICVF_INTR_CQ_SHIFT		0
#define	NICVF_INTR_SQ_SHIFT		8
#define	NICVF_INTR_RBDR_SHIFT		16
#define	NICVF_INTR_PKT_DROP_SHIFT	20
#define	NICVF_INTR_TCP_TIMER_SHIFT	21
#define	NICVF_INTR_MBOX_SHIFT		22
#define	NICVF_INTR_QS_ERR_SHIFT		23

#define	NICVF_INTR_CQ_MASK		(0xFF << NICVF_INTR_CQ_SHIFT)
#define	NICVF_INTR_SQ_MASK		(0xFF << NICVF_INTR_SQ_SHIFT)
#define	NICVF_INTR_RBDR_MASK		(0x03 << NICVF_INTR_RBDR_SHIFT)
#define	NICVF_INTR_PKT_DROP_MASK	(1 << NICVF_INTR_PKT_DROP_SHIFT)
#define	NICVF_INTR_TCP_TIMER_MASK	(1 << NICVF_INTR_TCP_TIMER_SHIFT)
#define	NICVF_INTR_MBOX_MASK		(1 << NICVF_INTR_MBOX_SHIFT)
#define	NICVF_INTR_QS_ERR_MASK		(1 << NICVF_INTR_QS_ERR_SHIFT)

/* MSI-X interrupts */
#define	NIC_PF_MSIX_VECTORS		10
#define	NIC_VF_MSIX_VECTORS		20

#define NIC_PF_INTR_ID_ECC0_SBE		0
#define NIC_PF_INTR_ID_ECC0_DBE		1
#define NIC_PF_INTR_ID_ECC1_SBE		2
#define NIC_PF_INTR_ID_ECC1_DBE		3
#define NIC_PF_INTR_ID_ECC2_SBE		4
#define NIC_PF_INTR_ID_ECC2_DBE		5
#define NIC_PF_INTR_ID_ECC3_SBE		6
#define NIC_PF_INTR_ID_ECC3_DBE		7
#define NIC_PF_INTR_ID_MBOX0		8
#define NIC_PF_INTR_ID_MBOX1		9

/* Global timer for CQ timer thresh interrupts
 * Calculated for SCLK of 700Mhz
 * value written should be a 1/16th of what is expected
 *
 * 1 tick per 0.05usec = value of 2.2
 * This 10% would be covered in CQ timer thresh value
 */
#define NICPF_CLK_PER_INT_TICK		2

struct nicvf_cq_poll {
	u8	cq_idx;		/* Completion queue index */
	struct	napi_struct napi;
};

#define	NIC_RSSI_COUNT			4096 /* Total no of RSS indices */
#define NIC_MAX_RSS_HASH_BITS		8
#define NIC_MAX_RSS_IDR_TBL_SIZE	(1 << NIC_MAX_RSS_HASH_BITS)
#define RSS_HASH_KEY_SIZE		5 /* 320 bit key */

#ifdef VNIC_RSS_SUPPORT
struct nicvf_rss_info {
	bool enable;
#define	RSS_L2_EXTENDED_HASH_ENA	(1 << 0)
#define	RSS_IP_HASH_ENA			(1 << 1)
#define	RSS_TCP_HASH_ENA		(1 << 2)
#define	RSS_TCP_SYN_DIS			(1 << 3)
#define	RSS_UDP_HASH_ENA		(1 << 4)
#define RSS_L4_EXTENDED_HASH_ENA	(1 << 5)
#define	RSS_ROCE_ENA			(1 << 6)
#define	RSS_L3_BI_DIRECTION_ENA		(1 << 7)
#define	RSS_L4_BI_DIRECTION_ENA		(1 << 8)
	u64 cfg;
	u8  hash_bits;
	u16 rss_size;
	u8  ind_tbl[NIC_MAX_RSS_IDR_TBL_SIZE];
	u64 key[RSS_HASH_KEY_SIZE];
} ____cacheline_aligned_in_smp;
#endif

enum rx_stats_reg_offset {
	RX_OCTS = 0x0,
	RX_UCAST = 0x1,
	RX_BCAST = 0x2,
	RX_MCAST = 0x3,
	RX_RED = 0x4,
	RX_RED_OCTS = 0x5,
	RX_ORUN = 0x6,
	RX_ORUN_OCTS = 0x7,
	RX_FCS = 0x8,
	RX_L2ERR = 0x9,
	RX_DRP_BCAST = 0xa,
	RX_DRP_MCAST = 0xb,
	RX_DRP_L3BCAST = 0xc,
	RX_DRP_L3MCAST = 0xd,
	RX_STATS_ENUM_LAST,
};

enum tx_stats_reg_offset {
	TX_OCTS = 0x0,
	TX_UCAST = 0x1,
	TX_BCAST = 0x2,
	TX_MCAST = 0x3,
	TX_DROP = 0x4,
	TX_STATS_ENUM_LAST,
};

struct nicvf_hw_stats {
	u64 rx_bytes_ok;
	u64 rx_ucast_frames_ok;
	u64 rx_bcast_frames_ok;
	u64 rx_mcast_frames_ok;
	u64 rx_fcs_errors;
	u64 rx_l2_errors;
	u64 rx_drop_red;
	u64 rx_drop_red_bytes;
	u64 rx_drop_overrun;
	u64 rx_drop_overrun_bytes;
	u64 rx_drop_bcast;
	u64 rx_drop_mcast;
	u64 rx_drop_l3_bcast;
	u64 rx_drop_l3_mcast;
	u64 tx_bytes_ok;
	u64 tx_ucast_frames_ok;
	u64 tx_bcast_frames_ok;
	u64 tx_mcast_frames_ok;
	u64 tx_drops;
};

struct nicvf_drv_stats {
	/* Rx */
	u64 rx_frames_ok;
	u64 rx_frames_64;
	u64 rx_frames_127;
	u64 rx_frames_255;
	u64 rx_frames_511;
	u64 rx_frames_1023;
	u64 rx_frames_1518;
	u64 rx_frames_jumbo;
	u64 rx_drops;
	/* Tx */
	u64 tx_frames_ok;
	u64 tx_drops;
	u64 tx_busy;
	u64 tx_tso;
};

struct nicvf {
	struct net_device	*netdev;
	struct pci_dev		*pdev;
	u8			vf_id;
	u8			node;
	u8			tns_mode;
	u16			mtu;
	struct queue_set	*qs;
	u64			reg_base;
	bool			link_up;
	u8			duplex;
	u32			speed;
	struct tasklet_struct	rbdr_task;
	struct tasklet_struct	qs_err_task;
	struct tasklet_struct	cq_task;
	struct nicvf_cq_poll	*napi[8];
#ifdef VNIC_RSS_SUPPORT
	struct nicvf_rss_info	rss_info;
#endif
	u8			cpi_alg;
	/* Interrupt coalescing settings */
	u32			cq_coalesce_usecs;

	u32			msg_enable;
	struct nicvf_hw_stats   stats;
	struct nicvf_drv_stats  drv_stats;
	struct bgx_stats	bgx_stats;
	struct work_struct	reset_task;

	/* MSI-X  */
	bool			msix_enabled;
	u8			num_vec;
	struct msix_entry	msix_entries[NIC_VF_MSIX_VECTORS];
	char			irq_name[NIC_VF_MSIX_VECTORS][20];
	bool			irq_allocated[NIC_VF_MSIX_VECTORS];
} ____cacheline_aligned_in_smp;

struct nicpf {
	struct net_device	*netdev;
	struct pci_dev		*pdev;
	u8			rev_id;
#define NIC_NODE_ID_MASK	0x300000000000
#define NIC_NODE_ID(x)		((x & NODE_ID_MASK) >> 44)
	u8			node;
	unsigned int		flags;
	u16			total_vf_cnt;   /* Total num of VF supported */
	u8			num_vf_en;      /* No of VF enabled */
	bool			vf_enabled[MAX_NUM_VFS_SUPPORTED];
	u64			reg_base;       /* Register start address */
	struct pkind_cfg	pkind;
	u8			bgx_cnt;
#define	NIC_SET_VF_LMAC_MAP(bgx, lmac)	(((bgx & 0xF) << 4) | (lmac & 0xF))
#define	NIC_GET_BGX_FROM_VF_LMAC_MAP(map)	((map >> 4) & 0xF)
#define	NIC_GET_LMAC_FROM_VF_LMAC_MAP(map)	(map & 0xF)
	u8			vf_lmac_map[MAX_LMAC];
	struct delayed_work     dwork;
	struct workqueue_struct *check_link;
	u8			link[MAX_LMAC];
	u8			duplex[MAX_LMAC];
	u32			speed[MAX_LMAC];
	u16			cpi_base[MAX_NUM_VFS_SUPPORTED];
	u16			rss_ind_tbl_size;
	u64			mac[MAX_NUM_VFS_SUPPORTED];
	bool			mbx_lock[MAX_NUM_VFS_SUPPORTED];

	/* MSI-X */
	bool			msix_enabled;
	u8			num_vec;
	struct msix_entry	msix_entries[NIC_PF_MSIX_VECTORS];
	bool			irq_allocated[NIC_PF_MSIX_VECTORS];
} ____cacheline_aligned_in_smp;

/* PF <--> VF Mailbox communication
 * Eight 64bit registers are shared between PF and VF.
 * Separate set for each VF.
 * Writing '1' into last register mbx7 means end of message.
 */

/* PF <--> VF mailbox communication */
#define	NIC_PF_VF_MAILBOX_SIZE		2
#define	NIC_MBOX_MSG_TIMEOUT		2000 /* ms */

/* Mailbox message types */
#define	NIC_MBOX_MSG_READY		0x01	/* Is PF ready to rcv msgs */
#define	NIC_MBOX_MSG_ACK		0x02	/* ACK the message received */
#define	NIC_MBOX_MSG_NACK		0x03	/* NACK the message received */
#define	NIC_MBOX_MSG_QS_CFG		0x04	/* Configure Qset */
#define	NIC_MBOX_MSG_RQ_CFG		0x05	/* Configure receive queue */
#define	NIC_MBOX_MSG_SQ_CFG		0x06	/* Configure Send queue */
#define	NIC_MBOX_MSG_RQ_DROP_CFG	0x07	/* Configure receive queue */
#define	NIC_MBOX_MSG_SET_MAC		0x08	/* Add MAC ID to DMAC filter */
#define	NIC_MBOX_MSG_SET_MAX_FRS	0x09	/* Set max frame size */
#define	NIC_MBOX_MSG_CPI_CFG		0x0A	/* Config CPI, RSSI */
#define	NIC_MBOX_MSG_RSS_SIZE		0x0B	/* Get RSS indir_tbl size */
#define	NIC_MBOX_MSG_RSS_CFG		0x0C	/* Config RSS table */
#define	NIC_MBOX_MSG_RSS_CFG_CONT	0x0D	/* RSS config continuation */
#define	NIC_MBOX_MSG_RQ_BP_CFG		0x0E	/* RQ backpressure config */
#define	NIC_MBOX_MSG_RQ_SW_SYNC		0x0F	/* Flush inflight pkts to RQ */
#define	NIC_MBOX_MSG_BGX_STATS		0x10	/* Get stats from BGX */
#define	NIC_MBOX_MSG_BGX_LINK_CHANGE	0x11	/* BGX:LMAC link status */

struct nic_cfg_msg {
	u8    vf_id;
	u8    tns_mode;
	u8    node_id;
	u8    unused0;
	u16   unused1;
	u64   mac_addr;
} __packed;

/* Qset configuration */
struct qs_cfg_msg {
	u8    num;
	u8    unused0;
	u32   unused1;
	u64   cfg;
} __packed;

/* Receive queue configuration */
struct rq_cfg_msg {
	u8    qs_num;
	u8    rq_num;
	u32   unused0;
	u64   cfg;
} __packed;

/* Send queue configuration */
struct sq_cfg_msg {
	u8    qs_num;
	u8    sq_num;
	u32   unused0;
	u64   cfg;
} __packed;

/* Set VF's MAC address */
struct set_mac_msg {
	u8    vf_id;
	u8    unused0;
	u32   unused1;
	u64   addr;
} __packed;

/* Set Maximum frame size */
struct set_frs_msg {
	u8    vf_id;
	u16   max_frs;
};

/* Set CPI algorithm type */
struct cpi_cfg_msg {
	u8    vf_id;
	u8    rq_cnt;
	u8    cpi_alg;
};

#ifdef VNIC_RSS_SUPPORT
/* Get RSS table size */
struct rss_sz_msg {
	u8    vf_id;
	u16   ind_tbl_size;
};

/* Set RSS configuration */
struct rss_cfg_msg {
	u8    vf_id;
	u8    hash_bits;
	u8    tbl_len;
	u8    tbl_offset;
	u16   unused0;
#define RSS_IND_TBL_LEN_PER_MBX_MSG	8
	u8    ind_tbl[RSS_IND_TBL_LEN_PER_MBX_MSG];
} __packed;
#endif

struct bgx_stats_msg {
	u8    vf_id;
	u8    rx;
	u8    idx;
	u8    unused0;
	u16   unused1;
	u64   stats;
} __packed;

/* Physical interface link status */
struct bgx_link_status {
	u8    link_up;
	u8    duplex;
	u32   speed;
};

/* 128 bit shared memory between PF and each VF */
struct nic_mbx {
	u8    msg;
	u8    unused;
	union	{
		struct nic_cfg_msg	nic_cfg;
		struct qs_cfg_msg	qs;
		struct rq_cfg_msg	rq;
		struct sq_cfg_msg	sq;
		struct set_mac_msg	mac;
		struct set_frs_msg	frs;
		struct cpi_cfg_msg	cpi_cfg;
#ifdef VNIC_RSS_SUPPORT
		struct rss_sz_msg	rss_size;
		struct rss_cfg_msg	rss_cfg;
#endif
		struct bgx_stats_msg    bgx_stats;
		struct bgx_link_status  link_status;
	} data;
} __packed;

int nicvf_set_real_num_queues(struct net_device *netdev,
			      int tx_queues, int rx_queues);
int nicvf_open(struct net_device *netdev);
int nicvf_stop(struct net_device *netdev);
int nicvf_send_msg_to_pf(struct nicvf *vf, struct nic_mbx *mbx);
void nicvf_config_cpi(struct nicvf *nic);
#ifdef VNIC_RSS_SUPPORT
void nicvf_config_rss(struct nicvf *nic);
void nicvf_set_rss_key(struct nicvf *nic);
#endif
void nicvf_free_skb(struct nicvf *nic, struct sk_buff *skb);
void nicvf_set_ethtool_ops(struct net_device *netdev);
void nicvf_update_stats(struct nicvf *nic);
void nicvf_update_lmac_stats(struct nicvf *nic);

/* Debug */
#undef	NIC_DEBUG

#ifdef	NIC_DEBUG
#define	nic_dbg(dev, fmt, arg...) \
		dev_info(dev, fmt, ##arg)
#else
#define	nic_dbg(dev, fmt, arg...) do {} while (0)
#endif

#endif /* NIC_H */

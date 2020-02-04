/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2013 Cavium, Inc.
 */
#ifndef _ASM_KVM_MIPS_VZ_H
#define _ASM_KVM_MIPS_VZ_H

#include <linux/threads.h>

#define KVM_MIPSVZ_IO_START		0x10000000
#define KVM_MIPSVZ_IO_END		0x20000000
#define KVM_MIPSVZ_MMIO_START		0x10000000
#define KVM_MIPSVZ_MMIO_END		0x1d000000
#define KVM_MIPSVZ_IOPORT_START		0x1e000000
#define KVM_MIPSVZ_IOPORT_END		0x1e010000
#define KVM_MIPSVZ_IRQCHIP_START	0x1e010000
#define KVM_MIPSVZ_IRQCHIP_END		0x1e020000

struct kvm;

struct mipsvz_irq_chip {
	u32	version;	/* version of irq-chip implementation */
	u32	numbits;	/* # of supported interrupt bits */
	u32	numcpus;	/* # of supported CPUs */
	u32	bm_length;	/* length as u32 of used bitmaps */
	u32	bm_size;	/* size of used bitmaps */

	struct page *page;
	void *base;

	/* per CPU irq-source bitmaps to signal interrupt to guest */
	unsigned long cpu_irq_src_bitmap;
	/* per CPU irq-pending bitmaps (for MBOX irqs) */
	unsigned long cpu_irq_pend_bitmap;
	/* per CPU irq-enable bitmaps for CPU affinity and esp. MBOX irqs */
	unsigned long cpu_irq_en_bitmap;
	/* one global irq-pending bitmap */
	unsigned long irq_pend_bitmap;
	/* one global irq-enable bitmap (to (un)mask PCI-irq/MSI globally */
	unsigned long irq_en_bitmap;
	/* temporary bitmap esp used in mipsvz_wr_irqchip_new_irqs */
	unsigned long tmp_bitmap;
};

#define KVM_MIPSVZ_IC_REG_NUM_BITS 0	/* number of IRQs supported */
#define KVM_MIPSVZ_IC_REG_NUM_CPUS 4	/* number of CPUs supported */
#define KVM_MIPSVZ_IC_REG_VERSION  8	/* version of this irq_chip */
#define KVM_MIPSVZ_IC_REG_FEATURES 0xc	/* feature flags (if any) */

#define KVM_MIPSVZ_IC_REG_IRQ_SET  0x10	/* set irq pending (except MBOX) */
#define KVM_MIPSVZ_IC_REG_IRQ_CLR  0x14	/* clear irq pending (except MBOX) */
#define KVM_MIPSVZ_IC_REG_IRQ_EN   0x18	/* enable irq globally (except MBOX) */
#define KVM_MIPSVZ_IC_REG_IRQ_DIS  0x1c	/* disable irq globally (except MBOX) */

#define KVM_MIPSVZ_IC_REG_CPU_IRQ_SET 0x20	/* set irq pending (MBOX) */
#define KVM_MIPSVZ_IC_REG_CPU_IRQ_CLR 0x24	/* clear irq pending (MBOX) */
#define KVM_MIPSVZ_IC_REG_CPU_IRQ_EN  0x28	/* enable irq per CPU */
#define KVM_MIPSVZ_IC_REG_CPU_IRQ_DIS 0x2c	/* disable irq per CPU */

/* mips_irq_chip MMIO area containing bitmaps */
#define KVM_MIPSVZ_IC_BM_AREA		0x40

#define KVM_MIPSVZ_IC_NUM_BITS		128
#define KVM_MIPSVZ_IC_NUM_CPUS		CONFIG_NR_CPUS
#define KVM_MIPSVZ_IC_VERSION		1

struct kvm_mips_vz {
	struct mutex guest_mm_lock;
	pgd_t *pgd;			/* Translations for this host. */
	spinlock_t irq_chip_lock;
	struct mipsvz_irq_chip *irq_chip;
	unsigned int asid[NR_CPUS];	/* Per CPU ASIDs for pgd. */
};

struct kvm_mips_vcpu_vz {
	struct kvm_vcpu *vcpu;
	u64 c0_entrylo0;
	u64 c0_entrylo1;
	u64 c0_context;
	u64 c0_userlocal;
	u64 c0_badvaddr;
	u64 c0_entryhi;
	u64 c0_epc;
	u64 c0_ebase;
	u64 c0_xcontext;
	u64 c0_errorepc;
	u64 c0_kscratch[6];
	u32 c0_pagemask;
	u32 c0_pagegrain;
	u32 c0_wired;
	u32 c0_hwrena;
	u32 c0_compare;
	u32 c0_status;
	u32 c0_cause;
	u32 c0_index;

	u32 c0_count; /* Not settable, value at last exit. */
	u32 c0_count_offset;

	int tlb_size;
	struct mipsvz_kvm_tlb_entry *tlb_state;

	u32 last_exit_insn;
	/* Saved  mips_kvm_rootsp[] value when we are off the CPU. */
	unsigned long rootsp;
	/* ASID used in guest context. */
	unsigned int guest_asid;
	/* ASID for kernel context, restored when leaving guest.*/
	unsigned int mm_asid;

	/* Protected by kvm_arch.irq_chip_lock, the value of Guestctl2[VIP] */
	u8 injected_ipx;

	struct hrtimer compare_timer;
	ktime_t compare_timer_read;

	bool have_counter_state;
};

struct kvm_mips_vz_regs {
	struct pt_regs pt;
	/* Only populated on TLB exceptions */
	unsigned int cp0_badinstr;
	unsigned int cp0_badinstrp;
};

int mipsvz_arch_init(const void *opaque);
void mipsvz_arch_exit(void);
int mipsvz_arch_hardware_enable(void *garbage);
int mipsvz_init_vm(struct kvm *kvm, unsigned long type);

#endif /* _ASM_KVM_MIPS_VZ_H */

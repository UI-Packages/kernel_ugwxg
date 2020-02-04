/*
* This file is subject to the terms and conditions of the GNU General Public
* License.  See the file "COPYING" in the main directory of this archive
* for more details.
*
* Copyright (C) 2012  MIPS Technologies, Inc.  All rights reserved.
* Authors: Sanjay Lal <sanjayl@kymasys.com>
*/

#ifndef __MIPS_KVM_HOST_H__
#define __MIPS_KVM_HOST_H__

#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/kvm.h>
#include <linux/kvm_types.h>
#include <linux/threads.h>
#include <linux/spinlock.h>


#define KVM_MAX_VCPUS		CONFIG_NR_CPUS
#define KVM_USER_MEM_SLOTS	8
/* memory slots that does not exposed to userspace */
#define KVM_PRIVATE_MEM_SLOTS 	0

#ifdef CONFIG_KVM_MIPS_TE
#define KVM_COALESCED_MMIO_PAGE_OFFSET 1
#endif

/* Don't support huge pages */
#define KVM_HPAGE_GFN_SHIFT(x)	0

/* We don't currently support large pages. */
#define KVM_NR_PAGE_SIZES	1
#define KVM_PAGES_PER_HPAGE(x)	1


struct kvm;
struct kvm_vcpu;
enum kvm_mr_change;

struct kvm_vm_stat {
	u32 remote_tlb_flush;
};

struct kvm_vcpu_stat {
	u32 wait_exits;
	u32 cache_exits;
	u32 signal_exits;
	u32 int_exits;
	u32 cop_unusable_exits;
	u32 tlbmod_exits;
	u32 tlbmiss_ld_exits;
	u32 tlbmiss_st_exits;
	u32 addrerr_st_exits;
	u32 addrerr_ld_exits;
	u32 syscall_exits;
	u32 resvd_inst_exits;
	u32 break_inst_exits;
	u32 flush_dcache_exits;
	u32 halt_wakeup;
};

struct kvm_arch_memory_slot {
};

struct kvm_mips_ops {
	int (*vcpu_runnable)(struct kvm_vcpu *vcpu);
	void (*free_vcpus)(struct kvm *kvm);
	void (*destroy_vm)(struct kvm *kvm);
	void (*commit_memory_region)(struct kvm *kvm,
				     struct kvm_userspace_memory_region *mem,
				     const struct kvm_memory_slot *old,
				     enum kvm_mr_change change);
	struct kvm_vcpu *(*vcpu_create)(struct kvm *kvm, unsigned int id);
	void (*vcpu_free)(struct kvm_vcpu *vcpu);
	int (*vcpu_run)(struct kvm_vcpu *vcpu, struct kvm_run *run);
	long (*vm_ioctl)(struct kvm *kvm, unsigned int ioctl,
			 unsigned long arg);
	long (*vcpu_ioctl)(struct kvm_vcpu *vcpu, unsigned int ioctl,
			   unsigned long arg);
	int (*get_reg)(struct kvm_vcpu *vcpu, const struct kvm_one_reg *reg, s64 *v);
	int (*set_reg)(struct kvm_vcpu *vcpu, const struct kvm_one_reg *reg,
		       u64 v);
	int (*cpu_has_pending_timer)(struct kvm_vcpu *vcpu);
	int (*vcpu_init)(struct kvm_vcpu *vcpu);
	int (*vcpu_setup)(struct kvm_vcpu *vcpu);
	void (*vcpu_load)(struct kvm_vcpu *vcpu, int cpu);
	void (*vcpu_put)(struct kvm_vcpu *vcpu);
};

struct kvm_arch {
	const struct kvm_mips_ops *ops;
	void *impl;
};


struct kvm_vcpu_arch {
	/* GPRS */
	unsigned long gprs[32];
	unsigned long hi;
	unsigned long lo;
	unsigned long epc;

	/* FPU state */
	u64 fpr[32];
	u32 fir;
	u32 fccr;
	u32 fexr;
	u32 fenr;
	u32 fcsr;

	void *impl;
};

#ifdef CONFIG_KVM_MIPS_VZ

/* should be maximum of number of pins for each of the irqchips */
#define KVM_IRQCHIP_NUM_PINS  128

static inline int irqchip_in_kernel(struct kvm *kvm)
{
	return 1;
}

#endif

#endif /* __MIPS_KVM_HOST_H__ */

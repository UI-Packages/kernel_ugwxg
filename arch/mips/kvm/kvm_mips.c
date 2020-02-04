/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * KVM/MIPS: MIPS specific KVM APIs
 *
 * Copyright (C) 2012  MIPS Technologies, Inc.  All rights reserved.
 * Authors: Sanjay Lal <sanjayl@kymasys.com>
*/

#include <linux/errno.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/fs.h>
#include <linux/bootmem.h>
#include <linux/kvm_host.h>
#include <asm/page.h>
#include <asm/cacheflush.h>
#include <asm/mmu_context.h>
#include <asm/kvm_mips_te.h>
#include <asm/kvm_mips_vz.h>

#define VCPU_STAT(x) offsetof(struct kvm_vcpu, stat.x), KVM_STAT_VCPU
struct kvm_stats_debugfs_item debugfs_entries[] = {
	{ "wait", VCPU_STAT(wait_exits) },
	{ "cache", VCPU_STAT(cache_exits) },
	{ "signal", VCPU_STAT(signal_exits) },
	{ "interrupt", VCPU_STAT(int_exits) },
	{ "cop_unsuable", VCPU_STAT(cop_unusable_exits) },
	{ "tlbmod", VCPU_STAT(tlbmod_exits) },
	{ "tlbmiss_ld", VCPU_STAT(tlbmiss_ld_exits) },
	{ "tlbmiss_st", VCPU_STAT(tlbmiss_st_exits) },
	{ "addrerr_st", VCPU_STAT(addrerr_st_exits) },
	{ "addrerr_ld", VCPU_STAT(addrerr_ld_exits) },
	{ "syscall", VCPU_STAT(syscall_exits) },
	{ "resvd_inst", VCPU_STAT(resvd_inst_exits) },
	{ "break_inst", VCPU_STAT(break_inst_exits) },
	{ "flush_dcache", VCPU_STAT(flush_dcache_exits) },
	{ "halt_wakeup", VCPU_STAT(halt_wakeup) },
	{NULL}
};

gfn_t unalias_gfn(struct kvm *kvm, gfn_t gfn)
{
	return gfn;
}

/* XXXKYMA: We are simulatoring a processor that has the WII bit set in Config7, so we
 * are "runnable" if interrupts are pending
 */
int kvm_arch_vcpu_runnable(struct kvm_vcpu *vcpu)
{
	return vcpu->kvm->arch.ops->vcpu_runnable(vcpu);
}

int kvm_arch_vcpu_should_kick(struct kvm_vcpu *vcpu)
{
	return 1;
}

int kvm_arch_hardware_enable(void *garbage)
{
#if IS_ENABLED(CONFIG_KVM_MIPS_VZ)
	return mipsvz_arch_hardware_enable(garbage);
#else
	return 0;
#endif
}

void kvm_arch_hardware_disable(void *garbage)
{
}

int kvm_arch_hardware_setup(void)
{
	return 0;
}

void kvm_arch_hardware_unsetup(void)
{
}

void kvm_arch_check_processor_compat(void *rtn)
{
	int *r = (int *)rtn;
	*r = 0;
	return;
}

int kvm_mips_te_init_vm(struct kvm *kvm, unsigned long type);

int kvm_arch_init_vm(struct kvm *kvm, unsigned long type)
{
#if IS_ENABLED(CONFIG_KVM_MIPS_TE)
	if (type == 0)
		return kvm_mips_te_init_vm(kvm, type);
#endif
#if IS_ENABLED(CONFIG_KVM_MIPS_VZ)
	if (type == 1)
		return mipsvz_init_vm(kvm, type);
#endif

	return -EINVAL;
}

void kvm_arch_sync_events(struct kvm *kvm)
{
}

void kvm_arch_destroy_vm(struct kvm *kvm)
{
	kvm->arch.ops->destroy_vm(kvm);
}

long
kvm_arch_dev_ioctl(struct file *filp, unsigned int ioctl, unsigned long arg)
{
	return -ENOIOCTLCMD;
}

void kvm_arch_free_memslot(struct kvm_memory_slot *free,
			   struct kvm_memory_slot *dont)
{
}

int kvm_arch_create_memslot(struct kvm_memory_slot *slot, unsigned long npages)
{
	return 0;
}

int kvm_arch_prepare_memory_region(struct kvm *kvm,
                                struct kvm_memory_slot *memslot,
                                struct kvm_userspace_memory_region *mem,
                                enum kvm_mr_change change)
{
	return 0;
}

void kvm_arch_commit_memory_region(struct kvm *kvm,
                                struct kvm_userspace_memory_region *mem,
                                const struct kvm_memory_slot *old,
                                enum kvm_mr_change change)
{
	kvm->arch.ops->commit_memory_region(kvm, mem, old, change);
}

void kvm_arch_flush_shadow_all(struct kvm *kvm)
{
}

void kvm_arch_flush_shadow_memslot(struct kvm *kvm,
				   struct kvm_memory_slot *slot)
{
}

void kvm_arch_flush_shadow(struct kvm *kvm)
{
}

struct kvm_vcpu *kvm_arch_vcpu_create(struct kvm *kvm, unsigned int id)
{
	return kvm->arch.ops->vcpu_create(kvm, id);
}

void kvm_arch_vcpu_free(struct kvm_vcpu *vcpu)
{
	vcpu->kvm->arch.ops->vcpu_free(vcpu);
}

void kvm_arch_vcpu_destroy(struct kvm_vcpu *vcpu)
{
	kvm_arch_vcpu_free(vcpu);
}

int
kvm_arch_vcpu_ioctl_set_guest_debug(struct kvm_vcpu *vcpu,
				    struct kvm_guest_debug *dbg)
{
	return -ENOIOCTLCMD;
}

int kvm_arch_vcpu_ioctl_run(struct kvm_vcpu *vcpu, struct kvm_run *run)
{
	return vcpu->kvm->arch.ops->vcpu_run(vcpu, run);
}


int
kvm_arch_vcpu_ioctl_get_mpstate(struct kvm_vcpu *vcpu,
				struct kvm_mp_state *mp_state)
{
	return -ENOIOCTLCMD;
}

int
kvm_arch_vcpu_ioctl_set_mpstate(struct kvm_vcpu *vcpu,
				struct kvm_mp_state *mp_state)
{
	return -ENOIOCTLCMD;
}

#define MIPS_CP0_32(_R, _S)					\
	(KVM_REG_MIPS | KVM_REG_SIZE_U32 | 0x10000 | (8 * (_R) + (_S)))

#define MIPS_CP0_64(_R, _S)					\
	(KVM_REG_MIPS | KVM_REG_SIZE_U64 | 0x10000 | (8 * (_R) + (_S)))

#define KVM_REG_MIPS_CP0_INDEX		MIPS_CP0_32(0, 0)
#define KVM_REG_MIPS_CP0_ENTRYLO0	MIPS_CP0_64(2, 0)
#define KVM_REG_MIPS_CP0_ENTRYLO1	MIPS_CP0_64(3, 0)
#define KVM_REG_MIPS_CP0_CONTEXT	MIPS_CP0_64(4, 0)
#define KVM_REG_MIPS_CP0_USERLOCAL	MIPS_CP0_64(4, 2)
#define KVM_REG_MIPS_CP0_PAGEMASK	MIPS_CP0_32(5, 0)
#define KVM_REG_MIPS_CP0_PAGEGRAIN	MIPS_CP0_32(5, 1)
#define KVM_REG_MIPS_CP0_WIRED		MIPS_CP0_32(6, 0)
#define KVM_REG_MIPS_CP0_HWRENA		MIPS_CP0_32(7, 0)
#define KVM_REG_MIPS_CP0_BADVADDR	MIPS_CP0_64(8, 0)
#define KVM_REG_MIPS_CP0_COUNT		MIPS_CP0_32(9, 0)
#define KVM_REG_MIPS_CP0_ENTRYHI	MIPS_CP0_64(10, 0)
#define KVM_REG_MIPS_CP0_COMPARE	MIPS_CP0_32(11, 0)
#define KVM_REG_MIPS_CP0_STATUS		MIPS_CP0_32(12, 0)
#define KVM_REG_MIPS_CP0_CAUSE		MIPS_CP0_32(13, 0)
#define KVM_REG_MIPS_CP0_EBASE		MIPS_CP0_64(15, 1)
#define KVM_REG_MIPS_CP0_CONFIG		MIPS_CP0_32(16, 0)
#define KVM_REG_MIPS_CP0_CONFIG1	MIPS_CP0_32(16, 1)
#define KVM_REG_MIPS_CP0_CONFIG2	MIPS_CP0_32(16, 2)
#define KVM_REG_MIPS_CP0_CONFIG3	MIPS_CP0_32(16, 3)
#define KVM_REG_MIPS_CP0_CONFIG7	MIPS_CP0_32(16, 7)
#define KVM_REG_MIPS_CP0_XCONTEXT	MIPS_CP0_64(20, 0)
#define KVM_REG_MIPS_CP0_ERROREPC	MIPS_CP0_64(30, 0)

static u64 kvm_mips_get_one_regs[] = {
	KVM_REG_MIPS_R0,
	KVM_REG_MIPS_R1,
	KVM_REG_MIPS_R2,
	KVM_REG_MIPS_R3,
	KVM_REG_MIPS_R4,
	KVM_REG_MIPS_R5,
	KVM_REG_MIPS_R6,
	KVM_REG_MIPS_R7,
	KVM_REG_MIPS_R8,
	KVM_REG_MIPS_R9,
	KVM_REG_MIPS_R10,
	KVM_REG_MIPS_R11,
	KVM_REG_MIPS_R12,
	KVM_REG_MIPS_R13,
	KVM_REG_MIPS_R14,
	KVM_REG_MIPS_R15,
	KVM_REG_MIPS_R16,
	KVM_REG_MIPS_R17,
	KVM_REG_MIPS_R18,
	KVM_REG_MIPS_R19,
	KVM_REG_MIPS_R20,
	KVM_REG_MIPS_R21,
	KVM_REG_MIPS_R22,
	KVM_REG_MIPS_R23,
	KVM_REG_MIPS_R24,
	KVM_REG_MIPS_R25,
	KVM_REG_MIPS_R26,
	KVM_REG_MIPS_R27,
	KVM_REG_MIPS_R28,
	KVM_REG_MIPS_R29,
	KVM_REG_MIPS_R30,
	KVM_REG_MIPS_R31,

	KVM_REG_MIPS_HI,
	KVM_REG_MIPS_LO,
	KVM_REG_MIPS_PC,

	KVM_REG_MIPS_CP0_INDEX,
	KVM_REG_MIPS_CP0_CONTEXT,
	KVM_REG_MIPS_CP0_PAGEMASK,
	KVM_REG_MIPS_CP0_WIRED,
	KVM_REG_MIPS_CP0_BADVADDR,
	KVM_REG_MIPS_CP0_ENTRYHI,
	KVM_REG_MIPS_CP0_STATUS,
	KVM_REG_MIPS_CP0_CAUSE,
	/* EPC set via kvm_regs, et al. */
	KVM_REG_MIPS_CP0_CONFIG,
	KVM_REG_MIPS_CP0_CONFIG1,
	KVM_REG_MIPS_CP0_CONFIG2,
	KVM_REG_MIPS_CP0_CONFIG3,
	KVM_REG_MIPS_CP0_CONFIG7,
	KVM_REG_MIPS_CP0_ERROREPC
};

static int kvm_mips_get_reg(struct kvm_vcpu *vcpu,
			    const struct kvm_one_reg *reg)
{
	s64 v;
	int r;

	switch (reg->id) {
	case KVM_REG_MIPS_R0 ... KVM_REG_MIPS_R31:
		v = (long)vcpu->arch.gprs[reg->id - KVM_REG_MIPS_R0];
		break;
	case KVM_REG_MIPS_HI:
		v = (long)vcpu->arch.hi;
		break;
	case KVM_REG_MIPS_LO:
		v = (long)vcpu->arch.lo;
		break;
	case KVM_REG_MIPS_PC:
		v = (long)vcpu->arch.epc;
		break;
	default:
		r = vcpu->kvm->arch.ops->get_reg(vcpu, reg, &v);
		if (r)
			return r;
	}
	if ((reg->id & KVM_REG_SIZE_MASK) == KVM_REG_SIZE_U64) {
		u64 __user *uaddr64 = (u64 __user *)(long)reg->addr;
		return put_user(v, uaddr64);
	} else if ((reg->id & KVM_REG_SIZE_MASK) == KVM_REG_SIZE_U32) {
		u32 __user *uaddr32 = (u32 __user *)(long)reg->addr;
		u32 v32 = (u32)v;
		return put_user(v32, uaddr32);
	} else {
		return -EINVAL;
	}
}

static int kvm_mips_set_reg(struct kvm_vcpu *vcpu,
			    const struct kvm_one_reg *reg)
{
	u64 v;

	if ((reg->id & KVM_REG_SIZE_MASK) == KVM_REG_SIZE_U64) {
		u64 __user *uaddr64 = (u64 __user *)(long)reg->addr;

		if (get_user(v, uaddr64) != 0)
			return -EFAULT;
	} else if ((reg->id & KVM_REG_SIZE_MASK) == KVM_REG_SIZE_U32) {
		u32 __user *uaddr32 = (u32 __user *)(long)reg->addr;
		s32 v32;

		if (get_user(v32, uaddr32) != 0)
			return -EFAULT;
		v = (s64)v32;
	} else {
		return -EINVAL;
	}

	switch (reg->id) {
	case KVM_REG_MIPS_R0:
		/* Silently ignore requests to set $0 */
		break;
	case KVM_REG_MIPS_R1 ... KVM_REG_MIPS_R31:
		vcpu->arch.gprs[reg->id - KVM_REG_MIPS_R0] = v;
		break;
	case KVM_REG_MIPS_HI:
		vcpu->arch.hi = v;
		break;
	case KVM_REG_MIPS_LO:
		vcpu->arch.lo = v;
		break;
	case KVM_REG_MIPS_PC:
		vcpu->arch.epc = v;
		break;

	default:
		return vcpu->kvm->arch.ops->set_reg(vcpu, reg, v);
	}
	return 0;
}

long kvm_arch_vcpu_ioctl(struct file *filp, unsigned int ioctl,
			 unsigned long arg)
{
	struct kvm_vcpu *vcpu = filp->private_data;
	void __user *argp = (void __user *)arg;
	long r;

	switch (ioctl) {
	case KVM_SET_ONE_REG:
	case KVM_GET_ONE_REG: {
		struct kvm_one_reg reg;
		if (copy_from_user(&reg, argp, sizeof(reg)))
			return -EFAULT;
		if (ioctl == KVM_SET_ONE_REG)
			return kvm_mips_set_reg(vcpu, &reg);
		else
			return kvm_mips_get_reg(vcpu, &reg);
	}
	case KVM_GET_REG_LIST: {
		struct kvm_reg_list __user *user_list = argp;
		u64 __user *reg_dest;
		struct kvm_reg_list reg_list;
		unsigned n;

		if (copy_from_user(&reg_list, user_list, sizeof(reg_list)))
			return -EFAULT;
		n = reg_list.n;
		reg_list.n = ARRAY_SIZE(kvm_mips_get_one_regs);
		if (copy_to_user(user_list, &reg_list, sizeof(reg_list)))
			return -EFAULT;
		if (n < reg_list.n)
			return -E2BIG;
		reg_dest = user_list->reg;
		if (copy_to_user(reg_dest, kvm_mips_get_one_regs,
				 sizeof(kvm_mips_get_one_regs)))
			return -EFAULT;
		return 0;
	}
	default:
		r = vcpu->kvm->arch.ops->vcpu_ioctl(vcpu, ioctl, arg);
	}
	return r;
}

/*
 * Get (and clear) the dirty memory log for a memory slot.
 */
int kvm_vm_ioctl_get_dirty_log(struct kvm *kvm, struct kvm_dirty_log *log)
{
	struct kvm_memory_slot *memslot;
	unsigned long ga, ga_end;
	int is_dirty = 0;
	int r;
	unsigned long n;

	mutex_lock(&kvm->slots_lock);

	r = kvm_get_dirty_log(kvm, log, &is_dirty);
	if (r)
		goto out;

	/* If nothing is dirty, don't bother messing with page tables. */
	if (is_dirty) {
		memslot = &kvm->memslots->memslots[log->slot];

		ga = memslot->base_gfn << PAGE_SHIFT;
		ga_end = ga + (memslot->npages << PAGE_SHIFT);

		printk("%s: dirty, ga: %#lx, ga_end %#lx\n", __func__, ga,
		       ga_end);

		n = kvm_dirty_bitmap_bytes(memslot);
		memset(memslot->dirty_bitmap, 0, n);
	}

	r = 0;
out:
	mutex_unlock(&kvm->slots_lock);
	return r;

}

long kvm_arch_vm_ioctl(struct file *filp, unsigned int ioctl, unsigned long arg)
{
	struct kvm *kvm = filp->private_data;
	long r;

	switch (ioctl) {
	default:
		r = kvm->arch.ops->vm_ioctl(kvm, ioctl, arg);
	}

	return r;
}

void kvm_mips_te_arch_exit(void);

int kvm_arch_init(const void *opaque)
{
	int r = 0;
#if IS_ENABLED(CONFIG_KVM_MIPS_TE)
	r = kvm_mips_te_arch_init(opaque);
	if (r)
		return r;
#endif
#if IS_ENABLED(CONFIG_KVM_MIPS_VZ)
	r = mipsvz_arch_init(opaque);
#endif
	return r;
}

void kvm_arch_exit(void)
{
#if IS_ENABLED(CONFIG_KVM_MIPS_TE)
	kvm_mips_te_arch_exit();
#endif
#if IS_ENABLED(CONFIG_KVM_MIPS_VZ)
	mipsvz_arch_exit();
#endif
}

void kvm_arch_vcpu_load(struct kvm_vcpu *vcpu, int cpu)
{
	vcpu->kvm->arch.ops->vcpu_load(vcpu, cpu);
}
EXPORT_SYMBOL(kvm_arch_vcpu_load);

void kvm_arch_vcpu_put(struct kvm_vcpu *vcpu)
{
	vcpu->kvm->arch.ops->vcpu_put(vcpu);
}
EXPORT_SYMBOL(kvm_arch_vcpu_put);

int
kvm_arch_vcpu_ioctl_get_sregs(struct kvm_vcpu *vcpu, struct kvm_sregs *sregs)
{
	return -ENOIOCTLCMD;
}

int
kvm_arch_vcpu_ioctl_set_sregs(struct kvm_vcpu *vcpu, struct kvm_sregs *sregs)
{
	return -ENOIOCTLCMD;
}

int kvm_arch_vcpu_postcreate(struct kvm_vcpu *vcpu)
{
	return 0;
}

int kvm_arch_vcpu_ioctl_get_fpu(struct kvm_vcpu *vcpu, struct kvm_fpu *fpu)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vcpu->arch.fpr); i++)
		fpu->fpr[i] = vcpu->arch.fpr[i];

	fpu->fir = vcpu->arch.fir;
	fpu->fccr = vcpu->arch.fccr;
	fpu->fexr = vcpu->arch.fexr;
	fpu->fenr = vcpu->arch.fenr;
	fpu->fcsr = vcpu->arch.fcsr;

	return 0;
}

int kvm_arch_vcpu_ioctl_set_fpu(struct kvm_vcpu *vcpu, struct kvm_fpu *fpu)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vcpu->arch.fpr); i++)
		vcpu->arch.fpr[i] = fpu->fpr[i];

	vcpu->arch.fir = fpu->fir;
	vcpu->arch.fccr = fpu->fccr;
	vcpu->arch.fexr = fpu->fexr;
	vcpu->arch.fenr = fpu->fenr;
	vcpu->arch.fcsr = fpu->fcsr;

	return 0;
}

int kvm_arch_vcpu_fault(struct kvm_vcpu *vcpu, struct vm_fault *vmf)
{
	return VM_FAULT_SIGBUS;
}

int kvm_dev_ioctl_check_extension(long ext)
{
	int r;

	switch (ext) {
	case KVM_CAP_ONE_REG:
		r = 1;
		break;
#ifndef CONFIG_KVM_MIPS_VZ
	case KVM_CAP_COALESCED_MMIO:
		r = KVM_COALESCED_MMIO_PAGE_OFFSET;
		break;
#else
	case KVM_CAP_IRQCHIP:
		r = 1;
		break;
	case KVM_CAP_IRQ_ROUTING:
		r = 1;
		break;
	case KVM_CAP_MAX_VCPUS:
		r = KVM_MAX_VCPUS;
		break;
	case KVM_CAP_IOEVENTFD:
		r = 1;
		break;
#endif
	default:
		r = 0;
		break;
	}
	return r;
}

int kvm_cpu_has_pending_timer(struct kvm_vcpu *vcpu)
{
	return vcpu->kvm->arch.ops->cpu_has_pending_timer(vcpu);
}

int kvm_arch_vcpu_ioctl_set_regs(struct kvm_vcpu *vcpu, struct kvm_regs *regs)
{
	int i;

	for (i = 1; i < ARRAY_SIZE(vcpu->arch.gprs); i++)
		vcpu->arch.gprs[i] = regs->gpr[i];
	vcpu->arch.gprs[0] = 0; /* zero is special, and cannot be set. */
	vcpu->arch.hi = regs->hi;
	vcpu->arch.lo = regs->lo;
	vcpu->arch.epc = regs->pc;

	return 0;
}

int kvm_arch_vcpu_ioctl_get_regs(struct kvm_vcpu *vcpu, struct kvm_regs *regs)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vcpu->arch.gprs); i++)
		regs->gpr[i] = vcpu->arch.gprs[i];

	regs->hi = vcpu->arch.hi;
	regs->lo = vcpu->arch.lo;
	regs->pc = vcpu->arch.epc;

	return 0;
}

int kvm_arch_vcpu_init(struct kvm_vcpu *vcpu)
{
	return vcpu->kvm->arch.ops->vcpu_init(vcpu);
}

void kvm_arch_vcpu_uninit(struct kvm_vcpu *vcpu)
{
	return;
}

int
kvm_arch_vcpu_ioctl_translate(struct kvm_vcpu *vcpu, struct kvm_translation *tr)
{
	return 0;
}

/* Initial guest state */
int kvm_arch_vcpu_setup(struct kvm_vcpu *vcpu)
{
	return vcpu->kvm->arch.ops->vcpu_setup(vcpu);
}

#ifdef CONFIG_KVM_MIPS_TE
static
void kvm_mips_set_c0_status(void)
{
	uint32_t status = read_c0_status();

	if (cpu_has_dsp)
		status |= (ST0_MX);

	write_c0_status(status);
	ehb();
}

/*
 * Return value is in the form (errcode<<2 | RESUME_FLAG_HOST | RESUME_FLAG_NV)
 */
int kvm_mips_handle_exit(struct kvm_run *run, struct kvm_vcpu *vcpu)
{
	uint32_t cause = vcpu->arch.host_cp0_cause;
	uint32_t exccode = (cause >> CAUSEB_EXCCODE) & 0x1f;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.pc;
	unsigned long badvaddr = vcpu->arch.host_cp0_badvaddr;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	/* Set a default exit reason */
	run->exit_reason = KVM_EXIT_UNKNOWN;
	run->ready_for_interrupt_injection = 1;

	/* Set the appropriate status bits based on host CPU features, before we hit the scheduler */
	kvm_mips_set_c0_status();

	local_irq_enable();

	kvm_debug("kvm_mips_handle_exit: cause: %#x, PC: %p, kvm_run: %p, kvm_vcpu: %p\n",
			cause, opc, run, vcpu);

	/* Do a privilege check, if in UM most of these exit conditions end up
	 * causing an exception to be delivered to the Guest Kernel
	 */
	er = kvm_mips_check_privilege(cause, opc, run, vcpu);
	if (er == EMULATE_PRIV_FAIL) {
		goto skip_emul;
	} else if (er == EMULATE_FAIL) {
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
		goto skip_emul;
	}

	switch (exccode) {
	case T_INT:
		kvm_debug("[%d]T_INT @ %p\n", vcpu->vcpu_id, opc);

		++vcpu->stat.int_exits;
		trace_kvm_exit(vcpu, INT_EXITS);

		if (need_resched()) {
			cond_resched();
		}

		ret = RESUME_GUEST;
		break;

	case T_COP_UNUSABLE:
		kvm_debug("T_COP_UNUSABLE: @ PC: %p\n", opc);

		++vcpu->stat.cop_unusable_exits;
		trace_kvm_exit(vcpu, COP_UNUSABLE_EXITS);
		ret = kvm_mips_callbacks->handle_cop_unusable(vcpu);
		/* XXXKYMA: Might need to return to user space */
		if (run->exit_reason == KVM_EXIT_IRQ_WINDOW_OPEN) {
			ret = RESUME_HOST;
		}
		break;

	case T_TLB_MOD:
		++vcpu->stat.tlbmod_exits;
		trace_kvm_exit(vcpu, TLBMOD_EXITS);
		ret = kvm_mips_callbacks->handle_tlb_mod(vcpu);
		break;

	case T_TLB_ST_MISS:
		kvm_debug
		    ("TLB ST fault:  cause %#x, status %#lx, PC: %p, BadVaddr: %#lx\n",
		     cause, kvm_read_c0_guest_status(vcpu->arch.cop0), opc,
		     badvaddr);

		++vcpu->stat.tlbmiss_st_exits;
		trace_kvm_exit(vcpu, TLBMISS_ST_EXITS);
		ret = kvm_mips_callbacks->handle_tlb_st_miss(vcpu);
		break;

	case T_TLB_LD_MISS:
		kvm_debug("TLB LD fault: cause %#x, PC: %p, BadVaddr: %#lx\n",
			  cause, opc, badvaddr);

		++vcpu->stat.tlbmiss_ld_exits;
		trace_kvm_exit(vcpu, TLBMISS_LD_EXITS);
		ret = kvm_mips_callbacks->handle_tlb_ld_miss(vcpu);
		break;

	case T_ADDR_ERR_ST:
		++vcpu->stat.addrerr_st_exits;
		trace_kvm_exit(vcpu, ADDRERR_ST_EXITS);
		ret = kvm_mips_callbacks->handle_addr_err_st(vcpu);
		break;

	case T_ADDR_ERR_LD:
		++vcpu->stat.addrerr_ld_exits;
		trace_kvm_exit(vcpu, ADDRERR_LD_EXITS);
		ret = kvm_mips_callbacks->handle_addr_err_ld(vcpu);
		break;

	case T_SYSCALL:
		++vcpu->stat.syscall_exits;
		trace_kvm_exit(vcpu, SYSCALL_EXITS);
		ret = kvm_mips_callbacks->handle_syscall(vcpu);
		break;

	case T_RES_INST:
		++vcpu->stat.resvd_inst_exits;
		trace_kvm_exit(vcpu, RESVD_INST_EXITS);
		ret = kvm_mips_callbacks->handle_res_inst(vcpu);
		break;

	case T_BREAK:
		++vcpu->stat.break_inst_exits;
		trace_kvm_exit(vcpu, BREAK_INST_EXITS);
		ret = kvm_mips_callbacks->handle_break(vcpu);
		break;

	default:
		kvm_err
		    ("Exception Code: %d, not yet handled, @ PC: %p, inst: 0x%08x  BadVaddr: %#lx Status: %#lx\n",
		     exccode, opc, kvm_get_inst(opc, vcpu), badvaddr,
		     kvm_read_c0_guest_status(vcpu->arch.cop0));
		kvm_arch_vcpu_dump_regs(vcpu);
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
		break;

	}

skip_emul:
	local_irq_disable();

	if (er == EMULATE_DONE && !(ret & RESUME_HOST))
		kvm_mips_deliver_interrupts(vcpu, cause);

	if (!(ret & RESUME_HOST)) {
		/* Only check for signals if not already exiting to userspace  */
		if (signal_pending(current)) {
			run->exit_reason = KVM_EXIT_INTR;
			ret = (-EINTR << 2) | RESUME_HOST;
			++vcpu->stat.signal_exits;
			trace_kvm_exit(vcpu, SIGNAL_EXITS);
		}
	}

	return ret;
}
#endif

int __init kvm_mips_init(void)
{
	return kvm_init(NULL, sizeof(struct kvm_vcpu), 0, THIS_MODULE);
}

void __exit kvm_mips_exit(void)
{
	kvm_exit();
}

module_init(kvm_mips_init);
module_exit(kvm_mips_exit);

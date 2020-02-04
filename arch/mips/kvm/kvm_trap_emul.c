/*
* This file is subject to the terms and conditions of the GNU General Public
* License.  See the file "COPYING" in the main directory of this archive
* for more details.
*
* KVM/MIPS: Deliver/Emulate exceptions to the guest kernel
*
* Copyright (C) 2012  MIPS Technologies, Inc.  All rights reserved.
* Authors: Sanjay Lal <sanjayl@kymasys.com>
*/

#include <linux/errno.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/vmalloc.h>

#include <linux/kvm_host.h>

#include <asm/fpu.h>

#include <asm/kvm_mips_te.h>

#include "kvm_mips_opcode.h"
#include "kvm_mips_int.h"
#include "kvm_mips_comm.h"

#define CREATE_TRACE_POINTS
#include "trace.h"

#ifndef VECTORSPACING
#define VECTORSPACING 0x100	/* for EI/VI mode */
#endif

static int kvm_mips_te_reset_vcpu(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_te *te = vcpu->arch.impl;
	int i;

	for_each_possible_cpu(i) {
		te->guest_kernel_asid[i] = 0;
		te->guest_user_asid[i] = 0;
	}
	return 0;
}

static int kvm_mips_te_vcpu_runnable(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_te *te = vcpu->arch.impl;
	return !!(te->pending_exceptions);
}

static void kvm_mips_te_init_tlbs(struct kvm *kvm)
{
	struct kvm_mips_te *kvm_mips_te = kvm->arch.impl;
	unsigned long wired;

	/*
	 * Add a wired entry to the TLB, it is used to map the
	 * commpage to the Guest kernel
	 */
	wired = read_c0_wired();
	write_c0_wired(wired + 1);
	mtc0_tlbw_hazard();
	kvm_mips_te->commpage_tlb = wired;

	kvm_debug("[%d] commpage TLB: %d\n", smp_processor_id(),
		  kvm_mips_te->commpage_tlb);
}

static void kvm_mips_te_init_vm_percpu(void *arg)
{
	struct kvm *kvm = (struct kvm *)arg;

	kvm_mips_te_init_tlbs(kvm);
	kvm_mips_callbacks->vm_init(kvm);
}

static void kvm_mips_te_free_vcpus(struct kvm *kvm)
{
	struct kvm_mips_te *kvm_mips_te = kvm->arch.impl;
	unsigned int i;
	struct kvm_vcpu *vcpu;

	/* Put the pages we reserved for the guest pmap */
	for (i = 0; i < kvm_mips_te->guest_pmap_npages; i++) {
		if (kvm_mips_te->guest_pmap[i] != KVM_INVALID_PAGE)
			kvm_mips_release_pfn_clean(kvm_mips_te->guest_pmap[i]);
	}

	kfree(kvm_mips_te->guest_pmap);

	kvm_for_each_vcpu(i, vcpu, kvm) {
		kvm_arch_vcpu_free(vcpu);
	}

	mutex_lock(&kvm->lock);

	for (i = 0; i < atomic_read(&kvm->online_vcpus); i++)
		kvm->vcpus[i] = NULL;

	atomic_set(&kvm->online_vcpus, 0);

	mutex_unlock(&kvm->lock);
}

static void kvm_mips_te_uninit_tlbs(void *arg)
{
	/* Restore wired count */
	write_c0_wired(0);
	mtc0_tlbw_hazard();
	/* Clear out all the TLBs */
	kvm_local_flush_tlb_all();
}

static void kvm_mips_te_destroy_vm(struct kvm *kvm)
{
	kvm_mips_te_free_vcpus(kvm);

	/* If this is the last instance, restore wired count */
	if (atomic_dec_return(&kvm_mips_instance) == 0) {
		kvm_info("%s: last KVM instance, restoring TLB parameters\n",
			 __func__);
		on_each_cpu(kvm_mips_te_uninit_tlbs, NULL, 1);
	}
	kfree(kvm->arch.impl);
}

static void kvm_mips_te_commit_memory_region(struct kvm *kvm,
					     struct kvm_userspace_memory_region *mem,
					     const struct kvm_memory_slot *old,
					     enum kvm_mr_change change)
{
	struct kvm_mips_te *kvm_mips_te = kvm->arch.impl;
	unsigned long npages = 0;
	int i, err = 0;

	kvm_debug("%s: kvm: %p slot: %d, GPA: %llx, size: %llx, QVA: %llx\n",
		  __func__, kvm, mem->slot, mem->guest_phys_addr,
		  mem->memory_size, mem->userspace_addr);

	/* Setup Guest PMAP table */
	if (!kvm_mips_te->guest_pmap) {
		if (mem->slot == 0)
			npages = mem->memory_size >> PAGE_SHIFT;

		if (npages) {
			kvm_mips_te->guest_pmap_npages = npages;
			kvm_mips_te->guest_pmap =
			    kzalloc(npages * sizeof(unsigned long), GFP_KERNEL);

			if (!kvm_mips_te->guest_pmap) {
				kvm_err("Failed to allocate guest PMAP");
				err = -ENOMEM;
				goto out;
			}

			kvm_info("Allocated space for Guest PMAP Table (%ld pages) @ %p\n",
				 npages, kvm_mips_te->guest_pmap);

			/* Now setup the page table */
			for (i = 0; i < npages; i++)
				kvm_mips_te->guest_pmap[i] = KVM_INVALID_PAGE;
		}
	}
out:
	return;
}

static struct kvm_vcpu *kvm_mips_te_vcpu_create(struct kvm *kvm, unsigned int id)
{
	extern char mips32_exception[], mips32_exceptionEnd[];
	extern char mips32_GuestException[], mips32_GuestExceptionEnd[];
	int err, size, offset;
	void *gebase;
	int i;
	struct kvm_mips_te *kvm_mips_te = kvm->arch.impl;
	struct kvm_mips_vcpu_te *vcpu_te;
	struct kvm_vcpu *vcpu;

	vcpu_te = kzalloc(sizeof(struct kvm_mips_vcpu_te), GFP_KERNEL);
	if (!vcpu_te) {
		err = -ENOMEM;
		goto out;
	}

	vcpu = kzalloc(sizeof(struct kvm_vcpu), GFP_KERNEL);
	if (!vcpu) {
		err = -ENOMEM;
		goto out;
	}
	vcpu->arch.impl = vcpu_te;
	vcpu_te->vcpu = vcpu;
	vcpu_te->kvm_mips_te = kvm_mips_te;

	err = kvm_vcpu_init(vcpu, kvm, id);

	if (err)
		goto out_free_cpu;

	kvm_info("kvm @ %p: create cpu %d at %p\n", kvm, id, vcpu);

	/* Allocate space for host mode exception handlers that handle
	 * guest mode exits
	 */
	if (cpu_has_veic || cpu_has_vint)
		size = 0x200 + VECTORSPACING * 64;
	else
		size = 0x200;

	/* Save Linux EBASE */
	vcpu_te->host_ebase = (void *)(long)(read_c0_ebase() & 0x3ff);

	gebase = kzalloc(ALIGN(size, PAGE_SIZE), GFP_KERNEL);

	if (!gebase) {
		err = -ENOMEM;
		goto out_free_cpu;
	}
	kvm_info("Allocated %d bytes for KVM Exception Handlers @ %p\n",
		 ALIGN(size, PAGE_SIZE), gebase);

	/* Save new ebase */
	vcpu_te->guest_ebase = gebase;

	/* Copy L1 Guest Exception handler to correct offset */

	/* TLB Refill, EXL = 0 */
	memcpy(gebase, mips32_exception,
	       mips32_exceptionEnd - mips32_exception);

	/* General Exception Entry point */
	memcpy(gebase + 0x180, mips32_exception,
	       mips32_exceptionEnd - mips32_exception);

	/* For vectored interrupts poke the exception code @ all offsets 0-7 */
	for (i = 0; i < 8; i++) {
		kvm_debug("L1 Vectored handler @ %p\n",
			  gebase + 0x200 + (i * VECTORSPACING));
		memcpy(gebase + 0x200 + (i * VECTORSPACING), mips32_exception,
		       mips32_exceptionEnd - mips32_exception);
	}

	/* General handler, relocate to unmapped space for sanity's sake */
	offset = 0x2000;
	kvm_info("Installing KVM Exception handlers @ %p, %#x bytes\n",
		 gebase + offset,
		 (unsigned)(mips32_GuestExceptionEnd - mips32_GuestException));

	memcpy(gebase + offset, mips32_GuestException,
	       mips32_GuestExceptionEnd - mips32_GuestException);

	/* Invalidate the icache for these ranges */
	mips32_SyncICache((unsigned long) gebase, ALIGN(size, PAGE_SIZE));

	/*
	 * Allocate comm page for guest kernel, a TLB will be reserved
	 * for mapping GVA @ 0xFFFF8000 to this page
	 */
	vcpu_te->kseg0_commpage = kzalloc(PAGE_SIZE << 1, GFP_KERNEL);

	if (!vcpu_te->kseg0_commpage) {
		err = -ENOMEM;
		goto out_free_gebase;
	}

	kvm_info("Allocated COMM page @ %p\n", vcpu_te->kseg0_commpage);
	kvm_mips_commpage_init(vcpu);

	/* Init */
	vcpu_te->last_sched_cpu = -1;

	/* Start off the timer */
	kvm_mips_emulate_count(vcpu);

	return vcpu;

out_free_gebase:
	kfree(gebase);

out_free_cpu:
	kfree(vcpu);

out:
	kfree(vcpu_te);
	return ERR_PTR(err);
}

static void kvm_mips_te_vcpu_free(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;
	hrtimer_cancel(&vcpu_te->comparecount_timer);

	kvm_vcpu_uninit(vcpu);

	kvm_mips_dump_stats(vcpu);

	kfree(vcpu_te->guest_ebase);
	kfree(vcpu_te->kseg0_commpage);
	kfree(vcpu_te);
}

static int kvm_mips_te_vcpu_run(struct kvm_vcpu *vcpu, struct kvm_run *run)
{
	int r = 0;
	sigset_t sigsaved;
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;

	if (vcpu->sigset_active)
		sigprocmask(SIG_SETMASK, &vcpu->sigset, &sigsaved);

	if (vcpu->mmio_needed) {
		if (!vcpu->mmio_is_write)
			kvm_mips_complete_mmio_load(vcpu, run);
		vcpu->mmio_needed = 0;
	}

	lose_fpu(1);

	local_irq_disable();
	/* Check if we have any exceptions/interrupts pending */
	kvm_mips_deliver_interrupts(vcpu,
				    kvm_read_c0_guest_cause(vcpu_te->cop0));

	kvm_guest_enter();

	r = __kvm_mips_vcpu_run(run, vcpu);

	kvm_guest_exit();
	local_irq_enable();

	if (vcpu->sigset_active)
		sigprocmask(SIG_SETMASK, &sigsaved, NULL);

	return r;
}

static int kvm_mips_te_vcpu_ioctl_interrupt(struct kvm_vcpu *vcpu,
					    struct kvm_mips_interrupt *irq)
{
	struct kvm_mips_vcpu_te *dvcpu_te;
	int intr = (int)irq->irq;
	struct kvm_vcpu *dvcpu = NULL;

	if (intr == 3 || intr == -3 || intr == 4 || intr == -4)
		kvm_debug("%s: CPU: %d, INTR: %d\n", __func__, irq->cpu,
			  (int)intr);

	if (irq->cpu == -1)
		dvcpu = vcpu;
	else
		dvcpu = vcpu->kvm->vcpus[irq->cpu];

	if (intr == 2 || intr == 3 || intr == 4) {
		kvm_mips_callbacks->queue_io_int(dvcpu, irq);

	} else if (intr == -2 || intr == -3 || intr == -4) {
		kvm_mips_callbacks->dequeue_io_int(dvcpu, irq);
	} else {
		kvm_err("%s: invalid interrupt ioctl (%d:%d)\n", __func__,
			irq->cpu, irq->irq);
		return -EINVAL;
	}
	dvcpu_te = dvcpu->arch.impl;
	dvcpu_te->wait = 0;

	if (waitqueue_active(&dvcpu->wq))
		wake_up_interruptible(&dvcpu->wq);

	return 0;
}

static long kvm_mips_te_vcpu_ioctl(struct kvm_vcpu *vcpu,
				   unsigned int ioctl,
				   unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	long r;

	switch (ioctl) {
	case KVM_NMI:
		/* Treat the NMI as a CPU reset */
		r = kvm_mips_te_reset_vcpu(vcpu);
		break;
	case KVM_INTERRUPT:
		{
			struct kvm_mips_interrupt irq;
			r = -EFAULT;
			if (copy_from_user(&irq, argp, sizeof(irq)))
				goto out;

			kvm_debug("[%d] %s: irq: %d\n", vcpu->vcpu_id, __func__,
				  irq.irq);

			r = kvm_mips_te_vcpu_ioctl_interrupt(vcpu, &irq);
			break;
		}
	default:
		r = -ENOIOCTLCMD;
		break;
	}
out:
	return r;
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

static int kvm_mips_te_get_reg(struct kvm_vcpu *vcpu,
			       const struct kvm_one_reg *reg,
			       s64 *v)
{
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;
	struct mips_coproc *cop0 = vcpu_te->cop0;

	switch (reg->id) {
	case KVM_REG_MIPS_CP0_INDEX:
		*v = (long)kvm_read_c0_guest_index(cop0);
		break;
	case KVM_REG_MIPS_CP0_CONTEXT:
		*v = (long)kvm_read_c0_guest_context(cop0);
		break;
	case KVM_REG_MIPS_CP0_PAGEMASK:
		*v = (long)kvm_read_c0_guest_pagemask(cop0);
		break;
	case KVM_REG_MIPS_CP0_WIRED:
		*v = (long)kvm_read_c0_guest_wired(cop0);
		break;
	case KVM_REG_MIPS_CP0_BADVADDR:
		*v = (long)kvm_read_c0_guest_badvaddr(cop0);
		break;
	case KVM_REG_MIPS_CP0_ENTRYHI:
		*v = (long)kvm_read_c0_guest_entryhi(cop0);
		break;
	case KVM_REG_MIPS_CP0_STATUS:
		*v = (long)kvm_read_c0_guest_status(cop0);
		break;
	case KVM_REG_MIPS_CP0_CAUSE:
		*v = (long)kvm_read_c0_guest_cause(cop0);
		break;
	case KVM_REG_MIPS_CP0_ERROREPC:
		*v = (long)kvm_read_c0_guest_errorepc(cop0);
		break;
	case KVM_REG_MIPS_CP0_CONFIG:
		*v = (long)kvm_read_c0_guest_config(cop0);
		break;
	case KVM_REG_MIPS_CP0_CONFIG1:
		*v = (long)kvm_read_c0_guest_config1(cop0);
		break;
	case KVM_REG_MIPS_CP0_CONFIG2:
		*v = (long)kvm_read_c0_guest_config2(cop0);
		break;
	case KVM_REG_MIPS_CP0_CONFIG3:
		*v = (long)kvm_read_c0_guest_config3(cop0);
		break;
	case KVM_REG_MIPS_CP0_CONFIG7:
		*v = (long)kvm_read_c0_guest_config7(cop0);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int kvm_mips_te_set_reg(struct kvm_vcpu *vcpu,
			       const struct kvm_one_reg *reg,
			       u64 v)
{
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;
	struct mips_coproc *cop0 = vcpu_te->cop0;

	switch (reg->id) {
	case KVM_REG_MIPS_CP0_INDEX:
		kvm_write_c0_guest_index(cop0, v);
		break;
	case KVM_REG_MIPS_CP0_CONTEXT:
		kvm_write_c0_guest_context(cop0, v);
		break;
	case KVM_REG_MIPS_CP0_PAGEMASK:
		kvm_write_c0_guest_pagemask(cop0, v);
		break;
	case KVM_REG_MIPS_CP0_WIRED:
		kvm_write_c0_guest_wired(cop0, v);
		break;
	case KVM_REG_MIPS_CP0_BADVADDR:
		kvm_write_c0_guest_badvaddr(cop0, v);
		break;
	case KVM_REG_MIPS_CP0_ENTRYHI:
		kvm_write_c0_guest_entryhi(cop0, v);
		break;
	case KVM_REG_MIPS_CP0_STATUS:
		kvm_write_c0_guest_status(cop0, v);
		break;
	case KVM_REG_MIPS_CP0_CAUSE:
		kvm_write_c0_guest_cause(cop0, v);
		break;
	case KVM_REG_MIPS_CP0_ERROREPC:
		kvm_write_c0_guest_errorepc(cop0, v);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

int kvm_mips_te_arch_init(void *opaque)
{
	int ret;

	if (kvm_mips_callbacks) {
		kvm_err("kvm: module already exists\n");
		return -EEXIST;
	}

	/*
	 * On MIPS, kernel modules are executed from "mapped space",
	 * which requires TLBs.  The TLB handling code is statically
	 * linked with the rest of the kernel (kvm_tlb.c) to avoid the
	 * possibility of double faulting. The issue is that the TLB
	 * code references routines that are part of the the KVM
	 * module, which are only available once the module is loaded.
	 */
	kvm_mips_gfn_to_pfn = gfn_to_pfn;
	kvm_mips_release_pfn_clean = kvm_release_pfn_clean;
	kvm_mips_is_error_pfn = is_error_pfn;

	ret = kvm_mips_emulation_init(&kvm_mips_callbacks);

	pr_info("KVM/MIPS Initialized\n");

	return ret;
}

void kvm_mips_te_arch_exit(void)
{
	kvm_mips_callbacks = NULL;

	kvm_mips_gfn_to_pfn = NULL;
	kvm_mips_release_pfn_clean = NULL;
	kvm_mips_is_error_pfn = NULL;

	pr_info("KVM/MIPS unloaded\n");
}

int kvm_mips_te_vcpu_dump_regs(struct kvm_vcpu *vcpu)
{
	int i;
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;
	struct mips_coproc *cop0 = vcpu_te->cop0;

	if (!vcpu)
		return -1;

	printk("VCPU Register Dump:\n");
	printk("\tepc = 0x%08lx\n", vcpu->arch.epc);;
	printk("\texceptions: %08lx\n", vcpu_te->pending_exceptions);

	for (i = 0; i < 32; i += 4) {
		printk("\tgpr%02d: %08lx %08lx %08lx %08lx\n", i,
		       vcpu->arch.gprs[i],
		       vcpu->arch.gprs[i + 1],
		       vcpu->arch.gprs[i + 2], vcpu->arch.gprs[i + 3]);
	}
	printk("\thi: 0x%08lx\n", vcpu->arch.hi);
	printk("\tlo: 0x%08lx\n", vcpu->arch.lo);

	printk("\tStatus: 0x%08lx, Cause: 0x%08lx\n",
	       kvm_read_c0_guest_status(cop0), kvm_read_c0_guest_cause(cop0));

	printk("\tEPC: 0x%08lx\n", kvm_read_c0_guest_epc(cop0));

	return 0;
}

static void kvm_mips_te_comparecount_func(unsigned long data)
{
	struct kvm_vcpu *vcpu = (struct kvm_vcpu *)data;
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;

	kvm_mips_callbacks->queue_timer_int(vcpu);

	vcpu_te->wait = 0;
	if (waitqueue_active(&vcpu->wq))
		wake_up_interruptible(&vcpu->wq);
}

/*
 * low level hrtimer wake routine.
 */
static enum hrtimer_restart kvm_mips_te_comparecount_wakeup(struct hrtimer *timer)
{
	struct kvm_mips_vcpu_te *vcpu_te;

	vcpu_te = container_of(timer,
			       struct kvm_mips_vcpu_te,
			       comparecount_timer);
	kvm_mips_te_comparecount_func((unsigned long)vcpu_te->vcpu);
	hrtimer_forward_now(&vcpu_te->comparecount_timer,
			    ktime_set(0, MS_TO_NS(10)));
	return HRTIMER_RESTART;
}

static int kvm_mips_te_vcpu_init(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;

	kvm_mips_callbacks->vcpu_init(vcpu);
	hrtimer_init(&vcpu_te->comparecount_timer, CLOCK_MONOTONIC,
		     HRTIMER_MODE_REL);
	vcpu_te->comparecount_timer.function = kvm_mips_te_comparecount_wakeup;
	kvm_mips_init_shadow_tlb(vcpu);
	return 0;
}

static int kvm_mips_te_vcpu_setup(struct kvm_vcpu *vcpu)
{
	return kvm_mips_callbacks->vcpu_setup(vcpu);
}

static void kvm_mips_set_c0_status(void)
{
	uint32_t status = read_c0_status();

	if (cpu_has_fpu)
		status |= (ST0_CU1);

	if (cpu_has_dsp)
		status |= (ST0_MX);

	write_c0_status(status);
	ehb();
}

/*
 * Return value is in the form (errcode<<2 | RESUME_FLAG_HOST | RESUME_FLAG_NV)
 */
int kvm_mips_te_handle_exit(struct kvm_run *run, struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;
	uint32_t cause = vcpu_te->host_cp0_cause;
	uint32_t exccode = (cause >> CAUSEB_EXCCODE) & 0x1f;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.epc;
	unsigned long badvaddr = vcpu_te->host_cp0_badvaddr;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	/* Set a default exit reason */
	run->exit_reason = KVM_EXIT_UNKNOWN;
	run->ready_for_interrupt_injection = 1;

	/*
	 * Set the appropriate status bits based on host CPU features,
	 * before we hit the scheduler
	 */
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

		if (need_resched())
			cond_resched();

		ret = RESUME_GUEST;
		break;

	case T_COP_UNUSABLE:
		kvm_debug("T_COP_UNUSABLE: @ PC: %p\n", opc);

		++vcpu->stat.cop_unusable_exits;
		trace_kvm_exit(vcpu, COP_UNUSABLE_EXITS);
		ret = kvm_mips_callbacks->handle_cop_unusable(vcpu);
		/* XXXKYMA: Might need to return to user space */
		if (run->exit_reason == KVM_EXIT_IRQ_WINDOW_OPEN)
			ret = RESUME_HOST;
		break;

	case T_TLB_MOD:
		++vcpu->stat.tlbmod_exits;
		trace_kvm_exit(vcpu, TLBMOD_EXITS);
		ret = kvm_mips_callbacks->handle_tlb_mod(vcpu);
		break;

	case T_TLB_ST_MISS:
		kvm_debug("TLB ST fault:  cause %#x, status %#lx, PC: %p, BadVaddr: %#lx\n",
		     cause, kvm_read_c0_guest_status(vcpu_te->cop0), opc,
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
		kvm_err("Exception Code: %d, not yet handled, @ PC: %p, inst: 0x%08x  BadVaddr: %#lx Status: %#lx\n",
		     exccode, opc, kvm_get_inst(opc, vcpu), badvaddr,
		     kvm_read_c0_guest_status(vcpu_te->cop0));
		kvm_mips_te_vcpu_dump_regs(vcpu);
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

static gpa_t kvm_trap_emul_gva_to_gpa_cb(gva_t gva)
{
	gpa_t gpa;
	uint32_t kseg = KSEGX(gva);

	if ((kseg == CKSEG0) || (kseg == CKSEG1))
		gpa = CPHYSADDR(gva);
	else {
		printk("%s: cannot find GPA for GVA: %#lx\n", __func__, gva);
		kvm_mips_dump_host_tlbs();
		gpa = KVM_INVALID_ADDR;
	}

#ifdef DEBUG
	kvm_debug("%s: gva %#lx, gpa: %#llx\n", __func__, gva, gpa);
#endif

	return gpa;
}


static int kvm_trap_emul_handle_cop_unusable(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.epc;
	unsigned long cause = vcpu_te->host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	if (((cause & CAUSEF_CE) >> CAUSEB_CE) == 1) {
		er = kvm_mips_emulate_fpu_exc(cause, opc, run, vcpu);
	} else
		er = kvm_mips_emulate_inst(cause, opc, run, vcpu);

	switch (er) {
	case EMULATE_DONE:
		ret = RESUME_GUEST;
		break;

	case EMULATE_FAIL:
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
		break;

	case EMULATE_WAIT:
		run->exit_reason = KVM_EXIT_INTR;
		ret = RESUME_HOST;
		break;

	default:
		BUG();
	}
	return ret;
}

static int kvm_trap_emul_handle_tlb_mod(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.epc;
	unsigned long badvaddr = vcpu_te->host_cp0_badvaddr;
	u32 cause = vcpu_te->host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	if (KVM_GUEST_KSEGX(badvaddr) < KVM_GUEST_KSEG0
	    || KVM_GUEST_KSEGX(badvaddr) == KVM_GUEST_KSEG23) {
#ifdef DEBUG
		kvm_debug("USER/KSEG23 ADDR TLB MOD fault: cause %#x, PC: %p, BadVaddr: %#lx\n",
			  cause, opc, badvaddr);
#endif
		er = kvm_mips_handle_tlbmod(cause, opc, run, vcpu);

		if (er == EMULATE_DONE)
			ret = RESUME_GUEST;
		else {
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		}
	} else if (KVM_GUEST_KSEGX(badvaddr) == KVM_GUEST_KSEG0) {
		/* XXXKYMA: The guest kernel does not expect to get this fault when we are not
		 * using HIGHMEM. Need to address this in a HIGHMEM kernel
		 */
		printk("TLB MOD fault not handled, cause %#x, PC: %p, BadVaddr: %#lx\n",
		       cause, opc, badvaddr);
		kvm_mips_dump_host_tlbs();
		kvm_mips_te_vcpu_dump_regs(vcpu);
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
	} else {
		printk("Illegal TLB Mod fault address , cause %#x, PC: %p, BadVaddr: %#lx\n",
		       cause, opc, badvaddr);
		kvm_mips_dump_host_tlbs();
		kvm_mips_te_vcpu_dump_regs(vcpu);
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
	}
	return ret;
}

static int kvm_trap_emul_handle_tlb_st_miss(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.epc;
	unsigned long badvaddr = vcpu_te->host_cp0_badvaddr;
	u32 cause = vcpu_te->host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	if (((badvaddr & PAGE_MASK) == KVM_GUEST_COMMPAGE_ADDR)
	    && KVM_GUEST_KERNEL_MODE(vcpu)) {
		if (kvm_mips_handle_commpage_tlb_fault(badvaddr, vcpu) < 0) {
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		}
	} else if (KVM_GUEST_KSEGX(badvaddr) < KVM_GUEST_KSEG0
		   || KVM_GUEST_KSEGX(badvaddr) == KVM_GUEST_KSEG23) {
#ifdef DEBUG
		kvm_debug("USER ADDR TLB LD fault: cause %#x, PC: %p, BadVaddr: %#lx\n",
		     cause, opc, badvaddr);
#endif
		er = kvm_mips_handle_tlbmiss(cause, opc, run, vcpu);
		if (er == EMULATE_DONE)
			ret = RESUME_GUEST;
		else {
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		}
	} else if (KVM_GUEST_KSEGX(badvaddr) == KVM_GUEST_KSEG0) {
		/* All KSEG0 faults are handled by KVM, as the guest kernel does not
		 * expect to ever get them
		 */
		if (kvm_mips_handle_kseg0_tlb_fault(vcpu_te->host_cp0_badvaddr, vcpu) < 0) {
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		}
	} else {
		kvm_err("Illegal TLB LD fault address , cause %#x, PC: %p, BadVaddr: %#lx\n",
			cause, opc, badvaddr);
		kvm_mips_dump_host_tlbs();
		kvm_mips_te_vcpu_dump_regs(vcpu);
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
	}
	return ret;
}

static int kvm_trap_emul_handle_tlb_ld_miss(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.epc;
	unsigned long badvaddr = vcpu_te->host_cp0_badvaddr;
	u32 cause = vcpu_te->host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	if (((badvaddr & PAGE_MASK) == KVM_GUEST_COMMPAGE_ADDR)
	    && KVM_GUEST_KERNEL_MODE(vcpu)) {
		if (kvm_mips_handle_commpage_tlb_fault(badvaddr, vcpu) < 0) {
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		}
	} else if (KVM_GUEST_KSEGX(badvaddr) < KVM_GUEST_KSEG0
		   || KVM_GUEST_KSEGX(badvaddr) == KVM_GUEST_KSEG23) {
#ifdef DEBUG
		kvm_debug("USER ADDR TLB ST fault: PC: %#lx, BadVaddr: %#lx\n",
			  vcpu->arch.epc, badvaddr);
#endif

		/* User Address (UA) fault, this could happen if
		 * (1) TLB entry not present/valid in both Guest and shadow host TLBs, in this
		 *     case we pass on the fault to the guest kernel and let it handle it.
		 * (2) TLB entry is present in the Guest TLB but not in the shadow, in this
		 *     case we inject the TLB from the Guest TLB into the shadow host TLB
		 */

		er = kvm_mips_handle_tlbmiss(cause, opc, run, vcpu);
		if (er == EMULATE_DONE)
			ret = RESUME_GUEST;
		else {
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		}
	} else if (KVM_GUEST_KSEGX(badvaddr) == KVM_GUEST_KSEG0) {
		if (kvm_mips_handle_kseg0_tlb_fault(vcpu_te->host_cp0_badvaddr, vcpu) < 0) {
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		}
	} else {
		printk("Illegal TLB ST fault address , cause %#x, PC: %p, BadVaddr: %#lx\n",
		       cause, opc, badvaddr);
		kvm_mips_dump_host_tlbs();
		kvm_mips_te_vcpu_dump_regs(vcpu);
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
	}
	return ret;
}

static int kvm_trap_emul_handle_addr_err_st(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.epc;
	unsigned long badvaddr = vcpu_te->host_cp0_badvaddr;
	u32 cause = vcpu_te->host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	if (KVM_GUEST_KERNEL_MODE(vcpu)
	    && (KSEGX(badvaddr) == CKSEG0 || KSEGX(badvaddr) == CKSEG1)) {
#ifdef DEBUG
		kvm_debug("Emulate Store to MMIO space\n");
#endif
		er = kvm_mips_emulate_inst(cause, opc, run, vcpu);
		if (er == EMULATE_FAIL) {
			printk("Emulate Store to MMIO space failed\n");
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		} else {
			run->exit_reason = KVM_EXIT_MMIO;
			ret = RESUME_HOST;
		}
	} else {
		printk("Address Error (STORE): cause %#x, PC: %p, BadVaddr: %#lx\n",
		       cause, opc, badvaddr);
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
	}
	return ret;
}

static int kvm_trap_emul_handle_addr_err_ld(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.epc;
	unsigned long badvaddr = vcpu_te->host_cp0_badvaddr;
	u32 cause = vcpu_te->host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	if (KSEGX(badvaddr) == CKSEG0 || KSEGX(badvaddr) == CKSEG1) {
#ifdef DEBUG
		kvm_debug("Emulate Load from MMIO space @ %#lx\n", badvaddr);
#endif
		er = kvm_mips_emulate_inst(cause, opc, run, vcpu);
		if (er == EMULATE_FAIL) {
			printk("Emulate Load from MMIO space failed\n");
			run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
			ret = RESUME_HOST;
		} else {
			run->exit_reason = KVM_EXIT_MMIO;
			ret = RESUME_HOST;
		}
	} else {
		printk("Address Error (LOAD): cause %#x, PC: %p, BadVaddr: %#lx\n",
		       cause, opc, badvaddr);
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
		er = EMULATE_FAIL;
	}
	return ret;
}

static int kvm_trap_emul_handle_syscall(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.epc;
	u32 cause = vcpu_te->host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	er = kvm_mips_emulate_syscall(cause, opc, run, vcpu);
	if (er == EMULATE_DONE)
		ret = RESUME_GUEST;
	else {
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
	}
	return ret;
}

static int kvm_trap_emul_handle_res_inst(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.epc;
	u32 cause = vcpu_te->host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	er = kvm_mips_handle_ri(cause, opc, run, vcpu);
	if (er == EMULATE_DONE)
		ret = RESUME_GUEST;
	else {
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
	}
	return ret;
}

static int kvm_trap_emul_handle_break(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;
	struct kvm_run *run = vcpu->run;
	uint32_t __user *opc = (uint32_t __user *) vcpu->arch.epc;
	u32 cause = vcpu_te->host_cp0_cause;
	enum emulation_result er = EMULATE_DONE;
	int ret = RESUME_GUEST;

	er = kvm_mips_emulate_bp_exc(cause, opc, run, vcpu);
	if (er == EMULATE_DONE)
		ret = RESUME_GUEST;
	else {
		run->exit_reason = KVM_EXIT_INTERNAL_ERROR;
		ret = RESUME_HOST;
	}
	return ret;
}

static int kvm_trap_emul_vm_init(struct kvm *kvm)
{
	return 0;
}

static int kvm_trap_emul_vcpu_init(struct kvm_vcpu *vcpu)
{
	return 0;
}

static int kvm_trap_emul_vcpu_setup(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;
	struct mips_coproc *cop0 = vcpu_te->cop0;
	uint32_t config1;
	int vcpu_id = vcpu->vcpu_id;

	/* Arch specific stuff, set up config registers properly so that the
	 * guest will come up as expected, for now we simulate a
	 * MIPS 24kc
	 */
	kvm_write_c0_guest_prid(cop0, 0x00019300);
	kvm_write_c0_guest_config(cop0,
				  MIPS_CONFIG0 | (0x1 << CP0C0_AR) |
				  (MMU_TYPE_R4000 << CP0C0_MT));

	/* Read the cache characteristics from the host Config1 Register */
	config1 = (read_c0_config1() & ~0x7f);

	/* Set up MMU size */
	config1 &= ~(0x3f << 25);
	config1 |= ((KVM_MIPS_GUEST_TLB_SIZE - 1) << 25);

	/* We unset some bits that we aren't emulating */
	config1 &=
	    ~((1 << CP0C1_C2) | (1 << CP0C1_MD) | (1 << CP0C1_PC) |
	      (1 << CP0C1_WR) | (1 << CP0C1_CA));
	kvm_write_c0_guest_config1(cop0, config1);

	kvm_write_c0_guest_config2(cop0, MIPS_CONFIG2);
	/* MIPS_CONFIG2 | (read_c0_config2() & 0xfff) */
	kvm_write_c0_guest_config3(cop0,
				   MIPS_CONFIG3 | (0 << CP0C3_VInt) | (1 <<
								       CP0C3_ULRI));

	/* Set Wait IE/IXMT Ignore in Config7, IAR, AR */
	kvm_write_c0_guest_config7(cop0, (MIPS_CONF7_WII) | (1 << 10));

	/* Setup IntCtl defaults, compatibilty mode for timer interrupts (HW5) */
	kvm_write_c0_guest_intctl(cop0, 0xFC000000);

	/* Put in vcpu id as CPUNum into Ebase Reg to handle SMP Guests */
	kvm_write_c0_guest_ebase(cop0, KVM_GUEST_KSEG0 | (vcpu_id & 0xFF));

	return 0;
}

static struct kvm_mips_callbacks kvm_trap_emul_callbacks = {
	/* exit handlers */
	.handle_cop_unusable = kvm_trap_emul_handle_cop_unusable,
	.handle_tlb_mod = kvm_trap_emul_handle_tlb_mod,
	.handle_tlb_st_miss = kvm_trap_emul_handle_tlb_st_miss,
	.handle_tlb_ld_miss = kvm_trap_emul_handle_tlb_ld_miss,
	.handle_addr_err_st = kvm_trap_emul_handle_addr_err_st,
	.handle_addr_err_ld = kvm_trap_emul_handle_addr_err_ld,
	.handle_syscall = kvm_trap_emul_handle_syscall,
	.handle_res_inst = kvm_trap_emul_handle_res_inst,
	.handle_break = kvm_trap_emul_handle_break,

	.vm_init = kvm_trap_emul_vm_init,
	.vcpu_init = kvm_trap_emul_vcpu_init,
	.vcpu_setup = kvm_trap_emul_vcpu_setup,
	.gva_to_gpa = kvm_trap_emul_gva_to_gpa_cb,
	.queue_timer_int = kvm_mips_queue_timer_int_cb,
	.dequeue_timer_int = kvm_mips_dequeue_timer_int_cb,
	.queue_io_int = kvm_mips_queue_io_int_cb,
	.dequeue_io_int = kvm_mips_dequeue_io_int_cb,
	.irq_deliver = kvm_mips_irq_deliver_cb,
	.irq_clear = kvm_mips_irq_clear_cb,
};

int kvm_mips_emulation_init(struct kvm_mips_callbacks **install_callbacks)
{
	*install_callbacks = &kvm_trap_emul_callbacks;
	return 0;
}

static long kvm_mips_te_vm_ioctl(struct kvm *kvm, unsigned int ioctl,
				 unsigned long arg)
{
	return -ENOIOCTLCMD;
}

static const struct kvm_mips_ops kvm_mips_te_ops = {
	.vcpu_runnable = kvm_mips_te_vcpu_runnable,
	.destroy_vm = kvm_mips_te_destroy_vm,
	.commit_memory_region = kvm_mips_te_commit_memory_region,
	.vcpu_create = kvm_mips_te_vcpu_create,
	.vcpu_free = kvm_mips_te_vcpu_free,
	.vcpu_run = kvm_mips_te_vcpu_run,
	.vm_ioctl = kvm_mips_te_vm_ioctl,
	.vcpu_ioctl = kvm_mips_te_vcpu_ioctl,
	.get_reg = kvm_mips_te_get_reg,
	.set_reg = kvm_mips_te_set_reg,
	.cpu_has_pending_timer = kvm_mips_pending_timer,
	.vcpu_init = kvm_mips_te_vcpu_init,
	.vcpu_setup = kvm_mips_te_vcpu_setup,
	.vcpu_load = kvm_mips_te_vcpu_load,
	.vcpu_put = kvm_mips_te_vcpu_put,
};

int kvm_mips_te_init_vm(struct kvm *kvm, unsigned long type)
{
	kvm->arch.ops = &kvm_mips_te_ops;
	kvm->arch.impl =  kzalloc(sizeof(struct kvm_mips_te), GFP_KERNEL);
	if (!kvm->arch.impl)
		return -ENOMEM;

	if (atomic_inc_return(&kvm_mips_instance) == 1) {
		kvm_info("%s: 1st KVM instance, setup host TLB parameters\n",
			 __func__);
		on_each_cpu(kvm_mips_te_init_vm_percpu, kvm, 1);
	}
	return 0;
}


EXPORT_TRACEPOINT_SYMBOL(kvm_exit);

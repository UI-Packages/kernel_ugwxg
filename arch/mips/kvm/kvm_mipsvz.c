/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2012-2014 Cavium, Inc.
 */

#include <linux/kvm_host.h>
#include <linux/bitmap.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/kvm.h>

#include <asm/mmu_context.h>
#include <asm/kvm_mips_vz.h>
#include <asm/mipsregs.h>
#include <asm/pgalloc.h>
#include <asm/branch.h>
#include <asm/setup.h>
#include <asm/inst.h>
#include <asm/time.h>
#include <asm/fpu.h>

void mipsvz_start_guest(struct kvm_vcpu *vcpu);
void mipsvz_exit_guest(void) __noreturn;

void mipsvz_install_fpu(struct kvm_vcpu *vcpu);
void mipsvz_readout_fpu(struct kvm_vcpu *vcpu);

unsigned long mips_kvm_rootsp[NR_CPUS];
static u32 mipsvz_cp0_count_offset[NR_CPUS];

static unsigned long mipsvz_entryhi_mask;

struct vcpu_mips {
	void *foo;
};

struct mipsvz_kvm_tlb_entry {
	u64 entryhi;
	u64 entrylo0;
	u64 entrylo1;
	u32 pagemask;
};


static void mipsvz_check_asid(struct kvm_vcpu *vcpu)
{
	int cpu = raw_smp_processor_id();
	struct kvm *kvm = vcpu->kvm;
	struct kvm_mips_vz *kvm_mips_vz = kvm->arch.impl;
	struct kvm_mips_vcpu_vz *vcpu_vz = vcpu->arch.impl;
	bool get_new_cntxt = 0;

	/*
	 * Make sure the Root guest and cpu_context are in the same
	 * ASID generation -- we want to (unconditionally) switch back
	 * to cpu_context when leaving the guest.
	 */
	do {
		get_new_cntxt = false;
		if ((kvm_mips_vz->asid[cpu] == 0) ||
			((kvm_mips_vz->asid[cpu] ^ asid_cache(cpu)) &
				ASID_VERSION_MASK)) {

			kvm_mips_vz->asid[cpu] = get_new_asid(cpu);

			if ((cpu_context(cpu, current->mm) ^ asid_cache(cpu)) &
				ASID_VERSION_MASK) {
				drop_mmu_context(current->mm, cpu);
				get_new_cntxt = true;
			}
		}
	} while (get_new_cntxt);

	vcpu_vz->mm_asid = read_c0_entryhi() & ASID_MASK;
	vcpu_vz->guest_asid = kvm_mips_vz->asid[cpu] & ASID_MASK;
}

static bool mipsvz_count_expired(u32 old_count, u32 new_count, u32 compare)
{
	if (new_count > old_count)
		return compare >= old_count && compare <= new_count;
	else
		return compare >= old_count || compare <= new_count;
}

static void mipsvz_install_guest_cp0(struct kvm_mips_vcpu_vz *vcpu_vz)
{
	u32 gconfig4 = read_gc0_config4();
	u32 count;

	write_gc0_index(vcpu_vz->c0_index);
	write_gc0_entrylo0(vcpu_vz->c0_entrylo0);
	write_gc0_entrylo1(vcpu_vz->c0_entrylo1);
	write_gc0_context(vcpu_vz->c0_context);
	write_gc0_userlocal(vcpu_vz->c0_userlocal);
	write_gc0_pagemask(vcpu_vz->c0_pagemask);
	write_gc0_pagegrain(vcpu_vz->c0_pagegrain);
	write_gc0_wired(vcpu_vz->c0_wired);
	write_gc0_hwrena(vcpu_vz->c0_hwrena);
	write_gc0_badvaddr(vcpu_vz->c0_badvaddr);
	write_gc0_entryhi(vcpu_vz->c0_entryhi);
	write_gc0_compare(vcpu_vz->c0_compare);
	write_gc0_cause(vcpu_vz->c0_cause);
	write_gc0_status(vcpu_vz->c0_status);
	write_gc0_epc(vcpu_vz->c0_epc);
	write_gc0_errorepc(vcpu_vz->c0_errorepc);
	write_gc0_ebase(vcpu_vz->c0_ebase);
	write_gc0_xcontext(vcpu_vz->c0_xcontext);

	count = read_gc0_count();

	if (mipsvz_count_expired(vcpu_vz->c0_count, count, vcpu_vz->c0_compare) &&
	    (vcpu_vz->c0_cause & CAUSEF_TI) == 0) {
		vcpu_vz->c0_cause |= CAUSEF_TI;
		write_gc0_cause(vcpu_vz->c0_cause);
	}
	vcpu_vz->have_counter_state = false;

#define MIPSVZ_GUEST_INSTALL_SCRATCH(_i)				\
	if (gconfig4 & (1 << (18 + (_i))))				\
		write_gc0_kscratch(2 + (_i), vcpu_vz->c0_kscratch[_i])

	MIPSVZ_GUEST_INSTALL_SCRATCH(0);
	MIPSVZ_GUEST_INSTALL_SCRATCH(1);
	MIPSVZ_GUEST_INSTALL_SCRATCH(2);
	MIPSVZ_GUEST_INSTALL_SCRATCH(3);
	MIPSVZ_GUEST_INSTALL_SCRATCH(4);
	MIPSVZ_GUEST_INSTALL_SCRATCH(5);
}

static void mipsvz_readout_cp0_counter_state(struct kvm_mips_vcpu_vz *vcpu_vz)
{
	/* Must read count before cause so we can emulate TI getting set. */
	vcpu_vz->compare_timer_read = ktime_get();
	vcpu_vz->c0_count = read_gc0_count();
	vcpu_vz->c0_cause = read_gc0_cause();
	vcpu_vz->c0_compare = read_gc0_compare();
	vcpu_vz->have_counter_state = true;
}

static void mipsvz_readout_guest_cp0(struct kvm_mips_vcpu_vz *vcpu_vz)
{
	u32 gconfig4 = read_gc0_config4();

	vcpu_vz->c0_index = read_gc0_index();
	vcpu_vz->c0_entrylo0 = read_gc0_entrylo0();
	vcpu_vz->c0_entrylo1 = read_gc0_entrylo1();
	vcpu_vz->c0_context = read_gc0_context();
	vcpu_vz->c0_userlocal = read_gc0_userlocal();
	vcpu_vz->c0_pagemask = read_gc0_pagemask();
	vcpu_vz->c0_pagegrain = read_gc0_pagegrain();
	vcpu_vz->c0_wired = read_gc0_wired();
	vcpu_vz->c0_hwrena = read_gc0_hwrena();
	vcpu_vz->c0_badvaddr = read_gc0_badvaddr();
	vcpu_vz->c0_entryhi = read_gc0_entryhi();
	vcpu_vz->c0_compare = read_gc0_compare();
	vcpu_vz->c0_status = read_gc0_status();

	/* Must read count before cause so we can emulate TI getting set. */
	vcpu_vz->c0_count = read_gc0_count();

	vcpu_vz->c0_cause = read_gc0_cause();
	vcpu_vz->c0_epc = read_gc0_epc();
	vcpu_vz->c0_errorepc = read_gc0_errorepc();
	vcpu_vz->c0_ebase = read_gc0_ebase();
	vcpu_vz->c0_xcontext = read_gc0_xcontext();
	if (!vcpu_vz->have_counter_state)
		mipsvz_readout_cp0_counter_state(vcpu_vz);


#define MIPSVZ_GUEST_READOUT_SCRATCH(_i)				\
	if (gconfig4 & (1 << (18 + (_i))))				\
		vcpu_vz->c0_kscratch[_i] = read_gc0_kscratch(2 + (_i))

	MIPSVZ_GUEST_READOUT_SCRATCH(0);
	MIPSVZ_GUEST_READOUT_SCRATCH(1);
	MIPSVZ_GUEST_READOUT_SCRATCH(2);
	MIPSVZ_GUEST_READOUT_SCRATCH(3);
	MIPSVZ_GUEST_READOUT_SCRATCH(4);
	MIPSVZ_GUEST_READOUT_SCRATCH(5);
}

static void mipsvz_exit_vm(struct kvm_vcpu *vcpu,
			   struct kvm_mips_vz_regs *regs,
			   u32 exit_reason)
{
	int i;
	struct kvm_run *kvm_run = vcpu->run;

	for (i = 1; i < ARRAY_SIZE(vcpu->arch.gprs); i++)
		vcpu->arch.gprs[i] = regs->pt.regs[i];
	vcpu->arch.gprs[0] = 0; /* zero is special, and cannot be set. */
	vcpu->arch.hi = regs->pt.hi;
	vcpu->arch.lo = regs->pt.lo;
	vcpu->arch.epc = regs->pt.cp0_epc;

	kvm_run->exit_reason = exit_reason;

	local_irq_disable();

	/* Note that PGD and ASID were already switched in
	   mipsvz_common_chain before handler (which triggered
	   mipsvz_exit_vm) was invoked */

	mipsvz_exit_guest();
	/* Never returns here */
}

static unsigned int  mipsvz_get_fcr31(void)
{
	kvm_err("Help!  missing mipsvz_get_fcr31\n");
	return 0;
}

static unsigned long mipsvz_compute_return_epc(struct kvm_mips_vz_regs *regs)
{
	if (delay_slot(&regs->pt)) {
		union mips_instruction insn;
		insn.word = regs->cp0_badinstrp;
		return __compute_return_epc_for_insn0(&regs->pt, insn, mipsvz_get_fcr31);
	} else {
		regs->pt.cp0_epc += 4;
		return 0;
	}
}

struct mipsvz_szreg {
	u8 size;
	s8 reg; /* negative value indicates error */
	bool sign_extend;
};

static struct mipsvz_szreg mipsvz_get_load_params(u32 insn)
{
	struct mipsvz_szreg r;
	r.size = 0;
	r.reg = -1;
	r.sign_extend = false;

	if ((insn & 0x80000000) == 0)
		goto out;

	switch ((insn >> 26) & 0x1f) {
	case 0x00: /* LB */
		r.size = 1;
		r.sign_extend = true;
		break;
	case 0x04: /* LBU */
		r.size = 1;
		break;
	case 0x01: /* LH */
		r.size = 2;
		r.sign_extend = true;
		break;
	case 0x05: /* LHU */
		r.size = 2;
		break;
	case 0x03: /* LW */
		r.size = 4;
		r.sign_extend = true;
		break;
	case 0x07: /* LWU */
		r.size = 4;
		break;
	case 0x17: /* LD */
		r.size = 8;
		break;
	default:
		goto out;
	}
	r.reg = (insn >> 16) & 0x1f;

out:
	return r;
}

static struct mipsvz_szreg mipsvz_get_store_params(u32 insn)
{
	struct mipsvz_szreg r;
	r.size = 0;
	r.reg = -1;
	r.sign_extend = false;

	if ((insn & 0x80000000) == 0)
		goto out;

	switch ((insn >> 26) & 0x1f) {
	case 0x08: /* SB */
		r.size = 1;
		break;
	case 0x09: /* SH */
		r.size = 2;
		break;
	case 0x0b: /* SW */
		r.size = 4;
		break;
	case 0x1f: /* SD */
		r.size = 8;
		break;
	default:
		goto out;
	}
	r.reg = (insn >> 16) & 0x1f;

out:
	return r;
}

static int mipsvz_handle_io_in(struct kvm_vcpu *vcpu, int is_mmio)
{
	unsigned long val = 0;
	struct kvm_mips_vcpu_vz *vcpu_vz = vcpu->arch.impl;
	void *dest;
	struct mipsvz_szreg r = mipsvz_get_load_params(vcpu_vz->last_exit_insn);

	if (is_mmio)
		dest = vcpu->run->mmio.data;
	else
		dest = sizeof(struct kvm_run) + (char *)((void *)vcpu->run);

	if (r.reg < 0)
		return -EINVAL;
	if (r.sign_extend)
		switch (r.size) {
		case 1:
			val = *(s8 *)dest;
			break;
		case 2:
			val = *(s16 *)dest;
			break;
		case 4:
			val = *(s32 *)dest;
			break;
		case 8:
			val = *(u64 *)dest;
			break;
		}
	else
		switch (r.size) {
		case 1:
			val = *(u8 *)dest;
			break;
		case 2:
			val = *(u16 *)dest;
			break;
		case 4:
			val = *(u32 *)dest;
			break;
		case 8:
			val = *(u64 *)dest;
			break;
		}

	vcpu->arch.gprs[r.reg] = val;
	kvm_debug("   ... %016lx  size %d\n", val, r.size);
	return 0;
}


unsigned long mipsvz_ebase_page;

extern char mipsvz_interrupt_chain;
extern char mipsvz_interrupt_chain_end;
extern char mipsvz_general_chain;
extern char mipsvz_general_chain_end;

void build_r4000_tlb_refill_handler(void *loc, bool kvm_root);

int mipsvz_arch_init(const void *opaque)
{
	unsigned long saved_entryhi;
	unsigned long flags;
	char *mipsvz_ebase_addr;

	mipsvz_ebase_page = get_zeroed_page(GFP_KERNEL);
	if (!mipsvz_ebase_page)
		return -ENOMEM;
	mipsvz_ebase_addr = (char *)mipsvz_ebase_page;

	build_r4000_tlb_refill_handler(mipsvz_ebase_addr, true);

	memcpy(mipsvz_ebase_addr + 0x180, &mipsvz_general_chain,
	       &mipsvz_general_chain_end - &mipsvz_general_chain);

	memcpy(mipsvz_ebase_addr + 0x200, &mipsvz_interrupt_chain,
	       &mipsvz_interrupt_chain_end - &mipsvz_interrupt_chain);
	flush_icache_range(mipsvz_ebase_page, mipsvz_ebase_page + PAGE_SIZE);

	local_irq_save(flags);
	saved_entryhi = read_c0_entryhi();

	write_c0_entryhi(~0x1ffful);
	mipsvz_entryhi_mask = read_c0_entryhi();

	write_c0_entryhi(saved_entryhi);
	local_irq_restore(flags);

	return 0;
}

void mipsvz_arch_exit(void)
{
	if (mipsvz_ebase_page)
		free_page(mipsvz_ebase_page);
	mipsvz_ebase_page = 0;
}

int mipsvz_arch_hardware_enable(void *garbage)
{
	unsigned long flags;
	int cpu = raw_smp_processor_id();
	u32 count;
	u64 cmv_count;

	local_irq_save(flags);
	count = read_c0_count();
	cmv_count = read_c0_cvmcount();
	local_irq_restore(flags);

	mipsvz_cp0_count_offset[cpu] = 0; /*((u32)cmv_count) - count;*/

	return 0;
}

#ifndef __PAGETABLE_PMD_FOLDED
static void mipsvz_release_pud(pud_t pud)
{
	pmd_t *pmd = (pmd_t *)pud_val(pud);
	int i;
	for (i = 0; i < PTRS_PER_PMD; i++) {
		if (pmd_present(pmd[i])) {
			pte_t *pte = (pte_t *)pmd_val(pmd[i]);
			int j;
			for (j = 0; j < PTRS_PER_PTE; j++) {
				if (pte_present(pte[j]))
					kvm_release_pfn_clean(pte_pfn(pte[j]));
			}
			pte_free_kernel(NULL, (pte_t *)pmd_val(pmd[i]));
		}
	}
	pmd_free(NULL, pmd);
}
#endif

static void mipsvz_destroy_vm(struct kvm *kvm)
{
	struct kvm_mips_vz *kvm_mips_vz = kvm->arch.impl;
	struct kvm_vcpu *vcpu;
	pgd_t *pgd;
	pud_t *pud;
	int i;

	pgd = kvm_mips_vz->pgd;
	pud = pud_offset(pgd, 0);
#ifndef __PAGETABLE_PMD_FOLDED
	for (i = 0; i < PTRS_PER_PGD; i++) {
		if (pud_present(pud[i]))
			mipsvz_release_pud((pud[i]));
	}
#else
	{
		pmd_t *pmd = pmd_offset(pud, 0);
		for (i = 0; i < PTRS_PER_PGD; i++) {
			if (pmd_present(pmd[i])) {
				pte_t *pte = (pte_t *)pmd_val(pmd[i]);
				int j;
				for (j = 0; j < PTRS_PER_PTE; j++) {
					if (pte_present(pte[j]))
						kvm_release_pfn_clean(pte_pfn(pte[j]));
				}
				pte_free_kernel(NULL, (pte_t *)pmd_val(pmd[i]));
			}
		}
	}
#endif

	free_pages((unsigned long)kvm_mips_vz->pgd, PGD_ORDER);

	kvm_for_each_vcpu(i, vcpu, kvm) {
		kvm_arch_vcpu_free(vcpu);
	}

	if (kvm_mips_vz->irq_chip) {
		__free_page(kvm_mips_vz->irq_chip->page);
		kfree(kvm_mips_vz->irq_chip);
	}

	kfree(kvm_mips_vz);
}

/* Must be called with guest_mm_lock held. */
static pte_t *mipsvz_pte_for_gpa(struct kvm *kvm, unsigned long addr)
{
	struct kvm_mips_vz *kvm_mips_vz = kvm->arch.impl;
	pgd_t *pgd;
	pud_t *pud;
	pmd_t *pmd;

	pgd = kvm_mips_vz->pgd + pgd_index(addr);
	if (pgd_none(*pgd)) {
		set_pgd(pgd, __pgd(0));
		BUG();  /* Not used on MIPS. */
	}
	pud = pud_offset(pgd, addr);
	if (pud_none(*pud)) {
		pmd_t *new_pmd = pmd_alloc_one(NULL, addr);
		WARN(!new_pmd, "We're hosed, no memory");
		pud_populate(NULL, pud, new_pmd);
	}
	pmd = pmd_offset(pud, addr);
	if (pmd_none(*pmd)) {
		pte_t *new_pte = pte_alloc_one_kernel(NULL, addr);
		WARN(!new_pte, "We're hosed, no memory");
		pmd_populate_kernel(NULL, pmd, new_pte);
	}
	return pte_offset(pmd, addr);
}

static int mipsvz_create_irqchip(struct kvm *kvm)
{
	struct kvm_mips_vz *kvm_mips_vz = kvm->arch.impl;
	int ret = 0;
	pfn_t pfn;
	pte_t *ptep, entry;
	struct mipsvz_irq_chip *ic;

	mutex_lock(&kvm->lock);

	if (kvm_mips_vz->irq_chip) {
		ret = -EEXIST;
		goto out;
	}
	ic = kzalloc(sizeof(struct mipsvz_irq_chip), GFP_KERNEL);
	if (!ic) {
		ret = -ENOMEM;
		goto out;
	}

	ic->page = alloc_page(GFP_KERNEL | __GFP_ZERO);
	if (!ic->page) {
		kfree(ic);
		ret = -ENOMEM;
		goto out;
	}
	ic->base = page_address(ic->page);
	ic->version = KVM_MIPSVZ_IC_VERSION;
	ic->numbits = KVM_MIPSVZ_IC_NUM_BITS;
	ic->numcpus = min(KVM_MIPSVZ_IC_NUM_CPUS, KVM_MAX_VCPUS);
	ic->bm_length = (ic->numbits + 32 - 1) / 32;
	ic->bm_size = ic->bm_length * 4;

	/* size of some bitmaps depends on this */
	BUG_ON(ic->numbits % 8);
	/* all bitmaps have to fit into one page */
	BUG_ON((ic->bm_size * (4 + ic->numcpus * 3)
			+ KVM_MIPSVZ_IC_BM_AREA) > PAGE_SIZE);

	*(unsigned int *)ic->base = ic->numbits;
	*(unsigned int *)(ic->base + 4) = ic->numcpus;
	*(unsigned int *)(ic->base + 8) = ic->version;

	ic->cpu_irq_src_bitmap = (unsigned long)ic->base + KVM_MIPSVZ_IC_BM_AREA;
	ic->cpu_irq_pend_bitmap = ic->cpu_irq_src_bitmap +
		ic->numcpus * ic->bm_size;
	ic->cpu_irq_en_bitmap = ic->cpu_irq_pend_bitmap +
		ic->numcpus * ic->bm_size;
	ic->irq_pend_bitmap = ic->cpu_irq_en_bitmap +
		ic->numcpus * ic->bm_size;
	ic->irq_en_bitmap = ic->irq_pend_bitmap + ic->bm_size;
	ic->tmp_bitmap = ic->irq_en_bitmap + ic->bm_size;

	ptep = mipsvz_pte_for_gpa(kvm, KVM_MIPSVZ_IRQCHIP_START);

	pfn = page_to_pfn(ic->page);
	entry = pfn_pte(pfn, __pgprot(_PAGE_VALID));
	set_pte(ptep, entry);

	kvm_mips_vz->irq_chip = ic;
out:
	mutex_unlock(&kvm->lock);
	return ret;
}

/*
 * IRQ-pending-bits or irq-enable-bits could have changed and thus
 * recalculation of irq-src bitmaps for each online CPU is required:
 *
 * - determine "global" irq-source bitmap (saved in tmp_bm)
 *
 * - for each CPU: OR "global" irq-source bitmap with per CPU
 *   irq-pending bitmap
 *
 * - AND resulting bitmap with per CPU irq-enable bitmap (and save
 *   to per CPU irq-src bitmap)
 *
 * Resulting per CPU irq-src bitmap is read by guest to dispatch irqs.
 */
static u32 mipsvz_write_irqchip_new_irqs(struct kvm *kvm)
{
	struct kvm_mips_vz *kvm_mips_vz = kvm->arch.impl;
	struct mipsvz_irq_chip *ic = kvm_mips_vz->irq_chip;
	struct kvm_mips_vcpu_vz *vcpu_vz = NULL;
	unsigned long *cpu_irq_src_bm, *cpu_irq_en_bm, *cpu_irq_pend_bm;
	unsigned long *irq_en_bm, *irq_pend_bm, *tmp_bm; /* *irq_src_bm; */
	int cpu, ret, offset;
	u32 r = 0;
	u8 injected_ipx = 0;

	tmp_bm = (void *)ic->tmp_bitmap;
	irq_en_bm = (void *)ic->irq_en_bitmap;
	irq_pend_bm = (void *)ic->irq_pend_bitmap;

	ret = bitmap_and(tmp_bm, irq_pend_bm, irq_en_bm,
			KVM_MIPSVZ_IC_NUM_BITS);

	for (cpu = 0; cpu < kvm_mips_vz->irq_chip->numcpus ; cpu++) {
		if (kvm->vcpus[cpu]) {
			vcpu_vz = kvm->vcpus[cpu]->arch.impl;
			injected_ipx = vcpu_vz->injected_ipx;
		}
		offset = cpu * ic->bm_size;
		cpu_irq_en_bm = (void *)(ic->cpu_irq_en_bitmap + offset);
		cpu_irq_pend_bm = (void *)(ic->cpu_irq_pend_bitmap + offset);
		cpu_irq_src_bm = (void *)(ic->cpu_irq_src_bitmap + offset);

		/* OR global src_bm w/ per CPU irq-pending bitmap */
		bitmap_or(tmp_bm, cpu_irq_pend_bm, tmp_bm,
			KVM_MIPSVZ_IC_NUM_BITS);
		ret = bitmap_and(tmp_bm, cpu_irq_en_bm, tmp_bm,
				KVM_MIPSVZ_IC_NUM_BITS);
		if (!ret) {
			bitmap_zero(cpu_irq_src_bm, KVM_MIPSVZ_IC_NUM_BITS);
			if (injected_ipx) {
				r |= 1 << cpu;
				vcpu_vz->injected_ipx = 0;
			}
			continue;
		}

		bitmap_copy(cpu_irq_src_bm, tmp_bm, KVM_MIPSVZ_IC_NUM_BITS);

		if (kvm->vcpus[cpu] && !injected_ipx) {
			r |= 1 << cpu;
			vcpu_vz->injected_ipx = 4;
		}
	}
	return r;
}

static void mipsvz_assert_irqs(struct kvm *kvm, u32 effected_cpus)
{
	int i, me;
	struct kvm_vcpu *vcpu;

	me = get_cpu();

	kvm_for_each_vcpu(i, vcpu, kvm) {
		if (!((1 << vcpu->vcpu_id) & effected_cpus))
			continue;

		if (me == vcpu->cpu) {
			u32 gc2 = read_c0_guestctl2();
			struct kvm_mips_vcpu_vz *vcpu_vz = vcpu->arch.impl;
			gc2 = (gc2 & ~0xff00) | (((u32)vcpu_vz->injected_ipx) << 8);
			write_c0_guestctl2(gc2);
		} else {
			kvm_vcpu_kick(vcpu);
		}
	}

	put_cpu();
}

/* Assumption mutex_lock(&kvm->lock) held */
static int mipsvz_write_irqchip_reg(struct kvm *kvm, unsigned long irq,
				unsigned int cpu, unsigned long offset)
{
	struct kvm_mips_vz *kvm_mips_vz = kvm->arch.impl;
	struct mipsvz_irq_chip *ic = kvm_mips_vz->irq_chip;
	unsigned long flags;
	unsigned long *bitmap;
	u32 effected_cpus;
	bool clear;

	spin_lock_irqsave(&kvm_mips_vz->irq_chip_lock, flags);

	clear = true;
	switch (offset) {
	case KVM_MIPSVZ_IC_REG_IRQ_SET:
		clear = false;		/* fallthrough */
	case KVM_MIPSVZ_IC_REG_IRQ_CLR:
		bitmap = (void *)ic->irq_pend_bitmap;
		break;

	case KVM_MIPSVZ_IC_REG_IRQ_EN:
		clear = false;		/* fallthrough */
	case KVM_MIPSVZ_IC_REG_IRQ_DIS:
		bitmap = (void *)ic->irq_en_bitmap;
		break;

	case KVM_MIPSVZ_IC_REG_CPU_IRQ_SET:
		clear = false;		/* fallthrough */
	case KVM_MIPSVZ_IC_REG_CPU_IRQ_CLR:
		bitmap = (void *)(ic->cpu_irq_pend_bitmap + (cpu * ic->bm_size));
		break;

	case KVM_MIPSVZ_IC_REG_CPU_IRQ_EN:
		clear = false;		/* fallthrough */
	case KVM_MIPSVZ_IC_REG_CPU_IRQ_DIS:
		bitmap = (void *)(ic->cpu_irq_en_bitmap + (cpu * ic->bm_size));
		break;

	default:
		kvm_err("Error: Bad irq_chip register write. offset: 0x%lx\n",
			offset);
		goto err;
	}

	if (clear)
		clear_bit(irq, bitmap);
	else
		set_bit(irq, bitmap);

	effected_cpus = mipsvz_write_irqchip_new_irqs(kvm);

	spin_unlock_irqrestore(&kvm_mips_vz->irq_chip_lock, flags);

	if (effected_cpus)
		mipsvz_assert_irqs(kvm, effected_cpus);

	return 0;

err:
	return -EINVAL;
}

static bool mipsvz_write_irqchip(struct kvm_mips_vz_regs *regs,
				 unsigned long write,
				 unsigned long address,
				 struct kvm *kvm,
				 struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vz *kvm_mips_vz = kvm->arch.impl;
	struct mipsvz_szreg szreg;
	u32 insn, val, cpu;
	unsigned long irq, offset;
	int ret;

	insn = regs->cp0_badinstr;
	offset = address - KVM_MIPSVZ_IRQCHIP_START;

	if (!write || ! kvm_mips_vz->irq_chip) {
		kvm_err("Error: Read fault in irqchip\n");
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
	}

	if (mipsvz_compute_return_epc(regs)) {
		kvm_err("Error: Bad EPC on store emulation: %08x\n", insn);
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
	}

	szreg = mipsvz_get_store_params(insn);
	val = (u32)regs->pt.regs[szreg.reg];

	if (szreg.size == 8) {
		kvm_err("Error: Bad szreg.size (8)\n");
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
	}

	irq = val & (BIT(20) - 1);
	cpu = val >> 20;

	mutex_lock(&kvm->lock);
	ret = mipsvz_write_irqchip_reg(kvm, irq, cpu, offset);
	mutex_unlock(&kvm->lock);
	if (ret)
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);

	return true;
}

int kvm_set_msi(struct kvm_kernel_irq_routing_entry *e,
		struct kvm *kvm, int irq_source_id, int level, bool line_status)
{
	struct kvm_mips_vz *kvm_mips_vz = kvm->arch.impl;
	unsigned long irq;

	if (!level)
		return -1;

	if (!kvm_mips_vz->irq_chip)
		return -ENODEV;

	/* we have stored msi_nr there (take care of byte swapping) */
	irq = le32_to_cpu(e->msi.data);

	if (irq >= kvm_mips_vz->irq_chip->numbits)
		return -EINVAL;

	mipsvz_write_irqchip_reg(kvm, irq, 0, KVM_MIPSVZ_IC_REG_IRQ_SET);

	return 0;
}

static int kvm_set_master_irq(struct kvm_kernel_irq_routing_entry *e,
			      struct kvm *kvm, int irq_source_id, int level,
			      bool line_status)
{
	printk(KERN_ERR "%s (%s: %d)\n", __func__, __FILE__, __LINE__);
	return 0;
}

int kvm_set_routing_entry(struct kvm_irq_routing_table *rt,
			  struct kvm_kernel_irq_routing_entry *e,
			  const struct kvm_irq_routing_entry *ue)
{
	switch (ue->type) {
	case KVM_IRQ_ROUTING_IRQCHIP:
		e->set = kvm_set_master_irq;
		e->irqchip.irqchip = ue->u.irqchip.irqchip;
		e->irqchip.pin = ue->u.irqchip.pin;
		rt->chip[ue->u.irqchip.irqchip][e->irqchip.pin] = ue->gsi;
		break;
	case KVM_IRQ_ROUTING_MSI:
		e->set = kvm_set_msi;
		e->msi.address_lo = ue->u.msi.address_lo;
		e->msi.address_hi = ue->u.msi.address_hi;
		e->msi.data = ue->u.msi.data;
		break;
	default:
		break;
	}
	return 0;
}

static int mipsvz_irq_line(struct kvm *kvm, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct kvm_mips_vz *kvm_mips_vz = kvm->arch.impl;
	struct kvm_irq_level irq_level;

	if (!kvm_mips_vz->irq_chip)
		return -ENODEV;

	if (copy_from_user(&irq_level, argp, sizeof(irq_level)))
		return -EFAULT;

	if (irq_level.irq < 9)
		return 0; /* Ignore */

	if (irq_level.irq >= kvm_mips_vz->irq_chip->numbits)
		return -EINVAL;

	mutex_lock(&kvm->lock);
	if (irq_level.level)
		mipsvz_write_irqchip_reg(kvm, irq_level.irq, 0, KVM_MIPSVZ_IC_REG_IRQ_SET);
	else
		mipsvz_write_irqchip_reg(kvm, irq_level.irq, 0, KVM_MIPSVZ_IC_REG_IRQ_CLR);
	mutex_unlock(&kvm->lock);

	return 0;
}

static enum hrtimer_restart mipsvz_compare_timer_expire(struct hrtimer *t)
{
	struct kvm_mips_vcpu_vz *vcpu_vz;
	vcpu_vz = container_of(t, struct kvm_mips_vcpu_vz, compare_timer);
	kvm_vcpu_kick(vcpu_vz->vcpu);

	return HRTIMER_NORESTART;
}

static long mipsvz_vm_ioctl(struct kvm *kvm, unsigned int ioctl,
			    unsigned long arg)
{
	int r = -ENOIOCTLCMD;

	switch (ioctl) {
	case KVM_CREATE_IRQCHIP:
		r = mipsvz_create_irqchip(kvm);
		break;
	case KVM_IRQ_LINE:
		r = mipsvz_irq_line(kvm, arg);
		break;
	default:
		break;
	}
	return r;
}

static struct kvm_vcpu *mipsvz_vcpu_create(struct kvm *kvm,
					   unsigned int id)
{
	int r;
	struct kvm_vcpu *vcpu = NULL;
	struct kvm_mips_vcpu_vz *vcpu_vz = NULL;
	struct mipsvz_kvm_tlb_entry *tlb_state = NULL;

	/* MIPS CPU numbers have a maximum of 10 significant bits. */
	if (id >= (1u << 10) || id >= KVM_MAX_VCPUS)
		return ERR_PTR(-EINVAL);

	vcpu_vz = kzalloc(sizeof(struct kvm_mips_vcpu_vz), GFP_KERNEL);
	if (!vcpu_vz) {
		r = -ENOMEM;
		goto err;
	}

	vcpu = kzalloc(sizeof(struct kvm_vcpu), GFP_KERNEL);
	if (!vcpu) {
		r = -ENOMEM;
		goto err;
	}
	vcpu->arch.impl = vcpu_vz;
	vcpu_vz->vcpu = vcpu;

	vcpu_vz->tlb_size = 128;
	tlb_state = kzalloc(sizeof(struct mipsvz_kvm_tlb_entry) * vcpu_vz->tlb_size,
			    GFP_KERNEL);
	if (!tlb_state) {
		r = -ENOMEM;
		goto err;
	}

	vcpu_vz->tlb_state = tlb_state;

	hrtimer_init(&vcpu_vz->compare_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
	vcpu_vz->compare_timer.function = mipsvz_compare_timer_expire;

	r = kvm_vcpu_init(vcpu, kvm, id);
	if (r)
		goto err;

	return vcpu;
err:
	kfree(vcpu);
	kfree(tlb_state);
	return ERR_PTR(r);
}

static int mipsvz_vcpu_setup(struct kvm_vcpu *vcpu)
{
	int i;
	unsigned long entryhi_base;
	struct kvm_mips_vcpu_vz *vcpu_vz = vcpu->arch.impl;

	entryhi_base = 0xffffffff90000000ul & mipsvz_entryhi_mask;

	vcpu_vz->c0_ebase = 0xffffffff80000000ull | vcpu->vcpu_id;
	vcpu_vz->c0_status = ST0_BEV | ST0_ERL;

	for (i = 0; i < vcpu_vz->tlb_size; i++) {
		vcpu_vz->tlb_state[i].entryhi = entryhi_base + 8192 * i;
		vcpu_vz->tlb_state[i].entrylo0 = 0;
		vcpu_vz->tlb_state[i].entrylo1 = 0;
		vcpu_vz->tlb_state[i].pagemask = 0;
	}
	return 0;
}

static void mipsvz_vcpu_free(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_vz *vcpu_vz = vcpu->arch.impl;
	hrtimer_cancel(&vcpu_vz->compare_timer);
	kfree(vcpu_vz->tlb_state);
	kfree(vcpu_vz);
	kvm_vcpu_uninit(vcpu);
	kfree(vcpu);
}

static void mipsvz_vcpu_put(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_vz *vcpu_vz = vcpu->arch.impl;
	unsigned long flags;
	int i;
	u64 memctl2, vmconfig;
	int mmu_sizem1;

	mipsvz_readout_guest_cp0(vcpu_vz);

	local_irq_save(flags);

	for (i = 0; i < vcpu_vz->tlb_size; i++) {
		write_gc0_index(i);
		guest_tlb_read();
		vcpu_vz->tlb_state[i].entryhi = read_gc0_entryhi();
		vcpu_vz->tlb_state[i].entrylo0 = read_gc0_entrylo0();
		vcpu_vz->tlb_state[i].entrylo1 = read_gc0_entrylo1();
		vcpu_vz->tlb_state[i].pagemask = read_gc0_pagemask();
	}

	memctl2 = __read_64bit_c0_register($16, 6); /* 16,6: CvmMemCtl2 */
	memctl2 |= (1ull << 17); /* INHIBITTS */
	__write_64bit_c0_register($16, 6, memctl2);

	vmconfig = __read_64bit_c0_register($16, 7); /* 16,7: CvmVMConfig */
	vmconfig &= ~0xffull;

	mmu_sizem1 = (vmconfig >> 12) & 0xff;
	vmconfig |= mmu_sizem1;		/* Root size TLBM1 */
	__write_64bit_c0_register($16, 7, vmconfig);

	current_cpu_data.tlbsize = mmu_sizem1 + 1;
	local_flush_tlb_all();

	memctl2 &= ~(1ull << 17); /* INHIBITTS */
	__write_64bit_c0_register($16, 6, memctl2);

	local_irq_restore(flags);

	vcpu_vz->rootsp = mips_kvm_rootsp[vcpu->cpu];
	mips_kvm_rootsp[vcpu->cpu] = 0;
	vcpu->cpu = -1;
}

static void mipsvz_vcpu_load(struct kvm_vcpu *vcpu, int cpu)
{
	struct kvm_mips_vcpu_vz *vcpu_vz = vcpu->arch.impl;
	struct kvm *kvm = vcpu->kvm;
	struct kvm_mips_vz *kvm_mips_vz = kvm->arch.impl;
	unsigned long flags;
	int i;
	u32 t32;
	u64 cp_val, t64;
	int mmu_size;
	int mmu_sizem1;

	vcpu->cpu = cpu;
	mips_kvm_rootsp[cpu] = vcpu_vz->rootsp;

	/* write_c0_gtoffset(mipsvz_cp0_count_offset[cpu] + vcpu_vz->c0_count_offset); */
	write_c0_gtoffset(0);

	local_irq_save(flags);

	mipsvz_check_asid(vcpu);

	t32 = read_c0_guestctl0();
	/* GM = RI = MC = SFC2 = PIP = 0; CP0 = GT = CG = CF = SFC1 = 1*/
	t32 |= 0xf380fc03;
	t32 ^= 0xe000fc02;

	write_c0_guestctl0(t32);

	t32 = read_gc0_config1();
	t32 &= ~(1u << 3); /* Guest.Config1[WR] = 0 */
	write_gc0_config1(t32);

	t64 = __read_64bit_gc0_register($9, 7); /* 9, 7: Guest.CvmCtl */
	t64 &= ~(7ull << 4); /* IPTI */
	t64 |= (7ull << 4);
	t64 &= ~(7ull << 7); /* IPPCI */
	t64 |= (6ull << 7);
	__write_64bit_gc0_register($9, 7, t64);

	cp_val = __read_64bit_c0_register($16, 7); /* 16, 7: CvmVMConfig */
	cp_val |= (1ull << 60); /* No I/O hole translation. */
	cp_val &= ~0xffull;

	mmu_size = ((cp_val >> 12) & 0xff) + 1;
	cp_val |= mmu_size - vcpu_vz->tlb_size - 1;	/* Root size TLBM1 */
	__write_64bit_c0_register($16, 7, cp_val);

	mmu_sizem1 = cp_val & 0xff;
	current_cpu_data.tlbsize = mmu_sizem1 + 1;

	cp_val = __read_64bit_c0_register($16, 6); /* 16, 6: CvmMemCtl2 */
	cp_val |= (1ull << 17); /* INHIBITTS */
	__write_64bit_c0_register($16, 6, cp_val);

	for (i = 0; i < vcpu_vz->tlb_size; i++) {
		write_gc0_index(i);
		write_gc0_entryhi(vcpu_vz->tlb_state[i].entryhi);
		write_gc0_entrylo0(vcpu_vz->tlb_state[i].entrylo0);
		write_gc0_entrylo1(vcpu_vz->tlb_state[i].entrylo1);
		write_gc0_pagemask(vcpu_vz->tlb_state[i].pagemask);
		guest_tlb_write_indexed();
	}

	cp_val &= ~(1ull << 17); /* INHIBITTS */
	__write_64bit_c0_register($16, 6, cp_val);


	spin_lock(&kvm_mips_vz->irq_chip_lock);
	if (kvm_mips_vz->irq_chip) {
		u32 gc2 = read_c0_guestctl2();
		gc2 = (gc2 & ~0xff00) | (((u32)vcpu_vz->injected_ipx) << 8);
		write_c0_guestctl2(gc2);
	}
	spin_unlock(&kvm_mips_vz->irq_chip_lock);

	local_irq_restore(flags);

	mipsvz_install_guest_cp0(vcpu_vz);
	vcpu_vz->have_counter_state = false;
	/* OCTEON need a local iCache flush on switching guests. */
	local_flush_icache_range(0, 0);
}

static bool mipsvz_emulate_io(struct kvm_mips_vz_regs *regs,
			      unsigned long write,
			      unsigned long address,
			      struct kvm *kvm,
			      struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_vz *vcpu_vz = vcpu->arch.impl;
	u32 insn = regs->cp0_badinstr;

	if (mipsvz_compute_return_epc(regs)) {
		kvm_err("Error: Bad EPC on store emulation: %08x\n", insn);
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
	}
	vcpu->run->io.port = address - KVM_MIPSVZ_IOPORT_START;
	vcpu->run->io.count = 1;
	/* Store the data after the end of the kvm_run */
	vcpu->run->io.data_offset = sizeof(struct kvm_run);
	if (write) {
		u64 val;
		void *dest = sizeof(struct kvm_run) + (char *)((void *)vcpu->run);
		struct mipsvz_szreg r = mipsvz_get_store_params(insn);
		if (r.reg < 0) {
			kvm_err("Error: Bad insn on store emulation: %08x\n", insn);
			mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
		}
		vcpu->run->io.size = r.size;
		vcpu->run->io.direction = KVM_EXIT_IO_OUT;
		val = regs->pt.regs[r.reg];
		switch (r.size) {
		case 1:
			*(u8 *)dest = (u8)val;
			kvm_debug("I/O out %02x -> %04x\n", (unsigned)(u8)val,
				  vcpu->run->io.port);
			break;
		case 2:
			*(u16 *)dest = (u16)val;
			kvm_debug("I/O out %04x -> %04x\n", (unsigned)(u16)val,
				  vcpu->run->io.port);
			break;
		case 4:
			*(u32 *)dest = (u32)val;
			kvm_debug("I/O out %08x -> %04x\n", (unsigned)(u32)val,
				  vcpu->run->io.port);
			break;
		default:
			*(u64 *)dest = val;
			kvm_debug("I/O out %016lx -> %04x\n", (unsigned long)val,
				  vcpu->run->io.port);
			break;
		}
	} else {
		struct mipsvz_szreg r = mipsvz_get_load_params(insn);
		if (r.reg < 0) {
			kvm_err("Error: Bad insn on load emulation: %08x\n", insn);
			mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
		}
		vcpu_vz->last_exit_insn = insn;
		vcpu->run->io.size = r.size;
		vcpu->run->io.direction = KVM_EXIT_IO_IN;
		kvm_debug("I/O in %04x ...\n", vcpu->run->io.port);
	}
	mipsvz_exit_vm(vcpu, regs, KVM_EXIT_IO);
	/* Never Gets Here. */
	return true;
}

static bool mipsvz_emulate_mmio(struct kvm_mips_vz_regs *regs,
				unsigned long write,
				unsigned long address,
				struct kvm *kvm,
				struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_vz *vcpu_vz = vcpu->arch.impl;
	u32 insn = regs->cp0_badinstr;
	void *data = vcpu->run->mmio.data;
	int srcu_idx;
	int ret;

	if (mipsvz_compute_return_epc(regs)) {
		kvm_err("Error: Bad EPC on store emulation: %08x\n", insn);
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
	}
	vcpu->run->mmio.phys_addr = address;
	if (write) {
		u64 val;
		struct mipsvz_szreg r = mipsvz_get_store_params(insn);
		if (r.reg < 0) {
			kvm_err("Error: Bad insn on store emulation: %08x\n", insn);
			mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
		}
		vcpu->run->mmio.len = r.size;
		vcpu->run->mmio.is_write = 1;
		val = regs->pt.regs[r.reg];
		switch (r.size) {
		case 1:
			*(u8 *)data = (u8)val;
			kvm_debug("MMIO out %02x -> %016llx, data: %016llx\n", (unsigned)(u8)val,
				  vcpu->run->mmio.phys_addr, *(u64*)data);
			break;
		case 2:
			*(u16 *)data = (u16)val;
			kvm_debug("MMIO out %04x -> %016llx, data: %016llx\n", (unsigned)(u16)val,
				  vcpu->run->mmio.phys_addr, *(u64*)data);
			break;
		case 4:
			*(u32 *)data = (u32)val;
			kvm_debug("MMIO out %08x -> %016llx, data: %016llx\n", (unsigned)(u32)val,
				  vcpu->run->mmio.phys_addr, *(u64*)data);
			break;
		default:
			*(u64 *)data = val;
			kvm_debug("MMIO out %016lx -> %016llx, data: %016llx\n", (unsigned long)val,
				  vcpu->run->mmio.phys_addr, *(u64*)data);
			break;
		}

		/*
		 * For eventfd/vhost support call io_bus_write instead
		 * of exiting to userspace.
		 */
		srcu_idx = srcu_read_lock(&kvm->srcu);
		ret = kvm_io_bus_write(kvm, KVM_MMIO_BUS, address, r.size, data);
		srcu_read_unlock(&kvm->srcu, srcu_idx);
		if (!ret)
			return true;

	} else {
		struct mipsvz_szreg r = mipsvz_get_load_params(insn);
		if (r.reg < 0) {
			kvm_err("Error: Bad insn on load emulation: %08x\n", insn);
			mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
		}
		vcpu_vz->last_exit_insn = insn;
		vcpu->run->mmio.len = r.size;
		vcpu->run->mmio.is_write = 0;
		kvm_debug("MMIO in %016llx ..., data: %016llx\n", vcpu->run->mmio.phys_addr,
			  *(u64*)data);
	}
	mipsvz_exit_vm(vcpu, regs, KVM_EXIT_MMIO);
	/* Never Gets Here. */
	return true;
}

/* Return true if its a mipsvz guest fault. */
static bool mipsvz_page_fault(struct kvm_vcpu *vcpu,
			      struct kvm_mips_vz_regs *regs,
			      unsigned long write,
			      unsigned long address)
{
	unsigned long flags;
	pte_t *ptep, entry;
	u64 saved_entryhi;
	pfn_t pfn;
	s32 idx;
	int srcu_idx;
	unsigned long prot_bits;
	struct kvm *kvm;
	struct kvm_mips_vz *kvm_mips_vz;
	struct kvm_mips_vcpu_vz *vcpu_vz = vcpu->arch.impl;
	bool writable;

	/*
	 * Guest Physical Addresses can only be in the XKUSEG range
	 * (which ends at XKSSEG).  Other addresses belong to the kernel.
	 */
	if (address >= XKSSEG)
		return false;

	kvm = vcpu->kvm;
	kvm_mips_vz = kvm->arch.impl;

	if ((address >= KVM_MIPSVZ_IO_START) && (address < KVM_MIPSVZ_IO_END)) {
		if (address < KVM_MIPSVZ_MMIO_END) {
			return mipsvz_emulate_mmio(regs, write, address,
						kvm, vcpu);
		} else if (address < KVM_MIPSVZ_IOPORT_END) {
			return mipsvz_emulate_io(regs, write, address,
						 kvm, vcpu);
		} else if (address < KVM_MIPSVZ_IRQCHIP_END) {
			return mipsvz_write_irqchip(regs, write, address,
						    kvm, vcpu);
		} else {
			mipsvz_exit_vm(vcpu, regs, KVM_EXIT_EXCEPTION);
			/* Never Gets Here. */
		}
	}

	writable = false;

	mutex_lock(&kvm_mips_vz->guest_mm_lock);

	srcu_idx = srcu_read_lock(&kvm->srcu);

	pfn = gfn_to_pfn_prot(kvm, address >> PAGE_SHIFT, write, &writable);

#if 0
	kvm_err("mipsvz_page_fault[%d] for %s: %lx -> page %x %s\n",
		vcpu->vcpu_id, write ? "write" : "read",
		address, (unsigned)pfn, writable ? "writable" : "read-only");
#endif

	if (!pfn) {
		kvm_err("mipsvz_page_fault -- no mapping, must exit\n");
		goto bad;
	}

	ptep = mipsvz_pte_for_gpa(kvm, address);

	prot_bits = __READABLE | _PAGE_PRESENT;

	/* If it is the same page, don't downgrade  _PAGE_DIRTY */
	if (pte_pfn(*ptep) == pfn  && (pte_val(*ptep) &  _PAGE_DIRTY))
		prot_bits |= __WRITEABLE;
	if (write) {
		if (!writable) {
			kvm_err("mipsvz_page_fault writing to RO memory.");
			goto bad;
		} else {
			prot_bits |= __WRITEABLE;
			kvm_set_pfn_dirty(pfn);
		}

	} else {
		kvm_set_pfn_accessed(pfn);
	}
	entry = pfn_pte(pfn, __pgprot(prot_bits));

	set_pte(ptep, entry);

	/* Directly set a valid TLB entry.  No more faults. */

	local_irq_save(flags);
	saved_entryhi = read_c0_entryhi();
	address &= (PAGE_MASK << 1);
	write_c0_entryhi(address | vcpu_vz->guest_asid);
	mtc0_tlbw_hazard();
	tlb_probe();
	tlb_probe_hazard();
	idx = read_c0_index();

	/* Goto a PTE pair boundry. */
	ptep = (pte_t *)(((unsigned long)ptep) & ~(2 * sizeof(pte_t) - 1));
	write_c0_entrylo0(pte_to_entrylo(pte_val(*ptep++)));
	write_c0_entrylo1(pte_to_entrylo(pte_val(*ptep)));
	mtc0_tlbw_hazard();
	if (idx < 0)
		tlb_write_random();
	else
		tlb_write_indexed();
	tlbw_use_hazard();
	write_c0_entryhi(saved_entryhi);
	local_irq_restore(flags);

	srcu_read_unlock(&kvm->srcu, srcu_idx);
	mutex_unlock(&kvm_mips_vz->guest_mm_lock);
	return true;

bad:
	srcu_read_unlock(&kvm->srcu, srcu_idx);
	mutex_unlock(&kvm_mips_vz->guest_mm_lock);
	mipsvz_exit_vm(vcpu, regs, KVM_EXIT_EXCEPTION);
	/* Never Gets Here. */
	return true;
}

int kvm_unmap_hva(struct kvm *kvm, unsigned long hva)
{
	kvm_debug("kvm_unmap_hva for %lx\n", hva);
	return 1;
}

void kvm_set_spte_hva(struct kvm *kvm, unsigned long hva, pte_t pte)
{
	kvm_err("kvm_set_spte_hva %lx\n", hva);
}

int kvm_age_hva(struct kvm *kvm, unsigned long hva)
{
	kvm_err("kvm_age_hva %lx\n", hva);
	return 0;
}

int kvm_test_age_hva(struct kvm *kvm, unsigned long hva)
{
	kvm_err("kvm_test_age_hva %lx\n", hva);
	return 0;
}

static void mipsvz_cp_unusable(struct kvm_vcpu *vcpu,
			       struct kvm_mips_vz_regs *regs)
{
	bool handled = false;
	unsigned int cpid = (regs->pt.cp0_cause >> CAUSEB_CE) & 3;

	/* This could take a while, turn interrupts back on. */
	local_irq_enable();
	preempt_disable();

	if (cpid != 1 || !cpu_has_fpu)
		goto out;

	regs->pt.cp0_status |= (ST0_CU1 | ST0_FR); /* Enable FPU in guest ... */
	set_c0_status(ST0_CU1 | ST0_FR);  /* ... and now so we can install its contents. */
	enable_fpu_hazard();
	mipsvz_install_fpu(vcpu);

	handled = true;
out:
	preempt_enable();
	if (!handled)
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
}

static void mipsvz_commit_memory_region(struct kvm *kvm,
					struct kvm_userspace_memory_region *mem,
					const struct kvm_memory_slot *old,
					enum kvm_mr_change change)
{
}

static int mipsvz_vcpu_init(struct kvm_vcpu *vcpu)
{
	return 0;
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

static int mipsvz_get_reg(struct kvm_vcpu *vcpu,
			  const struct kvm_one_reg *reg,
			  s64 *v)
{
	switch (reg->id) {
	case KVM_REG_MIPS_CP0_INDEX:
		*v = read_gc0_index();
		break;
	case KVM_REG_MIPS_CP0_ENTRYLO0:
		*v = read_gc0_entrylo0();
		break;
	case KVM_REG_MIPS_CP0_ENTRYLO1:
		*v = read_gc0_entrylo1();
		break;
	case KVM_REG_MIPS_CP0_CONTEXT:
		*v = read_gc0_context();
		break;
	case KVM_REG_MIPS_CP0_USERLOCAL:
		*v = read_gc0_userlocal();
		break;
	case KVM_REG_MIPS_CP0_PAGEMASK:
		*v = read_gc0_pagemask();
		break;
	case KVM_REG_MIPS_CP0_PAGEGRAIN:
		*v = read_gc0_pagegrain();
		break;
	case KVM_REG_MIPS_CP0_WIRED:
		*v = read_gc0_wired();
		break;
	case KVM_REG_MIPS_CP0_HWRENA:
		*v = read_gc0_hwrena();
		break;
	case KVM_REG_MIPS_CP0_BADVADDR:
		*v = read_gc0_badvaddr();
		break;
	case KVM_REG_MIPS_CP0_COUNT:
		*v = read_gc0_count();
		break;
	case KVM_REG_MIPS_CP0_ENTRYHI:
		*v = read_gc0_entryhi();
		break;
	case KVM_REG_MIPS_CP0_COMPARE:
		*v = read_gc0_compare();
		break;
	case KVM_REG_MIPS_CP0_STATUS:
		*v = read_gc0_status();
		break;
	case KVM_REG_MIPS_CP0_CAUSE:
		*v = read_gc0_cause();
		break;
	case KVM_REG_MIPS_CP0_EBASE:
		*v = read_gc0_ebase();
		break;
	case KVM_REG_MIPS_CP0_XCONTEXT:
		*v = read_gc0_xcontext();
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int mipsvz_set_reg(struct kvm_vcpu *vcpu,
			  const struct kvm_one_reg *reg,
			  u64 v)
{
	switch (reg->id) {
	case KVM_REG_MIPS_CP0_INDEX:
		write_gc0_index(v);
		break;
	case KVM_REG_MIPS_CP0_ENTRYLO0:
		write_gc0_entrylo0(v);
		break;
	case KVM_REG_MIPS_CP0_ENTRYLO1:
		write_gc0_entrylo1(v);
		break;
	case KVM_REG_MIPS_CP0_CONTEXT:
		write_gc0_context(v);
		break;
	case KVM_REG_MIPS_CP0_USERLOCAL:
		write_gc0_userlocal(v);
		break;
	case KVM_REG_MIPS_CP0_PAGEMASK:
		write_gc0_pagemask(v);
		break;
	case KVM_REG_MIPS_CP0_PAGEGRAIN:
		write_gc0_pagegrain(v);
		break;
	case KVM_REG_MIPS_CP0_WIRED:
		write_gc0_wired(v);
		break;
	case KVM_REG_MIPS_CP0_HWRENA:
		write_gc0_hwrena(v);
		break;
	case KVM_REG_MIPS_CP0_BADVADDR:
		write_gc0_badvaddr(v);
		break;
/*
	case MSR_MIPS_CP0_COUNT:
		????;
		break;
*/
	case KVM_REG_MIPS_CP0_ENTRYHI:
		write_gc0_entryhi(v);
		break;
	case KVM_REG_MIPS_CP0_COMPARE:
		write_gc0_compare(v);
		break;
	case KVM_REG_MIPS_CP0_STATUS:
		write_gc0_status(v);
		break;
	case KVM_REG_MIPS_CP0_CAUSE:
		write_gc0_cause(v);
		break;
	case KVM_REG_MIPS_CP0_EBASE:
		write_gc0_ebase(v);
		break;
	case KVM_REG_MIPS_CP0_XCONTEXT:
		write_gc0_xcontext(v);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int mipsvz_vcpu_run(struct kvm_vcpu *vcpu, struct kvm_run *kvm_run)
{
	int ret = 0;
	struct kvm *kvm = vcpu->kvm;
	struct kvm_mips_vz *kvm_mips_vz = kvm->arch.impl;
	struct kvm_mips_vcpu_vz *vcpu_vz = vcpu->arch.impl;

	if ((kvm_run->exit_reason == KVM_EXIT_MMIO) &&
	    !kvm_run->mmio.is_write)
		ret = mipsvz_handle_io_in(vcpu, 1);
	else if ((kvm_run->exit_reason == KVM_EXIT_IO) &&
	      (kvm_run->io.direction == KVM_EXIT_IO_IN))
		ret = mipsvz_handle_io_in(vcpu, 0);

	if (unlikely(ret)) {
		pr_warn("Error: Return from KVM_EXIT_IO with bad exit_insn state.\n");
		return ret;
	}

	lose_fpu(1);

	kvm_debug("mipsvz_vcpu_run enter epc: %016lx\n", vcpu->arch.epc);

	WARN(irqs_disabled(), "IRQs should be on here.");
	local_irq_disable();
	kvm_run->exit_reason = KVM_EXIT_UNKNOWN;

	kvm_guest_enter();

	write_c0_entryhi(vcpu_vz->guest_asid);
	TLBMISS_HANDLER_SETUP_PGD(kvm_mips_vz->pgd);

	mipsvz_start_guest(vcpu);

	/* Save FPU if needed. */
	if (read_c0_status() & ST0_CU1) {
		set_c0_status(ST0_FR);
		mipsvz_readout_fpu(vcpu);
		disable_fpu();
	}

	kvm_guest_exit();

	local_irq_enable();

	if (signal_pending(current)) {
		kvm_run->exit_reason = KVM_EXIT_INTR;
		ret = -EINTR;
	}

	kvm_debug("mipsvz_vcpu_run exit epc: %016lx\n", vcpu->arch.epc);
	return ret;
}

static int mipsvz_cpu_has_pending_timer(struct kvm_vcpu *vcpu)
{
	int me;
	int ret = 0;

	me = get_cpu();

	if (vcpu->cpu == me) {
		u32 cause = read_gc0_cause();
		ret = (cause & CAUSEF_TI) != 0;
	} else {
		kvm_err("kvm_cpu_has_pending_timer:  Argh!!\n");
	}

	put_cpu();

	kvm_debug("kvm_cpu_has_pending_timer: %d\n", ret);
	return ret;
}

static int mipsvz_vcpu_runnable(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_vz *vcpu_vz = vcpu->arch.impl;
	unsigned long flags;
	u64 *irqchip_regs;
	struct kvm *kvm = vcpu->kvm;
	struct kvm_mips_vz *kvm_mips_vz = kvm->arch.impl;
	u8 injected_ipx;
	int r = 0;

	if (!kvm_mips_vz->irq_chip)
		goto out;

	irqchip_regs = kvm_mips_vz->irq_chip->base;

	spin_lock_irqsave(&kvm_mips_vz->irq_chip_lock, flags);
	injected_ipx = vcpu_vz->injected_ipx;
	spin_unlock_irqrestore(&kvm_mips_vz->irq_chip_lock, flags);

	r =  injected_ipx != 0;
out:
	kvm_debug("kvm_arch_vcpu_runnable: %d\n", r);
	return r;
}

static void mipsvz_hypercall_exit_vm(struct kvm_vcpu *vcpu,
				     struct kvm_mips_vz_regs *regs)
{
	mipsvz_exit_vm(vcpu, regs, KVM_EXIT_SHUTDOWN);
}

static void mipsvz_hypercall_get_host_time(struct kvm_vcpu *vcpu,
					struct kvm_mips_vz_regs *regs)
{
	struct timespec ts;
	u64 ns;

	getrawmonotonic(&ts);
	ns = (u64)timespec_to_ns(&ts);

#ifdef CONFIG_CPU_BIG_ENDIAN
	regs->pt.regs[2] = (s64)(s32)(ns >> 32);
	regs->pt.regs[3] = (s64)(s32)ns;
#else
	regs->pt.regs[2] = (s64)(s32)ns;
	regs->pt.regs[3] = (s64)(s32)(ns >> 32);
#endif

}

static void mipsvz_hypercall_get_hpt_frequency(struct kvm_vcpu *vcpu,
					       struct kvm_mips_vz_regs *regs)
{
	regs->pt.regs[2] = mips_hpt_frequency;
}

typedef void (*mipsvz_hypervisor_handler_t)(struct kvm_vcpu *vcpu,
					    struct kvm_mips_vz_regs *);

static void mipsvz_hypercall(struct kvm_vcpu *vcpu,
			     struct kvm_mips_vz_regs *regs)
{
	unsigned long nr = regs->pt.regs[2];
	struct kvm_run *kvm_run;
	int i;

	kvm_debug("kvm_mipsvz_hypercall: %lx\n", nr);

	if (mipsvz_compute_return_epc(regs)) {
		kvm_err("Error: Bad EPC on hypercall\n");
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
	}

	switch(nr) {
	case KVM_HC_MIPS_CONSOLE_OUTPUT:
		kvm_run = vcpu->run;
		kvm_run->hypercall.nr = nr;
		for (i = 0; i < ARRAY_SIZE(kvm_run->hypercall.args); i++)
			kvm_run->hypercall.args[i] = regs->pt.regs[4 + i];
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_HYPERCALL);
		break;
	case KVM_HC_MIPS_GET_HOST_TIME:
		mipsvz_hypercall_get_host_time(vcpu, regs);
		break;
	case KVM_HC_MIPS_GET_CLOCK_FREQ:
		mipsvz_hypercall_get_hpt_frequency(vcpu, regs);
		break;
	case KVM_HC_MIPS_EXIT_VM:
		mipsvz_hypercall_exit_vm(vcpu, regs);
		break;
	default:
		/* hypercall not implemented */
		break;
	}
}

static void mipsvz_sfce(struct kvm_vcpu *vcpu, struct kvm_mips_vz_regs *regs)
{
	bool is_64bit;
	int rt, rd, sel;
	u64 rt_val;
	u32 t, m;
	u32 insn = regs->cp0_badinstr;

	if ((insn & 0xffc007f8) != 0x40800000) {
		kvm_err("Error: SFCE not on DMTC0/MTC0.\n");
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
	}
	/* Move past the DMTC0/MTC0 insn */
	if (mipsvz_compute_return_epc(regs)) {
		kvm_err("Error: Bad EPC on SFCE\n");
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
	}

	is_64bit = insn & (1 << 21);
	rt = (insn >> 16) & 0x1f;
	rd = (insn >> 11) & 0x1f;
	sel = insn & 7;

	rt_val = rt ? regs->pt.regs[rt] : 0;

	switch ((rd << 3) | sel) {
	case 0x60: /* Status */
		write_gc0_status((u32)rt_val);
		break;
	case 0x61: /* IntCtl */
		/* Ignore */
		break;
	case 0x68: /* Cause */
		m = (1 << 27) | (1 << 23); /* DC and IV bits only */
		t = read_gc0_cause();
		t &= ~m;
		t |= (m & (u32)rt_val);
		write_gc0_cause(t);
		break;
	default:
		kvm_err("Error: SFCE unknown target reg.\n");
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
		break;
	}
}

static void mipsvz_handle_cache(struct kvm_vcpu *vcpu,
				struct kvm_mips_vz_regs *regs,
				union mips_instruction insn)
{
	s64 ea;
	s16 offset;

	/* Move past the CACHE insn */
	if (mipsvz_compute_return_epc(regs)) {
		kvm_err("Error: Bad EPC on CACHE GPSI\n");
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
	}

	offset = insn.c_format.simmediate;

	switch (insn.c_format.cache) {
	case 0: /* Primary Instruction */
		switch (insn.c_format.c_op) {
		case 0: /* Index Invalidate */
			ea = regs->pt.regs[insn.c_format.rs] + offset;
			asm volatile("cache	0x00,0(%0)" : : "d" (ea));
			break;
		case 4: /* ICache invalidate EA */
			ea = regs->pt.regs[insn.c_format.rs] + offset;
			asm volatile("synci	0($0)");
			break;
		default:
			goto cannot_handle;
		}
		break;
	case 1: /* Primary Data */
		switch (insn.c_format.c_op) {
		case 0: /* writebadk/invalidate tag */
#if 0
			ea = regs->regs[insn.c_format.rs] + offset;
			asm volatile("cache	0x01,0(%0)" : : "d" (ea));
			break;
#endif
		case 5: /*  writebadk/invalidate EA */
			/* OCTEON has coherent caches, but clear the write buffers. */
			asm volatile("sync");
			break;
		default:
			goto cannot_handle;
		}
		break;
	case 2: /* Tertiary */
	case 3: /* Secondary */
	default:
		goto cannot_handle;
	}

	return;
cannot_handle:
		kvm_err("Error: GPSI Illegal cache op %08x\n", insn.word);
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_EXCEPTION);
}

static void mipsvz_handle_wait(struct kvm_vcpu *vcpu,
			       struct kvm_mips_vz_regs *regs)
{
	struct kvm_mips_vcpu_vz *vcpu_vz;

	/* Move past the WAIT insn */
	if (mipsvz_compute_return_epc(regs)) {
		kvm_err("Error: Bad EPC on WAIT GPSI\n");
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
	}

	preempt_disable();
	vcpu_vz = vcpu->arch.impl;
	mipsvz_readout_cp0_counter_state(vcpu_vz);
	if ((vcpu_vz->c0_cause & CAUSEF_TI) == 0) {
		ktime_t exp;
		u32 clk_to_exp = vcpu_vz->c0_compare - vcpu_vz->c0_count;
		u64 ns_to_exp = (clk_to_exp * 1000000000ull) / mips_hpt_frequency;
		/* Arm the timer */
		exp = ktime_add_ns(vcpu_vz->compare_timer_read, ns_to_exp);
		hrtimer_start(&vcpu_vz->compare_timer, exp, HRTIMER_MODE_ABS);
	}
	preempt_enable();

	kvm_vcpu_block(vcpu);

	if (signal_pending(current))
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTR);
}

static void mipsvz_handle_gpsi_mtc0(struct kvm_vcpu *vcpu,
				    struct kvm_mips_vz_regs *regs,
				    union mips_instruction insn)
{
	struct kvm_mips_vcpu_vz *vcpu_vz;
	u32 val;
	u32 offset;

	/* Move past the MTC0 insn */
	if (mipsvz_compute_return_epc(regs)) {
		kvm_err("Error: Bad EPC on MTC0 GPSI\n");
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
	}

	preempt_disable();
	vcpu_vz = vcpu->arch.impl;
	switch (insn.c0m_format.rd) {
	case 9:
		if (insn.c0m_format.sel != 0)
			goto bad_reg;
		/* Count */
		val = regs->pt.regs[insn.c0m_format.rt];
		offset = val - read_gc0_count();
		vcpu_vz->c0_count_offset += offset;
		/* write_c0_gtoffset(mipsvz_cp0_count_offset[vcpu->cpu] + vcpu_vz->c0_count_offset); */
		break;
	default:
		goto bad_reg;
	}

	preempt_enable();
	return;

bad_reg:
	kvm_err("Error: Bad Reg($%d,%d) on MTC0 GPSI\n",
		insn.c0m_format.rd, insn.c0m_format.sel);
	mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);

}

static void mipsvz_handle_gpsi_mfc0(struct kvm_vcpu *vcpu,
				    struct kvm_mips_vz_regs *regs,
				    union mips_instruction insn)
{
	/* Move past the MFC0 insn */
	if (mipsvz_compute_return_epc(regs)) {
		kvm_err("Error: Bad EPC on MFC0 GPSI\n");
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
	}

	switch (insn.c0m_format.rd) {
	case 12:
		if (insn.c0m_format.sel != 2)
			goto bad_reg;
		/* SRSCtl */
		regs->pt.regs[insn.c0m_format.rt] = 0;
		break;
	case 15:
		if (insn.c0m_format.sel != 0)
			goto bad_reg;
		/* PRId */
		regs->pt.regs[insn.c0m_format.rt] = (s64)read_c0_prid();
		break;
	case 26:
		if (insn.c0m_format.sel != 0)
			goto bad_reg;
		/* ErrCtl */
		regs->pt.regs[insn.c0m_format.rt] = 0;
		break;
	default:
		goto bad_reg;
	}
	return;

bad_reg:
	kvm_err("Error: Bad Reg($%d,%d) on MFC0 GPSI\n",
		insn.c0m_format.rd, insn.c0m_format.sel);
	mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
}

static void mipsvz_handle_gpsi_dmfc0(struct kvm_vcpu *vcpu,
				     struct kvm_mips_vz_regs *regs,
				     union mips_instruction insn)
{
	/* Move past the DMFC0 insn */
	if (mipsvz_compute_return_epc(regs)) {
		kvm_err("Error: Bad EPC on DMFC0 GPSI\n");
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
	}

	switch (insn.c0m_format.rd) {
	case 26:
		if (insn.c0m_format.sel != 0)
			goto bad_reg;
		/* ErrCtl */
		regs->pt.regs[insn.c0m_format.rt] = 0;
		break;
	default:
		goto bad_reg;
	}
	return;

bad_reg:
	kvm_err("Error: Bad Reg($%d,%d) on DMFC0 GPSI\n",
		insn.c0m_format.rd, insn.c0m_format.sel);
	mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
}

static void mipsvz_gpsi(struct kvm_vcpu *vcpu, struct kvm_mips_vz_regs *regs)
{
	union mips_instruction insn;

	insn.word = regs->cp0_badinstr;

	if (insn.c_format.opcode == cache_op)
		mipsvz_handle_cache(vcpu, regs, insn);
	else if (insn.c0_format.opcode == cop0_op &&
		 insn.c0_format.co == 1 &&
		 insn.c0_format.func == wait_op)
		mipsvz_handle_wait(vcpu, regs);
	else if (insn.c0m_format.opcode == cop0_op &&
		 insn.c0m_format.func == mtc_op &&
		 insn.c0m_format.code == 0)
		mipsvz_handle_gpsi_mtc0(vcpu, regs, insn);
	else if (insn.c0m_format.opcode == cop0_op &&
		 insn.c0m_format.func == mfc_op &&
		 insn.c0m_format.code == 0)
		mipsvz_handle_gpsi_mfc0(vcpu, regs, insn);
	else if (insn.c0m_format.opcode == cop0_op &&
		 insn.c0m_format.func == dmfc_op &&
		 insn.c0m_format.code == 0)
		mipsvz_handle_gpsi_dmfc0(vcpu, regs, insn);
	else {
		kvm_err("Error: GPSI not on CACHE, WAIT, MFC0 or MTC0: %08x\n",
			insn.word);
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
	}
}

static void mipsvz_default_ex(struct kvm_vcpu *vcpu,
			      struct kvm_mips_vz_regs *regs)
{
	u32 guestctl0 = read_c0_guestctl0();
	int gexc_code = (guestctl0 >> 2) & 0x1f;

	kvm_err("Hypervisor Exception (%d): Not handled yet\n", gexc_code);
	mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
}

#define mipsvz_gva mipsvz_default_ex
#define mipsvz_gpa mipsvz_default_ex
#define mipsvz_ghfc mipsvz_default_ex

static const mipsvz_hypervisor_handler_t mipsvz_hypervisor_handlers[] = {
	mipsvz_gpsi,		/* 0  - Guest Privileged Sensitive Instruction */
	mipsvz_sfce,		/* 1  - Guest Software Field Change */
	mipsvz_hypercall,	/* 2  - Hypercall */
	mipsvz_default_ex,	/* 3  - Guest Reserved Instruction Redirect. */
	mipsvz_default_ex,	/* 4  - Implementation defined */
	mipsvz_default_ex,	/* 5  - Implementation defined */
	mipsvz_default_ex,	/* 6  - Implementation defined */
	mipsvz_default_ex,	/* 7  - Implementation defined */
	mipsvz_gva,		/* 8  - Guest Mode Initiated Root TLB Execption: GVA */
	mipsvz_ghfc,		/* 9  - Guest Hardware Field Change */
	mipsvz_gpa,		/* 10 - Guest Mode Initiated Root TLB Execption: GPA */
	mipsvz_default_ex,	/* 11 - Reserved */
	mipsvz_default_ex,	/* 12 - Reserved */
	mipsvz_default_ex,	/* 13 - Reserved */
	mipsvz_default_ex,	/* 14 - Reserved */
	mipsvz_default_ex,	/* 15 - Reserved */
	mipsvz_default_ex,	/* 16 - Reserved */
	mipsvz_default_ex,	/* 17 - Reserved */
	mipsvz_default_ex,	/* 18 - Reserved */
	mipsvz_default_ex,	/* 19 - Reserved */
	mipsvz_default_ex,	/* 20 - Reserved */
	mipsvz_default_ex,	/* 21 - Reserved */
	mipsvz_default_ex,	/* 22 - Reserved */
	mipsvz_default_ex,	/* 23 - Reserved */
	mipsvz_default_ex,	/* 24 - Reserved */
	mipsvz_default_ex,	/* 25 - Reserved */
	mipsvz_default_ex,	/* 26 - Reserved */
	mipsvz_default_ex,	/* 27 - Reserved */
	mipsvz_default_ex,	/* 28 - Reserved */
	mipsvz_default_ex,	/* 29 - Reserved */
	mipsvz_default_ex,	/* 30 - Reserved */
	mipsvz_default_ex,	/* 31 - Reserved */
};

/*
 * Hypervisor Exception handler, called with interrupts disabled.
 */
asmlinkage void mipsvz_do_hypervisor(struct kvm_vcpu *vcpu,
				     struct kvm_mips_vz_regs *regs)
{
	int gexc_code;
	u32 guestctl0 = read_c0_guestctl0();

	/* Must read before any exceptions can happen. */
	regs->cp0_badinstr = read_c0_badinstr();
	regs->cp0_badinstrp = read_c0_badinstrp();

	/* This could take a while, turn interrupts back on. */
	local_irq_enable();

	gexc_code = (guestctl0 >> 2) & 0x1f;

	mipsvz_hypervisor_handlers[gexc_code](vcpu, regs);
}

asmlinkage void mipsvz_do_tlbs(struct kvm_vcpu *vcpu,
			       struct kvm_mips_vz_regs *regs)
{
	unsigned long addr = read_c0_badvaddr();

	/* Must read before any exceptions can happen. */
	regs->cp0_badinstr = read_c0_badinstr();
	regs->cp0_badinstrp = read_c0_badinstrp();

	/* This could take a while, turn interrupts back on. */
	local_irq_enable();

	mipsvz_page_fault(vcpu, regs, 1, addr);
}

asmlinkage void mipsvz_do_tlbl(struct kvm_vcpu *vcpu,
			       struct kvm_mips_vz_regs *regs)
{
	unsigned long addr = read_c0_badvaddr();

	/* Must read before any exceptions can happen. */
	regs->cp0_badinstr = read_c0_badinstr();
	regs->cp0_badinstrp = read_c0_badinstrp();

	/* This could take a while, turn interrupts back on. */
	local_irq_enable();

	mipsvz_page_fault(vcpu, regs, 0, addr);
}

asmlinkage void mipsvz_do_unimp(struct kvm_vcpu *vcpu,
				struct kvm_mips_vz_regs *regs)
{
	/* This could take a while, turn interrupts back on. */
	local_irq_enable();

	mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTERNAL_ERROR);
}

asmlinkage void mipsvz_do_resched(struct kvm_vcpu *vcpu,
				  struct kvm_mips_vz_regs *regs)
{
	/* irqs already enabled here. */
	cond_resched();
	if (signal_pending(current))
		mipsvz_exit_vm(vcpu, regs, KVM_EXIT_INTR);
}

static long mipsvz_vcpu_ioctl(struct kvm_vcpu *vcpu, unsigned int ioctl,
			      unsigned long arg)
{
	return -ENOIOCTLCMD;
}

typedef void (*vz_ex_handler_t)(struct kvm_vcpu *vcpu,
				struct kvm_mips_vz_regs *regs);

vz_ex_handler_t vz_ex_handlers[32] = {
	NULL,			/* 0  - irq (handled in asm code) */
	mipsvz_do_tlbs,		/* 1  - TLB Mod */
	mipsvz_do_tlbl,		/* 2  - TLB Load */
	mipsvz_do_tlbs,		/* 3  - TLB Store */
	mipsvz_do_unimp,	/* 4  - ADE Load */
	mipsvz_do_unimp,	/* 5  - ADE Store */
	mipsvz_do_unimp,	/* 6  - Bus Error I */
	mipsvz_do_unimp,	/* 7  - Bus Error D */
	mipsvz_do_unimp,	/* 8  - Scall */
	mipsvz_do_unimp,	/* 9  - BP */
	mipsvz_do_unimp,	/* 10 - RI */
	mipsvz_cp_unusable,	/* 11 - CP Unusable */
	mipsvz_do_unimp,	/* 12 - Overflow */
	mipsvz_do_unimp,	/* 13 - Trap */
	mipsvz_do_unimp,	/* 14 - MSA FPE */
	mipsvz_do_unimp,	/* 15 - FPE */
	mipsvz_do_unimp,	/* 16 - Implementation Defined */
	mipsvz_do_unimp,	/* 17 - Implementation Defined */
	mipsvz_do_unimp,	/* 18 - CP2 */
	mipsvz_do_unimp,	/* 19 - TLB RI */
	mipsvz_do_unimp,	/* 20 - TLB XI */
	mipsvz_do_unimp,	/* 21 - MSA Dis */
	mipsvz_do_unimp,	/* 22 - MDMX */
	mipsvz_do_unimp,	/* 23 - Watch */
	mipsvz_do_unimp,	/* 24 - MCheck */
	mipsvz_do_unimp,	/* 25 - Thread */
	mipsvz_do_unimp,	/* 26 - Scall */
	mipsvz_do_hypervisor,	/* 27 - Guest Exception */
	mipsvz_do_unimp,	/* 28 - Reserved */
	mipsvz_do_unimp,	/* 29 - Reserved */
	mipsvz_do_unimp,	/* 30 - Cache Error */
	mipsvz_do_unimp		/* 31 - Reserved */
};

static const struct kvm_mips_ops kvm_mips_vz_ops = {
	.vcpu_runnable = mipsvz_vcpu_runnable,
	.destroy_vm = mipsvz_destroy_vm,
	.commit_memory_region = mipsvz_commit_memory_region,
	.vcpu_create = mipsvz_vcpu_create,
	.vcpu_free = mipsvz_vcpu_free,
	.vcpu_run = mipsvz_vcpu_run,
	.vm_ioctl = mipsvz_vm_ioctl,
	.vcpu_ioctl = mipsvz_vcpu_ioctl,
	.get_reg = mipsvz_get_reg,
	.set_reg = mipsvz_set_reg,
	.cpu_has_pending_timer = mipsvz_cpu_has_pending_timer,
	.vcpu_init = mipsvz_vcpu_init,
	.vcpu_setup = mipsvz_vcpu_setup,
	.vcpu_load = mipsvz_vcpu_load,
	.vcpu_put = mipsvz_vcpu_put,
};

int mipsvz_init_vm(struct kvm *kvm, unsigned long type)
{
	struct kvm_mips_vz *kvm_mips_vz;

	if (!cpu_has_vz)
		return -ENODEV;
	if (type != 1)
		return -EINVAL;

	kvm->arch.ops = &kvm_mips_vz_ops;

	kvm_mips_vz = kzalloc(sizeof(struct kvm_mips_vz), GFP_KERNEL);
	if (!kvm_mips_vz)
		goto err;

	kvm->arch.impl = kvm_mips_vz;

	mutex_init(&kvm_mips_vz->guest_mm_lock);

	kvm_mips_vz->pgd = (pgd_t *)__get_free_pages(GFP_KERNEL, PGD_ORDER);
	if (!kvm_mips_vz->pgd)
		goto err;

	pgd_init((unsigned long)kvm_mips_vz->pgd);

	spin_lock_init(&kvm_mips_vz->irq_chip_lock);

	return 0;
err:
	kfree(kvm_mips_vz);
	return -ENOMEM;
}

/*
* This file is subject to the terms and conditions of the GNU General Public
* License.  See the file "COPYING" in the main directory of this archive
* for more details.
*
* commpage, currently used for Virtual COP0 registers.
* Mapped into the guest kernel @ 0x0.
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
#include <asm/page.h>
#include <asm/cacheflush.h>
#include <asm/mmu_context.h>

#include <linux/kvm_host.h>

#include <asm/kvm_mips_te.h>

#include "kvm_mips_comm.h"

void kvm_mips_commpage_init(struct kvm_vcpu *vcpu)
{
	struct kvm_mips_vcpu_te *vcpu_te = vcpu->arch.impl;
	struct kvm_mips_commpage *page = vcpu_te->kseg0_commpage;
	memset(page, 0, sizeof(struct kvm_mips_commpage));

	/* Specific init values for fields */
	vcpu_te->cop0 = &page->cop0;
	memset(vcpu_te->cop0, 0, sizeof(struct mips_coproc));

	return;
}

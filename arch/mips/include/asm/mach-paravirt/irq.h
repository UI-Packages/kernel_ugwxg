/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2013 Cavium, Inc.
 */
#ifndef __ASM_MACH_PARAVIRT_IRQ_H__
#define  __ASM_MACH_PARAVIRT_IRQ_H__

#define NR_IRQS			128	/* rquired in irqdesc.h */
#define MIPS_CPU_IRQ_BASE	1

#define MIPS_IRQ_PCI_BASE	(MIPS_CPU_IRQ_BASE + 8)
#define MIPS_IRQ_PCIA		MIPS_IRQ_PCI_BASE
#define MIPS_IRQ_PCI_MAX	31

#define MIPS_IRQ_MBOX0		(MIPS_IRQ_PCI_MAX + 1)
#define MIPS_IRQ_MBOX1		(MIPS_IRQ_MBOX0 + 1)
#define MIPS_IRQ_MBOX2		(MIPS_IRQ_MBOX0 + 2)
#define MIPS_IRQ_MBOX_MAX	48

#define MIPS_IRQ_MSI_BASE	(MIPS_IRQ_MBOX_MAX + 1)
#define MIPS_IRQ_MSI_MAX	(NR_IRQS - 1)

#endif /* __ASM_MACH_PARAVIRT_IRQ_H__ */

/************************************************************************
 * asm-arm/msa.h
 *
 * Provide an architecture-specific clock.
 */

#ifndef _ASM_ARM_MSA_H
#define _ASM_ARM_MSA_H

#if defined(CONFIG_MICROSTATE_ACCT_SCHED_CLOCK_CLOCKSOURCE)
# include <asm-generic/msa.h>
#else
# error "No clocksource defined for Microstate Accounting"
#endif

#endif

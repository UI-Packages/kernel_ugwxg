/*
 * asm-generic/msa.h
 * Provide a generic time-of-day clock for
 * microstate accounting.
 */

#ifndef _ASM_GENERIC_MSA_H
#define _ASM_GENERIC_MSA_H

# ifdef __KERNEL__
/*
 * Every architecture is supposed to provide sched_clock, a free-running,
 * non-wrapping, per-cpu clock in nanoseconds.
 */
#  define MSA_NOW(now) do { preempt_disable(); \
			    (now) = cpu_clock(smp_processor_id()); \
			    preempt_enable(); } while (0)
#  define MSA_TO_NSEC(clk) (clk)
#  define MICROSTATE_ACCT_USING_SCHED_CLOCK
# endif

#endif /* _ASM_GENERIC_MSA_H */

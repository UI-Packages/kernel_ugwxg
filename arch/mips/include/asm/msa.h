/************************************************************************
 * asm-mips/msa.h
 *
 * Provide an architecture-specific clock.
 */

#include <linux/timex.h>
#include <asm/time.h>

#ifndef _ASM_MIPS_MSA_H
#define _ASM_MIPS_MSA_H

# if defined(CONFIG_MICROSTATE_ACCT_SCHED_CLOCK_CLOCKSOURCE)
#  include <asm-generic/msa.h>
# elif defined(CONFIG_MICROSTATE_C0_COUNT_REGISTER)
/*
 * MSA uses MIPS 32 bit C0 counter register.
 */
extern msa_time_t msa_cycles_last;
extern u32 msa_last_count;
extern seqlock_t msa_seqlock;

static inline msa_time_t msa_now(void)
{
	u32 count;
	unsigned long seq, flags;
	msa_time_t ret;

	do {
		seq = read_seqbegin(&msa_seqlock);
		count = read_c0_count();
		/* Udate if delta > (0xffffffff/4) */
		if (count - msa_last_count > 0x3fffffffUL) {
			write_seqlock_irqsave(&msa_seqlock, flags);
			msa_cycles_last += (u32) (count - msa_last_count);
			msa_last_count = count;
			ret = msa_cycles_last;
			write_sequnlock_irqrestore(&msa_seqlock, flags);
			break;
		}
		ret = msa_cycles_last + (u32) (count - msa_last_count);
	} while (read_seqretry(&msa_seqlock, seq));

	return ret;
}

static inline u64 msa_to_nsec(msa_time_t cycles)
{
	msa_time_t sec, nsec;

	sec = cycles;
	/* To prevent overflow, first, extract seconds  */
	nsec = do_div(sec, mips_hpt_frequency);
	/* Then multiply reminder cycles value to get nsecs */
	nsec *= 1000000000ULL;
	do_div(nsec, mips_hpt_frequency);
	return sec * 1000000000ULL + nsec;
}

#  define MSA_NOW(now)  do { (now) = msa_now(); } while (0)
#  define MSA_TO_NSEC(clk) msa_to_nsec(clk)
# else
#  error "No clocksource defined for Microstate Accounting"
# endif

#endif

#ifndef _ASM_GENERIC_CPUTIME_MSA_H
#define _ASM_GENERIC_CPUTIME_MSA_H

#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/msa.h>
#include <linux/math64.h>

#define msa_to_cputime(msa)		MSA_TO_NSEC(msa)
#define msa_to_cputime64(msa)		MSA_TO_NSEC(msa)

typedef u64 cputime_t;

#define cputime_zero			0
#define cputime_one_jiffy		jiffies_to_cputime(1)
#define cputime_max			((~((cputime_t)0) >> 1) - 1)
#define cputime_add(__a, __b)		((__a) +  (__b))
#define cputime_sub(__a, __b)		((__a) -  (__b))
#define cputime_div(__a, __n)		({ u64 __x = (__a); do_div(__x, __n); __x; })
#define cputime_halve(__a)		((__a) >> 1)
#define cputime_eq(__a, __b)		((__a) == (__b))
#define cputime_gt(__a, __b)		((__a) >  (__b))
#define cputime_ge(__a, __b)		((__a) >= (__b))
#define cputime_lt(__a, __b)		((__a) <  (__b))
#define cputime_le(__a, __b)		((__a) <= (__b))
#define cputime_to_jiffies(__ct)	((unsigned long) cputime_div(__ct, TICK_NSEC))
#define cputime_to_scaled(__ct)		cputime_to_jiffies(__ct)
#define jiffies_to_cputime(__hz)	((cputime_t)__hz * TICK_NSEC)

typedef u64 cputime64_t;

#define cputime64_zero			0
#define cputime64_add(__a, __b)		((__a) + (__b))
#define cputime64_sub(__a, __b)		((__a) - (__b))
#define cputime64_to_jiffies64(__ct)	(nsecs_to_jiffies64(__ct))
#define jiffies64_to_cputime64(__jif)	\
	(__force cputime_t)((__jif) * (NSEC_PER_SEC / HZ))
#define cputime_to_cputime64(__ct)	(__ct)
#define nsecs_to_cputime64(__ct)	(__ct)
#define usecs_to_cputime64(__ct)	((__ct)*1000)
/*
 * Convert cputime to microseconds
 */
#define cputime_to_usecs(__ct)		\
	((unsigned int) cputime_div(__ct, NSEC_PER_USEC))

/*
 * Convert cputime to milliseconds and back.
 */
#define cputime_to_msecs(__ct)		((unsigned int) cputime_div(__ct, NSEC_PER_MSEC))
#define msecs_to_cputime(__msecs)	((cputime_t)(__msecs) * NSEC_PER_MSEC)

/*
 * Convert cputime to seconds and back.
 */
#define cputime_to_secs(msa)		((unsigned long) cputime_div(msa, NSEC_PER_SEC))
#define secs_to_cputime(sec)		((cputime_t)(sec) * NSEC_PER_SEC)

/*
 * Convert cputime to timespec and back.
 */
#define timespec_to_cputime(__spec)	\
  ((cputime_t)((__spec)->tv_sec) * NSEC_PER_SEC + (__spec)->tv_nsec)
#define cputime_to_timespec(__ct,__spec) \
  ({ s32 nsec; \
   (__spec)->tv_sec = div_s64_rem(__ct, NSEC_PER_SEC, &nsec); \
   (__spec)->tv_nsec = nsec; })

/*
 * Convert cputime to timeval and back.
 */
#define timeval_to_cputime(__val)	\
  ((cputime_t)((__val)->tv_sec) * NSEC_PER_SEC + (__val)->tv_usec * NSEC_PER_USEC)
#define cputime_to_timeval(__ct,__val)	(*(__val) = ns_to_timeval(__ct))

/*
 * Convert cputime to clock and back.
 */
#define cputime_to_clock_t(__ct)	((clock_t) nsec_to_clock_t(__ct))
#define clock_t_to_cputime(__x)		((cputime_t) (clock_t_to_jiffies(__x) * TICK_NSEC))

/*
 * Convert cputime64 to clock.
 */
#define cputime64_to_clock_t(__ct)	cputime_to_clock_t(__ct)

#endif

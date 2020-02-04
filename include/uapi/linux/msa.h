/*
 * msa.h
 *   microstate accounting.  See Documentation/MicrostateAccounting for info.
 *
 * Copyright (c) Peter Chubb 2005
 *  UNSW and National ICT Australia
 * Copyright (c) 2010 MontaVista Software, LLC
 *  Corey Minyard <minyard@mvista.com>, <minyard@acm.org>, <source@mvista.com>
 */

#ifndef _UAPI_LINUX_MSA_H
#define _UAPI_LINUX_MSA_H

#include <linux/types.h>

typedef uint64_t msa_time_t;

/*
 * Tracked states
 */
enum msa_thread_state {
	MSA_UNKNOWN = -1,
	MSA_ONCPU_USER,
	MSA_ONCPU_SYS,
	MSA_INTERRUPTIBLE_SLEEP,
	MSA_UNINTERRUPTIBLE_SLEEP,
	MSA_ONRUNQUEUE,
	MSA_ZOMBIE,
	MSA_STOPPED,
	MSA_INTERRUPTED,
	MSA_PAGING_SLEEP,
	MSA_FUTEX_SLEEP,
	MSA_POLL_SLEEP,
	MSA_PARKED,

	MSA_NR_STATES /* Must be last */
};

/* Values for "which" in the msa syscall */
#define MSA_THREAD	0	/* Just the current thread */
#define MSA_CHILDREN	1	/* All dead and waited-for threads */
#define MSA_SELF	2	/* All threads in current process */
#define MSA_GET_NOW	3	/* Current MSA timer in the first value */

#endif /* _LINUX_MSA_H */

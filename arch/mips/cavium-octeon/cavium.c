/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2015 Cavium Networks
 *   written by Carlos Munoz <cmunoz@caviumnetworks.com>
 */
#include <linux/types.h>

#include <linux/uaccess.h>

#include <asm/octeon/octeon.h>
#include <asm/octeon/cvmx-clock.h>
#include <asm/octeon/cvmx-wqe.h>
#include <asm/octeon/cvmx-fpa.h>


/*
 * cvmx_tim_status_t:		Error codes returned by timer api's
 *				This enum must be kept in sync with the one in
 *				executive/cvmx-tim.h
 */
typedef enum {
	CVMX_TIM_STATUS_SUCCESS = 0,
	CVMX_TIM_STATUS_NO_MEMORY = -1,
	CVMX_TIM_STATUS_TOO_FAR_AWAY = -2,
	CVMX_TIM_STATUS_BUSY = -3,
	CVMX_TIM_STATUS_LOCK = -4,
	CVMX_TIM_STATUS_NOT_SUPPORTED = -4
} cvmx_tim_status_t;

/*
 * cvmx_tim_entry_chunk_t:	Used to access the wqe's in the chunks.
 *
 *  entries:			Used to access the array or wqe's.
 */
typedef struct cvmx_tim_entry_chunk {
	volatile u64	entries[0];
} cvmx_tim_entry_chunk_t;

/*
 * cvmx_tim_bucket_entry_t:	This structure represents the ring bucket as
 *				seen by hardware.
 *
 *  first_chunk_addr:		Points to the first chunk.
 *  num_entries:		Zeroed by hw after processing the bucket.
 *  chunk_remainder:		Zeroed by hw after processing the bucket.
 *  last_chunk:			Not used by hw.
 *  pad:			Not used by hw.
 */
typedef struct {
	volatile u64			first_chunk_addr;
	volatile u32			num_entries;
	volatile u32			chunk_remainder;
	volatile cvmx_tim_entry_chunk_t	*last_chunk;
	u64				pad;
} cvmx_tim_bucket_entry_t;

/*
 * cvmx_tim_t:		Keeps all the timer management data. This structure
 *			has to be kept in sync with the structure
 *			cvmx_tim_kernel_t in executive/cvmx-tim.h
 *
 *  bucket:		Memory address of bucket array for all rings.
 *  tick_cycles:	Number of coprocessor cycles between buckets.
 *  start_time:		Coprocessor cycle count when timers were started.
 *  bucket_shift:	How long a bucket represents in ms.
 *  num_buckets:	Number of buckets per ring.
 *  max_ticks:		Maximum number of ticks (maximum number of buckets.
 *  num_rings:		Number of rings available.
 */
typedef struct {
	u64	bucket;
	u64	tick_cycles;
	u64	start_time;
	u32	bucket_shift;
	u32	num_buckets;
	u32	max_ticks;
	u32	num_rings;
} cvmx_tim_t;

/*
 * cvmx_tim_info_t:	Structure used to pass timer information to the Linux
 *			kernel to schedule a wqe. This structure must be kept in
 *			sync with the one in executive/cvmx-tim.h
 *
 *  wqe:		Physical address of work queue entry to add to the
 *			timer.
 *  ticks_from_now:	Number of ticks to delay the wqe.
 *  timer_pool:		Fpa pool used for chunks.
 *  timer_pool_size:	Size of fpa buffers in the timer pool.
 *  cvmx_tim:		Timer management data.
 */
typedef struct {
	u64			wqe;
	u64			ticks_from_now;
	u32			timer_pool;
	u32			timer_pool_size;
	cvmx_tim_t		cvmx_tim;
} cvmx_tim_info_t;

/*
 * cvmx_tim_delete_t:	Structure used to store state information needed to
 *			delete an already scheduled timer entry. An instance of
 *			this structure must be passed to arm_timer() in order
 *			to be able to delete an entry later with disarm_timer().
 *			This structure has to be kept in sync with the one in
 *			executive/cvmx-tim.h
 *
 *  commit_cyles:	Last time to safely delete timer.
 *  timer_entry_ptr:	Chunk wqe pointer used by timer.
 */
typedef struct {
	u64		commit_cycles;
	u64		*timer_entry_ptr;
} cvmx_tim_delete_t;


/*
 * cvmx_tim_wqe_store:	Store work queue entry into timer chunk.
 *
 *  ptr:		Updated to point to wqe.
 *  wqe:		Work queue entry to insert in the timer.
 */
static inline void cvmx_tim_wqe_store(volatile uint64_t *ptr, cvmx_wqe_t *wqe)
{
	if(wqe == NULL)
		*ptr = 0ULL;
	else if (OCTEON_IS_MODEL(OCTEON_CN78XX) ||
		 OCTEON_IS_MODEL(OCTEON_CN73XX) ||
		 OCTEON_IS_MODEL(OCTEON_CNF75XX)) {
		/* TIM_MEM_S structure in chunk */
		*ptr =
			(uint64_t)cvmx_ptr_to_phys(wqe) |
			(uint64_t)cvmx_wqe_get_xgrp(wqe) << 52 |
			(uint64_t)cvmx_wqe_get_tt(wqe) << 50;
	} else {
		/* pointer to wqe address in timer chunk */
		*ptr = cvmx_ptr_to_phys(wqe);
	}
}

/*
 * _arm_timer:		Perform the work to add a work queue entry to the timer.
 *
 *
 *  returns:		CVMX_TIM_STATUS_SUCCESS, or error.
 */
static int _arm_timer(cvmx_tim_info_t *tim_info, long arg2)
{
	cvmx_tim_t		*cvmx_tim = &tim_info->cvmx_tim;
	cvmx_tim_bucket_entry_t	*work_bucket_ptr;
	u64			cycles;
	u64			core_num;
	u64			work_bucket;
	u64			entries_per_chunk;
	volatile u64		*tim_entry_ptr = NULL;
	cvmx_tim_delete_t	tim_delete;

	cycles = cvmx_clock_get_count(CVMX_CLOCK_TIM);
	core_num = cvmx_get_core_num();

	if (unlikely(tim_info->ticks_from_now > cvmx_tim->max_ticks)) {
		pr_err("%s() Tried to schedule work too far away\n",
		       __func__);
		return CVMX_TIM_STATUS_TOO_FAR_AWAY;
	}

	/* Since we have no way to synchronize, we can't update a timer that is
	 * being used by the hardware. Two buckets forward should be safe.
	 */
	if (tim_info->ticks_from_now < 2) {
		pr_warn("%s() Tried to schedule work too soon. Delaying it\n",
			__func__);
		tim_info->ticks_from_now = 2;
	}

	/* Get the bucket this work queue entry should be in. Remember the bucket
	   array is circular */
	if (OCTEON_IS_MODEL(OCTEON_CN78XX) || OCTEON_IS_MODEL(OCTEON_CN73XX) ||
	    OCTEON_IS_MODEL(OCTEON_CNF75XX)) {
		cvmx_tim_ringx_ctl1_t ring_ctl1;
		unsigned node, ring_id;

		ring_id = core_num;
		node = cvmx_get_node_num();

		ring_ctl1.u64 =
			cvmx_read_csr_node(node, CVMX_TIM_RINGX_CTL1(ring_id));

		work_bucket = ring_ctl1.cn78xx.bucket;
		work_bucket += tim_info->ticks_from_now;
	} else {
		work_bucket = 
			(((tim_info->ticks_from_now * cvmx_tim->tick_cycles) + 
			  cycles - cvmx_tim->start_time) >> 
			 cvmx_tim->bucket_shift);
	}

	/* num_buckets is always a power of 2, use binary & instead of % for
	 * optimization reasons.
	 */
	work_bucket_ptr = (cvmx_tim_bucket_entry_t *)cvmx_tim->bucket + 
		core_num * cvmx_tim->num_buckets +
		(work_bucket & (cvmx_tim->num_buckets - 1));
	entries_per_chunk = (tim_info->timer_pool_size/8 - 1);

	/* Check if we have room to add this entry into the existing list */
	if (likely(work_bucket_ptr->chunk_remainder)) {
		unsigned ent = entries_per_chunk -
			work_bucket_ptr->chunk_remainder;
		tim_entry_ptr = &(work_bucket_ptr->last_chunk->entries[ent]);
		/* Adding the work entry to the end of the existing list */
		work_bucket_ptr->chunk_remainder--;
		work_bucket_ptr->num_entries++;
		cvmx_tim_wqe_store(tim_entry_ptr,
				   (cvmx_wqe_t *)phys_to_virt(tim_info->wqe));
	} else {
		/* Current bucket is either completely empty or completely full.
		 * We need to allocate a new chunk for storing this work entry.
		 */
		cvmx_tim_entry_chunk_t *new_chunk =
			(cvmx_tim_entry_chunk_t *) cvmx_fpa_alloc(tim_info->timer_pool);
		if (unlikely(new_chunk == NULL)) {
			pr_err("%s() Failed to allocate memory for new chunk\n",
			       __func__);
			return CVMX_TIM_STATUS_NO_MEMORY;
		}

		/* Does a chunk currently exist? We have to check num_entries
		 * since the hardware doesn't NULL out the chunk pointers on
		 * free.
		 */
		if (work_bucket_ptr->num_entries) {
			/* This chunk must be appended to an existing list by
			 * putting its address in the last spot of the existing
			 * chunk.
			 */
			work_bucket_ptr->last_chunk->entries[entries_per_chunk] = 
				cvmx_ptr_to_phys(new_chunk);
			work_bucket_ptr->num_entries++;
		} else {
			/* This is the very first chunk. Add it */
			work_bucket_ptr->first_chunk_addr = cvmx_ptr_to_phys(new_chunk);
			work_bucket_ptr->num_entries = 1;
		}
		work_bucket_ptr->last_chunk = new_chunk;
		work_bucket_ptr->chunk_remainder = entries_per_chunk - 1;
		tim_entry_ptr =&(new_chunk->entries[0]);
		cvmx_tim_wqe_store(tim_entry_ptr,
				   (cvmx_wqe_t *)phys_to_virt(tim_info->wqe));
	}

	/* If the user supplied a delete info structure then fill it in */
	if (arg2) {
		/* It would be very bad to delete a timer entry after, or during
		 * the timer's processing. During the processing could yield
		 * unpredicatable results, but after would always be bad.
		 * Modifying the entry after processing means we would be
		 * changing data in a buffer that has been freed, and possible
		 * allocated again. For this reason we store a commit cycle
		 * count in the delete structure. If we are after this count we
		 * will refuse to delete the timer entry.
		 */
		tim_delete.commit_cycles = cycles +
			(tim_info->ticks_from_now - 2) * cvmx_tim->tick_cycles;
		tim_delete.timer_entry_ptr = (u64 *)tim_entry_ptr;

		if (copy_to_user((cvmx_tim_delete_t __user *)arg2, &tim_delete,
				 sizeof(tim_delete))) {
			*tim_entry_ptr = 0;
			return -EFAULT;
		}
	}

	CVMX_SYNCW;

	return CVMX_TIM_STATUS_SUCCESS;
}

/*
 * disarm_timer:	Delete a work queue entry from a timer.
 *
 *  arg1:		Pointer to the cvmx_tim_delete_t structure, obtained via
 *			arm_timer(), for the wqe to delete.
 *  arg2:		Not used.
 *
 *  returns:		CVMX_TIM_STATUS_SUCCESS, or error.
 */
int disarm_timer(long arg1, long arg2)
{
	unsigned long		flags;
	cvmx_tim_delete_t	tim_delete;
	const u64		cycles = cvmx_clock_get_count(CVMX_CLOCK_TIM);
	int			rc;

	if (copy_from_user(&tim_delete, (cvmx_tim_delete_t *)arg1,
			   sizeof(tim_delete)))
		return -EFAULT;

	local_irq_save(flags);

	if (likely((s64)(cycles - tim_delete.commit_cycles) < 0)) {
		/* Timer is far enough away. Safe to delete */
		*tim_delete.timer_entry_ptr = 0;
		rc = CVMX_TIM_STATUS_SUCCESS;
	} else {
		/* Timer is passed the commit time. It cannot be stopped */
		rc = CVMX_TIM_STATUS_BUSY;
	}

	local_irq_restore(flags);

	return rc;
}

/*
 * arm_timer:		Adds a work queue entry to a timer.
 *
 *  arg1:		Pointer to cvmx_tim_info_t structure containing all
 *			information needed to add the wqe.
 *  arg2:		Pointer to a cvmx_tim_delete_t structure. Updated with
 *			information needed to delete the timer at a later time.
 *
 *  returns:		CVMX_TIM_STATUS_SUCCESS, or error.
 */
int arm_timer(long arg1, long arg2)
{
	unsigned long	flags;
	cvmx_tim_info_t	tim_info;
	int		rc;

	if (copy_from_user(&tim_info, (cvmx_tim_info_t *)arg1,
			   sizeof(tim_info)))
		return -EFAULT;

	local_irq_save(flags);
	rc = _arm_timer(&tim_info, arg2);
	local_irq_restore(flags);

	return rc;
}

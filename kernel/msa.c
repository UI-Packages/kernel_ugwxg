/*
 * Microstate accounting.
 * Try to account for various states much more accurately than
 * the normal code does.
 *
 * Copyright (c) Peter Chubb 2005
 *  UNSW and National ICT Australia
 * Copyright (c) 2010 MontaVista Software, LLC
 *  Corey Minyard <minyard@mvista.com>, <minyard@acm.org>, <source@mvista.com>
 * This code is released under the Gnu Public Licence, version 2.
 */


#include <linux/types.h>
#include <linux/errno.h>
#include <linux/linkage.h>
#include <linux/msa.h>
#include <linux/syscalls.h>
#ifdef CONFIG_MICROSTATE_ACCT
#include <linux/irq.h>
#include <linux/hardirq.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/acct.h>
#include <linux/tsacct_kern.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/init.h>
#include <linux/bug.h>
#include <linux/slab.h>

#include <asm/uaccess.h>

/*
 * Dummy this out for the moment.
 */
void account_process_tick(struct task_struct *p, int user_tick)
{
}

/**
 * fetch_msa_time: Get a sane time value.  The checking and workaround
 * code for timesource problems.
 * @msa1: The first microstates structure, must be supplied.
 * @msa2: The seconds microstates structure, may be NULL.
 *
 * Fetch a sane value for a task's microstate accounting value.  With
 * all the detection and workaround code turned off, this will just return
 * MSA_NOW().  If the detection code is turned on, it will report if time
 * goes backwards via a kernel warning.  If workaround code is enabled,
 * it will make sure that the returned value is >= the last_change value
 * for msp1 and msp2 (if msp2 is provided).
 *
 * Note that this is not required for interrupts, since preemption doesn't
 * happen in interrupts.
 */
msa_time_t fetch_msa_time(struct microstates *msp1, struct microstates *msp2)
{
	msa_time_t now;
#ifdef CONFIG_MICROSTATE_ACCT_TIMEPROB_DETECT
	int cpu = smp_processor_id();
#endif

	MSA_NOW(now);
#if defined(CONFIG_MICROSTATE_ACCT_TIMEPROB_DETECT) || \
		defined(CONFIG_MICROSTATE_ACCT_TIMEPROB_WORKAROUND)
	if (now < msp1->last_change) {
#ifdef CONFIG_MICROSTATE_ACCT_TIMEPROB_DETECT
		printk(KERN_WARNING "msa timesource problem: "
		       "last_change = %llu, now = %llu\n"
		       "last_cpu = %d, curr_cpu = %d\n",
		       (unsigned long long) msp1->last_change,
		       (unsigned long long) now,
		       msp1->last_change_cpu, cpu);
#endif
#ifdef CONFIG_MICROSTATE_ACCT_TIMEPROB_WORKAROUND
		now = msp1->last_change;
#endif
	}
	if (msp2 && now < msp2->last_change) {
#ifdef CONFIG_MICROSTATE_ACCT_TIMEPROB_DETECT
		printk(KERN_WARNING "msa timesource problem: "
		       "last_change = %llu, now = %llu\n"
		       "last_cpu = %d, curr_cpu = %d\n",
		       (unsigned long long) msp2->last_change,
		       (unsigned long long) now,
		       msp2->last_change_cpu, cpu);
#endif
#ifdef CONFIG_MICROSTATE_ACCT_TIMEPROB_WORKAROUND
		now = msp2->last_change;
#endif
	}
#ifdef CONFIG_MICROSTATE_ACCT_TIMEPROB_DETECT
	msp1->last_change_cpu = cpu;
	if (msp2)
		msp2->last_change_cpu = cpu;
#endif
#endif
	return now;
}

/**
 * msa_switch: Update microstate timers when switching from one task to
 * another.
 *
 * @prev, @next:  The prev task is coming off the processor;
 *                the new task is about to run on the processor.
 *
 * Update the times in both prev and next.  It may be necessary to infer the
 * next state for each task.
 *
 */
void msa_switch(struct task_struct *prev, struct task_struct *next)
{
	struct microstates *prev_msp = &prev->microstates;
	struct microstates *next_msp = &next->microstates;
	msa_time_t now;
	enum msa_thread_state next_state;
	unsigned long flags;

	local_irq_save(flags);

	now = fetch_msa_time(next_msp, prev_msp);

	next_msp->timers[next_msp->cur_state] += now - next_msp->last_change;
	prev_msp->timers[prev_msp->cur_state] += now - prev_msp->last_change;
	msa_system_time(prev, msa_to_cputime(now - prev_msp->last_change));

	/*
	 * Update states, state is sort of a bitmask, except that
	 * TASK_RUNNING is 0.
	 */
	if (prev->state == TASK_RUNNING)
		next_state = MSA_ONRUNQUEUE;
	else if (prev->state & TASK_INTERRUPTIBLE)
		next_state = MSA_INTERRUPTIBLE_SLEEP;
	else if (prev->state & TASK_UNINTERRUPTIBLE)
		next_state = MSA_UNINTERRUPTIBLE_SLEEP;
	else if (prev->state & (TASK_STOPPED | TASK_TRACED))
		next_state = MSA_STOPPED;
	else if (prev->state & (TASK_DEAD | EXIT_DEAD | EXIT_ZOMBIE))
		next_state = MSA_ZOMBIE;
	else if (prev->state & TASK_PARKED)
		next_state = MSA_PARKED;
	else {
		printk(KERN_WARNING "msa: Setting UNKNOWN state from %ld\n",
		       prev->state);
		WARN_ON(1);
		next_state = MSA_UNKNOWN;
	}

	/* special states */
	switch (prev_msp->next_state) {
	case MSA_PAGING_SLEEP:
	case MSA_FUTEX_SLEEP:
	case MSA_POLL_SLEEP:
		if (prev->state & TASK_INTERRUPTIBLE ||
		    prev->state & TASK_UNINTERRUPTIBLE)
			next_state = prev_msp->next_state;
	default:
		break;
	}

	prev_msp->next_state = prev_msp->cur_state;
	prev_msp->cur_state = next_state;
	prev_msp->last_change = now;

	next_msp->last_change = now;

	WARN_ON(next_msp->next_state == MSA_UNKNOWN);
	next_msp->cur_state = next_msp->next_state;
	if (next_msp->cur_state != MSA_ONCPU_USER)
		next_msp->cur_state = MSA_ONCPU_SYS;
	next_msp->next_state = MSA_UNKNOWN;

	local_irq_restore(flags);
}

/**
 * msa_init:  Initialise the struct microstates in a new task
 * @p: pointer to the struct task_struct to be initialised
 *
 * This function is called from copy_process().
 * It initialises the microstate timers to zero, and sets the
 * current state to MSA_UNINTERRUPTIBLE_SLEEP.
 */
void msa_init(struct task_struct *p)
{
	struct microstates *msp = &p->microstates;

	memset(msp, 0, sizeof *msp);
	preempt_disable();
	MSA_NOW(msp->last_change);
#ifdef CONFIG_MICROSTATE_ACCT_TIMEPROB_DETECT
	msp->last_change_cpu = -smp_processor_id() - 1;
#endif
	preempt_enable();
	msp->cur_state = MSA_UNINTERRUPTIBLE_SLEEP;
	msp->next_state = MSA_ONCPU_SYS;
}

/**
 * __msa_set_timer: Helper function to update microstate times.
 * &msp:  Pointer to the struct microstates to update
 * next_state: the state being changed to.
 *
 * The time spent in the current state is updated, and the time of
 * last state change set to MSA_NOW().  Then the current state is updated
 * to next_state.
 */
static inline msa_time_t __msa_set_timer(struct microstates *msp, int next_state)
{
	unsigned long flags;
	msa_time_t now, delta;

	local_irq_save(flags);
	now = fetch_msa_time(msp, NULL);
	delta = now - msp->last_change;
	msp->timers[msp->cur_state] += delta;
	msp->last_change = now;
	msp->cur_state = next_state;
	local_irq_restore(flags);

	return delta;
}

/**
 * __msa_set_timer_onswitch: same as __msa_set_timer, but it do timing
 * stuff only when state changes.
 */
static inline msa_time_t __msa_set_timer_onswitch(struct microstates *msp,
						  int next_state)
{
	unsigned long flags;
	msa_time_t now, delta;

	local_irq_save(flags);
	delta = 0;
	if (msp->cur_state == next_state)
		goto out;
	now = fetch_msa_time(msp, NULL);
	delta = now - msp->last_change;
	msp->timers[msp->cur_state] += delta;
	msp->last_change = now;
	msp->cur_state = next_state;
out:
	local_irq_restore(flags);
	return delta;
}

/**
 * msa_set_timer:  Time stamp an explicit state change.
 * @p: pointer to the task that has just changed state.
 * @next_state: the state being changed to.
 *
 * This function is called, e.g., from __activate_task(), when an
 * immediate state change happens.
 */
void msa_set_timer(struct task_struct *p, int next_state)
{
	struct microstates *msp = &p->microstates;
	msa_time_t delta;

	delta = __msa_set_timer(msp, MSA_ONCPU_SYS);
	msa_user_time(p, msa_to_cputime(delta));
}

/*
 * Helper routines, to be called from assembly language stubs
 */

/**
 * msa_kernel: change state to MSA_ONCPU_SYS.
 *
 * Should be called upon every entry in the kernel from user space.
 */
asmlinkage void msa_kernel(void)
{
	struct task_struct *p = current;
	struct microstates *msp = &p->microstates;
	msa_time_t delta;

	delta = __msa_set_timer_onswitch(msp, MSA_ONCPU_SYS);
	if (delta)
		msa_user_time(p, msa_to_cputime(delta));
}

/**
 * msa_user: change state out of MSA_ONCPU_SYS
 *
 * Called when about to leave the kernel.
 */
asmlinkage void msa_user(void)
{
	struct task_struct *p = current;
	struct microstates *msp = &p->microstates;
	msa_time_t delta;

	delta = __msa_set_timer_onswitch(msp, MSA_ONCPU_USER);
	if (delta)
		msa_system_time(p, msa_to_cputime(delta));
}


/*
 * Track time spend in interrupt handlers.
 */
struct msa_irq {
	msa_time_t times;
	msa_time_t last_entered;
#ifdef CONFIG_SPARSE_IRQ
	int irq;
	struct msa_irq *next;
#endif
};

#ifdef CONFIG_SPARSE_IRQ
#define MSA_IRQ_TBLSIZE 16
static DEFINE_PER_CPU(struct msa_irq *[MSA_IRQ_TBLSIZE], msa_irq);
#define MSA_IRQ_STORE_SIZE (NR_IRQS * NR_CPUS)
static struct msa_irq init_msa_irq_store[MSA_IRQ_STORE_SIZE];
static struct msa_irq *msa_irq_free;
DEFINE_RAW_SPINLOCK(msa_irq_free_lock);
static int msa_nr_irqs = NR_IRQS;
#define MSA_NR_IRQS msa_nr_irqs

static struct msa_irq *alloc_msa_irq(void)
{
	struct msa_irq *o;

	raw_spin_lock(&msa_irq_free_lock);
	o = msa_irq_free;
	BUG_ON(!o);
	msa_irq_free = o->next;
	raw_spin_unlock(&msa_irq_free_lock);
	return o;
}

int expand_msa_irqs(int new_nr_irqs)
{
	struct msa_irq *o;
	int i;
	unsigned long flags;

	for (i = msa_nr_irqs; i < new_nr_irqs; i++) {
		o = kzalloc(sizeof(*o), GFP_ATOMIC);
		if (!o)
			return -ENOMEM;
		raw_spin_lock_irqsave(&msa_irq_free_lock, flags);
		o->next = msa_irq_free;
		msa_irq_free = o;
		raw_spin_unlock_irqrestore(&msa_irq_free_lock, flags);
	}
	msa_nr_irqs = new_nr_irqs;
	return 0;
}

int __init init_msa_irq(void)
{
	struct msa_irq *o;
	int i;

	for (i = 0; i < MSA_IRQ_STORE_SIZE; i++) {
		o = &init_msa_irq_store[i];
		o->next = msa_irq_free;
		msa_irq_free = o;
	}
	return 0;
}

#else
#define MSA_IRQ_TBLSIZE (NR_IRQS + 1)
static DEFINE_PER_CPU(struct msa_irq[MSA_IRQ_TBLSIZE], msa_irq);
#define MSA_NR_IRQS NR_IRQS
#endif

static struct msa_irq *get_msa_irq_cpu(int irq, int cpu)
{
	int idx;
	struct msa_irq *miq;

#ifdef CONFIG_SPARSE_IRQ
	idx = irq % MSA_IRQ_TBLSIZE;
	miq = per_cpu(msa_irq, cpu)[idx];
	while (miq) {
		if (miq->irq == irq)
			break;
		miq = miq->next;
	}
#else
	idx = irq;
	miq = &per_cpu(msa_irq, cpu)[idx];
#endif
	return miq;
}

static struct msa_irq *get_msa_irq(int irq, bool alloc)
{
	int idx;
	struct msa_irq *miq;

	BUG_ON(irq >= MSA_NR_IRQS);
#ifdef CONFIG_SPARSE_IRQ
	idx = irq % MSA_IRQ_TBLSIZE;
	miq = __get_cpu_var(msa_irq)[idx];
	while (miq) {
		if (miq->irq == irq)
			break;
		miq = miq->next;
	}

	if (!miq) {
		BUG_ON(!alloc);
		miq = alloc_msa_irq();
		miq->irq = irq;
		miq->next = __get_cpu_var(msa_irq)[idx];
		__get_cpu_var(msa_irq)[idx] = miq;
	}
#else
	idx = irq;
	miq = &__get_cpu_var(msa_irq)[idx];
#endif
	return miq;
}

static inline void _msa_start_irq(int irq, int nested)
{
	struct task_struct *p = current;
	struct microstates *msp = &p->microstates;
	msa_time_t now;
	struct msa_irq *miq;

	/* we're in an interrupt handler... no possibility of preemption */
	now = fetch_msa_time(msp, NULL);

	miq = get_msa_irq(irq, true);
	BUG_ON(!miq);
	miq->last_entered = now;

	if (!nested) {
		msa_time_t delta = now - msp->last_change;
		msp->timers[msp->cur_state] += delta;
		msp->last_change = now;
		if (msp->cur_state == MSA_ONCPU_USER)
			msa_user_time(p, msa_to_cputime(delta));
		else
			msa_system_time(p, msa_to_cputime(delta));
		if (msp->cur_state == MSA_ONCPU_USER
				|| msp->cur_state == MSA_ONCPU_SYS) {
			msp->next_state = msp->cur_state;
			msp->cur_state = MSA_INTERRUPTED;
		}
	}
}

/*
 * msa_start_irq: mark the start of an interrupt handler.
 * @irq: irq number being handled.
 *
 * Update the current task state to MSA_INTERRUPTED, and start
 * accumulating time to the interrupt handler for irq.
 */
void msa_start_irq(int irq)
{
	int nested;

	/* we're in an interrupt handler... no possibility of preemption */
	nested = hardirq_count() - HARDIRQ_OFFSET;
	BUG_ON(nested < 0);
	_msa_start_irq(irq, nested);
}

/*
 * Same as msa_start_irq() except it's called from irq handler that
 * don't call irq_enter/irq_exit.
 */
void msa_start_irq_raw(int irq)
{
	int nested;

	/* we're in an interrupt handler... no possibility of preemption */
	nested = hardirq_count();
	_msa_start_irq(irq, nested);
}

/**
 * msa_continue_irq: While remaining in MSA_INTERRUPTED state, switch
 * to a new IRQ.
 *
 * @oldirq: the irq that was just serviced
 * @newirq: the irq that is about to be serviced.
 *
 * Architectures such as IA64 can handle more than one interrupt
 * without allowing the interrupted process to continue.  This function
 * is called when switching to a new interrupt.
 */
void msa_continue_irq(int oldirq_id, int newirq_id)
{
	msa_time_t now;
	struct msa_irq *old_miq, *new_miq;

	MSA_NOW(now);
	/* we're in an interrupt handler... no possibility of preemption */
	BUG_ON(!in_interrupt());

	old_miq = get_msa_irq(oldirq_id, false);
	new_miq = get_msa_irq(newirq_id, true);

	BUG_ON(!old_miq);
	BUG_ON(!new_miq);
	old_miq->times +=  now - old_miq->last_entered;
	new_miq->last_entered = now;
}

/**
 * msa_finish_irq: end processing for an interrupt.
 * @irq: the interrupt that was just serviced.
 *
 * Update the time spent handling irq, then update the current task's
 * state to MSA_ONCPU_USER or MSA_ONCPU_SYS.
 *
 * This MUST be called instead of irq_exit() whenever msa_start_irq()
 * was called for a given irq.  irq_exit() is implied by this function.
 *
 * See the notes in msa_start_irq() for info about irq_id
 */
void msa_irq_exit(int irq_id, int is_going_to_user)
{
	struct task_struct *p = current;
	struct microstates *msp = &p->microstates;
	msa_time_t now, delta;
	struct msa_irq *miq;
	int nested;

	miq = get_msa_irq(irq_id, false);
	BUG_ON(!miq);
	nested = hardirq_count() - HARDIRQ_OFFSET;
	BUG_ON(nested < 0);

	MSA_NOW(now);
	if (now < miq->last_entered)
		delta = 0;
	else
		delta = now - miq->last_entered;
	miq->times += delta;

	irq_exit();

	if (!nested) {
		msa_time_t before = now;
		u64 *cpustat = kcpustat_this_cpu->cpustat;

		cpustat[CPUTIME_IRQ] += msa_to_cputime64(delta);

		now = fetch_msa_time(msp, NULL);
		if (now < before)
			delta = 0;
		else
			delta = now - before;
		cpustat[CPUTIME_SOFTIRQ] += msa_to_cputime64(delta);
		msp->timers[msp->cur_state] += now - msp->last_change;
		msp->last_change = now;
		if (is_going_to_user)
			msp->cur_state = MSA_ONCPU_USER;
		else
			msp->cur_state = MSA_ONCPU_SYS;
	}
}

/*
 * Same as msa_irq_exit() except it's called from irq handler that
 * don't call irq_enter/irq_exit. So don't call irq_exit() here and
 * don't account softirqs time.
 */
void msa_irq_exit_raw(int irq_id)
{
	struct task_struct *p = current;
	struct microstates *mp = &p->microstates;
	msa_time_t now, delta;
	struct msa_irq *miq;
	int nested;

	miq = get_msa_irq(irq_id, false);
	BUG_ON(!miq);
	nested = hardirq_count();
	BUG_ON(nested < 0);

	MSA_NOW(now);
	if (now < miq->last_entered)
		delta = 0;
	else
		delta = now - miq->last_entered;

	if (!nested) {
		u64 *cpustat = kcpustat_this_cpu->cpustat;

		cpustat[CPUTIME_IRQ] += msa_to_cputime64(delta);
		if (mp->cur_state == MSA_INTERRUPTED) {
			mp->timers[mp->cur_state] += now - mp->last_change;
			mp->last_change = now;
			mp->cur_state = mp->next_state;
		}
	}
}

/**
 * msa_update_parent:  Accumulate child times into parent, after zombie is over.
 * @parent: pointer to parent task
 * @this: pointer to task that is now a zombie
 *
 * Called from release_task(). (Note: it may be better to call this
 * from wait_zombie())
 */
void msa_update_parent(struct task_struct *parent, struct task_struct *this)
{
	enum msa_thread_state s;
	msa_time_t *pmsp = parent->microstates.child_timers;
	struct microstates *msp = &this->microstates;
	msa_time_t *msc = msp->timers;
	msa_time_t *msgc = msp->child_timers;

	/*
	 * State could be MSA_ZOMBIE (if parent is interested)
	 * or something else (if the parent isn't interested)
	 */
	__msa_set_timer(msp, msp->cur_state);

	for (s = 0; s < MSA_NR_STATES; s++)
		*pmsp++ += *msc++ + *msgc++;
}

/**
 * sys_msa: get microstate data for self or waited-for children.
 * @ntimers: the number of timers requested
 * @which: which set of timers is wanted.
 * @timers: pointer in user space to an array of timers.
 *
 * 'which' can take the values
 *   MSA_THREAD: return times for current thread only
 *   MSA_SELF:  return times for current process,
 *		summing over all live threads
 *   MSA_CHILDREN: return sum of times for all dead children.
 *   MSA_GET_NOW: return the current msa timer value in the first value.
 *
 * The timers are ordered so that the most interesting ones are first.
 * Thus a user program can ask for only the  few most interesting ones
 * if it wishes.  Also, we can add more in the kernel as we need
 * to without invalidating user code.
 */
SYSCALL_DEFINE3(msa, int, ntimers, int, which, msa_time_t __user *, timers)
{
	msa_time_t *tp;
	int i;
	struct microstates *msp = &current->microstates;
	struct microstates out;

	WARN_ON(current->microstates.cur_state != MSA_ONCPU_SYS);

	if (ntimers <= 0 || ntimers > MSA_NR_STATES)
		return -EINVAL;

	switch (which) {
	case MSA_SELF:
	case MSA_THREAD:
		preempt_disable();
		__msa_set_timer(msp, MSA_ONCPU_SYS);

		if (which == MSA_SELF) {
			struct task_struct *task;
			struct task_struct *leader_task;
			msa_time_t *tp1;

			memset(out.timers, 0, sizeof(out.timers));
			read_lock(&tasklist_lock);
			leader_task = task = current->group_leader;
			do {
				tp = task->microstates.timers;
				tp1 = out.timers;
				for (i = 0; i < ntimers; i++)
					*tp1++ += *tp++;
			} while ((task = next_thread(task)) != leader_task);
			read_unlock(&tasklist_lock);
			tp = out.timers;
		} else
			tp = msp->timers;
		preempt_enable();
		break;

	case MSA_CHILDREN:
		tp =  msp->child_timers;
		break;

	case MSA_GET_NOW:
		ntimers = 1;
		tp = out.timers;
		MSA_NOW(*tp);
		break;

	default:
		return -EINVAL;
	}

	for (i = 0; i < ntimers; i++) {
		__u64 x = MSA_TO_NSEC(*tp++);
		if (copy_to_user(timers++, &x, sizeof x))
			return -EFAULT;
	}

	return 0;
}

#ifdef CONFIG_PROC_FS

/*
 * Display the total number of nanoseconds since boot spent
 * in handling each interrupt on each processor.
 */

static void *msa_irq_time_seq_start(struct seq_file *f, loff_t *pos)
{
	return (*pos <= MSA_NR_IRQS) ? pos : NULL;
}

static void *msa_irq_time_seq_next(struct seq_file *f, void *v, loff_t *pos)
{
	(*pos)++;
	if (*pos > MSA_NR_IRQS)
		return NULL;
	return pos;
}

static void msa_irq_time_seq_stop(struct seq_file *f, void *v)
{
	/* Nothing to do */
}

static int msa_irq_time_seq_show(struct seq_file *f, void *v)
{
	int i = *(loff_t *) v, cpu;
	struct msa_irq *miq;

	if (i == 0) {
		msa_time_t now;
		char cpuname[10];
		MSA_NOW(now);
		seq_printf(f, "Now: %15llu\n", now);
		seq_printf(f, "     ");
		for_each_present_cpu(cpu) {
			sprintf(cpuname, "CPU%d", cpu);
			seq_printf(f, " %15s", cpuname);
		}
		seq_putc(f, '\n');
	}

	if (i < MSA_NR_IRQS) {
		bool all_zeros = true;

		for_each_present_cpu(cpu) {
			miq = get_msa_irq_cpu(i, cpu);
			if (miq && miq->times) {
				all_zeros = false;
				break;
			}
		}
		if (all_zeros)
			return 0;
		seq_printf(f, "%3d: ", i);
		for_each_present_cpu(cpu) {
			msa_time_t x = 0;

			miq = get_msa_irq_cpu(i, cpu);
			if (miq && miq->times)
				x = MSA_TO_NSEC(miq->times);
			seq_printf(f, " %15llu", (unsigned long long) x);
		}
		seq_putc(f, '\n');
	}

	return 0;
}

static struct seq_operations msa_irq_time_seq_ops = {
	.start	= msa_irq_time_seq_start,
	.next	= msa_irq_time_seq_next,
	.stop	= msa_irq_time_seq_stop,
	.show	= msa_irq_time_seq_show,
};

static int msa_irq_time_open(struct inode *inode, struct file *filp)
{
	return seq_open(filp, &msa_irq_time_seq_ops);
}

static struct file_operations proc_msa_irq_time_ops = {
	.open		= msa_irq_time_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};

static int __init proc_msa_irq_time_iinit(void)
{
	struct proc_dir_entry *file;
	file = proc_create_data("msa_irq_time", 0, NULL,
				&proc_msa_irq_time_ops, NULL);
	if (!file)
		return -ENOMEM;
	return 0;
}

fs_initcall(proc_msa_irq_time_iinit);
#endif

#else
/*
 * Stub for sys_msa when CONFIG_MICROSTATE_ACCT is off.
 */
SYSCALL_DEFINE3(msa, int, ntimers, int, which, msa_time_t __user *, timers)
{
	return -ENOSYS;
}
#endif /* CONFIG_MICROSTATE_ACCT */

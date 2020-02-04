#ifdef CONFIG_DEBUG_KERNEL
# define DEBUG
#endif
#define pr_fmt(f) "uncore: " f
/*
 * Copyright (C) 2013 Cavium Networks
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * Inspired by the AMD uncore handling
 */

#include <linux/perf_event.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/module.h>
#include <linux/perf_event.h>
#include <linux/stringify.h>
#include <linux/printk.h>

#include <asm/cpu-features.h>
#include <asm/octeon/octeon.h>

/*
 * perf_uncore.debug=1 enables tracing here and cvmx-*.h
 */
static bool debug;
module_param(debug, bool, 0644);

static int unc_traces = 100; /* lines to log */
module_param(unc_traces, int, 0644);

static void safe_printk(const char *fmt, ...);
#undef pr_debug
#define pr_debug(fmt, ...) safe_printk(fmt, __VA_ARGS__)

#include <asm/octeon/cvmx-core.h>
#include <asm/octeon/cvmx-l2c.h>
#include <asm/octeon/cvmx-l2c-defs.h>
#include <asm/octeon/cvmx-lmcx-defs.h>

/* see also:
 * arch/mips/cavium-octeon/executive/cvmx-l2c.c
 *
 * sdk/bootloader/u-boot/arch/mips/cpu/octeon/octeon3_lmc.h
 * sdk/examples/lmc-dump/lmc-dump.c
 *
 * These are the un-core counters, those not associated with a cpu core.
 * (for per-core counters, see arch/mips/cavium-octeon/perf_counters.c)
 *
 * We have a mix of fixed-function counters (which count (if enabled)
 * a pre-determined event type), and variable-function counters which
 * are each configured to count a dynamically-selectable event class.
 *
 * A Mem-controller (LMC) has 3 fixed-function counters.
 * A Level-2-cache-controller has:
 *  4 variable-function counters for each tag-and-data (TAD) unit
 *  7 fixed-function counters (XMC/XMD/RSC/RSD/INV/IOC/IOR)
 *  We program each TAD's counters identically & sum their counts.
 */

#ifndef CONFIG_EARLY_PRINTK
/* if not supported, just stub out ... */
static void early_vprintk(const char *fmt, va_list ap) {}
#endif

/* use early_printk() for safe visibility in perf irq context ... */
static void safe_printk(const char *fmt, ...)
{
	va_list ap;

	if (!debug || unc_traces-- < 0)
		return;
	va_start(ap, fmt);
	if (irqs_disabled())
		early_vprintk(fmt, ap);
	else
		vprintk(fmt, ap);
	va_end(ap);
}

/*
 * Many of the fixed-function perf counters have multiple
 * instances, depending on chip-type and runtime config.
 * These 4 lim_xxx() functions cover all varieties ...
 */

/* register count for O23 L2C_IO[CR]X_PFC */
static int lim_one(void) { return 1; }

/* register count for CVMX_LMCX_DCLK/_OPS/_IFB */
static int lim_cvmx_lmcx(void)
{
	int lim = 0;
	int tad;
	int possible;

	/*
	 * Prepare to walk over all _enabled_ LMC banks:
	 * - o1/o1p have _exactly_ one, with no quad_dll_ena bit;
	 * - o2/o3 have 1/2/4 LMCs defined, but upper LMCs may be disabled.
	 * LMC banks will never be sparse (eg: 0+2 enabled, 1+3 disabled)
	 */
	if (OCTEON_IS_OCTEON1PLUS())
		return 1;
	possible = 1;
	if (OCTEON_IS_MODEL(OCTEON_CN73XX) || OCTEON_IS_MODEL(OCTEON_CNF75XX))
		possible = 2;
	else if (OCTEON_IS_MODEL(OCTEON_CN68XX) || OCTEON_IS_MODEL(OCTEON_CN78XX))
		possible = 4;
	for (tad = 0; tad < possible; tad++) {
		union cvmx_lmcx_dll_ctl2 ctl2;
		ctl2.u64 = cvmx_read_csr(CVMX_LMCX_DLL_CTL2(tad));
		if (current_cpu_type() == CPU_CAVIUM_OCTEON3) {
			if (ctl2.cn70xx.quad_dll_ena == 0)
				continue;
		} else if (ctl2.cn63xx.quad_dll_ena == 0)
			continue;
		lim++;
	}
	return lim;
}

/* register count for O23 L2C_XM[CD]X_PFC _RS[CD]X_PFC */
static int lim_o23_xmcd_rsdc(void)
{
	switch (cvmx_get_octeon_family()) {
	case OCTEON_CN68XX & OCTEON_FAMILY_MASK:
		return 4;
	case OCTEON_CN78XX & OCTEON_FAMILY_MASK:
		return 10;
	}
	return 1;
}

/* register count for O3-only L2C_INVX_PFC */
static int lim_o3_l2c_inv(void)
{
	switch (cvmx_get_octeon_family()) {
	case OCTEON_CN70XX & OCTEON_FAMILY_MASK:
		return 1;
	case OCTEON_CN78XX & OCTEON_FAMILY_MASK:
		return 8;
	}
	return 0;
}

/* find our extended state from a generic perf_event ... */
#define e_unc(e) (*(struct oct_uncore **)&(e)->hw.event_base)
#define e_uev(e) (&(e_unc(e)->uevent0[(e)->attr.config]))

/* set up unique enum ids within each scope, to tie to switch() in _init() */
#undef EV
#define EV(_family, _name, ...) EVID(_family, _name),
#include "perf_uncore_events.h"
enum { LMC_EVENTS };
enum { O1P_EVENTS };
enum { O23_EVENTS };
enum { TAD_EVENTS };

enum unc_flags { /* event attributes passed in EV() _flags */
	UNC_TYPES	= 0x1f, /* mask of PMU types */
	UNC_LMC		= 0x1,
	UNC_O1P		= 0x2,
	UNC_O2		= 0x4,
	UNC_O3		= 0x8,
	UNC_O23		= (UNC_O2|UNC_O3),
	UNC_ANY		= (UNC_O1P|UNC_O23),
	UNC_TAD		= 0x10,
	UNC_DIRECT	= 0, /* fixed counter/event mapping */
	UNC_MAPPED	= 0x80, /* selectable counter/event mapping */
};

#define NUM_COUNTERS_L2C	CVMX_L2C_MAX_PCNT /* 4 o1p or tad ctrs*/

/*
 * struct uncore_event_desc - a per-event-type object, statically allocated
 *
 * Why not use perf_event.hw.event_base_rdpmc for register addresses?
 * Pre-computed regs make _read faster, but the usual event_base_rdpmc
 * is 'int', and for Octeon1 we need 2 64bit counters addrs (_CNT_LO &
 * _CNT_HI), so we could choose 2 ulong objects (event_base & extra_reg.config)
 * to hold our counter-addresses, and squeeze .stride & .counters into other
 * standard members.
 * But it gets ugly & complicated, so break them out into a custom
 * struct oct_uncore, and just misuse one perf_event element (.hw.event_base)
 * to point to that.  Could be simplified.
 */
struct uncore_event_desc {
#define UEV_MAGIC 74548027
#ifdef UEV_MAGIC
	int magic; /* sanity check */
#endif
	int attno; /* linear attribute#, set in _init scan */
	enum unc_flags flags;
	enum cvmx_l2c_event cvmx_event; /* mapped events */
	struct kobj_attribute attr;
	u16 counters;
	u32 stride;
	u64 reglo;
	u64 reghi;
};

static ssize_t uncore_event_show(struct kobject *kobj,
	struct kobj_attribute *attr, char *buf)
{
	struct uncore_event_desc *uevent =
		container_of(attr, struct uncore_event_desc, attr);
	return sprintf(buf, "event=0x%x", uevent->attno);
}

#undef EV
#define EV(_family, _name, _cvmx, _flags, ...)			\
{								\
	.magic = UEV_MAGIC,					\
	.attno = EVID(_family, _name),				\
	.attr	= __ATTR(_name, 0444, uncore_event_show, NULL),	\
	.flags = _flags,					\
	/* mapped-only */ .cvmx_event = _cvmx,			\
}, /*end*/

#include "perf_uncore_events.h"

static struct uncore_event_desc lmc_events[] = {
	LMC_EVENTS
	{ /* zeros */ },
};
static struct uncore_event_desc o1p_events[] = {
	O1P_EVENTS
	{ /* zeros */ },
};
static struct uncore_event_desc o23_events[] = {
	O23_EVENTS
	{ /* zeros */ },
};
static struct uncore_event_desc tad_events[] = {
	TAD_EVENTS
	{ /* zeros */ },
};

static struct pmu oct_lmc_pmu;
static struct pmu o1p_l2c_pmu;
static struct pmu o23_l2c_pmu;
static struct pmu tad_l2c_pmu;


struct oct_uncore {
	int id;
	int refcnt;
	unsigned num_shared_regs:8;
	unsigned single_fixed:1;
	struct pmu *pmu;
	const struct attribute_group *attr_groups[4];
	int num_counters; /* dimensions uevents[] */
	struct uncore_event_desc *uevent0; /* static order, indexed by evno */
	struct uncore_event_desc *uevents[0]; /* current mapping */
};

#define events_group attr_groups[0]
#define format_group attr_groups[1]
#define pmu_group attr_groups[2]

/* all counters are 64bit, whole word */
PMU_FORMAT_ATTR(event, "config:0-63");
/* sample from just 1 cpu, always cpu0 for simplicity */
PMU_FORMAT_ATTR(cpumask, "0"); /* not a mask, but a list of cpu numbers */

static struct attribute *uncore_format_attrs[] = {
	&format_attr_event.attr,
	NULL,
};


static struct attribute_group uncore_format_group = {
	.name = "format",
	.attrs = uncore_format_attrs,
};

static struct attribute *uncore_pmu_attrs[] = {
	&format_attr_cpumask.attr,
	NULL,
};
static struct attribute_group uncore_pmu_attr_group = {
	.attrs = uncore_pmu_attrs,
};


static struct oct_uncore *oct_uncore_lmc;
static struct oct_uncore *o1p_uncore_l2c;
static struct oct_uncore *o23_uncore_l2c;
static struct oct_uncore *tad_uncore_l2c;
static struct oct_uncore *live_uncores[4+1]; /* list for teardown, simplify? */

static inline bool is_lmc_event(struct perf_event *event)
{
	/* FIXME: speedup by testing e_uev(e)_>flags & UNC_xxx... */
	return event->pmu == &oct_lmc_pmu;
}

static inline bool is_o1p_l2c_event(struct perf_event *event)
{
	return event->pmu == &o1p_l2c_pmu;
}

static inline bool is_o23_l2c_event(struct perf_event *event)
{
	return event->pmu == &o23_l2c_pmu;
}

static inline bool is_tad_l2c_event(struct perf_event *event)
{
	return event->pmu == &tad_l2c_pmu;
}

static inline bool is_mapped_event(struct perf_event *event)
{
	return is_o1p_l2c_event(event) || is_tad_l2c_event(event);
}

static struct oct_uncore *event_to_oct_uncore(struct perf_event *event)
{
	if (!event)
		return NULL;
	else if (e_unc(event))
		return e_unc(event);
	/* these are only for _init, should separate 2 use-cases ... */
	/* FIXME: faster with UNC_xxx flags */
	else if (is_lmc_event(event) && oct_uncore_lmc)
		return oct_uncore_lmc;
	else if (is_o1p_l2c_event(event) && o1p_uncore_l2c)
		return o1p_uncore_l2c;
	else if (is_o23_l2c_event(event) && o23_uncore_l2c)
		return o23_uncore_l2c;
	else if (is_tad_l2c_event(event) && tad_uncore_l2c)
		return tad_uncore_l2c;

	return NULL;
}

static inline
struct uncore_event_desc *event_to_oct_uevent(struct perf_event *event)
{
	struct uncore_event_desc *uev;

	if (!event)
		return NULL;
	uev = e_uev(event);
#ifdef UEV_MAGIC
	BUG_ON(uev->magic != UEV_MAGIC);
#endif
	return uev;
}

/*
 * o1 l2c_pfc(0..3) 36bits, so shift left/right (64-36) to sign-extend diff
 *    or use clear-on-read to avoid need, just dribble in increments
 * o1 lmc_xxx_cnt_lo/hi 32+32, 8 bytes apart (build [0],[8] in)
 * o1p (same)
 *
 * 6x lmc_xxx(0..3)_cnt 64bit
 * 6x l2c_xxx(0..3)_cnt 64bit
 * 6x l2c_tad(0..3)_pfc(0..3) 64bit
 * 0..3 on 68, 0-only on some
 *
 * 71 lmc_xxx(0)_cnt 64bit
 * 71 l2c_xxx(0)_cnt 64bit
 * 71 l2c_tad(0)_pfc(0..3) 64bit
 *
 * 78 lmc_xxx(0..3)_cnt 64bit
 * 78 l2c_xxx(0..9)_cnt 64bit
 * 78 l2c_tad(0..7)_pfc(0..3) 64bit
 */

/* mapped samples sign-extended by <<,>> ? */
static int mapped_sextend_bits;

static void new_sample(struct perf_event *event, u64 new, int shift)
{
	struct hw_perf_event *hwc = &event->hw;
	u64 prev = local64_read(&hwc->prev_count);
	s64 delta;

	local64_set(&hwc->prev_count, new);
	delta = (new << shift) - (prev << shift);
	delta >>= shift;
	local64_add(delta, &event->count);
}

static void unc_direct_read(struct perf_event *event)
{
	struct uncore_event_desc *uev = e_uev(event);
	uint64_t lo = uev->reglo;
	uint64_t hi = uev->reghi;
	u64 new = 0;
	int i;
	int counters = uev->counters;
	int stride = uev->stride;

	/*
	 * since we do not enable counter overflow interrupts,
	 * we do not have to worry about prev_count changing on us
	 */
	for (i = 0; i < counters; i++) {
		u64 inc = 0;
		if (lo)
			inc = cvmx_read_csr(lo+i*stride);
		else
			pr_debug("no cvmx mapping for ev=%llx\n",
				event->attr.config);
		if (hi)
			inc |= (cvmx_read_csr(hi+i*stride) << 32);
		new += inc;
	}

	new_sample(event, new, 0);
	pr_debug("%s direct ev=%d:%d"
		" new %llx, lo %llx, hi %llx, n %d += %d\n",
		__func__, event->pmu->type, event->hw.idx,
		new, lo, hi, counters, stride);
}

static void unc_mapped_read(struct perf_event *event)
{
	u64 new = cvmx_l2c_read_perf(event->hw.idx);

	/*
	 * since we do not enable counter overflow interrupts,
	 * we do not have to worry about prev_count changing on us
	 */
	new_sample(event, new, mapped_sextend_bits);
}

static void oct_uncore_start(struct perf_event *event, int flags)
{
	struct hw_perf_event *hwc = &event->hw;

	if (flags & PERF_EF_RELOAD)
		event->pmu->read(event);
	local64_set(&event->count, 0);

	hwc->state = 0;
}

static void oct_uncore_stop(struct perf_event *event, int flags)
{
	struct hw_perf_event *hwc = &event->hw;

	hwc->state |= PERF_HES_STOPPED;

	if ((flags & PERF_EF_UPDATE) && !(hwc->state & PERF_HES_UPTODATE)) {
		event->pmu->read(event);
		hwc->state |= PERF_HES_UPTODATE;
	}
}

/*
 * oct_uncore_add(event, flags)
 *
 * [maybe split into per-type _add_lmc/_add_tad/_add_o1p/_add_o23 ??
 * no, too much clutter]
 *
 * For PMDs which map N events down to C counters ( C < N )
 * MIPS-core perf counters are like this (but use own code),
 * as is L2C's 4-of-many event structure.
 * But LMC has only 3 fixed-function counters,
 * and L2C also has some fixed-function counters.
 * Leave this general, for later handling uncore mapped-counters from
 * SSO/PKI/PKO/ASE/OCLA-stack/... ???
 * OTOH, these have _mapped_ regs:
 * 4-way map poking via L2C_TADa_PFCb for a in 0..9, b in 0..3
 */
static int unc_mapped_add(struct perf_event *event,
	int flags, struct oct_uncore *uncore)
{
	int i;
	struct uncore_event_desc *uevent = event_to_oct_uevent(event);
	struct hw_perf_event *hwc = &event->hw;

	/* are we already assigned? */
	if (hwc->idx != -1 && uncore->uevents[hwc->idx] == uevent)
		goto out;

	for (i = 0; i < uncore->num_counters; i++) {
		if (uncore->uevents[i] == uevent) {
			hwc->idx = i;
			goto out;
		}
	}

	/* if not, take the first available counter */
	hwc->idx = -1;
	for (i = 0; i < uncore->num_counters; i++) {
		if (uncore->uevents[i] == uevent) {
			hwc->idx = i;
			break;
		} else if (cmpxchg(&uncore->uevents[i], NULL, uevent) == NULL) {
			hwc->idx = i;
			break;
		}
	}

out:
	if (hwc->idx < 0)
		return -EBUSY;

	cvmx_l2c_config_perf(hwc->idx, uevent->cvmx_event, 0);
	hwc->state = PERF_HES_UPTODATE | PERF_HES_STOPPED;

	if (flags & PERF_EF_START)
		oct_uncore_start(event, PERF_EF_RELOAD);

	perf_event_update_userpage(event);
	return 0;
}

static int unc_o1p_add(struct perf_event *event, int flags)
{
	return unc_mapped_add(event, flags, o1p_uncore_l2c);
}

static int unc_tad_add(struct perf_event *event, int flags)
{
	return unc_mapped_add(event, flags, tad_uncore_l2c);
}

static int unc_direct_add(struct perf_event *event,
	int flags, struct uncore_event_desc *uevents)
{
	struct hw_perf_event *hwc = &event->hw;

	hwc->state = PERF_HES_UPTODATE | PERF_HES_STOPPED;

	if (flags & PERF_EF_START)
		oct_uncore_start(event, PERF_EF_RELOAD);

	perf_event_update_userpage(event);
	return 0;
}

static int unc_lmc_add(struct perf_event *event, int flags)
{
	return unc_direct_add(event, flags, lmc_events);
}

static int unc_o23_add(struct perf_event *event, int flags)
{
	return unc_direct_add(event, flags, o23_events);
}

static void unc_direct_del(struct perf_event *event, int flags)
{
	oct_uncore_stop(event, PERF_EF_UPDATE);
	perf_event_update_userpage(event);
}

static void unc_mapped_del(struct perf_event *event,
	int flags, struct oct_uncore *uncore)
{
	int i;
	struct uncore_event_desc *uevent = event_to_oct_uevent(event);
	struct hw_perf_event *hwc = &event->hw;

	oct_uncore_stop(event, PERF_EF_UPDATE);

	for (i = 0; i < uncore->num_counters; i++) {
		if (cmpxchg(&uncore->uevents[i], uevent, NULL) == uevent)
			break;
	}

	hwc->idx = -1;
	perf_event_update_userpage(event);
}

static void unc_o1p_del(struct perf_event *event, int flags)
{
	unc_mapped_del(event, flags, o1p_uncore_l2c);
}

static void unc_tad_del(struct perf_event *event, int flags)
{
	unc_mapped_del(event, flags, tad_uncore_l2c);
}

static struct pmu oct_lmc_pmu = {
	.add		= unc_lmc_add,
	.del		= unc_direct_del,
	.read		= unc_direct_read,
};
static struct pmu o1p_l2c_pmu = {
	.add		= unc_o1p_add,
	.del		= unc_o1p_del,
	.read		= unc_mapped_read,
};
static struct pmu o23_l2c_pmu = {
	.add		= unc_o23_add,
	.del		= unc_direct_del,
	.read		= unc_direct_read,
};
static struct pmu tad_l2c_pmu = {
	.add		= unc_tad_add,
	.del		= unc_tad_del,
	.read		= unc_mapped_read,
};

static int oct_uncore_event_init(struct perf_event *event)
{
	struct oct_uncore *uncore;
	struct uncore_event_desc *uev;
	struct perf_event_attr *attr = &event->attr;
	struct hw_perf_event *hwc = &event->hw;

	if (!event || !event->pmu || !attr || attr->type != event->pmu->type)
		return -ENOENT;

	/*
	 * Octeon has a single coherent Level-2 cache, shared by all cores
	 * and all other DMAing hardware units, so L2C/LMC counters are shared.
	 * Interrupts can be directed to a single target core, however, event
	 * counts generated by processes running on other cores cannot be
	 * masked out. So we do not support sampling and * per-thread events.
	 */
	if (is_sampling_event(event) || event->attach_state & PERF_ATTACH_TASK)
		return -EINVAL;

	/* uncore counters do not have usr/os/guest/host bits */
	if (attr->exclude_user || attr->exclude_kernel ||
	    attr->exclude_host || attr->exclude_guest)
		return -EINVAL;
	pr_debug("new ev ty:%d eb:%p ac:%llx\n",
		attr->type, (void *)event->hw.event_base, attr->config);

	e_unc(event) = NULL; /* force lookup in ... */
	e_unc(event) = event_to_oct_uncore(event);

	pr_debug("link ev unc@%p pmu@%p ty:%d ev0@%p\n",
		e_unc(event),
		(e_unc(event) ? e_unc(event)->pmu : NULL),
		(e_unc(event) && e_unc(event)->pmu
				? e_unc(event)->pmu->type : 0),
		(e_unc(event) && e_unc(event)->uevent0
				? e_unc(event)->uevent0 : NULL));
	uev = event_to_oct_uevent(event);
	if (uev) {
		pr_debug("ev0[%lld]=%p\n", attr->config, uev);
		pr_debug(".magic %d .attno %d, f %x, cvmx_event %x,"
			" r %llx/%llx, n %d, stride %d\n",
			uev->magic, uev->attno, uev->flags, uev->cvmx_event,
			uev->reglo, uev->reghi, uev->counters, uev->stride);
	}
	uncore = event_to_oct_uncore(event);

	if (!uncore)
		return -ENODEV;

	BUG_ON(!uev);
	if (is_mapped_event(event)) {
		hwc->idx = -1;
	} else {
		int ev = event->attr.config;
		struct hw_perf_event *hwc = &event->hw;
		struct uncore_event_desc *uev = e_uev(event);
		int eflags = 0;
		int (*lim)(void) = NULL;

		hwc->idx = uev->attno; /* linearly mapped */

		/*
		 * UGLY, but is there a better way??
		 * Build-time enumeration of all events, deferring until runtime
		 * the eval of CVMX*_CNT & _CNT_LO/_CNT_HI macros
		 */
#undef EV
#define EV(_family, _name, _cvmx, _flags, _lo, _hi, _stride, _lim) \
	case EVID(_family, _name): \
		pr_debug("ev:%d _lo:%s _hi:%s _stride:%s _lim:%s\n", \
			ev, __stringify(_lo), __stringify(_hi), \
			__stringify(_stride), __stringify(_lim)); \
		hwc->idx = ev; \
		eflags = (_flags); \
		lim = (_lim); \
		uev->reglo = (_lo); \
		uev->reghi = (_hi); \
		uev->stride = (_stride); \
		break; /*end*/

#include "perf_uncore_events.h"

		if (is_lmc_event(event)) {
			switch (ev) {
				LMC_EVENTS /* one "case...break;" for each */
			default:
				WARN(true, "unexpected LMC event %d\n",
					ev);
				break;
			}
		}
		if (is_o23_l2c_event(event)) {
			switch (ev) {
				O23_EVENTS /* one "case...break;" for each */
			default:
				WARN(true, "unexpected o2/o3 L2C event %d\n",
					ev);
				break;
			}
		}
		uev->counters = (lim ? lim() : 1);
		pr_debug("e%d l:%llx h:%llx stride:%d lim:%d\n",
			ev, uev->reglo, uev->reghi,
			uev->stride, uev->counters);


		WARN_ONCE(!uev->reglo, "no counter for event %x\n", ev);
		if (!uev->reglo)
			return -ENOENT;

		if (lim)
			uev->counters = lim();

		if (hwc->idx == -1)
			return -EBUSY;
	}

	/* set counter baseline */
	local64_set(&event->count, 0);
	local64_set(&hwc->prev_count, 0);

	return 0;
}

static void __init uncore_type_exit(struct oct_uncore *type)
{
	kfree(type->events_group);
	type->events_group = NULL;
}

static int __init uncore_type_init(struct oct_uncore *type,
		struct uncore_event_desc *uevents, const char *name)
{
	struct attribute_group *attr_group;
	struct attribute **attrs;
	bool o1p = OCTEON_IS_OCTEON1PLUS();
	bool o2 = OCTEON_IS_OCTEON2();
	bool o3 = OCTEON_IS_OCTEON3();
	int i, j, k;

	type->pmu->event_init	= oct_uncore_event_init;
	type->pmu->start	= oct_uncore_start;
	type->pmu->stop		= oct_uncore_stop;
	type->pmu->task_ctx_nr	= perf_invalid_context;

	if (uevents) {
		for (i = 0; uevents[i].attr.attr.name; i++)
			; /* just count */

		attr_group = kzalloc(sizeof(struct attribute *) * i +
					sizeof(*attr_group), GFP_KERNEL);
		WARN_ON(!attr_group);
		if (!attr_group)
			goto fail;

		attrs = (struct attribute **)(attr_group + 1);
		attr_group->name = "events";
		attr_group->attrs = attrs;

		for (j = k = 0; j < i; j++) {
			enum unc_flags f = uevents[j].flags;

			/* skip counters not present on this chip */
			if (!((o1p && (f & UNC_O1P)) ||
			      (o2 && (f & UNC_O2)) ||
			      (o3 && (f & UNC_O3))))
				continue;

			attrs[k] = &uevents[j].attr.attr;
			pr_debug("attach %d:%s/%s:%s\n",
				k, name, attr_group->name,
				(attrs[k] ? attrs[k]->name : NULL));
			k++;
		}

		type->events_group = attr_group;
		type->format_group = &uncore_format_group;
	}

	type->pmu->attr_groups = type->attr_groups;
	type->pmu_group = &uncore_pmu_attr_group;

	return 0;
fail:
	pr_debug(pr_fmt("uncore_type_init(%s) ENOMEM\n"), name);
	uncore_type_exit(type);
	return -ENOMEM;
}

static struct oct_uncore *
__init oct_uncore_register(char *name, struct pmu *pmu,
		int num_counters, /* -ve for one-per uevent */
		struct uncore_event_desc *uevents)
{
	struct uncore_event_desc *e;
	struct oct_uncore *uncore;
	struct oct_uncore **next;
	size_t size;
	bool fixed_map = (num_counters < 0);
	int nevents;
	int n;

	/* count events */
	for (nevents = 0, e = uevents; e->attr.attr.name; nevents++, e++)
		;

	if (fixed_map)
		num_counters = nevents;

	size = sizeof(struct oct_uncore) +
		(num_counters+1) * sizeof(struct uncore_event_desc *);
	uncore = kzalloc(size, GFP_KERNEL);
	if (!uncore)
		return NULL;

	uncore->pmu = pmu;
	uncore->num_counters = num_counters;

	/* fixed list of possible events */
	uncore->uevent0 = uevents;

	/* count & (if possible) map */
	for (n = 0, e = uevents; e->attr.attr.name; n++, e++)
		if (fixed_map)
			uncore->uevents[n] = e;

	/* merge this into loop above ... */
	n = uncore_type_init(uncore, uevents, name);
	if (n) {
		kfree(uncore);
		return NULL;
	}

	n = perf_pmu_register(uncore->pmu, name, -1);
	if (n) {
		pr_debug(pr_fmt("perf_pmu_register(%s) err %d\n"),
			name, n);
		uncore_type_exit(uncore);
		return NULL;
	}

	/* all is complete, add to list */
	for (next = live_uncores; *next; next++)
		;
	*next = uncore;

	pr_info(pr_fmt("%s counters detected\n"), name);

	return uncore;
}

static void __init uncore_types_exit(struct oct_uncore **types)
{
	int i;

	for (i = 0; types[i]; i++) {
		uncore_type_exit(types[i]);
		kfree(types[i]);
		types[i] = NULL;
	}
}

static void oct_uncore_exit(void)
{
	uncore_types_exit(live_uncores);
	kfree(oct_uncore_lmc);
	kfree(o1p_uncore_l2c);
	kfree(o23_uncore_l2c);
	kfree(tad_uncore_l2c);
}
module_exit(oct_uncore_exit);

static int __init oct_uncore_init(void)
{
	/* octeon-1/2/3 Local Memory Controller */
	oct_uncore_lmc = oct_uncore_register("uncore_mc",
			&oct_lmc_pmu, -1, lmc_events);

	if (OCTEON_IS_OCTEON1PLUS()) {
		/* must sign-extend 36-bit O1P mapped counters */
		mapped_sextend_bits = (64 - 36);

		o1p_uncore_l2c = oct_uncore_register("uncore_l2c",
				&o1p_l2c_pmu, NUM_COUNTERS_L2C, o1p_events);

	} else {
		int tad;

		o23_uncore_l2c = oct_uncore_register("uncore_l2c",
				&o23_l2c_pmu, -1, o23_events);

		/* associate each TAD's 4 counters with the "NONE" event */
		for (tad = 0; tad < CVMX_L2C_TADS; tad++)
			cvmx_write_csr(CVMX_L2C_TADX_PRF(tad), 0);

		tad_uncore_l2c = oct_uncore_register("uncore_tad",
				&tad_l2c_pmu, NUM_COUNTERS_L2C, tad_events);
	}

	/* FIXME: generalize to fit existing perf_hw_cache_id hierarchy:
	 * add some l2c events as PERF_COUNT_HW_CACHE_LL
	 * and some perf_event_mipsxx.c events as other PERF_COUNT_HW_xxx
	 */

	return 0;

	uncore_types_exit(live_uncores);

	return -ENOMEM;
}
device_initcall(oct_uncore_init);

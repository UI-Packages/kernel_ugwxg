/*
 * Octeon Watchdog driver
 *
 * Copyright (C) 2007 - 2016 Cavium, Inc.
 *
 * Some parts derived from wdt.c
 *
 *	(c) Copyright 1996-1997 Alan Cox <alan@lxorguk.ukuu.org.uk>,
 *						All Rights Reserved.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License
 *	as published by the Free Software Foundation; either version
 *	2 of the License, or (at your option) any later version.
 *
 *	Neither Alan Cox nor CymruNet Ltd. admit liability nor provide
 *	warranty for any of this software. This material is provided
 *	"AS-IS" and at no charge.
 *
 *	(c) Copyright 1995    Alan Cox <alan@lxorguk.ukuu.org.uk>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 *
 * The OCTEON watchdog has a maximum timeout of 2^32 * io_clock.
 * For most systems this is less than 10 seconds, so to allow for
 * software to request longer watchdog heartbeats, we maintain software
 * counters to count multiples of the base rate.  If the system locks
 * up in such a manner that we can not run the software counters, the
 * only result is a watchdog reset sooner than was requested.  But
 * that is OK, because in this case userspace would likely not be able
 * to do anything anyhow.
 *
 * The hardware watchdog interval we call the period.  The OCTEON
 * watchdog goes through several stages, after the first period an
 * irq is asserted, then if it is not reset, after the next period NMI
 * is asserted, then after an additional period a chip wide soft reset.
 * So for the software counters, we reset watchdog after each period
 * and decrement the counter.  But for the last two periods we need to
 * let the watchdog progress to the NMI stage so we disable the irq
 * and let it proceed.  Once in the NMI, we print the register state
 * to the serial port and then wait for the reset.
 *
 * A watchdog is maintained for each CPU in the system, that way if
 * one CPU suffers a lockup, we also get a register dump and reset.
 * The userspace ping resets the watchdog on all CPUs.
 *
 * Before userspace opens the watchdog device, we still run the
 * watchdogs to catch any lockups that may be kernel related.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/watchdog.h>
#include <linux/cpumask.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/cpu.h>
#include <linux/smp.h>
#include <linux/fs.h>
#include <linux/irq.h>

#include <asm/mipsregs.h>
#include <asm/uasm.h>

#include <asm/octeon/octeon.h>
#include <asm/octeon/cvmx-boot-vector.h>
#include <asm/octeon/cvmx-ciu2-defs.h>
#include <asm/octeon/cvmx-gserx-defs.h>

/* Watchdog interrupt major block number (8 MSBs of intsn) */
#define WD_BLOCK_NUMBER		0x01

static int counter;

/* The count needed to achieve timeout_sec. */
static unsigned int timeout_cnt;

/* The maximum period supported. */
static unsigned int max_timeout_sec;

/* The current period.  */
static unsigned int timeout_sec;

/* Set to non-zero when userspace countdown mode active */
static bool do_countdown;
static unsigned int countdown_reset;
static unsigned int per_cpu_countdown[NR_CPUS];

static cpumask_t irq_enabled_cpus;

#define WD_TIMO 60			/* Default heartbeat = 60 seconds */

static int heartbeat = WD_TIMO;
module_param(heartbeat, int, S_IRUGO);
MODULE_PARM_DESC(heartbeat,
	"Watchdog heartbeat in seconds. (0 < heartbeat, default="
				__MODULE_STRING(WD_TIMO) ")");

static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, S_IRUGO);
MODULE_PARM_DESC(nowayout,
	"Watchdog cannot be stopped once started (default="
				__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

static int disable;
module_param(disable, int, S_IRUGO);
MODULE_PARM_DESC(disable,
	"Disable the watchdog entirely (default=0)");

static unsigned long octeon_wdt_is_open;
static char expect_close;
static struct cvmx_boot_vector_element *octeon_wdt_bootvector;

void octeon_wdt_nmi_stage2(void);

static int cpu2core(int cpu)
{
#ifdef CONFIG_SMP
	return cpu_logical_map(cpu) & 0x3f;
#else
	return cvmx_get_core_num();
#endif
}

/**
 * Poke the watchdog when an interrupt is received
 *
 * @cpl:
 * @dev_id:
 *
 * Returns
 */
static irqreturn_t octeon_wdt_poke_irq(int cpl, void *dev_id)
{
	int cpu = raw_smp_processor_id();
	unsigned int core = cpu2core(cpu);
	int node = cpu_to_node(cpu);

	if (do_countdown) {
		if (per_cpu_countdown[cpu] > 0) {
			/* We're alive, poke the watchdog */
			cvmx_write_csr_node(node, CVMX_CIU_PP_POKEX(core), 1);
			per_cpu_countdown[cpu]--;
		} else {
			/* Bad news, you are about to reboot. */
			disable_irq_nosync(cpl);
			cpumask_clear_cpu(cpu, &irq_enabled_cpus);
		}
	} else {
		/* Not open, just ping away... */
		cvmx_write_csr_node(node, CVMX_CIU_PP_POKEX(core), 1);
	}
	return IRQ_HANDLED;
}

/* From setup.c */
extern int prom_putchar(char c);

/**
 * Write a string to the uart
 *
 * @str:        String to write
 */
static void octeon_wdt_write_string(const char *str)
{
	/* Just loop writing one byte at a time */
	while (*str)
		prom_putchar(*str++);
}

/**
 * Write a hex number out of the uart
 *
 * @value:      Number to display
 * @digits:     Number of digits to print (1 to 16)
 */
static void octeon_wdt_write_hex(u64 value, int digits)
{
	int d;
	int v;
	for (d = 0; d < digits; d++) {
		v = (value >> ((digits - d - 1) * 4)) & 0xf;
		if (v >= 10)
			prom_putchar('a' + v - 10);
		else
			prom_putchar('0' + v);
	}
}

const char *reg_name[] = {
	"$0", "at", "v0", "v1", "a0", "a1", "a2", "a3",
	"a4", "a5", "a6", "a7", "t0", "t1", "t2", "t3",
	"s0", "s1", "s2", "s3", "s4", "s5", "s6", "s7",
	"t8", "t9", "k0", "k1", "gp", "sp", "s8", "ra"
};

/**
 * NMI stage 3 handler. NMIs are handled in the following manner:
 * 1) The first NMI handler enables CVMSEG and transfers from
 * the bootbus region into normal memory. It is careful to not
 * destroy any registers.
 * 2) The second stage handler uses CVMSEG to save the registers
 * and create a stack for C code. It then calls the third level
 * handler with one argument, a pointer to the register values.
 * 3) The third, and final, level handler is the following C
 * function that prints out some useful infomration.
 *
 * @reg:    Pointer to register state before the NMI
 */
void octeon_wdt_nmi_stage3(u64 reg[32])
{
	u64 i;

	unsigned int coreid = cvmx_get_core_num();
	/*
	 * Save status and cause early to get them before any changes
	 * might happen.
	 */
	u64 cp0_cause = read_c0_cause();
	u64 cp0_status = read_c0_status();
	u64 cp0_error_epc = read_c0_errorepc();
	u64 cp0_epc = read_c0_epc();
	bool is_ciu2 = OCTEON_IS_MODEL(OCTEON_CN68XX);

	/* Delay so output from all cores output is not jumbled together. */
	udelay(85000 * coreid);

	octeon_wdt_write_string("\r\n*** NMI Watchdog interrupt on Core 0x");
	octeon_wdt_write_hex(coreid, 2);
	octeon_wdt_write_string(" ***\r\n");
	for (i = 0; i < 32; i++) {
		octeon_wdt_write_string("\t");
		octeon_wdt_write_string(reg_name[i]);
		octeon_wdt_write_string("\t0x");
		octeon_wdt_write_hex(reg[i], 16);
		if (i & 1)
			octeon_wdt_write_string("\r\n");
	}
	octeon_wdt_write_string("\terr_epc\t0x");
	octeon_wdt_write_hex(cp0_error_epc, 16);

	octeon_wdt_write_string("\tepc\t0x");
	octeon_wdt_write_hex(cp0_epc, 16);
	octeon_wdt_write_string("\r\n");

	octeon_wdt_write_string("\tstatus\t0x");
	octeon_wdt_write_hex(cp0_status, 16);
	octeon_wdt_write_string("\tcause\t0x");
	octeon_wdt_write_hex(cp0_cause, 16);
	octeon_wdt_write_string("\r\n");

	/* The CIU register are different on the different octeon models */
	if (is_ciu2) {
		octeon_wdt_write_string("\tsrc_wd\t0x");
		octeon_wdt_write_hex(cvmx_read_csr(CVMX_CIU2_SRC_PPX_IP2_WDOG(coreid)), 16);
		octeon_wdt_write_string("\ten_wd\t0x");
		octeon_wdt_write_hex(cvmx_read_csr(CVMX_CIU2_EN_PPX_IP2_WDOG(coreid)), 16);
		octeon_wdt_write_string("\r\n");
		octeon_wdt_write_string("\tsrc_rml\t0x");
		octeon_wdt_write_hex(cvmx_read_csr(CVMX_CIU2_SRC_PPX_IP2_RML(coreid)), 16);
		octeon_wdt_write_string("\ten_rml\t0x");
		octeon_wdt_write_hex(cvmx_read_csr(CVMX_CIU2_EN_PPX_IP2_RML(coreid)), 16);
		octeon_wdt_write_string("\r\n");
		octeon_wdt_write_string("\tsum\t0x");
		octeon_wdt_write_hex(cvmx_read_csr(CVMX_CIU2_SUM_PPX_IP2(coreid)), 16);
		octeon_wdt_write_string("\r\n");
	} else if (!octeon_has_feature(OCTEON_FEATURE_CIU3)) {
		octeon_wdt_write_string("\tsum0\t0x");
		octeon_wdt_write_hex(cvmx_read_csr(CVMX_CIU_INTX_SUM0(coreid * 2)), 16);
		octeon_wdt_write_string("\ten0\t0x");
		octeon_wdt_write_hex(cvmx_read_csr(CVMX_CIU_INTX_EN0(coreid * 2)), 16);
		octeon_wdt_write_string("\r\n");
	}

	octeon_wdt_write_string("*** Chip soft reset soon ***\r\n");

	/*
	 * G-30204: We must trigger a soft reset before watchdog
	 * does an incomplete job of doing it.
	 */
	if (OCTEON_IS_OCTEON3() && !OCTEON_IS_MODEL(OCTEON_CN70XX)) {
		u64 scr;
		unsigned int node = cvmx_get_node_num();
		unsigned int lcore = cvmx_get_local_core_num();
		union cvmx_ciu_wdogx ciu_wdog;

		/*
		 * Wait for other cores to print out information, but
		 * not too long.  Do the soft reset before watchdog
		 * can trigger it.
		 */
		do {
			ciu_wdog.u64 = cvmx_read_csr_node(node, CVMX_CIU_WDOGX(lcore));
		} while (ciu_wdog.s.cnt > 0x10000);

		scr = cvmx_read_csr_node(0, CVMX_GSERX_SCRATCH(0));
		scr |= 1 << 11; /* Indicate watchdog in bit 11 */
		cvmx_write_csr_node(0, CVMX_GSERX_SCRATCH(0), scr);
		cvmx_write_csr_node(0, CVMX_RST_SOFT_RST, 1);
	}
}

static int octeon_wdt_cpu_to_irq(int cpu)
{
	unsigned int coreid;
	int node;
	int irq;

	coreid = cpu2core(cpu);
	node = cpu_to_node(cpu);

	if (octeon_has_feature(OCTEON_FEATURE_CIU3)) {
		struct irq_domain *domain;
		int hwirq;
		domain = octeon_irq_get_block_domain(node,
						     WD_BLOCK_NUMBER);
		hwirq = WD_BLOCK_NUMBER << 12 | 0x200 | coreid;
		irq = irq_find_mapping(domain, hwirq);
	} else {
		irq = OCTEON_IRQ_WDOG0 + coreid;
	}
	return irq;

}

static void octeon_wdt_disable_interrupt(int cpu)
{
	unsigned int core;
	int node;
	union cvmx_ciu_wdogx ciu_wdog;

	core = cpu2core(cpu);
	node = cpu_to_node(cpu);

	/* Poke the watchdog to clear out its state */
	cvmx_write_csr_node(node, CVMX_CIU_PP_POKEX(core), 1);

	/* Disable the hardware. */
	ciu_wdog.u64 = 0;
	cvmx_write_csr_node(node, CVMX_CIU_WDOGX(core), ciu_wdog.u64);

	free_irq(octeon_wdt_cpu_to_irq(cpu), octeon_wdt_poke_irq);
}

static void octeon_wdt_setup_interrupt(int cpu)
{
	unsigned int core;
	unsigned int irq;
	union cvmx_ciu_wdogx ciu_wdog;
	int node;
	struct irq_domain *domain;
	int hwirq;

	core = cpu2core(cpu);
	node = cpu_to_node(cpu);

	octeon_wdt_bootvector[core].target_ptr = (uint64_t)octeon_wdt_nmi_stage2;

	/* Disable it before doing anything with the interrupts. */
	ciu_wdog.u64 = 0;
	cvmx_write_csr_node(node, CVMX_CIU_WDOGX(core), ciu_wdog.u64);

	per_cpu_countdown[cpu] = countdown_reset;

	if (octeon_has_feature(OCTEON_FEATURE_CIU3)) {
		/* Must get the domain for the watchdog block */
		domain = octeon_irq_get_block_domain(node, WD_BLOCK_NUMBER);

		/* Get a irq for the wd intsn (hardware interrupt) */
		hwirq = WD_BLOCK_NUMBER << 12 | 0x200 | core;
		irq = irq_create_mapping(domain, hwirq);
	} else
		irq = OCTEON_IRQ_WDOG0 + core;

	if (request_irq(irq, octeon_wdt_poke_irq,
			IRQF_NO_THREAD, "octeon_wdt", octeon_wdt_poke_irq))
		panic("octeon_wdt: Couldn't obtain irq %d", irq);

	/* Must set the irq affinity here */
	if (octeon_has_feature(OCTEON_FEATURE_CIU3)) {
		cpumask_t mask;

		cpumask_clear(&mask);
		cpumask_set_cpu(cpu, &mask);
		irq_set_affinity(irq, &mask);
	}

	cpumask_set_cpu(cpu, &irq_enabled_cpus);

	/* Poke the watchdog to clear out its state */
	cvmx_write_csr_node(node, CVMX_CIU_PP_POKEX(core), 1);

	/* Finally enable the watchdog now that all handlers are installed */
	ciu_wdog.u64 = 0;
	ciu_wdog.s.len = timeout_cnt;
	ciu_wdog.s.mode = 3;	/* 3 = Interrupt + NMI + Soft-Reset */
	cvmx_write_csr_node(node, CVMX_CIU_WDOGX(core), ciu_wdog.u64);
}

static int octeon_wdt_cpu_callback(struct notifier_block *nfb,
					   unsigned long action, void *hcpu)
{
	unsigned int cpu = (unsigned long)hcpu;

	switch (action) {
	case CPU_DOWN_PREPARE:
		octeon_wdt_disable_interrupt(cpu);
		break;
	case CPU_ONLINE:
	case CPU_DOWN_FAILED:
		octeon_wdt_setup_interrupt(cpu);
		break;
	default:
		break;
	}
	return NOTIFY_OK;
}

static void octeon_wdt_ping(void)
{
	int cpu;
	int coreid;
	int node;

	if (disable)
		return;

	for_each_online_cpu(cpu) {
		coreid = cpu2core(cpu);
		node = cpu_to_node(cpu);
		cvmx_write_csr_node(node, CVMX_CIU_PP_POKEX(coreid), 1);
		per_cpu_countdown[cpu] = countdown_reset;
		if ((countdown_reset || !do_countdown) &&
		    !cpumask_test_cpu(cpu, &irq_enabled_cpus)) {
			/* We have to enable the irq */
			enable_irq(octeon_wdt_cpu_to_irq(cpu));
			cpumask_set_cpu(cpu, &irq_enabled_cpus);
		}
	}
}

static void octeon_wdt_calc_parameters(int t)
{
	unsigned int periods;

	timeout_sec = max_timeout_sec;


	/*
	 * Find the largest interrupt period, that can evenly divide
	 * the requested heartbeat time.
	 */
	while ((t % timeout_sec) != 0)
		timeout_sec--;

	periods = t / timeout_sec;

	/*
	 * The last two periods are after the irq is disabled, and
	 * then to the nmi, so we subtract them off.
	 */

	countdown_reset = periods > 2 ? periods - 2 : 0;
	heartbeat = t;
	timeout_cnt = ((octeon_get_io_clock_rate() / counter) * timeout_sec) >> 8;
}

static int octeon_wdt_set_heartbeat(int t)
{
	int cpu;
	int coreid;
	union cvmx_ciu_wdogx ciu_wdog;
	int node;

	if (t <= 0)
		return -1;

	octeon_wdt_calc_parameters(t);

	if (disable)
		return 0;

	for_each_online_cpu(cpu) {
		coreid = cpu2core(cpu);
		node = cpu_to_node(cpu);
		cvmx_write_csr_node(node, CVMX_CIU_PP_POKEX(coreid), 1);
		ciu_wdog.u64 = 0;
		ciu_wdog.s.len = timeout_cnt;
		ciu_wdog.s.mode = 3;	/* 3 = Interrupt + NMI + Soft-Reset */
		cvmx_write_csr_node(node, CVMX_CIU_WDOGX(coreid), ciu_wdog.u64);
		cvmx_write_csr_node(node, CVMX_CIU_PP_POKEX(coreid), 1);
	}
	octeon_wdt_ping(); /* Get the irqs back on. */
	return 0;
}

/**
 *	octeon_wdt_write:
 *	@file: file handle to the watchdog
 *	@buf: buffer to write (unused as data does not matter here
 *	@count: count of bytes
 *	@ppos: pointer to the position to write. No seeks allowed
 *
 *	A write to a watchdog device is defined as a keepalive signal. Any
 *	write of data will do, as we we don't define content meaning.
 */

static ssize_t octeon_wdt_write(struct file *file, const char __user *buf,
				size_t count, loff_t *ppos)
{
	if (count) {
		if (!nowayout) {
			size_t i;

			/* In case it was set long ago */
			expect_close = 0;

			for (i = 0; i != count; i++) {
				char c;
				if (get_user(c, buf + i))
					return -EFAULT;
				if (c == 'V')
					expect_close = 1;
			}
		}
		octeon_wdt_ping();
	}
	return count;
}

/**
 *	octeon_wdt_ioctl:
 *	@file: file handle to the device
 *	@cmd: watchdog command
 *	@arg: argument pointer
 *
 *	The watchdog API defines a common set of functions for all
 *	watchdogs according to their available features. We only
 *	actually usefully support querying capabilities and setting
 *	the timeout.
 */

static long octeon_wdt_ioctl(struct file *file, unsigned int cmd,
			     unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int __user *p = argp;
	int new_heartbeat;

	static struct watchdog_info ident = {
		.options =		WDIOF_SETTIMEOUT|
					WDIOF_MAGICCLOSE|
					WDIOF_KEEPALIVEPING,
		.firmware_version =	1,
		.identity =		"OCTEON",
	};

	switch (cmd) {
	case WDIOC_GETSUPPORT:
		return copy_to_user(argp, &ident, sizeof(ident)) ? -EFAULT : 0;
	case WDIOC_GETSTATUS:
	case WDIOC_GETBOOTSTATUS:
		return put_user(0, p);
	case WDIOC_KEEPALIVE:
		octeon_wdt_ping();
		return 0;
	case WDIOC_SETTIMEOUT:
		if (get_user(new_heartbeat, p))
			return -EFAULT;
		if (octeon_wdt_set_heartbeat(new_heartbeat))
			return -EINVAL;
		/* Fall through. */
	case WDIOC_GETTIMEOUT:
		return put_user(heartbeat, p);
	default:
		return -ENOTTY;
	}
}

/**
 *	octeon_wdt_open:
 *	@inode: inode of device
 *	@file: file handle to device
 *
 *	The watchdog device has been opened. The watchdog device is single
 *	open and on opening we do a ping to reset the counters.
 */

static int octeon_wdt_open(struct inode *inode, struct file *file)
{
	if (test_and_set_bit(0, &octeon_wdt_is_open))
		return -EBUSY;
	/*
	 *	Activate
	 */
	octeon_wdt_ping();
	do_countdown = true;
	return nonseekable_open(inode, file);
}

/**
 *	octeon_wdt_release:
 *	@inode: inode to board
 *	@file: file handle to board
 *
 *	The watchdog has a configurable API. There is a religious dispute
 *	between people who want their watchdog to be able to shut down and
 *	those who want to be sure if the watchdog manager dies the machine
 *	reboots. In the former case we disable the counters, in the latter
 *	case you have to open it again very soon.
 */

static int octeon_wdt_release(struct inode *inode, struct file *file)
{
	if (expect_close) {
		do_countdown = false;
		octeon_wdt_ping();
	} else {
		pr_crit("WDT device closed unexpectedly.  WDT will not stop!\n");
	}
	clear_bit(0, &octeon_wdt_is_open);
	expect_close = 0;
	return 0;
}

static const struct file_operations octeon_wdt_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.write		= octeon_wdt_write,
	.unlocked_ioctl	= octeon_wdt_ioctl,
	.open		= octeon_wdt_open,
	.release	= octeon_wdt_release,
};

static struct miscdevice octeon_wdt_miscdev = {
	.minor	= WATCHDOG_MINOR,
	.name	= "watchdog",
	.fops	= &octeon_wdt_fops,
};

static struct notifier_block octeon_wdt_cpu_notifier = {
	.notifier_call = octeon_wdt_cpu_callback,
};


/**
 * Module/ driver initialization.
 *
 * Returns Zero on success
 */
static int __init octeon_wdt_init(void)
{
	int ret;
	int cpu;

	octeon_wdt_bootvector = cvmx_boot_vector_get();
	if (!octeon_wdt_bootvector) {
		pr_err("Error: Cannot allocate boot vector.\n");
		return -ENOMEM;
	}

	if (OCTEON_IS_MODEL(OCTEON_CN68XX))
		counter = (0x1 << 9);
	else if (OCTEON_IS_MODEL(OCTEON_CN78XX))
		counter = 0x400;
	else
		counter = (0x1 << 8);

	/*
	 * Watchdog time expiration length = The 16 bits of LEN
	 * represent the most significant bits of a 24 bit decrementer
	 * that decrements every counter cycles.
	 *
	 * Try for a timeout of 5 sec, if that fails a smaller number
	 * of even seconds,
	 */
	max_timeout_sec = 6;
	do {
		max_timeout_sec--;
		timeout_cnt = ((octeon_get_io_clock_rate() / counter) * max_timeout_sec) >> 8;
	} while (timeout_cnt > 65535);

	BUG_ON(timeout_cnt == 0);

	octeon_wdt_calc_parameters(heartbeat);

	pr_info("Initial granularity %d Sec\n", timeout_sec);

	ret = misc_register(&octeon_wdt_miscdev);
	if (ret) {
		pr_err("cannot register miscdev on minor=%d (err=%d)\n",
		       WATCHDOG_MINOR, ret);
		goto out;
	}

	if (disable) {
		pr_notice("disabled\n");
		return 0;
	}

	cpumask_clear(&irq_enabled_cpus);

	for_each_online_cpu(cpu)
		octeon_wdt_setup_interrupt(cpu);

	register_hotcpu_notifier(&octeon_wdt_cpu_notifier);
out:
	return ret;
}

/**
 * Module / driver shutdown
 */
static void __exit octeon_wdt_cleanup(void)
{
	int cpu;
	int node;
	unsigned int irq;
	struct irq_domain *domain;
	int hwirq;

	misc_deregister(&octeon_wdt_miscdev);

	if (disable)
		return;

	unregister_hotcpu_notifier(&octeon_wdt_cpu_notifier);

	for_each_online_cpu(cpu) {
		int core = cpu2core(cpu);
		node = cpu_to_node(cpu);
		/* Disable the watchdog */
		cvmx_write_csr_node(node, CVMX_CIU_WDOGX(core), 0);

		/* Free the interrupt handler */
		if (octeon_has_feature(OCTEON_FEATURE_CIU3)) {
			domain = octeon_irq_get_block_domain(node,
							     WD_BLOCK_NUMBER);
			hwirq = WD_BLOCK_NUMBER << 12 | 0x200 | core;
			irq = irq_find_mapping(domain, hwirq);
		} else
			irq = OCTEON_IRQ_WDOG0 + core;
		free_irq(irq, octeon_wdt_poke_irq);
	}
	/*
	 * Disable the boot-bus memory, the code it points to is soon
	 * to go missing.
	 */
	cvmx_write_csr(CVMX_MIO_BOOT_LOC_CFGX(0), 0);
}

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cavium Inc. <support@cavium.com>");
MODULE_DESCRIPTION("Cavium Inc. OCTEON Watchdog driver.");
module_init(octeon_wdt_init);
module_exit(octeon_wdt_cleanup);

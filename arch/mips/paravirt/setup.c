/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2013 Cavium, Inc.
 */

#include <linux/kernel.h>
#include <linux/kvm_para.h>

#include <asm/reboot.h>
#include <asm/bootinfo.h>
#include <asm/smp-ops.h>
#include <asm/time.h>

extern struct plat_smp_ops paravirt_smp_ops;

const char *get_system_type(void)
{
	return "MIPS Para-Virtualized Guest";
}

static cycle_t csrc_host_time_read(struct clocksource *cs)
{
	return kvm_hypercall0_u64(KVM_HC_MIPS_GET_HOST_TIME);
}

static struct clocksource csrc_host_time = {
	.name		= "HOST_TIME",
	.read		= csrc_host_time_read,
	.mask		= CLOCKSOURCE_MASK(64),
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

void __init plat_time_init(void)
{
	mips_hpt_frequency = kvm_hypercall0(KVM_HC_MIPS_GET_CLOCK_FREQ);

	preset_lpj = mips_hpt_frequency / (2 * HZ);
	csrc_host_time.rating = 400;
	clocksource_register_hz(&csrc_host_time, 1000000000);
}

static void plat_halt_this_cpu(void *ignore)
{
	kvm_hypercall0(KVM_HC_MIPS_EXIT_VM);
}

static void plat_halt(void)
{
	on_each_cpu(plat_halt_this_cpu, NULL, 0);
}

/**
 * Early entry point for arch setup
 */
void __init prom_init(void)
{
	int i;
	int argc = fw_arg0;
	char **argv = (char **)fw_arg1;

#ifdef CONFIG_32BIT
	set_io_port_base(KSEG1ADDR(0x1e000000));
#else /* CONFIG_64BIT */
	set_io_port_base(PHYS_TO_XKSEG_UNCACHED(0x1e000000));
#endif

	for (i = 0; i < argc; i++) {
		strlcat(arcs_cmdline, argv[i], COMMAND_LINE_SIZE);
		if (i < argc - 1)
			strlcat(arcs_cmdline, " ", COMMAND_LINE_SIZE);
	}
	register_smp_ops(&paravirt_smp_ops);
	_machine_halt = plat_halt;
}

void __init plat_mem_setup(void)
{
	/* Do nothing, the "mem=???" parser handles our memory. */
}

void __init prom_free_prom_memory(void)
{
}

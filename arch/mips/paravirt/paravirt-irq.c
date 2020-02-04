/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2013 Cavium, Inc.
 */

#include <linux/interrupt.h>
#include <linux/cpumask.h>
#include <linux/bitmap.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/msi.h>

#include <asm/io.h>

#define MBOX_BITS_PER_CPU 3

struct mipsvz_irq_chip {
	u32	version;	/* version of irq-chip implementation */
	u32	numbits;	/* # of supported interrupt bits */
	u32	numcpus;	/* # of supported CPUs */
	u32	bm_length;	/* length as u32 of used bitmaps */
	u32	bm_size;	/* size of used bitmaps */

	void __iomem *base;

	/* per CPU irq-source bitmaps to signal interrupt to guest */
	unsigned long cpu_irq_src_bitmap;
	/*
	 * All other bitmaps defined in kvm_mips_vz.h are not directly
	 * accessed by guest code, instead writes are done using below
	 * defined register offsets.
	 */
} para_irq_chip;

#define KVM_MIPSVZ_IC_REG_NUM_BITS	0	/* number of IRQs supported */
#define KVM_MIPSVZ_IC_REG_NUM_CPUS	4	/* number of CPUs supported */
#define KVM_MIPSVZ_IC_REG_VERSION	8	/* version of this irq_chip */
#define KVM_MIPSVZ_IC_REG_FEATURES	0xc	/* feature flags (if any) */

#define KVM_MIPSVZ_IC_REG_IRQ_SET 0x10	/* set irq pending (except MBOX) */
#define KVM_MIPSVZ_IC_REG_IRQ_CLR 0x14	/* clear irq pending (except MBOX) */
#define KVM_MIPSVZ_IC_REG_IRQ_EN  0x18	/* enable irq globally (except MBOX) */
#define KVM_MIPSVZ_IC_REG_IRQ_DIS 0x1c	/* disable irq globally (except MBOX) */

#define KVM_MIPSVZ_IC_REG_CPU_IRQ_SET	0x20	/* set irq pending (MBOX) */
#define KVM_MIPSVZ_IC_REG_CPU_IRQ_CLR	0x24	/* clear irq pending (MBOX) */
#define KVM_MIPSVZ_IC_REG_CPU_IRQ_EN	0x28	/* enable irq per CPU */
#define KVM_MIPSVZ_IC_REG_CPU_IRQ_DIS	0x2c	/* disable irq per CPU */

/* mipsvz_irq_chip MMIO area containing bitmaps */
#define KVM_MIPSVZ_IC_BM_AREA		0x40

unsigned long *irq_msi_map;
DEFINE_SPINLOCK(irq_msi_map_lock);

static int cpunum_for_cpu(int cpu)
{
#ifdef CONFIG_SMP
	return cpu_logical_map(cpu);
#else
	return get_ebase_cpunum();
#endif
}

struct core_chip_data {
	struct mutex core_irq_mutex;
	bool current_en;
	bool desired_en;
	u8 bit;
};

static struct core_chip_data irq_core_chip_data[8];

static void irq_core_ack(struct irq_data *data)
{
	struct core_chip_data *cd = irq_data_get_irq_chip_data(data);
	unsigned int bit = cd->bit;

	/*
	 * We don't need to disable IRQs to make these atomic since
	 * they are already disabled earlier in the low level
	 * interrupt code.
	 */
	clear_c0_status(0x100 << bit);
	/* The two user interrupts must be cleared manually. */
	if (bit < 2)
		clear_c0_cause(0x100 << bit);
}

static void irq_core_eoi(struct irq_data *data)
{
	struct core_chip_data *cd = irq_data_get_irq_chip_data(data);

	/*
	 * We don't need to disable IRQs to make these atomic since
	 * they are already disabled earlier in the low level
	 * interrupt code.
	 */
	set_c0_status(0x100 << cd->bit);
}

static void irq_core_set_enable_local(void *arg)
{
	struct irq_data *data = arg;
	struct core_chip_data *cd = irq_data_get_irq_chip_data(data);
	unsigned int mask = 0x100 << cd->bit;

	/*
	 * Interrupts are already disabled, so these are atomic.
	 */
	if (cd->desired_en)
		set_c0_status(mask);
	else
		clear_c0_status(mask);

}

static void irq_core_disable(struct irq_data *data)
{
	struct core_chip_data *cd = irq_data_get_irq_chip_data(data);
	cd->desired_en = false;
}

static void irq_core_enable(struct irq_data *data)
{
	struct core_chip_data *cd = irq_data_get_irq_chip_data(data);
	cd->desired_en = true;
}

static void irq_core_bus_lock(struct irq_data *data)
{
	struct core_chip_data *cd = irq_data_get_irq_chip_data(data);

	mutex_lock(&cd->core_irq_mutex);
}

static void irq_core_bus_sync_unlock(struct irq_data *data)
{
	struct core_chip_data *cd = irq_data_get_irq_chip_data(data);

	if (cd->desired_en != cd->current_en) {
		/*
		 * Can be called in early init when on_each_cpu() will
		 * unconditionally enable irqs, so handle the case
		 * where only a single CPU is online specially, and
		 * directly call.
		 */
		if (num_online_cpus() == 1)
			irq_core_set_enable_local(data);
		else
			on_each_cpu(irq_core_set_enable_local, data, 1);

		cd->current_en = cd->desired_en;
	}

	mutex_unlock(&cd->core_irq_mutex);
}

static struct irq_chip irq_chip_core = {
	.name = "Core",
	.irq_enable = irq_core_enable,
	.irq_disable = irq_core_disable,
	.irq_ack = irq_core_ack,
	.irq_eoi = irq_core_eoi,
	.irq_bus_lock = irq_core_bus_lock,
	.irq_bus_sync_unlock = irq_core_bus_sync_unlock,

	.irq_cpu_online = irq_core_eoi,
	.irq_cpu_offline = irq_core_ack,
	.flags = IRQCHIP_ONOFFLINE_ENABLED,
};

static void __init irq_init_core(void)
{
	int i;
	int irq;
	struct core_chip_data *cd;

	/* Start with a clean slate */
	clear_c0_status(ST0_IM);
	clear_c0_cause(CAUSEF_IP0 | CAUSEF_IP1);

	for (i = 0; i < ARRAY_SIZE(irq_core_chip_data); i++) {
		cd = irq_core_chip_data + i;
		cd->current_en = false;
		cd->desired_en = false;
		cd->bit = i;
		mutex_init(&cd->core_irq_mutex);

		irq = MIPS_CPU_IRQ_BASE + i;

		switch (i) {
		case 0: /* SW0 */
		case 1: /* SW1 */
		case 5: /* IP5 */
		case 6: /* IP6 */
		case 7: /* IP7 */
			irq_set_chip_data(irq, cd);
			irq_set_chip_and_handler(irq, &irq_chip_core,
						 handle_percpu_irq);
			break;
		default:
			irq_reserve_irq(irq);
		}
	}
}



/* XXX - to be reviewed
 * (0) enable irq (PCI/MSI) globally
 * (1) enable irq for at least 1 CPU (default CPU0)
 * (2) "hardware" sets pending bit
 * (3) set irq_src for CPU(s) and raise guest irq
 * (4) guest handles irq and clears pending bit
 *
 * for MIPSVZ_IRQ_CHIP_REG_IRQ_{EN,DIS} pass
 *       (irq)                  as parameter
 *
 * for MIPSVZ_IRQ_CHIP_REG_CPU_IRQ_EN & friends pass
 *       (cpu << 20 | irq)      as parameter
 *
 * to set an irq:
 *      cpu_irq_src[irq] = irq_en[irq] & cpu_irq_en[irq];
 * to clear an irq
 *      cpu_irq_src[irq] = 0;
 * to mask an irq
 *      cpu_irq_en[irq] = 0;    (MBOX)
 *      irq_en[irq] = 0;        (PCI/MSI)
 * to unmask an irq
 *      cpu_irq_en[irq] = 1;    (MBOX)
 *      irq_en[irq] = 1;        (PCI/MSI)
 */

static void irq_pci_enable(struct irq_data *data)
{
	__raw_writel(data->irq, para_irq_chip.base + KVM_MIPSVZ_IC_REG_IRQ_EN);
}

static void irq_pci_disable(struct irq_data *data)
{
	__raw_writel(data->irq, para_irq_chip.base + KVM_MIPSVZ_IC_REG_IRQ_DIS);
}

static void irq_pci_ack(struct irq_data *data)
{
}

static void irq_pci_mask(struct irq_data *data)
{
	__raw_writel(data->irq, para_irq_chip.base + KVM_MIPSVZ_IC_REG_IRQ_DIS);
}

static void irq_pci_unmask(struct irq_data *data)
{
	__raw_writel(data->irq, para_irq_chip.base + KVM_MIPSVZ_IC_REG_IRQ_EN);
}

static struct irq_chip irq_chip_pci = {
	.name = "PCI",
	.irq_enable = irq_pci_enable,
	.irq_disable = irq_pci_disable,
	.irq_ack = irq_pci_ack,
	.irq_mask = irq_pci_mask,
	.irq_unmask = irq_pci_unmask,
};

static void irq_pci_msi_enable(struct irq_data *data)
{
	__raw_writel(data->irq, para_irq_chip.base + KVM_MIPSVZ_IC_REG_IRQ_EN);
}

static void irq_pci_msi_disable(struct irq_data *data)
{
	__raw_writel(data->irq, para_irq_chip.base + KVM_MIPSVZ_IC_REG_IRQ_DIS);
}

static void irq_pci_msi_ack(struct irq_data *data)
{
	__raw_writel(data->irq, para_irq_chip.base + KVM_MIPSVZ_IC_REG_IRQ_CLR);
}

static void irq_pci_msi_mask(struct irq_data *data)
{
	__raw_writel(data->irq, para_irq_chip.base + KVM_MIPSVZ_IC_REG_IRQ_DIS);
}

static void irq_pci_msi_unmask(struct irq_data *data)
{
	__raw_writel(data->irq, para_irq_chip.base + KVM_MIPSVZ_IC_REG_IRQ_EN);
}

static struct irq_chip irq_chip_pci_msi = {
	.name = "PCI-MSI",
	.irq_enable = irq_pci_msi_enable,
	.irq_disable = irq_pci_msi_disable,
	.irq_ack = irq_pci_msi_ack,
	.irq_mask = irq_pci_msi_mask,
	.irq_unmask = irq_pci_msi_unmask,
};

static void irq_mbox_enable(struct irq_data *data)
{
	int cpu;
	u32 val;

	for_each_online_cpu(cpu) {
		val = ((u32)cpunum_for_cpu(cpu) << 20) | data->irq;
		__raw_writel(val, para_irq_chip.base + KVM_MIPSVZ_IC_REG_CPU_IRQ_EN);
	}
}

static void irq_mbox_disable(struct irq_data *data)
{
	int cpu;
	u32 val;

	for_each_online_cpu(cpu) {
		val = ((u32)cpunum_for_cpu(cpu) << 20) | data->irq;
		__raw_writel(val, para_irq_chip.base + KVM_MIPSVZ_IC_REG_CPU_IRQ_DIS);
	}
}

/* per CPU only */
static void irq_mbox_ack(struct irq_data *data)
{
	u32 val = (get_ebase_cpunum() << 20) | data->irq;
	__raw_writel(val, para_irq_chip.base + KVM_MIPSVZ_IC_REG_CPU_IRQ_CLR);
}

/* per CPU only */
void irq_mbox_ipi(int cpu, unsigned int action)
{
	u32 val = (u32)cpunum_for_cpu(cpu) << 20;

	switch (action) {
	case SMP_RESCHEDULE_YOURSELF:
		val |= MIPS_IRQ_MBOX0;
		break;
	case SMP_CALL_FUNCTION:
		val |= MIPS_IRQ_MBOX1;
		break;
	case SMP_ICACHE_FLUSH:
		val |= MIPS_IRQ_MBOX2;
		break;
	default:
		pr_err("%s: Unhandled action: %u\n", __func__, action);
		return;
	}

	__raw_writel(val, para_irq_chip.base + KVM_MIPSVZ_IC_REG_CPU_IRQ_SET);
}

/* per CPU only */
static void irq_mbox_cpu_online(struct irq_data *data)
{
	u32 val = (get_ebase_cpunum() << 20) | data->irq;
	__raw_writel(val, para_irq_chip.base + KVM_MIPSVZ_IC_REG_CPU_IRQ_EN);
}

/* per CPU only */
static void irq_mbox_cpu_offline(struct irq_data *data)
{
	u32 val = (get_ebase_cpunum() << 20) | data->irq;
	__raw_writel(val, para_irq_chip.base + KVM_MIPSVZ_IC_REG_CPU_IRQ_DIS);
}

static struct irq_chip irq_chip_mbox = {
	.name = "MBOX",
	.irq_enable = irq_mbox_enable,
	.irq_disable = irq_mbox_disable,
	.irq_ack = irq_mbox_ack,
	.irq_cpu_online = irq_mbox_cpu_online,
	.irq_cpu_offline = irq_mbox_cpu_offline,
	.flags = IRQCHIP_ONOFFLINE_ENABLED,
};

static inline unsigned long cpu_irq_src_bitmap(int cpu)
{
	return para_irq_chip.cpu_irq_src_bitmap + (cpu * para_irq_chip.bm_size);
}

static void __init irq_pci_init(void)
{
	int i;
	struct mipsvz_irq_chip *ic = &para_irq_chip;

	ic->base = ioremap(0x1e010000, 4096);

	ic->numbits = __raw_readl(ic->base + KVM_MIPSVZ_IC_REG_NUM_BITS);
	ic->numcpus = __raw_readl(ic->base + KVM_MIPSVZ_IC_REG_NUM_CPUS);
	ic->version = __raw_readl(ic->base + KVM_MIPSVZ_IC_REG_VERSION);
	ic->bm_length = (ic->numbits + 32 - 1) / 32;
	ic->bm_size = ic->bm_length * 4;

	ic->cpu_irq_src_bitmap = (unsigned long)ic->base + KVM_MIPSVZ_IC_BM_AREA;

	pr_info("(%s) numbits: %d, numcpus: %d, version: %d\n",
		__func__, ic->numbits, ic->numcpus, ic->version);

	for (i = MIPS_IRQ_PCI_BASE; i <= MIPS_IRQ_PCI_MAX; i++) {
		irq_set_chip_and_handler(i, &irq_chip_pci, handle_level_irq);
		/* enable PCI irqs on CPU0 */
		__raw_writel(i, para_irq_chip.base + KVM_MIPSVZ_IC_REG_CPU_IRQ_EN);
	}

	for (i = MIPS_IRQ_MBOX0; i <= MIPS_IRQ_MBOX_MAX ; i++)
		irq_set_chip_and_handler(i, &irq_chip_mbox, handle_percpu_irq);

	for (i = MIPS_IRQ_MSI_BASE; i < ic->numbits; i++) {
		irq_set_chip_and_handler(i, &irq_chip_pci_msi, handle_level_irq);
		/* enable MSI irqs on CPU0 */
		__raw_writel(i, para_irq_chip.base + KVM_MIPSVZ_IC_REG_CPU_IRQ_EN);
	}

	set_c0_status(STATUSF_IP2);
}

static void __init irq_msi_init(void)
{
	struct mipsvz_irq_chip *ic = &para_irq_chip;
	int i;

	irq_msi_map = kzalloc(BITS_TO_LONGS(ic->numbits) * sizeof(long),
				GFP_KERNEL);
	if (!irq_msi_map)
		return;

	for (i=0; i < MIPS_IRQ_MSI_BASE; i++)
		set_bit(i, irq_msi_map);
}

static unsigned int irq_chip_bm_ffs(unsigned long bitmap)
{
	unsigned int i, ret = 0;
	u64 v, h, l;

	for (i = 0; i < para_irq_chip.bm_length / 2; i++) {
		h = __raw_readl((void *)(bitmap + i * 4));
		l = __raw_readl((void *)(bitmap + (i + 1) * 4));
		v = h << 32 | l;

		if (!v)
			continue;

		ret = __ffs(v);
		break;
	}

	return ret;
}

static void irq_pci_dispatch(void)
{
	unsigned int irq;
	struct mipsvz_irq_chip *ic = &para_irq_chip;

	irq = irq_chip_bm_ffs(cpu_irq_src_bitmap(get_ebase_cpunum()));

	if (9 <= irq && irq < ic->numbits)
		do_IRQ(irq);
	else
		spurious_interrupt();
}

void __init arch_init_irq(void)
{
	irq_init_core();
	irq_pci_init();
	irq_msi_init();
}

asmlinkage void plat_irq_dispatch(void)
{
	unsigned int pending = read_c0_cause() & read_c0_status() & ST0_IM;
	int ip;

	if (unlikely(!pending)) {
		spurious_interrupt();
		return;
	}

	ip = ffs(pending) - 1 - STATUSB_IP0;
	if (ip == 2)
		irq_pci_dispatch();
	else
		do_IRQ(MIPS_CPU_IRQ_BASE + ip);
}

static int get_msi_nr(void)
{
	struct mipsvz_irq_chip *ic = &para_irq_chip;
	int irq;
	unsigned long flags;

	spin_lock_irqsave(&irq_msi_map_lock, flags);
	irq = find_first_zero_bit(irq_msi_map, ic->numbits);
	if (irq == ic->numbits)
		return -ENOSPC;
	set_bit(irq, irq_msi_map);
	spin_unlock_irqrestore(&irq_msi_map_lock, flags);

	return irq;
}

static void put_msi_nr(int irq)
{
	struct mipsvz_irq_chip *ic = &para_irq_chip;
	unsigned long flags;

	if (irq < MIPS_IRQ_MSI_BASE || irq >= ic->numbits)
		return;

	spin_lock_irqsave(&irq_msi_map_lock, flags);
	clear_bit(irq, irq_msi_map);
	spin_unlock_irqrestore(&irq_msi_map_lock, flags);
}

int arch_setup_msi_irq(struct pci_dev *dev, struct msi_desc *desc)
{
	struct msi_msg msg;
	int irq;

	irq = get_msi_nr();
	if (irq < 0)
		return -ENOSPC;

	pr_info("Setting up irq %d for MSI\n", irq);

	irq_set_msi_desc(irq, desc);
	msg.data = irq;
	write_msi_msg(irq, &msg);

	return 0;
}

void arch_teardown_msi_irq(unsigned int irq)
{
	pr_info("Releasing MSI irq %d\n", irq);
	put_msi_nr(irq);
}

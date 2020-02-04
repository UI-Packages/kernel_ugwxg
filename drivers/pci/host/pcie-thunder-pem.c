/*
 * PCIe host controller driver for Cavium Thunder SOC
 *
 * Copyright (C) 2014,2015 Cavium Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */
/* #define DEBUG 1 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/pci.h>
#include <linux/irqdomain.h>
#include <linux/msi.h>

#define THUNDER_SLI_S2M_REG_ACC_BASE	0x874001000000ull

#define THUNDER_GIC			0x801000000000ull
#define THUNDER_GICD_SETSPI_NSR		0x801000000040ull
#define THUNDER_GICD_CLRSPI_NSR		0x801000000048ull

#define THUNDER_GSER_PCIE_MASK		0x01

#define PEM_CTL_STATUS	0x000
#define PEM_RD_CFG	0x030
#define P2N_BAR0_START	0x080
#define P2N_BAR1_START	0x088
#define P2N_BAR2_START	0x090
#define BAR_CTL		0x0a8
#define BAR2_MASK	0x0b0
#define BAR1_INDEX	0x100
#define PEM_CFG		0x410
#define PEM_ON		0x420

struct thunder_pem {
	struct list_head list; /* on thunder_pem_buses */
	bool		connected;
	unsigned int	id;
	unsigned int	sli;
	unsigned int	sli_group;
	unsigned int	node;
	u64		sli_window_base;
	void __iomem	*bar0;
	void __iomem	*bar4;
	void __iomem	*sli_s2m;
	void __iomem	*cfgregion;
	struct pci_bus	*bus;
	int		vwire_irqs[4];
	u32		vwire_data[4];
};

static LIST_HEAD(thunder_pem_buses);

static struct pci_device_id thunder_pem_pci_table[] = {
	{PCI_VENDOR_ID_CAVIUM, 0xa020, PCI_ANY_ID, PCI_ANY_ID, 0, 0, 0},
	{0,}
};
MODULE_DEVICE_TABLE(pci, thunder_pem_pci_table);

struct irq_domain *gic_get_irq_domain(void);

enum slix_s2m_ctype {
	CTYPE_MEMORY	= 0,
	CTYPE_CONFIG	= 1,
	CTYPE_IO	= 2
};

static u64 slix_s2m_reg_val(unsigned mac, enum slix_s2m_ctype ctype,
			    bool merge, bool relaxed, bool snoop, u32 ba_msb)
{
	u64 v;

	v = (u64)(mac % 3) << 49;
	v |= (u64)ctype << 53;
	if (!merge)
		v |= 1ull << 48;
	if (relaxed)
		v |= 5ull << 40;
	if (!snoop)
		v |= 5ull << 41;
	v |= (u64)ba_msb;

	return v;
}

static u32 thunder_pcierc_config_read(struct thunder_pem *pem, u32 reg, int size)
{
	unsigned int val;

	writeq(reg & ~3u, pem->bar0 + PEM_RD_CFG);
	val = readq(pem->bar0 + PEM_RD_CFG) >> 32;

	if (size == 1)
		val = (val >> (8 * (reg & 3))) & 0xff;
	else if (size == 2)
		val = (val >> (8 * (reg & 3))) & 0xffff;

	return val;
}

static int thunder_pem_read_config(struct pci_bus *bus, unsigned int devfn,
				   int reg, int size, u32 *val)
{
	void __iomem *addr;
	struct thunder_pem *pem = bus->sysdata;
	unsigned int busnr = bus->number;

	if (busnr > 255 || devfn > 255 || reg > 4095)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (PCI_SLOT(devfn)) {
		switch (size) {
		case 1:
			*val = 0xff;
			break;
		case 2:
			*val = 0xffff;
			break;
		case 4:
			*val = 0xffffffff;
			break;
		default:
			return PCIBIOS_BAD_REGISTER_NUMBER;
		}
		return PCIBIOS_SUCCESSFUL;
	}

	addr = pem->cfgregion + ((busnr << 24)  | (devfn << 16) | reg);

	switch (size) {
	case 1:
		*val = readb(addr);
		break;
	case 2:
		*val = readw(addr);
		break;
	case 4:
		*val = readl(addr);
		break;
	default:
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int thunder_pem_write_config(struct pci_bus *bus, unsigned int devfn,
				    int reg, int size, u32 val)
{
	void __iomem *addr;
	struct thunder_pem *pem = bus->sysdata;
	unsigned int busnr = bus->number;

	if (busnr > 255 || devfn > 255 || reg > 4095)
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (PCI_SLOT(devfn))
		return PCIBIOS_SUCCESSFUL;

	addr = pem->cfgregion + ((busnr << 24)  | (devfn << 16) | reg);

	switch (size) {
	case 1:
		writeb(val, addr);
		break;
	case 2:
		writew(val, addr);
		break;
	case 4:
		writel(val, addr);
		break;
	default:
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return PCIBIOS_SUCCESSFUL;
}

static struct pci_ops thunder_pem_ops = {
	.read	= thunder_pem_read_config,
	.write	= thunder_pem_write_config,
};

static struct thunder_pem *thunder_pem_from_dev(struct pci_dev *dev)
{
	struct thunder_pem *pem;
	struct pci_bus *bus = dev->bus;

	while (!pci_is_root_bus(bus))
		bus = bus->parent;

	list_for_each_entry(pem, &thunder_pem_buses, list) {
		if (pem->bus == bus)
			return pem;
	}
	return NULL;
}

int thunder_pem_requester_id(struct pci_dev *dev)
{
	struct thunder_pem *pem = thunder_pem_from_dev(dev);

	if (!pem)
		return -ENODEV;

	if (pem->id < 3)
		return ((1 << 16) |
			((dev)->bus->number << 8) |
			(dev)->devfn);

	if (pem->id < 6)
		return ((3 << 16) |
			((dev)->bus->number << 8) |
			(dev)->devfn);

	if (pem->id < 9)
		return ((1 << 19) | (1 << 16) |
			((dev)->bus->number << 8) |
			(dev)->devfn);

	if (pem->id < 12)
		return ((1 << 19) |
			(3 << 16) |
			((dev)->bus->number << 8) |
			(dev)->devfn);
	return -ENODEV;
}

int pcibios_add_device(struct pci_dev *dev)
{
	struct thunder_pem *pem;
	u8 pin;

	pem = thunder_pem_from_dev(dev);
	if (!pem)
		return 0;

	pci_read_config_byte(dev, PCI_INTERRUPT_PIN, &pin);

	/* Cope with illegal. */
	if (pin > 4)
		pin = 1;

	dev->irq = pin > 0 ? pem->vwire_irqs[pin - 1] : 0;

	if (pin)
		dev_dbg(&dev->dev, "assigning IRQ %02d\n", dev->irq);

	pci_write_config_byte(dev, PCI_INTERRUPT_LINE, dev->irq);

	return 0;
}

static int thunder_pem_pci_probe(struct pci_dev *pdev,
				 const struct pci_device_id *ent)
{
	struct thunder_pem *pem;
	resource_size_t bar0_start;
	u64 regval;
	u64 sliaddr, pciaddr;
	u32 cfgval;
	int primary_bus;
	int i;
	int ret = 0;
	struct resource *res;
	LIST_HEAD(resources);

	pem = devm_kzalloc(&pdev->dev, sizeof(*pem), GFP_KERNEL);
	if (!pem)
		return -ENOMEM;

	pci_set_drvdata(pdev, pem);

	bar0_start = pci_resource_start(pdev, 0);
	pem->node = (bar0_start >> 44) & 3;
	pem->id = ((bar0_start >> 24) & 7) + (6 * pem->node);
	pem->sli = pem->id % 3;
	pem->sli_group = (pem->id / 3) % 2;
	pem->sli_window_base = 0x880000000000ull | (((u64)pem->node) << 44) | ((u64)pem->sli_group << 40);
	pem->sli_window_base += 0x4000000000 * pem->sli;

	ret = pci_enable_device_mem(pdev);
	if (ret)
		goto out;

	pem->bar0 = pcim_iomap(pdev, 0, 0x100000);
	if (!pem->bar0) {
		ret = -ENOMEM;
		goto out;
	}

	pem->bar4 = pcim_iomap(pdev, 4, 0x100000);
	if (!pem->bar0) {
		ret = -ENOMEM;
		goto out;
	}

	sliaddr = THUNDER_SLI_S2M_REG_ACC_BASE | ((u64)pem->node << 44) | ((u64)pem->sli_group << 36);

	regval = readq(pem->bar0 + PEM_ON);
	if (!(regval & 1)) {
		dev_notice(&pdev->dev, "PEM%u_ON not set, skipping...\n", pem->id);
		goto out;
	}

	regval = readq(pem->bar0 + PEM_CTL_STATUS);
	regval |= 0x10; /* Set Link Enable bit */
	writeq(regval, pem->bar0 + PEM_CTL_STATUS);

	udelay(1000);

	cfgval = thunder_pcierc_config_read(pem, 32 * 4, 4); /* PCIERC_CFG032 */

	if (((cfgval >> 29 & 0x1) == 0x0) || ((cfgval >> 27 & 0x1) == 0x1)) {
		dev_notice(&pdev->dev, "PEM%u Link Timeout, skipping...\n", pem->id);
		goto out;
	}

	pem->sli_s2m = devm_ioremap(&pdev->dev, sliaddr, 0x1000);
	if (!pem->sli_s2m) {
		ret = -ENOMEM;
		goto out;
	}

	pem->cfgregion = devm_ioremap(&pdev->dev, pem->sli_window_base, 0x100000000ull);
	if (!pem->cfgregion) {
		ret = -ENOMEM;
		goto out;
	}
	regval = slix_s2m_reg_val(pem->sli, CTYPE_CONFIG, false, false, false, 0);
	writeq(regval, pem->sli_s2m + 0x10 * ((0x40 * pem->sli) + 0));

	cfgval = thunder_pcierc_config_read(pem, 6 * 4, 4); /* PCIERC_CFG006 */
	primary_bus = (cfgval >> 8) & 0xff;

	res = kzalloc(sizeof(*res), GFP_KERNEL);
	if (!res) {
		ret = -ENOMEM;
		goto out;
	}
	res->start = primary_bus;
	res->end = 255;
	res->flags = IORESOURCE_BUS;
	pci_add_resource(&resources, res);


	res = kzalloc(sizeof(*res), GFP_KERNEL);
	if (!res) {
		ret = -ENOMEM;
		goto out;
	}
	res->start = 0x100000 * pem->id;
	res->end = res->start + 0x100000 - 1;
	res->flags = IORESOURCE_IO;
	pci_add_resource(&resources, res);
	regval = slix_s2m_reg_val(pem->sli, CTYPE_IO, false, false, false, 0);
	writeq(regval, pem->sli_s2m + 0x10 * ((0x40 * pem->sli) + 1));

	res = kzalloc(sizeof(*res), GFP_KERNEL);
	if (!res) {
		ret = -ENOMEM;
		goto out;
	}
	pciaddr = 0x10000000ull;
	res->start = pem->sli_window_base + 0x1000000000ull + pciaddr;
	res->end = res->start + 0x1000000000ull - pciaddr - 1;
	res->flags = IORESOURCE_MEM;
	pci_add_resource_offset(&resources, res, res->start - pciaddr);
	for (i = 0; i < 16; i++) {
		regval = slix_s2m_reg_val(pem->sli, CTYPE_MEMORY, false, false, false, i);
		writeq(regval, pem->sli_s2m + 0x10 * ((0x40 * pem->sli) + (0x10 + i)));
	}

	res = kzalloc(sizeof(*res), GFP_KERNEL);
	if (!res) {
		ret = -ENOMEM;
		goto out;
	}
	pciaddr = 0x1000000000ull;
	res->start = pem->sli_window_base + 0x1000000000ull + pciaddr;
	res->end = res->start + 0x1000000000ull - 1;
	res->flags = IORESOURCE_MEM | IORESOURCE_PREFETCH;
	pci_add_resource_offset(&resources, res, res->start - pciaddr);
	for (i = 0; i < 16; i++) {
		regval = slix_s2m_reg_val(pem->sli, CTYPE_MEMORY, true, true, true, i + 0x10);
		writeq(regval, pem->sli_s2m + 0x10 * ((0x40 * pem->sli) + (0x20 + i)));
	}

	writeq(0, pem->bar0 + P2N_BAR0_START);
	writeq(0, pem->bar0 + P2N_BAR1_START);
	writeq(0, pem->bar0 + P2N_BAR2_START);

	regval = 0x10;	/* BAR_CTL[BAR1_SIZ] = 1 (64MB) */
	regval |= 0x8;	/* BAR_CTL[BAR2_ENB] = 1 */
	writeq(regval, pem->bar0 + BAR_CTL);

	/* 1st 4MB region -> GIC registers so 32-bit MSI can reach the GIC. */
	regval = (THUNDER_GIC + (((u64)pem->node) << 44)) >> 18;
	/* BAR1_INDEX[ADDR_V] = 1 */
	regval |= 1;
	writeq(regval, pem->bar0 + BAR1_INDEX);
	/* Remaining regions linear mapping to physical address space */
	for (i = 1; i < 16; i++) {
		regval = (i << 4) | 1;
		writeq(regval, pem->bar0 + BAR1_INDEX + 8 * i);
	}

	pem->bus = pci_create_root_bus(&pdev->dev, primary_bus, &thunder_pem_ops, pem, &resources);
	if (!pem->bus) {
		ret = -ENODEV;
		goto err_root_bus;
	}
	list_add_tail(&pem->list, &thunder_pem_buses);

	for (i = 0; i < 3; i++) {
		pem->vwire_data[i] = 40 + 4 * pem->id + i;
		pem->vwire_irqs[i] = irq_create_mapping(gic_get_irq_domain(), pem->vwire_data[i]);
		if (!pem->vwire_irqs[i]) {
			dev_err(&pdev->dev, "Error: No irq mapping for %u\n", pem->vwire_data[i]);
			continue;
		}
		irq_set_irq_type(pem->vwire_irqs[i], IRQ_TYPE_LEVEL_HIGH);

		writeq(THUNDER_GICD_SETSPI_NSR,	pem->bar4 + 0 + (i + 2) * 32);
		writeq(pem->vwire_data[i],	pem->bar4 + 8 + (i + 2) * 32);
		writeq(THUNDER_GICD_CLRSPI_NSR,	pem->bar4 + 16 + (i + 2) * 32);
		writeq(pem->vwire_data[i],	pem->bar4 + 24 + (i + 2) * 32);
	}
	ret = pci_read_config_dword(pdev, 44 * 4, &cfgval);
	if (WARN_ON(ret))
		goto err_free_root_bus;
	cfgval &= ~0x40000000; /* Clear FUNM */
	cfgval |= 0x80000000;  /* Set MSIXEN */
	pci_write_config_dword(pdev, 44 * 4, cfgval);
	pem->bus->msi = pdev->bus->msi;

	pci_scan_child_bus(pem->bus);
	pci_bus_add_devices(pem->bus);
	pci_assign_unassigned_root_bus_resources(pem->bus);

	return 0;

err_free_root_bus:
	pci_remove_root_bus(pem->bus);
err_root_bus:
	pci_free_resource_list(&resources);
out:
	return ret;
}

static void thunder_pem_pci_remove(struct pci_dev *pdev)
{
}

static struct pci_driver thunder_pem_driver = {
	.name		= "thunder_pem",
	.id_table	= thunder_pem_pci_table,
	.probe		= thunder_pem_pci_probe,
	.remove		= thunder_pem_pci_remove
};

static int __init thunder_pcie_init(void)
{
	int ret;

	ret = pci_register_driver(&thunder_pem_driver);

	return ret;
}
module_init(thunder_pcie_init);

static void __exit thunder_pcie_exit(void)
{
	pci_unregister_driver(&thunder_pem_driver);
}
module_exit(thunder_pcie_exit);

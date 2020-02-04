/*
 * PCIe host controller driver for Cavium Thunder SOC
 *
 * Copyright (C) 2014, Cavium Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 */
/* #define DEBUG 1 */
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>
#include <linux/of_pci.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/msi.h>

#define PCI_DEVICE_ID_THUNDER_BRIDGE	0xa002

#define THUNDER_PCIE_BUS_SHIFT		20
#define THUNDER_PCIE_DEV_SHIFT		15
#define THUNDER_PCIE_FUNC_SHIFT		12

#define THUNDER_ECAM0_CFG_BASE		0x848000000000
#define THUNDER_ECAM1_CFG_BASE		0x849000000000
#define THUNDER_ECAM2_CFG_BASE		0x84a000000000
#define THUNDER_ECAM3_CFG_BASE		0x84b000000000
#define THUNDER_ECAM4_CFG_BASE		0x948000000000
#define THUNDER_ECAM5_CFG_BASE		0x949000000000
#define THUNDER_ECAM6_CFG_BASE		0x94a000000000
#define THUNDER_ECAM7_CFG_BASE		0x94b000000000

#define THUNDER_GSER_N0_BASE		0x87e090000000
#define THUNDER_GSER_N1_BASE		0x97e090000000
#define THUNDER_GSER_SIZE		0x00000d000000
#define THUNDER_GSER_ADDR_SHIFT		24

#define THUNDER_GSER_CFG		0x80
#define THUNDER_GSER_CFG_PCIE		(1ULL << 0)
#define THUNDER_GSER_CFG_BGX		(1ULL << 2)
#define THUNDER_GSER_CFG_SATA		(1ULL << 5)

void __iomem *gser_base0;
void __iomem *gser_base1;

struct thunder_pcie {
	struct device_node	*node;
	struct device		*dev;
	void __iomem		*cfg_base;
	struct msi_chip		*msi;
	int			ecam;
	bool			valid;
};

int thunder_pem_requester_id(struct pci_dev *dev);

static int pci_requester_id_ecam(struct pci_dev *dev)
{
	return (((pci_domain_nr(dev->bus) >> 2) << 19) |
		((pci_domain_nr(dev->bus) % 4) << 16) |
		(dev->bus->number << 8) | dev->devfn);
}

int pci_requester_id(struct pci_dev *dev)
{
	int ret;

	ret = thunder_pem_requester_id(dev);
	if (ret >= 0)
		return ret;

	return pci_requester_id_ecam(dev);
}
EXPORT_SYMBOL(pci_requester_id);

/*
 * This bridge is just for the sake of supporting ARI for
 * downstream devices. No resources are attached to it.
 * Copy upstream root bus resources to bridge which aide in
 * resource claiming for downstream devices
 */
static void pci_bridge_resource_fixup(struct pci_dev *dev)
{
	struct pci_bus *bus;
	int resno;

	bus = dev->subordinate;
	for (resno = 0; resno < PCI_BRIDGE_RESOURCE_NUM; resno++) {
		bus->resource[resno] = pci_bus_resource_n(bus->parent,
			PCI_BRIDGE_RESOURCE_NUM + resno);
	}

	for (resno = PCI_BRIDGE_RESOURCES;
		resno <= PCI_BRIDGE_RESOURCE_END; resno++) {
		dev->resource[resno].start = dev->resource[resno].end = 0;
		dev->resource[resno].flags = 0;
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_CAVIUM, PCI_DEVICE_ID_THUNDER_BRIDGE,
			pci_bridge_resource_fixup);

/*
 * All PCIe devices in Thunder have fixed resources, shouldn't be reassigned.
 * Also claim the device's valid resources to set 'res->parent' hierarchy.
 */
static void pci_dev_resource_fixup(struct pci_dev *dev)
{
	struct resource *res;
	int resno;

	for (resno = 0; resno < PCI_NUM_RESOURCES; resno++)
		dev->resource[resno].flags |= IORESOURCE_PCI_FIXED;

	for (resno = 0; resno < PCI_BRIDGE_RESOURCES; resno++) {
		res = &dev->resource[resno];
		if (res->parent || !(res->flags & IORESOURCE_MEM))
			continue;
		pci_claim_resource(dev, resno);
	}
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_CAVIUM, PCI_ANY_ID,
			pci_dev_resource_fixup);

static void __iomem *thunder_get_gser_base(int node, int qlm)
{
	void __iomem *base;

	if (!node)
		base = gser_base0;
	else
		base = gser_base1;

	return base + ((u64)qlm << THUNDER_GSER_ADDR_SHIFT);
}

static u64 thunder_get_gser_cfg(int node, int qlm)
{
	return readq(thunder_get_gser_base(node, qlm) + THUNDER_GSER_CFG);
}

static int thunder_pcie_check_ecam_cfg_access(int ecam, unsigned int bus,
					 unsigned int devfn)
{
	int supported = 0;
	u16 bdf = (bus << 8) | devfn;
	u64 gser_cfg;

	if (ecam == 0) {
		switch (bdf) {
		case 0x008:   /* RSL bridge */
		case 0x010:   /* SMMU */
		case 0x030:   /* GPIO */
		case 0x038:   /* MPI */
		case 0x080:   /* USB0 */
		case 0x088:   /* USB1 */
		case 0x0A0:   /* RAD bridge */
		case 0x0A8:   /* ZIP bridge */
		case 0x0B0:   /* DFA bridge */
		case 0x100:   /* MRML */
		case 0x101:   /* RST */
		case 0x103:   /* FUS */
		case 0x104:   /* FUSF */
		case 0x109:   /* L2C */
		case 0x10A:   /* SGPIO */
		case 0x10C:   /* EMM */
		case 0x10D:   /* KEY */
		case 0x10e:   /* MIO_BOOT */
		case 0x130 ... 0x13f: /* L2C */
		case 0x140 ... 0x141: /* UART */
		case 0x144 ... 0x145: /* UART */
		case 0x148:   /* TWSI0 */
		case 0x149:   /* TWSI1 */
		case 0x14A:   /* TWSI2 */
		case 0x14B:   /* TWSI3 */
		case 0x14C:   /* TWSI4 */
		case 0x14D:   /* TWSI5 */

		case 0x150 ... 0x153: /* LMC */
		case 0x158 ... 0x159: /* IOBN */
		case 0x160 ... 0x163: /* OCLA */

		case 0x170:   /* PEM0 */
		case 0x171:   /* PEM1 */
		case 0x172:   /* PEM2 */
		case 0x173:   /* PEM3 */
		case 0x174:   /* PEM4 */
		case 0x175:   /* PEM5 */

		case 0x200:   /* RAD */
		case 0x300:   /* ZIP */
		case 0x400:   /* HFA */
			supported = 1;
			break;

		case 0x180:   /* BGX0 */
			gser_cfg = thunder_get_gser_cfg(0, 0);
			if (gser_cfg & THUNDER_GSER_CFG_BGX)
				supported = 1;
			break;
		case 0x181:   /* BGX1 */
			gser_cfg = thunder_get_gser_cfg(0, 1);
			if (gser_cfg & THUNDER_GSER_CFG_BGX)
				supported = 1;
			break;
		default:
			supported = 0;
		}
	} else if (ecam == 1) {
		switch (bdf) {
		case 0x008:   /* SMMU */
		case 0x020:   /* AHCI0*/
		case 0x028:   /* AHCI1 */
		case 0x030:   /* AHCI2 */
		case 0x038:   /* AHCI3 */
			gser_cfg = thunder_get_gser_cfg(0, 2);
			if (gser_cfg & THUNDER_GSER_CFG_SATA)
				supported = 1;
			break;
		case 0x040:   /* AHCI4 */
		case 0x048:   /* AHCI5 */
		case 0x050:   /* AHCI5 */
		case 0x058:   /* AHCI7 */
			gser_cfg = thunder_get_gser_cfg(0, 3);
			if (gser_cfg & THUNDER_GSER_CFG_SATA)
				supported = 1;
			break;
		case 0x080:   /* PCIRC0 */
		case 0x098:   /* PCIRC1 */
		case 0x0A8:   /* PCIRC2 */
		default:
			supported = 0;
		}
	} else if (ecam == 2) {
		switch (bdf) {
		case 0x008:   /* SMMU */
		case 0x010:   /* NIC Bridge */
		case 0x018:   /* TNS */
		case 0x100:   /* NIC PF */
		case 0x101 ... 0x180:   /* NIC VF */
			supported = 1;
			break;
		default:
			supported = 0;
		}
	} else if (ecam == 3) {
		switch (bdf) {
		case 0x008:   /* SMMU */
			supported = 1;
			break;
		case 0x020:   /* AHCI8 */
		case 0x028:   /* AHCI9*/
		case 0x030:   /* AHCI10 */
		case 0x038:   /* AHCI11 */
			gser_cfg = thunder_get_gser_cfg(0, 6);
			if (gser_cfg & THUNDER_GSER_CFG_SATA)
				supported = 1;
			break;
		case 0x040:   /* AHCI12 */
		case 0x048:   /* AHCI13 */
		case 0x050:   /* AHCI14 */
		case 0x058:   /* AHCI15 */
			gser_cfg = thunder_get_gser_cfg(0, 7);
			if (gser_cfg & THUNDER_GSER_CFG_SATA)
				supported = 1;
			break;
		case 0x080:   /* PCIRC3 */
		case 0x098:   /* PCIRC4 */
		case 0x0A8:   /* PCIRC5 */
		default:
			supported = 0;
		}
	}

	if (ecam == 4) {
		switch (bdf) {
		case 0x008:   /* RSL bridge */
		case 0x010:   /* SMMU */
		case 0x030:   /* GPIO */
		case 0x038:   /* MPI */
		case 0x080:   /* USB0 */
		case 0x088:   /* USB1 */
		case 0x0A0:   /* RAD bridge */
		case 0x0A8:   /* ZIP bridge */
		case 0x0B0:   /* DFA bridge */
		case 0x100:   /* MRML */
		case 0x101:   /* RST */
		case 0x103:   /* FUS */
		case 0x104:   /* FUSF */
		case 0x109:   /* L2C */
		case 0x10A:   /* SGPIO */
		case 0x10C:   /* EMM */
		case 0x10D:   /* KEY */
		case 0x10e:   /* MIO_BOOT */
		case 0x130 ... 0x13f: /* L2C */
		case 0x140 ... 0x141: /* UART */
		case 0x144 ... 0x145: /* UART */
		case 0x148:   /* TWSI0 */
		case 0x149:   /* TWSI1 */
		case 0x14A:   /* TWSI2 */
		case 0x14B:   /* TWSI3 */
		case 0x14C:   /* TWSI4 */
		case 0x14D:   /* TWSI5 */

		case 0x150 ... 0x153: /* LMC */
		case 0x158 ... 0x159: /* IOBN */
		case 0x160 ... 0x163: /* OCLA */

		case 0x170:   /* PEM0 */
		case 0x171:   /* PEM1 */
		case 0x172:   /* PEM2 */
		case 0x173:   /* PEM3 */
		case 0x174:   /* PEM4 */
		case 0x175:   /* PEM5 */

		case 0x200:   /* RAD */
		case 0x300:   /* ZIP */
		case 0x400:   /* HFA */
			supported = 1;
			break;
		case 0x180:   /* BGX0 */
			gser_cfg = thunder_get_gser_cfg(1, 0);
			if (gser_cfg & THUNDER_GSER_CFG_BGX)
				supported = 1;
			break;
		case 0x181:   /* BGX1 */
			gser_cfg = thunder_get_gser_cfg(1, 1);
			if (gser_cfg & THUNDER_GSER_CFG_BGX)
				supported = 1;
			break;
		default:
			supported = 0;
		}
	} else if (ecam == 5) {
		switch (bdf) {
		case 0x008:   /* SMMU */
		case 0x020:   /* AHCI0*/
		case 0x028:   /* AHCI1 */
		case 0x030:   /* AHCI2 */
		case 0x038:   /* AHCI3 */
			gser_cfg = thunder_get_gser_cfg(1, 2);
			if (gser_cfg & THUNDER_GSER_CFG_SATA)
				supported = 1;
			break;
		case 0x040:   /* AHCI4 */
		case 0x048:   /* AHCI5 */
		case 0x050:   /* AHCI5 */
		case 0x058:   /* AHCI7 */
			gser_cfg = thunder_get_gser_cfg(1, 3);
			if (gser_cfg & THUNDER_GSER_CFG_SATA)
				supported = 1;
			break;
		case 0x080:   /* PCIRC0 */
		case 0x098:   /* PCIRC1 */
		case 0x0A8:   /* PCIRC2 */
		default:
			supported = 0;
		}
	} else if (ecam == 6) {
		switch (bdf) {
		case 0x008:   /* SMMU */
		case 0x010:   /* NIC Bridge */
		case 0x100:   /* NIC PF */
		case 0x101:   /* NIC VF */
		case 0x102 ... 0x180:   /* NIC VF */
			supported = 1;
			break;
		default:
			supported = 0;
		}
	} else if (ecam == 7) {
		switch (bdf) {
		case 0x008:   /* SMMU */
			supported = 1;
			break;
		case 0x020:   /* AHCI8 */
		case 0x028:   /* AHCI9*/
		case 0x030:   /* AHCI10 */
		case 0x038:   /* AHCI11 */
			gser_cfg = thunder_get_gser_cfg(1, 6);
			if (gser_cfg & THUNDER_GSER_CFG_SATA)
				supported = 1;
			break;
		case 0x040:   /* AHCI12 */
		case 0x048:   /* AHCI13 */
		case 0x050:   /* AHCI14 */
		case 0x058:   /* AHCI15 */
			gser_cfg = thunder_get_gser_cfg(1, 7);
			if (gser_cfg & THUNDER_GSER_CFG_SATA)
				supported = 1;
			break;
		case 0x080:   /* PCIRC3 */
		case 0x098:   /* PCIRC4 */
		case 0x0A8:   /* PCIRC5 */
		default:
			supported = 0;
		}
	}

	return supported;
}

static void __iomem *thunder_pcie_cfg_base(struct thunder_pcie *pcie,
				 unsigned int bus, unsigned int devfn)
{
	return  pcie->cfg_base + ((bus << THUNDER_PCIE_BUS_SHIFT)
		| (PCI_SLOT(devfn) << THUNDER_PCIE_DEV_SHIFT)
		| (PCI_FUNC(devfn) << THUNDER_PCIE_FUNC_SHIFT));
}

static void __iomem *
thunder_pcie_get_cfg_addr(struct thunder_pcie *pcie, unsigned int busnr,
			unsigned int devfn, int reg)
{
	if (!thunder_pcie_check_ecam_cfg_access(pcie->ecam, busnr, devfn))
		return NULL;
	return thunder_pcie_cfg_base(pcie, busnr, devfn) + reg;
}

static int thunder_pcie_read_config(struct pci_bus *bus, unsigned int devfn,
				  int reg, int size, u32 *val)
{
	struct thunder_pcie *pcie = bus->sysdata;
	void __iomem *addr;
	unsigned int busnr = bus->number;

	if (busnr > 255 || devfn > 255 || reg > 4095)
		return PCIBIOS_DEVICE_NOT_FOUND;

	addr = thunder_pcie_get_cfg_addr(pcie, busnr, devfn, reg);

	switch (size) {
	case 1:
		if (!addr)
			*val = 0xff;
		else
			*val = readb(addr);
		break;
	case 2:
		if (!addr)
			*val = 0xffff;
		else
			*val = readw(addr);
		break;
	case 4:
		if (!addr)
			*val = 0xffffffff;
		else
			*val = readl(addr);
		break;
	default:
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int thunder_pcie_write_config(struct pci_bus *bus, unsigned int devfn,
				  int reg, int size, u32 val)
{
	struct thunder_pcie *pcie = bus->sysdata;
	void __iomem *addr;
	unsigned int busnr = bus->number;

	if (busnr > 255 || devfn > 255 || reg > 4095)
		return PCIBIOS_DEVICE_NOT_FOUND;

	addr = thunder_pcie_get_cfg_addr(pcie, busnr, devfn, reg);

	if (!addr)
		return PCIBIOS_SUCCESSFUL;

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

static struct pci_ops thunder_pcie_ops = {
	.read	= thunder_pcie_read_config,
	.write	= thunder_pcie_write_config,
};

static int thunder_pcie_msi_enable(struct thunder_pcie *pcie,
					struct pci_bus *bus)
{
	struct device_node *msi_node;

	msi_node = of_parse_phandle(pcie->node, "msi-parent", 0);
	if (!msi_node)
		return -ENODEV;

	pcie->msi = of_pci_find_msi_chip_by_node(msi_node);
	if (!pcie->msi)
		return -ENODEV;

	pcie->msi->dev = pcie->dev;
	bus->msi = pcie->msi;

	return 0;
}

static void thunder_pcie_config(struct thunder_pcie *pcie, u64 addr)
{
	pcie->valid = true;

	switch (addr) {
	case THUNDER_ECAM0_CFG_BASE:
		pcie->ecam = 0;
		break;
	case THUNDER_ECAM1_CFG_BASE:
		pcie->ecam = 1;
		break;
	case THUNDER_ECAM2_CFG_BASE:
		pcie->ecam = 2;
		break;
	case THUNDER_ECAM3_CFG_BASE:
		pcie->ecam = 3;
		break;
	case THUNDER_ECAM4_CFG_BASE:
		pcie->ecam = 4;
		break;
	case THUNDER_ECAM5_CFG_BASE:
		pcie->ecam = 5;
		break;
	case THUNDER_ECAM6_CFG_BASE:
		pcie->ecam = 6;
		break;
	case THUNDER_ECAM7_CFG_BASE:
		pcie->ecam = 7;
		break;
	default:
		pcie->valid = false;
	}

	if (gser_base0 == NULL)
		gser_base0 = devm_ioremap(pcie->dev,
					 THUNDER_GSER_N0_BASE,
					 THUNDER_GSER_SIZE);
#ifdef CONFIG_NUMA
	if (gser_base1 == NULL)
		gser_base1 = devm_ioremap(pcie->dev,
					 THUNDER_GSER_N1_BASE,
					 THUNDER_GSER_SIZE);
#endif
}

static int thunder_pcie_probe(struct platform_device *pdev)
{
	struct thunder_pcie *pcie;
	struct resource *cfg_base;
	struct pci_bus *bus;
	int ret = 0;
	int primary_bus = 0;
	LIST_HEAD(res);

	pcie = devm_kzalloc(&pdev->dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pcie->node = of_node_get(pdev->dev.of_node);
	pcie->dev = &pdev->dev;


	/* Get controller's configuration space range */
	cfg_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	thunder_pcie_config(pcie, cfg_base->start);

	pcie->cfg_base = devm_ioremap_resource(&pdev->dev, cfg_base);
	if (IS_ERR(pcie->cfg_base) || IS_ERR(gser_base0)) {
		ret = PTR_ERR(pcie->cfg_base);
		goto err_ioremap;
	}

	pr_err("%s: ECAM%d CFG BASE 0x%llx gser_base0:%llx\n", __func__,
	       pcie->ecam, (u64)cfg_base->start, (u64)gser_base0);

#ifdef CONFIG_NUMA
	if (IS_ERR(gser_base1)) {
		ret = PTR_ERR(pcie->cfg_base);
		goto err_ioremap;
	}
	pr_err("%s: ECAM%d CFG BASE 0x%llx gser_base1:%llx\n", __func__,
	       pcie->ecam, (u64)cfg_base->start, (u64)gser_base1);
#endif
	ret = of_pci_get_host_bridge_resources(pdev->dev.of_node,
					       0, 255, &res, NULL);
	if (ret)
		goto err_root_bus;

	bus = pci_create_root_bus(&pdev->dev, primary_bus, &thunder_pcie_ops, pcie, &res);
	if (!bus) {
		ret = -ENODEV;
		goto err_root_bus;
	}

	/* Set reference to MSI chip */
	ret = thunder_pcie_msi_enable(pcie, bus);
	if (ret) {
		pr_err("%s: Unable to set reference to MSI chip: ret=%d\n", __func__, ret);
		goto err_msi;
	}

	platform_set_drvdata(pdev, pcie);

	pci_scan_child_bus(bus);
	pci_bus_add_devices(bus);

	return 0;
err_msi:
	pci_remove_root_bus(bus);
err_root_bus:
	pci_free_resource_list(&res);
err_ioremap:
	of_node_put(pcie->node);
	return ret;
}

static const struct of_device_id thunder_pcie_of_match[] = {
	{ .compatible = "cavium,thunder-pcie", },
	{},
};
MODULE_DEVICE_TABLE(of, thunder_pcie_of_match);

static struct platform_driver thunder_pcie_driver = {
	.driver = {
		.name = "thunder-pcie",
		.owner = THIS_MODULE,
		.of_match_table = thunder_pcie_of_match,
	},
	.probe = thunder_pcie_probe,
};
module_platform_driver(thunder_pcie_driver);

#ifdef CONFIG_ACPI

static int
thunder_mmcfg_read_config(struct pci_mmcfg_region *cfg, unsigned int busnr,
			unsigned int devfn, int reg, int len, u32 *value)
{
	struct thunder_pcie *pcie = cfg->data;
	void __iomem *addr;

	if (!pcie->valid) {
		/* Not support for now */
		pr_err("RC PEM not supported !!!\n");
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	addr = thunder_pcie_get_cfg_addr(pcie, busnr, devfn, reg);

	switch (len) {
	case 1:
		if (!addr)
			*value = 0xff;
		else
			*value = readb(addr);
		break;
	case 2:
		if (!addr)
			*value = 0xffff;
		else
			*value = readw(addr);
		break;
	case 4:
		if (!addr)
			*value = 0xffffffff;
		else
			*value = readl(addr);
		break;
	default:
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return PCIBIOS_SUCCESSFUL;
}

static int thunder_mmcfg_write_config(struct pci_mmcfg_region *cfg,
		unsigned int busnr, unsigned int devfn, int reg, int len,
		u32 value) {
	struct thunder_pcie *pcie = cfg->data;
	void __iomem *addr;

	if (!pcie->valid) {
		/* Not support for now */
		pr_err("RC PEM not supported !!!\n");
		return PCIBIOS_DEVICE_NOT_FOUND;
	}

	addr = thunder_pcie_get_cfg_addr(pcie, busnr, devfn, reg);

	if (!addr)
		return PCIBIOS_SUCCESSFUL;

	switch (len) {
	case 1:
		writeb(value, addr);
		break;
	case 2:
		writew(value, addr);
		break;
	case 4:
		writel(value, addr);
		break;
	default:
		return PCIBIOS_BAD_REGISTER_NUMBER;
	}

	return PCIBIOS_SUCCESSFUL;
}

int thunder_acpi_mcfg_fixup(struct acpi_pci_root *root,
			    struct pci_mmcfg_region *cfg)
{
	struct thunder_pcie *pcie;

	pcie = kzalloc(sizeof(*pcie), GFP_KERNEL);
	if (!pcie) {
		dev_err(&root->device->dev,
			"pci_bus %04x:%02x: ignored (out of memory)\n",
			(int)root->segment, (int)root->secondary.start);
		return -ENOMEM;
	}

	pcie->dev = &root->device->dev;

	thunder_pcie_config(pcie, cfg->address);

	pcie->cfg_base = cfg->virt;
	cfg->data = pcie;
	cfg->read = thunder_mmcfg_read_config;
	cfg->write = thunder_mmcfg_write_config;

	return 0;
}
DECLARE_ACPI_MCFG_FIXUP("CAVIUM", "THUNDERX", thunder_acpi_mcfg_fixup);
#endif

MODULE_AUTHOR("Sunil Goutham");
MODULE_DESCRIPTION("Cavium Thunder PCIe host controller driver");
MODULE_LICENSE("GPL v2");

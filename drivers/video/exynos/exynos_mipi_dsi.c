/* linux/drivers/video/exynos/exynos_mipi_dsi.c
 *
 * Samsung SoC MIPI-DSIM driver.
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd
 *
 * InKi Dae, <inki.dae@samsung.com>
 * Donghwa Lee, <dh09.lee@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/fb.h>
#include <linux/ctype.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/memory.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/notifier.h>
#include <linux/regulator/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/err.h>
#include <linux/lcd.h>

#include <video/exynos_mipi_dsim.h>

#include "exynos_mipi_dsi_common.h"
#include "exynos_mipi_dsi_lowlevel.h"

struct mipi_dsim_ddi {
	int				bus_id;
	struct list_head		list;
	struct device_node		*ofnode_dsim_lcd_dev;
	struct device_node		*ofnode_dsim_dphy;
	struct mipi_dsim_lcd_device	*dsim_lcd_dev;
	struct mipi_dsim_lcd_driver	*dsim_lcd_drv;
};

static LIST_HEAD(dsim_ddi_list);

static DEFINE_MUTEX(mipi_dsim_lock);

static struct mipi_dsim_platform_data *to_dsim_plat(struct platform_device
							*pdev)
{
	return pdev->dev.platform_data;
}

static struct regulator_bulk_data supplies[] = {
	{ .supply = "vdd11", },
	{ .supply = "vdd18", },
};

static int exynos_mipi_regulator_enable(struct mipi_dsim_device *dsim)
{
	int ret;

	mutex_lock(&dsim->lock);
	ret = regulator_bulk_enable(ARRAY_SIZE(supplies), supplies);
	mutex_unlock(&dsim->lock);

	return ret;
}

static int exynos_mipi_regulator_disable(struct mipi_dsim_device *dsim)
{
	int ret;

	mutex_lock(&dsim->lock);
	ret = regulator_bulk_disable(ARRAY_SIZE(supplies), supplies);
	mutex_unlock(&dsim->lock);

	return ret;
}

/* update all register settings to MIPI DSI controller. */
static void exynos_mipi_update_cfg(struct mipi_dsim_device *dsim)
{
	/*
	 * data from Display controller(FIMD) is not transferred in video mode
	 * but in case of command mode, all settings is not updated to
	 * registers.
	 */
	exynos_mipi_dsi_stand_by(dsim, 0);

	exynos_mipi_dsi_init_dsim(dsim);
	exynos_mipi_dsi_init_link(dsim);

	exynos_mipi_dsi_set_hs_enable(dsim);

	/* set display timing. */
	exynos_mipi_dsi_set_display_mode(dsim, dsim->dsim_config);

	exynos_mipi_dsi_init_interrupt(dsim);

	/*
	 * data from Display controller(FIMD) is transferred in video mode
	 * but in case of command mode, all settings are updated to registers.
	 */
	exynos_mipi_dsi_stand_by(dsim, 1);
}

static int exynos_mipi_dsi_early_blank_mode(struct mipi_dsim_device *dsim,
		int power)
{
	struct mipi_dsim_lcd_driver *client_drv = dsim->dsim_lcd_drv;
	struct mipi_dsim_lcd_device *client_dev = dsim->dsim_lcd_dev;

	switch (power) {
	case FB_BLANK_POWERDOWN:
		if (dsim->suspended)
			return 0;

		if (client_drv && client_drv->suspend)
			client_drv->suspend(client_dev);

		clk_disable(dsim->clock);

		exynos_mipi_regulator_disable(dsim);

		dsim->suspended = true;

		break;
	default:
		break;
	}

	return 0;
}

static int exynos_mipi_dsi_blank_mode(struct mipi_dsim_device *dsim, int power)
{
	struct platform_device *pdev = to_platform_device(dsim->dev);
	struct mipi_dsim_lcd_driver *client_drv = dsim->dsim_lcd_drv;
	struct mipi_dsim_lcd_device *client_dev = dsim->dsim_lcd_dev;

	switch (power) {
	case FB_BLANK_UNBLANK:
		if (!dsim->suspended)
			return 0;

		/* lcd panel power on. */
		if (client_drv && client_drv->power_on)
			client_drv->power_on(client_dev, 1);

		exynos_mipi_regulator_enable(dsim);

		/* enable MIPI-DSI PHY. */
		if (dsim->pd->phy_enable)
			dsim->pd->phy_enable(pdev, true);

		clk_enable(dsim->clock);

		exynos_mipi_update_cfg(dsim);

		/* set lcd panel sequence commands. */
		if (client_drv && client_drv->set_sequence)
			client_drv->set_sequence(client_dev);

		dsim->suspended = false;

		break;
	case FB_BLANK_NORMAL:
		/* TODO. */
		break;
	default:
		break;
	}

	return 0;
}

struct mipi_dsim_ddi *exynos_mipi_dsi_find_lcd_driver(
			struct mipi_dsim_lcd_device *lcd_dev)
{
	struct mipi_dsim_ddi *dsim_ddi, *next;
	struct mipi_dsim_lcd_driver *lcd_drv;

	mutex_lock(&mipi_dsim_lock);

	list_for_each_entry_safe(dsim_ddi, next, &dsim_ddi_list, list) {
		if (!dsim_ddi)
			goto out;

		lcd_drv = dsim_ddi->dsim_lcd_drv;
		if (!lcd_drv)
			continue;

		if ((strcmp(lcd_dev->name, lcd_drv->name)) == 0) {

			mutex_unlock(&mipi_dsim_lock);
			return dsim_ddi;
		}

		list_del(&dsim_ddi->list);
		kfree(dsim_ddi);
	}

out:
	mutex_unlock(&mipi_dsim_lock);
	return NULL;
}

int exynos_mipi_dsi_register_lcd_device(struct mipi_dsim_lcd_device *lcd_dev)
{
	struct mipi_dsim_ddi *dsim_ddi;

	if (!lcd_dev->name) {
		pr_err("dsim_lcd_device name is NULL.\n");
		return -EFAULT;
	}

	dsim_ddi = exynos_mipi_dsi_find_lcd_driver(lcd_dev);
	if (!dsim_ddi) {
		dsim_ddi = kzalloc(sizeof(struct mipi_dsim_ddi), GFP_KERNEL);
		if (!dsim_ddi) {
			pr_err("failed to allocate dsim_ddi object.\n");
			return -ENOMEM;
		}
		mutex_lock(&mipi_dsim_lock);
		list_add_tail(&dsim_ddi->list, &dsim_ddi_list);
		mutex_unlock(&mipi_dsim_lock);
	}

	dsim_ddi->dsim_lcd_dev = lcd_dev;
	dsim_ddi->bus_id = lcd_dev->bus_id;

	return 0;
}

static struct mipi_dsim_ddi *exynos_mipi_dsi_find_lcd_device(
					struct mipi_dsim_lcd_driver *lcd_drv)
{
	struct mipi_dsim_ddi *dsim_ddi, *next;
	struct mipi_dsim_lcd_device *lcd_dev;

	mutex_lock(&mipi_dsim_lock);

	list_for_each_entry_safe(dsim_ddi, next, &dsim_ddi_list, list) {
		if (!dsim_ddi)
			goto out;

		lcd_dev = dsim_ddi->dsim_lcd_dev;
		if (!lcd_dev)
			continue;

		if ((strcmp(lcd_drv->name, lcd_dev->name)) == 0) {
			/**
			 * bus_id would be used to identify
			 * connected bus.
			 */
			dsim_ddi->bus_id = lcd_dev->bus_id;
			mutex_unlock(&mipi_dsim_lock);

			return dsim_ddi;
		}

		list_del(&dsim_ddi->list);
		kfree(dsim_ddi);
	}

out:
	mutex_unlock(&mipi_dsim_lock);

	return NULL;
}

int exynos_mipi_dsi_register_lcd_driver(struct mipi_dsim_lcd_driver *lcd_drv)
{
	struct mipi_dsim_ddi *dsim_ddi;

	if (!lcd_drv->name) {
		pr_err("dsim_lcd_driver name is NULL.\n");
		return -EFAULT;
	}

	dsim_ddi = exynos_mipi_dsi_find_lcd_device(lcd_drv);
	if (!dsim_ddi) {
		/*
		 * If driver specific device is not registered then create a
		 * dsim_ddi object, fill the driver information and add to the
		 * end of the dsim_ddi_list list
		 */
		dsim_ddi = kzalloc(sizeof(struct mipi_dsim_ddi), GFP_KERNEL);
		if (!dsim_ddi) {
			pr_err("failed to allocate dsim_ddi object.\n");
			return -ENOMEM;
		}

		dsim_ddi->dsim_lcd_drv = lcd_drv;

		mutex_lock(&mipi_dsim_lock);
		list_add_tail(&dsim_ddi->list, &dsim_ddi_list);
		mutex_unlock(&mipi_dsim_lock);

	} else {
		dsim_ddi->dsim_lcd_drv = lcd_drv;
	}

	pr_info("registered panel driver(%s) to mipi-dsi driver.\n",
		lcd_drv->name);

	return 0;

}

static struct mipi_dsim_ddi *exynos_mipi_dsi_bind_lcd_ddi(
						struct mipi_dsim_device *dsim,
						const char *name)
{
	struct mipi_dsim_ddi *dsim_ddi, *next;
	struct mipi_dsim_lcd_driver *lcd_drv;
	struct mipi_dsim_lcd_device *lcd_dev;
	int ret;

	mutex_lock(&dsim->lock);

	list_for_each_entry_safe(dsim_ddi, next, &dsim_ddi_list, list) {
		lcd_drv = dsim_ddi->dsim_lcd_drv;
		lcd_dev = dsim_ddi->dsim_lcd_dev;
		if (!lcd_drv || !lcd_dev ||
			(dsim->id != dsim_ddi->bus_id))
				continue;

		dev_dbg(dsim->dev, "lcd_drv->id = %d, lcd_dev->id = %d\n",
				lcd_drv->id, lcd_dev->id);
		dev_dbg(dsim->dev, "lcd_dev->bus_id = %d, dsim->id = %d\n",
				lcd_dev->bus_id, dsim->id);

		if ((strcmp(lcd_drv->name, name) == 0)) {
			lcd_dev->master = dsim;

			lcd_dev->dev.parent = dsim->dev;
			dev_set_name(&lcd_dev->dev, "%s", lcd_drv->name);

			ret = device_register(&lcd_dev->dev);
			if (ret < 0) {
				dev_err(dsim->dev,
					"can't register %s, status %d\n",
					dev_name(&lcd_dev->dev), ret);
				mutex_unlock(&dsim->lock);

				return NULL;
			}

			dsim->dsim_lcd_dev = lcd_dev;
			dsim->dsim_lcd_drv = lcd_drv;

			mutex_unlock(&dsim->lock);

			return dsim_ddi;
		}
	}

	mutex_unlock(&dsim->lock);

	return NULL;
}

/* define MIPI-DSI Master operations. */
static struct mipi_dsim_master_ops master_ops = {
	.cmd_read			= exynos_mipi_dsi_rd_data,
	.cmd_write			= exynos_mipi_dsi_wr_data,
	.get_dsim_frame_done		= exynos_mipi_dsi_get_frame_done_status,
	.clear_dsim_frame_done		= exynos_mipi_dsi_clear_frame_done,
	.set_early_blank_mode		= exynos_mipi_dsi_early_blank_mode,
	.set_blank_mode			= exynos_mipi_dsi_blank_mode,
};

struct device_node *exynos_mipi_find_ofnode_dsim_phy(
				struct platform_device *pdev)
{
	struct device_node *dn, *dn_dphy;
	const __be32 *prop;

	dn = pdev->dev.of_node;
	prop = of_get_property(dn, "mipi-phy", NULL);
	if (NULL == prop) {
		dev_err(&pdev->dev, "Could not find property mipi-phy\n");
		return NULL;
	}

	dn_dphy = of_find_node_by_phandle(be32_to_cpup(prop));
	if (NULL == dn_dphy) {
		dev_err(&pdev->dev, "Could not find node\n");
		return NULL;
	}

	return dn_dphy;
}

struct device_node *exynos_mipi_find_ofnode_lcd_device(
			struct platform_device *pdev)
{
	struct device_node *dn, *dn_lcd_panel;
	const __be32 *prop;

	dn = pdev->dev.of_node;
	prop = of_get_property(dn, "mipi-lcd", NULL);
	if (NULL == prop) {
		dev_err(&pdev->dev, "could not find property mipi-lcd\n");
		return NULL;
	}

	dn_lcd_panel = of_find_node_by_phandle(be32_to_cpup(prop));
	if (NULL == dn_lcd_panel) {
		dev_err(&pdev->dev, "could not find node\n");
		return NULL;
	}

	return dn_lcd_panel;
}

static void exynos_mipi_dsim_enable_d_phy_type1(
			struct platform_device *pdev,
			bool enable)
{
	struct mipi_dsim_device *dsim = (struct mipi_dsim_device *)
					platform_get_drvdata(pdev);
	struct mipi_dsim_phy_config_type1 *dphy_cfg_type1 =
					&dsim->dsim_phy_config->phy_cfg_type1;
	u32 reg_enable;

	reg_enable = __raw_readl(dphy_cfg_type1->reg_enable_dphy);
	reg_enable &= ~(dphy_cfg_type1->ctrlbit_enable_dphy);

	if (enable)
		reg_enable |= dphy_cfg_type1->ctrlbit_enable_dphy;

	__raw_writel(reg_enable, dphy_cfg_type1->reg_enable_dphy);
}

static void exynos_mipi_dsim_reset_type1(
			struct platform_device *pdev,
			bool enable)
{
	struct mipi_dsim_device *dsim = (struct mipi_dsim_device *)
					platform_get_drvdata(pdev);
	struct mipi_dsim_phy_config_type1 *dphy_cfg_type1 =
					&dsim->dsim_phy_config->phy_cfg_type1;
	u32 reg_reset;

	reg_reset = __raw_readl(dphy_cfg_type1->reg_reset_dsim);
	reg_reset &= ~(dphy_cfg_type1->ctrlbit_reset_dsim);

	if (enable)
		reg_reset |= dphy_cfg_type1->ctrlbit_reset_dsim;

	__raw_writel(reg_reset, dphy_cfg_type1->reg_reset_dsim);
}

static int exynos_mipi_dsim_phy_init_type1(
			struct platform_device *pdev,
			bool on_off)
{
	exynos_mipi_dsim_enable_d_phy_type1(pdev, on_off);
	exynos_mipi_dsim_reset_type1(pdev, on_off);
	return 0;
}

static int exynos_mipi_parse_ofnode_dsim_phy_type1(
		struct platform_device *pdev,
		struct mipi_dsim_phy_config_type1 *dphy_cfg_type1,
		struct device_node *np)
{
	struct mipi_dsim_device *dsim = (struct mipi_dsim_device *)
					platform_get_drvdata(pdev);
	u32 paddr_phy_enable, paddr_dsim_reset;

	if (of_property_read_u32(np, "reg_enable_dphy", &paddr_phy_enable))
		return -EINVAL;

	dphy_cfg_type1->reg_enable_dphy = ioremap(paddr_phy_enable, SZ_4);
	if (!dphy_cfg_type1->reg_enable_dphy)
		return -EINVAL;

	if (of_property_read_u32(np, "reg_reset_dsim", &paddr_dsim_reset))
		return -EINVAL;

	dphy_cfg_type1->reg_reset_dsim = ioremap(paddr_dsim_reset, SZ_4);
	if (!dphy_cfg_type1->reg_reset_dsim)
		goto err_ioremap_01;

	if (of_property_read_u32(np, "mask_enable_dphy",
					&dphy_cfg_type1->ctrlbit_enable_dphy))
		goto err_ioremap_02;

	if (of_property_read_u32(np, "mask_reset_dsim",
					&dphy_cfg_type1->ctrlbit_reset_dsim))
		goto err_ioremap_02;

	dsim->pd->phy_enable = exynos_mipi_dsim_phy_init_type1;

	return 0;

err_ioremap_02:
	iounmap(dphy_cfg_type1->reg_reset_dsim);

err_ioremap_01:
	iounmap(dphy_cfg_type1->reg_enable_dphy);
	return -EINVAL;
}

static struct mipi_dsim_phy_config *exynos_mipi_parse_ofnode_dsim_phy(
		struct platform_device *pdev,
		struct device_node *np)
{
	struct mipi_dsim_phy_config *mipi_dphy_config;
	const char *compatible;

	mipi_dphy_config = devm_kzalloc(&pdev->dev,
			sizeof(struct mipi_dsim_phy_config), GFP_KERNEL);
	if (!mipi_dphy_config) {
		dev_err(&pdev->dev,
			"failed to allocate mipi_dsim_phy_config object.\n");
		return NULL;
	}

	if (of_property_read_string(np, "compatible", &compatible)) {
		dev_err(&pdev->dev, "compatible property not found");
		return NULL;
	}

	if (!strcmp(compatible, "samsung-exynos,mipi-phy-type1"))
		mipi_dphy_config->type = MIPI_DSIM_PHY_CONFIG_TYPE1;
	else
		mipi_dphy_config->type = -1;

	switch (mipi_dphy_config->type) {
	case MIPI_DSIM_PHY_CONFIG_TYPE1:
		if (exynos_mipi_parse_ofnode_dsim_phy_type1(
			pdev, &mipi_dphy_config->phy_cfg_type1, np))
			return NULL;
		break;
	default:
		dev_err(&pdev->dev, "mipi phy - unknown type");
		return NULL;
	}

	return mipi_dphy_config;
}

static struct mipi_dsim_lcd_device *exynos_mipi_parse_ofnode_lcd(
		struct platform_device *pdev, struct device_node *np)
{
	struct mipi_dsim_lcd_device *active_mipi_dsim_lcd_device;
	struct lcd_platform_data *active_lcd_platform_data;
	const char *lcd_name;

	active_mipi_dsim_lcd_device = devm_kzalloc(&pdev->dev,
			sizeof(struct mipi_dsim_lcd_device), GFP_KERNEL);
	if (!active_mipi_dsim_lcd_device) {
		dev_err(&pdev->dev,
		"failed to allocate active_mipi_dsim_lcd_device object.\n");
		return NULL;
	}

	if (of_property_read_string(np, "lcd-name", &lcd_name)) {
		dev_err(&pdev->dev, "lcd name property not found");
		return NULL;
	}

	active_mipi_dsim_lcd_device->name = (char *)lcd_name;

	if (of_property_read_u32(np, "id", &active_mipi_dsim_lcd_device->id))
		active_mipi_dsim_lcd_device->id = -1;

	if (of_property_read_u32(np, "bus-id",
					&active_mipi_dsim_lcd_device->bus_id))
		active_mipi_dsim_lcd_device->bus_id = -1;

	active_lcd_platform_data = devm_kzalloc(&pdev->dev,
				sizeof(struct lcd_platform_data), GFP_KERNEL);
	if (!active_lcd_platform_data) {
		dev_err(&pdev->dev,
		"failed to allocate active_lcd_platform_data object.\n");
		return NULL;
	}

	/* store the lcd node pointer for futher use in lcd driver */
	active_lcd_platform_data->pdata = (void *) np;
	active_mipi_dsim_lcd_device->platform_data =
				(void *)active_lcd_platform_data;

	return active_mipi_dsim_lcd_device;
}

static int exynos_mipi_parse_ofnode_config(struct platform_device *pdev,
		struct device_node *np, struct mipi_dsim_config *dsim_config)
{
	unsigned int u32Val;

	if (of_property_read_u32(np, "e_interface", &u32Val)) {
		dev_err(&pdev->dev, "e_interface property not found\n");
		return -EINVAL;
	}
	dsim_config->e_interface = (enum mipi_dsim_interface_type)u32Val;

	if (of_property_read_u32(np, "e_pixel_format", &u32Val)) {
		dev_err(&pdev->dev, "e_pixel_format property not found\n");
		return -EINVAL;
	}
	dsim_config->e_pixel_format = (enum mipi_dsim_pixel_format)u32Val;

	if (of_property_read_u32(np, "auto_flush", &u32Val)) {
		dev_err(&pdev->dev, "auto_flush property not found\n");
		return -EINVAL;
	}
	dsim_config->auto_flush = (unsigned char)u32Val;

	if (of_property_read_u32(np, "eot_disable", &u32Val)) {
		dev_err(&pdev->dev, "eot_disable property not found\n");
		return -EINVAL;
	}
	dsim_config->eot_disable = (unsigned char)u32Val;

	if (of_property_read_u32(np, "auto_vertical_cnt", &u32Val)) {
		dev_err(&pdev->dev, "auto_vertical_cnt property not found\n");
		return -EINVAL;
	}
	dsim_config->auto_vertical_cnt = (unsigned char)u32Val;

	if (of_property_read_u32(np, "hse", &u32Val)) {
		dev_err(&pdev->dev, "hse property not found\n");
		return -EINVAL;
	}
	dsim_config->hse = (unsigned char)u32Val;

	if (of_property_read_u32(np, "hfp", &u32Val)) {
		dev_err(&pdev->dev, "hfp property not found\n");
		return -EINVAL;
	}
	dsim_config->hfp = (unsigned char)u32Val;

	if (of_property_read_u32(np, "hbp", &u32Val)) {
		dev_err(&pdev->dev, "hbp property not found\n");
		return -EINVAL;
	}
	dsim_config->hbp = (unsigned char)u32Val;

	if (of_property_read_u32(np, "hsa", &u32Val)) {
		dev_err(&pdev->dev, "hsa property not found\n");
		return -EINVAL;
	}
	dsim_config->hsa = (unsigned char)u32Val;

	if (of_property_read_u32(np, "e_no_data_lane", &u32Val)) {
		dev_err(&pdev->dev, "e_no_data_lane property not found\n");
		return -EINVAL;
	}
	dsim_config->e_no_data_lane = (enum mipi_dsim_no_of_data_lane)u32Val;

	if (of_property_read_u32(np, "e_byte_clk", &u32Val)) {
		dev_err(&pdev->dev, "e_byte_clk property not found\n");
		return -EINVAL;
	}
	dsim_config->e_byte_clk = (enum mipi_dsim_byte_clk_src)u32Val;

	if (of_property_read_u32(np, "e_burst_mode", &u32Val)) {
		dev_err(&pdev->dev, "e_burst_mode property not found\n");
		return -EINVAL;
	}
	dsim_config->e_burst_mode = (enum mipi_dsim_burst_mode_type)u32Val;

	if (of_property_read_u32(np, "p", &u32Val)) {
		dev_err(&pdev->dev, "p property not found\n");
		return -EINVAL;
	}
	dsim_config->p = (unsigned char)u32Val;

	if (of_property_read_u32(np, "m", &u32Val)) {
		dev_err(&pdev->dev, "m property not found\n");
		return -EINVAL;
	}
	dsim_config->m = (unsigned short)u32Val;

	if (of_property_read_u32(np, "s", &u32Val)) {
		dev_err(&pdev->dev, "s property not found\n");
		return -EINVAL;
	}
	dsim_config->s = (unsigned char)u32Val;

	if (of_property_read_u32(np, "pll_stable_time", &u32Val)) {
		dev_err(&pdev->dev, "pll_stable_time property not found\n");
		return -EINVAL;
	}
	dsim_config->pll_stable_time = (unsigned int)u32Val;

	if (of_property_read_u32(np, "esc_clk", &u32Val)) {
		dev_err(&pdev->dev, "esc_clk property not found\n");
		return -EINVAL;
	}
	dsim_config->esc_clk = (unsigned long)u32Val;

	if (of_property_read_u32(np, "stop_holding_cnt", &u32Val)) {
		dev_err(&pdev->dev, "stop_holding_cnt property not found\n");
		return -EINVAL;
	}
	dsim_config->stop_holding_cnt = (unsigned short)u32Val;

	if (of_property_read_u32(np, "bta_timeout", &u32Val)) {
		dev_err(&pdev->dev, "bta_timeout property not found\n");
		return -EINVAL;
	}
	dsim_config->bta_timeout = (unsigned char)u32Val;

	if (of_property_read_u32(np, "rx_timeout", &u32Val)) {
		dev_err(&pdev->dev, "rx_timeout property not found\n");
		return -EINVAL;
	}
	dsim_config->rx_timeout = (unsigned short)u32Val;

	if (of_property_read_u32(np, "e_virtual_ch", &u32Val)) {
		dev_err(&pdev->dev, "e_virtual_ch property not found\n");
		return -EINVAL;
	}
	dsim_config->e_virtual_ch = (enum mipi_dsim_virtual_ch_no)u32Val;

	if (of_property_read_u32(np, "cmd_allow", &u32Val)) {
		dev_err(&pdev->dev, "cmd_allow property not found\n");
		return -EINVAL;
	}
	dsim_config->cmd_allow = (unsigned char)u32Val;

	return 0;
}

static int exynos_mipi_parse_ofnode_panel_info(struct platform_device *pdev,
		struct device_node *np, struct fb_videomode *panel_info)
{
	unsigned int data[4];

	if (of_property_read_u32_array(np, "lcd-htiming", data, 4)) {
		dev_err(&pdev->dev, "invalid horizontal timing\n");
		return -EINVAL;
	}
	panel_info->left_margin = data[0];
	panel_info->right_margin = data[1];
	panel_info->hsync_len = data[2];
	panel_info->xres = data[3];

	if (of_property_read_u32_array(np, "lcd-vtiming", data, 4)) {
		dev_err(&pdev->dev, "invalid vertical timing\n");
		return -EINVAL;
	}
	panel_info->upper_margin = data[0];
	panel_info->lower_margin = data[1];
	panel_info->vsync_len = data[2];
	panel_info->yres = data[3];

	return 0;
}

static int exynos_mipi_parse_ofnode(struct platform_device *pdev,
	struct mipi_dsim_config *dsim_config, struct fb_videomode *panel_info)
{
	struct device_node *np_dsim_config, *np_panel_info;
	struct device_node *np = pdev->dev.of_node;

	np_dsim_config = of_find_node_by_name(np, "mipi-config");
	if (!np_dsim_config)
		return -EINVAL;

	if (exynos_mipi_parse_ofnode_config(pdev, np_dsim_config, dsim_config))
		return -EINVAL;

	np_panel_info = of_parse_phandle(np, "panel-info", 0);
	if (!np_panel_info)
		return -EINVAL;

	if (exynos_mipi_parse_ofnode_panel_info(pdev,
					np_panel_info, panel_info))
		return -EINVAL;

	return 0;
}

static int exynos_mipi_dsi_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct mipi_dsim_device *dsim;
	struct mipi_dsim_config *dsim_config;
	struct mipi_dsim_platform_data *dsim_pd;
	struct mipi_dsim_ddi *dsim_ddi;
	struct device_node *ofnode_lcd = NULL;
	struct device_node *ofnode_dphy = NULL;
	struct mipi_dsim_lcd_device *active_mipi_dsim_lcd_device = NULL;
	struct mipi_dsim_phy_config *mipi_dphy_config;
	struct fb_videomode *panel_info;
	unsigned int u32Val;
	int ret = -EINVAL;

	dsim = devm_kzalloc(&pdev->dev, sizeof(struct mipi_dsim_device),
				GFP_KERNEL);
	if (!dsim) {
		dev_err(&pdev->dev, "failed to allocate dsim object.\n");
		return -ENOMEM;
	}

	if (pdev->dev.of_node) {
		ofnode_lcd = exynos_mipi_find_ofnode_lcd_device(pdev);
		if (!ofnode_lcd)
			return -EINVAL;

		active_mipi_dsim_lcd_device =
				exynos_mipi_parse_ofnode_lcd(pdev, ofnode_lcd);

		if (NULL == active_mipi_dsim_lcd_device)
			return -EINVAL;

		if (NULL == exynos_mipi_dsi_find_lcd_driver(
						active_mipi_dsim_lcd_device))
			return -EPROBE_DEFER;

		exynos_mipi_dsi_register_lcd_device(
						active_mipi_dsim_lcd_device);
	}

	dsim->pd = to_dsim_plat(pdev);
	dsim->dev = &pdev->dev;
	dsim->id = pdev->id;

	if (pdev->dev.of_node) {
		dsim_config = devm_kzalloc(&pdev->dev,
			sizeof(struct mipi_dsim_config), GFP_KERNEL);
		if (!dsim_config) {
			dev_err(&pdev->dev,
				"failed to allocate dsim_config object.\n");
			return -ENOMEM;
		}

		panel_info = devm_kzalloc(&pdev->dev,
				sizeof(struct fb_videomode), GFP_KERNEL);
		if (!panel_info) {
			dev_err(&pdev->dev,
				"failed to allocate fb_videomode object.\n");
			return -ENOMEM;
		}

		/* parse the mipi of_node for dism_config and panel info. */
		if (exynos_mipi_parse_ofnode(pdev, dsim_config, panel_info)) {
			dev_err(&pdev->dev,
				"failed to read mipi-config, panel-info\n");
			return -EINVAL;
		}

		dsim_pd = devm_kzalloc(&pdev->dev,
			sizeof(struct mipi_dsim_platform_data), GFP_KERNEL);
		if (!dsim_pd) {
			dev_err(&pdev->dev,
				"failed to allocate mipi_dsim_platform_data\n");
			return -ENOMEM;
		}

		if (of_property_read_u32(pdev->dev.of_node, "enabled", &u32Val))
			dev_err(&pdev->dev, "enabled property not found\n");
		else
			dsim_pd->enabled = !(!u32Val);

		dsim_pd->lcd_panel_info = (void *)panel_info;
		dsim_pd->dsim_config = dsim_config;
		dsim->pd = dsim_pd;

	} else {
		/* get mipi_dsim_platform_data. */
		dsim_pd = (struct mipi_dsim_platform_data *)dsim->pd;
		if (dsim_pd == NULL) {
			dev_err(&pdev->dev,
				"failed to get platform data for dsim.\n");
			return -EFAULT;
		}

		/* get mipi_dsim_config. */
		dsim_config = dsim_pd->dsim_config;
		if (dsim_config == NULL) {
			dev_err(&pdev->dev,
				"failed to get dsim config data.\n");
			return -EFAULT;
		}
	}

	dsim->dsim_config = dsim_config;
	dsim->master_ops = &master_ops;

	mutex_init(&dsim->lock);

	ret = devm_regulator_bulk_get(&pdev->dev, ARRAY_SIZE(supplies),
					supplies);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get regulators: %d\n", ret);
		return ret;
	}

	dsim->clock = devm_clk_get(&pdev->dev, "dsim0");
	if (IS_ERR(dsim->clock)) {
		dev_err(&pdev->dev, "failed to get dsim clock source\n");
		return -ENODEV;
	}

	clk_prepare_enable(dsim->clock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	dsim->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dsim->reg_base)) {
		ret = PTR_ERR(dsim->reg_base);
		goto error;
	}

	mutex_init(&dsim->lock);

	/* bind lcd ddi matched with panel name. */
	if (pdev->dev.of_node) {
		dsim_ddi = exynos_mipi_dsi_bind_lcd_ddi(dsim,
					active_mipi_dsim_lcd_device->name);
	} else {
		dsim_ddi = exynos_mipi_dsi_bind_lcd_ddi(dsim,
					dsim_pd->lcd_panel_name);
	}

	if (!dsim_ddi) {
		dev_err(&pdev->dev, "mipi_dsim_ddi object not found.\n");
		ret = -ENXIO;
		goto error;
	} else if (pdev->dev.of_node) {
		dsim_ddi->ofnode_dsim_lcd_dev = ofnode_lcd;
		dsim_ddi->ofnode_dsim_dphy = ofnode_dphy;
 	}

	dsim->irq = platform_get_irq(pdev, 0);
	if (IS_ERR_VALUE(dsim->irq)) {
		dev_err(&pdev->dev, "failed to request dsim irq resource\n");
		ret = -EINVAL;
		goto error;
	}

	init_completion(&dsim_wr_comp);
	init_completion(&dsim_rd_comp);
	platform_set_drvdata(pdev, dsim);

	/* update dsim phy config node */
	if (pdev->dev.of_node) {
		ofnode_dphy = exynos_mipi_find_ofnode_dsim_phy(pdev);
		if (!ofnode_dphy)
			return -EINVAL;

		mipi_dphy_config = exynos_mipi_parse_ofnode_dsim_phy(pdev,
								ofnode_dphy);
		if (NULL == mipi_dphy_config)
			return -EINVAL;

		dsim->dsim_phy_config = mipi_dphy_config;
	}

	ret = devm_request_irq(&pdev->dev, dsim->irq,
			exynos_mipi_dsi_interrupt_handler,
			IRQF_SHARED, dev_name(&pdev->dev), dsim);
	if (ret != 0) {
		dev_err(&pdev->dev, "failed to request dsim irq\n");
		ret = -EINVAL;
		goto error;
	}

	/* enable interrupts */
	exynos_mipi_dsi_init_interrupt(dsim);

	/* initialize mipi-dsi client(lcd panel). */
	if (dsim_ddi->dsim_lcd_drv && dsim_ddi->dsim_lcd_drv->probe)
		dsim_ddi->dsim_lcd_drv->probe(dsim_ddi->dsim_lcd_dev);

	/* in case mipi-dsi has been enabled by bootloader */
	if (dsim_pd->enabled) {
		exynos_mipi_regulator_enable(dsim);
		goto done;
	}

	/* lcd panel power on. */
	if (dsim_ddi->dsim_lcd_drv && dsim_ddi->dsim_lcd_drv->power_on)
		dsim_ddi->dsim_lcd_drv->power_on(dsim_ddi->dsim_lcd_dev, 1);

	exynos_mipi_regulator_enable(dsim);

	/* enable MIPI-DSI PHY. */
	if (dsim->pd->phy_enable)
		dsim->pd->phy_enable(pdev, true);

	exynos_mipi_update_cfg(dsim);

	/* enable the LPDT mode */
	exynos_mipi_dsi_stand_by(dsim, 0);
	exynos_mipi_dsi_set_lcdc_transfer_mode(dsim, 1);
	exynos_mipi_dsi_set_cpu_transfer_mode(dsim, 1);
	exynos_mipi_dsi_enable_hs_clock(dsim, 0);

	/* set lcd panel sequence commands. */
	if (dsim_ddi->dsim_lcd_drv && dsim_ddi->dsim_lcd_drv->set_sequence)
		dsim_ddi->dsim_lcd_drv->set_sequence(dsim_ddi->dsim_lcd_dev);

	/* enable the HS mode */
	exynos_mipi_dsi_set_lcdc_transfer_mode(dsim, 0);
	exynos_mipi_dsi_set_cpu_transfer_mode(dsim, 0);
	exynos_mipi_dsi_enable_hs_clock(dsim, 1);
	exynos_mipi_dsi_stand_by(dsim, 1);

	dsim->suspended = false;

done:
	platform_set_drvdata(pdev, dsim);

	dev_dbg(&pdev->dev, "%s() completed successfully (%s mode)\n", __func__,
		dsim_config->e_interface == DSIM_COMMAND ? "CPU" : "RGB");

	return 0;

error:
	clk_disable(dsim->clock);
	return ret;
}

static int exynos_mipi_dsi_remove(struct platform_device *pdev)
{
	struct mipi_dsim_device *dsim = platform_get_drvdata(pdev);
	struct mipi_dsim_ddi *dsim_ddi, *next;
	struct mipi_dsim_lcd_driver *dsim_lcd_drv;

	clk_disable(dsim->clock);

	list_for_each_entry_safe(dsim_ddi, next, &dsim_ddi_list, list) {
		if (dsim_ddi) {
			if (dsim->id != dsim_ddi->bus_id)
				continue;

			dsim_lcd_drv = dsim_ddi->dsim_lcd_drv;

			if (dsim_lcd_drv->remove)
				dsim_lcd_drv->remove(dsim_ddi->dsim_lcd_dev);

			kfree(dsim_ddi);
		}
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int exynos_mipi_dsi_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mipi_dsim_device *dsim = platform_get_drvdata(pdev);
	struct mipi_dsim_lcd_driver *client_drv = dsim->dsim_lcd_drv;
	struct mipi_dsim_lcd_device *client_dev = dsim->dsim_lcd_dev;

	disable_irq(dsim->irq);

	if (dsim->suspended)
		return 0;

	if (client_drv && client_drv->suspend)
		client_drv->suspend(client_dev);

	/* enable MIPI-DSI PHY. */
	if (dsim->pd->phy_enable)
		dsim->pd->phy_enable(pdev, false);

	clk_disable(dsim->clock);

	exynos_mipi_regulator_disable(dsim);

	dsim->suspended = true;

	return 0;
}

static int exynos_mipi_dsi_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mipi_dsim_device *dsim = platform_get_drvdata(pdev);
	struct mipi_dsim_lcd_driver *client_drv = dsim->dsim_lcd_drv;
	struct mipi_dsim_lcd_device *client_dev = dsim->dsim_lcd_dev;

	enable_irq(dsim->irq);

	if (!dsim->suspended)
		return 0;

	/* lcd panel power on. */
	if (client_drv && client_drv->power_on)
		client_drv->power_on(client_dev, 1);

	exynos_mipi_regulator_enable(dsim);

	/* enable MIPI-DSI PHY. */
	if (dsim->pd->phy_enable)
		dsim->pd->phy_enable(pdev, true);

	clk_enable(dsim->clock);

	exynos_mipi_update_cfg(dsim);

	/* set lcd panel sequence commands. */
	if (client_drv && client_drv->set_sequence)
		client_drv->set_sequence(client_dev);

	dsim->suspended = false;

	return 0;
}
#endif

static const struct dev_pm_ops exynos_mipi_dsi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(exynos_mipi_dsi_suspend, exynos_mipi_dsi_resume)
};

static struct platform_device_id exynos_mipi_driver_ids[] = {
	{
		.name		= "exynos-mipidsim",
		.driver_data	= (unsigned long)0,
	},
	{},
};
MODULE_DEVICE_TABLE(platform, exynos_mipi_driver_ids);

static const struct of_device_id exynos_mipi_match[] = {
	{
		.compatible = "samsung,exynos-mipidsim",
		.data = NULL,
	},
	{},
};
MODULE_DEVICE_TABLE(of, exynos_mipi_match);

static struct platform_driver exynos_mipi_dsi_driver = {
	.probe = exynos_mipi_dsi_probe,
	.remove = exynos_mipi_dsi_remove,
	.id_table = exynos_mipi_driver_ids,
	.driver = {
		   .name = "exynos-mipi-dsim",
		   .owner = THIS_MODULE,
		   .pm = &exynos_mipi_dsi_pm_ops,
		   .of_match_table = exynos_mipi_match,
	},
};

module_platform_driver(exynos_mipi_dsi_driver);

MODULE_AUTHOR("InKi Dae <inki.dae@samsung.com>");
MODULE_DESCRIPTION("Samusung SoC MIPI-DSI driver");
MODULE_LICENSE("GPL");

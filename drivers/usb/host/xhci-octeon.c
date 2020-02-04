/*
 * XHCI HCD glue for Cavium Octeon III SOCs.
 *
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2010 Cavium Networks
 *
 */

#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <asm/octeon/octeon.h>
#include <asm/octeon/cvmx-usbdrdx-defs.h>
#include <asm/octeon/cvmx-gpio-defs.h>

#define UCTL_CTL	0
#define UCTL_HOST_CFG	0xe0
#define UCTL_SHIM_CFG	0xe8

#define OCTEON_USB3_MIN_H_CLK_RATE	(150000000)
#define OCTEON_USB3_MAX_H_CLK_RATE	(300000000)
#define OCTEON_USB3_NUM_CLK_DEV_SEL	(8)

static DEFINE_MUTEX(octeon3_usb_clocks_mutex);
static int octeon3_usb_clock_start_cnt[4][2]; /* 4 nodes x 2 indexes */

static uint8_t clk_div[OCTEON_USB3_NUM_CLK_DEV_SEL] = {1, 2, 4, 6, 8,
							16, 24, 32};

static int octeon3_usb_base2index(u64 base)
{
	return (base >> 24) & 1;
}
static int octeon3_usb_base2node(u64 base)
{
	return (base >> 36) & 3;
}

static int octeon3_usb_config_power(struct device_node *node, u64 base)
{
	union cvmx_usbdrdx_uctl_host_cfg uctl_host_cfg;
	union cvmx_gpio_bit_cfgx gpio_bit;
	uint32_t gpio_pwr[3];
	int power_active_low;
	int gpio;
	int len;
	int index = octeon3_usb_base2index(base);

	if (of_find_property(node, "power", &len) != NULL) {
		if (len == 12) {
			of_property_read_u32_array(node, "power", gpio_pwr, 3);
			power_active_low = gpio_pwr[2] & 0x01;
			gpio = gpio_pwr[1];
		} else if (len == 8) {
			of_property_read_u32_array(node, "power", gpio_pwr, 2);
			power_active_low = 0;
			gpio = gpio_pwr[1];
		} else {
			pr_err("Invalid power GPIO device tree entry\n");
			return -1;
		}
		if ((OCTEON_IS_MODEL(OCTEON_CN73XX)
		     || OCTEON_IS_MODEL(OCTEON_CNF75XX))
		    && gpio <= 31) {
			gpio_bit.u64 = cvmx_read_csr(CVMX_GPIO_BIT_CFGX(gpio));
			gpio_bit.s.tx_oe = 1;
			gpio_bit.cn73xx.output_sel = (index == 0 ? 0x14 : 0x15);
			cvmx_write_csr(CVMX_GPIO_BIT_CFGX(gpio), gpio_bit.u64);
		} else if (gpio <= 15) {
			gpio_bit.u64 = cvmx_read_csr(CVMX_GPIO_BIT_CFGX(gpio));
			gpio_bit.s.tx_oe = 1;
			gpio_bit.cn70xx.output_sel = (index == 0 ? 0x14 : 0x19);
			cvmx_write_csr(CVMX_GPIO_BIT_CFGX(gpio), gpio_bit.u64);
		} else {
			gpio_bit.u64 = cvmx_read_csr(CVMX_GPIO_XBIT_CFGX(gpio));
			gpio_bit.s.tx_oe = 1;
			gpio_bit.cn70xx.output_sel = (index == 0 ? 0x14 : 0x19);
			cvmx_write_csr(CVMX_GPIO_XBIT_CFGX(gpio), gpio_bit.u64);
		}
		/* Enable XHCI power control and set if active high or low */
		uctl_host_cfg.u64 = cvmx_read_csr(base + UCTL_HOST_CFG);
		uctl_host_cfg.s.ppc_en = 1;
		uctl_host_cfg.s.ppc_active_high_en = !power_active_low;
		cvmx_write_csr(base + UCTL_HOST_CFG, uctl_host_cfg.u64);
	} else {
		pr_err("No power GPIO device tree entry\n");
		/* Disable XHCI power control and set if active high*/
		uctl_host_cfg.u64 = cvmx_read_csr(base + UCTL_HOST_CFG);
		uctl_host_cfg.s.ppc_en = 0;
		uctl_host_cfg.s.ppc_active_high_en = 0;
		cvmx_write_csr(base + UCTL_HOST_CFG, uctl_host_cfg.u64);
	}
	return 0;
}

static int octeon3_usb_clocks_start(struct device *dev, u64 base)
{
	u32 clock_rate = 100000000;
	union cvmx_usbdrdx_uctl_ctl uctl_ctl;
	int ref_clk_sel = 2;
	u64 div;
	int mpll_mul;
	int i;
	u64 h_clk_rate;
	u64 uctl_ctl_reg = base + UCTL_CTL;
	int index = octeon3_usb_base2index(base);

	if (dev->of_node) {
		const char *ss_clock_type;
		const char *hs_clock_type;

		i = of_property_read_u32(dev->of_node,
					 "refclk-frequency", &clock_rate);
		if (i) {
			pr_err("No UCTL \"refclk-frequency\"\n");
			return -EINVAL;
		}
		i = of_property_read_string(dev->of_node,
					    "refclk-type-ss", &ss_clock_type);
		if (i) {
			pr_err("No UCTL \"refclk-type-ss\"\n");
			return -EINVAL;
		}
		i = of_property_read_string(dev->of_node,
					    "refclk-type-hs", &hs_clock_type);
		if (i) {
			pr_err("No UCTL \"refclk-type-hs\"\n");
			return -EINVAL;
		}
		if (strcmp("dlmc_ref_clk0", ss_clock_type) == 0) {
			if (strcmp(hs_clock_type, "dlmc_ref_clk0") == 0)
				ref_clk_sel = 0;
			else if (strcmp(hs_clock_type, "pll_ref_clk") == 0)
				ref_clk_sel = 2;
			else
				pr_err("Invalid HS clock type %s, using  pll_ref_clk instead\n",
				       hs_clock_type);
		} else if (strcmp(ss_clock_type, "dlmc_ref_clk1") == 0) {
			if (strcmp(hs_clock_type, "dlmc_ref_clk1") == 0)
				ref_clk_sel = 1;
			else if (strcmp(hs_clock_type, "pll_ref_clk") == 0)
				ref_clk_sel = 3;
			else {
				pr_err("Invalid HS clock type %s, using  pll_ref_clk instead\n",
				       hs_clock_type);
				ref_clk_sel = 3;
			}
		} else
			pr_err("Invalid SS clock type %s, using  dlmc_ref_clk0 instead\n",
			       ss_clock_type);

		if ((ref_clk_sel == 0 || ref_clk_sel == 1) &&
				  (clock_rate != 100000000))
			pr_err("Invalid UCTL clock rate of %u, using 100000000 instead\n",
			       clock_rate);

	} else {
		pr_err("No USB UCTL device node\n");
		return -EINVAL;
	}

	/*
	* Step 1: Wait for voltages stable.  That surely happened
	* before starting the kernel.
	* Ensure the reference clock is up and stable ??
	*/

	/* Step 2: Wait for IOI reset to deassert  ?? */

	/* Step 3: program over current indication if desired, later */

	/* Step 3: program the port power control feature if desired, later */

	/* Step 4: Assert all resets */
	uctl_ctl.u64 = cvmx_read_csr(uctl_ctl_reg);
	uctl_ctl.s.uphy_rst = 1;
	uctl_ctl.s.uahc_rst = 1;
	uctl_ctl.s.uctl_rst = 1;
	cvmx_write_csr(uctl_ctl_reg, uctl_ctl.u64);

	/* Step 5a: Reset the clock dividers */
	uctl_ctl.u64 = cvmx_read_csr(uctl_ctl_reg);
	uctl_ctl.s.h_clkdiv_rst = 1;
	cvmx_write_csr(uctl_ctl_reg, uctl_ctl.u64);

	/* 5b */
	/* Step 5b: Select controller clock frequency */
	for (div = 0; div < OCTEON_USB3_NUM_CLK_DEV_SEL; div++) {
		h_clk_rate = octeon_get_io_clock_rate() / clk_div[div];
		if (h_clk_rate <= OCTEON_USB3_MAX_H_CLK_RATE &&
				 h_clk_rate >= OCTEON_USB3_MIN_H_CLK_RATE)
			break;
	}
	uctl_ctl.u64 = cvmx_read_csr(uctl_ctl_reg);
	uctl_ctl.s.h_clkdiv_sel = div;
	uctl_ctl.s.h_clk_en = 1;
	cvmx_write_csr(uctl_ctl_reg, uctl_ctl.u64);
	uctl_ctl.u64 = cvmx_read_csr(uctl_ctl_reg);
	if ((div != uctl_ctl.s.h_clkdiv_sel) || (!uctl_ctl.s.h_clk_en)) {
		dev_err(dev, "ERROR: usb controller clock init\n");
			return -EINVAL;
	}

	/* Step 5c: Deassert the controller clock divider reset */
	uctl_ctl.u64 = cvmx_read_csr(uctl_ctl_reg);
	uctl_ctl.s.h_clkdiv_rst = 0;
	cvmx_write_csr(uctl_ctl_reg, uctl_ctl.u64);

	/* Step ??*/
	udelay(10);

	/* Step 6a-6d & 7: Reference clock configuration */
	uctl_ctl.u64 = cvmx_read_csr(uctl_ctl_reg);
	uctl_ctl.s.ssc_en = 1;
	uctl_ctl.s.ref_ssp_en = 1;
	uctl_ctl.s.ref_clk_sel = ref_clk_sel;
	uctl_ctl.s.ref_clk_fsel = 0x07;
	uctl_ctl.s.ref_clk_div2 = 0;
	switch (clock_rate) {
	default:
		pr_err("Invalid UCTL ref_clk %u, using 100000000 instead\n",
		       clock_rate);
		/* Fall through */
	case 100000000:
		mpll_mul = 0x19;
		if (ref_clk_sel < 2)
			uctl_ctl.s.ref_clk_fsel = 0x27;
		break;
	case 50000000:
		mpll_mul = 0x32;
		break;
	case 125000000:
		mpll_mul = 0x28;
		break;
	}
	uctl_ctl.s.mpll_multiplier = mpll_mul;
	uctl_ctl.s.ss_power_en = 1;
	uctl_ctl.s.hs_power_en = 1;
	cvmx_write_csr(uctl_ctl_reg, uctl_ctl.u64);

	/* Step 9a */
	uctl_ctl.u64 = cvmx_read_csr(uctl_ctl_reg);
	uctl_ctl.s.uctl_rst = 0;
	cvmx_write_csr(uctl_ctl_reg, uctl_ctl.u64);

	/* Configure power */
	if (octeon3_usb_config_power(dev->of_node, base)) {
		dev_err(dev, "Error configuring power.\n");
		return -EINVAL;
	}

	/* Step 9b */
	uctl_ctl.u64 = cvmx_read_csr(uctl_ctl_reg);
	uctl_ctl.s.uahc_rst = 0;
	cvmx_write_csr(uctl_ctl_reg, uctl_ctl.u64);

	/* Step 9c*/
	ndelay(200);

	/* Step 10*/
	uctl_ctl.u64 = cvmx_read_csr(uctl_ctl_reg);
	uctl_ctl.s.csclk_en = 1;
	cvmx_write_csr(uctl_ctl_reg, uctl_ctl.u64);

	/*Step 11*/
	uctl_ctl.u64 = cvmx_read_csr(uctl_ctl_reg);
	uctl_ctl.s.drd_mode = 0;
	cvmx_write_csr(uctl_ctl_reg, uctl_ctl.u64);

	octeon_error_tree_enable(CVMX_ERROR_GROUP_USB, index);

	return 0;
}

static void octeon3_usb_clocks_stop(u64 base)
{
	int index = octeon3_usb_base2index(base);

	octeon_error_tree_disable(CVMX_ERROR_GROUP_USB, index);
}

void octeon3_usb_set_endian_mode(u64 base)
{
	union cvmx_usbdrdx_uctl_shim_cfg shim_cfg;
	shim_cfg.u64 = cvmx_read_csr(base + UCTL_SHIM_CFG);
#ifdef __BIG_ENDIAN
	shim_cfg.s.dma_endian_mode = 1;
	shim_cfg.s.csr_endian_mode = 1;
#else
	shim_cfg.s.dma_endian_mode = 0;
	shim_cfg.s.csr_endian_mode = 0;
#endif
	cvmx_write_csr(base + UCTL_SHIM_CFG, shim_cfg.u64);
}
EXPORT_SYMBOL(octeon3_usb_set_endian_mode);

void octeon3_usb_phy_reset(u64 base)
{
	union cvmx_usbdrdx_uctl_ctl uctl_ctl;
	int index = (base >> 40) & 1;
	int node = octeon3_usb_base2node(base);

	uctl_ctl.u64 = cvmx_read_csr_node(node, CVMX_USBDRDX_UCTL_CTL(index));
	uctl_ctl.s.uphy_rst = 0;
	cvmx_write_csr_node(node, CVMX_USBDRDX_UCTL_CTL(index), uctl_ctl.u64);
}
EXPORT_SYMBOL(octeon3_usb_phy_reset);

int xhci_octeon_start(struct platform_device *pdev)
{
	struct resource *res_mem;
	struct platform_device *parent_pdev;
	int node, index;
	u64 base;

	parent_pdev = container_of(pdev->dev.parent, struct platform_device, dev);
	res_mem = platform_get_resource(parent_pdev, IORESOURCE_MEM, 0);
	if (res_mem == NULL) {
		dev_err(&parent_pdev->dev, "found no memory resource\n");
		return -ENXIO;
	}
	index = octeon3_usb_base2index(res_mem->start);
	node = octeon3_usb_base2node(res_mem->start);

	/* Hack alert:  Should use ioremap() */
	base = res_mem->start | 0x8000000000000000ull;

	mutex_lock(&octeon3_usb_clocks_mutex);
	octeon3_usb_clock_start_cnt[node][index]++;
	if (octeon3_usb_clock_start_cnt[node][index] == 1) {
		octeon3_usb_clocks_start(&parent_pdev->dev, base);
		octeon3_usb_set_endian_mode(base);
		dev_info(&parent_pdev->dev, "clocks initialized.\n");
	}
	mutex_unlock(&octeon3_usb_clocks_mutex);

	return 0;
}
EXPORT_SYMBOL(xhci_octeon_start);

int xhci_octeon_stop(struct platform_device *pdev)
{
	struct resource *res_mem;
	struct platform_device *parent_pdev;
	int node, index;

	parent_pdev = container_of(pdev->dev.parent, struct platform_device, dev);

	res_mem = platform_get_resource(parent_pdev, IORESOURCE_MEM, 0);
	if (res_mem == NULL) {
		dev_err(&parent_pdev->dev, "found no memory resource\n");
		return -ENXIO;
	}
	index = octeon3_usb_base2index(res_mem->start);
	node = octeon3_usb_base2node(res_mem->start);


	mutex_lock(&octeon3_usb_clocks_mutex);
	octeon3_usb_clock_start_cnt[node][index]--;
	if (octeon3_usb_clock_start_cnt[node][index] == 0)
		octeon3_usb_clocks_stop(res_mem->start);
	mutex_unlock(&octeon3_usb_clocks_mutex);

	platform_set_drvdata(pdev, NULL);
	return 0;
}
EXPORT_SYMBOL(xhci_octeon_stop);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Cavium, Inc. <support@cavium.com>");
MODULE_DESCRIPTION("Cavium Inc. octeon usb3 clock init.");

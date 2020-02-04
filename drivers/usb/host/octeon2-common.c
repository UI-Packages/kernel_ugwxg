/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2010 - 2012 Cavium, Inc.
 */

#include <linux/device.h>
#include <linux/export.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/of.h>

#include <asm/octeon/octeon.h>
#include <asm/octeon/cvmx-uctlx-defs.h>
#include <asm/octeon/cvmx-uahcx-defs.h>

static DEFINE_MUTEX(octeon2_usb_clocks_mutex);

static int octeon2_usb_clock_start_cnt;

void octeon2_usb_clocks_start(struct device *dev)
{
	u64 div;
	union cvmx_uctlx_if_ena if_ena;
	union cvmx_uctlx_clk_rst_ctl clk_rst_ctl;
	union cvmx_uctlx_uphy_portx_ctl_status port_ctl_status;
	struct device_node *uctl_node;
	u32 clock_rate = 12000000;
	bool is_crystal_clock = false;
	int i;
	unsigned long io_clk_64_to_ns;


	mutex_lock(&octeon2_usb_clocks_mutex);

	octeon2_usb_clock_start_cnt++;
	if (octeon2_usb_clock_start_cnt != 1)
		goto exit;

	io_clk_64_to_ns = 64000000000ull / octeon_get_io_clock_rate();

	if (dev->of_node) {
		const char *clock_type;

		uctl_node = of_get_parent(dev->of_node);
		if (!uctl_node) {
			pr_err("No UCTL device node\n");
			goto exit;
		}
		i = of_property_read_u32(uctl_node,
					 "refclk-frequency", &clock_rate);
		if (i) {
			pr_err("No UCTL \"refclk-frequency\"\n");
			goto exit;
		}
		i = of_property_read_string(uctl_node,
					    "refclk-type", &clock_type);

		if (!i && strcmp("crystal", clock_type) == 0)
			is_crystal_clock = true;
	}

	/*
	 * Step 1: Wait for voltages stable.  That surely happened
	 * before starting the kernel.
	 *
	 * Step 2: Enable  SCLK of UCTL by writing UCTL0_IF_ENA[EN] = 1
	 */
	if_ena.u64 = 0;
	if_ena.s.en = 1;
	cvmx_write_csr(CVMX_UCTLX_IF_ENA(0), if_ena.u64);

	for (i = 0; i <= 1; i++) {
		port_ctl_status.u64 =
			cvmx_read_csr(CVMX_UCTLX_UPHY_PORTX_CTL_STATUS(i, 0));
		/* Set txvreftune to 15 to obtain compliant 'eye' diagram. */
		port_ctl_status.s.txvreftune = 15;
		port_ctl_status.s.txrisetune = 1;
		port_ctl_status.s.txpreemphasistune = 1;
		cvmx_write_csr(CVMX_UCTLX_UPHY_PORTX_CTL_STATUS(i, 0),
			       port_ctl_status.u64);
	}

	/* Step 3: Configure the reference clock, PHY, and HCLK */
	clk_rst_ctl.u64 = cvmx_read_csr(CVMX_UCTLX_CLK_RST_CTL(0));

	/*
	 * If the UCTL looks like it has already been started, skip
	 * the initialization, otherwise bus errors are obtained.
	 */
	if (clk_rst_ctl.s.hrst)
		goto end_clock;
	/* 3a */
	clk_rst_ctl.s.p_por = 1;
	clk_rst_ctl.s.hrst = 0;
	clk_rst_ctl.s.p_prst = 0;
	clk_rst_ctl.s.h_clkdiv_rst = 0;
	clk_rst_ctl.s.o_clkdiv_rst = 0;
	clk_rst_ctl.s.h_clkdiv_en = 0;
	clk_rst_ctl.s.o_clkdiv_en = 0;
	cvmx_write_csr(CVMX_UCTLX_CLK_RST_CTL(0), clk_rst_ctl.u64);

	/* 3b */
	clk_rst_ctl.s.p_refclk_sel = is_crystal_clock ? 0 : 1;
	switch (clock_rate) {
	default:
		pr_err("Invalid UCTL clock rate of %u, using 12000000 instead\n",
			clock_rate);
		/* Fall through */
	case 12000000:
		clk_rst_ctl.s.p_refclk_div = 0;
		break;
	case 24000000:
		clk_rst_ctl.s.p_refclk_div = 1;
		break;
	case 48000000:
		clk_rst_ctl.s.p_refclk_div = 2;
		break;
	}
	cvmx_write_csr(CVMX_UCTLX_CLK_RST_CTL(0), clk_rst_ctl.u64);

	/* 3c */
	div = octeon_get_io_clock_rate() / 130000000ull;

	switch (div) {
	case 0:
		div = 1;
		break;
	case 1:
	case 2:
	case 3:
	case 4:
		break;
	case 5:
		div = 4;
		break;
	case 6:
	case 7:
		div = 6;
		break;
	case 8:
	case 9:
	case 10:
	case 11:
		div = 8;
		break;
	default:
		div = 12;
		break;
	}
	clk_rst_ctl.s.h_div = div;
	cvmx_write_csr(CVMX_UCTLX_CLK_RST_CTL(0), clk_rst_ctl.u64);
	/* Read it back, */
	clk_rst_ctl.u64 = cvmx_read_csr(CVMX_UCTLX_CLK_RST_CTL(0));
	clk_rst_ctl.s.h_clkdiv_en = 1;
	cvmx_write_csr(CVMX_UCTLX_CLK_RST_CTL(0), clk_rst_ctl.u64);
	/* 3d */
	clk_rst_ctl.s.h_clkdiv_rst = 1;
	cvmx_write_csr(CVMX_UCTLX_CLK_RST_CTL(0), clk_rst_ctl.u64);

	/* 3e: delay 64 io clocks */
	ndelay(io_clk_64_to_ns);

	/*
	 * Step 4: Program the power-on reset field in the UCTL
	 * clock-reset-control register.
	 */
	clk_rst_ctl.s.p_por = 0;
	cvmx_write_csr(CVMX_UCTLX_CLK_RST_CTL(0), clk_rst_ctl.u64);

	/* Step 5:    Wait 3 ms for the PHY clock to start. */
	mdelay(3);

	/* Steps 6..9 for ATE only, are skipped. */

	/* Step 10: Configure the OHCI_CLK48 and OHCI_CLK12 clocks. */
	/* 10a */
	clk_rst_ctl.s.o_clkdiv_rst = 1;
	cvmx_write_csr(CVMX_UCTLX_CLK_RST_CTL(0), clk_rst_ctl.u64);

	/* 10b */
	clk_rst_ctl.s.o_clkdiv_en = 1;
	cvmx_write_csr(CVMX_UCTLX_CLK_RST_CTL(0), clk_rst_ctl.u64);

	/* 10c */
	ndelay(io_clk_64_to_ns);

	/*
	 * Step 11: Program the PHY reset field:
	 * UCTL0_CLK_RST_CTL[P_PRST] = 1
	 */
	clk_rst_ctl.s.p_prst = 1;
	cvmx_write_csr(CVMX_UCTLX_CLK_RST_CTL(0), clk_rst_ctl.u64);

	/* Step 11b */
	udelay(1);

	/* Step 11c */
	clk_rst_ctl.s.p_prst = 0;
	cvmx_write_csr(CVMX_UCTLX_CLK_RST_CTL(0), clk_rst_ctl.u64);

	/* Step 11d */
	mdelay(1);

	/* Step 11e */
	clk_rst_ctl.s.p_prst = 1;
	cvmx_write_csr(CVMX_UCTLX_CLK_RST_CTL(0), clk_rst_ctl.u64);

	/* Step 12: Wait 1 uS. */
	udelay(1);

	/* Step 13: Program the HRESET_N field: UCTL0_CLK_RST_CTL[HRST] = 1 */
	clk_rst_ctl.s.hrst = 1;
	cvmx_write_csr(CVMX_UCTLX_CLK_RST_CTL(0), clk_rst_ctl.u64);

end_clock:
	/* Set uSOF cycle period to 60,000 bits. */
	cvmx_write_csr(CVMX_UCTLX_EHCI_FLA(0), 0x20ull);

	octeon_error_tree_enable(CVMX_ERROR_GROUP_USB, -1);
exit:
	mutex_unlock(&octeon2_usb_clocks_mutex);
}
EXPORT_SYMBOL(octeon2_usb_clocks_start);

void octeon2_usb_clocks_stop(void)
{
	mutex_lock(&octeon2_usb_clocks_mutex);
	octeon2_usb_clock_start_cnt--;
	if (octeon2_usb_clock_start_cnt == 0)
		octeon_error_tree_disable(CVMX_ERROR_GROUP_USB, -1);
	mutex_unlock(&octeon2_usb_clocks_mutex);
}
EXPORT_SYMBOL(octeon2_usb_clocks_stop);

static int __init octeon2_usb_reset(void)
{
	union cvmx_uctlx_clk_rst_ctl clk_rst_ctl;
	union cvmx_uahcx_ehci_usbcmd ehci_usbcmd;
	union cvmx_uahcx_ohci0_hccommandstatus ohci_usbcmd;

	if (!OCTEON_IS_OCTEON2())
		return 0;

	clk_rst_ctl.u64 = cvmx_read_csr(CVMX_UCTLX_CLK_RST_CTL(0));
	if (clk_rst_ctl.s.hrst) {
		ehci_usbcmd.u32 = cvmx_read64_uint32(CVMX_UAHCX_EHCI_USBCMD(0));
		ehci_usbcmd.s.rs = 0;
		cvmx_write64_uint32(CVMX_UAHCX_EHCI_USBCMD(0), ehci_usbcmd.u32);
		mdelay(2);
		ehci_usbcmd.s.hcreset = 1;
		cvmx_write64_uint32(CVMX_UAHCX_EHCI_USBCMD(0), ehci_usbcmd.u32);
		ohci_usbcmd.u32 = cvmx_read64_uint32(CVMX_UAHCX_OHCI0_HCCOMMANDSTATUS(0));
		ohci_usbcmd.s.hcr = 1;
		cvmx_write64_uint32(CVMX_UAHCX_OHCI0_HCCOMMANDSTATUS(0), ohci_usbcmd.u32);

	}
	return 0;
}

arch_initcall(octeon2_usb_reset);

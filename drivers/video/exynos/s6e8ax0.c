/* linux/drivers/video/exynos/s6e8aa0.c
 *
 * MIPI-DSI based s6e8aa0 AMOLED lcd 4.65 inch panel driver.
 * This driver is implemented according to the s6e8ax0 panel driver.
 *
 * Shaik Ameer Basha <shaik.ameer@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/ctype.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/lcd.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#include <video/mipi_display.h>
#include <video/exynos_mipi_dsim.h>

#define LDI_MTP_LENGTH		24
#define DSIM_PM_STABLE_TIME	10
#define MIN_BRIGHTNESS		0
#define MAX_BRIGHTNESS		26
#define GAMMA_TABLE_COUNT	26

#define POWER_IS_ON(pwr)	((pwr) == FB_BLANK_UNBLANK)
#define POWER_IS_OFF(pwr)	((pwr) == FB_BLANK_POWERDOWN)
#define POWER_IS_NRM(pwr)	((pwr) == FB_BLANK_NORMAL)

#define lcd_to_master(a)	(a->dsim_dev->master)
#define lcd_to_master_ops(a)	((lcd_to_master(a))->master_ops)
#define lcd_to_master_ver(a)	((lcd_to_master(a))->version)

enum {
	DSIM_NONE_STATE = 0,
	DSIM_RESUME_COMPLETE = 1,
	DSIM_FRAME_DONE = 2,
};

struct s6e8aa0 {
	struct device	*dev;
	unsigned int			gpio_reset;
	unsigned int			gpio_power;
	unsigned int			gpio_bl;
	unsigned int			power;
	unsigned int			id;
	unsigned int			gamma;

	struct lcd_device	*ld;
	struct backlight_device	*bd;

	struct mipi_dsim_lcd_device	*dsim_dev;
	struct lcd_platform_data	*ddi_pd;
	struct mutex			lock;
	bool  enabled;
};

static struct s6e8aa0 *lcd_global;


static struct regulator_bulk_data supplies[] = {
	{ .supply = "vdd3", },
	{ .supply = "vci", },
};

static void s6e8aa0_regulator_enable(struct s6e8aa0 *lcd)
{
	int ret = 0;
	struct lcd_platform_data *pd = NULL;

	pd = lcd->ddi_pd;
	mutex_lock(&lcd->lock);
	if (!lcd->enabled) {
		ret = regulator_bulk_enable(ARRAY_SIZE(supplies), supplies);
		if (ret)
			goto out;

		lcd->enabled = true;
	}
	msleep(pd->power_on_delay);
out:
	mutex_unlock(&lcd->lock);
}

static void s6e8aa0_regulator_disable(struct s6e8aa0 *lcd)
{
	int ret = 0;

	mutex_lock(&lcd->lock);
	if (lcd->enabled) {
		ret = regulator_bulk_disable(ARRAY_SIZE(supplies), supplies);
		if (ret)
			goto out;

		lcd->enabled = false;
	}
out:
	mutex_unlock(&lcd->lock);
}


static const unsigned char gamma22_30[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xDF, 0x86, 0xF5,
	0xD5, 0xC7, 0xCF, 0xDF, 0xE0, 0xE0,
	0xC9, 0xC9, 0xCC, 0xD7, 0xD6, 0xD5,
	0x00, 0x68, 0x00, 0x68, 0x00, 0x75,
};

static const unsigned char gamma22_40[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xE5, 0xAA, 0xF2,
	0xD6, 0xCC, 0xCF, 0xE0, 0xE2, 0xE2,
	0xC8, 0xC9, 0xCA, 0xD2, 0xD2, 0xCF,
	0x00, 0x71, 0x00, 0x70, 0x00, 0x80,
};

static const unsigned char gamma22_50[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xE7, 0xBB, 0xEE,
	0xD6, 0xCE, 0xD0, 0xE0, 0xE3, 0xE4,
	0xC5, 0xC4, 0xC5, 0xD2, 0xD2, 0xCF,
	0x00, 0x78, 0x00, 0x78, 0x00, 0x88,
};

static const unsigned char gamma22_60[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xE9, 0xC4, 0xEB,
	0xD6, 0xD0, 0xD1, 0xE0, 0xE3, 0xE4,
	0xC3, 0xC2, 0xC2, 0xD2, 0xD1, 0xCF,
	0x00, 0x7E, 0x00, 0x7E, 0x00, 0x8F,
};

static const unsigned char gamma22_70[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xEA, 0xC9, 0xEA,
	0xD6, 0xD2, 0xD2, 0xDF, 0xE1, 0xE3,
	0xC2, 0xC1, 0xC0, 0xD1, 0xD0, 0xCE,
	0x00, 0x84, 0x00, 0x84, 0x00, 0x96,
};

static const unsigned char gamma22_80[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xEB, 0xCC, 0xE9,
	0xD5, 0xD4, 0xD3, 0xDE, 0xE1, 0xE2,
	0xC2, 0xBF, 0xBF, 0xCF, 0xCF, 0xCC,
	0x00, 0x89, 0x00, 0x89, 0x00, 0x9C,
};

static const unsigned char gamma22_90[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xEB, 0xD0, 0xE9,
	0xD4, 0xD5, 0xD4, 0xDF, 0xE0, 0xE1,
	0xC1, 0xBE, 0xBD, 0xCD, 0xCD, 0xCA,
	0x00, 0x8E, 0x00, 0x8F, 0x00, 0xA2,
};

static const unsigned char gamma22_100[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xEA, 0xD2, 0xE7,
	0xD7, 0xD6, 0xD6, 0xDF, 0xDF, 0xE2,
	0xBF, 0xBD, 0xBC, 0xCD, 0xCD, 0xC9,
	0x00, 0x92, 0x00, 0x93, 0x00, 0xA7,
};

static const unsigned char gamma22_110[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xEB, 0xD4, 0xE5,
	0xD6, 0xD6, 0xD7, 0xDE, 0xDF, 0xE0,
	0xBE, 0xBC, 0xBB, 0xCE, 0xCC, 0xC9,
	0x00, 0x96, 0x00, 0x97, 0x00, 0xAC,
};

static const unsigned char gamma22_120[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xED, 0xD6, 0xE6,
	0xD6, 0xD7, 0xD8, 0xDE, 0xDE, 0xE0,
	0xBC, 0xBC, 0xB9, 0xCD, 0xCA, 0xC8,
	0x00, 0x9A, 0x00, 0x9C, 0x00, 0xB1,
};

static const unsigned char gamma22_130[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xEC, 0xD7, 0xE6,
	0xD3, 0xD8, 0xD7, 0xDE, 0xDD, 0xDF,
	0xBD, 0xBB, 0xB8, 0xCA, 0xC9, 0xC6,
	0x00, 0x9F, 0x00, 0xA0, 0x00, 0xB7,
};

static const unsigned char gamma22_140[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xEC, 0xD9, 0xE5,
	0xD4, 0xD8, 0xD9, 0xDE, 0xDD, 0xDF,
	0xBB, 0xB9, 0xB7, 0xCA, 0xC9, 0xC5,
	0x00, 0xA3, 0x00, 0xA4, 0x00, 0xBB,
};

static const unsigned char gamma22_150[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xEC, 0xDA, 0xE5,
	0xD4, 0xD8, 0xD9, 0xDD, 0xDD, 0xDD,
	0xBB, 0xB9, 0xB6, 0xC9, 0xC7, 0xC5,
	0x00, 0xA6, 0x00, 0xA8, 0x00, 0xBF,
};

static const unsigned char gamma22_160[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xED, 0xDB, 0xE6,
	0xD4, 0xD7, 0xD9, 0xDC, 0xDD, 0xDD,
	0xB9, 0xB8, 0xB4, 0xC9, 0xC6, 0xC4,
	0x00, 0xAA, 0x00, 0xAC, 0x00, 0xC4,
};

static const unsigned char gamma22_170[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xEC, 0xDC, 0xE5,
	0xD5, 0xD8, 0xD9, 0xDD, 0xDC, 0xDD,
	0xBA, 0xB7, 0xB5, 0xC7, 0xC6, 0xC3,
	0x00, 0xAD, 0x00, 0xAF, 0x00, 0xC7,
};

static const unsigned char gamma22_180[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xEE, 0xDD, 0xE6,
	0xD4, 0xD7, 0xD9, 0xDB, 0xDC, 0xDB,
	0xB9, 0xB7, 0xB4, 0xC6, 0xC4, 0xC2,
	0x00, 0xB1, 0x00, 0xB3, 0x00, 0xCC,
};

static const unsigned char gamma22_190[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xED, 0xDE, 0xE6,
	0xD3, 0xD8, 0xD8, 0xDD, 0xDB, 0xDC,
	0xB9, 0xB6, 0xB4, 0xC5, 0xC4, 0xC0,
	0x00, 0xB4, 0x00, 0xB6, 0x00, 0xD0,
};

static const unsigned char gamma22_200[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xED, 0xDF, 0xE6,
	0xD3, 0xD7, 0xD8, 0xDB, 0xDB, 0xDA,
	0xB8, 0xB6, 0xB3, 0xC4, 0xC3, 0xC0,
	0x00, 0xB8, 0x00, 0xB9, 0x00, 0xD4,
};

static const unsigned char gamma22_210[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xEC, 0xE0, 0xE5,
	0xD5, 0xD7, 0xD9, 0xDB, 0xDA, 0xDA,
	0xB7, 0xB5, 0xB1, 0xC4, 0xC2, 0xC0,
	0x00, 0xBA, 0x00, 0xBD, 0x00, 0xD7,
};

static const unsigned char gamma22_220[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xED, 0xE0, 0xE6,
	0xD4, 0xD7, 0xD9, 0xDA, 0xDA, 0xD9,
	0xB7, 0xB4, 0xB1, 0xC2, 0xC2, 0xBE,
	0x00, 0xBE, 0x00, 0xC0, 0x00, 0xDC,
};

static const unsigned char gamma22_230[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xEC, 0xE2, 0xE6,
	0xD3, 0xD6, 0xD8, 0xDC, 0xD9, 0xD9,
	0xB6, 0xB4, 0xB1, 0xC1, 0xC1, 0xBD,
	0x00, 0xC1, 0x00, 0xC3, 0x00, 0xDF,
};

static const unsigned char gamma22_240[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xED, 0xE2, 0xE6,
	0xD4, 0xD6, 0xD8, 0xDA, 0xDA, 0xDA,
	0xB6, 0xB3, 0xB0, 0xC1, 0xBF, 0xBC,
	0x00, 0xC4, 0x00, 0xC7, 0x00, 0xE3,
};

static const unsigned char gamma22_250[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xED, 0xE3, 0xE7,
	0xD4, 0xD6, 0xD8, 0xDB, 0xD9, 0xD9,
	0xB3, 0xB2, 0xAE, 0xC1, 0xC0, 0xBC,
	0x00, 0xC7, 0x00, 0xC9, 0x00, 0xE7,
};

static const unsigned char gamma22_260[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xED, 0xE4, 0xE7,
	0xD4, 0xD5, 0xD7, 0xDA, 0xD9, 0xD9,
	0xB3, 0xB2, 0xAD, 0xC1, 0xBE, 0xBC,
	0x00, 0xC9, 0x00, 0xCD, 0x00, 0xEA,
};

static const unsigned char gamma22_270[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xED, 0xE5, 0xE8,
	0xD3, 0xD5, 0xD5, 0xDB, 0xD9, 0xD9,
	0xB3, 0xB1, 0xAE, 0xBF, 0xBE, 0xBA,
	0x00, 0xCC, 0x00, 0xD0, 0x00, 0xEE,
};

static const unsigned char gamma22_280[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xEC, 0xE5, 0xE6,
	0xD2, 0xD4, 0xD6, 0xDA, 0xD9, 0xD8,
	0xB3, 0xB1, 0xAD, 0xBF, 0xBD, 0xBA,
	0x00, 0xCF, 0x00, 0xD3, 0x00, 0xF1,
};

static const unsigned char gamma22_290[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xEC, 0xE6, 0xE7,
	0xD2, 0xD4, 0xD5, 0xDB, 0xD8, 0xD8,
	0xB1, 0xB0, 0xAC, 0xBE, 0xBD, 0xB9,
	0x00, 0xD3, 0x00, 0xD6, 0x00, 0xF5,
};

static const unsigned char gamma22_300[] = {
	0xFA, 0x01,
	0x1F, 0x1F, 0x1F, 0xED, 0xE6, 0xE7,
	0xD1, 0xD3, 0xD4, 0xDA, 0xD8, 0xD7,
	0xB1, 0xAF, 0xAB, 0xBD, 0xBB, 0xB8,
	0x00, 0xD6, 0x00, 0xDA, 0x00, 0xFA,
};


static const unsigned char *s6e8aa0_22_gamma_table[] = {
	gamma22_30,
	gamma22_40,
	gamma22_50,
	gamma22_60,
	gamma22_70,
	gamma22_80,
	gamma22_90,
	gamma22_100,
	gamma22_110,
	gamma22_120,
	gamma22_130,
	gamma22_140,
	gamma22_150,
	gamma22_160,
	gamma22_170,
	gamma22_180,
	gamma22_190,
	gamma22_200,
	gamma22_210,
	gamma22_220,
	gamma22_230,
	gamma22_240,
	gamma22_250,
	gamma22_260,
	gamma22_270,
	gamma22_280,
	gamma22_290,
};

static void s6e8aa0_panel_cond(struct s6e8aa0 *lcd)
{
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);

	static const unsigned char data_to_send[] = {
		0xF8,
		0x25, 0x34, 0x00, 0x00, 0x00, 0x95, 0x00, 0x3c, 0x7d, 0x08,
		0x27, 0x00, 0x00, 0x10, 0x00, 0x00, 0x20, 0x02, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x02, 0x08, 0x08, 0x23, 0x63, 0xc0, 0xc1,
		0x01, 0x81, 0xc1, 0x00, 0xc8, 0xc1, 0xd3, 0x01

	};

	static const unsigned char data_to_send_panel_reverse[] = {
		0xf8, 0x19, 0x35, 0x00, 0x00, 0x00, 0x93, 0x00, 0x3c, 0x7d,
		0x08, 0x27, 0x7d, 0x3f, 0x00, 0x00, 0x00, 0x20, 0x04, 0x08,
		0x6e, 0x00, 0x00, 0x00, 0x02, 0x08, 0x08, 0x23, 0x23, 0xc0,
		0xc1, 0x01, 0x41, 0xc1, 0x00, 0xc1, 0xf6, 0xf6, 0xc1
	};

	if (lcd->dsim_dev->panel_reverse) {
		pr_err("Panel Reverse Called\n");
		ops->cmd_write(lcd_to_master(lcd), MIPI_DSI_DCS_LONG_WRITE,
				data_to_send_panel_reverse,
				ARRAY_SIZE(data_to_send_panel_reverse));
	} else {
		ops->cmd_write(lcd_to_master(lcd),
		MIPI_DSI_DCS_LONG_WRITE, data_to_send,
		ARRAY_SIZE(data_to_send));
	}
}

static void s6e8aa0_display_cond(struct s6e8aa0 *lcd)
{
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);
	static const unsigned char data_to_send[] = {
		0xF2,
		0x80, 0x03, 0x0D
	};
	pr_err("func: %s, line: %d\n", __func__, __LINE__);

	ops->cmd_write(lcd_to_master(lcd), MIPI_DSI_DCS_LONG_WRITE,
		data_to_send, ARRAY_SIZE(data_to_send));
}

/* Gamma 2.2 Setting (200cd, 7500K, 10MPCD) */
static void s6e8aa0_gamma_cond(struct s6e8aa0 *lcd)
{
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);
	unsigned int gamma = lcd->bd->props.brightness;
	gamma = 15;

	ops->cmd_write(lcd_to_master(lcd), MIPI_DSI_DCS_LONG_WRITE,
			s6e8aa0_22_gamma_table[gamma],
			GAMMA_TABLE_COUNT);
}

static void s6e8aa0_gamma_update(struct s6e8aa0 *lcd)
{
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);
	static const unsigned char data_to_send[] = {
		0xF7,
		0x03, 0x00, 0x00
	};

	ops->cmd_write(lcd_to_master(lcd),
		MIPI_DSI_DCS_SHORT_WRITE_PARAM, data_to_send,
		ARRAY_SIZE(data_to_send));
}

static void s6e8aa0_etc_cond1(struct s6e8aa0 *lcd)
{
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);
	static const unsigned char data_to_send[] = {
		0xF6,
		0x00, 0x02, 0x00
	};

	ops->cmd_write(lcd_to_master(lcd), MIPI_DSI_DCS_LONG_WRITE,
		data_to_send, ARRAY_SIZE(data_to_send));
}

static void s6e8aa0_etc_cond2(struct s6e8aa0 *lcd)
{
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);
	static const unsigned char data_to_send[] = {
		0xB6,
		0x0C, 0x02, 0x03, 0x32, 0xC0, 0x44, 0x44, 0xC0, 0x00
	};

	ops->cmd_write(lcd_to_master(lcd), MIPI_DSI_DCS_LONG_WRITE,
		data_to_send, ARRAY_SIZE(data_to_send));
}

static void s6e8aa0_etc_cond3(struct s6e8aa0 *lcd)
{
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);
	static const unsigned char data_to_send[] = {
		0xF4,
		0xCF, 0x0A, 0x15, 0x10, 0x19, 0x33, 0x02
	};

	ops->cmd_write(lcd_to_master(lcd), MIPI_DSI_DCS_LONG_WRITE,
		data_to_send, ARRAY_SIZE(data_to_send));
}

static void s6e8aa0_elvss_set(struct s6e8aa0 *lcd)
{
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);
	static const unsigned char data_to_send[] = {
		0xB1, 0x04, 0x00
	};

	ops->cmd_write(lcd_to_master(lcd), MIPI_DSI_DCS_LONG_WRITE,
		data_to_send, ARRAY_SIZE(data_to_send));
}

static void s6e8aa0_elvss_nvm_set(struct s6e8aa0 *lcd)
{
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);
	static const unsigned char data_to_send[] = {
		0xD9,
		0x14, 0x40, 0x0C, 0xCB, 0xCE, 0x6E, 0xC4, 0x07, 0x40, 0x41,
		0xC1, 0x00, 0x60, 0x19
	};

	ops->cmd_write(lcd_to_master(lcd), MIPI_DSI_DCS_LONG_WRITE,
		data_to_send, ARRAY_SIZE(data_to_send));
}

static void s6e8aa0_sleep_in(struct s6e8aa0 *lcd)
{
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);
	static const unsigned char data_to_send[] = {
		0x10, 0x00
	};

	ops->cmd_write(lcd_to_master(lcd),
		MIPI_DSI_DCS_SHORT_WRITE,
		data_to_send, ARRAY_SIZE(data_to_send));
}

static void s6e8aa0_sleep_out(struct s6e8aa0 *lcd)
{
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);
	static const unsigned char data_to_send[] = {
		0x11, 0x00
	};

	ops->cmd_write(lcd_to_master(lcd),
		MIPI_DSI_DCS_SHORT_WRITE,
		data_to_send, ARRAY_SIZE(data_to_send));
}

static void s6e8aa0_display_on(struct s6e8aa0 *lcd)
{
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);
	static const unsigned char data_to_send[] = {
		0x29, 0x00
	};

	ops->cmd_write(lcd_to_master(lcd),
		MIPI_DSI_DCS_SHORT_WRITE,
		data_to_send, ARRAY_SIZE(data_to_send));
}

static void s6e8aa0_display_off(struct s6e8aa0 *lcd)
{
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);
	static const unsigned char data_to_send[] = {
		0x28, 0x00
	};

	ops->cmd_write(lcd_to_master(lcd),
		MIPI_DSI_DCS_SHORT_WRITE,
		data_to_send, ARRAY_SIZE(data_to_send));
}

static void s6e8aa0_apply_level2_key(struct s6e8aa0 *lcd)
{
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);
	static const unsigned char data_to_send[] = {
		0xfc, 0x5a, 0x5a
	};

	ops->cmd_write(lcd_to_master(lcd), MIPI_DSI_DCS_LONG_WRITE,
		data_to_send, ARRAY_SIZE(data_to_send));
}

static void s6e8aa0_read_id(struct s6e8aa0 *lcd, u8 *mtp_id)
{
	unsigned int ret;
	unsigned int addr = 0xD1;	/* MTP ID */
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);

	ret = ops->cmd_read(lcd_to_master(lcd),
			MIPI_DSI_GENERIC_READ_REQUEST_1_PARAM,
			addr, 3, mtp_id);
}

static int s6e8aa0_panel_init(struct s6e8aa0 *lcd)
{
	s6e8aa0_apply_level2_key(lcd);
	mdelay(16);
	s6e8aa0_sleep_out(lcd);
	mdelay(5);
	s6e8aa0_panel_cond(lcd);
	s6e8aa0_display_cond(lcd);
	s6e8aa0_gamma_cond(lcd);
	s6e8aa0_gamma_update(lcd);

	s6e8aa0_etc_cond1(lcd);
	s6e8aa0_etc_cond2(lcd);
	s6e8aa0_etc_cond3(lcd);

	s6e8aa0_elvss_nvm_set(lcd);
	s6e8aa0_elvss_set(lcd);
	mdelay(120);

	return 0;
}

static int s6e8aa0_update_gamma_ctrl(struct s6e8aa0 *lcd, int brightness)
{
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);

	ops->cmd_write(lcd_to_master(lcd), MIPI_DSI_DCS_LONG_WRITE,
			s6e8aa0_22_gamma_table[brightness],
			ARRAY_SIZE(s6e8aa0_22_gamma_table));

	/* update gamma table. */
	s6e8aa0_gamma_update(lcd);
	lcd->gamma = brightness;

	return 0;
}

static int s6e8aa0_gamma_ctrl(struct s6e8aa0 *lcd, int gamma)
{
	s6e8aa0_update_gamma_ctrl(lcd, gamma);
	return 0;
}

static int s6e8aa0_set_power(struct lcd_device *ld, int power)
{
	struct s6e8aa0 *lcd = lcd_get_data(ld);
	struct mipi_dsim_master_ops *ops = lcd_to_master_ops(lcd);
	int ret = 0;

	if (power != FB_BLANK_UNBLANK && power != FB_BLANK_POWERDOWN &&
			power != FB_BLANK_NORMAL) {
		dev_err(lcd->dev, "power value should be 0, 1 or 4.\n");
		return -EINVAL;
	}

	if ((power == FB_BLANK_UNBLANK) && ops->set_blank_mode) {
		/* LCD power on */
		if ((POWER_IS_ON(power) && POWER_IS_OFF(lcd->power))
			|| (POWER_IS_ON(power) && POWER_IS_NRM(lcd->power))) {
			ret = ops->set_blank_mode(lcd_to_master(lcd), power);
			if (!ret && lcd->power != power)
				lcd->power = power;
		}
	} else if ((power == FB_BLANK_POWERDOWN) && ops->set_early_blank_mode) {
		/* LCD power off */
		if ((POWER_IS_OFF(power) && POWER_IS_ON(lcd->power)) ||
		(POWER_IS_ON(lcd->power) && POWER_IS_NRM(power))) {
			ret = ops->set_early_blank_mode(lcd_to_master(lcd),
							power);
			if (!ret && lcd->power != power)
				lcd->power = power;
		}
	}

	return ret;
}

static int s6e8aa0_get_power(struct lcd_device *ld)
{
	struct s6e8aa0 *lcd = lcd_get_data(ld);

	return lcd->power;
}

static int s6e8aa0_get_brightness(struct backlight_device *bd)
{
	return bd->props.brightness;
}

static int s6e8aa0_set_brightness(struct backlight_device *bd)
{
	int ret = 0, brightness = bd->props.brightness;
	struct s6e8aa0 *lcd = bl_get_data(bd);

	if (brightness < MIN_BRIGHTNESS ||
		brightness > bd->props.max_brightness) {
		dev_err(lcd->dev, "lcd brightness should be %d to %d.\n",
			MIN_BRIGHTNESS, MAX_BRIGHTNESS);
		return -EINVAL;
	}

	ret = s6e8aa0_gamma_ctrl(lcd, brightness);
	if (ret) {
		dev_err(&bd->dev, "lcd brightness setting failed.\n");
		return -EIO;
	}

	return ret;
}

static struct lcd_ops s6e8aa0_lcd_ops = {
	.set_power = s6e8aa0_set_power,
	.get_power = s6e8aa0_get_power,
};

static const struct backlight_ops s6e8aa0_backlight_ops = {
	.get_brightness = s6e8aa0_get_brightness,
	.update_status = s6e8aa0_set_brightness,
};

static void s6e8aa0_power_on(struct mipi_dsim_lcd_device *dsim_dev, int power)
{
	struct s6e8aa0 *lcd = dev_get_drvdata(&dsim_dev->dev);

	msleep(lcd->ddi_pd->power_on_delay);

       if (power) {
               gpio_request_one(lcd->gpio_reset, GPIOF_OUT_INIT_HIGH, 0);
               mdelay(500);
               gpio_set_value(lcd->gpio_reset, 0);
               mdelay(500);
               gpio_set_value(lcd->gpio_reset, 1);
               gpio_free(lcd->gpio_reset);
               mdelay(5);

               gpio_request_one(lcd->gpio_power, GPIOF_OUT_INIT_HIGH, 0);
               gpio_set_value(lcd->gpio_power, 1);
               gpio_free(lcd->gpio_power);

               gpio_request_one(lcd->gpio_bl, GPIOF_OUT_INIT_HIGH, 0);
               gpio_set_value(lcd->gpio_bl, 1);
               gpio_free(lcd->gpio_bl);

       } else {
               gpio_request_one(lcd->gpio_reset, GPIOF_OUT_INIT_LOW, 0);
               mdelay(500);
               gpio_free(lcd->gpio_reset);

               gpio_request_one(lcd->gpio_power, GPIOF_OUT_INIT_LOW, 0);
               mdelay(500);
               gpio_free(lcd->gpio_power);
       }

       mdelay(500);

	/* lcd power on */
	if (power)
		s6e8aa0_regulator_enable(lcd);
	else
		s6e8aa0_regulator_disable(lcd);

	msleep(lcd->ddi_pd->reset_delay);

	/* lcd reset */
	if (lcd->ddi_pd->reset)
		lcd->ddi_pd->reset(lcd->ld);
}

static void s6e8aa0_set_sequence(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct s6e8aa0 *lcd = dev_get_drvdata(&dsim_dev->dev);
	u8 mtp_id[3] = {0, };

	s6e8aa0_read_id(lcd, mtp_id);
	if (mtp_id[0] == 0x00)
		dev_err(lcd->dev, "read id failed\n");

	dev_info(lcd->dev, "Read ID : %x, %x, %x\n",
			mtp_id[0], mtp_id[1], mtp_id[2]);

	if (mtp_id[2] == 0x33)
		dev_info(lcd->dev,
			"ID-3 is 0xff does not support dynamic elvss\n");
	else
		dev_info(lcd->dev,
			"ID-3 is 0x%x support dynamic elvss\n", mtp_id[2]);

	s6e8aa0_panel_init(lcd);
	s6e8aa0_display_on(lcd);

	lcd->power = FB_BLANK_UNBLANK;
}

static int s6e8aa0_update_platform_lcd_data(
					struct s6e8aa0 *lcd_s6e8aa0)
{
	struct lcd_platform_data *ddi_pd = lcd_s6e8aa0->ddi_pd;
	struct device_node *np = (struct device_node *)
					lcd_s6e8aa0->ddi_pd->pdata;

	lcd_s6e8aa0->gpio_reset = of_get_named_gpio(np, "gpio-reset", 0);
	if (!gpio_is_valid(lcd_s6e8aa0->gpio_reset)) {
		dev_err(lcd_s6e8aa0->dev,
			"failed to get poweron gpio-reset information.\n");
		return -EINVAL;
	}

	lcd_s6e8aa0->gpio_power = of_get_named_gpio(np, "gpio-power", 0);
	if (!gpio_is_valid(lcd_s6e8aa0->gpio_power)) {
		dev_err(lcd_s6e8aa0->dev,
			"failed to get poweron gpio-power information.\n");
		return -EINVAL;
	}

	lcd_s6e8aa0->gpio_bl = of_get_named_gpio(np, "gpio-bl", 0);
	if (!gpio_is_valid(lcd_s6e8aa0->gpio_bl)) {
		dev_err(lcd_s6e8aa0->dev,
			"failed to get pwm-bl information.\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "enabled",
			(unsigned int *)&ddi_pd->lcd_enabled))
		ddi_pd->lcd_enabled = 0;

	if (of_property_read_u32(np, "reset-delay",
				&ddi_pd->reset_delay)) {
		dev_err(lcd_s6e8aa0->dev, "reset-delay property not found");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "power-on-delay",
				&ddi_pd->power_on_delay)) {
		dev_err(lcd_s6e8aa0->dev, "power-on-delay property not found");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "power-off-delay",
				&ddi_pd->power_off_delay)) {
		dev_err(lcd_s6e8aa0->dev,
			"power-off-delay property not found");
		return -EINVAL;
	}

	return 0;
}



static int s6e8aa0_probe(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct s6e8aa0 *lcd;
	int ret;

	lcd = kzalloc(sizeof(struct s6e8aa0), GFP_KERNEL);
	if (!lcd) {
		dev_err(&dsim_dev->dev, "failed to allocate s6e8aa0 structure.\n");
		return -ENOMEM;
	}

	lcd_global = lcd;

	lcd->dsim_dev = dsim_dev;
	lcd->ddi_pd = (struct lcd_platform_data *)dsim_dev->platform_data;
	lcd->dev = &dsim_dev->dev;

	/* get platform data information, if lcd device node is present */
	if (lcd->ddi_pd->pdata)
		if (s6e8aa0_update_platform_lcd_data(lcd))
			return -EINVAL;

	mutex_init(&lcd->lock);

	ret = regulator_bulk_get(dsim_dev->master->dev,
				ARRAY_SIZE(supplies), supplies);
	if (ret) {
		dev_err(lcd->dev, "Failed to get regulators: %d\n", ret);
		goto err_lcd_register;
	}

	lcd->ld = lcd_device_register("s6e8aa0", lcd->dev, lcd,
			&s6e8aa0_lcd_ops);
	if (IS_ERR(lcd->ld)) {
		dev_err(lcd->dev, "failed to register lcd ops.\n");
		ret = PTR_ERR(lcd->ld);
		goto err_lcd_register;
	}

	lcd->bd = backlight_device_register("s6e8aa0-bl", lcd->dev, lcd,
			&s6e8aa0_backlight_ops, NULL);
	if (IS_ERR(lcd->bd)) {
		dev_err(lcd->dev, "failed to register backlight ops.\n");
		ret = PTR_ERR(lcd->bd);
		goto err_backlight_register;
	}
	lcd->bd->props.max_brightness = MAX_BRIGHTNESS;
	lcd->bd->props.brightness = MAX_BRIGHTNESS;

	dev_set_drvdata(&dsim_dev->dev, lcd);

	dev_dbg(lcd->dev, "probed s6e8aa0 panel driver.\n");

	return 0;

err_backlight_register:
	lcd_device_unregister(lcd->ld);

err_lcd_register:
	regulator_bulk_free(ARRAY_SIZE(supplies), supplies);
	kfree(lcd);

	return ret;
}

#ifdef CONFIG_PM
static int s6e8aa0_suspend(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct s6e8aa0 *lcd = dev_get_drvdata(&dsim_dev->dev);

	s6e8aa0_sleep_in(lcd);
	msleep(lcd->ddi_pd->power_off_delay);
	s6e8aa0_display_off(lcd);

	s6e8aa0_regulator_disable(lcd);

	return 0;
}

static int s6e8aa0_resume(struct mipi_dsim_lcd_device *dsim_dev)
{
	struct s6e8aa0 *lcd = dev_get_drvdata(&dsim_dev->dev);

	s6e8aa0_sleep_out(lcd);
	msleep(lcd->ddi_pd->power_on_delay);

	s6e8aa0_regulator_enable(lcd);
	s6e8aa0_set_sequence(dsim_dev);

	return 0;
}
#else
#define s6e8aa0_suspend		NULL
#define s6e8aa0_resume		NULL
#endif

static struct mipi_dsim_lcd_driver s6e8aa0_dsim_ddi_driver = {
	.name = "s6e8ax0",
	.id = -1,

	.power_on = s6e8aa0_power_on,
	.set_sequence = s6e8aa0_set_sequence,
	.probe = s6e8aa0_probe,
	.suspend = s6e8aa0_suspend,
	.resume = s6e8aa0_resume,
};

static int s6e8aa0_init(void)
{
	exynos_mipi_dsi_register_lcd_driver(&s6e8aa0_dsim_ddi_driver);

	return 0;
}

static void s6e8aa0_exit(void)
{
	return;
}

module_init(s6e8aa0_init);
module_exit(s6e8aa0_exit);

MODULE_AUTHOR("Shaik Ameer Basha <shaik.ameer@samsung.com>");
MODULE_DESCRIPTION("MIPI-DSI based s6e8aa0 AMOLED LCD Panel Driver");
MODULE_LICENSE("GPL");

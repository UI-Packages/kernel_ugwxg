/*
 * (C) Copyright 2009-2010
 * Nokia Siemens Networks, michael.lawnick.ext@nsn.com
 *
 * Portions Copyright (C) 2010 - 2013 Cavium, Inc.
 *
 * This is a driver for the i2c adapter in Cavium Networks' OCTEON processors.
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/atomic.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_i2c.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/of.h>

#include <asm/octeon/octeon.h>

#define DRV_NAME "i2c-octeon"

#define DRV_VERSION	"2.6"

/* register offsets */
#define SW_TWSI		0x00
#define TWSI_INT	0x10
#define SW_TWSI_EXT	0x18

/* Controller command patterns */
#define SW_TWSI_V               0x8000000000000000ull
#define SW_TWSI_EIA		(1ull << 61)
#define SW_TWSI_R		(1ull << 56)
#define SW_TWSI_SOVR		(1ull << 55)
#define SW_TWSI_OP_7		(0ull << 57)
#define SW_TWSI_OP_7_IA		(1ull << 57)
#define SW_TWSI_OP_10		(2ull << 57)
#define SW_TWSI_OP_10_IA	(3ull << 57)
#define SW_TWSI_SIZE_SHIFT	52
#define SW_TWSI_A_SHIFT		40
#define SW_TWSI_IA_SHIFT	32
#define SW_TWSI_EOP_TWSI_DATA   0x0C00000100000000ull
#define SW_TWSI_EOP_TWSI_CTL    0x0C00000200000000ull
#define SW_TWSI_EOP_TWSI_CLKCTL 0x0C00000300000000ull
#define SW_TWSI_EOP_TWSI_STAT   0x0C00000300000000ull
#define SW_TWSI_EOP_TWSI_RST    0x0C00000700000000ull
#define SW_TWSI_OP_TWSI_CLK     0x0800000000000000ull

/* Controller command and status bits */
#define TWSI_CTL_CE   0x80	/* HighLevelController enable */
#define TWSI_CTL_ENAB 0x40	/* bus enable */
#define TWSI_CTL_STA  0x20	/* master-mode start, hw clears when done */
#define TWSI_CTL_STP  0x10	/* master-mode stop, hw clears when done */
#define TWSI_CTL_IFLG 0x08	/* hw event, sw writes 0 to ack */
#define TWSI_CTL_AAK  0x04	/* Assert ACK */

/* Some status values - named with value, as HRM speaks of values */
#define STAT_ERROR_00		0x00
#define STAT_START_08		0x08
#define STAT_RSTART_10		0x10
#define STAT_TXADDR_ACK_18	0x18
#define STAT_TXADDR_NAK_20	0x20
#define STAT_TXDATA_ACK_28	0x28
#define STAT_TXDATA_NAK_30	0x30
#define STAT_LOST_ARB_38	0x38
#define STAT_RXADDR_ACK_40	0x40
#define STAT_RXADDR_NAK_48	0x48
#define STAT_RXDATA_ACK_50	0x50
#define STAT_RXDATA_NAK_58	0x58
#define STAT_SLAVE_60		0x60
#define STAT_LOST_ARB_68	0x68
#define STAT_SLAVE_70		0x70
#define STAT_LOST_ARB_78	0x78
#define STAT_SLAVE_80		0x80
#define STAT_SLAVE_88		0x88
#define STAT_GENDATA_ACK_90	0x90
#define STAT_GENDATA_NAK_98	0x98
#define STAT_SLAVE_A0		0xA0
#define STAT_SLAVE_A8		0xA8
#define STAT_LOST_ARB_B0	0xB0
#define STAT_SLAVE_LOST_B8	0xB8
#define STAT_SLAVE_NAK_C0	0xC0
#define STAT_SLAVE_ACK_C8	0xC8
#define STAT_AD2W_ACK_D0	0xD0
#define STAT_AD2W_NAK_D8	0xD8
#define STAT_IDLE_F8		0xF8

/* TWSI_INT values */
#define ST_INT			0x01
#define TS_INT			0x02
#define CORE_INT		0x04
#define ST_EN			0x10
#define TS_EN			0x20
#define CORE_EN			0x40
#define SDA_OVR			0x100
#define SCL_OVR			0x200
#define SDA			0x400
#define SCL			0x800

#define _DEAD_LOCK_RECOVERY_

struct octeon_i2c {
	wait_queue_head_t queue;
	struct i2c_adapter adap;
	int irq;
	int hlc_irq;		/* For cn7890 only */
	u32 twsi_freq;
	int sys_freq;
	void __iomem *twsi_base;
	struct device *dev;
	int broken_irq_mode;
	bool octeon_i2c_hlc_enabled;
	int cvmx_channel;
	void (*int_en)(struct octeon_i2c *);
	void (*int_dis)(struct octeon_i2c *);
	void (*hlc_int_en)(struct octeon_i2c *);
	void (*hlc_int_dis)(struct octeon_i2c *);
	atomic_t int_en_cnt;
	atomic_t hlc_int_en_cnt;
};

static int timeout = 2;
module_param(timeout, int, 0444);
MODULE_PARM_DESC(timeout, "low-level device timeout (mS)");

/*
 * on some hardware IFLG is not visible in TWSI_CTL until after
 * low-level IRQ, so re-sample CTL a short time later to avoid stalls
 */
static int irq_early_us = 80;
module_param(irq_early_us, int, 0644);
MODULE_PARM_DESC(irq_early_us, "re-poll for IFLG after IRQ (uS)");

static int octeon_i2c_initlowlevel(struct octeon_i2c *i2c);
static int octeon_i2c_enable_hlc(struct octeon_i2c *i2c);
static int octeon_i2c_disable_hlc(struct octeon_i2c *i2c);
static void octeon_i2c_stop(struct octeon_i2c *i2c);
static void bitbang_unblock(struct octeon_i2c *i2c);

static inline void writeqflush(u64 v, volatile void __iomem *a)
{
	__raw_writeq(v, a);
	__raw_readq(a); /* wait for write to land */
}

/**
 * octeon_i2c_write_sw - write an I2C core register.
 * @i2c: The struct octeon_i2c.
 * @eop_reg: Register selector.
 * @data: Value to be written.
 *
 * The I2C core registers are accessed indirectly via the SW_TWSI CSR.
 */
static void octeon_i2c_write_sw(struct octeon_i2c *i2c,
				u64 eop_reg,
				u32 data)
{
	u64 tmp;
#ifdef _DEAD_LOCK_RECOVERY_
	u64 end = get_jiffies_64() + (1 * HZ);
#endif

	__raw_writeq(SW_TWSI_V | eop_reg | data, i2c->twsi_base + SW_TWSI);
	do {
		tmp = __raw_readq(i2c->twsi_base + SW_TWSI);
#ifdef _DEAD_LOCK_RECOVERY_
		if (get_jiffies_64() <= end) {
			udelay(50);
		} else {
			bitbang_unblock(i2c);
			return;
		}
#endif
	} while ((tmp & SW_TWSI_V) != 0);
}

/**
 * octeon_i2c_read_sw64 - read an I2C core register.
 * @i2c: The struct octeon_i2c.
 * @eop_reg: Register selector.
 *
 * Returns the data.
 *
 * The I2C core registers are accessed indirectly via the SW_TWSI CSR.
 */
static u64 octeon_i2c_read_sw64(struct octeon_i2c *i2c, u64 eop_reg)
{
	u64 tmp;
#ifdef _DEAD_LOCK_RECOVERY_
	u64 end = get_jiffies_64() + (1 * HZ);
#endif

	__raw_writeq(SW_TWSI_V | eop_reg | SW_TWSI_R, i2c->twsi_base + SW_TWSI);
	do {
		tmp = __raw_readq(i2c->twsi_base + SW_TWSI);
#ifdef _DEAD_LOCK_RECOVERY_
		if (get_jiffies_64() <= end) {
			udelay(50);
		} else {
			bitbang_unblock(i2c);
			return 0;
		}
#endif
	} while ((tmp & SW_TWSI_V) != 0);

	return tmp;
}

/**
 * octeon_i2c_read_sw - read lower bits of an I2C core register.
 * @i2c: The struct octeon_i2c.
 * @eop_reg: Register selector.
 *
 * Returns the data.
 *
 * The I2C core registers are accessed indirectly via the SW_TWSI CSR.
 */
static inline u8 octeon_i2c_read_sw(struct octeon_i2c *i2c, u64 eop_reg)
{
	return (u8)octeon_i2c_read_sw64(i2c, eop_reg);
}
/**
 * octeon_i2c_write_int - write the TWSI_INT register
 * @i2c: The struct octeon_i2c.
 * @data: Value to be written.
 */
static inline void octeon_i2c_write_int(struct octeon_i2c *i2c, u64 data)
{
	writeqflush(data, i2c->twsi_base + TWSI_INT);
}

/**
 * octeon_i2c_int_enable - enable the CORE interrupt.
 * @i2c: The struct octeon_i2c.
 *
 * The interrupt will be asserted when there is non-STAT_IDLE_F8 state in
 * the SW_TWSI_EOP_TWSI_STAT register.
 */
static void octeon_i2c_int_enable(struct octeon_i2c *i2c)
{
	/* enable CORE_INT */
	octeon_i2c_write_int(i2c, CORE_EN);
}

/**
 * octeon_i2c_int_disable - disable the CORE interrupt.
 * @i2c: The struct octeon_i2c.
 */
static void octeon_i2c_int_disable(struct octeon_i2c *i2c)
{
	/* disable CORE_INT, clear TS/ST/IFLG events */
	octeon_i2c_write_int(i2c, TS_INT | ST_INT);
}

/**
 * octeon_i2c_int_enable78 - enable the CORE interrupt.
 * @i2c: The struct octeon_i2c.
 *
 * The interrupt will be asserted when there is non-STAT_IDLE_F8 state in
 * the SW_TWSI_EOP_TWSI_STAT register.
 */
static void octeon_i2c_int_enable78(struct octeon_i2c *i2c)
{
	atomic_inc_return(&i2c->int_en_cnt);
	enable_irq(i2c->irq);
}

/**
 * octeon_i2c_int_disable78 - disable the CORE interrupt.
 * @i2c: The struct octeon_i2c.
 */
static void octeon_i2c_int_disable78(struct octeon_i2c *i2c)
{
	/*
	 * The interrupt can be disabled in two places, but we only
	 * want to make the disable_irq_nosync() call once, so keep
	 * track with the atomic variable.
	 */
	int c = atomic_dec_if_positive(&i2c->int_en_cnt);
	if (c >= 0)
		disable_irq_nosync(i2c->irq);
}

/**
 * octeon_i2c_hlc_int_enable78 - enable the ST interrupt.
 * @i2c: The struct octeon_i2c.
 *
 * The interrupt will be asserted when there is non-STAT_IDLE_F8 state in
 * the SW_TWSI_EOP_TWSI_STAT register.
 */
static void octeon_i2c_hlc_int_enable78(struct octeon_i2c *i2c)
{
	atomic_inc_return(&i2c->hlc_int_en_cnt);
	enable_irq(i2c->hlc_irq);
}

/**
 * octeon_i2c_hlc_int_disable78 - disable the ST interrupt.
 * @i2c: The struct octeon_i2c.
 */
static void octeon_i2c_hlc_int_disable78(struct octeon_i2c *i2c)
{
	/*
	 * The interrupt can be disabled in two places, but we only
	 * want to make the disable_irq_nosync() call once, so keep
	 * track with the atomic variable.
	 */
	int c = atomic_dec_if_positive(&i2c->hlc_int_en_cnt);
	if (c >= 0)
		disable_irq_nosync(i2c->hlc_irq);
}

/**
 * bitbang_unblock - unblock the bus.
 * @i2c: The struct octeon_i2c.
 *
 * If there was a reset while a device was driving 0 to bus,
 * bus is blocked. We toggle it free manually by some clock
 * cycles and send a stop.
 */
static void bitbang_unblock(struct octeon_i2c *i2c)
{
	int i;

	dev_dbg(i2c->dev, "%s\n", __func__);
	octeon_i2c_disable_hlc(i2c);

	/* cycle 8+1 clocks with SDA high */
	for (i = 0; i < 9; i++) {
		int state;
		octeon_i2c_write_int(i2c, 0);
		udelay(5);
		state = __raw_readq(i2c->twsi_base + TWSI_INT);
		if (state & (SDA|SCL))
			break;
		octeon_i2c_write_int(i2c, SCL_OVR);
		udelay(5);
	}
	/* hand-crank a STOP */
	octeon_i2c_write_int(i2c, SDA_OVR | SCL_OVR);
	udelay(5);
	octeon_i2c_write_int(i2c, SDA_OVR);
	udelay(5);
	octeon_i2c_write_int(i2c, 0);
}

/**
 * octeon_bitbang_unblock - unblock the bus without HLC disabled.
 * @adap: The i2c adapter.
 *
 * If there was a reset while a device was driving 0 to bus,
 * bus is blocked. We toggle it free manually by some clock
 * cycles and send a stop.
 */
void octeon_bitbang_unblock(struct i2c_adapter *adap)
{
	struct octeon_i2c *i2c = i2c_get_adapdata(adap);
	int i;

	/* cycle 8+1 clocks with SDA high */
	for (i = 0; i < 9; i++) {
		int state;
		octeon_i2c_write_int(i2c, 0);
		udelay(5);
		state = __raw_readq(i2c->twsi_base + TWSI_INT);
		if (state & (SDA|SCL))
			break;
		octeon_i2c_write_int(i2c, SCL_OVR);
		udelay(5);
	}
	/* hand-crank a STOP */
	octeon_i2c_write_int(i2c, SDA_OVR | SCL_OVR);
	udelay(5);
	octeon_i2c_write_int(i2c, SDA_OVR);
	udelay(5);
	octeon_i2c_write_int(i2c, 0);
}
EXPORT_SYMBOL(octeon_bitbang_unblock);

/**
 * octeon_i2c_isr - the interrupt service routine.
 * @int: The irq, unused.
 * @dev_id: Our struct octeon_i2c.
 */
static irqreturn_t octeon_i2c_isr(int irq, void *dev_id)
{
	struct octeon_i2c *i2c = dev_id;

	i2c->int_dis(i2c);
	wake_up(&i2c->queue);

	return IRQ_HANDLED;
}

/**
 * octeon_hlc_i2c_isr78 - the interrupt service routine.
 * @int: The irq, unused.
 * @dev_id: Our struct octeon_i2c.
 */
static irqreturn_t octeon_i2c_hlc_isr78(int irq, void *dev_id)
{
	struct octeon_i2c *i2c = dev_id;

	i2c->hlc_int_dis(i2c);
	wake_up(&i2c->queue);

	return IRQ_HANDLED;
}

static inline u64 octeon_i2c_read_ctl(struct octeon_i2c *i2c)
{
	return octeon_i2c_read_sw64(i2c, SW_TWSI_EOP_TWSI_CTL);
}

static inline int octeon_i2c_test_iflg(struct octeon_i2c *i2c)
{
	return (octeon_i2c_read_ctl(i2c) & TWSI_CTL_IFLG) != 0;
}

/*
 * poll_iflg - a wait-helper which addresses the delayed-IFLAG problem
 * by re-polling for missing TWSI_CTL[IFLG] a few uS later,
 * when irq has signalled an event, but none found.
 * Skip this re-poll on the first (non-wakeup) call
 */
static bool poll_iflg(struct octeon_i2c *i2c, bool *first_p)
{
	int iflg = octeon_i2c_test_iflg(i2c);

	if (iflg)
		return true;
	if (*first_p) {
		*first_p = false;
	} else {
		usleep_range(irq_early_us, 2 * irq_early_us);
		iflg = octeon_i2c_test_iflg(i2c);
	}
	return iflg;
}

/**
 * octeon_i2c_wait - wait for the IFLG to be set.
 * @i2c: The struct octeon_i2c.
 *
 * Returns 0 on success, otherwise a negative errno.
 */
static int octeon_i2c_wait(struct octeon_i2c *i2c)
{
	bool first = true;
	int result;

	if (i2c->broken_irq_mode) {
		/*
		 * Some cn38xx boards did not assert the irq in
		 * the interrupt controller.  So we must poll for the
		 * IFLG change.
		 */
		u64 end = get_jiffies_64() + i2c->adap.timeout;

		while (!octeon_i2c_test_iflg(i2c) && get_jiffies_64() <= end)
			udelay(50);

		return octeon_i2c_test_iflg(i2c) ? 0 : -ETIMEDOUT;
	}

	i2c->int_en(i2c);

	result = wait_event_timeout(i2c->queue,
				    poll_iflg(i2c, &first),
				    i2c->adap.timeout);

	i2c->int_dis(i2c);


	if (result <= 0 && OCTEON_IS_MODEL(OCTEON_CN38XX) &&
			octeon_i2c_test_iflg(i2c)) {
		dev_err(i2c->dev, "broken irq connection detected, switching to polling mode.\n");
		i2c->broken_irq_mode = 1;
		return 0;
	}

	if (result < 0) {
		dev_dbg(i2c->dev, "%s: wait interrupted\n", __func__);
		return result;
	} else if (result == 0) {
		dev_dbg(i2c->dev, "%s: timeout\n", __func__);
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * octeon_i2c_enable_hlc - cleanup low-level state & enable high-level
 *
 * Returns -EAGAIN if low-level state could not be cleaned
 */
static inline int octeon_i2c_enable_hlc(struct octeon_i2c *i2c)
{
	u64 v;
	int try = 0;
	int ret = 0;

	if (i2c->octeon_i2c_hlc_enabled)
		return 0;

	i2c->octeon_i2c_hlc_enabled = true;

	while ((v = octeon_i2c_read_ctl(i2c)) & (TWSI_CTL_STA | TWSI_CTL_STP)) {
		/* clear _IFLG event */
		if (v & TWSI_CTL_IFLG)
			octeon_i2c_write_sw(i2c,
				SW_TWSI_EOP_TWSI_CTL, TWSI_CTL_ENAB);

		if (try++ > 100) {
			static bool once = 1;
			if (once)
				dev_dbg(i2c->dev, "%s v:%llx EAGAIN\n",
					__func__, v);
			once = 0;
			ret = -EAGAIN;
			break;
		}

		/* spin until any start/stop has finished */
		udelay(10);
	}

	octeon_i2c_write_sw(i2c, SW_TWSI_EOP_TWSI_CTL,
		TWSI_CTL_CE | TWSI_CTL_AAK | TWSI_CTL_ENAB);
	return ret;
}

static int octeon_i2c_disable_hlc(struct octeon_i2c *i2c)
{
	if (!i2c->octeon_i2c_hlc_enabled)
		return 0;

	i2c->octeon_i2c_hlc_enabled = false;
	octeon_i2c_write_sw(i2c, SW_TWSI_EOP_TWSI_CTL, TWSI_CTL_ENAB);
	return 0;
}

static int octeon_i2c_lost_arb(u8 code, bool final_read)
{
	switch (code) {
	/* Arbitration lost */
	case STAT_LOST_ARB_38:
	case STAT_LOST_ARB_68:
	case STAT_LOST_ARB_78:
	case STAT_LOST_ARB_B0:
		return -EAGAIN;

	/* being addressed as slave, should back off & listen */
	case STAT_SLAVE_60:
	case STAT_SLAVE_70:
	case STAT_GENDATA_ACK_90:
	case STAT_GENDATA_NAK_98:
		return -EIO;

	/* Core busy as slave */
	case STAT_SLAVE_80:
	case STAT_SLAVE_88:
	case STAT_SLAVE_A0:
	case STAT_SLAVE_A8:
	case STAT_SLAVE_LOST_B8:
	case STAT_SLAVE_NAK_C0:
	case STAT_SLAVE_ACK_C8:
		return -EIO;

	/* ACK allowed on pre-terminal bytes only */
	case STAT_RXDATA_ACK_50:
		if (!final_read)
			return 0;
		return -EAGAIN;
	/* NAK allowed on terminal byte only */
	case STAT_RXDATA_NAK_58:
		if (final_read)
			return 0;
		return -EAGAIN;
	case STAT_TXDATA_NAK_30:
	case STAT_TXADDR_NAK_20:
	case STAT_RXADDR_NAK_48:
	case STAT_AD2W_NAK_D8:
		return -EAGAIN;
	default:
		return 0;
	}
}

static inline int check_arb(struct octeon_i2c *i2c, bool final_read)
{
	return octeon_i2c_lost_arb(
		octeon_i2c_read_sw(i2c, SW_TWSI_EOP_TWSI_STAT),
		final_read);
}

/**
 * octeon_i2c_start - send START to the bus.
 * @i2c: The struct octeon_i2c.
 * @first: Start, not ReStart?
 *
 * Returns 0 on success, otherwise a negative errno.
 */
static int octeon_i2c_start(struct octeon_i2c *i2c, bool first)
{
	u8 data;
	int result;
	static int reset_how;

	octeon_i2c_disable_hlc(i2c);

	while (true) {
		octeon_i2c_write_sw(i2c, SW_TWSI_EOP_TWSI_CTL,
					TWSI_CTL_ENAB | TWSI_CTL_STA);

		result = octeon_i2c_wait(i2c);
		data = octeon_i2c_read_sw(i2c, SW_TWSI_EOP_TWSI_STAT);

		switch (data) {
		case STAT_START_08:
			if (!first)
				return -EAGAIN;
			reset_how = 0;
			return 0;

		case STAT_RSTART_10:
			if (first)
				return -EAGAIN;
			reset_how = 0;
			return 0;

		case STAT_RXADDR_ACK_40:
			if (first)
				return -EAGAIN;
			goto unstick;
		case STAT_IDLE_F8:
		case STAT_ERROR_00:
		default:
			if (!first)
				return -EAGAIN;
unstick:
			/*
			 * TWSI state seems stuck. Not sure if it's TWSI-engine
			 * state or something else on bus.
			 * The initial _stop() is always harmless, it just
			 * resets state machine, does not _transmit_ STOP
			 * unless engine was active
			 */
			octeon_i2c_stop(i2c);

			/*
			 * response is escalated over successive calls,
			 * as EAGAIN provokes retries from i2c/core
			 */
			switch (reset_how++ % 4) {
			case 0:
				/* just the _stop above */
				break;
			case 1:
				/*
				 * Controller refused to send start flag
				 * May be a client is holding SDA low?
				 * Let's try to free it.
				 */
				bitbang_unblock(i2c);
				break;

			case 2:
				/* re-init our TWSI hardware */
				octeon_i2c_initlowlevel(i2c);
				break;
			default:
				/* retry in caller */
				reset_how = 0;
			return -EAGAIN;
			}
		}
	}
}

/**
 * octeon_i2c_stop - send STOP to the bus.
 * @i2c: The struct octeon_i2c.
 */
static void octeon_i2c_stop(struct octeon_i2c *i2c)
{
	octeon_i2c_write_sw(i2c, SW_TWSI_EOP_TWSI_CTL,
			    TWSI_CTL_ENAB | TWSI_CTL_STP);
}

/**
 * octeon_i2c_write - send data to the bus via low-level controller.
 * @i2c: The struct octeon_i2c.
 * @target: Target address.
 * @data: Pointer to the data to be sent.
 * @length: Length of the data.
 * @last: is last msg in combined operation?
 *
 * The address is sent over the bus, then the data.
 *
 * Returns 0 on success, otherwise a negative errno.
 */
static int octeon_i2c_write(struct octeon_i2c *i2c, int target,
			    const u8 *data, int length, bool first, bool last)
{
	int i, result;

	result = octeon_i2c_start(i2c, first);
	if (result)
		return result;

	octeon_i2c_write_sw(i2c, SW_TWSI_EOP_TWSI_DATA, target << 1);
	octeon_i2c_write_sw(i2c, SW_TWSI_EOP_TWSI_CTL, TWSI_CTL_ENAB);

	result = octeon_i2c_wait(i2c);
	if (result)
		return result;

	for (i = 0; i < length; i++) {
		result = check_arb(i2c, false);
		if (result)
			return result;

		octeon_i2c_write_sw(i2c, SW_TWSI_EOP_TWSI_DATA, data[i]);
		octeon_i2c_write_sw(i2c, SW_TWSI_EOP_TWSI_CTL, TWSI_CTL_ENAB);

		result = octeon_i2c_wait(i2c);
		if (result)
			return result;
		result = check_arb(i2c, false);
		if (result)
			return result;

	}

	return 0;
}

/**
 * octeon_i2c_read - receive data from the bus via low-level controller.
 * @i2c: The struct octeon_i2c.
 * @target: Target address.
 * @data: Pointer to the location to store the data.
 * @length: Length of the data.
 * @last: is last msg in combined operation?
 *
 * The address is sent over the bus, then the data is read.
 *
 * Returns 0 on success, otherwise a negative errno.
 */
static int octeon_i2c_read(struct octeon_i2c *i2c, int target,
			   u8 *data, int length, bool first, bool last)
{
	int i, result;
	u8 tmp;
	u8 ctl = TWSI_CTL_ENAB | TWSI_CTL_AAK;

	if (length < 1)
		return -EINVAL;

	result = octeon_i2c_start(i2c, first);
	if (result)
		return result;

	octeon_i2c_write_sw(i2c, SW_TWSI_EOP_TWSI_DATA, (target<<1) | 1);

	for (i = 0; i < length; ) {
		tmp = octeon_i2c_read_sw(i2c, SW_TWSI_EOP_TWSI_STAT);
		result = octeon_i2c_lost_arb(tmp, !(ctl & TWSI_CTL_AAK));
		if (result)
			return result;

		switch (tmp) {
		case STAT_RXDATA_ACK_50:
		case STAT_RXDATA_NAK_58:
			data[i++] = octeon_i2c_read_sw(i2c,
					SW_TWSI_EOP_TWSI_DATA);
		}

		/* NAK last recv'd byte, as a no-more-please */
		if (last && i == length - 1)
			ctl &= ~TWSI_CTL_AAK;

		/* clr iflg to allow next event */
		octeon_i2c_write_sw(i2c, SW_TWSI_EOP_TWSI_CTL, ctl);
		result = octeon_i2c_wait(i2c);
		if (result)
			return result;

	}
	return 0;
}

static inline bool octeon_i2c_hlc_test_ready(struct octeon_i2c *i2c)
{
	u64 v = __raw_readq(i2c->twsi_base + SW_TWSI);
	return (v & SW_TWSI_V) == 0;
}

static void octeon_i2c_hlc_int_enable(struct octeon_i2c *i2c)
{
	octeon_i2c_write_int(i2c, ST_EN);
}

static void octeon_i2c_hlc_int_clear(struct octeon_i2c *i2c)
{
	/* clear ST/TS events, listen for neither */
	octeon_i2c_write_int(i2c, ST_INT | TS_INT);
}

/**
 * octeon_i2c_hlc_wait - wait for an HLC operation to complete.
 * @i2c: The struct octeon_i2c.
 *
 * Returns 0 on success, otherwise a negative errno.
 */
static int octeon_i2c_hlc_wait(struct octeon_i2c *i2c)
{
	int result;

	if (i2c->broken_irq_mode) {
		/*
		 * Some cn38xx boards did not assert the irq in
		 * the interrupt controller.  So we must poll for the
		 * IFLG change.
		 */
		u64 end = get_jiffies_64() + i2c->adap.timeout;

		while (!octeon_i2c_hlc_test_ready(i2c) && get_jiffies_64() <= end)
			udelay(50);

		return octeon_i2c_hlc_test_ready(i2c) ? 0 : -ETIMEDOUT;
	}

	i2c->hlc_int_en(i2c);

	result = wait_event_interruptible_timeout(i2c->queue,
						  octeon_i2c_hlc_test_ready(i2c),
						  i2c->adap.timeout);
	i2c->hlc_int_dis(i2c);
	if (!result)
		octeon_i2c_hlc_int_clear(i2c);


	if (result <= 0 && OCTEON_IS_MODEL(OCTEON_CN38XX) &&
			octeon_i2c_hlc_test_ready(i2c)) {
		dev_err(i2c->dev, "broken irq connection detected, switching to polling mode.\n");
		i2c->broken_irq_mode = 1;
		return 0;
	}

	if (result < 0) {
		dev_dbg(i2c->dev, "%s: wait interrupted\n", __func__);
		return result;
	} else if (result == 0) {
		dev_dbg(i2c->dev, "%s: timeout\n", __func__);
		return -ETIMEDOUT;
	}

	return 0;
}

/* high-level-controller pure read of up to 8 bytes */
static int octeon_i2c_simple_read(struct octeon_i2c *i2c, struct i2c_msg *msgs)
{
	u64 cmd;
	int i, j;
	int ret = 0;

	octeon_i2c_enable_hlc(i2c);
	cmd = SW_TWSI_V | SW_TWSI_R | SW_TWSI_SOVR;
	/* SIZE */
	cmd |= (u64)(msgs[0].len - 1) << SW_TWSI_SIZE_SHIFT;
	/* A */
	cmd |= (u64)(msgs[0].addr & 0x7full) << SW_TWSI_A_SHIFT;

	if (msgs[0].flags & I2C_M_TEN)
		cmd |= SW_TWSI_OP_10;
	else
		cmd |= SW_TWSI_OP_7;

	octeon_i2c_hlc_int_clear(i2c);
	writeqflush(cmd, i2c->twsi_base + SW_TWSI);

	ret = octeon_i2c_hlc_wait(i2c);

	if (ret)
		goto err;

	cmd = __raw_readq(i2c->twsi_base + SW_TWSI);

	if ((cmd & SW_TWSI_R) == 0)
		return -EAGAIN;

	for (i = 0, j = msgs[0].len - 1; i  < msgs[0].len && i < 4; i++, j--)
		msgs[0].buf[j] = (cmd >> (8 * i)) & 0xff;

	if (msgs[0].len > 4) {
		cmd = __raw_readq(i2c->twsi_base + SW_TWSI_EXT);
		for (i = 0; i  < msgs[0].len - 4 && i < 4; i++, j--)
			msgs[0].buf[j] = (cmd >> (8 * i)) & 0xff;
	}

err:
	return ret;
}

/* high-level-controller pure write of up to 8 bytes */
static int octeon_i2c_simple_write(struct octeon_i2c *i2c, struct i2c_msg *msgs)
{
	u64 cmd;
	int i, j;
	int ret = 0;

	octeon_i2c_enable_hlc(i2c);
	octeon_i2c_hlc_int_clear(i2c);

	ret = check_arb(i2c, false);
	if (ret)
		goto err;

	cmd = SW_TWSI_V | SW_TWSI_SOVR;
	/* SIZE */
	cmd |= (u64)(msgs[0].len - 1) << SW_TWSI_SIZE_SHIFT;
	/* A */
	cmd |= (u64)(msgs[0].addr & 0x7full) << SW_TWSI_A_SHIFT;

	if (msgs[0].flags & I2C_M_TEN)
		cmd |= SW_TWSI_OP_10;
	else
		cmd |= SW_TWSI_OP_7;

	for (i = 0, j = msgs[0].len - 1; i  < msgs[0].len && i < 4; i++, j--)
		cmd |= (u64)msgs[0].buf[j] << (8 * i);

	if (msgs[0].len > 4) {
		u64 ext = 0;
		for (i = 0; i < msgs[0].len - 4 && i < 4; i++, j--)
			ext |= (u64)msgs[0].buf[j] << (8 * i);
		writeqflush(ext, i2c->twsi_base + SW_TWSI_EXT);
	}

	writeqflush(cmd, i2c->twsi_base + SW_TWSI);

	ret = octeon_i2c_hlc_wait(i2c);
	if (ret)
		goto err;

	cmd = __raw_readq(i2c->twsi_base + SW_TWSI);
	if ((cmd & SW_TWSI_R) == 0)
		return -EAGAIN;

	ret = check_arb(i2c, false);

err:
	return ret;
}

/* high-level-controller composite write+read, msg0=addr, msg1=data */
static int octeon_i2c_ia_read(struct octeon_i2c *i2c, struct i2c_msg *msgs)
{
	u64 cmd;
	int i, j;
	int ret = 0;

	octeon_i2c_enable_hlc(i2c);

	cmd = SW_TWSI_V | SW_TWSI_R | SW_TWSI_SOVR;
	/* SIZE */
	cmd |= (u64)(msgs[1].len - 1) << SW_TWSI_SIZE_SHIFT;
	/* A */
	cmd |= (u64)(msgs[0].addr & 0x7full) << SW_TWSI_A_SHIFT;

	if (msgs[0].flags & I2C_M_TEN)
		cmd |= SW_TWSI_OP_10_IA;
	else
		cmd |= SW_TWSI_OP_7_IA;

	if (msgs[0].len == 2) {
		u64 ext = 0;
		cmd |= SW_TWSI_EIA;
		ext = (u64)msgs[0].buf[0] << SW_TWSI_IA_SHIFT;
		cmd |= (u64)msgs[0].buf[1] << SW_TWSI_IA_SHIFT;
		writeqflush(ext, i2c->twsi_base + SW_TWSI_EXT);
	} else {
		cmd |= (u64)msgs[0].buf[0] << SW_TWSI_IA_SHIFT;
	}

	octeon_i2c_hlc_int_clear(i2c);
	writeqflush(cmd, i2c->twsi_base + SW_TWSI);
	ret = octeon_i2c_hlc_wait(i2c);

	if (ret)
		goto err;

	cmd = __raw_readq(i2c->twsi_base + SW_TWSI);

	if ((cmd & SW_TWSI_R) == 0)
		return -EAGAIN;

	for (i = 0, j = msgs[1].len - 1; i  < msgs[1].len && i < 4; i++, j--)
		msgs[1].buf[j] = (cmd >> (8 * i)) & 0xff;

	if (msgs[1].len > 4) {
		cmd = __raw_readq(i2c->twsi_base + SW_TWSI_EXT);
		for (i = 0; i  < msgs[1].len - 4 && i < 4; i++, j--)
			msgs[1].buf[j] = (cmd >> (8 * i)) & 0xff;
	}

err:
	return ret;
}

/* high-level-controller composite write+write, m[0]len<=2, m[1]len<=8 */
static int octeon_i2c_ia_write(struct octeon_i2c *i2c, struct i2c_msg *msgs)
{
	u64 cmd;
	int i, j;
	int ret = 0;
	u64 ext = 0;
	bool set_ext = false;

	octeon_i2c_enable_hlc(i2c);

	cmd = SW_TWSI_V | SW_TWSI_SOVR;
	/* SIZE */
	cmd |= (u64)(msgs[1].len - 1) << SW_TWSI_SIZE_SHIFT;
	/* A */
	cmd |= (u64)(msgs[0].addr & 0x7full) << SW_TWSI_A_SHIFT;

	if (msgs[0].flags & I2C_M_TEN)
		cmd |= SW_TWSI_OP_10_IA;
	else
		cmd |= SW_TWSI_OP_7_IA;

	if (msgs[0].len == 2) {
		cmd |= SW_TWSI_EIA;
		ext |= (u64)msgs[0].buf[0] << SW_TWSI_IA_SHIFT;
		set_ext = true;
		cmd |= (u64)msgs[0].buf[1] << SW_TWSI_IA_SHIFT;
	} else {
		cmd |= (u64)msgs[0].buf[0] << SW_TWSI_IA_SHIFT;
	}
	for (i = 0, j = msgs[1].len - 1; i  < msgs[1].len && i < 4; i++, j--)
		cmd |= (u64)msgs[1].buf[j] << (8 * i);

	if (msgs[1].len > 4) {
		for (i = 0; i < msgs[1].len - 4 && i < 4; i++, j--)
			ext |= (u64)msgs[1].buf[j] << (8 * i);
		set_ext = true;
	}
	if (set_ext)
		writeqflush(ext, i2c->twsi_base + SW_TWSI_EXT);

	octeon_i2c_hlc_int_clear(i2c);
	writeqflush(cmd, i2c->twsi_base + SW_TWSI);

	ret = octeon_i2c_hlc_wait(i2c);

	if (ret)
		goto err;

	cmd = octeon_i2c_read_sw64(i2c, SW_TWSI_EOP_TWSI_STAT);
	if ((cmd & SW_TWSI_R) == 0)
		return -EAGAIN;
	ret = octeon_i2c_lost_arb(cmd, false);

err:
	return ret;
}

/**
 * octeon_i2c_xfer - The driver's master_xfer function.
 * @adap: Pointer to the i2c_adapter structure.
 * @msgs: Pointer to the messages to be processed.
 * @num: Length of the MSGS array.
 *
 * Returns the number of messages processed, or a negative errno on
 * failure.
 */
static int octeon_i2c_xfer(struct i2c_adapter *adap,
			   struct i2c_msg *msgs,
			   int num)
{
	struct i2c_msg *pmsg;
	int i;
	int ret = 0;
	struct octeon_i2c *i2c = i2c_get_adapdata(adap);

	if (num == 1) {
		if (msgs[0].len > 0 && msgs[0].len <= 8) {
			if (msgs[0].flags & I2C_M_RD)
				ret = octeon_i2c_simple_read(i2c, msgs);
			else
				ret = octeon_i2c_simple_write(i2c, msgs);
			goto out;
		}
	} else if (num == 2) {
		if ((msgs[0].flags & I2C_M_RD) == 0 &&
		    msgs[0].len > 0 && msgs[0].len <= 2 &&
		    msgs[1].len > 0 && msgs[1].len <= 8 &&
		    msgs[0].addr == msgs[1].addr) {
			if (msgs[1].flags & I2C_M_RD)
				ret = octeon_i2c_ia_read(i2c, msgs);
			else
				ret = octeon_i2c_ia_write(i2c, msgs);
			goto out;
		}
	}

	for (i = 0; ret == 0 && i < num; i++) {
		bool last = (i == (num - 1));
		pmsg = &msgs[i];
		dev_dbg(i2c->dev,
			"Doing %s %d byte(s) to/from 0x%02x - %d of %d messages\n",
			 pmsg->flags & I2C_M_RD ? "read" : "write",
			 pmsg->len, pmsg->addr, i + 1, num);
		if (pmsg->flags & I2C_M_RD)
			ret = octeon_i2c_read(i2c, pmsg->addr, pmsg->buf,
						      pmsg->len, !i, last);
		else
			ret = octeon_i2c_write(i2c, pmsg->addr, pmsg->buf,
						       pmsg->len, !i, last);
	}
	octeon_i2c_stop(i2c);
out:
	return ret ? -EAGAIN : num;
}

static u32 octeon_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static const struct i2c_algorithm octeon_i2c_algo = {
	.master_xfer = octeon_i2c_xfer,
	.functionality = octeon_i2c_functionality,
};

static struct i2c_adapter octeon_i2c_ops = {
	.owner = THIS_MODULE,
	.name = "OCTEON adapter",
	.algo = &octeon_i2c_algo,
};

/**
 * octeon_i2c_setclock - Calculate and set clock divisors.
 */
static int octeon_i2c_setclock(struct octeon_i2c *i2c)
{
	int tclk, thp_base, inc, thp_idx, mdiv_idx, ndiv_idx, foscl, diff;
	int thp = 0x18, mdiv = 2, ndiv = 0, delta_hz = 1000000;

	for (ndiv_idx = 0; ndiv_idx < 8 && delta_hz != 0; ndiv_idx++) {
		/*
		 * An mdiv value of less than 2 seems to not work well
		 * with ds1337 RTCs, so we constrain it to larger
		 * values.
		 */
		for (mdiv_idx = 15; mdiv_idx >= 2 && delta_hz != 0; mdiv_idx--) {
			/*
			 * For given ndiv and mdiv values check the
			 * two closest thp values.
			 */
			tclk = i2c->twsi_freq * (mdiv_idx + 1) * 10;
			tclk *= (1 << ndiv_idx);
			thp_base = (i2c->sys_freq / (tclk * 2)) - 1;
			for (inc = 0; inc <= 1; inc++) {
				thp_idx = thp_base + inc;
				if (thp_idx < 5 || thp_idx > 0xff)
					continue;

				foscl = i2c->sys_freq / (2 * (thp_idx + 1));
				foscl = foscl / (1 << ndiv_idx);
				foscl = foscl / (mdiv_idx + 1) / 10;
				diff = abs(foscl - i2c->twsi_freq);
				if (diff < delta_hz) {
					delta_hz = diff;
					thp = thp_idx;
					mdiv = mdiv_idx;
					ndiv = ndiv_idx;
				}
			}
		}
	}
	octeon_i2c_write_sw(i2c, SW_TWSI_OP_TWSI_CLK, thp);
	octeon_i2c_write_sw(i2c, SW_TWSI_EOP_TWSI_CLKCTL, (mdiv << 3) | ndiv);

	return 0;
}

static int octeon_i2c_initlowlevel(struct octeon_i2c *i2c)
{
	u8 status;
	int tries;

	/* reset controller */
	octeon_i2c_write_sw(i2c, SW_TWSI_EOP_TWSI_RST, 0);

	status = 0;
	for (tries = 10; tries && status != STAT_IDLE_F8; tries--) {
		udelay(1);
		status = octeon_i2c_read_sw(i2c, SW_TWSI_EOP_TWSI_STAT);
	}

	if (status != STAT_IDLE_F8) {
		dev_err(i2c->dev, "%s: TWSI_RST failed! (0x%x)\n",
			__func__, status);
		return -EIO;
	}


	/* toggle twice to force both teardowns */
	octeon_i2c_enable_hlc(i2c);
	octeon_i2c_disable_hlc(i2c);

	return 0;
}

static int octeon_i2c_cvmx_map[3] = {-ENODEV, -ENODEV, -ENODEV};

int octeon_i2c_cvmx2i2c(unsigned int cvmx_twsi_bus_num)
{
	if (cvmx_twsi_bus_num < ARRAY_SIZE(octeon_i2c_cvmx_map))
		return octeon_i2c_cvmx_map[cvmx_twsi_bus_num];
	else
		return -ENODEV;
}
EXPORT_SYMBOL(octeon_i2c_cvmx2i2c);

static int octeon_i2c_probe(struct platform_device *pdev)
{
	int irq, hlc_irq = 0, result = 0;
	struct octeon_i2c *i2c;
	struct resource *res_mem;
	struct device_node *node = pdev->dev.of_node;
	bool cn78xx_style;

	cn78xx_style = of_device_is_compatible(node, "cavium,octeon-7890-twsi");

	if (cn78xx_style) {
		hlc_irq = platform_get_irq(pdev, 0);
		if (hlc_irq < 0)
			return hlc_irq;

		irq = platform_get_irq(pdev, 2);
		if (irq < 0)
			return irq;
	} else {
		/* All adaptors have an irq.  */
		irq = platform_get_irq(pdev, 0);
		if (irq < 0)
			return irq;
	}

	i2c = devm_kzalloc(&pdev->dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		result = -ENOMEM;
		goto out;
	}
	i2c->cvmx_channel = -1;
	i2c->dev = &pdev->dev;

	res_mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (res_mem == NULL) {
		dev_err(i2c->dev, "found no memory resource\n");
		result = -ENXIO;
		goto out;
	}

	/*
	 * "clock-rate" is a legacy binding, the official binding is
	 * "clock-frequency".  Try the official one first and then
	 * fall back if it doesn't exist.
	 */
	if (of_property_read_u32(node,
				 "clock-frequency", &i2c->twsi_freq) &&
	    of_property_read_u32(node,
				 "clock-rate", &i2c->twsi_freq)) {
		dev_err(i2c->dev,
			"no I2C 'clock-rate' or 'clock-frequency' property\n");
		result = -ENXIO;
		goto out;
	}

	i2c->sys_freq = octeon_get_io_clock_rate();

	switch (res_mem->start) {
	case 0x1180000001000:
		i2c->cvmx_channel = 0;
		break;
	case 0x1180000001100:
		if (OCTEON_IS_MODEL(OCTEON_CNF75XX))
			i2c->cvmx_channel = 1;
		break;
	case 0x1180000001200:
		if (OCTEON_IS_MODEL(OCTEON_CNF75XX))
			i2c->cvmx_channel = 2;
		else
		i2c->cvmx_channel = 1;
		break;
	default:
		break;
	}

	if (!devm_request_mem_region(&pdev->dev, res_mem->start, resource_size(res_mem),
				     res_mem->name)) {
		dev_err(i2c->dev, "request_mem_region failed\n");
		goto out;
	}
	i2c->twsi_base = devm_ioremap(&pdev->dev, res_mem->start, resource_size(res_mem));

	init_waitqueue_head(&i2c->queue);

	i2c->irq = irq;

	if (cn78xx_style) {
		i2c->hlc_irq = hlc_irq;

		i2c->int_en = octeon_i2c_int_enable78;
		i2c->int_dis = octeon_i2c_int_disable78;
		i2c->hlc_int_en = octeon_i2c_hlc_int_enable78;
		i2c->hlc_int_dis = octeon_i2c_hlc_int_disable78;

		irq_set_status_flags(i2c->irq, IRQ_NOAUTOEN);
		irq_set_status_flags(i2c->hlc_irq, IRQ_NOAUTOEN);

		result = devm_request_irq(&pdev->dev, i2c->irq,
					  octeon_i2c_isr, 0, DRV_NAME, i2c);

		if (result < 0) {
			dev_err(i2c->dev, "failed to attach interrupt\n");
			goto out;
		}
		result = devm_request_irq(&pdev->dev, i2c->hlc_irq,
					  octeon_i2c_hlc_isr78, 0, DRV_NAME, i2c);

		if (result < 0) {
			dev_err(i2c->dev, "failed to attach interrupt\n");
			goto out;
		}
	} else {
		i2c->int_en = octeon_i2c_int_enable;
		i2c->int_dis = octeon_i2c_int_disable;
		i2c->hlc_int_en = octeon_i2c_hlc_int_enable;
		i2c->hlc_int_dis = octeon_i2c_int_disable;

		result = devm_request_irq(&pdev->dev, i2c->irq,
					  octeon_i2c_isr, 0, DRV_NAME, i2c);
		if (result < 0) {
			dev_err(i2c->dev, "failed to attach interrupt\n");
			goto out;
		}
	}

	result = octeon_i2c_initlowlevel(i2c);
	if (result) {
		dev_err(i2c->dev, "init low level failed\n");
		goto  out;
	}

	result = octeon_i2c_setclock(i2c);
	if (result) {
		dev_err(i2c->dev, "clock init failed\n");
		goto  out;
	}

	i2c->adap = octeon_i2c_ops;
	i2c->adap.timeout = msecs_to_jiffies(timeout);
	i2c->adap.retries = 10;
	i2c->adap.dev.parent = &pdev->dev;
	i2c->adap.dev.of_node = node;
	i2c_set_adapdata(&i2c->adap, i2c);
	platform_set_drvdata(pdev, i2c);

	result = i2c_add_adapter(&i2c->adap);
	if (result < 0) {
		dev_err(i2c->dev, "failed to add adapter\n");
		goto out;
	}
	dev_info(i2c->dev, "version %s\n", DRV_VERSION);

	of_i2c_register_devices(&i2c->adap);
	if (i2c->cvmx_channel >= 0)
		octeon_i2c_cvmx_map[i2c->cvmx_channel] = i2c->adap.nr;

	return 0;

out:
	return result;
};

static int octeon_i2c_remove(struct platform_device *pdev)
{
	struct octeon_i2c *i2c = platform_get_drvdata(pdev);

	if (i2c->cvmx_channel >= 0)
		octeon_i2c_cvmx_map[i2c->cvmx_channel] = -ENODEV;
	i2c_del_adapter(&i2c->adap);
	return 0;
};

static struct of_device_id octeon_i2c_match[] = {
	{
		.compatible = "cavium,octeon-3860-twsi",
	},
	{
		.compatible = "cavium,octeon-7890-twsi",
	},
	{},
};
MODULE_DEVICE_TABLE(of, octeon_i2c_match);

static struct platform_driver octeon_i2c_driver = {
	.probe		= octeon_i2c_probe,
	.remove		= octeon_i2c_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DRV_NAME,
		.of_match_table = octeon_i2c_match,
	},
};

module_platform_driver(octeon_i2c_driver);

MODULE_AUTHOR("Michael Lawnick <michael.lawnick.ext@nsn.com>");
MODULE_DESCRIPTION("I2C-Bus adapter for Cavium OCTEON processors");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRV_VERSION);

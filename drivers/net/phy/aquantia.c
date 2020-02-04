/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2015 Cavium, Inc.
 */

#include <linux/module.h>
#include <linux/phy.h>

#define PHY_ID_AQR105			0x03a1b4a0

#define PMA_RECEIVE_VENDOR_STATE_1	(MII_ADDR_C45 | 0x01 << 16 | 0xe800)

#define AN_TX_VENDOR_ALARMS_2		(MII_ADDR_C45 | 0x07 << 16 | 0xcc01)
#define AN_VENDOR_STATUS_1		(MII_ADDR_C45 | 0x07 << 16 | 0xc800)
#define AN_TX_VENDOR_INT_MASK_2		(MII_ADDR_C45 | 0x07 << 16 | 0xd401)

#define GLOBAL_CW_VENDOR_INT_FLAGS	(MII_ADDR_C45 | 0x1e << 16 | 0xfc01)
#define GLOBAL_INT_CW_VENDOR_MASK	(MII_ADDR_C45 | 0x1e << 16 | 0xff01)


static int aqr105_config_aneg(struct phy_device *phydev)
{
	return 0;
}

static int aqr105_read_status(struct phy_device *phydev)
{
	int	reg;

	reg = phy_read(phydev, PMA_RECEIVE_VENDOR_STATE_1);
	/* Set the link state */
	if ((reg & 1) == 0)
		phydev->link = 0;
	else {
		phydev->link = 1;

		reg = phy_read(phydev, AN_VENDOR_STATUS_1);
		/* Set the duplex mode */
		if ((reg & 1) == 0)
			phydev->duplex = 0;
		else
			phydev->duplex = 1;

		/* Set the speed */
		reg = (reg >> 1) & 7;
		switch (reg) {
		case 0:
			phydev->speed = 10;
			break;
		case 1:
			phydev->speed = 100;
			break;
		case 2:
			phydev->speed = 1000;
			break;
		case 3:
			phydev->speed = 10000;
			break;
		case 4:
			phydev->speed = 2500;
			break;
		case 5:
			phydev->speed = 5000;
			break;
		default:
			phydev->speed = -1;
			break;
		}
	}

	return 0;
}

static int  aqr105_ack_interrupt(struct phy_device *phydev)
{
	int	reg;

	reg = phy_read(phydev, AN_TX_VENDOR_ALARMS_2);

	return 0;
}

static int aqr105_config_intr(struct phy_device *phydev)
{
	int	reg;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED) {
		reg = phy_read(phydev, AN_TX_VENDOR_INT_MASK_2);
		reg |= 0x1;
		phy_write(phydev, AN_TX_VENDOR_INT_MASK_2, reg);

		reg = phy_read(phydev, GLOBAL_INT_CW_VENDOR_MASK);
		reg |= 0x1000;
		phy_write(phydev, GLOBAL_INT_CW_VENDOR_MASK, reg);
	} else {
		reg = phy_read(phydev, GLOBAL_INT_CW_VENDOR_MASK);
		reg &= ~0x1000;
		phy_write(phydev, GLOBAL_INT_CW_VENDOR_MASK, reg);

		reg = phy_read(phydev, AN_TX_VENDOR_INT_MASK_2);
		reg &= ~0x1;
		phy_write(phydev, AN_TX_VENDOR_INT_MASK_2, reg);
	}

	return 0;
}

static int aqr105_did_interrupt(struct phy_device *phydev)
{
	int	reg;

	reg = phy_read(phydev, GLOBAL_CW_VENDOR_INT_FLAGS);
	if (reg & 0x1000)
		return 1;

	return 0;
}

static int aqr105_match_phy_device(struct phy_device *phydev)
{
	return (phydev->c45_ids.device_ids[1] & 0xfffffff0) == PHY_ID_AQR105;
}

static struct phy_driver aqr105_driver[] = {
{
	.phy_id			= 0,
	.phy_id_mask		= 0,
	.name			= "Aquantia aqr105",
	.flags			= PHY_HAS_INTERRUPT,
	.config_aneg		= aqr105_config_aneg,
	.read_status		= aqr105_read_status,
	.ack_interrupt		= aqr105_ack_interrupt,
	.config_intr		= aqr105_config_intr,
	.did_interrupt		= aqr105_did_interrupt,
	.match_phy_device 	= aqr105_match_phy_device,
	.driver			= {
		.owner = THIS_MODULE,
	},
} };

static int __init aquantia_init(void)
{
	return phy_drivers_register(aqr105_driver, ARRAY_SIZE(aqr105_driver));
}
module_init(aquantia_init);

static void __exit aquantia_exit(void)
{
	phy_drivers_unregister(aqr105_driver, ARRAY_SIZE(aqr105_driver));
}
module_exit(aquantia_exit);

MODULE_AUTHOR("Carlos Munoz <cmunoz@caviumnetworks.com>");
MODULE_LICENSE("GPL");

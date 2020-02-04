#include <linux/module.h>
#include <linux/delay.h>
#include <linux/phy.h>
#include <linux/of.h>
#include <asm/octeon/cvmx.h>
#include <asm/octeon/cvmx-helper.h>
#include <asm/octeon/cvmx-clock.h>
#include <asm/octeon/cvmx-qlm.h>
#include <asm/octeon/cvmx-helper-bgx.h>
#include <asm/octeon/cvmx-helper-board.h>
#include <asm/octeon/cvmx-helper-cfg.h>
#include <asm/octeon/cvmx-bgxx-defs.h>
#include <asm/octeon/cvmx-gserx-defs.h>
#include <asm/octeon/cvmx-xcv-defs.h>


MODULE_LICENSE("GPL");

#define CS4223_GLOBAL_CHIP_ID_LSB			0x0
#define CS4223_GLOBAL_CHIP_ID_MSB			0x1
#define CS4223_DEV1_ID 28
#define CS4223_DEV2_ID 29

#define CS4223_MII_BUS_NAME "8001180000003880"

struct mii_bus *cs4223_mii_bus = NULL;

#define CS4224_MONITOR_CONTROL0                   0x200
#define CS4224_MONITOR_CONFIG_MASK                0x204
#define CS4224_MONITOR_STATUS_FINAL6              0x260
#define CS4224_SENSE_POINT_LOCAL_TEMPERATURE  1 << 0
#define CS4224_SENSE_POINT_REMOTE_TEMPERATURE 1 << 1
#define CS4224_SENSE_POINT_0p9_RX_VOLTAGE     1 << 2
#define CS4224_SENSE_POINT_0p9_TX_VOLTAGE     1 << 3
#define CS4224_SENSE_POINT_0p9_DIG_RX_VOLTAGE 1 << 4
#define CS4224_SENSE_POINT_0p9_DIG_TX_VOLTAGE 1 << 5
#define CS4224_SENSE_POINT_1p8_RX_VOLTAGE     1 << 6
#define CS4224_SENSE_POINT_1p8_TX_VOLTAGE     1 << 7
#define CS4224_SENSE_POINT_1p5_RX_VOLTAGE     1 << 8
#define CS4224_SENSE_POINT_1p5_TX_VOLTAGE     1 << 9

#if 0
struct phy_device *cs4223_phydev = NULL;
static struct phy_device *get_cs4223_phy_device()
{
	return cs4223_phydev;
}
#endif

extern struct mii_bus* mdiobus_find(char *name);
struct mii_bus *get_cs4223_mii_bus(void)
{
	cs4223_mii_bus = mdiobus_find(CS4223_MII_BUS_NAME);
	
	return cs4223_mii_bus;
}

int cs4223_phy_read(int set_num, int regnum)
{
	int phy_addr;
	struct mii_bus *miibus = get_cs4223_mii_bus();

	if(!miibus)
		return -1;
	
	if(set_num)
		phy_addr = CS4223_DEV1_ID;
	else
		phy_addr = CS4223_DEV2_ID;	
	
	return mdiobus_read(miibus, phy_addr, MII_ADDR_C45 | regnum);
	
}
EXPORT_SYMBOL(cs4223_phy_read);
int cs4223_phy_write(int set_num, int regnum, int val)
{
	int phy_addr;
	struct mii_bus *miibus = get_cs4223_mii_bus();

	if(!miibus)
		return -1;
	
	if(!set_num)
		phy_addr = CS4223_DEV1_ID;
	else
		phy_addr = CS4223_DEV2_ID;	
	
	return mdiobus_write(miibus, phy_addr, MII_ADDR_C45 | regnum, val);
}

EXPORT_SYMBOL(cs4223_phy_write);

int cs4224_enable_monitor_sense_points(int set_num)
{
   	int phy_addr;
	int regnum ,reg_data;
	
	regnum = CS4224_MONITOR_CONTROL0;
	reg_data = cs4223_phy_read(set_num, regnum);

	if(reg_data < 0 )
		return reg_data;
	
    if(reg_data != 0x630F)
    {
		regnum = CS4224_MONITOR_CONFIG_MASK;
		reg_data = cs4223_phy_write(set_num, regnum, CS4224_SENSE_POINT_LOCAL_TEMPERATURE |
            										CS4224_SENSE_POINT_0p9_RX_VOLTAGE |
            										CS4224_SENSE_POINT_1p8_RX_VOLTAGE);
		if(reg_data < 0)
			return reg_data;	
	
		regnum = CS4224_MONITOR_CONTROL0;
		reg_data = cs4223_phy_write(set_num, regnum, 0x630F);
		
		if(reg_data < 0)
			return reg_data;
    
        {
            int last = 0;
            int i;

            regnum = CS4224_MONITOR_STATUS_FINAL6;
			reg_data = cs4223_phy_read(set_num, regnum);

			if(reg_data < 0)
				return reg_data;	

            for(i = 0; i < 100; i++)
            {
                cvmx_wait_usec(1000);
	            if(last > (int)reg_data)
                {
                    break;
                }
                
                last = (int)reg_data;
                reg_data = cs4223_phy_read(set_num, regnum);

				if(reg_data < 0)
					return reg_data;	
            }
        }
    }

    return 0;
}

#define CS4224_MONITOR_STATUS_FINAL0              0x25A

int cs4223_read_temp(int set_num)
{
	int regnum, reg_data ,temp, dac;
	  
	if(cs4224_enable_monitor_sense_points(set_num) < 0)
		return -1;

	regnum = CS4224_MONITOR_STATUS_FINAL0;
	reg_data = cs4223_phy_read(set_num, regnum);
	if(reg_data < 0)
		return -1;	
	
	dac = reg_data*1000;
	temp = ((2563 * ((dac / 256) - 78170))/1000) + 85000; /* fixed-point math    */

	return temp;
}

EXPORT_SYMBOL(cs4223_read_temp);

static int cortina_phy_read_x(struct phy_device *phydev, int off, u16 regnum)
{
	return mdiobus_read(phydev->bus, phydev->addr + off,
			    MII_ADDR_C45 | regnum);
}

static int cortina_phy_write_x(struct phy_device *phydev, int off,
			      u16 regnum, u16 val)
{
	return mdiobus_write(phydev->bus, phydev->addr + off,
			     MII_ADDR_C45 | regnum, val);
}
static int cortina_phy_read(struct phy_device *phydev, u16 regnum)
{
	return cortina_phy_read_x(phydev, 0, regnum);
}

static int cortina_phy_write(struct phy_device *phydev, u16 regnum, u16 val)
{
	return cortina_phy_write_x(phydev, 0, regnum, val);
}

int cs4223_read_status(struct phy_device *phydev)
{
// TBD : To check CS4223 register spec
	return 0;
}

int cs4223_config_init(struct phy_device *phydev)
{
	int ret;
	
	phydev->supported = phydev->advertising =
		SUPPORTED_40000baseKR4_Full |
		ADVERTISED_40000baseKR4_Full;
	phydev->duplex = DUPLEX_FULL;
	phydev->speed = 10000;
	
	return ret = 0;
}

int cs4223_probe(struct phy_device *phydev)
{
	int ret = 0;
	int id_lsb, id_msb;

	id_lsb = cortina_phy_read(phydev, CS4223_GLOBAL_CHIP_ID_LSB);
	if (id_lsb < 0) {
		ret = id_lsb;
		goto err;
	}
	id_msb = cortina_phy_read(phydev, CS4223_GLOBAL_CHIP_ID_MSB);
	if (id_msb < 0) {
		ret = id_msb;
		goto err;
	}

	if (id_lsb != 0x03E5 || id_msb != 0x7003) {
		ret = -ENODEV;
		goto err;
	}
#if 0	
	cs4223_phydev = phydev;
#endif

err:
	
	return ret;
}

int cs4223_config_aneg(struct phy_device *phydev)
{
	int err;

	err = genphy_config_aneg(phydev);
	if (err < 0)
		return err;

	return 0;
}

static struct of_device_id cs4223_match[] = {
	{
		.compatible = "cortina,cs4223",
	},
	{},
};
MODULE_DEVICE_TABLE(of, cs4223_match);

static struct phy_driver cs4223_phy_driver = {
	.phy_id		= 0xffffffff,
	.phy_id_mask	= 0xffffffff,
	.name		= "Cortina CS4223",
	.config_init	= cs4223_config_init,
	.probe		= cs4223_probe,
	.config_aneg	= cs4223_config_aneg,
	.read_status	= cs4223_read_status,
	.driver		= {
		.owner = THIS_MODULE,
		.of_match_table = cs4223_match,
	},
};

int __init cs4223_drv_init(void)
{
	int ret = 0;

//	ret = phy_driver_register(&cs4223_phy_driver);

	return ret;
}
module_init(cs4223_drv_init);

static void __exit cs4223_drv_exit(void)
{
//	phy_driver_unregister(&cs4223_phy_driver);
}
module_exit(cs4223_drv_exit);


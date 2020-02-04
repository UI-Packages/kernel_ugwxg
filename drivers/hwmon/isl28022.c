/*
 * Driver for ISL28022 power monitor chips
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/jiffies.h>

//#include "isl28022.h"

#define ISL28022_CONFIG_REG				0x00
#define ISL28022_SHUNT_VOL_REG			0x01
#define ISL28022_BUS_VOL_REG			0x02
#define ISL28022_POWER_REG				0x03
#define ISL28022_CURRENT_REG			0x04
#define ISL28022_CALIB_REG				0x05
#define ISL28022_SHUNT_THR_REG			0x06
#define ISL28022_BUS_THR_REG			0x07
#define ISL28022_THR_INTR_REG			0x08
#define ISL28022_CONTROL_REG			0x09

#define ISL28022_MAX_REGISTERS		10
#define ISL28022_REGISTERS		10

/* worst case is 64.01 ms (~15.6Hz) */
#define ISL28022_CONVERSION_RATE		16
enum isl28022_ids { isl28022 };

struct isl28022_config {

	int registers;
	int shunt_resistor;
	int bus_voltage_shift;
	int bus_voltage_lsb;	//mv
	u16 shunt_voltage_sign_mask;
	int shunt_voltage_lsb;	//uv
	int shunt_voltage_neg_value;	
		
};

struct isl28022_data {
	struct device *hwmon_dev;
	const struct isl28022_config *config;

	struct mutex update_lock;
	bool valid;
	unsigned long last_updated;

	int kind;

	u16 regs[ISL28022_MAX_REGISTERS];
};

static const struct isl28022_config isl28022_config[] = {
	[isl28022] = {
		.registers = ISL28022_REGISTERS,
		.shunt_resistor = 8, //mini-ohm
		.bus_voltage_shift = 2,
		.bus_voltage_lsb = 4, //mv	
		.shunt_voltage_sign_mask = 0x8000,
		.shunt_voltage_lsb= 10, //uv	
		.shunt_voltage_neg_value = -32768,
	},
};

#define SUPPORT_UBNT_SYSFS
#ifdef SUPPORT_UBNT_SYSFS
static struct i2c_client *isl28022_i2c_client = NULL;
#endif
static struct isl28022_data *isl28022_update_device(struct i2c_client *client)
{
	struct isl28022_data *data = i2c_get_clientdata(client);
	struct isl28022_data *ret = data;

	mutex_lock(&data->update_lock);

	if (time_after(jiffies, data->last_updated +
		       HZ / ISL28022_CONVERSION_RATE) || !data->valid) {

		int i;

		dev_dbg(&client->dev, "Starting isl2802 update\n");

		/* Read all registers */
		for (i = 0; i < data->config->registers; i++) {
			int rv = i2c_smbus_read_word_swapped(client, i);
			if (rv < 0) {
				ret = ERR_PTR(rv);
				goto abort;
			}
			data->regs[i] = rv;
		}
		data->last_updated = jiffies;
		data->valid = 1;
	}
abort:
	mutex_unlock(&data->update_lock);
	return ret;
}

static int isl28022_CompShuntVol(struct isl28022_data *data)
{
	u8 reg = ISL28022_SHUNT_VOL_REG;
	int val = 0;
	
	if(data->regs[reg] & data->config->shunt_voltage_sign_mask)
		val += data->config->shunt_voltage_neg_value;
	val += data->regs[reg] & ~(data->config->shunt_voltage_sign_mask);
	val *= data->config->shunt_voltage_lsb;

	return val; //uv

}
static int isl28022_CompBusVol(struct isl28022_data *data)
{
	u8 reg  = ISL28022_BUS_VOL_REG;
	int val = data->regs[reg] >> data->config->bus_voltage_shift;

	return val*= data->config->bus_voltage_lsb; //mv
}

static int isl28022_CompCurrent(struct isl28022_data *data)
{
	int shunt_vol, val;

	mutex_lock(&data->update_lock);
	shunt_vol = isl28022_CompShuntVol(data);
	val = DIV_ROUND_CLOSEST(shunt_vol, data->config->shunt_resistor); //mA
	mutex_unlock(&data->update_lock);

	return val;

}

static int isl28022_CompPower(struct isl28022_data *data)
{
	return isl28022_CompCurrent(data) *isl28022_CompBusVol(data); //W
}
static int isl28022_get_value(struct isl28022_data *data, u8 reg)
{
	int val;

	switch (reg) {
	case ISL28022_SHUNT_VOL_REG:
		val = DIV_ROUND_CLOSEST(isl28022_CompShuntVol(data),1000); //mV
		break;
	case ISL28022_BUS_VOL_REG:
		val = isl28022_CompBusVol(data); //V
		break;
	case ISL28022_POWER_REG:
		val = isl28022_CompPower(data);
		break;
	case ISL28022_CURRENT_REG:
		val = isl28022_CompCurrent(data);
		break;
	default:
		/* programmer goofed */
		WARN_ON_ONCE(1);
		val = 0;
		break;
	}

	return val;
}

static ssize_t isl28022_show_value(struct device *dev,
				 struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct i2c_client *client = to_i2c_client(dev);
	struct isl28022_data *data = isl28022_update_device(client);

	if (IS_ERR(data))
		return PTR_ERR(data);

	return snprintf(buf, PAGE_SIZE, "%d\n", isl28022_get_value(data, attr->index));

}

#ifdef SUPPORT_UBNT_SYSFS
int isl28022_get_val(u8 cmd)
{
	struct isl28022_data *data = isl28022_update_device(isl28022_i2c_client);

	if (IS_ERR(data))
		return PTR_ERR(data);
	else
		return isl28022_get_value(data, cmd);
	
}
EXPORT_SYMBOL(isl28022_get_val);

#endif 

/* shunt voltage */
static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, isl28022_show_value, NULL,
			  ISL28022_SHUNT_VOL_REG);

/* bus voltage */
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, isl28022_show_value, NULL,
			  ISL28022_BUS_VOL_REG);

/* calculated current */
static SENSOR_DEVICE_ATTR(curr1_input, S_IRUGO, isl28022_show_value, NULL,
			  ISL28022_CURRENT_REG);

/* calculated power */
static SENSOR_DEVICE_ATTR(power1_input, S_IRUGO, isl28022_show_value, NULL,
			  ISL28022_POWER_REG);

/* pointers to created device attributes */
static struct attribute *isl28022_attributes[] = {
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_curr1_input.dev_attr.attr,
	&sensor_dev_attr_power1_input.dev_attr.attr,
	NULL,
};

static const struct attribute_group isl28022_group = {
	.attrs = isl28022_attributes,
};

static int isl28022_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = client->adapter;
	struct isl28022_data *data;
	int ret;
	int revision = 1;
	static const char * const names[] = {
		[isl28022] = "ISL28022",
	};
	
	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* set the device type */
	data->kind = id->driver_data;
	data->config = &isl28022_config[data->kind];
	
	i2c_set_clientdata(client, data);
	mutex_init(&data->update_lock);
	
	ret = sysfs_create_group(&client->dev.kobj, &isl28022_group);
	
	if (ret)
		return ret;

	data->hwmon_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(data->hwmon_dev)) {
		ret = PTR_ERR(data->hwmon_dev);
		goto out_err_hwmon;
	}

	dev_info(&client->dev, "%s device, revision %d\n",
		 names[0], revision);
	
#ifdef SUPPORT_UBNT_SYSFS
	isl28022_i2c_client = client;
#endif
	
	return 0;

out_err_hwmon:
	sysfs_remove_group(&client->dev.kobj, &isl28022_group);
	return ret;
}

static int isl28022_remove(struct i2c_client *client)
{
	struct isl28022_data *data = i2c_get_clientdata(client);

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_group(&client->dev.kobj, &isl28022_group);

	return 0;
}

static const struct i2c_device_id isl28022_id[] = {
	{ "isl28022", isl28022 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, isl28022_id);

static struct i2c_driver isl28022_driver = {
	.driver = {
		.name	= "isl28022",
	},
	.probe		= isl28022_probe,
	.remove		= isl28022_remove,
	.id_table	= isl28022_id,
};

module_i2c_driver(isl28022_driver);

MODULE_AUTHOR("Eric Chang <eric.chang@ubnt.com>");
MODULE_DESCRIPTION("isl28022 driver");
MODULE_LICENSE("GPL");

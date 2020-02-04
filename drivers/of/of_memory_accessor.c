/*
 * Memory accessor OF helpers
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2012 Cavium Inc.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/mod_devicetable.h>
#include <linux/of_memory_accessor.h>
#include <linux/list.h>
#include <linux/memory.h>

struct of_macc_entry {
	struct list_head list;
	struct device *dev;
	struct memory_accessor *macc;
	int ref;
};

static DEFINE_MUTEX(lock);
static LIST_HEAD(macc_list);

/**
 * Adds a mapping of a device node to a memory accessor
 *
 * @param[in] dev - device
 * @param[in] macc - memory accessor
 *
 * @returns 0 for success or -ENOMEM
 */
int of_memory_accessor_register(struct device *dev,
				struct memory_accessor *macc)
{
	struct of_macc_entry *mentry;

	mentry = kmalloc(sizeof(*mentry), GFP_KERNEL);
	if (mentry == NULL)
		return -ENOMEM;

	mentry->dev = dev;
	mentry->macc = macc;
	mentry->ref = 0;

	mutex_lock(&lock);

	list_add(&(mentry->list), &macc_list);

	mutex_unlock(&lock);

	return 0;
}
EXPORT_SYMBOL(of_memory_accessor_register);

/**
 * removes the mapping of a device node to a memory accessor
 *
 * @param[in] devnode - device node to remove
 *
 * @returns 0 for success or -ENODEV if device node not found, -EBUSY if still
 *	    in use
 */

int of_memory_accessor_remove(struct device *dev)
{
	struct of_macc_entry *mentry;
	struct list_head *pos, *q;
	int ret = -ENODEV;

	mutex_lock(&lock);

	list_for_each_safe(pos, q, &macc_list) {
		mentry = list_entry(pos, struct of_macc_entry, list);
		if (mentry->dev == dev) {
			if (mentry->ref > 0) {
				ret = -EBUSY;
				goto done;
			}
			list_del(pos);
			kfree(mentry);
			ret = 0;
			goto done;
		}
	}

	/* Not found */
done:
	mutex_unlock(&lock);
	return ret;
}
EXPORT_SYMBOL(of_memory_accessor_remove);

/**
 * Returns the memory accessor for a device node and increments a reference
 * count
 *
 * @param[in] devnode - device node to look up
 *
 * @returns memory accessor for device node or NULL if none found.
 */
struct memory_accessor *
of_memory_accessor_get(const struct device_node *devnode)
{
	struct of_macc_entry *mentry;
	struct list_head *pos;
	struct memory_accessor *macc = NULL;

	mutex_lock(&lock);

	list_for_each(pos, &macc_list) {
		mentry = list_entry(pos, struct of_macc_entry, list);
		if (mentry->dev->of_node == devnode) {
			macc = mentry->macc;
			if (!mentry->ref) {
			    if (!try_module_get(mentry->dev->driver->owner)) {
				macc = NULL;
				pr_info("Warning: module for %s not found!",
					mentry->dev->of_node->full_name);
			    }
			}
			mentry->ref++;
			goto done;
		}
	}
done:
	mutex_unlock(&lock);
	return macc;
}
EXPORT_SYMBOL(of_memory_accessor_get);

/**
 * Decrements the reference count for the memory accessor attached to the
 * device node.
 *
 * @param[in] devnode - device node to look up
 *
 * @returns 0 for success or -ENODEV if the device node was not found.
 */
int of_memory_accessor_put(const struct device_node *devnode)
{
	struct of_macc_entry *mentry;
	struct list_head *pos;
	int ret = -ENODEV;

	mutex_lock(&lock);
	list_for_each(pos, &macc_list) {
		mentry = list_entry(pos, struct of_macc_entry, list);
		if (mentry->dev->of_node == devnode) {
			if (mentry->ref > 0)
				mentry->ref--;
			if (!mentry->ref)
				module_put(mentry->dev->driver->owner);

			module_put(THIS_MODULE);
			ret = 0;
			goto done;
		}
	}
done:
	mutex_unlock(&lock);
	return ret;
}
EXPORT_SYMBOL(of_memory_accessor_put);

static void __exit of_memory_accessor_exit(void)
{
	struct of_macc_entry *mentry;
	struct list_head *pos, *q;

	list_for_each_safe(pos, q, &macc_list) {
		mentry = list_entry(pos, struct of_macc_entry, list);
		if (mentry->ref)
			module_put(mentry->dev->driver->owner);
		list_del(pos);
		kfree(mentry);
	}

	/* Not found */
	mutex_destroy(&lock);
	list_del(&macc_list);
}
module_exit(of_memory_accessor_exit);

MODULE_DESCRIPTION("Driver for mapping device nodes to memory accessors");
MODULE_AUTHOR("Aaron Williams");
MODULE_LICENSE("GPL");

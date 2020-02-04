#ifndef _LINUX_OF_MEMORY_ACCESSOR_H
#define _LINUX_OF_MEMORY_ACCESSOR_H
/*
 * Memory accessor OF helpers
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2012 Cavium Inc.
 */

#include <linux/of.h>
#include <linux/memory.h>

/**
 * Adds a mapping of a device node to a memory accessor
 *
 * @param[in] dev - device
 * @param[in] macc - memory accessor
 *
 * @returns 0 for success or -ENOMEM
 */
#ifdef CONFIG_OF_MEMORY_ACCESSOR
int of_memory_accessor_register(struct device *dev,
				struct memory_accessor *macc);
#else
static inline int of_memory_accessor_register(struct device *dev,
					      struct memory_accessor *macc)
{
	return 0;
}
#endif

/**
 * removes the mapping of a device node to a memory accessor
 *
 * @param[in] devnode - device node to remove
 *
 * @returns 0 for success or 1 if device node not found
 */
#ifdef CONFIG_OF_MEMORY_ACCESSOR
int of_memory_accessor_remove(struct device *dev);
#else
static inline int of_memory_accessor_remove(struct device *dev)
{
	return 0;
}
#endif

/**
 * Returns the memory accessor for a device node
 *
 * @param[in] devnode - device node to look up
 *
 * @returns memory accessor for device node or NULL if none found.
 */
struct memory_accessor *
of_memory_accessor_get(const struct device_node *devnode);

/**
 * Decrements the reference count for the memory accessor attached to the
 * device node.
 *
 * @param[in] devnode - device node to look up
 *
 * @returns 0 for success or -1 if the device node was not found.
 */
int of_memory_accessor_put(const struct device_node *devnode);

#endif /* _LINUX_OF_MEMORY_ACCESSOR_H */

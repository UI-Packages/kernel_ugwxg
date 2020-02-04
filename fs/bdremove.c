/*
 * Block Device Removal implementation
 *
 *		Author:	Montavista Software, Inc.
 *			Steve Dake (sdake@mvista.com)
 *			Dave Jiang (djiang@mvista.com)
 *			source@mvista.com
 *
 * 2005-2010 (c) MontaVista Software, LLC. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/list.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fdtable.h>
#include <linux/blkdev.h>
#include <linux/semaphore.h>

#include "internal.h"
#include "mount.h"

static DEFINE_SEMAPHORE(bdremove_fd_sem);
static LIST_HEAD(bdremove_fd_head);

struct bdremove_fd_item {
	struct list_head list;
	struct task_struct *task;
	int fd;
	dev_t dev;
};

int bdremove_insertfd(struct task_struct *task, int fd, dev_t dev)
{
	struct bdremove_fd_item *bdremove_fd_item;

	bdremove_fd_item = kmalloc(sizeof(struct bdremove_fd_item), GFP_KERNEL);
	if (bdremove_fd_item == 0)
		return -ENOMEM;

	bdremove_fd_item->task = task;
	bdremove_fd_item->fd = fd;
	bdremove_fd_item->dev = dev;

	down(&bdremove_fd_sem);
	list_add(&bdremove_fd_item->list, &bdremove_fd_head);
	up(&bdremove_fd_sem);
	return 0;
}

int bdremove_removefdbytask(struct task_struct *task)
{
	struct bdremove_fd_item *bdremove_fd_item;
	struct list_head *p, *tmp;
	int result = -ENOENT;

	/*
	 * Find any file descriptors associated with task and safely
	 * remove them
	 */
	down(&bdremove_fd_sem);
	list_for_each_safe(p, tmp, &bdremove_fd_head) {
		bdremove_fd_item = list_entry(p, struct bdremove_fd_item, list);

		if (bdremove_fd_item->task == task) {
			list_del(&bdremove_fd_item->list);
			kfree (bdremove_fd_item);
			result = 0;
		}
	}
	up (&bdremove_fd_sem);

	return result;
}

int bdremove_removefd(struct task_struct *task, int fd, dev_t dev)
{
	struct bdremove_fd_item *bdremove_fd_item;
	struct list_head *p, *tmp;
	int result = -ENOENT;

	/*
	 * Find matching task and fd in list, if found remove it
	 */
	down (&bdremove_fd_sem);
	list_for_each_safe(p, tmp, &bdremove_fd_head) {
		bdremove_fd_item = list_entry(p, struct bdremove_fd_item, list);
		if ((bdremove_fd_item->task == task) &&
		    (bdremove_fd_item->fd == fd) &&
		    (bdremove_fd_item->dev == dev)) {
			list_del(&bdremove_fd_item->list);
			kfree(bdremove_fd_item);
			result = 0;
			break;
		}
	}
	up (&bdremove_fd_sem);
	return result;
}

int bdremove_resetfd(dev_t dev)
{
	struct list_head *p, *tmp;
	struct file *filp;
	struct bdremove_fd_item *bdremove_fd_item;
	int result = -ENOENT;

	/*
	 * Loop until entire list shows no entries
	 *  Find match
	 *      delete from list and release semaphore
	 *      release file information and free file descriptor
	 *      break from find match into loop until list shows no entries
	 */
	if (down_interruptible(&bdremove_fd_sem))
		return -ERESTARTSYS;

	list_for_each_safe(p, tmp, &bdremove_fd_head) {
		bdremove_fd_item = list_entry(p, struct bdremove_fd_item, list);

		if (bdremove_fd_item->dev == dev) {
			list_del(&bdremove_fd_item->list);

			result = 0;
			spin_lock(&bdremove_fd_item->task->files->file_lock);
			filp = files_fdtable(bdremove_fd_item->task->files)
				->fd[bdremove_fd_item->fd];
			files_fdtable(bdremove_fd_item->task->files)
				->fd[bdremove_fd_item->fd] = NULL;

			spin_unlock(&bdremove_fd_item->task->files->file_lock);
			filp_close(filp, bdremove_fd_item->task->files);
			kfree(bdremove_fd_item);
		}
	}
	up(&bdremove_fd_sem);

	return result;
}

static int bdremove_fumount(dev_t dev)
{
	struct super_block *sb;
	int ret = 0;

	sb = user_get_super(dev);
	if (!sb)
	    return 0;

	br_write_lock(&vfsmount_lock);
	while (!list_empty(&sb->s_mounts)) {
		struct mount *mnt = list_first_entry(&sb->s_mounts,
					 struct mount, mnt_instance);
		mntget(&mnt->mnt);
		mnt->mnt_ghosts++;
		br_write_unlock(&vfsmount_lock);
		ret = do_umount(mnt, MNT_FFORCE);
		br_write_lock(&vfsmount_lock);
		mnt->mnt_ghosts--;
		br_write_unlock(&vfsmount_lock);
		up_read(&sb->s_umount);
		mntput(&mnt->mnt);
		down_read(&sb->s_umount);
		if (ret) {
			printk(KERN_WARNING "%s: umount failed: %d\n",
			       __func__, ret);
			goto out;
		}
		br_write_lock(&vfsmount_lock);
	}
	br_write_unlock(&vfsmount_lock);
out:
	drop_super(sb);
	return ret;
}

ssize_t disk_remove_store(struct device *dev,
			  struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct gendisk *disk = dev_to_disk(dev);
	int ret = 0;
	dev_t devt;
	struct block_device *bdev = NULL;
	struct disk_part_iter piter;
	struct hd_struct *part;

	disk_part_iter_init(&piter, disk, DISK_PITER_INCL_EMPTY);
	while ((part = disk_part_iter_next(&piter))) {
		devt = part_devt(part);
		ret = bdremove_fumount(devt);
		if (ret)
			goto out;
		ret = bdremove_resetfd(devt);
		if (ret && ret != -ENOENT)
			goto out;

		/* ok, now lets remove the partition from kernel */
		bdev = bdget_disk(disk, part->partno);
		if(!bdev)
			continue;
		mutex_lock(&bdev->bd_mutex);
		invalidate_bdev(bdev);
		delete_partition(disk, part->partno);
		mutex_unlock(&bdev->bd_mutex);
		bdput(bdev);
	}
	disk_part_iter_exit(&piter);

	devt = disk_devt(disk);
	/* non-physical disk, most likely LVM or MD */
	if (!disk_part_scan_enabled(disk))
		bdremove_fumount(devt);

	ret = bdremove_resetfd(devt);
	if (ret && ret != -ENOENT)
		goto out;

	if (disk->fops->remove)
		ret = disk->fops->remove(disk, 0);

out:
	if (ret)
		printk(KERN_WARNING "%s: blkdev remove failed\n", __func__);

	return strlen(buf);
}
EXPORT_SYMBOL(disk_remove_store);

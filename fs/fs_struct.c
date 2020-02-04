#include <linux/export.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/path.h>
#include <linux/slab.h>
#include <linux/fs_struct.h>
#include "internal.h"

static void modify_fs_path(struct fs_struct *fs, struct path *old_path,
			   struct path *new_path, const struct path *path)
{
	write_seqcount_begin(&fs->seq);
	*old_path = *new_path;
	*new_path = *path;
	write_seqcount_end(&fs->seq);
}

/*
 * Replace the fs->{rootmnt,root} with {mnt,dentry}. Put the old values.
 * It can block.
 */
void set_fs_root(struct fs_struct *fs, const struct path *path)
{
	struct path old_root;

	path_get(path);
	spin_lock(&fs->lock);
	modify_fs_path(fs, &old_root, &fs->root, path);
	spin_unlock(&fs->lock);
	if (old_root.dentry)
		path_put(&old_root);
}

/*
 * Replace the fs->{pwdmnt,pwd} with {mnt,dentry}. Put the old values.
 * It can block.
 */
void set_fs_pwd(struct fs_struct *fs, const struct path *path)
{
	struct path old_pwd;

	path_get(path);
	spin_lock(&fs->lock);
	modify_fs_path(fs, &old_pwd, &fs->pwd, path);
	spin_unlock(&fs->lock);

	if (old_pwd.dentry)
		path_put(&old_pwd);
}

static inline int replace_path(struct path *p, const struct path *old, const struct path *new)
{
	if (likely(p->dentry != old->dentry || p->mnt != old->mnt))
		return 0;
	*p = *new;
	return 1;
}

#ifdef CONFIG_FUMOUNT
/*
 * Note that this whole process is obviously racy, in that changes
 * can sneak in while it is processing.  That's ok, it is retried.
 * Making it non-racy would be extraordinarly complex.
 */
static void fumount_clear_one_path(struct fs_struct *fs, struct vfsmount *mnt,
				   struct path *new_path, struct path *path)
{
	struct path old_path;

	path_get(path);
	modify_fs_path(fs, &old_path, new_path, path);
	spin_unlock(&fs->lock);
	rcu_read_unlock();
	path_put(&old_path);
}

/*
 * Move the cwd and root of any process that points to this mount for
 * pwd to use '/'.
 */
void fs_fumount_clear_cwd(struct vfsmount *mnt, struct path *root_path)
{
	struct task_struct *p;
	struct fs_struct *fs;

	/*
	 * We have to release the rcu lock (and fs lock, of course) to
	 * put a path, so fumount_clear_one_path does that.
	 * Unfortunately, that mean the processes could have all
	 * changed, so we have to start over from scratch when this
	 * happens.
	 */
restart:
	rcu_read_lock();
	for_each_process(p) {
		fs = p->fs;
		if (!fs)
			continue;
		spin_lock(&fs->lock);
		if (fs->pwd.mnt == mnt) {
			fumount_clear_one_path(fs, mnt, &fs->pwd, root_path);
			goto restart;
		}
		if (fs->root.mnt == mnt) {
			fumount_clear_one_path(fs, mnt, &fs->root, root_path);
			goto restart;
		}
		spin_unlock(&fs->lock);
	}
	rcu_read_unlock();
}
#endif

void chroot_fs_refs(const struct path *old_root, const struct path *new_root)
{
	struct task_struct *g, *p;
	struct fs_struct *fs;
	int count = 0;

	read_lock(&tasklist_lock);
	do_each_thread(g, p) {
		task_lock(p);
		fs = p->fs;
		if (fs) {
			int hits = 0;
			spin_lock(&fs->lock);
			write_seqcount_begin(&fs->seq);
			hits += replace_path(&fs->root, old_root, new_root);
			hits += replace_path(&fs->pwd, old_root, new_root);
			write_seqcount_end(&fs->seq);
			while (hits--) {
				count++;
				path_get(new_root);
			}
			spin_unlock(&fs->lock);
		}
		task_unlock(p);
	} while_each_thread(g, p);
	read_unlock(&tasklist_lock);
	while (count--)
		path_put(old_root);
}

void free_fs_struct(struct fs_struct *fs)
{
	path_put(&fs->root);
	path_put(&fs->pwd);
	kmem_cache_free(fs_cachep, fs);
}

void exit_fs(struct task_struct *tsk)
{
	struct fs_struct *fs = tsk->fs;

	if (fs) {
		int kill;
		task_lock(tsk);
		spin_lock(&fs->lock);
		tsk->fs = NULL;
		kill = !atomic_dec_return(&fs->users);
		spin_unlock(&fs->lock);
		task_unlock(tsk);
		if (kill)
			free_fs_struct(fs);
	}
}

struct fs_struct *copy_fs_struct(struct fs_struct *old)
{
	struct fs_struct *fs = kmem_cache_alloc(fs_cachep, GFP_KERNEL);
	/* We don't need to lock fs - think why ;-) */
	if (fs) {
		atomic_set(&fs->users, 1);
		fs->in_exec = 0;
		spin_lock_init(&fs->lock);
		seqcount_init(&fs->seq);
		fs->umask = old->umask;

		spin_lock(&old->lock);
		fs->root = old->root;
		path_get(&fs->root);
		fs->pwd = old->pwd;
		path_get(&fs->pwd);
		spin_unlock(&old->lock);
	}
	return fs;
}

int unshare_fs_struct(void)
{
	struct fs_struct *fs = current->fs;
	struct fs_struct *new_fs = copy_fs_struct(fs);
	int kill;

	if (!new_fs)
		return -ENOMEM;

	task_lock(current);
	spin_lock(&fs->lock);
	kill = !atomic_dec_return(&fs->users);
	current->fs = new_fs;
	spin_unlock(&fs->lock);
	task_unlock(current);

	if (kill)
		free_fs_struct(fs);

	return 0;
}
EXPORT_SYMBOL_GPL(unshare_fs_struct);

int current_umask(void)
{
	return current->fs->umask;
}
EXPORT_SYMBOL(current_umask);

/* to be mentioned only in INIT_TASK */
struct fs_struct init_fs = {
	.users		= ATOMIC_INIT(1),
	.lock		= __SPIN_LOCK_UNLOCKED(init_fs.lock),
	.seq		= SEQCNT_ZERO,
	.umask		= 0022,
};

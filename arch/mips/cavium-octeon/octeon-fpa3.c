/*
 * Driver for the Octeon III Free Pool Unit (fpa).
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2015 Cavium Networks, Inc.
 */

#include <linux/module.h>

#include <asm/octeon/octeon.h>


#define GENMASK_ULL(h, l) \
	(((~0ULL) << (l)) & (~0ULL >> (BITS_PER_LONG_LONG - 1 - (h))))

/* Registers are accessed via xkphys */
#define SET_XKPHYS			(1ull << 63)
#define NODE_OFFSET			0x1000000000ull
#define SET_NODE(node)			((node) * NODE_OFFSET)

#define FPA_BASE			0x1280000000000ull
#define SET_FPA_BASE(node)		(SET_XKPHYS + SET_NODE(node) + FPA_BASE)

#define FPA_GEN_CFG(n)			(SET_FPA_BASE(n)           + 0x00000050)

#define FPA_POOLX_CFG(n, p)		(SET_FPA_BASE(n) + (p<<3)  + 0x10000000)
#define FPA_POOLX_START_ADDR(n, p)	(SET_FPA_BASE(n) + (p<<3)  + 0x10500000)
#define FPA_POOLX_END_ADDR(n, p)	(SET_FPA_BASE(n) + (p<<3)  + 0x10600000)
#define FPA_POOLX_STACK_BASE(n, p)	(SET_FPA_BASE(n) + (p<<3)  + 0x10700000)
#define FPA_POOLX_STACK_END(n, p)	(SET_FPA_BASE(n) + (p<<3)  + 0x10800000)
#define FPA_POOLX_STACK_ADDR(n, p)	(SET_FPA_BASE(n) + (p<<3)  + 0x10900000)

#define FPA_AURAX_POOL(n, a)		(SET_FPA_BASE(n) + (a<<3)  + 0x20000000)
#define FPA_AURAX_CFG(n, a)		(SET_FPA_BASE(n) + (a<<3)  + 0x20100000)
#define FPA_AURAX_CNT(n, a)		(SET_FPA_BASE(n) + (a<<3)  + 0x20200000)
#define FPA_AURAX_CNT_LIMIT(n, a)	(SET_FPA_BASE(n) + (a<<3)  + 0x20400000)
#define FPA_AURAX_CNT_THRESHOLD(n, a)	(SET_FPA_BASE(n) + (a<<3)  + 0x20500000)
#define FPA_AURAX_POOL_LEVELS(n, a)	(SET_FPA_BASE(n) + (a<<3)  + 0x20700000)
#define FPA_AURAX_CNT_LEVELS(n, a)	(SET_FPA_BASE(n) + (a<<3)  + 0x20800000)

static inline u64 oct_csr_read(u64 addr)
{
	return __raw_readq((void __iomem *)addr);
}

static inline void oct_csr_write(u64 data, u64 addr)
{
	__raw_writeq(data, (void __iomem *)addr);
}

static DEFINE_MUTEX(octeon_fpa3_lock);

static int get_num_pools(void)
{
	if (OCTEON_IS_MODEL(OCTEON_CN78XX))
		return 64;
	if (OCTEON_IS_MODEL(OCTEON_CNF75XX) || OCTEON_IS_MODEL(OCTEON_CN73XX))
		return 32;
	return 0;
}

static int get_num_auras(void)
{
	if (OCTEON_IS_MODEL(OCTEON_CN78XX))
		return 1024;
	if (OCTEON_IS_MODEL(OCTEON_CNF75XX) || OCTEON_IS_MODEL(OCTEON_CN73XX))
		return 512;
	return 0;
}

/*
 * octeon_fpa3_init:		Initialize the fpa to default values.
 *
 *  node:			Node of fpa to initialize.
 *
 *  Returns:			Zero on success, error otherwise.
 */
int octeon_fpa3_init(int node)
{
	static bool	init_done[2];
	u64		data;
	int			i;
	int			aura_cnt;

	mutex_lock(&octeon_fpa3_lock);

	if (init_done[node])
		goto done;

	aura_cnt = get_num_auras();
	for (i = 0; i < aura_cnt; i++) {
		oct_csr_write(0x100000000ull, FPA_AURAX_CNT(node, i));
		oct_csr_write(0xfffffffffull, FPA_AURAX_CNT_LIMIT(node, i));
		oct_csr_write(0xffffffffeull, FPA_AURAX_CNT_THRESHOLD(node, i));
	}

	data = oct_csr_read(FPA_GEN_CFG(node));
	data &= ~GENMASK_ULL(9, 4);
	data |= 3 << 4;
	oct_csr_write(data, FPA_GEN_CFG(node));

	init_done[node] = 1;
 done:
	mutex_unlock(&octeon_fpa3_lock);
	return 0;
}
EXPORT_SYMBOL(octeon_fpa3_init);

/*
 * octeon_fpa3_pool_init:	Initialize a pool.
 *
 *  node:			Node to initialize pool on.
 *  pool_num:			Requested pool number (-1 for don't care).
 *  pool:			Updated with the initialized pool number.
 *  pool_stack:			Updated with the base of the memory allocated
 *				for the pool stack.
 *  num_ptrs:			Number of pointers to allocated on the stack.
 *
 *  Returns:			Zero on success, error otherwise.
 */
int octeon_fpa3_pool_init(int			node,
			  int			pool_num,
			  int	*pool,
			  void			**pool_stack,
			  int			num_ptrs)
{
	struct global_resource_tag	tag;
	char				buf[16];
	u64				pool_stack_start;
	u64				pool_stack_end;
	u64				data;
	int				stack_size;
	int				rc = 0;

	mutex_lock(&octeon_fpa3_lock);

	strncpy((char *)&tag.lo, "cvm_pool", 8);
	snprintf(buf, 16, "_%d......", node);
	memcpy(&tag.hi, buf, 8);

	res_mgr_create_resource(tag, get_num_pools());
	*pool = res_mgr_alloc(tag, pool_num, true);
	if (*pool < 0) {
		rc = -ENODEV;
		goto error;
	}

	oct_csr_write(0, FPA_POOLX_CFG(node, *pool));
	oct_csr_write(128, FPA_POOLX_START_ADDR(node, *pool));
	oct_csr_write(GENMASK_ULL(41, 7), FPA_POOLX_END_ADDR(node, *pool));

	stack_size = (DIV_ROUND_UP(num_ptrs, 29) + 1) * 128;
	*pool_stack = kmalloc_node(stack_size, GFP_KERNEL, node);
	if (!*pool_stack) {
		pr_err("Failed to allocate pool stack memory pool=%d\n",
		       pool_num);
		rc = -ENOMEM;
		goto error_stack;
	}

	pool_stack_start = virt_to_phys(*pool_stack);
	pool_stack_end = round_down(pool_stack_start + stack_size, 128);
	pool_stack_start = round_up(pool_stack_start, 128);
	oct_csr_write(pool_stack_start, FPA_POOLX_STACK_BASE(node, *pool));
	oct_csr_write(pool_stack_start, FPA_POOLX_STACK_ADDR(node, *pool));
	oct_csr_write(pool_stack_end, FPA_POOLX_STACK_END(node, *pool));

	data = (2 << 3) | BIT(0);
	oct_csr_write(data, FPA_POOLX_CFG(node, *pool));

	mutex_unlock(&octeon_fpa3_lock);
	return 0;

 error_stack:
	res_mgr_free(tag, *pool);
 error:
	mutex_unlock(&octeon_fpa3_lock);
	return rc;
}
EXPORT_SYMBOL(octeon_fpa3_pool_init);

/*
 * octeon_fpa3_release_pool:	Release a pool.
 *
 *  node:			Node pool is on.
 *  pool:			Pool to release.
 */
void octeon_fpa3_release_pool(int node, int pool)
{
	struct global_resource_tag	tag;
	char				buf[16];

	mutex_lock(&octeon_fpa3_lock);

	strncpy((char *)&tag.lo, "cvm_pool", 8);
	snprintf(buf, 16, "_%d......", node);
	memcpy(&tag.hi, buf, 8);

	res_mgr_free(tag, pool);

	mutex_unlock(&octeon_fpa3_lock);
}
EXPORT_SYMBOL(octeon_fpa3_release_pool);

/*
 * octeon_fpa3_aura_init:	Initialize an aura.
 *
 *  node:			Node to initialize aura on.
 *  pool:			Pool the aura belongs to.
 *  aura_num:			Requested aura number (-1 for don't care).
 *  aura:			Updated with the initialized aura number.
 *  num_bufs:			Number of buffers in the aura.
 *  limit:			Limit for the aura.
 *
 *  Returns:			Zero on success, error otherwise.
 */
int octeon_fpa3_aura_init(int		node,
			  int		pool,
			  int			aura_num,
			  int		*aura,
			  int			num_bufs,
			  unsigned int		limit)
{
	struct global_resource_tag	tag;
	char				buf[16];
	u64				data;
	u64				shift;
	unsigned int			drop;
	unsigned int			pass;
	int				rc = 0;

	mutex_lock(&octeon_fpa3_lock);

	strncpy((char *)&tag.lo, "cvm_aura", 8);
	snprintf(buf, 16, "_%d......", node);
	memcpy(&tag.hi, buf, 8);

	res_mgr_create_resource(tag, get_num_auras());
	*aura = res_mgr_alloc(tag, aura_num, true);
	if (*aura < 0) {
		rc = -ENODEV;
		goto error;
	}

	oct_csr_write(0, FPA_AURAX_CFG(node, *aura));

	/* Allow twice the limit before saturation at zero */
	limit *= 2;
	data = limit;
	oct_csr_write(data, FPA_AURAX_CNT_LIMIT(node, *aura));
	oct_csr_write(data, FPA_AURAX_CNT(node, *aura));

	oct_csr_write(pool, FPA_AURAX_POOL(node, *aura));

	/* No per-pool RED/Drop */
	oct_csr_write(0, FPA_AURAX_POOL_LEVELS(node, *aura));

	shift = 0;
	while ((limit >> shift) > 255)
		shift++;

	drop = (limit - num_bufs / 20) >> shift;	/* 95% */
	pass = (limit - (num_bufs * 3) / 20) >> shift;	/* 85% */

	/* Enable per aura RED/drop */
	data = BIT(38) | (shift << 32) | (drop << 16) | (pass << 8);
	oct_csr_write(data, FPA_AURAX_CNT_LEVELS(node, *aura));

 error:
	mutex_unlock(&octeon_fpa3_lock);
	return rc;
}
EXPORT_SYMBOL(octeon_fpa3_aura_init);

/*
 * octeon_fpa3_release_aura:	Release an aura.
 *
 *  node:			Node to aura is on.
 *  aura:			Aura to release.
 */
void octeon_fpa3_release_aura(int node, int aura)
{
	struct global_resource_tag	tag;
	char				buf[16];

	mutex_lock(&octeon_fpa3_lock);

	strncpy((char *)&tag.lo, "cvm_aura", 8);
	snprintf(buf, 16, "_%d......", node);
	memcpy(&tag.hi, buf, 8);

	res_mgr_free(tag, aura);

	mutex_unlock(&octeon_fpa3_lock);
}
EXPORT_SYMBOL(octeon_fpa3_release_aura);

/*
 * octeon_fpa3_alloc:		Get a buffer from a aura's pool.
 *
 *  node:			Node to free memory to.
 *  aura:			Aura to free memory to.
 *
 *  returns:			Allocated buffer pointer, or NULL on error.
 */
void *octeon_fpa3_alloc(u64 node, int aura)
{
	u64	addr;
	u64	buf_phys;
	void	*buf = NULL;

	/* Buffer pointers are obtained using load operations */
	addr = BIT(63) | BIT(48) | (0x29ull << 40) | (node << 36) |
		(aura << 16);
	buf_phys = *(u64 *)addr;

	if (buf_phys)
		buf = phys_to_virt(buf_phys);

	return buf;
}
EXPORT_SYMBOL(octeon_fpa3_alloc);

/*
 * octeon_fpa3_free:		Add a buffer back to the aura's pool.
 *
 *  node:			Node to free memory to.
 *  aura:			Aura to free memory to.
 *  buf:			Address of buffer to free to the aura's pool.
 */
void octeon_fpa3_free(u64 node, int aura, const void *buf)
{
	u64	buf_phys;
	u64	addr;

	buf_phys = virt_to_phys(buf);

	/* Make sure that any previous writes to memory go out before we free
	   this buffer. This also serves as a barrier to prevent GCC from
	   reordering operations to after the free. */
	wmb();

	/* Buffers are added to fpa pools using store operations */
	addr = BIT(63) | BIT(48) | (0x29ull << 40) | (node << 36) |
		(aura << 16);
	*(u64 *)addr = buf_phys;
}
EXPORT_SYMBOL(octeon_fpa3_free);

/*
 * octeon_fpa3_mem_fill:	Add buffers to an aura.
 *
 *  node:			Node to get memory from.
 *  cache:			Memory cache to allocate from.
 *  aura:			Aura to add buffers to.
 *  num_bufs:			Number of buffers to add to the aura.
 *
 *  Returns:			Zero on success, error otherwise.
 */
int octeon_fpa3_mem_fill(int			node,
			 struct kmem_cache	*cache,
			 int			aura,
			 int			num_bufs)
{
	void	*mem;
	int	i;
	int	rc = 0;

	mutex_lock(&octeon_fpa3_lock);

	for (i = 0; i < num_bufs; i++) {
		mem = kmem_cache_alloc_node(cache, GFP_KERNEL, node);
		if (!mem) {
			pr_err("Failed to allocate memory for aura=%d\n", aura);
			rc = -ENOMEM;
			break;
		}
		octeon_fpa3_free(node, aura, mem);
	}

	mutex_unlock(&octeon_fpa3_lock);
	return rc;
}
EXPORT_SYMBOL(octeon_fpa3_mem_fill);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cavium, Inc. Octeon III FPA manager.");

/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (c) 2003-2013 Cavium Inc. All rights reserved.
 *
 */
#ifndef __ASM_OCTEON_OCTEON_H
#define __ASM_OCTEON_OCTEON_H

#include <linux/irqflags.h>
#include <linux/notifier.h>
#include <asm/octeon/cvmx.h>
#include <asm/octeon/cvmx-fpa3.h>
#include <linux/irq.h>
#include <linux/slab.h>

extern int octeon_is_simulation(void);
extern int octeon_is_pci_host(void);
extern int octeon_usb_is_ref_clk(void);
extern uint64_t octeon_get_clock_rate(void);
extern u64 octeon_get_io_clock_rate(void);
extern const char *octeon_board_type_string(void);
extern const char *octeon_get_pci_interrupts(void);
extern void octeon_user_io_init(void);
extern asmlinkage void octeon_cop2_restore(struct octeon_cop2_state *task);

extern void octeon_init_cvmcount(void);
extern void octeon_setup_delays(void);
extern void octeon_io_clk_delay(unsigned long);

#define OCTEON_ARGV_MAX_ARGS	64
#define OCTOEN_SERIAL_LEN	20

struct octeon_boot_descriptor {
#ifdef __BIG_ENDIAN_BITFIELD
	/* Start of block referenced by assembly code - do not change! */
	uint32_t desc_version;
	uint32_t desc_size;
	uint64_t stack_top;
	uint64_t heap_base;
	uint64_t heap_end;
	/* Only used by bootloader */
	uint64_t entry_point;
	uint64_t desc_vaddr;
	/* End of This block referenced by assembly code - do not change! */
	uint32_t exception_base_addr;
	uint32_t stack_size;
	uint32_t heap_size;
	/* Argc count for application. */
	uint32_t argc;
	uint32_t argv[OCTEON_ARGV_MAX_ARGS];

#define	 BOOT_FLAG_INIT_CORE		(1 << 0)
#define	 OCTEON_BL_FLAG_DEBUG		(1 << 1)
#define	 OCTEON_BL_FLAG_NO_MAGIC	(1 << 2)
	/* If set, use uart1 for console */
#define	 OCTEON_BL_FLAG_CONSOLE_UART1	(1 << 3)
	/* If set, use PCI console */
#define	 OCTEON_BL_FLAG_CONSOLE_PCI	(1 << 4)
	/* Call exit on break on serial port */
#define	 OCTEON_BL_FLAG_BREAK		(1 << 5)

	uint32_t flags;
	uint32_t core_mask;
	/* DRAM size in megabyes. */
	uint32_t dram_size;
	/* physical address of free memory descriptor block. */
	uint32_t phy_mem_desc_addr;
	/* used to pass flags from app to debugger. */
	uint32_t debugger_flags_base_addr;
	/* CPU clock speed, in hz. */
	uint32_t eclock_hz;
	/* DRAM clock speed, in hz. */
	uint32_t dclock_hz;
	/* SPI4 clock in hz. */
	uint32_t spi_clock_hz;
	uint16_t board_type;
	uint8_t board_rev_major;
	uint8_t board_rev_minor;
	uint16_t chip_type;
	uint8_t chip_rev_major;
	uint8_t chip_rev_minor;
	char board_serial_number[OCTOEN_SERIAL_LEN];
	uint8_t mac_addr_base[6];
	uint8_t mac_addr_count;
	uint64_t cvmx_desc_vaddr;
#else
	uint32_t desc_size;
	uint32_t desc_version;
	uint64_t stack_top;
	uint64_t heap_base;
	uint64_t heap_end;
	/* Only used by bootloader */
	uint64_t entry_point;
	uint64_t desc_vaddr;
	/* End of This block referenced by assembly code - do not change! */
	uint32_t stack_size;
	uint32_t exception_base_addr;
	uint32_t argc;
	uint32_t heap_size;
	/* Argc count for application. Warning low bit scrambled in little-endian. */
	uint32_t argv[OCTEON_ARGV_MAX_ARGS];

#define  BOOT_FLAG_INIT_CORE		(1 << 0)
#define  OCTEON_BL_FLAG_DEBUG		(1 << 1)
#define  OCTEON_BL_FLAG_NO_MAGIC	(1 << 2)
	/* If set, use uart1 for console */
#define  OCTEON_BL_FLAG_CONSOLE_UART1	(1 << 3)
	/* If set, use PCI console */
#define  OCTEON_BL_FLAG_CONSOLE_PCI	(1 << 4)
	/* Call exit on break on serial port */
#define  OCTEON_BL_FLAG_BREAK		(1 << 5)

	uint32_t core_mask;
	uint32_t flags;
	/* physical address of free memory descriptor block. */
	uint32_t phy_mem_desc_addr;
	/* DRAM size in megabyes. */
	uint32_t dram_size;
	/* CPU clock speed, in hz. */
	uint32_t eclock_hz;
	/* used to pass flags from app to debugger. */
	uint32_t debugger_flags_base_addr;
	/* SPI4 clock in hz. */
	uint32_t spi_clock_hz;
	/* DRAM clock speed, in hz. */
	uint32_t dclock_hz;
	uint8_t chip_rev_minor;
	uint8_t chip_rev_major;
	uint16_t chip_type;
	uint8_t board_rev_minor;
	uint8_t board_rev_major;
	uint16_t board_type;

	uint64_t unused1[4]; /* Not even filled in by bootloader. */

	uint64_t cvmx_desc_vaddr;
#endif
};

union octeon_cvmemctl {
	uint64_t u64;
	struct {
#ifdef __BIG_ENDIAN_BITFIELD
		/* RO 1 = BIST fail, 0 = BIST pass */
		uint64_t tlbbist:1;
		/* RO 1 = BIST fail, 0 = BIST pass */
		uint64_t l1cbist:1;
		/* RO 1 = BIST fail, 0 = BIST pass */
		uint64_t l1dbist:1;
		/* RO 1 = BIST fail, 0 = BIST pass */
		uint64_t dcmbist:1;
		/* RO 1 = BIST fail, 0 = BIST pass */
		uint64_t ptgbist:1;
		/* RO 1 = BIST fail, 0 = BIST pass */
		uint64_t wbfbist:1;
		/* Reserved */
		uint64_t reserved:6;
		/* When set, LMTDMA/LMTST operations are permitted */
		uint64_t lmtena:1;
		/* Selects the CVMSEG LM cacheline used by LMTDMA
		   LMTST and wide atomic store operations */
		uint64_t lmtline:6;
		/* When set, TLB parity errors can occur. */
		uint64_t tlbperrena:1;
		/* OCTEON II - When set, CVMSET LM parity errors are enabled. */
		uint64_t lmemperrena:1;
		/* OCTEON II - If set, NUDGE/WRITEBACK_INVALIDATE,
		 * NUDGE_WB, EVICT_SOON, LC, CONT_WRITE_BACK,
		 * PREPARE_FOR_STORE and PREPARE_FOR_STORE_THROUGH
		 * prefetch operations become NOPs. */
		uint64_t disstpref:1;
		/* OCTEON II - If set, NORMAL and NOTL2 prefetch
		 * operations become NOPs. */
		uint64_t disldpref:1;
		/* OCTEON II - TLB replacement policy: 0 = bitmask LRU; 1 = NLU.
		 * This field selects between the TLB replacement policies:
		 * bitmask LRU or NLU. Bitmask LRU maintains a mask of
		 * recently used TLB entries and avoids them as new entries
		 * are allocated. NLU simply guarantees that the next
		 * allocation is not the last used TLB entry. */
		uint64_t tlbnlu:1;
		/* OCTEON II - Selects the bit in the counter used for releasing
		 * a PAUSE. This counter trips every 2(8+PAUSETIME) cycles. If
		 * not already released, the cnMIPS II core will always release
		 * a given PAUSE instruction within 2(8+PAUSETIME). If the
		 * counter trip happens to line up, the cnMIPS II core may
		 * release the PAUSE instantly. */
		uint64_t pausetime:3;
		/* OCTEON II - This field is an extension of CvmMemCtl[DIDTTO] */
		uint64_t didtto2:1;
		/* R/W If set, marked write-buffer entries time out
		 * the same as as other entries; if clear, marked
		 * write-buffer entries use the maximum timeout. */
		uint64_t dismarkwblongto:1;
		/* R/W If set, a merged store does not clear the
		 * write-buffer entry timeout state. */
		uint64_t dismrgclrwbto:1;
		/* R/W Two bits that are the MSBs of the resultant
		 * CVMSEG LM word location for an IOBDMA. The other 8
		 * bits come from the SCRADDR field of the IOBDMA. */
		uint64_t iobdmascrmsb:2;
		/* R/W If set, SYNCWS and SYNCS only order marked
		 * stores; if clear, SYNCWS and SYNCS only order
		 * unmarked stores. SYNCWSMARKED has no effect when
		 * DISSYNCWS is set. */
		uint64_t syncwsmarked:1;
		/* R/W If set, SYNCWS acts as SYNCW and SYNCS acts as
		 * SYNC. */
		uint64_t dissyncws:1;
		/* R/W If set, no stall happens on write buffer
		 * full. */
		uint64_t diswbfst:1;
		/* R/W If set (and SX set), supervisor-level
		 * loads/stores can use XKPHYS addresses with
		 * VA<48>==0 */
		uint64_t xkmemenas:1;
		/* R/W If set (and UX set), user-level loads/stores
		 * can use XKPHYS addresses with VA<48>==0 */
		uint64_t xkmemenau:1;
		/* R/W If set (and SX set), supervisor-level
		 * loads/stores can use XKPHYS addresses with
		 * VA<48>==1 */
		uint64_t xkioenas:1;
		/* R/W If set (and UX set), user-level loads/stores
		 * can use XKPHYS addresses with VA<48>==1 */
		uint64_t xkioenau:1;
		/* R/W If set, all stores act as SYNCW (NOMERGE must
		 * be set when this is set) RW, reset to 0. */
		uint64_t allsyncw:1;
		/* R/W If set, no stores merge, and all stores reach
		 * the coherent bus in order. */
		uint64_t nomerge:1;
		/* R/W Selects the bit in the counter used for DID
		 * time-outs 0 = 231, 1 = 230, 2 = 229, 3 =
		 * 214. Actual time-out is between 1x and 2x this
		 * interval. For example, with DIDTTO=3, expiration
		 * interval is between 16K and 32K. */
		uint64_t didtto:2;
		/* R/W If set, the (mem) CSR clock never turns off. */
		uint64_t csrckalwys:1;
		/* R/W If set, mclk never turns off. */
		uint64_t mclkalwys:1;
		/* R/W Selects the bit in the counter used for write
		 * buffer flush time-outs (WBFLT+11) is the bit
		 * position in an internal counter used to determine
		 * expiration. The write buffer expires between 1x and
		 * 2x this interval. For example, with WBFLT = 0, a
		 * write buffer expires between 2K and 4K cycles after
		 * the write buffer entry is allocated. */
		uint64_t wbfltime:3;
		/* R/W If set, do not put Istream in the L2 cache. */
		uint64_t istrnol2:1;
		/* R/W The write buffer threshold. */
		uint64_t wbthresh:4;
		/* Reserved */
		uint64_t reserved2:2;
		/* R/W If set, CVMSEG is available for loads/stores in
		 * kernel/debug mode. */
		uint64_t cvmsegenak:1;
		/* R/W If set, CVMSEG is available for loads/stores in
		 * supervisor mode. */
		uint64_t cvmsegenas:1;
		/* R/W If set, CVMSEG is available for loads/stores in
		 * user mode. */
		uint64_t cvmsegenau:1;
		/* R/W Size of local memory in cache blocks, 54 (6912
		 * bytes) is max legal value. */
		uint64_t lmemsz:6;
#else
		uint64_t lmemsz:6;
		uint64_t cvmsegenau:1;
		uint64_t cvmsegenas:1;
		uint64_t cvmsegenak:1;
		uint64_t reserved2:2;
		uint64_t wbthresh:4;
		uint64_t istrnol2:1;
		uint64_t wbfltime:3;
		uint64_t mclkalwys:1;
		uint64_t csrckalwys:1;
		uint64_t didtto:2;
		uint64_t nomerge:1;
		uint64_t allsyncw:1;
		uint64_t xkioenau:1;
		uint64_t xkioenas:1;
		uint64_t xkmemenau:1;
		uint64_t xkmemenas:1;
		uint64_t diswbfst:1;
		uint64_t dissyncws:1;
		uint64_t syncwsmarked:1;
		uint64_t iobdmascrmsb:2;
		uint64_t dismrgclrwbto:1;
		uint64_t dismarkwblongto:1;
		uint64_t didtto2:1;
		uint64_t pausetime:3;
		uint64_t tlbnlu:1;
		uint64_t disldpref:1;
		uint64_t disstpref:1;
		uint64_t lmemperrena:1;
		uint64_t tlbperrena:1;
		uint64_t lmtline:6;
		uint64_t lmtena:1;
		uint64_t reserved:6;
		uint64_t wbfbist:1;
		uint64_t ptgbist:1;
		uint64_t dcmbist:1;
		uint64_t l1dbist:1;
		uint64_t l1cbist:1;
		uint64_t tlbbist:1;
#endif
	} s;
};

struct octeon_ciu_chip_data {
	union {
		struct {		/* only used for ciu3 */
			u64 ciu3_addr;
			union {
				unsigned int intsn;
				unsigned int idt; /* For errbit irq */
			};
		};
		struct {		/* only used for ciu/ciu2 */
			u8 line;
			u8 bit;
		};
	};
	int gpio_line;
	int current_cpu;	/* Next CPU expected to take this irq */
	int ciu_node; /* NUMA node number of the CIU */
	int trigger_type;
};

struct octeon_edac_lmc_data {
	uint8_t node;		/** CPU node number */
	uint8_t lmc;		/** LMC interface number on node */
};

struct edac_device_ctl_info;

struct octeon_edac_l2c_data {
	struct edac_device_ctl_info *ed;
	struct platform_device *pdev;
	uint8_t node;		/** CPU node number */
	uint8_t tad;		/** L2C TAD number */
};

extern void octeon_write_lcd(const char *s);
extern void octeon_check_cpu_bist(void);
extern int octeon_get_boot_debug_flag(void);
extern int octeon_get_boot_uart(void);

struct uart_port;
extern unsigned int octeon_serial_in(struct uart_port *, int);
extern void octeon_serial_out(struct uart_port *, int, int);

/**
 * Write a 32bit value to the Octeon NPI register space
 *
 * @address: Address to write to
 * @val:     Value to write
 */
static inline void octeon_npi_write32(uint64_t address, uint32_t val)
{
	cvmx_write64_uint32(address ^ 4, val);
	cvmx_read64_uint32(address ^ 4);
}


/**
 * Read a 32bit value from the Octeon NPI register space
 *
 * @address: Address to read
 * Returns The result
 */
static inline uint32_t octeon_npi_read32(uint64_t address)
{
	return cvmx_read64_uint32(address ^ 4);
}

extern struct cvmx_bootinfo *octeon_bootinfo;

extern u32 octeon_cvmseg_lines;

static inline uint64_t octeon_read_ptp_csr(u64 csr)
{
	if (OCTEON_IS_MODEL(OCTEON_CN63XX_PASS1_X)) {
		u64 result;
		unsigned long flags;
		/*
		 * CN63XX pass 1.x has an errata where you must read
		 * this register twice to get the correct result.
		 */
		local_irq_save(flags);
		cvmx_read_csr(csr);
		result = cvmx_read_csr(csr);
		local_irq_restore(flags);
		return result;
	} else {
		return cvmx_read_csr(csr);
	}
}

extern void (*octeon_irq_setup_secondary)(void);

int octeon_coreid_for_cpu(int cpu);
int octeon_cpu_for_coreid(int coreid);

void octeon_pci_console_init(const char *);

typedef void (*octeon_message_fn_t)(void);
int octeon_request_ipi_handler(octeon_message_fn_t fn);
void octeon_send_ipi_single(int cpu, unsigned int action);
void octeon_release_ipi_handler(int action);
void octeon_ciu3_mbox_send(int cpu, unsigned int mbox);
void octeon_irq_ciu3_enable(struct irq_data *data);
void octeon_irq_ciu3_disable(struct irq_data *data);
void octeon_irq_ciu3_mask(struct irq_data *data);
void octeon_irq_ciu3_ack(struct irq_data *data);
void octeon_irq_ciu3_mask_ack(struct irq_data *data);
int octeon_irq_ciu_set_type(struct irq_data *data, unsigned int flow_type);
int octeon_irq_ciu3_set_affinity(struct irq_data *data,
				 const struct cpumask *dest, bool force);
void octeon_irq_free_cd(struct irq_domain *d, unsigned int irq);
int octeon_irq_ciu3_xlat(struct irq_domain *d, struct device_node *node,
			 const u32 *intspec, unsigned int intsize,
			 unsigned long *out_hwirq, unsigned int *out_type);
int octeon_irq_ciu3_mapx(struct irq_domain *d, unsigned int virq,
			 irq_hw_number_t hw, struct irq_chip *chip);
void *octeon_irq_get_ciu3_info(int node);
void octeon_irq_add_block_domain(int node, uint8_t block,
				 struct irq_domain *domain);
struct irq_domain *octeon_irq_get_block_domain(int node, uint8_t block);

#define OCTEON_DEBUG_UART 1

#if IS_ENABLED(CONFIG_CAVIUM_OCTEON_ERROR_TREE)
int octeon_error_tree_enable(enum cvmx_error_groups group, int unit);
int octeon_error_tree_disable(enum cvmx_error_groups group, int unit);
int octeon_error_tree_shutdown(void);
int octeon_error3_tree_enable(enum cvmx_error_groups group, int unit);
int octeon_error3_tree_disable(enum cvmx_error_groups group, int unit);
#else
static inline int octeon_error_tree_enable(enum cvmx_error_groups group, int unit)
{
	return 0;
}
static inline int octeon_error_tree_disable(enum cvmx_error_groups group, int unit)
{
	return 0;
}
static inline int octeon_error_tree_shutdown(void)
{
	return 0;
}
#endif

int octeon_ciu3_errbits_set_handler(void (* handler)(int node, int intsn));
int octeon_ciu3_errbits_enable_intsn(int node, int intsn);
int octeon_ciu3_errbits_disable_intsn(int node , int intsn);

int octeon_i2c_cvmx2i2c(unsigned int cvmx_twsi_bus_num);

#ifdef CONFIG_SMP
void octeon_setup_smp(void);
#else
static inline void octeon_setup_smp(void) {}
#endif
#ifdef CONFIG_NUMA
void octeon_setup_numa(void);
void octeon_numa_cpu_online(void);
#else
static inline void octeon_setup_numa(void) {}
static inline void octeon_numa_cpu_online(void) {}
#endif

extern struct semaphore octeon_bootbus_sem;

extern void (*octeon_scache_init)(void);
int register_co_cache_error_notifier(struct notifier_block *nb);
int unregister_co_cache_error_notifier(struct notifier_block *nb);
#define CO_CACHE_ERROR_RECOVERABLE 0
#define CO_CACHE_ERROR_UNRECOVERABLE 1
#define CO_CACHE_ERROR_WB_PARITY 2
#define CO_CACHE_ERROR_TLB_PARITY 3

extern unsigned long long cache_err_dcache[];

/* Octeon multiplier save/restore routines from octeon_switch.S */
void octeon_mult_save(void);
void octeon_mult_restore(void);
void octeon_mult_save_end(void);
void octeon_mult_restore_end(void);
void octeon_mult_save3(void);
void octeon_mult_save3_end(void);
void octeon_mult_save2(void);
void octeon_mult_save2_end(void);
void octeon_mult_restore3(void);
void octeon_mult_restore3_end(void);
void octeon_mult_restore2(void);
void octeon_mult_restore2_end(void);

/*
 * This definition must be kept in sync with the one in
 * cvmx-global-resources.c
 */
struct global_resource_tag {
	uint64_t lo;
	uint64_t hi;
};

void res_mgr_free(struct global_resource_tag tag, int inst);
void res_mgr_free_range(struct global_resource_tag tag, int *inst, int req_cnt);
int res_mgr_alloc(struct global_resource_tag tag, int req_inst,
		  bool use_last_avail);
int res_mgr_alloc_range(struct global_resource_tag tag, int req_inst,
			int req_cnt, bool use_last_avail, int *inst);
int res_mgr_create_resource(struct global_resource_tag tag, int inst_cnt);

#if IS_ENABLED(CONFIG_OCTEON_FPA3)
int octeon_fpa3_init(int node);
int octeon_fpa3_pool_init(int node, int pool_num, int *pool, void **pool_stack,
			  int num_ptrs);
int octeon_fpa3_aura_init(int node, int pool, int aura_num, int *aura,
			  int num_bufs, unsigned int limit);
int octeon_fpa3_mem_fill(int node, struct kmem_cache *cache, int aura,
			 int num_bufs);
void octeon_fpa3_free(u64 node, int aura, const void *buf);
void *octeon_fpa3_alloc(u64 node, int aura);
void octeon_fpa3_release_pool(int node, int pool);
void octeon_fpa3_release_aura(int node, int aura);
#endif

#endif /* __ASM_OCTEON_OCTEON_H */

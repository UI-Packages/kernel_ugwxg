/*
 * Per-event defines for arch/mips/cavium-octeon/perf_events_uncore.c
 *
 * Included multiple times, because some properties are setup cleaner
 * at build time, but others (like the _CNT/(_CNT_LO,_CNT_HI)) accessors
 * cannot be resolved at run-time.
 *
 * Different calls can make use of different mix of properties,
 * but they're all summarized here in one place.
 *
 * Each inclusion can differently define
 *    EV(_family, _name, _cvmx, _flags, ...)
 * to extract different subsets of properties.
 *
 * Where tables cannot be resolved at build time (octeon-model-specific
 * addresses and/or existence), tie different pieces together at runtime
 * with identifier EVID(f,n), which creates an enum used for runtime switch.
 * This is carried in event->attr.config (notionally u64, but something zeros
 * upper bits during perf-tool negotiation, so just use lower byte!)
 */
#ifndef EVID
# define EVID(_family, _name) UNC_##_family##_##_name
#endif

/* (re)undef to allow re-include, with varying EV() definitions */
#undef OCT_EVENTS
#undef LMC_EVENTS
#undef O1P_EVENTS
#undef O23_EVENTS
#undef TAD_EVENTS

/*
 * LMCX: fixed-function counters for mem-controller stats:
 * - Octeon1 has _HI/_LO, 2/3 has just _CNT
 */
#undef LMC_EV
#define LMC_EV(_name, _cvmx) EV(lmc, _name, EVID(lmc, _name), \
	UNC_DIRECT|UNC_ANY, \
	/* _lo */ (OCTEON_IS_OCTEON1PLUS() \
		? _cvmx##_CNT_LO(0) : _cvmx##_CNT(0)), \
	/* _hi */ (OCTEON_IS_OCTEON1PLUS() \
		? _cvmx##_CNT_HI(0) : 0), \
	/* _stride */ (OCTEON_IS_OCTEON1PLUS() \
		? (_cvmx##_CNT_LO(1) - _cvmx##_CNT_LO(0)) \
		: (_cvmx##_CNT(1) - _cvmx##_CNT(0))), \
	/* _lim */ lim_cvmx_lmcx)
#define LMC_EVENTS \
	LMC_EV(dclk, CVMX_LMCX_DCLK) \
	LMC_EV(ops, CVMX_LMCX_OPS) \
	LMC_EV(ifb, CVMX_LMCX_IFB) \
	/*end*/

/* Octeon1/1-plus L2-cache counters, mapped into 4 variable-func counters */
#undef O1P_EV
#define O1P_EV(_name, _cvmx) EV(o1p, _name, _cvmx, \
	UNC_MAPPED|UNC_O1P, 0, 0, 0, NULL)
#define O1P_EVENTS \
	O1P_EV(cycles, CVMX_L2C_EVENT_CYCLES) \
	O1P_EV(imiss, CVMX_L2C_EVENT_INSTRUCTION_MISS) \
	O1P_EV(ihit, CVMX_L2C_EVENT_INSTRUCTION_HIT) \
	O1P_EV(dmiss, CVMX_L2C_EVENT_DATA_MISS)\
	O1P_EV(dhit, CVMX_L2C_EVENT_DATA_HIT) \
	O1P_EV(miss, CVMX_L2C_EVENT_MISS) \
	O1P_EV(hit, CVMX_L2C_EVENT_HIT) \
	O1P_EV(victim_buffer_hit, CVMX_L2C_EVENT_VICTIM_HIT) \
	O1P_EV(lfb_nq_index_conflict, CVMX_L2C_EVENT_INDEX_CONFLICT) \
	O1P_EV(tag_probe, CVMX_L2C_EVENT_TAG_PROBE) \
	O1P_EV(tag_update, CVMX_L2C_EVENT_TAG_UPDATE) \
	O1P_EV(tag_probe_completed, CVMX_L2C_EVENT_TAG_COMPLETE) \
	O1P_EV(tag_dirty_victim, CVMX_L2C_EVENT_TAG_DIRTY) \
	O1P_EV(data_store_nop, CVMX_L2C_EVENT_DATA_STORE_NOP) \
	O1P_EV(data_store_read, CVMX_L2C_EVENT_DATA_STORE_READ) \
	O1P_EV(data_store_write, CVMX_L2C_EVENT_DATA_STORE_WRITE) \
	O1P_EV(memory_fill_data_valid, CVMX_L2C_EVENT_FILL_DATA_VALID) \
	O1P_EV(memory_write_request, CVMX_L2C_EVENT_WRITE_REQUEST) \
	O1P_EV(memory_read_request, CVMX_L2C_EVENT_READ_REQUEST) \
	O1P_EV(memory_write_data_valid, CVMX_L2C_EVENT_WRITE_DATA_VALID) \
	O1P_EV(xmc_nop, CVMX_L2C_EVENT_XMC_NOP) \
	O1P_EV(xmc_ldt, CVMX_L2C_EVENT_XMC_LDT) \
	O1P_EV(xmc_ldi, CVMX_L2C_EVENT_XMC_LDI) \
	O1P_EV(xmc_ldd, CVMX_L2C_EVENT_XMC_LDD) \
	O1P_EV(xmc_stf, CVMX_L2C_EVENT_XMC_STF) \
	O1P_EV(xmc_stt, CVMX_L2C_EVENT_XMC_STT) \
	O1P_EV(xmc_stp, CVMX_L2C_EVENT_XMC_STP) \
	O1P_EV(xmc_stc, CVMX_L2C_EVENT_XMC_STC) \
	O1P_EV(xmc_dwb, CVMX_L2C_EVENT_XMC_DWB) \
	O1P_EV(xmc_pl2, CVMX_L2C_EVENT_XMC_PL2) \
	O1P_EV(xmc_psl1, CVMX_L2C_EVENT_XMC_PSL1) \
	O1P_EV(xmc_iobld, CVMX_L2C_EVENT_XMC_IOBLD) \
	O1P_EV(xmc_iobst, CVMX_L2C_EVENT_XMC_IOBST) \
	O1P_EV(xmc_iobdma, CVMX_L2C_EVENT_XMC_IOBDMA) \
	O1P_EV(xmc_iobrsp, CVMX_L2C_EVENT_XMC_IOBRSP) \
	O1P_EV(xmd_bus_valid, CVMX_L2C_EVENT_XMC_BUS_VALID) \
	O1P_EV(xmd_bus_valid_dst_l2c, CVMX_L2C_EVENT_XMC_MEM_DATA) \
	O1P_EV(xmd_bus_valid_dst_iob, CVMX_L2C_EVENT_XMC_REFL_DATA) \
	O1P_EV(xmd_bus_valid_dst_pp, CVMX_L2C_EVENT_XMC_IOBRSP_DATA) \
	O1P_EV(rsc_nop, CVMX_L2C_EVENT_RSC_NOP) \
	O1P_EV(rsc_stdn, CVMX_L2C_EVENT_RSC_STDN) \
	O1P_EV(rsc_fill, CVMX_L2C_EVENT_RSC_FILL) \
	O1P_EV(rsc_refl, CVMX_L2C_EVENT_RSC_REFL) \
	O1P_EV(rsc_stin, CVMX_L2C_EVENT_RSC_STIN) \
	O1P_EV(rsc_scin, CVMX_L2C_EVENT_RSC_SCIN) \
	O1P_EV(rsc_scfl, CVMX_L2C_EVENT_RSC_SCFL) \
	O1P_EV(rsc_scdn, CVMX_L2C_EVENT_RSC_SCDN) \
	O1P_EV(rsd_data_valid, CVMX_L2C_EVENT_RSC_DATA_VALID) \
	O1P_EV(rsd_data_valid_fill, CVMX_L2C_EVENT_RSC_VALID_FILL) \
	O1P_EV(rsd_data_valid_strsp, CVMX_L2C_EVENT_RSC_VALID_STRSP) \
	O1P_EV(rsd_data_valid_refl, CVMX_L2C_EVENT_RSC_VALID_REFL) \
	O1P_EV(lrf_req, CVMX_L2C_EVENT_LRF_REQ) \
	O1P_EV(dt_rd_alloc, CVMX_L2C_EVENT_DT_RD_ALLOC) \
	O1P_EV(dt_wr_inva, CVMX_L2C_EVENT_DT_WR_INVAL) \
	/*end*/

/*
 * o23: L2C fixed-function counters for Octeon2/3 L2-cache
 * - Values summed over N banks as 'offset' increments
 * - no _HI/_LO needed, but passed as params for symmetric macros
 */
#undef O23_EV
#define O23_EV(_name, _cvmx, _f, _lim) EV(o23, _name, EVID(o23, _name), \
	UNC_DIRECT|(_f), \
	/* _lo */ _cvmx##X_PFC(0), \
	/* _hi */ 0, \
	/* _stride */ (_cvmx##X_PFC(1) - _cvmx##X_PFC(0)), \
	_lim)
#define O23_EVENTS \
	O23_EV(add, CVMX_L2C_XMC, UNC_O23, lim_o23_xmcd_rsdc) \
	O23_EV(store, CVMX_L2C_XMD, UNC_O23, lim_o23_xmcd_rsdc) \
	O23_EV(commit, CVMX_L2C_RSC, UNC_O23, lim_o23_xmcd_rsdc) \
	O23_EV(fill, CVMX_L2C_RSD, UNC_O23, lim_o23_xmcd_rsdc) \
	O23_EV(inval, CVMX_L2C_INV, UNC_O3, lim_o3_l2c_inv) \
	O23_EV(ioc, CVMX_L2C_IOC, UNC_O23, lim_one) \
	O23_EV(ior, CVMX_L2C_IOR, UNC_O23, lim_one) \
	/*end*/

/*
 * Octeon2/3 L2-cache TAG-and-DATA counters:
 * - mapped into 4 variable-function counters spanning the 4-or-8 "quadrants"
 * - the per-quad counters should be summed over the 4-or-8 "quadrants"
 *   and for meaningful results should be sampled as perf -e group
 *   {u_tad/q0index/,u_tad/q0read,...},{u_tad/q1index/,u_tad/q1read/,...},..
 *   so event-multiplexing pulls in a coherent sample-set
 */
#undef TAD_EV
#define TAD_EV(_name, _f, _cvmx) \
	EV(tad, _name, _cvmx, UNC_MAPPED|(_f), 0, 0, 0, NULL)
#define TAD_EVENTS \
	/* omitted: TAD_EV(none, UNC_O23, CVMX_L2C_TAD_EVENT_NONE)*/ \
	TAD_EV(hit, UNC_O23, CVMX_L2C_TAD_EVENT_TAG_HIT) \
	TAD_EV(miss, UNC_O23, CVMX_L2C_TAD_EVENT_TAG_MISS) \
	TAD_EV(no_alloc, UNC_O23, CVMX_L2C_TAD_EVENT_TAG_NOALLOC) \
	TAD_EV(victim, UNC_O23, CVMX_L2C_TAD_EVENT_TAG_VICTIM) \
	TAD_EV(sc_fail, UNC_O23, CVMX_L2C_TAD_EVENT_SC_FAIL) \
	TAD_EV(sc_pass, UNC_O23, CVMX_L2C_TAD_EVENT_SC_PASS) \
	TAD_EV(lfb_valid, UNC_O23, CVMX_L2C_TAD_EVENT_LFB_VALID) \
	TAD_EV(lfb_wait_lfb, UNC_O23, CVMX_L2C_TAD_EVENT_LFB_WAIT_LFB) \
	TAD_EV(lfb_wait_vab, UNC_O23, CVMX_L2C_TAD_EVENT_LFB_WAIT_VAB) \
	TAD_EV(quad0_index, UNC_O23, CVMX_L2C_TAD_EVENT_QUAD0_INDEX) \
	TAD_EV(quad0_read, UNC_O23, CVMX_L2C_TAD_EVENT_QUAD0_READ) \
	TAD_EV(quad0_bank, UNC_O23, CVMX_L2C_TAD_EVENT_QUAD0_BANK) \
	TAD_EV(quad0_wdat, UNC_O23, CVMX_L2C_TAD_EVENT_QUAD0_WDAT) \
	TAD_EV(quad1_index, UNC_O23, CVMX_L2C_TAD_EVENT_QUAD1_INDEX) \
	TAD_EV(quad1_read, UNC_O23, CVMX_L2C_TAD_EVENT_QUAD1_READ) \
	TAD_EV(quad1_bank, UNC_O23, CVMX_L2C_TAD_EVENT_QUAD1_BANK) \
	TAD_EV(quad1_wdat, UNC_O23, CVMX_L2C_TAD_EVENT_QUAD1_WDAT) \
	TAD_EV(quad2_index, UNC_O23, CVMX_L2C_TAD_EVENT_QUAD2_INDEX) \
	TAD_EV(quad2_read, UNC_O23, CVMX_L2C_TAD_EVENT_QUAD2_READ) \
	TAD_EV(quad2_bank, UNC_O23, CVMX_L2C_TAD_EVENT_QUAD2_BANK) \
	TAD_EV(quad2_wdat, UNC_O23, CVMX_L2C_TAD_EVENT_QUAD2_WDAT) \
	TAD_EV(quad3_index, UNC_O23, CVMX_L2C_TAD_EVENT_QUAD3_INDEX) \
	TAD_EV(quad3_read, UNC_O23, CVMX_L2C_TAD_EVENT_QUAD3_READ) \
	TAD_EV(quad3_bank, UNC_O23, CVMX_L2C_TAD_EVENT_QUAD3_BANK) \
	TAD_EV(quad3_wdat, UNC_O23, CVMX_L2C_TAD_EVENT_QUAD3_WDAT) \
	TAD_EV(quad4_index, UNC_O3, CVMX_L2C_TAD_EVENT_QUAD4_INDEX) \
	TAD_EV(quad4_read, UNC_O3, CVMX_L2C_TAD_EVENT_QUAD4_READ) \
	TAD_EV(quad4_bank, UNC_O3, CVMX_L2C_TAD_EVENT_QUAD4_BANK) \
	TAD_EV(quad4_wdat, UNC_O3, CVMX_L2C_TAD_EVENT_QUAD4_WDAT) \
	TAD_EV(quad5_index, UNC_O3, CVMX_L2C_TAD_EVENT_QUAD5_INDEX) \
	TAD_EV(quad5_read, UNC_O3, CVMX_L2C_TAD_EVENT_QUAD5_READ) \
	TAD_EV(quad5_bank, UNC_O3, CVMX_L2C_TAD_EVENT_QUAD5_BANK) \
	TAD_EV(quad5_wdat, UNC_O3, CVMX_L2C_TAD_EVENT_QUAD5_WDAT) \
	TAD_EV(quad6_index, UNC_O3, CVMX_L2C_TAD_EVENT_QUAD6_INDEX) \
	TAD_EV(quad6_read, UNC_O3, CVMX_L2C_TAD_EVENT_QUAD6_READ) \
	TAD_EV(quad6_bank, UNC_O3, CVMX_L2C_TAD_EVENT_QUAD6_BANK) \
	TAD_EV(quad6_wdat, UNC_O3, CVMX_L2C_TAD_EVENT_QUAD6_WDAT) \
	TAD_EV(quad7_index, UNC_O3, CVMX_L2C_TAD_EVENT_QUAD7_INDEX) \
	TAD_EV(quad7_read, UNC_O3, CVMX_L2C_TAD_EVENT_QUAD7_READ) \
	TAD_EV(quad7_bank, UNC_O3, CVMX_L2C_TAD_EVENT_QUAD7_BANK) \
	TAD_EV(quad7_wdat, UNC_O3, CVMX_L2C_TAD_EVENT_QUAD7_WDAT) \
	/*end*/

#define OCT_EVENTS	\
	 LMC_EVENTS	\
	 O1P_EVENTS	\
	 O23_EVENTS	\
	 TAD_EVENTS	\
	/*end*/

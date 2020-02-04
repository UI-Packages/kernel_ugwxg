/* Reload some registers clobbered by trace_hardirqs_on */
#ifdef CONFIG_MICROSTATE_ACCT
#ifdef CONFIG_64BIT
# define MSA_SAVE_REGS					\
	LONG_SUB	sp, sp, 64;			\
	LONG_S	$11, 0(sp);				\
	LONG_S	$10, 8(sp);				\
	LONG_S	$9, 16(sp);				\
	LONG_S	$8, 24(sp);				\
	LONG_S	$7, 32(sp);				\
	LONG_S	$6, 40(sp);				\
	LONG_S	$5, 48(sp);				\
	LONG_S	$4, 56(sp);				\
	LONG_S	$2, 64(sp)

# define MSA_RESTORE_REGS				\
	LONG_L	$11, 0(sp);				\
	LONG_L	$10, 8(sp);				\
	LONG_L	$9, 16(sp);				\
	LONG_L	$8, 24(sp);				\
	LONG_L	$7, 32(sp);				\
	LONG_L	$6, 40(sp);				\
	LONG_L	$5, 48(sp);				\
	LONG_L	$4, 56(sp);				\
	LONG_L	$2, 64(sp);				\
	LONG_ADD	sp, sp, 64

#else
# define MSA_SAVE_REGS					\
	LONG_SUB	sp, sp, 32;			\
	LONG_S	$7, 0(sp);				\
	LONG_S	$6, 8(sp);				\
	LONG_S	$5, 16(sp);				\
	LONG_S	$4, 24(sp);				\
	LONG_S	$2, 32(sp)

# define MSA_RESTORE_REGS				\
	LONG_L	$7, 0(sp);				\
	LONG_L	$6, 8(sp);				\
	LONG_L	$5, 16(sp);				\
	LONG_L	$4, 24(sp);				\
	LONG_L	$2, 32(sp);				\
	LONG_ADD	sp, sp, 32
#endif
#endif

	.macro MSA_USER
#ifdef CONFIG_MICROSTATE_ACCT
	nop
	MSA_SAVE_REGS
	jal	msa_user
	nop
	MSA_RESTORE_REGS
	nop
#endif
	.endm

	.macro MSA_KERNEL
#ifdef CONFIG_MICROSTATE_ACCT
	nop
	MSA_SAVE_REGS
	jal	msa_kernel
	nop
	MSA_RESTORE_REGS
	nop
#endif
	.endm

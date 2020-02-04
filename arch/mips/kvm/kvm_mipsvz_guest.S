/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2013 Cavium, Inc.
 */

#include <asm/stackframe.h>

#ifdef CONFIG_SMP
	.macro	set_mips_kvm_rootsp stackp temp
	CPU_ID_MFC0	\temp, CPU_ID_REG
	LONG_SRL	\temp, PTEBASE_SHIFT
	andi	k0, CPU_ID_MASK /* high bits indicate guest mode. */
	LONG_S	\stackp, mips_kvm_rootsp(\temp)
	.endm

	.macro	get_mips_kvm_rootsp	/* SMP variation */
# if defined(CONFIG_32BIT) || defined(KBUILD_64BIT_SYM32)
	lui	k1, %hi(mips_kvm_rootsp)
# else
	lui	k1, %highest(mips_kvm_rootsp)
	daddiu	k1, %higher(mips_kvm_rootsp)
	dsll	k1, 16
	daddiu	k1, %hi(mips_kvm_rootsp)
	dsll	k1, 16
# endif
	LONG_SRL	k0, PTEBASE_SHIFT
	andi	k0, CPU_ID_MASK /* high bits indicate guest mode. */
	LONG_ADDU	k1, k0
	LONG_L	k1, %lo(mips_kvm_rootsp)(k1)
	.endm
#else /* Uniprocessor */
	.macro	set_mips_kvm_rootsp stackp temp
	LONG_S	\stackp, mips_kvm_rootsp
	.endm

	.macro	get_mips_kvm_rootsp	/* Uniprocessor variation */
# if defined(CONFIG_32BIT) || defined(KBUILD_64BIT_SYM32)
	lui	k1, %hi(mips_kvm_rootsp)
# else
	lui	k1, %highest(mips_kvm_rootsp)
	daddiu	k1, %higher(mips_kvm_rootsp)
	dsll	k1, k1, 16
	daddiu	k1, %hi(mips_kvm_rootsp)
	dsll	k1, k1, 16
# endif
	LONG_L	k1, %lo(mips_kvm_rootsp)(k1)
	.endm
#endif

#define START_GUEST_STACK_ADJUST (8 * 12)
#define VCPU_STACK_OFFSET (8 * 11)
	.set noreorder
	.p2align 5
LEAF(mipsvz_start_guest)
	daddiu	sp, sp, -START_GUEST_STACK_ADJUST
	sd	$16, (0 * 8)(sp)
	sd	$17, (1 * 8)(sp)
	sd	$18, (2 * 8)(sp)
	sd	$19, (3 * 8)(sp)
	sd	$20, (4 * 8)(sp)
	sd	$21, (5 * 8)(sp)
	sd	$22, (6 * 8)(sp)
	sd	$23, (7 * 8)(sp)
	/*	$24, t8 */
	/*	$25, t9 */
	/*	$26, K0 */
	/*	$27, K1 */
	sd	$28, (8 * 8)(sp) /* gp/current */
	/*	$29, sp */
	sd	$30, (9 * 8)(sp)
	sd	$31, (10 * 8)(sp)
	sd	a0, VCPU_STACK_OFFSET(sp)

	/* Save sp in the CPU specific slot */
	set_mips_kvm_rootsp sp, v0

	/*
	 * Move to EXL with interrupts enabled.  When we ERET to Guest
	 * mode, we can again process interrupts.
	 */
	mfc0	v0, CP0_STATUS
	ori	v0, ST0_EXL | ST0_IE
	mtc0	v0, CP0_STATUS

	/* Set GuestMode (GM) bit */
	mfc0	v1, CP0_GUESTCTL0
	ins	v1, v0, MIPS_GUESTCTL0B_GM, 1
	mtc0	v1, CP0_GUESTCTL0

	PTR_LA	v1, mipsvz_ebase_page
	LONG_L	v1, 0(v1)
	ori	v1, 1 << 11
	MTC0	v1, CP0_EBASE

	/* Load Guest register state */
	ld	v0, KVM_VCPU_ARCH_EPC(a0)
	ld	v1, KVM_VCPU_ARCH_HI(a0)
	ld	ta0, KVM_VCPU_ARCH_LO(a0)
	dmtc0	v0, CP0_EPC
	mthi	v1
	mtlo	ta0

	.set	push
	.set	noat
	ld	$1, KVM_VCPU_ARCH_R1(a0)
	ld	$2, KVM_VCPU_ARCH_R2(a0)
	ld	$3, KVM_VCPU_ARCH_R3(a0)
	ld	$5, KVM_VCPU_ARCH_R5(a0)
	ld	$6, KVM_VCPU_ARCH_R6(a0)
	ld	$7, KVM_VCPU_ARCH_R7(a0)
	ld	$8, KVM_VCPU_ARCH_R8(a0)
	ld	$9, KVM_VCPU_ARCH_R9(a0)
	ld	$10, KVM_VCPU_ARCH_R10(a0)
	ld	$11, KVM_VCPU_ARCH_R11(a0)
	ld	$12, KVM_VCPU_ARCH_R12(a0)
	ld	$13, KVM_VCPU_ARCH_R13(a0)
	ld	$14, KVM_VCPU_ARCH_R14(a0)
	ld	$15, KVM_VCPU_ARCH_R15(a0)
	ld	$16, KVM_VCPU_ARCH_R16(a0)
	ld	$17, KVM_VCPU_ARCH_R17(a0)
	ld	$18, KVM_VCPU_ARCH_R18(a0)
	ld	$19, KVM_VCPU_ARCH_R19(a0)
	ld	$20, KVM_VCPU_ARCH_R20(a0)
	ld	$21, KVM_VCPU_ARCH_R21(a0)
	ld	$22, KVM_VCPU_ARCH_R22(a0)
	ld	$23, KVM_VCPU_ARCH_R23(a0)
	ld	$24, KVM_VCPU_ARCH_R24(a0)
	ld	$25, KVM_VCPU_ARCH_R25(a0)
	ld	$26, KVM_VCPU_ARCH_R26(a0)
	ld	$27, KVM_VCPU_ARCH_R27(a0)
	ld	$28, KVM_VCPU_ARCH_R28(a0)
	ld	$29, KVM_VCPU_ARCH_R29(a0)
	ld	$30, KVM_VCPU_ARCH_R30(a0)
	ld	$31, KVM_VCPU_ARCH_R31(a0)
	ld	$4, KVM_VCPU_ARCH_R4(a0) /* $4 == a0, do it last. */
#ifdef CONFIG_CAVIUM_ERET_MISPREDICT_WORKAROUND
	dmtc0	$0, $30,7		/* force misprediction for ERET */
#endif
	eret
	.set	pop

	.p2align 7
.Lmipsvz_exit_guest:
FEXPORT(mipsvz_exit_guest)

	/* Clear sp in the CPU specific slot */
	CPU_ID_MFC0	k0, CPU_ID_REG
	get_mips_kvm_rootsp
	move	sp, k1
	set_mips_kvm_rootsp zero, v0

	ld	$16, (0 * 8)(sp)
	ld	$17, (1 * 8)(sp)
	ld	$18, (2 * 8)(sp)
	ld	$19, (3 * 8)(sp)
	ld	$20, (4 * 8)(sp)
	ld	$21, (5 * 8)(sp)
	ld	$22, (6 * 8)(sp)
	ld	$23, (7 * 8)(sp)
	/*	$24, t8 */
	/*	$25, t9 */
	/*	$26, K0 */
	/*	$27, K1 */
	ld	$28, (8 * 8)(sp) /* gp/current */
	/*	$29, sp */
	ld	$30, (9 * 8)(sp)
	ld	$31, (10 * 8)(sp)

	jr	ra
	 daddiu	sp, sp, START_GUEST_STACK_ADJUST
	END(mipsvz_start_guest)

	.p2align 5
	.set mips64r2

LEAF(mipsvz_install_fpu)
	ldc1	$f0, (KVM_VCPU_ARCH_FPR + (0 * 8))(a0)
	ldc1	$f1, (KVM_VCPU_ARCH_FPR + (1 * 8))(a0)
	ldc1	$f2, (KVM_VCPU_ARCH_FPR + (2 * 8))(a0)
	ldc1	$f3, (KVM_VCPU_ARCH_FPR + (3 * 8))(a0)
	ldc1	$f4, (KVM_VCPU_ARCH_FPR + (4 * 8))(a0)
	ldc1	$f5, (KVM_VCPU_ARCH_FPR + (5 * 8))(a0)
	ldc1	$f6, (KVM_VCPU_ARCH_FPR + (6 * 8))(a0)
	ldc1	$f7, (KVM_VCPU_ARCH_FPR + (7 * 8))(a0)
	ldc1	$f8, (KVM_VCPU_ARCH_FPR + (8 * 8))(a0)
	ldc1	$f9, (KVM_VCPU_ARCH_FPR + (9 * 8))(a0)
	ldc1	$f10, (KVM_VCPU_ARCH_FPR + (10 * 8))(a0)
	ldc1	$f11, (KVM_VCPU_ARCH_FPR + (11 * 8))(a0)
	ldc1	$f12, (KVM_VCPU_ARCH_FPR + (12 * 8))(a0)
	ldc1	$f13, (KVM_VCPU_ARCH_FPR + (13 * 8))(a0)
	ldc1	$f14, (KVM_VCPU_ARCH_FPR + (14 * 8))(a0)
	ldc1	$f15, (KVM_VCPU_ARCH_FPR + (15 * 8))(a0)
	ldc1	$f16, (KVM_VCPU_ARCH_FPR + (16 * 8))(a0)
	ldc1	$f17, (KVM_VCPU_ARCH_FPR + (17 * 8))(a0)
	ldc1	$f18, (KVM_VCPU_ARCH_FPR + (18 * 8))(a0)
	ldc1	$f19, (KVM_VCPU_ARCH_FPR + (19 * 8))(a0)
	ldc1	$f20, (KVM_VCPU_ARCH_FPR + (20 * 8))(a0)
	ldc1	$f21, (KVM_VCPU_ARCH_FPR + (21 * 8))(a0)
	ldc1	$f22, (KVM_VCPU_ARCH_FPR + (22 * 8))(a0)
	ldc1	$f23, (KVM_VCPU_ARCH_FPR + (23 * 8))(a0)
	ldc1	$f24, (KVM_VCPU_ARCH_FPR + (24 * 8))(a0)
	ldc1	$f25, (KVM_VCPU_ARCH_FPR + (25 * 8))(a0)
	ldc1	$f26, (KVM_VCPU_ARCH_FPR + (26 * 8))(a0)
	ldc1	$f27, (KVM_VCPU_ARCH_FPR + (27 * 8))(a0)
	ldc1	$f28, (KVM_VCPU_ARCH_FPR + (28 * 8))(a0)
	ldc1	$f29, (KVM_VCPU_ARCH_FPR + (29 * 8))(a0)
	ldc1	$f30, (KVM_VCPU_ARCH_FPR + (30 * 8))(a0)
	ldc1	$f31, (KVM_VCPU_ARCH_FPR + (31 * 8))(a0)

	lw	t0, KVM_VCPU_ARCH_FCSR(a0)
	ctc1	t0, $31

	lw	t0, KVM_VCPU_ARCH_FENR(a0)
	ctc1	t0, $28

	lw	t0, KVM_VCPU_ARCH_FEXR(a0)
	ctc1	t0, $26

	lw	t0, KVM_VCPU_ARCH_FCCR(a0)

	jr	ra
	 ctc1	t0, $25

	END(mipsvz_install_fpu)

LEAF(mipsvz_readout_fpu)
	sdc1	$f0, (KVM_VCPU_ARCH_FPR + (0 * 8))(a0)
	sdc1	$f1, (KVM_VCPU_ARCH_FPR + (1 * 8))(a0)
	sdc1	$f2, (KVM_VCPU_ARCH_FPR + (2 * 8))(a0)
	sdc1	$f3, (KVM_VCPU_ARCH_FPR + (3 * 8))(a0)
	sdc1	$f4, (KVM_VCPU_ARCH_FPR + (4 * 8))(a0)
	sdc1	$f5, (KVM_VCPU_ARCH_FPR + (5 * 8))(a0)
	sdc1	$f6, (KVM_VCPU_ARCH_FPR + (6 * 8))(a0)
	sdc1	$f7, (KVM_VCPU_ARCH_FPR + (7 * 8))(a0)
	sdc1	$f8, (KVM_VCPU_ARCH_FPR + (8 * 8))(a0)
	sdc1	$f9, (KVM_VCPU_ARCH_FPR + (9 * 8))(a0)
	sdc1	$f10, (KVM_VCPU_ARCH_FPR + (10 * 8))(a0)
	sdc1	$f11, (KVM_VCPU_ARCH_FPR + (11 * 8))(a0)
	sdc1	$f12, (KVM_VCPU_ARCH_FPR + (12 * 8))(a0)
	sdc1	$f13, (KVM_VCPU_ARCH_FPR + (13 * 8))(a0)
	sdc1	$f14, (KVM_VCPU_ARCH_FPR + (14 * 8))(a0)
	sdc1	$f15, (KVM_VCPU_ARCH_FPR + (15 * 8))(a0)
	sdc1	$f16, (KVM_VCPU_ARCH_FPR + (16 * 8))(a0)
	sdc1	$f17, (KVM_VCPU_ARCH_FPR + (17 * 8))(a0)
	sdc1	$f18, (KVM_VCPU_ARCH_FPR + (18 * 8))(a0)
	sdc1	$f19, (KVM_VCPU_ARCH_FPR + (19 * 8))(a0)
	sdc1	$f20, (KVM_VCPU_ARCH_FPR + (20 * 8))(a0)
	sdc1	$f21, (KVM_VCPU_ARCH_FPR + (21 * 8))(a0)
	sdc1	$f22, (KVM_VCPU_ARCH_FPR + (22 * 8))(a0)
	sdc1	$f23, (KVM_VCPU_ARCH_FPR + (23 * 8))(a0)
	sdc1	$f24, (KVM_VCPU_ARCH_FPR + (24 * 8))(a0)
	sdc1	$f25, (KVM_VCPU_ARCH_FPR + (25 * 8))(a0)
	sdc1	$f26, (KVM_VCPU_ARCH_FPR + (26 * 8))(a0)
	sdc1	$f27, (KVM_VCPU_ARCH_FPR + (27 * 8))(a0)
	sdc1	$f28, (KVM_VCPU_ARCH_FPR + (28 * 8))(a0)
	sdc1	$f29, (KVM_VCPU_ARCH_FPR + (29 * 8))(a0)
	sdc1	$f30, (KVM_VCPU_ARCH_FPR + (30 * 8))(a0)
	sdc1	$f31, (KVM_VCPU_ARCH_FPR + (31 * 8))(a0)

	cfc1	t0, $31
	sw	t0, KVM_VCPU_ARCH_FCSR(a0)

	cfc1	t0, $28
	sw	t0, KVM_VCPU_ARCH_FENR(a0)

	cfc1	t0, $26
	sw	t0, KVM_VCPU_ARCH_FEXR(a0)

	cfc1	t0, $25
	sw	t0, KVM_VCPU_ARCH_FCCR(a0)

	cfc1	t0, $0

	jr	ra
	 sw	t0, KVM_VCPU_ARCH_FIR(a0)

	END(mipsvz_readout_fpu)

	.macro	mipsvz_chain_header target
	MTC0	k0, CP0_KSCRATCH1
	MTC0	k1, CP0_KSCRATCH2
	CPU_ID_MFC0	k0, CPU_ID_REG
	get_mips_kvm_rootsp
	PTR_SUBU k1, KVM_MIPS_VZ_REGS_SIZE
	LONG_S	sp, PT_R29(k1)
	move	sp, k1
	LONG_S	$1, PT_R1(sp)
	LONG_S	$2, PT_R2(sp)
	MFC0	v0, CP0_EPC
	LONG_S	$3, PT_R3(sp)
	LONG_S	$4, PT_R4(sp)
	LONG_S	$5, PT_R5(sp)
	LONG_S	$6, PT_R6(sp)
	LONG_S	v0, PT_EPC(sp)
	LONG_S	$16, PT_R16(sp)
	PTR_LI	$16, \target
	PTR_LA	v1, mipsvz_common_chain
	jr	v1
	 LONG_S	$7, PT_R7(sp)
	.endm


LEAF(mipsvz_interrupt_chain)
	.set	push
	.set	noat
	mipsvz_chain_header 0x200
	.set	pop
.Lmipsvz_interrupt_chain_end:
FEXPORT(mipsvz_interrupt_chain_end)
	END(mipsvz_interrupt_chain)

LEAF(mipsvz_general_chain)
	.set	push
	.set	noat
	mipsvz_chain_header 0
	.set	pop
.Lmipsvz_general_chain_end:
FEXPORT(mipsvz_general_chain_end)
	END(mipsvz_general_chain)


LEAF(mipsvz_common_chain)
	.set	push
	.set	noat
	mfhi	v0
	LONG_S	$8, PT_R8(sp)
	LONG_S	$9, PT_R9(sp)
	LONG_S	$10, PT_R10(sp)
	LONG_S	$11, PT_R11(sp)
	LONG_S	v0, PT_HI(sp)
	mflo	v0
	LONG_S	$12, PT_R12(sp)
	LONG_S	$13, PT_R13(sp)
	LONG_S	$17, PT_R17(sp)
	LONG_S	$18, PT_R18(sp)
	LONG_S	v0, PT_LO(sp)
	mfc0	$17, CP0_STATUS /* Status in $17 */
	LONG_S	$14, PT_R14(sp)
	LONG_S	$15, PT_R15(sp)
	LONG_S	$19, PT_R19(sp)
	LONG_S	$17, PT_STATUS(sp)
	MFC0	k0, CP0_KSCRATCH1
	LONG_S	$20, PT_R20(sp)
	LONG_S	$21, PT_R21(sp)
	LONG_S	$22, PT_R22(sp)
	MFC0	k1, CP0_KSCRATCH2
	LONG_S	$23, PT_R23(sp)
	LONG_S	$24, PT_R24(sp)
	LONG_S	$25, PT_R25(sp)
	mfc0	$18, CP0_CAUSE /* Cause in $18 */

	LONG_S	k0, PT_R26(sp)
	LONG_S	k1, PT_R27(sp)

	LONG_S	$28, PT_R28(sp)
	LONG_S	$18, PT_CAUSE(sp)
	MFC0	v0, CP0_BADVADDR
	LONG_S	$30, PT_R30(sp)
	LONG_S	$31, PT_R31(sp)

	ori	$28, sp, _THREAD_MASK
	LONG_S	v0, PT_BVADDR(sp)
	xori	$28, _THREAD_MASK

	mfc0	v0, CP0_GUESTCTL0
	ins	v0, zero, MIPS_GUESTCTL0B_GM, 1
	mtc0	v0, CP0_GUESTCTL0

	PTR_L	a0, (VCPU_STACK_OFFSET + KVM_MIPS_VZ_REGS_SIZE)(sp)
	PTR_L	v0, TI_TASK($28)
	PTR_L	a0, KVM_VCPU_ARCH_IMPL(a0)
	lw	v1, KVM_MIPS_VCPU_VZ_MM_ASID(a0)
	dmtc0	v1, CP0_ENTRYHI
	LONG_L	v1, TASK_MM(v0)

	jal	tlbmiss_handler_setup_pgd_array
	 LONG_L	a0, MM_PGD(v1)
#ifdef CONFIG_CPU_CAVIUM_OCTEON
	/*
	* The Octeon multiplier state is affected by general
	* multiply instructions. It must be saved before and
	* kernel code might corrupt it
	*/
	jal     octeon_mult_save
	 nop
#endif

	PTR_LA	v1, ebase
	LONG_L	v1, 0(v1)
	ori	v1, 1 << 11
	MTC0	v1, CP0_EBASE
	beqz	$16, try_general
	 xori	v1, 1 << 11

chain_to_root:
	PTR_LA	v0, resume_intercept
	MTC0	v0, CP0_EPC
	PTR_ADDU v1, v1, $16
	jr	v1
	 nop

try_general:
	andi	a0, $18, 0x7c
	beqz	a0, chain_to_root
	 li	$16, 0x180
call_handler:
#ifdef CONFIG_64BIT
	SLL	a0, 1
#endif
	PTR_LA	v0, vz_ex_handlers
	PTR_ADDU v0, v0, a0
	PTR_L	v0, 0(v0)
	ori	$17, ST0_EXL | ST0_IE
	xori	$17, ST0_EXL | ST0_IE
	move	a1, sp
	PTR_L	a0, (VCPU_STACK_OFFSET + KVM_MIPS_VZ_REGS_SIZE)(sp)
	jalr	v0
	 mtc0	$17, CP0_STATUS
resume_intercept:
	LONG_L	t0, TI_FLAGS($28)
	ori	$17, ST0_EXL | ST0_IE
	xori	$17, ST0_EXL
	andi	t0, (_TIF_NEED_RESCHED | _TIF_SIGPENDING)
	beqz	t0, resume_intercept0
	 nop
	move	a1, sp
	PTR_L	a0, (VCPU_STACK_OFFSET + KVM_MIPS_VZ_REGS_SIZE)(sp)
	jal	mipsvz_do_resched
	 mtc0	$17, CP0_STATUS
resume_intercept0:
	LONG_L	v1, PT_STATUS(sp)
	mtc0	v1, CP0_STATUS
	PTR_LA	v1, mipsvz_ebase_page
	LONG_L	v1, 0(v1)
	ori	v1, 1 << 11
	MTC0	v1, CP0_EBASE

#ifdef CONFIG_CPU_CAVIUM_OCTEON
	jal	octeon_mult_restore
	 nop
#endif

	LONG_L	v1, PT_LO(sp)
	mtlo	v1
	LONG_L	v1, PT_HI(sp)
	mthi	v1

	/* Must set GuestCtl0[GM] */
	PTR_LI	v0, 1
	mfc0	v1, CP0_GUESTCTL0
	PTR_L	a0, (VCPU_STACK_OFFSET + KVM_MIPS_VZ_REGS_SIZE)(sp) /* a0 <- vcpu */
	ins	v1, v0, MIPS_GUESTCTL0B_GM, 1
	PTR_L	a1, KVM_VCPU_ARCH_IMPL(a0) /* a1 <- vcpu->arch.impl */
	mtc0	v1, CP0_GUESTCTL0

	lw	v1, KVM_MIPS_VCPU_VZ_GUEST_ASID(a1)
	lbu	v0, KVM_MIPS_VCPU_VZ_INJECTED_IPX(a1)
	dmtc0	v1, CP0_ENTRYHI
	PTR_L	v1, KVM_VCPU_KVM(a0) /* v1 <- kvm */
	/* Inject any interrupts that may have been requested. */
	mfc0	k1, CP0_GUESTCTL2
	PTR_L	v1, KVM_ARCH_IMPL(v1) /* v1 <- kvm->arch.impl */
	ins	k1, v0, 8, 8
	mtc0	k1, CP0_GUESTCTL2

	jal	tlbmiss_handler_setup_pgd_array
	 LONG_L	a0, KVM_MIPS_VZ_PGD(v1)

	LONG_L	v0, PT_EPC(sp)

	LONG_L	$31, PT_R31(sp)
	LONG_L	$30, PT_R30(sp)
	LONG_L	$28, PT_R28(sp)
	MTC0	v0, CP0_EPC
	LONG_L	$27, PT_R27(sp)
	LONG_L	$26, PT_R26(sp)
	LONG_L	$25, PT_R25(sp)
	LONG_L	$24, PT_R24(sp)
	LONG_L	$23, PT_R23(sp)
	LONG_L	$22, PT_R22(sp)
	LONG_L	$21, PT_R21(sp)
	LONG_L	$20, PT_R20(sp)
	LONG_L	$19, PT_R19(sp)
	LONG_L	$18, PT_R18(sp)
	LONG_L	$17, PT_R17(sp)
	LONG_L	$16, PT_R16(sp)
	LONG_L	$15, PT_R15(sp)
	LONG_L	$14, PT_R14(sp)
	LONG_L	$13, PT_R13(sp)
	LONG_L	$12, PT_R12(sp)
	LONG_L	$11, PT_R11(sp)
	LONG_L	$10, PT_R10(sp)
	LONG_L	$9, PT_R9(sp)
	LONG_L	$8, PT_R8(sp)
	LONG_L	$7, PT_R7(sp)
	LONG_L	$6, PT_R6(sp)
	LONG_L	$5, PT_R5(sp)
	LONG_L	$4, PT_R4(sp)
	LONG_L	$3, PT_R3(sp)
	LONG_L	$2, PT_R2(sp)
	LONG_L	$1, PT_R1(sp)
	LONG_L	sp, PT_R29(sp)
#ifdef CONFIG_CAVIUM_ERET_MISPREDICT_WORKAROUND
	dmtc0	$0, $30, 7		/* force misprediction for ERET */
#endif
	eret
	.set	pop
	END(mipsvz_common_chain)

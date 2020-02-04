/*
 * Based on arch/arm/kernel/insn.c
 *
 * Copyright (C) 2013 Cavium Inc
 * Author: Ganapatrao Kulkarni <ganapatrao.kulkarni@cavium.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/bug.h>
#include <linux/kernel.h>

static unsigned int
__arm_gen_branch_arm(unsigned long pc, unsigned long addr, bool link)
{
	unsigned int opcode = 0x14000000;
	int offset;

	if (link)
		opcode |= 1 << 31;

	offset = (long)addr - (long)(pc);
	/* bl label on arm64 is of width 26 bits/imm26 */
	if (unlikely(offset < -67108863 || offset > 67108859)) {
		WARN_ON_ONCE(1);
		return 0;
	}

	offset = (offset >> 2) & 0x03ffffff;  /* imm26 */
	return opcode | offset;
}

unsigned int
__arm_gen_branch(unsigned long pc, unsigned long addr, bool link)
{
		return __arm_gen_branch_arm(pc, addr, link);
}

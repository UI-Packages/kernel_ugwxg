/*
 * Based on arch/arm/include/asm/ftrace.h
 *
 * Copyright (C) 2008 Abhishek Sagar <sagar.abhishek@gmail.com>
 * Copyright (C) 2010 Rabin Vincent <rabin@rab.in>
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
#ifndef _ASM_ARM_FTRACE
#define _ASM_ARM_FTRACE

#ifdef CONFIG_FUNCTION_TRACER
#define MCOUNT_ADDR		((unsigned long) _mcount)
#define MCOUNT_INSN_SIZE	4 /* sizeof mcount call */
#define	ARM64_NOP_OPCODE	0xd503201f	/* nop */

#ifndef __ASSEMBLY__
extern void _mcount(void);

#ifdef CONFIG_DYNAMIC_FTRACE

extern unsigned long ftrace_graph_call;
extern void ftrace_graph_caller(void);

struct dyn_arch_ftrace {
		/* No extra data needed */
};

static inline unsigned long ftrace_call_adjust(unsigned long addr)
{
	return addr;
}

#endif /* ifdef CONFIG_DYNAMIC_FTRACE */
#endif /* ifndef__ASSEMBLY__ */
#endif /* ifdef CONFIG_FUNCTION_TRACER */

#ifndef __ASSEMBLY__

#ifdef CONFIG_FRAME_POINTER
/*
 * return_address uses walk_stackframe to do it's work.  If both
 * CONFIG_FRAME_POINTER=y and CONFIG_ARM_UNWIND=y walk_stackframe uses unwind
 * information.  For this to work in the function tracer many functions would
 * have to be marked with __notrace.  So for now just depend on
 * !CONFIG_ARM_UNWIND.
 */

void *return_address(unsigned int);

#else /* #ifdef CONFIG_FRAME_POINTER */

extern inline void *return_address(unsigned int level)
{
	return NULL;
}

#endif /* #ifdef CONFIG_FRAME_POINTER */

#define HAVE_ARCH_CALLER_ADDR

#define CALLER_ADDR0 ((unsigned long)__builtin_return_address(0))
#define CALLER_ADDR1 ((unsigned long)return_address(1))
#define CALLER_ADDR2 ((unsigned long)return_address(2))
#define CALLER_ADDR3 ((unsigned long)return_address(3))
#define CALLER_ADDR4 ((unsigned long)return_address(4))
#define CALLER_ADDR5 ((unsigned long)return_address(5))
#define CALLER_ADDR6 ((unsigned long)return_address(6))

#endif /* ifndef __ASSEMBLY__ */

#endif /* _ASM_ARM_FTRACE */

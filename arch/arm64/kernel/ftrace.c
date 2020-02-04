/*
 * Dynamic and Graph function tracing support.
 *
 * Based on arch/arm/kernel/ftrace.c
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
 * Defines low-level handling of mcount calls when the kernel
 * is compiled with the -pg flag. When using dynamic ftrace, the
 * mcount call-sites get patched with NOP till they are enabled.
 * All code mutation routines here are called under stop_machine().
 */

#include <linux/ftrace.h>
#include <linux/uaccess.h>

#include <asm/cacheflush.h>
#include <asm/ftrace.h>

#include "insn.h"

#ifdef CONFIG_DYNAMIC_FTRACE

static unsigned long ftrace_nop_replace(struct dyn_ftrace *rec)
{
	return ARM64_NOP_OPCODE;
}

static unsigned long ftrace_call_replace(unsigned long pc, unsigned long addr)
{
	return arm_gen_branch_link(pc, addr);
}

static int ftrace_modify_code(unsigned long pc, unsigned int old,
			      unsigned int new, bool validate)
{
	unsigned int replaced;

#ifdef CONFIG_CPU_BIG_ENDIAN
	old = swab32(old);
	new = swab32(new);
#endif

	if (validate) {
		if (probe_kernel_read(&replaced, (void *)pc, MCOUNT_INSN_SIZE))
			return -EFAULT;

		if (replaced != old)
			return -EINVAL;
	}

	pax_open_kernel();
	if (probe_kernel_write((void *)pc, &new, MCOUNT_INSN_SIZE)) {
		pax_close_kernel();
		return -EPERM;
	}
	pax_close_kernel();

	flush_icache_range(pc, pc + MCOUNT_INSN_SIZE);

	return 0;
}

int ftrace_update_ftrace_func(ftrace_func_t func)
{
	unsigned long pc;
	unsigned int new;
	int ret = 0;

	pc = (unsigned long)&ftrace_call;
	new = ftrace_call_replace(pc, (unsigned long)func);
	ret = ftrace_modify_code(pc, 0, new, false);

	return ret;
}

int ftrace_make_call(struct dyn_ftrace *rec, unsigned long addr)
{
	unsigned int new, old;
	unsigned long ip = rec->ip;
	int ret;

	old = ftrace_nop_replace(rec);
	new = ftrace_call_replace(ip, addr);
	ret = ftrace_modify_code(rec->ip, old, new, true);

	return ret;
}

int ftrace_make_nop(struct module *mod,
		    struct dyn_ftrace *rec, unsigned long addr)
{
	unsigned long ip = rec->ip;
	unsigned int old;
	unsigned int new;
	int ret;

	old = ftrace_call_replace(ip, addr);
	new = ftrace_nop_replace(rec);
	ret = ftrace_modify_code(ip, old, new, true);

	return ret;
}

int __init ftrace_dyn_arch_init(void *data)
{
	*(unsigned long *)data = 0;

	return 0;
}
#endif /* CONFIG_DYNAMIC_FTRACE */

#ifdef CONFIG_FUNCTION_GRAPH_TRACER
void prepare_ftrace_return(unsigned long *parent, unsigned long self_addr,
			   unsigned long frame_pointer)
{
	unsigned long return_hooker = (unsigned long) &return_to_handler;
	struct ftrace_graph_ent trace;
	unsigned long old;
	int err;

	if (unlikely(atomic_read(&current->tracing_graph_pause)))
		return;

	old = *parent;
	*parent = return_hooker;

	trace.func = self_addr;
	trace.depth = current->curr_ret_stack + 1;

	/* Only trace if the calling function expects to */
	if (!ftrace_graph_entry(&trace)) {
		*parent = old;
		return;
	}

	err = ftrace_push_return_trace(old, self_addr, &trace.depth,
				       frame_pointer);
	if (err == -EBUSY) {
		*parent = old;
		return;
	}
}

#ifdef CONFIG_DYNAMIC_FTRACE
static int __ftrace_modify_caller(unsigned long *callsite,
				  void (*func) (void), bool enable)
{
	unsigned long caller_fn = (unsigned long) func;
	unsigned long pc = (unsigned long) callsite;
	unsigned int  branch = arm_gen_branch(pc, caller_fn);
	unsigned int nop = ARM64_NOP_OPCODE;
	unsigned int old = enable ? nop : branch;
	unsigned int new = enable ? branch : nop;

	return ftrace_modify_code(pc, old, new, true);
}

static int ftrace_modify_graph_caller(bool enable)
{
	int ret;

	ret = __ftrace_modify_caller(&ftrace_graph_call,
					     ftrace_graph_caller,
					     enable);

	return ret;
}

int ftrace_enable_ftrace_graph_caller(void)
{
	return ftrace_modify_graph_caller(true);
}

int ftrace_disable_ftrace_graph_caller(void)
{
	return ftrace_modify_graph_caller(false);
}
#endif /* CONFIG_DYNAMIC_FTRACE */
#endif /* CONFIG_FUNCTION_GRAPH_TRACER */

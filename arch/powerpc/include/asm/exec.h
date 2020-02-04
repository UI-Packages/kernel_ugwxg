/*
 * Copyright (C) 1999 Cort Dougan <cort@cs.nmt.edu>
 */
#ifndef _ASM_POWERPC_EXEC_H
#define _ASM_POWERPC_EXEC_H

#ifdef CONFIG_PAX_ASLR
#define arch_align_stack(x) ((x) & ~0xfUL)
#else
extern unsigned long arch_align_stack(unsigned long sp);
#endif

#endif /* _ASM_POWERPC_EXEC_H */

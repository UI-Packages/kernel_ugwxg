/*
 * Copyright IBM Corp. 1999, 2009
 *
 * Author(s): Martin Schwidefsky <schwidefsky@de.ibm.com>
 */

#ifndef __ASM_EXEC_H
#define __ASM_EXEC_H

#ifdef CONFIG_PAX_ASLR
#define arch_align_stack(x) ((x) & ~0xfUL)
#else
extern unsigned long arch_align_stack(unsigned long sp);
#endif

#endif /* __ASM_EXEC_H */

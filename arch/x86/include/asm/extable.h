#ifndef _ASM_X86_EXTABLE_H
#define _ASM_X86_EXTABLE_H

/*
 * The exception table consists of pairs of addresses relative to the
 * exception table enty itself: the first is the address of an
 * instruction that is allowed to fault, and the second is the address
 * at which the program should continue.  No registers are modified,
 * so it is entirely up to the continuation code to figure out what to
 * do.
 *
 * All the routines below use bits of fixup code that are out of line
 * with the main instruction path.  This means when everything is well,
 * we don't even have to jump over them.  Further, they do not intrude
 * on our cache or tlb entries.
 */

struct exception_table_entry {
	int insn, fixup;
};
/* This is not the generic standard exception_table_entry format */
#define ARCH_HAS_SORT_EXTABLE
#define ARCH_HAS_SEARCH_EXTABLE

#endif /* _ASM_X86_EXTABLE_H */

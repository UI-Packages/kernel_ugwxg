/*
 * Copyright (C) 2012 ARM Ltd.
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
 */

#include <linux/linkage.h>
#include <linux/const.h>
#include <asm/assembler.h>
#include <asm/page.h>

/*
 * Copy a page from src to dest (both are page aligned)
 *
 * Parameters:
 *	x0 - dest
 *	x1 - src
 */
ENTRY(copy_page)
	/* Special case 128 byte or more cache lines */
	mrs	x2, dczid_el0
	and	w2, w2, #0xf
	cmp	w2, 5
	b.ge    2f

	/* Assume cache line size is 64 bytes. */
	prfm	pldl1strm, [x1, #64]
1:	ldp	x2, x3, [x1]
	ldp	x4, x5, [x1, #16]
	ldp	x6, x7, [x1, #32]
	ldp	x8, x9, [x1, #48]
	add	x1, x1, #64
	prfm	pldl1strm, [x1, #64]
	stnp	x2, x3, [x0]
	stnp	x4, x5, [x0, #16]
	stnp	x6, x7, [x0, #32]
	stnp	x8, x9, [x0, #48]
	add	x0, x0, #64
	tst     x1, #(PAGE_SIZE - 1)
	b.ne	1b
	ret
2:
	/* The cache line size is at least 128 bytes. */
	prfm	pldl1strm, [x1, #128]
1:	prfm	pldl1strm, [x1, #256]
	ldp	x2, x3, [x1]
	ldp	x4, x5, [x1, #16]
	ldp	x6, x7, [x1, #32]
	ldp	x8, x9, [x1, #48]
	stnp	x2, x3, [x0]
	stnp	x4, x5, [x0, #16]
	stnp	x6, x7, [x0, #32]
	stnp	x8, x9, [x0, #48]

	ldp	x2, x3, [x1, #64]
	ldp	x4, x5, [x1, #80]
	ldp	x6, x7, [x1, #96]
	ldp	x8, x9, [x1, #112]
	add	x1, x1, #128
	stnp	x2, x3, [x0, #64]
	stnp	x4, x5, [x0, #80]
	stnp	x6, x7, [x0, #96]
	stnp	x8, x9, [x0, #112]
	add	x0, x0, #128
	tst	x1, #(PAGE_SIZE - 1)
	b.ne	1b
	ret
ENDPROC(copy_page)

/*
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 *
 * Copyright (C) 2012-2013 Cavium Inc., All Rights Reserved.
 */
#ifndef __ASM_OCTEON_OCTEON_CRYPTO_H
#define __ASM_OCTEON_OCTEON_CRYPTO_H

struct octeon_cop2_state;

/* Assembly context-switch functions */
extern void octeon_cop2_save(struct octeon_cop2_state *);
extern void octeon_cop2_restore(struct octeon_cop2_state *);

/* Exported entry points */
extern unsigned long octeon_crypto_enable(struct octeon_cop2_state *state);
extern void octeon_crypto_disable(struct octeon_cop2_state *state,
				  unsigned long flags);

#endif /* __ASM_OCTEON_OCTEON_CRYPTO_H */

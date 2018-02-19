/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <plat_marvell.h>
#include <plat_private.h>

#include <ap810_setup.h>
#include <cp110_setup.h>
#include <marvell_pm.h>

/* Initialize the CP110 in all APs */
static void cp110_die_init(void)
{
	int ap_id, cp_id;

	for (ap_id = 0; ap_id < get_ap_count(); ap_id++)
		for (cp_id = 0; cp_id < get_connected_cp_per_ap(ap_id); cp_id++)
			cp110_init(MVEBU_CP_REGS_BASE(ap_id, cp_id));
}

/* This function overruns the same function in marvell_bl31_setup.c */
void bl31_plat_arch_setup(void)
{
	uintptr_t *mailbox = (void *)PLAT_MARVELL_MAILBOX_BASE;

	/* initiliaze the timer for mdelay/udelay functionality */
	plat_delay_timer_init();

	/* Configure ap810 */
	ap810_init();

	/* Configure the connected CP110 (if any) */
	cp110_die_init();

	/* In marvell_bl31_plat_arch_setup, el3 mmu is configured.
	 * el3 mmu configuration MUST be called after ap810_init, if not,
	 * this will cause an hang in init_io_win
	 * (after setting the IO windows GCR values).
	 */
	if (mailbox[MBOX_IDX_MAGIC] != MVEBU_MAILBOX_MAGIC_NUM ||
	    mailbox[MBOX_IDX_SUSPEND_MAGIC] != MVEBU_MAILBOX_SUSPEND_STATE)
		marvell_bl31_plat_arch_setup();
}

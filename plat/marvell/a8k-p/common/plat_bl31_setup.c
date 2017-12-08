/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <plat_marvell.h>
#include <plat_config.h>
#include <plat_private.h>
#include <ap810_setup.h>
#include <marvell_pm.h>
#include <mvebu.h>
#include <mmio.h>
#include <debug.h>

/* CP110 has configuration space address set by the default to 0xf200_0000
** In Armada-8k-plus family there is an option to connect more than
** a single CP110 to AP810.
** Need to update the configuration space according to the address map of
** Armada-8k-plus family.
** This function opens a temporary windows in GWIN/CCU/IO-WIN to access different
** CPs, changes the configuration space of every CP & closes the temporary windows
** */
static void update_cp110_default_win(void)
{
	int ap_id, cp_id, mci_id;
	uintptr_t cp110_base, cp110_temp_base;
	struct addr_map_win gwin_temp_win, ccu_temp_win, iowin_temp_win;

	debug_enter();

	/* CP110 default configuration address space */
	cp110_temp_base = MVEBU_CP_DEFAULT_BASE_ADDR;

	/* Prepare GWIN/CCU/IOWIN windows */
	gwin_temp_win.base_addr = cp110_temp_base;
	gwin_temp_win.win_size = MVEBU_CP_DEFAULT_BASE_SIZE;

	ccu_temp_win.base_addr = cp110_temp_base;
	ccu_temp_win.win_size = MVEBU_CP_DEFAULT_BASE_SIZE;
	/* Target of the CCU to access CP should be IO */
	ccu_temp_win.target_id = IO_0_TID;

	iowin_temp_win.base_addr = cp110_temp_base;
	iowin_temp_win.win_size = MVEBU_CP_DEFAULT_BASE_SIZE;

	/* Go over the APs and update every CP with
	** the new configuration address
	** */
	for (ap_id = 0; ap_id < get_ap_count(); ap_id++) {
		/* If initialize the MCIs in another APs, need to open
		** window in GWIN to enable access from AP0 to APx
		** */
		if (ap_id > 0) {
			gwin_temp_win.target_id = ap_id;
			/* All the initialization will be done from AP0,
			** these GWIN settings allow the control to reach
			** remote APs.
			** */
			gwin_temp_win_insert(0, &gwin_temp_win, 1);
		}

		/* Open temp window in CCU unit */
		ccu_temp_win_insert(ap_id, &ccu_temp_win, 1);

		/* Go over the connected CPx in the APx */
		for (cp_id = 0; cp_id < get_connected_cp_per_ap(ap_id); cp_id++) {
			/* Get the MCI index */
			mci_id = marvell_get_mci_map(ap_id, cp_id);
			INFO("AP%d MCI-%d CP-%d\n", ap_id, mci_id, cp_id);

			/* Open temp window in IO_WIN unit with relevant target ID */
			iowin_temp_win.target_id = mci_id;
			iow_temp_win_insert(ap_id, &iowin_temp_win, 1);

			/* Calculate the new CP110 - base address */
			cp110_base = MVEBU_CP_REGS_BASE(ap_id, cp_id);
			/* Go and update the CP110 configuration address space */
			iob_cfg_space_update(cp_id, cp110_temp_base, cp110_base);

			/* Remove the temporary IO-WIN window */
			iow_temp_win_remove(ap_id, &iowin_temp_win, 1);
		}

		/* Remove the temporary CCU window */
		ccu_temp_win_remove(ap_id, &ccu_temp_win, 1);

		/* Remove the temporary GWIN window */
		if (ap_id > 0)
			gwin_temp_win_remove(0, &gwin_temp_win, 1);
	}

	debug_exit();
}

/* This function overruns the same function in marvell_bl31_setup.c */
void bl31_plat_arch_setup(void)
{
	uintptr_t *mailbox = (void *)PLAT_MARVELL_MAILBOX_BASE;

	/* initiliaze the timer for mdelay/udelay functionality */
	plat_delay_timer_init();

	/* Configure ap810 */
	ap810_init();

	/* Update configuration space of CP110 from 0xf200_0000, to the
	** new address according to address map of Armada-8k-plus family.
	** */
	update_cp110_default_win();

	/* configure AP810 address decode - call it after
	** update_cp110_default_win to make sure that temporary windows do
	** not override any window that will be configured in GWIN/CCU/IOWIN
	** */
	ap810_addr_decode_init();

	/* In marvell_bl31_plat_arch_setup, el3 mmu is configured.
	 * el3 mmu configuration MUST be called after ap810_init, if not,
	 * this will cause an hang in init_io_win
	 * (after setting the IO windows GCR values).
	 */
	if (mailbox[MBOX_IDX_MAGIC] != MVEBU_MAILBOX_MAGIC_NUM ||
	    mailbox[MBOX_IDX_SUSPEND_MAGIC] != MVEBU_MAILBOX_SUSPEND_STATE)
		marvell_bl31_plat_arch_setup();
}

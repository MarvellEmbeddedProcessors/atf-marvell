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
#include <cp110_setup.h>
#include <marvell_pm.h>
#include <mmio.h>
#include <debug.h>

#ifdef SCP_IMAGE
#include <mss_ipc_drv.h>
#include <mss_mem.h>
#endif

/* Set a weak stub for platforms that don't need to configure GPIO */
#pragma weak marvell_gpio_config
int marvell_gpio_config(void)
{
	return 0;
}

void marvell_bl31_mpp_init(void)
{
	return;
}

void marvell_bl31_mss_init(void)
{
#ifdef SCP_IMAGE
	struct mss_pm_ctrl_block *mss_pm_crtl =
			(struct mss_pm_ctrl_block *)MSS_SRAM_PM_CONTROL_BASE;

	INFO("MSS IPC init\n");

	if (mss_pm_crtl->ipc_state == IPC_INITIALIZED)
		mv_pm_ipc_init(mss_pm_crtl->ipc_base_address | MVEBU_REGS_BASE);
#else
	INFO("MSS is not supported in this build\n");
#endif
}

/* This function overruns the same function in marvell_bl31_setup.c */
void bl31_plat_arch_setup(void)
{
	uintptr_t *mailbox = (void *)PLAT_MARVELL_MAILBOX_BASE;

	/* initiliaze the timer for mdelay/udelay functionality */
	plat_delay_timer_init();

	/* configure apn806 */
	ap810_init();

	/* In marvell_bl31_plat_arch_setup, el3 mmu is configured.
	 * el3 mmu configuration MUST be called after ap810_init, if not,
	 * this will cause an hang in init_io_win
	 * (after setting the IO windows GCR values).
	 */
	if (mailbox[MBOX_IDX_MAGIC] != MVEBU_MAILBOX_MAGIC_NUM ||
	    mailbox[MBOX_IDX_SUSPEND_MAGIC] != MVEBU_MAILBOX_SUSPEND_STATE)
		marvell_bl31_plat_arch_setup();

	/* Configure CP110 */
	INFO("Call CP110 init function\n");

	/* Should be called only after setting IOB windows */
	marvell_bl31_mpp_init();

	/* initialize IPC between MSS and ATF */
	if (mailbox[MBOX_IDX_MAGIC] != MVEBU_MAILBOX_MAGIC_NUM ||
	    mailbox[MBOX_IDX_SUSPEND_MAGIC] != MVEBU_MAILBOX_SUSPEND_STATE)
		marvell_bl31_mss_init();

	/* Configure GPIO */
	marvell_gpio_config();
}

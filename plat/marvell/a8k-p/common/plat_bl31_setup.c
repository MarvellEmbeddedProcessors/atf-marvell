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
#include <mvebu.h>
#include <mmio.h>
#include <debug.h>
#include <delay_timer.h>
#include <mci.h>

#define MVEBU_AP_SYSTEM_SOFT_RESET_REG(ap)	(MVEBU_AP_MISC_SOC_BASE(ap) + 0x54)
/* For every MCI have 2 reset parameters MAIN/PHY SW reset */
#define SOFT_RESET_IHBx_MAIN_SW_RESET(mci)		((0x1) << ((mci * 2) + 3))
#define SOFT_RESET_IHBx_PHY_SW_RESET(mci)		((0x1) << ((mci * 2) + 4))

/* MCIx_REG_START_ADDRESS */
#define MVEBU_MCI_REG_START_ADDRESS(ap, mci)		(MVEBU_AR_RFU_BASE(ap) + 0x4158 + mci * 0x4)
#define MVEBU_MCI_REG_START_ADDR_SHIFT			12

#define MVEBU_IHB_CNTRL_REG_1(ap, mci_idx)		(MVEBU_MCI_PHY(ap, mci_idx) + 0x4)
#define IHB_CNTRL_REG1_DEVICE_WDAT(val)			(((val) & 0xff) << 0)
#define IHB_CNTRL_REG1_DEVICE_ADDR_BYTE_SEL(val)	(((val) & 0x3) << 16)
#define IHB_CNTRL_REG1_DEVICE_ADDR(val)			(((val) & 0xff) << 18)
#define IHB_CNTRL_REG1_DEVICE_REG_WEN(val)		(((val) & 0x1) << 28)
#define IHB_CNTRL_REG1_DEVICE_WEN_DONE(val)		(((val) & 0x1) << 29)

#define MVEBU_IHB_PWM_CTRL_REG3(ap, mci_idx)		(MVEBU_MCI_PHY(ap, mci_idx) + 0x1c)
#define IHB_PWM_CTRL_REG3_AUTO_SPEED_OFFSET		0
#define IHB_PWM_CTRL_REG3_AUTO_SPEED_MASK		(0xf << IHB_PWM_CTRL_REG3_AUTO_SPEED_OFFSET)

#define MCI_RETRY_COUNT					10

uint32_t mci_get_link_speed(int ap_idx, int mci_idx)
{
	return mmio_read_32(MVEBU_IHB_PWM_CTRL_REG3(ap_idx, mci_idx)) & IHB_PWM_CTRL_REG3_AUTO_SPEED_MASK;
}

void mci_phy_config(int ap_idx, int mci_idx)
{
	mmio_write_32(MVEBU_IHB_CNTRL_REG_1(ap_idx, mci_idx),
				IHB_CNTRL_REG1_DEVICE_WDAT(0x50) |
				IHB_CNTRL_REG1_DEVICE_ADDR_BYTE_SEL(0x1) |
				IHB_CNTRL_REG1_DEVICE_ADDR(0x21) |
				IHB_CNTRL_REG1_DEVICE_REG_WEN(0x1));
	mdelay(5);

	mmio_write_32(MVEBU_IHB_CNTRL_REG_1(ap_idx, mci_idx),
				IHB_CNTRL_REG1_DEVICE_WDAT(0x50) |
				IHB_CNTRL_REG1_DEVICE_ADDR_BYTE_SEL(0x1) |
				IHB_CNTRL_REG1_DEVICE_ADDR(0x21) |
				IHB_CNTRL_REG1_DEVICE_REG_WEN(0x0));
	mdelay(5);
}

void ap810_mci_phy_soft_reset(int ap_id, int mci_idx)
{
	uint32_t reg;

	/* For every MCI, there is MAIN SW reset & PHY SW reset */
	reg = mmio_read_32(MVEBU_AP_SYSTEM_SOFT_RESET_REG(ap_id));
	reg &= ~(SOFT_RESET_IHBx_MAIN_SW_RESET(mci_idx) | SOFT_RESET_IHBx_PHY_SW_RESET(mci_idx));
	mmio_write_32(MVEBU_AP_SYSTEM_SOFT_RESET_REG(ap_id), reg);

	/* Wait 5ms before get into reset */
	mdelay(5);
	reg |= (SOFT_RESET_IHBx_MAIN_SW_RESET(mci_idx) | SOFT_RESET_IHBx_PHY_SW_RESET(mci_idx));
	mmio_write_32(MVEBU_AP_SYSTEM_SOFT_RESET_REG(ap_id), reg);
}

static void a8kp_mci_turn_off_links(uintptr_t mci_base, struct addr_map_win gwin_win,
				struct addr_map_win ccu_win, struct addr_map_win iowin_win)
{
	int ap_id, cp_id, mci_id;

	/* Go over the APs and turn off the link of MCIs */
	for (ap_id = 0; ap_id < get_ap_count(); ap_id++) {
		/* If initialize the MCIs in another APs, need to open
		** window in GWIN to enable access from AP0 to APx
		** */
		if (ap_id > 0) {
			gwin_win.target_id = ap_id;
			/* All the initialization will be done from AP0,
			** this GWIN configure the access to exit from AP0
			** and reach the target AP
			** */
			gwin_temp_win_insert(0, &gwin_win, 1);

			/* Open temp window for access to GWIN */
			ccu_win.target_id = GLOBAL_TID;
			ccu_temp_win_insert(0, &ccu_win, 1);
		}

		/* Open temp window in CCU unit */
		ccu_temp_win_insert(ap_id, &ccu_win, 1);

		/* Go over the MCIs  */
		for (cp_id = 0; cp_id < get_static_cp_per_ap(ap_id); cp_id++) {
			/* Get the MCI index */
			mci_id = marvell_get_mci_map(ap_id, cp_id);
			INFO("Turn link off for AP-%d MCI-%d\n", ap_id, mci_id);

			/* Open temp window IO_WIN unit with relevant target ID */
			iowin_win.target_id = mci_id;
			iow_temp_win_insert(ap_id, &iowin_win, 1);

			/* Open window for MCI indirect access from APx */
			mmio_write_32(MVEBU_MCI_REG_START_ADDRESS(ap_id, mci_id),
					mci_base >> MVEBU_MCI_REG_START_ADDR_SHIFT);
			/* Turn link off */
			mci_turn_link_down();
			/* Remove the temporary IO-WIN window */
			iow_temp_win_remove(ap_id, &iowin_win, 1);
		}

		/* Remove the temporary CCU window */
		ccu_temp_win_remove(ap_id, &ccu_win, 1);

		/* Remove the temporary GWIN window */
		if (ap_id > 0) {
			gwin_temp_win_remove(0, &gwin_win, 1);
			ccu_temp_win_remove(0, &ccu_win, 1);
		}
	}
}

static void a8kp_mci_mpp_reset(int mpp)
{
	uint32_t val;

	/* Set MPP to low */
	val = mmio_read_32(MVEBU_AP_GPIO_DATA_IN(0));
	val &= ~(0x1 << mpp);
	mmio_write_32(MVEBU_AP_GPIO_DATA_IN(0), val);

	/* Clear data out */
	val = mmio_read_32(MVEBU_AP_GPIO_DATA_OUT_VAL(0));
	val &= ~(0x1 << mpp);
	mmio_write_32(MVEBU_AP_GPIO_DATA_OUT_VAL(0), val);

	/* Enable data out */
	val = mmio_read_32(MVEBU_AP_GPIO_DATA_OUT_EN(0));
	val &= ~(0x1 << mpp);
	mmio_write_32(MVEBU_AP_GPIO_DATA_OUT_EN(0), val);

	INFO("Get out CPs from reset\n");
	val = mmio_read_32(MVEBU_AP_GPIO_DATA_OUT_VAL(0));
	val |= (0x1 << mpp);
	mmio_write_32(MVEBU_AP_GPIO_DATA_OUT_VAL(0), val);

	/* Wait until CP release from reset */
	mdelay(2);
}

/* MCI initialize for all APs, the sequence split to 3 parts:
** 1. Turn off the link on all MCIs
** 2. Reset the CPs via MPP
** 3. Re-init the MCI phy in AP side & in CP side
** */
static int mci_wa_initialize(void)
{
	int ap_id, mci_id, cp_id;
	uintptr_t mci_base;
	struct addr_map_win mci_gwin_temp_win, mci_ccu_temp_win, mci_iowin_temp_win;

	debug_enter();

	/* Got the MCI base to work with - need to open windows in different units
	** GWIN - if MCI is on the remote AP, CCU, and IO-WIN
	** */
	mci_base = MVEBU_MCI_REG_BASE_REMAP(0);

	/* Prepare GWIN/CCU/IOWIN windows */
	mci_gwin_temp_win.base_addr = mci_base;
	mci_gwin_temp_win.win_size = MVEBU_MCI_REG_SIZE_REMAP;

	mci_ccu_temp_win.base_addr = mci_base;
	mci_ccu_temp_win.win_size = MVEBU_MCI_REG_SIZE_REMAP;
	/* Target of the CCU to access CP should be IO */
	mci_ccu_temp_win.target_id = IO_0_TID;

	mci_iowin_temp_win.base_addr = mci_base;
	mci_iowin_temp_win.win_size = MVEBU_MCI_REG_SIZE_REMAP;

	/* 1st stage - Turn off the link on all MCIs */
	a8kp_mci_turn_off_links(mci_base, mci_gwin_temp_win, mci_ccu_temp_win, mci_iowin_temp_win);

	/* 2nd stage - reset CPs via MPP */
	a8kp_mci_mpp_reset(MPP_MCI_RELEASE_FROM_RESET);

	/* 3rd stage - Re-init the MCI phy in AP side & in CP side */
	for (ap_id = 0; ap_id < get_ap_count(); ap_id++) {
		/* If initialize the MCIs in another APs, need to open
		** window in GWIN to enable access from AP0 to APx
		** */
		if (ap_id > 0) {
			mci_gwin_temp_win.target_id = ap_id;
			/* All the initialization will be done from AP0,
			** this GWIN configure the access to exit from AP0
			** and reach the target AP
			** */
			gwin_temp_win_insert(0, &mci_gwin_temp_win, 1);

			/* Open temp window for access to GWIN */
			mci_ccu_temp_win.target_id = GLOBAL_TID;
			ccu_temp_win_insert(0, &mci_ccu_temp_win, 1);
		}

		/* Open temp window in CCU unit */
		ccu_temp_win_insert(ap_id, &mci_ccu_temp_win, 1);

		/* Go over the MCIs in every APx */
		for (cp_id = 0; cp_id < get_static_cp_per_ap(ap_id); cp_id++) {
			uint32_t reg;
			/* Get the MCI index */
			mci_id = marvell_get_mci_map(ap_id, cp_id);
			INFO("Turn link on & ID assign AP%d MCI-%d\n", ap_id, mci_id);

			/* Config MCI phy on AP side */
			mci_phy_config(ap_id, mci_id);

			/* Reset MCI phy on AP side */
			ap810_mci_phy_soft_reset(ap_id, mci_id);

			/* Open temp window IO_WIN unit with relevant target ID */
			mci_iowin_temp_win.target_id = mci_id;
			iow_temp_win_insert(ap_id, &mci_iowin_temp_win, 1);

			/* Open window for MCI indirect access from APx */
			mmio_write_32(MVEBU_MCI_REG_START_ADDRESS(ap_id, mci_id),
					mci_base >> MVEBU_MCI_REG_START_ADDR_SHIFT);

			/* Turn on link on CP side */
			mci_turn_link_on();

			/* Wait 20ms, until link is stable*/
			mdelay(20);

			/* Check the link status on CP side */
			reg = mci_get_link_status();
			if (reg == -1) {
				ERROR("bad link on MCI-%d - status register is %x\n", mci_id, reg);
				return -1;
			}

			reg = mci_get_link_speed(ap_id, mci_id);
			if (reg != 0x3) {
				ERROR("link speed is not correct on MCI-%d - link speed is %x\n", mci_id, reg);
				return -1;
			}

			INFO("MCI-%d link is 8G (speed = %x)\n", mci_id, reg);

			/* Remove the temporary IO-WIN window */
			iow_temp_win_remove(ap_id, &mci_iowin_temp_win, 1);
		}

		/* Remove the temporary CCU window */
		ccu_temp_win_remove(ap_id, &mci_ccu_temp_win, 1);

		/* Remove the temporary GWIN window */
		if (ap_id > 0) {
			gwin_temp_win_remove(0, &mci_gwin_temp_win, 1);
			ccu_temp_win_remove(0, &mci_ccu_temp_win, 1);
		}
	}

	debug_exit();
	return 0;
}

/* Armada-8k-plus have bug on MCI, that link between AP & CP is
** not stable and should be re-initialize.
** */
void a8kp_mci_wa_initialize(void)
{
	int retry = MCI_RETRY_COUNT;

	/* Retry until success, if the MCI initialization failed, reset is required */
	while (retry > 0) {
		if (mci_wa_initialize() == 0)
			break;
		ERROR("MCIx failed to create link - retry again %d of %d\n", retry, MCI_RETRY_COUNT);
		retry--;
	}

	if (retry == 0) {
		ERROR("MCIx failed to create link after %d times, reset is required\n", MCI_RETRY_COUNT);
		panic();
	}
}

/* Configure the threshold of every MCI */
static int a8kp_mci_configure_threshold(void)
{
	int ap_id, mci_id, cp_id;
	uintptr_t mci_base;
	struct addr_map_win mci_gwin_temp_win, mci_ccu_temp_win, mci_iowin_temp_win;

	debug_enter();

	/* Got the MCI base to work with - need to open windows in different units
	** GWIN - if MCI is on the remote AP, CCU, and IO-WIN
	** */
	mci_base = MVEBU_MCI_REG_BASE_REMAP(0);

	/* Prepare GWIN/CCU/IOWIN windows */
	mci_gwin_temp_win.base_addr = mci_base;
	mci_gwin_temp_win.win_size = MVEBU_MCI_REG_SIZE_REMAP;

	mci_ccu_temp_win.base_addr = mci_base;
	mci_ccu_temp_win.win_size = MVEBU_MCI_REG_SIZE_REMAP;
	/* Target of the CCU to access CP should be IO */
	mci_ccu_temp_win.target_id = IO_0_TID;

	mci_iowin_temp_win.base_addr = mci_base;
	mci_iowin_temp_win.win_size = MVEBU_MCI_REG_SIZE_REMAP;

	/* Run MCI WA for performance improvements */
	for (ap_id = 0; ap_id < get_ap_count(); ap_id++) {
		/* If initialize the MCIs in another APs, need to open
		** window in GWIN to enable access from AP0 to APx
		** */
		if (ap_id > 0) {
			mci_gwin_temp_win.target_id = ap_id;
			/* All the initialization will be done from AP0,
			** this GWIN configure the access to exit from AP0
			** and reach the target AP
			** */
			gwin_temp_win_insert(0, &mci_gwin_temp_win, 1);

			/* Open temp window for access to GWIN */
			mci_ccu_temp_win.target_id = GLOBAL_TID;
			ccu_temp_win_insert(0, &mci_ccu_temp_win, 1);
		}

		/* Open temp window in CCU unit */
		ccu_temp_win_insert(ap_id, &mci_ccu_temp_win, 1);

		/* Go over the MCIs in every APx */
		for (cp_id = 0; cp_id < get_connected_cp_per_ap(ap_id); cp_id++) {
			/* Get the MCI index */
			mci_id = marvell_get_mci_map(ap_id, cp_id);
			INFO("Configure threshold & ID assin AP%d MCI-%d\n", ap_id, mci_id);

			/* Open temp window IO_WIN unit with relevant target ID */
			mci_iowin_temp_win.target_id = mci_id;
			iow_temp_win_insert(ap_id, &mci_iowin_temp_win, 1);

			/* Open window for MCI indirect access from APx */
			mmio_write_32(MVEBU_MCI_REG_START_ADDRESS(ap_id, mci_id),
					mci_base >> MVEBU_MCI_REG_START_ADDR_SHIFT);

			/* Run MCI WA for performance improvements */
			mci_initialize(mci_id);

			/* Remove the temporary IO-WIN window */
			iow_temp_win_remove(ap_id, &mci_iowin_temp_win, 1);
		}

		/* Remove the temporary CCU window */
		ccu_temp_win_remove(ap_id, &mci_ccu_temp_win, 1);

		/* Remove the temporary GWIN window */
		if (ap_id > 0) {
			gwin_temp_win_remove(0, &mci_gwin_temp_win, 1);
			ccu_temp_win_remove(0, &mci_ccu_temp_win, 1);
		}
	}

	debug_exit();
	return 0;
}

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

	/* Re-init MCI connection due bug in Armada-8k-plus */
	/* TODO; Check revision */
	a8kp_mci_wa_initialize();

	/* Initialize the MCI threshold to improve performance */
	a8kp_mci_configure_threshold();

	/* Update configuration space of CP110 from 0xf200_0000, to the
	** new address according to address map of Armada-8k-plus family.
	** */
	update_cp110_default_win();

	/* configure AP810 address decode - call it after
	** update_cp110_default_win to make sure that temporary windows do
	** not override any window that will be configured in GWIN/CCU/IOWIN
	** */
	ap810_addr_decode_init();

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

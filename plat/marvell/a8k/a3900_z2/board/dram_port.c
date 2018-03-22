/*
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <arch_helpers.h>
#include <plat_marvell.h>
#include <debug.h>
#include <mv_ddr_if.h>

/*
 * This function may modify the default DRAM parameters
 * based on information recieved from SPD or bootloader
 * configuration located on non volatile storage
 */
void plat_dram_update_topology(void)
{
	NOTICE("Gathering DRAM information\n");
}

/*
 * This struct provides the DRAM training code with
 * the appropriate board DRAM configuration
 */
struct mv_ddr_iface dram_iface_ap0 = {
	.ap_base = MVEBU_REGS_BASE_AP(0),
	.state = MV_DDR_IFACE_NRDY,
	.id = 0,
	.iface_base_addr = 0,
	.tm = {
	/* FIXME: Z0 board 1CS 8Gb x16 devices of micron - 2400P */
		DEBUG_LEVEL_ERROR,
		0x1, /* active interfaces */
		/* cs_mask, mirror, dqs_swap, ck_swap X subphys */
		{ { { {0x1, 0x0, 0, 0},
		      {0x1, 0x0, 0, 0},
		      {0x1, 0x0, 0, 0},
		      {0x1, 0x0, 0, 0},
		      {0x1, 0x0, 0, 0},
		      {0x1, 0x0, 0, 0},
		      {0x1, 0x0, 0, 0},
		      {0x1, 0x0, 0, 0},
		      {0x1, 0x0, 0, 0} },
		   SPEED_BIN_DDR_2400R,		/* speed_bin */
		   MV_DDR_DEV_WIDTH_16BIT,	/* sdram device width */
		   MV_DDR_DIE_CAP_8GBIT,	/* die capacity */
		   MV_DDR_FREQ_SAR,		/* frequency */
		   0, 0,			/* cas_l, cas_wl */
		   MV_DDR_TEMP_LOW} },		/* temperature */
		BUS_MASK_32BIT,			/* subphys mask */
		MV_DDR_CFG_DEFAULT,		/* ddr configuration data source */
		{ {0} },			/* raw spd data */
		{0}				/* timing parameters */
	},
};

/* Pointer to the first DRAM interface in the system */
struct mv_ddr_iface *ptr_iface = &dram_iface_ap0;

struct mv_ddr_iface *mv_ddr_iface_get(void)
{
	/* Return current ddr interface */
	return ptr_iface;
}

struct mv_ddr_topology_map *mv_ddr_topology_map_get(void)
{
	/* Return the board topology as defined in the board code */
	return &ptr_iface->tm;
}

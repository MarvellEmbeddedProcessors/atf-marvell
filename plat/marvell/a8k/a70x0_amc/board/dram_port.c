/*
 * Copyright (C) 2016 - 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */
#include <arch_helpers.h>
#include <debug.h>
#include <mv_ddr_if.h>
#include <plat_marvell.h>

/*
 * This function may modify the default DRAM parameters
 * based on information received from SPD or bootloader
 * configuration located on non volatile storage
 */
void plat_dram_update_topology(void)
{
	INFO("Gathering DRAM information\n");
}

/*
 * This struct provides the DRAM training code with
 * the appropriate board DRAM configuration
 */
static struct mv_ddr_topology_map board_topology_map = {
/* FIXME: MISL board 2CS 8Gb x8 devices of micron - 2133P */
	DEBUG_LEVEL_ERROR,
	0x1, /* active interfaces */
	/* cs_mask, mirror, dqs_swap, ck_swap X subphys */
	{ { { {0x3, 0x2, 0, 0},
	      {0x3, 0x2, 0, 0},
	      {0x3, 0x2, 0, 0},
	      {0x3, 0x2, 0, 0},
	      {0x3, 0x2, 0, 0},
	      {0x3, 0x2, 0, 0},
	      {0x3, 0x2, 0, 0},
	      {0x3, 0x2, 0, 0},
	      {0x3, 0x2, 0, 0} },
	   SPEED_BIN_DDR_2400T,		/* speed_bin */
	   MV_DDR_DEV_WIDTH_8BIT,	/* sdram device width */
	   MV_DDR_DIE_CAP_8GBIT,	/* die capacity */
	   MV_DDR_FREQ_SAR,		/* frequency */
	   0, 0,			/* cas_l, cas_wl */
	   MV_DDR_TEMP_LOW} },		/* temperature */
	MV_DDR_32BIT_ECC_PUP8_BUS_MASK,	/* subphys mask */
	MV_DDR_CFG_DEFAULT,		/* ddr configuration data source */
	{ {0} },			/* raw spd data */
	{0}				/* timing parameters */
};

struct mv_ddr_topology_map *mv_ddr_topology_map_get(void)
{
	/* Return the board topology as defined in the board code */
	return &board_topology_map;
}

/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <arch_helpers.h>
#include <plat_marvell.h>
#include <debug.h>
#include <mv_ddr_if.h>
#include <plat_def.h>
#include <mmio.h>

struct dram_config dram_cfg;

/*
 * This struct provides the DRAM training code with
 * the appropriate board DRAM configuration
 */
static struct mv_ddr_topology_map board_topology_map = {
	/* MISL board with 1CS 8Gb x4 devices of Micron 2400T */
	DEBUG_LEVEL_ERROR,
	0x1, /* active interfaces */
	/* cs_mask, mirror, dqs_swap, ck_swap X subphys */
	{ { { {0x1, 0x0, 0, 0},	/* FIXME: change the cs mask for all 64 bit */
	      {0x1, 0x0, 0, 0},
	      {0x1, 0x0, 0, 0},
	      {0x1, 0x0, 0, 0},
	      {0x1, 0x0, 0, 0},
	      {0x1, 0x0, 0, 0},
	      {0x1, 0x0, 0, 0},
	      {0x1, 0x0, 0, 0},
	      {0x1, 0x0, 0, 0} },
	   /* TODO: double check if the speed bin is 2400S */
	   SPEED_BIN_DDR_2400S,		/* speed_bin */
	   MV_DDR_DEV_WIDTH_8BIT,	/* sdram device width */
	   MV_DDR_DIE_CAP_8GBIT,	/* die capacity */
	   MV_DDR_FREQ_SAR,		/* frequency */
	   0, 0,			/* cas_l, cas_wl */
	   MV_DDR_TEMP_LOW} },		/* temperature */
	MV_DDR_64BIT_ECC_PUP8_BUS_MASK, /* subphys mask */
	MV_DDR_CFG_SPD,			/* ddr configuration data source */
	{ {0} },			/* raw spd data */
	{0}				/* timing parameters */
};

struct mv_ddr_topology_map *mv_ddr_topology_map_get(void)
{
	/* Return the board topology as defined in the board code */
	return &board_topology_map;
}

struct dram_config *mv_ddr_dram_config_get(void)
{
	/* Return dram configuration as defined in the board code */
	return &dram_cfg;
}

/*
 * This function may modify the default DRAM parameters
 * based on information recieved from SPD or bootloader
 * configuration located on non volatile storage
 */
int update_dram_info(struct dram_config *cfg)
{
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();

	INFO("Gathering DRAM information\n");

	if (tm->cfg_src == MV_DDR_CFG_SPD)
		INFO("SPD functionality is not implemented\n");

	return 0;
}

void *plat_get_dram_data(void)
{
	/* Update DRAM for dynamic platforms */
	update_dram_info(&dram_cfg);

	return &dram_cfg;
}

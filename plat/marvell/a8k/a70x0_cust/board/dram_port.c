/*
 * Copyright (C) 2016 - 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */
#include <arch_helpers.h>
#include <a8k_i2c.h>
#include <debug.h>
#include <mmio.h>
#include <mv_ddr_if.h>
#include <plat_def.h>
#include <plat_marvell.h>

/*
 * This struct provides the DRAM training code with
 * the appropriate board DRAM configuration
 */
static struct mv_ddr_topology_map board_topology_map = {
/* FIXME: Customer board with 2CS 4Gb x8 devices of Micron 2133P */
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
	   SPEED_BIN_DDR_2133P,		/* speed_bin */
	   MV_DDR_DEV_WIDTH_8BIT,	/* sdram device width */
	   MV_DDR_DIE_CAP_4GBIT,	/* die capacity */
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

static void mpp_config(void)
{
	uint32_t val;

	/* Enable CP0 I2C MPPs (MPP: 37-38) */
	val = mmio_read_32(MVEBU_CP_MPP_REGS(0, 4));
	mmio_write_32(MVEBU_CP_MPP_REGS(0, 4), val | 0x2200000);
}

/*
 * This function may modify the default DRAM parameters
 * based on information received from SPD or bootloader
 * configuration located on non volatile storage
 */
void plat_dram_update_topology(void)
{
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();

	INFO("Gathering DRAM information\n");

	if (tm->cfg_src == MV_DDR_CFG_SPD) {
		/* configure mpps for i2c functionality */
		mpp_config();
		/* initialize ap i2c */
		i2c_init((void *)MVEBU_CP0_I2C_BASE);
		/* select SPD memory page 0 to access DRAM configuration */
		i2c_write(CP0_I2C_SPD_P0_ADDR, 0x0, 2,
			  tm->spd_data.all_bytes, 1);
		/* read data from spd */
		i2c_read(CP0_I2C_SPD_ADDR, 0x0, 2, tm->spd_data.all_bytes,
			 sizeof(tm->spd_data.all_bytes));
	}
}

/*
 * ***************************************************************************
 * Copyright (C) 2016 Marvell International Ltd.
 * ***************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * Neither the name of Marvell nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ***************************************************************************
 */

#include <arch_helpers.h>
#include <plat_marvell.h>
#include <debug.h>
#include <dram_if.h>
#include <plat_def.h>
#include <mmio.h>
#include <a8k_i2c.h>

#include <mv_ddr_atf_wrapper.h>
#include <apn806/mv_ddr_apn806.h>
#include <apn806/mv_ddr_apn806_topology.h>
#include <ddr3_topology_def.h>

#define MVEBU_AP_MPP_CTRL0_7_REG		MVEBU_AP_MPP_REGS(0)
#define MVEBU_AP_MPP_CTRL4_OFFS			16
#define MVEBU_AP_MPP_CTRL5_OFFS			20
#define MVEBU_AP_MPP_CTRL4_I2C0_SDA_ENA		0x3
#define MVEBU_AP_MPP_CTRL5_I2C0_SCK_ENA		0x3

#define MVEBU_CP_MPP_CTRL37_OFFS		20
#define MVEBU_CP_MPP_CTRL38_OFFS		24
#define MVEBU_CP_MPP_CTRL37_I2C0_SCK_ENA	0x2
#define MVEBU_CP_MPP_CTRL38_I2C0_SDA_ENA	0x2

#define MVEBU_MPP_CTRL_MASK			0xf

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
	   DDR_FREQ_SAR,		/* frequency */
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

static void mpp_config(void)
{
	uintptr_t reg;
	uint32_t val;

	/*
	 * The DRAM SPD on A0 and A1 boards is located on different i2c channels
	 * The A80x0 A0 DB boards are using the AP i2c channel (MPP4 and MPP5),
	 * while A80x0 A1 DB boards - the CP0 i2c one (MPP37, MPP38).
	 */
	if (apn806_rev_id_get() == APN806_REV_ID_A0) {
		/* configure ap mmps 4, 5 to i2c */
		reg = MVEBU_AP_MPP_CTRL0_7_REG;
		val = mmio_read_32(reg);

		val &= ~((MVEBU_MPP_CTRL_MASK << MVEBU_AP_MPP_CTRL4_OFFS) |
			(MVEBU_MPP_CTRL_MASK << MVEBU_AP_MPP_CTRL5_OFFS));
		val |= ((MVEBU_AP_MPP_CTRL4_I2C0_SDA_ENA <<
				MVEBU_AP_MPP_CTRL4_OFFS) |
			(MVEBU_AP_MPP_CTRL5_I2C0_SCK_ENA <<
				MVEBU_AP_MPP_CTRL5_OFFS));
		mmio_write_32(reg, val);
		val = mmio_read_32(reg);
	} else {
		reg = MVEBU_CP_MPP_REGS(0, 4);
		/* configure CP0 MPP 37 and 38 to i2c */
		val = mmio_read_32(reg);
		val &= ~((MVEBU_MPP_CTRL_MASK << MVEBU_CP_MPP_CTRL37_OFFS) |
			(MVEBU_MPP_CTRL_MASK << MVEBU_CP_MPP_CTRL38_OFFS));
		val |= (MVEBU_CP_MPP_CTRL37_I2C0_SCK_ENA <<
				MVEBU_CP_MPP_CTRL37_OFFS) |
			(MVEBU_CP_MPP_CTRL38_I2C0_SDA_ENA <<
				MVEBU_CP_MPP_CTRL38_OFFS);
		mmio_write_32(reg, val);
	}
}

/*
 * This function may modify the default DRAM parameters
 * based on information recieved from SPD or bootloader
 * configuration located on non volatile storage
 */
int update_dram_info(struct dram_config *cfg)
{
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();

	NOTICE("Gathering DRAM information\n");

	if (tm->cfg_src == MV_DDR_CFG_SPD) {
		/* configure MPPs to enable i2c */
		mpp_config();
		/*
		 * The DRAM SPD on A0 and A1 boards is located on different i2c
		 * channels
		 * The A80x0 A0 DB boards are using the AP i2c channel,
		 * while A80x0 A1 DB boards - the CP0 i2c one.
		 * In both cases the SPD device address on i2c bus is the same.
		 */
		if (apn806_rev_id_get() == APN806_REV_ID_A0)
			/* initialize ap i2c */
			i2c_init((void *)MVEBU_AP_I2C_BASE);
		else
			 /* initialize ap i2c */
			i2c_init((void *)MVEBU_CP0_I2C_BASE);
		/*
		 * Dummy read to the SPD chip memory page selector.
		 * It is needed for for selecting the SPD memory page 0
		 * prior to accessing the DRAM configuration data
		 */
		i2c_read(I2C_SPD_P0_ADDR, 0x0, 1, tm->spd_data.all_bytes, 1);
		/* read data from spd */
		i2c_read(I2C_SPD_ADDR, 0x0, 1, tm->spd_data.all_bytes,
			 sizeof(tm->spd_data.all_bytes));
	}

	return 0;
}

void *plat_get_dram_data(void)
{
	/* Update DRAM for dynamic platforms */
	update_dram_info(&dram_cfg);

	return &dram_cfg;
}

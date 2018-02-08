/*
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <mmio.h>
#include <debug.h>
#include <plat_def.h>
#include <plat_dram.h>
#include <mv_ddr_if.h>
#include <a8k_i2c.h>
#include <ap810_setup.h>
#include <ap810_init_clocks.h>

#define CCU_RGF_WIN0_REG(ap)		(MVEBU_CCU_BASE(ap) + 0x90)
#define CCU_RGF_WIN_UNIT_ID_OFFS	2
#define CCU_RGF_WIN_UNIT_ID_MASK	0xf

#define DSS_SCR_REG(ap, iface)		(MVEBU_AR_RFU_BASE(ap) + 0x208 + ((iface) * 0x4))
#define DSS_PPROT_OFFS			4
#define DSS_PPROT_MASK			0x7
#define DSS_PPROT_PRIV_SECURE_DATA	0x1

/* Extern the parameters from porting file */
extern struct dram_config dram_cfg;
extern struct mv_ddr_iface dram_iface_ap0[DDR_MAX_UNIT_PER_AP];
extern struct mv_ddr_iface *ptr_iface;

/* Use global varibale to check if i2c initialization done */
int i2c_init_done = 0;

int plat_dram_ap_ifaces_get(int ap_id, struct mv_ddr_iface **ifaces, uint32_t *size)
{
	*size = sizeof(dram_iface_ap0)/sizeof(dram_iface_ap0[0]);
	*ifaces = dram_iface_ap0;

	return 0;
}

void plat_dram_iface_set(struct mv_ddr_iface *iface)
{
	ptr_iface = iface;
}

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

struct dram_config *mv_ddr_dram_config_get(void)
{
	/* Return dram configuration as defined in the board code */
	return &dram_cfg;
}

/*
 * TODO: Move to the pin control driver API once it becomes available
 */
#define MVEBU_AP_MPP_CTRL16_23_REG		MVEBU_AP_MPP_REGS(0, 2)
#define MVEBU_AP_MPP_CTRL18_OFFS		8
#define MVEBU_AP_MPP_CTRL19_OFFS		12
#define MVEBU_AP_MPP_CTRL4_I2C0_SDA_ENA		0x3
#define MVEBU_AP_MPP_CTRL5_I2C0_SCK_ENA		0x3

#define MVEBU_MPP_CTRL_MASK			0xf

static void mpp_config(void)
{
	uintptr_t reg;
	uint32_t val;

	/*
	 * The Ax0x0 A0 DB boards are using the AP0 i2c channel (MPP18 and MPP19)
	 * for accessing all DIMM SPDs available on board.
	 */
	reg = MVEBU_AP_MPP_CTRL16_23_REG;
	val = mmio_read_32(reg);

	val &= ~((MVEBU_MPP_CTRL_MASK << MVEBU_AP_MPP_CTRL18_OFFS) |
		(MVEBU_MPP_CTRL_MASK << MVEBU_AP_MPP_CTRL19_OFFS));
	val |= ((MVEBU_AP_MPP_CTRL4_I2C0_SDA_ENA << MVEBU_AP_MPP_CTRL18_OFFS) |
		(MVEBU_AP_MPP_CTRL5_I2C0_SCK_ENA << MVEBU_AP_MPP_CTRL19_OFFS));

	mmio_write_32(reg, val);
	val = mmio_read_32(reg);
}

void dram_freq_update(enum ddr_freq freq_option)
{
	struct mv_ddr_topology_map *tm = mv_ddr_topology_map_get();
	tm->interface_params[0].memory_freq = freq_option;
}

/*
 * This function may modify the default DRAM parameters
 * based on information received from SPD or bootloader
 * configuration located on non volatile storage
 */
void plat_dram_update_topology(struct mv_ddr_topology_map *tm)
{
	INFO("Update DRAM information\n");

	if (tm->cfg_src == MV_DDR_CFG_SPD) {
		/* Initialize I2C of AP-0 to read SPD
		** need to initialize the I2C once.
		** */
		if (i2c_init_done == 0) {
			/* Configure MPPs to enable i2c */
			mpp_config();
			/* Enable I2C on AP0 */
			i2c_init((void *)MVEBU_AP_I2C_BASE(0));
			/* Mark done */
			i2c_init_done = 1;
		}
		/* select SPD memory page 0 to access DRAM configuration */
		i2c_write(I2C_SPD_P0_SEL_ADDR, 0x0, 1, tm->spd_data.all_bytes, 1);
		/* TODO: Support multiple intefaces when reading the SPD data */
		/* read data from spd */
		i2c_read(I2C_SPD_DATA_ADDR(0), 0x0, 1, tm->spd_data.all_bytes,
			 sizeof(tm->spd_data.all_bytes));
	}
}

static void plat_dram_phy_access_config(uint32_t ap_id, uint32_t iface_id)
{
	uint32_t reg_val, dram_target;

	if (iface_id == 1)
		dram_target = DRAM_1_TID;
	else
		dram_target = DRAM_0_TID;

	/* Update PHY destination in RGF window */
	reg_val = mmio_read_32(CCU_RGF_WIN0_REG(ap_id));
	reg_val &= ~(CCU_RGF_WIN_UNIT_ID_MASK << CCU_RGF_WIN_UNIT_ID_OFFS);
	reg_val |= ((dram_target & CCU_RGF_WIN_UNIT_ID_MASK) << CCU_RGF_WIN_UNIT_ID_OFFS);
	mmio_write_32(CCU_RGF_WIN0_REG(ap_id), reg_val);

	/* Update DSS port access permission to DSS_PHY */
	reg_val = mmio_read_32(DSS_SCR_REG(ap_id, iface_id));
	reg_val &= ~(DSS_PPROT_MASK << DSS_PPROT_OFFS);
	reg_val |= ((DSS_PPROT_PRIV_SECURE_DATA & DSS_PPROT_MASK) << DSS_PPROT_OFFS);
	mmio_write_32(DSS_SCR_REG(ap_id, iface_id), reg_val);
}

int plat_dram_init(void)
{
	struct mv_ddr_iface *iface = NULL;
	uint32_t ifaces_size, i, ap_id, ret;

	for (ap_id = 0; ap_id < get_ap_count(); ap_id++) {
		/* Get interfaces of AP-ID */
		plat_dram_ap_ifaces_get(ap_id, &iface, &ifaces_size);
		/* Go over the interfaces of AP and initialize them */
		for (i = 0; i < ifaces_size; i++, iface++) {
			/* Set the pointer to current interface */
			plat_dram_iface_set(iface);

			/* Skip if not exist */
			if (iface->state == MV_DDR_IFACE_DNE)
				continue;

			/* Set phy accesses */
			plat_dram_phy_access_config(ap_id, iface->id);

			/* Update DRAM topology (scan DIMM SPDs) */
			plat_dram_update_topology(&iface->tm);

			/* Call DRAM init per interface */
			ret = dram_init();
			if (ret)
				return ret;

			/* Update status of interface */
			iface->state = MV_DDR_IFACE_RDY;
		}
	}

	return 0;
}

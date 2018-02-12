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

/* In the below macros the "iftid" is either DRAM_0_TID or DRAM_1_TID */
#define CCU_MC_RCR_OFFSET(ap, iface_tid)	(MVEBU_A2_BANKED_STOP_BASE(ap, \
							iface_tid) + 0x300)
#define CCU_MC_RSBR_OFFSET(ap, iface_tid)	(MVEBU_A2_BANKED_STOP_BASE(ap, \
							iface_tid) + 0x304)
#define CCU_MC_RTBR_OFFSET(ap, iface_tid)	(MVEBU_A2_BANKED_STOP_BASE(ap, \
							iface_tid) + 0x308)
#define CCU_MC_ITR_OFFSET(ap, iface_tid)	(MVEBU_A2_BANKED_STOP_BASE(ap, \
							iface_tid) + 0x314)
#define CCU_MC_RAR_OFFSET(ap, iface_tid)	(MVEBU_CCU_BASE(ap) + \
						((iface_tid == DRAM_0_TID) ? \
							0xe0 : 0xe4))
#define MC_RAR_ENABLE			1
#define MC_RAR_TID_OFFSET		4
#define MC_RAR_ADDR_MASK_OFFSET		20
#define MC_RAR_ADDR_VALUE_OFFSET	8
#define MC_INTERLEAVE_SHIFT		6

#define REMAP_ADDR_OFFSET		10
#define REMAP_ADDR_MASK			0xfffff
#define REMAP_SIZE_OFFSET		20
#define REMAP_SIZE_MASK			0xfff
#define REMAP_ENABLE_MASK		0x1

/* iface is 0 or 1 */
#define DSS_SCR_REG(ap, iface)         (MVEBU_AR_RFU_BASE(ap) + 0x208 + ((iface) * 0x4))
#define DSS_PPROT_OFFS			4
#define DSS_PPROT_MASK			0x7
#define DSS_PPROT_PRIV_SECURE_DATA	0x1

/* Extern the parameters from porting file */
extern struct mv_ddr_iface dram_iface_ap0[DDR_MAX_UNIT_PER_AP];
extern struct mv_ddr_iface dram_iface_ap1[DDR_MAX_UNIT_PER_AP];
extern struct mv_ddr_iface *ptr_iface;

/* Use global varibale to check if i2c initialization done */
int i2c_init_done = 0;

int plat_dram_ap_ifaces_get(int ap_id, struct mv_ddr_iface **ifaces, uint32_t *size)
{
	/* For now support DRAM on AP0/AP1 - TODO: add support for all APs */
	if (ap_id == 0) {
		*size = sizeof(dram_iface_ap0)/sizeof(dram_iface_ap0[0]);
		*ifaces = dram_iface_ap0;
	} else if (ap_id == 1) {
		*size = sizeof(dram_iface_ap1)/sizeof(dram_iface_ap1[0]);
		*ifaces = dram_iface_ap1;
	} else {
		*ifaces = NULL;
		*size = 0;
	}


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

void plat_dram_freq_update(enum ddr_freq freq_option)
{
	struct mv_ddr_iface *iface = NULL;
	uint32_t ifaces_size, i, ap_id;

	/* Update DDR topology for all APs for all interfaces */
	for (ap_id = 0; ap_id < get_ap_count(); ap_id++) {
		plat_dram_ap_ifaces_get(ap_id, &iface, &ifaces_size);
		for (i = 0; i < ifaces_size; i++, iface++)
			iface->tm.interface_params[0].memory_freq = freq_option;
	}
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

/* Setup RAR interleave value and enable RAR mode (data striping) for AP DRAM */
static void plat_dram_rar_mode_set(uint32_t ap_id)
{
	uint32_t val;
	uint32_t interleave = dram_rar_interleave() >> MC_INTERLEAVE_SHIFT;

	/* Enable ITR for the DDR interface */
	mmio_write_32(CCU_MC_ITR_OFFSET(ap_id, DRAM_0_TID),
		      interleave);
	mmio_write_32(CCU_MC_ITR_OFFSET(ap_id, DRAM_1_TID),
		      interleave);
	/* Configure RAR registers:
	 * For RAR 0: mask = interleave, value = 0x0, target = 0x3, enable =0x1.
	 * For RAR 1: mask = interleave, value = interleave, target = 0x8, enable =0x1
	 */
	val = interleave << MC_RAR_ADDR_MASK_OFFSET;
	val |= (DRAM_0_TID << MC_RAR_TID_OFFSET) | MC_RAR_ENABLE;
	mmio_write_32(CCU_MC_RAR_OFFSET(ap_id, DRAM_0_TID), val);

	val = interleave << MC_RAR_ADDR_MASK_OFFSET;
	val |= interleave << MC_RAR_ADDR_VALUE_OFFSET |
			(DRAM_1_TID << MC_RAR_TID_OFFSET) | MC_RAR_ENABLE;
	mmio_write_32(CCU_MC_RAR_OFFSET(ap_id, DRAM_1_TID), val);
}

/* Remap Physical address range to Memory Controller addrress range (PA->MCA) */
void plat_dram_mca_remap(int ap_index, int dram_tgt, uint64_t from, uint64_t to, uint64_t size)
{
	int dram_if[] = { -1, -1 };
	int if_idx;

	if (dram_tgt == RAR_TID) {
		dram_if[0] = DRAM_0_TID;
		dram_if[1] = DRAM_1_TID;
	} else {
		dram_if[0] = dram_tgt;
	}

	/* Size should be non-zero, up to 4GB and multiple of 1MB */
	if (!size || (size >> 32) || (size % (1 << 20))) {
		ERROR("Invalid remap size %lx\n", size);
		return;
	}

	/* Remap addresses must be multiple of remap size */
	if ((from % size) || (to % size)) {
		ERROR("Invalid remap address %lx -> %lx\n", from, to);
		return;
	}

	from >>= 20; /* bit[39:20] */
	to   >>= 20; /* bit[39:20] */
	size >>= 20; /* Size is in 1MB chunks */
	for (if_idx = 0; if_idx < DDR_MAX_UNIT_PER_AP; if_idx++) {
		uint32_t val;

		if (dram_if[if_idx] == -1)
			break;
		/* set mc remap source base to the top of dram */
		val = (from & REMAP_ADDR_MASK) << REMAP_ADDR_OFFSET;
		mmio_write_32(CCU_MC_RSBR_OFFSET(ap_index, dram_if[if_idx]), val);

		/* set mc remap target base to the overlapped dram region */
		val = (to & REMAP_ADDR_MASK) << REMAP_ADDR_OFFSET;
		mmio_write_32(CCU_MC_RTBR_OFFSET(ap_index, dram_if[if_idx]), val);

		/* set mc remap size to the size of the overlapped dram region */
		/* up to 4GB region for remapping */
		val = ((size - 1) & REMAP_SIZE_MASK) << REMAP_SIZE_OFFSET;
		/* enable remapping */
		val |= REMAP_ENABLE_MASK;
		mmio_write_32(CCU_MC_RCR_OFFSET(ap_index, dram_if[if_idx]), val);
	}
}

int plat_dram_init(void)
{
	struct mv_ddr_iface *iface = NULL;
	uint32_t ifaces_size, i, ap_id, ret, iface_cnt;
	uint64_t ap_dram_size;
	uint32_t ap_dram_tgt = DRAM_0_TID;

	/* Go over the interfaces, and update the topology */
	for (ap_id = 0; ap_id < get_ap_count(); ap_id++) {
		iface_cnt = 0;
		/* Get interfaces of AP-ID */
		plat_dram_ap_ifaces_get(ap_id, &iface, &ifaces_size);
		/* Go over the interfaces of AP and initialize them */
		for (i = 0; i < ifaces_size; i++, iface++) {
			/* Skip if not exist */
			if (iface->state == MV_DDR_IFACE_DNE)
				continue;
			/* Update AP base address */
			iface->ap_base = MVEBU_REGS_BASE_AP(ap_id);
			/* Initialize iface mode with single interface */
			iface->iface_mode = MV_DDR_RAR_DIS;
			/* Update base address of interface */
			iface->iface_base_addr = AP_DRAM_BASE_ADDR(ap_id, get_ap_count());
			/* Update DRAM topology (scan DIMM SPDs) */
			plat_dram_update_topology(&iface->tm);
			/* Count number of interfaces are ready */
			iface_cnt++;
		}
		/* If the number of interfaces equal to MAX (enable RAR) */
		if (iface_cnt == DDR_MAX_UNIT_PER_AP) {
			/* Get interfaces of AP-ID */
			plat_dram_ap_ifaces_get(ap_id, &iface, &ifaces_size);
			/* Go over the interfaces of AP and initialize them */
			for (i = 0; i < ifaces_size; i++, iface++) {
				iface->iface_mode = MV_DDR_RAR_ENA;
				/* If the base address not 0x0, need to divide
				** the base address, the dram region will be
				** splited into dual DRAMs
				** */
				iface->iface_base_addr >>= 1;
				/* TODO: add EERATA */
				if (iface->id == 1)
					iface->iface_base_addr |= 1UL << 43;
			}
		}
	}

	for (ap_id = 0; ap_id < get_ap_count(); ap_id++) {
		ap_dram_size = 0;
		/* Get interfaces of AP-ID */
		plat_dram_ap_ifaces_get(ap_id, &iface, &ifaces_size);
		/* Go over the interfaces of AP and initialize them */
		for (i = 0; i < ifaces_size; i++, iface++) {
			if (iface->state == MV_DDR_IFACE_DNE)
				continue;
			/* Set the pointer to current interface */
			plat_dram_iface_set(iface);

			/* Set phy accesses */
			plat_dram_phy_access_config(ap_id, iface->id);

			/* Call DRAM init per interface */
			ret = dram_init();
			if (ret)
				return ret;

			/* Update status of interface */
			iface->state = MV_DDR_IFACE_RDY;

			ap_dram_size += iface->iface_byte_size;
			if (iface->iface_mode == MV_DDR_RAR_ENA)
				ap_dram_tgt = RAR_TID;
			else if (iface->id == 1)
				ap_dram_tgt = DRAM_1_TID;
			else
				ap_dram_tgt = DRAM_0_TID;
		}
		/* Remap the physical memory shadowed by the internal registers configration
		 * address space to the top of the detected memory area.
		 * Only the AP0 overlaps this configuration area with the DRAM, so only its memory
		 * controller has to remap the overlapped region to the upper memory.
		 * With less than 3GB of DRAM the internal registers space remapping is not needed
		 * since there is no overlap between DRAM and the configuration address spaces
		 */
		if ((ap_id == 0)  && (ap_dram_size > (3 * _1GB_)))
			plat_dram_mca_remap(0, ap_dram_tgt, ap_dram_size, 3 * _1GB_, _1GB_);

		if (ap_dram_tgt == RAR_TID)
			plat_dram_rar_mode_set(ap_id);
	}

	return 0;
}

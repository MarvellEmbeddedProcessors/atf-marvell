/*
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <debug.h>
#include <delay_timer.h>
#include <errno.h>
#include <mmio.h>
#include <plat_def.h>
#include <spinlock.h>
#include "mvebu.h"
#include "comphy-cp110.h"

/* #define DEBUG_COMPHY */
#ifdef DEBUG_COMPHY
#define debug(format...) NOTICE(format)
#else
#define debug(format, arg...)
#endif

/* A lane is described by 4 fields:
 *      - bit 1~0 represent comphy polarity invert
 *      - bit 7~2 represent comphy speed
 *      - bit 11~8 represent unit index
 *      - bit 16~12 represent mode
 *      - bit 17 represent comphy indication of clock source
 *      - bit 19-18 represents pcie width (in case of pcie comphy config.)
 *      - bit 31~20 reserved
 */

#define COMPHY_INVERT_OFFSET	0
#define COMPHY_INVERT_LEN	2
#define COMPHY_INVERT_MASK	COMPHY_MASK(COMPHY_INVERT_OFFSET, COMPHY_INVERT_LEN)
#define COMPHY_SPEED_OFFSET	(COMPHY_INVERT_OFFSET + COMPHY_INVERT_LEN)
#define COMPHY_SPEED_LEN	6
#define COMPHY_SPEED_MASK	COMPHY_MASK(COMPHY_SPEED_OFFSET, COMPHY_SPEED_LEN)
#define COMPHY_UNIT_ID_OFFSET	(COMPHY_SPEED_OFFSET + COMPHY_SPEED_LEN)
#define COMPHY_UNIT_ID_LEN	4
#define COMPHY_UNIT_ID_MASK	COMPHY_MASK(COMPHY_UNIT_ID_OFFSET, COMPHY_UNIT_ID_LEN)
#define COMPHY_MODE_OFFSET	(COMPHY_UNIT_ID_OFFSET + COMPHY_UNIT_ID_LEN)
#define COMPHY_MODE_LEN		5
#define COMPHY_MODE_MASK	COMPHY_MASK(COMPHY_MODE_OFFSET, COMPHY_MODE_LEN)
#define COMPHY_CLK_SRC_OFFSET	(COMPHY_MODE_OFFSET + COMPHY_MODE_LEN)
#define COMPHY_CLK_SRC_LEN	1
#define COMPHY_CLK_SRC_MASK	COMPHY_MASK(COMPHY_CLK_SRC_OFFSET, COMPHY_CLK_SRC_LEN)
#define COMPHY_PCI_WIDTH_OFFSET	(COMPHY_CLK_SRC_OFFSET + COMPHY_CLK_SRC_LEN)
#define COMPHY_PCI_WIDTH_LEN	3
#define COMPHY_PCI_WIDTH_MASK	COMPHY_MASK(COMPHY_PCI_WIDTH_OFFSET, COMPHY_PCI_WIDTH_LEN)

#define COMPHY_MASK(offset, len)	(((1 << (len)) - 1) << (offset))

/* Macro which extracts mode from lane description */
#define COMPHY_GET_MODE(x)		(((x) & COMPHY_MODE_MASK) >> COMPHY_MODE_OFFSET)
/* Macro which extracts unit index from lane description */
#define COMPHY_GET_ID(x)		(((x) & COMPHY_UNIT_ID_MASK) >> COMPHY_UNIT_ID_OFFSET)
/* Macro which extracts speed from lane description */
#define COMPHY_GET_SPEED(x)		(((x) & COMPHY_SPEED_MASK) >> COMPHY_SPEED_OFFSET)
/* Macro which extracts clock source indication from lane description */
#define COMPHY_GET_CLK_SRC(x)		(((x) & COMPHY_CLK_SRC_MASK) >> COMPHY_CLK_SRC_OFFSET)
/* Macro which extracts pcie width indication from lane description */
#define COMPHY_GET_PCIE_WIDTH(x)	(((x) & COMPHY_PCI_WIDTH_MASK) >> COMPHY_PCI_WIDTH_OFFSET)

#define COMPHY_SATA_MODE	0x1
#define COMPHY_SGMII_MODE	0x2	/* SGMII 1G */
#define COMPHY_HS_SGMII_MODE	0x3	/* SGMII 2.5G */
#define COMPHY_USB3H_MODE	0x4
#define COMPHY_USB3D_MODE	0x5
#define COMPHY_PCIE_MODE	0x6
#define COMPHY_RXAUI_MODE	0x7
#define COMPHY_XFI_MODE		0x8
#define COMPHY_SFI_MODE		0x9
#define COMPHY_USB3_MODE	0xa
#define COMPHY_AP_MODE		0xb

/* COMPHY speed macro */
#define COMPHY_SPEED_1_25G		0 /* SGMII 1G */
#define COMPHY_SPEED_2_5G		1
#define COMPHY_SPEED_3_125G		2 /* SGMII 2.5G */
#define COMPHY_SPEED_5G			3
#define COMPHY_SPEED_5_15625G		4 /* XFI 5G */
#define COMPHY_SPEED_6G			5
#define COMPHY_SPEED_10_3125G		6 /* XFI 10G */
#define COMPHY_SPEED_MAX		0x3F
/* The  default speed for IO with fixed known speed */
#define COMPHY_SPEED_DEFAULT		COMPHY_SPEED_MAX

#define COMPHY_PIPE_FROM_COMPHY_ADDR(x)	((x & ~0xffffff) + 0x120000)

/* System controler registers */
#define PCIE_MAC_RESET_MASK_PORT0	BIT(13)
#define PCIE_MAC_RESET_MASK_PORT1	BIT(11)
#define PCIE_MAC_RESET_MASK_PORT2	BIT(12)
#define SYS_CTRL_UINIT_SOFT_RESET_REG	0x268
#define SYS_CTRL_FROM_COMPHY_ADDR(x)	((x & ~0xffffff) + 0x440000)

/* DFX register spaces */
#define SAR_RST_PCIE0_CLOCK_CONFIG_CP1_OFFSET	(0)
#define SAR_RST_PCIE0_CLOCK_CONFIG_CP1_MASK	(0x1 << SAR_RST_PCIE0_CLOCK_CONFIG_CP1_OFFSET)
#define SAR_RST_PCIE1_CLOCK_CONFIG_CP1_OFFSET	(1)
#define SAR_RST_PCIE1_CLOCK_CONFIG_CP1_MASK	(0x1 << SAR_RST_PCIE1_CLOCK_CONFIG_CP1_OFFSET)
#define SAR_STATUS_0_REG			200
#define DFX_FROM_COMPHY_ADDR(x)			((x & ~0xffffff) + DFX_BASE)

/* The same Units Soft Reset Config register are accessed in all PCIe ports
 * initialization, so a spin lock is defined in case when more than 1 CPUs
 * resets PCIe MAC and need to access the register in the same time. The spin
 * lock is shared by all CP110 units.
 */
spinlock_t cp110_mac_reset_lock;

enum reg_width_type {
	REG_16BIT = 0,
	REG_32BIT,
};

enum {
	COMPHY_LANE0 = 0,
	COMPHY_LANE1,
	COMPHY_LANE2,
	COMPHY_LANE3,
	COMPHY_LANE4,
	COMPHY_LANE5,
	COMPHY_LANE_MAX,
};

/* These values come from the PCI Express Spec */
enum pcie_link_width {
	PCIE_LNK_WIDTH_RESRV	= 0x00,
	PCIE_LNK_X1		= 0x01,
	PCIE_LNK_X2		= 0x02,
	PCIE_LNK_X4		= 0x04,
	PCIE_LNK_X8		= 0x08,
	PCIE_LNK_X12		= 0x0C,
	PCIE_LNK_X16		= 0x10,
	PCIE_LNK_X32		= 0x20,
	PCIE_LNK_WIDTH_UNKNOWN  = 0xFF,
};

static inline uint32_t polling_with_timeout(uintptr_t addr, uint32_t val,
		  uint32_t mask, uint32_t usec_timout, enum reg_width_type type)
{
	uint32_t data;

	do {
		udelay(1);
		if (type == REG_16BIT)
			data = mmio_read_16(addr) & mask;
		else
			data = mmio_read_32(addr) & mask;
	} while (data != val  && --usec_timout > 0);

	if (usec_timout == 0)
		return data;

	return 0;
}

static inline void reg_set(uintptr_t addr, uint32_t data, uint32_t mask)
{
	debug("Write to addr = 0x%lx, data = 0x%x (mask = 0x%x) - old val = 0x%x",
	      addr, data, mask, mmio_read_32(addr));
	mmio_clrsetbits_32(addr, mask, data);

	debug("new val 0x%x\n", mmio_read_32(addr));
}

/* Clear PHY selector - avoid collision with previous configuration */
static void mvebu_cp110_comphy_clr_phy_selector(uint64_t comphy_base, uint8_t comphy_index)
{
	uint32_t reg, mask, field;
	uint32_t comphy_offset = COMMON_SELECTOR_COMPHYN_FIELD_WIDTH * comphy_index;

	mask = COMMON_SELECTOR_COMPHY_MASK << comphy_offset;
	reg = mmio_read_32(comphy_base + COMMON_SELECTOR_PHY_REG_OFFSET);
	field = reg & mask;

	/* Clear comphy selector - if it was already configured.
	 * (might be that this comphy was configured as PCIe/USB,
	 * in such case, no need to clear comphy selector because PCIe/USB
	 * are controlled by hpipe selector).
	 */
	if (field) {
		reg &= ~mask;
		mmio_write_32(comphy_base + COMMON_SELECTOR_PHY_REG_OFFSET, reg);
	}
}

/* PHY selector configures SATA and Network modes */
static void mvebu_cp110_comphy_set_phy_selector(uint64_t comphy_base,
				     uint8_t comphy_index, uint32_t comphy_mode)
{
	uint32_t reg, mask;
	uint32_t comphy_offset = COMMON_SELECTOR_COMPHYN_FIELD_WIDTH * comphy_index;
	int mode;

	/* Comphy mode (compound of the IO mode and id). Here, only the IO mode
	 * is required to distinguish between SATA and network modes.
	 */
	mode = COMPHY_GET_MODE(comphy_mode);

	mask = COMMON_SELECTOR_COMPHY_MASK << comphy_offset;
	reg = mmio_read_32(comphy_base + COMMON_SELECTOR_PHY_REG_OFFSET);
	reg &= ~mask;

	/* SATA port 0/1 require the same configuration */
	if (mode == COMPHY_SATA_MODE) {
		/* SATA selector values is always 4 */
		reg |= COMMON_SELECTOR_COMPHYN_SATA << comphy_offset;
	} else {
		switch (comphy_index) {
		case(0):
		case(1):
		case(2):
			/* For comphy 0,1, and 2:
			 * Network selector value is always 1.
			 */
			reg |= COMMON_SELECTOR_COMPHY0_1_2_NETWORK << comphy_offset;
			break;
		case(3):
			/* For comphy 3:
			 * 0x1 = RXAUI_Lane1
			 * 0x2 = SGMII/HS-SGMII Port1
			 */
			if (mode == COMPHY_RXAUI_MODE)
				reg |= COMMON_SELECTOR_COMPHY3_RXAUI << comphy_offset;
			else
				reg |= COMMON_SELECTOR_COMPHY3_SGMII << comphy_offset;
			break;
		case(4):
			 /* For comphy 4:
			  * 0x1 = SGMII/HS-SGMII Port2
			  * 0x2 = SGMII/HS-SGMII Port0: XFI/SFI, RXAUI_Lane0
			  *
			  * We want to check if SGMII1/HS_SGMII1 is the requested mode in order to
			  * determine which value should be set (all other modes use the same value)
			  * so we need to strip the mode, and check the ID because we might handle
			  * SGMII0/HS_SGMII0 too.
			  */
			if ((mode == COMPHY_SGMII_MODE || mode == COMPHY_HS_SGMII_MODE) &&
			    COMPHY_GET_ID(comphy_mode) == 2)
				reg |= COMMON_SELECTOR_COMPHY4_SGMII2 << comphy_offset;
			else
				reg |= COMMON_SELECTOR_COMPHY4_ALL_OTHERS << comphy_offset;
			break;
		case(5):
			/* For comphy 5:
			 * 0x1 = SGMII/HS-SGMII Port2
			 * 0x2 = RXAUI Lane1
			 */
			if (mode == COMPHY_RXAUI_MODE)
				reg |= COMMON_SELECTOR_COMPHY5_RXAUI << comphy_offset;
			else
				reg |= COMMON_SELECTOR_COMPHY5_SGMII << comphy_offset;
			break;
		}
	}

	mmio_write_32(comphy_base + COMMON_SELECTOR_PHY_REG_OFFSET, reg);
}

/* Clear PIPE selector - avoid collision with previous configuration */
void mvebu_cp110_comphy_clr_pipe_selector(uint64_t comphy_base, uint8_t comphy_index)
{
	uint32_t reg, mask, field;
	uint32_t comphy_offset = COMMON_SELECTOR_COMPHYN_FIELD_WIDTH * comphy_index;

	mask = COMMON_SELECTOR_COMPHY_MASK << comphy_offset;
	reg = mmio_read_32(comphy_base + COMMON_SELECTOR_PIPE_REG_OFFSET);
	field = reg & mask;

	if (field) {
		reg &= ~mask;
		mmio_write_32(comphy_base + COMMON_SELECTOR_PIPE_REG_OFFSET, reg);
	}
}

/* PIPE selector configures for PCIe, USB 3.0 Host, and USB 3.0 Device mode */
static void mvebu_cp110_comphy_set_pipe_selector(uint64_t comphy_base,
				     uint8_t comphy_index, uint32_t comphy_mode)
{
	uint32_t reg;
	uint32_t shift = COMMON_SELECTOR_COMPHYN_FIELD_WIDTH * comphy_index;
	int mode = COMPHY_GET_MODE(comphy_mode);
	uint32_t mask = COMMON_SELECTOR_COMPHY_MASK << shift;
	uint32_t pipe_sel = 0x0;

	reg = mmio_read_32(comphy_base + COMMON_SELECTOR_PIPE_REG_OFFSET);
	reg &= ~mask;

	switch (mode) {
	case (COMPHY_PCIE_MODE):
		/* For lanes support PCIE, selector value are all same */
		pipe_sel = COMMON_SELECTOR_PIPE_COMPHY_PCIE;
		break;

	case (COMPHY_USB3H_MODE):
		/* Only lane 1-4 support USB host, selector value is same */
		if (comphy_index == COMPHY_LANE0 ||
		    comphy_index == COMPHY_LANE5)
			ERROR("COMPHY[%d] mode[%d] is invalid\n",
			      comphy_index, mode);
		else
			pipe_sel = COMMON_SELECTOR_PIPE_COMPHY_USBH;
		break;

	case (COMPHY_USB3D_MODE):
		/* Lane 1 and 4 support USB device, selector value is same */
		if (comphy_index == COMPHY_LANE1 ||
		    comphy_index == COMPHY_LANE4)
			pipe_sel = COMMON_SELECTOR_PIPE_COMPHY_USBD;
		else
			ERROR("COMPHY[%d] mode[%d] is invalid\n", comphy_index,
			      mode);
		break;

	default:
		ERROR("COMPHY[%d] mode[%d] is invalid\n", comphy_index, mode);
		break;
	}

	mmio_write_32(comphy_base + COMMON_SELECTOR_PIPE_REG_OFFSET, reg |
		      (pipe_sel << shift));
}

int mvebu_cp110_comphy_is_pll_locked(uint64_t comphy_base, uint8_t comphy_index)
{
	uintptr_t sd_ip_addr, addr;
	uint32_t mask, data;
	int ret = 0;

	debug_enter();

	sd_ip_addr = SD_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base), comphy_index);

	addr = sd_ip_addr + SD_EXTERNAL_STATUS0_REG;
	data = SD_EXTERNAL_STATUS0_PLL_TX_MASK &
		SD_EXTERNAL_STATUS0_PLL_RX_MASK;
	mask = data;
	data = polling_with_timeout(addr, data, mask, PLL_LOCK_TIMEOUT, REG_32BIT);
	if (data != 0) {
		if (data & SD_EXTERNAL_STATUS0_PLL_RX_MASK)
			ERROR("RX PLL is not locked\n");
		if (data & SD_EXTERNAL_STATUS0_PLL_TX_MASK)
			ERROR("TX PLL is not locked\n");

		ret = -ETIMEDOUT;
	}

	debug_exit();

	return ret;
}

static int mvebu_cp110_comphy_sata_power_on(uint64_t comphy_base,
				     uint8_t comphy_index, uint32_t comphy_mode)
{
	uintptr_t hpipe_addr, sd_ip_addr, comphy_addr;
	uint32_t mask, data;
	int ret = 0;

	debug_enter();

	/* configure phy selector for SATA */
	mvebu_cp110_comphy_set_phy_selector(comphy_base, comphy_index, comphy_mode);

	hpipe_addr = HPIPE_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base), comphy_index);
	sd_ip_addr = SD_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base), comphy_index);
	comphy_addr = COMPHY_ADDR(comphy_base, comphy_index);

	debug(" add hpipe 0x%lx, sd 0x%lx, comphy 0x%lx\n",
					   hpipe_addr, sd_ip_addr, comphy_addr);
	debug("stage: RFU configurations - hard reset comphy\n");
	/* RFU configurations - hard reset comphy */
	mask = COMMON_PHY_CFG1_PWR_UP_MASK;
	data = 0x1 << COMMON_PHY_CFG1_PWR_UP_OFFSET;
	mask |= COMMON_PHY_CFG1_PIPE_SELECT_MASK;
	data |= 0x0 << COMMON_PHY_CFG1_PIPE_SELECT_OFFSET;
	mask |= COMMON_PHY_CFG1_PWR_ON_RESET_MASK;
	data |= 0x0 << COMMON_PHY_CFG1_PWR_ON_RESET_OFFSET;
	mask |= COMMON_PHY_CFG1_CORE_RSTN_MASK;
	data |= 0x0 << COMMON_PHY_CFG1_CORE_RSTN_OFFSET;
	reg_set(comphy_addr + COMMON_PHY_CFG1_REG, data, mask);

	/* Set select data  width 40Bit - SATA mode only */
	reg_set(comphy_addr + COMMON_PHY_CFG6_REG,
		0x1 << COMMON_PHY_CFG6_IF_40_SEL_OFFSET, COMMON_PHY_CFG6_IF_40_SEL_MASK);

	/* release from hard reset in SD external */
	mask = SD_EXTERNAL_CONFIG1_RESET_IN_MASK;
	data = 0x1 << SD_EXTERNAL_CONFIG1_RESET_IN_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RESET_CORE_MASK;
	data |= 0x1 << SD_EXTERNAL_CONFIG1_RESET_CORE_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG1_REG, data, mask);

	/* Wait 1ms - until band gap and ref clock ready */
	mdelay(1);

	debug("stage: Comphy configuration\n");
	/* Start comphy Configuration */
	/* Set reference clock to comes from group 1 - choose 25Mhz */
	reg_set(hpipe_addr + HPIPE_MISC_REG,
		0x0 << HPIPE_MISC_REFCLK_SEL_OFFSET, HPIPE_MISC_REFCLK_SEL_MASK);
	/* Reference frequency select set 1 (for SATA = 25Mhz) */
	mask = HPIPE_PWR_PLL_REF_FREQ_MASK;
	data = 0x1 << HPIPE_PWR_PLL_REF_FREQ_OFFSET;
	/* PHY mode select (set SATA = 0x0 */
	mask |= HPIPE_PWR_PLL_PHY_MODE_MASK;
	data |= 0x0 << HPIPE_PWR_PLL_PHY_MODE_OFFSET;
	reg_set(hpipe_addr + HPIPE_PWR_PLL_REG, data, mask);
	/* Set max PHY generation setting - 6Gbps */
	reg_set(hpipe_addr + HPIPE_INTERFACE_REG,
		0x2 << HPIPE_INTERFACE_GEN_MAX_OFFSET, HPIPE_INTERFACE_GEN_MAX_MASK);
	/* Set select data  width 40Bit (SEL_BITS[2:0]) */
	reg_set(hpipe_addr + HPIPE_LOOPBACK_REG,
		0x2 << HPIPE_LOOPBACK_SEL_OFFSET, HPIPE_LOOPBACK_SEL_MASK);

	debug("stage: Analog parameters from ETP(HW)\n");
	/* G1 settings */
	mask = HPIPE_G1_SET_1_G1_RX_SELMUPI_MASK;
	data = 0x0 << HPIPE_G1_SET_1_G1_RX_SELMUPI_OFFSET;
	mask |= HPIPE_G1_SET_1_G1_RX_SELMUPP_MASK;
	data |= 0x1 << HPIPE_G1_SET_1_G1_RX_SELMUPP_OFFSET;
	mask |= HPIPE_G1_SET_1_G1_RX_SELMUFI_MASK;
	data |= 0x0 << HPIPE_G1_SET_1_G1_RX_SELMUFI_OFFSET;
	mask |= HPIPE_G1_SET_1_G1_RX_SELMUFF_MASK;
	data |= 0x3 << HPIPE_G1_SET_1_G1_RX_SELMUFF_OFFSET;
	mask |= HPIPE_G1_SET_1_G1_RX_DIGCK_DIV_MASK;
	data |= 0x1 << HPIPE_G1_SET_1_G1_RX_DIGCK_DIV_OFFSET;
	reg_set(hpipe_addr + HPIPE_G1_SET_1_REG, data, mask);

	mask = HPIPE_G1_SETTINGS_3_G1_FFE_CAP_SEL_MASK;
	data = 0xf << HPIPE_G1_SETTINGS_3_G1_FFE_CAP_SEL_OFFSET;
	mask |= HPIPE_G1_SETTINGS_3_G1_FFE_RES_SEL_MASK;
	data |= 0x2 << HPIPE_G1_SETTINGS_3_G1_FFE_RES_SEL_OFFSET;
	mask |= HPIPE_G1_SETTINGS_3_G1_FFE_SETTING_FORCE_MASK;
	data |= 0x1 << HPIPE_G1_SETTINGS_3_G1_FFE_SETTING_FORCE_OFFSET;
	mask |= HPIPE_G1_SETTINGS_3_G1_FFE_DEG_RES_LEVEL_MASK;
	data |= 0x1 << HPIPE_G1_SETTINGS_3_G1_FFE_DEG_RES_LEVEL_OFFSET;
	mask |= HPIPE_G1_SETTINGS_3_G1_FFE_LOAD_RES_LEVEL_MASK;
	data |= 0x1 << HPIPE_G1_SETTINGS_3_G1_FFE_LOAD_RES_LEVEL_OFFSET;
	reg_set(hpipe_addr + HPIPE_G1_SETTINGS_3_REG, data, mask);

	/* G2 settings */
	mask = HPIPE_G2_SET_1_G2_RX_SELMUPI_MASK;
	data = 0x0 << HPIPE_G2_SET_1_G2_RX_SELMUPI_OFFSET;
	mask |= HPIPE_G2_SET_1_G2_RX_SELMUPP_MASK;
	data |= 0x1 << HPIPE_G2_SET_1_G2_RX_SELMUPP_OFFSET;
	mask |= HPIPE_G2_SET_1_G2_RX_SELMUFI_MASK;
	data |= 0x0 << HPIPE_G2_SET_1_G2_RX_SELMUFI_OFFSET;
	mask |= HPIPE_G2_SET_1_G2_RX_SELMUFF_MASK;
	data |= 0x3 << HPIPE_G2_SET_1_G2_RX_SELMUFF_OFFSET;
	mask |= HPIPE_G2_SET_1_G2_RX_DIGCK_DIV_MASK;
	data |= 0x1 << HPIPE_G2_SET_1_G2_RX_DIGCK_DIV_OFFSET;
	reg_set(hpipe_addr + HPIPE_G2_SET_1_REG, data, mask);

	/* G3 settings */
	mask = HPIPE_G3_SET_1_G3_RX_SELMUPI_MASK;
	data = 0x2 << HPIPE_G3_SET_1_G3_RX_SELMUPI_OFFSET;
	mask |= HPIPE_G3_SET_1_G3_RX_SELMUPF_MASK;
	data |= 0x2 << HPIPE_G3_SET_1_G3_RX_SELMUPF_OFFSET;
	mask |= HPIPE_G3_SET_1_G3_RX_SELMUFI_MASK;
	data |= 0x3 << HPIPE_G3_SET_1_G3_RX_SELMUFI_OFFSET;
	mask |= HPIPE_G3_SET_1_G3_RX_SELMUFF_MASK;
	data |= 0x3 << HPIPE_G3_SET_1_G3_RX_SELMUFF_OFFSET;
	mask |= HPIPE_G3_SET_1_G3_RX_DFE_EN_MASK;
	data |= 0x1 << HPIPE_G3_SET_1_G3_RX_DFE_EN_OFFSET;
	mask |= HPIPE_G3_SET_1_G3_RX_DIGCK_DIV_MASK;
	data |= 0x2 << HPIPE_G3_SET_1_G3_RX_DIGCK_DIV_OFFSET;
	mask |= HPIPE_G3_SET_1_G3_SAMPLER_INPAIRX2_EN_MASK;
	data |= 0x0 << HPIPE_G3_SET_1_G3_SAMPLER_INPAIRX2_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_G3_SET_1_REG, data, mask);

	/* DTL Control */
	mask = HPIPE_PWR_CTR_DTL_SQ_DET_EN_MASK;
	data = 0x1 << HPIPE_PWR_CTR_DTL_SQ_DET_EN_OFFSET;
	mask |= HPIPE_PWR_CTR_DTL_SQ_PLOOP_EN_MASK;
	data |= 0x1 << HPIPE_PWR_CTR_DTL_SQ_PLOOP_EN_OFFSET;
	mask |= HPIPE_PWR_CTR_DTL_FLOOP_EN_MASK;
	data |= 0x1 << HPIPE_PWR_CTR_DTL_FLOOP_EN_OFFSET;
	mask |= HPIPE_PWR_CTR_DTL_CLAMPING_SEL_MASK;
	data |= 0x1 << HPIPE_PWR_CTR_DTL_CLAMPING_SEL_OFFSET;
	mask |= HPIPE_PWR_CTR_DTL_INTPCLK_DIV_FORCE_MASK;
	data |= 0x1 << HPIPE_PWR_CTR_DTL_INTPCLK_DIV_FORCE_OFFSET;
	mask |= HPIPE_PWR_CTR_DTL_CLK_MODE_MASK;
	data |= 0x1 << HPIPE_PWR_CTR_DTL_CLK_MODE_OFFSET;
	mask |= HPIPE_PWR_CTR_DTL_CLK_MODE_FORCE_MASK;
	data |= 0x1 << HPIPE_PWR_CTR_DTL_CLK_MODE_FORCE_OFFSET;
	reg_set(hpipe_addr + HPIPE_PWR_CTR_DTL_REG, data, mask);

	/* Trigger sampler enable pulse */
	mask = HPIPE_SMAPLER_MASK;
	data = 0x1 << HPIPE_SMAPLER_OFFSET;
	reg_set(hpipe_addr + HPIPE_SAMPLER_N_PROC_CALIB_CTRL_REG, data, mask);
	mask = HPIPE_SMAPLER_MASK;
	data = 0x0 << HPIPE_SMAPLER_OFFSET;
	reg_set(hpipe_addr + HPIPE_SAMPLER_N_PROC_CALIB_CTRL_REG, data, mask);

	/* VDD Calibration Control 3 */
	mask = HPIPE_EXT_SELLV_RXSAMPL_MASK;
	data = 0x10 << HPIPE_EXT_SELLV_RXSAMPL_OFFSET;
	reg_set(hpipe_addr + HPIPE_VDD_CAL_CTRL_REG, data, mask);

	/* DFE Resolution Control */
	mask = HPIPE_DFE_RES_FORCE_MASK;
	data = 0x1 << HPIPE_DFE_RES_FORCE_OFFSET;
	reg_set(hpipe_addr + HPIPE_DFE_REG0, data, mask);

	/* DFE F3-F5 Coefficient Control */
	mask = HPIPE_DFE_F3_F5_DFE_EN_MASK;
	data = 0x0 << HPIPE_DFE_F3_F5_DFE_EN_OFFSET;
	mask |= HPIPE_DFE_F3_F5_DFE_CTRL_MASK;
	data = 0x0 << HPIPE_DFE_F3_F5_DFE_CTRL_OFFSET;
	reg_set(hpipe_addr + HPIPE_DFE_F3_F5_REG, data, mask);

	/* G3 Setting 3 */
	mask = HPIPE_G3_FFE_CAP_SEL_MASK;
	data = 0xf << HPIPE_G3_FFE_CAP_SEL_OFFSET;
	mask |= HPIPE_G3_FFE_RES_SEL_MASK;
	data |= 0x4 << HPIPE_G3_FFE_RES_SEL_OFFSET;
	mask |= HPIPE_G3_FFE_SETTING_FORCE_MASK;
	data |= 0x1 << HPIPE_G3_FFE_SETTING_FORCE_OFFSET;
	mask |= HPIPE_G3_FFE_DEG_RES_LEVEL_MASK;
	data |= 0x1 << HPIPE_G3_FFE_DEG_RES_LEVEL_OFFSET;
	mask |= HPIPE_G3_FFE_LOAD_RES_LEVEL_MASK;
	data |= 0x3 << HPIPE_G3_FFE_LOAD_RES_LEVEL_OFFSET;
	reg_set(hpipe_addr + HPIPE_G3_SETTING_3_REG, data, mask);

	/* G3 Setting 4 */
	mask = HPIPE_G3_DFE_RES_MASK;
	data = 0x1 << HPIPE_G3_DFE_RES_OFFSET;
	reg_set(hpipe_addr + HPIPE_G3_SETTING_4_REG, data, mask);

	/* Offset Phase Control */
	mask = HPIPE_OS_PH_OFFSET_MASK;
	data = 0x61 << HPIPE_OS_PH_OFFSET_OFFSET;
	mask |= HPIPE_OS_PH_OFFSET_FORCE_MASK;
	data |= 0x1 << HPIPE_OS_PH_OFFSET_FORCE_OFFSET;
	mask |= HPIPE_OS_PH_VALID_MASK;
	data |= 0x0 << HPIPE_OS_PH_VALID_OFFSET;
	reg_set(hpipe_addr + HPIPE_PHASE_CONTROL_REG, data, mask);
	mask = HPIPE_OS_PH_VALID_MASK;
	data = 0x1 << HPIPE_OS_PH_VALID_OFFSET;
	reg_set(hpipe_addr + HPIPE_PHASE_CONTROL_REG, data, mask);
	mask = HPIPE_OS_PH_VALID_MASK;
	data = 0x0 << HPIPE_OS_PH_VALID_OFFSET;
	reg_set(hpipe_addr + HPIPE_PHASE_CONTROL_REG, data, mask);

	/* Set G1 TX amplitude and TX post emphasis value */
	mask = HPIPE_G1_SET_0_G1_TX_AMP_MASK;
	data = 0x8 << HPIPE_G1_SET_0_G1_TX_AMP_OFFSET;
	mask |= HPIPE_G1_SET_0_G1_TX_AMP_ADJ_MASK;
	data |= 0x1 << HPIPE_G1_SET_0_G1_TX_AMP_ADJ_OFFSET;
	mask |= HPIPE_G1_SET_0_G1_TX_EMPH1_MASK;
	data |= 0x1 << HPIPE_G1_SET_0_G1_TX_EMPH1_OFFSET;
	mask |= HPIPE_G1_SET_0_G1_TX_EMPH1_EN_MASK;
	data |= 0x1 << HPIPE_G1_SET_0_G1_TX_EMPH1_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_G1_SET_0_REG, data, mask);

	/* Set G2 TX amplitude and TX post emphasis value */
	mask = HPIPE_G2_SET_0_G2_TX_AMP_MASK;
	data = 0xa << HPIPE_G2_SET_0_G2_TX_AMP_OFFSET;
	mask |= HPIPE_G2_SET_0_G2_TX_AMP_ADJ_MASK;
	data |= 0x1 << HPIPE_G2_SET_0_G2_TX_AMP_ADJ_OFFSET;
	mask |= HPIPE_G2_SET_0_G2_TX_EMPH1_MASK;
	data |= 0x2 << HPIPE_G2_SET_0_G2_TX_EMPH1_OFFSET;
	mask |= HPIPE_G2_SET_0_G2_TX_EMPH1_EN_MASK;
	data |= 0x1 << HPIPE_G2_SET_0_G2_TX_EMPH1_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_G2_SET_0_REG, data, mask);

	/* Set G3 TX amplitude and TX post emphasis value */
	mask = HPIPE_G3_SET_0_G3_TX_AMP_MASK;
	data = 0x1e << HPIPE_G3_SET_0_G3_TX_AMP_OFFSET;
	mask |= HPIPE_G3_SET_0_G3_TX_AMP_ADJ_MASK;
	data |= 0x1 << HPIPE_G3_SET_0_G3_TX_AMP_ADJ_OFFSET;
	mask |= HPIPE_G3_SET_0_G3_TX_EMPH1_MASK;
	data |= 0xe << HPIPE_G3_SET_0_G3_TX_EMPH1_OFFSET;
	mask |= HPIPE_G3_SET_0_G3_TX_EMPH1_EN_MASK;
	data |= 0x1 << HPIPE_G3_SET_0_G3_TX_EMPH1_EN_OFFSET;
	mask |= HPIPE_G3_SET_0_G3_TX_SLEW_RATE_SEL_MASK;
	data |= 0x4 << HPIPE_G3_SET_0_G3_TX_SLEW_RATE_SEL_OFFSET;
	mask |= HPIPE_G3_SET_0_G3_TX_SLEW_CTRL_EN_MASK;
	data |= 0x0 << HPIPE_G3_SET_0_G3_TX_SLEW_CTRL_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_G3_SET_0_REG, data, mask);

	/* SERDES External Configuration 2 register */
	mask = SD_EXTERNAL_CONFIG2_SSC_ENABLE_MASK;
	data = 0x1 << SD_EXTERNAL_CONFIG2_SSC_ENABLE_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG2_REG, data, mask);

	/* DFE reset sequence */
	reg_set(hpipe_addr + HPIPE_PWR_CTR_REG,
		0x1 << HPIPE_PWR_CTR_RST_DFE_OFFSET, HPIPE_PWR_CTR_RST_DFE_MASK);
	reg_set(hpipe_addr + HPIPE_PWR_CTR_REG,
		0x0 << HPIPE_PWR_CTR_RST_DFE_OFFSET, HPIPE_PWR_CTR_RST_DFE_MASK);
	/* SW reset for interrupt logic */
	reg_set(hpipe_addr + HPIPE_PWR_CTR_REG,
		0x1 << HPIPE_PWR_CTR_SFT_RST_OFFSET, HPIPE_PWR_CTR_SFT_RST_MASK);
	reg_set(hpipe_addr + HPIPE_PWR_CTR_REG,
		0x0 << HPIPE_PWR_CTR_SFT_RST_OFFSET, HPIPE_PWR_CTR_SFT_RST_MASK);

	debug_exit();

	return ret;
}

static int mvebu_cp110_comphy_sgmii_power_on(uint64_t comphy_base,
				     uint8_t comphy_index, uint32_t comphy_mode)
{
	uintptr_t hpipe_addr, sd_ip_addr, comphy_addr, addr;
	uint32_t mask, data, sgmii_speed = COMPHY_GET_SPEED(comphy_mode);
	int ret = 0;

	debug_enter();

	hpipe_addr = HPIPE_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base), comphy_index);
	sd_ip_addr = SD_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base), comphy_index);
	comphy_addr = COMPHY_ADDR(comphy_base, comphy_index);

	/* configure phy selector for SGMII */
	mvebu_cp110_comphy_set_phy_selector(comphy_base, comphy_index, comphy_mode);

	/* Confiugre the lane */
	debug("stage: RFU configurations - hard reset comphy\n");
	/* RFU configurations - hard reset comphy */
	mask = COMMON_PHY_CFG1_PWR_UP_MASK;
	data = 0x1 << COMMON_PHY_CFG1_PWR_UP_OFFSET;
	mask |= COMMON_PHY_CFG1_PIPE_SELECT_MASK;
	data |= 0x0 << COMMON_PHY_CFG1_PIPE_SELECT_OFFSET;
	reg_set(comphy_addr + COMMON_PHY_CFG1_REG, data, mask);

	/* Select Baud Rate of Comphy And PD_PLL/Tx/Rx */
	mask = SD_EXTERNAL_CONFIG0_SD_PU_PLL_MASK;
	data = 0x0 << SD_EXTERNAL_CONFIG0_SD_PU_PLL_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_SD_PHY_GEN_RX_MASK;
	mask |= SD_EXTERNAL_CONFIG0_SD_PHY_GEN_TX_MASK;

	if (sgmii_speed == COMPHY_SPEED_1_25G) {
		/* SGMII 1G, SerDes speed 1.25G */
		data |= 0x6 << SD_EXTERNAL_CONFIG0_SD_PHY_GEN_RX_OFFSET;
		data |= 0x6 << SD_EXTERNAL_CONFIG0_SD_PHY_GEN_TX_OFFSET;
	} else if (sgmii_speed == COMPHY_SPEED_3_125G) {
		/* HS SGMII (2.5G), SerDes speed 3.125G */
		data |= 0x8 << SD_EXTERNAL_CONFIG0_SD_PHY_GEN_RX_OFFSET;
		data |= 0x8 << SD_EXTERNAL_CONFIG0_SD_PHY_GEN_TX_OFFSET;
	} else {
		/* Other rates are not supported */
		ERROR("unsupported SGMII speed on comphy%d\n", comphy_index);
		return -EINVAL;
	}

	mask |= SD_EXTERNAL_CONFIG0_SD_PU_RX_MASK;
	data |= 0 << SD_EXTERNAL_CONFIG0_SD_PU_RX_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_SD_PU_TX_MASK;
	data |= 0 << SD_EXTERNAL_CONFIG0_SD_PU_TX_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_HALF_BUS_MODE_MASK;
	data |= 1 << SD_EXTERNAL_CONFIG0_HALF_BUS_MODE_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG0_REG, data, mask);

	/* Set hard reset */
	mask = SD_EXTERNAL_CONFIG1_RESET_IN_MASK;
	data = 0x0 << SD_EXTERNAL_CONFIG1_RESET_IN_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RESET_CORE_MASK;
	data |= 0x0 << SD_EXTERNAL_CONFIG1_RESET_CORE_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RF_RESET_IN_MASK;
	data |= 0x0 << SD_EXTERNAL_CONFIG1_RF_RESET_IN_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG1_REG, data, mask);

	/* Release hard reset */
	mask = SD_EXTERNAL_CONFIG1_RESET_IN_MASK;
	data = 0x1 << SD_EXTERNAL_CONFIG1_RESET_IN_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RESET_CORE_MASK;
	data |= 0x1 << SD_EXTERNAL_CONFIG1_RESET_CORE_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG1_REG, data, mask);

	/* Wait 1ms - until band gap and ref clock ready */
	mdelay(1);

	/* Make sure that 40 data bits is disabled
	 * This bit is not cleared by reset
	 */
	mask = COMMON_PHY_CFG6_IF_40_SEL_MASK;
	data = 0 << COMMON_PHY_CFG6_IF_40_SEL_OFFSET;
	reg_set(comphy_addr + COMMON_PHY_CFG6_REG, data, mask);

	/* Start comphy Configuration */
	debug("stage: Comphy configuration\n");
	/* set reference clock */
	mask = HPIPE_MISC_REFCLK_SEL_MASK;
	data = 0x0 << HPIPE_MISC_REFCLK_SEL_OFFSET;
	reg_set(hpipe_addr + HPIPE_MISC_REG, data, mask);
	/* Power and PLL Control */
	mask = HPIPE_PWR_PLL_REF_FREQ_MASK;
	data = 0x1 << HPIPE_PWR_PLL_REF_FREQ_OFFSET;
	mask |= HPIPE_PWR_PLL_PHY_MODE_MASK;
	data |= 0x4 << HPIPE_PWR_PLL_PHY_MODE_OFFSET;
	reg_set(hpipe_addr + HPIPE_PWR_PLL_REG, data, mask);
	/* Loopback register */
	mask = HPIPE_LOOPBACK_SEL_MASK;
	data = 0x1 << HPIPE_LOOPBACK_SEL_OFFSET;
	reg_set(hpipe_addr + HPIPE_LOOPBACK_REG, data, mask);
	/* rx control 1 */
	mask = HPIPE_RX_CONTROL_1_RXCLK2X_SEL_MASK;
	data = 0x1 << HPIPE_RX_CONTROL_1_RXCLK2X_SEL_OFFSET;
	mask |= HPIPE_RX_CONTROL_1_CLK8T_EN_MASK;
	data |= 0x0 << HPIPE_RX_CONTROL_1_CLK8T_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_RX_CONTROL_1_REG, data, mask);
	/* DTL Control */
	mask = HPIPE_PWR_CTR_DTL_FLOOP_EN_MASK;
	data = 0x0 << HPIPE_PWR_CTR_DTL_FLOOP_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_PWR_CTR_DTL_REG, data, mask);

	/* Set analog parameters from ETP(HW) - for now use the default datas */
	debug("stage: Analog parameters from ETP(HW)\n");

	reg_set(hpipe_addr + HPIPE_G1_SET_0_REG,
		0x1 << HPIPE_G1_SET_0_G1_TX_EMPH1_OFFSET, HPIPE_G1_SET_0_G1_TX_EMPH1_MASK);

	debug("stage: RFU configurations- Power Up PLL,Tx,Rx\n");
	/* SERDES External Configuration */
	mask = SD_EXTERNAL_CONFIG0_SD_PU_PLL_MASK;
	data = 0x1 << SD_EXTERNAL_CONFIG0_SD_PU_PLL_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_SD_PU_RX_MASK;
	data |= 0x1 << SD_EXTERNAL_CONFIG0_SD_PU_RX_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_SD_PU_TX_MASK;
	data |= 0x1 << SD_EXTERNAL_CONFIG0_SD_PU_TX_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG0_REG, data, mask);

	ret = mvebu_cp110_comphy_is_pll_locked(comphy_base, comphy_index);
	if (ret)
		return ret;

	/* RX init */
	mask = SD_EXTERNAL_CONFIG1_RX_INIT_MASK;
	data = 0x1 << SD_EXTERNAL_CONFIG1_RX_INIT_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG1_REG, data, mask);

	/* check that RX init done */
	addr = sd_ip_addr + SD_EXTERNAL_STATUS0_REG;
	data = SD_EXTERNAL_STATUS0_RX_INIT_MASK;
	mask = data;
	data = polling_with_timeout(addr, data, mask, 100, REG_32BIT);
	if (data != 0) {
		ERROR("RX init failed\n");
		ret = -ETIMEDOUT;
	}

	debug("stage: RF Reset\n");
	/* RF Reset */
	mask = SD_EXTERNAL_CONFIG1_RX_INIT_MASK;
	data = 0x0 << SD_EXTERNAL_CONFIG1_RX_INIT_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RF_RESET_IN_MASK;
	data |= 0x1 << SD_EXTERNAL_CONFIG1_RF_RESET_IN_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG1_REG, data, mask);

	debug_exit();

	return ret;
}

static int mvebu_cp110_comphy_xfi_power_on(uint64_t comphy_base, uint8_t comphy_index,
					   uint32_t comphy_mode)
{
	uintptr_t hpipe_addr, sd_ip_addr, comphy_addr, addr;
	uint32_t mask, data, speed = COMPHY_GET_SPEED(comphy_mode);
	int ret = 0;

	debug_enter();

	if ((speed != COMPHY_SPEED_5_15625G) &&
	     (speed != COMPHY_SPEED_10_3125G) &&
	     (speed != COMPHY_SPEED_DEFAULT)) {
		ERROR("comphy:%d: unsupported sfi/xfi speed\n", comphy_index);
		return -EINVAL;
	}

	hpipe_addr = HPIPE_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base), comphy_index);
	sd_ip_addr = SD_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base), comphy_index);
	comphy_addr = COMPHY_ADDR(comphy_base, comphy_index);

	/* configure phy selector for XFI/SFI */
	mvebu_cp110_comphy_set_phy_selector(comphy_base, comphy_index, comphy_mode);

	debug("stage: RFU configurations - hard reset comphy\n");
	/* RFU configurations - hard reset comphy */
	mask = COMMON_PHY_CFG1_PWR_UP_MASK;
	data = 0x1 << COMMON_PHY_CFG1_PWR_UP_OFFSET;
	mask |= COMMON_PHY_CFG1_PIPE_SELECT_MASK;
	data |= 0x0 << COMMON_PHY_CFG1_PIPE_SELECT_OFFSET;
	reg_set(comphy_addr + COMMON_PHY_CFG1_REG, data, mask);

	/* Make sure that 40 data bits is disabled
	 * This bit is not cleared by reset
	 */
	mask = COMMON_PHY_CFG6_IF_40_SEL_MASK;
	data = 0 << COMMON_PHY_CFG6_IF_40_SEL_OFFSET;
	reg_set(comphy_addr + COMMON_PHY_CFG6_REG, data, mask);

	/* Select Baud Rate of Comphy And PD_PLL/Tx/Rx */
	mask = SD_EXTERNAL_CONFIG0_SD_PU_PLL_MASK;
	data = 0x0 << SD_EXTERNAL_CONFIG0_SD_PU_PLL_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_SD_PHY_GEN_RX_MASK;
	data |= 0xE << SD_EXTERNAL_CONFIG0_SD_PHY_GEN_RX_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_SD_PHY_GEN_TX_MASK;
	data |= 0xE << SD_EXTERNAL_CONFIG0_SD_PHY_GEN_TX_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_SD_PU_RX_MASK;
	data |= 0 << SD_EXTERNAL_CONFIG0_SD_PU_RX_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_SD_PU_TX_MASK;
	data |= 0 << SD_EXTERNAL_CONFIG0_SD_PU_TX_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_HALF_BUS_MODE_MASK;
	data |= 0 << SD_EXTERNAL_CONFIG0_HALF_BUS_MODE_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG0_REG, data, mask);

	/* release from hard reset */
	mask = SD_EXTERNAL_CONFIG1_RESET_IN_MASK;
	data = 0x0 << SD_EXTERNAL_CONFIG1_RESET_IN_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RESET_CORE_MASK;
	data |= 0x0 << SD_EXTERNAL_CONFIG1_RESET_CORE_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RF_RESET_IN_MASK;
	data |= 0x0 << SD_EXTERNAL_CONFIG1_RF_RESET_IN_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG1_REG, data, mask);

	mask = SD_EXTERNAL_CONFIG1_RESET_IN_MASK;
	data = 0x1 << SD_EXTERNAL_CONFIG1_RESET_IN_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RESET_CORE_MASK;
	data |= 0x1 << SD_EXTERNAL_CONFIG1_RESET_CORE_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG1_REG, data, mask);

	/* Wait 1ms - until band gap and ref clock ready */
	mdelay(1);

	/* Start comphy Configuration */
	debug("stage: Comphy configuration\n");
	/* set reference clock */
	mask = HPIPE_MISC_ICP_FORCE_MASK;
	data = (speed == COMPHY_SPEED_5_15625G) ?
		(0x0 << HPIPE_MISC_ICP_FORCE_OFFSET) :
		(0x1 << HPIPE_MISC_ICP_FORCE_OFFSET);
	mask |= HPIPE_MISC_REFCLK_SEL_MASK;
	data |= 0x0 << HPIPE_MISC_REFCLK_SEL_OFFSET;
	reg_set(hpipe_addr + HPIPE_MISC_REG, data, mask);
	/* Power and PLL Control */
	mask = HPIPE_PWR_PLL_REF_FREQ_MASK;
	data = 0x1 << HPIPE_PWR_PLL_REF_FREQ_OFFSET;
	mask |= HPIPE_PWR_PLL_PHY_MODE_MASK;
	data |= 0x4 << HPIPE_PWR_PLL_PHY_MODE_OFFSET;
	reg_set(hpipe_addr + HPIPE_PWR_PLL_REG, data, mask);
	/* Loopback register */
	mask = HPIPE_LOOPBACK_SEL_MASK;
	data = 0x1 << HPIPE_LOOPBACK_SEL_OFFSET;
	reg_set(hpipe_addr + HPIPE_LOOPBACK_REG, data, mask);
	/* rx control 1 */
	mask = HPIPE_RX_CONTROL_1_RXCLK2X_SEL_MASK;
	data = 0x1 << HPIPE_RX_CONTROL_1_RXCLK2X_SEL_OFFSET;
	mask |= HPIPE_RX_CONTROL_1_CLK8T_EN_MASK;
	data |= 0x1 << HPIPE_RX_CONTROL_1_CLK8T_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_RX_CONTROL_1_REG, data, mask);
	/* DTL Control */
	mask = HPIPE_PWR_CTR_DTL_FLOOP_EN_MASK;
	data = 0x1 << HPIPE_PWR_CTR_DTL_FLOOP_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_PWR_CTR_DTL_REG, data, mask);

	/* Transmitter/Receiver Speed Divider Force */
	if (speed == COMPHY_SPEED_5_15625G) {
		mask = HPIPE_SPD_DIV_FORCE_RX_SPD_DIV_MASK;
		data = 1 << HPIPE_SPD_DIV_FORCE_RX_SPD_DIV_OFFSET;
		mask |= HPIPE_SPD_DIV_FORCE_RX_SPD_DIV_FORCE_MASK;
		data |= 1 << HPIPE_SPD_DIV_FORCE_RX_SPD_DIV_FORCE_OFFSET;
		mask |= HPIPE_SPD_DIV_FORCE_TX_SPD_DIV_MASK;
		data |= 1 << HPIPE_SPD_DIV_FORCE_TX_SPD_DIV_OFFSET;
		mask |= HPIPE_SPD_DIV_FORCE_TX_SPD_DIV_FORCE_MASK;
		data |= 1 << HPIPE_SPD_DIV_FORCE_TX_SPD_DIV_FORCE_OFFSET;
	} else {
		mask = HPIPE_TXDIGCK_DIV_FORCE_MASK;
		data = 0x1 << HPIPE_TXDIGCK_DIV_FORCE_OFFSET;
	}
	reg_set(hpipe_addr + HPIPE_SPD_DIV_FORCE_REG, data, mask);

	/* Set analog parameters from ETP(HW) */
	debug("stage: Analog parameters from ETP(HW)\n");
	/* SERDES External Configuration 2 */
	mask = SD_EXTERNAL_CONFIG2_PIN_DFE_EN_MASK;
	data = 0x1 << SD_EXTERNAL_CONFIG2_PIN_DFE_EN_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG2_REG, data, mask);
	/* 0x7-DFE Resolution control */
	mask = HPIPE_DFE_RES_FORCE_MASK;
	data = 0x1 << HPIPE_DFE_RES_FORCE_OFFSET;
	reg_set(hpipe_addr + HPIPE_DFE_REG0, data, mask);
	/* 0xd-G1_Setting_0 */
	if (speed == COMPHY_SPEED_5_15625G) {
		mask = HPIPE_G1_SET_0_G1_TX_EMPH1_MASK;
		data = 0x6 << HPIPE_G1_SET_0_G1_TX_EMPH1_OFFSET;
	} else {
		mask = HPIPE_G1_SET_0_G1_TX_AMP_MASK;
		data = 0x1c << HPIPE_G1_SET_0_G1_TX_AMP_OFFSET;
		mask |= HPIPE_G1_SET_0_G1_TX_EMPH1_MASK;
		data |= 0xe << HPIPE_G1_SET_0_G1_TX_EMPH1_OFFSET;
	}
	reg_set(hpipe_addr + HPIPE_G1_SET_0_REG, data, mask);
	/* Genration 1 setting 2 (G1_Setting_2) */
	mask = HPIPE_G1_SET_2_G1_TX_EMPH0_MASK;
	data = 0x0 << HPIPE_G1_SET_2_G1_TX_EMPH0_OFFSET;
	mask |= HPIPE_G1_SET_2_G1_TX_EMPH0_EN_MASK;
	data |= 0x1 << HPIPE_G1_SET_2_G1_TX_EMPH0_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_G1_SET_2_REG, data, mask);
	/* Transmitter Slew Rate Control register (tx_reg1) */
	mask = HPIPE_TX_REG1_TX_EMPH_RES_MASK;
	data = 0x3 << HPIPE_TX_REG1_TX_EMPH_RES_OFFSET;
	mask |= HPIPE_TX_REG1_SLC_EN_MASK;
	data |= 0x3f << HPIPE_TX_REG1_SLC_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_TX_REG1_REG, data, mask);
	/* Impedance Calibration Control register (cal_reg1) */
	mask = HPIPE_CAL_REG_1_EXT_TXIMP_MASK;
	data = 0xe << HPIPE_CAL_REG_1_EXT_TXIMP_OFFSET;
	mask |= HPIPE_CAL_REG_1_EXT_TXIMP_EN_MASK;
	data |= 0x1 << HPIPE_CAL_REG_1_EXT_TXIMP_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_CAL_REG1_REG, data, mask);
	/* Generation 1 Setting 5 (g1_setting_5) */
	mask = HPIPE_G1_SETTING_5_G1_ICP_MASK;
	data = 0 << HPIPE_CAL_REG_1_EXT_TXIMP_OFFSET;
	reg_set(hpipe_addr + HPIPE_G1_SETTING_5_REG, data, mask);

	/* 0xE-G1_Setting_1 */
	mask = HPIPE_G1_SET_1_G1_RX_DFE_EN_MASK;
	data = 0x1 << HPIPE_G1_SET_1_G1_RX_DFE_EN_OFFSET;
	if (speed == COMPHY_SPEED_5_15625G) {
		mask |= HPIPE_G1_SET_1_G1_RX_SELMUPI_MASK;
		data |= 0x1 << HPIPE_G1_SET_1_G1_RX_SELMUPI_OFFSET;
		mask |= HPIPE_G1_SET_1_G1_RX_SELMUPP_MASK;
		data |= 0x1 << HPIPE_G1_SET_1_G1_RX_SELMUPP_OFFSET;
	} else {
		mask |= HPIPE_G1_SET_1_G1_RX_SELMUPI_MASK;
		data |= 0x2 << HPIPE_G1_SET_1_G1_RX_SELMUPI_OFFSET;
		mask |= HPIPE_G1_SET_1_G1_RX_SELMUPP_MASK;
		data |= 0x2 << HPIPE_G1_SET_1_G1_RX_SELMUPP_OFFSET;
		mask |= HPIPE_G1_SET_1_G1_RX_SELMUFI_MASK;
		data |= 0x0 << HPIPE_G1_SET_1_G1_RX_SELMUFI_OFFSET;
		mask |= HPIPE_G1_SET_1_G1_RX_SELMUFF_MASK;
		data |= 0x1 << HPIPE_G1_SET_1_G1_RX_SELMUFF_OFFSET;
		mask |= HPIPE_G1_SET_1_G1_RX_DIGCK_DIV_MASK;
		data |= 0x3 << HPIPE_G1_SET_1_G1_RX_DIGCK_DIV_OFFSET;
	}
	reg_set(hpipe_addr + HPIPE_G1_SET_1_REG, data, mask);

	/* 0xA-DFE_Reg3 */
	mask = HPIPE_DFE_F3_F5_DFE_EN_MASK;
	data = 0x0 << HPIPE_DFE_F3_F5_DFE_EN_OFFSET;
	mask |= HPIPE_DFE_F3_F5_DFE_CTRL_MASK;
	data |= 0x0 << HPIPE_DFE_F3_F5_DFE_CTRL_OFFSET;
	reg_set(hpipe_addr + HPIPE_DFE_F3_F5_REG, data, mask);

	/* 0x111-G1_Setting_4 */
	mask = HPIPE_G1_SETTINGS_4_G1_DFE_RES_MASK;
	data = 0x1 << HPIPE_G1_SETTINGS_4_G1_DFE_RES_OFFSET;
	reg_set(hpipe_addr + HPIPE_G1_SETTINGS_4_REG, data, mask);
	/* Genration 1 setting 3 (G1_Setting_3) */
	mask = HPIPE_G1_SETTINGS_3_G1_FBCK_SEL_MASK;
	data = 0x1 << HPIPE_G1_SETTINGS_3_G1_FBCK_SEL_OFFSET;
	if (speed == COMPHY_SPEED_5_15625G) {
		/* Force FFE (Feed Forward Equalization) to 5G */
		mask |= HPIPE_G1_SETTINGS_3_G1_FFE_CAP_SEL_MASK;
		data |= 0xf << HPIPE_G1_SETTINGS_3_G1_FFE_CAP_SEL_OFFSET;
		mask |= HPIPE_G1_SETTINGS_3_G1_FFE_RES_SEL_MASK;
		data |= 0x4 << HPIPE_G1_SETTINGS_3_G1_FFE_RES_SEL_OFFSET;
		mask |= HPIPE_G1_SETTINGS_3_G1_FFE_SETTING_FORCE_MASK;
		data |= 0x1 << HPIPE_G1_SETTINGS_3_G1_FFE_SETTING_FORCE_OFFSET;
	}
	reg_set(hpipe_addr + HPIPE_G1_SETTINGS_3_REG, data, mask);

	/* Connfigure RX training timer */
	mask = HPIPE_RX_TRAIN_TIMER_MASK;
	data = 0x13 << HPIPE_RX_TRAIN_TIMER_OFFSET;
	reg_set(hpipe_addr + HPIPE_TX_TRAIN_CTRL_5_REG, data, mask);

	/* Enable TX train peak to peak hold */
	mask = HPIPE_TX_TRAIN_P2P_HOLD_MASK;
	data = 0x1 << HPIPE_TX_TRAIN_P2P_HOLD_OFFSET;
	reg_set(hpipe_addr + HPIPE_TX_TRAIN_CTRL_0_REG, data, mask);

	/* Configure TX preset index */
	mask = HPIPE_TX_PRESET_INDEX_MASK;
	data = 0x2 << HPIPE_TX_PRESET_INDEX_OFFSET;
	reg_set(hpipe_addr + HPIPE_TX_PRESET_INDEX_REG, data, mask);

	/* Disable pattern lock lost timeout */
	mask = HPIPE_PATTERN_LOCK_LOST_TIMEOUT_EN_MASK;
	data = 0x0 << HPIPE_PATTERN_LOCK_LOST_TIMEOUT_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_FRAME_DETECT_CTRL_3_REG, data, mask);

	/* Configure TX training pattern and TX training 16bit auto */
	mask = HPIPE_TX_TRAIN_16BIT_AUTO_EN_MASK;
	data = 0x1 << HPIPE_TX_TRAIN_16BIT_AUTO_EN_OFFSET;
	mask |= HPIPE_TX_TRAIN_PAT_SEL_MASK;
	data |= 0x1 << HPIPE_TX_TRAIN_PAT_SEL_OFFSET;
	reg_set(hpipe_addr + HPIPE_TX_TRAIN_REG, data, mask);

	/* Configure Training patten number */
	mask = HPIPE_TRAIN_PAT_NUM_MASK;
	data = 0x88 << HPIPE_TRAIN_PAT_NUM_OFFSET;
	reg_set(hpipe_addr + HPIPE_FRAME_DETECT_CTRL_0_REG, data, mask);

	/* Configure differencial manchester encoter to ethernet mode */
	mask = HPIPE_DME_ETHERNET_MODE_MASK;
	data = 0x1 << HPIPE_DME_ETHERNET_MODE_OFFSET;
	reg_set(hpipe_addr + HPIPE_DME_REG, data, mask);

	/* Configure VDD Continuous Calibration */
	mask = HPIPE_CAL_VDD_CONT_MODE_MASK;
	data = 0x1 << HPIPE_CAL_VDD_CONT_MODE_OFFSET;
	reg_set(hpipe_addr + HPIPE_VDD_CAL_0_REG, data, mask);

	/* Trigger sampler enable pulse (by toggleing the bit) */
	mask = HPIPE_RX_SAMPLER_OS_GAIN_MASK;
	data = 0x3 << HPIPE_RX_SAMPLER_OS_GAIN_OFFSET;
	mask |= HPIPE_SMAPLER_MASK;
	data |= 0x1 << HPIPE_SMAPLER_OFFSET;
	reg_set(hpipe_addr + HPIPE_SAMPLER_N_PROC_CALIB_CTRL_REG, data, mask);
	mask = HPIPE_SMAPLER_MASK;
	data = 0x0 << HPIPE_SMAPLER_OFFSET;
	reg_set(hpipe_addr + HPIPE_SAMPLER_N_PROC_CALIB_CTRL_REG, data, mask);

	/* Set External RX Regulator Control */
	mask = HPIPE_EXT_SELLV_RXSAMPL_MASK;
	data = 0x1A << HPIPE_EXT_SELLV_RXSAMPL_OFFSET;
	reg_set(hpipe_addr + HPIPE_VDD_CAL_CTRL_REG, data, mask);

	debug("stage: RFU configurations- Power Up PLL,Tx,Rx\n");
	/* SERDES External Configuration */
	mask = SD_EXTERNAL_CONFIG0_SD_PU_PLL_MASK;
	data = 0x1 << SD_EXTERNAL_CONFIG0_SD_PU_PLL_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_SD_PU_RX_MASK;
	data |= 0x1 << SD_EXTERNAL_CONFIG0_SD_PU_RX_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_SD_PU_TX_MASK;
	data |= 0x1 << SD_EXTERNAL_CONFIG0_SD_PU_TX_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG0_REG, data, mask);

	/* check PLL rx & tx ready */
	addr = sd_ip_addr + SD_EXTERNAL_STATUS0_REG;
	data = SD_EXTERNAL_STATUS0_PLL_RX_MASK | SD_EXTERNAL_STATUS0_PLL_TX_MASK;
	mask = data;
	data = polling_with_timeout(addr, data, mask, PLL_LOCK_TIMEOUT, REG_32BIT);
	if (data != 0) {
		if (data & SD_EXTERNAL_STATUS0_PLL_RX_MASK)
			ERROR("RX PLL is not locked\n");
		if (data & SD_EXTERNAL_STATUS0_PLL_TX_MASK)
			ERROR("TX PLL is not locked\n");

		ret = -ETIMEDOUT;
	}

	/* RX init */
	mask = SD_EXTERNAL_CONFIG1_RX_INIT_MASK;
	data = 0x1 << SD_EXTERNAL_CONFIG1_RX_INIT_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG1_REG, data, mask);

	/* check that RX init done */
	addr = sd_ip_addr + SD_EXTERNAL_STATUS0_REG;
	data = SD_EXTERNAL_STATUS0_RX_INIT_MASK;
	mask = data;
	data = polling_with_timeout(addr, data, mask, 100, REG_32BIT);
	if (data != 0) {
		ERROR("RX init failed\n");
		ret = -ETIMEDOUT;
	}

	debug("stage: RF Reset\n");
	/* RF Reset */
	mask =  SD_EXTERNAL_CONFIG1_RX_INIT_MASK;
	data = 0x0 << SD_EXTERNAL_CONFIG1_RX_INIT_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RF_RESET_IN_MASK;
	data |= 0x1 << SD_EXTERNAL_CONFIG1_RF_RESET_IN_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG1_REG, data, mask);

	debug_exit();

	return ret;
}

static int mvebu_cp110_comphy_pcie_power_on(uint64_t comphy_base,
				     uint8_t comphy_index, uint32_t comphy_mode)
{
	int ret = 0;
	uint32_t reg, mask, data, pcie_width;
	uint32_t clk_dir;
	uintptr_t hpipe_addr, comphy_addr, addr;
	_Bool clk_src = COMPHY_GET_CLK_SRC(comphy_mode);

	hpipe_addr = HPIPE_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base),
				comphy_index);
	comphy_addr = COMPHY_ADDR(comphy_base, comphy_index);
	pcie_width = COMPHY_GET_PCIE_WIDTH(comphy_mode);

	debug_enter();

	spin_lock(&cp110_mac_reset_lock);

	reg = mmio_read_32(SYS_CTRL_FROM_COMPHY_ADDR(comphy_base) +
						 SYS_CTRL_UINIT_SOFT_RESET_REG);
	switch (comphy_index) {
	case COMPHY_LANE0:
		reg |= PCIE_MAC_RESET_MASK_PORT0;
		break;
	case COMPHY_LANE4:
		reg |= PCIE_MAC_RESET_MASK_PORT1;
		break;
	case COMPHY_LANE5:
		reg |= PCIE_MAC_RESET_MASK_PORT2;
		break;
	}

	mmio_write_32(SYS_CTRL_FROM_COMPHY_ADDR(comphy_base) +
					    SYS_CTRL_UINIT_SOFT_RESET_REG, reg);
	spin_unlock(&cp110_mac_reset_lock);

	/* Configure PIPE selector for PCIE */
	mvebu_cp110_comphy_set_pipe_selector(comphy_base, comphy_index,
								   comphy_mode);

	/*
	 * Read SAR (Sample-At-Reset) configuration for the PCIe clock
	 * direction.
	 *
	 * SerDes Lane 4/5 got the PCIe ref-clock #1,
	 * and SerDes Lane 0 got PCIe ref-clock #0
	 */
	reg = mmio_read_32(DFX_FROM_COMPHY_ADDR(comphy_base) +
			   SAR_STATUS_0_REG);
	if (comphy_index == COMPHY_LANE4 || comphy_index == COMPHY_LANE5)
		clk_dir = (reg & SAR_RST_PCIE1_CLOCK_CONFIG_CP1_MASK) >>
					  SAR_RST_PCIE1_CLOCK_CONFIG_CP1_OFFSET;
	else
		clk_dir = (reg & SAR_RST_PCIE0_CLOCK_CONFIG_CP1_MASK) >>
					  SAR_RST_PCIE0_CLOCK_CONFIG_CP1_OFFSET;

	debug("On lane %d\n", comphy_index);
	debug("PCIe clock direction = %x\n", clk_dir);
	debug("PCIe Width = %d\n", pcie_width);

	/* enable PCIe X4 and X2 */
	if (comphy_index == COMPHY_LANE0) {
		if (pcie_width == PCIE_LNK_X4) {
			data = 0x1 << COMMON_PHY_SD_CTRL1_PCIE_X4_EN_OFFSET;
			mask = COMMON_PHY_SD_CTRL1_PCIE_X4_EN_MASK;
			reg_set(comphy_base + COMMON_PHY_SD_CTRL1,
				data, mask);
		} else if (pcie_width == PCIE_LNK_X2) {
			data = 0x1 << COMMON_PHY_SD_CTRL1_PCIE_X2_EN_OFFSET;
			mask = COMMON_PHY_SD_CTRL1_PCIE_X2_EN_MASK;
			reg_set(comphy_base + COMMON_PHY_SD_CTRL1, data, mask);
		}
	}

	/* If PCIe clock is output and clock source from SerDes lane 5,
	 * need to configure the clock-source MUX.
	 * By default, the clock source is from lane 4
	 */
	if (clk_dir && clk_src && (comphy_index == COMPHY_LANE5)) {
		data = DFX_DEV_GEN_PCIE_CLK_SRC_MUX <<
						DFX_DEV_GEN_PCIE_CLK_SRC_OFFSET;
		mask = DFX_DEV_GEN_PCIE_CLK_SRC_MASK;
		reg_set(DFX_FROM_COMPHY_ADDR(comphy_base) +
			DFX_DEV_GEN_CTRL12_REG, data, mask);
	}

	debug("stage: RFU configurations - hard reset comphy\n");
	/* RFU configurations - hard reset comphy */
	mask = COMMON_PHY_CFG1_PWR_UP_MASK;
	data = 0x1 << COMMON_PHY_CFG1_PWR_UP_OFFSET;
	mask |= COMMON_PHY_CFG1_PIPE_SELECT_MASK;
	data |= 0x1 << COMMON_PHY_CFG1_PIPE_SELECT_OFFSET;
	mask |= COMMON_PHY_CFG1_PWR_ON_RESET_MASK;
	data |= 0x0 << COMMON_PHY_CFG1_PWR_ON_RESET_OFFSET;
	mask |= COMMON_PHY_CFG1_CORE_RSTN_MASK;
	data |= 0x0 << COMMON_PHY_CFG1_CORE_RSTN_OFFSET;
	mask |= COMMON_PHY_PHY_MODE_MASK;
	data |= 0x0 << COMMON_PHY_PHY_MODE_OFFSET;
	reg_set(comphy_addr + COMMON_PHY_CFG1_REG, data, mask);

	/* release from hard reset */
	mask = COMMON_PHY_CFG1_PWR_ON_RESET_MASK;
	data = 0x1 << COMMON_PHY_CFG1_PWR_ON_RESET_OFFSET;
	mask |= COMMON_PHY_CFG1_CORE_RSTN_MASK;
	data |= 0x1 << COMMON_PHY_CFG1_CORE_RSTN_OFFSET;
	reg_set(comphy_addr + COMMON_PHY_CFG1_REG, data, mask);

	/* Wait 1ms - until band gap and ref clock ready */
	mdelay(1);
	/* Start comphy Configuration */
	debug("stage: Comphy configuration\n");
	/* Set PIPE soft reset */
	mask = HPIPE_RST_CLK_CTRL_PIPE_RST_MASK;
	data = 0x1 << HPIPE_RST_CLK_CTRL_PIPE_RST_OFFSET;
	/* Set PHY datapath width mode for V0 */
	mask |= HPIPE_RST_CLK_CTRL_FIXED_PCLK_MASK;
	data |= 0x1 << HPIPE_RST_CLK_CTRL_FIXED_PCLK_OFFSET;
	/* Set Data bus width USB mode for V0 */
	mask |= HPIPE_RST_CLK_CTRL_PIPE_WIDTH_MASK;
	data |= 0x0 << HPIPE_RST_CLK_CTRL_PIPE_WIDTH_OFFSET;
	/* Set CORE_CLK output frequency for 250Mhz */
	mask |= HPIPE_RST_CLK_CTRL_CORE_FREQ_SEL_MASK;
	data |= 0x0 << HPIPE_RST_CLK_CTRL_CORE_FREQ_SEL_OFFSET;
	reg_set(hpipe_addr + HPIPE_RST_CLK_CTRL_REG, data, mask);
	/* Set PLL ready delay for 0x2 */
	data = 0x2 << HPIPE_CLK_SRC_LO_PLL_RDY_DL_OFFSET;
	mask = HPIPE_CLK_SRC_LO_PLL_RDY_DL_MASK;
	if (pcie_width != PCIE_LNK_X1) {
		data |= 0x1 << HPIPE_CLK_SRC_LO_BUNDLE_PERIOD_SEL_OFFSET;
		mask |= HPIPE_CLK_SRC_LO_BUNDLE_PERIOD_SEL_MASK;
		data |= 0x1 << HPIPE_CLK_SRC_LO_BUNDLE_PERIOD_SCALE_OFFSET;
		mask |= HPIPE_CLK_SRC_LO_BUNDLE_PERIOD_SCALE_MASK;
	}
	reg_set(hpipe_addr + HPIPE_CLK_SRC_LO_REG, data, mask);

	/* Set PIPE mode interface to PCIe3 - 0x1  & set lane order */
	data = 0x1 << HPIPE_CLK_SRC_HI_MODE_PIPE_OFFSET;
	mask = HPIPE_CLK_SRC_HI_MODE_PIPE_MASK;
	if (pcie_width != PCIE_LNK_X1) {
		mask |= HPIPE_CLK_SRC_HI_LANE_STRT_MASK;
		mask |= HPIPE_CLK_SRC_HI_LANE_MASTER_MASK;
		mask |= HPIPE_CLK_SRC_HI_LANE_BREAK_MASK;
		if (comphy_index == 0) {
			data |= 0x1 << HPIPE_CLK_SRC_HI_LANE_STRT_OFFSET;
			data |= 0x1 << HPIPE_CLK_SRC_HI_LANE_MASTER_OFFSET;
		} else if (comphy_index == (pcie_width - 1)) {
			data |= 0x1 << HPIPE_CLK_SRC_HI_LANE_BREAK_OFFSET;
		}
	}
	reg_set(hpipe_addr + HPIPE_CLK_SRC_HI_REG, data, mask);
	/* Config update polarity equalization */
	data = 0x1 << HPIPE_CFG_UPDATE_POLARITY_OFFSET;
	mask = HPIPE_CFG_UPDATE_POLARITY_MASK;
	reg_set(hpipe_addr + HPIPE_LANE_EQ_CFG1_REG, data, mask);
	/* Set PIPE version 4 to mode enable */
	data = 0x1 << HPIPE_DFE_CTRL_28_PIPE4_OFFSET;
	mask = HPIPE_DFE_CTRL_28_PIPE4_MASK;
	reg_set(hpipe_addr + HPIPE_DFE_CTRL_28_REG, data, mask);
	/* TODO: check if pcie clock is output/input - for bringup use input*/
	/* Enable PIN clock 100M_125M */
	mask = 0;
	data = 0;
	/* Only if clock is output, configure the clock-source mux */
	if (clk_dir) {
		mask |= HPIPE_MISC_CLK100M_125M_MASK;
		data |= 0x1 << HPIPE_MISC_CLK100M_125M_OFFSET;
	}
	/* Set PIN_TXDCLK_2X Clock Frequency Selection for outputs 500MHz clock */
	mask |= HPIPE_MISC_TXDCLK_2X_MASK;
	data |= 0x0 << HPIPE_MISC_TXDCLK_2X_OFFSET;
	/* Enable 500MHz Clock */
	mask |= HPIPE_MISC_CLK500_EN_MASK;
	data |= 0x1 << HPIPE_MISC_CLK500_EN_OFFSET;
	if (clk_dir) { /* output */
		/* Set reference clock comes from group 1 */
		mask |= HPIPE_MISC_REFCLK_SEL_MASK;
		data |= 0x0 << HPIPE_MISC_REFCLK_SEL_OFFSET;
	} else {
		/* Set reference clock comes from group 2 */
		mask |= HPIPE_MISC_REFCLK_SEL_MASK;
		data |= 0x1 << HPIPE_MISC_REFCLK_SEL_OFFSET;
	}
	mask |= HPIPE_MISC_ICP_FORCE_MASK;
	data |= 0x1 << HPIPE_MISC_ICP_FORCE_OFFSET;
	reg_set(hpipe_addr + HPIPE_MISC_REG, data, mask);
	if (clk_dir) { /* output */
		/* Set reference frequcency select - 0x2 for 25MHz*/
		mask = HPIPE_PWR_PLL_REF_FREQ_MASK;
		data = 0x2 << HPIPE_PWR_PLL_REF_FREQ_OFFSET;
	} else {
		/* Set reference frequcency select - 0x0 for 100MHz*/
		mask = HPIPE_PWR_PLL_REF_FREQ_MASK;
		data = 0x0 << HPIPE_PWR_PLL_REF_FREQ_OFFSET;
	}
	/* Set PHY mode to PCIe */
	mask |= HPIPE_PWR_PLL_PHY_MODE_MASK;
	data |= 0x3 << HPIPE_PWR_PLL_PHY_MODE_OFFSET;
	reg_set(hpipe_addr + HPIPE_PWR_PLL_REG, data, mask);

	/* ref clock alignment */
	if (pcie_width != PCIE_LNK_X1) {
		mask = HPIPE_LANE_ALIGN_OFF_MASK;
		data = 0x0 << HPIPE_LANE_ALIGN_OFF_OFFSET;
		reg_set(hpipe_addr + HPIPE_LANE_ALIGN_REG, data, mask);
	}

	/* Set the amount of time spent in the LoZ state - set for 0x7 only if
	 * the PCIe clock is output
	 */
	if (clk_dir)
		reg_set(hpipe_addr + HPIPE_GLOBAL_PM_CTRL,
			0x7 << HPIPE_GLOBAL_PM_RXDLOZ_WAIT_OFFSET,
			HPIPE_GLOBAL_PM_RXDLOZ_WAIT_MASK);

	/* Set Maximal PHY Generation Setting(8Gbps) */
	mask = HPIPE_INTERFACE_GEN_MAX_MASK;
	data = 0x2 << HPIPE_INTERFACE_GEN_MAX_OFFSET;
	/* Bypass frame detection and sync detection for RX DATA */
	mask |= HPIPE_INTERFACE_DET_BYPASS_MASK;
	data |= 0x1 << HPIPE_INTERFACE_DET_BYPASS_OFFSET;
	/* Set Link Train Mode (Tx training control pins are used) */
	mask |= HPIPE_INTERFACE_LINK_TRAIN_MASK;
	data |= 0x1 << HPIPE_INTERFACE_LINK_TRAIN_OFFSET;
	reg_set(hpipe_addr + HPIPE_INTERFACE_REG, data, mask);

	/* Set Idle_sync enable */
	mask = HPIPE_PCIE_IDLE_SYNC_MASK;
	data = 0x1 << HPIPE_PCIE_IDLE_SYNC_OFFSET;
	/* Select bits for PCIE Gen3(32bit) */
	mask |= HPIPE_PCIE_SEL_BITS_MASK;
	data |= 0x2 << HPIPE_PCIE_SEL_BITS_OFFSET;
	reg_set(hpipe_addr + HPIPE_PCIE_REG0, data, mask);

	/* Enable Tx_adapt_g1 */
	mask = HPIPE_TX_TRAIN_CTRL_G1_MASK;
	data = 0x1 << HPIPE_TX_TRAIN_CTRL_G1_OFFSET;
	/* Enable Tx_adapt_gn1 */
	mask |= HPIPE_TX_TRAIN_CTRL_GN1_MASK;
	data |= 0x1 << HPIPE_TX_TRAIN_CTRL_GN1_OFFSET;
	/* Disable Tx_adapt_g0 */
	mask |= HPIPE_TX_TRAIN_CTRL_G0_MASK;
	data |= 0x0 << HPIPE_TX_TRAIN_CTRL_G0_OFFSET;
	reg_set(hpipe_addr + HPIPE_TX_TRAIN_CTRL_REG, data, mask);

	/* Set reg_tx_train_chk_init */
	mask = HPIPE_TX_TRAIN_CHK_INIT_MASK;
	data = 0x0 << HPIPE_TX_TRAIN_CHK_INIT_OFFSET;
	/* Enable TX_COE_FM_PIN_PCIE3_EN */
	mask |= HPIPE_TX_TRAIN_COE_FM_PIN_PCIE3_MASK;
	data |= 0x1 << HPIPE_TX_TRAIN_COE_FM_PIN_PCIE3_OFFSET;
	reg_set(hpipe_addr + HPIPE_TX_TRAIN_REG, data, mask);

	debug("stage: TRx training parameters\n");
	/* Set Preset sweep configurations */
	mask = HPIPE_TX_TX_STATUS_CHECK_MODE_MASK;
	data = 0x1 << HPIPE_TX_STATUS_CHECK_MODE_OFFSET;
	mask |= HPIPE_TX_NUM_OF_PRESET_MASK;
	data |= 0x7 << HPIPE_TX_NUM_OF_PRESET_OFFSET;
	mask |= HPIPE_TX_SWEEP_PRESET_EN_MASK;
	data |= 0x1 << HPIPE_TX_SWEEP_PRESET_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_TX_TRAIN_CTRL_11_REG, data, mask);

	/* Tx train start configuration */
	mask = HPIPE_TX_TRAIN_START_SQ_EN_MASK;
	data = 0x1 << HPIPE_TX_TRAIN_START_SQ_EN_OFFSET;
	mask |= HPIPE_TX_TRAIN_START_FRM_DET_EN_MASK;
	data |= 0x0 << HPIPE_TX_TRAIN_START_FRM_DET_EN_OFFSET;
	mask |= HPIPE_TX_TRAIN_START_FRM_LOCK_EN_MASK;
	data |= 0x0 << HPIPE_TX_TRAIN_START_FRM_LOCK_EN_OFFSET;
	mask |= HPIPE_TX_TRAIN_WAIT_TIME_EN_MASK;
	data |= 0x1 << HPIPE_TX_TRAIN_WAIT_TIME_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_TX_TRAIN_CTRL_5_REG, data, mask);

	/* Enable Tx train P2P */
	mask = HPIPE_TX_TRAIN_P2P_HOLD_MASK;
	data = 0x1 << HPIPE_TX_TRAIN_P2P_HOLD_OFFSET;
	reg_set(hpipe_addr + HPIPE_TX_TRAIN_CTRL_0_REG, data, mask);

	/* Configure Tx train timeout */
	mask = HPIPE_TRX_TRAIN_TIMER_MASK;
	data = 0x17 << HPIPE_TRX_TRAIN_TIMER_OFFSET;
	reg_set(hpipe_addr + HPIPE_TX_TRAIN_CTRL_4_REG, data, mask);

	/* Disable G0/G1/GN1 adaptation */
	mask = HPIPE_TX_TRAIN_CTRL_G1_MASK | HPIPE_TX_TRAIN_CTRL_GN1_MASK
		| HPIPE_TX_TRAIN_CTRL_G0_OFFSET;
	data = 0;
	reg_set(hpipe_addr + HPIPE_TX_TRAIN_CTRL_REG, data, mask);

	/* Disable DTL frequency loop */
	mask = HPIPE_PWR_CTR_DTL_FLOOP_EN_MASK;
	data = 0x0 << HPIPE_PWR_CTR_DTL_FLOOP_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_PWR_CTR_DTL_REG, data, mask);

	/* Configure G3 DFE */
	mask = HPIPE_G3_DFE_RES_MASK;
	data = 0x3 << HPIPE_G3_DFE_RES_OFFSET;
	reg_set(hpipe_addr + HPIPE_G3_SETTING_4_REG, data, mask);

	/* Use TX/RX training result for DFE */
	mask = HPIPE_DFE_RES_FORCE_MASK;
	data = 0x0 << HPIPE_DFE_RES_FORCE_OFFSET;
	reg_set(hpipe_addr + HPIPE_DFE_REG0,  data, mask);

	/* Configure initial and final coefficient value for receiver */
	mask = HPIPE_G3_SET_1_G3_RX_SELMUPI_MASK;
	data = 0x1 << HPIPE_G3_SET_1_G3_RX_SELMUPI_OFFSET;

	mask |= HPIPE_G3_SET_1_G3_RX_SELMUPF_MASK;
	data |= 0x1 << HPIPE_G3_SET_1_G3_RX_SELMUPF_OFFSET;

	mask |= HPIPE_G3_SET_1_G3_SAMPLER_INPAIRX2_EN_MASK;
	data |= 0x0 << HPIPE_G3_SET_1_G3_SAMPLER_INPAIRX2_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_G3_SET_1_REG,  data, mask);

	/* Trigger sampler enable pulse */
	mask = HPIPE_SMAPLER_MASK;
	data = 0x1 << HPIPE_SMAPLER_OFFSET;
	reg_set(hpipe_addr + HPIPE_SAMPLER_N_PROC_CALIB_CTRL_REG, data, mask);
	udelay(5);
	reg_set(hpipe_addr + HPIPE_SAMPLER_N_PROC_CALIB_CTRL_REG, 0, mask);

	/* FFE resistor tuning for different bandwidth  */
	mask = HPIPE_G3_FFE_DEG_RES_LEVEL_MASK;
	data = 0x1 << HPIPE_G3_FFE_DEG_RES_LEVEL_OFFSET;
	mask |= HPIPE_G3_FFE_LOAD_RES_LEVEL_MASK;
	data |= 0x3 << HPIPE_G3_FFE_LOAD_RES_LEVEL_OFFSET;
	reg_set(hpipe_addr + HPIPE_G3_SETTING_3_REG, data, mask);

	/* Pattern lock lost timeout disable */
	mask = HPIPE_PATTERN_LOCK_LOST_TIMEOUT_EN_MASK;
	data = 0x0 << HPIPE_PATTERN_LOCK_LOST_TIMEOUT_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_FRAME_DETECT_CTRL_3_REG, data, mask);

	/* Configure DFE adaptations */
	mask = HPIPE_CDR_RX_MAX_DFE_ADAPT_0_MASK;
	data = 0x0 << HPIPE_CDR_RX_MAX_DFE_ADAPT_0_OFFSET;
	mask |= HPIPE_CDR_RX_MAX_DFE_ADAPT_1_MASK;
	data |= 0x0 << HPIPE_CDR_RX_MAX_DFE_ADAPT_1_OFFSET;
	mask |= HPIPE_CDR_MAX_DFE_ADAPT_0_MASK;
	data |= 0x0 << HPIPE_CDR_MAX_DFE_ADAPT_0_OFFSET;
	mask |= HPIPE_CDR_MAX_DFE_ADAPT_1_MASK;
	data |= 0x1 << HPIPE_CDR_MAX_DFE_ADAPT_1_OFFSET;
	reg_set(hpipe_addr + HPIPE_CDR_CONTROL_REG, data, mask);

	mask = HPIPE_DFE_TX_MAX_DFE_ADAPT_MASK;
	data = 0x0 << HPIPE_DFE_TX_MAX_DFE_ADAPT_OFFSET;
	reg_set(hpipe_addr + HPIPE_DFE_CONTROL_REG, data, mask);

	/* Genration 2 setting 1*/
	mask = HPIPE_G2_SET_1_G2_RX_SELMUPI_MASK;
	data = 0x0 << HPIPE_G2_SET_1_G2_RX_SELMUPI_OFFSET;
	mask |= HPIPE_G2_SET_1_G2_RX_SELMUPP_MASK;
	data |= 0x1 << HPIPE_G2_SET_1_G2_RX_SELMUPP_OFFSET;
	mask |= HPIPE_G2_SET_1_G2_RX_SELMUFI_MASK;
	data |= 0x0 << HPIPE_G2_SET_1_G2_RX_SELMUFI_OFFSET;
	reg_set(hpipe_addr + HPIPE_G2_SET_1_REG, data, mask);

	/* DFE enable */
	mask = HPIPE_G2_DFE_RES_MASK;
	data = 0x3 << HPIPE_G2_DFE_RES_OFFSET;
	reg_set(hpipe_addr + HPIPE_G2_SETTINGS_4_REG, data, mask);

	/* Configure DFE Resolution */
	mask = HPIPE_LANE_CFG4_DFE_EN_SEL_MASK;
	data = 0x1 << HPIPE_LANE_CFG4_DFE_EN_SEL_OFFSET;
	reg_set(hpipe_addr + HPIPE_LANE_CFG4_REG, data, mask);

	/* VDD calibration control */
	mask = HPIPE_EXT_SELLV_RXSAMPL_MASK;
	data = 0x16 << HPIPE_EXT_SELLV_RXSAMPL_OFFSET;
	reg_set(hpipe_addr + HPIPE_VDD_CAL_CTRL_REG, data, mask);

	/* Set PLL Charge-pump Current Control */
	mask = HPIPE_G3_SETTING_5_G3_ICP_MASK;
	data = 0x4 << HPIPE_G3_SETTING_5_G3_ICP_OFFSET;
	reg_set(hpipe_addr + HPIPE_G3_SETTING_5_REG, data, mask);

	/* Set lane rqualization remote setting */
	mask = HPIPE_LANE_CFG_FOM_DIRN_OVERRIDE_MASK;
	data = 0x1 << HPIPE_LANE_CFG_FOM_DIRN_OVERRIDE_OFFSET;
	mask |= HPIPE_LANE_CFG_FOM_ONLY_MODE_MASK;
	data |= 0x1 << HPIPE_LANE_CFG_FOM_ONLY_MODE_OFFFSET;
	mask |= HPIPE_LANE_CFG_FOM_PRESET_VECTOR_MASK;
	data |= 0x6 << HPIPE_LANE_CFG_FOM_PRESET_VECTOR_OFFSET;
	reg_set(hpipe_addr + HPIPE_LANE_EQ_REMOTE_SETTING_REG, data, mask);

	mask = HPIPE_CFG_EQ_BUNDLE_DIS_MASK;
	data = 0x1 << HPIPE_CFG_EQ_BUNDLE_DIS_OFFSET;
	reg_set(hpipe_addr + HPIPE_LANE_EQ_CFG2_REG, data, mask);

	debug("stage: Comphy power up\n");

	/* For PCIe X4 or X2:
	 * release from reset only after finish to configure all lanes
	 */
	if ((pcie_width == PCIE_LNK_X1) || (comphy_index == (pcie_width - 1))) {
		uint32_t i, start_lane, end_lane;

		if (pcie_width != PCIE_LNK_X1) {
			/* allows writing to all lanes in one write */
			data = 0x0;
			if (pcie_width == PCIE_LNK_X2)
				mask = COMMON_PHY_SD_CTRL1_COMPHY_0_1_PORT_MASK;
			else if (pcie_width == PCIE_LNK_X4)
				mask = COMMON_PHY_SD_CTRL1_COMPHY_0_3_PORT_MASK;
			reg_set(comphy_base + COMMON_PHY_SD_CTRL1, data, mask);
			start_lane = 0;
			end_lane = pcie_width;

			/* Release from PIPE soft reset
			 * For PCIe by4 or by2:
			 * release from soft reset all lanes - can't use
			 *read modify write
			 */
			reg_set(HPIPE_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base), 0) +
				HPIPE_RST_CLK_CTRL_REG, 0x24, 0xffffffff);
		} else {
			start_lane = comphy_index;
			end_lane = comphy_index + 1;

			/* Release from PIPE soft reset
			 * for PCIe by4 or by2:
			 * release from soft reset all lanes
			 */
			reg_set(hpipe_addr + HPIPE_RST_CLK_CTRL_REG,
				0x0 << HPIPE_RST_CLK_CTRL_PIPE_RST_OFFSET,
				HPIPE_RST_CLK_CTRL_PIPE_RST_MASK);
		}

		if (pcie_width != PCIE_LNK_X1) {
			/* disable writing to all lanes with one write */
			if (pcie_width == PCIE_LNK_X2) {
				data = (COMPHY_LANE0 <<
				COMMON_PHY_SD_CTRL1_COMPHY_0_PORT_OFFSET) |
				(COMPHY_LANE1 <<
				COMMON_PHY_SD_CTRL1_COMPHY_1_PORT_OFFSET);
				mask = COMMON_PHY_SD_CTRL1_COMPHY_0_1_PORT_MASK;
			} else if (pcie_width == PCIE_LNK_X4) {
				data = (COMPHY_LANE0 <<
				COMMON_PHY_SD_CTRL1_COMPHY_0_PORT_OFFSET) |
				(COMPHY_LANE1 <<
				COMMON_PHY_SD_CTRL1_COMPHY_1_PORT_OFFSET) |
				(COMPHY_LANE2 <<
				COMMON_PHY_SD_CTRL1_COMPHY_2_PORT_OFFSET) |
				(COMPHY_LANE3 <<
				COMMON_PHY_SD_CTRL1_COMPHY_3_PORT_OFFSET);
				mask = COMMON_PHY_SD_CTRL1_COMPHY_0_3_PORT_MASK;
			}
			reg_set(comphy_base + COMMON_PHY_SD_CTRL1,
				data, mask);
		}

		debug("stage: Check PLL\n");
		/* Read lane status */
		for (i = start_lane; i < end_lane; i++) {
			addr = HPIPE_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base), i) +
				HPIPE_LANE_STATUS1_REG;
			data = HPIPE_LANE_STATUS1_PCLK_EN_MASK;
			mask = data;
			ret = polling_with_timeout(addr, data, mask,
						   PLL_LOCK_TIMEOUT,
						   REG_32BIT);
			if (ret)
				ERROR("Failed to lock PCIE PLL\n");
		}
	}

	debug_exit();

	return ret;
}

static int mvebu_cp110_comphy_rxaui_power_on(uint64_t comphy_base,
				     uint8_t comphy_index, uint32_t comphy_mode)
{
	uintptr_t hpipe_addr, sd_ip_addr, comphy_addr, addr;
	uint32_t mask, data;
	int ret = 0;

	debug_enter();

	hpipe_addr = HPIPE_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base),
				comphy_index);
	comphy_addr = COMPHY_ADDR(comphy_base, comphy_index);
	sd_ip_addr = SD_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base), comphy_index);

	/* configure phy selector for RXAUI */
	mvebu_cp110_comphy_set_phy_selector(comphy_base, comphy_index, comphy_mode);

	/* RFU configurations - hard reset comphy */
	mask = COMMON_PHY_CFG1_PWR_UP_MASK;
	data = 0x1 << COMMON_PHY_CFG1_PWR_UP_OFFSET;
	mask |= COMMON_PHY_CFG1_PIPE_SELECT_MASK;
	data |= 0x0 << COMMON_PHY_CFG1_PIPE_SELECT_OFFSET;
	reg_set(comphy_addr + COMMON_PHY_CFG1_REG, data, mask);

	if (comphy_index == 2) {
		reg_set(comphy_base + COMMON_PHY_SD_CTRL1,
			0x1 << COMMON_PHY_SD_CTRL1_RXAUI0_OFFSET,
			COMMON_PHY_SD_CTRL1_RXAUI0_MASK);
	}
	if (comphy_index == 4) {
		reg_set(comphy_base + COMMON_PHY_SD_CTRL1,
			0x1 << COMMON_PHY_SD_CTRL1_RXAUI1_OFFSET,
			COMMON_PHY_SD_CTRL1_RXAUI1_MASK);
	}

	/* Select Baud Rate of Comphy And PD_PLL/Tx/Rx */
	mask = SD_EXTERNAL_CONFIG0_SD_PU_PLL_MASK;
	data = 0x0 << SD_EXTERNAL_CONFIG0_SD_PU_PLL_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_SD_PHY_GEN_RX_MASK;
	data |= 0xB << SD_EXTERNAL_CONFIG0_SD_PHY_GEN_RX_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_SD_PHY_GEN_TX_MASK;
	data |= 0xB << SD_EXTERNAL_CONFIG0_SD_PHY_GEN_TX_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_SD_PU_RX_MASK;
	data |= 0x0 << SD_EXTERNAL_CONFIG0_SD_PU_RX_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_SD_PU_TX_MASK;
	data |= 0x0 << SD_EXTERNAL_CONFIG0_SD_PU_TX_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_HALF_BUS_MODE_MASK;
	data |= 0x0 << SD_EXTERNAL_CONFIG0_HALF_BUS_MODE_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_MEDIA_MODE_MASK;
	data |= 0x1 << SD_EXTERNAL_CONFIG0_MEDIA_MODE_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG0_REG, data, mask);

	/* release from hard reset */
	mask = SD_EXTERNAL_CONFIG1_RESET_IN_MASK;
	data = 0x0 << SD_EXTERNAL_CONFIG1_RESET_IN_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RESET_CORE_MASK;
	data |= 0x0 << SD_EXTERNAL_CONFIG1_RESET_CORE_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RF_RESET_IN_MASK;
	data |= 0x0 << SD_EXTERNAL_CONFIG1_RF_RESET_IN_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG1_REG, data, mask);

	mask = SD_EXTERNAL_CONFIG1_RESET_IN_MASK;
	data = 0x1 << SD_EXTERNAL_CONFIG1_RESET_IN_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RESET_CORE_MASK;
	data |= 0x1 << SD_EXTERNAL_CONFIG1_RESET_CORE_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG1_REG, data, mask);

	/* Wait 1ms - until band gap and ref clock ready */
	mdelay(1);

	/* Start comphy Configuration */
	debug("stage: Comphy configuration\n");
	/* set reference clock */
	reg_set(hpipe_addr + HPIPE_MISC_REG,
		0x0 << HPIPE_MISC_REFCLK_SEL_OFFSET,
		HPIPE_MISC_REFCLK_SEL_MASK);
	/* Power and PLL Control */
	mask = HPIPE_PWR_PLL_REF_FREQ_MASK;
	data = 0x1 << HPIPE_PWR_PLL_REF_FREQ_OFFSET;
	mask |= HPIPE_PWR_PLL_PHY_MODE_MASK;
	data |= 0x4 << HPIPE_PWR_PLL_PHY_MODE_OFFSET;
	reg_set(hpipe_addr + HPIPE_PWR_PLL_REG, data, mask);
	/* Loopback register */
	reg_set(hpipe_addr + HPIPE_LOOPBACK_REG,
		0x1 << HPIPE_LOOPBACK_SEL_OFFSET, HPIPE_LOOPBACK_SEL_MASK);
	/* rx control 1 */
	mask = HPIPE_RX_CONTROL_1_RXCLK2X_SEL_MASK;
	data = 0x1 << HPIPE_RX_CONTROL_1_RXCLK2X_SEL_OFFSET;
	mask |= HPIPE_RX_CONTROL_1_CLK8T_EN_MASK;
	data |= 0x1 << HPIPE_RX_CONTROL_1_CLK8T_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_RX_CONTROL_1_REG, data, mask);
	/* DTL Control */
	reg_set(hpipe_addr + HPIPE_PWR_CTR_DTL_REG,
		0x0 << HPIPE_PWR_CTR_DTL_FLOOP_EN_OFFSET,
		HPIPE_PWR_CTR_DTL_FLOOP_EN_MASK);

	/* Set analog paramters from ETP(HW) */
	debug("stage: Analog paramters from ETP(HW)\n");
	/* SERDES External Configuration 2 */
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG2_REG,
		0x1 << SD_EXTERNAL_CONFIG2_PIN_DFE_EN_OFFSET,
		SD_EXTERNAL_CONFIG2_PIN_DFE_EN_MASK);
	/* 0x7-DFE Resolution control */
	reg_set(hpipe_addr + HPIPE_DFE_REG0, 0x1 << HPIPE_DFE_RES_FORCE_OFFSET,
		HPIPE_DFE_RES_FORCE_MASK);
	/* 0xd-G1_Setting_0 */
	reg_set(hpipe_addr + HPIPE_G1_SET_0_REG,
		0xd << HPIPE_G1_SET_0_G1_TX_EMPH1_OFFSET,
		HPIPE_G1_SET_0_G1_TX_EMPH1_MASK);
	/* 0xE-G1_Setting_1 */
	mask = HPIPE_G1_SET_1_G1_RX_SELMUPI_MASK;
	data = 0x1 << HPIPE_G1_SET_1_G1_RX_SELMUPI_OFFSET;
	mask |= HPIPE_G1_SET_1_G1_RX_SELMUPP_MASK;
	data |= 0x1 << HPIPE_G1_SET_1_G1_RX_SELMUPP_OFFSET;
	mask |= HPIPE_G1_SET_1_G1_RX_DFE_EN_MASK;
	data |= 0x1 << HPIPE_G1_SET_1_G1_RX_DFE_EN_OFFSET;
	reg_set(hpipe_addr + HPIPE_G1_SET_1_REG, data, mask);
	/* 0xA-DFE_Reg3 */
	mask = HPIPE_DFE_F3_F5_DFE_EN_MASK;
	data = 0x0 << HPIPE_DFE_F3_F5_DFE_EN_OFFSET;
	mask |= HPIPE_DFE_F3_F5_DFE_CTRL_MASK;
	data |= 0x0 << HPIPE_DFE_F3_F5_DFE_CTRL_OFFSET;
	reg_set(hpipe_addr + HPIPE_DFE_F3_F5_REG, data, mask);

	/* 0x111-G1_Setting_4 */
	mask = HPIPE_G1_SETTINGS_4_G1_DFE_RES_MASK;
	data = 0x1 << HPIPE_G1_SETTINGS_4_G1_DFE_RES_OFFSET;
	reg_set(hpipe_addr + HPIPE_G1_SETTINGS_4_REG, data, mask);

	debug("stage: RFU configurations- Power Up PLL,Tx,Rx\n");
	/* SERDES External Configuration */
	mask = SD_EXTERNAL_CONFIG0_SD_PU_PLL_MASK;
	data = 0x1 << SD_EXTERNAL_CONFIG0_SD_PU_PLL_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_SD_PU_RX_MASK;
	data |= 0x1 << SD_EXTERNAL_CONFIG0_SD_PU_RX_OFFSET;
	mask |= SD_EXTERNAL_CONFIG0_SD_PU_TX_MASK;
	data |= 0x1 << SD_EXTERNAL_CONFIG0_SD_PU_TX_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG0_REG, data, mask);


	/* check PLL rx & tx ready */
	addr = sd_ip_addr + SD_EXTERNAL_STATUS0_REG;
	data = SD_EXTERNAL_STATUS0_PLL_RX_MASK |
		SD_EXTERNAL_STATUS0_PLL_TX_MASK;
	mask = data;
	data = polling_with_timeout(addr, data, mask, 15000, REG_32BIT);
	if (data != 0) {
		debug("Read from reg = %lx - value = 0x%x\n",
		      sd_ip_addr + SD_EXTERNAL_STATUS0_REG, data);
		ERROR("SD_EXTERNAL_STATUS0_PLL_RX is %d, SD_EXTERNAL_STATUS0_PLL_TX is %d\n",
		      (data & SD_EXTERNAL_STATUS0_PLL_RX_MASK),
		      (data & SD_EXTERNAL_STATUS0_PLL_TX_MASK));
		ret = -ETIMEDOUT;
	}

	/* RX init */
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG1_REG,
		0x1 << SD_EXTERNAL_CONFIG1_RX_INIT_OFFSET,
		SD_EXTERNAL_CONFIG1_RX_INIT_MASK);

	/* check that RX init done */
	addr = sd_ip_addr + SD_EXTERNAL_STATUS0_REG;
	data = SD_EXTERNAL_STATUS0_RX_INIT_MASK;
	mask = data;
	data = polling_with_timeout(addr, data, mask, 100, REG_32BIT);
	if (data != 0) {
		debug("Read from reg = %lx - value = 0x%x\n",
		      sd_ip_addr + SD_EXTERNAL_STATUS0_REG, data);
		ERROR("SD_EXTERNAL_STATUS0_RX_INIT is 0\n");
		ret = -ETIMEDOUT;
	}

	debug("stage: RF Reset\n");
	/* RF Reset */
	mask =  SD_EXTERNAL_CONFIG1_RX_INIT_MASK;
	data = 0x0 << SD_EXTERNAL_CONFIG1_RX_INIT_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RF_RESET_IN_MASK;
	data |= 0x1 << SD_EXTERNAL_CONFIG1_RF_RESET_IN_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG1_REG, data, mask);

	debug_exit();

	return ret;
}

int mvebu_cp110_comphy_power_on(uint64_t comphy_base, uint64_t comphy_index, uint64_t comphy_mode)
{
	int mode = COMPHY_GET_MODE(comphy_mode);
	int err = 0;

	debug_enter();

	switch (mode) {
	case(COMPHY_SATA_MODE):
		err = mvebu_cp110_comphy_sata_power_on(comphy_base, comphy_index, comphy_mode);
		break;
	case(COMPHY_SGMII_MODE):
	case(COMPHY_HS_SGMII_MODE):
		err = mvebu_cp110_comphy_sgmii_power_on(comphy_base, comphy_index, comphy_mode);
		break;
	/* From comphy perspective, XFI and SFI are the same */
	case (COMPHY_XFI_MODE):
	case (COMPHY_SFI_MODE):
		err = mvebu_cp110_comphy_xfi_power_on(comphy_base, comphy_index, comphy_mode);
		break;
	case (COMPHY_PCIE_MODE):
		err = mvebu_cp110_comphy_pcie_power_on(comphy_base, comphy_index, comphy_mode);
		break;
	case (COMPHY_RXAUI_MODE):
		err = mvebu_cp110_comphy_rxaui_power_on(comphy_base, comphy_index, comphy_mode);
	default:
		ERROR("comphy%ld: unsupported comphy mode\n", comphy_index);
		err = -EINVAL;
		break;
	}

	debug_exit();

	return err;
}

int mvebu_cp110_comphy_power_off(uint64_t comphy_base, uint64_t comphy_index)
{
	uintptr_t sd_ip_addr, comphy_ip_addr;
	uint32_t mask, data;

	debug_enter();

	sd_ip_addr = SD_ADDR(COMPHY_PIPE_FROM_COMPHY_ADDR(comphy_base), comphy_index);
	comphy_ip_addr = COMPHY_ADDR(comphy_base, comphy_index);

	/* Hard reset the comphy, for Ethernet modes and Sata */
	mask = SD_EXTERNAL_CONFIG1_RESET_IN_MASK;
	data = 0x0 << SD_EXTERNAL_CONFIG1_RESET_IN_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RESET_CORE_MASK;
	data |= 0x0 << SD_EXTERNAL_CONFIG1_RESET_CORE_OFFSET;
	mask |= SD_EXTERNAL_CONFIG1_RF_RESET_IN_MASK;
	data |= 0x0 << SD_EXTERNAL_CONFIG1_RF_RESET_IN_OFFSET;
	reg_set(sd_ip_addr + SD_EXTERNAL_CONFIG1_REG, data, mask);

	/* PCIe reset */
	spin_lock(&cp110_mac_reset_lock);

	/* The mvebu_cp110_comphy_power_off will be called only from Linux (to
	 * override settings done by bootloader) and it will be relevant only
	 * to PCIe (called before check if to skip pcie power off or not).
	 */
	data = mmio_read_32(SYS_CTRL_FROM_COMPHY_ADDR(comphy_base) +
						 SYS_CTRL_UINIT_SOFT_RESET_REG);
	switch (comphy_index) {
	case COMPHY_LANE0:
		data &= ~PCIE_MAC_RESET_MASK_PORT0;
		break;
	case COMPHY_LANE4:
		data &= ~PCIE_MAC_RESET_MASK_PORT1;
		break;
	case COMPHY_LANE5:
		data &= ~PCIE_MAC_RESET_MASK_PORT2;
		break;
	}

	mmio_write_32(SYS_CTRL_FROM_COMPHY_ADDR(comphy_base) +
					   SYS_CTRL_UINIT_SOFT_RESET_REG, data);
	spin_unlock(&cp110_mac_reset_lock);

	/* Hard reset the comphy, for PCIe and usb3 */
	mask = COMMON_PHY_CFG1_PWR_ON_RESET_MASK;
	data = 0x0 << COMMON_PHY_CFG1_PWR_ON_RESET_OFFSET;
	mask |= COMMON_PHY_CFG1_CORE_RSTN_MASK;
	data |= 0x0 << COMMON_PHY_CFG1_CORE_RSTN_OFFSET;
	reg_set(comphy_ip_addr + COMMON_PHY_CFG1_REG, data, mask);

	/* Clear comphy PHY and PIPE selector, can't rely on previous config. */
	mvebu_cp110_comphy_clr_phy_selector(comphy_base, comphy_index);
	mvebu_cp110_comphy_clr_pipe_selector(comphy_base, comphy_index);

	debug_exit();

	return 0;
}

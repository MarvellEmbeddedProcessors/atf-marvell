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

#include <plat_marvell.h>
#include <plat_config.h>
#include <plat_def.h>
#include <debug.h>
#include <sys_info.h>
#include <dram_if.h>
#include <ccu.h>
#include <aro.h>
#include <apn806_setup.h>
#include <cp110_setup.h>

/* Register for skip image use */
#define SCRATCH_PAD_REG2		0xF06F00A8
#define SCRATCH_PAD_SKIP_VAL		0x01
#define NUM_OF_GPIO_PER_REG 32

/* CCU windows configuration defines */
#define CCU_CFG_IO_WIN_NUM	(3)
#define CCU_CFG_WIN_REGS_NUM	(4) /* CR + SCR + ALR + AHR */
#define CCU_WIN_TARGET_OFFSET	(8)
#define CCU_WIN_TARGET_MASK	(0xf)
#define CCU_WIN_ADDR_SHIFT	(20)
#define CCU_WIN_ADDR_OFFSET	(4)
#define CCU_WIN_ADDR_MASK	(0xFFFFFFF)
#define CCU_WIN_CR_OFFSET(w)	(0x0 + (w)*0x10)
#define CCU_WIN_SCR_OFFSET(w)	(0x4 + (w)*0x10)
#define CCU_WIN_ALR_OFFSET(w)	(0x8 + (w)*0x10)
#define CCU_WIN_AHR_OFFSET(w)	(0xC + (w)*0x10)
#define CCU_WIN_GCR_OFFSET	(0xD0)

/* IO windows configuration */
#define IOW_CFG_IO_WIN_NUM_ST	(2)
#define IOW_CFG_IO_WIN_NUM_END	(IOW_CFG_IO_WIN_NUM_ST + CP_COUNT - 1)
#define IOW_GCR_OFFSET		(0x70)
#define IOW_WIN_ALR_OFFSET(w)	(0x10 + 0x10*(w))

#define MMAP_SAVE_AND_CONFIG	0
#define MMAP_RESTORE_SAVED	1

/* SAR clock settings */
#define MVEBU_AP_GEN_MGMT_BASE		(MVEBU_RFU_BASE + 0x8000)
#define MVEBU_AP_SAR_REG_BASE(r)	(MVEBU_AP_GEN_MGMT_BASE + 0x200 +\
								((r) << 2))

#define SAR_CLOCK_FREQ_MODE_OFFSET	(0)
#define SAR_CLOCK_FREQ_MODE_MASK	(0x1f << SAR_CLOCK_FREQ_MODE_OFFSET)
#define SAR_PIDI_LOW_SPEED_OFFSET	(20)
#define SAR_PIDI_LOW_SPEED_MASK		(1 << SAR_PIDI_LOW_SPEED_OFFSET)
#define SAR_PIDI_LOW_SPEED_SHIFT	(15)
#define SAR_PIDI_LOW_SPEED_SET		(1 << SAR_PIDI_LOW_SPEED_SHIFT)

#define FREQ_MODE_AP_SAR_REG_NUM	(0)
#define SAR_CLOCK_FREQ_MODE(v)		(((v) & SAR_CLOCK_FREQ_MODE_MASK) >> \
					SAR_CLOCK_FREQ_MODE_OFFSET)

#define AVS_EN_CTRL_REG			(MVEBU_AP_GEN_MGMT_BASE + 0x130)
#define AVS_ENABLE_OFFSET		(0)
#define AVS_SOFT_RESET_OFFSET		(2)
#define AVS_LOW_VDD_LIMIT_OFFSET	(4)
#define AVS_HIGH_VDD_LIMIT_OFFSET	(12)
#define AVS_TARGET_DELTA_OFFSET		(21)
#define AVS_VDD_LOW_LIMIT_MASK	        (0xFF << AVS_LOW_VDD_LIMIT_OFFSET)
#define AVS_VDD_HIGH_LIMIT_MASK	        (0xFF << AVS_HIGH_VDD_LIMIT_OFFSET)
/* VDD limit is 0.9V for A70x0 @ CPU frequency < 1600MHz */
#define AVS_A7K_LOW_CLK_VALUE		((0x80 << AVS_TARGET_DELTA_OFFSET) | \
					 (0x1A << AVS_HIGH_VDD_LIMIT_OFFSET) | \
					 (0x1A << AVS_LOW_VDD_LIMIT_OFFSET) | \
					 (0x1 << AVS_SOFT_RESET_OFFSET) | \
					 (0x1 << AVS_ENABLE_OFFSET))
/* VDD limit is 1.0V for all A80x0 devices */
#define AVS_A8K_CLK_VALUE		((0x80 << AVS_TARGET_DELTA_OFFSET) | \
					 (0x24 << AVS_HIGH_VDD_LIMIT_OFFSET) | \
					 (0x24 << AVS_LOW_VDD_LIMIT_OFFSET) | \
					 (0x1 << AVS_SOFT_RESET_OFFSET) | \
					 (0x1 << AVS_ENABLE_OFFSET))

#define MVEBU_AP_EFUSE_SRV_CTRL_REG	(MVEBU_AP_GEN_MGMT_BASE + 0x8)
#define EFUSE_SRV_CTRL_LD_SELECT_OFFS	6
#define EFUSE_SRV_CTRL_LD_SEL_USER_MASK	(1 << EFUSE_SRV_CTRL_LD_SELECT_OFFS)

/*
 - AVS work points in the LD0 eFuse:
	SVC1 work point:     LD0[88:81]
	SVC2 work point:     LD0[96:89]
	SVC3 work point:     LD0[104:97]
	SVC4 work point:     LD0[112:105]
 - Identification information in the LD-0 eFuse:
	DRO:           LD0[74:65] - Not used by the SW
	Revision:      LD0[78:75] - Not used by the SW
	Bin:           LD0[80:79] - Not used by the SW
	SW Revision:   LD0[115:113]
	Cluster 1 PWR: LD0[193] - if set to 1, power down CPU Cluster-1
				  resulting in 2 CPUs active only (7020)
*/
#define MVEBU_AP_LD_EFUSE_BASE		(MVEBU_AP_GEN_MGMT_BASE + 0xF00)
/* Bits [94:63] - 32 data bits total */
#define MVEBU_AP_LD0_94_63_EFUSE_OFFS	(MVEBU_AP_LD_EFUSE_BASE + 0x8)
/* Bits [125:95] - 31 data bits total, 32nd bit is parity for bits [125:63] */
#define MVEBU_AP_LD0_125_95_EFUSE_OFFS	(MVEBU_AP_LD_EFUSE_BASE + 0xC)
/* Bits [220:189] - 32 data bits total */
#define MVEBU_AP_LD0_220_189_EFUSE_OFFS	(MVEBU_AP_LD_EFUSE_BASE + 0x18)
/* Offsets for the above 2 fields combined into single 64-bit value [125:63] */
#define EFUSE_AP_LD0_DRO_OFFS		2		/* LD0[74:65] */
#define EFUSE_AP_LD0_DRO_MASK		0x3FF
#define EFUSE_AP_LD0_REVID_OFFS		12		/* LD0[78:75] */
#define EFUSE_AP_LD0_REVID_MASK		0xF
#define EFUSE_AP_LD0_BIN_OFFS		16		/* LD0[80:79] */
#define EFUSE_AP_LD0_BIN_MASK		0x3
#define EFUSE_AP_LD0_SWREV_OFFS		50		/* LD0[115:113] */
#define EFUSE_AP_LD0_SWREV_MASK		0x7

#define EFUSE_AP_LD0_SVC1_OFFS		18		/* LD0[88:81] */
#define EFUSE_AP_LD0_SVC2_OFFS		26		/* LD0[96:89] */
#define EFUSE_AP_LD0_SVC3_OFFS		34		/* LD0[104:97] */
#define EFUSE_AP_LD0_SVC4_OFFS		42		/* LD0[112:105] */
#define EFUSE_AP_LD0_WP_MASK		0xFF

#define EFUSE_AP_LD0_CLUSTER_DOWN_OFFS	4

/* Notify bootloader on DRAM setup */
void pass_dram_sys_info(struct dram_config *cfg)
{
	set_info(DRAM_BUS_WIDTH, cfg->iface[0].bus_width);
	set_info(DRAM_CS0_SIZE, cfg->iface[0].size_mbytes);
	set_info(DRAM_CS0, 1);
	set_info(DRAM_CS1, 0);
	set_info(DRAM_CS2, 0);
	set_info(DRAM_CS3, 0);
}
/******************************************************************************
 * The routine allows to save the CCU and IO windows configuration during DRAM
 * setup and restore them afterwards before exiting the BLE stage.
 * Such window configuration is requred since not all default settings coming
 * from the HW and the BootROM akkow access to periferals connected to
 * all available CPn components.
 * For instance, when the boot device is located on CP0, the IO window to CP1
 * is not opened automatically by the HW and if the DRAM SPD is located on CP1
 * i2c channel, it cannot be read at BLE stage.
 * Therefore the DRAM init procedure have to provide access to all available
 * CPn periferals during the BLE stage by setting the CCU IO window to all CPn
 * addresses and by enabling the IO windows accordingly.
 * Additionally this function configures the CCU GCR to DRAM, which allows
 * usage or more than 4GB DRAM as it configured by the default CCU DRAM window.
 *
 * IN:
 *	MMAP_SAVE_AND_CONFIG	- save the existing configuration and update it
 *	MMAP_RESTORE_SAVED	- restore saved configuration
 * OUT:
 *	NONE
 ****************************************************************************
 */
static void ble_plat_mmap_config(int restore)
{
	static uint32_t ccu_win_regs[CCU_CFG_WIN_REGS_NUM];
	static uint32_t io_win_regs[CP_COUNT];
	static uint32_t ccu_gcr, iow_gcr;
	uintptr_t ccu_base = MVEBU_CCU_BASE;
	uintptr_t iow_base = MVEBU_IO_WIN_BASE;
	uint32_t reg_val, win_num;

	if (restore == MMAP_RESTORE_SAVED) {
		/* Restore all orig. settings that were modified by BLE stage */
		/* Restore CCU */
		mmio_write_32(ccu_base + CCU_WIN_CR_OFFSET(CCU_CFG_IO_WIN_NUM),
			      ccu_win_regs[0]);
		mmio_write_32(ccu_base + CCU_WIN_SCR_OFFSET(CCU_CFG_IO_WIN_NUM),
			      ccu_win_regs[1]);
		mmio_write_32(ccu_base + CCU_WIN_ALR_OFFSET(CCU_CFG_IO_WIN_NUM),
			      ccu_win_regs[2]);
		mmio_write_32(ccu_base + CCU_WIN_AHR_OFFSET(CCU_CFG_IO_WIN_NUM),
			      ccu_win_regs[3]);
		mmio_write_32(ccu_base + CCU_WIN_GCR_OFFSET, ccu_gcr);
		/* Restore IO Windows */
		for (win_num = IOW_CFG_IO_WIN_NUM_ST;
		     win_num < IOW_CFG_IO_WIN_NUM_END; win_num++)
			mmio_write_32(iow_base + IOW_WIN_ALR_OFFSET(win_num),
				      io_win_regs[win_num -
				      IOW_CFG_IO_WIN_NUM_ST]);
		mmio_write_32(iow_base + IOW_GCR_OFFSET, iow_gcr);
		return;
	} else {
		/* Store original values */
		/* Save CCU */
		ccu_win_regs[0] = mmio_read_32(ccu_base +
					CCU_WIN_CR_OFFSET(CCU_CFG_IO_WIN_NUM));
		ccu_win_regs[1] = mmio_read_32(ccu_base +
					CCU_WIN_SCR_OFFSET(CCU_CFG_IO_WIN_NUM));
		ccu_win_regs[2] = mmio_read_32(ccu_base +
					CCU_WIN_ALR_OFFSET(CCU_CFG_IO_WIN_NUM));
		ccu_win_regs[3] = mmio_read_32(ccu_base +
					CCU_WIN_AHR_OFFSET(CCU_CFG_IO_WIN_NUM));
		ccu_gcr = mmio_read_32(ccu_base + CCU_WIN_GCR_OFFSET);
		/* Save IO Windows */
		for (win_num = IOW_CFG_IO_WIN_NUM_ST;
		     win_num < IOW_CFG_IO_WIN_NUM_END; win_num++)
			io_win_regs[win_num - IOW_CFG_IO_WIN_NUM_ST] =
				mmio_read_32(iow_base +
					     IOW_WIN_ALR_OFFSET(win_num));
		iow_gcr = mmio_read_32(iow_base + IOW_GCR_OFFSET);
	}

	/* The configuration saved, now all the changes can be done */
	/* Set the default CCU target ID to DRAM 0 */
	reg_val = ccu_gcr & ~(CCU_WIN_TARGET_MASK << CCU_WIN_TARGET_OFFSET);
	reg_val |= (DRAM_0_TID & CCU_WIN_TARGET_MASK) << CCU_WIN_TARGET_OFFSET;
	mmio_write_32(ccu_base + CCU_WIN_GCR_OFFSET, reg_val);

	/* Set CCU IO window for covering all available CP addresses */
	/* Set the CCU IO window Low Address to the start of CP0 region */
	mmio_write_32(ccu_base + CCU_WIN_CR_OFFSET(CCU_CFG_IO_WIN_NUM), 0);
	reg_val = (MVEBU_CP_REGS_BASE(0) >> CCU_WIN_ADDR_SHIFT)
		  << CCU_WIN_ADDR_OFFSET;
	mmio_write_32(ccu_base +
		      CCU_WIN_ALR_OFFSET(CCU_CFG_IO_WIN_NUM), reg_val);

	/*
	 * Set the CCU IO window High Address to the end of
	 * CPn region (n = CP_COUNT)
	 */
	reg_val = (MVEBU_CP_REGS_BASE(CP_COUNT - 1) >> CCU_WIN_ADDR_SHIFT)
		   << CCU_WIN_ADDR_OFFSET;
	reg_val |= 0xF << CCU_WIN_ADDR_OFFSET;
	mmio_write_32(ccu_base + CCU_WIN_AHR_OFFSET(CCU_CFG_IO_WIN_NUM),
		      reg_val);

	/* Set the CCU IO window Control target to IO and enable this window */
	reg_val = (IO_0_TID << CCU_WIN_TARGET_OFFSET) | 0x1;
	mmio_write_32(ccu_base + CCU_WIN_CR_OFFSET(CCU_CFG_IO_WIN_NUM),
		      reg_val);

	/* Set IO windows GCR (IO decode) to PIDI as a default (CP0) */
	mmio_write_32(iow_base + IOW_GCR_OFFSET, PIDI_TID);

	/*
	 * Optionally enable IO windows for CP1, CP2, CP3 (depends on CP_COUNT).
	 * All the rest of windows default settings (ALR, AHR, CR) are already
	 * set by the HW
	 */
	for (win_num = IOW_CFG_IO_WIN_NUM_ST;
	     win_num < IOW_CFG_IO_WIN_NUM_END; win_num++) {
		reg_val = mmio_read_32(iow_base + IOW_WIN_ALR_OFFSET(win_num));
		reg_val |= 0x1;
		mmio_write_32(iow_base + IOW_WIN_ALR_OFFSET(win_num), reg_val);
	}
}

/******************************************************************************
 * Setup Adaptive Voltage Switching - this is required for some platforms
 *****************************************************************************/
static void ble_plat_avs_config(void)
{
	uint32_t reg_val, device_id;

	/* Do nothing on A0 revision SoCs */
	if (apn806_rev_id_get() == APN806_REV_ID_A0)
		return;

	/* Check which SoC is running and act accordingly */
	device_id = cp110_device_id_get();
	switch (device_id) {
	case MVEBU_80X0_DEV_ID:
		/* Set the new AVS value - fix the default one on A80x0 */
		mmio_write_32(AVS_EN_CTRL_REG, AVS_A8K_CLK_VALUE);
		break;

	case MVEBU_70X0_DEV_ID:
		/* Only fix AVS for CPU clocks lower than 1600MHz on A70x0 */
		reg_val = mmio_read_32(MVEBU_AP_SAR_REG_BASE(
						FREQ_MODE_AP_SAR_REG_NUM));
		reg_val &= SAR_CLOCK_FREQ_MODE_MASK;
		reg_val >>= SAR_CLOCK_FREQ_MODE_OFFSET;
		if ((reg_val > CPU_1600_DDR_900_RCLK_900_2) &&
		    (reg_val < CPU_DDR_RCLK_INVALID))
			mmio_write_32(AVS_EN_CTRL_REG, AVS_A7K_LOW_CLK_VALUE);
		break;

	default:
		ERROR("Unsupported Device ID 0x%x\n", device_id);
	}
}

/******************************************************************************
 * SVC flow - v0.8
 * The feature is inteded  to configure AVS value according to eFuse values
 * that are burned individually for each SoC during the test process.
 * Primary AVS value is stored in HD efuse and processed on power on by the HW engine
 * Secondary AVS value is located in LD efuse and contains 4 work points for
 * various CPU frequencies.
 * The Secondary AVS value is only taken into account if the SW Revision stored
 * in the efuse is greater than 0 and the CPU is running in a certain speed.
 *****************************************************************************/
static void ble_plat_svc_config(void)
{
	uint32_t reg_val, avs_workpoint, freq_pidi_mode;
	uint64_t efuse;
	uint32_t device_id, single_cluster;
	uint8_t  svc[4], i, sw_ver;

	/* Set access to LD0 */
	reg_val = mmio_read_32(MVEBU_AP_EFUSE_SRV_CTRL_REG);
	reg_val &= ~EFUSE_SRV_CTRL_LD_SELECT_OFFS;
	mmio_write_32(MVEBU_AP_EFUSE_SRV_CTRL_REG, reg_val);

	/* Obtain the value of LD0[125:63] */
	efuse = mmio_read_32(MVEBU_AP_LD0_125_95_EFUSE_OFFS);
	efuse <<= 32;
	efuse |= mmio_read_32(MVEBU_AP_LD0_94_63_EFUSE_OFFS);

	/* SW Revision:
	 * Starting from SW revision 1 the SVC flow is supported.
	 * SW version 0 (efuse not programmed) should follow the
	 * regular AVS update flow.
	 */
	sw_ver = (efuse >> EFUSE_AP_LD0_SWREV_OFFS) & EFUSE_AP_LD0_SWREV_MASK;
	if (sw_ver < 1) {
		NOTICE("SVC: SW Revision 0x%x. SVC is not supported\n", sw_ver);
		ble_plat_avs_config();
		return;
	}

	/* Frequency mode from SAR */
	freq_pidi_mode = SAR_CLOCK_FREQ_MODE(
				mmio_read_32(
					MVEBU_AP_SAR_REG_BASE(
						FREQ_MODE_AP_SAR_REG_NUM)));

	/* Decode all SVC work points */
	svc[0] = (efuse >> EFUSE_AP_LD0_SVC1_OFFS) & EFUSE_AP_LD0_WP_MASK;
	svc[1] = (efuse >> EFUSE_AP_LD0_SVC2_OFFS) & EFUSE_AP_LD0_WP_MASK;
	svc[2] = (efuse >> EFUSE_AP_LD0_SVC3_OFFS) & EFUSE_AP_LD0_WP_MASK;
	svc[3] = (efuse >> EFUSE_AP_LD0_SVC4_OFFS) & EFUSE_AP_LD0_WP_MASK;
	INFO("SVC: Efuse WP: [0]=0x%x, [1]=0x%x, [2]=0x%x, [3]=0x%x\n",
		svc[0], svc[1], svc[2], svc[3]);

	/* Validate parity of SVC workpoint values */
	for (i = 0; i < 4; i++) {
		uint8_t parity, bit;

		for (bit = 1, parity = svc[i] & 1; bit < 7; bit++)
			parity ^= (svc[i] >> bit) & 1;

		if (parity != ((svc[i] >> 7) & 1)) {
			/* Starting from SW version 2,
			 * the parity check is mandatory
			 */
			if (sw_ver > 1) {
				ERROR("Failed SVC WP[%d] parity check!\n", i);
				ERROR("Ignoring the WP values\n");
				return;
			}
		}
	}

	single_cluster = mmio_read_32(MVEBU_AP_LD0_220_189_EFUSE_OFFS);
	single_cluster = (single_cluster >> EFUSE_AP_LD0_CLUSTER_DOWN_OFFS) & 1;

	device_id = cp110_device_id_get();
	if (device_id == MVEBU_80X0_DEV_ID) {
		/* A8040/A8020 */
		NOTICE("SVC: DEV ID: %s, FREQ Mode: 0x%x\n",
			single_cluster == 0 ? "8040" : "8020", freq_pidi_mode);
		switch (freq_pidi_mode) {
		case CPU_2000_DDR_1200_RCLK_1200:
		case CPU_2000_DDR_1050_RCLK_1050:
			avs_workpoint = svc[0];
			/* MAX speed is already in HD efuse,
			 * the AVS should not be updated
			 */
			if (avs_workpoint != 0) {
				NOTICE("SVC: AVS work point was not changed\n");
				return;
			}
			break;
		case CPU_1800_DDR_1200_RCLK_1200:
		case CPU_1800_DDR_1050_RCLK_1050:
			avs_workpoint = svc[1];
			break;
		case CPU_1600_DDR_1050_RCLK_1050:
		case CPU_1600_DDR_900_RCLK_900_2:
			avs_workpoint = svc[2];
			break;
		case CPU_1300_DDR_800_RCLK_800:
		case CPU_1300_DDR_650_RCLK_650:
			avs_workpoint = svc[3];
			break;
		default:
			avs_workpoint = 0;
			break;
		}
	} else if (device_id == MVEBU_70X0_DEV_ID) {
		/* A7040/A7020/A6040 */
		NOTICE("SVC: DEV ID: %s, FREQ Mode: 0x%x\n",
			single_cluster == 0 ? "7040" : "7020", freq_pidi_mode);
		switch (freq_pidi_mode) {
		case CPU_1400_DDR_800_RCLK_800:
			avs_workpoint = svc[0];
			if (avs_workpoint == 0)
				break;
			/* MAX speed is already in HD efuse,
			 * the AVS should not be updated for 7040/6040
			 */
			if (!single_cluster) {
				NOTICE("SVC: AVS work point was not changed\n");
				return;
			}
			break;
		case CPU_1200_DDR_800_RCLK_800:
			avs_workpoint = svc[1];
			break;
		case CPU_800_DDR_800_RCLK_800:
		case CPU_1000_DDR_800_RCLK_800:
			avs_workpoint = svc[2];
			break;
		case CPU_600_DDR_800_RCLK_800:
			avs_workpoint = svc[3];
			break;
		default:
			avs_workpoint = 0;
			break;
		}
	} else {
		ERROR("SVC: Unsupported Device ID 0x%x\n", device_id);
		return;
	}

	/* Set AVS control if needed */
	if (avs_workpoint == 0) {
		ERROR("SVC: Wrong SAR 0x%x for this BIN\n", freq_pidi_mode);
		return;
	}

	/* Remove parity bit */
	if (sw_ver > 1)
		avs_workpoint &= 0x7F;

	reg_val  = mmio_read_32(AVS_EN_CTRL_REG);
	NOTICE("SVC: AVS work point changed from 0x%x to 0x%x\n",
		(reg_val & AVS_VDD_LOW_LIMIT_MASK) >> AVS_LOW_VDD_LIMIT_OFFSET,
		avs_workpoint);
	reg_val &= ~(AVS_VDD_LOW_LIMIT_MASK | AVS_VDD_HIGH_LIMIT_MASK);
	reg_val |= 0x1 << AVS_ENABLE_OFFSET;
	reg_val |= avs_workpoint << AVS_HIGH_VDD_LIMIT_OFFSET;
	reg_val |= avs_workpoint << AVS_LOW_VDD_LIMIT_OFFSET;
	mmio_write_32(AVS_EN_CTRL_REG, reg_val);
}

static int ble_skip_image_i2c(struct skip_image *skip_im)
{
	ERROR("skipping image using i2c is not supported\n");
	/* not supported */
	return 0;
}

static int ble_skip_image_other(struct skip_image *skip_im)
{
	ERROR("implementation missing for skip image request\n");
	/* not supported, make your own implementation */
	return 0;
}

static int ble_skip_image_gpio(struct skip_image *skip_im)
{
	unsigned int val;
	unsigned int mpp_address = 0;
	unsigned int offset = 0;

	switch (skip_im->info.test.cp_ap) {
	case(CP):
		mpp_address = MVEBU_CP_GPIO_DATA_IN(skip_im->info.test.cp_index,
						    skip_im->info.gpio.num);
		if (skip_im->info.gpio.num > NUM_OF_GPIO_PER_REG)
			offset = skip_im->info.gpio.num - NUM_OF_GPIO_PER_REG;
		else
			offset = skip_im->info.gpio.num;
		break;
	case(AP):
		mpp_address = MVEBU_AP_GPIO_DATA_IN;
		offset = skip_im->info.gpio.num;
		break;
	}

	val = mmio_read_32(mpp_address);
	val &= (1 << offset);
	if ((!val && skip_im->info.gpio.button_state == HIGH) ||
	    (val && skip_im->info.gpio.button_state == LOW)) {
		mmio_write_32(SCRATCH_PAD_REG2, SCRATCH_PAD_SKIP_VAL);
		return 1;
	}

	return 0;
}

/*
 * This function checks if there's a skip image request:
 * return values:
 * 1: (true) images request been made.
 * 0: (false) no image request been made.
 */
static int  ble_skip_current_image(void)
{
	struct skip_image *skip_im;

	/*fetching skip image info*/
	skip_im = (struct skip_image *)plat_get_skip_image_data();

	if (skip_im == NULL)
		return 0;

	/* check if skipping image request has already been made */
	if (mmio_read_32(SCRATCH_PAD_REG2) == SCRATCH_PAD_SKIP_VAL)
		return 0;

	switch (skip_im->detection_method) {
	case GPIO:
		return ble_skip_image_gpio(skip_im);
	case I2C:
		return ble_skip_image_i2c(skip_im);
	case USER_DEFINED:
		return ble_skip_image_other(skip_im);
	}

	return 0;
}

int ble_plat_setup(int *skip)
{
	int ret;
	struct dram_config *cfg;

	/* Power down unused CPUs */
	plat_marvell_early_cpu_powerdown();

	/*
	 * Save the current CCU configuration and make required changes:
	 * - Allow access to DRAM larger than 4GB
	 * - Open memory access to all CPn periferals
	 */
	ble_plat_mmap_config(MMAP_SAVE_AND_CONFIG);

	/* Check if there's a skip request to bootRom recovey Image */
	if (ble_skip_current_image()) {
		/* close memory access to all CPn periferals. */
		ble_plat_mmap_config(MMAP_RESTORE_SAVED);
		*skip = 1;
		return 0;
	}

	/* Do required CP-110 setups for BLE stage */
	cp110_ble_init(0);

	/* Setup AVS */
	ble_plat_svc_config();

#if ARO_ENABLE
	init_aro();
#endif

	/* Get dram data from platform */
	cfg = (struct dram_config *)plat_get_dram_data();

	/* Kick it in */
	ret = dram_init(cfg);

	/* Restore the original CCU configuration before exit from BLE */
	ble_plat_mmap_config(MMAP_RESTORE_SAVED);

	/* Pass DRAM information to bootloader */
	pass_dram_sys_info(cfg);

	return ret;
}

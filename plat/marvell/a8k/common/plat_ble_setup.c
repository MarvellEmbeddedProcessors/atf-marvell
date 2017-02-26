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
#include <rfu.h>
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

#define FREQ_MODE_AP_SAR_REG_NUM	(0)
#define SAR_CLOCK_FREQ_MODE_OFFSET	(0)
#define SAR_CLOCK_FREQ_MODE_MASK	(0x1f << SAR_CLOCK_FREQ_MODE_OFFSET)

#define AVS_EN_CTRL_REG			(MVEBU_AP_GEN_MGMT_BASE + 0x130)
#define AVS_ENABLE_OFFSET		(0)
#define AVS_SOFT_RESET_OFFSET		(2)
#define AVS_LOW_VDD_LIMIT_OFFSET	(4)
#define AVS_HIGH_VDD_LIMIT_OFFSET	(12)
#define AVS_TARGET_DELTA_OFFSET		(21)
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

enum cpu_clock_freq_mode {
	CPU_2000_DDR_1200_RCLK_1200 = 0x0,
	CPU_2000_DDR_1050_RCLK_1050 = 0x1,
	CPU_1600_DDR_800_RCLK_800   = 0x4,
	CPU_1800_DDR_1200_RCLK_1200 = 0x6,
	CPU_1800_DDR_1050_RCLK_1050 = 0x7,
	CPU_1600_DDR_900_RCLK_900   = 0x0B,
	CPU_1600_DDR_1050_RCLK_1050 = 0x0D,
	CPU_1600_DDR_900_RCLK_900_2 = 0x0E,
	CPU_1000_DDR_650_RCLK_650   = 0x13,
	CPU_1300_DDR_800_RCLK_800   = 0x14,
	CPU_1300_DDR_650_RCLK_650   = 0x17,
	CPU_1200_DDR_800_RCLK_800   = 0x19,
	CPU_1400_DDR_800_RCLK_800   = 0x1a,
	CPU_600_DDR_800_RCLK_800    = 0x1B,
	CPU_800_DDR_800_RCLK_800    = 0x1C,
	CPU_1000_DDR_800_RCLK_800   = 0x1D,
	CPU_DDR_RCLK_INVALID
};

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
	uintptr_t iow_base = MVEBU_RFU_BASE;
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
	ble_plat_avs_config();

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

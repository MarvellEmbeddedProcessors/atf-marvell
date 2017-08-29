/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <plat_marvell.h>
#include <plat_config.h>
#include <plat_def.h>
#include <debug.h>
#include <dram_if.h>
#include <ap810_setup.h>
#include <cp110_setup.h>

/* Register for skip image use */
#define SCRATCH_PAD_REG2		0xF06F00A8
#define SCRATCH_PAD_SKIP_VAL		0x01
#define NUM_OF_GPIO_PER_REG		32

static void ble_plat_svc_config(void)
{
	INFO("%s: is not implemented yet\n", __func__);
	return;
}

static int ble_skip_image_i2c(struct skip_image *skip_im)
{
	/* not supported */
	ERROR("skipping image using i2c is not supported\n");
	return 0;
}

static int ble_skip_image_other(struct skip_image *skip_im)
{
	/* not supported, make your own implementation */
	ERROR("implementation missing for skip image request\n");
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
		mpp_address = MVEBU_AP_GPIO_DATA_IN(0);
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

	/* fetching skip image info */
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

	/* TODO: need to check if need early cpu powerdown */

	/* TODO: Check if need to save window configuration (CCU) */

	/* Check if there's a skip request to bootRom recovey Image */
	if (ble_skip_current_image()) {
		*skip = 1;
		return 0;
	}

	/* Do required CP-110 setups for BLE stage */
	cp110_ble_init(0);

	/* Setup SVC configuration */
	ble_plat_svc_config();

#if ARO_ENABLE
	init_aro();
#endif

	/* TODO: Add DRAM initialization call */

	/* TODO: Check if need to restore window configuration (CCU) */

	return ret;
}

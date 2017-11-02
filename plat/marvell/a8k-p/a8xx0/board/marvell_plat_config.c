/*
 * Copyright (C) 2017 Marvell International Ltd.
 *
 * SPDX-License-Identifier:	BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <plat_config.h>
/*
 * If bootrom is currently at BLE there's no need to include the memory
 * maps structure at this point
 */
#ifndef IMAGE_BLE
#include <plat_def.h>

/*******************************************************************************
 * IO WIN Configuration
 ******************************************************************************/
struct io_win *io_win_memory_map = NULL;

uint32_t marvell_get_io_win_gcr_target(int ap_index)
{
	return MCI_2_TID;
}

int marvell_get_io_win_memory_map(int ap_index, struct io_win **win, uint32_t *size)
{
	*win = io_win_memory_map;
	if (*win == NULL)
		*size = 0;
	else
		*size = sizeof(io_win_memory_map)/sizeof(struct io_win);

	return 0;
}

/*******************************************************************************
 * CCU Configuration
 ******************************************************************************/
struct ccu_win *ccu_memory_map = NULL;

uint32_t marvell_get_ccu_gcr_target(int ap)
{
	return DRAM_0_TID;
}

int marvell_get_ccu_memory_map(int ap, struct ccu_win **win, uint32_t *size)
{
	*win = ccu_memory_map;
	*size = sizeof(ccu_memory_map)/sizeof(struct ccu_win);

	return 0;
}

/*******************************************************************************
 * IOB Configuration
 ******************************************************************************/
#define MARVELL_IOB_MAX_WIN		16

struct iob_win *iob_memory_map = NULL;

int marvell_get_iob_max_win(void)
{
	return MARVELL_IOB_MAX_WIN;
}

int marvell_get_iob_memory_map(struct iob_win **win, uint32_t *size, int cp_index)
{
	*win = iob_memory_map;
	*size = sizeof(iob_memory_map)/sizeof(struct iob_win);

	return 0;
}

/*******************************************************************************
 * AMB Configuration
 ******************************************************************************/
struct amb_win *amb_memory_map = NULL;

int marvell_get_amb_memory_map(struct amb_win **win, uint32_t *size)
{
	*win = amb_memory_map;
	if (*win == NULL)
		*size = 0;
	else
		*size = sizeof(amb_memory_map)/sizeof(struct amb_win);

	return 0;
}

/*******************************************************************************
 * SoC PM configuration
 ******************************************************************************/
/* CP GPIO should be used and the GPIOs should be within same GPIO register */
struct power_off_method *pm_cfg = NULL;

void *plat_get_pm_cfg(void)
{
	/* Return the PM configurations */
	return &pm_cfg;
}

/* In reference to #ifndef IMAGE_BLE, this part is used for BLE only. */
#else

/*******************************************************************************
 * SKIP IMAGE Configuration
 ******************************************************************************/
void *plat_get_skip_image_data(void)
{
	return 0;
}
#endif

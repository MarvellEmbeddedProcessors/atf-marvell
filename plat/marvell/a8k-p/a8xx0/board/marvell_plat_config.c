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

/* This array describe how CPx connect to APx, via which MCI interface
** For AP0: CP0 connected via MCI3
**          CP1 connected via MCI2
**          CP2 connected via MCI1
**          CP3 connected via MCI0
** For AP1: CP0 connected via MCI4
**          CP1 connected via MCI5
**          CP2 connected via MCI6
**          CP3 connected via MCI7
** */
int ap0_mci_connect_cps[] = {3, 2, 1, 0};
int ap1_mci_connect_cps[] = {4, 5, 6, 7};

/* Return the MCI index that connect cp_id in ap_id */
int marvell_get_mci_map(int ap_id, int cp_id)
{
	if (ap_id == 1)
		return ap1_mci_connect_cps[cp_id];
	else
		return ap0_mci_connect_cps[cp_id];
}

/*******************************************************************************
 * GWIN Configuration
 ******************************************************************************/
struct addr_map_win *gwin_memory_map = NULL;

int marvell_get_gwin_memory_map(int ap, struct addr_map_win **win, uint32_t *size)
{
	*win = gwin_memory_map;
	if (*win == NULL)
		*size = 0;
	else
		*size = sizeof(gwin_memory_map)/sizeof(gwin_memory_map[0]);

	return 0;
}

/*******************************************************************************
 * CCU Configuration
 ******************************************************************************/
struct addr_map_win *ccu_memory_map = NULL;

uint32_t marvell_get_ccu_gcr_target(int ap)
{
	return DRAM_0_TID;
}

int marvell_get_ccu_memory_map(int ap, struct addr_map_win **win, uint32_t *size)
{
	*win = ccu_memory_map;
	if (*win == NULL)
		*size = 0;
	else
		*size = sizeof(ccu_memory_map)/sizeof(ccu_memory_map[0]);

	return 0;
}

/*******************************************************************************
 * IO WIN Configuration
 ******************************************************************************/
struct addr_map_win *io_win_memory_map = NULL;

uint32_t marvell_get_io_win_gcr_target(int ap_index)
{
	return MCI_2_TID;
}

int marvell_get_io_win_memory_map(int ap_index, struct addr_map_win **win, uint32_t *size)
{
	*win = io_win_memory_map;
	if (*win == NULL)
		*size = 0;
	else
		*size = sizeof(io_win_memory_map)/sizeof(io_win_memory_map[0]);

	return 0;
}

/*******************************************************************************
 * IOB Configuration
 ******************************************************************************/
struct addr_map_win *iob_memory_map = NULL;

int marvell_get_iob_memory_map(struct addr_map_win **win, uint32_t *size, uintptr_t base)
{
	*win = iob_memory_map;
	if (*win == NULL)
		*size = 0;
	else
		*size = sizeof(iob_memory_map)/sizeof(iob_memory_map[0]);

	return 0;
}

/*******************************************************************************
 * AMB Configuration
 ******************************************************************************/
struct addr_map_win *amb_memory_map = NULL;

int marvell_get_amb_memory_map(struct addr_map_win **win, uint32_t *size)
{
	*win = amb_memory_map;
	if (*win == NULL)
		*size = 0;
	else
		*size = sizeof(amb_memory_map)/sizeof(amb_memory_map[0]);

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

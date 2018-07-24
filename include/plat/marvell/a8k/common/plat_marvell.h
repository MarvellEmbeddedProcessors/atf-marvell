/*
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */

#ifndef __PLAT_MARVELL_H__
#define __PLAT_MARVELL_H__

#include <cassert.h>
#include <cpu_data.h>
#include <stdint.h>
#include <utils.h>
#include <xlat_tables.h>

/*
 * Extern declarations common to Marvell standard platforms
 */
extern const mmap_region_t plat_marvell_mmap[];

#define MARVELL_CASSERT_MMAP						\
	CASSERT((ARRAY_SIZE(plat_marvell_mmap) + MARVELL_BL_REGIONS)	\
		<= MAX_MMAP_REGIONS,					\
		assert_max_mmap_regions)

/*
 * Utility functions common to Marvell standard platforms
 */
void marvell_setup_page_tables(uintptr_t total_base,
			       size_t total_size,
			       uintptr_t code_start,
			       uintptr_t code_limit,
			       uintptr_t rodata_start,
			       uintptr_t rodata_limit
#if USE_COHERENT_MEM
			     , uintptr_t coh_start,
			       uintptr_t coh_limit
#endif
);

/* IO storage utility functions */
void marvell_io_setup(void);

/* Systimer utility function */
void marvell_configure_sys_timer(void);

/* Topology utility function */
int marvell_check_mpidr(u_register_t mpidr);

/* BLE utility functions */
int ble_plat_setup(int *skip);
void plat_marvell_dram_update_topology(void);
void ble_plat_pcie_ep_setup(void);
struct pci_hw_cfg *plat_get_pcie_hw_data(void);

/* BL1 utility functions */
void marvell_bl1_early_platform_setup(void);
void marvell_bl1_platform_setup(void);
void marvell_bl1_plat_arch_setup(void);

/* BL2 utility functions */
void marvell_bl2_early_platform_setup(meminfo_t *mem_layout);
void marvell_bl2_platform_setup(void);
void marvell_bl2_plat_arch_setup(void);
uint32_t marvell_get_spsr_for_bl32_entry(void);
uint32_t marvell_get_spsr_for_bl33_entry(void);

/* BL31 utility functions */
void marvell_bl31_early_platform_setup(bl31_params_t *from_bl2,
				void *plat_params_from_bl2);
void marvell_bl31_platform_setup(void);
void marvell_bl31_plat_runtime_setup(void);
void marvell_bl31_plat_arch_setup(void);

/* Power management config to power off the SoC */
void *plat_marvell_get_pm_cfg(void);

/* Check if MSS AP CM3 firmware contains PM support */
_Bool is_pm_fw_running(void);

/* Bootrom image recovery utility functions */
void *plat_marvell_get_skip_image_data(void);

/* FIP TOC validity check */
int marvell_io_is_toc_valid(void);

/*
 * PSCI functionality
 */
void marvell_psci_arch_init(int ap_idx);
void plat_marvell_system_reset(void);

/*
 * Optional functions required in Marvell standard platforms
 */
void plat_marvell_io_setup(void);
int plat_marvell_get_alt_image_source(
	unsigned int image_id,
	uintptr_t *dev_handle,
	uintptr_t *image_spec);
unsigned int plat_marvell_calc_core_pos(u_register_t mpidr);

#if PALLADIUM
void marvell_bl1_setup_mpps(void);
#endif

const mmap_region_t *plat_marvell_get_mmap(void);
void marvell_ble_prepare_exit(void);
void marvell_exit_bootrom(uintptr_t base);

int plat_marvell_early_cpu_powerdown(void);
#endif /* __PLAT_MARVELL_H__ */

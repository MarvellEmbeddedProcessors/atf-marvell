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
void *plat_get_dram_data(void);
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
void *plat_get_pm_cfg(void);

/* Bootrom image recovery utility functions */
void *plat_get_skip_image_data(void);

/* FIP TOC validity check */
int marvell_io_is_toc_valid(void);

/*
 * PSCI functionality
 */
void psci_arch_init(int);
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
void ble_prepare_exit(void);
void exit_bootrom(uintptr_t);

int plat_marvell_early_cpu_powerdown(void);
#endif /* __PLAT_MARVELL_H__ */

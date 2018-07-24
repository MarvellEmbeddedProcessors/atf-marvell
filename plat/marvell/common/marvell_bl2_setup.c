/*
 * Copyright (C) 2018 Marvell International Ltd.
 *
 * SPDX-License-Identifier:     BSD-3-Clause
 * https://spdx.org/licenses
 */

#include <arch_helpers.h>
#include <bl_common.h>
#include <console.h>
#include <marvell_def.h>
#include <platform_def.h>
#include <plat_marvell.h>
#include <string.h>

/* Data structure which holds the extents of the trusted SRAM for BL2 */
static meminfo_t bl2_tzram_layout __aligned(CACHE_WRITEBACK_GRANULE);


/*****************************************************************************
 * This structure represents the superset of information that is passed to
 * BL31, e.g. while passing control to it from BL2, bl31_params
 * and other platform specific parameters
 *****************************************************************************
 */
typedef struct bl2_to_bl31_params_mem {
	bl31_params_t bl31_params;
	image_info_t bl31_image_info;
	image_info_t bl32_image_info;
	image_info_t bl33_image_info;
	entry_point_info_t bl33_ep_info;
	entry_point_info_t bl32_ep_info;
	entry_point_info_t bl31_ep_info;
} bl2_to_bl31_params_mem_t;


static bl2_to_bl31_params_mem_t bl31_params_mem;


/* Weak definitions may be overridden in specific MARVELL standard platform */
#pragma weak bl2_early_platform_setup
#pragma weak bl2_platform_setup
#pragma weak bl2_plat_arch_setup
#pragma weak bl2_plat_sec_mem_layout
#pragma weak bl2_plat_get_bl31_params
#pragma weak bl2_plat_get_bl31_ep_info
#pragma weak bl2_plat_flush_bl31_params
#pragma weak bl2_plat_set_bl31_ep_info
#pragma weak bl2_plat_get_scp_bl2_meminfo
#pragma weak bl2_plat_get_bl32_meminfo
#pragma weak bl2_plat_set_bl32_ep_info
#pragma weak bl2_plat_get_bl33_meminfo
#pragma weak bl2_plat_set_bl33_ep_info


meminfo_t *bl2_plat_sec_mem_layout(void)
{
	return &bl2_tzram_layout;
}

/*****************************************************************************
 * This function assigns a pointer to the memory that the platform has kept
 * aside to pass platform specific and trusted firmware related information
 * to BL31. This memory is allocated by allocating memory to
 * bl2_to_bl31_params_mem_t structure which is a superset of all the
 * structure whose information is passed to BL31
 * NOTE: This function should be called only once and should be done
 * before generating params to BL31
 *****************************************************************************
 */
bl31_params_t *bl2_plat_get_bl31_params(void)
{
	bl31_params_t *bl2_to_bl31_params;

	/*
	 * Initialise the memory for all the arguments that needs to
	 * be passed to BL31
	 */
	memset(&bl31_params_mem, 0, sizeof(bl2_to_bl31_params_mem_t));

	/* Assign memory for TF related information */
	bl2_to_bl31_params = &bl31_params_mem.bl31_params;
	SET_PARAM_HEAD(bl2_to_bl31_params, PARAM_BL31, VERSION_1, 0);

	/* Fill BL31 related information */
	bl2_to_bl31_params->bl31_image_info = &bl31_params_mem.bl31_image_info;
	SET_PARAM_HEAD(bl2_to_bl31_params->bl31_image_info, PARAM_IMAGE_BINARY,
		VERSION_1, 0);

	/* Fill BL32 related information if it exists */
#if BL32_BASE
	bl2_to_bl31_params->bl32_ep_info = &bl31_params_mem.bl32_ep_info;
	SET_PARAM_HEAD(bl2_to_bl31_params->bl32_ep_info, PARAM_EP,
		VERSION_1, 0);
	bl2_to_bl31_params->bl32_image_info = &bl31_params_mem.bl32_image_info;
	SET_PARAM_HEAD(bl2_to_bl31_params->bl32_image_info, PARAM_IMAGE_BINARY,
		VERSION_1, 0);
#endif

	/* Fill BL33 related information */
	bl2_to_bl31_params->bl33_ep_info = &bl31_params_mem.bl33_ep_info;
	SET_PARAM_HEAD(bl2_to_bl31_params->bl33_ep_info,
		PARAM_EP, VERSION_1, 0);

	/* BL33 expects to receive the primary CPU MPID (through x0) */
	bl2_to_bl31_params->bl33_ep_info->args.arg0 = 0xffff & read_mpidr();

	bl2_to_bl31_params->bl33_image_info = &bl31_params_mem.bl33_image_info;
	SET_PARAM_HEAD(bl2_to_bl31_params->bl33_image_info, PARAM_IMAGE_BINARY,
		VERSION_1, 0);

	return bl2_to_bl31_params;
}

/* Flush the TF params and the TF plat params */
void bl2_plat_flush_bl31_params(void)
{
	flush_dcache_range((unsigned long)&bl31_params_mem,
			sizeof(bl2_to_bl31_params_mem_t));
}

/*****************************************************************************
 * This function returns a pointer to the shared memory that the platform
 * has kept to point to entry point information of BL31 to BL2
 *****************************************************************************
 */
struct entry_point_info *bl2_plat_get_bl31_ep_info(void)
{
#if DEBUG
	bl31_params_mem.bl31_ep_info.args.arg1 = MARVELL_BL31_PLAT_PARAM_VAL;
#endif

	return &bl31_params_mem.bl31_ep_info;
}

/*****************************************************************************
 * BL1 has passed the extents of the trusted SRAM that should be visible to BL2
 * in x0. This memory layout is sitting at the base of the free trusted SRAM.
 * Copy it to a safe location before its reclaimed by later BL2 functionality.
 *****************************************************************************
 */
void marvell_bl2_early_platform_setup(meminfo_t *mem_layout)
{
	/* Initialize the console to provide early debug support */
	console_init(PLAT_MARVELL_BOOT_UART_BASE,
		     PLAT_MARVELL_BOOT_UART_CLK_IN_HZ,
		     MARVELL_CONSOLE_BAUDRATE);

	/* Setup the BL2 memory layout */
	bl2_tzram_layout = *mem_layout;

	/* Initialise the IO layer and register platform IO devices */
	plat_marvell_io_setup();
}

void bl2_early_platform_setup(meminfo_t *mem_layout)
{
	marvell_bl2_early_platform_setup(mem_layout);
}

void bl2_platform_setup(void)
{
	/* Nothing to do */
}

/*****************************************************************************
 * Perform the very early platform specific architectural setup here. At the
 * moment this is only initializes the mmu in a quick and dirty way.
 *****************************************************************************
 */
void marvell_bl2_plat_arch_setup(void)
{
	marvell_setup_page_tables(bl2_tzram_layout.total_base,
				  bl2_tzram_layout.total_size,
				  BL_CODE_BASE,
				  BL_CODE_END,
				  BL_RO_DATA_BASE,
				  BL_RO_DATA_END
#if USE_COHERENT_MEM
				, BL_COHERENT_RAM_BASE,
				  BL_COHERENT_RAM_END
#endif
			      );
	enable_mmu_el1(0);
}

void bl2_plat_arch_setup(void)
{
	marvell_bl2_plat_arch_setup();
}

/*****************************************************************************
 * Populate the extents of memory available for loading SCP_BL2 (if used),
 * i.e. anywhere in trusted RAM as long as it doesn't overwrite BL2.
 *****************************************************************************
 */
void bl2_plat_get_scp_bl2_meminfo(meminfo_t *scp_bl2_meminfo)
{
	*scp_bl2_meminfo = bl2_tzram_layout;
}

/*****************************************************************************
 * Before calling this function BL31 is loaded in memory and its entrypoint
 * is set by load_image. This is a placeholder for the platform to change
 * the entrypoint of BL31 and set SPSR and security state.
 * On MARVELL std. platforms we only set the security state of the entrypoint
 *****************************************************************************
 */
void bl2_plat_set_bl31_ep_info(image_info_t *bl31_image_info,
			       entry_point_info_t *bl31_ep_info)
{
	SET_SECURITY_STATE(bl31_ep_info->h.attr, SECURE);
	bl31_ep_info->spsr = SPSR_64(MODE_EL3, MODE_SP_ELX,
					DISABLE_ALL_EXCEPTIONS);
}

/*****************************************************************************
 * Populate the extents of memory available for loading BL32
 *****************************************************************************
 */
#ifdef BL32_BASE
void bl2_plat_get_bl32_meminfo(meminfo_t *bl32_meminfo)
{
	/*
	 * Populate the extents of memory available for loading BL32.
	 */
	bl32_meminfo->total_base = BL32_BASE;
	bl32_meminfo->free_base = BL32_BASE;
	bl32_meminfo->total_size =
			(TRUSTED_DRAM_BASE + TRUSTED_DRAM_SIZE) - BL32_BASE;
	bl32_meminfo->free_size =
			(TRUSTED_DRAM_BASE + TRUSTED_DRAM_SIZE) - BL32_BASE;
}
#endif

/*****************************************************************************
 * Before calling this function BL32 is loaded in memory and its entrypoint
 * is set by load_image. This is a placeholder for the platform to change
 * the entrypoint of BL32 and set SPSR and security state.
 * On MARVELL std. platforms we only set the security state of the entrypoint
 *****************************************************************************
 */
void bl2_plat_set_bl32_ep_info(image_info_t *bl32_image_info,
			       entry_point_info_t *bl32_ep_info)
{
	SET_SECURITY_STATE(bl32_ep_info->h.attr, SECURE);
	bl32_ep_info->spsr = marvell_get_spsr_for_bl32_entry();
}

/*****************************************************************************
 * Before calling this function BL33 is loaded in memory and its entrypoint
 * is set by load_image. This is a placeholder for the platform to change
 * the entrypoint of BL33 and set SPSR and security state.
 * On MARVELL std. platforms we only set the security state of the entrypoint
 *****************************************************************************
 */
void bl2_plat_set_bl33_ep_info(image_info_t *image,
			       entry_point_info_t *bl33_ep_info)
{

	SET_SECURITY_STATE(bl33_ep_info->h.attr, NON_SECURE);
	bl33_ep_info->spsr = marvell_get_spsr_for_bl33_entry();
}

/*****************************************************************************
 * Populate the extents of memory available for loading BL33
 *****************************************************************************
 */
void bl2_plat_get_bl33_meminfo(meminfo_t *bl33_meminfo)
{
	bl33_meminfo->total_base = MARVELL_DRAM_BASE;
	bl33_meminfo->total_size = MARVELL_DRAM_SIZE;
	bl33_meminfo->free_base = MARVELL_DRAM_BASE;
	bl33_meminfo->free_size = MARVELL_DRAM_SIZE;
}

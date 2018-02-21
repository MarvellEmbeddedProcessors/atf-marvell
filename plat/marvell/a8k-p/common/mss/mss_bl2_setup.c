/*
 * ***************************************************************************
 * Copyright (C) 2018 Marvell International Ltd.
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

#include <bl_common.h>
#include <debug.h>
#include <plat_private.h> /* timer functionality */
#include "mss_scp_bootloader.h"
#include <platform_def.h>
#include <mmio.h>
#include <ap810_setup.h>

#define MSS_AP_REG_BASE			0x580000
#define MSS_CP_REG_BASE			0x280000

/* MSS windows configuration */
#define MSS_AEBR(base)			(base + 0x160)
#define MSS_AIBR(base)			(base + 0x164)
#define MSS_AEBR_MASK			0xFFF
#define MSS_AIBR_MASK			0xFFF

#define MSS_EXTERNAL_SPACE		0x50000000
#define MSS_EXTERNAL_ACCESS_BIT		28
#define MSS_EXTERNAL_ADDR_MASK		0xfffffff
#define MSS_INTERNAL_ACCESS_BIT		28

#define MSS_ADDR_REMAP			0x6f0600
#define MSS_REMAP_WIN1_ALR(ap)		(MVEBU_REGS_BASE_AP(ap) + MSS_ADDR_REMAP + 0x10)
#define MSS_WIN_ALR_VAL			0xf401
#define MSS_REMAP_WIN1_AHR(ap)		(MVEBU_REGS_BASE_AP(ap) + MSS_ADDR_REMAP + 0x14)
#define MSS_WIN_AHR_VAL			0xf7f0
#define MSS_REMAP_WIN1_CR(ap)		(MVEBU_REGS_BASE_AP(ap) + MSS_ADDR_REMAP + 0x18)
#define MSS_WIN1_CR_ADDR_MASK		0x3f000000
#define MSS_WIN1_CR_REMAP_ADDR_OFFSET	20

static void bl2_plat_mss_remap(void)
{
	int i;
	/* This init the MSS remap to enable access for CP0 via CM3
	 * AP0 -> 0x40000000 MSS AIBR will change the address to 0xF000_0000,
	 * then MSS remap will remap the address to 0xEC00_0000
	 * CP0 -> 0x44000000 MSS AIBR will change the address to 0xF400_0000,
	 * then MSS remap will remap the address to 0x81_0000_0000
	 */
	/* MSS remap for CP1 */
	for (i = 0; i < ap810_get_ap_count(); i++) {
		mmio_write_32(MSS_REMAP_WIN1_ALR(i), MSS_WIN_ALR_VAL);
		mmio_write_32(MSS_REMAP_WIN1_AHR(i), MSS_WIN_AHR_VAL);
		mmio_write_32(MSS_REMAP_WIN1_CR(i), MSS_WIN1_CR_ADDR_MASK |
						    (MVEBU_CP_REGS_BASE(i, 0) >>
						    MSS_WIN1_CR_REMAP_ADDR_OFFSET));
	}
}

/*******************************************************************************
 * Transfer SCP_BL2 from Trusted RAM using the SCP Download protocol.
 * Return 0 on success, -1 otherwise.
 ******************************************************************************/
int bl2_plat_handle_scp_bl2(image_info_t *scp_bl2_image_info)
{
	int ret;

	INFO("BL2: Initiating SCP_BL2 transfer to SCP\n");
	printf("BL2: Initiating SCP_BL2 transfer to SCP\n");

	/* initialize time (for delay functionality) */
	plat_delay_timer_init();

	/* Remap MSS */
	bl2_plat_mss_remap();

	ret = scp_bootloader_transfer((void *)scp_bl2_image_info->image_base,
		scp_bl2_image_info->image_size);

	if (ret == 0)
		INFO("BL2: SCP_BL2 transferred to SCP\n");
	else
		ERROR("BL2: SCP_BL2 transfer failure\n");

	return ret;
}

uintptr_t bl2_plat_get_cp_mss_regs(int ap_idx, int cp_idx)
{
	return MVEBU_CP_REGS_BASE(ap_idx, cp_idx) + MSS_CP_REG_BASE;
}

uintptr_t bl2_plat_get_ap_mss_regs(int ap_idx)
{
	return MVEBU_REGS_BASE_AP(ap_idx) + MSS_AP_REG_BASE;
}

uint32_t bl2_plat_get_cp_count(int ap_idx)
{
	return ap810_get_cp_per_ap_cnt(ap_idx);
}

uint32_t bl2_plat_get_ap_count(void)
{
	return ap810_get_ap_count();
}

void bl2_plat_configure_mss_windows(uintptr_t mss_regs)
{
	/* Configure the AEBR to enable access 256MB for AP config space */
	mmio_write_32(MSS_AEBR(mss_regs), ((0xe0000000 >> MSS_EXTERNAL_ACCESS_BIT) & MSS_AEBR_MASK));
	mmio_write_32(MSS_AIBR(mss_regs), ((0xf0000000 >> MSS_INTERNAL_ACCESS_BIT) & MSS_AIBR_MASK));
}

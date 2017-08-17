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

#include <bl_common.h>
#include <debug.h>
#include <plat_private.h> /* timer functionality */
#include "mss_scp_bootloader.h"
#include <plat_config.h>
#include <platform_def.h>
#include <ccu.h>
#include <mmio.h>

/* IO windows configuration */
#define IOW_GCR_OFFSET		(0x70)

struct ccu_win mem_map[] = {
	{0x0, MVEBU_CP_REGS_BASE(0), 0x0, 0x2000000, IO_0_TID},
	{0x0, MVEBU_CP_REGS_BASE(1), 0x0, 0x2000000, IO_0_TID},
};

/* Since the scp_bl2 image can contain firmware for cp1 and cp0 coprocessors,
 * the access to cp0 and cp1 need to be provided. More precisely it is
 * required to:
 *  - get the information about device id which is stored in CP0 registers
 *    (to distinguish between cases where we have cp0 and cp1 or standalone cp0)
 *  - get the access to cp which is needed for loading fw for cp0/cp1
 *    coprocessors
 * This function configures ccu windows accordingly.
 *
 * Note: there is no need to restore previous ccu configuration, since in next
 * phase (BL31) the init_ccu will be called (via apn806_init/
 * bl31_plat_arch_setu) and therefore the ccu configuration will be overwritten.
 */
static int bl2_plat_mmap_init(void)
{
	size_t win_nr, win_id;
	uintptr_t iow_base = MVEBU_IO_WIN_BASE;

	win_nr =  sizeof(mem_map) / sizeof(struct ccu_win);

	if (win_nr > marvell_get_ccu_max_win()) {
		ERROR("BL2: %s: trying to open too many windows\n", __func__);
		return -1;
	}

	for (win_id = 0; win_id < win_nr; win_id++) {
		/* Enable required CCU windows */
		ccu_win_check(&mem_map[win_id], win_id);
		ccu_enable_win(&mem_map[win_id], win_id);
	}

	/* Set the default target id to PIDI */
	mmio_write_32(iow_base + IOW_GCR_OFFSET, PIDI_TID);

	return 0;
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

	ret = bl2_plat_mmap_init();
	if (ret != 0)
		return ret;

	ret = scp_bootloader_transfer((void *)scp_bl2_image_info->image_base,
		scp_bl2_image_info->image_size);

	if (ret == 0)
		INFO("BL2: SCP_BL2 transferred to SCP\n");
	else
		ERROR("BL2: SCP_BL2 transfer failure\n");

	return ret;
}

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

#include <plat_config.h>
/*
 * If bootrom is currently at BLE there's no need to include the memory
 * maps structure at this point
 */
#include <plat_def.h>
#ifndef IMAGE_BLE

/*******************************************************************************
 * AMB Configuration
 ******************************************************************************/
struct addr_map_win amb_memory_map[] = {
	/* CP0 SPI1 CS0 Direct Mode access */
	{0xf900,	0x1000000,	AMB_SPI1_CS0_ID},
};

int marvell_get_amb_memory_map(struct addr_map_win **win, uint32_t *size, uintptr_t base)
{
	*win = amb_memory_map;
	if (*win == NULL)
		*size = 0;
	else
		*size = sizeof(amb_memory_map)/sizeof(amb_memory_map[0]);

	return 0;
}
#endif

/*******************************************************************************
 * IO_WIN Configuration
 ******************************************************************************/
struct addr_map_win io_win_memory_map[] = {
#ifndef IMAGE_BLE
	/* MCI 0 indirect window */
	{MVEBU_MCI_REG_BASE_REMAP(0),	0x100000,	MCI_0_TID},
	/* MCI 1 indirect window */
	{MVEBU_MCI_REG_BASE_REMAP(1),	0x100000,	MCI_1_TID},
#endif
};

uint32_t marvell_get_io_win_gcr_target(int ap_index)
{
	return PIDI_TID;
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

#ifndef IMAGE_BLE
/*******************************************************************************
 * IOB Configuration
 ******************************************************************************/
struct addr_map_win iob_memory_map[] = {
	/* PEX1_X1 window */
	{0x00000000f7000000,	0x1000000,	PEX1_TID},
	/* PEX2_X1 window */
	{0x00000000f8000000,	0x1000000,	PEX2_TID},
	/* PEX0_X4 window */
	{0x00000000f6000000,	0x1000000,	PEX0_TID},
	/* SPI1_CS0 (RUNIT) window */
	{0x00000000f9000000,	0x1000000,	RUNIT_TID},
};

int marvell_get_iob_memory_map(struct addr_map_win **win, uint32_t *size, uintptr_t base)
{
	*win = iob_memory_map;
	*size = sizeof(iob_memory_map)/sizeof(iob_memory_map[0]);

	return 0;
}
#endif

/*******************************************************************************
 * CCU Configuration
 ******************************************************************************/
struct addr_map_win ccu_memory_map[] = {	/* IO window */
#ifdef IMAGE_BLE
	{0x00000000f2000000,	0x4000000,	IO_0_TID}, /* IO window */
#else
	{0x00000000f2000000,	0xe000000,	IO_0_TID},
#endif
};

uint32_t marvell_get_ccu_gcr_target(int ap)
{
	return DRAM_0_TID;
}

int marvell_get_ccu_memory_map(int ap_index, struct addr_map_win **win, uint32_t *size)
{
	*win = ccu_memory_map;
	*size = sizeof(ccu_memory_map)/sizeof(ccu_memory_map[0]);

	return 0;
}

#ifdef IMAGE_BLE
/*******************************************************************************
 * SKIP IMAGE Configuration
 ******************************************************************************/

#if PLAT_RECOVERY_IMAGE_ENABLE
struct skip_image skip_im = {
	.detection_method = GPIO,
	.info.gpio.num = 33,
	.info.gpio.button_state = HIGH,
	.info.test.cp_ap = CP,
	.info.test.cp_index = 0,
};

void *plat_get_skip_image_data(void)
{
	/* Return the skip_image configurations */
	return &skip_im;
}
#endif
#endif

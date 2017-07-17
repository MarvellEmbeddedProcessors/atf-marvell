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
struct amb_win *amb_memory_map;

uintptr_t marvell_get_amb_reg_offs(int cp_index)
{
	return MVEBU_AMB_ADEC_BASE(cp_index);
}

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
 * RFU Configuration
 ******************************************************************************/

struct rfu_win rfu_memory_map[] = {
	/* CP1 (MCI0) internal regs */
	{0x0,	0xf4000000,			0x0,	0x2000000,  MCI_0_TID},
	/* MCI 0 indirect window */
	{0x0,	MVEBU_MCI_REG_BASE_REMAP(0),	0x0,	0x100000,   MCI_0_TID},
	/* MCI 1 indirect window */
	{0x0,	MVEBU_MCI_REG_BASE_REMAP(1),	0x0,	0x100000,   MCI_1_TID},
};

uintptr_t marvell_get_rfu_reg_offs(void)
{
	return MVEBU_RFU_BASE;
}

int marvell_get_rfu_memory_map(struct rfu_win **win, uint32_t *size)
{
	*win = rfu_memory_map;
	if (*win == NULL)
		*size = 0;
	else
		*size = sizeof(rfu_memory_map)/sizeof(struct rfu_win);

	return 0;
}

/*******************************************************************************
 * IOB Configuration
 ******************************************************************************/
#define MARVELL_IOB_MAX_WIN		16

struct iob_win iob_memory_map_cp0[] = {
	/* CP1 */
	/* PEX0_X4 window */
	{0x80,	0x00000000,	0x80,	0x0000000,	PEX0_TID}
};

uintptr_t marvell_get_iob_reg_offs(int cp_index)
{
	return MVEBU_IOB_BASE(cp_index);
}

int marvell_get_iob_max_win(void)
{
	return MARVELL_IOB_MAX_WIN;
}

int marvell_get_iob_memory_map(struct iob_win **win,
			       uint32_t *size, int cp_index)
{
	switch (cp_index) {
	case 0:
		*win = iob_memory_map_cp0;
		*size = sizeof(iob_memory_map_cp0)/sizeof(struct iob_win);
		return 0;
	case 1:
		*size = 0;
		*win = 0;
		return 0;
	default:
		*size = 0;
		*win = 0;
		return 1;
	}
}

/*******************************************************************************
 * CCU Configuration
 ******************************************************************************/
#define MARVELL_CCU_MAX_WIN	8

struct ccu_win ccu_memory_map[] = {
	{0x0,	0xf2000000,	0x0,	0xe000000,  IO_0_TID}, /* IO window */
	{0x80,	0x00000000,	0x80,	0x0000000,   IO_0_TID}, /* IO window */
};

uintptr_t marvell_get_ccu_reg_offs(void)
{
	return MVEBU_CCU_BASE;
}

int marvell_get_ccu_max_win(void)
{
	return MARVELL_CCU_MAX_WIN;
}

int marvell_get_ccu_memory_map(struct ccu_win **win, uint32_t *size)
{
	*win = ccu_memory_map;
	*size = sizeof(ccu_memory_map)/sizeof(struct ccu_win);

	return 0;
}
/* In reference to #ifndef IMAGE_BLE, this part is used for BLE only. */
#else
/*******************************************************************************
 * PCIe Configuration
 ******************************************************************************/
struct pci_hw_cfg ocp_pci_hw_cfg = {
	.delay_cfg	= 1,
	.master_en	= 1,
	.lane_width	= 4,
	.lane_ids	= {0, 1, 2, 3},
	.clk_src	= 0,
	.clk_out	= 0,	/* clk is not output */
	.is_end_point	= 1,
	.mac_base	= MVEBU_PCIE_X4_MAC_BASE(0),
	.comphy_base	= MVEBU_COMPHY_BASE(0),
	.hpipe_base	= MVEBU_HPIPE_BASE(0),
	.dfx_base	= MVEBU_CP_DFX_BASE(0),
};

struct pci_hw_cfg *plat_get_pcie_hw_data(void)
{
	return &ocp_pci_hw_cfg;
}

/*******************************************************************************
 * SKIP IMAGE Configuration
 ******************************************************************************/

void *plat_get_skip_image_data(void)
{
	/* Return the skip_image configurations */
	return NULL;
}
#endif

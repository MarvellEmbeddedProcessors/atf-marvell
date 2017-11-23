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

#include <delay_timer.h>
#include <mmio.h>
#include <plat_config.h>
/*
 * If bootrom is currently at BLE there's no need to include the memory
 * maps structure at this point
 */
#include <plat_def.h>
#ifndef IMAGE_BLE


/*******************************************************************************
 * GPIO Configuration
 ******************************************************************************/
#define MPP_CONTROL_REGISTER		0xf2440018
#define MPP_CONTROL_MPP_SEL_52_MASK	0xf0000
#define GPIO_DATA_OUT1_REGISTER		0xf2440140
#define GPIO_DATA_OUT_EN_CTRL1_REGISTER 0xf2440144
#define GPIO52_MASK			0x100000

/* Reset PCIe via GPIO number 52 */
int marvell_gpio_config(void)
{
	uint32_t reg;

	reg = mmio_read_32(MPP_CONTROL_REGISTER);
	reg |= MPP_CONTROL_MPP_SEL_52_MASK;
	mmio_write_32(MPP_CONTROL_REGISTER, reg);

	reg = mmio_read_32(GPIO_DATA_OUT1_REGISTER);
	reg |= GPIO52_MASK;
	mmio_write_32(GPIO_DATA_OUT1_REGISTER, reg);

	reg = mmio_read_32(GPIO_DATA_OUT_EN_CTRL1_REGISTER);
	reg &= ~GPIO52_MASK;
	mmio_write_32(GPIO_DATA_OUT_EN_CTRL1_REGISTER, reg);
	udelay(100);

	return 0;
}

/*******************************************************************************
 * AMB Configuration
 ******************************************************************************/
struct amb_win amb_memory_map[] = {
	/* CP1 SPI1 CS0 Direct Mode access */
	{0xf900,	0x1000000,	AMB_SPI1_CS0_ID},
};

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
 * IO WIN Configuration
 ******************************************************************************/

struct io_win io_win_memory_map[] = {
	/* CP1 (MCI0) internal regs */
	{0x0,	0xf4000000,			0x0,	0x2000000,  MCI_0_TID},
	/* PCIe0 and SPI1_CS0 (RUNIT) on CP1*/
	{0x0,	0xf9000000,			0x0,	0x2000000,  MCI_0_TID},
	/* PCIe1 on CP1*/
	{0x0,	0xfb000000,			0x0,	0x1000000,  MCI_0_TID},
	/* PCIe2 on CP1*/
	{0x0,	0xfc000000,			0x0,	0x1000000,  MCI_0_TID},
	/* MCI 0 indirect window */
	{0x0,	MVEBU_MCI_REG_BASE_REMAP(0),	0x0,	0x100000,   MCI_0_TID},
	/* MCI 1 indirect window */
	{0x0,	MVEBU_MCI_REG_BASE_REMAP(1),	0x0,	0x100000,   MCI_1_TID},
};

uintptr_t marvell_get_io_win_reg_offs(int ap_index)
{
	return MVEBU_IO_WIN_BASE;
}

uint32_t marvell_get_io_win_gcr_target(int ap_index)
{
	return PIDI_TID;
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
 * IOB Configuration
 ******************************************************************************/
#define MARVELL_IOB_MAX_WIN		16

struct iob_win iob_memory_map_cp0[] = {
	/* CP0 */
	/* PEX1_X1 window */
	{0x0,	0xf7000000,	0x0,	0x1000000,	PEX1_TID},
	/* PEX2_X1 window */
	{0x0,	0xf8000000,	0x0,	0x1000000,	PEX2_TID},
	/* PEX0_X4 window */
	{0x0,	0xf6000000,	0x0,	0x1000000,	PEX0_TID},
	{0x0,	0xc0000000,	0x0,   0x30000000,	PEX0_TID},
	{0x8,	0x00000000,	0x1,   0x00000000,	PEX0_TID},
};

struct iob_win iob_memory_map_cp1[] = {
	/* CP1 */
	/* SPI1_CS0 (RUNIT) window */
	{0x0,	0xf9000000,	0x0,	0x1000000,	RUNIT_TID},
	/* PEX1_X1 window */
	{0x0,	0xfb000000,	0x0,	0x1000000,	PEX1_TID},
	/* PEX2_X1 window */
	{0x0,	0xfc000000,	0x0,	0x1000000,	PEX2_TID},
	/* PEX0_X4 window */
	{0x0,	0xfa000000,	0x0,	0x1000000,	PEX0_TID}
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
		*win = iob_memory_map_cp1;
		*size = sizeof(iob_memory_map_cp1)/sizeof(struct iob_win);
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
	{0x0,	0xc0000000,	0x0,   0x30000000,  IO_0_TID}, /* IO window */
	{0x8,	0x00000000,	0x1,   0x00000000,  IO_0_TID}, /* IO window */
};

uintptr_t marvell_get_ccu_reg_offs(int ap_index)
{
	return MVEBU_CCU_BASE;
}

uint32_t marvell_get_ccu_gcr_target(int ap)
{
	return DRAM_0_TID;
}

int marvell_get_ccu_max_win(void)
{
	return MARVELL_CCU_MAX_WIN;
}

int marvell_get_ccu_memory_map(int ap_index, struct ccu_win **win, uint32_t *size)
{
	*win = ccu_memory_map;
	*size = sizeof(ccu_memory_map)/sizeof(struct ccu_win);

	return 0;
}

/* In reference to #ifndef IMAGE_BLE, this part is used for BLE only. */
#else
/*******************************************************************************
 * SKIP IMAGE Configuration
 ******************************************************************************/

void *plat_get_skip_image_data(void)
{
	/* No recovery button on A8k-MCBIN board */
	return NULL;
}
#endif

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

#include <plat_def.h>
#include <debug.h>
#include <mmio.h>
#include <mvebu.h>
#include <iob.h>
#include <plat_config.h>

#if LOG_LEVEL >= LOG_LEVEL_INFO
#define DEBUG_ADDR_MAP
#endif

/* common defines */
#define WIN_ENABLE_BIT			(0x1)
/* Physical address of the base of the window = {AddrLow[19:0],20`h0} */
#define ADDRESS_SHIFT			(20 - 4)
#define ADDRESS_MASK			(0xFFFFFFF0)
#define IOB_WIN_ALIGNMENT		(0x100000)

/* IOB registers */
#define IOB_MAX_WIN_NUM			(24)

#define IOB_WIN_CR_OFFSET(win)		(iob_info->iob_base + 0x0 + (0x20 * win))
#define IOB_TARGET_ID_OFFSET		(8)
#define IOB_TARGET_ID_MASK		(0xF)

#define IOB_WIN_SCR_OFFSET(win)		(iob_info->iob_base + 0x4 + (0x20 * win))
#define IOB_WIN_ENA_CTRL_WRITE_SECURE	(0x1)
#define IOB_WIN_ENA_CTRL_READ_SECURE	(0x2)
#define IOB_WIN_ENA_WRITE_SECURE	(0x4)
#define IOB_WIN_ENA_READ_SECURE		(0x8)

#define IOB_WIN_ALR_OFFSET(win)		(iob_info->iob_base + 0x8 + (0x20 * win))
#define IOB_WIN_AHR_OFFSET(win)		(iob_info->iob_base + 0xC + (0x20 * win))

struct iob_configuration {
	uintptr_t iob_base;
	uint32_t max_win;
};

struct iob_configuration iob_config;
struct iob_configuration *iob_info = &iob_config;

static void iob_win_check(struct iob_win *win, uint32_t win_num)
{
	uint64_t base_addr, win_size;

	/* check if address is aligned to the size */
	base_addr = ((uint64_t)win->base_addr_high << 32) + win->base_addr_low;
	if (IS_NOT_ALIGN(base_addr, IOB_WIN_ALIGNMENT)) {
		base_addr = ALIGN_UP(base_addr, IOB_WIN_ALIGNMENT);
		ERROR("Window %d: base address unaligned to 0x%x\n", win_num, IOB_WIN_ALIGNMENT);
		printf("Align up the base address to 0x%lx\n", base_addr);
		win->base_addr_high = (uint32_t)(base_addr >> 32);
		win->base_addr_low = (uint32_t)(base_addr);
	}

	/* size parameter validity check */
	win_size = ((uint64_t)win->win_size_high << 32) + win->win_size_low;
	if (IS_NOT_ALIGN(win_size, IOB_WIN_ALIGNMENT)) {
		win_size = ALIGN_UP(win_size, IOB_WIN_ALIGNMENT);
		ERROR("Window %d: window size unaligned to 0x%x\n", win_num, IOB_WIN_ALIGNMENT);
		printf("Aligning size to 0x%lx\n", win_size);
		win->win_size_high = (uint32_t)(win_size >> 32);
		win->win_size_low = (uint32_t)(win_size);
	}
}

static void iob_enable_win(struct iob_win *win, uint32_t win_id)
{
	uint32_t iob_win_reg;
	uint32_t alr, ahr;
	uint64_t start_addr, end_addr;

	iob_win_reg = WIN_ENABLE_BIT;
	iob_win_reg |= (win->target_id & IOB_TARGET_ID_MASK) << IOB_TARGET_ID_OFFSET;
	mmio_write_32(IOB_WIN_CR_OFFSET(win_id), iob_win_reg);

	start_addr = ((uint64_t)win->base_addr_high << 32) + win->base_addr_low;
	end_addr = (start_addr + (((uint64_t)win->win_size_high << 32) + win->win_size_low) - 1);
	alr = (uint32_t)((start_addr >> ADDRESS_SHIFT) & ADDRESS_MASK);
	ahr = (uint32_t)((end_addr >> ADDRESS_SHIFT) & ADDRESS_MASK);

	mmio_write_32(IOB_WIN_ALR_OFFSET(win_id), alr);
	mmio_write_32(IOB_WIN_AHR_OFFSET(win_id), ahr);
}

#ifdef DEBUG_ADDR_MAP
static void dump_iob(void)
{
	uint32_t win_id, win_cr, alr, ahr;
	uint8_t target_id;
	uint64_t start, end;
	char *iob_target_name[IOB_MAX_TID] = {"CONFIG", "MCI0 ", "PEX1 ", "PEX2 ",
					      "PEX0 ", "NAND ", "RUNIT", "MCI1 "};

	/* Dump all IOB windows */
	printf("bank  id target  start              end\n");
	printf("----------------------------------------------------\n");
	for (win_id = 0; win_id < iob_info->max_win; win_id++) {
		win_cr = mmio_read_32(IOB_WIN_CR_OFFSET(win_id));
		if (win_cr & WIN_ENABLE_BIT) {
			target_id = (win_cr >> IOB_TARGET_ID_OFFSET) & IOB_TARGET_ID_MASK;
			alr = mmio_read_32(IOB_WIN_ALR_OFFSET(win_id));
			start = ((uint64_t)alr << ADDRESS_SHIFT);
			if (win_id != 0) {
				ahr = mmio_read_32(IOB_WIN_AHR_OFFSET(win_id));
				end = (((uint64_t)ahr + 0x10) << ADDRESS_SHIFT);
			} else {
				/* Window #0 size is hardcoded to 16MB, as it's
				** reserved for CP configuration space. */
				end = start + (16 << 20);
			}
			printf("iob   %02d %s   0x%016lx 0x%016lx\n"
					, win_id, iob_target_name[target_id], start, end);
		}
	}

	return;
}
#endif

int init_iob(int cp_index)
{
	struct iob_win *win;
	uint32_t win_id, win_reg;
	uint32_t win_count;

	INFO("Initializing IOB Address decoding\n");

	/* Get the base address of the address decoding MBUS */
	iob_info->iob_base = marvell_get_iob_reg_offs(cp_index);

	/* Get the maximum number of iob windows supported */
	iob_info->max_win = marvell_get_iob_max_win();
	if (iob_info->max_win == 0) {
		iob_info->max_win = IOB_MAX_WIN_NUM;
		ERROR("IOB win num cannot be 0. Setting to default num (%d)\n", IOB_MAX_WIN_NUM);
	}

	/* Get the array of the windows and fill the map data */
	marvell_get_iob_memory_map(&win, &win_count, cp_index);
	if (win_count <= 0) {
		INFO("no windows configurations found\n");
		return 0;
	}

	/* disable all IOB windows, start from win_id = 1 because can't disable internal register window */
	for (win_id = 1; win_id < iob_info->max_win; win_id++) {
		win_reg = mmio_read_32(IOB_WIN_CR_OFFSET(win_id));
		win_reg &= ~WIN_ENABLE_BIT;
		mmio_write_32(IOB_WIN_CR_OFFSET(win_id), win_reg);

		win_reg = ~IOB_WIN_ENA_CTRL_WRITE_SECURE;
		win_reg &= ~IOB_WIN_ENA_CTRL_READ_SECURE;
		win_reg &= ~IOB_WIN_ENA_WRITE_SECURE;
		win_reg &= ~IOB_WIN_ENA_READ_SECURE;
		mmio_write_32(IOB_WIN_SCR_OFFSET(win_id), win_reg);
	}

	for (win_id = 1; win_id < win_count + 1; win_id++, win++) {
		iob_win_check(win, win_id);
		iob_enable_win(win, win_id);
	}

#ifdef DEBUG_ADDR_MAP
	dump_iob();
#endif

	INFO("Done IOB Address decoding Initializing\n");

	return 0;
}

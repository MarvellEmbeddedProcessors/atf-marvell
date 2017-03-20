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
#include <rfu.h>
#include <plat_config.h>

#if LOG_LEVEL >= LOG_LEVEL_INFO
#define DEBUG_ADDR_MAP
#endif

/* common defines */
#define WIN_ENABLE_BIT			(0x1)
/* Physical address of the base of the window = {Addr[19:0],20`h0} */
#define ADDRESS_SHIFT			(20 - 4)
#define ADDRESS_MASK			(0xFFFFFFF0)
#define RFU_WIN_ALIGNMENT_1M		(0x100000)
#define RFU_WIN_ALIGNMENT_64K		(0x10000)

/* AP registers */
#define RFU_WIN_ALR_OFFSET(win)		(rfu_base + 0x0 + (0x10 * win))
#define RFU_WIN_AHR_OFFSET(win)		(rfu_base + 0x8 + (0x10 * win))
#define RFU_WIN_CR_OFFSET(win)		(rfu_base + 0xC + (0x10 * win))
#define RFU_GCR_OFFSET			(rfu_base + 0x70)


uintptr_t rfu_base;


static void rfu_win_check(struct rfu_win *win, uint32_t win_num)
{
	uint64_t base_addr, win_size;
	uint32_t alignment_value = RFU_WIN_ALIGNMENT_1M;

	/* for RFU The base is always 1M aligned */
	/* check if address is aligned to 1M */
	base_addr = ((uint64_t)win->base_addr_high << 32) + win->base_addr_low;
	if (IS_NOT_ALIGN(base_addr, RFU_WIN_ALIGNMENT_1M)) {
		base_addr = ALIGN_UP(base_addr, RFU_WIN_ALIGNMENT_1M);
		ERROR("Window %d: base address unaligned to 0x%x\n", win_num, RFU_WIN_ALIGNMENT_1M);
		printf("Align up the base address to 0x%lx\n", base_addr);
		win->base_addr_high = (uint32_t)(base_addr >> 32);
		win->base_addr_low = (uint32_t)(base_addr);
	}

	/* size parameter validity check */
	win_size = ((uint64_t)win->win_size_high << 32) + win->win_size_low;
	if (IS_NOT_ALIGN(win_size, alignment_value)) {
		win_size = ALIGN_UP(win_size, alignment_value);
		ERROR("Window %d: window size unaligned to 0x%x\n", win_num, alignment_value);
		printf("Aligning size to 0x%lx\n", win_size);
		win->win_size_high = (uint32_t)(win_size >> 32);
		win->win_size_low = (uint32_t)(win_size);
	}
}

static void rfu_enable_win(struct rfu_win *win, uint32_t win_num)
{
	uint32_t alr, ahr;
	uint64_t start_addr, end_addr;

	if (win->target_id < 0 || win->target_id >= RFU_MAX_WIN_ID) {
		ERROR("target ID = %d, is invalid\n", win->target_id);
		return;
	}

	/* calculate 64bit start-address and end-address */
	start_addr = ((uint64_t)win->base_addr_high << 32) + win->base_addr_low;
	end_addr = (start_addr + (((uint64_t)win->win_size_high << 32) + win->win_size_low) - 1);

	alr = (uint32_t)((start_addr >> ADDRESS_SHIFT) & ADDRESS_MASK);
	alr |= WIN_ENABLE_BIT;
	ahr = (uint32_t)((end_addr >> ADDRESS_SHIFT) & ADDRESS_MASK);

	/* write start address and end address for rfu window */
	mmio_write_32(RFU_WIN_ALR_OFFSET(win_num), alr);
	mmio_write_32(RFU_WIN_AHR_OFFSET(win_num), ahr);

	/* write window target */
	mmio_write_32(RFU_WIN_CR_OFFSET(win_num), win->target_id);
}

#ifdef DEBUG_ADDR_MAP
static void dump_rfu(void)
{
	uint32_t trgt_id, win_id;
	uint32_t alr, ahr;
	uint64_t start, end;
	char *rfu_target_name[RFU_MAX_TID] = {"MCI-0    ", "MCI-1    ", "MCI-2    ", "PIDI     ",
						"SPI      ", "STM      ", "BootRoom "};

	/* Dump all RFU windows */
	printf("bank  target     start              end\n");
	printf("----------------------------------------------------\n");
	for (win_id = 0; win_id < RFU_MAX_WIN_ID; win_id++) {
		alr = mmio_read_32(RFU_WIN_ALR_OFFSET(win_id));
		if (alr & WIN_ENABLE_BIT) {
			alr &= ~WIN_ENABLE_BIT;
			/* in case this is BOOTROM window */
			if (win_id == 0) {
				ahr = alr;
				trgt_id = BOOTROM_TID;
			} else {
				ahr = mmio_read_32(RFU_WIN_AHR_OFFSET(win_id));
				trgt_id = mmio_read_32(RFU_WIN_CR_OFFSET(win_id));
			}
			start = ((uint64_t)alr << ADDRESS_SHIFT);
			end = (((uint64_t)ahr + 0x10) << ADDRESS_SHIFT);
			printf("rfu   %s  0x%016lx 0x%016lx\n", rfu_target_name[trgt_id], start, end);
		}
	}
	printf("rfu   PIDI-port  - all other IO transactions\n");

	return;
}
#endif

int init_rfu(void)
{
	struct rfu_win *win;
	uint32_t win_id, win_reg;
	uint32_t win_count;

	INFO("Initializing RFU Address decoding\n");

	/* Get the base address of the address decoding MBUS */
	rfu_base = marvell_get_rfu_reg_offs();

	/* Get the array of the windows and its size */
	marvell_get_rfu_memory_map(&win, &win_count);
	if (win_count <= 0) {
		INFO("no windows configurations found\n");
	}
	if (win_count > RFU_MAX_WIN_ID) {
		INFO("number of windows is bigger than %d\n", RFU_MAX_WIN_ID);
		return 0;
	}

	/* set the default target id to PIDI */
	win_reg = PIDI_TID;
	mmio_write_32(RFU_GCR_OFFSET, win_reg);

	/* disable all RFU windows */
	for (win_id = 0; win_id < RFU_MAX_WIN_ID; win_id++) {
		win_reg = mmio_read_32(RFU_WIN_ALR_OFFSET(win_id));
		win_reg &= ~WIN_ENABLE_BIT;
		mmio_write_32(RFU_WIN_ALR_OFFSET(win_id), win_reg);
	}

	/* enable relevant windows, starting from win_id=1 because index 0 dedicated for BootRom */
	for (win_id = 1; win_id <= win_count; win_id++, win++) {
		rfu_win_check(win, win_id);
		rfu_enable_win(win, win_id);
	}

#ifdef DEBUG_ADDR_MAP
	dump_rfu();
#endif

	INFO("Done RFU Address decoding Initializing\n");

	return 0;
}

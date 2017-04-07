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

#include <mmio.h>
#include <string.h>
#include <plat_marvell.h>
#include <io_addr_dec.h>
#include <dram_win.h>

/*
 * dram_win_map_build
 *
 * This function builds cpu dram windows mapping
 * which includes base address and window size by
 * reading cpu dram decode windows registers.
 *
 * @input: N/A
 *
 * @output:
 *     - win_map: cpu dram windows mapping
 *
 * @return:  N/A
 */
void dram_win_map_build(struct dram_win_map *win_map)
{
	int32_t win_id;
	struct dram_win *win;
	uint32_t base_reg, ctrl_reg, size_reg, enabled, target;

	memset(win_map, 0, sizeof(struct dram_win_map));
	for (win_id = 0; win_id < DRAM_WIN_MAP_NUM_MAX; win_id++) {
		ctrl_reg = mmio_read_32(CPU_DEC_WIN_CTRL_REG(win_id));
		target = (ctrl_reg & CPU_DEC_CR_WIN_TARGET_MASK) >> CPU_DEC_CR_WIN_TARGET_OFFS;
		enabled = ctrl_reg & CPU_DEC_CR_WIN_ENABLE;
		/* Ignore invalid and non-dram windows*/
		if ((enabled == 0) || (target != DRAM_CPU_DEC_TARGET_NUM))
			continue;

		win = win_map->dram_windows + win_map->dram_win_num;
		base_reg = mmio_read_32(CPU_DEC_WIN_BASE_REG(win_id));
		size_reg = mmio_read_32(CPU_DEC_WIN_SIZE_REG(win_id));
		/* Base reg [15:0] corresponds to transaction address [39:16] */
		win->base_addr = (base_reg & CPU_DEC_BR_BASE_MASK) >> CPU_DEC_BR_BASE_OFFS;
		win->base_addr *= CPU_DEC_CR_WIN_SIZE_ALIGNMENT;
		/*
		 * Size reg [15:0] is programmed from LSB to MSB as a sequence of 1s followed by a sequence of 0s,
		 * and the number of 1s specifies the size of the window in 64 KB granularity,
		 * for example, a value of 00FFh specifies 256 x 64 KB = 16 MB
		 */
		win->win_size = (size_reg & CPU_DEC_CR_WIN_SIZE_MASK) >> CPU_DEC_CR_WIN_SIZE_OFFS;
		win->win_size = (win->win_size + 1) * CPU_DEC_CR_WIN_SIZE_ALIGNMENT;

		win_map->dram_win_num++;
	}

	return;
}


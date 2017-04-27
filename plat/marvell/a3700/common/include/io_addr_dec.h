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

#ifndef _IO_ADDR_DEC_H_
#define _IO_ADDR_DEC_H_

/* There are 5 configurable cpu decoder windows. */
#define DRAM_WIN_MAP_NUM_MAX	5
/* Target number for dram in cpu decoder windows. */
#define DRAM_CPU_DEC_TARGET_NUM	0

/*
* Not all configurable decode windows could be used for dram, some units have
* to reserve one decode window for other unit they have to communicate with;
* for example, DMA engineer has 3 configurable windows, but only two could be
* for dram while the last one has to be for pcie, so for DMA, its max_dram_win
* is 2.
*/
struct dec_win_config {
	uint32_t dec_reg_base; /* IO address decoder register base address */
	uint32_t win_attr;     /* IO address decoder windows attributes */
	uint32_t max_dram_win; /* How many configurable dram decoder windows that this unit has; */
	uint32_t max_remap;    /* The decoder windows number including remapping that this unit has */
	uint32_t win_offset;   /* The offset between continuous decode windows within the same unit, typically 0x10 */
};

struct dram_win {
	uintptr_t base_addr;
	uintptr_t win_size;
};

struct  dram_win_map {
	int dram_win_num;
	struct dram_win dram_windows[DRAM_WIN_MAP_NUM_MAX];
};

/*
 * init_io_addr_dec
 *
 * This function initializes io address decoder windows by
 * cpu dram window mapping information
 *
 * @input: N/A
 *     - dram_wins_map: cpu dram windows mapping
 *     - io_dec_config: io address decoder windows configuration
 *     - io_unit_num: io address decoder unit number
 * @output: N/A
 *
 * @return:  0 on success and others on failure
 */
int init_io_addr_dec(struct dram_win_map *dram_wins_map, struct dec_win_config *io_dec_config, uint32_t io_unit_num);

#endif /* _IO_ADDR_DEC_H_ */


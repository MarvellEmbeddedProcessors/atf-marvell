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

#ifndef _CCU_H_
#define _CCU_H_

#include <addr_map.h>

/* CCU registers definitions */
#define CCU_WIN_CR_OFFSET(ap, win)		(MVEBU_CCU_BASE(ap) + 0x0 + (0x10 * win))
#define CCU_TARGET_ID_OFFSET			(8)
#define CCU_TARGET_ID_MASK			(0x7F)

#define CCU_WIN_SCR_OFFSET(ap, win)		(MVEBU_CCU_BASE(ap) + 0x4 + (0x10 * win))
#define CCU_WIN_ENA_WRITE_SECURE		(0x1)
#define CCU_WIN_ENA_READ_SECURE			(0x2)

#define CCU_WIN_ALR_OFFSET(ap, win)		(MVEBU_CCU_BASE(ap) + 0x8 + (0x10 * win))
#define CCU_WIN_AHR_OFFSET(ap, win)		(MVEBU_CCU_BASE(ap) + 0xC + (0x10 * win))

#define CCU_WIN_GCR_OFFSET(ap)			(MVEBU_CCU_BASE(ap) + 0xD0)
#define CCU_GCR_TARGET_OFFSET			(8)
#define CCU_GCR_TARGET_MASK			(0xF)

#define CCU_MC_RCR_OFFSET(ap, iface)		(MVEBU_REGS_BASE_AP(ap) + \
						0x1700 + (0x1400 * (iface)))
#define CCU_MC_RSBR_OFFSET(ap, iface)		(CCU_MC_RCR_OFFSET(ap, iface) + 0x4)
#define CCU_MC_RTBR_OFFSET(ap, iface)		(CCU_MC_RCR_OFFSET(ap, iface) + 0x8)

int init_ccu(int);
void ccu_win_check(struct addr_map_win *win);
void ccu_enable_win(int ap_index, struct addr_map_win *win, uint32_t win_id);
void ccu_temp_win_insert(int ap_index, struct addr_map_win *win, int size);
void ccu_temp_win_remove(int ap_index, struct addr_map_win *win, int size);
void ccu_dram_win_config(int ap_index, struct addr_map_win *win);
void ccu_dram_target_set(int ap_index, uint32_t target);
void ccu_dram_mca_remap(int ap_index, int dram_tgt, uint64_t from, uint64_t to, uint64_t size);

#endif /* _CCU_H_ */

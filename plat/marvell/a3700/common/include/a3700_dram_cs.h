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

#ifndef __MVEBU_A3700_DRAM_CS_H__
#define __MVEBU_A3700_DRAM_CS_H__

/*
 * marvell_get_dram_cs_base_size
 *
 * This function is used to get DRAM CS's memory base address
 * and memory block length from DRAM CS memory map registers,
 * the block length unit is MB.
 *
 * @input:
 *     - cs_num: DRAM CS number
 *
 * @output:
 *     - base_low: DRAM CS's memory base address in low 4 bytes
 *     - base_high: DRAM CS's memory base address in high 4 bytes
 *     - size_mbytes: DRAM CS's memory length in unit of MB
 *
 * @return:  N/A
 */
int marvell_get_dram_cs_base_size(uint32_t cs_num,
				  uint32_t *base_low,
				  uint32_t *base_high,
				  uint32_t *size_mbytes);

#endif /* __MVEBU_A3700_DRAM_CS_H__ */

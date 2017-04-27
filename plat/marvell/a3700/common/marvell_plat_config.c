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
#include <types.h>
#include <plat_def.h>
#include <io_addr_dec.h>

struct dec_win_config io_dec_win_conf[] = {
	/* dec_reg_base  win_attr  max_dram_win  max_remap  win_offset */
	{0xc000,         0x3d,     2,       0,        0x08}, /* USB */
	{0xc100,         0x3d,     2,       0,        0x10}, /* USB3 */
	{0xc200,         0x3d,     2,       0,        0x10}, /* DMA */
	{0xc300,         0x3d,     2,       0,        0x10}, /* NETA0 */
	{0xc400,         0x3d,     2,       0,        0x10}, /* NETA1 */
	{0xc500,         0x3d,     2,       0,        0x10}, /* PCIe */
	{0xc800,         0x3d,     2,       0,        0x10}, /* SATA */
	{0xca00,         0x3d,     2,       0,        0x08}, /* SD */
	{0xcb00,         0x3d,     2,       0,        0x10}, /* eMMC */
};

int marvell_get_io_dec_win_conf(struct dec_win_config **win, uint32_t *size)
{
	*win = io_dec_win_conf;
	*size = sizeof(io_dec_win_conf)/sizeof(struct dec_win_config);

	return 0;
}


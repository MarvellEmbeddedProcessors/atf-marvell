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
#ifndef __APN806_SETUP_H__
#define __APN806_SETUP_H__

#include <mmio.h>

#define APN806_REV_ID_A0		0
#define APN806_REV_ID_A1		1

/* APN806 revision ID */
#define MVEBU_CSS_GWD_CTRL_IIDR2_REG		(MVEBU_REGS_BASE + 0x610FCC)
#define GWD_IIDR2_REV_ID_OFFSET			12
#define GWD_IIDR2_REV_ID_MASK			0xF

#define AP806_SAR0_REG_BASE			(MVEBU_REGS_BASE + 0x6F82D4)
#define AP806_SAR0_BOOT_SOURCE_OFFSET		8
#define AP806_SAR0_BOOT_SOURCE_MASK		0x7

enum ap806_sar_target_dev {
	SAR_PIDI_MCIX2		= 0x0,
	SAR_MCIX4		= 0x1,
	SAR_SPI			= 0x2,
	SAR_SD			= 0x3,
	SAR_PIDI_MCIX2_BD	= 0x4, /* BootRom disabled */
	SAR_MCIX4_DB		= 0x5, /* BootRom disabled */
	SAR_SPI_DB		= 0x6, /* BootRom disabled */
	SAR_EMMC		= 0x7,
};

void apn806_init(void);
uint32_t apn806_sar_get_bootsrc(void);

static inline int apn806_rev_id_get(void)
{
	/* Returns:
	 * - 0 (APN806_REV_ID_A0) for A0
	 * - 1 (APN806_REV_ID_A1) for A1
	 */
	return (mmio_read_32(MVEBU_CSS_GWD_CTRL_IIDR2_REG) >>
		GWD_IIDR2_REV_ID_OFFSET) &
		GWD_IIDR2_REV_ID_MASK;
}

#endif /* __APN806_SETUP_H__ */

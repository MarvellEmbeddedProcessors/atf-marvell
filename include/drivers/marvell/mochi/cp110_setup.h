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
#ifndef __PLAT_CP110_H__
#define __PLAT_CP110_H__

#include <apn806_setup.h>

#define MVEBU_DEVICE_ID_REG		(MVEBU_CP_DFX_BASE(0) + 0x40)
#define MVEBU_DEVICE_ID_OFFSET		(0)
#define MVEBU_DEVICE_ID_MASK		(0xffff << MVEBU_DEVICE_ID_OFFSET)
#define MVEBU_DEVICE_REV_OFFSET		(16)
#define MVEBU_DEVICE_REV_MASK		(0xf << MVEBU_DEVICE_REV_OFFSET)
#define MVEBU_70X0_DEV_ID		(0x7040)
#define MVEBU_80X0_DEV_ID		(0x8040)

/*******************************************************************************
 * RTC Configuration
 ******************************************************************************/
#define MVEBU_RTC_BASE(cp_index)		(MVEBU_CP_REGS_BASE(cp_index) + 0x284000)
#define MVEBU_RTC_STATUS_REG			0x0
#define MVEBU_RTC_STATUS_ALARM1_MASK		0x1
#define MVEBU_RTC_STATUS_ALARM2_MASK		0x2
#define MVEBU_RTC_TIME_REG			0xC
#define MVEBU_RTC_IRQ_1_CONFIG_REG		0x4
#define MVEBU_RTC_IRQ_2_CONFIG_REG		0x8
#define MVEBU_RTC_ALARM_1_REG			0x10
#define MVEBU_RTC_ALARM_2_REG			0x14
#define MVEBU_RTC_CCR_REG			0x18
#define MVEBU_RTC_NOMINAL_TIMING		0x2000
#define MVEBU_RTC_NOMINAL_TIMING_MASK		0x7FFF
#define MVEBU_RTC_TEST_CONFIG_REG		0x1C
#define MVEBU_RTC_BRIDGE_TIMING_CTRL0_REG_OFFS	0x0
#define MVEBU_RTC_WRCLK_PERIOD_MASK		0xFFFF
#define MVEBU_RTC_WRCLK_PERIOD_DEFAULT		0x3FF
#define MVEBU_RTC_WRCLK_SETUP_OFFS		16
#define MVEBU_RTC_WRCLK_SETUP_MASK		0xFFFF0000
#define MVEBU_RTC_WRCLK_SETUP_DEFAULT		0x29
#define MVEBU_RTC_BRIDGE_TIMING_CTRL1_REG_OFFS	0x84
#define MVEBU_RTC_READ_OUTPUT_DELAY_MASK	0xFFFF
#define MVEBU_RTC_READ_OUTPUT_DELAY_DEFAULT	0x1F

static inline uint32_t cp110_device_id_get(void)
{
	/* Returns:
	 * - MVEBU_70X0_DEV_ID for A70X0 family
	 * - MVEBU_80X0_DEV_ID for A80X0 family
	 */
	return (mmio_read_32(MVEBU_DEVICE_ID_REG) >>
		MVEBU_DEVICE_ID_OFFSET) &
		MVEBU_DEVICE_ID_MASK;
}

void cp110_init(int cp_index);
void cp110_ble_init(int cp_index);

#endif /* __PLAT_CP110_H__ */

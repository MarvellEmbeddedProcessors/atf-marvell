/*
* ***************************************************************************
* Copyright (C) 2017 Marvell International Ltd.
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

#ifndef _ARO_H_
#define _ARO_H_

#include <stdint.h>

enum hws_freq {
	CPU_FREQ_2000,
	CPU_FREQ_1800,
	CPU_FREQ_1600,
	CPU_FREQ_1400,
	CPU_FREQ_1300,
	CPU_FREQ_1200,
	CPU_FREQ_1000,
	CPU_FREQ_600,
	CPU_FREQ_800,
	DDR_FREQ_LAST,
	DDR_FREQ_SAR
};

enum cpu_clock_freq_mode {
	CPU_2000_DDR_1200_RCLK_1200 = 0x0,
	CPU_2000_DDR_1050_RCLK_1050 = 0x1,
	CPU_1600_DDR_800_RCLK_800   = 0x4,
	CPU_1800_DDR_1200_RCLK_1200 = 0x6,
	CPU_1800_DDR_1050_RCLK_1050 = 0x7,
	CPU_1600_DDR_900_RCLK_900   = 0x0B,
	CPU_1600_DDR_1050_RCLK_1050 = 0x0D,
	CPU_1600_DDR_900_RCLK_900_2 = 0x0E,
	CPU_1000_DDR_650_RCLK_650   = 0x13,
	CPU_1300_DDR_800_RCLK_800   = 0x14,
	CPU_1300_DDR_650_RCLK_650   = 0x17,
	CPU_1200_DDR_800_RCLK_800   = 0x19,
	CPU_1400_DDR_800_RCLK_800   = 0x1a,
	CPU_600_DDR_800_RCLK_800    = 0x1B,
	CPU_800_DDR_800_RCLK_800    = 0x1C,
	CPU_1000_DDR_800_RCLK_800   = 0x1D,
	CPU_DDR_RCLK_INVALID
};

int init_aro(void);

#endif /* _ARO_H_ */


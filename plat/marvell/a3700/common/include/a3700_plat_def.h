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

#ifndef __MVEBU_A3700_DEF_H__
#define __MVEBU_A3700_DEF_H__

#include <arm_def.h>


#define MVEBU_MAX_CPUS_PER_CLUSTER	2

#define MVEBU_PRIMARY_CPU		0x0

/*
 * The counter on A3700 is always fed from reference 25M clock (XTAL).
 * However minimal CPU counter prescaler is 2, so the counter
 * frequency will be divided by 2, the number is 12.5M
 */
#define COUNTER_FREQUENCY		12500000

#define MVEBU_REGS_BASE			0xD0000000

/*******************************************************************************
 * MVEBU memory map related constants
 ******************************************************************************/

/* Aggregate of all devices in the first GB */
#define DEVICE0_BASE			MVEBU_REGS_BASE
#define DEVICE0_SIZE			0x10000000

/*******************************************************************************
 * GIC-500 & interrupt handling related constants
 ******************************************************************************/
/* Base MVEBU compatible GIC memory map */
#define MVEBU_GICD_BASE			0x1D00000
#define MVEBU_GICR_BASE			0x1D40000
#define MVEBU_GICC_BASE			0x1D80000

/* CCI-400 */
#define MVEBU_CCI_BASE			0x8000000

/*******************************************************************************
 * North and south bridge register base
 ******************************************************************************/
#define MVEBU_NB_REGS_BASE			(MVEBU_REGS_BASE + 0x13000)
#define MVEBU_SB_REGS_BASE			(MVEBU_REGS_BASE + 0x18000)

/*******************************************************************************
 * GPIO registers related constants
 ******************************************************************************/
/* North and south bridge GPIO register base address */
#define MVEBU_NB_GPIO_REG_BASE			(MVEBU_NB_REGS_BASE + 0x800)
#define MVEBU_NB_GPIO_IRQ_REG_BASE		(MVEBU_NB_REGS_BASE + 0xC00)
#define MVEBU_SB_GPIO_REG_BASE			(MVEBU_SB_REGS_BASE + 0x800)
#define MVEBU_SB_GPIO_IRQ_REG_BASE		(MVEBU_SB_REGS_BASE + 0xC00)
#define MVEBU_NB_SB_IRQ_REG_BASE                (MVEBU_REGS_BASE + 0x8A00)

/* North Bridge GPIO selection regsiter */
#define MVEBU_NB_GPIO_SEL_REG			(MVEBU_NB_GPIO_REG_BASE + 0x30)
#define MVEBU_NB_GPIO_OUTPUT_EN_HIGH_REG	(MVEBU_NB_GPIO_REG_BASE + 0x04)
/* I2C1 GPIO Enable bit offset */
#define MVEBU_GPIO_TW1_GPIO_EN_OFF		(10)
/* SPI pins mode bit offset */
#define MVEBU_GPIO_NB_SPI_PIN_MODE_OFF		(28)

/*******************************************************************************
 * DRAM registers related constants
 ******************************************************************************/
#define MVEBU_DRAM_REG_BASE			(MVEBU_REGS_BASE)

/*******************************************************************************
 * SB wake-up registers related constants
 ******************************************************************************/
#define MVEBU_SB_WAKEUP_REG_BASE		(MVEBU_REGS_BASE + 0x19000)

/*******************************************************************************
 * PMSU registers related constants
 ******************************************************************************/
#define MVEBU_PMSU_REG_BASE			(MVEBU_REGS_BASE + 0x14000)

/*******************************************************************************
 * North Bridge Step-Down Registers
 ******************************************************************************/
#define MVEBU_NB_STEP_DOWN_REG_BASE		(MVEBU_REGS_BASE + 0x12800)

/*******************************************************************************
 * DRAM CS memory map register base
 ******************************************************************************/
#define MVEBU_CS_MMAP_REG_BASE			(MVEBU_REGS_BASE + 0x200)

/*******************************************************************************
 * CPU decoder window registers related constants
 ******************************************************************************/
#define MVEBU_CPU_DEC_WIN_REG_BASE		(MVEBU_REGS_BASE + 0xCF00)

/*******************************************************************************
 * AVS registers related constants
 ******************************************************************************/
#define MVEBU_AVS_REG_BASE			(MVEBU_REGS_BASE + 0x11500)

#endif /* __MVEBU_A3700_DEF_H__ */

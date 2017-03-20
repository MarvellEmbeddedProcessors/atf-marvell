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
#include <mmio.h>

#define PCIE_CFG_VENDOR			0x0
#define PCIE_CFG_DEVICE			0x2
#define PCIE_CFG_CMD			0x4
#define  PCIE_CFG_CMD_IO_EN		(1 << 0)
#define  PCIE_CFG_CMD_MEM_EN		(1 << 1)
#define  PCIE_CFG_CMD_MASTER_EN		(1 << 2)
#define PCIE_CFG_STATUS			0x6

#define PCIE_GLOBAL_CONTROL		0x8000
#define PCIE_CFG_RETRY_EN		(1 << 9)
#define PCIE_APP_LTSSM_EN		(1 << 2)
#define PCIE_DEVICE_TYPE_OFFSET		(4)
#define PCIE_DEVICE_TYPE_MASK		(0xF)
#define PCIE_DEVICE_TYPE_EP		(0x0) /* Endpoint */
#define PCIE_DEVICE_TYPE_LEP		(0x1) /* Legacy endpoint */
#define PCIE_DEVICE_TYPE_RC		(0x4) /* Root complex */

#define PCIE_LINK_CTL_2			0xA0
#define TARGET_LINK_SPEED_MASK		0xF
#define PCIE_LINK_CAPABILITY		0x7C

#define PCIE_GEN3_EQU_CTRL		0x8A8
#define GEN3_EQU_EVAL_2MS_DISABLE	(1 << 5)

#define PCIE_ARCACHE_TRC		0x8050
#define PCIE_AWCACHE_TRC		0x8054
#define ARCACHE_SHAREABLE_CACHEABLE	0x3511
#define AWCACHE_SHAREABLE_CACHEABLE	0x5311

#define LINK_SPEED_GEN_1                0x1
#define LINK_SPEED_GEN_2                0x2
#define LINK_SPEED_GEN_3                0x3

/* iATU registers */
#define PCIE_IATU_VIEWPORT		0x900
#define PCIE_IATU_CR1			0x904
#define PCIE_IATU_CR2			0x908
#define  PCIE_IATU_CR2_EN		(0x1 << 31)
#define PCIE_IATU_LOWER_BASE		0x90C
#define PCIE_IATU_UPPER_BASE		0x910
#define PCIE_IATU_LIMIT			0x914
#define PCIE_IATU_LOWER_TARGET		0x918
#define PCIE_IATU_UPPER_TARGET		0x91C

#define DW_OUTBOUND_ATU_BASE		0x8000000000ull
#define DW_OUTBOUND_ATU_SIZE		0x100000000ull
#define DW_OUTBOUND_ATU_TARGET		0x0


void dw_pcie_configure(uintptr_t regs_base, uint32_t cap_speed)
{
	uint32_t reg;

	/*  Set link to GEN 3 */;
	reg  = mmio_read_32(regs_base + PCIE_LINK_CTL_2);
	reg &= ~TARGET_LINK_SPEED_MASK;
	reg |= cap_speed;
	mmio_write_32(regs_base + PCIE_LINK_CTL_2, reg);

	reg  = mmio_read_32(regs_base + PCIE_LINK_CAPABILITY);
	reg &= ~TARGET_LINK_SPEED_MASK;
	reg |= cap_speed;
	mmio_write_32(regs_base + PCIE_LINK_CAPABILITY, reg);

	reg = mmio_read_32(regs_base + PCIE_GEN3_EQU_CTRL);
	reg |= GEN3_EQU_EVAL_2MS_DISABLE;
	mmio_write_32(regs_base + PCIE_GEN3_EQU_CTRL, reg);
}

int dw_pcie_link_up(uintptr_t regs_base, uint32_t cap_speed, int is_end_point)
{
	uint32_t reg;

	/* Disable LTSSM state machine to enable configuration */
	reg = mmio_read_32(regs_base + PCIE_GLOBAL_CONTROL);
	reg &= ~(PCIE_APP_LTSSM_EN);
	reg &= ~(PCIE_DEVICE_TYPE_MASK << PCIE_DEVICE_TYPE_OFFSET);
	if (!is_end_point)
		reg |= (PCIE_DEVICE_TYPE_RC << PCIE_DEVICE_TYPE_OFFSET);
	mmio_write_32(regs_base + PCIE_GLOBAL_CONTROL, reg);

	/* Set the PCIe master AXI attributes */
	mmio_write_32(regs_base + PCIE_ARCACHE_TRC, ARCACHE_SHAREABLE_CACHEABLE);
	mmio_write_32(regs_base + PCIE_AWCACHE_TRC, AWCACHE_SHAREABLE_CACHEABLE);

	/* DW pre link configurations */
	dw_pcie_configure(regs_base, cap_speed);

	/* Configuration done. Start LTSSM */
	reg = mmio_read_32(regs_base + PCIE_GLOBAL_CONTROL);
	reg |= PCIE_APP_LTSSM_EN;
	mmio_write_32(regs_base + PCIE_GLOBAL_CONTROL, reg);

	/* As the end-point we dont need to check if a
	 * link was established*/

	return 1;
}

void dw_pcie_open_out_atu(uintptr_t dw_base, int win_id, uint64_t local_base,
			  uint64_t remote_base, uint64_t size)
{
	mmio_write_32(dw_base + PCIE_IATU_VIEWPORT, win_id);
	mmio_write_32(dw_base + PCIE_IATU_LOWER_BASE, local_base & UINT32_MAX);
	mmio_write_32(dw_base + PCIE_IATU_UPPER_BASE, local_base >> 32);
	mmio_write_32(dw_base + PCIE_IATU_LOWER_TARGET, remote_base & UINT32_MAX);
	mmio_write_32(dw_base + PCIE_IATU_UPPER_TARGET, remote_base >> 32);
	mmio_write_32(dw_base + PCIE_IATU_LIMIT, size - 1);
	mmio_write_32(dw_base + PCIE_IATU_CR2, PCIE_IATU_CR2_EN);
}

void dw_pcie_master_enable(uintptr_t dw_base)
{
	mmio_write_32(dw_base + PCIE_CFG_CMD, PCIE_CFG_CMD_MEM_EN |
					      PCIE_CFG_CMD_IO_EN |
					      PCIE_CFG_CMD_MASTER_EN);
}

void dw_pcie_delay_cfg(uintptr_t dw_base)
{
	uint32_t ctrl;
	ctrl = mmio_read_32(dw_base + PCIE_GLOBAL_CONTROL);
	ctrl |= PCIE_CFG_RETRY_EN;
	mmio_write_32(dw_base + PCIE_GLOBAL_CONTROL, ctrl);
}

void dw_pcie_ep_init(uintptr_t dw_base, uint8_t delay_cfg, uint8_t master_en)
{
	dw_pcie_link_up(dw_base, LINK_SPEED_GEN_3, 1);

	if (master_en) {
		dw_pcie_open_out_atu(dw_base, 0, DW_OUTBOUND_ATU_BASE,
				     DW_OUTBOUND_ATU_TARGET, DW_OUTBOUND_ATU_SIZE);
		dw_pcie_master_enable(dw_base);
	}

	if (delay_cfg)
		dw_pcie_delay_cfg(dw_base);

	return;
}

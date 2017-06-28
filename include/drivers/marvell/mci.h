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

#ifndef _MCI_H_
#define _MCI_H_

/* MCI indirect access definitions */
#define MCI_MAX_UNIT_ID				2
/* SoC RFU / IHBx4 Control */
#define MCIX4_REG_START_ADDRESS_REG(unit_id)	(MVEBU_REGS_BASE + 0x6F4218 + (unit_id * 0x20))
#define MCI_REMAP_OFF_SHIFT			8

/* /HB /Units /Direct_regs /Direct regs /Configuration Register Write/Read Data Register */
#define MCI_WRITE_READ_DATA_REG(mci_index)	MVEBU_MCI_REG_BASE_REMAP(mci_index)
/* /HB /Units /Direct_regs /Direct regs /Configuration Register Access Command Register */
#define MCI_ACCESS_CMD_REG(mci_index)		(MVEBU_MCI_REG_BASE_REMAP(mci_index) + 0x4)
/* Access Command fields :
 * bit[3:0]   - Sub command: 1 => Periferal Config Register Read,
 *			     0 => Periferal Config Refister Write,
 *			     2 => Periferal Assign ID request,
 *			     3 => Circular Config Write
 * bit[5]     - 1 => Local (same chip access) 0 => Remote
 * bit[15:8]  - Destination hop ID. Put Global ID (GID) here (see scheme below).
 * bit[23:22] - 0x3 IHB PHY REG address space, 0x0 IHB Controller space
 * bit[21:16] - Low 6 bits of offset. Hight 2 bits are taken from bit[28:27]
 *              of IHB_PHY_CTRL (must be set before any PHY register access occures):
 *              /IHB_REG /IHB_REGInterchip Hopping Bus Registers /IHB Version Control Register
 *
 *              ixi_ihb_top               IHB PHY
 *  AXI -----------------------------   -------------
 *   <--| axi_hb_top | ihb_pipe_top |-->|           |
 *   -->|   GID=1    |     GID=0    |<--|           |
 *      -----------------------------   -------------
 */
#define MCI_INDIRECT_CTRL_READ_CMD		0x1
#define MCI_INDIRECT_CTRL_ASSIGN_CMD		0x2
#define MCI_INDIRECT_CTRL_CIRCULAR_CMD		0x3
#define MCI_INDIRECT_CTRL_LOCAL_PKT		(1 << 5)
#define MCI_INDIRECT_CTRL_CMD_DONE_OFFSET	6
#define MCI_INDIRECT_CTRL_CMD_DONE		(1 << MCI_INDIRECT_CTRL_CMD_DONE_OFFSET)
#define MCI_INDIRECT_CTRL_DATA_READY_OFFSET	7
#define MCI_INDIRECT_CTRL_DATA_READY		(1 << MCI_INDIRECT_CTRL_DATA_READY_OFFSET)
#define MCI_INDIRECT_CTRL_HOPID_OFFSET		8
#define MCI_INDIRECT_CTRL_HOPID(id)		(((id) & 0xFF) << MCI_INDIRECT_CTRL_HOPID_OFFSET)
#define MCI_INDIRECT_CTRL_REG_CHIPID_OFFSET	16
#define MCI_INDIRECT_REG_CTRL_ADDR(reg_num)	(reg_num << MCI_INDIRECT_CTRL_REG_CHIPID_OFFSET)
/* PHY access domain */
#define MCI_INDIRECT_CTRL_PHY_ACCESS_EN		(0xC0  << MCI_INDIRECT_CTRL_REG_CHIPID_OFFSET)
/* Hop ID values */
#define GID_IHB_PIPE				0
#define GID_AXI_HB				1
#define GID_IHB_EXT				2


#define MCI_DID_GLOBAL_ASSIGNMENT_REQUEST_REG		0x2
/* Target MCi Local ID (LID, which is = self DID) */
#define MCI_DID_GLOBAL_ASSIGN_REQ_MCI_LOCAL_ID(val)	(((val) & 0xFF) << 16)
/* Bits [15:8]: Number of MCis on chip of target MCi */
#define MCI_DID_GLOBAL_ASSIGN_REQ_MCI_COUNT(val)	(((val) & 0xFF) << 8)
/* Bits [7:0]: Number of hops on chip of target MCi */
#define MCI_DID_GLOBAL_ASSIGN_REQ_HOPS_NUM(val)		(((val) & 0xFF) << 0)

/* IHB_REG domain registers */
/* /HB /Units /IHB_REG /IHB_REGInterchip Hopping Bus Registers/
 * Rx Memory Configuration Register (RX_MEM_CFG) */
#define MCI_CTRL_RX_MEM_CFG_REG_NUM		0x0
#define MCI_CTRL_RX_TX_MEM_CFG_RQ_THRESH(val)		(((val) & 0xFF) << 24)
#define MCI_CTRL_RX_TX_MEM_CFG_PQ_THRESH(val)		(((val) & 0xFF) << 16)
#define MCI_CTRL_RX_TX_MEM_CFG_NQ_THRESH(val)		(((val) & 0xFF) << 8)
#define MCI_CTRL_RX_TX_MEM_CFG_DELTA_THRESH(val)	(((val) & 0xF) << 4)
#define MCI_CTRL_RX_TX_MEM_CFG_RTC(val)			(((val) & 0x3) << 2)
#define MCI_CTRL_RX_TX_MEM_CFG_WTC(val)			(((val) & 0x3) << 0)
#define MCI_CTRL_RX_MEM_CFG_REG_DEF_CP_VAL	(MCI_CTRL_RX_TX_MEM_CFG_RQ_THRESH(0x07) | \
						 MCI_CTRL_RX_TX_MEM_CFG_PQ_THRESH(0x3f) | \
						 MCI_CTRL_RX_TX_MEM_CFG_NQ_THRESH(0x3f) | \
						 MCI_CTRL_RX_TX_MEM_CFG_DELTA_THRESH(0xf) | \
						 MCI_CTRL_RX_TX_MEM_CFG_RTC(1) | \
						 MCI_CTRL_RX_TX_MEM_CFG_WTC(1))

#define MCI_CTRL_RX_MEM_CFG_REG_DEF_AP_VAL	(MCI_CTRL_RX_TX_MEM_CFG_RQ_THRESH(0x3f) | \
						 MCI_CTRL_RX_TX_MEM_CFG_PQ_THRESH(0x03) | \
						 MCI_CTRL_RX_TX_MEM_CFG_NQ_THRESH(0x3f) | \
						 MCI_CTRL_RX_TX_MEM_CFG_DELTA_THRESH(0xf) | \
						 MCI_CTRL_RX_TX_MEM_CFG_RTC(1) | \
						 MCI_CTRL_RX_TX_MEM_CFG_WTC(1))


/* /HB /Units /IHB_REG /IHB_REGInterchip Hopping Bus Registers/
 * Tx Memory Configuration Register (TX_MEM_CFG) */
#define MCI_CTRL_TX_MEM_CFG_REG_NUM		0x1
/* field mapping for TX mem config register are the as for RX register - see register above */
#define MCI_CTRL_TX_MEM_CFG_REG_DEF_VAL		(MCI_CTRL_RX_TX_MEM_CFG_RQ_THRESH(0x20) | \
						 MCI_CTRL_RX_TX_MEM_CFG_PQ_THRESH(0x20) | \
						 MCI_CTRL_RX_TX_MEM_CFG_NQ_THRESH(0x20) | \
						 MCI_CTRL_RX_TX_MEM_CFG_DELTA_THRESH(2) | \
						 MCI_CTRL_RX_TX_MEM_CFG_RTC(1) | \
						 MCI_CTRL_RX_TX_MEM_CFG_WTC(1))

/* /HB /Units /IHB_REG /IHB_REGInterchip Hopping Bus Registers /IHB Link CRC Control */
/* MCi Link CRC Control Register (MCi_CRC_CTRL) */
#define MCI_LINK_CRC_CTRL_REG_NUM		0x4

/* /HB /Units /IHB_REG /IHB_REGInterchip Hopping Bus Registers /IHB Status Register */
/* MCi Status Register (MCi_STS) */
#define MCI_CTRL_STATUS_REG_NUM			0x5
#define MCI_CTRL_STATUS_REG_PHY_READY		(1 << 12)
#define MCI_CTRL_STATUS_REG_LINK_PRESENT	(1 << 15)
#define MCI_CTRL_STATUS_REG_PHY_CID_VIO_OFFSET	24
#define MCI_CTRL_STATUS_REG_PHY_CID_VIO_MASK	(0xF << MCI_CTRL_STATUS_REG_PHY_CID_VIO_OFFSET)
/* Expected successful Link result, including reserved bit */
#define MCI_CTRL_PHY_READY			(MCI_CTRL_STATUS_REG_PHY_READY | \
						 MCI_CTRL_STATUS_REG_LINK_PRESENT | \
						 MCI_CTRL_STATUS_REG_PHY_CID_VIO_MASK)

/* /HB /Units /IHB_REG /IHB_REGInterchip Hopping Bus Registers/
 * MCi PHY Speed Settings Register (MCi_PHY_SETTING) */
#define MCI_CTRL_MCI_PHY_SETTINGS_REG_NUM		0x8
#define MCI_CTRL_MCI_PHY_SET_DLO_FIFO_FULL_TRESH(val)	(((val) & 0xF) << 28)
#define MCI_CTRL_MCI_PHY_SET_PHY_MAX_SPEED(val)		(((val) & 0xF) << 12)
#define MCI_CTRL_MCI_PHY_SET_PHYCLK_SEL(val)		(((val) & 0xF) << 8)
#define MCI_CTRL_MCI_PHY_SET_REFCLK_FREQ_SEL(val)	(((val) & 0xF) << 4)
#define MCI_CTRL_MCI_PHY_SET_AUTO_LINK_EN(val)		(((val) & 0x1) << 1)
#define MCI_CTRL_MCI_PHY_SET_REG_DEF_VAL		(MCI_CTRL_MCI_PHY_SET_DLO_FIFO_FULL_TRESH(0x3) | \
							 MCI_CTRL_MCI_PHY_SET_PHY_MAX_SPEED(0x3) | \
							 MCI_CTRL_MCI_PHY_SET_PHYCLK_SEL(0x2) | \
							 MCI_CTRL_MCI_PHY_SET_REFCLK_FREQ_SEL(0x1))

/* /HB /Units /IHB_REG /IHB_REGInterchip Hopping Bus Registers /IHB Mode Config */
#define MCI_CTRL_IHB_MODE_CFG_REG_NUM			0x25
#define MCI_CTRL_IHB_MODE_HBCLK_DIV(val)		((val) & 0xFF)
#define MCI_CTRL_IHB_MODE_CHUNK_MOD_OFFSET		8
#define MCI_CTRL_IHB_MODE_CHUNK_MOD			(1 << MCI_CTRL_IHB_MODE_CHUNK_MOD_OFFSET)
#define MCI_CTRL_IHB_MODE_FWD_MOD_OFFSET		9
#define MCI_CTRL_IHB_MODE_FWD_MOD			(1 << MCI_CTRL_IHB_MODE_FWD_MOD_OFFSET)
#define MCI_CTRL_IHB_MODE_SEQFF_FINE_MOD(val)		(((val) & 0xF) << 12)
#define MCI_CTRL_IHB_MODE_RX_COMB_THRESH(val)		(((val) & 0xFF) << 16)
#define MCI_CTRL_IHB_MODE_TX_COMB_THRESH(val)		(((val) & 0xFF) << 24)
#define MCI_CTRL_IHB_MODE_CFG_REG_DEF_VAL		(MCI_CTRL_IHB_MODE_HBCLK_DIV(7) | \
							MCI_CTRL_IHB_MODE_FWD_MOD | \
							MCI_CTRL_IHB_MODE_SEQFF_FINE_MOD(0xF) | \
							MCI_CTRL_IHB_MODE_RX_COMB_THRESH(0x2D) | \
							MCI_CTRL_IHB_MODE_TX_COMB_THRESH(0x45))

#define MCI_CTRL_IHB_MODE_CFG_REG_DEF_VAL_A1		(MCI_CTRL_IHB_MODE_HBCLK_DIV(6) | \
							MCI_CTRL_IHB_MODE_FWD_MOD | \
							MCI_CTRL_IHB_MODE_SEQFF_FINE_MOD(0xF) | \
							MCI_CTRL_IHB_MODE_RX_COMB_THRESH(0x3f) | \
							MCI_CTRL_IHB_MODE_TX_COMB_THRESH(0x40))
/* AXI_HB registers */
#define MCI_AXI_ACCESS_DATA_REG_NUM			0x0
#define MCI_AXI_ACCESS_PCIE_MODE			1
#define MCI_AXI_ACCESS_CACHE_CHECK_OFFSET		5
#define MCI_AXI_ACCESS_CACHE_CHECK			(1 << MCI_AXI_ACCESS_CACHE_CHECK_OFFSET)
#define MCI_AXI_ACCESS_FORCE_POST_WR_OFFSET		6
#define MCI_AXI_ACCESS_FORCE_POST_WR			(1 << MCI_AXI_ACCESS_FORCE_POST_WR_OFFSET)
#define MCI_AXI_ACCESS_DISABLE_CLK_GATING_OFFSET	9
#define MCI_AXI_ACCESS_DISABLE_CLK_GATING		(1 << MCI_AXI_ACCESS_DISABLE_CLK_GATING_OFFSET)

/* /HB /Units /HB_REG /HB_REGHopping Bus Registers /Window 0 Address Mask Register */
#define MCI_HB_CTRL_WIN0_ADDRESS_MASK_REG_NUM		0x2

/* /HB /Units /HB_REG /HB_REGHopping Bus Registers /Window 0 Destination Register */
#define MCI_HB_CTRL_WIN0_DESTINATION_REG_NUM		0x3
#define MCI_HB_CTRL_WIN0_DEST_VALID_FLAG(val)		(((val) & 0x1) << 16)
#define MCI_HB_CTRL_WIN0_DEST_ID(val)			(((val) & 0xFF) << 0)

/* /HB /Units /HB_REG /HB_REGHopping Bus Registers /Tx Control Register */
#define MCI_HB_CTRL_TX_CTRL_REG_NUM			0xD
#define MCI_HB_CTRL_TX_CTRL_PCIE_MODE_OFFSET		24
#define MCI_HB_CTRL_TX_CTRL_PCIE_MODE			(1 << MCI_HB_CTRL_TX_CTRL_PCIE_MODE_OFFSET)
#define MCI_HB_CTRL_TX_CTRL_PRI_TH_QOS(val)		(((val) & 0xF) << 12)
#define MCI_HB_CTRL_TX_CTRL_MAX_RD_CNT(val)		(((val) & 0x1F) << 6)
#define MCI_HB_CTRL_TX_CTRL_MAX_WR_CNT(val)		(((val) & 0x1F) << 0)

/* HB /Units /IHB_REG /IHB_REGInterchip Hopping Bus Registers /IHB PHY Idle Control Register */
#define MCI_PHY_P0_IDLE_CTRL_REG_NUM		0x6
#define MCI_PHY_P0_IDLE_MIN_IDLE_COUNT_OFFSET	12
#define MCI_PHY_P0_IDLE_MIN_IDLE_COUNT		(0xF << MCI_PHY_P0_IDLE_MIN_IDLE_COUNT_OFFSET)
#define MCI_PHY_P0_IDLE_SW_PWR_REQ_EN_OFFSET	16
#define MCI_PHY_P0_IDLE_SW_PWR_REQ_EN		(1 << MCI_PHY_P0_IDLE_SW_PWR_REQ_EN_OFFSET)
#define MCI_PHY_P0_IDLE_SW_RETRAIN_MODE_OFFSET	20
#define MCI_PHY_P0_IDLE_SW_RETRAIN_MODE		(1 << MCI_PHY_P0_IDLE_SW_RETRAIN_MODE_OFFSET)

/* /HB /Units /IHB_REG /IHB_REGInterchip Hopping Bus Registers /IHB Version Control Register */
#define MCI_PHY_CTRL_REG_NUM			0x7
#define MCI_PHY_CTRL_MCI_MINOR			0x6 /* BITS [3:0] */
#define MCI_PHY_CTRL_MCI_MINOR_A1		0x8 /* BITS [3:0] */
#define MCI_PHY_CTRL_MCI_MAJOR_OFFSET		4
#define MCI_PHY_CTRL_MCI_MAJOR			(1 << MCI_PHY_CTRL_MCI_MAJOR_OFFSET)
#define MCI_PHY_CTRL_MCI_SLEEP_REQ_OFFSET	11
#define MCI_PHY_CTRL_MCI_SLEEP_REQ		(1 << MCI_PHY_CTRL_MCI_SLEEP_REQ_OFFSET)
#define MCI_PHY_CTRL_MCI_PHY_MODE_OFFSET	24  /* Host=1 / Device=0 PHY mode */
#define MCI_PHY_CTRL_MCI_PHY_MODE_HOST		(1 << MCI_PHY_CTRL_MCI_PHY_MODE_OFFSET)
#define MCI_PHY_CTRL_MCI_PHY_REG_IF_MODE_OFFSET	25  /* Register=1 / PWM=0 interface */
#define MCI_PHY_CTRL_MCI_PHY_REG_IF_MODE	(1 << MCI_PHY_CTRL_MCI_PHY_REG_IF_MODE_OFFSET)
#define MCI_PHY_CTRL_MCI_PHY_RESET_CORE_OFFSET	26  /* PHY code InReset=1 */
#define MCI_PHY_CTRL_MCI_PHY_RESET_CORE		(1 << MCI_PHY_CTRL_MCI_PHY_RESET_CORE_OFFSET)
#define MCI_PHY_CTRL_PHY_ADDR_MSB_OFFSET	27
#define MCI_PHY_CTRL_PHY_ADDR_MSB(addr)		(((addr) & 0x3) << MCI_PHY_CTRL_PHY_ADDR_MSB_OFFSET)
#define MCI_PHY_CTRL_PIDI_MODE_OFFSET		31
#define MCI_PHY_CTRL_PIDI_MODE			(1 << MCI_PHY_CTRL_PIDI_MODE_OFFSET)
/* /IHB_8G_X4_PHY_V /Units /Registers /PWM Control Register 2 */
#define MCI_PHY_PWM2_REG_NUM			0x6
#define PWM2_SPEED_V3_8G			(3)
#define PWM2_SPEED_V2_4G			(2)
#define PWM2_SPEED_V1_2G			(1)
#define PWM2_SPEED_V0_1G			(0)
#define PWM2_SPEED_FORCE_OFFSET			10
#define PWM2_SPEED_FORCE			(1 << PWM2_SPEED_FORCE_OFFSET)
#define PWM2_RX_LINE_EN_OFFSET			21
#define PWM2_RX_LINE_EN				(1 << PWM2_RX_LINE_EN_OFFSET)
#define PWM2_TX_LINE_EN_OFFSET			29
#define PWM2_TX_LINE_EN				(1 << PWM2_TX_LINE_EN_OFFSET)

/* /IHB_8G_X4_PHY_V /Units /Registers /PWM Control Register 3 */
#define MCI_PHY_PWM3_REG_NUM			0x7
#define PWM3_LINK_ERROR_OFFSET			8	/* [11:8] : LINK_ERROR */
#define PWM3_LINK_ERROR_MASK			(0xF << PWM3_LINK_ERROR_OFFSET)
#define PWM3_NUM_OF_LANES_OFFSET		20	/* [20:22] : AUTO_NUMBER_OF_LANES */
#define PWM3_NUM_OF_LANES_MASK			(0x7 << PWM3_NUM_OF_LANES_OFFSET)
#define PWM3_LINK_SPEED_MASK			0x7	/* [3:0] AUTO_SPEED */

/* Number of times to wait for the MCI link ready after MCI configurations
 * Normally takes 34-35 successive reads
 */
#define LINK_READY_TIMEOUT			100

enum mci_register_type {
	MCI_REG_TYPE_PHY = 0,
	MCI_REG_TYPE_CTRL,
};

void mci_remap_indirect_access_base(void);
int mci_initialize(int mci_index);

#endif /* _MCI_H_ */

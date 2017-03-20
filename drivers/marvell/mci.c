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
#include <plat_marvell.h>
#include <debug.h>
#include <mmio.h>
#include <mci.h>
#include <apn806_setup.h>
#include <delay_timer.h>

enum {
	MCI_CMD_WRITE,
	MCI_CMD_READ
};

/* Write wrapper callback for debug:
 * will print written data in case LOG_LEVEL >= 40
 */
static void mci_mmio_write_32(uintptr_t addr, uint32_t value)
{
	VERBOSE("Write:\t0x%x = 0x%x\n", (uint32_t)addr, value);
	mmio_write_32(addr, value);
}
/* Read wrapper callback for debug:
 * will print read data in case LOG_LEVEL >= 40
 */
static uint32_t mci_mmio_read_32(uintptr_t addr)
{
	uint32_t value;
	value = mmio_read_32(addr);
	VERBOSE("Read:\t0x%x = 0x%x\n", (uint32_t)addr, value);
	return value;
}

/* MCI indirect access command completion polling:
 * Each write/read command done via MCI indirect registers must be polled
 * for command completions status.
 *
 * Returns 1 in case of error
 * Returns 0 in case of command completed successfully.
 */
static int mci_poll_command_completion(int mci_index, int command_type)
{
	uint32_t mci_cmd_value = 0, retry_count = 100;
	uint32_t completion_flags = MCI_INDIRECT_CTRL_CMD_DONE;

	/* Read commands require validating that requested data is ready */
	if (command_type == MCI_CMD_READ)
		completion_flags |= MCI_INDIRECT_CTRL_DATA_READY;

	do {
		/* wait 1 ms before each polling */
		mdelay(1);
		mci_cmd_value = mci_mmio_read_32(MCI_ACCESS_CMD_REG(mci_index));
	} while (((mci_cmd_value & completion_flags) != completion_flags) &&
			 (retry_count-- > 0));

	if (retry_count == 0) {
		ERROR("%s: MCI command timeout (command status = 0x%x)\n", __func__, mci_cmd_value);
		return 1;
	}

	return 0;
}

/* Routine to enable register-mode access to PHYs over MCI0 indirect read/write*/
static void mci_enable_phy_regs_access(int mci_index)
{
	uint32_t reg_data = MCI_PHY_CTRL_MCI_PHY_REG_IF_MODE |
			    MCI_PHY_CTRL_MCI_MAJOR | MCI_PHY_CTRL_MCI_MINOR;

	/* Enable PHY REG access on remote (CP, device mode, GID=2) */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index), reg_data);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_PHY_CTRL_REG_NUM) |
			  MCI_INDIRECT_CTRL_HOPID(GID_IHB_EXT));
	mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* Enable PHY REG access localy (AP, host mode, GID=0) */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  reg_data | MCI_PHY_CTRL_MCI_PHY_MODE_HOST);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_PHY_CTRL_REG_NUM) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);
	mci_poll_command_completion(mci_index, MCI_CMD_WRITE);
}

/* read mci0 PHY/CTRL registers via indirect (local) access */
static uint32_t mci_indirect_read(int reg_num, enum mci_register_type reg_type, int mci_index)
{
	uint32_t indirect_reg_address = MCI_INDIRECT_REG_CTRL_ADDR(reg_num) |
					MCI_INDIRECT_CTRL_READ_CMD;

	/* Access to PHY registers requires special configuration */
	if (reg_type == MCI_REG_TYPE_PHY)
		indirect_reg_address |= MCI_INDIRECT_CTRL_PHY_ACCESS_EN;

	/* Local access - same chip */
	indirect_reg_address |= MCI_INDIRECT_CTRL_LOCAL_PKT;

	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index), indirect_reg_address);
	return mci_mmio_read_32(MCI_WRITE_READ_DATA_REG(mci_index));
}

/* Force MCI link speed to 8Gbps */
static void mci_link_force_speed_8g(int mci_index)
{
	uint32_t reg_data;

	/* Force link speed localy on AP PHY */
	reg_data = PWM2_SPEED_V3_8G | PWM2_SPEED_FORCE |
		   PWM2_RX_LINE_EN | PWM2_TX_LINE_EN;

	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index), reg_data);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_PHY_PWM2_REG_NUM) |
			  MCI_INDIRECT_CTRL_PHY_ACCESS_EN |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);

	/* Force link speed remotely on CP PHY */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index), reg_data);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_PHY_PWM2_REG_NUM) |
			  MCI_INDIRECT_CTRL_PHY_ACCESS_EN |
			  MCI_INDIRECT_CTRL_HOPID(GID_IHB_EXT));

	/* Enable SW power state requests control mode */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_PHY_P0_IDLE_MIN_IDLE_COUNT |
			  MCI_PHY_P0_IDLE_SW_PWR_REQ_EN |
			  MCI_PHY_P0_IDLE_SW_RETRAIN_MODE);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_PHY_P0_IDLE_CTRL_REG_NUM) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);

	/* Toggle power state request */
	reg_data = MCI_PHY_CTRL_MCI_PHY_REG_IF_MODE |
		   MCI_PHY_CTRL_MCI_PHY_MODE_HOST |
		   MCI_PHY_CTRL_MCI_SLEEP_REQ |
		   MCI_PHY_CTRL_MCI_MAJOR |
		   MCI_PHY_CTRL_MCI_MINOR;

	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index), reg_data);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_PHY_CTRL_REG_NUM) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);

	reg_data &= ~MCI_PHY_CTRL_MCI_SLEEP_REQ;

	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index), reg_data);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_PHY_CTRL_REG_NUM) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);

	/* Return power state requests control mode back to HW */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_PHY_P0_IDLE_MIN_IDLE_COUNT |
			  MCI_PHY_P0_IDLE_SW_RETRAIN_MODE);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_PHY_P0_IDLE_CTRL_REG_NUM) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);

	/* Reset all fields in link CRC control register */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index), 0);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_LINK_CRC_CTRL_REG_NUM) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);
}

/* Perform 3 configurations in one command: PCI mode, queues separation and cache bit */
static int mci_axi_set_pcie_mode(int mci_index)
{
	uint32_t reg_data;

	/* This configuration makes MCI IP behave consistently with AXI protocol.
	 * It should be configured at one side only (for example localy at AP).
	 * The IP takes care of performing the same configurations at MCI on another
	 * side (for example remotely at CP).
	 */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_AXI_ACCESS_PCIE_MODE |
			  MCI_AXI_ACCESS_CACHE_CHECK |
			  MCI_AXI_ACCESS_FORCE_POST_WR |
			  MCI_AXI_ACCESS_DISABLE_CLK_GATING);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_AXI_ACCESS_DATA_REG_NUM) |
			  MCI_INDIRECT_CTRL_HOPID(GID_AXI_HB) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT |
			  MCI_INDIRECT_CTRL_CIRCULAR_CMD);

	/* if Write command was successful, verify PCIe mode */
	if (mci_poll_command_completion(mci_index, MCI_CMD_WRITE) == 0) {
		/* Verify the PCIe mode selected */
		mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
				  MCI_INDIRECT_REG_CTRL_ADDR(MCI_HB_CTRL_TX_CTRL_REG_NUM) |
				  MCI_INDIRECT_CTRL_HOPID(GID_AXI_HB) |
				  MCI_INDIRECT_CTRL_LOCAL_PKT |
				  MCI_INDIRECT_CTRL_READ_CMD);
		/* if read was completed, verify PCIe mode */
		if (mci_poll_command_completion(mci_index, MCI_CMD_READ) == 0) {
			reg_data = mci_mmio_read_32(MCI_WRITE_READ_DATA_REG(mci_index));
			if (reg_data & MCI_HB_CTRL_TX_CTRL_PCIE_MODE)
				return 0;
		}
	}
	return 1;
}

/* Reduce sequence FIFO timer expiration threshold */
static int mci_axi_set_fifo_thresh(int mci_index)
{
	uint32_t reg_data;

	/* This configuration reduces sequence FIFO timer expiration threshold (to 0x7 instead of 0xA).
	 * In MCI 1.6 version this configuration prevents possible functional issues.
	 * In version 1.82 the configuration prevents performance degradation
	 */
	/* Configure local AP side */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_CTRL_IHB_MODE_CFG_REG_DEF_VAL);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_CTRL_IHB_MODE_CFG_REG_NUM) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);

	/* Check the command execution status */
	reg_data = mci_mmio_read_32(MCI_ACCESS_CMD_REG(mci_index));
	if (reg_data | MCI_INDIRECT_CTRL_CMD_DONE) {
		/* Configure remote CP side */
		mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
				  MCI_CTRL_IHB_MODE_CFG_REG_DEF_VAL);
		mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
				  MCI_INDIRECT_REG_CTRL_ADDR(MCI_CTRL_IHB_MODE_CFG_REG_NUM) |
				  MCI_INDIRECT_CTRL_HOPID(GID_IHB_EXT));

		/* Check the command execution status */
		reg_data = mci_mmio_read_32(MCI_ACCESS_CMD_REG(mci_index));
		if (reg_data | MCI_INDIRECT_CTRL_CMD_DONE)
			return 0;
	}

	return 1;
}

/* Reduce sequence FIFO timer expiration threshold for A1, including PIDI workaround */
static int mci_axi_set_fifo_thresh_a1(int mci_index)
{
	uint32_t reg_data, ret = 0;

	/* This configuration reduces sequence FIFO timer expiration threshold (to 0x7 instead of 0xA).
	 * In MCI 1.6 version this configuration prevents possible functional issues.
	 * In version 1.82 the configuration prevents performance degradation
	 */

	/* Configure local AP side */
	/* PIDI Workaround for entering PIDI mode */
	reg_data = MCI_PHY_CTRL_PIDI_MODE | MCI_PHY_CTRL_MCI_PHY_REG_IF_MODE
		| MCI_PHY_CTRL_MCI_PHY_MODE_HOST
		| MCI_PHY_CTRL_MCI_MAJOR | MCI_PHY_CTRL_MCI_MINOR_A1;
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index), reg_data);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_PHY_CTRL_REG_NUM) | MCI_INDIRECT_CTRL_LOCAL_PKT);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* Reduce the threshold */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_CTRL_IHB_MODE_CFG_REG_DEF_VAL_A1);

	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_CTRL_IHB_MODE_CFG_REG_NUM) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* Exit PIDI mode */
	reg_data = MCI_PHY_CTRL_MCI_PHY_REG_IF_MODE | MCI_PHY_CTRL_MCI_PHY_MODE_HOST
		| MCI_PHY_CTRL_MCI_MAJOR | MCI_PHY_CTRL_MCI_MINOR_A1;
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index), reg_data);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_PHY_CTRL_REG_NUM) | MCI_INDIRECT_CTRL_LOCAL_PKT);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);


	/* Configure remote CP side */
	/* PIDI Workaround for entering PIDI mode */
	reg_data = MCI_PHY_CTRL_PIDI_MODE | MCI_PHY_CTRL_MCI_MAJOR | MCI_PHY_CTRL_MCI_MINOR_A1;
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index), reg_data);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_PHY_CTRL_REG_NUM) | MCI_CTRL_IHB_MODE_FWD_MOD);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* Reduce the threshold */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_CTRL_IHB_MODE_CFG_REG_DEF_VAL_A1);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_CTRL_IHB_MODE_CFG_REG_NUM) |
			  MCI_INDIRECT_CTRL_HOPID(GID_IHB_EXT));
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* Exit PIDI mode */
	reg_data = MCI_PHY_CTRL_MCI_MAJOR | MCI_PHY_CTRL_MCI_MINOR_A1;
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index), reg_data);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_PHY_CTRL_REG_NUM) | MCI_CTRL_IHB_MODE_FWD_MOD);

	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	return ret;
}

/* Configure:
 * 1. AP & CP TX thresholds and delta configurations
 * 2. DLO & DLI FIFO full threshold
 * 3. RX thresholds and delta configurations
 * 4. CP AR and AW outstanding
 * 5. AP AR and AW outstanding
 */
static int mci_axi_set_fifo_rx_tx_thresh_a1(int mci_index)
{
	uint32_t ret = 0;

	/* AP TX thresholds and delta configurations (IHB_reg 0x1) */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_CTRL_TX_MEM_CFG_REG_DEF_VAL);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_CTRL_TX_MEM_CFG_REG_NUM) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* CP TX thresholds and delta configurations (IHB_reg 0x1) */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_CTRL_TX_MEM_CFG_REG_DEF_VAL);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_CTRL_TX_MEM_CFG_REG_NUM) |
			  MCI_INDIRECT_CTRL_HOPID(GID_IHB_EXT));
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* AP DLO & DLI FIFO full threshold & Auto-Link enable (IHB_reg 0x8) */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_CTRL_MCI_PHY_SET_REG_DEF_VAL | MCI_CTRL_MCI_PHY_SET_AUTO_LINK_EN(1));
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_CTRL_MCI_PHY_SETTINGS_REG_NUM) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* CP DLO & DLI FIFO full threshold (IHB_reg 0x8) */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_CTRL_MCI_PHY_SET_REG_DEF_VAL);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_CTRL_MCI_PHY_SETTINGS_REG_NUM) |
			  MCI_INDIRECT_CTRL_HOPID(GID_IHB_EXT));
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* AP RX thresholds and delta configurations (IHB_reg 0x0) */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_CTRL_RX_MEM_CFG_REG_DEF_VAL);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_CTRL_RX_MEM_CFG_REG_NUM) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* CP RX thresholds and delta configurations (IHB_reg 0x0) */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_CTRL_RX_MEM_CFG_REG_DEF_VAL);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_CTRL_RX_MEM_CFG_REG_NUM) |
			  MCI_INDIRECT_CTRL_HOPID(GID_IHB_EXT));
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* AP AR & AW maximum AXI outstanding request configuration (HB_reg 0xd) */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_HB_CTRL_TX_CTRL_PRI_TH_QOS(8) |
			  MCI_HB_CTRL_TX_CTRL_MAX_RD_CNT(7) |
			  MCI_HB_CTRL_TX_CTRL_MAX_WR_CNT(7));
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_HB_CTRL_TX_CTRL_REG_NUM) |
			  MCI_INDIRECT_CTRL_HOPID(GID_AXI_HB) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* CP AR & AW maximum AXI outstanding request configuration (HB_reg 0xd) */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_HB_CTRL_TX_CTRL_PRI_TH_QOS(8) |
			  MCI_HB_CTRL_TX_CTRL_MAX_RD_CNT(15) |
			  MCI_HB_CTRL_TX_CTRL_MAX_WR_CNT(15));
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_HB_CTRL_TX_CTRL_REG_NUM) |
			  MCI_INDIRECT_CTRL_HOPID(GID_IHB_EXT) |
			  MCI_INDIRECT_CTRL_HOPID(GID_AXI_HB));
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	return ret;
}

/* configure MCI to allow read & write transactions to arrive at the same time.
 * Without the below configuration, MCI won't sent response to CPU for transactions
 * which arrived simultaneously and will lead to CPU hang.
 * The below will configure MCI to be able to pass transactions from/to CP/AP.
 */
int mci_enable_simultaneous_transactions(int mci_index)
{
	uint32_t ret = 0;

	/* ID assignment (assigning global ID offset to CP) */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(0),
			  MCI_DID_GLOBAL_ASSIGN_REQ_MCI_LOCAL_ID(2) |
			  MCI_DID_GLOBAL_ASSIGN_REQ_MCI_COUNT(2) |
			  MCI_DID_GLOBAL_ASSIGN_REQ_HOPS_NUM(2));
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(0),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_DID_GLOBAL_ASSIGNMENT_REQUEST_REG) |
			  MCI_INDIRECT_CTRL_ASSIGN_CMD);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* Assigning destination ID=3 to all transactions entering from AXI at AP */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(0),
			  MCI_HB_CTRL_WIN0_DEST_VALID_FLAG(1) |
			  MCI_HB_CTRL_WIN0_DEST_ID(3));
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(0),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_HB_CTRL_WIN0_DESTINATION_REG_NUM) |
			  MCI_INDIRECT_CTRL_HOPID(GID_AXI_HB) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* Assigning destination ID=1 to all transactions entering from AXI at CP */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(0),
			  MCI_HB_CTRL_WIN0_DEST_VALID_FLAG(1) |
			  MCI_HB_CTRL_WIN0_DEST_ID(1));
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(0),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_HB_CTRL_WIN0_DESTINATION_REG_NUM) |
			  MCI_INDIRECT_CTRL_HOPID(GID_IHB_EXT) |
			  MCI_INDIRECT_CTRL_HOPID(GID_AXI_HB));
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* End address to all transactions entering from AXI at AP. This will lead to
	 * get match for any AXI address, and receive destination ID=3 */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(0), 0xffffffff);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(0),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_HB_CTRL_WIN0_ADDRESS_MASK_REG_NUM) |
			  MCI_INDIRECT_CTRL_HOPID(GID_AXI_HB) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* End address to all transactions entering from AXI at CP. This will lead to
	 * get match for any AXI address, and receive destination ID=1 */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(0), 0xffffffff);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(0),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_HB_CTRL_WIN0_ADDRESS_MASK_REG_NUM) |
			  MCI_INDIRECT_CTRL_HOPID(GID_IHB_EXT) |
			  MCI_INDIRECT_CTRL_HOPID(GID_AXI_HB));
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	return ret;
}

/* For A1 revision, configure the MCI link for performance improvement:
 * - set MCI to support read/write transactions to arrive at the same time
 * - Switch AXI to PCIe mode
 * - Reduce sequence FIFO threshold
 * - Configure RX/TX FIFO thresholds
 *
 *   Note:
 *   We don't exit on error code from any sub routine, to try (best effort) to
 *   complete the MCI configuration.
 *   (If we exit - Bootloader will surely fail to boot)
 */
int mci_configure_a1(int mci_index)
{
	int rval;

	/* set MCI to support read/write transactions to arrive at the same time */
	rval = mci_enable_simultaneous_transactions(mci_index);
	if (rval)
		ERROR("Failed to set MCI for simultaneous read/write transactions\n");

	/* enable PHY register mode read/write access */
	mci_enable_phy_regs_access(mci_index);

	/* Configure MCI for more consistent behavior with AXI protocol */
	rval = mci_axi_set_pcie_mode(mci_index);
	if (rval)
		ERROR("Failed to set MCI to AXI PCIe mode\n");

	/* reduce FIFO global threshold */
	rval = mci_axi_set_fifo_thresh_a1(mci_index);
	if (rval)
		ERROR("Failed to set MCI FIFO global threshold\n");

	/* configure RX/TX FIFO thresholds */
	rval = mci_axi_set_fifo_rx_tx_thresh_a1(mci_index);
	if (rval)
		ERROR("Failed to set MCI RX/TX FIFO threshold\n");

	return 1;
}

/* For A0 revision, Initialize the MCI link and check its status.
 * - Configure MCI indirect access registers for register-mode access
 * - Foce MCI link speed to 8Gbps
 * - Switch AXI to PCIe mode
 * - Reduce sequence FIFO threshold
 * - Check the status of MCI link
 * - Reset SoC on unrecoverable link fail
 */
int mci_link_init_a0(int mci_index)
{
	uint32_t ctrl_status, phy_status, link_error;
	int rval, n;

	/* enable PHY register mode read/write access */
	mci_enable_phy_regs_access(mci_index);

	INFO("Force the MCI0 link speed to 8GBps\n");
	/* APN806-A0: Force link speed to 8Gbps */
	mci_link_force_speed_8g(mci_index);

	/* Configure MCI for more consistent behavior with AXI protocol */
	rval = mci_axi_set_pcie_mode(mci_index);
	if (rval)
		ERROR("Failed to set AXI PCIe mode\n");

	/* reduce FIFO threshold */
	rval = mci_axi_set_fifo_thresh(mci_index);
	if (rval)
		ERROR("Failed to set FIFO threshold\n");

	for (n = 0; n < LINK_READY_TIMEOUT; n++) {
		/* MCI Controller link status*/
		ctrl_status = mci_indirect_read(MCI_CTRL_STATUS_REG_NUM, MCI_REG_TYPE_CTRL, mci_index);

		/* MCI PHY link status: PWM Control #3*/
		phy_status = mci_indirect_read(MCI_PHY_PWM3_REG_NUM, MCI_REG_TYPE_PHY, mci_index);

		if (ctrl_status == MCI_CTRL_PHY_READY)
			break;
	}

	if (ctrl_status != MCI_CTRL_PHY_READY) {
		ERROR(" Link failed (MCI status: 0x%x, PWM_CTRL #3 = 0x%x)\n",
		      ctrl_status, phy_status);
		goto reboot;
	}

	link_error = (phy_status & PWM3_LINK_ERROR_MASK) >> PWM3_LINK_ERROR_OFFSET;
	if (link_error) {
		ERROR(" Link Error #%d: (MCI status: 0%x)\n", link_error, ctrl_status);
		goto reboot;
	}
	INFO("MCI0 link is UP\n");

	return 1;

reboot:
	/* Unrecoverable error, no WA exists, requires HW reset */
	ERROR("REBOOTING...");
	plat_marvell_system_reset();
	/* Never reached */
	return 0;
}

/* Initialize MCI
 * - Performance improvements for A0 & A1
 * - A0 revision configuration include MCI link initialization */
int mci_initialize(int mci_index)
{
	INFO("MCI%d initialization:\n", mci_index);

	if (apn806_rev_id_get() == APN806_REV_ID_A0)
		return mci_link_init_a0(0);

	/* Else, for A1 configure MCI for improved performance */
	mci_configure_a1(0);
	return 1; /* Link is always guaranteed for A1 */
}

/* MCIx indirect access register are based by default at 0xf4000000/0xf6000000
 * to avoid conflict of internal registers of units connected via MCIx, which
 * can be based on the same address (i.e CP1 base is also 0xf4000000),
 * the following routines remaps the MCIx indirect bases to another domain
 */
void mci_remap_indirect_access_base(void)
{
	uint32_t i;

	for (i = 0; i < MCI_MAX_UNIT_ID; ++i) {
		mci_mmio_write_32(MCIX4_REG_START_ADDRESS_REG(i),
				  MVEBU_MCI_REG_BASE_REMAP(i) >> MCI_REMAP_OFF_SHIFT);
	}
}

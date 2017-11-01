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

/* Reduce sequence FIFO timer expiration threshold, including PIDI workaround */
static int mci_axi_set_fifo_thresh(int mci_index)
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
		| MCI_PHY_CTRL_MCI_MAJOR | MCI_PHY_CTRL_MCI_MINOR;
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index), reg_data);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_PHY_CTRL_REG_NUM) | MCI_INDIRECT_CTRL_LOCAL_PKT);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* Reduce the threshold */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_CTRL_IHB_MODE_CFG_REG_DEF_VAL);

	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_CTRL_IHB_MODE_CFG_REG_NUM) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* Exit PIDI mode */
	reg_data = MCI_PHY_CTRL_MCI_PHY_REG_IF_MODE | MCI_PHY_CTRL_MCI_PHY_MODE_HOST
		| MCI_PHY_CTRL_MCI_MAJOR | MCI_PHY_CTRL_MCI_MINOR;
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index), reg_data);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_PHY_CTRL_REG_NUM) | MCI_INDIRECT_CTRL_LOCAL_PKT);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);


	/* Configure remote CP side */
	/* PIDI Workaround for entering PIDI mode */
	reg_data = MCI_PHY_CTRL_PIDI_MODE | MCI_PHY_CTRL_MCI_MAJOR | MCI_PHY_CTRL_MCI_MINOR |
		   MCI_PHY_CTRL_MCI_PHY_REG_IF_MODE;
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index), reg_data);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_PHY_CTRL_REG_NUM) | MCI_CTRL_IHB_MODE_FWD_MOD);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* Reduce the threshold */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_CTRL_IHB_MODE_CFG_REG_DEF_VAL);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_CTRL_IHB_MODE_CFG_REG_NUM) |
			  MCI_INDIRECT_CTRL_HOPID(GID_IHB_EXT));
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* Exit PIDI mode */
	reg_data = MCI_PHY_CTRL_MCI_MAJOR | MCI_PHY_CTRL_MCI_MINOR | MCI_PHY_CTRL_MCI_PHY_REG_IF_MODE;
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
static int mci_axi_set_fifo_rx_tx_thresh(int mci_index)
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
			  MCI_CTRL_RX_MEM_CFG_REG_DEF_AP_VAL);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_CTRL_RX_MEM_CFG_REG_NUM) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* CP RX thresholds and delta configurations (IHB_reg 0x0) */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_CTRL_RX_MEM_CFG_REG_DEF_CP_VAL);
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_CTRL_RX_MEM_CFG_REG_NUM) |
			  MCI_INDIRECT_CTRL_HOPID(GID_IHB_EXT));
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* AP AR & AW maximum AXI outstanding request configuration (HB_reg 0xd) */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_HB_CTRL_TX_CTRL_PRI_TH_QOS(8) |
			  MCI_HB_CTRL_TX_CTRL_MAX_RD_CNT(3) |
			  MCI_HB_CTRL_TX_CTRL_MAX_WR_CNT(3));
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(mci_index),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_HB_CTRL_TX_CTRL_REG_NUM) |
			  MCI_INDIRECT_CTRL_HOPID(GID_AXI_HB) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT);
	ret |= mci_poll_command_completion(mci_index, MCI_CMD_WRITE);

	/* CP AR & AW maximum AXI outstanding request configuration (HB_reg 0xd) */
	mci_mmio_write_32(MCI_WRITE_READ_DATA_REG(mci_index),
			  MCI_HB_CTRL_TX_CTRL_PRI_TH_QOS(8) |
			  MCI_HB_CTRL_TX_CTRL_MAX_RD_CNT(0xB) |
			  MCI_HB_CTRL_TX_CTRL_MAX_WR_CNT(0x11));
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
static int mci_enable_simultaneous_transactions(int mci_index)
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

/* Check if MCI simultaneous transaction was already enabled.
 * Currently bootrom does this mci configuration only when the boot source is
 * SAR_MCIX4, in other cases it should be done at this stage.
 * It is worth noticing that in case of booting from uart, the bootrom
 * flow is different and this mci initialization is skipped even if boot
 * source is SAR_MCIX4. Therefore new verification bases on appropriate mci's
 * register content: if the appropriate reg contains 0x0 it means that the
 * bootrom didn't perform required mci configuration.
 *
 * Returns:
 * 0 - configuration already done
 * 1 - configuration missing
 */
static _Bool mci_simulatenous_trans_missing(int mci_index)
{
	uint32_t reg, ret;

	/* read 'Window 0 Destination ID assignment' from HB register 0x3
	 * (TX_CFG_W0_DST_ID) to check whether ID assignment was already
	  * performed by BootROM.
	 */
	mci_mmio_write_32(MCI_ACCESS_CMD_REG(0),
			  MCI_INDIRECT_REG_CTRL_ADDR(MCI_HB_CTRL_WIN0_DESTINATION_REG_NUM) |
			  MCI_INDIRECT_CTRL_HOPID(GID_AXI_HB) |
			  MCI_INDIRECT_CTRL_LOCAL_PKT |
			  MCI_INDIRECT_CTRL_READ_CMD);
	ret = mci_poll_command_completion(mci_index, MCI_CMD_READ);

	reg = mci_mmio_read_32(MCI_WRITE_READ_DATA_REG(mci_index));

	if (ret)
		ERROR("Failed to verify if MCI simultaneous read/write was enabled\n");

	/* default ID assignment is 0, so if register doesn't contain zeros
	 * it means that bootrom already performed required configuration.
	 */
	if (reg != 0)
		return 0;

	return 1;
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
int mci_configure(int mci_index)
{
	int rval;

	/* According to design guidelines the MCI simultaneous transaction
	 * shouldn't be enabled more then once - therefore make sure that it
	 * wasn't already enabled in bootrom.
	 */
	if (mci_simulatenous_trans_missing(mci_index)) {
		VERBOSE("Enabling MCI simultaneous transaction\n");
		/* set MCI to support read/write transactions to arrive at the same time */
		rval = mci_enable_simultaneous_transactions(mci_index);
		if (rval)
			ERROR("Failed to set MCI for simultaneous read/write transactions\n");
	} else
		VERBOSE("Skipping MCI ID assignment - already done by bootrom\n");

	/* Configure MCI for more consistent behavior with AXI protocol */
	rval = mci_axi_set_pcie_mode(mci_index);
	if (rval)
		ERROR("Failed to set MCI to AXI PCIe mode\n");

	/* reduce FIFO global threshold */
	rval = mci_axi_set_fifo_thresh(mci_index);
	if (rval)
		ERROR("Failed to set MCI FIFO global threshold\n");

	/* configure RX/TX FIFO thresholds */
	rval = mci_axi_set_fifo_rx_tx_thresh(mci_index);
	if (rval)
		ERROR("Failed to set MCI RX/TX FIFO threshold\n");

	return 1;
}

/* Initialize MCI for performance improvements */
int mci_initialize(int mci_index)
{
	INFO("MCI%d initialization:\n", mci_index);

	return mci_configure(mci_index);
}

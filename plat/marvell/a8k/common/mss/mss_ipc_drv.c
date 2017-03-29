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

#include <debug.h>
#include <string.h>
#include <mss_ipc_drv.h>
#include <mmio.h>

#define IPC_MSG_BASE_MASK		MVEBU_REGS_BASE_MASK


unsigned long mv_pm_ipc_msg_base;
unsigned int  mv_pm_ipc_num_of_ch;
unsigned int  mv_pm_ipc_channel_size;
unsigned int  mv_pm_ipc_queue_size;

/*******************************************************************************
 * mss_pm_ipc_init
 *
 * DESCRIPTION: Initialize PM IPC infrastructure
 ******************************************************************************
 */
int mv_pm_ipc_init(unsigned long ipc_control_addr)
{
	struct mss_pm_ipc_ctrl *ipc_control =
			(struct mss_pm_ipc_ctrl *)ipc_control_addr;

	/* Initialize PM IPC control block */
	mv_pm_ipc_msg_base     = ipc_control->msg_base_address |
						IPC_MSG_BASE_MASK;
	mv_pm_ipc_num_of_ch    = ipc_control->num_of_channels;
	mv_pm_ipc_channel_size = ipc_control->channel_size;
	mv_pm_ipc_queue_size   = ipc_control->queue_size;

	return 0;
}

/*******************************************************************************
 * mv_pm_ipc_queue_addr_get
 *
 * DESCRIPTION: Returns the IPC queue address
 ******************************************************************************
 */
unsigned int mv_pm_ipc_queue_addr_get(unsigned int channel_id,
				      unsigned int direction)
{
	return  (unsigned int)(mv_pm_ipc_msg_base +
			       (channel_id * mv_pm_ipc_channel_size) +
			       (direction  * mv_pm_ipc_queue_size));
}

/*******************************************************************************
 * mv_pm_ipc_msg_rx
 *
 * DESCRIPTION: Retrieve message from IPC channel
 ******************************************************************************
 */
int mv_pm_ipc_msg_rx(unsigned int channel_id, unsigned int direction,
		     struct mss_pm_ipc_msg *msg)
{
	unsigned int addr = mv_pm_ipc_queue_addr_get(channel_id, direction);

	msg->msg_reply = mmio_read_32(addr + IPC_MSG_REPLY_LOC);

	return 0;
}

/*******************************************************************************
 * mss_pm_ipc_msg_reply
 *
 * DESCRIPTION: Send message via IPC channel
 ******************************************************************************
 */
int mv_pm_ipc_msg_tx(unsigned int channel_id, unsigned int direction,
		     unsigned int cluster_power_state)
{
	unsigned int addr = mv_pm_ipc_queue_addr_get(channel_id, direction);

	mmio_write_32(addr + IPC_MSG_POWER_STATE_LOC, cluster_power_state);
	mmio_write_32(addr + IPC_MSG_STATE_LOC, IPC_MSG_OCCUPY);

	return 0;
}

/*******************************************************************************
 * mv_pm_ipc_msg_validate
 *
 * DESCRIPTION: Validate IPC channel state
 ******************************************************************************
 */
int mv_pm_ipc_msg_validate(unsigned int channel_id, unsigned int direction,
			   unsigned int state)
{
	unsigned int addr = mv_pm_ipc_queue_addr_get(channel_id, direction);

	if (mmio_read_32(addr + IPC_MSG_STATE_LOC) != state)
		return -1;

	return 0;
}

/*******************************************************************************
 * mv_pm_ipc_msg_update
 *
 * DESCRIPTION: Update IPC channel state
 ******************************************************************************
 */
int mv_pm_ipc_msg_update(unsigned int channel_id, unsigned int direction,
			 unsigned int state)
{
	unsigned int addr = mv_pm_ipc_queue_addr_get(channel_id, direction);

	mmio_write_32(addr + IPC_MSG_STATE_LOC, state);

	return 0;
}

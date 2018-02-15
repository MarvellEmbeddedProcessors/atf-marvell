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

#include <plat_marvell.h>
#include <debug.h>
#include <string.h>
#include <mss_ipc_drv.h>
#include <mmio.h>

#define IPC_MSG_BASE_MASK		MVEBU_REGS_BASE_MASK

#define IPC_CH_NUM_OF_MSG		(16)
#define IPC_CH_MSG_IDX			(-1)

unsigned long mv_pm_ipc_msg_base;
unsigned int  mv_pm_ipc_queue_size;

unsigned int msg_sync = 0;
int msg_index = IPC_CH_MSG_IDX;

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
	mv_pm_ipc_msg_base     = ipc_control->msg_base_address | IPC_MSG_BASE_MASK;
	mv_pm_ipc_queue_size   = ipc_control->queue_size;

	return 0;
}

/*******************************************************************************
* mv_pm_ipc_queue_addr_get
*
* DESCRIPTION: Returns the IPC queue address
*******************************************************************************/
unsigned int mv_pm_ipc_queue_addr_get(void)
{
	unsigned int addr;

	inv_dcache_range((uint64_t)&msg_index, sizeof(msg_index));
	msg_index = msg_index + 1;
	if (msg_index >= IPC_CH_NUM_OF_MSG)
		msg_index = 0;

	addr = (unsigned int)(mv_pm_ipc_msg_base + (msg_index * mv_pm_ipc_queue_size));

	flush_dcache_range((uint64_t)&msg_index, sizeof(msg_index));

	return addr;
}

/*******************************************************************************
* mv_pm_ipc_msg_rx
*
* DESCRIPTION: Retrieve message from IPC channel
*******************************************************************************/
int mv_pm_ipc_msg_rx(unsigned int channel_id, struct mss_pm_ipc_msg *msg)
{
	unsigned int addr = mv_pm_ipc_queue_addr_get();
	msg->msg_reply = mmio_read_32(addr + IPC_MSG_REPLY_LOC);

	return 0;
}

/*******************************************************************************
* mv_pm_ipc_msg_tx
*
* DESCRIPTION: Send message via IPC channel
*******************************************************************************/
int mv_pm_ipc_msg_tx(unsigned int channel_id, unsigned int msg_id,
					unsigned int cluster_power_state)
{
	unsigned int addr = mv_pm_ipc_queue_addr_get();

	/* Validate the entry for message placed by the host is free */
	if (mmio_read_32(addr + IPC_MSG_STATE_LOC) == IPC_MSG_FREE) {
		inv_dcache_range((uint64_t)&msg_sync, sizeof(msg_sync));
		msg_sync = msg_sync + 1;
		flush_dcache_range((uint64_t)&msg_sync, sizeof(msg_sync));

		mmio_write_32(addr + IPC_MSG_SYNC_ID_LOC, msg_sync);
		mmio_write_32(addr + IPC_MSG_ID_LOC, msg_id);
		mmio_write_32(addr + IPC_MSG_CPU_ID_LOC, channel_id);
		mmio_write_32(addr + IPC_MSG_POWER_STATE_LOC, cluster_power_state);
		mmio_write_32(addr + IPC_MSG_STATE_LOC, IPC_MSG_OCCUPY);

	} else {
		printf("mv_pm_ipc_msg_tx failed!!!\n");
	}

	return 0;
}

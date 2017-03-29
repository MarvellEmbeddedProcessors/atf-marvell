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

#include <mmio.h>
#include <psci.h>
#include <debug.h>
#include <string.h>

#include <mss_ipc_drv.h>
#include <mss_pm_ipc.h>

/*
 ** SISR is 32 bit interrupt register representing 32 interrupts
 **
 ** +==================+==================+==================+=============+
 ** + Suspend msg int  + Off msg int      + On msg int       + Reserved    +
 ** +==================+==================+==================+=============+
 ** + Bits 31 30 29 28 + Bits 27 26 25 24 + Bits 23 22 21 20 + Bits 19 - 0 +
 ** +==================+==================+==================+=============+
 ** + Core  3  2  1  0 + Core  3  2  1  0 + Core  3  2  1  0 +             +
 ** +==================+==================+==================+=============+
 **
 ** Example: Bit 26 = Off Message Interrupt to core #2
 */
#define MSS_SISR		(MVEBU_REGS_BASE + 0x5800D0)

#define MSS_CPU_SUSPEND_INT_SET_OFFSET		(28)
#define MSS_CPU_OFF_INT_SET_OFFSET		(24)
#define MSS_CPU_ON_INT_SET_OFFSET		(20)

/*******************************************************************************
 * mss_pm_ipc_msg_send
 *
 * DESCRIPTION: create and transmit IPC message
 ******************************************************************************
 */
int mss_pm_ipc_msg_send(unsigned int channel_id,
			const psci_power_state_t *target_state)
{
	/* Transmit IPC message */
#ifndef DISABLE_CLUSTER_LEVEL
	mv_pm_ipc_msg_tx(channel_id, IPC_MSG_TX,
		(unsigned int)target_state->pwr_domain_state[MPIDR_AFFLVL1]);
#else
	mv_pm_ipc_msg_tx(channel_id, IPC_MSG_TX, 0);
#endif

	return 0;
}

/*******************************************************************************
 * mss_pm_ipc_msg_recv
 *
 * DESCRIPTION: wait from reception of IPC message indication,
 *              once received, read the message from IPC channel,
 *              mark IPC channel as Free, and validate reply
 ******************************************************************************
 */
int mss_pm_ipc_msg_recv(unsigned int channel_id, unsigned int msg_id)
{
	struct mss_pm_ipc_msg msg;

	/* Wait for PC message indication */
	do {} while (mv_pm_ipc_msg_validate(channel_id,
					    IPC_MSG_RX, IPC_MSG_OCCUPY) != 0);

	/* Read the message from IPC channel */
	mv_pm_ipc_msg_rx(channel_id, IPC_MSG_RX, &msg);

	/* Mark IPC channel as Free */
	mv_pm_ipc_msg_update(channel_id, IPC_MSG_RX, IPC_MSG_FREE);

	if (msg_id != msg.msg_reply) {
		ERROR("MSS Error, Invalid reply message type %d\n",
		      msg.msg_reply);
		return -1;
	}

	return 0;
}

/*******************************************************************************
 * mss_pm_ipc_on_msg_trigger
 *
 * DESCRIPTION: Trigger IPC ON message interrupt to MSS
 ******************************************************************************
 */
int mss_pm_ipc_on_msg_trigger(unsigned int cpu_id)
{
	mmio_write_32(MSS_SISR, 1 << (MSS_CPU_ON_INT_SET_OFFSET + cpu_id));
	return 0;
}

/*******************************************************************************
 * mss_pm_ipc_msg_trigger
 *
 * DESCRIPTION: Trigger IPC OFF message interrupt to MSS
 ******************************************************************************
 */
int mss_pm_ipc_suspend_msg_trigger(unsigned int cpu_id)
{
	mmio_write_32(MSS_SISR, 1 << (MSS_CPU_SUSPEND_INT_SET_OFFSET + cpu_id));
	return 0;
}

/*******************************************************************************
 * mss_pm_ipc_msg_trigger
 *
 * DESCRIPTION: Trigger IPC SUSPEND message interrupt to MSS
 ******************************************************************************
 */
int mss_pm_ipc_off_msg_trigger(unsigned int cpu_id)
{
	mmio_write_32(MSS_SISR, 1 << (MSS_CPU_OFF_INT_SET_OFFSET + cpu_id));
	return 0;
}

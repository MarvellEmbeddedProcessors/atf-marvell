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
** +======+=============+=============+
** + Bits + 31          + 30 - 00     +
** +======+=============+=============+
** + Desc + MSS Msg Int + Reserved    +
** +======+=============+=============+
*/
#define MSS_SISR		(MVEBU_REGS_BASE + 0x5800D0)
#define MSS_SISTR		(MVEBU_REGS_BASE + 0x5800D8)

#define MSS_MSG_INT_MASK	(0x80000000)

/*******************************************************************************
* mss_pm_ipc_msg_send
*
* DESCRIPTION: create and transmit IPC message
*******************************************************************************/
int mss_pm_ipc_msg_send(unsigned int channel_id, unsigned int msg_id, const psci_power_state_t *target_state)
{
	/* Transmit IPC message */
#ifndef DISABLE_CLUSTER_LEVEL
	mv_pm_ipc_msg_tx(channel_id, msg_id, (unsigned int)target_state->pwr_domain_state[MPIDR_AFFLVL1]);
#else
	mv_pm_ipc_msg_tx(channel_id, msg_id, 0);
#endif

	return 0;
}

/*******************************************************************************
* mss_pm_ipc_msg_trigger
*
* DESCRIPTION: Trigger IPC message interrupt to MSS
*******************************************************************************/
int mss_pm_ipc_msg_trigger(void)
{
	mmio_write_32(MSS_SISR, MSS_MSG_INT_MASK);

	do {
		/* wait while SCP process incoming interrupt */

	} while (mmio_read_32(MSS_SISTR) == MSS_MSG_INT_MASK);

	return 0;
}

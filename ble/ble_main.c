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

#include <arch_helpers.h>
#include <debug.h>
#include <console.h>
#include <mmio.h>
#include <platform_def.h>
#include <plat_marvell.h>
#include <plat_private.h>
#include <marvell_pm.h>
#include <string.h>

#define BR_FLAG_SILENT		0x1
#define SKIP_IMAGE_CODE		0xDEADB002

void mailbox_clean(void)
{
	uintptr_t *mailbox = (void *)PLAT_MARVELL_MAILBOX_BASE;

	memset(mailbox, 0, PLAT_MARVELL_MAILBOX_SIZE);
}

int  __attribute__ ((section(".entry"))) ble_main(int bootrom_flags)
{
	int skip = 0;
	uintptr_t *mailbox = (void *)PLAT_MARVELL_MAILBOX_BASE;

	/*
	 * In some situations, like boot from UART, bootrom will
	 * request to avoid printing to console. in that case don't
	 * initialize the console and prints will be ignored
	 */
	if ((bootrom_flags & BR_FLAG_SILENT) == 0)
		console_init(PLAT_MARVELL_BOOT_UART_BASE,
			     PLAT_MARVELL_BOOT_UART_CLK_IN_HZ,
			     MARVELL_CONSOLE_BAUDRATE);

	NOTICE("Starting binary extension\n");

	/* initiliaze time (for delay functionality) */
	plat_delay_timer_init();

#if PALLADIUM
	NOTICE("Skip DRAM setup in PALLADIUM mode\n");
#else
	ble_plat_setup(&skip);
#endif
	/* if there's skip image request, bootrom will load from the image
	 * saved on the next address of the flash
	 */
	if (skip)
		return SKIP_IMAGE_CODE;

	/*
	 * Check if the mailbox magic number is stored at index MBOX_IDX_MAGIC
	 * and the suspend to RAM magic number at index MBOX_IDX_SUSPEND_MAGIC.
	 * If the above is true, this is the recovery from suspend to RAM state.
	 * In such case the mailbox should remain intact, since it stores the
	 * warm boot jump address to be used by the ATF in BL31.
	 * Othervise the mailbox should be cleaned from a garbage data.
	 */
	if (mailbox[MBOX_IDX_MAGIC] != MVEBU_MAILBOX_MAGIC_NUM ||
	    mailbox[MBOX_IDX_SUSPEND_MAGIC] != MVEBU_MAILBOX_SUSPEND_STATE) {
		NOTICE("Cold boot\n");
		mailbox_clean();
	} else {
		void (*bootrom_exit)(void) =
			(void (*)(void))mailbox[MBOX_IDX_ROM_EXIT_ADDR];

		INFO("Recovery...\n");
		/*
		 * If this is recovery from suspend, two things has to be done:
		 * 1. Define the DRAM region as executable memory for preparing
		 *    jump to ATF
		 * 2. Instead of returning control to the BootROM, invalidate
		 *    and flush caches, and continue execution at address stored
		 *    in the mailbox.
		 * This should be done until the BootROM have a native support
		 * for the system restore flow.
		 */
		ble_prepare_exit();
		bootrom_exit();
	}

	return 0;
}

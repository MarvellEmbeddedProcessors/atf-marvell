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

#include <assert.h>
#include <debug.h>
#include <mmio.h>
#include <arch_helpers.h> /* for cache maintanance operations */
#include <platform_def.h>
#include <delay_timer.h>
#include <apn806_setup.h>
#include <cp110_setup.h>

#include <plat_pm_trace.h>
#include <mss_scp_bootloader.h>
#include <mss_ipc_drv.h>
#include <mss_mem.h>
#include <mss_scp_bl2_format.h>


#define MSS_DMA_SRCBR(base)		(base + 0xC0)
#define MSS_DMA_DSTBR(base)		(base + 0xC4)
#define MSS_DMA_CTRLR(base)		(base + 0xC8)
#define MSS_M3_RSTCR(base)		(base + 0xFC)
#define MSS_AEBR(base)			(base + 0x160)
#define MSS_AIBR(base)			(base + 0x164)

#define MSS_DMA_CTRLR_SIZE_OFFSET	(0)
#define MSS_DMA_CTRLR_REQ_OFFSET	(15)
#define MSS_DMA_CTRLR_REQ_SET		(1)
#define MSS_DMA_CTRLR_ACK_OFFSET	(12)
#define MSS_DMA_CTRLR_ACK_MASK		(0x1)
#define MSS_DMA_CTRLR_ACK_READY		(1)
#define MSS_M3_RSTCR_RST_OFFSET		(0)
#define MSS_M3_RSTCR_RST_OFF		(1)
#define MSS_AEBR_MASK			0xFFF
#define MSS_AIBR_MASK			0xFFF

#define MSS_DMA_TIMEOUT			1000
#define MSS_EXTERNAL_SPACE		0x50000000
#define MSS_EXTERNAL_ACCESS_BIT		28
#define MSS_EXTERNAL_ADDR_MASK		0xfffffff
#define MSS_INTERNAL_ACCESS_BIT		28

#define DMA_SIZE			128

#define MSS_HANDSHAKE_TIMEOUT		50
/* TODO: Fix this */
#define AP_MSS_REG_BASE			(MVEBU_REGS_BASE + 0x580000)
#define CP_MSS_REG_BASE(CP)		(MVEBU_CP_REGS_BASE(CP) + 0x280000)


static int mss_check_image_ready(struct mss_pm_ctrl_block *mss_pm_crtl)
{
	int timeout = MSS_HANDSHAKE_TIMEOUT;

	/* Wait for SCP to signal it's ready */
	while ((mss_pm_crtl->handshake != MSS_ACKNOWLEDGEMENT) &&
						(timeout-- > 0))
		mdelay(1);

	/*
	 * Check that the handshake was completed successfully
	 * Note: since A0 doesn't support MSS load, this check is skipped
	 */
	if ((mss_pm_crtl->handshake != MSS_ACKNOWLEDGEMENT) &&
	    (apn806_rev_id_get() != APN806_REV_ID_A0))
		return -1;

	mss_pm_crtl->handshake = HOST_ACKNOWLEDGEMENT;

	return 0;
}


static int mss_image_load(uint32_t src_addr, uint32_t size, uintptr_t  mss_regs)
{
	uint32_t i, loop_num, timeout;

	/* Check if the img size is not bigger than ID-RAM size of MSS CM3 */
	if (size > MSS_IDRAM_SIZE) {
		ERROR("image is too big to fit into MSS CM3 memory\n");
		return 1;
	}

	NOTICE("Loading MSS image from address 0x%x Size 0x%x to MSS at 0x%x\n",
	       src_addr, size, (uint32_t)mss_regs);
	/* load image to MSS RAM using DMA */
	loop_num = (size / DMA_SIZE) + (((size & (DMA_SIZE - 1)) == 0) ? 0 : 1);

	/* set AXI External and Internal Address Bus extension */
	mmio_write_32(MSS_AEBR(mss_regs), ((src_addr >> MSS_EXTERNAL_ACCESS_BIT)
		      & MSS_AEBR_MASK));
	mmio_write_32(MSS_AIBR(mss_regs), ((mss_regs >> MSS_INTERNAL_ACCESS_BIT)
		      & MSS_AIBR_MASK));

	for (i = 0; i < loop_num; i++) {
		/* write destination and source addresses */
		mmio_write_32(MSS_DMA_SRCBR(mss_regs),
			      MSS_EXTERNAL_SPACE |
			      ((src_addr & MSS_EXTERNAL_ADDR_MASK) +
			      (i * DMA_SIZE)));
		mmio_write_32(MSS_DMA_DSTBR(mss_regs), (i * DMA_SIZE));

		dsb(); /* make sure DMA data is ready before triggering it */

		/* set the DMA control register */
		mmio_write_32(MSS_DMA_CTRLR(mss_regs), ((MSS_DMA_CTRLR_REQ_SET
			      << MSS_DMA_CTRLR_REQ_OFFSET) |
			      (DMA_SIZE << MSS_DMA_CTRLR_SIZE_OFFSET)));

		/* Poll DMA_ACK at MSS_DMACTLR until it is ready */
		timeout = MSS_DMA_TIMEOUT;
		while (timeout) {
			if ((mmio_read_32(MSS_DMA_CTRLR(mss_regs)) >>
			     MSS_DMA_CTRLR_ACK_OFFSET & MSS_DMA_CTRLR_ACK_MASK)
				== MSS_DMA_CTRLR_ACK_READY) {
				break;
			}

			udelay(50);
			timeout--;
		}

		if (timeout == 0) {
			ERROR("\nDMA failed to load MSS image\n");
			return 1;
		}
	}

	/* Release M3 from reset */
	mmio_write_32(MSS_M3_RSTCR(mss_regs), (MSS_M3_RSTCR_RST_OFF <<
		      MSS_M3_RSTCR_RST_OFFSET));

	NOTICE("Done\n");

	return 0;
}

/* Load image to MSS AP and do PM related initialization
 * Note that this routine is different than other CM3 loading routines, because
 * firmware for AP is dedicated for PM and therefore some additional PM
 * initialization is required
 */
static int mss_ap_load_image(uintptr_t single_img, uint32_t image_size)
{
	struct mss_pm_ctrl_block *mss_pm_crtl;
	int ret;

	/* TODO: add PM Control Info from platform */
	mss_pm_crtl = (struct mss_pm_ctrl_block *)MSS_SRAM_PM_CONTROL_BASE;
	mss_pm_crtl->ipc_version                = MV_PM_FW_IPC_VERSION;
	mss_pm_crtl->num_of_clusters            = PLAT_MARVELL_CLUSTER_COUNT;
	mss_pm_crtl->num_of_cores_per_cluster   =
						PLAT_MARVELL_CLUSTER_CORE_COUNT;
	mss_pm_crtl->num_of_cores               = PLAT_MARVELL_CLUSTER_COUNT *
						PLAT_MARVELL_CLUSTER_CORE_COUNT;
	mss_pm_crtl->pm_trace_ctrl_base_address = AP_MSS_ATF_CORE_CTRL_BASE;
	mss_pm_crtl->pm_trace_info_base_address = AP_MSS_ATF_CORE_INFO_BASE;
	mss_pm_crtl->pm_trace_info_core_size    = AP_MSS_ATF_CORE_INFO_SIZE;
	VERBOSE("MSS Control Block = 0x%x\n", MSS_SRAM_PM_CONTROL_BASE);
	VERBOSE("mss_pm_crtl->ipc_version                = 0x%x\n",
		mss_pm_crtl->ipc_version);
	VERBOSE("mss_pm_crtl->num_of_cores               = 0x%x\n",
		mss_pm_crtl->num_of_cores);
	VERBOSE("mss_pm_crtl->num_of_clusters            = 0x%x\n",
		mss_pm_crtl->num_of_clusters);
	VERBOSE("mss_pm_crtl->num_of_cores_per_cluster   = 0x%x\n",
		mss_pm_crtl->num_of_cores_per_cluster);
	VERBOSE("mss_pm_crtl->pm_trace_ctrl_base_address = 0x%x\n",
		mss_pm_crtl->pm_trace_ctrl_base_address);
	VERBOSE("mss_pm_crtl->pm_trace_info_base_address = 0x%x\n",
		mss_pm_crtl->pm_trace_info_base_address);
	VERBOSE("mss_pm_crtl->pm_trace_info_core_size    = 0x%x\n",
		mss_pm_crtl->pm_trace_info_core_size);

	/* TODO: add checksum to image */
	VERBOSE("Send info about the SCP_BL2 image to be transferred to SCP\n");

	ret = mss_image_load(single_img, image_size, AP_MSS_REG_BASE);
	if (ret != 0) {
		ERROR("SCP Image load failed\n");
		return -1;
	}

	/* check that the image was loaded successfully */
	ret = mss_check_image_ready(mss_pm_crtl);
	if (ret != 0)
		NOTICE("SCP Image doesn't contain PM firmware\n");

	return 0;
}

/* Load CM3 image (single_img) to CM3 pointed by cm3_type */
static int load_img_to_cm3(enum cm3_t cm3_type, uintptr_t single_img, uint32_t image_size)
{
	_Bool has_cp1 = 0;
	int ret;

	if (cp110_device_id_get() == MVEBU_80X0_DEV_ID)
		has_cp1 = 1;

	switch (cm3_type) {
	case MSS_AP:
		NOTICE("Load image to AP MSS\n");
		ret = mss_ap_load_image(single_img, image_size);
		if (ret != 0)
			return ret;
		break;
	case MSS_CP0:
		NOTICE("Load image to CP0 MSS\n");
		ret = mss_image_load(single_img, image_size, CP_MSS_REG_BASE(0));
		if (ret != 0) {
			ERROR("SCP Image load failed\n");
			return -1;
		}
		break;
	case MSS_CP1:
		if (!has_cp1) {
			NOTICE("Skipping MSS CP1 related image\n");
			break;
		}
		NOTICE("Load image to CP1 MSS\n");
		ret = mss_image_load(single_img, image_size, CP_MSS_REG_BASE(1));
		if (ret != 0) {
			ERROR("SCP Image load failed\n");
			return -1;
		}
		break;
	case MG_CP0:
		/* TODO: */
		NOTICE("Load image to CP0 MG not supported\n");
		break;
	case MG_CP1:
		if (!has_cp1) {
			NOTICE("Skipping MG CP1 related image\n");
			break;
		}
		/* TODO: */
		NOTICE("Load image to CP1 MG not supported\n");
		break;
	default:
		ERROR("SCP_BL2 wrong img format (cm3_type=%d)\n", cm3_type);
		break;
	}

	return 0;
}

/* The Armada 8K has 5 service CPUs and Armada 7K has 3. Therefore it was
 * required to provide a method for loading firmware to all of the service CPUs.
 * To achieve that, the scp_bl2 image in fact is file containing up to 5
 * concatenated firmwares and this routine splits concatenated image into single
 * images dedicated for appropriate service CPU and then load them.
 */
static int split_and_load_bl2_image(void *image)
{
	file_header_t *file_hdr;
	img_header_t *img_hdr;
	uintptr_t single_img;
	int i;

	file_hdr = (file_header_t *)image;

	if (file_hdr->magic != FILE_MAGIC) {
		ERROR("SCP_BL2 wrong img format\n");
		return -1;
	}

	if (file_hdr->nr_of_imgs > MAX_NR_OF_FILES) {
		ERROR("SCP_BL2 concatenated image contains to many images\n");
		return -1;
	}

	img_hdr = (img_header_t *)((uintptr_t)image + sizeof(file_header_t));
	single_img = (uintptr_t)image + sizeof(file_header_t) +
				    sizeof(img_header_t) * file_hdr->nr_of_imgs;

	NOTICE("SCP_BL2 contains %d concatenated images\n",
							  file_hdr->nr_of_imgs);
	for (i = 0; i < file_hdr->nr_of_imgs; i++) {

		/* Before loading make sanity check on header */
		if (img_hdr->version != HEADER_VERSION) {
			ERROR("Wrong header, img corrupted exiting\n");
			return -1;
		}

		load_img_to_cm3(img_hdr->type, single_img, img_hdr->length);

		/* Prepare offsets for next run */
		single_img += img_hdr->length;
		img_hdr++;
	}

	return 0;
}

int scp_bootloader_transfer(void *image, unsigned int image_size)
{
#ifdef SCP_BL2_BASE
	assert((uintptr_t) image == SCP_BL2_BASE);
#endif

	VERBOSE("Concatenated img size %d\n", image_size);

	if (image_size == 0) {
		ERROR("SCP_BL2 image size can't be 0 (current size = 0x%x)\n",
								    image_size);
		return -1;
	}

	if (split_and_load_bl2_image(image))
		return -1;

	return 0;
}

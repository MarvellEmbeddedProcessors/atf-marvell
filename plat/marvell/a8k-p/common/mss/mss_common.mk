#
# Copyright (C) 2017 Marvell International Ltd.
#
# SPDX-License-Identifier:	BSD-3-Clause
# https://spdx.org/licenses
#

PLAT_MARVELL		:= 	plat/marvell
MSS_SOURCE		:= 	$(PLAT_MARVELL)/a8k-p/common/mss

BL2_SOURCES		+=	$(MSS_SOURCE)/mss_bl2_setup.c		\
				$(MSS_SOURCE)/mss_scp_bootloader.c 	\
				$(PLAT_MARVELL)/common/plat_delay_timer.c \
				drivers/delay_timer/delay_timer.c	\
				$(MARVELL_DRV)				\
				$(PLAT_FAMILY_BASE)/$(PLAT)/board/marvell_plat_config.c

BL31_SOURCES		+=	$(MSS_SOURCE)/mss_ipc_drv.c		\
				$(MSS_SOURCE)/mss_pm_ipc.c

PLAT_INCLUDES           +=      -I$(MSS_SOURCE)

ifneq (${SCP_BL2},)
# Subsystems require a SCP_BL2 image
$(eval $(call FIP_ADD_IMG,SCP_BL2,--scp-fw))

# This define is used to inidcate the SCP image is present
$(eval $(call add_define,SCP_IMAGE))
endif

#
# Copyright (C) 2016 - 2018 Marvell International Ltd.
#
# SPDX-License-Identifier:     BSD-3-Clause
# https://spdx.org/licenses
#

PCI_EP_SUPPORT		:= 1

DOIMAGE_SEC     	:=	tools/doimage/secure/sec_img_7K.cfg

MARVELL_MOCHI_DRV	:=	drivers/marvell/mochi/apn806_setup.c

include plat/marvell/a8k/common/a8k_common.mk

include plat/marvell/common/marvell_common.mk

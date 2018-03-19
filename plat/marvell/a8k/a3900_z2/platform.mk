#
# Copyright (C) 2018 Marvell International Ltd.
#
# SPDX-License-Identifier:	BSD-3-Clause
# https://spdx.org/licenses
#

include plat/marvell/a8k/common/a8k_common.mk

include plat/marvell/common/marvell_common.mk

# A3900 Z2 use AP807 instead AP806, update the source list
BL31_SOURCES	:=	$(filter-out $(MARVELL_DRV_BASE)/mochi/apn806_setup.c, $(BL31_SOURCES))
BL31_SOURCES	+=	$(MARVELL_DRV_BASE)/mochi/ap807_setup.c

BLE_SOURCES	:= $(filter-out $(MARVELL_DRV_BASE)/mochi/apn806_setup.c, $(BLE_SOURCES))
BLE_SOURCES	+= $(MARVELL_DRV_BASE)/mochi/ap807_setup.c
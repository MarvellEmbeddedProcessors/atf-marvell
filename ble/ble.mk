# Copyright (C) 2016, 2018 Marvell International Ltd.
#
# SPDX-License-Identifier:     BSD-3-Clause
# https://spdx.org/licenses

MV_DDR_PATH		?=	drivers/marvell/mv_ddr

MV_DDR_LIB		= 	$(CURDIR)/$(BUILD_PLAT)/ble/mv_ddr_lib.a
BLE_LIBS		= 	$(MV_DDR_LIB)
PLAT_MARVELL		=	plat/marvell

BLE_SOURCES		+= 	ble/ble_main.c				\
				ble/ble_mem.S				\
				drivers/delay_timer/delay_timer.c	\
				$(PLAT_MARVELL)/common/plat_delay_timer.c

PLAT_INCLUDES		+= 	-I$(MV_DDR_PATH) \
				-I$(CURDIR)/include/lib/stdlib \
				-I$(CURDIR)/include/lib/stdlib/sys \
				-Idrivers/marvell

BLE_LINKERFILE		:=	ble/ble.ld.S

FORCE:

$(MV_DDR_LIB): FORCE
	@+make -C $(MV_DDR_PATH) --no-print-directory PLAT_INCLUDES="$(PLAT_INCLUDES)" PLATFORM=$(PLAT) ARCH=AARCH64 OBJ_DIR=$(CURDIR)/$(BUILD_PLAT)/ble

#
# Copyright (c) 2015, ARM Limited and Contributors. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# Neither the name of ARM nor the names of its contributors may be used
# to endorse or promote products derived from this software without specific
# prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

MV_DDR_PATH		?=	drivers/marvell/mv_ddr

MV_DDR_LIB		= 	$(CURDIR)/$(BUILD_PLAT)/ble/mv_ddr_lib.a
BLE_LIBS		= 	$(MV_DDR_LIB)
PLAT_MARVELL		=	plat/marvell

BLE_SOURCES		+= 	ble/ble_main.c				\
				drivers/delay_timer/delay_timer.c	\
				$(PLAT_MARVELL)/common/plat_delay_timer.c

PLAT_INCLUDES		+= 	-I$(MV_DDR_PATH) \
				-I$(CURDIR)/include/lib/stdlib \
				-I$(CURDIR)/include/lib/stdlib/sys \
				-Idrivers/marvell

BLE_LINKERFILE		:=	ble/ble.ld.S

$(MV_DDR_LIB):
	@make -C $(MV_DDR_PATH) --no-print-directory PLAT_INCLUDES="$(PLAT_INCLUDES)" PLATFORM=$(PLAT) ARCH=AARCH64 OBJ_DIR=$(CURDIR)/$(BUILD_PLAT)/ble

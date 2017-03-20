/*
 * Copyright (c) 2015, ARM Limited and Contributors. All rights reserved.
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
 * Neither the name of ARM nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific
 * prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <platform_def.h>

OUTPUT_FORMAT(PLATFORM_LINKER_FORMAT)
OUTPUT_ARCH(PLATFORM_LINKER_ARCH)
ENTRY(ble_main)

MEMORY {
    RAM (rwx): ORIGIN = BLE_BASE, LENGTH = BLE_LIMIT - BLE_BASE
}

SECTIONS
{
    . = BLE_BASE;

    ro . : {
        __RO_START__ = .;
        *ble_main.o(.entry*)
        *(.text*)
        *(.rodata*)
        __RO_END_UNALIGNED__ = .;
        __RO_END__ = .;
    } >RAM

    /*
     * Define a linker symbol to mark start of the RW memory area for this
     * image.
     */
    __RW_START__ = . ;

    .data . : {
        __DATA_START__ = .;
        *(.data*)
        __DATA_END__ = .;
    } >RAM

    .bss : {
        __BSS_START__ = .;
        *(.bss*)
        __BSS_END__ = .;
    } >RAM

   /*
    * Extend the BLE binary to the maximum size allocated for it in platform
    * definition files and prevent overlapping between BLE BSS section and
    * additional extensions that can follow the BLE in flash image preamble.
    * This situation happens for instance when secure extension is added to
    * the image preamble.
    */
   .fill LOADADDR(.bss) + SIZEOF(.bss) : {
       FILL(0xDEADC0DE);
       . = ORIGIN(RAM) + LENGTH(RAM) - 1;
       BYTE(0x00)
   } >RAM

    /*
     * Define a linker symbol to mark end of the RW memory area for this
     * image.
     */
    __RW_END__ = .;
    __BLE_END__ = .;

    __BSS_SIZE__ = SIZEOF(.bss);
}

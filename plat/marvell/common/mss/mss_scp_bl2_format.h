/*
 * ***************************************************************************
 * Copyright (C) 2017 Marvell International Ltd.
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

#ifndef __MSS_SCP_BL2_FORMAT_H
#define __MSS_SCP_BL2_FORMAT_H

#define MAX_NR_OF_FILES	5
#define FILE_MAGIC	0xddd01ff
#define HEADER_VERSION	0x1

#define MSS_IDRAM_SIZE	0x10000 /* 64KB */
#define MG_SRAM_SIZE	0x20000 /* 128KB */

/* Types definitions */
typedef struct file_header {
	uint32_t magic;		/* Magic specific for concatenated file (used for validation) */
	uint32_t nr_of_imgs;	/* Number of images concatenated */
} file_header_t;

/* Types definitions */
enum cm3_t {
	MSS_AP,
	MSS_CP0,
	MSS_CP1,
	MG_CP0,
	MG_CP1,
};

typedef struct img_header {
	uint32_t type;		/* CM3 type, can be one of cm3_t */
	uint32_t length;	/* Image length */
	uint32_t version;	/* For sanity checks and future extended functionality */
} img_header_t;

#endif /* __MSS_SCP_BL2_FORMAT_H */

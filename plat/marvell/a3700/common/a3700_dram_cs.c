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

#include <platform_def.h>
#include <sys_info.h>
#include <mmio.h>
#include <errno.h>
#include <a3700_dram_cs.h>

/* In this file MB is the unit, 1GB is 1024 MB  */
#define GB_2_MB(gb)	(1024 * gb)
/* 1TB is 1024 GB */
#define TB_2_MB(tb)	(1024 * GB_2_MB(tb))

struct dram_cs_addr_len_to_size {
	uint32_t	addr_len_value;
	uint32_t	size_mbytes;
};

/*
 * In the dram cs memory map registers, area_len field holds bits[20:16],
 * its value indicates the length of dram cs memory block as below
 *      0h: 384 MB
 *      1h: 768 MB
 *      2h: 1536 MB
 *      3h: 3 GB
 *      7h: 8 MB
 *      8h: 16 MB
 *      9h: 32 MB
 *      Ah: 64 MB
 *      Bh: 128 MB
 *      Ch: 256 MB
 *      Dh: 512 MB
 *      Eh: 1 GB
 *      Fh: 2 GB
 *      10h: 4 GB
 *      11h: 8 GB
 *      12h: 16 GB
 *      13h: 32 GB
 *      14h: 64 GB
 *      15h: 128 GB
 *      16h: 256 GB
 *      17h: 512 GB
 *      18h: 1 TB
 *      19h: 2 TB
 *      1Ah: 4 TB
 *      Others:Reserved
 */
static struct dram_cs_addr_len_to_size dram_cs_addr_len_to_size_map[] = {
	{0x0,	384		},
	{0x1,	768		},
	{0x2,	1536		},
	{0x3,	GB_2_MB(3)	},
	{0x7,	8		},
	{0x8,	16		},
	{0x9,	32		},
	{0xA,	64		},
	{0xB,	128		},
	{0xC,	256		},
	{0xD,	512		},
	{0xE,	GB_2_MB(1)	},
	{0xF,	GB_2_MB(2)	},
	{0x10,	GB_2_MB(4)	},
	{0x11,	GB_2_MB(8)	},
	{0x12,	GB_2_MB(16)	},
	{0x13,	GB_2_MB(32)	},
	{0x14,	GB_2_MB(64)	},
	{0x15,	GB_2_MB(128)	},
	{0x16,	GB_2_MB(256)	},
	{0x17,	GB_2_MB(512)	},
	{0x18,	TB_2_MB(1)	},
	{0x19,	TB_2_MB(2)	},
	{0x1A,	TB_2_MB(4)	}
};

static int marvell_dram_cs_get_size_by_addr_len(uint32_t addr_len_value, uint32_t *size_mbytes)
{
	int i;

	for (i = 0; i < sizeof(dram_cs_addr_len_to_size_map)/sizeof(struct dram_cs_addr_len_to_size); i++) {
		if (dram_cs_addr_len_to_size_map[i].addr_len_value == addr_len_value) {
			*size_mbytes = dram_cs_addr_len_to_size_map[i].size_mbytes;
			return 0;
		}
	}

	return -EFAULT;
}

/*
 * marvell_get_dram_cs_base_size
 *
 * This function is used to get DRAM CS's memory base address
 * and memory block length from DRAM CS memory map registers,
 * the block length unit is MB.
 *
 * @input:
 *     - cs_num: DRAM CS number
 *
 * @output:
 *     - base_low: DRAM CS's memory base address in low 4 bytes
 *     - base_high: DRAM CS's memory base address in high 4 bytes
 *     - size_mbytes: DRAM CS's memory length in unit of MB
 *
 * @return:  N/A
 */
int marvell_get_dram_cs_base_size(uint32_t cs_num,
				  uint32_t *base_low,
				  uint32_t *base_high,
				  uint32_t *size_mbytes)
{
	uint32_t	cs_mmap_reg, area_len;

	if (cs_num >= MVEBU_MAX_CS_MMAP_NUM)
		return -EPERM;

	cs_mmap_reg = mmio_read_32(MVEBU_CS_MMAP_LOW(cs_num));
	if (!(cs_mmap_reg & MVEBU_CS_MMAP_ENABLE))
		return -ENODEV;

	*base_low = (cs_mmap_reg & MVEBU_CS_MMAP_START_ADDR_LOW_MASK) >> MVEBU_CS_MMAP_START_ADDR_LOW_OFFS;
	*base_high = mmio_read_32(MVEBU_CS_MMAP_HIGH(cs_num));
	area_len = (cs_mmap_reg & MVEBU_CS_MMAP_AREA_LEN_MASK) >> MVEBU_CS_MMAP_AREA_LEN_OFFS;
	if (marvell_dram_cs_get_size_by_addr_len(area_len, size_mbytes))
		return -EFAULT;

	/*
	* For area lengths of 384 MB, 768 MB, 1536 MB, or 3 GB,
	* the start address must be on 1/3 of the area length boundary.
	*/
	if ((area_len <= 3) && (((*base_low >> 20) * 3) != *size_mbytes))
		return -EFAULT;

	return 0;
}


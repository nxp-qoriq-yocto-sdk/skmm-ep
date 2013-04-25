/* Copyright (c) 2013 Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <common.h>
#include <skmm_sram.h>
#include <skmm_uio.h>

static phys_addr_t sram_phys_addr;
static va_addr_t sram_virt_addr;

inline va_addr_t pa_to_va(phys_addr_t addr)
{
	va_addr_t offset;

	offset = (va_addr_t)(addr - sram_phys_addr);

	return sram_virt_addr + offset;
}

inline phys_addr_t va_to_pa(va_addr_t addr)
{
	phys_addr_t offset;

	offset = (phys_addr_t)(addr - sram_virt_addr);

	return sram_phys_addr + offset;
}

va_addr_t *fsl_sram_init(void)
{
	char sram_name[NAME_MAX];
	int sram_fd;
	u32 sram_size;
	int ret;
	u32 map = 0;
	va_addr_t *sram_addr;

	snprintf(sram_name, sizeof(sram_name), "/dev/fsl-sram");
	sram_fd = open(sram_name, O_RDWR);
	if (sram_fd < 0) {
		error(0, -errno, "no %s\n", sram_name);
		return NULL;
	}

	ret = get_map_size(sram_name, map, &sram_size);
	if (ret < 0)
		goto err;

	ret = get_map_addr(sram_name, map, &sram_phys_addr);
	if (ret < 0)
		goto err;

	sram_addr = mmap(0, sram_size, PROT_READ | PROT_WRITE,MAP_SHARED,\
			sram_fd, 0);
	if (sram_addr == NULL) {
		fprintf(stderr, "mmap: %s\n", strerror(errno));
		goto err;
	}
	sram_virt_addr = (va_addr_t)sram_addr;

	return sram_addr;

err:
	close(sram_fd);
	return NULL;
}

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
#include <skmm.h>
#include <skmm_sram.h>
#include <skmm_pci.h>
#include <skmm_uio.h>
#include <skmm_cache.h>

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

static int setup_law_for_plt_sram(phys_addr_t l2sram, u32 *size)
{
	int fd;
	u32 *law;
	void *ccsr;
	int i;
	u32 trgt_id;

	fd = open("/dev/mem", O_RDWR);
	if (fd < 0) {
		error(0, -errno, "fail to open /dev/mem\n");
		return -ENODEV;
	}

	ccsr = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE,
		    MAP_SHARED, fd, get_ccsr_phys_addr());
	law = (u32 *)(ccsr + LAW_OFFSET);

	for (i = 0; i < LAW_MAX_NUM; i++) {
		trgt_id = (read_reg(LAWAR_ADDR(law, i)) >> LAW_TRGT_ID_SHIFT)
				& LAW_TRGT_ID_MASK;

		if (trgt_id != PLATFORM_SRAM_TRGT_ID)
			continue;

		write_reg(LAWAR_ADDR(law, i), 0);

		write_reg(LAWBAR_ADDR(law, i),
				(l2sram + (phys_addr_t)*size) >> LAW_BAR_SHIFT);

		write_reg(LAWAR_ADDR(law, i), LAW_ATR(trgt_id,
					LAW_SIZE_8G));
		read_reg(LAWAR_ADDR(law, i));

		print_debug("set platform sram %llx\n",
				l2sram + (phys_addr_t)*size);
		*size += PLATFORM_SRAM_SIZE;
		munmap(ccsr, 0x1000);
		close(fd);
		return 0;
	}
	return -EPERM;
}

va_addr_t *fsl_mem_init(void)
{
	char sram_name[NAME_MAX];
	int sram_fd;
	u32 sram_size;
	va_addr_t *sram_addr;

#ifdef USE_SRAM_UIO
	int ret;
	u32 map = 0;

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
#else
	sram_fd = open("/dev/mem", O_RDWR);
	if (sram_fd < 0) {
		error(0, -errno, "no %s\n", sram_name);
		return NULL;
	}

	sram_phys_addr = L2_SRAM_ADDR;
	sram_size = L2_SRAM_SIZE;
	fsl_init_l2ctrl(sram_phys_addr, sram_size);
	setup_law_for_plt_sram(sram_phys_addr, &sram_size);

	sram_addr = mmap(0, sram_size, PROT_READ | PROT_WRITE,MAP_SHARED,\
			sram_fd, sram_phys_addr);
	if (sram_addr == NULL) {
		fprintf(stderr, "mmap: %s\n", strerror(errno));
		goto err;
	}
#endif
	print_debug("sram address :%p\n", sram_addr);
	sram_virt_addr = (va_addr_t)sram_addr;

	return sram_addr;

err:
	close(sram_fd);
	return NULL;
}

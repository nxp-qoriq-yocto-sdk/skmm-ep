/* Copyright 2012 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
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

#include <stdio.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/mman.h>
#include <common/access.h>
#include <usdpaa/pci.h>

#define MEM_DEV "/dev/mem"

int main(int argc, char *argv[])
{
	int fd;
	void *access_p;
	struct ccsr_pci *pci;
	unsigned short val16, data;
	uint64_t addr;

	fd = open(MEM_DEV, O_RDWR);
	if (fd < 0) {
		printf("fail to open the /dev/mem\n");
		return 1;
	}

	/*
	 * map the PCI-e controller's ccsr to read PCI configuration space
	 * and setup outbound, offset 0xffe0a000 is mpc8536ds' PCIE1, adjust
	 * it according to your own situation.
	 */
	access_p = mmap(NULL, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, fd,
					0xffe0a000);
	pci = (struct ccsr_pci *)access_p;

	if (!get_ep_msi_addr_data(pci, &data, &addr)) {
		printf("success to call get_ep_msi_addr_data\n");
		printf("addr: %llx    data: %04x\n", addr, data);
	}

	/*
	 * set outbound size 1M for msi interrupt trigger, 0x90000000 is
	 * the base address correponded to PCI-e address space, adjust it
	 * according to your board and the slot linked to RC.
	 */
	setup_atmu_outbound(pci, 0, (addr >> 20) << 20, 0x90000000,
							0x100000);
	munmap(access_p, 0x1000);

	access_p = mmap(NULL, 0x100000, PROT_READ|PROT_WRITE, MAP_SHARED, fd,
					0x90000000);
	/*
	 * powerpc is big-endian, and PCI-e is little-endian, we need to write
	 * the 16bit data to a 32bit register according to little-endian as
	 * hardware.
	 */
	val16 = ((data & 0xff) << 8) | ((data & 0xff00) >> 8);
	iowrite32be(val16 << 16, access_p + (addr & 0xfffff));
	munmap(access_p, 0x100000);
	close(fd);

	return 0;
}

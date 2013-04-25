/* Copyright (c) 2013 Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * Neither the name of Freescale Semiconductor nor the
 * names of its contributors may be used to endorse or promote products
 * derived from this software without specific prior written permission.
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
#include <usdpaa/of.h>
#include <internal/of.h>
#include <skmm_pci.h>
#include <skmm_sram.h>

static phys_addr_t pcie_out_win_base;

static const char * const pci_compatibles[] = {
	"fsl,qoriq-pcie-v2.1",
	"fsl,qoriq-pcie-v2.2",
	"fsl,qoriq-pcie-v2.3",
	"fsl,qoriq-pcie-v3.0"
};

static void pci_process_of_ranges(const struct device_node *dev)
{
	const u32 *ranges;
	int rlen;
	int pna = of_n_addr_cells(dev);
	int np = pna + 5;
	int memno = 0;
	u32 pci_space;
	unsigned long long pci_addr, cpu_addr, pci_next, cpu_next, size;

	/* Get ranges property */
	ranges = of_get_property(dev, "ranges", (size_t *)&rlen);
	if (!ranges)
		return;

	/* Parse it */
	while ((rlen -= np * 4) >= 0) {
		/* Read next ranges element */
		pci_space = ranges[0];
		pci_addr = of_read_number(ranges + 1, 2);
		cpu_addr = of_translate_address(dev, ranges + 3);
		size = of_read_number(ranges + pna + 3, 2);
		ranges += np;

		if (size == 0)
			continue;

		/* Now consume following elements while they are contiguous */
		for (; rlen >= np * sizeof(u32);
		     ranges += np, rlen -= np * 4) {
			if (ranges[0] != pci_space)
				break;
			pci_next = of_read_number(ranges + 1, 2);
			cpu_next = of_translate_address(dev, ranges + 3);
			if (pci_next != pci_addr + size ||
			    cpu_next != cpu_addr + size)
				break;
			size += of_read_number(ranges + pna + 3, 2);
		}

		/* Act based on address space type */
		switch ((pci_space >> 24) & 0x3) {
		case 1:		/* PCI IO space */
			break;
		case 2:		/* PCI Memory space */
		case 3:		/* PCI 64 bits Memory space */
			/* We support only 1 memory ranges */
			if (memno >= 1) {
				pr_info(" --> Skipped (too many) !\n");
				continue;
			}
			pcie_out_win_base = cpu_addr;
			break;
		}
	}
}

static void set_inbound_window(ccsr_pci_t *pci, phys_addr_t addr, u32 size)
{
	pit_t *pi;
	u32 flag = PIWAR_EN | PIWAR_LOCAL | PIWAR_PF |
			PIWAR_READ_SNOOP | PIWAR_WRITE_SNOOP;
	u32 block_rev;

	block_rev = read_reg(&pci->block_rev1);
	if (PEX_IP_BLK_REV_2_2 <= block_rev) {
		pi = &pci->pit[2];	/* 0xDC0 */
	} else {
		pi = &pci->pit[3];	/* 0xDE0 */
	}

	write_reg(&pi->pitar, addr >> 12);
	write_reg(&pi->piwar, flag | size);
}

static void set_outbound_window(ccsr_pci_t *pci, phys_addr_t addr, u32 size)
{
	pot_t *po = &pci->pot[1];
	u32 flag = POWAR_EN | POWAR_MEM_READ | POWAR_MEM_WRITE;

	write_reg(&po->potar, 0);
	write_reg(&po->potear, 0);
	write_reg(&po->powbar, addr >> 12);
	write_reg(&po->powar, flag | size);
}

int fsl_pci_init(u32 pci_idx, phys_addr_t in_win_base)
{
	const struct device_node *pci_node;
	int fd, i;
	int idx = 0, find = 0;
	u64 regs_size;
	phys_addr_t regs_phy;
	const u32 *regs_addr;
	ccsr_pci_t *pci;
	int ret;

	fd = open("/dev/mem", O_RDWR);
	if (fd < 0) {
		error(0, -errno, "fail to open /dev/mem\n");
		return -ENODEV;
	}

	for (i = 0; i < ARRAY_SIZE(pci_compatibles); i++) {
		for_each_compatible_node(pci_node, NULL, pci_compatibles[i]) {
			if (idx == pci_idx) {
				find = 1;
				break;
			}
			idx++;
		}

		if (find)
			break;
	}

	if (!find) {
		error(0, -errno, "Not find the pci controller %d\n", pci_idx);
		ret = -EINVAL;
		goto err;
	}

	regs_addr = of_get_address(pci_node, 0, &regs_size, NULL);
	if (!regs_addr) {
		error(0, 0, "failed to of_get_address\n");
		ret = -EINVAL;
		goto err;
	}

	regs_phy = of_translate_address(pci_node, regs_addr);
	if (!regs_phy) {
		error(0, 0, "failed to of_translate_address(%p)\n", regs_addr);
		ret = -EINVAL;
		goto err;
	}

	pci = mmap(NULL, regs_size, PROT_READ | PROT_WRITE, MAP_SHARED,
			fd, regs_phy);
	if (pci == MAP_FAILED) {
		error(0, -errno, " failed to mmap(0x%llx)\n", regs_phy);
		ret = -EINVAL;
		goto err;
	}

	pci_process_of_ranges(pci_node);

	print_debug("find PCIe controller %d regs_phy is %llx, \
			pci_bar_addr is %llx\n", pci_idx, regs_phy,
			pcie_out_win_base);

	/* Set outbound and inbound */
	set_outbound_window(pci, pcie_out_win_base, POWAR_MEM_8G);
	set_inbound_window(pci, in_win_base, PIWAR_MEM_1M);

	return 0;
err:
	close(fd);
	return ret;
}

phys_addr_t fsl_pci_get_out_win_base(void)
{
	return pcie_out_win_base;
}

int fsl_pci_setup_law(void)
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

		if (trgt_id != SKMM_EP_TRGT_ID)
			continue;

		write_reg(LAWAR_ADDR(law, i), LAW_ATR(trgt_id,
					LAW_SIZE_8G));
		read_reg(LAWAR_ADDR(law, i));
		munmap(ccsr, 0x1000);
		return 0;
	}

	return -EPERM;
}

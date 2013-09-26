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
#include <skmm_cache.h>

int fsl_init_l2ctrl(phys_addr_t sram_addr, u32 sram_size)
{
	int fd;
	void *ccsr;
	struct mpc85xx_l2ctlr *l2ctlr;
	unsigned char ways;
	unsigned int ctl_reg;

	fd = open("/dev/mem", O_RDWR);
	if (fd < 0) {
		error(0, -errno, "fail to open /dev/mem\n");
		return -ENODEV;
	}

	ccsr = mmap(NULL, 0x100000, PROT_READ | PROT_WRITE,
		    MAP_SHARED, fd, get_ccsr_phys_addr());
	l2ctlr = (struct mpc85xx_l2ctlr *)(ccsr + L2_CTRL_OFFSET);

	ways = LOCK_WAYS_FULL * sram_size / L2_CACHE_SIZE;

	/* Write bits[0-17] to srbar0 */
	write_reg(&l2ctlr->srbar0,
		lower_32_bits(sram_addr) & L2SRAM_BAR_MSK_LO18);
	/* Write bits[18-21] to srbare0 */
	write_reg(&l2ctlr->srbarea0,
		upper_32_bits(sram_addr) & L2SRAM_BARE_MSK_HI4);

	ctl_reg = read_reg(&l2ctlr->ctl);
	ctl_reg &= ~L2CR_L2E;
	ctl_reg |= L2CR_L2FI;
	write_reg(&l2ctlr->ctl, ctl_reg);

	ctl_reg = read_reg(&l2ctlr->ctl);
	ctl_reg |= L2CR_L2E | L2CR_L2FI;
	switch (ways) {
	case LOCK_WAYS_EIGHTH:
		ctl_reg |= L2CR_SRAM_EIGHTH;
		break;

	case LOCK_WAYS_TWO_EIGHTH:
		ctl_reg |= L2CR_SRAM_QUART;
		break;

	case LOCK_WAYS_HALF:
		ctl_reg |= L2CR_SRAM_HALF;
		break;

	case LOCK_WAYS_FULL:
	default:
		ctl_reg |= L2CR_SRAM_FULL;
		break;
	}

	write_reg(&l2ctlr->ctl, ctl_reg);

	return 0;
}

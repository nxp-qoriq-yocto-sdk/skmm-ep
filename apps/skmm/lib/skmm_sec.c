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
#include <skmm.h>
#include <skmm_sec.h>
#include <skmm_sram.h>
#include <skmm_uio.h>

static u32 fsl_sec_eng_count;

static void init_rng(sec_engine_t *sec)
{
	u32 x;
	u32 delay;

	struct rng_regs *regs = (struct rng_regs *)sec->rng;

	/*
	 * Put TRNG (entropy generator or True RNG) into program mode
	 *  reset to default values
	 */
	write_reg(&regs->rtmctl, 0x00010000u);

	/*
	 * Set the entropy delay values to the three CAAMs
	 *  (Leave the SAMP_SIZE field set to 2500)
	 */
	switch (sec->id) {
	case 1:
		delay = RNG4_ENT_DLY0;
		break;
	case 2:
		delay = RNG4_ENT_DLY1;
		break;
	case 3:
		delay = RNG4_ENT_DLY2;
		break;
	default:
		delay = RNG4_ENT_DLY0;
		break;

	}

	/* Set entropy delay */
	x = delay * ACTUAL_CLOCK / DEFAULT_CLOCK;
	write_reg(&regs->rtsdctl, ((x << 16) | 0x09C4u));
	/*
	 * Set the Frequency maximum
	 */
	write_reg(&regs->rtfreqmax, (delay * 8));
	/*
	 * Set the Frequency minimum
	 */
	write_reg(&regs->rtfreqmin, (delay / 2));

	/*
	 * Put the TRNG back into run mode
	 * Clear any error that may have occured, just in case
	 *
	 * Note that putting the TRNG back into run mode will
	 * cause it to automatically start generating entropy
	 *
	 * It will take about 30,000,000 clock cycles to generate entropy,
	 * so this should be done early in the boot process.
	 */
	write_reg(&regs->rtmctl, 0x00001000u);
}

/*
 * Function     : sec_reset
 *
 * Arguments    : void
 *
 * Return Value : int
 *
 * Description  : Reset the SEC engine
 *
 */
int sec_reset(ccsr_sec_t *sec)
{
	u32 mcfgr = read_reg(&sec->mcfgr);
	u32 timeout = 100000;

	mcfgr |= MCFGR_SWRST;
	write_reg(&sec->mcfgr, mcfgr);

	mcfgr |= MCFGR_DMA_RST;
	write_reg(&sec->mcfgr, mcfgr);
	do {
		mcfgr = read_reg(&sec->mcfgr);
	} while ((mcfgr & MCFGR_DMA_RST) == MCFGR_DMA_RST && --timeout);

	if (timeout == 0)
		return -1;

	timeout = 100000;
	do {
		mcfgr = read_reg(&sec->mcfgr);
	} while ((mcfgr & MCFGR_SWRST) == MCFGR_SWRST && --timeout);

	if (timeout == 0)
		return -1;

	return 0;
}

static int map_sec(sec_engine_t *sec)
{
	char sec_name[NAME_MAX];
	int sec_fd;
	u32 sec_size = 0;
	u32 map = 0;
	int ret;

	snprintf(sec_name, sizeof(sec_name), "/dev/fsl-sec%d", sec->id - 1);

	sec_fd = open(sec_name, O_RDWR);
	if (sec_fd < 0) {
		error(0, -errno, "no %s\n", sec_name);
		return -ENOENT;
	}

	ret = get_map_size(sec_name, map, &sec_size);
	if (ret < 0)
		return -ENOENT;

	sec->info = (ccsr_sec_t *)mmap(0, sec_size, PROT_READ | \
				PROT_WRITE, MAP_SHARED, sec_fd, 0);
	if (!sec->info) {
		error(0, -errno, "mmap %s failed.\n", sec_name);
		return -ENOENT;
	}
	sec->kek = (kek_regs_t *)((u32)sec->info + SEC_KEK_OFFSET);
	sec->scfg = (u32 *)((u32)sec->info + SEC_SCFG_OFFSET);
	sec->rng = (rng_regs_t *)((u32)sec->info + SEC_RNG_OFFSET);
	sec->rdsta = (u32 *)((u32)sec->info + 0x6c0);
	init_rng(sec);

	return 0;
}

static int map_jr0(sec_engine_t *sec)
{
	char jr_name[NAME_MAX];
	int jr_fd = 0;
	u32 jr_size;
	u32 ring_id = 1;
	u32 map = 0;
	int ret;

	snprintf(jr_name, sizeof(jr_name), "/dev/sec_job_ring%d-%d",
			sec->id - 1, ring_id);

	jr_fd = open(jr_name, O_RDWR);
	if (jr_fd < 0) {
		error(0, -errno, "no %s\n", jr_name);
		return -ENOENT;
	}

	ret = get_map_size(jr_name, map, &jr_size);
	if (ret < 0)
		return -ENOENT;

	sec->jr.regs = (sec_jr_regs_t *)mmap(0, jr_size, \
			PROT_READ | PROT_WRITE, MAP_SHARED, jr_fd, 0);
	if (!sec->jr.regs) {
		error(0, -errno, "mmap %s failed.\n", jr_name);
		return -ENOENT;
	}

	return 0;
}

void fsl_sec_calc_eng_num(void)
{
	fsl_sec_eng_count = SEC_ENG_COUNT;
}

u32 fsl_sec_get_eng_num(void)
{
	return fsl_sec_eng_count;
}

void init_sec_regs_offset(sec_engine_t *sec)
{
	map_sec(sec);
	map_jr0(sec);
}

int fsl_sec_init(sec_engine_t *sec)
{
	u32 jrcfg;
	u32 mcr;
	sec_jr_regs_t *regs = sec->jr.regs;

	phys_addr_t ip_r_base ;
	phys_addr_t op_r_base ;

	ip_r_base = va_to_pa((va_addr_t)sec->jr.i_ring);

	op_r_base = va_to_pa((va_addr_t)sec->jr.o_ring);

	print_debug("--- SEC Initialisaion ---\n");
	print_debug("\tInput ring virtual address   %p\n", sec->jr.i_ring);
	print_debug("\tInput ring physical address  0x%0llx\n", ip_r_base);
	print_debug("\tOutput ring virtual adddress %p\n", sec->jr.o_ring);
	print_debug("\tOutput ring physical address 0x%0llx\n", op_r_base);

	mcr = read_reg(&sec->info->mcfgr);
	write_reg(&sec->info->mcfgr, mcr | 1 << MCFGR_PS_SHIFT);

	sec->jr.size = 128;

	sec_reset(sec->info);
	/* Initialising the jr regs */
	write_reg(&regs->irba_h, ip_r_base >> 32);
	write_reg(&regs->irba_l, (u32)ip_r_base);
	write_reg(&regs->orba_h, op_r_base >> 32);
	write_reg(&regs->orba_l, (u32)op_r_base);
	write_reg(&regs->ors, 128);
	write_reg(&regs->irs, 128);

	write_reg(&(sec->jr.regs->irja),  0);
	write_reg(&(sec->jr.regs->irsa),  0);
	write_reg(&(sec->jr.regs->orsf), 0);
	write_reg(&(sec->jr.regs->orjr), 0);

	sec->jr.head = 0;
	sec->jr.tail = 0;
	jrcfg = read_reg(&regs->jrcfg1);
	/* Disabling interrupt from sec */
	jrcfg = jrcfg | JR_INTMASK;
	write_reg(&regs->jrcfg1, jrcfg);
	print_debug("\n Sec hw eng init done for id : %d\n", sec->id);

	return 0;
}

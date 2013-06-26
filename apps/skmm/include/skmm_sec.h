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

#ifndef	__SKMM_SEC_H__
#define	__SKMM_SEC_H__

#include <common.h>
#include <fcntl.h>
#include <skmm.h>

#ifdef C293PCIE
#define SEC_ENG_COUNT		3
#endif
#ifdef P4080DS
#define SEC_ENG_COUNT		1
#endif

#define	SEC_RNG_OFFSET		0x600
#define	SEC_KEK_OFFSET		0x400
#define	SEC_SCFG_OFFSET		0x0c

#define MCFGR_SWRST		((u32)(1)<<31) /* Software Reset */
#define MCFGR_DMA_RST		((u32)(1)<<28) /* DMA Reset */
#define	MCFGR_PS_SHIFT		16
#define JR_INTMASK		0x00000001

typedef struct sec_jr_regs {
	u32 irba_h;
	u32 irba_l;
	u32 rsvd1;
	u32 irs;
	u32 rsvd2;
	u32 irsa;
	u32 rsvd3;
	u32 irja;
	u32 orba_h;
	u32 orba_l;
	u32 rsvd4;
	u32 ors;
	u32 rsvd5;
	u32 orjr;
	u32 rsvd6;
	u32 orsf;
	u32 rsvd7;
	u32 jrsta;
	u32 rsvd8;
	u32 jrint;
	u32 jrcfg0;
	u32 jrcfg1;
	u32 rsvd9;
	u32 irri;
	u32 rsvd10;
	u32 orwi;
	u32 rsvd11;
	u32 jrcr;
} sec_jr_regs_t;

typedef struct ccsr_sec {
	u32 res0;
	u32 mcfgr;      /* Master CFG Register */
	u8  res1[0x8];
	struct {
		u32 ms; /* Job Ring LIODN Register, MS */
		u32 ls; /* Job Ring LIODN Register, LS */
	} jrliodnr[4];
	u8  res2[0x30];
	struct {
		u32 ms; /* RTIC LIODN Register, MS */
		u32 ls; /* RTIC LIODN Register, LS */
	} rticliodnr[4];
	u8  res3[0x1c];
	u32 decorr;     /* DECO Request Register */
	struct {
		u32 ms; /* DECO LIODN Register, MS */
		u32 ls; /* DECO LIODN Register, LS */
	} decoliodnr[8];
	u8  res4[0x40];
	u32 dar;        /* DECO Avail Register */
	u32 drr;        /* DECO Reset Register */
	u8  res5[0xad0];
	u32 secvid;     /* Version ID Register */
	u8  res6[0x3a4];
	u32 crnr_ms;    /* CHA Revision Number Register, MS */
	u32 crnr_ls;    /* CHA Revision Number Register, LS */
	u32 ctpr_ms;    /* Compile Time Parameters Register, MS */
	u32 ctpr_ls;    /* Compile Time Parameters Register, LS */
	u8  res7[0x10];
	u32 far_ms;     /* Fault Address Register, MS */
	u32 far_ls;     /* Fault Address Register, LS */
	u32 falr;       /* Fault Address LIODN Register */
	u32 fadr;       /* Fault Address Detail Register */
	u8  res8[0x4];
	u32 csta;       /* CAAM Status Register */
	u8  res9[0x8];
	u32 rvid;       /* Run Time Integrity Checking Version ID Reg.*/
	u32 ccbvid;     /* CHA Cluster Block Version ID Register */
	u32 chavid_ms;  /* CHA Version ID Register, MS */
	u32 chavid_ls;  /* CHA Version ID Register, LS */
	u32 chanum_ms;  /* CHA Number Register, MS */
	u32 chanum_ls;  /* CHA Number Register, LS */
	u32 secvid_ms;  /* SEC Version ID Register, MS */
	u32 secvid_ls;  /* SEC Version ID Register, LS */
	u8  res10[0x6020];
	u32 qilcr_ms;   /* Queue Interface LIODN CFG Register, MS */
	u32 qilcr_ls;   /* Queue Interface LIODN CFG Register, LS */
	u8  res11[0x8fd8];
} ccsr_sec_t;

typedef struct sec_ip_ring {
	dma_addr_t  desc;
} sec_ip_ring_t;

struct sec_op_ring {
	dma_addr_t desc;
	u32 status;
}__packed;

typedef struct sec_op_ring sec_op_ring_t;

typedef struct sec_jr {
	sec_jr_regs_t  *regs;
	u32 size;
	u32 head;
	u32 tail;
	u32 enq_cnt;
	u32 deq_cnt;
	u32 id;
	sec_ip_ring_t			*i_ring;
	struct sec_op_ring		*o_ring;
} sec_jr_t;

#define	DEFAULT_CLOCK	400
#define	ACTUAL_CLOCK	400
#define	RNG4_ENT_DLY0	2400
#define	RNG4_ENT_DLY1	2600
#define	RNG4_ENT_DLY2	2800

typedef struct rng_regs {
	u32 rtmctl;
	u32 rtscmisc;
	u32 rtpkrrng;
	u32 rtpkrmax;
	u32 rtsdctl;
	u32 rtsblim;
	u32 rtfreqmin;
	u32 rtfreqmax;
	u32 rtscml;    /* was rtfreqcnt; which is the same as rtfreqmax */
} rng_regs_t;

typedef struct kek_regs {
	u32 jdkek;
	u32 tdkek;
	u32 tdsk;
} kek_regs_t;

/* The container data structure be in platform SRAM */
typedef struct sec_engine {
	u8 id;
	u8 dequeue_resp;
	u32 tot_req_cnt;        /* TOTAL JOBS ENQUEUED IN SEC */
	u32 tot_resp_cnt;       /* TOTAL JOBS DEQUEUED IN SEC */

	ccsr_sec_t *info;
	rng_regs_t *rng;
	kek_regs_t *kek;
	u32        *scfg;
	u32        *rdsta;
	sec_jr_t   jr;

	struct sec_engine *next;
} sec_engine_t;

int sec_reset(ccsr_sec_t *sec);
void init_sec_regs_offset(sec_engine_t *sec);
int fsl_sec_init(sec_engine_t *sec);
void fsl_sec_calc_eng_num(void);
u32 fsl_sec_get_eng_num(void);

#endif /* __SKMM_SEC_H__ */

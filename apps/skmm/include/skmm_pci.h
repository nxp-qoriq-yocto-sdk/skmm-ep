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
#ifndef	__SKMM_PCI_H__
#define	__SKMM_PCI_H__

#include "common.h"

#ifdef C293PCIE
#define SKMM_EP_PCIe_IDX	0x0
#endif
#ifdef P4080DS
#define SKMM_EP_PCIe_IDX	0x0 /* PCIe1 as EP */
#endif

/*
 * PCIe setting for EP
 */
#define PEX_IP_BLK_REV_2_2	0x02080202
#define PEX_IP_BLK_REV_2_3	0x02080203
#define PEX_IP_BLK_REV_3_0	0x02080300

#define POWAR_EN		0x80000000
#define POWAR_MEM_READ		0x00040000
#define POWAR_MEM_WRITE		0x00004000
#define POWAR_MEM_8G		0x00000020

#define PIWAR_EN		0x80000000
#define PIWAR_PF		0x20000000
#define PIWAR_LOCAL		0x00f00000
#define PIWAR_READ_SNOOP	0x00050000
#define PIWAR_WRITE_SNOOP	0x00005000
#define PIWAR_MEM_1M		0x00000011

/*
 * LAW setting for EP
 */
#define LAW_SIZE_8G		0x20
#define LAW_ATR(trgt_id, size)	((1 << 31) | (trgt_id << 20) | size)

#ifdef C293PCIE
#define SKMM_EP_TRGT_ID		0x2

#define LAW_MAX_NUM		12
#define LAW_OFFSET		0xc08
#define LAW_TRGT_ID_SHIFT	20
#define LAW_TRGT_ID_MASK	0x1f
#define LAWAR_ADDR(base, x) 	((u32 *)base + 8 * x + 2)
#define LAWBAR_ADDR(base, x) 	((u32 *)base + 8 * x)
#endif

#ifdef P4080DS
#define SKMM_EP_TRGT_ID		0x0 /* PCIe1 */

#define LAW_MAX_NUM		32
#define LAW_OFFSET		0xc00
#define LAW_TRGT_ID_SHIFT	20
#define LAW_TRGT_ID_MASK	0xff
#define LAWAR_ADDR(base, x) 	((u32 *)base + 4 * x + 2)
#define LAWBARL_ADDR(base, x) 	((u32 *)base + 4 * x + 1)
#define LAWBARH_ADDR(base, x) 	((u32 *)base + 4 * x)
#endif

/*
 * PCI Translation Registers
 */
typedef struct pci_outbound_window {
	u32	potar;		/* 0x00 - Address */
	u32	potear;		/* 0x04 - Address Extended */
	u32	powbar;		/* 0x08 - Window Base Address */
	u32	res1;
	u32	powar;		/* 0x10 - Window Attributes */
	u32	res2[3];
} pot_t;

typedef struct pci_inbound_window {
	u32	pitar;		/* 0x00 - Address */
	u32	res1;
	u32	piwbar;		/* 0x08 - Window Base Address */
	u32	piwbear;	/* 0x0c - Window Base Address Extended */
	u32	piwar;		/* 0x10 - Window Attributes */
	u32	res2[3];
} pit_t;

/* PCI/PCI Express Registers */
typedef struct ccsr_pci {
	u32	cfg_addr;	/* 0x000 - PCI Configuration Address Register */
	u32	cfg_data;	/* 0x004 - PCI Configuration Data Register */
	u32	int_ack;	/* 0x008 - PCI Interrupt Acknowledge Register */
	u32	out_comp_to;	/* 0x00C - PCI Outbound Completion Timeout Register */
	u32	out_conf_to;	/* 0x010 - PCI Configuration Timeout Register */
	u32	config;		/* 0x014 - PCIE CONFIG Register */
	u32	int_status;	/* 0x018 - PCIE interrupt status register */
	char	res2[4];
	u32	pme_msg_det;	/* 0x020 - PCIE PME & message detect register */
	u32	pme_msg_dis;	/* 0x024 - PCIE PME & message disable register */
	u32	pme_msg_int_en;	/* 0x028 - PCIE PME & message interrupt enable register */
	u32	pm_command;	/* 0x02c - PCIE PM Command register */
	char	res4[3016];	/*     (- #xbf8	 #x30)3016 */
	u32	block_rev1;	/* 0xbf8 - PCIE Block Revision register 1 */
	u32	block_rev2;	/* 0xbfc - PCIE Block Revision register 2 */

	pot_t	pot[5];		/* 0xc00 - 0xc9f Outbound ATMU's 0, 1, 2, 3, and 4 */
	u32	res5[24];
	pit_t	pmit;		/* 0xd00 - 0xd9c Inbound ATMU's MSI */
	u32	res6[24];
	pit_t	pit[4];		/* 0xd80 - 0xdff Inbound ATMU's 3, 2, 1 and 0 */
	u32	pedr;		/* 0xe00 - PCI Error Detect Register */
	u32	pecdr;		/* 0xe04 - PCI Error Capture Disable Register */
	u32	peer;		/* 0xe08 - PCI Error Interrupt Enable Register */
	u32	peattrcr;	/* 0xe0c - PCI Error Attributes Capture Register */
	u32	peaddrcr;	/* 0xe10 - PCI Error Address Capture Register */
	u32	peextaddrcr;	/* 0xe14 - PCI	Error Extended Address Capture Register */
	u32	pedlcr;		/* 0xe18 - PCI Error Data Low Capture Register */
	u32	pedhcr;		/* 0xe1c - PCI Error Error Data High Capture Register */
	u32	gas_timr;	/* 0xe20 - PCI Gasket Timer Register */
	char	res22[4];
	u32	perr_cap0;	/* 0xe28 - PCIE Error Capture Register 0 */
	u32	perr_cap1;	/* 0xe2c - PCIE Error Capture Register 1 */
	u32	perr_cap2;	/* 0xe30 - PCIE Error Capture Register 2 */
	u32	perr_cap3;	/* 0xe34 - PCIE Error Capture Register 3 */
	char	res23[200];
	u32	pdb_stat;	/* 0xf00 - PCIE Debug Status */
	char	res24[16];
	u32	pex_csr0;	/* 0xf14 - PEX Control/Status register 0*/
	u32	pex_csr1;	/* 0xf18 - PEX Control/Status register 1*/
	char	res25[228];
} ccsr_pci_t;

int fsl_pci_init(u32 pci_idx, phys_addr_t in_win_base);
int fsl_pci_setup_law(void);
phys_addr_t fsl_pci_get_out_win_base(void);

#endif /* __SKMM_PCI_H__ */

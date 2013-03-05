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

#include <stdint.h>
#include <errno.h>

#define PCI_ENABLE				0x80000000
#define PCI_ATMU_MEM_FLAG		0x80044000
#define PCI_STATUS				0x06
#define PCI_STATUS_CAP_LIST		0x10
#define PCI_CAPABILITY_LIST		0x34
#define PCI_CB_CAPABILITY_LIST	0x14
#define PCI_HEADER_TYPE			0x0e
#define PCI_HEADER_TYPE_NORMAL	0
#define PCI_HEADER_TYPE_BRIDGE	1
#define PCI_HEADER_TYPE_CARDBUS	2
#define PCI_MAX_CAP_NUM			48
#define PCI_CAP_ID_MSI			0x05
#define PCI_MSI_FLAGS			2
#define PCI_MSI_FLAGS_ENABLE	0x01

/* PCI/PCI Express outbound window reg */
struct pci_outbound_window_regs {
	uint32_t	potar;	/* 0x.0
				   * - Outbound translation address register
				   */
	uint32_t	potear;	/* 0x.4
				   * - Outbound translation extended
				   * - address register
				   */
	uint32_t	powbar;	/* 0x.8
				   * - Outbound window base address register
				   */
	uint8_t	res_0c[4];	/* 0x.c */
	uint32_t	powar;	/* 0x.10
				   * - Outbound window attributes register
				   */
	uint8_t	res_14[12];	/* 0x.14 */
};

/* PCI/PCI Express inbound window reg */
struct pci_inbound_window_regs {
	uint32_t	pitar;	/* 0x.0
				   * - Inbound translation address register
				   */
	uint8_t	res_04[4];	/* 0x.4 */
	uint32_t	piwbar;	/* 0x.8
				   * - Inbound window base address register
				   */
	uint32_t	piwbear;	/* 0x.c
				   * - Inbound window base extended address
				   * - register
				   */
	uint32_t	piwar;	/* 0x.10
				   * - Inbound window attributes register
				   */
	uint8_t	res_14[12];	/* 0x.14 */
};

/* PCI/PCI Express IO block registers for 85xx/86xx */
struct ccsr_pci {
	uint32_t	config_addr;	/* 0x.000
				   * - PCI/PCIE Configuration Address Register
				   */
	uint32_t	config_data;	/* 0x.004
				   * - PCI/PCIE Configuration Data Register
				   */
	uint32_t	int_ack;	/* 0x.008
				   * - PCI Interrupt Acknowledge Register
				   */
	uint32_t	pex_otb_cpl_tor;	/* 0x.00c
				   * - PCIE Outbound completion timeout register
				   */
	uint32_t	pex_conf_tor;	/* 0x.010
				   * - PCIE configuration timeout register
				   */
	uint32_t	pex_config;	/* 0x.014
				   * - PCIE CONFIG Register
				   */
	uint32_t	pex_int_status;	/* 0x.018
				   * - PCIE interrupt status
				   */
	uint8_t	res_01c[4];			/* 0x.01c */
	uint32_t	pex_pme_mes_dr;	/* 0x.020
				   * - PCIE PME and message detect register
				   */
	uint32_t	pex_pme_mes_disr;	/* 0x.024
				   * - PCIE PME and message disable register
				   */
	uint32_t	pex_pme_mes_ier;	/* 0x.028
				   * - PCIE PME and message interrupt
				   * - enable register
				   */
	uint32_t	pex_pmcr;	/* 0x.02c
				   * - PCIE power management command register
				   */
	uint8_t	res_030[3024];	/* 0x.030 */

/* PCI/PCI Express outbound window 0-4
 * Window 0 is the default window and is the only window enabled upon reset.
 * The default outbound register set is used when a transaction misses
 * in all of the other outbound windows.
 */
	struct	pci_outbound_window_regs pow[5];
	uint8_t	res_ca0[96];	/* 0x.ca0 */
	struct	pci_inbound_window_regs pmit;	/* 0xd00 - 0xd9c
			   * - Inbound MSI
			   */
	uint8_t	res_d20[96];	/* 0x.d20 */
/* PCI/PCI Express inbound window 3-0
 * inbound window 1 supports only a 32-bit base address and does not
 * define an inbound window base extended address register.
 */
	struct pci_inbound_window_regs piw[4];
/* Merge PCI/PCI Express error management registers */
	uint32_t	pex_err_dr;	  /* 0x.e00
				   * - PCI/PCIE error detect register
				   */
	uint32_t	pex_err_cap_dr;	  /* 0x.e04
				   * - PCI error capture disabled register
				   * - PCIE has no this register
				   */
	uint32_t	pex_err_en;	  /* 0x.e08
				   * - PCI/PCIE error interrupt enable register
				   */
	uint32_t	pex_err_attrib;	  /* 0x.e0c
				   * - PCI error attributes capture register
				   * - PCIE has no this register
				   */
	uint32_t	pex_err_disr;	  /* 0x.e10
				   * - PCI error address capture register
				   * - PCIE error disable register
				   */
	uint32_t	pex_err_ext_addr; /* 0x.e14
				   * - PCI error extended addr capture register
				   * - PCIE has no this register
				   */
	uint32_t	pex_err_dl;	  /* 0x.e18
				   * - PCI error data low capture register
				   * - PCIE has no this register
				   */
	uint32_t	pex_err_dh;	  /* 0x.e1c
				   * - PCI error data high capture register
				   * - PCIE has no this register
				   */
	uint32_t	pex_err_cap_stat; /* 0x.e20
				   * - PCI gasket timer register
				   * - PCIE error capture status register
				   */
	uint8_t		res_e24[4];	/* 0x.e24 */
	uint32_t	pex_err_cap_r0;	/* 0x.e28
				   * - PCIE error capture register 0
				   */
	uint32_t	pex_err_cap_r1;	/* 0x.e2c
				   * - PCIE error capture register 0
				   */
	uint32_t	pex_err_cap_r2;	/* 0x.e30
				   * - PCIE error capture register 0
				   */
	uint32_t	pex_err_cap_r3;	/* 0x.e34
				   * - PCIE error capture register 0
				   */
};

/**
 * pci_bus_read_config_byte() - Read a byte in pci configuration space of
 *		specified device at the offset.
 *
 * @pci:	the memory-mapped address of PCI-e/PCI controller that
 *		the device belongs to
 * @bus_no:	the device's bus number
 * @devfn:	the device's combination of device number and function number
 * @offset:	the offset you want to read in pci configuration space
 * @value:	the address to store the value you want to read
 */
void pci_bus_read_config_byte(struct ccsr_pci *pci, uint8_t bus_no,
			uint8_t devfn, uint16_t offset, uint8_t *value);

/**
 * pci_bus_read_config_word() - Read a word(16 bit) in pci configuration space
 *		of specified device at the offset.
 *
 * @pci:	the memory-mapped address of PCI-e/PCI controller that
 *		the device belongs to
 * @bus_no:	the device's bus number
 * @devfn:	the device's combination of device number and function number
 * @offset:	the offset you want to read in pci configuration space
 * @value:	the address to store the value you want to read
 */
void pci_bus_read_config_word(struct ccsr_pci *pci, uint8_t bus_no,
			uint8_t devfn, uint16_t offset, uint16_t *value);

/**
 * pci_bus_read_config_dword() - Read a dword(32 bit) in pci configuration space
 *		of specified device at the offset.
 *
 * @pci:	the memory-mapped address of PCI-e/PCI controller that
 *		the device belongs to
 * @bus_no:	the device's bus number
 * @devfn:	the device's combination of device number and function number
 * @offset:	the offset you want to read in pci configuration space
 * @value:	the address to store the value you want to read
 */
void pci_bus_read_config_dword(struct ccsr_pci *pci, uint8_t bus_no,
			uint8_t devfn, uint16_t offset, uint32_t *value);

/**
 * pci_bus_write_config_byte() - Write a byte to pci configuration space
 *		of specified device at the offset.
 *
 * @pci:	the memory-mapped address of PCI-e/PCI controller that
 *		the device belongs to
 * @bus_no:	the device's bus number
 * @devfn:	the device's combination of device number and function number
 * @offset:	the offset of pci configuration space you want to write to
 * @value:	the data you want to write
 */
void pci_bus_write_config_byte(struct ccsr_pci *pci, uint8_t bus_no,
			uint8_t devfn, uint16_t offset, uint8_t value);

/**
 * pci_bus_write_config_word() - Write a word(16 bit) to pci configuration space
 *		of specified device at the offset.
 *
 * @pci:	the memory-mapped address of PCI-e/PCI controller that
 *		the device belongs to
 * @bus_no:	the device's bus number
 * @devfn:	the device's combination of device number and function number
 * @offset:	the offset of pci configuration space you want to write to
 * @value:	the data you want to write
 */
void pci_bus_write_config_word(struct ccsr_pci *pci, uint8_t bus_no,
			uint8_t devfn, uint16_t offset, uint16_t value);

/**
 * pci_bus_write_config_dword() - Write a dword(32 bit) to pci configuration
 *		space of specified device at the offset.
 *
 * @pci:	the memory-mapped address of PCI-e/PCI controller that
 *		the device belongs to
 * @bus_no:	the device's bus number
 * @devfn:	the device's combination of device number and function number
 * @offset:	the offset of pci configuration space you want to write to
 * @value:	the data you want to write
 */
void pci_bus_write_config_dword(struct ccsr_pci *pci, uint8_t bus_no,
			uint8_t devfn, uint16_t offset, uint32_t value);

/**
 * setup_atmu_outbound() - Configure a outbound
 *
 * @pci:	the memory-mapped address of PCI-e controller
 * @index:	the outbound window you want to setup
 * @pci_addr:	the trans addr of the outbound
 * @phy_addr:	the base addr of the outbound
 * @size:	the size of the outbound window
 */
void setup_atmu_outbound(struct ccsr_pci *pci, uint8_t index, uint64_t pci_addr,
			uint64_t phys_addr, uint64_t size);

/**
 * setup_atmu_inbound() - Configure a outbound
 *
 * @pci:	the memory-mapped address of PCI-e controller
 * @index:	the inbound window you want to setup
 * @pci_addr:	the base addr of the outbound
 * @phy_addr:	the trans addr of the outbound
 * @size:	the size of the outbound window
 */
void setup_atmu_inbound(struct ccsr_pci *pci, uint8_t index, uint64_t pci_addr,
			uint64_t phys_addr, uint64_t size);

/**
 * pci_find_capability() - To find the capability you specified
 *
 * @pci:	the memory-mapped address of PCI-e controller
 * @bus_no:	the device's bus number
 * @devfn:	the device's combination of device number and function number
 * @cap:	the capability you want to find
 * @return:	0 if don't find the capability, otherwise return the offset of
 *		capability in pci configuration space
 */
int pci_find_capability(struct ccsr_pci *pci, uint8_t bus_no, uint8_t devfn,
			uint8_t cap);

/**
 * get_ep_msi_addr_data() - To get ep's msi interrupt data and address from
 *		its PCI configuration space
 *
 * @pci:	the memory-mapped address of PCI-e controller acted as ep mode
 * @data:	the address to store the value which can trigger msi interrupt
 *		on RC
 * @addr:	the address to store the value, a pci bus address where the data
 *		should be written to for triggering msi interrupt on RC
 * @return:	0 success to get the data and addr, other value indicates
 *		failure
 */
int get_ep_msi_addr_data(struct ccsr_pci *pci, uint16_t *data, uint64_t *addr);

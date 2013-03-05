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

#include <common/access.h>
#include <common/endian.h>
#include <usdpaa/pci.h>

#define PCI_OP_READ(size, type, len) \
void pci_bus_read_config_##size \
	(struct ccsr_pci *pci, uint8_t bus_no, uint8_t devfn, \
		uint16_t pos, type * value) \
{ \
	uint32_t data = 0; \
	indirect_read_config(pci, bus_no, devfn, pos, len, &data); \
	*value = (type)data; \
}

#define PCI_OP_WRITE(size, type, len) \
void pci_bus_write_config_##size\
	(struct ccsr_pci *pci, uint8_t bus_no, uint8_t devfn, \
			uint16_t pos, type value) \
{ \
	indirect_write_config(pci, bus_no, devfn, pos, len, value);	\
}

static void
indirect_read_config(struct ccsr_pci *pci, uint8_t bus_no, uint8_t devfn,
		uint16_t offset, uint8_t len, uint32_t *val)
{
	void *cfg_addr = &pci->config_addr;
	void *cfg_data = (void *)&pci->config_data + (offset & 3);
	uint32_t reg = ((offset & 0xf00) << 16) | (offset & 0xfc);

	iowrite32be(PCI_ENABLE | (bus_no << 16) | (devfn << 8) | reg, cfg_addr);
	switch (len) {
	case 1:
		*val = ioread8be(cfg_data);
		break;
	case 2:
		*val = ioread16be(cfg_data);
		*val = SWAP_UINT16(*val);
		break;
	case 4:
		*val = ioread32be(cfg_data);
		*val = SWAP_UINT32(*val);
		break;
	}
}

static void
indirect_write_config(struct ccsr_pci *pci, uint8_t bus_no, uint8_t devfn,
		uint16_t offset, uint8_t len, uint32_t val)
{
	void *cfg_addr = &pci->config_addr;
	void *cfg_data = (void *)&pci->config_data + (offset & 3);
	uint32_t reg = ((offset & 0xf00) << 16) | (offset & 0xfc);

	iowrite32be(PCI_ENABLE | (bus_no << 16) | (devfn << 8) | reg, cfg_addr);
	switch (len) {
	case 1:
		iowrite8be(val, cfg_data);
		break;
	case 2:
		val = SWAP_UINT16(val);
		iowrite16be(val, cfg_data);
		break;
	case 4:
		val = SWAP_UINT32(val);
		iowrite32be(val, cfg_data);
		break;
	}
}

PCI_OP_READ(byte, uint8_t, 1)
PCI_OP_READ(word, uint16_t, 2)
PCI_OP_READ(dword, uint32_t, 4)
PCI_OP_WRITE(byte, uint8_t, 1)
PCI_OP_WRITE(word, uint16_t, 2)
PCI_OP_WRITE(dword, uint32_t, 4)

static int __log2(uint64_t val)
{
	int i = 0;

	while (val > 1) {
		val = val >> 1;
		i++;
	}

	return i;
}

void setup_atmu_outbound(struct ccsr_pci *pci, uint8_t index, uint64_t pci_addr,
		uint64_t phys_addr, uint64_t size)
{
	iowrite32be(pci_addr >> 12, &pci->pow[index].potar);
	iowrite32be(pci_addr >> 44, &pci->pow[index].potear);
	iowrite32be(phys_addr >> 12, &pci->pow[index].powbar);
	iowrite32be(PCI_ATMU_MEM_FLAG | (__log2(size) - 1),
		&pci->pow[index].powar);
}

void setup_atmu_inbound(struct ccsr_pci *pci, uint8_t index, uint64_t pci_addr,
		uint64_t phys_addr, uint64_t size)
{
	iowrite32be(phys_addr >> 12, &pci->piw[index].pitar);
	iowrite32be(pci_addr >> 12, &pci->piw[index].piwbar);
	iowrite32be(phys_addr >> 44, &pci->piw[index].piwbear);
	iowrite32be(PCI_ATMU_MEM_FLAG | (__log2(size) - 1),
		&pci->piw[index].piwar);
}

static int pci_bus_find_cap_start(struct ccsr_pci *pci, uint8_t bus_no,
				uint8_t devfn, uint8_t hdr_type)
{
	uint16_t status;

	pci_bus_read_config_word(pci, bus_no, devfn, PCI_STATUS, &status);
	if (!(status & PCI_STATUS_CAP_LIST))
		return 0;

	switch (hdr_type) {
	case PCI_HEADER_TYPE_NORMAL:
	case PCI_HEADER_TYPE_BRIDGE:
		return PCI_CAPABILITY_LIST;
	case PCI_HEADER_TYPE_CARDBUS:
		return PCI_CB_CAPABILITY_LIST;
	default:
		return 0;
	}

	return 0;
}

static int pci_find_next_cap(struct ccsr_pci *pci, uint8_t bus_no,
				uint8_t devfn, uint8_t pos, uint8_t cap)
{
	int num = PCI_MAX_CAP_NUM;
	uint8_t id;

	while (num--) {
		pci_bus_read_config_byte(pci, bus_no, devfn, pos, &pos);
		if (pos < 0x40)
			break;
		pos &= ~3;
		pci_bus_read_config_byte(pci, bus_no, devfn, pos, &id);
		if (id == 0xff)
			break;
		if (id == cap)
			return pos;
		pos += 1;
	}

	return 0;
}

int pci_find_capability(struct ccsr_pci *pci, uint8_t bus_no, uint8_t devfn,
		uint8_t cap)
{
	uint8_t hdr_type, pos = 0;

	pci_bus_read_config_byte(pci, bus_no, devfn, PCI_HEADER_TYPE,
		&hdr_type);
	hdr_type &= 0x7f;

	pos = pci_bus_find_cap_start(pci, bus_no, devfn, hdr_type);
	if (pos)
		pos = pci_find_next_cap(pci, bus_no, devfn, pos, cap);

	return pos;
}

int get_ep_msi_addr_data(struct ccsr_pci *pci, uint16_t *data, uint64_t *addr)
{
	int pos, is_64 = 0;
	uint16_t control;
	uint32_t value;

	pos = pci_find_capability(pci, 0, 0, PCI_CAP_ID_MSI);
	if (!pos)
		return -EINVAL;

	pci_bus_read_config_word(pci, 0, 0, pos + PCI_MSI_FLAGS, &control);
	if (control & 0x80)
		is_64 = 1;

	if (is_64) {
		pci_bus_read_config_dword(pci, 0, 0, pos + 8, &value);
		*addr = value;
		pci_bus_read_config_dword(pci, 0, 0, pos + 4, &value);
		*addr = (*addr << 32) | value;
		pci_bus_read_config_word(pci, 0, 0, pos + 12, data);
	} else {
		pci_bus_read_config_dword(pci, 0, 0, pos + 4, &value);
		*addr = value;
		pci_bus_read_config_word(pci, 0, 0, pos + 8, data);
	}
	return 0;
}

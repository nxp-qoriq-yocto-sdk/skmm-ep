/*
 * Copyright 2013 Freescale Semiconductor, Inc.
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

#ifndef _PCI_EP_VFIO_H
#define _PCI_EP_VFIO_H

#include <stdint.h>
#include <usdpaa/vfio.h>
#include <usdpaa/fsl_pci_ep_vfio.h>

struct vfio_us_container {
	int fd;
	struct vfio_us_group *group;
};

struct vfio_us_group {
	int fd;
	int groupid;
	struct vfio_us_container *container;
};

struct pci_ep {
	int fd;
	char name[32];
	struct vfio_us_group *group;
	struct pci_ep_info info;
	struct pci_ep_win iw[4];
	struct pci_ep_win ow[5];
	struct pci_ep_win reg;
	struct pci_ep_win config;
};

int vfio_pci_ep_read_config(struct pci_ep *ep, void *buf, int len, int addr);

int vfio_pci_ep_read_reg(struct pci_ep *ep, void *buf, int len, int addr);
int vfio_pci_ep_write_reg(struct pci_ep *ep, void *buf, int len, int addr);
void *vfio_pci_ep_map_win(struct pci_ep *ep, struct pci_ep_win *win,
				off_t off, size_t len);

int vfio_pci_ep_get_win(struct pci_ep *ep, struct pci_ep_win *win);
int vfio_pci_ep_set_win(struct pci_ep *ep, struct pci_ep_win *win);

void vfio_pci_ep_info(struct pci_ep *ep);

struct pci_ep *vfio_pci_ep_open(int controller, int pf, int vf);
void vfio_pci_ep_close(struct pci_ep *ep);

#endif /* _PCI_EP_VFIO_H */

# Copyright 2012 Freescale Semiconductor, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#	notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#	notice, this list of conditions and the following disclaimer in the
#	documentation and/or other materials provided with the distribution.
#     * Neither the name of Freescale Semiconductor nor the
#	names of its contributors may be used to endorse or promote products
#	derived from this software without specific prior written permission.
#
#
# ALTERNATIVELY, this software may be distributed under the terms of the
# GNU General Public License ("GPL") as published by the Free Software
# Foundation, either version 2 of that License or (at your option) any
# later version.
#
# THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

lib_LIBRARIES = skmm_common skmm_pci skmm_sec skmm_mem skmm_sec_blob \
		skmm_memmgr skmm_uio

AM_CFLAGS := $(LIBEDIT_CFLAGS) -I$(TOP_LEVEL)/apps/skmm/include

dist_DATA = 80-sec-uio.rules
dist_DATA = 81-sram-uio.rules

skmm_common_SOURCES := common.c
skmm_pci_SOURCES := skmm_pci.c
skmm_sec_SOURCES := skmm_sec.c
ifdef SKMM_SRAM
skmm_mem_SOURCES := skmm_sram.c
else
skmm_mem_SOURCES := skmm_ddr.c
endif
skmm_sec_blob_SOURCES := skmm_sec_blob.c
skmm_memmgr_SOURCES := skmm_memmgr.c
skmm_uio_SOURCES := skmm_uio.c

80-sec-uio.rules_install := etc/udev/rules.d
81-sram-uio.rules_install := etc/udev/rules.d

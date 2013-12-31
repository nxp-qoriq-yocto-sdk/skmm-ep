/*
 * Copyright 2013 Freescale Semiconductor, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
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

#include <usdpaa/pci_ep_vfio.h>
#include <sys/ioctl.h>

static LIST_HEAD(group_list);

int vfio_pci_ep_read_config(struct pci_ep *ep, void *buf, int len, int addr)
{
	if (pread(ep->fd, buf, len, ep->config.offset + addr) != len)
		return -errno;

	return len;
}

int vfio_pci_ep_read_reg(struct pci_ep *ep, void *buf, int len, int addr)
{
	if (pread(ep->fd, buf, len, ep->reg.offset + addr) != len)
		return -errno;

	return len;
}

int vfio_pci_ep_write_reg(struct pci_ep *ep, void *buf, int len, int addr)
{
	if (pwrite(ep->fd, buf, len, ep->reg.offset + addr) != len)
		return -errno;

	return len;
}

void *vfio_pci_ep_map_win(struct pci_ep *ep, struct pci_ep_win *win,
				off_t off, size_t len)
{
	void *mem;

	mem = mmap(NULL, len, PROT_READ|PROT_WRITE, MAP_SHARED,
		   ep->fd, win->offset + off);

	if (mem == MAP_FAILED) {
		error(0, -errno,
		      "failed to map win%d type:%d offset:0x%llx length:%zd\n",
		      win->idx, win->type, off, len);
		return NULL;
	}

	return mem;
}

int vfio_pci_ep_get_win(struct pci_ep *ep, struct pci_ep_win *win)
{
	if (ioctl(ep->fd, VFIO_DEVICE_GET_WIN_INFO, win))
		return -EINVAL;

	return 0;
}

int vfio_pci_ep_set_win(struct pci_ep *ep, struct pci_ep_win *win)
{
	if (ioctl(ep->fd, VFIO_DEVICE_SET_WIN_INFO, win))
		return -EINVAL;

	return 0;
}

int vfio_pci_ep_init(struct pci_ep *ep)
{
	int ret, i;
	struct pci_ep_info *info;
	struct pci_ep_win *win;

	if (!ep)
		return -EINVAL;

	info = &ep->info;
	ret = ioctl(ep->fd, VFIO_DEVICE_GET_INFO, info);
	if (ret) {
		error(0, -errno, "failed to get device info %s\n", ep->name);
		return errno;
	}

	/* Initialize inbound windows */
	for (i = 0; i < info->iw_num; i++) {
		win = &ep->iw[i];
		win->type = PCI_EP_REGION_IBWIN;
		win->idx = i;
		vfio_pci_ep_get_win(ep, win);
	}

	/* Initialize outbound windows */
	for (i = 0; i < info->ow_num; i++) {
		win = &ep->ow[i];
		win->type = PCI_EP_REGION_OBWIN;
		win->idx = i;
		vfio_pci_ep_get_win(ep, win);
	}

	/* Initialize VF inbound windows */
	for (i = 0; i < info->vf_iw_num; i++) {
		win = &ep->vfiw[i];
		win->type = PCI_EP_REGION_VF_IBWIN;
		win->idx = i;
		vfio_pci_ep_get_win(ep, win);
	}

	/* Initialize VF outbound windows */
	for (i = 0; i < info->vf_ow_num; i++) {
		win = &ep->vfow[i];
		win->type = PCI_EP_REGION_VF_OBWIN;
		win->idx = i;
		vfio_pci_ep_get_win(ep, win);
	}

	/* Initialize PCI controller registers  window */
	win = &ep->reg;
	win->type = PCI_EP_REGION_REGS;
	win->idx = 0;
	vfio_pci_ep_get_win(ep, win);

	/* Initialize PCI EP configuration window */
	win = &ep->config;
	win->type = PCI_EP_REGION_CONFIG;
	win->idx = 0;
	vfio_pci_ep_get_win(ep, win);

	return 0;
}

void vfio_pci_ep_info(struct pci_ep *ep)
{
	int i;
	struct pci_ep_win *win;

	/* Update inbound windows */
	for (i = 0; i < ep->info.iw_num; i++) {
		win = &ep->iw[i];
		vfio_pci_ep_get_win(ep, win);
	}

	/* Update VF inbound windows */
	for (i = 0; i < ep->info.vf_iw_num; i++) {
		win = &ep->vfiw[i];
		vfio_pci_ep_get_win(ep, win);
	}

	/* Update outbound windows */
	for (i = 0; i < ep->info.ow_num; i++) {
		win = &ep->ow[i];
		vfio_pci_ep_get_win(ep, win);
	}

	/* Update VF outbound windows */
	for (i = 0; i < ep->info.vf_ow_num; i++) {
		win = &ep->vfow[i];
		vfio_pci_ep_get_win(ep, win);
	}

	printf("PCI EP device %s info:\n", ep->name);

	printf("\nOutbound windows:\n");
	for (i = 0; i < ep->info.ow_num; i++) {
		win = &ep->ow[i];
		printf(
		       "Win%d: cpu_addr:0x%llx pci_addr:0x%llx size:0x%llx "
		       "attr:0x%x\n",
			win->idx, win->cpu_addr, win->pci_addr, win->size,
			win->attr);
	}

	for (i = 0; i < ep->info.vf_ow_num; i++) {
		win = &ep->vfow[i];
		printf(
		       "VF Win%d: cpu_addr:0x%llx pci_addr:0x%llx size:0x%llx "
		       "attr:0x%x\n",
			win->idx, win->cpu_addr, win->pci_addr, win->size,
			win->attr);
		}


	printf("\nInbound windows:\n");
	for (i = 0; i < ep->info.iw_num; i++) {
		win = &ep->iw[i];
		printf(
		       "Win%d: cpu_addr:0x%llx pci_addr:0x%llx size:0x%llx "
		       "attr:0x%x\n",
			win->idx, win->cpu_addr, win->pci_addr, win->size,
			win->attr);
	}

	for (i = 0; i < ep->info.vf_iw_num; i++) {
		win = &ep->vfiw[i];
		printf(
		       "VF Win%d: cpu_addr:0x%llx pci_addr:0x%llx size:0x%llx "
		       "attr:0x%x\n",
			win->idx, win->cpu_addr, win->pci_addr, win->size,
			win->attr);
		}

	printf("\nRegisters window:\n");
	win = &ep->reg;
	printf("\tcpu_addr:0x%llx size:0x%llx\n",
		win->cpu_addr, win->size);
}

static int vfio_connect_container(struct vfio_us_group *group)
{
	int ret, fd;
	struct vfio_us_container *container;

	fd = open("/dev/vfio/vfio", O_RDWR);
	if (fd < 0) {
		error(0, 0, "fail to open /dev/vfio/vfio\n");
		return -errno;
	}

	ret = ioctl(fd, VFIO_GET_API_VERSION);
	if (ret != VFIO_API_VERSION) {
		error(0, 0, " not support vfio version: %d, "
			"reported version: %d\n", VFIO_API_VERSION, ret);
		close(fd);
		return -EINVAL;
	}

	container = malloc(sizeof(*container));
	memset(container, 0, sizeof(*container));

	container->fd = fd;
	group->container = container;

	/* Todo When need DMA operations */
	if (ioctl(fd, VFIO_CHECK_EXTENSION, VFIO_FSL_PAMU_IOMMU)) {
		ret = ioctl(group->fd, VFIO_GROUP_SET_CONTAINER, &fd);
		if (ret) {
			printf("vfio: failed to set group container:\n");
			free(container);
			close(fd);
			return -errno;
		}
	}

	ret = ioctl(fd, VFIO_SET_IOMMU, VFIO_FSL_PAMU_IOMMU);
	if (ret) {
		error(0, -errno, "vfio: failed to set iommu for container\n");
		free(container);
		close(fd);
		return -errno;
	}

	return 0;
}

static void vfio_disconnect_container(struct vfio_us_group *group)
{
	struct vfio_us_container *container = group->container;

	if (!container)
		return;

	if (ioctl(group->fd, VFIO_GROUP_UNSET_CONTAINER, &container->fd)) {
		error(0, 0, "disconnecting group %d from container\n",
			group->groupid);
	}

	group->container = NULL;

	close(container->fd);
	free(container);
}

static struct vfio_us_group *vfio_get_group(int groupid)
{
	struct vfio_us_group *group;
	int fd;
	char path[32];
	struct vfio_group_status status = { .argsz = sizeof(status) };

	list_for_each_entry(group, &group_list, node) {
		if (group->groupid == groupid) {
			group->count++;
			return group;
		}
	}

	snprintf(path, sizeof(path), "/dev/vfio/%d", groupid);
	fd = open(path, O_RDWR);
	if (fd < 0) {
		error(0, 0, "fail to open %s\n", path);
		return NULL;
	}

	group = malloc(sizeof(*group));
	memset(group, 0, sizeof(*group));

	if (ioctl(fd, VFIO_GROUP_GET_STATUS, &status)) {
		error(0, 0, "fail to get group%d status\n", groupid);
		close(fd);
		return NULL;
	}

	group->fd = fd;
	group->groupid = groupid;

	if (vfio_connect_container(group)) {
		error(0, 0, "fail to setup container for group %d\n", groupid);
		goto _err;
	}

	group->count++;
	list_add_tail(&group->node, &group_list);
	return group;

_err:
	close(fd);
	free(group);
	return NULL;
}

void vfio_put_group(struct vfio_us_group *group)
{
	group->count--;

	if (group->count > 0)
		return;

	vfio_disconnect_container(group);
	close(group->fd);
	list_del(&group->node);
	free(group);
}

struct pci_ep *vfio_pci_ep_open(int controller, int pf, int vf)
{
	char path[PATH_MAX], iommu_group_path[PATH_MAX];
	char ep_name[PATH_MAX], *group_name;
	ssize_t len;
	struct stat st;
	int groupid;
	int ret;
	struct vfio_us_group *group;
	struct pci_ep *ep;

	if (vf)
		snprintf(ep_name, sizeof(ep_name), "pci%d-pf%d-vf%d",
			 controller, pf, vf);
	else
		snprintf(ep_name, sizeof(ep_name), "pci%d-pf%d",
			 controller, pf);

	snprintf(path, sizeof(path), "/sys/class/pci_ep/%s/",
		 ep_name);

	if (stat(path, &st) < 0) {
		error(0, -errno, "no such pci ep device: %s\n", path);
		return NULL;
	}

	strncat(path, "iommu_group", sizeof(path) - strlen(path) - 1);

	len = readlink(path, iommu_group_path, PATH_MAX);
	if (len <= 0) {
		error(0, -errno, "no iommu_group for device\n");
		return NULL;
	}

	iommu_group_path[len] = 0;
	group_name = basename(iommu_group_path);

	if (sscanf(group_name, "%d", &groupid) != 1) {
		error(0, -errno, "can not get groupid frome %s\n",
			iommu_group_path);
		return NULL;
	}

	group = vfio_get_group(groupid);
	if (!group) {
		error(0, -errno, "can not open group %d\n", groupid);
		return NULL;
	}

	ret = ioctl(group->fd, VFIO_GROUP_GET_DEVICE_FD, ep_name);
	if (ret < 0) {
		error(0, -errno, "fail to get device %s from group %d\n",
			ep_name, group->groupid);
		goto _err;
	}

	ep = malloc(sizeof(*ep));
	if (!ep)
		goto _err;
	memset(ep, 0, sizeof(*ep));

	ep->fd = ret;
	ep->group = group;
	snprintf(ep->name, sizeof(ep->name), ep_name);
	vfio_pci_ep_init(ep);

	return ep;

_err:
	vfio_put_group(group);
	return NULL;
}

void vfio_pci_ep_close(struct pci_ep *ep)
{
	struct vfio_us_group *group;
	if (!ep)
		return;

	group = ep->group;

	ep->group = NULL;
	close(ep->fd);

	vfio_put_group(group);
}

/* Copyright (c) 2013 Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *   names of its contributors may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
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

#define MAX_DATA_TYPE	64

static int get_map_property(char *path, u32 map, const char *type,
		void *property)
{
	char buf[PATH_MAX];
	char full_path[PATH_MAX];
	ssize_t len;
	int fd;

	len = readlink(path, buf, PATH_MAX);
	if (len <= 0) {
		error(0, -errno, "no %s\n", path);
		return -ENOENT;
	}
	buf[len] = 0;

	snprintf(full_path, sizeof(full_path),
			"/sys/class/uio/%s/maps/map%d/%s", buf, map, type);

	fd = open(full_path, O_RDONLY);
	if (fd < 0) {
		error(0, -errno, "no %s\n", full_path);
		return -ENOENT;
	}

	read(fd, property, MAX_DATA_TYPE);

	return 0;
}

int get_map_size(char *path, u32 map, u32 *size)
{
	int ret;
	char buf[MAX_DATA_TYPE];

	ret = get_map_property(path, map, "size", buf);
	if (ret < 0)
		return -ENOENT;

	*size = strtol(buf, NULL, 0);

	return 0;
}

int get_map_addr(char *path, u32 map, phys_addr_t *addr)
{
	int ret;
	char buf[MAX_DATA_TYPE];

	memset(buf, 0, sizeof(buf));
	ret = get_map_property(path, map, "addr", buf);
	if (ret < 0)
		return -ENOENT;

	*addr = strtoll(buf, NULL, 0);

	return 0;
}

int get_map_offset(char *path, u32 map, u32 *offset)
{
	int ret;
	char buf[MAX_DATA_TYPE];

	ret = get_map_property(path, map, "offset", buf);
	if (ret < 0)
		return -ENOENT;

	*offset = strtol(buf, NULL, 0);

	return 0;
}

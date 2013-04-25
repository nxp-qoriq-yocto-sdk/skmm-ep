/* Copyright (c) 2013 Freescale Semiconductor, Inc.
 * All rights reserved.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <skmm.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <skmm_memmgr.h>
#include <abstract_req.h>
#include <skmm_sec_blob.h>
#include <usdpaa/of.h>

static void firmware_up(c_mem_layout_t *mem)
{
	print_debug("\n		FIRMWARE UP\n");
	ASSIGN32(mem->h_hs_mem->data.device.p_ib_mem_base_l,
		 (u32) mem->p_ib_mem);
	ASSIGN32(mem->h_hs_mem->data.device.p_ib_mem_base_h,
		 (u32) (mem->p_ib_mem >> 32));
	ASSIGN32(mem->h_hs_mem->data.device.p_ob_mem_base_l,
		 (u32) mem->p_pci_mem);
	ASSIGN32(mem->h_hs_mem->data.device.p_ob_mem_base_h,
		 (u32) (mem->p_pci_mem >> 32));

	ASSIGN8(mem->h_hs_mem->state, FIRMWARE_UP);

	SYNC_MEM
}

static void init_p_q(priority_q_t *p_q, u8 num)
{
	u8 i = 0;

	for (i = 0; i < (num - 1); i++) {
		p_q[i].ring = NULL;
		p_q[i].next = &(p_q[i + 1]);
	}
	p_q[i].ring = NULL;
	p_q[i].next = NULL;
}

static void init_rps(c_mem_layout_t *mem, u8 num, u32 *cursor)
{
	u8 i = 0;

	app_ring_pair_t *rps = mem->rsrc_mem->rps;

	print_debug("\t Init Ring Pairs	:\n");
	mem->rsrc_mem->idxs_mem = (indexes_mem_t *)*cursor;
	*cursor += (sizeof(indexes_mem_t) * (num + 1));
	memset(mem->rsrc_mem->idxs_mem, 0,
	       (sizeof(indexes_mem_t) * (num + 1)));
	print_debug("\t \t \t Indexes mem: %p\n",
		    mem->rsrc_mem->idxs_mem);

	mem->rsrc_mem->cntrs_mem = (counters_mem_t *)*cursor;
	*cursor += (sizeof(counters_mem_t));
	memset(mem->rsrc_mem->cntrs_mem, 0, sizeof(counters_mem_t));
	print_debug("\t \t \t Counters mem          : %p\n",
		    mem->rsrc_mem->cntrs_mem);

	mem->rsrc_mem->s_c_cntrs_mem = (counters_mem_t *)*cursor;
	*cursor += (sizeof(counters_mem_t));
	memset(mem->rsrc_mem->s_c_cntrs_mem, 0, sizeof(counters_mem_t));
	print_debug("\t \t \t S C Counters mem		: %p\n",
		    mem->rsrc_mem->s_c_cntrs_mem);

	mem->rsrc_mem->r_cntrs_mem = (ring_counters_mem_t *)*cursor;
	*cursor += (sizeof(ring_counters_mem_t) * (num + 1));
	memset(mem->rsrc_mem->r_cntrs_mem, 0,
	       (sizeof(ring_counters_mem_t) * (num + 1)));
	print_debug("\t \t \t R counters mem		: %p\n",
		    mem->rsrc_mem->r_cntrs_mem);

	mem->rsrc_mem->r_s_c_cntrs_mem = (ring_counters_mem_t *)*cursor;
	*cursor += (sizeof(ring_counters_mem_t) * (num + 1));
	memset(mem->rsrc_mem->r_s_c_cntrs_mem, 0,
	       (sizeof(ring_counters_mem_t) * (num + 1)));
	print_debug("\t \t \t R S C counters mem	: %p\n",
		    mem->rsrc_mem->r_s_c_cntrs_mem);

	for (i = 0; i < num; i++) {
		print_debug("\t \t \t Ring: %d	Details....\n", i);
		rps[i].req_r = NULL;
		rps[i].msi_addr = NULL;
		rps[i].sec = NULL;
		rps[i].r_s_cntrs = NULL;

		rps[i].idxs = &(mem->rsrc_mem->idxs_mem[i]);
		rps[i].cntrs = &(mem->rsrc_mem->r_cntrs_mem[i]);
		rps[i].r_s_c_cntrs = &(mem->rsrc_mem->r_s_c_cntrs_mem[i]);
		rps[i].ip_pool = mem->rsrc_mem->ip_pool;

		rps[i].next = NULL;

		print_debug("\t \t \t \t	Idxs addr: %p\n", rps[i].idxs);
		print_debug("\t \t \t \t	Cntrs: %p\n", rps[i].cntrs);
		print_debug("\t \t \t \t	R S C cntrs: %p\n",
				rps[i].r_s_c_cntrs);
		print_debug("\t \t \t \t	Ip pool: %p\n", rps[i].ip_pool);
	}
}

static void init_drv_resp_ring(c_mem_layout_t *mem, u32 offset, u32 depth,
		u32 *cursor)
{
	u8 loc = mem->rsrc_mem->ring_count;
	drv_resp_ring_t *ring = mem->rsrc_mem->drv_resp_ring;

	print_debug("\t	Init Drv Resp Ring:\n");
	ring->msi_data = 0;
	ring->intr_ctrl_flag = 0;
	ring->msi_addr = ring->r_s_cntrs = NULL;
	ring->depth = depth;
	ring->resp_r = (resp_ring_t *)(mem->v_ob_mem +
			(u32)((mem->p_pci_mem + offset) - mem->p_ob_mem));
	ring->idxs = &(mem->rsrc_mem->idxs_mem[loc]);
	ring->r_cntrs = &(mem->rsrc_mem->r_cntrs_mem[loc]);
	ring->r_s_c_cntrs = &(mem->rsrc_mem->r_s_c_cntrs_mem[loc]);

	print_debug("\t \t	Resp ring addr: %p\n", ring->resp_r);
	print_debug("\t \t	Indexes addr: %p\n", ring->idxs);
	print_debug("\t \t	R Cntrs addr: %p\n", ring->r_cntrs);
	print_debug("\t \t	R S C cntrs addr: %p\n", ring->r_s_c_cntrs);
}

static void init_scs(c_mem_layout_t *mem)
{
	u32 i = 0;
	app_ring_pair_t *rps = mem->rsrc_mem->rps;

	print_debug("\t \t \t Init R S mem.... \n");
	for (i = 0; i < mem->rsrc_mem->ring_count; i++) {
		rps[i].r_s_cntrs = &(mem->rsrc_mem->r_s_cntrs_mem[i]);
		print_debug("\t \t \t \t \t Ring	:%d R S Cntrs: %p\n",
				i, rps[i].r_s_cntrs);
	}

	mem->rsrc_mem->drv_resp_ring->r_s_cntrs =
		&(mem->rsrc_mem->r_s_cntrs_mem[i]);
	print_debug
		("\t \t \t \t \t Driver resp ring R S Cntrs: %p\n",
		 mem->rsrc_mem->drv_resp_ring->r_s_cntrs);
}

static void add_ring_to_pq(priority_q_t *p_q, app_ring_pair_t *rp, u8 pri)
{
	app_ring_pair_t *cursor = p_q[pri].ring;

	print_debug("\t \t \t ********* Pri:%d p_q:%p rp:%p cursor:%p\n",
		    pri, p_q, rp, cursor);

	if (!rp->id)
		return;

	if (!cursor)
		p_q[pri].ring = rp;
	else {
		while (cursor->next)
			cursor = cursor->next;
		cursor->next = rp;
	}
}

static void make_rp_circ_list(c_mem_layout_t *mem)
{
	priority_q_t *p_q = mem->rsrc_mem->p_q;
	app_ring_pair_t *r = NULL;

	while (p_q) {
		r = p_q->ring;
		while (r->next)
			r = r->next;
		p_q = p_q->next;
		if (p_q)
			r->next = p_q->ring;

	}
	r->next = mem->rsrc_mem->p_q->ring;
	mem->rsrc_mem->rps = r;
}

static u32 handshake(c_mem_layout_t *mem, u32 *cursor)
{
	u32 r_offset = 0;
	u8 max_pri;
	u8 max_rps;
	u32 req_mem_size;
	u32 resp_ring_off;
	u32 s_cntrs, r_s_cntrs;
	u32 offset = 0;
	u32 rid, prio, msi_addr_l;
	app_ring_pair_t *rp;

	print_debug("\n	HANDSHAKE \n");
	print_debug("\t State address: %p\n", &(mem->c_hs_mem->state));

	/*
	 * Mark the firmware up to the driver
	 */
	firmware_up(mem);

	while (true) {
		WAIT_FOR_STATE_CHANGE(mem->c_hs_mem->state);
		print_debug("\t State updated by driver: %d\n",
				mem->c_hs_mem->state);

		switch (mem->c_hs_mem->state) {
		case FW_INIT_CONFIG:
			mem->c_hs_mem->state = DEFAULT;
			print_debug("\n	FW_INIT_CONFIG:\n");
			mem->rsrc_mem->ring_count =
				mem->c_hs_mem->data.config.num_of_rps;

			/*
			 *
			 */
			max_pri = mem->c_hs_mem->data.config.max_pri;
			print_debug ("\t	Max pri	: %d\n", max_pri);
			*cursor = ALIGN_TO_L1_CACHE_LINE(*cursor);
			/* Alloc memory for prio q first */
			mem->rsrc_mem->p_q = (priority_q_t *)(*cursor);
			*cursor += (max_pri * sizeof(priority_q_t));
			init_p_q(mem->rsrc_mem->p_q, max_pri);

			/*
			 *
			 */
			max_rps = mem->c_hs_mem->data.config.num_of_rps;
			print_debug ("\t	Max rps	: %d\n", max_rps);
			mem->rsrc_mem->ring_count = max_rps;
			*cursor = ALIGN_TO_L1_CACHE_LINE(*cursor);
			mem->rsrc_mem->rps = (app_ring_pair_t *) *cursor;
			*cursor += (max_rps * sizeof(app_ring_pair_t));

			init_rps(mem, max_rps, cursor);


			/*
			 *
			 */
			req_mem_size = mem->c_hs_mem->data.config.req_mem_size;
			print_debug("\t	Req mem size: %d\n", req_mem_size);

			*cursor = ALIGN_TO_L1_CACHE_LINE(*cursor);
			mem->rsrc_mem->req_mem = (void *)*cursor;
			*cursor += req_mem_size;
			print_debug("\t	Req mem addr: %p\n", mem->rsrc_mem->req_mem);

			/*
			 *
			 */
#define RESP_RING_DEPTH	(4 * 128)
			resp_ring_off = mem->c_hs_mem->data.config.fw_resp_ring;
			print_debug("\t	Resp ring off :%0x\n", resp_ring_off);

			*cursor = ALIGN_TO_L1_CACHE_LINE(*cursor);
			mem->rsrc_mem->drv_resp_ring =
				(drv_resp_ring_t *)*cursor;
			*cursor += sizeof(drv_resp_ring_t);

			init_drv_resp_ring(mem, resp_ring_off, RESP_RING_DEPTH,
					cursor);

			/*
			 *
			 */
			s_cntrs = mem->c_hs_mem->data.config.s_cntrs;
			r_s_cntrs = mem->c_hs_mem->data.config.r_s_cntrs;
			mem->rsrc_mem->s_cntrs_mem =
				(shadow_counters_mem_t *) (mem->v_ob_mem +
				(u32)((mem->p_pci_mem + s_cntrs) - mem->p_ob_mem));
			mem->rsrc_mem->r_s_cntrs_mem =
				(ring_shadow_counters_mem_t *)(mem->v_ob_mem +
				 (u32)((mem->p_pci_mem + r_s_cntrs) - mem->p_ob_mem));

			print_debug("\t Shadow counters details from Host\n");
			print_debug("\t \t \t S CNTRS OFFSET :%0x\n", s_cntrs);
			print_debug("\t \t \t R S CNTRS OFFSET :%0x\n", r_s_cntrs);
			init_scs(mem);

			/*
			 *
			 */
			print_debug("\n SENDING FW_INIT_CONFIG_COMPLETE\n");
			offset = (u32)mem->rsrc_mem->r_s_c_cntrs_mem -
					mem->v_ib_mem;
			ASSIGN32(mem->h_hs_mem->data.config.s_r_cntrs, offset);
			print_debug("\t \t \t S R CNTRS OFFSET :%0x\n", offset);

			offset = (u32) mem->rsrc_mem->s_c_cntrs_mem - \
					mem->v_ib_mem;
			ASSIGN32(mem->h_hs_mem->data.config.s_cntrs, offset);
			print_debug("\t \t \t S CNTRS OFFSET :%0x\n", offset);

			offset = (u32)mem->rsrc_mem->ip_pool - mem->v_ib_mem;
			ASSIGN32(mem->h_hs_mem->data.config.ip_pool, offset);

			offset = (u32)&(mem->rsrc_mem->drv_resp_ring->intr_ctrl_flag)
					- mem->v_ib_mem;
			ASSIGN32(mem->h_hs_mem->data.config.resp_intr_ctrl_flag,
					offset);

			ASSIGN8(mem->h_hs_mem->result, RESULT_OK);
			ASSIGN8(mem->h_hs_mem->state, FW_INIT_CONFIG_COMPLETE);

			break;

		case FW_INIT_RING_PAIR:
			mem->c_hs_mem->state = DEFAULT;
			print_debug("\n	FW_INIT_RING_PAIR\n");
			/*
			 *
			 */
			rid = mem->c_hs_mem->data.ring.rid;
			prio = (mem->c_hs_mem->data.ring.props \
				& APP_RING_PROP_PRIO_MASK) \
			       >> APP_RING_PROP_PRIO_SHIFT;
			msi_addr_l = mem->c_hs_mem->data.ring.msi_addr_l;
			rp = &(mem->rsrc_mem->rps[rid]);

			rp->id = rid;
			rp->props = mem->c_hs_mem->data.ring.props;
			rp->depth = mem->c_hs_mem->data.ring.depth;
			rp->msi_data = mem->c_hs_mem->data.ring.msi_data;
			rp->msi_addr = (void *)((mem->v_msi_mem +
					(u32)((mem->p_pci_mem + msi_addr_l)
					- mem->p_msi_mem)));
			rp->req_r = mem->rsrc_mem->req_mem + r_offset;
			r_offset += (rp->depth * sizeof(req_ring_t));
			rp->resp_r = (resp_ring_t *) (mem->v_ob_mem +
					(u32)((mem->p_pci_mem +
					mem->c_hs_mem->data.ring.resp_ring)
					- mem->p_ob_mem));

			print_debug("\t	Rid: %d\n", rid);
			print_debug("\t	Prio :%d\n", prio);
			print_debug("\t	Depth :%d\n", rp->depth);
			print_debug("\t	MSI Data :%0x\n", rp->msi_data);
			print_debug("\t MSI addr :%p\n", rp->msi_addr);
			print_debug("\t	Req r addr :%p\n", rp->req_r);
			print_debug("\t Resp r addr :%p\n", rp->resp_r);

			add_ring_to_pq(mem->rsrc_mem->p_q, rp, (prio - 1));

			/*
			 *
			 */
			offset = 0;
			offset = (u32)((u8 *)rp->req_r - mem->v_ib_mem);
			ASSIGN32(mem->h_hs_mem->data.ring.req_r, offset);
			offset = (u32)((u8 *)&(rp->intr_ctrl_flag) - mem->v_ib_mem);
			ASSIGN32(mem->h_hs_mem->data.ring.intr_ctrl_flag, offset);

			ASSIGN8(mem->h_hs_mem->result, RESULT_OK);
			ASSIGN8(mem->h_hs_mem->state, FW_INIT_RING_PAIR_COMPLETE);
			break;

		case HS_COMPLETE:
			mem->rsrc_mem->drv_resp_ring->msi_addr =
				mem->rsrc_mem->rps[0].msi_addr;
			mem->rsrc_mem->drv_resp_ring->msi_data =
				mem->rsrc_mem->rps[0].msi_data;
			/*
			 * make_rp_circ_list(mem);
			 */
			print_debug("\n	HS_COMPLETE:	\n");
			return 0;
		}
	}
}

static void make_sec_circ_list(sec_engine_t *sec)
{
	int i = 0;
	u32 sec_eng_cnt = fsl_sec_get_eng_num();

	for (i = 0; i < (sec_eng_cnt - 1); i++)
		sec[i].next = &(sec[i + 1]);
	sec[i].next = &sec[0];
}

static void copy_kek_and_set_scr(c_mem_layout_t *c_mem)
{
	u32 kek, i;
	sec_engine_t *sec1 = c_mem->rsrc_mem->sec;
	sec_engine_t *sec2 = c_mem->rsrc_mem->sec + 1;
	sec_engine_t *sec3 = c_mem->rsrc_mem->sec + 2;
	struct kek_regs *kek1 = (struct kek_regs *)sec1->kek;
	struct kek_regs *kek2 = (struct kek_regs *)sec2->kek;
	struct kek_regs *kek3 = (struct kek_regs *)sec3->kek;

	for (i = 0; i < 24; i += 1) {
		kek = in_be32((u32 *)(kek1 + i));
		out_be32((u32 *)(kek2 + i), kek);
		out_be32((u32 *)(kek3 + i), kek);
	}

	out_be32(sec1->scfg, 0x00000703);
	out_be32(sec2->scfg, 0x00000703);
	out_be32(sec3->scfg, 0x00000703);
}

static u32 init_rsrc_sec(sec_engine_t *sec, u32 *cursor)
{
	u32 l2_cursor = *cursor;
	u32 mem = 0;

	sec->jr.id = sec->id;

	sec->jr.i_ring = (sec_ip_ring_t *)l2_cursor;
	l2_cursor +=
		ALIGN_TO_L1_CACHE_LINE((SEC_JR_DEPTH * sizeof(sec_ip_ring_t)));

	mem += SEC_JR_DEPTH * sizeof(sec_ip_ring_t);
	memset(sec->jr.i_ring, 0, (SEC_JR_DEPTH * sizeof(sec_ip_ring_t)));
	print_debug("\t sec ip ring :%p\n", sec->jr.i_ring);

	sec->jr.o_ring = (struct sec_op_ring *)l2_cursor;
	l2_cursor += ALIGN_TO_L1_CACHE_LINE((SEC_JR_DEPTH
				* sizeof(struct sec_op_ring)));

	mem += SEC_JR_DEPTH * sizeof(struct sec_op_ring);
	memset(sec->jr.o_ring, 0, (SEC_JR_DEPTH * sizeof(struct sec_op_ring)));
	print_debug("\t sec op ring: %p\n", sec->jr.o_ring);

	/*
	 * Call for hardware init of sec engine
	 */
	init_sec_regs_offset(sec);
	fsl_sec_init(sec);

	*cursor = l2_cursor;

	return mem;
}

static void alloc_rsrc_mem(c_mem_layout_t *c_mem, u32 *cursor)
{
	resource_t *rsrc = c_mem->rsrc_mem;
	u32 l2_cursor = *cursor;
	sec_engine_t *sec = NULL;
	int i = 0;
	u32 sec_eng_count = fsl_sec_get_eng_num();

	print_debug("\n	alloc_rsrc_mem\n");
	print_debug("\t rsrc addr :%p\n", rsrc);

	memset(rsrc, 0, sizeof(resource_t));
	rsrc->sec_eng_cnt = sec_eng_count;

	/*
	 * Initialize the SEC engine
	 * * All the required memory for SEC engine will be allocated in L2 SRAM
	 * * Max we may need = 3sec engines * (sizeof(sec_engine_t)) -- Given 128
	 * * as depth of rings the max size required is approx 2624 bytes.
	 */
	rsrc->sec = (sec_engine_t *)(l2_cursor);
	memset(rsrc->sec, 0, sizeof(sec_engine_t) * sec_eng_count);
	print_debug("\t sec addr :%p\n", rsrc->sec);

	l2_cursor += ALIGN_TO_L1_CACHE_LINE((sec_eng_count * sizeof(sec_engine_t)));
	c_mem->free_mem -= sec_eng_count * sizeof(sec_engine_t);
	make_sec_circ_list(rsrc->sec);

	/*
	 * Call for hardware init of sec engine
	 */
	sec = rsrc->sec;
	for (i = 0; i < sec_eng_count; i++) {
		sec->id = (i + 1);
		c_mem->free_mem -= init_rsrc_sec(sec, &l2_cursor);
		sec = sec->next;
	}

#ifdef COMMON_IP_BUFFER_POOL
	rsrc->ip_pool = (void *)(l2_cursor);
	l2_cursor += ALIGN_TO_L1_CACHE_LINE(DEFAULT_POOL_SIZE);
	c_mem->free_mem -= (DEFAULT_POOL_SIZE);

	reg_mem_pool((u8 *)l2_cursor, DEFAULT_EP_POOL_SIZE);
	l2_cursor += ALIGN_TO_L1_CACHE_LINE(DEFAULT_EP_POOL_SIZE);
	print_debug("\t	ip pool addr :%p\n", rsrc->ip_pool);
#endif
	*cursor = l2_cursor;
}

/* Switch controls */
#define TIMED_WAIT_FOR_JOBS

#define BUDGET_NO_OF_TOT_JOBS		50

#ifndef TIMED_WAIT_FOR_JOBS
#define WAIT_FOR_DRIVER_JOBS(x, y) \
	while (BUDGET_NO_OF_TOT_JOBS > (x - y)) { SYNC_MEM }
#else
#define WAIT_FOR_DRIVER_JOBS	conditional_timed_wait_for_driver_jobs
#endif

#define RINGS_JOBS_ADDED(ring) \
	(ring->r_s_c_cntrs->jobs_added - ring->cntrs->jobs_processed)
#define MOD_ADD(x, value, size)	((x + value) & (size - 1))

#ifdef TIMED_WAIT_FOR_JOBS
static inline void wait_for_timeout(u64 usecs)
{
	u64 start_ticks = 0;
	u64 timeout_ticks = 0;

	start_ticks = getticks();
	timeout_ticks = usec2ticks(usecs);

	while (getticks() - start_ticks < timeout_ticks);
}

static inline u32 timed_wait_for_driver_jobs(u32 x, u32 y)
{
#define HOST_JOB_WAIT_TIME_OUT  100000ull
	wait_for_timeout((HOST_JOB_WAIT_TIME_OUT));

	SYNC_MEM
	return x - y;
}

static inline u32 conditional_timed_wait_for_driver_jobs(u32 *x, u32 *y)
{
	u64 start_ticks = 0;
	u64 timeout_ticks = 0;

	start_ticks = getticks();
	timeout_ticks = usec2ticks(HOST_JOB_WAIT_TIME_OUT);

	while ((getticks() - start_ticks < timeout_ticks)
	       && ((*x - *y) < BUDGET_NO_OF_TOT_JOBS))
		SYNC_MEM
		return *x - *y;
}

static inline u8 intr_time_threshold(c_mem_layout_t *c_mem)
{
	c_mem->intr_ticks += getticks();
	return ((c_mem->intr_ticks > c_mem->intr_timeout_ticks));
}

#endif /* RESP_RING_DEPTH */

static inline void Enq_Cpy(sec_ip_ring_t *sec_i, req_ring_t *req_r, u32 count)
{
	while (count--) {
		phys_addr_t desc, abs_req = *(u64 *)req_r++;

		desc = parse_abs_to_desc(abs_req);
		if (!desc)
			break;

		*(u64 *) sec_i++ = desc;
	}
}

static inline void memcpy_rev(void *d, void *s, u32 cnt)
{
	s = (u8 *) s + cnt;
	while (cnt--)
		*((u8 *) d++) = *((u8 *) s--);
}

#define HOST_TYPE_P4080
static inline void Deq_Cpy(resp_ring_t *resp_r, sec_op_ring_t *sec_o, u32 count)
{
	int i = 0;

	while (i < count) {
		struct abs_req_s **abs_req;

		abs_req = (struct abs_req_s **)(pa_to_va(sec_o[i].desc)
				- sizeof(struct abs_req_s *));

		ASSIGN64(resp_r[i].desc, va_to_pa((va_addr_t)*abs_req));
		put_buffer(abs_req);
		ASSIGN32(resp_r[i].result, sec_o[i].status);
		i++;
	}
}

inline int circ_room(u32 wi, u32 ri, u32 w_depth, u32 r_depth, u32 count)
{
	int val1 = LINEAR_ROOM(wi, w_depth, count);
	int val2 = LINEAR_ROOM(ri, r_depth, count);

	return MIN(val1, val2);
}

static inline u32 sel_sec_enqueue(c_mem_layout_t *c_mem, sec_engine_t **psec,
		app_ring_pair_t *rp, u32 cnt, u32 *todeq)
{
	u32 sec_sel = 0;
	sec_engine_t *sec = NULL;
	u32 addr;
	sec_jr_t *jr;
	u32 room, wi, ri;

	if (!cnt)
		return 0;

	addr = pa_to_va((rp->req_r[rp->idxs->r_index].desc -
			sizeof(u32)));
	sec_sel = *(u32 *) addr;
	switch (sec_sel) {
	case 1:
		sec = c_mem->rsrc_mem->sec;
		break;
	case 2:
		sec = c_mem->rsrc_mem->sec + 1;
		break;
	case 3:
		sec = c_mem->rsrc_mem->sec + 2;
		break;
	default:
		sec = *psec;
		break;
	}

	jr = &(sec->jr);
	room = in_be32(&(jr->regs->irsa));
	wi = jr->tail;
	ri = rp->idxs->r_index;

	room = MIN(room, cnt);
	room = circ_room(wi, ri, jr->size, rp->depth, room);
	if (!room)
		goto RET;

	jr->enq_cnt += room;
	sec->tot_req_cnt += room;

	Enq_Cpy(&jr->i_ring[wi], &rp->req_r[ri], (room));

	jr->tail = MOD_ADD(wi, room, jr->size);
	rp->idxs->r_index = MOD_ADD(ri, room, rp->depth);

	rp->cntrs->jobs_processed += room;
	ASSIGN32(rp->r_s_cntrs->req_jobs_processed, rp->cntrs->jobs_processed);
	out_be32(&(jr->regs->irja), room);

RET:
	*psec = sec->next;
	*todeq += room;
	return room;
}

static inline u32 sec_enqueue(sec_engine_t **psec, app_ring_pair_t *rp,
		u32 cnt, u32 *todeq)
{
	sec_engine_t *sec;
	sec_jr_t *jr;
	u32 room, wi, ri;

	if (!cnt)
		return 0;

	/*
	 * How much room do we have
	 */
	sec = *psec;
	jr = &(sec->jr);
	room = in_be32(&(jr->regs->irsa));
	wi = jr->tail;
	ri = rp->idxs->r_index;

	room = MIN(room, cnt);
	room = circ_room(wi, ri, jr->size, rp->depth, room);
	if (!room)
		goto RET;

	jr->enq_cnt += room;
	sec->tot_req_cnt += room;

	Enq_Cpy(&jr->i_ring[wi], &rp->req_r[ri], (room));

	jr->tail = MOD_ADD(wi, room, jr->size);
	rp->idxs->r_index = MOD_ADD(ri, room, rp->depth);

	rp->cntrs->jobs_processed += room;
	ASSIGN32(rp->r_s_cntrs->req_jobs_processed, rp->cntrs->jobs_processed);
	out_be32(&(jr->regs->irja), room);

RET:
	if (cnt - room)
		*psec = sec->next;
	*todeq += room;
	return room;
}

static inline u32 sec_dequeue(sec_jr_t *jr, drv_resp_ring_t *r, u32 *todeq)
{
	u32 room, wi, ri, cnt;

	if (!*todeq)
		return 0;

	room = r->depth - (r->r_cntrs->jobs_added -
			r->r_s_c_cntrs->jobs_processed);
	wi = r->idxs->w_index;
	ri = jr->head;
	cnt = in_be32(&jr->regs->orsf);

	room = MIN(room, cnt);
	room = circ_room(wi, ri, r->depth, jr->size, room);
	if (!room)
		return 0;

	jr->deq_cnt += room;

	r->r_cntrs->jobs_added += room;

	Deq_Cpy(&r->resp_r[wi], &jr->o_ring[ri], room);

	jr->head = MOD_ADD(ri, room, jr->size);
	r->idxs->w_index = MOD_ADD(wi, room, r->depth);
	ASSIGN32(r->r_s_cntrs->resp_jobs_added, r->r_cntrs->jobs_added);

	out_be32(&jr->regs->orjr, room);

	*todeq -= room;
	return room;
}

static inline void raise_intr(drv_resp_ring_t *r)
{
	ASSIGN16_PTR(r->msi_addr, r->msi_data);
}

#ifdef CMD_RING_SUPPORT
void invalidate_pending_app_reqs(c_mem_layout_t *c_mem)
{
	indexes_mem_t *ring_indexes = NULL;
	ring_counters_mem_t *ring_counters = NULL;
	ring_shadow_counters_mem_t *s_ring_counters = NULL;
	priority_q_t *p_q_cursor = c_mem->rsrc_mem->p_q;
	app_ring_pair_t *ring_cursor = NULL;
	app_ring_pair_t *ring_cursor_head = NULL;
	drv_resp_ring_t *drv_r = c_mem->rsrc_mem->drv_resp_ring;

	u32 ri = 0;
	u32 wi = 0;

	print_debug("\n Invalidating the pending reqs on all the app rings\n");

	while (p_q_cursor) {
		ring_cursor_head = ring_cursor = p_q_cursor->ring;
		while (ring_cursor) {

			ring_counters = ring_cursor->cntrs;
			ring_indexes = ring_cursor->idxs;
			s_ring_counters = ring_cursor->r_s_cntrs;
			ri = ring_indexes->r_index;
			wi = drv_r->idxs->w_index;

			print_debug("Read index %d, Write index : %d\n", ri, wi);
			print_debug("\t Jobs added  :%d Jobs Processed  :%d\n",
				    ring_cursor->r_s_c_cntrs->jobs_added,
				    ring_counters->jobs_processed);
			print_debug("\t Jobs pending    :%d on Ring     :%d\n",
				    ring_cursor->r_s_c_cntrs->jobs_added -
				    ring_counters->jobs_processed,
				    ring_cursor->id);

			while (0 != (ring_cursor->r_s_c_cntrs->jobs_added -
				ring_counters->jobs_processed)) {
				ASSIGN64(drv_r->resp_r[wi].desc,
					 ring_cursor->req_r[ri].desc);
#define JOB_DISACRDED   -1
				ASSIGN32(drv_r->resp_r[wi].result,
					 JOB_DISACRDED);
				wi = MOD_INC(wi, drv_r->depth);
				ri = MOD_INC(ri, ring_cursor->depth);

				ring_counters->jobs_processed += 1;
				drv_r->r_cntrs->jobs_added += 1;
			}
			ring_indexes->r_index = ri;
			drv_r->idxs->w_index = wi;

			print_debug
				("Updated read %d and write %d \
				index for ring %d, \
				ring_counters->jobs_processed %d, \
				ring_counters->jobs_added %d\n",\
				ri, wi, ring_cursor->id, \
				ring_counters->jobs_processed, \
				ring_counters->jobs_added);

			ASSIGN32(drv_r->r_s_cntrs->resp_jobs_added,
				 drv_r->r_cntrs->jobs_added);
			ASSIGN32(s_ring_counters->req_jobs_processed,
				 ring_counters->jobs_processed);
			print_debug("\t Giving interrupt for ring :%d\n",
				    ring_cursor->id);
			ASSIGN16_PTR(ring_cursor->msi_addr,
				     ring_cursor->msi_data);
			ring_cursor = ring_cursor->next;
			print_debug ("ring_cursor : %p, ring_cursor_head : %p\n",
				 ring_cursor, ring_cursor_head);
			if (ring_cursor_head == ring_cursor)
				ring_cursor = NULL;
		}
		p_q_cursor = p_q_cursor->next;
	}
}

void resetcounters(c_mem_layout_t *mem, u32 sec_id)
{
	u32 i = 0;

	indexes_mem_t *ring_indexes = NULL;
	ring_counters_mem_t *ring_counters = NULL;
	priority_q_t *p_q_cursor = mem->rsrc_mem->p_q;
	app_ring_pair_t *ring_cursor = NULL;
	app_ring_pair_t *ring_cursor_head = NULL;


	print_debug("\n Reset counters .............\n");
	mem->rsrc_mem->sec[sec_id].tot_req_cnt =
		mem->rsrc_mem->sec[sec_id].tot_resp_cnt = 0;
	mem->rsrc_mem->cntrs_mem->tot_jobs_added =
		mem->rsrc_mem->cntrs_mem->tot_jobs_processed = 0;
	for (i = 1; i < mem->rsrc_mem->ring_count; ++i) {
		ring_indexes = &(mem->rsrc_mem->idxs_mem[i]);
		ring_counters = &(mem->rsrc_mem->r_cntrs_mem[i]);
		print_debug("Updates for ring %d, ring_indexes : %p \
				ring_counters: %p\n",
				i, ring_indexes, ring_counters);
		ring_indexes->w_index = ring_indexes->r_index = 0;
		print_debug("index update  Finished\n");
		ring_counters->jobs_added = ring_counters->jobs_processed = 0;
		print_debug("jobs_added Finished\n");
	}
	while (p_q_cursor) {
		ring_cursor_head = ring_cursor = p_q_cursor->ring;
		while (ring_cursor) {
			ring_cursor->cntrs->jobs_added =
				ring_cursor->cntrs->jobs_processed = 0;
			ring_cursor->r_s_c_cntrs->jobs_added = 0;
			ring_cursor = ring_cursor->next;
			print_debug("ring_cursor : %p, ring_cursor_head : %p\n",
					ring_cursor, ring_cursor_head);
			if (ring_cursor_head == ring_cursor)
				ring_cursor = NULL;
		}
		p_q_cursor = p_q_cursor->next;
		print_debug("Going for Next priority queue\n");
	}
	print_debug("resetting counters for response ring r_cntrs %p, \
			r_s_c_cntrs %p\n",
			mem->rsrc_mem->drv_resp_ring->r_cntrs,
			mem->rsrc_mem->drv_resp_ring->r_s_c_cntrs);
	mem->rsrc_mem->drv_resp_ring->r_cntrs->jobs_added =
		mem->rsrc_mem->drv_resp_ring->r_cntrs->jobs_processed = 0;
	mem->rsrc_mem->drv_resp_ring->idxs->w_index =
		mem->rsrc_mem->drv_resp_ring->idxs->r_index = 0;
}

/*
 * Function     : process_command
 *
 * Arguments    : mem : Pointer to the cards memory where all the resources start
 *                cmd_req: Dequeue command req ring desc
 *
 * Return Value : u32
 *
 * Description  : Process the command from the host driver
 *
 */
u32 process_command(c_mem_layout_t *mem, cmd_ring_req_desc_t *cmd_req)
{
	cmd_op_t *cmd_op = NULL;
	u32 i = 0;
	int fd;
	void *ccsr;
	volatile u32 *rstcr;
	priority_q_t *p_q_cursor = NULL;
	app_ring_pair_t *ring_cursor = NULL;
	app_ring_pair_t *head_ring_cursor = NULL;
	u32 total_job_added = 0;
	u32 total_job_processed = 0;
	u32 pending_cnt = 0;
	u32 r_id = 0;
	u32 prop = 0;

	print_debug("   ---- Command Ring Processing ----\n");
	switch (cmd_req->cmd_type) {
	case BLOCK_APP_JOBS:
		/*
		 * This command is sent by driver to block the jobs on app rings
		 * * Driver may use this feature for smooth exits for some of
		 * the commands like
		 * * RESET SEC, RESET DEV etc.
		 */

		/*
		 * Invalidate the current pending app reqs on all the request rings
		 */
		invalidate_pending_app_reqs(mem);
		/*
		 * This return value is useful for the callee to block jobs
		 */
		return BLOCK_APP_JOBS;
		break;

	case RESETDEV:
#define RSTCR_HRESET_REQ	0x2
		print_debug("\t \t Resetting Device\n");
		fd = open("/dev/mem", O_RDWR);
		if (fd < 0) {
			printf("fail to open /dev/mem\n");
			return -1;
		}
		ccsr = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE,
				MAP_SHARED, fd, get_ccsr_phys_addr());
		rstcr = ccsr + 0xe00b0;
		*rstcr = RSTCR_HRESET_REQ;
		munmap(ccsr, 0x1000);
		break;

	case RESETSEC:
		print_debug("\t \t Resetting sec engine :%d\n",
			    cmd_req->ip_info.sec_id);
		sec_reset(mem->rsrc_mem->sec[cmd_req->ip_info.sec_id].info);
		{
			sec_jr_t *sec_jr = &(mem->rsrc_mem->sec->jr);

			print_debug("\t Before SEC Input ring virtual address:%p\n",
					sec_jr->i_ring);
			print_debug("\t SEC Output ring virtual address:%p\n",
					sec_jr->o_ring);
		}
		fsl_sec_init(&(mem->rsrc_mem->sec[cmd_req->ip_info.sec_id]));
		{
			sec_jr_t *sec_jr = &(mem->rsrc_mem->sec->jr);

			print_debug("\t After  fsl_sec_init SEC Input ring \
					virtual address :%p\n",
					sec_jr->i_ring);
			print_debug("\t SEC Output ring virtual address :%p\n",
					sec_jr->o_ring);
		}
		resetcounters(mem, cmd_req->ip_info.sec_id);
		{
			int secid = cmd_req->ip_info.sec_id;
			sec_jr_t *sec_jr = &(mem->rsrc_mem->sec[secid].jr);

			print_debug("\t After resetcounters SEC Input ring \
					virtual address :%p\n",
					sec_jr->i_ring);
			print_debug("\t SEC Output ring virtual address :%p\n",
					sec_jr->o_ring);
		}
		/*
		 * Driver as part of protocol would have sent the
		 * block command earlier..
		 * Since now the RESET is done, we can unblock the
		 * rings and accept the jobs.
		 */
		return UNBLOCK_APP_JOBS;
		break;

	case DEVSTAT:
		print_debug("\t \t Device stats\n");

		p_q_cursor = mem->rsrc_mem->p_q;

		cmd_op = (cmd_op_t *)(mem->v_ob_mem +
				(u32)(cmd_req->cmd_op - mem->p_ob_mem));

		ASSIGN32(cmd_op->buffer.dev_stat_op.fvversion, FW_VERSION);
		ASSIGN32(cmd_op->buffer.dev_stat_op.totalmem, TOTAL_CARD_MEMORY);
		ASSIGN32(cmd_op->buffer.dev_stat_op.codemem, FIRMWARE_SIZE);
		ASSIGN32(cmd_op->buffer.dev_stat_op.heapmem,
				TOTAL_CARD_MEMORY - FIRMWARE_SIZE);
		ASSIGN32(cmd_op->buffer.dev_stat_op.freemem, mem->free_mem);
		ASSIGN32(cmd_op->buffer.dev_stat_op.num_of_sec_engine,
				mem->rsrc_mem->sec_eng_cnt);
		ASSIGN32(cmd_op->buffer.dev_stat_op.no_of_app_rings,
				mem->rsrc_mem->ring_count);
		while (p_q_cursor) {
			head_ring_cursor = ring_cursor = p_q_cursor->ring;

			while (ring_cursor) {
				total_job_added += ring_cursor->r_s_c_cntrs->jobs_added;
				total_job_processed += ring_cursor->cntrs->jobs_processed;
				ring_cursor = ring_cursor->next;
				if (head_ring_cursor == ring_cursor)
					ring_cursor = NULL;
			}

			p_q_cursor = p_q_cursor->next;
		}
		print_debug("Total job added : %d, Total job processed : %d\n",
				total_job_added, total_job_processed);
		ASSIGN32(cmd_op->buffer.dev_stat_op.total_jobs_rx,
				total_job_added);
		ASSIGN32(cmd_op->buffer.dev_stat_op.total_jobs_pending,
				total_job_processed);
		break;

	case RINGSTAT:
		print_debug("\t \t Ring Statistics\n");

		cmd_op = (cmd_op_t *)(mem->v_ob_mem +
				(u32)(cmd_req->cmd_op - mem->p_ob_mem));
		p_q_cursor = mem->rsrc_mem->p_q;

		while (p_q_cursor) {
			ring_cursor = p_q_cursor->ring;

			if (0 == cmd_req->ip_info.ring_id)
				ring_cursor = mem->rsrc_mem->cmdrp;
			while (ring_cursor) {
				if (cmd_req->ip_info.ring_id == (r_id = ring_cursor->id)) {
					ASSIGN32(cmd_op->buffer.ring_stat_op.depth,
							ring_cursor->depth);
#ifndef COMMON_IP_BUFFER_POOL
					ASSIGN32(cmd_op->buffer.ring_stat_op.tot_size,
							ring_cursor->depth * sizeof(app_ring_pair_t) +
							DEFAULT_POOL_SIZE);	/* TOTAL SIZE OF RING */
#else
					ASSIGN32(cmd_op->buffer.ring_stat_op.tot_size,
							ring_cursor->depth *
							sizeof(app_ring_pair_t));
#endif
					prop = (ring_cursor->props & APP_RING_PROP_PRIO_MASK)
						       >> APP_RING_PROP_PRIO_SHIFT;
					print_debug("priority : %d\n", prop);
					ASSIGN32(cmd_op->buffer.ring_stat_op.priority, prop);
					prop = (ring_cursor->props & APP_RING_PROP_AFFINE_MASK)
						       >> APP_RING_PROP_AFFINE_SHIFT;
					print_debug("affinity : %d\n", prop);
					ASSIGN32(cmd_op->buffer.ring_stat_op.affinity, prop);
					prop = (ring_cursor->props & APP_RING_PROP_ORDER_MASK)
						       >> APP_RING_PROP_ORDER_SHIFT;
					print_debug("order : %d\n", prop);
					ASSIGN32(cmd_op->buffer.ring_stat_op.order, prop);
					print_debug("Ring : %d, Job added : %d\n",
							ring_cursor->id, ring_cursor->r_s_c_cntrs->jobs_added);
					print_debug("Ring: %d, Job processed: %d\n",
							ring_cursor->id, ring_cursor->cntrs->jobs_processed);
					pending_cnt = ring_cursor->r_s_c_cntrs->jobs_added
							- ring_cursor->cntrs->jobs_processed;
					/*
					 * FREE SPACE IN RING
					 */
					ASSIGN32(cmd_op->buffer.ring_stat_op.free_count,
							ring_cursor->depth - pending_cnt);
					/*
					 * TOTAL JOBS PROCESSED BY RING
					 */
					ASSIGN32(cmd_op->buffer.ring_stat_op.jobs_processed,
							ring_cursor->cntrs->jobs_processed);
					/*
					 * TOTAL JOBS PENDING
					 */
					ASSIGN32(cmd_op->buffer.ring_stat_op.jobs_pending,
							pending_cnt);

					return 0;
				}
				ring_cursor = ring_cursor->next;
			}

			p_q_cursor = p_q_cursor->next;
		}
		break;

	case PINGDEV:
		print_debug("Ping Dev command.....\n");
		cmd_op = (cmd_op_t *)(mem->v_ob_mem +
				(u32)(cmd_req->cmd_op - mem->p_ob_mem));
		ASSIGN32(cmd_op->buffer.ping_op.resp, 556);
		break;

	case REHANDSHAKE:
		print_debug("\t\t SETTING DEVICE IN REHANDSHAKE\n");
		mem->c_hs_mem->state = DEFAULT;
		return REHANDSHAKE;
	case SECSTAT:
		print_debug("\t\t SENDING THE SEC STATISTICS\n");

		cmd_op = (cmd_op_t *)(mem->v_ob_mem +
				(u32)(cmd_req->cmd_op - mem->p_ob_mem));

		ASSIGN32(cmd_op->buffer.sec_op.sec_ver,
			 mem->rsrc_mem->sec->info->secvid_ms);
		ASSIGN32(cmd_op->buffer.sec_op.cha_ver,
			 mem->rsrc_mem->sec->info->chavid_ls);
		ASSIGN32(cmd_op->buffer.sec_op.no_of_sec_engines,
			 mem->rsrc_mem->sec_eng_cnt);
		ASSIGN32(cmd_op->buffer.sec_op.no_of_sec_jr,
			 mem->rsrc_mem->sec_eng_cnt);
		ASSIGN32(cmd_op->buffer.sec_op.jr_size,
			 mem->rsrc_mem->sec->jr.size);
		ASSIGN32(cmd_op->buffer.sec_op.no_of_sec_engines,
			 mem->rsrc_mem->sec_eng_cnt);
		for (i = 0; i < mem->rsrc_mem->sec_eng_cnt; ++i) {
			ASSIGN32(cmd_op->buffer.sec_op.sec[i].sec_tot_req_jobs,
				 mem->rsrc_mem->sec[i].tot_req_cnt);
			ASSIGN32(cmd_op->buffer.sec_op.sec[i].sec_tot_resp_jobs,
				 mem->rsrc_mem->sec[i].tot_resp_cnt);
		}

		print_debug("\t\t SEC STATISTIC SENT\n");
		break;

	case RNG_INSTANTIATE:
		copy_kek_and_set_scr(mem);
		break;

	default:
		perror("\t \t Invalid Command  !!\n");
		return 1;
	}
	return 0;

}

int cmd_ring_processing(c_mem_layout_t *mem)
{
	app_ring_pair_t *cmdrp = mem->rsrc_mem->cmdrp;
	u32 ri = cmdrp->idxs->r_index;
	u32 wi = cmdrp->idxs->w_index;
	u64 desc = cmdrp->req_r[ri].desc;
	u32 res = 0;
	va_addr_t desc_va;

	if (cmdrp->r_s_c_cntrs->jobs_added - cmdrp->cntrs->jobs_processed) {
		print_debug
			("GOT THE DESC: %0llx ---AT : %u---DEPTH OF RING: %u",
			 desc, ri, cmdrp->depth);
		desc_va = pa_to_va(desc);
		res = process_command(mem, (cmd_ring_req_desc_t *)((u32)desc_va));
	} else
		goto out;

	ri = (ri + 1) % cmdrp->depth;
	cmdrp->idxs->r_index = ri;	/* MOD_ADD(ri, 1, cmdrp->depth); */
	cmdrp->cntrs->jobs_processed += 1;

	/*
	 * Shadow counter
	 */
	ASSIGN32(cmdrp->r_s_cntrs->req_jobs_processed,
		 cmdrp->cntrs->jobs_processed);

	/*
	 * Add the response
	 */
	print_debug("SENDING DESC TO DRIVER : %0llx AT WI : %u -----[%p]\n", \
			desc, wi, &(cmdrp->resp_r[wi].desc));
	ASSIGN64(cmdrp->resp_r[wi].desc, desc);
	ASSIGN32(cmdrp->resp_r[wi].result, res);

	wi = (wi + 1) % cmdrp->depth;
	cmdrp->idxs->w_index = wi;
	cmdrp->cntrs->jobs_added += 1;

	/*
	 * Shadow counter
	 */
	ASSIGN32(cmdrp->r_s_cntrs->resp_jobs_added, cmdrp->cntrs->jobs_added);

	ASSIGN16_PTR(cmdrp->msi_addr, cmdrp->msi_data);
	print_debug("INTERRUPTED DRIVER\n");
out:
	return res;
}
#endif
#define SEL_SEC_ENQ
static u32
ring_processing(c_mem_layout_t *c_mem, u32 *deq, sec_engine_t **enq_sec,
		sec_engine_t **deq_sec)
{
	u32 cnt = 0, ring_jobs = 0, dq_cnt = 0;
	u32 cnt_one = 1;

	app_ring_pair_t *rp_head = c_mem->rsrc_mem->rps;
	app_ring_pair_t *rp = rp_head;
	drv_resp_ring_t *drv_r = c_mem->rsrc_mem->drv_resp_ring;

	u32 *r_deq_cnt = NULL;
	static u32 cnt_update;

LOOP:
	r_deq_cnt = &(rp->cntrs->jobs_processed);
	cnt = WAIT_FOR_DRIVER_JOBS(&(rp->r_s_c_cntrs->jobs_added), r_deq_cnt);

	if (!cnt)
		goto DEQ;

#ifdef SEL_SEC_ENQ
	if (cnt)
		ring_jobs = sel_sec_enqueue(c_mem, enq_sec, rp, cnt_one, deq);
	else
		ring_jobs = 0;
#else

#ifdef SEC_ENQ_RR
	ring_jobs = sec_enqueue(enq_sec, rp, (ring_jobs ? 1 : 0), deq);
#else
	ring_jobs = sec_enqueue(enq_sec, rp, cnt, deq);
#endif
#endif

	cnt -= ring_jobs;
#ifndef CMD_RING_SUPPORT
	rp = rp->next;
#endif

DEQ:
	ring_jobs = sec_dequeue(&(*deq_sec)->jr, drv_r, deq);
	(*deq_sec)->tot_resp_cnt += ring_jobs;
#ifndef CMD_RING_SUPPORT
	*deq_sec = (*deq_sec)->next;
#endif
	dq_cnt += ring_jobs;

	if (dq_cnt && !drv_r->intr_ctrl_flag) {
		raise_intr(c_mem->rsrc_mem->drv_resp_ring);

		dq_cnt = c_mem->intr_ticks = 0;
		drv_r->intr_ctrl_flag = 1;
	}

	if (drv_r->r_cntrs->jobs_added - drv_r->r_s_c_cntrs->jobs_processed){
		if (cnt_update != drv_r->r_s_c_cntrs->jobs_processed) {
			raise_intr(drv_r);
			cnt_update = drv_r->r_s_c_cntrs->jobs_processed;
		}
	}

#ifdef CMD_RING_SUPPORT
	rp = rp->next;
	*deq_sec = (*deq_sec)->next;
	if (rp_head != rp)
		goto LOOP;

#endif

#ifndef CMD_RING_SUPPORT
	/*
	 * If command ring support is not enabled then this function will
	 * run infinitely, else it returns back to the caller for
	 * command polling.
	 */
	goto LOOP;
#endif

	return 0;
}

int main(int argc, char *argv[])
{
	u8 block_app_jobs;
	u32 l2_cursor = 0;
	u32 p_cursor = 0;
	u32 res = 0;
	int i;
	int fd;
	phys_addr_t pcie_out_win_base;

	c_mem_layout_t *c_mem = NULL;
	phys_addr_t p_addr = 0;
	phys_addr_t p_aligned_addr = 0;

	u32 deq = 0;
	sec_engine_t *enq_sec = NULL;
	sec_engine_t *deq_sec = NULL;

	if (of_init()) {
		pr_err("of_init() failed");
		exit(EXIT_FAILURE);
	}

	fsl_sec_calc_eng_num();

START:
	fsl_pci_setup_law();

	l2_cursor = (u32) fsl_sram_init();
	p_cursor = l2_cursor + PLATFORM_SRAM_SIZE;
	p_cursor = ALIGN_TO_L1_CACHE_LINE_REV(p_cursor);
	p_cursor -= L1_CACHE_LINE_SIZE;

	c_mem = (c_mem_layout_t *)(p_cursor - sizeof(c_mem_layout_t));
	c_mem->free_mem = TOTAL_CARD_MEMORY;
	print_debug("c_mem :%p\n", c_mem);

	/*
	 * Allocating top cache line size number of bytes for handshake
	 */
	c_mem->c_hs_mem = (c_hs_mem_t *)p_cursor;
	p_cursor -= sizeof(c_mem_layout_t);
	c_mem->free_mem -= sizeof(c_mem_layout_t);
	print_debug("c_hs_mem:%p\n", c_mem->c_hs_mem);

	c_mem->v_ib_mem = l2_cursor;
	c_mem->p_ib_mem = va_to_pa(l2_cursor);
	print_debug("v_ib_mem: %x\n", c_mem->v_ib_mem);
	print_debug("p_ib_mem: %llx\n", c_mem->p_ib_mem);

	/*
	 * Get the PCIE1 controller  address
	 */
	fsl_pci_init(SKMM_EP_PCIe_IDX, va_to_pa(l2_cursor));
	pcie_out_win_base = fsl_pci_get_out_win_base();
	c_mem->p_pci_mem = pcie_out_win_base;
	print_debug("p_pci_mem: %llx\n", c_mem->p_pci_mem);

	p_cursor = ALIGN_TO_L1_CACHE_LINE_REV(p_cursor);

	p_cursor -= sizeof(resource_t);
	c_mem->rsrc_mem = (resource_t *) (p_cursor);
	alloc_rsrc_mem(c_mem, &l2_cursor);

	/*
	 * Init the intr time counters
	 */
	c_mem->intr_ticks = 0;
	c_mem->intr_timeout_ticks = usec2ticks(2);

	if ((argc == 3) && (strncmp(argv[1], "update-key", 10) == 0)) {
		encrypt_priv_key_to_blob(c_mem->rsrc_mem->sec, argv[2]);
		return 0;
	}

	decrypt_priv_key_from_blob(c_mem->rsrc_mem->sec);

	c_mem->c_hs_mem->state = DEFAULT;
	for (i = 0; (DEFAULT == c_mem->c_hs_mem->state); i++)
		__sync_synchronize();

	print_debug("Host ob mem l: %0x\n", c_mem->c_hs_mem->h_ob_mem_l);
	print_debug("Host ob mem h: %0x\n", c_mem->c_hs_mem->h_ob_mem_h);
	print_debug("MSI mem l: %0x\n", c_mem->c_hs_mem->h_msi_mem_l);
	print_debug("MSI mem h: %0x\n", c_mem->c_hs_mem->h_msi_mem_h);

	fd = open("/dev/mem", O_RDWR);
	if (fd < 0) {
		printf("fail to open /dev/mem\n");
		return -1;
	}

	p_addr = c_mem->c_hs_mem->h_ob_mem_h;
	p_addr = (p_addr << 32) + c_mem->c_hs_mem->h_ob_mem_l;
	p_aligned_addr = (p_addr & 0xffff00000);
	c_mem->v_ob_mem = (va_addr_t) mmap(NULL, 0x100000, PROT_READ | \
					PROT_WRITE, MAP_SHARED, fd, \
					p_aligned_addr + pcie_out_win_base);
	c_mem->p_ob_mem = pcie_out_win_base + p_aligned_addr;

	print_debug("p_ob_mem: %llx\n", c_mem->p_ob_mem);
	print_debug("v_ob_mem: %0x\n", c_mem->v_ob_mem);
	c_mem->h_hs_mem = (h_hs_mem_t *)((c_mem->v_ob_mem +
				(u32)(p_addr - p_aligned_addr)));
	print_debug("h_hs_mem %p\n", c_mem->h_hs_mem);

	p_addr = c_mem->c_hs_mem->h_msi_mem_h;
	p_addr = (p_addr << 32) + c_mem->c_hs_mem->h_msi_mem_l;
	p_aligned_addr = (p_addr & 0xffffff000);
	c_mem->v_msi_mem = (va_addr_t) mmap(NULL, 0x1000, PROT_READ |
			    PROT_WRITE, MAP_SHARED, fd,
			    p_aligned_addr + fsl_pci_get_out_win_base());
	c_mem->p_msi_mem = fsl_pci_get_out_win_base() + p_aligned_addr;

	print_debug("p_msi_mem: %0llx\n", (u64)c_mem->p_msi_mem);
	print_debug("v_msi_mem: %0llx\n", (u64)c_mem->v_msi_mem);

	c_mem->c_hs_mem->state = DEFAULT;
	handshake(c_mem, &l2_cursor);
	if ((NULL == c_mem->rsrc_mem->p_q)
	    || (NULL == c_mem->rsrc_mem->p_q->ring)) {
		perror("Nothing to process....");
		return -1;
	}

/* Separate the command ring */
	c_mem->rsrc_mem->cmdrp = c_mem->rsrc_mem->rps;
	c_mem->rsrc_mem->rps = c_mem->rsrc_mem->rps->next;
	make_rp_circ_list(c_mem);
	enq_sec = c_mem->rsrc_mem->sec;
	deq_sec = c_mem->rsrc_mem->sec;
#ifndef CMD_RING_SUPPORT
	ring_processing(c_mem, &deq, &enq_sec, &deq_sec);
#else
RING_PROCESSING:
	res = cmd_ring_processing(c_mem);
	if (0 == res)
		goto APP_RING_PROCESSING;

	if (BLOCK_APP_JOBS == res) {
		print_debug("--------> Stopped processing app jobs........\n");
		/*
		 * Wait for some timeout inorder driver to get the ACK
		 * * and process it..
		 * * Can wait for long as anyways RESET operation is in progress
		 */
		usleep(50000000ull);
		block_app_jobs = 1;
	}
	if (UNBLOCK_APP_JOBS == res) {
		print_debug("Releasing the block condition on app rings...\n");
		block_app_jobs = 0;
	}

	if (REHANDSHAKE == res) {
		print_debug("Going for rehandshake WAITING FOR FLAG TO SET\n");
		WAIT_FOR_STATE_CHANGE(c_mem->c_hs_mem->state);
		block_app_jobs = 0;
		deq = 0;
		print_debug("FLAG HAS BEEN SET GOOING TO START\n");
		goto START;
	}

APP_RING_PROCESSING:
	if (!block_app_jobs)
		ring_processing(c_mem, &deq, &enq_sec, &deq_sec);

	goto RING_PROCESSING;
#endif
	return 0;
}

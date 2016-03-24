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
#include <sys/stat.h>

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
	mem->h_hs_mem->data.device.no_secs = (u32) mem->rsrc_mem->sec_eng_cnt;

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

static app_ring_pair_t *next_ring(app_ring_pair_t *rp)
{
	rp->c_link = ((rp->c_link + 1) % rp->max_next_link);
	return rp->rp_links[rp->c_link];
}

static inline void init_order_mem(c_mem_layout_t *mem, u32 *cursor)
{
	app_ring_pair_t *rp = mem->rsrc_mem->rps;
	app_ring_pair_t *rp_head = mem->rsrc_mem->rps;

	print_debug("\t \t init_order_mem\n");
	print_debug("\t \t rp_head : %p, rp : %p\n", rp_head, rp);
	while (NULL != rp) {

		print_debug("First rp : %d\n", rp->id);
		/* Check if the rp is ordered */
		if (!((rp->props & APP_RING_PROP_ORDER_MASK)
					>> APP_RING_PROP_ORDER_SHIFT)) {

			print_debug("Order bit is not set for ring : %d\n",
					rp->id);
			goto NEXT_RP;
		}

		print_debug("Order bit is set for ring : %d\n", rp->id);
		rp->order_j_d_index = 0;
		*cursor -= (rp->depth / BITS_PER_BYTE);
		rp->resp_j_done_flag = (u8 *)*cursor;
		print_debug("Resp job done flag : %p\n", rp->resp_j_done_flag);
		memset(rp->resp_j_done_flag, 0, (rp->depth/BITS_PER_BYTE));
NEXT_RP:
		print_debug("rp_head : %p, rp : %p\n", rp_head, rp);
		rp = next_ring(rp);
		if (rp_head == rp)
			rp = NULL;
		print_debug("\t \t rp_head : %p, next rp : %p\n", rp_head, rp);
	}

}


static void init_rps(c_mem_layout_t *mem, u8 num, u8 respringcount, u32 *cursor)
{
	u8 i = 0;
	u8 j = 0;

	app_ring_pair_t *rps = mem->rsrc_mem->rps;

	print_debug("\t Init Ring Pairs	:\n");
	*cursor -=  (sizeof(indexes_mem_t) * (num+respringcount));
	mem->rsrc_mem->idxs_mem = (indexes_mem_t *)*cursor;
	memset((u8 *)mem->rsrc_mem->idxs_mem, 0,
		(sizeof(indexes_mem_t) * (num + respringcount)));
	print_debug("\t \t \t Indexes mem: %p\n", mem->rsrc_mem->idxs_mem);

	*cursor -=  (sizeof(counters_mem_t));
	mem->rsrc_mem->cntrs_mem = (counters_mem_t *)*cursor;
	memset((u8 *)mem->rsrc_mem->cntrs_mem, 0, sizeof(counters_mem_t));
	print_debug("\t \t \t Counters mem :%p\n", mem->rsrc_mem->cntrs_mem);

	*cursor -= (sizeof(counters_mem_t));
	mem->rsrc_mem->s_c_cntrs_mem = (counters_mem_t *)*cursor;
	memset((u8 *)mem->rsrc_mem->s_c_cntrs_mem, 0, sizeof(counters_mem_t));
	print_debug("\t \t \t S C Counters mem	:%p\n",
		mem->rsrc_mem->s_c_cntrs_mem);

	*cursor -= (sizeof(ring_counters_mem_t) * (num+respringcount));
	mem->rsrc_mem->r_cntrs_mem = (ring_counters_mem_t *)*cursor;
	memset((u8 *)mem->rsrc_mem->r_cntrs_mem, 0,
			(sizeof(ring_counters_mem_t) * (num + respringcount)));
	print_debug("\t \t \t R counters mem :%p\n",
			mem->rsrc_mem->r_cntrs_mem);

	*cursor -= (sizeof(ring_counters_mem_t) * (num+respringcount));
	mem->rsrc_mem->r_s_c_cntrs_mem = (ring_counters_mem_t *)*cursor;
	memset((u8 *)mem->rsrc_mem->r_s_c_cntrs_mem, 0,
			(sizeof(ring_counters_mem_t) * (num + respringcount)));
	print_debug("\t \t \t R S C counters mem :%p\n",
			mem->rsrc_mem->r_s_c_cntrs_mem);

	for (i = 0; i < num; i++) {
		print_debug
			("\t \t \t	Ring	:%d	Details....\n", i);
		rps[i].req_r = NULL;
		rps[i].msi_addr = NULL;
		rps[i].sec = NULL;
		rps[i].r_s_cntrs = NULL;

		rps[i].idxs = &(mem->rsrc_mem->idxs_mem[i]);
		rps[i].cntrs = &(mem->rsrc_mem->r_cntrs_mem[i]);
		rps[i].r_s_c_cntrs = &(mem->rsrc_mem->r_s_c_cntrs_mem[i]);
		rps[i].ip_pool = mem->rsrc_mem->ip_pool;
		rps[i].intr_ctrl_flag = 0;

		rps[i].next = NULL;

		for (j = 0; j < FSL_CRYPTO_MAX_RING_PAIRS; j++)
			rps[i].rp_links[j] = NULL;

		print_debug
			("\t \t \t \t	Idxs addr	:%p\n",
			 rps[i].idxs);
		print_debug
			("\t \t \t \t	Cntrs		:%p\n",
			 rps[i].cntrs);
		print_debug
			("\t \t \t \t	R S C cntrs	:%p\n",
			 rps[i].r_s_c_cntrs);
		print_debug
			("\t \t \t \t	Ip pool		:%p\n",
			 rps[i].ip_pool);
	}
}

static void init_drv_resp_ring(c_mem_layout_t *mem, u32 offset, u32 depth,
		u8 count, u32 *cursor)
{
	u8 loc = mem->rsrc_mem->ring_count;
	drv_resp_ring_t *ring = NULL;
	i32 i = 0;
	cursor = cursor;

	for (i = 0; i < count; i++) {
		ring = &(mem->rsrc_mem->drv_resp_ring[i]);

		print_debug("\t Init Drv Resp Ring:\n");
		ring->id             = i;
		ring->msi_data       = 0;
		ring->intr_ctrl_flag = 0;

		ring->msi_addr = NULL;
		ring->r_s_cntrs = NULL;

		ring->depth  = depth;
		ring->resp_r = (resp_ring_t *)((u8 *)mem->v_ob_mem +
				((mem->p_pci_mem + offset) - mem->p_ob_mem));

		ring->idxs    = &(mem->rsrc_mem->idxs_mem[loc + i]);
		ring->r_cntrs = &(mem->rsrc_mem->r_cntrs_mem[loc + i]);

		ring->r_s_c_cntrs = &(mem->rsrc_mem->r_s_c_cntrs_mem[loc + i]);

		print_debug("\t \t  Resp ring addr :%p\n", ring->resp_r);
		print_debug("\t \t  Indexes addr   :%p\n", ring->idxs);
		print_debug("\t \t  R Cntrs addr   :%p\n", ring->r_cntrs);
		print_debug("\t \t  R S C cntrs addr :%p\n", ring->r_s_c_cntrs);
		print_debug("\t \t  Depth            :%d\n", ring->depth);

		offset += (depth * sizeof(resp_ring_t));
	}

}

static void make_drv_resp_ring_circ_list(c_mem_layout_t *mem, u32 count)
{
	i32 i = 0;

	for (i = 1; i < count; i++) {
		mem->rsrc_mem->drv_resp_ring[i-1].next =
				&mem->rsrc_mem->drv_resp_ring[i];
	}
	mem->rsrc_mem->drv_resp_ring[i-1].next =
			&mem->rsrc_mem->drv_resp_ring[0];
}

static void init_scs(c_mem_layout_t *mem)
{
	u32 i = 0;
	app_ring_pair_t *rps = mem->rsrc_mem->rps;

	print_debug("\t \t \t Init R S mem....\n");

	for (i = 0; i < mem->rsrc_mem->ring_count; i++) {
		rps[i].r_s_cntrs = &(mem->rsrc_mem->r_s_cntrs_mem[i]);
		print_debug
			("\t \t \t \t \t Ring :%d R S Cntrs :%p\n",
			i, rps[i].r_s_cntrs);
	}

	mem->rsrc_mem->drv_resp_ring->r_s_cntrs =
			&(mem->rsrc_mem->r_s_cntrs_mem[i]);
	print_debug
		("\t \t \t \t \t Driver resp ring R S Cntrs :%p\n",
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
		while (cursor->rp_links[0])
			cursor = cursor->rp_links[0];

		cursor->rp_links[0] = rp;
	}
	rp->prio = pri;
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

static void make_rp_prio_links(c_mem_layout_t *mem)
{
	u8 i = 0;
	u8 max_pri = 0;
	priority_q_t *p_q = mem->rsrc_mem->p_q;
	priority_q_t *p_q_n = mem->rsrc_mem->p_q;
	app_ring_pair_t *r = NULL;
	app_ring_pair_t *r_next = NULL;
	app_ring_pair_t *r_head = mem->rsrc_mem->p_q->ring;

	while (NULL != p_q_n) {
		max_pri++;
		p_q_n = p_q_n->next;
	}

	while (p_q) {
		r = p_q->ring;
		p_q_n = p_q->next;
		while (r) {
			r_next = r->rp_links[0];
			if (NULL != r_next) {
				for (i = 0; i < max_pri; i++)
					r->rp_links[i] = r_next;

				r->max_next_link = max_pri;
			} else {
				r->rp_links[0] = r_head;

				for (i = 1; i < max_pri; i++) {
					if (p_q_n)
						r->rp_links[i] = p_q_n->ring;
					else
						r->rp_links[i] = r_head;
				}

				r->max_next_link = max_pri;
				max_pri--;
			}

			r = r_next;
		}
		p_q = p_q->next;
	}
	mem->rsrc_mem->rps = r_head;
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

static inline void enq_cpy(sec_ip_ring_t *sec_i, req_ring_t *req_r, u32 count)
{
	while (count--)
		*(u64 *)sec_i++ = *(u64 *)req_r++;
}

static inline void deq_cpy(resp_ring_t *resp_r, sec_op_ring_t *sec_o,
			u32 count)
{
	memcpy(resp_r, sec_o, (sizeof(resp_ring_t) * count));
}

static void make_sec_circ_list(sec_engine_t *sec, u8 count)
{
	i32 i = 0;

	for (i = 0; i < (count - 1); i++)
		sec[i].next = &(sec[i + 1]);
	sec[i].next = &sec[0];
}

static u32 init_rsrc_sec(sec_engine_t *sec, u32 *cursor)
{
	u32 p_cursor = *cursor;
	u32 mem = 0;

	sec->jr.id = sec->id;
	p_cursor = ALIGN_TO_L1_CACHE_LINE_REV(p_cursor);
	p_cursor -=
		ALIGN_TO_L1_CACHE_LINE((SEC_JR_DEPTH * sizeof(sec_ip_ring_t)));
	sec->jr.i_ring = (sec_ip_ring_t *)p_cursor;

	mem += SEC_JR_DEPTH * sizeof(sec_ip_ring_t);
	memset((u8 *)sec->jr.i_ring, 0, (SEC_JR_DEPTH * sizeof(sec_ip_ring_t)));

	print_debug("\t sec ip ring :%p\n", sec->jr.i_ring);

	p_cursor = ALIGN_TO_L1_CACHE_LINE_REV(p_cursor);
	p_cursor -= ALIGN_TO_L1_CACHE_LINE(
			(SEC_JR_DEPTH * sizeof(struct sec_op_ring)));

	sec->jr.o_ring = (struct sec_op_ring *)p_cursor;
	mem += SEC_JR_DEPTH * sizeof(struct sec_op_ring);
	memset((u8 *)sec->jr.o_ring, 0,
		(SEC_JR_DEPTH * sizeof(struct sec_op_ring)));

	print_debug("\t sec op ring	:%p\n", sec->jr.o_ring);

	/* Call for hardware init of sec engine */
	init_sec_regs_offset(sec);
	fsl_sec_init(sec);
	*cursor = p_cursor;

	return mem;
}

static void alloc_rsrc_mem(c_mem_layout_t *c_mem, u32 *pcursor, u32 *pltcursor)
{
	resource_t *rsrc  = c_mem->rsrc_mem;
	u32 plt_cursor     = *pltcursor;
	u32 p_cursor      = *pcursor;
	sec_engine_t *sec = NULL;
	i32 i            = 0;
	u32 sec_nums     = 0;
	void *local_pool;

	print_debug("\n alloc_rsrc_mem\n");
	print_debug("\t rsrc addr :%p\n", rsrc);

	local_pool = (void *)(plt_cursor - DEFAULT_EP_POOL_SIZE - HOLE_SIZE);
	memset((u8 *)rsrc, 0, sizeof(resource_t));

	sec_nums = fsl_sec_get_eng_num();
	if (!sec_nums)
		sec_nums = 1;

	rsrc->sec_eng_cnt = sec_nums;

	/* Initialize the SEC engine
	 * All the required memory for SEC engine will be allocated in L2 SRAM
	 * Max we may need = 3sec engines * (sizeof(sec_engine_t)) --
	 * Given 128 as depth of rings the max size required is
	 * approx 2624 bytes.
	 */
	p_cursor -= ALIGN_TO_L1_CACHE_LINE((sec_nums * sizeof(sec_engine_t)));
	rsrc->sec = (sec_engine_t *) (p_cursor);
	memset((u8 *)rsrc->sec, 0, sizeof(sec_engine_t) * sec_nums);
	print_debug("\t sec addr :%p\n", rsrc->sec);

	c_mem->free_mem -= sec_nums * sizeof(sec_engine_t);
	make_sec_circ_list(rsrc->sec, sec_nums);

	/* Call for hardware init of sec engine */
	sec = rsrc->sec;
	for (i = 0; i < sec_nums; i++) {
		sec->id = (i + 1);
		c_mem->free_mem -= init_rsrc_sec(sec, &p_cursor);
		sec = sec->next;
	}

#ifdef COMMON_IP_BUFFER_POOL
	rsrc->ip_pool = (void *)(plt_cursor);
	memset(rsrc->ip_pool, 0, DEFAULT_POOL_SIZE);
	plt_cursor += ALIGN_TO_L1_CACHE_LINE(DEFAULT_POOL_SIZE);
	c_mem->free_mem -= (DEFAULT_POOL_SIZE);

	print_debug("\t	ip pool addr: %p\n", rsrc->ip_pool);
	print_debug("\t free memory: %d\n", c_mem->free_mem);

	rsrc->ep_pool = local_pool;

	memset(rsrc->ep_pool, 0, DEFAULT_EP_POOL_SIZE);
	reg_mem_pool(rsrc->ep_pool, DEFAULT_EP_POOL_SIZE);

	print_debug("\t	ep pool addr: %p\n", rsrc->ep_pool);
	print_debug("\t free memory: %d\n", c_mem->free_mem);

#endif
	*pltcursor = plt_cursor;
	*pcursor  = p_cursor;
}


static inline u32 sel_sec_enqueue(c_mem_layout_t *c_mem, sec_engine_t **psec,
		app_ring_pair_t *rp,  u32 *todeq)
{
	sec_engine_t *sec = NULL;
	sec_jr_t *jr	  = NULL;
	dma_addr_t desc  = 0;
	u64 sec_sel	= 0;
	u32 secroom	= 0;
	u32 wi		= 0;
	u32 ri		= rp->idxs->r_index;
	u32 sec_cnt	= c_mem->rsrc_mem->sec_eng_cnt;

	print_debug("%s( ): rp: %d ri: %d\n", __func__,
			rp->id, rp->idxs->r_index);

	desc = rp->req_r[ri].desc;
	print_debug("%s( ): DESC: %0llx SEC number :%llx\n",
			__func__, desc, (desc & (u64) 0x03));

	sec_sel = (desc & (u64) 0x03);
	if (sec_cnt < sec_sel)
		sec_sel = 0;

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
		*psec = sec->next;
		break;
	}

	jr = &(sec->jr);
	secroom = in_be32(&(jr->regs->irsa));
	if (!secroom)
		goto RET;

	wi = jr->tail;
	rp->req_r[ri].desc = desc & ~((u64) 0x03);
	jr->i_ring[wi].desc = rp->req_r[ri].desc;

	jr->enq_cnt += 1;
	sec->tot_req_cnt += 1;

	jr->tail = MOD_ADD(wi, 1, jr->size);
	rp->idxs->r_index = MOD_ADD(ri, 1, rp->depth);

	rp->cntrs->jobs_processed += 1;
	rp->r_s_cntrs->req_jobs_processed = rp->cntrs->jobs_processed;
	out_be32(&(jr->regs->irja), 1);

RET:
	*todeq += 1;
	return 1;
}

static inline void loop_inorder(app_ring_pair_t *resp_ring)
{
	u32 flag = 0;
	u32 byte_pos = 0;
	u8  *pos_ptr = NULL;
	u32 bit_pos = 0;

	do {
		print_debug("\t\t\tOrdered job done idx:%d\n",
			resp_ring->order_j_d_index);
		/* Checking whether next ordered response bit is set  */
		byte_pos = resp_ring->order_j_d_index / BITS_PER_BYTE;
		bit_pos = resp_ring->order_j_d_index % BITS_PER_BYTE;
		pos_ptr = resp_ring->resp_j_done_flag + byte_pos;
		print_debug("\t\t\tOrdered byte pos:%d, bit pos:%d\n",
				byte_pos, bit_pos);
		print_debug("\t\t\tOrdered byte addr:%p, value:%x\n",
				pos_ptr, *pos_ptr);
		flag = 0x1 & (*pos_ptr >> (bit_pos));
		print_debug("\t\t\tFlag value : %x\n", flag);
		if (0x1 == flag) {
			*pos_ptr &= ~(1 << bit_pos);
			resp_ring->cntrs->jobs_added += 1;
			resp_ring->order_j_d_index =
				(resp_ring->order_j_d_index + 1) %
				resp_ring->depth;
		}

	} while (flag);

}

static inline void inorder_dequeue(app_ring_pair_t *resp_ring, sec_jr_t *jr,
		u32 ri, u32 wi)
{
	u8  *pos_ptr = NULL;
	u32 byte_pos = 0;
	u32 bit_pos = 0;

	/* Setting the proper bit position for this response */
	byte_pos = (wi - 1) / BITS_PER_BYTE;
	pos_ptr = resp_ring->resp_j_done_flag + byte_pos;
	bit_pos = (wi - 1) % (BITS_PER_BYTE);

	print_debug("\n \t \tJob byte pos : %d, bit pos : %d,addr : %p, value: %x\n",
			byte_pos, bit_pos, pos_ptr, *pos_ptr);
	*pos_ptr |= (1 << bit_pos);
	print_debug("\t\tAddr value after set bit : %x\n", *pos_ptr);

	memcpy(&(resp_ring->resp_r[wi - 1]),
			&jr->o_ring[ri], sizeof(resp_ring_t));
	print_debug("\t \tIndex : %d, Desc : %0llx\n",
			wi - 1, resp_ring->resp_r[wi - 1].desc);

	loop_inorder(resp_ring);

}

static inline u32 sec_dequeue(c_mem_layout_t *c_mem, sec_engine_t **deq_sec,
		u32 *todeq)
{
	sec_jr_t *jr = &(*deq_sec)->jr;
	u32 cnt = in_be32(&jr->regs->orsf);
	u32 room = 0;
	u32 wi = 0;
	u32 ri = jr->head;
	app_ring_pair_t *resp_ring = NULL;
	u32 ret_cnt = 0;

	if (!cnt) {
		*deq_sec = (*deq_sec)->next;
		return 0;
	}

	while (cnt) {
		resp_ring = (c_mem->rsrc_mem->orig_rps);

		room =	resp_ring->depth -
			(resp_ring->cntrs->jobs_added -
			resp_ring->r_s_c_cntrs->jobs_processed);
		if (!room)
			return ret_cnt;

		wi = resp_ring->idxs->w_index;
		deq_cpy(&(resp_ring->resp_r[wi]), &jr->o_ring[ri], 1);
		resp_ring->idxs->w_index =
				MOD_ADD(wi, 1, resp_ring->depth);
		resp_ring->cntrs->jobs_added += 1;

		ri = MOD_ADD(ri, 1, jr->size);
		jr->head = ri;
		jr->deq_cnt += 1;

		resp_ring->r_s_cntrs->resp_jobs_added =
			resp_ring->cntrs->jobs_added;

		out_be32(&jr->regs->orjr, 1);
		print_debug("remove one\n");
		*todeq -= 1;

		--cnt;
		++ret_cnt;
	}

	(*deq_sec)->tot_resp_cnt += ret_cnt;
	return ret_cnt;
}

static inline void raise_intr(drv_resp_ring_t *r)
{
	r->intr_ctrl_flag = 1;
	out_le16(r->msi_addr, r->msi_data);
}

#ifndef HIGH_PERF
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
				ring_counters->jobs_processed, ring_cursor->id);

			while (0 !=
			       (ring_cursor->r_s_c_cntrs->jobs_added -
				ring_counters->jobs_processed)) {
				drv_r->resp_r[wi].desc =
					ring_cursor->req_r[ri].desc;
#define JOB_DISACRDED   -1
				drv_r->resp_r[wi].result = JOB_DISACRDED;
				wi = MOD_INC(wi, drv_r->depth);
				ri = MOD_INC(ri, ring_cursor->depth);

				ring_counters->jobs_processed += 1;
				drv_r->r_cntrs->jobs_added += 1;
			}
			ring_indexes->r_index = ri;
			drv_r->idxs->w_index = wi;

			print_debug("Updated read %d and write %d", ri, wi);
			print_debug("index for ring %d", ring_cursor->id);
			print_debug("ring_counters->jobs_processed %d",
				ring_counters->jobs_processed);
			print_debug("ring_counters->jobs_added %d\n",
				ring_counters->jobs_added);

			drv_r->r_s_cntrs->resp_jobs_added =
			    drv_r->r_cntrs->jobs_added;
			s_ring_counters->req_jobs_processed =
			    ring_counters->jobs_processed;
			print_debug("\t Giving interrupt for ring :%d\n",
				ring_cursor->id);
			out_le16(ring_cursor->msi_addr, ring_cursor->msi_data);
			ring_cursor = ring_cursor->next;
			print_debug("ring_cursor : %p, ring_cursor_head : %p\n",
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
	app_ring_pair_t *ring_cursor = NULL;
	ring_counters_mem_t *ring_counters = NULL;
	app_ring_pair_t *ring_cursor_head = NULL;
	priority_q_t *p_q_cursor = mem->rsrc_mem->p_q;

	print_debug("\n Reset counters .............\n");
	mem->rsrc_mem->sec[sec_id].tot_req_cnt =
		mem->rsrc_mem->sec[sec_id].tot_resp_cnt = 0;
	mem->rsrc_mem->cntrs_mem->tot_jobs_added =
		mem->rsrc_mem->cntrs_mem->tot_jobs_processed = 0;
	for (i = 1; i < mem->rsrc_mem->ring_count; ++i) {
		ring_indexes = &(mem->rsrc_mem->idxs_mem[i]);
		ring_counters = &(mem->rsrc_mem->r_cntrs_mem[i]);
		print_debug("Updates for ring %d, ring_indexes : %p",
				i, ring_indexes);
		print_debug("ring_counters : %p\n", ring_counters);

		ring_indexes->w_index = ring_indexes->r_index = 0;
		print_debug("index update  Finished\n");

		ring_counters->jobs_added = ring_counters->jobs_processed = 0;
		print_debug("jobs_added Finished\n");
	}
	while (p_q_cursor) {
		ring_cursor_head = ring_cursor = p_q_cursor->ring;
		while (ring_cursor) {
			ring_cursor->cntrs->jobs_added = 0;
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
	print_debug("resetting counters for response ring r_cntrs %p",
		mem->rsrc_mem->drv_resp_ring->r_cntrs);
	print_debug("r_s_c_cntrs %p\n",
		mem->rsrc_mem->drv_resp_ring->r_s_c_cntrs);

	mem->rsrc_mem->drv_resp_ring->r_cntrs->jobs_added =
		mem->rsrc_mem->drv_resp_ring->r_cntrs->jobs_processed = 0;

	mem->rsrc_mem->drv_resp_ring->idxs->w_index =
		mem->rsrc_mem->drv_resp_ring->idxs->r_index = 0;
}

static int command_debug(c_mem_layout_t *mem, cmd_ring_req_desc_t *cmd_req)
{
#define CONFIG_SYS_INIT_L3_ADDR 0xffffd0000 /*not sure*/
	/* int i;*/
	cmd_op_t *cmd_op = NULL;

	print_debug("DEBUGGING IN FW\n");
	print_debug("GOT THE COMMAND : %d\n", cmd_req->ip_info.dgb.cmd_id);
	print_debug("HE ADDRESS : %u\n", cmd_req->ip_info.dgb.address);
	print_debug("GOT THE VALUE   : %x\n", cmd_req->ip_info.dgb.val);

	switch (cmd_req->ip_info.dgb.cmd_id) {
	case MD:
		cmd_op = (cmd_op_t *)((u8 *)mem->v_ob_mem +
			(cmd_req->cmd_op - mem->p_ob_mem));

		print_debug("Cmd op buffer address: %p\n", cmd_op);

		if ((cmd_req->ip_info.dgb.address < CONFIG_SYS_INIT_L3_ADDR)
			|| (cmd_req->ip_info.dgb.address >
			((CONFIG_SYS_INIT_L3_ADDR + TOTAL_CARD_MEMORY) - 1))) {

			print_debug("Trying to access a non-accessable memory");
			print_debug("return :%d\n", NON_ACCESS_MEM);

			return NON_ACCESS_MEM;
		}

		/*for (i = 0; i < 64; ++i) {
		cmd_op->buffer.debug_op[i] =
			*((u32 *)cmd_req->ip_info.dgb.address + i);

			print_debug("DUMPING AT %p : %x\n",
				((u32 *) cmd_req->ip_info.dgb.address) + i,
				*((u32 *)(cmd_req->ip_info.dgb.address) + i));
		} */

		break;

	case MW:
		if ((cmd_req->ip_info.dgb.address < CONFIG_SYS_INIT_L3_ADDR)
			|| (cmd_req->ip_info.dgb.address >
			((CONFIG_SYS_INIT_L3_ADDR + TOTAL_CARD_MEMORY) - 1))) {
			print_debug("Trying to access a non-accessable memory,");
			print_debug("return : %d\n", NON_ACCESS_MEM);
			return NON_ACCESS_MEM;
		}

		*(u32 *)(cmd_req->ip_info.dgb.address) =
					cmd_req->ip_info.dgb.val;
		print_debug("WRITING AT ADDRESS : %x\n",
				cmd_req->ip_info.dgb.address);
		break;

	case PRINT1_DEBUG:
		print_debug("DEBUG PRINTS ARE ENABLED\n");
		mem->dgb_print = cmd_req->ip_info.dgb.val;
		print_debug("DEBUG PRINTS ARE ENABLED\n");
		break;

	case PRINT1_ERROR:
		mem->err_print = cmd_req->ip_info.dgb.val;
		break;

	default:
		return 1;
	}
	return 0;
}

static void output_ring_state(cmd_op_t *cmd_op, app_ring_pair_t *ring_cursor)
{
	u32 depth = 0;
	u32 prop = 0;
	u32 pending_cnt = 0;

	cmd_op->buffer.ring_stat_op.depth = ring_cursor->depth;
	depth = ring_cursor->depth;
#ifndef COMMON_IP_BUFFER_POOL
	u32 tot_size  = 0;
	tot_size = depth * sizeof(app_ring_pair_t) + DEFAULT_POOL_SIZE;
	cmd_op->buffer.ring_stat_op.tot_size = tot_size;
#else
	cmd_op->buffer.ring_stat_op.tot_size = depth * sizeof(app_ring_pair_t);
#endif
	prop = (ring_cursor->props & APP_RING_PROP_PRIO_MASK) >>
		APP_RING_PROP_PRIO_SHIFT;
	print_debug("priority : %d\n", prop);

	cmd_op->buffer.ring_stat_op.priority = prop;
	prop = (ring_cursor->props & APP_RING_PROP_AFFINE_MASK) >>
		APP_RING_PROP_AFFINE_SHIFT;
	print_debug("affinity : %d\n", prop);

	prop = (ring_cursor->props & APP_RING_PROP_ORDER_MASK) >>
		APP_RING_PROP_ORDER_SHIFT;
	print_debug("order : %d\n", prop);

	cmd_op->buffer.ring_stat_op.order = prop;
	print_debug("Ring : %d, Job added : %d\n", ring_cursor->id,
		ring_cursor->r_s_c_cntrs->jobs_added);
	print_debug("Ring : %d, Job processed : %d\n", ring_cursor->id,
		ring_cursor->cntrs->jobs_processed);

	pending_cnt = ring_cursor->r_s_c_cntrs->jobs_added -
			ring_cursor->cntrs->jobs_processed;
	/* FREE SPACE IN RING */
	cmd_op->buffer.ring_stat_op.free_count = (depth - pending_cnt);

	/* TOT JOBS PROCESSED BY RING */
	cmd_op->buffer.ring_stat_op.jobs_processed =
			ring_cursor->cntrs->jobs_processed;

	/* TOTAL JOBS PENDING */
	cmd_op->buffer.ring_stat_op.jobs_pending = pending_cnt;

}

static int debug_ring(c_mem_layout_t *mem, cmd_ring_req_desc_t *cmd_req)
{
	cmd_op_t *cmd_op = NULL;
	priority_q_t *p_q_cursor = NULL;
	app_ring_pair_t *ring_cursor = NULL;
	u32 r_id = 0;

	print_debug("\t \t Ring Statistics\n");

	cmd_op = (cmd_op_t *)((u8 *)mem->v_ob_mem +
			(cmd_req->cmd_op - mem->p_ob_mem));

	p_q_cursor = mem->rsrc_mem->p_q;

	while (p_q_cursor) {
		ring_cursor = p_q_cursor->ring;

		if (0 == cmd_req->ip_info.ring_id)
			ring_cursor = mem->rsrc_mem->cmdrp;

		while (ring_cursor) {
			r_id = ring_cursor->id;
			if (cmd_req->ip_info.ring_id == r_id) {
				output_ring_state(cmd_op, ring_cursor);
				return 0;
			}
			ring_cursor = ring_cursor->next;
		}

		p_q_cursor = p_q_cursor->next;
	}
	return 0;
}

/*******************************************************************************
 * Function     : process_command
 *
 * Arguments    : mem: Pointer to the cards memory where all the resources start
 *                cmd_req: Dequeue command req ring desc
 *
 * Return Value : u32
 *
 * Description  : Process the command from the host driver
 *
 ******************************************************************************/
u32 process_command(c_mem_layout_t *mem, cmd_ring_req_desc_t *cmd_req)
{
	u32 i = 0;
	u32 secid;
	u32 reset_val = 0;
	u32 total_job_added = 0;
	u32 total_job_processed = 0;
	app_ring_pair_t *ring_cursor = NULL;

	cmd_op_t *cmd_op = NULL;
	sec_jr_t *sec_jr = NULL;
	u32 *rreg = NULL;

#ifdef P4080_EP_TYPE
#define RESET_REG_ADDR      0xfe0e00b0
#define RESET_REG_VALUE     0x2
#define RESET_PIC_PIR_ADDR  0xfe041090
#define RESET_PIC_PIR_VALUE 0x1
#elif C293_EP_TYPE
#define RESET_REG_ADDR      (CCSR_VIRT_ADDR + 0Xe0000 + 0xb0)
#define RESET_REG_VALUE     0x2
#define RESET_PIC_PIR_ADDR  (CCSR_VIRT_ADDR + 0x40000 + 0x1090)
#define RESET_PIC_PIR_VALUE 0x1
#endif

#define CONFIG_SYS_INIT_L3_ADDR 0xffffd0000
#ifdef P4080_EP_TYPE
	rreg = (u32 *)RESET_REG_ADDR;
	reset_val = RESET_REG_VALUE;
#elif C293_EP_TYPE
	rreg = (u32 *)RESET_PIC_PIR_ADDR;
	reset_val = RESET_PIC_PIR_VALUE;
#endif


	print_debug("   ---- Command Ring Processing ----\n");
	switch (cmd_req->cmd_type) {
	case BLOCK_APP_JOBS:
		/* This command is sent by driver to block the jobs
		 * on app rings; Driver may use this feature for
		 * smooth exits for some of the
		 * commands like RESET SEC, RESET DEV etc.
		 */

		/* Invalidate the current pending app reqs
		 * on all the request rings
		 */
		invalidate_pending_app_reqs(mem);
		/* This return value is useful for the callee
		 * to block jobs
		 */
		return BLOCK_APP_JOBS;
		break;

	case DEBUG:
		command_debug(mem, cmd_req);
		break;

	case RESETDEV:
		print_debug("\t \t Resetting Device\n");
		*rreg = reset_val;
		break;

	case RESETSEC:
		print_debug("\t \t Resetting sec engine :%d\n",
				cmd_req->ip_info.sec_id);
		sec_reset(mem->rsrc_mem->sec[cmd_req->ip_info.sec_id].info);

		sec_jr = &(mem->rsrc_mem->sec->jr);
		print_debug("\t Before SEC i/p ring ""virtual address: %p\n",
			sec_jr->i_ring);
		print_debug("\t SEC Output ring virtual address :%p\n",
			sec_jr->o_ring);

		sec_jr = &(mem->rsrc_mem->sec->jr);
		print_debug("\t After SEC Input ring virtual address:%p\n",
			sec_jr->i_ring);
		print_debug("\t SEC Output ring virtual address :%p\n",
			sec_jr->o_ring);

		resetcounters(mem, cmd_req->ip_info.sec_id);
		secid = cmd_req->ip_info.sec_id;
		sec_jr = &(mem->rsrc_mem->sec[secid].jr);
		print_debug("\t resetcounters SEC Input ring virtual address :%p\n",
			sec_jr->i_ring);
		print_debug("\t SEC Output ring virtual address :%p\n",
			sec_jr->o_ring);

		/* Driver as part of protocol would have sent the block
		 * command earlier. Since now the RESET is done, we can
		 * unblock the rings and accept the jobs.
		 */
		return UNBLOCK_APP_JOBS;
		break;

	case DEVSTAT:
		print_debug("\t \t Device stats\n");

		cmd_op = (cmd_op_t *) ((u8 *)mem->v_ob_mem +
				(cmd_req->cmd_op - mem->p_ob_mem));

		print_debug("Cmd op buffer address:%p\n", cmd_op);

		cmd_op->buffer.dev_stat_op.fwversion = FW_VERSION;
		cmd_op->buffer.dev_stat_op.totalmem = TOTAL_CARD_MEMORY;
		cmd_op->buffer.dev_stat_op.codemem = FIRMWARE_SIZE;
		cmd_op->buffer.dev_stat_op.heapmem =
			(TOTAL_CARD_MEMORY - FIRMWARE_SIZE);
		cmd_op->buffer.dev_stat_op.freemem = mem->free_mem;
		cmd_op->buffer.dev_stat_op.num_of_sec_engine =
			mem->rsrc_mem->sec_eng_cnt;
		cmd_op->buffer.dev_stat_op.no_of_app_rings =
			mem->rsrc_mem->ring_count;
		cmd_op->buffer.dev_stat_op.total_jobs_rx =
			mem->rsrc_mem->cntrs_mem->tot_jobs_added;
		cmd_op->buffer.dev_stat_op.total_jobs_pending =
			mem->rsrc_mem->cntrs_mem->tot_jobs_processed;

		for (i = 1 ; i < mem->rsrc_mem->ring_count; ++i) {
			ring_cursor = &(mem->rsrc_mem->orig_rps[i]);
			total_job_added +=
				ring_cursor->r_s_c_cntrs->jobs_added;

			total_job_processed +=
				ring_cursor->cntrs->jobs_processed;
		}

		cmd_op->buffer.dev_stat_op.total_jobs_rx =
			total_job_added;
		cmd_op->buffer.dev_stat_op.total_jobs_pending =
			total_job_processed;

		break;

	case RINGSTAT:
		debug_ring(mem, cmd_req);
		break;

	case PINGDEV:
		print_debug("Changed Ping Dev command.....\n");
		cmd_op = (cmd_op_t *) ((u8 *)mem->v_ob_mem +
					(cmd_req->cmd_op - mem->p_ob_mem));

		cmd_op->buffer.ping_op.resp = 556;
		print_debug("Resp : %d\n", cmd_op->buffer.ping_op.resp);
		print_debug("Sending Resp : %d to driver\n",
				cmd_op->buffer.ping_op.resp);
		break;

	case REHANDSHAKE:
		print_debug("\t\t SETTING DEVICE IN REHANDSHAKE\n");
		mem->c_hs_mem->state = DEFAULT;
		return REHANDSHAKE;

	case SECSTAT:
		print_debug("\t\t SENDING THE SEC STATISTICS\n");

		cmd_op = (cmd_op_t *)((u8 *)mem->v_ob_mem +
				(cmd_req->cmd_op - mem->p_ob_mem));

		cmd_op->buffer.sec_op.sec_ver =
			mem->rsrc_mem->sec->info->secvid_ms;
		cmd_op->buffer.sec_op.cha_ver =
			mem->rsrc_mem->sec->info->chavid_ls;
		cmd_op->buffer.sec_op.no_of_sec_jr = mem->rsrc_mem->sec_eng_cnt;
		cmd_op->buffer.sec_op.jr_size = mem->rsrc_mem->sec->jr.size;
		cmd_op->buffer.sec_op.no_of_sec_engines =
			mem->rsrc_mem->sec_eng_cnt;

		for (i = 0; i < mem->rsrc_mem->sec_eng_cnt; ++i) {
			cmd_op->buffer.sec_op.sec[i].sec_tot_req_jobs =
				mem->rsrc_mem->sec[i].tot_req_cnt;
			cmd_op->buffer.sec_op.sec[i].sec_tot_resp_jobs =
				mem->rsrc_mem->sec[i].tot_resp_cnt;
		}

		print_debug("\t\t SEC STATISTIC SENT\n");
		break;

	default:
		print_debug("\t \t Invalid Command !\n");
		return 1;
	}
	return 0;

}

static i32 cmd_ring_processing(c_mem_layout_t *mem)
{
	app_ring_pair_t *cmdrp = mem->rsrc_mem->cmdrp;
	u32 ri = cmdrp->idxs->r_index;
	u32 wi = cmdrp->idxs->w_index;
	u64 desc = 0;
	u32 res = 0;

	if (cmdrp->r_s_c_cntrs->jobs_added - cmdrp->cntrs->jobs_processed) {
		desc = cmdrp->req_r[ri].desc;
		print_debug("GOT THE DESC : %0llx AT: %u DEPTH OF RING: %u\n",
			desc, ri, cmdrp->depth);
		res = process_command(mem, (cmd_ring_req_desc_t *)((u32)desc));
	} else
		goto out;

	ri = (ri + 1) % cmdrp->depth;
	cmdrp->idxs->r_index = ri;	/* MOD_ADD(ri, 1, cmdrp->depth); */
	cmdrp->cntrs->jobs_processed += 1;

	/* Shadow counter */
	cmdrp->r_s_cntrs->req_jobs_processed = cmdrp->cntrs->jobs_processed;

	/* Add the response */
	print_debug("SENDING DESC TO DRIVER : %llx AT WI : %u -----[%p]\n",
		desc, wi, &(cmdrp->resp_r[wi].desc));
	cmdrp->resp_r[wi].desc = desc;
	cmdrp->resp_r[wi].result = res;
	wi = (wi + 1) % cmdrp->depth;
	cmdrp->idxs->w_index = wi;	/* MOD_ADD(wi, 1, cmdrp->depth); */
	cmdrp->cntrs->jobs_added += 1;

	/* Shadow counter */
	cmdrp->r_s_cntrs->resp_jobs_added = cmdrp->cntrs->jobs_added;

	out_le16(cmdrp->msi_addr, cmdrp->msi_data);
	print_debug("INTERRUPTED DRIVER\n");
out:
	return res;
}
#endif

static inline void raise_intr_app_ring(app_ring_pair_t *r)
{
	print_debug("%s( ): MSI addr : %p, MSI data :%0x\n",
			__func__, r->msi_addr, r->msi_data);
	r->intr_ctrl_flag = 1;
	out_le16(r->msi_addr, r->msi_data);
}

inline int enq_circ_cpy(sec_jr_t *jr, app_ring_pair_t *rp,
			app_ring_pair_t *resp, u32 *count)
{
	i32 i = 0, ri = 0, jrdepth = jr->size, rpdepth = rp->depth;
	i32 rps_wi = 0, rpsdepth = resp->depth;
	u32 cnt = *count;
	phys_addr_t desc;
	u32 sw_cnt = 0;

	print_debug("%s(): id: %d\n", __func__, jr->id);

	ri = rp->idxs->r_index;
	rps_wi = resp->idxs->w_index;

	for (i = 0; i < cnt; i++) {
		print_debug("%s():Adding desc : %llx from ri: %d"
			"to sec at wi: %d\n",
			__func__, rp->req_r[jr->tail].desc, ri, jr->tail);

		desc = parse_abs_to_desc(rp->req_r[ri].desc);
		if (desc == 0)
			break;
		if (desc == SKMM_SW_REQ) {
			if (parse_abs_to_sw_req(rp->req_r[ri].desc)) {
				resp->resp_r[rps_wi].desc = rp->req_r[ri].desc;
				resp->resp_r[rps_wi].result = 0;
				sw_cnt++;
				rps_wi = (rps_wi + 1) & ~rpsdepth;
				ri = (ri+1) & ~rpdepth;
				continue;
			}
		}

		jr->i_ring[jr->tail].desc = desc;
	print_debug("sec id: %d hw desc :%llx index: %d\n",
			jr->id, jr->i_ring[jr->tail].desc, jr->tail);
		ri = (ri+1) & ~rpdepth;
		jr->tail = MOD_INC(jr->tail, jrdepth);
	}
	rp->idxs->r_index = ri;

	*count = i;

	if (sw_cnt) {
		resp->cntrs->jobs_added  +=  sw_cnt;
		resp->r_s_cntrs->resp_jobs_added = resp->cntrs->jobs_added;
		resp->idxs->w_index = rps_wi;
		raise_intr_app_ring(resp);
	}

	print_debug("sw cnt :%d, sum cnt :%d\n", sw_cnt, i);

	return sw_cnt;
}


static inline u32 enqueue_to_sec(sec_engine_t **sec,
				app_ring_pair_t *rp,
				app_ring_pair_t *resp, u32 cnt)
{
	sec_engine_t *psec = *sec;
	u32 secroom = in_be32(&(psec->jr.regs->irsa));
	u32 room    = MIN(MIN(secroom, cnt), SEC_ENQ_BUDGET);
	u32 sw_cnt = 0;

	if (!secroom)
		goto RET;

	sw_cnt = enq_circ_cpy(&(psec->jr), rp, resp, &room);
	print_debug("hw cnt %d\n", room - sw_cnt);
	out_be32(&(psec->jr.regs->irja), (room - sw_cnt));
	rp->cntrs->jobs_processed += room;
	rp->r_s_cntrs->req_jobs_processed = rp->cntrs->jobs_processed;

RET:
	*sec = psec->next;
	return room - sw_cnt;
}

inline void ceq_circ_cpy(sec_jr_t *jr, app_ring_pair_t *r, u32 count)
{
	i32 i = 0, wi = 0, jrdepth = jr->size, rdepth = r->depth;

	print_debug("%s( ) cnt: %d id: %d\n", __func__, count, jr->id);
	wi = r->idxs->w_index;

	print_debug("%s( ): Wi: %d, ri: %d\n", __func__, wi, jr->head);
	for (i = 0; i < count; i++) {
		print_debug("%s( ): Dequeued desc : %0llx from ri: %d"
				"storing at wi: %d\n",
				__func__, jr->o_ring[jr->head].desc,
				jr->head, wi);
		print_debug("sec id: %d hw desc :%llx index: %d\n",
				jr->id, jr->o_ring[jr->head].desc, jr->head);
		r->resp_r[wi].desc  = get_abs_req(jr->o_ring[jr->head].desc);
		free_resource(jr->o_ring[jr->head].desc);
		r->resp_r[wi].result = jr->o_ring[jr->head].status;
		wi =  (wi + 1) & ~(rdepth);
		jr->head = MOD_INC(jr->head, jrdepth);
	}
	r->idxs->w_index = wi;
}

inline void resp_ring_enqueue(sec_jr_t *jr, app_ring_pair_t *r, u32 cnt)
{
	print_debug("%s( ) room: %d\n", __func__, cnt);
	ceq_circ_cpy(jr, r, cnt);
	r->cntrs->jobs_added  +=  cnt;
	r->r_s_cntrs->resp_jobs_added = r->cntrs->jobs_added;
	out_be32(&jr->regs->orjr, cnt);
}


static inline u32 dequeue_from_sec(sec_engine_t **sec, app_ring_pair_t **r)
{
	sec_engine_t    *psec = *sec;
	app_ring_pair_t *pr   = *r;
	u32 resproom    = pr->depth -
		(pr->cntrs->jobs_added - pr->r_s_c_cntrs->jobs_processed);
	u32 secroom     = in_be32(&(psec->jr.regs->orsf));
	u32 room        = MIN(MIN(secroom, resproom), MAX_DEQ_BUDGET);

	print_debug("%s( ): secroom: %d, respring:%d resproom: %d, room : %d\n",
			__func__, secroom, pr->id, resproom, room);

	if (!secroom) {
		*sec = psec->next;
		return 0;
	}

	if (!resproom) {
		*r = pr->next;
		return 0;
	}

	resp_ring_enqueue(&(psec->jr), pr, room);
	*sec = psec->next;
	*r   = pr->next;
	return room;
}

static inline void check_intr(app_ring_pair_t *r, u32 deq, u32 *processedcount)
{
#ifdef HIGH_PERF
	if (deq) {
		print_debug("%s( ): Dequeued :%d\n",
				__func__, deq);
	}
	if (deq && r->intr_ctrl_flag) {
		print_debug("%s( ): Dequeued :%d intr ctrl flag: %d\n",
				__func__, deq, r->intr_ctrl_flag);
	}
#endif
	if (deq && (!r->intr_ctrl_flag))
		raise_intr_app_ring(r);
	else {
		if (r->cntrs->jobs_added - r->r_s_c_cntrs->jobs_processed) {
			if (*processedcount != r->r_s_c_cntrs->jobs_processed) {
				*processedcount =
					r->r_s_c_cntrs->jobs_processed;
				raise_intr_app_ring(r);
			}
		}
	}
}

static inline void wait_for_timeout(u64 usecs)
{
	u64 ticks = 0;
	u64 start_ticks = 0;
	u64 timeout_ticks = 0;

	start_ticks = getticks();
	timeout_ticks = usec2ticks(usecs);
	do {
		ticks = getticks() - start_ticks;
	} while (ticks < timeout_ticks);
}

#ifdef HIGH_PERF
static void ring_processing_perf(c_mem_layout_t *c_mem)
{
	app_ring_pair_t     *recv_r  = c_mem->rsrc_mem->rps;
	app_ring_pair_t     *resp_r  = c_mem->rsrc_mem->rps;
	sec_engine_t        *enq_sec = c_mem->rsrc_mem->sec;
	sec_engine_t        *deq_sec = c_mem->rsrc_mem->sec;

	u32 enq_cnt = 0;
	u32 deq_cnt = 0;
	u32 in_flight = 0;
	u32 processedcount = 0;

	while (1) {
		enq_cnt = recv_r->r_s_c_cntrs->jobs_added -
			recv_r->cntrs->jobs_processed;

		if (enq_cnt) {
			print_debug("%s( ): Count of jobs added %d\n", __func__,
				enq_cnt);

			/* Enqueue jobs to sec engine */
			in_flight += enqueue_to_sec(&enq_sec, recv_r, resp_r,
				enq_cnt);
		}

		if (in_flight) {

			/* Dequeue jobs from sec engine */
			deq_cnt = dequeue_from_sec(&deq_sec, &resp_r);
			in_flight  -= deq_cnt;

			/* Check interrupt */
			check_intr(resp_r, deq_cnt, &processedcount);
		} else {

			if (resp_r->cntrs->jobs_added >
				resp_r->r_s_c_cntrs->jobs_processed) {

				print_debug("jobs added %d, jobs processed %d\n",
					resp_r->cntrs->jobs_added,
					resp_r->r_s_c_cntrs->jobs_processed);

				raise_intr_app_ring(resp_r);
			}

			resp_r = resp_r->next;
		}

		recv_r = recv_r->next;
	}
}

#else

static void ring_processing(c_mem_layout_t *c_mem)
{
	u32 deq = 0;
	u32 cnt = 0;
	u32 res = 0;
	u32 ring_jobs = 0;
	u32 block_req = 0;
	u32 processedcount = 0;
	u32 block_app_jobs = 0;

	sec_engine_t *enq_sec = c_mem->rsrc_mem->sec;
	sec_engine_t *deq_sec = c_mem->rsrc_mem->sec;
	app_ring_pair_t *rp = c_mem->rsrc_mem->rps;

RP_START:
	res = cmd_ring_processing(c_mem);

	if (0 == res)
		goto APP_RING;

	if (BLOCK_APP_JOBS == res) {
		print_debug("-----> Stopped processing app jobs....\n");
		/* Wait for some timeout inorder driver
		 * to get the ACK and process it..
		 * Can wait for long as anyways RESET operation is in progress..
		 */
		wait_for_timeout(50000000ull);
		block_app_jobs = 1;
	}

	if (UNBLOCK_APP_JOBS == res) {
		print_debug("Releasing the block condition on app rings...\n");
		block_app_jobs = 0;
	}

	if (REHANDSHAKE == res) {
		print_debug("Going for rehandshake.WAITING FOR FLAG TO SET\n");
		WAIT_FOR_STATE_CHANGE(c_mem->c_hs_mem->state);
		block_app_jobs = 0;
		print_debug("FLAG HAS BEEN SET GOOING TO START\n");
		return ;
	}

APP_RING:
	if (block_app_jobs)
		goto RP_START;

	if ((rp->r_s_c_cntrs->jobs_added - rp->cntrs->jobs_added) >=
		rp->depth - 1)
		block_req = 1;
	else
		block_req = 0;

	cnt =   rp->r_s_c_cntrs->jobs_added - rp->cntrs->jobs_processed;

	if ((!cnt) || (block_req))
		goto DEQ;

	sel_sec_enqueue(c_mem, &enq_sec, rp, &deq);

DEQ:
	if (!deq)
		goto NEXTRING;

	ring_jobs = sec_dequeue(c_mem, &deq_sec, &deq);

	/* CHECK INTERRUPTS */
	check_intr(rp, ring_jobs, &processedcount);

NEXTRING:
	rp = next_ring(rp);
	goto RP_START;
}
#endif

static inline void rng_processing(c_mem_layout_t *c_mem)
{
	u32 cnt = 0;
	u32 deq = 0;
	u32 ring_jobs = 0;
	app_ring_pair_t *rp = c_mem->rsrc_mem->rps;
	sec_engine_t    *sec = c_mem->rsrc_mem->sec;

	/*We need to check the jobs only in ring pair 1 because
	 *for RNG Instantiation the driver is using only the
	 *ring pair 1 to send the job.
	 */

	do {
		rp = rp->next;
	} while (1 != rp->id);

	cnt = rp->r_s_c_cntrs->jobs_added - rp->cntrs->jobs_processed;

	if (cnt)
		ring_jobs = sel_sec_enqueue(c_mem, &sec, rp, &deq);
	else
		return;

DEQ:
	ring_jobs = sec_dequeue(c_mem, &sec, &deq);
	if (!ring_jobs)
		goto DEQ;

	if (ring_jobs)
		raise_intr(c_mem->rsrc_mem->drv_resp_ring);
}

static void handshake(c_mem_layout_t *mem, u32 *cursor)
{
	u8 count = 0;
	u8 max_pri = 0;
	u8 max_rps = 0;
	u8 respr_count = 0;

	u32 rid = 0;
	u32 prio = 0;
	u32 depth = 0;
	u32 msi_addr_l = 0;
	u32 r_s_cntrs = 0;
	u32 s_cntrs = 0;
	u32 offset = 0;
	u32 resp_ring_off = 0;
	u32 req_mem_size = 0;
	u32 r_offset = 0;
	app_ring_pair_t *rp = NULL;

	print_debug("\n	HANDSHAKE\n");
	print_debug("\t State address	:%p\n", &(mem->c_hs_mem->state));

	firmware_up(mem);

	while (true) {
		WAIT_FOR_STATE_CHANGE(mem->c_hs_mem->state);
		print_debug("\t State updated by driver	:%d\n",
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
			print_debug("\t	Max pri	:%d\n", max_pri);

			*cursor = ALIGN_TO_L1_CACHE_LINE_REV(*cursor);
			*cursor -= (max_pri * sizeof(priority_q_t));
			/* Alloc memory for prio q first */
			mem->rsrc_mem->p_q = (priority_q_t *) (*cursor);
			init_p_q(mem->rsrc_mem->p_q, max_pri);
			/*
			 *
			 */
			max_rps = mem->c_hs_mem->data.config.num_of_rps;
			respr_count =
				mem->c_hs_mem->data.config.num_of_fwresp_rings;
			print_debug("\t	Max rps	%d\n", max_rps);

			mem->rsrc_mem->ring_count = max_rps;

			*cursor = ALIGN_TO_L1_CACHE_LINE_REV(*cursor);
			*cursor -= (max_rps * sizeof(app_ring_pair_t));
			mem->rsrc_mem->rps = (app_ring_pair_t *)*cursor;
			mem->rsrc_mem->orig_rps = mem->rsrc_mem->rps;
			init_rps(mem, max_rps, respr_count, cursor);

			req_mem_size = mem->c_hs_mem->data.config.req_mem_size;
			print_debug("\t	Req mem size :%d\n", req_mem_size);

			*cursor = ALIGN_TO_L1_CACHE_LINE_REV(*cursor);

			*cursor -= req_mem_size;
			mem->rsrc_mem->req_mem = (void *)*cursor;
			print_debug("\t	Req mem addr :%p\n",
				mem->rsrc_mem->req_mem);

			resp_ring_off =
				mem->c_hs_mem->data.config.fw_resp_ring;

			depth = mem->c_hs_mem->data.config.fw_resp_ring_depth;
			count = mem->c_hs_mem->data.config.num_of_fwresp_rings;
			print_debug("\t	Resp ring off	:%x\n", resp_ring_off);

			*cursor = ALIGN_TO_L1_CACHE_LINE_REV(*cursor);
			*cursor -= (count * sizeof(drv_resp_ring_t));
			*cursor -= (count * sizeof(u32 *));
			mem->rsrc_mem->drv_resp_ring_count  = count;
			mem->rsrc_mem->intr_ctrl_flags      = (u32 *)*cursor;
			*cursor += (count * sizeof(u32 *));

			mem->rsrc_mem->drv_resp_ring =
					(drv_resp_ring_t *)*cursor;

			init_drv_resp_ring(mem, resp_ring_off,
						depth, count, cursor);

			make_drv_resp_ring_circ_list(mem, count);

			s_cntrs = mem->c_hs_mem->data.config.s_cntrs;
			r_s_cntrs = mem->c_hs_mem->data.config.r_s_cntrs;
			mem->rsrc_mem->s_cntrs_mem = (shadow_counters_mem_t *)
				((u8 *)mem->v_ob_mem +
				((mem->p_pci_mem + s_cntrs) - mem->p_ob_mem));

			mem->rsrc_mem->r_s_cntrs_mem =
				(ring_shadow_counters_mem_t *)
				((u8 *)mem->v_ob_mem +
				((mem->p_pci_mem + r_s_cntrs) - mem->p_ob_mem));

			print_debug("\t Shadow counters details from Host.\n");
			print_debug("\t \t \t S CNTRS OFFSET :%x\n", s_cntrs);
			print_debug("\t \t \t R S CNTRS OFFSET :%x\n",
					r_s_cntrs);

			init_scs(mem);

			print_debug("\n- SENDING FW_INIT_CONFIG_COMPLETE -\n");
			offset = (u8 *)mem->rsrc_mem->r_s_c_cntrs_mem -
				(u8 *)mem->v_ib_mem;

			mem->h_hs_mem->data.config.s_r_cntrs = offset;
			print_debug("\t \t S R CNTRS OFFSET :%x\n", offset);

			offset = (u8 *)mem->rsrc_mem->s_c_cntrs_mem -
				(u8 *)mem->v_ib_mem;
			mem->h_hs_mem->data.config.s_cntrs = offset;
			print_debug("\t \t S CNTRS OFFSET :%x\n", offset);

			offset = (u8 *)mem->rsrc_mem->ip_pool -
				(u8 *)mem->v_ib_mem;
			mem->h_hs_mem->data.config.ip_pool = offset;

			offset = (u8 *)&(mem->rsrc_mem->drv_resp_ring->
				intr_ctrl_flag) - (u8 *)mem->v_ib_mem;

			mem->h_hs_mem->data.config.resp_intr_ctrl_flag = offset;

			mem->h_hs_mem->result = RESULT_OK;
			mem->h_hs_mem->state = FW_INIT_CONFIG_COMPLETE;

			break;

		case FW_INIT_RING_PAIR:
			mem->c_hs_mem->state = DEFAULT;
			print_debug("\n	FW_INIT_RING_PAIR\n");

			rid = mem->c_hs_mem->data.ring.rid;
			prio = (mem->c_hs_mem->data.ring.props &
				APP_RING_PROP_PRIO_MASK) >>
				APP_RING_PROP_PRIO_SHIFT;

			msi_addr_l = mem->c_hs_mem->data.ring.msi_addr_l;
			rp = &(mem->rsrc_mem->rps[rid]);

			rp->id = rid;
			rp->props = mem->c_hs_mem->data.ring.props;
			rp->depth = mem->c_hs_mem->data.ring.depth;
			rp->msi_data = mem->c_hs_mem->data.ring.msi_data;

			rp->msi_addr = (void *)(((u8 *)mem->v_msi_mem +
				((mem->p_pci_mem + msi_addr_l) -
				mem->p_msi_mem)));

			rp->req_r = mem->rsrc_mem->req_mem + r_offset;
			r_offset += (rp->depth * sizeof(req_ring_t));
			rp->resp_r = (resp_ring_t *)((u8 *) mem->v_ob_mem +
					((mem->p_pci_mem +
					mem->c_hs_mem->data.ring.resp_ring)
					- mem->p_ob_mem));

			print_debug("\t	Rid :%d\n", rid);
			print_debug("\t	Order :%d\n",
				(mem->c_hs_mem->data.ring.props &
				APP_RING_PROP_ORDER_MASK) >>
				APP_RING_PROP_ORDER_SHIFT);

			print_debug("\t	Prio	:%d\n", prio);
			print_debug("\t	Depth	:%d\n", rp->depth);
			print_debug("\t	MSI Data :%0x\n", rp->msi_data);
			print_debug("\t	MSI addr :%p\n", rp->msi_addr);
			print_debug("\t	Req r addr :%p\n", rp->req_r);
			print_debug("\t	Resp r addr :%p\n", rp->resp_r);

			add_ring_to_pq(mem->rsrc_mem->p_q, rp, (prio - 1));

			offset = 0;

			offset = (u8 *) rp->req_r - (u8 *) mem->v_ib_mem;
			mem->h_hs_mem->data.ring.req_r = offset;

			offset = (u8 *) &(rp->intr_ctrl_flag) -
					(u8 *) mem->v_ib_mem;
			mem->h_hs_mem->data.ring.intr_ctrl_flag = offset;

			mem->h_hs_mem->result = RESULT_OK;
			mem->h_hs_mem->state = FW_INIT_RING_PAIR_COMPLETE;
			break;

		case FW_HS_COMPLETE:
			mem->c_hs_mem->state = DEFAULT;
			mem->rsrc_mem->drv_resp_ring->msi_addr =
			    mem->rsrc_mem->rps[0].msi_addr;
			mem->rsrc_mem->drv_resp_ring->msi_data =
			    mem->rsrc_mem->rps[0].msi_data;

			mem->rsrc_mem->cmdrp = mem->rsrc_mem->rps;
			mem->rsrc_mem->rps = mem->rsrc_mem->rps->next;
			make_rp_prio_links(mem);
			make_rp_circ_list(mem);

			*cursor = ALIGN_TO_L1_CACHE_LINE_REV(*cursor);
			init_order_mem(mem, cursor);

			print_debug("\n	HS_COMPLETE:\n");

			if (!(in_be32(mem->rsrc_mem->sec->rdsta) & 0x1)) {
				mem->h_hs_mem->result = RESULT_OK;
				mem->h_hs_mem->state = FW_INIT_RNG;
			} else {
				mem->h_hs_mem->result = RESULT_OK;
				mem->h_hs_mem->state = FW_RNG_COMPLETE;
				return;
			}
			break;

		case FW_WAIT_FOR_RNG:
			rng_processing(mem);
			break;

		case FW_RNG_DONE:
			copy_kek_and_set_scr(mem);
			return;
		}
	}
}

extern void ecdsa_blob(void);
extern void dh_blob(void);
extern void ecdh_blob(void);
sec_engine_t *blob_sec;

int main(int argc, char *argv[])
{
	u32 l2_cursor = 0, plt_cursor;
	u32 p_cursor = 0;
	int i;
	int fd;
	phys_addr_t pcie_out_win_base;

	c_mem_layout_t *c_mem = NULL;
	phys_addr_t p_addr = 0;
	phys_addr_t p_aligned_addr = 0;
	phys_addr_t l2_sram_addr;

	if(argc < 2) {
		l2_sram_addr = L2_SRAM_ADDR;
		printf("The default l2_sram_addr is 0x%llx\n", l2_sram_addr);
	} else if (argc == 2) {
		l2_sram_addr = (phys_addr_t)strtoll(argv[1], NULL, 16);
		printf("The l2_sram_addr is 0x%llx\n", l2_sram_addr);
	} else {
		l2_sram_addr = L2_SRAM_ADDR;
		printf("Too much arguments! The default l2_sram_addr"
			" is 0x%llx\n", l2_sram_addr);
	}

	mkdir(".key", S_IRUSR | S_IWUSR);

	if (of_init()) {
		pr_err("of_init() failed");
		exit(EXIT_FAILURE);
	}

	fsl_sec_calc_eng_num();

#ifndef HIGH_PERF
START:
#endif
	fsl_pci_setup_law();

	/* l2sram size 256k platform sram size 521k,
	 * allign platform phy addr base on platform size.
	 * there is a hole between l2sram and platform sram,
	 * the size is 256k.
	 */
	l2_cursor = (u32) fsl_mem_init(l2_sram_addr);
	plt_cursor = l2_cursor + DEFAULT_EP_POOL_SIZE + HOLE_SIZE;
	p_cursor = plt_cursor + TOTAL_CARD_MEMORY;
	p_cursor = ALIGN_TO_L1_CACHE_LINE_REV(p_cursor);
	p_cursor -= L1_CACHE_LINE_SIZE;

	c_mem = (c_mem_layout_t *)(p_cursor - sizeof(c_mem_layout_t));
	c_mem->free_mem = TOTAL_CARD_MEMORY - L1_CACHE_LINE_SIZE;
	print_debug("c_mem :%p\n", c_mem);

	/*
	 * Allocating top cache line size number of bytes for handshake
	 */
	c_mem->c_hs_mem = (c_hs_mem_t *)p_cursor;
	p_cursor -= sizeof(c_mem_layout_t);
	c_mem->free_mem -= sizeof(c_mem_layout_t);
	print_debug("c_hs_mem:%p\n", c_mem->c_hs_mem);

	c_mem->v_ib_mem = plt_cursor;
	c_mem->p_ib_mem = va_to_pa(plt_cursor);
	print_debug("v_ib_mem: %x\n", c_mem->v_ib_mem);
	print_debug("p_ib_mem: %llx\n", c_mem->p_ib_mem);

	/*
	 * Get the PCIE1 controller  address
	 */
#ifdef SKMM_PCI_EP_VFIO
	fsl_pci_vfio_init(SKMM_EP_PCIe_IDX, va_to_pa(plt_cursor));
#else
	fsl_pci_init(SKMM_EP_PCIe_IDX, va_to_pa(plt_cursor));
#endif
	pcie_out_win_base = fsl_pci_get_out_win_base();
	c_mem->p_pci_mem = pcie_out_win_base;
	print_debug("p_pci_mem: %llx\n", c_mem->p_pci_mem);

	p_cursor = ALIGN_TO_L1_CACHE_LINE_REV(p_cursor);

	p_cursor -= sizeof(resource_t);
	c_mem->rsrc_mem = (resource_t *) (p_cursor);
	alloc_rsrc_mem(c_mem, &p_cursor , &plt_cursor);
	/*
	 * Init the intr time counters
	 */
	c_mem->intr_ticks = 0;
	c_mem->intr_timeout_ticks = usec2ticks(2);
	blob_sec = c_mem->rsrc_mem->sec;

	ecdsa_blob();
	dh_blob();
	ecdh_blob();


	c_mem->c_hs_mem->state = DEFAULT;
	for (i = 0; (DEFAULT == c_mem->c_hs_mem->state); i++)
		__sync_synchronize();

	print_debug("Host ob mem l: %0x\n", c_mem->c_hs_mem->h_ob_mem_l);
	print_debug("Host ob mem h: %0x\n", c_mem->c_hs_mem->h_ob_mem_h);
	print_debug("MSI mem l: %0x\n", c_mem->c_hs_mem->h_msi_mem_l);
	print_debug("MSI mem h: %0x\n", c_mem->c_hs_mem->h_msi_mem_h);

	fd = open("/dev/mem", O_RDWR);
	if (fd < 0) {
		print_debug("fail to open /dev/mem\n");
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
	handshake(c_mem, &p_cursor);
	if ((NULL == c_mem->rsrc_mem->p_q)
			|| (NULL == c_mem->rsrc_mem->p_q->ring)) {
		perror("Nothing to process....");
		return -1;
	}

	decrypt_priv_key_from_blob(c_mem->rsrc_mem->sec, BLOB_RSA);
	decrypt_priv_key_from_blob(c_mem->rsrc_mem->sec, BLOB_DSA);

#ifdef HIGH_PERF
	ring_processing_perf(c_mem);
#else
	ring_processing(c_mem);
	goto START;
#endif
	return 0;
}

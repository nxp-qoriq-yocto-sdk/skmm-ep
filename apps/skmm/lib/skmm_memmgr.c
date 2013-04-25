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

#include <common.h>
#include <skmm_memmgr.h>

/* Minimum quantum size in bytes */
#define MIN_QUANT_SIZE	64

#define INIT_BH(buf)	{((bh *)buf)->bn = NULL; }

#define INIT_BN(node, buf, len)	{ \
	node->prev_link = NULL; \
	node->next_link = NULL; \
	node->len = len - sizeof(bh); \
	node->buff = (buf + sizeof(bh)); \
	node->header = (bh *)buf; \
}

#define spin_lock_bh	pthread_spin_lock
#define spin_unlock_bh	pthread_spin_unlock
#define spin_lock_init	pthread_spin_init

static bp *pool_id;

static bh *first_fit(bp *pool, u32 len)
{
	bh *head = pool->free_list;

	while (head && (head->len < len))
		head = head->next_link;

	return head;
}

static void link_add(bp *pool, bh *node)
{
	print_debug("\n Link Add    ...........\n");

	if (!pool->free_list) {
		print_debug("\t First node to add\n");
		pool->free_list = node;
		node->next_link = node->prev_link = NULL;
		node->in_use = 0;
	} else {
		/*
		 * See if we can merge
		 */
		if (((u8 *)node + sizeof(bh)) + node->len ==
				(u8 *)pool->free_list) {
			print_debug("\t Merging the node    :%p"
					"and free list head     :%p\n",
					node, pool->free_list);
			node->len += pool->free_list->len + sizeof(bh);

			node->next_link = pool->free_list->next_link;
			if (pool->free_list->next_link)
				pool->free_list->next_link->prev_link = node;

			node->prev_link = NULL;
			pool->free_list->next_link =
				pool->free_list->prev_link = NULL;
			pool->free_list->in_use = 0;
			pool->free_list = node;
			node->in_use = 0;
			pool->tot_free_mem += sizeof(bh);
		} else {

			print_debug("\tNot merging\n");
			pool->free_list->prev_link = node;
			node->prev_link = NULL;
			node->next_link = pool->free_list;

			pool->free_list = node;
			node->in_use = 0;
		}
	}
}

static void link_after(bp *pool, bh *node, bh *after)
{
	void *n_buff = NULL;
	bh *prev = NULL;
	bh *next = NULL;


	print_debug("\n Link After  .........\n");

	/*
	 * First create the link
	 */
	if (after->next_link)
		after->next_link->prev_link = node;

	node->next_link = after->next_link;

	after->next_link = node;
	node->prev_link = after;


	/*
	 * Now see if we can merge with prev and next
	 */

	/*
	 * Check if we can merge with the prev node
	 */
	prev = node->prev_link;
	n_buff = (u8 *)node;

	if (prev)
		print_debug("Prev buff   :%p    Node buff   :%p\n",
				((u8 *)prev + prev->len + sizeof(bh)), n_buff);

	if (prev && (((u8 *)prev + prev->len + sizeof(bh)) == n_buff)) {
		print_debug("Merging with previous node ........\n");
		prev->len += node->len + sizeof(bh);
		prev->in_use = 0;
		node->in_use = 0;
		prev->next_link = node->next_link;
		if (node->next_link)
			node->next_link->prev_link = prev;

		node->next_link = node->prev_link = NULL;
		node = prev;
		pool->tot_free_mem += sizeof(bh);
	}
	/*
	 * Check if we can merge with next node
	 */
	next = node->next_link;
	n_buff = (u8 *)node + sizeof(bh);

	if (next)
		print_debug("Node buff   :%p    next buff   :%p\n",
				(n_buff + node->len), ((u8 *)next + sizeof(bh)));
	if (next && ((n_buff + node->len) == ((u8 *)next))) {
		print_debug("Merging with next node ............\n");
		node->len += next->len + sizeof(bh);
		node->in_use = 0;
		next->in_use = 0;
		if (next->next_link)
			next->next_link->prev_link = node;

		node->next_link = next->next_link;
		next->next_link = next->prev_link = NULL;
		pool->tot_free_mem += sizeof(bh);
	}

	node->in_use = 0;
}

static void link_and_merge(bp *pool, bh *node)
{
	bh *head = pool->free_list;
	bh *add_after = NULL;

	while (head) {
		if (node >= head)
			add_after = head;
		else
			break;

		head = head->next_link;
	}

	if (!add_after) {
		/*
		 * Add before the current head
		 */
		print_debug("\t \t Adding before the list head.....\n");
		link_add(pool, node);
	} else {
		/*
		 * Add after the current node
		 */
		print_debug("\t \t Adding after the node with address   :%p\n",
				add_after);
		link_after(pool, node, add_after);
	}

}

static void free_link(bp *pool, bh *node)
{
	print_debug("\t Freeing link for node       :%p\n", node);

	if (node->prev_link)
		node->prev_link->next_link = node->next_link;

	if (node->next_link)
		node->next_link->prev_link = node->prev_link;

	if (pool->free_list == node) {
		print_debug("\t List has gone completely empty.........\n");
		/*
		 * pool->free_list = NULL;
		 */
		pool->free_list = node->next_link;
	}
	node->in_use = 1;
	node->next_link = NULL;
	node->prev_link = NULL;
}

void reset_pool(void)
{
	bp *pool = pool_id;
	bh *header = NULL;

	spin_lock_bh(&pool->mem_lock);
	/*
	 * Initialise the header
	 */
	header = (bh *)pool->buff;
	header->len = pool->len - sizeof(bh);
	header->prev_link = NULL;
	header->next_link = NULL;
	header->in_use = 0;

	/*
	 * Link this header to the free list
	 */
	pool->free_list = header;
	pool->tot_free_mem = pool->len - sizeof(bh);

	spin_unlock_bh(&pool->mem_lock);
}

void *reg_mem_pool(void *buf, u32 len)
{
	bp *pool = NULL;
	bh *header = NULL;

	print_debug("\nCreating Pool\n");

	print_debug("\t Buffer  :%p\n", buf);
	print_debug("\t Len     :%d\n", len);

	if (len < MIN_QUANT_SIZE) {
		perror("Cannot register a buffer less the quant size\n");
		return NULL;
	}

	pool = malloc(sizeof(bp));
	if (!pool) {
		perror("Mem allocation for pool failed\n");
		goto error;
	}

	memset(pool, 0, sizeof(bp));

	pool->buff = buf;
	pool->len = len;

	/*
	 * Spinlock for shared access protection
	 */
	spin_lock_init(&(pool->mem_lock), 1);

	/*
	 * Changing len into multiples of quant blocks
	 */
	len &= ~(MIN_QUANT_SIZE - 1);

	/*
	 * Initialise the header
	 */
	header = (bh *)buf;
	header->len = len - sizeof(bh);
	header->prev_link = NULL;
	header->next_link = NULL;
	header->in_use = 0;

	/*
	 * Link this header to the free list
	 */
	pool->free_list = header;
	pool->tot_free_mem = len - sizeof(bh);
	print_debug("\t Total free mem  :%d\n", pool->tot_free_mem);

	print_debug("\t Creating pool done\n");

	pool_id = pool;
	return pool;

error:
	free(pool);

	return NULL;
}

void *get_buffer(u32 len)
{
	bp *pool = pool_id;
	bh *f_node = pool->free_list;
	bh *a_node = NULL;
	bh *new_node = NULL;

	print_debug("\nAllocating buffer\n");

	spin_lock_bh(&(pool->mem_lock));

	f_node = pool->free_list;
	if (!f_node) {
		perror("No free buffers to allocate ...  ");
		printf("Avail mem: %d\n", pool->tot_free_mem);
		spin_unlock_bh(&(pool->mem_lock));
		return NULL;
	}

	/*
	 * If the requested length does not fit to overall available free mem
	 */

	if (len > pool->tot_free_mem) {
		print_debug("Not enough space ...  asked: %d Left: %d\n",
				len, pool->tot_free_mem);
		spin_unlock_bh(&(pool->mem_lock));
		return NULL;
	}

	f_node = first_fit(pool, len);

	if (!f_node) {
		perror("No free node has mem ...");
		printf("asked :%d tot mem avail :%d\n",	len, pool->tot_free_mem);
		goto error;
	}

	if (f_node->in_use == 1) {
		perror("Free nod is in use....\n");
		spin_unlock_bh(&(pool->mem_lock));
		return NULL;
	}

	if (len == f_node->len
			|| ((f_node->len - len) < (MIN_QUANT_SIZE + sizeof(bh)))) {
		print_debug("Giving the free node itself... Asked len   :%d,"
				"f node len :%d\n",  len, f_node->len);

		a_node = f_node;
		free_link(pool, f_node);

		pool->tot_free_mem -= f_node->len;
	} else {
		print_debug("f_node is bigger than what has been asked..."
				"Asked :%d, f node len :%d\n",
				len, f_node->len);

		new_node = (bh *)((u8 *)f_node + sizeof(bh) + len);
		new_node->len = f_node->len - len - sizeof(bh);
		new_node->in_use = 0;

		f_node->len = len;
		a_node = f_node;

		new_node->prev_link = f_node->prev_link;
		if (f_node->prev_link)
			f_node->prev_link->next_link = new_node;
		new_node->next_link = f_node->next_link;
		if (f_node->next_link)
			f_node->next_link->prev_link = new_node;

		if (pool->free_list == f_node)
			pool->free_list = new_node;

		f_node->next_link = f_node->prev_link = NULL;
		f_node->in_use = 1;

		pool->tot_free_mem -= (len + sizeof(bh));
	}
	spin_unlock_bh(&(pool->mem_lock));

	return (u8 *)a_node + sizeof(bh);
error:
	spin_unlock_bh(&(pool->mem_lock));
	return NULL;
}

void put_buffer(void *buffer)
{
	bp *pool = pool_id;
	bh *header = NULL;

	if (!buffer)
		return;

	print_debug("\n Free Buffer\n");

	spin_lock_bh(&(pool->mem_lock));

	print_debug("\t Buffer  :%p\n", buffer);
	header = (bh *)(buffer - sizeof(bh));

	if (header->in_use == 0)
		goto out;

	print_debug("\t Header  :%p\n", header);

	pool->tot_free_mem += header->len;
	header->in_use = 0;
	/*
	 * Link the free node to the list of free nodes and merge if possible
	 */
	link_and_merge(pool, header);
out:
	spin_unlock_bh(&(pool->mem_lock));

	print_debug("\t Free buffer done !!!!\n");
}

void store_priv_data(void *id, void *buffer, unsigned long priv)
{
	bh *header = NULL;
	bp *pool = pool_id;

	spin_lock_bh(&(pool->mem_lock));
	header = (buffer - sizeof(bh));
	header->priv = priv;
	spin_unlock_bh(&(pool->mem_lock));
}

unsigned long get_priv_data(void *id, void *buffer)
{
	bh *header = NULL;
	bp *pool = pool_id;
	unsigned long priv = 0;

	spin_lock_bh(&(pool->mem_lock));
	header = (buffer - sizeof(bh));
	priv = header->priv;
	spin_unlock_bh(&(pool->mem_lock));

	return priv;
}

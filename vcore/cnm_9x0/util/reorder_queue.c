/*
 * SIC LABORATORY, LG ELECTRONICS INC., SEOUL, KOREA
 * Copyright(c) 2013 by LG Electronics Inc.
 * Youngki Lyu <youngki.lyu@lge.com>
 * Jungmin Park <jungmin016.park@lge.com>
 * Younghyun Jo <younghyun.jo@lge.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/vmalloc.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/timer.h>

#include <media/odin/vcodec/vlog.h>

#include "vcore/cnm_9x0/util/reorder_queue.h"
#include "vcore/cnm_9x0/cnm_ddk/vdi/vdi_osal.h"


#define MAX_LIST_NUM 32

struct decode_node
{
	unsigned long long timestamp;
	struct list_head list;
};

struct reorder_inst
{
	unsigned int num_of_fb;
	unsigned int ref_frame_cnt;

	struct list_head decode_order_free_list;
	struct list_head decode_order_decoded_list;
	unsigned int decode_list_num;

	struct reorder_meta display_order_queue[MAX_LIST_NUM];
	unsigned int alive_node_bitmask;
	unsigned int display_list_num;

	spinlock_t reorder_lock;

	struct decode_node dec_node_local;
	struct reorder_meta disp_node_local;
};

static vcore_bool_t _push_decode_order_queue(struct reorder_inst *handle,
												struct decode_node *dec_node)
{
	struct decode_node *node;

	if (list_empty( &handle->decode_order_free_list))
	{
		vlog_error("list is empty inst(%X)\n", (unsigned int)handle);
		return VCORE_FALSE;
	}

	node = list_entry(handle->decode_order_free_list.next,
					struct decode_node,
					list );
	list_del(&node->list);

	node->timestamp = dec_node->timestamp;
	list_add_tail(&node->list, &handle->decode_order_decoded_list);

	handle->decode_list_num++;

	return VCORE_TRUE;
}

static vcore_bool_t  _pop_decode_order_queue(struct reorder_inst *handle,
												struct decode_node* dec_node,
												vcore_bool_t reuse)
{
	struct decode_node *node;

	if (list_empty( &handle->decode_order_decoded_list))
	{
		vlog_error("list is empty \n");
		return VCORE_FALSE;
	}

	node = list_entry(handle->decode_order_decoded_list.next,
					struct decode_node,
					list );
	list_del( &node->list );

	dec_node->timestamp = node->timestamp;
	list_add_tail(&node->list, &handle->decode_order_free_list);

	if (reuse == VCORE_FALSE)
		handle->decode_list_num--;

	return VCORE_TRUE;
}

static vcore_bool_t _push_display_order_queue(struct reorder_inst *handle,
												int decoded_idx,
												struct reorder_meta *disp_node)
{
	if (handle->alive_node_bitmask & (1 << decoded_idx)) {
		vlog_error("decoded index(%d) frame already existed - alive:0x%X\n",
					decoded_idx, handle->alive_node_bitmask);
	}
	else {
		handle->alive_node_bitmask |= (1 << decoded_idx);
		handle->display_list_num++;
	}

	osal_memcpy(&handle->display_order_queue[decoded_idx],
					disp_node,
					sizeof(struct reorder_meta));

	return VCORE_TRUE;
}

static vcore_bool_t _pop_display_order_queue(struct reorder_inst *handle,
												int display_idx,
												struct reorder_meta *disp_node)
{
	if (handle->alive_node_bitmask & (1 << display_idx)) {
		handle->alive_node_bitmask &= ~(1 << display_idx);
		handle->display_list_num--;
	}
	else {
		vlog_error("display index(%d) frame not existed - alive:0x%X\n",
					display_idx, handle->alive_node_bitmask);
	}

	osal_memcpy(disp_node,
					&handle->display_order_queue[display_idx],
					sizeof(struct reorder_meta) );

	return VCORE_TRUE;
}

void reorder_queue_flush(void *reorder_id)
{
	struct reorder_inst *handle = (struct reorder_inst *)reorder_id;
	int i = 0;
	unsigned long flags;

	if (reorder_id == NULL) {
		vlog_error("invalid reorder_id(0x%08X)\n", (unsigned int)reorder_id);
		return;
	}

	vdi_spin_lock(&handle->reorder_lock, &flags);

	if (handle->decode_list_num != handle->display_list_num)
		vlog_error("dec_list (%u), disp_list (%u:0x%X)\n",
					handle->decode_list_num,
					handle->display_list_num, handle->alive_node_bitmask);

	while (handle->decode_list_num)
	{
		_pop_decode_order_queue(handle, &handle->dec_node_local, VCORE_FALSE);

		i++;
		if (i > handle->num_of_fb) {
			vlog_error("dec_list (%u), disp_list (%u:0x%X)\n",
						handle->decode_list_num,
						handle->display_list_num, handle->alive_node_bitmask);
			break;
		}
	}

	osal_memset(handle->display_order_queue,
				0x0,
				sizeof(struct reorder_meta)*MAX_LIST_NUM);
	handle->display_list_num = 0;
	handle->alive_node_bitmask = 0x0;

	vdi_spin_unlock(&handle->reorder_lock, &flags);
}

void* reorder_queue_init(unsigned int num_of_fb, unsigned int ref_frame_cnt)
{
	int i;
	struct decode_node *dec_node;
	struct reorder_inst *handle;

	handle = (struct reorder_inst *)osal_malloc( sizeof(struct reorder_inst) );
	if (!handle) {
		vlog_error("failed to allocate memory\n");
		return NULL;
	}

	handle->num_of_fb = num_of_fb;
	handle->ref_frame_cnt = ref_frame_cnt;

	INIT_LIST_HEAD(&handle->decode_order_decoded_list);
	INIT_LIST_HEAD(&handle->decode_order_free_list);

	for (i = 0; i < MAX_LIST_NUM; i++) {
		dec_node = \
			(struct decode_node *)osal_malloc( sizeof(struct decode_node) );
		if (!dec_node) {
			vlog_error("failed to allocate memory\n");
			osal_free(handle);
			return NULL;
		}
		list_add_tail( &dec_node->list, &handle->decode_order_free_list );
	}
	handle->decode_list_num = 0;

	handle->alive_node_bitmask = 0x0;
	handle->display_list_num = 0;

	vdi_spin_lock_init(&handle->reorder_lock);

	return (void*)handle;
}

void reorder_queue_deinit(void *reorder_id)
{
	struct reorder_inst *handle = (struct reorder_inst *)reorder_id;
	struct decode_node *dec_node, *tmp;

	if (reorder_id == NULL) {
		vlog_error("invalid reorder_id(0x%08X)\n", (unsigned int)reorder_id);
		return;
	}

	list_for_each_entry_safe( dec_node,
						tmp,
						&handle->decode_order_free_list,
						list)
	{
		list_del(&dec_node->list);
		osal_free(dec_node);
	}

	list_for_each_entry_safe( dec_node,
						tmp,
						&handle->decode_order_decoded_list,
						list )
	{
		list_del(&dec_node->list);
		osal_free(dec_node);
	}

	osal_free(handle);
}

vcore_bool_t reorder_enqueue(void *reorder_id,
								int decoded_idx,
								struct reorder_meta* meta)
{
	struct reorder_inst *handle = (struct reorder_inst *)reorder_id;
	unsigned long flags;
	vcore_bool_t ret = VCORE_TRUE;

	if (reorder_id == NULL) {
		vlog_error("invalid reorder_id(0x%08X)\n", (unsigned int)reorder_id);
		return VCORE_FALSE;
	}

	if ((decoded_idx < 0) || (decoded_idx >= handle->num_of_fb)) {
		vlog_error("invalid decode frame index (%d)\n", decoded_idx);
		return VCORE_FALSE;
	}

	vdi_spin_lock(&handle->reorder_lock, &flags);

	handle->dec_node_local.timestamp = meta->au_meta.timestamp;
	if (_push_decode_order_queue(handle,
								&handle->dec_node_local) == VCORE_FALSE) {
		vlog_error("failed to push decode order queue (%d)\n", decoded_idx);
		vdi_spin_unlock(&handle->reorder_lock, &flags);
		return VCORE_FALSE;
	}

	osal_memcpy(&handle->disp_node_local, meta, sizeof(struct reorder_meta));
	_push_display_order_queue(handle, decoded_idx, &handle->disp_node_local);

	/* vlog_info("handle:0x%X, dec_list (%u), disp_list (%u) - \
				decode frame index (%d)\n",
				(unsigned int)handle,
				handle->decode_list_num,
				handle->display_list_num, decoded_idx); */

	/* validate num of node */
	if (handle->decode_list_num != handle->display_list_num)
	{
		vlog_error("dec_list (%u), disp_list (%u:0x%X) - \
					decode frame index (%d)\n",
					handle->decode_list_num,
					handle->display_list_num, handle->alive_node_bitmask,
					decoded_idx);

		while (handle->decode_list_num > handle->display_list_num)
		{
			_pop_decode_order_queue(handle,
										&handle->dec_node_local,
										VCORE_FALSE);
			vlog_error("dec_list (%d)\n", handle->decode_list_num);
		}

		ret = VCORE_FALSE;
	}

	if (handle->display_list_num > handle->ref_frame_cnt) {
		vlog_error("overflow reordering queue (%u), \
					reference frame count (%u)\n",
					handle->display_list_num,
					handle->ref_frame_cnt);
	}

	vdi_spin_unlock(&handle->reorder_lock, &flags);

	return ret;
}

vcore_bool_t reorder_dequeue(void *reorder_id,
								int display_idx,
								struct reorder_meta* meta)
{
	struct reorder_inst *handle = (struct reorder_inst *)reorder_id;
	unsigned long flags;
	vcore_bool_t ret = VCORE_TRUE;
	vcore_bool_t reuse = VCORE_FALSE;

	if (reorder_id == NULL) {
		vlog_error("invalid reorder_id(0x%08X)\n", (unsigned int)reorder_id);
		return VCORE_FALSE;
	}

	if ((display_idx < 0) || (display_idx >= handle->num_of_fb)) {
		vlog_error("invalid display frame index (%d)\n", display_idx);
		return VCORE_FALSE;
	}

	vdi_spin_lock(&handle->reorder_lock, &flags);

	/* validate num of node */
	if (handle->decode_list_num != handle->display_list_num)
	{
		vlog_error("dec_list (%u), disp_list (%u:0x%X)\n",
					handle->decode_list_num,
					handle->display_list_num, handle->alive_node_bitmask);

		if (handle->decode_list_num < handle->display_list_num)
			reuse = VCORE_TRUE;

		ret = VCORE_FALSE;
	}

	if (_pop_decode_order_queue(handle, \
							&handle->dec_node_local, reuse) == VCORE_FALSE) {
		vlog_error("failed to pop decode order queue (%d)\n", display_idx);
		vdi_spin_unlock(&handle->reorder_lock, &flags);
		return VCORE_FALSE;
	}

	_pop_display_order_queue(handle, display_idx, &handle->disp_node_local);
	osal_memcpy(meta, &handle->disp_node_local, sizeof(struct reorder_meta));

	/* vlog_info("handle:0x%X, dec_list (%u), disp_list (%u) - \
				display frame index (%d)\n",
				(unsigned int)handle,
				handle->decode_list_num,
				handle->display_list_num, display_idx); */

	vdi_spin_unlock(&handle->reorder_lock, &flags);

	return ret;
}

void reorder_debug_print(void *reorder_id)
{
	struct reorder_inst *handle = (struct reorder_inst *)reorder_id;
	unsigned long flags;

	if (reorder_id == NULL) {
		vlog_error("invalid reorder_id(0x%08X)\n", (unsigned int)reorder_id);
		return ;
	}

	vdi_spin_lock(&handle->reorder_lock, &flags);

	vlog_info("dec_list (%u), disp_list (%u)\n",
				handle->decode_list_num, handle->display_list_num);

	vdi_spin_unlock(&handle->reorder_lock, &flags);
}


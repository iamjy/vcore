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

#ifndef __REORDER_Q_H__
#define __REORDER_Q_H__

#include <media/odin/vcodec/vcore/decoder.h>
#include "vcore/cnm_9x0/cnm_ddk/vpuapi/vpuapi.h"

struct reorder_meta
{
	DecOutputInfo dec_info;
	struct vcore_dec_au_meta au_meta;
};

void* reorder_queue_init(unsigned int num_of_fb, unsigned int ref_frame_cnt);
void reorder_queue_deinit(void *reorder_id);
vcore_bool_t reorder_dequeue(void *reorder_id, int display_idx,
								struct reorder_meta* meta);
vcore_bool_t reorder_enqueue(void *reorder_id, int decoded_idx,
								struct reorder_meta* meta);
void reorder_debug_print(void *reorder_id);
void reorder_queue_flush(void *reorder_id);

#endif /* #ifndef __REORDER_Q_H__ */


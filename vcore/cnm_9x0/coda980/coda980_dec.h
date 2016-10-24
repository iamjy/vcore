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

#ifndef _CODA980_DEC_H_
#define _CODA980_DEC_H_

#include "media/odin/vcodec/vcore/decoder.h"

void coda980_dec_init(struct vcore_dec_ops *ops);
void coda980_dec_report_reset(void);
void coda980_dec_broadcast(void *cnm_id, void *vcore_id);

#endif /* #ifndef _CODA980_DEC_H_ */


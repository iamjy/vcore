/*
 * SIC LABORATORY, LG ELECTRONICS INC., SEOUL, KOREA
 * Copyright(c) 2013 by LG Electronics Inc.
 * Youngki Lyu <youngki.lyu@lge.com>
 * Jungmin Park <jungmin016.park@lge.com>
 * Younghyun Jo <younghyun.jo@lge.com>
 * Seokhoon Kang <m4seokhoon.kang@lgepartner.com>
 * Inpyo Cho <inpyo.cho@lge.com>
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

#ifndef _CODA980_H_
#define _CODA980_H_

void* coda980_get_cnm_core_instance(void);

void (*coda980_init(unsigned int reg_base, struct device *_dev))(void);
int coda980_runtime_resume(void);
int coda980_runtime_suspend(void);
int coda980_resume(void);
int coda980_suspend(void);
void coda980_broadcast(void *vcore_id);
#endif /* #ifndef _CODA980_H_ */


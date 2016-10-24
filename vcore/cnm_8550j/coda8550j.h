/*
 * SIC LABORATORY, LG ELECTRONICS INC., SEOUL, KOREA
 * Copyright(c) 2013 by LG Electronics Inc.
 * Youngki Lyu <youngki.lyu@lge.com>
 * Jungmin Park <jungmin016.park@lge.com>
 * Younghyun Jo <younghyun.jo@lge.com>
 * Seokhoon.Kang <m4seokhoon.kang@lgepartner.com>
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

#ifndef _CODA8550J_H_
#define _CODA8550J_H_

extern void (*coda8550j_init(unsigned int reg_base, struct device *_dev))(void);
extern void coda8550j_runtime_resume(void);
extern void  coda8550j_runtime_suspend(void);
extern void coda8550j_reserve_running_weight(unsigned int width, unsigned int height, 
		unsigned int frame_rate_residual, unsigned int frame_rate_divider);
extern void coda8550j_unreserve_running_weight(unsigned int width, unsigned int height,
		unsigned int frame_rate_residual, unsigned int frame_rate_divider);
extern int coda8550j_ch_lock(void *ch, void (*isr_func)(void*, unsigned long));
extern int coda8550j_ch_unlock(void *ch);
extern int coda8550j_ch_is_locked(void *ch);
extern void coda8550j_update_run_time(int run);
extern void  coda8550j_clock_on(void);
extern void  coda8550j_clock_off(void);

#endif //#ifndef _CODA8550J_H_
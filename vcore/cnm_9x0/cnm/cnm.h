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

#ifndef _CNM_H_
#define _CNM_H_

enum core_id
{
	CODA980_COREID,
	BODA950_COREID,
};

void *cnm_init(unsigned long coreid,
				unsigned int reg_base,
				unsigned short *firmware, unsigned int firmware_size,
				unsigned int default_running_weight,
				void (*cb_broadcast)(void *last_vcore_id),
				void (*cb_report_reset)(void),
				void (*vcore_clock_on)(void),
				void (*vcore_clock_off)(void));
void cnm_cleanup(void* cnm_id);

int cnm_ch_lock(void *cnm_id, void *ch, void (*isr_func)(void*, unsigned long));
int cnm_ch_unlock(void *cnm_id, void *ch);
int cnm_ch_is_locked(void *cnm_id, void *ch);
void cnm_spin_lock(void *cnm_id);
void cnm_spin_unlock(void *cnm_id);

int cnm_clock_on(void *cnm_id);
int cnm_clock_off(void *cnm_id);

unsigned int cnm_reserve_running_weight(void *cnm_id,
		unsigned int width, unsigned int height,
		unsigned int frame_rate_residual, unsigned int frame_rate_divider);
void cnm_unreserve_running_weight(void *cnm_id, unsigned int running_weight);
unsigned int cnm_get_running_weight(void *cnm_id);

void cnm_update_run_time(void *cnm_id, int run);
unsigned long cnm_get_short_utilization(void *cnm_id);
unsigned long cnm_get_long_utilization(void *cnm_id);

void cnm_isr(void *cnm_id);
void cnm_resume(void *cnm_id);
void cnm_suspend(void* cnm_id);
void cnm_reset(void *cnm_id, void *handle);
unsigned int cnm_get_coreid(void *cnm_id);

#endif /* #ifndef _CNM_H_ */


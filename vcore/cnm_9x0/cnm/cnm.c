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

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/err.h>
#include <linux/ion.h>
#include <linux/odin_iommu.h>

#include <media/odin/vcodec/vlog.h>

#include "vcore/cnm_9x0/cnm_ddk/vdi/vdi.h"
#include "vcore/cnm_9x0/cnm_ddk/vpuapi/vpuapi.h"

#include "cnm.h"

#define	VCORE_UTILIZATION_LOG_CNT					0x80
#define	VCORE_UTILIZATION_MEASURE_CNT				0x10
#define	VCORE_UTILIZATION_MOVING_AVERAGE_BUF		\
		((VCORE_UTILIZATION_LOG_CNT > VCORE_UTILIZATION_MEASURE_CNT) ? \
		VCORE_UTILIZATION_LOG_CNT : VCORE_UTILIZATION_MEASURE_CNT)

struct cnm_core
{
	unsigned long coreid;

	spinlock_t lock;
	unsigned long flags;

	void *locked_ch;
	void (*locked_isr_func)(void *vcore_id, unsigned long reason);

	void (*cb_broadcast)(void *vcore_id);
	void (*cb_report_reset)(void);

	void (*vcore_clock_on)(void);
	void (*vcore_clock_off)(void);
	int vcore_clock_state;

	unsigned short *firmware;
	unsigned int firmware_size;

	unsigned int running_instance;
	unsigned int running_weight;

	unsigned long reg_paddr;
	unsigned long *reg_vaddr;
	struct {
		struct ion_client *client;

		struct {
			struct ion_handle *handle;
			unsigned long paddr;
			unsigned long *vaddr;
			unsigned int size;
		}common;
	}ion;

	struct {
		unsigned long prev_time;

		struct {
			int run;
			unsigned long duration_time;
		} update_duration[VCORE_UTILIZATION_MOVING_AVERAGE_BUF];
		unsigned int update_index;

		unsigned int update_cnt;
		/* short */
		unsigned long run_time_for_short;
		unsigned long total_time_for_short;
		/* long */
		unsigned long run_time_for_long;
		unsigned long total_time_for_long;

		unsigned int measure_cnt;
	} usage;
};

bool _cnm_alloc_common_memory(struct cnm_core *cnm_core)
{
	char ion_client_name[80];

	sprintf(ion_client_name, "cnm core%ld", cnm_core->coreid);
	cnm_core->ion.client = odin_ion_client_create(ion_client_name);

	cnm_core->ion.common.size = SIZE_COMMON;
	cnm_core->ion.common.handle = ion_alloc(cnm_core->ion.client,
											cnm_core->ion.common.size,
											SZ_2M,
											(1<<ODIN_ION_SYSTEM_HEAP1),
											0,
											ODIN_SUBSYS_VSP);
	if (IS_ERR_OR_NULL(cnm_core->ion.common.handle)) {
		vlog_error("ion_alloc failed\n");
		goto err_alloc;
	}

	cnm_core->ion.common.paddr = (unsigned long)odin_ion_get_iova_of_buffer(
										cnm_core->ion.common.handle,
										ODIN_SUBSYS_VSP);
	if (cnm_core->ion.common.paddr == 0) {
		vlog_error("odin_ion_get_iova_of_buffer_failed\n");
		goto err_iova;
	}

	cnm_core->ion.common.vaddr = (unsigned long *)ion_map_kernel(
										cnm_core->ion.client,
										cnm_core->ion.common.handle);
	if (cnm_core->ion.common.vaddr == 0) {
		vlog_error("ion_map_kernel failed\n");
		goto err_iova;
	}

	cnm_core->reg_vaddr = (unsigned long*)ioremap(cnm_core->reg_paddr, 0x4000);
	if (cnm_core->reg_vaddr == 0) {
		vlog_error("reg vaddr is 0\n");
		goto err_ioremap;
	}

	return true;

err_ioremap:
	ion_unmap_kernel(cnm_core->ion.client, cnm_core->ion.common.handle);
err_iova:
	ion_free(cnm_core->ion.client, cnm_core->ion.common.handle);
err_alloc:
	odin_ion_client_destroy(cnm_core->ion.client);

	return false;
}

bool _cnm_free_common_memory(struct cnm_core *cnm_core)
{
	if (cnm_core->reg_vaddr == 0 ||
		cnm_core->ion.common.vaddr == 0 ||
		cnm_core->ion.common.handle == NULL ||
		cnm_core->ion.client == NULL) {
		vlog_error("invalid status : %lu %lu %u %u\n",
				(unsigned long)cnm_core->reg_vaddr,
				(unsigned long)cnm_core->ion.common.vaddr,
				(unsigned int)cnm_core->ion.common.handle,
				(unsigned int)cnm_core->ion.client);
		return false;
	}

	iounmap((void *)cnm_core->reg_vaddr);
	ion_unmap_kernel(cnm_core->ion.client, cnm_core->ion.common.handle);
	ion_free(cnm_core->ion.client, cnm_core->ion.common.handle);
	odin_ion_client_destroy(cnm_core->ion.client);

	return true;
}

void _cnm_get_fw_version(void* cnm_id)
{
	RetCode ret = RETCODE_SUCCESS;
	unsigned int version_info, revision, product_id;
	struct cnm_core *cnm_core;

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	ret = VPU_GetVersionInfo(cnm_core->coreid, &version_info, &revision,
								&product_id);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_GetVersionInfo failed. ret : %d\n", ret);
		return;
	}

	vlog_print(VLOG_VCORE_INFO, "VPU coreNum : [%d]\n", (int)cnm_core->coreid);
	vlog_print(VLOG_VCORE_INFO, "Firmware Version => projectId : %x | version : %04d.%04d.%08d\n",
		    (Uint32)(version_info>>16), (Uint32)((version_info>>(12))&0x0f),
		    (Uint32)((version_info>>(8))&0x0f), (Uint32)((version_info)&0xff));
	vlog_print(VLOG_VCORE_INFO, "revision : r%d\n", revision);
	vlog_print(VLOG_VCORE_INFO, "Hardware Version => %04x\n", product_id);
	vlog_print(VLOG_VCORE_INFO, "API Version => %d\n\n", API_VERSION);
}

void* cnm_init(unsigned long coreid,
				unsigned int reg_base,
				unsigned short *firmware, unsigned int firmware_size,
				unsigned int default_running_weight,
				void (*cb_broadcast)(void *last_vcore_id),
				void (*cb_report_reset)(void),
				void (*vcore_clock_on)(void),
				void (*vcore_clock_off)(void))
{
	struct cnm_core *cnm_core;

	cnm_core =  vmalloc(sizeof(struct cnm_core));
	if (cnm_core == NULL)
	{
		vlog_error("vmalloc failed\n");
		return NULL;
	}

	cnm_core->running_instance = 0;
	cnm_core->running_weight = default_running_weight;
	cnm_core->locked_ch = NULL;
	cnm_core->locked_isr_func = NULL;
	cnm_core->reg_paddr = reg_base;
	cnm_core->coreid = coreid;
	cnm_core->firmware = firmware;
	cnm_core->firmware_size = firmware_size;
	cnm_core->cb_report_reset = cb_report_reset;
	cnm_core->vcore_clock_on = vcore_clock_on;
	cnm_core->vcore_clock_off = vcore_clock_off;
	cnm_core->vcore_clock_state = 0;
	cnm_core->cb_broadcast = cb_broadcast;

	cnm_core->usage.prev_time = 0x80000000;
	cnm_core->usage.update_index = 0;
	cnm_core->usage.update_cnt = 0;
	cnm_core->usage.run_time_for_short = 0;
	cnm_core->usage.total_time_for_short = 0;
	cnm_core->usage.run_time_for_long = 0;
	cnm_core->usage.total_time_for_long = 0;
	cnm_core->usage.measure_cnt = 0;

	spin_lock_init(&cnm_core->lock);

	return (void*)cnm_core;
}

void cnm_cleanup(void *cnm_id)
{
	struct cnm_core *cnm_core;

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	vfree(cnm_core);
}

int cnm_ch_lock(void *cnm_id, void *ch, void (*isr_func)(void*, unsigned long))
{
	struct cnm_core *cnm_core;

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return -1;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	if (cnm_core->locked_ch != NULL) {
		if (cnm_core->locked_ch == ch) {
			vlog_warning("already locked ch(0x%X)\n", (unsigned int)ch);
			return 1;
		}

		vlog_error("another locked ch(0x%X), new ch(0x%X)\n",
					(unsigned int)cnm_core->locked_ch, (unsigned int)ch);
		return -1;
	}

	cnm_core->locked_ch = ch;
	cnm_core->locked_isr_func = isr_func;

	return 0;
}

int cnm_ch_unlock(void *cnm_id, void *ch)
{
	struct cnm_core *cnm_core;

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return -1;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	if (cnm_core->locked_ch != ch) {
		vlog_error("another locked ch(0x%X, cur 0x%X)\n",
					(unsigned int)cnm_core->locked_ch, (unsigned int)ch);
		return -1;
	}

	cnm_core->locked_ch = NULL;
	cnm_core->locked_isr_func = NULL;

	return 0;
}

int cnm_ch_is_locked(void *cnm_id, void *ch)
{
	struct cnm_core *cnm_core;

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return -1;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	if (cnm_core->locked_ch == NULL) { /* unlocked */
		return 0;
	}
	else if (cnm_core->locked_ch == ch) { /* the ch is locking */
		return 1;
	}

	/*another ch is locking */
	return -1;
}

void cnm_spin_lock(void *cnm_id)
{
	struct cnm_core *cnm_core;

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	spin_lock_irqsave(&cnm_core->lock, cnm_core->flags);
}

void cnm_spin_unlock(void *cnm_id)
{
	struct cnm_core *cnm_core;

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	spin_unlock_irqrestore( &cnm_core->lock , cnm_core->flags);
}

int cnm_clock_on(void *cnm_id)
{
	struct cnm_core *cnm_core;

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return -1;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	if (cnm_core->vcore_clock_state == 1) {
		vlog_warning("vcore clock already on\n");
	}
	else {
		cnm_core->vcore_clock_on();
		cnm_core->vcore_clock_state = 1;
	}

	return 0;
}

int cnm_clock_off(void *cnm_id)
{
	struct cnm_core *cnm_core;

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return -1;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	if (cnm_core->vcore_clock_state == 0) {
		vlog_error("vcore clock already off\n");
	}
	else {
		cnm_core->vcore_clock_off();
		cnm_core->vcore_clock_state = 0;
	}

	return 0;
}

static unsigned int _cnm_calculate_running_weight(unsigned int width,
											unsigned int height,
											unsigned int frame_rate_residual,
											unsigned int frame_rate_divider)
{
	unsigned int resolution_scaled;
	unsigned int residual_scaled;
	unsigned int divider_scaled;
	unsigned int ffffffff_boundary;

	resolution_scaled = width * height;
	residual_scaled = frame_rate_residual;
	divider_scaled = frame_rate_divider;

	while (resolution_scaled) {
		ffffffff_boundary = 0xFFFFFFFF / resolution_scaled;

		if (residual_scaled <= ffffffff_boundary)
			break;

		resolution_scaled >>= 1;
		residual_scaled >>= 1;
		divider_scaled >>= 2;
	}

	return resolution_scaled * residual_scaled / divider_scaled;
}

unsigned int cnm_reserve_running_weight(void *cnm_id,
		unsigned int width, unsigned int height,
		unsigned int frame_rate_residual, unsigned int frame_rate_divider)
{
	unsigned int running_weight;
	struct cnm_core *cnm_core;

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return 0;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	if (frame_rate_divider == 0) {
		vlog_error("no frame rate(%u/%u)\n",
					frame_rate_residual, frame_rate_divider);
		frame_rate_residual = 30;
		frame_rate_divider = 1;
	}
	else if ((frame_rate_residual / frame_rate_divider) > 60) {
		vlog_error("over frame rate(%u/%u)\n",
					frame_rate_residual, frame_rate_divider);
		frame_rate_residual = 60;
		frame_rate_divider = 1;
	}

	running_weight = _cnm_calculate_running_weight(width, height,
													frame_rate_residual,
													frame_rate_divider);

	cnm_core->running_weight += running_weight;
	cnm_core->running_instance++;

	return running_weight;
}

void cnm_unreserve_running_weight(void *cnm_id, unsigned int running_weight)
{
	struct cnm_core *cnm_core;

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	cnm_core->running_weight -= running_weight;
	cnm_core->running_instance--;
	if (cnm_core->running_instance == 0) {
		vlog_info("initialize utilization\n");

		cnm_core->usage.prev_time = 0x80000000;
		cnm_core->usage.update_index = 0;
		cnm_core->usage.update_cnt = 0;
		cnm_core->usage.run_time_for_short = 0;
		cnm_core->usage.total_time_for_short = 0;
		cnm_core->usage.run_time_for_long = 0;
		cnm_core->usage.total_time_for_long = 0;
		cnm_core->usage.measure_cnt = 0;
	}
}

unsigned int cnm_get_running_weight(void *cnm_id)
{
	unsigned int running_weight;
	struct cnm_core *cnm_core;

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return 0;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	running_weight = cnm_core->running_weight;

	return running_weight;
}

void cnm_update_run_time(void *cnm_id, int run)
{
	unsigned long curr_time;
	unsigned long prev_time;
	unsigned long duration_time;
	struct timespec now;
	struct cnm_core *cnm_core;
#define	VCORE_MAX_DURATION_TIME			(0xFFFFFFFF / VCORE_UTILIZATION_MOVING_AVERAGE_BUF)

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	prev_time = cnm_core->usage.prev_time;
	getnstimeofday(&now);
	curr_time = (now.tv_sec*1000*1000 + now.tv_nsec/1000) & 0x7FFFFFFF;

	if (prev_time != 0x80000000) {
		unsigned int minus_index;

		duration_time = (curr_time >= prev_time) ? \
					curr_time - prev_time : curr_time + 0x80000000 - prev_time;
		if (!duration_time)
			duration_time = 1;
		else if (duration_time > VCORE_MAX_DURATION_TIME)
			duration_time = VCORE_MAX_DURATION_TIME;

		/* for measure - short */
		if (cnm_core->usage.update_cnt >= VCORE_UTILIZATION_MEASURE_CNT) {
			minus_index = \
				(cnm_core->usage.update_index >= \
					VCORE_UTILIZATION_MEASURE_CNT) ? \
				cnm_core->usage.update_index - \
					VCORE_UTILIZATION_MEASURE_CNT : \
				cnm_core->usage.update_index + \
					VCORE_UTILIZATION_MOVING_AVERAGE_BUF - \
					VCORE_UTILIZATION_MEASURE_CNT;

			if (cnm_core->usage.update_duration[minus_index].run)
				cnm_core->usage.run_time_for_short -= \
					cnm_core->usage.update_duration[minus_index].duration_time;

			cnm_core->usage.total_time_for_short -= \
				cnm_core->usage.update_duration[minus_index].duration_time;
		}
		if (run)
			cnm_core->usage.run_time_for_short += duration_time;

		cnm_core->usage.total_time_for_short += duration_time;

		/* for log - long */
		if (cnm_core->usage.update_cnt >= VCORE_UTILIZATION_LOG_CNT) {
			minus_index = \
				(cnm_core->usage.update_index >= \
					VCORE_UTILIZATION_LOG_CNT) ? \
				cnm_core->usage.update_index - \
					VCORE_UTILIZATION_LOG_CNT : \
				cnm_core->usage.update_index + \
					VCORE_UTILIZATION_MOVING_AVERAGE_BUF - \
					VCORE_UTILIZATION_LOG_CNT;

			if (cnm_core->usage.update_duration[minus_index].run)
				cnm_core->usage.run_time_for_long -= \
					cnm_core->usage.update_duration[minus_index].duration_time;

			cnm_core->usage.total_time_for_long -= \
				cnm_core->usage.update_duration[minus_index].duration_time;
		}
		if (run)
			cnm_core->usage.run_time_for_long += duration_time;

		cnm_core->usage.total_time_for_long += duration_time;

		/* save the duration into history buf */
		cnm_core->usage.update_duration[cnm_core->usage.update_index]\
			.run = run;
		cnm_core->usage.update_duration[cnm_core->usage.update_index]\
			.duration_time = duration_time;
		cnm_core->usage.update_index++;
		if (cnm_core->usage.update_index == \
				VCORE_UTILIZATION_MOVING_AVERAGE_BUF) {
			cnm_core->usage.update_index = 0;
		}

		if (cnm_core->usage.update_cnt < \
				VCORE_UTILIZATION_MOVING_AVERAGE_BUF) {
			cnm_core->usage.update_cnt++;
		}

		/* log */
		cnm_core->usage.measure_cnt++;
		if ((cnm_core->usage.measure_cnt % VCORE_UTILIZATION_LOG_CNT) == 0) {
			unsigned long utilization;

			utilization = cnm_core->usage.run_time_for_long * 100 \
						/ cnm_core->usage.total_time_for_long;

			vlog_print(VLOG_VCORE_MONITOR, "vcore:0x%X, #instance:%d, utilization: %lu%%\n",
							(unsigned int)cnm_core, cnm_core->running_instance,
							utilization);
		}
	}

	cnm_core->usage.prev_time = curr_time;
}

unsigned long cnm_get_short_utilization(void *cnm_id)
{
	struct cnm_core *cnm_core;
	unsigned long utilization;

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return 10000;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	if (cnm_core->usage.total_time_for_short)
		utilization = cnm_core->usage.run_time_for_short * 100 \
					/ cnm_core->usage.total_time_for_short;
	else
		utilization = 0;

	return utilization;
}

unsigned long cnm_get_long_utilization(void *cnm_id)
{
	struct cnm_core *cnm_core;
	unsigned long utilization;

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return 10000;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	if (cnm_core->usage.total_time_for_long)
		utilization = cnm_core->usage.run_time_for_long * 100 \
					/ cnm_core->usage.total_time_for_long;
	else
		utilization = 0;

	return utilization;
}

void cnm_isr(void *cnm_id)
{
	unsigned long reason;
	void *locked_ch;
	void (*locked_isr_func)(void *vcore_id, unsigned long reason);
	void (*cb_broadcast)(void *vcore_id);
	struct cnm_core *cnm_core;

	if (IS_ERR_OR_NULL((const void *)cnm_id)) {
		vlog_error("0x%08X cnm_id is NULL\n", (unsigned int)cnm_id);
		return;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	if (vdi_get_instance_pool(cnm_core->coreid) == NULL) {
		vlog_error("core_id %ld : vpu_instance is NULL\n", cnm_core->coreid);
		return;
	}

	reason = VpuReadReg(cnm_core->coreid, BIT_INT_REASON);

	locked_ch = cnm_core->locked_ch;
	locked_isr_func = cnm_core->locked_isr_func;
	cb_broadcast = cnm_core->cb_broadcast;

	if (locked_ch == NULL) {
		vlog_error("no locked chnnel - reason:0x%X\n", (unsigned int)reason);
		VPU_ClearInterrupt(cnm_get_coreid(cnm_id));
	}
	else {
		locked_isr_func(locked_ch, reason);
	}

	cb_broadcast(locked_ch);
}

void cnm_resume(void *cnm_id)
{
	RetCode ret = RETCODE_SUCCESS;
	struct cnm_core *cnm_core;


	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	_cnm_alloc_common_memory(cnm_core);

	spin_lock_irqsave(&cnm_core->lock, cnm_core->flags);

	ret = VPU_InitWithBitcode(cnm_core->coreid, cnm_core->firmware,
						cnm_core->firmware_size,
						(unsigned int)cnm_core->reg_paddr,
						(unsigned int *)cnm_core->reg_vaddr,
						(unsigned int)cnm_core->ion.common.paddr,
						(unsigned int *)cnm_core->ion.common.vaddr,
						cnm_core->ion.common.size);
	if (ret != RETCODE_SUCCESS && ret != RETCODE_CALLED_BEFORE) {
		vlog_error("VPU_InitWithBitcode error =%d \n", ret);
		return;
	}

	_cnm_get_fw_version(cnm_id);

	spin_unlock_irqrestore( &cnm_core->lock , cnm_core->flags);
}

void cnm_suspend(void* cnm_id)
{
	struct cnm_core *cnm_core;

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	spin_lock_irqsave(&cnm_core->lock, cnm_core->flags);

	VPU_DeInit(cnm_core->coreid);

	spin_unlock_irqrestore( &cnm_core->lock , cnm_core->flags);

	_cnm_free_common_memory(cnm_core);
}

void cnm_reset(void *cnm_id, void *handle)
{
	RetCode ret = RETCODE_SUCCESS;
	struct cnm_core *cnm_core;
	void (*cb_report_reset)(void);

	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return;
	}

	cnm_core = (struct cnm_core *)cnm_id;

	cb_report_reset = cnm_core->cb_report_reset;

	vlog_error("ch 0x%X, isr_func 0x%X\n",
			(unsigned int)cnm_core->locked_ch,
			(unsigned int)cnm_core->locked_isr_func);

	vlog_error("core %lu, busy %d, pc 0x%x, mbc+0x74 0x%x, bwb 0x%08x\n",
		cnm_core->coreid,
		(unsigned int)VpuReadReg(cnm_core->coreid, BIT_BUSY_FLAG),
		(unsigned int)VpuReadReg(cnm_core->coreid, BIT_CUR_PC),
		(unsigned int)VpuReadReg(cnm_core->coreid, MBC_BASE+0x74),
		(unsigned int)VpuReadReg(cnm_core->coreid, GDI_BWB_STATUS));

	ret = VPU_SWReset(cnm_core->coreid, SW_RESET_SAFETY, handle);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_SWReset error =%d \n", ret);
		return;
	}

	cb_report_reset();
}

unsigned int cnm_get_coreid(void *cnm_id)
{
	if (cnm_id == NULL) {
		vlog_error("invalied cnm_id\n");
		return 0xffffffff;
	}

	return ((struct cnm_core *)cnm_id)->coreid;
}


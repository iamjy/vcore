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

#include <media/odin/vcodec/vcore/ppu.h>
#include <media/odin/vcodec/vlog.h>

#include "vcore/cnm_9x0/cnm/cnm.h"

#include "vcore/cnm_9x0/cnm_ddk/vpuapi/vpuapi.h"
#include "vcore/cnm_9x0/cnm_ddk/vpuapi/vpuapifunc.h"

struct cnm_ppu_instance
{
	struct list_head channel_list;
};

struct ppu_channel
{
	PpuHandle handle;
	void *vppu_id;

	void *cnm_core_instance;
	struct cnm_ppu_instance *cnm_ppu_instance;
	unsigned int core_idx;

	unsigned int running_weight;

	struct list_head list;

	/* CBs */
	void (*cb_vcore_ppu_report)(void *vppu_id,
								struct vcore_ppu_report *ppu_report);
};

static vcore_bool_t _cnm_ppu_report_pic(void *vcore_id,
								struct vcore_ppu_report *ppu_report)
{
	struct ppu_channel *ch = (struct ppu_channel *)vcore_id;

	ppu_report->hdr = VCORE_PPU_DONE;
	ppu_report->info.done = VCORE_PPU_OK;

	if (cnm_ch_unlock(ch->cnm_core_instance, ch) < 0)
		vlog_error("unlock: 0x%X\n", (unsigned int)ch);

	return VCORE_TRUE;
}

static void cnm_ppu_isr(void *vcore_id, unsigned long reason)
{
	struct ppu_channel *ch;
	struct vcore_ppu_report ppu_report = \
				{.hdr = VCORE_PPU_DONE, .info.done = VCORE_PPU_NO};
	vcore_bool_t work_done = VCORE_FALSE;

	if (vcore_id == NULL) {
		vlog_error("no vcore id, reason: %x\n", (unsigned int)reason);
		return;
	}

	ch = (struct ppu_channel *)vcore_id;
	if (ch->cnm_core_instance == NULL) {
		vlog_error("no vcore instance, reason: %x\n", (unsigned int)reason);
		return;
	}

	cnm_spin_lock(ch->cnm_core_instance);
	cnm_update_run_time(ch->cnm_core_instance, 1);

	if (reason & (1 << INT_BIT_INIT)) {
		vlog_error("no handle of INT_BIT_INIT\n");
	}
	else if (reason & (1 << INT_BIT_SEQ_INIT)) {
		vlog_error("no handle of INT_BIT_SEQ_INIT\n");
	}
	else if (reason & (1 << INT_BIT_SEQ_END)) {
		vlog_error("no handle of INT_BIT_SEQ_END\n");
	}
	else if (reason & (1 << INT_BIT_PIC_RUN)) {
		work_done = _cnm_ppu_report_pic(ch, &ppu_report);
	}
	else if (reason & (1 << INT_BIT_FRAMEBUF_SET)) {
		vlog_error("no handle of INT_BIT_FRAMEBUF_SET\n");
	}
	else if (reason & (1 << INT_BIT_ENC_HEADER)) {
		vlog_error("no handle of INT_BIT_ENC_HEADER\n");
	}
	else if (reason & (1 << INT_BIT_DEC_PARA_SET)) {
		vlog_error("no handle of INT_BIT_DEC_PARA_SET\n");
	}
	else if (reason & (1 << INT_BIT_DEC_BUF_FLUSH)) {
		vlog_error("no handle of INT_BIT_DEC_BUF_FLUSH\n");
	}
	else if (reason & (1 << INT_BIT_USERDATA)) {
		vlog_error("no handle of INT_BIT_USERDATA\n");
	}
	else if (reason & (1 << INT_BIT_DEC_FIELD)) {
		vlog_error("no handle of INT_BIT_DEC_FIELD\n");
	}
	else if (reason & (1 << INT_BIT_DEC_MB_ROWS)) {
		vlog_error("no handle of INT_BIT_DEC_MB_ROWS\n");
	}
	else if (reason & (1 << INT_BIT_BIT_BUF_EMPTY)) {
		vlog_error("no handle of INT_BIT_BIT_BUF_EMPTY\n");
	}
	else if (reason & (1 << INT_BIT_BIT_BUF_FULL)) {
		vlog_error("no handle of INT_BIT_BIT_BUF_FULL\n");
	}
	else {
		vlog_error("no handle of unknown\n");
	}

	VPU_ClearInterrupt(ch->core_idx);

	if (work_done == VCORE_TRUE)
		cnm_clock_off(ch->cnm_core_instance);

	cnm_spin_unlock(ch->cnm_core_instance);

	ch->cb_vcore_ppu_report(ch->vppu_id, &ppu_report);
}

enum vcore_ppu_ret cnm_ppu_open(void **vcore_id,
				void *cnm_core_instance, void *cnm_ppu_instance,
				unsigned int workbuf_paddr, unsigned long *workbuf_vaddr,
				unsigned int workbuf_size,
				unsigned int width, unsigned int height,
				unsigned int fr_residual, unsigned int fr_divider,
				void *vppu_id,
				void (*vcore_ppu_report)(void *vppu_id,
									struct vcore_ppu_report *vcore_report))
{
	RetCode ret = RETCODE_SUCCESS;
	PpuHandle handle;
	vpu_buffer_t vb_work;

	struct cnm_ppu_instance *ppu_instance = \
							(struct cnm_ppu_instance *)cnm_ppu_instance;
	struct ppu_channel *ch;
	unsigned int core_idx = cnm_get_coreid(cnm_core_instance);
	int lock;

	*vcore_id = (void *)NULL;

	ch = (struct ppu_channel *)osal_malloc(sizeof(struct ppu_channel));
	if (!ch) {
		vlog_error("ch malloc failed \n");
		return VCORE_PPU_FAIL;
	}

	ch->cnm_core_instance = cnm_core_instance;

	cnm_spin_lock(ch->cnm_core_instance);
	lock = cnm_ch_is_locked(cnm_core_instance, NULL);
	if (lock != 0) {
		cnm_spin_unlock(ch->cnm_core_instance);

		osal_free(ch);
		return VCORE_PPU_RETRY;
	}

	cnm_clock_on(cnm_core_instance);


	if (workbuf_size) {
		vb_work.phys_addr = (unsigned long)workbuf_paddr;
		vb_work.virt_addr = workbuf_vaddr;
		vb_work.base = (unsigned long)workbuf_vaddr;
		vb_work.size = (int)workbuf_size;
	}

	ret = VPU_PpuOpen(&handle, core_idx, vb_work);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_PpuOpen failed. %d\n", ret);
		cnm_clock_off(cnm_core_instance);
		cnm_spin_unlock(ch->cnm_core_instance);

		osal_free(ch);
		return VCORE_PPU_FAIL;
	}

	ch->cnm_ppu_instance = ppu_instance;
	ch->core_idx = core_idx;

	ch->vppu_id= vppu_id;
	ch->handle = handle;
	ch->cb_vcore_ppu_report = vcore_ppu_report;
	list_add_tail(&ch->list, &ch->cnm_ppu_instance->channel_list);
	vlog_trace("ch open ok\n");

	cnm_reserve_running_weight(ch->cnm_core_instance,
									width, height,
									fr_residual, fr_divider);
	cnm_update_run_time(ch->cnm_core_instance, 0);

	cnm_clock_off(cnm_core_instance);
	cnm_spin_unlock(ch->cnm_core_instance);

	*vcore_id = (void *)ch;
	return VCORE_PPU_SUCCESS;
}

enum vcore_ppu_ret cnm_ppu_close(void *vcore_id)
{
	struct ppu_channel *ch = (struct ppu_channel *)vcore_id;
	RetCode ret = RETCODE_SUCCESS;
	int lock;

	cnm_spin_lock(ch->cnm_core_instance);
	lock = cnm_ch_is_locked(ch->cnm_core_instance, ch);
	if (lock != 0) {
		cnm_spin_unlock(ch->cnm_core_instance);
		return VCORE_PPU_RETRY;
	}

	cnm_clock_on(ch->cnm_core_instance);

	ret = VPU_PpuClose(ch->handle);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_DecClose fail\n");
	}

	if (ch->running_weight) {
		cnm_unreserve_running_weight(ch->cnm_core_instance, ch->running_weight);
		ch->running_weight = 0;
	}
	cnm_update_run_time(ch->cnm_core_instance, 0);

	vlog_trace("ch close success\n");

	list_del(&ch->list);

	cnm_clock_off(ch->cnm_core_instance);
	cnm_spin_unlock(ch->cnm_core_instance);

	ch->cnm_core_instance = NULL;
	osal_free(ch);

	return VCORE_PPU_SUCCESS;
}

enum vcore_ppu_ret cnm_ppu_rotate(void *vcore_id,
							unsigned int src_addr, unsigned int dst_addr,
							unsigned int width, unsigned int height,
							int angle, enum vcore_ppu_image_format format)
{
	struct ppu_channel *ch = (struct ppu_channel *)vcore_id;
	RetCode ret = RETCODE_SUCCESS;
	FrameBuffer src = {0};
	FrameBuffer dst = {0};
	int y_size = width * height;
	static int gdi_index;
	int lock;
	int interleave = 0;

	switch (format)
	{
	case VCORE_PPU_IMAGE_FORMAT_YUV420_2P:
		interleave = 1;
		break;
	case VCORE_PPU_IMAGE_FORMAT_YUV420_3P:
		interleave = 0;
		break;
	default:
		vlog_error("invald format(%d)\n", format);
		return VCORE_FALSE;
	}

	cnm_spin_lock(ch->cnm_core_instance);
	lock = cnm_ch_is_locked(ch->cnm_core_instance, ch);
	if (lock != 0) {
		vlog_error("locked\n");
		cnm_spin_unlock(ch->cnm_core_instance);
		return VCORE_PPU_RETRY;
	}

	cnm_clock_on(ch->cnm_core_instance);

	cnm_update_run_time(ch->cnm_core_instance, 0);
	cnm_ch_lock(ch->cnm_core_instance, ch, cnm_ppu_isr);

	gdi_index = gdi_index%32;
	src.bufY = src_addr;
	src.bufCb = src.bufY + y_size;
	src.bufCr = src.bufCb + (y_size >>2);
	dst.bufY = dst_addr;
	dst.bufCb = dst.bufY + y_size;
	dst.bufCr = dst.bufCb + (y_size >>2);
	src.stride = width;
	src.height = height;
	src.mapType = LINEAR_FRAME_MAP;
	if (angle == 90 || angle == 270) {
		dst.stride = height;
		dst.height = width;
	}
	else {
		dst.stride = width;
		dst.height = height;
	}
	dst.mapType = LINEAR_FRAME_MAP;
	src.myIndex = gdi_index++;
	dst.myIndex = gdi_index++;

	ret = VPU_PpuStartOneFrame(ch->handle, &src, &dst,
							LINEAR_FRAME_MAP,
							interleave,
							VDI_LITTLE_ENDIAN,
							width, height,
							angle, 0,
							FF_NONE, 0 );
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_PpuStartOneFrame... fail(%d) \n", ret);
		goto update_buffer_err;
	}

	cnm_spin_unlock(ch->cnm_core_instance);
	return VCORE_PPU_SUCCESS;

update_buffer_err:
	if (cnm_ch_unlock(ch->cnm_core_instance, ch) < 0)
		vlog_error("unlock: 0x%X\n", (unsigned int)ch);

	cnm_clock_off(ch->cnm_core_instance);
	cnm_spin_unlock(ch->cnm_core_instance);
	return VCORE_PPU_FAIL;
}

void cnm_ppu_reset(void *vcore_id)
{
	struct ppu_channel *ch = (struct ppu_channel *)vcore_id;
	struct ppu_channel *_ch, *tmp;
	struct vcore_ppu_report ppu_report;
	struct cnm_ppu_instance *cnm_ppu_instance = ch->cnm_ppu_instance;

	if (cnm_ppu_instance == NULL)
		return;

	list_for_each_entry_safe(_ch, tmp, &cnm_ppu_instance->channel_list, list) {
		ppu_report.hdr= VCORE_PPU_RESET;
		ppu_report.info.reset = VCORE_PPU_REPORT_RESET_START;
		vlog_error("callback ... ch(0x%X) reset start \n", (unsigned int)_ch);

		_ch->cb_vcore_ppu_report(_ch->vppu_id, &ppu_report);
	}

	cnm_spin_lock(ch->cnm_core_instance);
	cnm_clock_on(ch->cnm_core_instance);
	cnm_ch_lock(ch->cnm_core_instance, ch, cnm_ppu_isr);
	cnm_spin_unlock(ch->cnm_core_instance);

	cnm_reset(ch->cnm_core_instance, ch->handle);
}

void cnm_ppu_report_reset(void *_cnm_ppu_instance)
{
	struct ppu_channel *ch, *tmp;
	struct vcore_ppu_report ppu_report;
	struct cnm_ppu_instance *cnm_ppu_instance = \
							(struct cnm_ppu_instance *)_cnm_ppu_instance;
	if (cnm_ppu_instance == NULL)
		return;

	list_for_each_entry_safe(ch, tmp, &cnm_ppu_instance->channel_list, list) {
		cnm_spin_lock(ch->cnm_core_instance);

		ppu_report.hdr= VCORE_PPU_RESET;
		ppu_report.info.reset = VCORE_PPU_REPORT_RESET_END;
		vlog_error("callback ... ch(0x%X) reset end \n", (unsigned int)ch);

		cnm_ch_unlock(ch->cnm_core_instance, ch);
		cnm_spin_unlock(ch->cnm_core_instance);

		ch->cb_vcore_ppu_report(ch->vppu_id, &ppu_report);
	}
}

void cnm_ppu_broadcast(void *cnm_core_instance, void *_cnm_ppu_instance, void *last_vcore_id)
{
	bool last_vcore_id_is_ppu = false;
	struct ppu_channel *ch, *tmp;
	struct vcore_ppu_report ppu_report;
	struct cnm_ppu_instance *cnm_ppu_instance;

	cnm_spin_lock(cnm_core_instance);

	cnm_ppu_instance = (struct cnm_ppu_instance *)_cnm_ppu_instance;
	if (cnm_ppu_instance == NULL) {
		cnm_spin_unlock(cnm_core_instance);
		vlog_error("cnm_ppu_instance is NULL\n");
		return;
	}

	list_for_each_entry_safe(ch, tmp, &cnm_ppu_instance->channel_list, list) {
		if (ch == last_vcore_id) {
			last_vcore_id_is_ppu = true;
			continue;
		}

		ppu_report.hdr = VCORE_PPU_FEED;
		ch->cb_vcore_ppu_report(ch->vppu_id, &ppu_report);
	}

	if (last_vcore_id_is_ppu) {
		ch = (struct ppu_channel *)last_vcore_id;
		ppu_report.hdr = VCORE_PPU_DONE;
		ppu_report.info.done = VCORE_PPU_OK;
		ch->cb_vcore_ppu_report(ch->vppu_id, &ppu_report);
	}

	cnm_spin_unlock(cnm_core_instance);
}

void *cnm_ppu_init(void)
{
	struct cnm_ppu_instance *instance =
							osal_malloc(sizeof(struct cnm_ppu_instance));
	if (instance == NULL)
		return NULL;

	INIT_LIST_HEAD(&instance->channel_list);

	return (void*)instance;
}


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
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/list.h>
#include <linux/spinlock.h>

#include <media/odin/vcodec/vcore/decoder.h>
#include <media/odin/vcodec/vlog.h>
#include "coda8550j.h"
#include "coda8550j_dec.h"
#include "vcore/cnm_8550j/cnm_ddk/jpuapi/jpuapifunc.h"

#define ALIGN8(x)	( ((x)+(unsigned int)0x7) & (~(unsigned int)0x7))
#define ALIGN16(x)	( ((x)+(unsigned int)0xf) & (~(unsigned int)0xf))
static void coda8550j_dec_isr(void *vcore_id, unsigned long reason);

static spinlock_t dec_lock;
static struct list_head jpg_channel_list;

struct dec_cpb
{
	unsigned int phy_addr;
	unsigned char *vir_ptr;
	unsigned int size;
};

struct jpg_channel
{
	JpgDecHandle handle;
	void *vdc_id;

	struct list_head list;

	struct vcore_dec_au au;

	int f_register_dpb;
	int f_seq_init;
	int f_eos;
	int check_busy_status;
	int source_format;
	int frame_cnt;

	int total_fb_num;
	int frame_dpb_index;

	struct dec_cpb cpb;

	FrameBuffer dpb[VDEC_MAX_NUM_OF_DPB];

	/* CBs */
	void (*cb_vcore_dec_report)(void *vdc_id, struct vcore_dec_report *vcore_report);
};

static void __coda8550j_dec_report_seq(struct jpg_channel *ch, JpgDecInitialInfo initialInfo)
{
	struct vcore_dec_report dec_report = {0};
	unsigned long flags;

	jdi_spin_lock(&dec_lock, &flags);

	coda8550j_update_run_time(1);

	dec_report.hdr = VCORE_DEC_DONE;
	dec_report.info.done.vcore_complete = VCORE_TRUE;
	dec_report.info.done.need_more_au = VCORE_TRUE;
	dec_report.info.done.hdr = VCORE_DEC_REPORT_SEQ;
	dec_report.info.done.info.seq.success = VCORE_TRUE;

	switch (initialInfo.sourceFormat) {
	case FORMAT_420 :
		dec_report.info.done.info.seq.buf_width = ALIGN16(initialInfo.picWidth);
		dec_report.info.done.info.seq.buf_height = ALIGN16(initialInfo.picHeight);
		break;
	case FORMAT_422 :
		dec_report.info.done.info.seq.buf_width = ALIGN16(initialInfo.picWidth);
		dec_report.info.done.info.seq.buf_height = ALIGN8(initialInfo.picHeight);
		break;
	case FORMAT_224 :
		dec_report.info.done.info.seq.buf_width = ALIGN8(initialInfo.picWidth);
		dec_report.info.done.info.seq.buf_height = ALIGN16(initialInfo.picHeight);
		break;
	case FORMAT_444 :
	case FORMAT_400 :
		dec_report.info.done.info.seq.buf_width = ALIGN8(initialInfo.picWidth);
		dec_report.info.done.info.seq.buf_height = ALIGN8(initialInfo.picHeight);
		break;
	}
	dec_report.info.done.info.seq.pic_width = initialInfo.picWidth;
	dec_report.info.done.info.seq.pic_height = initialInfo.picHeight;
	dec_report.info.done.info.seq.ref_frame_cnt = initialInfo.minFrameBufferCount;
	dec_report.info.done.info.seq.format = initialInfo.sourceFormat;
	jdi_memcpy(&dec_report.info.done.info.seq.meta, &ch->au.meta, sizeof(struct vcore_dec_au_meta));
	vlog_trace("callback ... Sequence (%d)\n", initialInfo.minFrameBufferCount);

	ch->f_seq_init = 1;
	ch->source_format = initialInfo.sourceFormat;

	coda8550j_reserve_running_weight(initialInfo.picWidth, initialInfo.picHeight, 1, 1);

	coda8550j_ch_unlock(ch);

	jdi_spin_unlock(&dec_lock, &flags);

	ch->cb_vcore_dec_report(ch->vdc_id, &dec_report);
}

vcore_bool_t __coda8550j_dec_clear_dpb(void *vcore_id, unsigned int dpb_reg)
{
	struct jpg_channel *ch = (struct jpg_channel *)vcore_id;
	int cnt, dpb_index = 0;
	vcore_bool_t clear_done = VCORE_FALSE;

	for (cnt = 0; cnt < ch->total_fb_num; cnt++) {
		if (ch->dpb[cnt].bufY == dpb_reg) {
			ch->dpb[cnt].used = 0;
			dpb_index = ch->dpb[cnt].myIndex;
			break;
		}
	}

	if (cnt == VDEC_MAX_NUM_OF_DPB) {
		vlog_error("clear dpb fail .. (x%08X)\n", dpb_reg);
		clear_done = VCORE_FALSE;
	} else {
		clear_done = VCORE_TRUE;
	}

	return clear_done;
}

static int __coda8550j_dec_update_rdwr_ptr(struct jpg_channel *ch, struct vcore_dec_au *au)
{
	JpgRet ret = JPG_RET_SUCCESS;
	int size = 0;
	PhysicalAddress paRdPtr, paWrPtr;
	unsigned int offset;

	ret = JPU_DecGetBitstreamBuffer(ch->handle, &paRdPtr, &paWrPtr, &size);
	if (ret != JPG_RET_SUCCESS) {
		vlog_error("JPU_DecGetBitstreamBuffer failed Error code is 0x%x \n", ret );
		return -1;
	}

	if( size <= 0  || size < au->buf.size) {
//		JPU_DecGiveCommand(ch->handle, ENABLE_LOGGING, NULL);
		JPU_DecSetRdPtr(ch->handle, ch->cpb.phy_addr, 1);
		offset = paRdPtr - ch->cpb.phy_addr;
	}

	vlog_print(VLOG_VCORE_JPG, "phy: 0x%X ~ 0x%X, size %d \n", au->buf.start_phy_addr, au->buf.end_phy_addr, au->buf.size);
	ch->frame_cnt += 1;
	ch->check_busy_status = VCORE_TRUE;
	ret = JPU_DecUpdateBitstreamBuffer(ch->handle, au->buf.size);
	if (ret != JPG_RET_SUCCESS) {
		vlog_error("JPU_DecUpdateBitstreamBuffer failed Error code is 0x%x \n", ret );
		return -1;
	}

	return 0;
}

static void __coda8550j_dec_set_dpb(void *vcore_id)
{
	struct jpg_channel *ch = (struct jpg_channel *)vcore_id;
	int cnt;

	for (cnt = 0; cnt < ch->total_fb_num; cnt++)
		ch->dpb[cnt].used = 1;
}

static void __coda8550j_dec_dpb_cnt_find(void *vcore_id, JpgDecParam *dec_param)
{
	struct jpg_channel *ch = (struct jpg_channel *)vcore_id;
	int cnt;

	if (ch->frame_dpb_index == 0) {
		for (cnt = 0; cnt < ch->total_fb_num; cnt++) {
			if (ch->dpb[cnt].used == 0) {
				dec_param->dpb_index = cnt;
				dec_param->dpb_used_flag = 1;
				return ;
			}
		}
	} else {
		for (cnt = ch->frame_dpb_index; cnt < ch->total_fb_num; cnt++) {
			if (ch->dpb[cnt].used == 0) {
				dec_param->dpb_index = cnt;
				dec_param->dpb_used_flag = 1;
				return ;
			}
		}
		for (cnt = 0; cnt < ch->frame_dpb_index; cnt++) {
			if (ch->dpb[cnt].used == 0) {
				dec_param->dpb_index = cnt;
				dec_param->dpb_used_flag = 1;
				return ;
			}
		}
	}

	vlog_error("dpb buffer is all used \n");

	return ;
}


static int __coda8550j_dec_internal_decode(void *vcore_id)
{
	struct jpg_channel *ch = (struct jpg_channel *)vcore_id;
	int ret = 0;
	JpgDecParam	dec_param = {0};

	vlog_print(VLOG_VCORE_JPG, "\n");

	__coda8550j_dec_dpb_cnt_find(vcore_id, &dec_param);

	ret = JPU_DecStartOneFrame(ch->handle, &dec_param);
	if (ret != JPG_RET_SUCCESS && ret != JPG_RET_EOS) {
		vlog_error("JPU_DecStartOneFrame... fail (%d)\n", ret);
		return ret;
	}

	coda8550j_update_run_time(0);

	return ret;
}

enum vcore_dec_ret _coda8550j_dec_open(void **vcore_id,
			enum vcore_dec_codec codec_type,
			unsigned int cpb_phy_addr, unsigned char *cpb_vir_ptr,
			unsigned int cpb_size,
			unsigned int workbuf_paddr, unsigned long *workbuf_vaddr,
			unsigned int workbuf_size,
			vcore_bool_t reordering,
			vcore_bool_t rendering_dpb,
			void *vdc_id,
			void (*vcore_dec_report)(void *vdc_id,
						struct vcore_dec_report *vcore_report))
{
	JpgDecOpenParam	decOP = {0};
	JpgRet ret = JPG_RET_SUCCESS;
	JpgDecHandle	handle = {0};
	unsigned long flags;
	int lock;

	struct jpg_channel* ch;

	vlog_print(VLOG_VCORE_JPG, "\n");

	lock = coda8550j_ch_is_locked(NULL);
	if (lock != 0) {
		return VCORE_DEC_RETRY;
	}

	ch = (struct jpg_channel*)jdi_malloc(sizeof(struct jpg_channel));
	if (ch == NULL) {
		vlog_error("ch alloc fail\n");
		return VCORE_DEC_FAIL;
	}

	jdi_spin_lock(&dec_lock, &flags);
	coda8550j_clock_on();
	coda8550j_ch_lock(ch, coda8550j_dec_isr);
	jdi_spin_unlock(&dec_lock, &flags);

	decOP.streamEndian = JPU_STREAM_ENDIAN;
	decOP.frameEndian = JPU_FRAME_ENDIAN;
	decOP.chromaInterleave = CBCR_SEPARATED; /* JPU_CBCR_INTERLEAVE; */
	decOP.bitstreamBuffer = cpb_phy_addr;
	decOP.bitstreamBufferSize = cpb_size;
	decOP.pBitStream = cpb_vir_ptr; /* set virtual address mapped of physical address */
	ret = JPU_DecOpen(&handle, &decOP);
	if (ret != JPG_RET_SUCCESS) {
		jdi_free(ch);
		*vcore_id = (void *)NULL;
		vlog_error("JPU_DecOpen failed Error code is 0x%x \n", ret );
		return VCORE_DEC_FAIL;
	}

/*	JPU_DecGiveCommand(handle, ENABLE_LOGGING, NULL); */

	jdi_spin_lock(&dec_lock, &flags);

	ch->vdc_id= vdc_id;
	ch->handle = handle;
	ch->cb_vcore_dec_report = vcore_dec_report;
	ch->cpb.phy_addr = cpb_phy_addr;
	ch->cpb.vir_ptr = cpb_vir_ptr;
	ch->cpb.size = cpb_size;
	ch->frame_cnt = 0;

	ch->frame_dpb_index = 0;

	list_add_tail(&ch->list, &jpg_channel_list);

	coda8550j_ch_unlock(ch);
	jdi_spin_unlock(&dec_lock, &flags);

	*vcore_id = (void *)ch;

	return VCORE_DEC_SUCCESS;
}

enum vcore_dec_ret _coda8550j_dec_close(void *vcore_id)
{
	struct jpg_channel *ch = (struct jpg_channel *)vcore_id;
	unsigned long flags;
	JpgRet ret = JPG_RET_SUCCESS;
	int lock;

	lock = coda8550j_ch_is_locked(ch);
	if (lock < 0) {
		return VCORE_DEC_RETRY;
	}

	vlog_print(VLOG_VCORE_JPG, "\n");

	jdi_spin_lock(&dec_lock, &flags);
	coda8550j_clock_on();
	coda8550j_ch_lock(ch, coda8550j_dec_isr);

	coda8550j_unreserve_running_weight(ch->handle->JpgInfo.decInfo.picWidth, ch->handle->JpgInfo.decInfo.picHeight, 1, 1);

	ret = JPU_DecClose(ch->handle);
	if (ret != JPG_RET_SUCCESS) {
		vlog_error("JPU_DecClose fail(%d)\n", ret);
	}

	coda8550j_ch_unlock(ch);
	coda8550j_clock_off();
	list_del(&ch->list);
	jdi_spin_unlock(&dec_lock, &flags);

	jdi_free(ch);

	return VCORE_DEC_SUCCESS;
}

static enum vcore_dec_ret _coda8550j_dec_register_dpb(void *vcore_id, struct vcore_dec_fb *fbinfo)
{
	struct jpg_channel *ch = (struct jpg_channel *)vcore_id;

	int i;
	unsigned long flags;
	JpgRet ret = JPG_RET_SUCCESS;

	int width, height;
	int lum_size, chr_size;
	int divX, divY;
	int  mvcol_size;

	divX = ch->source_format == FORMAT_420 || ch->source_format == FORMAT_422 ? 2 : 1;
	divY = ch->source_format == FORMAT_420 || ch->source_format == FORMAT_224 ? 2 : 1;

	vlog_print(VLOG_VCORE_JPG, "\n");

	switch (ch->source_format) {
	case FORMAT_420 :
		width = (fbinfo->buf_width + 1)/2*2;
		height = (fbinfo->buf_height + 1)/2*2;
		break;

	case FORMAT_224 :
		width = fbinfo->buf_width;
		height = (fbinfo->buf_height + 1)/2*2;
		break;

	case FORMAT_422 :
		width = (fbinfo->buf_width + 1)/2*2;
		height = fbinfo->buf_height;
		break;

	case FORMAT_444:
		width = fbinfo->buf_width;
		height = fbinfo->buf_height;
		break;

	case FORMAT_400 :
		width = (fbinfo->buf_width + 1)/2*2;
		height = (fbinfo->buf_height + 1)/2*2;
		break;

	default :
		width = fbinfo->buf_width;
		height = fbinfo->buf_height;
		break;
	}

	lum_size = width * height;
	chr_size = lum_size / divX / divY;

	mvcol_size = ((lum_size+2*chr_size)+3)/5; // round up by 5
	mvcol_size = ((mvcol_size + 7)/8)*8; // round up by 8

	jdi_spin_lock(&dec_lock, &flags);
	coda8550j_clock_on();

	for (i = 0; i < fbinfo->num; i++) {
		ch->dpb[i].bufY = fbinfo->linear_dpb_addr[i];
		ch->dpb[i].bufCb = ch->dpb[i].bufY + ((lum_size + 7)/8*8);
		ch->dpb[i].bufCr = ch->dpb[i].bufCb + ((chr_size + 7)/8*8);
		ch->dpb[i].stride = fbinfo->buf_width;
		ch->dpb[i].myIndex = i;
		ch->dpb[i].used = 1;
		vlog_print(VLOG_VCORE_JPG, "dpb bufY 0x%08X \n", ch->dpb[i].bufY);
	}

	ret = JPU_DecRegisterFrameBuffer(ch->handle, ch->dpb, fbinfo->num, fbinfo->buf_width);
	if (ret != JPG_RET_SUCCESS) {
		vlog_error("JPU_DecRegisterFrameBuffer failed Error code is 0x%x \n", ret );
		return VCORE_FALSE;
	}

	ch->total_fb_num = fbinfo->num;
	ch->f_register_dpb = 1;
	vlog_print(VLOG_VCORE_JPG, "register dpb OK ... fb_num(%d) buf_width(%d) buf_height(%d)\n", fbinfo->num , fbinfo->buf_width, fbinfo->buf_height);

	jdi_spin_unlock(&dec_lock, &flags);

	return VCORE_TRUE;
}

static enum vcore_dec_ret _coda8550j_dec_update_buffer(void *vcore_id,
								struct vcore_dec_au *au, vcore_bool_t *running)
{
	struct jpg_channel *ch = (struct jpg_channel *)vcore_id;
	JpgRet ret = JPG_RET_SUCCESS;
	unsigned long flags;
	JpgDecInitialInfo initialInfo = {0};
	JpgDecParam	dec_param = {0};
	int lock;

	lock = coda8550j_ch_is_locked(ch);
	if (lock != 0) {
		vlog_error("\n");
		return VCORE_DEC_RETRY;
	}

	jdi_spin_lock(&dec_lock, &flags);

	coda8550j_ch_lock(ch, coda8550j_dec_isr);
	jdi_memcpy(&ch->au, au, sizeof(struct vcore_dec_au));

	switch (au->buf.au_type) {
	case VCORE_DEC_AU_SEQUENCE:
		vlog_print(VLOG_VCORE_JPG, "AU_SEQUENCE\n");
		if (au->buf.size > 0) {
			if (__coda8550j_dec_update_rdwr_ptr(ch, au) < 0)
				goto ERR_DEC_UPDATE_BUFFER;
		} else {
			vlog_error("seq hdr size:%u\n", au->buf.size);
		}

		if (ch->f_seq_init) {
			vlog_error("already sequence init done\n");
			goto ERR_DEC_UPDATE_BUFFER;
		}

		vlog_print(VLOG_VCORE_JPG, "JPU_DecGetInitialInfo Start \n");
		ret = JPU_DecGetInitialInfo(ch->handle, &initialInfo);
		if (ret != JPG_RET_SUCCESS) {
			vlog_error( "JPU_DecGetInitialInfo failed Error code is 0x%x\n", ret);
			goto ERR_DEC_UPDATE_BUFFER;
		}

		vlog_print(VLOG_VCORE_JPG, "\n width %d\n height %d\n minFrameBufferCount %d\n sourceFormat %d\n ecsPtr 0x%x\n",
							initialInfo.picWidth,
							initialInfo.picHeight,
							initialInfo.minFrameBufferCount,
							initialInfo.sourceFormat,
							initialInfo.ecsPtr);

		jdi_spin_unlock(&dec_lock, &flags);

		__coda8550j_dec_report_seq(ch, initialInfo);
		break;

	case VCORE_DEC_AU_PICTURE:
		//vlog_print(VLOG_VCORE_JPG, "AU_PICTURE\n");
		if (!(ch->f_seq_init) || !(ch->f_register_dpb)) {
			vlog_error("not registered dpb yet\n");
			goto ERR_DEC_UPDATE_BUFFER;
		}

		if (au->buf.size > 0) {
			if (__coda8550j_dec_update_rdwr_ptr(ch, au) < 0)
				goto ERR_DEC_UPDATE_BUFFER;
		}

		__coda8550j_dec_dpb_cnt_find(ch, &dec_param);
		ret = JPU_DecStartOneFrame(ch->handle, &dec_param);
		if (ret != JPG_RET_SUCCESS && ret != JPG_RET_EOS) {
			if (ret == JPG_RET_BIT_EMPTY) {
				vlog_error("JPG_RET_BIT_EMPTY \n");
				coda8550j_ch_unlock(ch);
			} else {
				vlog_error("JPU_DecStartOneFrame... fail (%d)\n", ret);
				goto ERR_DEC_UPDATE_BUFFER;
			}
		}

		if (au->eos == VCORE_TRUE) {
			if (ch->f_eos == 1) {
				vlog_error("already received EoS\n");
			}
			else {
				vlog_trace("EOS start ... size(%d)\n", au->buf.size);
				ch->f_eos = 1;
			}

			vlog_print(VLOG_VCORE_JPG, "TOTAL FRAME count %d \n", ch->frame_cnt);
		}

		jdi_spin_unlock(&dec_lock, &flags);
		break;

	default:
		vlog_error("au type %d\n", au->buf.au_type);
		jdi_spin_unlock(&dec_lock, &flags);
		break;
	}

	coda8550j_update_run_time(0);
	return VCORE_DEC_SUCCESS;

ERR_DEC_UPDATE_BUFFER :
	coda8550j_ch_unlock(ch);
ERR_DEC_UPDATE_BUFFER_EOS :
	jdi_spin_unlock(&dec_lock, &flags);
	return VCORE_DEC_FAIL;

}

enum vcore_dec_ret _coda8550j_dec_flush(void *vcore_id, unsigned int rd_addr)
{
	struct jpg_channel *ch = (struct jpg_channel *)vcore_id;
	JpgRet ret = JPG_RET_SUCCESS;
	unsigned long flags;
	struct vcore_dec_report dec_report;
	int lock;

	lock = coda8550j_ch_is_locked(ch);
	if (lock < 0) {
		return VCORE_DEC_RETRY;
	}

	jdi_spin_lock(&dec_lock, &flags);
	coda8550j_clock_on();
	coda8550j_ch_lock(ch, coda8550j_dec_isr);

	vlog_print(VLOG_VCORE_JPG, "address 0x%08X \n", rd_addr);
	ret = JPU_DecSetRdPtr(ch->handle, rd_addr, 1);
	if (ret != JPG_RET_SUCCESS)
		vlog_error("JPU_DecSetRdPtr ret %d \n", ret);

	__coda8550j_dec_set_dpb(ch);
	ch->frame_dpb_index = 0;

	dec_report.hdr = VCORE_DEC_DONE;
	dec_report.info.done.vcore_complete = VCORE_TRUE;
	dec_report.info.done.need_more_au = VCORE_TRUE;
	dec_report.info.done.hdr = VCORE_DEC_REPORT_FLUSH_DONE;

	coda8550j_ch_unlock(ch);
	coda8550j_clock_off();
	jdi_spin_unlock(&dec_lock, &flags);

	ch->cb_vcore_dec_report(ch->vdc_id, &dec_report);

	return VCORE_DEC_SUCCESS;
}

void _coda8550j_dec_reset(void *vcore_id)
{
/*	struct jpg_channel *ch = (struct jpg_channel *)vcore_id; */
	unsigned long flags;

	jdi_spin_lock(&dec_lock, &flags);

	vlog_print(VLOG_VCORE_JPG, " \n");

	/* TODO report */

	jdi_spin_unlock(&dec_lock, &flags);
}

static vcore_bool_t _coda8550j_dec_report_pic(struct jpg_channel *ch, struct vcore_dec_report *dec_report)
{
	JpgInst *pJpgInst;
	JpgDecInfo *pDecInfo;
	JpgRet ret = JPG_RET_SUCCESS;
	JpgDecOutputInfo output_info = {0};
	vcore_bool_t work_done = VCORE_FALSE;

	coda8550j_update_run_time(1);

	dec_report->hdr = VCORE_DEC_DONE;
	dec_report->info.done.vcore_complete = VCORE_TRUE;
	dec_report->info.done.need_more_au = VCORE_TRUE;
	dec_report->info.done.hdr = VCORE_DEC_REPORT_NONE;

	ret = JPU_DecGetOutputInfo(ch->handle, &output_info);
	if (ret != JPG_RET_SUCCESS) {
		vlog_error("JPU_DecGetOutputInfo... fail (%d)\n", ret);
		goto report_pic_err;
	}

	vlog_print(VLOG_VCORE_JPG, "disp_idx %d, success %d \n", output_info.indexFrameDisplay, output_info.decodingSuccess);
	if ((output_info.decodingSuccess & 0x01) == 0) {
		dec_report->info.done.vcore_complete = VCORE_TRUE;
		dec_report->info.done.need_more_au = VCORE_TRUE;
		dec_report->info.done.hdr = VCORE_DEC_REPORT_PIC;
		dec_report->info.done.info.pic.success = VCORE_FALSE;
		work_done = VCORE_TRUE;
		goto report_pic_err;
	}

	if (output_info.indexFrameDisplay >= 0) {
		pJpgInst = ch->handle;
		pDecInfo = &pJpgInst->JpgInfo.decInfo;

		dec_report->info.done.vcore_complete = VCORE_TRUE;
		dec_report->info.done.need_more_au = VCORE_TRUE;
		dec_report->info.done.hdr = VCORE_DEC_REPORT_PIC;
		dec_report->info.done.info.pic.success = VCORE_TRUE;
		dec_report->info.done.info.pic.pic_width = (unsigned int)output_info.decPicWidth;
		dec_report->info.done.info.pic.pic_height = (unsigned int)output_info.decPicHeight;
		dec_report->info.done.info.pic.err_mbs = (unsigned int)output_info.numOfErrMBs;
		dec_report->info.done.info.pic.dpb_addr = ch->dpb[output_info.indexFrameDisplay].bufY;
		ch->frame_dpb_index = output_info.indexFrameDisplay;
		ch->dpb[ch->frame_dpb_index].used = 1;
		ch->check_busy_status = VCORE_FALSE;
		vlog_print(VLOG_VCORE_JPG, "dpb bufY 0x%08X \n", ch->dpb[output_info.indexFrameDisplay].bufY);
		jdi_memcpy(&dec_report->info.done.info.pic.meta, &ch->au.meta, sizeof(struct vcore_dec_au_meta));

		work_done = VCORE_TRUE;
	} else {
		vlog_error("%d : indexFrameDisplay %d \n", __LINE__, output_info.indexFrameDisplay);
	}

report_pic_err:
	coda8550j_ch_unlock(ch);

	return work_done;
}

void coda8550j_dec_init(struct vcore_dec_ops *ops)
{
	ops->open = _coda8550j_dec_open;
	ops->close = _coda8550j_dec_close;
	ops->register_dpb = _coda8550j_dec_register_dpb;
	ops->clear_dpb = __coda8550j_dec_clear_dpb;
	ops->update_buffer = _coda8550j_dec_update_buffer;
	ops->reset = _coda8550j_dec_reset;
	ops->flush = _coda8550j_dec_flush;

	jdi_spin_lock_init(&dec_lock);
	INIT_LIST_HEAD(&jpg_channel_list);

}

void coda8550j_dec_report_reset(void)
{
}

static void coda8550j_dec_isr(void *vcore_id, unsigned long reason)
{
	struct jpg_channel *ch = (struct jpg_channel *)vcore_id;
	vcore_bool_t work_done = VCORE_FALSE;
	struct vcore_dec_report dec_report = {0};
	unsigned long flags;

	if (vcore_id == NULL) {
		vlog_error("no locked channel, reason: %x\n", (unsigned int)reason);
		return;
	}

	jdi_spin_lock(&dec_lock, &flags);

	if (reason & (1 << INT_JPU_DONE)) {
		work_done = _coda8550j_dec_report_pic(ch, &dec_report);
	}
	else if (reason & (1 << INT_JPU_ERROR)) {
		vlog_error("no handle of INT_JPU_ERROR\n");
		work_done= _coda8550j_dec_report_pic(ch, &dec_report);
	}
	else if (reason & (1 << INT_JPU_BIT_BUF_EMPTY)) { //INT_JPU_BIT_BUF_FULL
		vlog_print(VLOG_VCORE_JPG, "no handle of INT_JPU_BIT_BUF_EMPTY\n");
		JPU_ClrStatus((1 << INT_JPU_BIT_BUF_EMPTY));
	}
	else if (reason & (1 << INT_JPU_PARIAL_OVERFLOW)) {
		vlog_print(VLOG_VCORE_JPG, "no handle of INT_JPU_PARIAL_OVERFLOW\n");
		//JPU_ClrStatus((1 << INT_JPU_PARIAL_OVERFLOW));
	}
	else if (reason & (1 << INT_JPU_PARIAL_BUF0_EMPTY)) {
		vlog_print(VLOG_VCORE_JPG, "no handle of INT_JPU_PARIAL_BUF0_EMPTY\n");
	}
	else if (reason & (1 << INT_JPU_PARIAL_BUF1_EMPTY)) {
		vlog_print(VLOG_VCORE_JPG, "no handle of INT_JPU_PARIAL_BUF1_EMPTY\n");
	}
	else if (reason & (1 << INT_JPU_PARIAL_BUF2_EMPTY)) {
		vlog_print(VLOG_VCORE_JPG, "no handle of INT_JPU_PARIAL_BUF2_EMPTY\n");
	}
	else if (reason & (1 << INT_JPU_PARIAL_BUF3_EMPTY)) {
		vlog_print(VLOG_VCORE_JPG, "no handle of INT_JPU_PARIAL_BUF3_EMPTY\n");
	}
#ifdef SUPPORT_STOP_COMMAND
	else if (reason & (1 << INT_JPU_BIT_BUF_STOP)) {
		vlog_print(VLOG_VCORE_JPG, "no handle of INT_JPU_BIT_BUF_STOP\n");
	}
#endif
	else {
		vlog_error("no handle of unknown\n");
	}

	if (work_done == VCORE_TRUE)
		coda8550j_clock_off();

	jdi_spin_unlock(&dec_lock, &flags);

	ch->cb_vcore_dec_report(ch->vdc_id, &dec_report);
}



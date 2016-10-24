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
#include <linux/err.h>
#include <asm/uaccess.h>

#include <media/odin/vcodec/vcore/encoder.h>
#include <media/odin/vcodec/vlog.h>
#include "coda8550j.h"
#include "coda8550j_enc.h"
#if 1
#include "vcore/cnm_8550j/cnm_ddk/jpuapi/jputable.h"
#include "vcore/cnm_8550j/cnm_ddk/jpuapi/jpuapifunc.h"
#else
#include "./jpuapi/jputable.h"
#include "vcore/cnm_8550j/cnm_ddk/jpuapi/jpuapifunc.h"
#endif
#include "misc/vbuf/vbuf.h"

#define ALIGN16(x)	( ((x)+(unsigned int)0xf) & (~(unsigned int)0xf))
#define ALIGN8(x)	( ((x)+(unsigned int)0x7) & (~(unsigned int)0x7))

static void coda8550j_enc_isr(void *vcore_id, unsigned long reason);

static spinlock_t jpg_enc_lock;
static struct list_head jpg_enc_channel_list;

struct vcore_enc_rect {
	int div_x;
	int div_y;
	int lum_size;
	int chr_size;
	int mvcol_size;

	int stride_y;
	int stride_c;
};

struct epb_buf {
	unsigned int phy_addr;
	unsigned char *vir_ptr;
	unsigned int size;
};

struct enc_epb {
	struct epb_buf buf;
	/* rd, wr ptr for a chunk processing */
	unsigned int start_phy_addr;
	unsigned int end_phy_addr;
	unsigned char *vir_ptr;
	unsigned int epb_end_phy_addr;
};

struct jpg_enc_channel
{
	JpgEncHandle handle;
	void *vec_id;
	int jpeg_initial_flag;

	struct list_head list;

	struct enc_epb epb;
	int index;

	int pixel_format;
	int pic_width;
	int pic_height;
	int align_width;
	int align_height;

	int max_encoded_size;

	JpgEncOpenParam encOP;
	/* CBs */
	void (*cb_vcore_enc_report)(void *vec_id,
					struct vcore_enc_report *vcore_report);
};

static void __coda8550j_enc_huff_table(JpgEncOpenParam *encOP)
{
	/* Luma DC BitLength */
	jdi_memcpy(encOP->huffBits[DC_TABLE_INDEX0], lumaDcBits, 16);
	/* Luma DC HuffValue */
	jdi_memcpy(encOP->huffVal[DC_TABLE_INDEX0], lumaDcValue, 16);

	/* Luma DC BitLength */
	jdi_memcpy(encOP->huffBits[AC_TABLE_INDEX0], lumaAcBits, 16);
	/* Luma DC HuffValue */
	jdi_memcpy(encOP->huffVal[AC_TABLE_INDEX0], lumaAcValue, 162);

	/* Chroma DC BitLength */
	jdi_memcpy(encOP->huffBits[DC_TABLE_INDEX1], chromaDcBits, 16);
	/* Chroma DC HuffValue */
	jdi_memcpy(encOP->huffVal[DC_TABLE_INDEX1], chromaDcValue, 16);

	/* Chroma AC BitLength */
	jdi_memcpy(encOP->huffBits[AC_TABLE_INDEX1], chromaAcBits, 16);
	/* Chorma AC HuffValue */
	jdi_memcpy(encOP->huffVal[AC_TABLE_INDEX1], chromaAcValue, 162);

	return;
}

static void __coda8550j_enc_q_matrix(JpgEncOpenParam *encOP,
								unsigned int qulity)
{
	int scale_factor;
	long temp;
	const int force_baseline = 1;
	int i;

	if(qulity <=0)
		qulity = 1;
	if(qulity > 100)
		qulity = 100;
	if(qulity < 50)
		scale_factor = 5000 / qulity;
	else
		scale_factor = 200 - qulity*2;

	for (i = 0; i < 64; i++) {
		temp = ((long) std_luminance_quant_tbl[i] * scale_factor + 50L)/100L;
		/* limit the values to the valid range */
		if (temp <= 0L) temp = 1L;
		if (temp > 32767L) temp = 32767L; /* max quantizer needed for 12 bits */
		if (force_baseline && temp > 255L)
			temp = 255L;		/* limit to baseline range if requested */

		encOP->qMatTab[DC_TABLE_INDEX0][i] = (BYTE)temp;
	}

	for (i = 0; i < 64; i++) {
		temp = ((long) std_chrominance_quant_tbl[i] * scale_factor + 50L)/100L;
		/* limit the values to the valid range */
		if (temp <= 0L) temp = 1L;
		if (temp > 32767L) temp = 32767L; /* max quantizer needed for 12 bits */
		if (force_baseline && temp > 255L)
			temp = 255L;		/* limit to baseline range if requested */

		encOP->qMatTab[AC_TABLE_INDEX0][i] = (BYTE)temp;
	}

	jdi_memcpy(encOP->qMatTab[DC_TABLE_INDEX1],
					encOP->qMatTab[DC_TABLE_INDEX0], 64);
	jdi_memcpy(encOP->qMatTab[AC_TABLE_INDEX1],
					encOP->qMatTab[AC_TABLE_INDEX0], 64);

	return;
}

static vcore_bool_t __coda8550j_enc_report_seq(void *vcore_id)
{
	struct jpg_enc_channel *ch = (struct jpg_enc_channel *)vcore_id;
	struct vcore_enc_report enc_report = {0};
	JpgRet ret = JPG_RET_SUCCESS;
	unsigned int size = 0;
	JpgEncParamSet enc_header_param = {0};
	JpgInst * pJpgInst;
	JpgEncInfo * pEncInfo;
	unsigned int paRdPtr, paWrPtr;

	pJpgInst = ch->handle;
	pEncInfo = &pJpgInst->JpgInfo.encInfo;

	/* read wr ptr*/
	ret = JPU_EncGetBitstreamBuffer(ch->handle, &paRdPtr, &paWrPtr, &size);
	if (ret != JPG_RET_SUCCESS) {
		vlog_error("VPU_EncGetBitstreamBuffer failed Error code is 0x%x \n",
						ret );
	}

	/* make header */
	enc_header_param.size = STREAM_BUF_SIZE;
	/* Encoder header disable/enable control. Annex:A 1.2.3 item 13 */
	enc_header_param.headerMode = ENC_HEADER_MODE_NORMAL;
	/* Merge quantization table. Annex:A 1.2.3 item 7 */
	enc_header_param.quantMode = JPG_TBL_NORMAL; /* JPG_TBL_MERGE */
	/* Merge huffman table. Annex:A 1.2.3 item 6 */
	enc_header_param.huffMode = JPG_TBL_NORMAL; /* JPG_TBL_MERGE */
	/* Remove APPn. Annex:A item 11 */
	enc_header_param.disableAPPMarker = 0;
	enc_header_param.enableSofStuffing = 0;	/* no padding */
	enc_header_param.pParaSet =  ch->epb.vir_ptr +
						(paRdPtr - pEncInfo->streamBufStartAddr);
	size = JpgEncEncodeHeader(ch->handle, &enc_header_param);
	if (size <= 0) {
		vlog_error("JpgEncEncodeHeader failed. %d\n", size);
		return VCORE_FALSE;
	}

	enc_report.hdr = VCORE_ENC_DONE;
	enc_report.info.done.hdr = VCORE_ENC_REPORT_PIC;
	enc_report.info.done.info.pic.success = VCORE_TRUE;
	enc_report.info.done.info.pic.pic_type = VCORE_ENC_SEQ_HDR;
	enc_report.info.done.info.pic.start_phy_addr = paWrPtr;
	enc_report.info.done.info.pic.end_phy_addr = paWrPtr + size;
	enc_report.info.done.info.pic.size = size;

	ch->cb_vcore_enc_report(ch->vec_id, &enc_report);

	/* wrptr update */
	pEncInfo->streamWrPtr = ALIGN8(paWrPtr + size);
	JpuWriteReg(MJPEG_BBC_WR_PTR_REG, pEncInfo->streamWrPtr);

	/* start ptr update for picture*/
	ch->epb.start_phy_addr = pEncInfo->streamWrPtr;

	return VCORE_TRUE;
}

vcore_bool_t __coda8550j_enc_frame_buf(int format,
						int width,
						int height,
						struct vcore_enc_rect *enc_rect)
{
	struct vcore_enc_rect *rect;
	rect = enc_rect;

	rect->div_x = format == FORMAT_420 || format == FORMAT_422 ? 2 : 1;
	rect->div_y = format == FORMAT_420 || format == FORMAT_224 ? 2 : 1;

	switch (format) {
	case FORMAT_420 :
		height = (height+1)/2*2;
		width = (width+1)/2*2;
		break;

	case FORMAT_224 :
		height = (height+1)/2*2;
		break;

	case FORMAT_422 :
		width = (width+1)/2*2;
		break;

	case FORMAT_444 :
		break;

	case FORMAT_400 :
		height = (height+1)/2*2;
		width = (width+1)/2*2;
		break;
	}

	rect->lum_size = width * height;
	rect->chr_size = rect->lum_size / rect->div_x / rect->div_y;

	/* round up by 5 */
	rect->mvcol_size = ((rect->lum_size + 2 * rect->chr_size) + 3) / 5;
	/* round up by 8 */
	rect->mvcol_size = ((rect->mvcol_size + 7) / 8) * 8;

	if (format == FORMAT_420 || format == FORMAT_422)
		rect->stride_y = (((width + 15)/16)*16);
	else
		rect->stride_y = (((width + 7)/8)*8);

	rect->stride_c = width / rect->div_x;

	return VCORE_TRUE;
}

enum vcore_enc_ret _coda8550j_enc_open(void **vcore_id,
				struct vcore_enc_config *config,
				unsigned int workbuf_paddr,
				unsigned long *workbuf_vaddr,
				unsigned int workbuf_size,
				void *vec_id,
				void (*vcore_enc_report)(void *vec_id,
						struct vcore_enc_report *vcore_report))
{
	unsigned long flags;
	JpgRet ret = JPG_RET_SUCCESS;
	unsigned int quality = 0;
	struct jpg_enc_channel* ch;
	int lock;

	*vcore_id = NULL;

	ch = (struct jpg_enc_channel*)jdi_malloc(sizeof(struct jpg_enc_channel));
	if (ch == NULL) {
		vlog_error("jdi alloc fail\n");
		return VCORE_ENC_FAIL;
	}

	jdi_spin_lock(&jpg_enc_lock, &flags);
	lock = coda8550j_ch_is_locked(NULL);
	if (lock != 0) {
		jdi_spin_unlock(&jpg_enc_lock, &flags);

		jdi_free(ch);
		return VCORE_ENC_RETRY;
	}
	coda8550j_clock_on();

	vlog_print(VLOG_VCORE_JPG, "epb paddr 0x%X, vaddr 0x%X, size 0x%X\n",
					config->epb_phy_addr,
					(unsigned int)config->epb_vir_ptr,
					config->epb_size);

	ch->encOP.bitstreamBuffer = config->epb_phy_addr;
	ch->encOP.bitstreamBufferSize = config->epb_size;
	ch->encOP.picWidth = config->pic_width;
	ch->encOP.picHeight = config->pic_height;
	ch->encOP.sourceFormat = config->format;
	ch->encOP.restartInterval = 0;
	ch->encOP.streamEndian = JPU_STREAM_ENDIAN;
	ch->encOP.frameEndian = JPU_ENC_FRAME_ENDIAN;

	switch (config->cbcr_interleave) {
	case VCORE_ENC_CBCR_INTERLEAVE_NONE :
		ch->encOP.chromaInterleave = 0;
		break;
	case VCORE_ENC_CBCR_INTERLEAVE_NV12 :
		ch->encOP.chromaInterleave = 1;
		break;
	case VCORE_ENC_CBCR_INTERLEAVE_NV21 :
	default :
		vlog_error("invalid cbcr_interleave %d\n", config->cbcr_interleave);
		goto err_jpu_enc_open;
		break;
	}

	if(config->bit_rate.control_mode != VCORE_ENC_RATE_CONTROL_DISABLE)
		quality = config->bit_rate.target_kbps;

	__coda8550j_enc_huff_table(&ch->encOP);
	__coda8550j_enc_q_matrix(&ch->encOP, quality);

	ret = JPU_EncOpen(&ch->handle, &ch->encOP);
	if (ret != JPG_RET_SUCCESS) {
		vlog_error("JPU_EncOpen failed. ret : %d\n", ret);
		goto err_jpu_enc_open;
	}

	ch->epb.buf.phy_addr = config->epb_phy_addr;
	ch->epb.buf.vir_ptr = config->epb_vir_ptr;
	ch->epb.buf.size = config->epb_size;
	ch->epb.epb_end_phy_addr = config->epb_phy_addr + config->epb_size;

	ch->vec_id = vec_id;
	ch->index = 0;
	ch->epb.start_phy_addr = ch->epb.buf.phy_addr;
	ch->epb.vir_ptr = ch->epb.buf.vir_ptr;
	ch->pixel_format = config->format;
	ch->pic_width = config->pic_width;
	ch->pic_height = config->pic_height;
	ch->align_width = ch->handle->JpgInfo.encInfo.alignedWidth;
	ch->align_height = ch->handle->JpgInfo.encInfo.alignedHeight;
	ch->max_encoded_size = 0;
	ch->cb_vcore_enc_report = vcore_enc_report;
	ch->jpeg_initial_flag = 0;
	vlog_print(VLOG_VCORE_JPG, "pic w %d x h %d :: align w %d x h %d \n",
						ch->pic_width, ch->pic_height,
						ch->align_width, ch->align_height);

	list_add_tail(&ch->list, &jpg_enc_channel_list);

	coda8550j_reserve_running_weight(ch->align_width, ch->align_height, 1, 1);

	coda8550j_clock_off();
	jdi_spin_unlock(&jpg_enc_lock, &flags);

	*vcore_id = ch;
	return VCORE_ENC_SUCCESS;

err_jpu_enc_open:
	coda8550j_ch_unlock(ch);
	jdi_spin_unlock(&jpg_enc_lock, &flags);
	jdi_free(ch);

	*vcore_id = NULL;
	return VCORE_ENC_FAIL;
}

enum vcore_enc_ret _coda8550j_enc_close(void *vcore_id)
{
	struct jpg_enc_channel *ch = (struct jpg_enc_channel *)vcore_id;
	unsigned long flags;
	JpgRet ret = JPG_RET_SUCCESS;
	int lock;

	jdi_spin_lock(&jpg_enc_lock, &flags);
	lock = coda8550j_ch_is_locked(ch);
	if (lock != 0) {
		jdi_spin_unlock(&jpg_enc_lock, &flags);
		return VCORE_ENC_RETRY;
	}

	coda8550j_clock_on();

	list_del(&ch->list);

	coda8550j_unreserve_running_weight(ch->align_width, ch->align_height, 1, 1);

	ret = JPU_EncClose(ch->handle);
	if (ret != JPG_RET_SUCCESS) {
		vlog_error("JPU_EncClose fail(%d)\n", ret);
		goto err_jpg_enc_close;
	}

	coda8550j_clock_off();
	jdi_spin_unlock(&jpg_enc_lock, &flags);

	jdi_free(ch);

	vlog_trace("ch close success \n");

	return VCORE_ENC_SUCCESS;

err_jpg_enc_close:
	coda8550j_clock_off();
	coda8550j_ch_unlock(ch);
	jdi_spin_unlock(&jpg_enc_lock, &flags);

	jdi_free(ch);

	vlog_error("ch close fail \n");

	return VCORE_ENC_FAIL;

}

enum vcore_enc_ret _coda8550j_enc_update_buffer(void *vcore_id,
										struct vcore_enc_fb *fb)
{
	struct jpg_enc_channel *ch = (struct jpg_enc_channel*)vcore_id;
	JpgRet ret = JPG_RET_SUCCESS;
	unsigned long flags;
	JpgEncParam enc_param = {0};
	FrameBuffer framebuffer = {0};
	struct vcore_enc_rect enc_rect = {0};
	JpgEncInitialInfo init_info = {0};
	int lock;

	jdi_spin_lock(&jpg_enc_lock, &flags);
	lock = coda8550j_ch_is_locked(ch);
	if (lock != 0) {
		jdi_spin_unlock(&jpg_enc_lock, &flags);
		return VCORE_ENC_RETRY;
	}

	coda8550j_clock_on();
	coda8550j_update_run_time(0);
	coda8550j_ch_lock(ch, coda8550j_enc_isr);

	if (ch->jpeg_initial_flag == 0) {
		ret = JPU_EncGetInitialInfo(ch->handle, &init_info);
		if (ret != JPG_RET_SUCCESS && ret != JPG_RET_CALLED_BEFORE) {
			vlog_error("VPU_EncGetInitialInfo failed. ret : %d\n", ret);
			goto ERR_UPDATA_BUFFER;
		}
		ch->jpeg_initial_flag = 1;
	}

	/* make header */
	if(__coda8550j_enc_report_seq(ch) == VCORE_FALSE) {
		vlog_error("__coda8550j_enc_report_seq failed.\n");
		goto ERR_UPDATA_BUFFER;
	}

	/* make picture */
	__coda8550j_enc_frame_buf(ch->pixel_format,
							ch->pic_width, ch->pic_height, &enc_rect);

	framebuffer.bufY = fb->fb_phy_addr;
	framebuffer.bufCb = framebuffer.bufY + enc_rect.lum_size;
	framebuffer.bufCr = framebuffer.bufCb + enc_rect.chr_size;
	framebuffer.myIndex = ch->index;
	framebuffer.stride = enc_rect.stride_y;
	framebuffer.used = 0;
	enc_param.sourceFrame = &framebuffer;

	ret = JPU_EncStartOneFrame(ch->handle, &enc_param);
	if (ret != JPG_RET_SUCCESS) {
		vlog_error("JPU_EncStartOneFrame failed. ret : %d \n", ret);
		goto ERR_UPDATA_BUFFER;
	}

	jdi_spin_unlock(&jpg_enc_lock, &flags);
	return VCORE_ENC_SUCCESS;

ERR_UPDATA_BUFFER :
	coda8550j_ch_unlock(ch);
	coda8550j_clock_off();
	jdi_spin_unlock(&jpg_enc_lock, &flags);

	return VCORE_ENC_FAIL;
}

void _coda8550j_enc_reset(void *vcore_id)
{
	struct jpg_enc_channel *ch = (struct jpg_enc_channel*)vcore_id;
	unsigned long flags;

	vlog_print(VLOG_VCORE_JPG, "\n");
	jdi_spin_lock(&jpg_enc_lock, &flags);
	coda8550j_clock_on();

	ch->index = 0;
	jdi_spin_unlock(&jpg_enc_lock, &flags);

#ifdef SUPPORT_STOP_COMMAND
	JpuWriteReg(MJPEG_PIC_START_REG, 1<< JPG_START_STOP);
#endif
}

enum vcore_enc_ret _coda8550j_enc_update_epb_rdaddr(void *vcore_id,
												unsigned int rd_addr)
{
	struct jpg_enc_channel *ch = (struct jpg_enc_channel *)vcore_id;
	JpgRet ret = JPG_RET_SUCCESS;
	unsigned int prdPtr, pwrPtr;
	int encoded_size;
	unsigned int offset;
	unsigned long flags;

	jdi_spin_lock(&jpg_enc_lock, &flags);
	coda8550j_clock_on();

	ret = JPU_EncGetBitstreamBuffer(ch->handle, &prdPtr, &pwrPtr, &encoded_size);
	if (ret != JPG_RET_SUCCESS) {
		vlog_error("JPU_EncGetBitstreamBuffer failed. ret : %d\n", ret);
		jdi_spin_unlock(&jpg_enc_lock, &flags);
		return VCORE_ENC_FAIL;
	}

	offset = prdPtr - ch->epb.buf.phy_addr;
	vlog_print(VLOG_VCORE_JPG, "0x%X++0x%X=0x%X (0x%02X 0x%02X 0x%02X 0x%02X\n",
								prdPtr, encoded_size, rd_addr,
								*(ch->epb.buf.vir_ptr + offset),
								*(ch->epb.buf.vir_ptr + offset+1),
								*(ch->epb.buf.vir_ptr + offset+2),
								*(ch->epb.buf.vir_ptr + offset+3));

	ret = JPU_EncUpdateBitstreamBuffer(ch->handle, rd_addr);
	if (ret != JPG_RET_SUCCESS ) {
		vlog_error("JPU_EncUpdateBitstreamBuffer err 0x%x, addr (0x%08x)\n",
						ret, rd_addr);
		jdi_spin_unlock(&jpg_enc_lock, &flags);
		return VCORE_ENC_FAIL;
	}

	jdi_spin_unlock(&jpg_enc_lock, &flags);

	return VCORE_ENC_SUCCESS;
}

static vcore_bool_t _coda8550j_enc_report_pic(void *vcore_id,
										struct vcore_enc_report *enc_report)
{
	struct jpg_enc_channel *ch = (struct jpg_enc_channel*)vcore_id;
	JpgRet ret = JPG_RET_SUCCESS;
	JpgEncOutputInfo output_info = {0};
	JpgInst * pJpgInst;
	JpgEncInfo * pEncInfo;
	unsigned int offset, room = 0, paRdPtr;
	vcore_bool_t work_done = VCORE_FALSE;

	pJpgInst = ch->handle;
	pEncInfo = &pJpgInst->JpgInfo.encInfo;

	coda8550j_update_run_time(1);

	/* read wr ptr */
	ret = JPU_EncGetBitstreamBuffer(ch->handle,
								&paRdPtr, &ch->epb.end_phy_addr, &room);
	if (ret != JPG_RET_SUCCESS) {
		vlog_error("VPU_EncGetBitstreamBuffer failed Error code is 0x%x \n",
						ret);
		goto report_pic_err;
	}
	room = ch->epb.epb_end_phy_addr - ch->epb.end_phy_addr;

	ret = JPU_EncGetOutputInfo(ch->handle, &output_info);
	if (ret != JPG_RET_SUCCESS) {
		vlog_error("JPU_EncGetOutputInfo... failed : %d \n", ret);
		goto report_pic_err;
	}

	enc_report->hdr = VCORE_ENC_DONE;
	enc_report->info.done.hdr = VCORE_ENC_REPORT_PIC;
	enc_report->info.done.info.pic.success =
						(ret == JPG_RET_SUCCESS) ? VCORE_TRUE : VCORE_FALSE;
	enc_report->info.done.info.pic.pic_type = VCORE_ENC_PIC_I;
	enc_report->info.done.info.pic.start_phy_addr = ch->epb.start_phy_addr;
	enc_report->info.done.info.pic.end_phy_addr = ALIGN8(ch->epb.end_phy_addr);
	enc_report->info.done.info.pic.size =
						ch->epb.end_phy_addr - ch->epb.start_phy_addr;
	ch->max_encoded_size =
			(ch->max_encoded_size > enc_report->info.done.info.pic.size) ?
					ch->max_encoded_size : enc_report->info.done.info.pic.size;
	if (room < (ch->max_encoded_size * 2)) {
		vlog_print(VLOG_VCORE_JPG, "room %d, max_encoded_size %d \n",
						room, (ch->max_encoded_size * 2));
		enc_report->info.done.info.pic.end_phy_addr = ch->epb.buf.phy_addr;

		/* wrptr update */
		pEncInfo->streamWrPtr = ch->epb.buf.phy_addr;
		/* JpuWriteReg(MJPEG_BBC_WR_PTR_REG, pEncInfo->streamWrPtr); */
	} else {
		pEncInfo->streamWrPtr = ALIGN8(ch->epb.end_phy_addr);
		/* JpuWriteReg(MJPEG_BBC_WR_PTR_REG, pEncInfo->streamWrPtr); */
	}

	offset = ch->epb.start_phy_addr - ch->epb.buf.phy_addr;
	vlog_print(VLOG_VCORE_JPG, "success (%d), au:0x%X++0x%X=0x%X \n",
					ret,
					enc_report->info.done.info.pic.start_phy_addr,
					enc_report->info.done.info.pic.size,
					enc_report->info.done.info.pic.end_phy_addr);
	vlog_print(VLOG_VCORE_JPG, "(0x%02X 0x%02X 0x%02X 0x%02X 0x%02X)\n",
								*(ch->epb.buf.vir_ptr + offset),
								*(ch->epb.buf.vir_ptr + offset+1),
								*(ch->epb.buf.vir_ptr + offset+2),
								*(ch->epb.buf.vir_ptr + offset+3),
								*(ch->epb.buf.vir_ptr + offset+4));
	work_done = VCORE_TRUE;

report_pic_err:
	coda8550j_ch_unlock(ch);

	return work_done;
}

void coda8550j_enc_init(struct vcore_enc_ops *ops)
{
	ops->open = _coda8550j_enc_open;
	ops->close = _coda8550j_enc_close;
	ops->reset = _coda8550j_enc_reset;
	ops->update_buffer = _coda8550j_enc_update_buffer;
	ops->update_epb_rdaddr = _coda8550j_enc_update_epb_rdaddr;
	ops->set_config = NULL;

	jdi_spin_lock_init(&jpg_enc_lock);
	INIT_LIST_HEAD(&jpg_enc_channel_list);
}

void coda8550j_enc_report_reset(void)
{
	vlog_print(VLOG_VCORE_JPG, "\n");
}

static void coda8550j_enc_isr(void *vcore_id, unsigned long reason)
{
	struct jpg_enc_channel *ch = (struct jpg_enc_channel *)vcore_id;
	vcore_bool_t work_done = VCORE_FALSE;
	vcore_bool_t report = VCORE_FALSE;
	struct vcore_enc_report enc_report = {0};
	unsigned long flags;

	if (vcore_id == NULL) {
		vlog_error("no locked channel, reason: %x\n", (unsigned int)reason);
		return;
	}

	jdi_spin_lock(&jpg_enc_lock, &flags);

	if (reason & (1 << INT_JPU_DONE)) {
		work_done = _coda8550j_enc_report_pic(ch, &enc_report);
		report = VCORE_TRUE;
	}
	else if (reason & (1 << INT_JPU_ERROR)) {
		vlog_error("isr : INT_JPU_ERROR\n");
		work_done = _coda8550j_enc_report_pic(ch, &enc_report);
	}
	else if (reason & (1 << INT_JPU_BIT_BUF_EMPTY)) { //INT_JPU_BIT_BUF_FULL
		vlog_error("isr : INT_JPU_BIT_BUF_EMPTY\n");
		JPU_ClrStatus((1 << INT_JPU_BIT_BUF_EMPTY));
	}
	else if (reason & (1 << INT_JPU_PARIAL_OVERFLOW)) {
		vlog_error("isr : INT_JPU_PARIAL_OVERFLOW\n");
		//JPU_ClrStatus((1 << INT_JPU_PARIAL_OVERFLOW));
	}
	else if (reason & (1 << INT_JPU_PARIAL_BUF0_EMPTY)) {
		vlog_error("isr : INT_JPU_PARIAL_BUF0_EMPTY\n");
	}
	else if (reason & (1 << INT_JPU_PARIAL_BUF1_EMPTY)) {
		vlog_error("isr : INT_JPU_PARIAL_BUF1_EMPTY\n");
	}
	else if (reason & (1 << INT_JPU_PARIAL_BUF2_EMPTY)) {
		vlog_error("isr : INT_JPU_PARIAL_BUF2_EMPTY\n");
	}
	else if (reason & (1 << INT_JPU_PARIAL_BUF3_EMPTY)) {
		vlog_error("isr : INT_JPU_PARIAL_BUF3_EMPTY\n");
	}
#ifdef SUPPORT_STOP_COMMAND
	else if (reason & (1 << INT_JPU_BIT_BUF_STOP)) {
		vlog_error("isr : INT_JPU_BIT_BUF_STOP\n");
	}
#endif
	else {
		vlog_error("no handle of unknown\n");
	}

	if (work_done == VCORE_TRUE)
		coda8550j_clock_off();

	jdi_spin_unlock(&jpg_enc_lock, &flags);

	if (report == VCORE_TRUE)
		ch->cb_vcore_enc_report(ch->vec_id, &enc_report);
}


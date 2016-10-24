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
#include <linux/sizes.h>
#include <media/odin/vcodec/vcore/decoder.h>
#include <media/odin/vcodec/vlog.h>

#include "vcore/cnm_9x0/cnm/cnm.h"
#include "vcore/cnm_9x0/cnm/cnm_dec.h"
#include "vcore/cnm_9x0/util/reorder_queue.h"

#include "vcore/cnm_9x0/cnm_ddk/vpuapi/vpuapi.h"
#include "vcore/cnm_9x0/cnm_ddk/vpuapi/vpuapifunc.h"

#define DEFAULT_RUNNING_WEIGHT_RESIDUAL	60
#define DEFAULT_RUNNING_WEIGHT_DIVIDER	1

/* workaround code for cnm hang */
#define CNM_WORKAROUND

struct dec_cpb
{
	unsigned int phy_addr;
	unsigned char *vir_ptr;
	unsigned int size;
};
struct cnm_dec_instance
{
	struct list_head channel_list;
};

struct dec_channel
{
	DecHandle handle;
	void *vdc_id;
	void *reorder_id;

	void *cnm_core_instance;
	struct cnm_dec_instance *cnm_dec_instance;
	unsigned int core_idx;

	struct list_head list;

	enum vcore_dec_codec codec_type;
	int maptype;
	struct dec_cpb cpb;

	unsigned int pic_width;
	unsigned int pic_height;

	struct
	{
		unsigned int residual;
		unsigned int divider;
	} frame_rate;

	struct vcore_dec_au au;

	vcore_bool_t rendering_dpb;
	vcore_bool_t secure_buf;

	unsigned int ref_frame_cnt;
	int total_fb_num;

	unsigned int continuous_chunk;

	/* flags */
	vcore_bool_t f_register_dpb;
	vcore_bool_t f_seq_init;
	vcore_bool_t f_eos;
	vcore_bool_t f_dpb_full;
	vcore_bool_t f_chunk_reuse_required;
	vcore_bool_t f_chunk_more_request;

	unsigned int running_weight;

	/* debug */
	unsigned int disp_flag;
	unsigned int dec_flag;

	/* CBs */
	void (*cb_vcore_dec_report)(void *vdc_id,
								struct vcore_dec_report *vcore_report);

	struct reorder_meta enq_meta_local, deq_meta_local;
	FrameBuffer dpb_local[VDEC_MAX_NUM_OF_DPB*2]; /* wtl */
	unsigned int linear_dpb_paddr[VDEC_MAX_NUM_OF_DPB];
};

enum cnm_mpeg4_e
{
	CNM_MPEG4_MPEG4,
	CNM_MPEG4_XVID = 8, 		/*	DivX 4, 5, 6, XVID */
	CNM_MPEG4_SORENSON = 256, /*	FLV1 - 256 */
	CNM_MPEG4_NONE,
};

#define STREAM_END_SIZE 0
#define mpeg(CodStd) ((CodStd == STD_MPEG4 || CodStd == STD_MPEG2 )  ? 1 : 0 )
#define mpeg4(CodStd) (CodStd == STD_MPEG4 ? 1 : 0 )
#define ALIGN16(x)	( ((x)+(unsigned int)0xf) & (~(unsigned int)0xf))
#define ALIGN32(x)	( ((x)+(unsigned int)0x1f) & (~(unsigned int)0x1f))

#if 0
static int __cnm_dec_print(unsigned int addr, unsigned int size)
{
	int i=0;

	for (i=0; i<size; i++) {
		vlog_info("0x%08X : 0x%02X\n",addr+i ,*(unsigned char *)(addr+i));
	}

	return 0;
}
#endif

static enum vocre_dec_pic_type __cnm_dec_get_pic_type(DecOutputInfo *dec_info)
{
	unsigned int pic_type;

	if (dec_info->interlacedFrame)
		pic_type = dec_info->picTypeFirst;
	else
		pic_type = dec_info->picType;

	if (pic_type == 0 || pic_type ==5)
		return VCORE_DEC_PIC_I;
	else if (pic_type == 1 || pic_type ==4)
		return VCORE_DEC_PIC_P;
	else if (pic_type == 2 || pic_type ==3)
		return VCORE_DEC_PIC_B;
	else 	{
		vlog_error("unsupported pic type. %d %d %d (%d %d)\n",
			dec_info->topFieldFirst, dec_info->picTypeFirst, dec_info->picType,
			dec_info->indexFrameDecoded, dec_info->decFrameInfo);
		return VCORE_DEC_PIC_B;
	}
}

static void __cnm_dec_set_2nd_axi(void *vcore_id, DecInitialInfo initinfo)
{
	struct dec_channel *ch = (struct dec_channel *)vcore_id;
	SecAxiUse axi_2nd_config = {0};

	axi_2nd_config.useBitEnable  = 1;
	axi_2nd_config.useIpEnable   = 1;
	axi_2nd_config.useDbkYEnable = 1;
	axi_2nd_config.useDbkCEnable = 1;
	axi_2nd_config.useBtpEnable  = 1;
	axi_2nd_config.useOvlEnable  = 1;

	switch (ch->core_idx) {
	case CODA980_COREID :
		break;
	case BODA950_COREID :
		if (ch->codec_type == VCORE_DEC_VC1) {
			axi_2nd_config.useBtpEnable  = 0;
			axi_2nd_config.useOvlEnable  = 0;

			switch (initinfo.profile) {
			case 0:
			case 1:
			case 2: /* mp */
				break;
			case 3: /* ap */
				axi_2nd_config.useDbkCEnable = 0;
				break;
			}
		}
		break;
	default :
		vlog_error("unknown core idx %d\n", ch->core_idx);
		return;
	}

	VPU_DecGiveCommand(ch->handle, SET_SEC_AXI, &axi_2nd_config);
}

static void __cnm_dec_set_cache(void *vcore_id)
{
	struct dec_channel *ch = (struct dec_channel *)vcore_id;
	CodecInst *pcodecinst;
	DecInfo *pdecinfo;
	MaverickCacheConfig dec_cache_maverick;
	int frame_cache_bypass = 0;
	int frame_cache_burst = 1;
	int frame_cache_merge = 3;
	int frame_cache_wayshape = 15;
	unsigned int maverick_cache_config;

	pcodecinst = ch->handle;
	pdecinfo = &pcodecinst->CodecInfo.decInfo;

	if (ch->maptype == LINEAR_FRAME_MAP) {
		/* VC1 opposite field padding is not allowable in UV separated,
			burst 8 and linear map */
		if (!pdecinfo->openParam.cbcrInterleave)
			frame_cache_burst = 0;
	}

	maverick_cache_config = (frame_cache_merge & 0x3) << 9;
	maverick_cache_config = \
			maverick_cache_config | ((frame_cache_wayshape & 0xf) << 5);
	maverick_cache_config = \
			maverick_cache_config | ((frame_cache_burst & 0x1) << 3);
	maverick_cache_config = \
			maverick_cache_config | (frame_cache_bypass & 0x3);

	if (ch->maptype != LINEAR_FRAME_MAP)
		maverick_cache_config = maverick_cache_config | 0x00000004;

	/* {16'b0, 5'b0, merge[1:0], wayshape[3:0],
									1'b0, burst[0], map[0], bypass[1:0]}; */
	dec_cache_maverick.CacheMode = maverick_cache_config;
	VPU_DecGiveCommand(ch->handle, SET_CACHE_CONFIG, &dec_cache_maverick);
}

static unsigned int _cnm_dec_to_cnm_codec_type(enum vcore_dec_codec c)
{
#define STD_UNKNOWN 0xFF
	unsigned int table[] = {
		STD_AVC,	/*	VDEC_CODEC_AVC */
		STD_MPEG4,	/*	VDEC_CODEC_MPEG4 */
		STD_H263,	/*	VDEC_CODEC_H263 */
		STD_VP8,	/*	VDEC_CODEC_VP8 */
		STD_MPEG2,	/*	VDEC_CODEC_MPEG2 */
		STD_DIV3,	/*	VDEC_CODEC_DIVX3 */
		STD_MPEG4,	/*	VDEC_CODEC_DIVX4 */
		STD_MPEG4,	/*	VDEC_CODEC_DIVX5_6 */
		STD_VC1,	/*	VDEC_CODEC_VC1 */
		STD_AVS,	/*	VDEC_CODEC_AVS */
		STD_AVC,	/*	VDEC_CODEC_MVC */
		STD_THO,	/*	VDEC_CODEC_THO */
		STD_UNKNOWN,	/*	VDEC_CODEC_UNKNOWN */
	};
	return table[c];
}

static unsigned int _cnm_dec_to_cnm_mpeg4_type(enum vcore_dec_codec c)
{
	unsigned int table[] = {
		CNM_MPEG4_NONE, 	/*	VDEC_CODEC_AVC */
		CNM_MPEG4_MPEG4,	/*	VDEC_CODEC_MPEG4 */
		CNM_MPEG4_NONE, 	/*	VDEC_CODEC_H263 */
		CNM_MPEG4_NONE, 	/*	VDEC_CODEC_VP8 */
		CNM_MPEG4_NONE, 	/*	VDEC_CODEC_MPEG2 */
		CNM_MPEG4_NONE, 	/*	VDEC_CODEC_DIVX3 */
		CNM_MPEG4_XVID, 	/*	VDEC_CODEC_XVID */
		CNM_MPEG4_SORENSON, /*	VDEC_CODEC_SORENSON */
		CNM_MPEG4_NONE, 	/*	VDEC_CODEC_VC1 */
		CNM_MPEG4_NONE, 	/*	VDEC_CODEC_AVS */
		CNM_MPEG4_NONE, 	/*	VDEC_CODEC_MVC */
		CNM_MPEG4_NONE, 	/*	VDEC_CODEC_THO */
		CNM_MPEG4_NONE, 	/*	VDEC_CODEC_UNKNOWN */
	};
	return table[c];
}

static vcore_bool_t _cnm_dec_update_rdwr_ptr(struct dec_channel *ch,
											struct vcore_dec_au *au)
{
	RetCode ret = RETCODE_SUCCESS;
	int size = 0;
	PhysicalAddress pa_rd_ptr = 0;
	PhysicalAddress pa_wr_ptr = 0;
	CodecInst *pcodecinst = ch->handle;
	DecInfo *pdecinfo = &pcodecinst->CodecInfo.decInfo;

	if (ch->secure_buf == VCORE_TRUE) {
		ret = VPU_DecSetRdPtr(ch->handle, au->buf.start_phy_addr, 1);
		if (ret != RETCODE_SUCCESS ) {
			vlog_error("VPU_DecUpdateBitstreamBuffer failed Error code is 0x%x \n", ret );
			return VCORE_FALSE;
		}
	}
	else {
		if (au->meta.chunk_id != 0) {
			if (au->buf.start_phy_addr != ch->au.buf.end_phy_addr)
				vlog_error("Not continuous au !!! >> old 0x%x(%d), new 0x%x(%d), inst %d_%d\n",
						ch->au.buf.end_phy_addr, ch->au.meta.chunk_id,
						au->buf.start_phy_addr, au->meta.chunk_id,
						pcodecinst->coreIdx, pcodecinst->instIndex);
		}

		ret = VPU_DecGetBitstreamBuffer(ch->handle, &pa_rd_ptr, &pa_wr_ptr, &size);
		if (ret != RETCODE_SUCCESS ) {
			vlog_error("VPU_DecGetBitstreamBuffer failed Error code is 0x%x \n", ret);
			return VCORE_FALSE;
		}

		if (size <= 0  || size < au->buf.size) {
			vlog_error("coreIdx %d instIndex %d \n",
					pcodecinst->coreIdx,
					pcodecinst->instIndex);
			vlog_error("Start 0x%X -- End 0x%X \n",
					pdecinfo->streamBufStartAddr,
					pdecinfo->streamBufEndAddr);
			vlog_error("cpb size is NOT enough: 0x%X 0x%X (0x%X, 0x%X)\n",
					pa_rd_ptr, pa_wr_ptr, size, au->buf.size);
			return VCORE_FALSE;
		}
	}

	ret = VPU_DecUpdateBitstreamBuffer(ch->handle, au->buf.size);
	if (ret != RETCODE_SUCCESS ) {
		vlog_error("VPU_DecUpdateBitstreamBuffer failed Error code is 0x%x \n", ret );
		return VCORE_FALSE;
	}

	ret = VPU_DecGetBitstreamBuffer(ch->handle, &pa_rd_ptr, &pa_wr_ptr, &size);
	if (ret != RETCODE_SUCCESS ) {
		vlog_error("VPU_DecGetBitstreamBuffer failed Error code is 0x%x \n", ret);
		return VCORE_FALSE;
	}

	vlog_print(VLOG_VCORE_DEC,"r 0x%08x, w 0x%08x, s 0x%08x\n", pa_rd_ptr, pa_wr_ptr, size);

/*
	if (ch->codec_type == VCORE_DEC_AVC) {
		if (*au->buf.start_vir_ptr == 0x0 &&
			*(au->buf.start_vir_ptr+1) == 0x0 &&
			*(au->buf.start_vir_ptr+2) == 0x0 &&
			*(au->buf.start_vir_ptr+3) == 0x1)
			;
		else
			vlog_error("no startcode 0x%08x\n", *(unsigned int *)au->buf.start_vir_ptr );
	}
*/
#if 0
	if (au->buf.start_vir_ptr != NULL) {
		vlog_info("\nr: 0x%x, w: 0x%x", pdecinfo->streamRdPtr,
										pdecinfo->streamWrPtr);
		vlog_info("<0x%02x : %02x %02x %02x %02x %02x %02x %02x %02x>\n",
					au->buf.start_phy_addr,
					*au->buf.start_vir_ptr,
					*(au->buf.start_vir_ptr+1),
					*(au->buf.start_vir_ptr+2),
					*(au->buf.start_vir_ptr+3),
					*(au->buf.start_vir_ptr+4),
					*(au->buf.start_vir_ptr+5),
					*(au->buf.start_vir_ptr+6),
					*(au->buf.start_vir_ptr+7)
					);
	}
#endif
	return VCORE_TRUE;
}

static void _cnm_dec_internal_decode(void *vcore_id)
{
	struct dec_channel *ch = (struct dec_channel *)vcore_id;
	RetCode ret = RETCODE_SUCCESS;
	DecParam param_dec = {0};

	cnm_update_run_time(ch->cnm_core_instance, 0);

	vlog_print(VLOG_VCORE_DEC,"ch %x\n", (unsigned int)ch);
	ret = VPU_DecStartOneFrame(ch->handle, &param_dec);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_DecStartOneFrame... fail (%d)\n", ret);
		return;
	}
}

static void _cnm_dec_set_dpb_disp_flag(void *vcore_id, int frame_num)
{
	struct dec_channel *ch = (struct dec_channel *)vcore_id;
	CodecInst *pcodecinst;
	DecInfo *pdecinfo;
	int i;

	pcodecinst = ch->handle;
	pdecinfo = &pcodecinst->CodecInfo.decInfo;

	for (i=0; i<ch->total_fb_num; i++) {
		VPU_DecGiveCommand(ch->handle, DEC_SET_DISPLAY_FLAG, &i);
		ch->disp_flag |= (1<<i);
	}
	vlog_print(VLOG_VCORE_DEC,"disp flag init (0x%08X), frame_num (%d)\n",
				pdecinfo->frameDisplayFlag, ch->total_fb_num);
}

static vcore_bool_t __cnm_dec_get_can_rotate(struct dec_channel *ch, enum vcore_dec_codec codec_type)
{
	int ret;

	if (ch->rendering_dpb == VCORE_FALSE)
		return VCORE_FALSE;

	if (ch->continuous_chunk < 0x10)
		return VCORE_FALSE;

	switch (codec_type){
	case VCORE_DEC_AVC :
		ret = VCORE_TRUE;
		break;
	default :
		ret = VCORE_FALSE;
		return ret;
		break;
	}

	if (cnm_get_short_utilization(ch->cnm_core_instance) > 70) {
		ret = VCORE_FALSE;
	}

	return ret;
}

static void _cnm_dec_set_rotation(struct dec_channel *ch,
									DecHandle handle,
									struct vcore_dec_au *au,
									enum vcore_dec_codec codec_type,
									unsigned int pic_height,
									unsigned int pic_width)
{
	int rotate_angle_value;
	int rotation_stride;

	if (au->meta.rotation_angle == VCORE_DEC_ROTATE_0 ||
		__cnm_dec_get_can_rotate(ch, codec_type) == VCORE_FALSE) {

		VPU_DecGiveCommand(handle, DISABLE_ROTATION, 0);
		au->meta.rotation_angle = VCORE_DEC_ROTATE_0;
	}
	else {
		switch (au->meta.rotation_angle) {
		case VCORE_DEC_ROTATE_90 :
			rotate_angle_value = 270;
			rotation_stride = ALIGN32(pic_height);
			break;
		case VCORE_DEC_ROTATE_180 :
			rotate_angle_value = 180;
			rotation_stride = ALIGN32(pic_width);
			break;
		case VCORE_DEC_ROTATE_270 :
			rotate_angle_value = 90;
			rotation_stride = ALIGN32(pic_height);
			break;
		default :
			vlog_error("rotation_angle(%d) is wrong value\n",
				au->meta.rotation_angle);
			rotate_angle_value = 0;
			rotation_stride = ALIGN32(pic_width);
			break;
		}

		VPU_DecGiveCommand(handle, SET_ROTATION_ANGLE, &rotate_angle_value);
		VPU_DecGiveCommand(handle, SET_ROTATOR_STRIDE, &rotation_stride);
		VPU_DecGiveCommand(handle, ENABLE_ROTATION, 0);
	}

}

static vcore_bool_t _cnm_dec_check_overspec(enum vcore_dec_codec codec,
	unsigned int dec_buffering_num, unsigned int pic_width, unsigned int pic_height,
	unsigned int fps_numerator, unsigned int fps_denominator)
{
	/* maximum dpb size (13056 x 1024 x 16 x 16 / 384) in level 4.2 */
	const int h264_max_dpb_size = 8912896;
	const int h264_max_dpb_num = 16;
	const int genral_max_fps = 30;
	int max_dpb_num_in_level42 = 0;
	vcore_bool_t ret = VCORE_FALSE;

	if (dec_buffering_num > 2) {
		/* CnM decoder holds the number of buffring picture, which is
		   equal to 2 (CnM dependent) plus number of DPB in H.264/AVC spec. */
		dec_buffering_num -= 2;
	}

	switch (codec)
	{
	case VCORE_DEC_AVC :
		max_dpb_num_in_level42 = h264_max_dpb_size / (pic_width * pic_height);
		if (max_dpb_num_in_level42 > h264_max_dpb_num) {
			max_dpb_num_in_level42 = h264_max_dpb_num;
		}

		if (dec_buffering_num > max_dpb_num_in_level42) {
			vlog_error("DPB size (%d) > level limit (%d)\n", dec_buffering_num, max_dpb_num_in_level42);
			ret = VCORE_TRUE;
		}

		break;
	case VCORE_DEC_MPEG4 :
	case VCORE_DEC_MPEG2 :
	case VCORE_DEC_DIVX :
	case VCORE_DEC_SORENSON :
	case VCORE_DEC_AVS :
	case VCORE_DEC_MVC :
	case VCORE_DEC_VC1 :
	case VCORE_DEC_H263 :
	case VCORE_DEC_VP8 :
	case VCORE_DEC_THO :
	case VCORE_DEC_DIVX3 :
		if (fps_numerator > genral_max_fps * fps_denominator) {
			vlog_error("FPS [numerator(%d) / denominator(%d)] should be less than or equal to 30\n", \
				fps_numerator, fps_denominator);
			ret = VCORE_TRUE;
		}
		break;
	default :
		break;
	}

	return ret;
}

static void _cnm_dec_report_seq(struct dec_channel *ch,
							struct vcore_dec_report *dec_report)
{
	RetCode ret = RETCODE_SUCCESS;
	DecInitialInfo initinfo = {0};
	CodecInst *pcodecinst;
	DecInfo *pdecinfo;
	pcodecinst = ch->handle;
	pdecinfo = &pcodecinst->CodecInfo.decInfo;

	ret = VPU_DecCompleteSeqInit(ch->handle, &initinfo);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_DecCompleteSeqInit... fail (%d), reason (0x%x)\n",
					ret, initinfo.seqInitErrReason);
	}

	vlog_print(VLOG_VCORE_DEC, "r: 0x%x, w: 0x%x\n",
				pdecinfo->streamRdPtr, pdecinfo->streamWrPtr);

	dec_report->hdr = VCORE_DEC_DONE;
	dec_report->info.done.vcore_complete = VCORE_TRUE;
	dec_report->info.done.need_more_au = VCORE_TRUE;
	dec_report->info.done.hdr = VCORE_DEC_REPORT_SEQ;

	dec_report->info.done.info.seq.success = \
			(ret == RETCODE_SUCCESS) ? VCORE_TRUE : VCORE_FALSE;
	dec_report->info.done.info.seq.over_spec = \
		_cnm_dec_check_overspec(ch->codec_type, initinfo.minFrameBufferCount,
		initinfo.picWidth, initinfo.picHeight, initinfo.fRateNumerator, initinfo.fRateDenominator);

	dec_report->info.done.info.seq.buf_width = ALIGN32(initinfo.picWidth);
	dec_report->info.done.info.seq.buf_height = ALIGN32(initinfo.picHeight);
	dec_report->info.done.info.seq.pic_width = initinfo.picWidth;
	dec_report->info.done.info.seq.pic_height = initinfo.picHeight;
#ifdef VCORE_WTL_ENABLE
	dec_report->info.done.info.seq.tiled_buf_size = \
				VPU_GetFrameBufSize(ch->core_idx, initinfo.picWidth,
								initinfo.picHeight, ch->maptype,
								FORMAT_420, CBCR_INTERLEAVE,
								&pdecinfo->dramCfg);
/*	vlog_info("tiled buf size 0x%x\n",
				dec_report->info.done.info.seq.tiled_buf_size); */
#endif
	dec_report->info.done.info.seq.mvcol_buf_size = \
			VPU_GetMvColBufSize(_cnm_dec_to_cnm_codec_type(ch->codec_type),
								initinfo.picWidth, initinfo.picHeight,
								1);

	/*vlog_info("mvcol buf size 0x%x\n",
			dec_report->info.done.info.seq.mvcol_buf_size);*/
	dec_report->info.done.info.seq.slice_buf_size = \
					(ch->codec_type == VCORE_DEC_VP8)? VP8_MB_SAVE_SIZE : \
					(ch->codec_type == VCORE_DEC_AVC)? SLICE_SAVE_SIZE : 0;
	/*vlog_info("slice buf size 0x%x\n",
			dec_report->info.done.info.seq.slice_buf_size);*/

	dec_report->info.done.info.seq.ref_frame_cnt = initinfo.minFrameBufferCount;
	dec_report->info.done.info.seq.profile = initinfo.profile;
	dec_report->info.done.info.seq.level = initinfo.level;
	if ((initinfo.fRateNumerator == -1) || (initinfo.fRateDenominator == -1)) {
		dec_report->info.done.info.seq.frame_rate.residual = 0;
		dec_report->info.done.info.seq.frame_rate.divider = 1;
	}
	else {
		dec_report->info.done.info.seq.frame_rate.residual = \
				(unsigned int)initinfo.fRateNumerator;
		dec_report->info.done.info.seq.frame_rate.divider = \
				(unsigned int)initinfo.fRateDenominator;
	}
	dec_report->info.done.info.seq.crop.left = initinfo.picCropRect.left;
	dec_report->info.done.info.seq.crop.right = initinfo.picCropRect.right;
	dec_report->info.done.info.seq.crop.top = initinfo.picCropRect.top;
	dec_report->info.done.info.seq.crop.bottom = initinfo.picCropRect.bottom;
	osal_memcpy(&dec_report->info.done.info.seq.meta, \
			&ch->au.meta, sizeof(struct vcore_dec_au_meta));

	__cnm_dec_set_2nd_axi(ch, initinfo);
	__cnm_dec_set_cache(ch);

	vlog_trace("Sequence OK ... interlace %d, minDpbNum %d, inst %d_%d\n",
				(initinfo.interlace==1) ? 1 : 0, initinfo.minFrameBufferCount,
				ch->core_idx, ch->handle->instIndex);

	if (ch->f_seq_init == VCORE_FALSE) {
		ch->f_seq_init = VCORE_TRUE;
		if (dec_report->info.done.info.seq.frame_rate.residual == 0)
			dec_report->info.done.info.seq.frame_rate.residual = \
										DEFAULT_RUNNING_WEIGHT_RESIDUAL;

		if (dec_report->info.done.info.seq.frame_rate.divider == 0)
			dec_report->info.done.info.seq.frame_rate.divider  = \
										DEFAULT_RUNNING_WEIGHT_DIVIDER;

		ch->running_weight = cnm_reserve_running_weight(
						ch->cnm_core_instance,
						dec_report->info.done.info.seq.pic_width,
						dec_report->info.done.info.seq.pic_height,
						dec_report->info.done.info.seq.frame_rate.residual,
						dec_report->info.done.info.seq.frame_rate.divider);
		ch->ref_frame_cnt = initinfo.minFrameBufferCount;
		ch->pic_width = dec_report->info.done.info.seq.pic_width;
		ch->pic_height = dec_report->info.done.info.seq.pic_height;
		ch->frame_rate.residual = dec_report->info.done.info.seq.frame_rate.residual;
		ch->frame_rate.divider = dec_report->info.done.info.seq.frame_rate.divider;
	}

	switch (ch->codec_type)
	{
	case VCORE_DEC_AVC :
	case VCORE_DEC_MPEG4 :
	case VCORE_DEC_MPEG2 :
	case VCORE_DEC_DIVX :
	case VCORE_DEC_SORENSON :
	case VCORE_DEC_AVS :
	case VCORE_DEC_MVC :
	case VCORE_DEC_VC1 :
		break;
	case VCORE_DEC_H263 :
	case VCORE_DEC_VP8 :
	case VCORE_DEC_THO :
	case VCORE_DEC_DIVX3 :
		ch->f_chunk_reuse_required = VCORE_TRUE;
		vlog_warning("chunkReuseRequired, codec_type:%d\n", ch->codec_type);
		break;
	default :
		vlog_error("codec_type(%d)\n", ch->codec_type);
		break;
	}

	if (cnm_ch_unlock(ch->cnm_core_instance, ch) < 0)
		vlog_error("unlock: 0x%X\n", (unsigned int)ch);
}

static vcore_bool_t _cnm_dec_report_field(struct dec_channel *ch,
										struct vcore_dec_report *dec_report)
{
	vcore_bool_t work_done = VCORE_FALSE;

	vlog_print(VLOG_VCORE_DEC, "ch 0x%X\n", (unsigned int)ch);

	ch->f_chunk_more_request = VCORE_TRUE;

	dec_report->hdr = VCORE_DEC_DONE;
	dec_report->info.done.vcore_complete = VCORE_FALSE;
	dec_report->info.done.need_more_au = VCORE_TRUE;
	dec_report->info.done.hdr = VCORE_DEC_REPORT_NONE;

	return work_done;
}

static vcore_bool_t _cnm_dec_report_pic(struct dec_channel *ch,
									struct vcore_dec_report *dec_report)
{
	RetCode ret = RETCODE_SUCCESS;
	DecOutputInfo output_info;
	CodecInst *pcodecinst;
	DecInfo *pdecinfo;
	vcore_bool_t work_done = VCORE_FALSE;

	dec_report->hdr = VCORE_DEC_DONE;
	dec_report->info.done.vcore_complete = VCORE_TRUE;
	dec_report->info.done.need_more_au = VCORE_FALSE;
	dec_report->info.done.hdr = VCORE_DEC_REPORT_NONE;

	pcodecinst = ch->handle;
	pdecinfo = &pcodecinst->CodecInfo.decInfo;

	ret = VPU_DecGetOutputInfo(ch->handle, &output_info);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_DecGetOutputInfo... fail (%d)\n", ret);
		goto err_report_pic;
	}

#ifdef CNM_WORKAROUND
	if (VpuReadReg(ch->core_idx, BIT_MSG_0) != 0) {
		vlog_error("CnM Defensive>> msg0 0x%08x inst %d_%d\n",
				(unsigned int)VpuReadReg(ch->core_idx, BIT_MSG_0),
				pcodecinst->coreIdx, pcodecinst->instIndex);
//		vdi_log(ch->core_idx, 3, 0);
	}
#endif

	vlog_print(VLOG_VCORE_DEC, "r: 0x%X, w: 0x%X, dec_idx %d, disp_idx %d \n",
						pdecinfo->streamRdPtr,
						pdecinfo->streamWrPtr,
						output_info.indexFrameDecoded,
						output_info.indexFrameDisplay);
	vlog_print(VLOG_VCORE_DEC, "pic_type(%d), dpb_addr(0x%08X) \n",
						output_info.picType,
						output_info.dispFrame.bufY);
	vlog_print(VLOG_VCORE_DEC, "dispflag(0x%X (0x%X)), consumed:0x%X\n",
						pdecinfo->frameDisplayFlag,
						(unsigned int)VpuReadReg(ch->core_idx, BIT_FRM_DIS_FLG),
						output_info.consumedByte);
	if (ch->f_eos == VCORE_FALSE &&
		output_info.interlacedFrame == 0 &&
		ch->secure_buf == VCORE_FALSE)
		if (pdecinfo->streamRdPtr != ch->au.buf.end_phy_addr) {
			vlog_warning("PIC_DONE>> Not all consumed, inst %d_%d -\n",
					pcodecinst->coreIdx , pcodecinst->instIndex);
			vlog_warning("r: 0x%08x, w: 0x%08x,  au start 0x%08x, end 0x%08x -\n",
					pdecinfo->streamRdPtr, pdecinfo->streamWrPtr,
					ch->au.buf.start_phy_addr, ch->au.buf.end_phy_addr);
			vlog_warning("dec_idx %d, disp_idx %d, pic_type(%d), consumed 0x%x\n",
					output_info.indexFrameDecoded, output_info.indexFrameDisplay,
					output_info.picType, output_info.consumedByte);
		}

	if ((output_info.decodingSuccess & 0x1) == 0) {
		vlog_error("decodingSuccess 0x%x\n", output_info.decodingSuccess);
		goto err_report_pic;
	}

	if (output_info.indexFrameDecoded == -1) {
		vlog_error("decoded index -1, inst %d_%d\n",
				pcodecinst->coreIdx , pcodecinst->instIndex);
		ch->f_dpb_full = VCORE_TRUE;
	}

	if (output_info.indexFrameDecoded == -2) {
		if (output_info.indexFrameDecoded < 0 && output_info.indexFrameDisplay < 0) {
			/* even pps is updated as picture type */
			vlog_print(VLOG_VCORE_DEC,"it seems a pps chunk, size(0x%x)\n", ch->au.buf.size);
			dec_report->info.done.vcore_complete = VCORE_FALSE;
			dec_report->info.done.need_more_au = VCORE_TRUE;
			dec_report->info.done.hdr = VCORE_DEC_REPORT_NONE;
			work_done = VCORE_TRUE;
		}

		if (ch->f_eos == VCORE_FALSE) {
			vlog_error("decoded index -2, inst %d_%d\n",
					pcodecinst->coreIdx , pcodecinst->instIndex);
			dec_report->info.done.info.pic.success = VCORE_FALSE;
		}
	}

	if (output_info.numOfErrMBs)
		vlog_error("errMBs %d\n", output_info.numOfErrMBs);

	/* if field is Non Paired Field (NPF) */
	if (output_info.decFrameInfo == 0) 	{
		if (output_info.interlacedFrame == 0)
			vlog_error("NOT interlaced but NPF!!!\n");
		else {
			vlog_print(VLOG_VCORE_DEC, "chunkMoreRequest\n");

			dec_report->info.done.vcore_complete = VCORE_FALSE;
			dec_report->info.done.need_more_au = VCORE_TRUE;
			dec_report->info.done.hdr = VCORE_DEC_REPORT_NONE;

			ch->f_chunk_more_request = VCORE_TRUE;
			output_info.picTypeFirst = (output_info.picTypeFirst > 5) ? \
							output_info.picType : output_info.picTypeFirst;
		}
	}

	if (output_info.chunkReuseRequired) {
		vlog_error("chunkReuseRequired\n");
		ch->f_chunk_reuse_required = VCORE_TRUE;
	}

	if ((output_info.indexFrameDecoded >= ch->total_fb_num) ||
		(output_info.indexFrameDisplay >= ch->total_fb_num)) {
		vlog_error("dec_idx (%d), disp_idx(%d), total fb num(%d)\n",
					output_info.indexFrameDecoded, output_info.indexFrameDisplay,
					ch->total_fb_num);
		goto err_report_pic;
	}

	if (output_info.indexFrameDecoded >= 0) {
		if (ch->dec_flag & (1 << output_info.indexFrameDecoded))
			vlog_error("dec_index is already exist. 0x%X %d 0x%X\n",
						ch->dec_flag, output_info.indexFrameDecoded,
						(unsigned int)VpuReadReg(ch->core_idx, BIT_FRM_DIS_FLG));
		ch->dec_flag |= (1 << output_info.indexFrameDecoded);

		osal_memcpy(&ch->enq_meta_local.dec_info,
						&output_info, sizeof(DecOutputInfo));
		osal_memcpy(&ch->enq_meta_local.au_meta,
						&ch->au.meta, sizeof(struct vcore_dec_au_meta));
		if (reorder_enqueue(ch->reorder_id, output_info.indexFrameDecoded, \
						&ch->enq_meta_local) == VCORE_FALSE)
			vlog_error("dec_index is already exist. 0x%X %d 0x%X\n",
						ch->dec_flag, output_info.indexFrameDecoded,
						(unsigned int)VpuReadReg(ch->core_idx, BIT_FRM_DIS_FLG));
		/* vlog_print(VLOG_VCORE_DEC,
					"reordering enq : pic(%d) chuck_id(%u) timestamp(%llu)\n",
						ch->enq_meta_local.dec_info.picType,
						ch->enq_meta_local.au_meta.chunk_id,
						ch->enq_meta_local.au_meta.timestamp); */
	}

	if (output_info.indexFrameDisplay >= 0) 	{
		if ((ch->dec_flag & (1 << output_info.indexFrameDisplay)) == 0) {
			vlog_error("disp_index is not exist. 0x%X %d 0x%X\n",
						ch->dec_flag, output_info.indexFrameDisplay,
						(unsigned int)VpuReadReg(ch->core_idx, BIT_FRM_DIS_FLG));
			goto err_report_pic;
		}

		ch->dec_flag &= ~(1 << output_info.indexFrameDisplay);
		ch->disp_flag |= (1 << output_info.indexFrameDisplay);

		reorder_dequeue(ch->reorder_id,
					output_info.indexFrameDisplay,
					&ch->deq_meta_local);
		/* vlog_print(VLOG_VCORE_DEC, \
			"reordering deq : dpb_addr(0x%X) pic(%d) chuck_id(%u) timestamp(%llu)\n",
				output_info.dispFrame.bufY, ch->deq_meta_local.dec_info.picType,
				ch->deq_meta_local.au_meta.chunk_id,
				ch->deq_meta_local.au_meta.timestamp);
		*/

		/* vlog_error("dispFrame.bufY : 0x%08X\n", output_info.dispFrame.bufY);
		 */

		dec_report->info.done.vcore_complete = VCORE_TRUE;
		dec_report->info.done.need_more_au = VCORE_TRUE;
		dec_report->info.done.hdr = VCORE_DEC_REPORT_PIC;
		dec_report->info.done.info.pic.success = VCORE_TRUE;
		dec_report->info.done.info.pic.pic_width = \
								(unsigned int)output_info.dispPicWidth;
		dec_report->info.done.info.pic.pic_height = \
								(unsigned int)output_info.dispPicHeight;
		dec_report->info.done.info.pic.dpb_addr = output_info.dispFrame.bufY;
		dec_report->info.done.info.pic.crop.left = output_info.rcDisplay.left;
		dec_report->info.done.info.pic.crop.right = output_info.rcDisplay.right;
		dec_report->info.done.info.pic.crop.top = output_info.rcDisplay.top;
		dec_report->info.done.info.pic.crop.bottom = output_info.rcDisplay.bottom;
		dec_report->info.done.info.pic.aspect_ratio = \
					(unsigned int)ch->deq_meta_local.dec_info.aspectRateInfo;
		dec_report->info.done.info.pic.err_mbs = \
					(unsigned int)ch->deq_meta_local.dec_info.numOfErrMBs;
		if ((output_info.fRateNumerator == -1) ||
			(output_info.fRateDenominator == -1)) {
			dec_report->info.done.info.pic.frame_rate.residual = 0;
			dec_report->info.done.info.pic.frame_rate.divider = 0;
		}
		else 	{
			dec_report->info.done.info.pic.frame_rate.residual = \
								(unsigned int)output_info.fRateNumerator;
			dec_report->info.done.info.pic.frame_rate.divider = \
								(unsigned int)output_info.fRateDenominator;
		}
		dec_report->info.done.info.pic.pic_type = \
						__cnm_dec_get_pic_type(&ch->deq_meta_local.dec_info);
		osal_memcpy(&dec_report->info.done.info.pic.meta, \
				&ch->deq_meta_local.au_meta, sizeof(struct vcore_dec_au_meta));

		vlog_print(VLOG_VCORE_CHUNK, "uid %d, ts %lld\n",
				dec_report->info.done.info.pic.meta.uid, dec_report->info.done.info.pic.meta.timestamp);
	}

	if (ch->f_eos == VCORE_TRUE) {
		if (output_info.indexFrameDisplay == -1) {
			vlog_trace("EOS done !!!\n");
			VPU_DecUpdateBitstreamBuffer(ch->handle, -1);
			ch->f_eos = VCORE_FALSE;

			dec_report->info.done.vcore_complete = VCORE_TRUE;
			dec_report->info.done.need_more_au = VCORE_FALSE;
			dec_report->info.done.hdr = VCORE_DEC_REPORT_EOS;
			work_done = VCORE_TRUE;
		}
		else if (output_info.indexFrameDisplay >= 0) {
			dec_report->info.done.vcore_complete = VCORE_TRUE;
			dec_report->info.done.need_more_au = VCORE_FALSE;

			/* not unlock*/
			work_done = VCORE_FALSE;
			return work_done;
		}
		else {
			vlog_error("EOS, dec_i %d, disp_i %d\n",
				output_info.indexFrameDecoded, output_info.indexFrameDisplay);
			dec_report->info.done.vcore_complete = VCORE_TRUE;
			dec_report->info.done.need_more_au = VCORE_FALSE;

			VPU_DecUpdateBitstreamBuffer(ch->handle, -1);
			ch->f_eos = VCORE_FALSE;
			work_done = VCORE_TRUE;
		}
	}

	if ((output_info.indexFrameDecoded >= 0) &&
		(output_info.indexFrameDecoded < ch->total_fb_num) &&
		(output_info.indexFrameDisplay >= 0) &&
		(output_info.indexFrameDisplay < ch->total_fb_num))
		work_done = VCORE_TRUE;

err_report_pic:

	if (cnm_ch_unlock(ch->cnm_core_instance, ch) < 0)
		vlog_error("unlock: 0x%X\n", (unsigned int)ch);

	return work_done;
}

static vcore_bool_t _cnm_dec_report_bs_empty(struct dec_channel *ch,
										struct vcore_dec_report *dec_report)
{
	vcore_bool_t work_done = VCORE_FALSE;
	DecOutputInfo output_info;

	vlog_warning("\n");

	VPU_DecGetOutputInfo(ch->handle, &output_info); /* cnm ddk sequnece error */

	dec_report->hdr = VCORE_DEC_DONE;
	dec_report->info.done.vcore_complete = VCORE_FALSE;
	dec_report->info.done.need_more_au = VCORE_TRUE;
	dec_report->info.done.hdr = VCORE_DEC_REPORT_NONE;

	return work_done;
}

static void _cnm_dec_report_flush(struct dec_channel *ch,
								struct vcore_dec_report *dec_report)
{
	vlog_print(VLOG_VCORE_DEC, "ch 0x%X\n", (unsigned int)ch);

	dec_report->hdr = VCORE_DEC_DONE;
	dec_report->info.done.vcore_complete = VCORE_TRUE;
	dec_report->info.done.need_more_au = VCORE_TRUE;
	dec_report->info.done.hdr = VCORE_DEC_REPORT_FLUSH_DONE;

	if (cnm_ch_unlock(ch->cnm_core_instance, ch) < 0)
		vlog_error("unlock: 0x%X\n",(unsigned int)ch);
}

static void cnm_dec_isr(void *vcore_id, unsigned long reason)
{
	struct dec_channel *ch;
	struct vcore_dec_report dec_report;
	vcore_bool_t work_done = VCORE_FALSE;

	if (vcore_id == NULL) {
		vlog_error("no vcore idl, reason: %x\n", (unsigned int)reason);
		return;
	}

	ch = (struct dec_channel *)vcore_id;
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
		_cnm_dec_report_seq(ch, &dec_report);
	}
	else if (reason & (1 << INT_BIT_SEQ_END)) {
		vlog_error("no handle of INT_BIT_SEQ_END\n");
	}
	else if (reason & (1 << INT_BIT_PIC_RUN)) {
		work_done = _cnm_dec_report_pic(ch, &dec_report);
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
		_cnm_dec_report_flush(ch, &dec_report);
		work_done = VCORE_TRUE;
	}
	else if (reason & (1 << INT_BIT_USERDATA)) {
		vlog_error("no handle of INT_BIT_USERDATA\n");
	}
	else if (reason & (1 << INT_BIT_DEC_FIELD)) {
		_cnm_dec_report_field(ch, &dec_report);
		VpuWriteReg(ch->core_idx, BIT_INT_CLEAR, 1);
		goto dec_isr_no_clear;
	}
	else if (reason & (1 << INT_BIT_DEC_MB_ROWS)) {
		vlog_error("no handle of INT_BIT_DEC_MB_ROWS\n");
	}
	else if (reason & (1 << INT_BIT_BIT_BUF_EMPTY)) {
		work_done = _cnm_dec_report_bs_empty(ch, &dec_report);
	}
	else if (reason & (1 << INT_BIT_BIT_BUF_FULL)) {
		vlog_error("no handle of INT_BIT_BIT_BUF_FULL\n");
	}
	else {
		vlog_error("no handle of unknown 0x%X\n", (unsigned int)reason);
	}

	VPU_ClearInterrupt(ch->core_idx);

dec_isr_no_clear:

	if (work_done == VCORE_TRUE)
		cnm_clock_off(ch->cnm_core_instance);

	if (ch->f_eos == VCORE_TRUE)
		_cnm_dec_internal_decode(ch);

	cnm_spin_unlock(ch->cnm_core_instance);

	ch->cb_vcore_dec_report(ch->vdc_id, &dec_report);
}

enum vcore_dec_ret cnm_dec_open(void **vcore_id,
		void* cnm_core_instance,
		void* cnm_dec_instance,
		enum vcore_dec_codec codec_type,
		unsigned int cpb_phy_addr, unsigned char *cpb_vir_ptr,
		unsigned int cpb_size,
		unsigned int workbuf_paddr, unsigned long *workbuf_vaddr,
		unsigned int workbuf_size,
		vcore_bool_t reordering,
		vcore_bool_t rendering_dpb,
		vcore_bool_t secure_buf,
		void *vdc_id,
		void (*vcore_dec_report)(void *vdc_id,
								struct vcore_dec_report *vcore_report))
{
	DecHandle handle;
	DecOpenParam param_open = {0};
	RetCode ret = RETCODE_SUCCESS;
	int lock;

	struct cnm_dec_instance *dec_instance = \
							(struct cnm_dec_instance *)cnm_dec_instance;
	struct dec_channel *ch;
	unsigned int core_idx;

	*vcore_id = NULL;

	ch = (struct dec_channel *)osal_malloc(sizeof(struct dec_channel));
	if (!ch) {
		vlog_error("cnm_dec_get_instance malloc failed \n");
		return VCORE_DEC_FAIL;
	}

	cnm_spin_lock(cnm_core_instance);
	lock = cnm_ch_is_locked(cnm_core_instance, NULL);
	if (lock != 0) {
		cnm_spin_unlock(cnm_core_instance);

		osal_free(ch);
		return VCORE_DEC_RETRY;
	}

	cnm_clock_on(cnm_core_instance);

	core_idx = cnm_get_coreid(cnm_core_instance);
	if (core_idx == 0xFFFFFFFF) {
		vlog_error("core idx is invalid\n");
		goto err_dec_open;
	}

	param_open.bitstreamFormat = _cnm_dec_to_cnm_codec_type(codec_type);
	param_open.bitstreamBuffer = cpb_phy_addr;
	param_open.bitstreamBufferSize = cpb_size;
	param_open.coreIdx = core_idx;
	param_open.bwbEnable = VPU_ENABLE_BWB;
#ifdef VCORE_WTL_ENABLE
	param_open.wtlEnable = 1;
	param_open.wtlMode = FF_FRAME;
#endif

	switch (codec_type) {
	case VCORE_DEC_AVC :
	case VCORE_DEC_MPEG4 :
	case VCORE_DEC_H263 :
	case VCORE_DEC_MPEG2 :
	case VCORE_DEC_DIVX :
	case VCORE_DEC_SORENSON :
	case VCORE_DEC_AVS :
	case VCORE_DEC_MVC :
//		param_open.bitstreamMode = BS_MODE_INTERRUPT;
//		break;
	case VCORE_DEC_VP8 :
	case VCORE_DEC_VC1 :
	case VCORE_DEC_THO :
		param_open.bitstreamMode = BS_MODE_PIC_END;	/* align 0 */
		break;
	case VCORE_DEC_DIVX3 :
		param_open.bwbEnable = 0;
		param_open.bitstreamMode = BS_MODE_PIC_END;	/* align 0 */
		break;
	default :
		vlog_error("codec_type(%d)\n", codec_type);
		goto err_dec_open;
	}

	if (mpeg(param_open.bitstreamFormat)) {
		param_open.mp4DeblkEnable = 0;
		if (mpeg4(param_open.bitstreamFormat))
			param_open.mp4Class = _cnm_dec_to_cnm_mpeg4_type(codec_type);
	}

	if (workbuf_size) {
		param_open.vbWork.phys_addr = (unsigned long)workbuf_paddr;
		param_open.vbWork.virt_addr = workbuf_vaddr;
		param_open.vbWork.base = (unsigned long)workbuf_vaddr;
		param_open.vbWork.size = (int)workbuf_size;
	}

	ret = VPU_DecOpen(&handle, &param_open);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_DecOpen ... fail(%d)\n", ret);
		goto err_dec_open;
	}

	ch->cnm_dec_instance = dec_instance;
	ch->core_idx = core_idx;
	ch->vdc_id= vdc_id;
	ch->handle = handle;
#ifdef VCORE_WTL_ENABLE
	ch->maptype = TILED_MIXED_V_MAP;
#else
	ch->maptype = LINEAR_FRAME_MAP;
#endif

	ch->cnm_core_instance = cnm_core_instance;
	ch->cb_vcore_dec_report = vcore_dec_report;
	ch->codec_type = codec_type;
	ch->cpb.phy_addr = cpb_phy_addr;
	ch->cpb.vir_ptr = cpb_vir_ptr;
	ch->cpb.size = cpb_size;
	ch->rendering_dpb = rendering_dpb;
	ch->secure_buf = secure_buf;
	ch->ref_frame_cnt = 0;
	ch->total_fb_num = 0;
	ch->continuous_chunk = 0;
	ch->f_register_dpb = VCORE_FALSE;
	ch->f_register_dpb = VCORE_FALSE;
	ch->f_seq_init = VCORE_FALSE;
	ch->f_eos = VCORE_FALSE;
	ch->f_dpb_full = VCORE_TRUE;
	ch->f_chunk_reuse_required = VCORE_FALSE;
	ch->f_chunk_more_request = VCORE_FALSE;
	memset(ch->linear_dpb_paddr, (unsigned int)-1, \
				sizeof(unsigned int)*VDEC_MAX_NUM_OF_DPB);
	ch->running_weight = 0;
	ch->reorder_id = NULL;

	list_add_tail(&ch->list, &ch->cnm_dec_instance->channel_list);

	vlog_trace("ch open ok : format(%x), reordering(%d), inst %d_%d\n",
						param_open.bitstreamFormat, reordering, handle->coreIdx, handle->instIndex);

	cnm_update_run_time(ch->cnm_core_instance, 0);

	/*VPU_DecGiveCommand(handle, ENABLE_LOGGING, 0);*/

	if (reordering == VCORE_FALSE) {
		VPU_DecGiveCommand(ch->handle, DEC_DISABLE_REORDER, NULL);
		VPU_DecGiveCommand(ch->handle, ENABLE_DEC_THUMBNAIL_MODE, NULL);
	}

	cnm_clock_off(cnm_core_instance);
	cnm_spin_unlock(cnm_core_instance);

	*vcore_id = (void *)ch;

	return VCORE_DEC_SUCCESS;

err_dec_open:

	cnm_clock_off(cnm_core_instance);
	cnm_spin_unlock(cnm_core_instance);
	osal_free(ch);
	return VCORE_DEC_FAIL;
}

enum vcore_dec_ret cnm_dec_close(void *vcore_id)
{
	struct dec_channel *ch = (struct dec_channel *)vcore_id;
	RetCode ret = RETCODE_SUCCESS;
	DecOutputInfo output_info;
	int lock;

	cnm_spin_lock(ch->cnm_core_instance);
	lock = cnm_ch_is_locked(ch->cnm_core_instance, ch);
	if (lock != 0) {
		cnm_spin_unlock(ch->cnm_core_instance);
		return VCORE_DEC_RETRY;
	}

	cnm_clock_on(ch->cnm_core_instance);

	VPU_DecGetOutputInfo(ch->handle, &output_info); /* cnm ddk sequnece error */

	ret = VPU_DecClose(ch->handle);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_DecClose fail(%d)\n", ret);
		goto err_dec_close;
	}

	if (ch->reorder_id != NULL) {
		reorder_queue_deinit(ch->reorder_id);
		ch->reorder_id = NULL;
	}

	if (ch->running_weight) {
		cnm_unreserve_running_weight(ch->cnm_core_instance, ch->running_weight);
		ch->running_weight = 0;
	}
	cnm_update_run_time(ch->cnm_core_instance, 0);

	vlog_trace("ch close success. inst %d_%d\n", ch->handle->coreIdx, ch->handle->instIndex);

	list_del(&ch->list);

	cnm_clock_off(ch->cnm_core_instance);
	cnm_spin_unlock(ch->cnm_core_instance);

	ch->cnm_core_instance = NULL;
	osal_free(ch);

	return VCORE_DEC_SUCCESS;

err_dec_close:

	vlog_error("ch close fail\n");

	cnm_clock_off(ch->cnm_core_instance);
	cnm_spin_unlock(ch->cnm_core_instance);

	osal_free(ch);

	return VCORE_DEC_FAIL;
}

enum vcore_dec_ret cnm_dec_clear_dpb(void *vcore_id, unsigned int dpb_addr)
{
	struct dec_channel *ch = (struct dec_channel *)vcore_id;
	RetCode ret = RETCODE_SUCCESS;
	int dpb_index=0;
	unsigned int dpb_i;
	CodecInst *pcodecinst;
	DecInfo *pdecinfo;

	cnm_spin_lock(ch->cnm_core_instance);

	pcodecinst = ch->handle;
	pdecinfo = &pcodecinst->CodecInfo.decInfo;

	/* find index */
	for (dpb_i = 0; dpb_i < VDEC_MAX_NUM_OF_DPB; dpb_i++) {
		if (ch->linear_dpb_paddr[dpb_i] == dpb_addr) {
			dpb_index = dpb_i;
			break;
		}
	}

	if (dpb_i == VDEC_MAX_NUM_OF_DPB) {
		vlog_error("can't find dpb (x%08X)\n", dpb_addr);
		goto err_clear_dpb;
	}

	/* vlog_info("clear dpb 0x%08X (%d)\n", dpb_addr, dpb_index ); */
	ret = VPU_DecClrDispFlag(ch->handle, dpb_index);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("clear dpb fail .. 0x%08X. (%d)\n", dpb_addr, ret);
		goto err_clear_dpb;
	}
	else {
		if (!(ch->disp_flag & (1 << dpb_index)))
			vlog_error("disp flag: 0x%08X, index:%d 0x%x\n", \
				ch->disp_flag, dpb_index, pdecinfo->clearDisplayIndexes);
		ch->disp_flag &= ~(1 << dpb_index);
		ch->f_dpb_full = VCORE_FALSE;
	}

	cnm_spin_unlock(ch->cnm_core_instance);

	return VCORE_DEC_SUCCESS;

err_clear_dpb:
	cnm_spin_unlock(ch->cnm_core_instance);

	return VCORE_DEC_FAIL;
}

enum vcore_dec_ret cnm_dec_register_dpb(void *vcore_id,
										struct vcore_dec_fb *fbinfo)
{
	RetCode ret = RETCODE_SUCCESS;
	struct dec_channel *ch= (struct dec_channel *)vcore_id;
	int linear_dpb_i;
	int tiled_dpb_i;
	void *reorder_id;
	int lock;
	FrameBufferAllocInfo vpu_fbinfo = {0};
	CodecInst *pcodecinst;
	DecInfo *pdecinfo;

	pcodecinst = ch->handle;
	pdecinfo = &pcodecinst->CodecInfo.decInfo;

	reorder_id = reorder_queue_init(fbinfo->num, ch->ref_frame_cnt);
	if (reorder_id == NULL) {
		vlog_error("reorder_queue_init... fail \n");
		return VCORE_DEC_FAIL;
	}

	cnm_spin_lock(ch->cnm_core_instance);
	lock = cnm_ch_is_locked(ch->cnm_core_instance, ch);
	if (lock != 0) {
		vlog_print(VLOG_VCORE_DEC,"is_lock %d\n", lock);
		cnm_spin_unlock(ch->cnm_core_instance);
		reorder_queue_deinit(reorder_id);
		return VCORE_DEC_RETRY;
	}

	cnm_clock_on(ch->cnm_core_instance);
#if 0
	{
		int i;

		printk("\ninst %d_%d\n", ch->handle->coreIdx, ch->handle->instIndex);

		for(i=0; i<fbinfo->num; i++)
			printk("tiled 0x%08x++0x%08x\n", fbinfo->tiled_dpb_addr[i],
				VPU_GetFrameBufSize(ch->core_idx, fbinfo->buf_width,
								fbinfo->buf_height, ch->maptype,
								FORMAT_420, CBCR_INTERLEAVE,
								&pdecinfo->dramCfg));

		for(i=0; i<fbinfo->num; i++)
			printk("linear 0x%08x++0x08%x\n", fbinfo->linear_dpb_addr[i],
				VPU_GetFrameBufSize(ch->core_idx, fbinfo->buf_width,
								fbinfo->buf_height, LINEAR_FRAME_MAP,
								FORMAT_420, CBCR_INTERLEAVE,
								&pdecinfo->dramCfg));

		printk("mvcol 0x%08x++0x%08x\n", fbinfo->mvcol_buf.addr, fbinfo->mvcol_buf.size);
		printk("slice 0x%08x++0x%08x\n", fbinfo->slice_buf.addr, fbinfo->slice_buf.size);
	}
#endif

	/* slice */
	if (fbinfo->slice_buf.size) {
		vpu_buffer_t slice_buf = {0};

		slice_buf.size = fbinfo->slice_buf.size;
		slice_buf.phys_addr = (unsigned int)fbinfo->slice_buf.addr;
		VPU_DecGiveCommand(ch->handle, DEC_SET_EXT_SLICE_BUFF, &slice_buf);
	}

	vpu_fbinfo.num = fbinfo->num;
	vpu_fbinfo.stride = fbinfo->buf_width;
	vpu_fbinfo.height = fbinfo->buf_height;
	vpu_fbinfo.type = FB_TYPE_CODEC;
	vpu_fbinfo.mapType = ch->maptype;

	memset(ch->dpb_local, 0x0, sizeof(FrameBuffer)*VDEC_MAX_NUM_OF_DPB*2);
#ifdef VCORE_WTL_ENABLE
	/* tiled */
	for (tiled_dpb_i = 0; tiled_dpb_i < fbinfo->num; tiled_dpb_i++) {
		ch->dpb_local[tiled_dpb_i].bufY = fbinfo->tiled_dpb_addr[tiled_dpb_i];
		ch->dpb_local[tiled_dpb_i].bufCb = (unsigned int)-1;
		ch->dpb_local[tiled_dpb_i].bufCr = (unsigned int)-1;
	}
	/* mvcol */
	ch->dpb_local[0].bufMvCol = fbinfo->mvcol_buf.addr;

	ret = VPU_DecAllocateFrameBuffer(ch->handle, vpu_fbinfo, &ch->dpb_local[0]);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_DecAllocateFrameBuffer... fail (%d)\n", ret);
		goto err_dec_register_dpb;
	}

	/* linear */
	vpu_fbinfo.mapType = LINEAR_FRAME_MAP;
	for (linear_dpb_i = fbinfo->num;
		linear_dpb_i < fbinfo->num*2;
		linear_dpb_i++) {
		ch->dpb_local[linear_dpb_i].bufY =
			fbinfo->linear_dpb_addr[linear_dpb_i - fbinfo->num];
		ch->dpb_local[linear_dpb_i].bufCb = (unsigned int)-1;
		ch->dpb_local[linear_dpb_i].bufCr = (unsigned int)-1;
		ch->linear_dpb_paddr[linear_dpb_i - fbinfo->num] =
			fbinfo->linear_dpb_addr[linear_dpb_i - fbinfo->num];
	}

	ret = VPU_DecAllocateFrameBuffer(ch->handle, vpu_fbinfo,
								&ch->dpb_local[fbinfo->num]);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_DecAllocateFrameBuffer... fail (%d)\n", ret);
		goto err_dec_register_dpb;
	}
#else
	for (linear_dpb_i = 0; linear_dpb_i < fbinfo->num; linear_dpb_i++) {
		ch->dpb_local[linear_dpb_i].bufY = fbinfo->linear_dpb_addr[linear_dpb_i];
		ch->dpb_local[linear_dpb_i].bufCb = (unsigned int)-1;
		ch->dpb_local[linear_dpb_i].bufCr = (unsigned int)-1;
		ch->linear_dpb_paddr[linear_dpb_i] = fbinfo->linear_dpb_addr[linear_dpb_i];
	}

	/* mvcol */
	ch->dpb_local[0].bufMvCol = fbinfo->mvcol_buf.addr;

	vpu_fbinfo.mapType = LINEAR_FRAME_MAP;
	ret = VPU_DecAllocateFrameBuffer(ch->handle, vpu_fbinfo, &ch->dpb_local[0]);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_DecAllocateFrameBuffer... fail (%d)\n", ret);
		goto err_dec_register_dpb;
	}
#endif

	ret = VPU_DecRegisterFrameBuffer(ch->handle, ch->dpb_local,
								fbinfo->num, fbinfo->buf_width,
								fbinfo->buf_height, ch->maptype);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_DecRegisterFrameBuffer... fail (%d)\n", ret);
		goto  err_dec_register_dpb;
	}

	ch->total_fb_num = fbinfo->num;
	ch->f_register_dpb = VCORE_TRUE;
	ch->reorder_id = reorder_id;
	_cnm_dec_set_dpb_disp_flag(ch, ch->total_fb_num);

	vlog_trace("register dpb OK ... fb_num(%d) buf_width(%d) buf_height(%d)\n",
						fbinfo->num , fbinfo->buf_width, fbinfo->buf_height);

	cnm_clock_off(ch->cnm_core_instance);
	cnm_spin_unlock(ch->cnm_core_instance);
	return VCORE_DEC_SUCCESS;

err_dec_register_dpb :

	cnm_clock_off(ch->cnm_core_instance);
	cnm_spin_unlock(ch->cnm_core_instance);

	reorder_queue_deinit(reorder_id);
	return VCORE_DEC_FAIL;
}

enum vcore_dec_ret cnm_dec_update_buffer(void *vcore_id,
						struct vcore_dec_au *au, vcore_bool_t *running)
{
	struct dec_channel *ch = (struct dec_channel *)vcore_id;
	DecParam param_dec = {0};
	RetCode ret = RETCODE_SUCCESS;
	int lock;

	cnm_spin_lock(ch->cnm_core_instance);
	cnm_clock_on(ch->cnm_core_instance);

	vlog_print(VLOG_VCORE_CHUNK, "uid %d, ts %lld, size %d\n", \
			au->meta.uid, au->meta.timestamp, au->buf.size);

	if (ch->au.meta.uid +1 !=  au->meta.uid)
		vlog_warning("uid %d -> %d\n", ch->au.meta.uid, au->meta.uid);

	switch (au->buf.au_type)
	{
		case VCORE_DEC_AU_SEQUENCE:
			lock = cnm_ch_is_locked(ch->cnm_core_instance, ch);
			if (lock != 0) {
				cnm_spin_unlock(ch->cnm_core_instance);

				*running = VCORE_FALSE;
				return VCORE_DEC_RETRY;
			}

			cnm_update_run_time(ch->cnm_core_instance, 0);
			cnm_ch_lock(ch->cnm_core_instance, ch, cnm_dec_isr);

			vlog_trace("AU_SEQUENCE\n");
			if (au->buf.size > 0) {
				if (_cnm_dec_update_rdwr_ptr(ch, au) == VCORE_FALSE)
					goto update_buffer_err;
			}
			else
				vlog_error("seq hdr size:%u\n", au->buf.size);

			if (ch->f_seq_init == VCORE_TRUE) {
				vlog_error("already sequence init done\n");
				goto update_buffer_err;
			}

			vlog_print(VLOG_VCORE_DEC,
					"VPU_DecIssueSeqInit : phy_addr(0x%08X)\n",
					au->buf.start_phy_addr);
			vlog_print(VLOG_VCORE_DEC,
					"VPU_DecIssueSeqInit : vir_ptr(0x%08X), size(0x%x)\n",
					(unsigned int)au->buf.start_vir_ptr,
					au->buf.size);

			osal_memcpy(&ch->au, au, sizeof(struct vcore_dec_au));
			ret = VPU_DecIssueSeqInit(ch->handle);
			if (ret != RETCODE_SUCCESS) {
				vlog_error("VPU_DecIssueSeqInit... fail (%d)\n", ret);
				goto update_buffer_err;
			}
			break;
		case VCORE_DEC_AU_PICTURE:
			if ((ch->f_seq_init == VCORE_FALSE) ||
				(ch->f_register_dpb == VCORE_FALSE)) {
				vlog_error("not initialized yet. seq %d, dpb %d\n", \
								ch->f_seq_init, ch->f_register_dpb);
				cnm_spin_unlock(ch->cnm_core_instance);

				*running = VCORE_FALSE;
				return VCORE_DEC_RETRY;
			}
			if (ch->f_dpb_full == VCORE_TRUE) {
				vlog_error("dpb full\n");
				cnm_spin_unlock(ch->cnm_core_instance);

				*running = VCORE_FALSE;
				return VCORE_DEC_RETRY;
			}

			lock = cnm_ch_is_locked(ch->cnm_core_instance, ch);
			if (ch->f_chunk_more_request == VCORE_TRUE) {
				if (lock != 1) {
					cnm_spin_unlock(ch->cnm_core_instance);
					*running = VCORE_FALSE;
					return VCORE_DEC_RETRY;
				}
			}
			else if (lock != 0){
				cnm_spin_unlock(ch->cnm_core_instance);

				*running = VCORE_FALSE;
				return VCORE_DEC_RETRY;
			}

			cnm_update_run_time(ch->cnm_core_instance, 0);
			cnm_ch_lock(ch->cnm_core_instance, ch, cnm_dec_isr);

			_cnm_dec_set_rotation(ch,
								ch->handle,
								au,
								ch->codec_type,
								ch->pic_height,
								ch->pic_width);

			if (ch->f_chunk_reuse_required == VCORE_TRUE) {
				ret = VPU_DecStartOneFrame(ch->handle, &param_dec);
				if (ret == RETCODE_SUCCESS) {
					ch->f_chunk_reuse_required = VCORE_FALSE;
				}
				else {
					vlog_error("VPU_DecStartOneFrame... fail - reuse (%d)\n", ret);
					goto update_buffer_err;
				}

				cnm_spin_unlock(ch->cnm_core_instance);

				*running = VCORE_TRUE;
				return VCORE_DEC_RETRY;
			}

			if (au->eos == VCORE_TRUE) {
				if (ch->f_eos == VCORE_TRUE) {
					vlog_error("already received EoS\n");
					cnm_spin_unlock(ch->cnm_core_instance);

					*running = VCORE_TRUE;
					return VCORE_DEC_SUCCESS;
				}

				vlog_trace("EOS start ... size(%d) ch %x\n", au->buf.size, (unsigned int)ch);
				ch->f_eos = VCORE_TRUE;
				VPU_DecUpdateBitstreamBuffer(ch->handle, 0);
			}
			else if (ch->f_eos == VCORE_TRUE) {
				vlog_trace("restart stream\n");
				ch->f_eos = VCORE_FALSE;
				VPU_DecUpdateBitstreamBuffer(ch->handle, -1);
			}

			if (au->buf.size > 0) {
				if (_cnm_dec_update_rdwr_ptr(ch, au) == VCORE_FALSE)
					goto update_buffer_err;
			}
			else
				vlog_print(VLOG_VCORE_DEC,"buf.size 0x%x\n", au->buf.size );

			osal_memcpy(&ch->au, au, sizeof(struct vcore_dec_au));

			if (ch->f_chunk_more_request == VCORE_TRUE) {
				ch->f_chunk_more_request = VCORE_FALSE;

				vlog_print(VLOG_VCORE_DEC, "chunkMoreRequest\n");
				VpuWriteReg(ch->core_idx, BIT_INT_REASON, 0);

				cnm_spin_unlock(ch->cnm_core_instance);

				*running = VCORE_TRUE;
				return VCORE_DEC_SUCCESS;
			}

			vlog_print(VLOG_VCORE_DEC,"VPU_DecStartOneFrame. ch 0x%X\n",
										(unsigned int)ch);
#ifdef CNM_WORKAROUND
			VpuWriteReg(ch->core_idx, BIT_MSG_0, 0);
#endif
			ret = VPU_DecStartOneFrame(ch->handle, &param_dec);
			if (ret != RETCODE_SUCCESS) {
				vlog_error("VPU_DecStartOneFrame... fail (%d), inst %d_%d\n",
						ret, ch->handle->coreIdx, ch->handle->instIndex);
				goto update_buffer_err;
			}
			break;
		default:
			vlog_error("au type %d\n", au->buf.au_type);
			goto update_buffer_err;
			break;
	}

	if (ch->continuous_chunk < 0x0FFFFFFF)
		ch->continuous_chunk++;

	cnm_spin_unlock(ch->cnm_core_instance);

	*running = VCORE_TRUE;
	return VCORE_DEC_SUCCESS;

update_buffer_err:
	if (cnm_ch_unlock(ch->cnm_core_instance, ch) < 0)
		vlog_error("unlock: 0x%X\n", (unsigned int)ch);

	cnm_clock_off(ch->cnm_core_instance);
	cnm_spin_unlock(ch->cnm_core_instance);

	*running = VCORE_FALSE;
	return VCORE_DEC_FAIL;
}

enum vcore_dec_ret cnm_dec_flush(void *vcore_id, unsigned int rd_addr)
{
	struct dec_channel *ch = (struct dec_channel *)vcore_id;
	RetCode ret = RETCODE_SUCCESS;
	int lock;

	cnm_spin_lock(ch->cnm_core_instance);
	lock = cnm_ch_is_locked(ch->cnm_core_instance, ch);
	if (lock != 0) {
		cnm_spin_unlock(ch->cnm_core_instance);
		return VCORE_DEC_RETRY;
	}

	cnm_clock_on(ch->cnm_core_instance);

	cnm_update_run_time(ch->cnm_core_instance, 0);
	cnm_ch_lock(ch->cnm_core_instance, ch, cnm_dec_isr);

	if (ch->cpb.phy_addr <= rd_addr && rd_addr < (ch->cpb.phy_addr+ch->cpb.size)) {
		ret = VPU_DecSetRdPtr(ch->handle, rd_addr, 1);
		if (ret != RETCODE_SUCCESS) {
			vlog_error("VPU_DecSetRdPtr... fail(%d) \n", ret);
			goto err_dec_flush;
		}
	}
	else
		vlog_error("invalid addr 0x%08x, cpb 0x%08x++0x%08x\n",
				rd_addr, ch->cpb.phy_addr, ch->cpb.phy_addr+ch->cpb.size);

	vlog_trace("FLUSH >> 0x%08X \n", rd_addr);

	ret = VPU_DecFrameBufferFlush(ch->handle);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_DecSetRdPtr... fail(%d) \n", ret);
		goto err_dec_flush;
	}

	_cnm_dec_set_dpb_disp_flag(ch, ch->total_fb_num);
	if (ch->reorder_id != NULL)
		reorder_queue_flush(ch->reorder_id);

	ch->continuous_chunk = 0;
	ch->f_dpb_full = VCORE_TRUE;
	ch->f_chunk_more_request = VCORE_FALSE;
	ch->f_chunk_reuse_required = VCORE_FALSE;
	ch->dec_flag = 0;

	cnm_spin_unlock(ch->cnm_core_instance);

	return VCORE_DEC_SUCCESS;

err_dec_flush :

	cnm_ch_unlock(ch->cnm_core_instance, ch);
	cnm_clock_off(ch->cnm_core_instance);
	cnm_spin_unlock(ch->cnm_core_instance);

	return VCORE_DEC_FAIL;
}

void cnm_dec_reset(void *vcore_id)
{
	struct dec_channel *ch = (struct dec_channel *)vcore_id;
	struct dec_channel *_ch, *tmp;
	struct vcore_dec_report dec_report;
	struct cnm_dec_instance *cnm_dec_instance = ch->cnm_dec_instance;

	if (cnm_dec_instance == NULL)
		return;

	list_for_each_entry_safe(_ch, tmp, &cnm_dec_instance->channel_list, list) {
		dec_report.hdr = VCORE_DEC_RESET;
		dec_report.info.reset = VCORE_DEC_REPORT_RESET_START;
		vlog_error("callback ... ch(0x%X) reset start \n", (unsigned int)_ch);

		_ch->cb_vcore_dec_report(_ch->vdc_id, &dec_report);
	}

	cnm_spin_lock(ch->cnm_core_instance);

	vlog_error("lock %d, inst %d_%d\n", cnm_ch_is_locked(ch->cnm_core_instance, ch),
			ch->handle->coreIdx, ch->handle->instIndex);
	cnm_clock_on(ch->cnm_core_instance);
	cnm_ch_lock(ch->cnm_core_instance, ch, cnm_dec_isr);
/*	vdi_log(ch->core_idx, 3, 0);*/

	cnm_spin_unlock(ch->cnm_core_instance);

	cnm_reset(ch->cnm_core_instance, ch->handle);
}

void cnm_dec_report_reset(void *_cnm_dec_instance)
{
	struct dec_channel *ch, *tmp;
	struct vcore_dec_report dec_report;
	struct cnm_dec_instance *cnm_dec_instance = \
							(struct cnm_dec_instance *)_cnm_dec_instance;
	if (cnm_dec_instance == NULL)
		return;

	list_for_each_entry_safe(ch, tmp, &cnm_dec_instance->channel_list, list) {
		cnm_spin_lock(ch->cnm_core_instance);

		dec_report.hdr = VCORE_DEC_RESET;
		dec_report.info.reset = VCORE_DEC_REPORT_RESET_END;

		vlog_error("callback ... ch(0x%X) reset end \n", (unsigned int)ch);

		cnm_ch_unlock(ch->cnm_core_instance, ch);

		_cnm_dec_set_dpb_disp_flag(ch, ch->total_fb_num);
		ch->f_dpb_full = VCORE_TRUE;
		ch->f_chunk_more_request = VCORE_FALSE;
		ch->f_chunk_reuse_required = VCORE_FALSE;
/*		reorder_queue_flush(ch->reorder_id);*/
/*		ch->dec_flag = 0;*/

		cnm_spin_unlock(ch->cnm_core_instance);

		ch->cb_vcore_dec_report(ch->vdc_id, &dec_report);
		cnm_dec_broadcast(ch->cnm_core_instance, cnm_dec_instance, ch);
	}
}

void cnm_dec_broadcast(void *cnm_core_instance, void *_cnm_dec_instance, void *last_vcore_id)
{
	bool last_vcore_id_is_dec = false;
	struct dec_channel *ch, *tmp;
	struct vcore_dec_report dec_report;
	struct cnm_dec_instance *cnm_dec_instance;

	cnm_spin_lock(cnm_core_instance);

	cnm_dec_instance = (struct cnm_dec_instance *)_cnm_dec_instance;
	if (cnm_dec_instance == NULL) {
		cnm_spin_unlock(cnm_core_instance);
		vlog_error("cnm_dec_instance is NULL\n");
		return;
	}

	dec_report.hdr = VCORE_DEC_FEED;

	list_for_each_entry_safe(ch, tmp, &cnm_dec_instance->channel_list, list) {
		if (ch == last_vcore_id) {
			last_vcore_id_is_dec = true;
			continue;
		}
		ch->cb_vcore_dec_report(ch->vdc_id, &dec_report);
	}

	if (last_vcore_id_is_dec) {
		ch = (struct dec_channel *)last_vcore_id;
		ch->cb_vcore_dec_report(ch->vdc_id, &dec_report);
	}

	cnm_spin_unlock(cnm_core_instance);
}

void *cnm_dec_init(void)
{
	struct cnm_dec_instance *instance = \
							osal_malloc(sizeof(struct cnm_dec_instance));
	if (instance == NULL)
		return NULL;

	INIT_LIST_HEAD(&instance->channel_list);

	return (void*)instance;
}



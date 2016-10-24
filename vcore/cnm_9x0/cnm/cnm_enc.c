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

#include <linux/vmalloc.h>

#include <media/odin/vcodec/vcore/encoder.h>
#include <media/odin/vcodec/vlog.h>

#include "vcore/cnm_9x0/cnm/cnm.h"
#include "vcore/cnm_9x0/cnm/cnm_enc.h"

#include "vcore/cnm_9x0/cnm_ddk/vpuapi/vpuapi.h"
#include "vcore/cnm_9x0/cnm_ddk/vpuapi/vpuapifunc.h"

#define DEFAULT_RUNNING_WEIGHT_RESIDUAL	60
#define DEFAULT_RUNNING_WEIGHT_DIVIDER	1

#define ALIGN16(x)	( ((x)+(unsigned int)0xf) & (~(unsigned int)0xf))

struct cnm_enc_instance
{
	struct list_head channel_list;
};

struct epb_buf
{
	unsigned int phy_addr;
	unsigned char *vir_ptr;
	unsigned int size;
};

struct enc_epb
{
	struct epb_buf buf;
	unsigned int start_phy_addr;
	unsigned int end_phy_addr;
	unsigned char *vir_ptr;
};

struct enc_channel
{
	enum vcore_enc_codec codec_type;

	EncHandle handle;
	void *vec_id;

	void *cnm_core_instance;
	struct cnm_enc_instance *cnm_enc_instance;
	unsigned int core_idx;

	struct list_head list;
	struct enc_epb epb;
	unsigned int rd_addr_update;

	unsigned int ref_frame_cnt;
	EncOpenParam conf;
	unsigned int buf_width;
	unsigned int buf_height;
	int map_type;
	vcore_bool_t f_seq_init;
	vcore_bool_t f_dpb_register;
	vcore_bool_t f_intra_refresh;

	FrameBuffer dpb_local[VENC_MAX_NUM_OF_DPB];

	struct
	{
		unsigned int residual;
		unsigned int divider;
	} frame_rate;

	unsigned int running_weight;

	/* CBs */
	void (*cb_vcore_enc_report)(void *vec_id,
								struct vcore_enc_report *vcore_report);
};

static void __cnm_enc_default_conf(EncOpenParam *param,
								struct vcore_enc_config *config)
{
	param->ringBufferEnable = 1;
	param->linear2TiledEnable = 1;
	param->linear2TiledMode = 1;
	param->MESearchRangeX = 0;
	param->MESearchRangeY = 0;
	param->userQpMax = -1;
	param->userMaxDeltaQp = -1;
	param->userQpMin = -1;
	param->userMinDeltaQp = -1;
	param->MEUseZeroPmv = 0;
	param->intraCostWeight = 0;
	param->rcIntraQp = -1;
	param->userGamma = -1;
	param->idrInterval = 1;

	/* performance up even if bitrate on*/
	param->rcIntervalMode = 3; /* 3: user defined Mb_level */
	param->mbInterval = config->buf_width >> 4 ; /*number of MB per Row */

	if (config->codec_type == VCORE_ENC_AVC) {
		param->EncStdParam.avcParam.constrainedIntraPredFlag = 0;
		param->EncStdParam.avcParam.disableDeblk = 1;
		param->EncStdParam.avcParam.deblkFilterOffsetAlpha = -6;
		param->EncStdParam.avcParam.deblkFilterOffsetBeta = -6;
		param->EncStdParam.avcParam.chromaQpOffset = 0;
		param->EncStdParam.avcParam.audEnable = 0;

		param->EncStdParam.avcParam.frameCroppingFlag = 0;
		param->EncStdParam.avcParam.frameCropLeft = 0;
		param->EncStdParam.avcParam.frameCropTop = 0;
		param->EncStdParam.avcParam.frameCropRight = 0;
		param->EncStdParam.avcParam.frameCropBottom = 0;
		if (config->buf_width != config->pic_width) {
			param->EncStdParam.avcParam.frameCropRight = \
							ALIGN16(config->pic_width) - config->pic_width;
		}
		if (config->buf_height != config->pic_height) {
			param->EncStdParam.avcParam.frameCropBottom = \
							ALIGN16(config->pic_height) - config->pic_height;
		}
		if (param->EncStdParam.avcParam.frameCropLeft ||
			param->EncStdParam.avcParam.frameCropTop ||
			param->EncStdParam.avcParam.frameCropRight ||
			param->EncStdParam.avcParam.frameCropBottom) {
			param->EncStdParam.avcParam.frameCroppingFlag = 1;
		}

		param->EncStdParam.avcParam.level = 0;
		param->EncStdParam.avcParam.profile = 0;
		param->EncStdParam.avcParam.fieldFlag = 0;
		param->EncStdParam.avcParam.fieldRefMode = 1;
		param->EncStdParam.avcParam.chromaFormat400 = 0;
		param->EncStdParam.avcParam.ppsNum = 1;
		param->EncStdParam.avcParam.ppsParam[0].ppsId = 0;
		param->EncStdParam.avcParam.ppsParam[0].entropyCodingMode = 0;
		param->EncStdParam.avcParam.ppsParam[0].cabacInitIdc = 0;
		param->EncStdParam.avcParam.ppsParam[0].transform8x8Mode = 0;
		param->EncStdParam.avcParam.ppsParam[1].ppsId = 0;
		param->EncStdParam.avcParam.ppsParam[1].entropyCodingMode = 0;
		param->EncStdParam.avcParam.ppsParam[1].cabacInitIdc = 0;
		param->EncStdParam.avcParam.ppsParam[1].transform8x8Mode = 0;
	}
	else if (config->codec_type == VCORE_ENC_MPEG4) {
		param->EncStdParam.mp4Param.mp4DataPartitionEnable = 0;
		param->EncStdParam.mp4Param.mp4ReversibleVlcEnable = 0;
		param->EncStdParam.mp4Param.mp4IntraDcVlcThr = 0;
		param->EncStdParam.mp4Param.mp4HecEnable = 0;
		param->EncStdParam.mp4Param.mp4Verid = 0;
	}
	else if (config->codec_type == VCORE_ENC_H263) {
		param->EncStdParam.h263Param.h263AnnexIEnable=0;
		param->EncStdParam.h263Param.h263AnnexJEnable=0;
		param->EncStdParam.h263Param.h263AnnexKEnable=0;
		param->EncStdParam.h263Param.h263AnnexTEnable=0;
	}

	param->frameCacheBypass = 0;
	param->frameCacheBurst = 1;
	param->frameCacheMerge = 3;
	param->frameCacheWayShape = 15;
	param->cbcrInterleave = 0;
	param->frameEndian = 0;
	param->streamEndian = 0;

	param->lineBufIntEn = 0;
	param->coreIdx = 0;
}

static void _cnm_enc_set_config(EncOpenParam *param,
							struct vcore_enc_config *config)
{
	__cnm_enc_default_conf(param, config);

	param->bitstreamBuffer = config->epb_phy_addr;
	param->bitstreamBufferSize = config->epb_size;
	param->picWidth = config->pic_width;
	param->picHeight = config->pic_height;

	switch (config->codec_type) {
	case VCORE_ENC_AVC :
		param->bitstreamFormat = STD_AVC;
		if (config->codec_param.h264.mb_num_per_slice == 0)
			param->sliceMode.sliceMode = 0;
		else {
			param->sliceMode.sliceMode = 1;
			param->sliceMode.sliceSizeMode = 1;
			param->sliceMode.sliceSize = \
								config->codec_param.h264.mb_num_per_slice;
		}

		switch (config->codec_param.h264.profile) {
		case VCORE_ENC_H264_PROFILE_BASE :
			param->EncStdParam.avcParam.profile = 0;
			break;
		case VCORE_ENC_H264_PROFILE_MAIN :
			param->EncStdParam.avcParam.profile = 1;
			break;
		case VCORE_ENC_H264_PROFILE_HIGH :
			param->EncStdParam.avcParam.profile = 2;
			break;
		default :
			vlog_error("invalid h264.profile %d\n",
								config->codec_param.h264.profile);
			break;
		}

		switch (config->codec_param.h264.loop_filter_mode) {
		case VCORE_ENC_LOOP_FILTER_ENABLE :
			param->EncStdParam.avcParam.disableDeblk = 0;
			break;
		case VCORE_ENC_LOOP_FILTER_DISABLE :
			param->EncStdParam.avcParam.disableDeblk = 1;
			break;
		case VCORE_ENC_LOOP_FILTER_DISABLE_SLICE_BOUNDARY :
			param->EncStdParam.avcParam.disableDeblk = 2;
			break;
		default :
			vlog_error("invalid h264.loop_filter_mode %d\n",
								config->codec_param.h264.loop_filter_mode);
			break;
		}

		param->gopSize = config->codec_param.h264.gop_size;
		param->EncStdParam.avcParam.level = config->codec_param.h264.level_idc;
		param->EncStdParam.avcParam.constrainedIntraPredFlag = \
			(config->codec_param.h264.const_intra_pred_enable == VCORE_TRUE) ? \
				1 : 0;
		param->EncStdParam.avcParam.fieldFlag = \
				(config->codec_param.h264.field_enable == VCORE_TRUE) ? 1 : 0;
		param->EncStdParam.avcParam.ppsParam[0].cabacInitIdc = \
				config->codec_param.h264.cabac_init_idx;
		param->EncStdParam.avcParam.ppsParam[0].entropyCodingMode =
				(config->codec_param.h264.cabac_enable == VCORE_TRUE) ? 1 : 0;
		break;
	case VCORE_ENC_MPEG4 :
		param->bitstreamFormat = STD_MPEG4;
		param->EncStdParam.mp4Param.mp4IntraDcVlcThr = \
				config->codec_param.mpeg4.intra_dc_vlc_threshold;
		param->gopSize = config->codec_param.mpeg4.gop_size;

		if (config->codec_param.mpeg4.short_video_header_enable == VCORE_TRUE)
			param->bitstreamFormat  = STD_H263;

		/* error_correction */
		param->EncStdParam.mp4Param.mp4HecEnable = \
				(config->error_correction.hec_enable == VCORE_TRUE) ? 1 : 0;
		param->EncStdParam.mp4Param.mp4DataPartitionEnable = \
			(config->error_correction.data_partitioning_enable == VCORE_TRUE) ?\
				1 : 0;
		param->EncStdParam.mp4Param.mp4ReversibleVlcEnable = \
			(config->error_correction.reversible_vlc_enble == VCORE_TRUE) ? \
				1 : 0;

		break;
	case VCORE_ENC_H263 :
		param->bitstreamFormat = STD_H263;
		param->gopSize = config->codec_param.h263.gop_size;
		param->MESearchRangeX = 3;
		param->MESearchRangeY = 2;
		switch (config->codec_param.h263.profile) {
		case VCORE_ENC_H263_PROFILE_BASE :
			param->EncStdParam.h263Param.h263AnnexIEnable = 0;
			param->EncStdParam.h263Param.h263AnnexJEnable = 0;
			param->EncStdParam.h263Param.h263AnnexTEnable = 0;

			/* for GOB */
			param->EncStdParam.h263Param.h263AnnexKEnable = 0;
			param->sliceMode.sliceMode = 1;
			break;
		case VCORE_ENC_H263_PROFILE_SWV2 :
			/* AnnexI is Not supported */
			param->EncStdParam.h263Param.h263AnnexIEnable = 0;
			param->EncStdParam.h263Param.h263AnnexJEnable = 1;
			param->EncStdParam.h263Param.h263AnnexKEnable = 1;
			param->EncStdParam.h263Param.h263AnnexTEnable = 1;
			break;
		default :
			break;
		}
		break;
	case VCORE_ENC_JPEG :
		break;
	}

	/* frame rate */
	if (config->frame_rate.divider == 0) {
		vlog_error("divider value is 0\n");
		config->frame_rate.divider = 1;
	}
	param->frameRateInfo = (config->frame_rate.divider-1) << 16;
	param->frameRateInfo |= config->frame_rate.residual;

	/* bitrate */
	switch (config->bit_rate.control_mode) {
	case VCORE_ENC_RATE_CONTROL_DISABLE :
		param->rcEnable = 0;
		param->bitRate = 0;
		param->enableAutoSkip = \
					(config->bit_rate.auto_skip_enable == VCORE_TRUE) ? 1 : 0;
		break;
	case VCORE_ENC_RATE_CONTROL_CBR :
		param->rcEnable = 1;
		param->initialDelay = 1000;
		param->bitRate = config->bit_rate.target_kbps;
		param->enableAutoSkip = \
					(config->bit_rate.auto_skip_enable == VCORE_TRUE) ? 1 : 0;
		break;
	case VCORE_ENC_RATE_CONTROL_ABR :
		param->rcEnable = 2;
		param->bitRate = config->bit_rate.target_kbps;
		param->enableAutoSkip =
					(config->bit_rate.auto_skip_enable == VCORE_TRUE) ? 1 : 0;
		break;
	default :
		vlog_error("invalid bit_rate.control_mode %d\n",
						config->bit_rate.control_mode);
		break;
	}

	/* cbcr interleave */
	switch (config->cbcr_interleave) {
	case VCORE_ENC_CBCR_INTERLEAVE_NONE :
		param->cbcrInterleave = 0;
		break;
	case VCORE_ENC_CBCR_INTERLEAVE_NV12 :
		param->cbcrInterleave = 1;
		param->cbcrOrder = 0;
		break;
	case VCORE_ENC_CBCR_INTERLEAVE_NV21 :
		param->cbcrInterleave = 1;
		param->cbcrOrder = 1;
		break;
	default :
		vlog_error("invalid cbcr_interleave %d\n", config->cbcr_interleave);
		break;
  }

	if (config->cyclic_intra_block_refresh.enable == VCORE_TRUE) {
		param->ConscIntraRefreshEnable =  1;
		param->intraRefresh = config->cyclic_intra_block_refresh.mb_num;
	}

}

static int _cnm_enc_cache_config(MaverickCacheConfig* pCache, int decoder,
									int interleave, int bypass,
									int burst, int merge,
									int mapType, int wayshape)
{
	int ret = 1;
	unsigned int cache_config = 0;

	if (decoder) /* decoder */
	{
		if (mapType == 0)	/* LINEAR_FRAME_MAP */
		{
			/* VC1 opposite field padding is not allowable in UV separated,
				burst 8 and linear map
			 */
			if (!interleave)
				burst = 0;

			wayshape = 15;

			if (merge == 1) {
				merge = 3;
				if (burst)
					burst = 0;
			}
		}
		else
		{
			/* horizontal merge constraint in tiled map */
			if (merge == 1)
				merge = 3;
		}
	}
	else	/* encoder */
	{
		if (mapType == 0)	/* LINEAR_FRAME_MAP */
		{
			wayshape = 15;
			/* GDI constraint. Width should not be over 64 */
			if ((merge == 1) && (burst))
				burst= 0;
		}
		else
		{
			/* horizontal merge constraint in tiled map */
			if (merge == 1)
				merge = 3;
		}

	}

	cache_config = (merge & 0x3) << 9;
	cache_config = cache_config | ((wayshape & 0xf) << 5);
	cache_config = cache_config | ((burst & 0x1) << 3);
	cache_config = cache_config | (bypass & 0x3);

	if (mapType != 0)	/* LINEAR_FRAME_MAP */
		cache_config = cache_config | 0x00000004;

	/* {16'b0, 5'b0, merge[1:0], wayshape[3:0],
					1'b0, burst[0], map[0], bypass[1:0]};
	 */
	pCache->CacheMode = cache_config;

	return ret;
}

static vcore_bool_t _cnm_enc_make_header(void *vcore_id)
{
	RetCode ret = RETCODE_SUCCESS;
	EncHeaderParam enc_header_param={0};
	struct enc_channel *ch = (struct enc_channel *)vcore_id;
	int i;

	if (ch->conf.bitstreamFormat== STD_MPEG4) {
		enc_header_param.headerType = VOS_HEADER;
		enc_header_param.size = ch->conf.bitstreamBufferSize;
		VPU_EncGiveCommand(ch->handle, ENC_PUT_VIDEO_HEADER, &enc_header_param);

		enc_header_param.headerType = VOL_HEADER;
		enc_header_param.size = ch->conf.bitstreamBufferSize;
		VPU_EncGiveCommand(ch->handle, ENC_PUT_VIDEO_HEADER, &enc_header_param);

	}
	else if (ch->conf.bitstreamFormat == STD_AVC ) {
		int active_pps_idx;

		enc_header_param.headerType = SPS_RBSP;
		enc_header_param.size = ch->conf.bitstreamBufferSize;
		ret = VPU_EncGiveCommand(ch->handle, ENC_PUT_VIDEO_HEADER, \
									&enc_header_param);
		if (ret != RETCODE_SUCCESS)
		{
			vlog_error("VPU_EncGiveCommand ( ENC_PUT_VIDEO_HEADER )");
			vlog_error(" for SPS_RBSP failed Error code is 0x%x \n", ret);
			return VCORE_FALSE;
		}
		VPU_EncGiveCommand(ch->handle, ENC_GET_ACTIVE_PPS, &active_pps_idx);

		enc_header_param.headerType = PPS_RBSP;
		enc_header_param.size = ch->conf.bitstreamBufferSize;
		for (i=0; i<ch->conf.EncStdParam.avcParam.ppsNum; i++) {
			VPU_EncGiveCommand(ch->handle, ENC_SET_ACTIVE_PPS, &i);
			ret = VPU_EncGiveCommand(ch->handle, ENC_PUT_VIDEO_HEADER, \
										&enc_header_param);
			if (ret != RETCODE_SUCCESS)
			{
				vlog_error("VPU_EncGiveCommand ( ENC_PUT_VIDEO_HEADER )");
				vlog_error(" for PPS_RBSP failed Error code is 0x%x \n", ret);
				return VCORE_FALSE;
			}
		}
		VPU_EncGiveCommand(ch->handle, ENC_SET_ACTIVE_PPS, &active_pps_idx);
	}

	return VCORE_TRUE;
}

static vcore_bool_t _cnm_enc_seq_init(struct enc_channel *ch)
{
	EncInitialInfo	initial_info = {0};
	RetCode ret = RETCODE_SUCCESS;

	ret = VPU_EncGetInitialInfo(ch->handle, &initial_info);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_EncGetInitialInfo failed. ret : %d\n", ret);
		return VCORE_FALSE;
	}

	ch->ref_frame_cnt= initial_info.minFrameBufferCount;

	return VCORE_TRUE;
}

vcore_bool_t _cnm_enc_update_epb_rdaddr(struct enc_channel *ch)
{
	RetCode ret = RETCODE_SUCCESS;
	unsigned int pa_rd_ptr, pa_wr_ptr;
	int encoded_size;
	int size;
	unsigned int offset;

	if (ch->rd_addr_update == 0x0) {
		return VCORE_FALSE;
	}

	ret = VPU_EncGetBitstreamBuffer(ch->handle, &pa_rd_ptr, &pa_wr_ptr,
								&encoded_size);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_EncGetBitstreamBuffer failed. ret : %d\n", ret );
		return VCORE_FALSE;
	}

	if (pa_rd_ptr > ch->rd_addr_update) /* wrap around */
		size = ch->epb.buf.size - (pa_rd_ptr - ch->rd_addr_update);
	else
		size = ch->rd_addr_update - pa_rd_ptr;

	if (size > encoded_size) {
		vlog_error(" encoded size(0x%X), size(0x%X)\n", encoded_size, size);
		return VCORE_FALSE;
	}

	offset = pa_rd_ptr - ch->epb.buf.phy_addr;
	vlog_print(VLOG_VCORE_ENC, "0x%X++0x%X=0x%X ",
								pa_rd_ptr, size, ch->rd_addr_update);
	vlog_print(VLOG_VCORE_ENC, "(0x%02X 0x%02X 0x%02X 0x%02X 0x%02X)\n",
								*(ch->epb.buf.vir_ptr + offset),
								*(ch->epb.buf.vir_ptr + offset+1),
								*(ch->epb.buf.vir_ptr + offset+2),
								*(ch->epb.buf.vir_ptr + offset+3),
								*(ch->epb.buf.vir_ptr + offset+4));

	ret = VPU_EncUpdateBitstreamBuffer(ch->handle, size);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_EncUpdateBitstreamBuffer failed. - \n");
		vlog_error("Error code is 0x%x, size(%u)\n", ret, size);
		return VCORE_FALSE;
	}

	ch->rd_addr_update = 0x0;

	return VCORE_TRUE;
}

static vcore_bool_t _cnm_enc_report_seq(void *vcore_id,
									struct vcore_enc_report *enc_report)
{
	struct enc_channel *ch = (struct enc_channel *)vcore_id;
	CodecInst *pcodec_inst;
	EncInfo *pencinfo;
	pcodec_inst = ch->handle;
	pencinfo = &pcodec_inst->CodecInfo.encInfo;

	ch->f_seq_init = VCORE_TRUE;
	vlog_trace("Seq Init ... OK\n");

	enc_report->hdr = VCORE_ENC_DONE;
	enc_report->info.done.hdr = VCORE_ENC_REPORT_SEQ;
	enc_report->info.done.info.seq.buf_width = ch->buf_width;
	enc_report->info.done.info.seq.buf_height = ch->buf_height;
	enc_report->info.done.info.seq.ref_frame_size =	\
				VPU_GetFrameBufSize(cnm_get_coreid(ch->cnm_core_instance),
				ch->buf_width, ch->buf_height,
				ch->map_type, FORMAT_420,
				ch->conf.cbcrInterleave, &pencinfo->dramCfg);
	enc_report->info.done.info.seq.ref_frame_cnt = ch->ref_frame_cnt;
	enc_report->info.done.info.seq.scratch_buf_size = \
				(ch->codec_type == VCORE_ENC_MPEG4 || \
				ch->codec_type == VCORE_ENC_H263) ? \
				SIZE_MP4ENC_DATA_PARTITION : 0;

	if (cnm_ch_unlock(ch->cnm_core_instance, ch) < 0)
		vlog_error("unlock: 0x%X\n", (unsigned int)ch);

	return VCORE_TRUE;
}

static vcore_bool_t _cnm_enc_report_pic(void *vcore_id,
									struct vcore_enc_report *enc_report)
{
	struct enc_channel *ch = (struct enc_channel *)vcore_id;
	RetCode ret = RETCODE_SUCCESS;
	EncOutputInfo	output_info;
	unsigned int offset;
	vcore_bool_t work_done = VCORE_FALSE;

	ret = VPU_EncGetOutputInfo(ch->handle, &output_info);
	if (ret == RETCODE_SUCCESS)
		work_done = VCORE_TRUE;
	else {
		vlog_error("VPU_EncGetOutputInfo failed. ret : %d\n", ret);
		vlog_error("GDI_WPROT_ERR_ADR 0x%08x\n",
			(unsigned int)VpuReadReg(ch->core_idx, GDI_WPROT_ERR_ADR));
	}

	ch->epb.end_phy_addr = output_info.wrPtr;

	enc_report->hdr = VCORE_ENC_DONE;
	enc_report->info.done.hdr = VCORE_ENC_REPORT_PIC;
	enc_report->info.done.info.pic.success = \
				(ret == RETCODE_SUCCESS) ? VCORE_TRUE : VCORE_FALSE;
	enc_report->info.done.info.pic.pic_type= \
				(output_info.picType == 0) ? VCORE_ENC_PIC_I : VCORE_ENC_PIC_P;

	enc_report->info.done.info.pic.start_phy_addr=ch->epb.start_phy_addr;
	enc_report->info.done.info.pic.end_phy_addr= ch->epb.end_phy_addr;

	if (ch->epb.start_phy_addr == ch->epb.end_phy_addr)
		vlog_error("encoded size 0. 0x%08X\n", ch->epb.start_phy_addr);

	if (ch->epb.start_phy_addr > ch->epb.end_phy_addr) { /* wrap around */
		enc_report->info.done.info.pic.size = \
			ch->epb.buf.size - (ch->epb.start_phy_addr -  ch->epb.end_phy_addr);
		vlog_print(VLOG_VCORE_ENC, "EPB WrapAround (0x%X - 0x%X)\n",
								enc_report->info.done.info.pic.start_phy_addr,
								enc_report->info.done.info.pic.end_phy_addr);
	}
	else {
		enc_report->info.done.info.pic.size =
						ch->epb.end_phy_addr - ch->epb.start_phy_addr;
	}

	offset = ch->epb.start_phy_addr - ch->epb.buf.phy_addr;
	vlog_print(VLOG_VCORE_ENC, "success (%d), encoded pic(%d)\n",
							ret, output_info.picType);
	vlog_print(VLOG_VCORE_ENC,
			"au:0x%X++0x%X=0x%X (0x%02X 0x%02X 0x%02X 0x%02X 0x%02X)\n",
					enc_report->info.done.info.pic.start_phy_addr,
					enc_report->info.done.info.pic.size,
					enc_report->info.done.info.pic.end_phy_addr,
					*(ch->epb.buf.vir_ptr + offset),
					*(ch->epb.buf.vir_ptr + offset+1),
					*(ch->epb.buf.vir_ptr + offset+2),
					*(ch->epb.buf.vir_ptr + offset+3),
					*(ch->epb.buf.vir_ptr + offset+4));
#if 0
	for (i=0; i<10 ; i++)
	{
		vlog_error("0x%08X : 0x%02X\n", ch->epb.buf.vir_ptr+offset+i,
										*(ch->epb.buf.vir_ptr+offset+i));
	}
#endif
	if (cnm_ch_unlock(ch->cnm_core_instance, ch) < 0)
		vlog_error("unlock: 0x%X\n", (unsigned int)ch);

	return work_done;
}

static void cnm_enc_isr(void *vcore_id, unsigned long reason)
{
	struct enc_channel *ch;
	struct vcore_enc_report enc_report;
	vcore_bool_t work_done = VCORE_FALSE;
	vcore_bool_t report = VCORE_FALSE;

	if (vcore_id == NULL) {
		vlog_error("no vcore id, reason: %x\n", (unsigned int)reason);
		return;
	}

	ch = (struct enc_channel *)vcore_id;
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
		report = _cnm_enc_report_seq(ch, &enc_report);
	}
	else if (reason & (1 << INT_BIT_SEQ_END)) {
		vlog_error("no handle of INT_BIT_SEQ_END\n");
	}
	else if (reason & (1 << INT_BIT_PIC_RUN)) {
		work_done = _cnm_enc_report_pic(ch, &enc_report);
		report = VCORE_TRUE;
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

	if (report == VCORE_TRUE)
		ch->cb_vcore_enc_report(ch->vec_id, &enc_report);
}

enum vcore_enc_ret cnm_enc_open(void **vcore_id, void *cnm_core_instance,
					void *cnm_enc_instance, struct vcore_enc_config *enc_config,
					unsigned int workbuf_paddr, unsigned long *workbuf_vaddr,
					unsigned int workbuf_size, void *vec_id,
					void (*vcore_enc_report)(void *vec_id,
						struct vcore_enc_report *vcore_report))
{
	RetCode ret = RETCODE_SUCCESS;
	EncHandle handle;
	EncOpenParam param = {0};
	struct enc_channel *ch;
	int lock;

	*vcore_id = NULL;

	ch = (struct enc_channel *)osal_malloc(sizeof(struct enc_channel));
	if (ch == NULL ) {
		vlog_error("malloc fail\n");
		return VCORE_ENC_FAIL;
	}

	cnm_spin_lock(cnm_core_instance);
	lock = cnm_ch_is_locked(cnm_core_instance, NULL);
	if (lock != 0) {
		vlog_print(VLOG_VCORE_ENC, "is_lock %d\n", lock);
		cnm_spin_unlock(cnm_core_instance);

		osal_free(ch);
		return VCORE_ENC_RETRY;
	}

	cnm_clock_on(cnm_core_instance);

	ch->cnm_enc_instance = cnm_enc_instance;
	ch->cnm_core_instance = cnm_core_instance;
	ch->core_idx = cnm_get_coreid(ch->cnm_core_instance);
	if (ch->core_idx == 0xFFFFFFFF) {
		vlog_error("core idx is invalid\n");
		cnm_spin_unlock(cnm_core_instance);

		osal_free(ch);
		return VCORE_ENC_FAIL;
	}

	vlog_print(VLOG_VCORE_ENC,"epb paddr 0x%X, vaddr 0x%X, size 0x%X\n",
							enc_config->epb_phy_addr,
							(unsigned int)enc_config->epb_vir_ptr,
							enc_config->epb_size);
	vlog_trace("buf_width (%d), buf_height (%d)",
							enc_config->buf_width, enc_config->buf_height);
	vlog_trace(" framerate(%d/%d), bitrate(%d %d)\n",
							enc_config->frame_rate.residual,
							enc_config->frame_rate.divider,
							enc_config->bit_rate.control_mode,
							enc_config->bit_rate.target_kbps);

	_cnm_enc_set_config(&param, enc_config);
	param.coreIdx = ch->core_idx;

	if (workbuf_size) {
		param.vbWork.phys_addr = (unsigned long)workbuf_paddr;
		param.vbWork.virt_addr = workbuf_vaddr;
		param.vbWork.base = (unsigned long)workbuf_vaddr;
		param.vbWork.size = (int)workbuf_size;
	}

	ret = VPU_EncOpen(&handle, &param);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_EncOpen failed. ret : %d\n", ret);
		goto err_enc_open;
	}

	ch->epb.buf.phy_addr = enc_config->epb_phy_addr;
	ch->epb.buf.vir_ptr = enc_config->epb_vir_ptr;
	ch->epb.buf.size = enc_config->epb_size;
	ch->conf = param;
	ch->handle = handle;
	ch->epb.start_phy_addr = enc_config->epb_phy_addr;
	ch->buf_width = enc_config->buf_width;
	ch->buf_height = enc_config->buf_height;
	ch->vec_id = vec_id;
	ch->map_type = TILED_FRAME_V_MAP;
	ch->cb_vcore_enc_report = vcore_enc_report;
	ch->codec_type = enc_config->codec_type;
	ch->rd_addr_update = 0x0;
	ch->f_seq_init = VCORE_FALSE;
	ch->f_dpb_register = VCORE_FALSE;
	ch->f_intra_refresh = VCORE_FALSE;

	/*VPU_EncGiveCommand(handle, ENABLE_LOGGING, 0);*/

	list_add_tail(&ch->list, &ch->cnm_enc_instance->channel_list);
	vlog_trace("ch open ok, pic_width (%d), pic_height (%d)\n",
				ch->conf.picWidth, ch->conf.picHeight);

	ch->frame_rate.residual = enc_config->frame_rate.residual;
	ch->frame_rate.divider = enc_config->frame_rate.divider;
	ch->frame_rate.residual = ch->frame_rate.residual ? \
			ch->frame_rate.residual : DEFAULT_RUNNING_WEIGHT_RESIDUAL;
	ch->frame_rate.divider = ch->frame_rate.divider ? \
			ch->frame_rate.divider : DEFAULT_RUNNING_WEIGHT_DIVIDER;

	ch->running_weight = \
			cnm_reserve_running_weight(ch->cnm_core_instance,
					ch->buf_width, ch->buf_height,
					ch->frame_rate.residual, ch->frame_rate.divider);
	cnm_update_run_time(ch->cnm_core_instance, 0);

	cnm_clock_off(cnm_core_instance);
	cnm_spin_unlock(cnm_core_instance);

	*vcore_id = ch;
	return VCORE_ENC_SUCCESS;

err_enc_open:

	cnm_clock_off(cnm_core_instance);
	cnm_spin_unlock(cnm_core_instance);

	osal_free(ch);
	return VCORE_ENC_FAIL;
}

enum vcore_enc_ret cnm_enc_close(void *vcore_id)
{
	struct enc_channel *ch = (struct enc_channel *)vcore_id;
	RetCode ret = RETCODE_SUCCESS;
	EncOutputInfo	output_info;
	int lock;

	cnm_spin_lock(ch->cnm_core_instance);
	lock = cnm_ch_is_locked(ch->cnm_core_instance, ch);
	if (lock != 0) {
		vlog_print(VLOG_VCORE_ENC, "is_lock %d\n", lock);
		cnm_spin_unlock(ch->cnm_core_instance);
		return VCORE_ENC_RETRY;
	}

	cnm_clock_on(ch->cnm_core_instance);

	VPU_EncGetOutputInfo(ch->handle, &output_info);  /* cnm ddk sequnece error */

	ret = VPU_EncClose(ch->handle);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_EncClose fail(%d)\n", ret);
		goto err_enc_close;
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
	return VCORE_ENC_SUCCESS;

err_enc_close:

	vlog_error("ch close fail\n");

	cnm_clock_off(ch->cnm_core_instance);
	cnm_spin_unlock(ch->cnm_core_instance);

	osal_free(ch);
	return VCORE_ENC_FAIL;
}

enum vcore_enc_ret cnm_enc_set_config(void *vcore_id,
									struct vcore_enc_running_config *config)
{
	struct enc_channel *ch = (struct enc_channel *)vcore_id;
	int lock;

	cnm_spin_lock(ch->cnm_core_instance);
	lock = cnm_ch_is_locked(ch->cnm_core_instance, ch);
	if (lock != 0) {
		vlog_print(VLOG_VCORE_ENC, "is_lock %d\n", lock);
		cnm_spin_unlock(ch->cnm_core_instance);
		return VCORE_ENC_RETRY;
	}

	cnm_clock_on(ch->cnm_core_instance);

	switch (config->index) {
	case VCORE_ENC_RUNNING_PARAM_BITRATE :
		vlog_trace("set bitrate %d kbps\n", config->data.bit_rate_kbps);
		VPU_EncGiveCommand(ch->handle, ENC_SET_BITRATE,
							&config->data.bit_rate_kbps);
		break;
	case VCORE_ENC_RUNNING_PARAM_INTRA_REFRESH :
		vlog_trace("set intra refresh\n");
		ch->f_intra_refresh = VCORE_TRUE;
		break;
	default :
		vlog_error("invalid index %d\n", config->index);
		break;
	}

	cnm_clock_off(ch->cnm_core_instance);
	cnm_spin_unlock(ch->cnm_core_instance);

	return VCORE_ENC_SUCCESS;
}

enum vcore_enc_ret cnm_enc_update_epb_rdaddr(void *vcore_id,
													unsigned int rd_addr)
{
	struct enc_channel *ch = (struct enc_channel *)vcore_id;

	cnm_spin_lock(ch->cnm_core_instance);

	ch->rd_addr_update = rd_addr;

	cnm_spin_unlock(ch->cnm_core_instance);

	return VCORE_ENC_SUCCESS;
}

enum vcore_enc_ret cnm_enc_update_buffer(void *vcore_id,
											struct vcore_enc_fb *fb)
{
	struct enc_channel *ch = (struct enc_channel *)vcore_id;
	RetCode ret = RETCODE_SUCCESS;
	FrameBuffer framebuffer = {0};
	EncParam enc_param = {0};
	unsigned int pa_rd_ptr;
	unsigned int size;
	unsigned int y_size = ch->buf_width * ch->buf_height;
	int lock;

	cnm_spin_lock(ch->cnm_core_instance);
	lock = cnm_ch_is_locked(ch->cnm_core_instance, ch);
	if (lock != 0) {
		vlog_print(VLOG_VCORE_ENC, "is_lock %d\n", lock);
		cnm_spin_unlock(ch->cnm_core_instance);
		return VCORE_ENC_RETRY;
	}

	cnm_clock_on(ch->cnm_core_instance);

	cnm_update_run_time(ch->cnm_core_instance, 0);
	cnm_ch_lock(ch->cnm_core_instance, ch, cnm_enc_isr );

	if (ch->f_seq_init == VCORE_FALSE) {
		_cnm_enc_seq_init(ch);
		vlog_trace("do seq_init\n");
		cnm_spin_unlock(ch->cnm_core_instance);
		return VCORE_ENC_RETRY;
	}

	if (ch->f_dpb_register == VCORE_FALSE) {
		vlog_warning("dpb is NOT registered\n");
		goto err_enc_update_buffer;
	}

	_cnm_enc_update_epb_rdaddr(ch);

	framebuffer.bufY = fb->fb_phy_addr;
	framebuffer.bufCb = framebuffer.bufY + y_size;
	framebuffer.bufCr = framebuffer.bufCb + (y_size>> 2);
	framebuffer.endian = ch->conf.frameEndian;
	framebuffer.cbcrInterleave = ch->conf.cbcrInterleave;
	framebuffer.sourceLBurstEn = 1;
	framebuffer.stride = ch->buf_width;
	framebuffer.height = ch->buf_height;
	framebuffer.myIndex = 2;

	enc_param.sourceFrame = &framebuffer;

	if (ch->f_intra_refresh == VCORE_TRUE) {
		enc_param.forceIPicture = 1;
		ch->f_intra_refresh = VCORE_FALSE;
	}
	enc_param.skipPicture   = 0;
	enc_param.quantParam    = 23;

	ret = VPU_EncGetBitstreamBuffer(ch->handle, &pa_rd_ptr,
								&ch->epb.start_phy_addr, &size);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_EncGetBitstreamBuffer failed. ret : %d\n", ret );
		goto err_enc_update_buffer;
	}

	vlog_print(VLOG_VCORE_ENC, "PIC_RUN\n");
	ret = VPU_EncStartOneFrame(ch->handle, &enc_param);
	if (ret != RETCODE_SUCCESS)	{
		vlog_error("VPU_EncStartOneFrame failed. ret : %d\n", ret);
		goto err_enc_update_buffer;
	}

	cnm_spin_unlock(ch->cnm_core_instance);
	return VCORE_ENC_SUCCESS;

err_enc_update_buffer :
	if (cnm_ch_unlock(ch->cnm_core_instance, ch) < 0)
		vlog_error("unlock: 0x%X\n", (unsigned int)ch);

	cnm_clock_off(ch->cnm_core_instance);
	cnm_spin_unlock(ch->cnm_core_instance);
	return VCORE_ENC_FAIL;
}

enum vcore_enc_ret cnm_enc_register_dpb(void *vcore_id,
											struct vcore_enc_dpb *fbinfo)
{
	struct enc_channel *ch = (struct enc_channel *)vcore_id;
	struct vcore_enc_report enc_report;
	RetCode ret = RETCODE_SUCCESS;
	unsigned int pa_rd_ptr;
	unsigned int size;

	SecAxiUse sec_axi_use;
	MaverickCacheConfig enc_cache_config;
	unsigned int dpb_i;
	FrameBufferAllocInfo vpu_fbinfo = {0};
	int lock;

	cnm_spin_lock(ch->cnm_core_instance);
	lock = cnm_ch_is_locked(ch->cnm_core_instance, ch);
	if (lock != 0) {
		vlog_print(VLOG_VCORE_ENC, "is_lock %d\n", lock);
		cnm_spin_unlock(ch->cnm_core_instance);
		return VCORE_ENC_RETRY;
	}

	cnm_clock_on(ch->cnm_core_instance);

	/* scratch buf */
	if ( fbinfo->scratch_buf.size ) {
		vpu_buffer_t scratch_buf = {0};

		scratch_buf.phys_addr = (unsigned long)fbinfo->scratch_buf.addr;
		scratch_buf.size = fbinfo->scratch_buf.size;
		VPU_EncGiveCommand(ch->handle, ENC_SET_EXT_SCRATCH_BUFF, &scratch_buf);
	}

	/* 2nd axi, cache */
	sec_axi_use.useBitEnable = 1;
	sec_axi_use.useIpEnable = 1;
	sec_axi_use.useDbkYEnable = 1;
	sec_axi_use.useDbkCEnable = 1;
	sec_axi_use.useBtpEnable = 1;
	sec_axi_use.useOvlEnable = 1;
	VPU_EncGiveCommand(ch->handle, SET_SEC_AXI, &sec_axi_use);

	_cnm_enc_cache_config(&enc_cache_config, 0,
								ch->conf.cbcrInterleave,
								ch->conf.frameCacheBypass,
								ch->conf.frameCacheBurst,
								ch->conf.frameCacheMerge,
								ch->map_type,
								ch->conf.frameCacheWayShape);
	VPU_EncGiveCommand(ch->handle, SET_CACHE_CONFIG, &enc_cache_config);

	/* dpb */
	vpu_fbinfo.num = fbinfo->num;
	vpu_fbinfo.stride = fbinfo->buf_width;
	vpu_fbinfo.height = fbinfo->buf_height;
	vpu_fbinfo.type = FB_TYPE_CODEC;
	vpu_fbinfo.mapType = ch->map_type;

	for (dpb_i = 0; dpb_i < fbinfo->num; dpb_i++) {
		ch->dpb_local[dpb_i].bufY = fbinfo->dpb_addr[dpb_i];
		ch->dpb_local[dpb_i].bufCb = (unsigned int)-1;
		ch->dpb_local[dpb_i].bufCr = (unsigned int)-1;
	}

	ret = VPU_EncAllocateFrameBuffer(ch->handle, vpu_fbinfo, &ch->dpb_local[0]);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_EncAllocateFrameBuffer... fail (%d)\n", ret);
		goto err_enc_register_dpb;
	}


	ret = VPU_EncRegisterFrameBuffer(ch->handle, ch->dpb_local,
								ch->ref_frame_cnt, ch->buf_width,
								ch->buf_height, ch->map_type);
	if (ret != RETCODE_SUCCESS) {
		vlog_error("VPU_EncRegisterFrameBuffer fail (%d)\n", ret);
		goto err_enc_register_dpb;
	}


	/* make seq_header */
	if (ch->codec_type != VCORE_ENC_H263) {
		_cnm_enc_make_header(ch);

		ret = VPU_EncGetBitstreamBuffer(ch->handle, &pa_rd_ptr, \
										&ch->epb.end_phy_addr, &size);
		if (ret != RETCODE_SUCCESS) {
			vlog_error("VPU_EncGetBitstreamBuffer fail. 0x%x\n", ret );
			goto err_enc_register_dpb;
		}

		enc_report.hdr = VCORE_ENC_DONE;
		enc_report.info.done.hdr = VCORE_ENC_REPORT_PIC;
		enc_report.info.done.info.pic.success = \
					(ret == RETCODE_SUCCESS) ? VCORE_TRUE : VCORE_FALSE;
		enc_report.info.done.info.pic.pic_type = VCORE_ENC_SEQ_HDR;
		enc_report.info.done.info.pic.start_phy_addr = ch->epb.buf.phy_addr;
		enc_report.info.done.info.pic.end_phy_addr = ch->epb.end_phy_addr;
		enc_report.info.done.info.pic.size = \
					ch->epb.end_phy_addr - ch->epb.buf.phy_addr;

		ch->cb_vcore_enc_report(ch->vec_id, &enc_report);

		vlog_trace("success (%d), encoded seq es:0x%X++0x%X=0x%X\n", ret,
							enc_report.info.done.info.pic.start_phy_addr,
							enc_report.info.done.info.pic.size,
							enc_report.info.done.info.pic.end_phy_addr);
		vlog_trace("(0x%02X 0x%02X 0x%02X 0x%02X 0x%02X)\n",
							*(ch->epb.buf.vir_ptr),
							*(ch->epb.buf.vir_ptr+1),
							*(ch->epb.buf.vir_ptr+2),
							*(ch->epb.buf.vir_ptr+3),
							*(ch->epb.buf.vir_ptr+4));
	}

	ch->f_dpb_register = VCORE_TRUE;
	vlog_trace("regitser dpb OK. %d\n", ch->ref_frame_cnt);

	cnm_clock_off(ch->cnm_core_instance);
	cnm_spin_unlock(ch->cnm_core_instance);
	return VCORE_ENC_SUCCESS;

err_enc_register_dpb :

	cnm_clock_off(ch->cnm_core_instance);
	cnm_spin_unlock(ch->cnm_core_instance);
	return VCORE_ENC_FAIL;
}

void cnm_enc_reset(void *vcore_id)
{
	struct enc_channel *ch = (struct enc_channel *)vcore_id;
	struct enc_channel *_ch, *tmp;
	struct vcore_enc_report enc_report;
	struct cnm_enc_instance *cnm_enc_instance = ch->cnm_enc_instance;

	if (cnm_enc_instance == NULL)
		return;

	list_for_each_entry_safe(_ch, tmp, &cnm_enc_instance->channel_list, list)
	{
		enc_report.hdr= VCORE_ENC_RESET;
		enc_report.info.reset = VCORE_ENC_REPORT_RESET_START;
		vlog_error("callback ... ch(0x%X) reset start \n", (unsigned int)_ch);

		_ch->cb_vcore_enc_report(_ch->vec_id, &enc_report);
	}

	cnm_spin_lock(ch->cnm_core_instance);
	cnm_clock_on(ch->cnm_core_instance);
	cnm_ch_lock(ch->cnm_core_instance, ch, cnm_enc_isr );
	cnm_spin_unlock(ch->cnm_core_instance);

	cnm_reset(ch->cnm_core_instance, ch->handle);
}

void cnm_enc_report_reset(void *_cnm_enc_instance)
{
	struct enc_channel *ch, *tmp;
	struct vcore_enc_report enc_report;
	struct cnm_enc_instance *cnm_enc_instance = \
				(struct cnm_enc_instance *)_cnm_enc_instance;
	if (cnm_enc_instance == NULL)
		return;

	list_for_each_entry_safe(ch, tmp, &cnm_enc_instance->channel_list, list) {
		cnm_spin_lock(ch->cnm_core_instance);

		enc_report.hdr= VCORE_ENC_RESET;
		enc_report.info.reset = VCORE_ENC_REPORT_RESET_END;
		vlog_error("callback ... ch(0x%X) reset end \n", (unsigned int)ch);

		cnm_ch_unlock(ch->cnm_core_instance, ch);
		cnm_spin_unlock(ch->cnm_core_instance);

		ch->cb_vcore_enc_report(ch->vec_id, &enc_report);
	}
}

void cnm_enc_broadcast(void *cnm_core_instance, void *_cnm_enc_instance, void *last_vcore_id)
{
	bool last_vcore_id_is_enc = false;
	struct enc_channel *ch, *tmp;
	struct vcore_enc_report enc_report;
	struct cnm_enc_instance *cnm_enc_instance;

	cnm_spin_lock(cnm_core_instance);

	cnm_enc_instance = (struct cnm_enc_instance *)_cnm_enc_instance;
	if (cnm_enc_instance == NULL) {
		cnm_spin_unlock(cnm_core_instance);
		vlog_error("cnm_enc_instance is NULL\n");
		return;
	}

	enc_report.hdr = VCORE_ENC_FEED;
	list_for_each_entry_safe(ch, tmp, &cnm_enc_instance->channel_list, list) {
		if (ch == last_vcore_id) {
			last_vcore_id_is_enc = true;
			continue;
		}
		ch->cb_vcore_enc_report(ch->vec_id, &enc_report);
	}

	if (last_vcore_id_is_enc) {
		ch = (struct enc_channel *)last_vcore_id;
		ch->cb_vcore_enc_report(ch->vec_id, &enc_report);
	}

	cnm_spin_unlock(cnm_core_instance);
}

void *cnm_enc_init(void)
{
	struct cnm_enc_instance *instance = vmalloc(sizeof(struct cnm_enc_instance));
	if (instance == NULL)
		return NULL;

	INIT_LIST_HEAD(&instance->channel_list);

	return (void*)instance;
}


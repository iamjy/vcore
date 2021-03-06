//--=========================================================================--
//  This file is a part of VPU Reference API project
//-----------------------------------------------------------------------------
//
//       This confidential and proprietary software may be used only
//     as authorized by a licensing agreement from Chips&Media Inc.
//     In the event of publication, the following notice is applicable:
//
//            (C) COPYRIGHT 2006 - 2011  CHIPS&MEDIA INC.
//                      ALL RIGHTS RESERVED
//
//       The entire notice above must be reproduced on all authorized
//       copies.
//
//--=========================================================================--

#include "vpuapifunc.h"
#include "vpuapi.h"



#ifdef BIT_CODE_FILE_PATH
#include BIT_CODE_FILE_PATH
#endif

static const Uint16* s_pusBitCode[MAX_NUM_VPU_CORE] = {NULL,};
static int s_bitCodeSize[MAX_NUM_VPU_CORE] = {0,};




Uint32 VPU_IsBusy(Uint32 coreIdx){

	Uint32 ret = 0;
	SetClockGate(coreIdx, 1);
	ret = VpuReadReg(coreIdx, BIT_BUSY_FLAG);
	SetClockGate(coreIdx, 0);

	return ret != 0;
}

Uint32 VPU_IsInit(Uint32 coreIdx)
{
	Uint32 pc;

	SetClockGate(coreIdx, 1);
	pc = VpuReadReg(coreIdx, BIT_CUR_PC);
	SetClockGate(coreIdx, 0);

	return pc;
}

Uint32	VPU_WaitInterrupt(Uint32 coreIdx, int timeout)
{
	Uint32 reason = 0;

	SetClockGate(coreIdx, 1);

	reason = vdi_wait_interrupt(coreIdx, timeout, BIT_INT_REASON);

	if (reason != (Uint32)-1)
		reason = VpuReadReg(coreIdx, BIT_INT_REASON);

	if (reason != (Uint32)-1) {
		VpuWriteReg(coreIdx, BIT_INT_CLEAR, 1);		// clear HW signal
	}

	SetClockGate(coreIdx, 0);

	return reason;
}

void VPU_ClearInterrupt(Uint32 coreIdx)
{
	SetClockGate(coreIdx, 1);
	VpuWriteReg(coreIdx, BIT_INT_REASON, 0);     // tell to F/W that HOST received an interrupt.
	VpuWriteReg(coreIdx, BIT_INT_CLEAR, 1);	// clear HW signal => move to VPU_WaitInterrupt. HW interrupt signal should be cleared as soon as possible.
	SetClockGate(coreIdx, 0);
}


int VPU_GetMvColBufSize(CodStd codStd, int width, int height, int num)
{
	int size_mvcolbuf;

	size_mvcolbuf = 0;
	if (codStd == STD_AVC || codStd == STD_VC1 || codStd == STD_MPEG4 || codStd == STD_H263 || codStd == STD_RV || codStd == STD_AVS ) {

		size_mvcolbuf =  ((width+31)&~31)*((height+31)&~31);
		size_mvcolbuf = (size_mvcolbuf*3)/2;
		size_mvcolbuf = (size_mvcolbuf+4)/5;
		size_mvcolbuf = ((size_mvcolbuf+7)/8)*8;

		if (codStd == STD_AVC) {
			size_mvcolbuf *= num;
		}
	}

	return size_mvcolbuf;
}



static int   VPU_GetFrameBufSizeV20(int width, int height, int mapType, int format, int interleave)
{
	int chr_size_y, chr_size_x;
	int chr_vscale, chr_hscale;
	int unit_size_hor_lum, unit_size_ver_lum, unit_size_hor_chr, unit_size_ver_chr;
	int size_dpb_lum, size_dpb_chr, size_dpb_all;
	int size_dpb_lum_4k, size_dpb_chr_4k, size_dpb_all_4k;
	int field_map = 0;

	switch (format)
	{
	case FORMAT_420:
		height = (height+1)/2*2;
		width = (width+1)/2*2;
		break;
	case FORMAT_224:
		height = (height+1)/2*2;
		break;
	case FORMAT_422:
		width = (width+1)/2*2;
		break;
	case FORMAT_444:
		break;
	case FORMAT_400:
		height = (height+1)/2*2;
		width = (width+1)/2*2;
		break;
	}

	chr_hscale  = (format == FORMAT_420 || format == FORMAT_422 ? 2 : 1);  // 4:2:0, 4:2:2h
	chr_vscale = (format == FORMAT_420 || format == FORMAT_224 ? 2 : 1);// 4:2:0, 4:2:2v

	if (mapType == LINEAR_FIELD_MAP || mapType == TILED_FIELD_V_MAP ||
		mapType == TILED_FIELD_NO_BANK_MAP)
		field_map = 1;
	else
		field_map = 0;

	if (height>width) {
		if ((mapType >= TILED_FRAME_V_MAP && mapType <= TILED_MIXED_V_MAP) ||
			mapType == TILED_FRAME_NO_BANK_MAP ||
			mapType == TILED_FIELD_NO_BANK_MAP)
			width = height;	// TiledMap constraints
	}

	if (mapType == LINEAR_FRAME_MAP || mapType == LINEAR_FIELD_MAP)
	{
		chr_size_y = ((height>>field_map)/chr_hscale);
		chr_size_x = interleave ? width : (width/chr_vscale);

		size_dpb_lum   = width * (height>>field_map);
		size_dpb_chr   = chr_size_y * chr_size_x;
		size_dpb_all = interleave ? (size_dpb_lum + size_dpb_chr)/*only one cbcr plane (interleaved)*/ : (size_dpb_lum + size_dpb_chr*2); /*two chroma plane (separate)*/
		if (field_map)
			size_dpb_all *= 2;
	}
	else if (mapType == TILED_FRAME_NO_BANK_MAP || mapType == TILED_FIELD_NO_BANK_MAP)
	{
		chr_size_y = ((height>>field_map)/chr_hscale);
		chr_size_x = interleave ? width : (width/chr_vscale);

		unit_size_hor_lum = (width > 2048) ? 4096 :
			(width > 1024) ? 2048 :
			(width >  512) ? 1024 : 512;
		unit_size_ver_lum = (((height>>field_map)+127)/128) * 128; // unit vertical size is 128 pixel (8MB)

		unit_size_hor_chr = (chr_size_x > 2048) ? 4096 :
			(chr_size_x > 1024) ? 2048 :
			(chr_size_x >  512) ? 1024 : 512;
		unit_size_ver_chr = ((chr_size_y+127)/128) * 128; // unit vertical size is 128 pixel (8MB)


		size_dpb_lum = unit_size_hor_lum * unit_size_ver_lum;
		size_dpb_chr = (format==FORMAT_400) ? 0 : (unit_size_hor_chr * unit_size_ver_chr);
		size_dpb_all = interleave ? (size_dpb_lum + size_dpb_chr) /*only one cbcr plane (interleaved)*/ :  (size_dpb_lum + size_dpb_chr*2); // two chroma plane (separate)
		if (field_map)
			size_dpb_all *= 2;
	}
	else if (mapType == TILED_FRAME_MB_RASTER_MAP || mapType == TILED_FIELD_MB_RASTER_MAP)
	{

		size_dpb_lum = width * height;
		size_dpb_chr = (format==FORMAT_400) ? 0 : (size_dpb_lum/chr_hscale);
		size_dpb_all = (size_dpb_lum + size_dpb_chr*2);

		size_dpb_lum_4k = ((size_dpb_lum+(16384-1))/16384)*16384;
		size_dpb_chr_4k = ((size_dpb_chr+(16384-1))/16384)*16384;
		size_dpb_all_4k = (size_dpb_lum_4k + 2*size_dpb_chr_4k);

		size_dpb_lum = size_dpb_lum_4k;
		size_dpb_chr = size_dpb_chr_4k;
		size_dpb_all = size_dpb_all_4k;
	}
	else
	{
		chr_size_y = ((height>>field_map)/chr_hscale);
		chr_size_x = interleave ? width : (width/chr_vscale);

		unit_size_hor_lum = (width > 2048) ? 4096 :
			(width > 1024) ? 2048 :
			(width >  512) ? 1024 : 512;
		unit_size_ver_lum = (((height>>field_map)+63)/64) * 64; // unit vertical size is 64 pixel (4MB)

		unit_size_hor_chr = (chr_size_x > 2048) ? 4096 :
			(chr_size_x > 1024) ? 2048 :
			(chr_size_x >  512) ? 1024 : 512;
		unit_size_ver_chr = ((chr_size_y+63)/64) * 64; // unit vertical size is 64 pixel (4MB)


		size_dpb_lum = unit_size_hor_lum * unit_size_ver_lum;
		size_dpb_chr = (format==FORMAT_400) ? 0 : (unit_size_hor_chr * unit_size_ver_chr);
		size_dpb_all = interleave ? (size_dpb_lum + size_dpb_chr)/*only one cbcr plane (interleaved)*/ : (size_dpb_lum + size_dpb_chr*2); /*two chroma plane (separate)*/
		if (field_map)
			size_dpb_all *= 2;
	}


	return size_dpb_all;
}

static int   VPU_GetFrameBufSizeV10(int width, int height, int mapType, int format, DRAMConfig *pDramCfg)
{
	int framebufSize = 0;
	int framebufWidth, framebufHeight;

	framebufWidth = ((width+15)&~15);
	framebufHeight = ((height+31)&~31);

	if (mapType == LINEAR_FRAME_MAP)
	{
		switch (format)
		{
		case FORMAT_420:
			framebufSize = framebufWidth * ((framebufHeight+1)/2*2) + ((framebufWidth+1)/2)*((framebufHeight+1)/2)*2;
			break;
		case FORMAT_224:
			framebufSize = framebufWidth * ((framebufHeight+1)/2*2) + framebufWidth*((framebufHeight+1)/2)*2;
			break;
		case FORMAT_422:
			framebufSize = framebufWidth * framebufHeight + ((framebufWidth+1)/2)*framebufHeight*2;
			break;
		case FORMAT_444:
			framebufSize = framebufWidth * framebufHeight * 3;
			break;
		case FORMAT_400:
			framebufSize = framebufWidth * framebufHeight;
			break;
		}

		framebufSize = (framebufSize+7)/8*8;
	}
	else if (mapType == TILED_FRAME_MB_RASTER_MAP || mapType == TILED_FIELD_MB_RASTER_MAP)
	{
		int	 divX, divY;
		int  lum_size, chr_size;
		int  dpbsizeluma_aligned;
		int  dpbsizechroma_aligned;
		int  dpbsizeall_aligned;

		divX = format == FORMAT_420 || format == FORMAT_422 ? 2 : 1;
		divY = format == FORMAT_420 || format == FORMAT_224 ? 2 : 1;

		lum_size   = framebufWidth * framebufHeight;
		chr_size   = lum_size/divX/divY;

		if (mapType == TILED_FRAME_MB_RASTER_MAP)
		{
			dpbsizeluma_aligned   =  ((lum_size + 16383)/16384)*16384;
			dpbsizechroma_aligned =  ((chr_size + 16383)/16384)*16384;
		}
		else
		{
			dpbsizeluma_aligned   =  ((lum_size + (0x8000-1))/0x8000)*0x8000;
			dpbsizechroma_aligned =  ((chr_size + (0x8000-1))/0x8000)*0x8000;
		}
		dpbsizeall_aligned    =  dpbsizeluma_aligned + 2*dpbsizechroma_aligned;

		framebufSize = dpbsizeall_aligned;
		framebufSize = (framebufSize+7)/8*8;
	}
	else
	{

		int  LumSizeYFrame,LumSizeYField,ChrSizeYFrame,ChrSizeYField;
		int  ChrFieldRasSize,ChrFrameRasSize,LumFieldRasSize,LumFrameRasSize;
		int  VerSizePerRas,HorSizePerRas,Ras1DBit;

		int	 divX, divY;
		int  lum_size, chr_size;


		divX = format == FORMAT_420 || format == FORMAT_422 ? 2 : 1;
		divY = format == FORMAT_420 || format == FORMAT_224 ? 2 : 1;

		lum_size   = framebufWidth * framebufHeight;
		chr_size   = lum_size/divX/divY;


		LumSizeYFrame = framebufHeight;
		LumSizeYField = ((LumSizeYFrame+1)>>1);
		ChrSizeYFrame = framebufHeight/divY;
		ChrSizeYField = ((ChrSizeYFrame+1)>>1);


		if (mapType == TILED_FRAME_V_MAP)
		{
			if (pDramCfg->casBit == 9 && pDramCfg->bankBit == 2 && pDramCfg->rasBit == 13)	// CNN setting
			{
				VerSizePerRas = 64;
				HorSizePerRas = 256;
				Ras1DBit = 3;
			}
			else if(pDramCfg->casBit == 10 && pDramCfg->bankBit == 3 && pDramCfg->rasBit == 13)
			{
				VerSizePerRas = 64;
				HorSizePerRas = 512;
				Ras1DBit = 2;
			}
			else
				return 0;
		}
		else if (mapType == TILED_FRAME_H_MAP)
		{
			if (pDramCfg->casBit == 9 && pDramCfg->bankBit == 2 && pDramCfg->rasBit == 13)	// CNN setting
			{
				VerSizePerRas = 64;
				HorSizePerRas = 256;
				Ras1DBit = 3;
			}
			else if(pDramCfg->casBit == 10 && pDramCfg->bankBit == 3 && pDramCfg->rasBit == 13)
			{
				VerSizePerRas = 64;
				HorSizePerRas = 512;
				Ras1DBit = 2;
			}
			else
				return 0;
		}
		else if (mapType == TILED_FIELD_V_MAP)
		{
			if (pDramCfg->casBit == 9 && pDramCfg->bankBit == 2 && pDramCfg->rasBit == 13)	// CNN setting
			{
				VerSizePerRas = 64;
				HorSizePerRas = 256;
				Ras1DBit = 3;
			}
			else if(pDramCfg->casBit == 10 && pDramCfg->bankBit == 3 && pDramCfg->rasBit == 13)
			{
				VerSizePerRas = 64;
				HorSizePerRas = 512;
				Ras1DBit = 2;
			}
			else
				return 0;
		}
		else
		{
			if (pDramCfg->casBit == 9 && pDramCfg->bankBit == 2 && pDramCfg->rasBit == 13)	// CNN setting
			{
				VerSizePerRas = 64;
				HorSizePerRas = 256;
				Ras1DBit = 3;
			}
			else if(pDramCfg->casBit == 10 && pDramCfg->bankBit == 3 && pDramCfg->rasBit == 13)
			{
				VerSizePerRas = 64;
				HorSizePerRas = 512;
				Ras1DBit = 2;
			}
			else
				return 0;
		}

		UNREFERENCED_PARAMETER(chr_size);
		UNREFERENCED_PARAMETER(HorSizePerRas);
		UNREFERENCED_PARAMETER(LumSizeYField);



		ChrFieldRasSize = ((ChrSizeYField + (VerSizePerRas-1))/VerSizePerRas) << Ras1DBit;
		ChrFrameRasSize = ChrFieldRasSize *2;
		LumFieldRasSize = ChrFrameRasSize;
		LumFrameRasSize = LumFieldRasSize *2;

		framebufSize = (LumFrameRasSize + ChrFrameRasSize) << (pDramCfg->bankBit+pDramCfg->casBit+pDramCfg->busBit);
		framebufSize = (framebufSize+7)/8*8;
	}


	return framebufSize;
}


int VPU_GetFrameBufSize(int coreIdx, int width, int height, int mapType, int format, int interleave, DRAMConfig *pDramCfg)
{
	int ret;
	int productId;
	Uint32 val;

	UNREFERENCED_PARAMETER(pDramCfg);
	UNREFERENCED_PARAMETER(interleave);

	SetClockGate(coreIdx, 1);
	val = VpuReadReg(coreIdx, DBG_CONFIG_REPORT_1);
    if ((val&0xff00) == 0x3200) val = 0x3200;
	SetClockGate(coreIdx, 0);

	if (val == CODA960_CODE || val == BODA950_CODE) {
		productId = PRODUCT_ID_960;
	}
	else if (val == CODA980_CODE || val == WAVE320_CODE) {
		productId = PRODUCT_ID_980;
	}
	else {
		productId = PRODUCT_ID_980;	// coda7L
	}

	switch(productId)
	{
	case PRODUCT_ID_980:
		ret = VPU_GetFrameBufSizeV20(width, height, mapType, format, interleave);
		break;
	case PRODUCT_ID_960:
		ret = VPU_GetFrameBufSizeV10(width, height, mapType, format, pDramCfg);
		break;
	default:
		ret = 0;
		break;
	}

	return ret;
}




int VPU_GetProductId(int coreIdx)
{
	int productId;
	Uint32 val;
	char *pstr;

	if (vdi_init(coreIdx, 0, 0, 0, 0, 0) < 0)
		return RETCODE_FAILURE;

	SetClockGate(coreIdx, 1);

	val = vdi_read_register(coreIdx, DBG_CONFIG_REPORT_0);
	pstr = (char*)&val;
	pstr = pstr;

	val = vdi_read_register(coreIdx, DBG_CONFIG_REPORT_1);
    if ((val&0xff00) == 0x3200) val = 0x3200;
	if (val == CODA960_CODE || val == BODA950_CODE) {
		productId = PRODUCT_ID_960;
	}
	else if (val == CODA980_CODE || val == CODA980_CODE) {
		productId = PRODUCT_ID_980;
	}
	else {
		productId = 0;
	}
	SetClockGate(coreIdx, 0);

	vdi_release(coreIdx);
	return productId;
}

Uint32 VPU_GetOpenInstanceNum(Uint32 coreIdx)
{
	return vdi_get_instance_num(coreIdx);
}

static RetCode InitializeVPU(Uint32 coreIdx, const Uint16* code, Uint32 size,
							Uint32 reg_paddr,
							Uint32 *reg_vaddr,
							Uint32 cbuf_paddr,
							Uint32 *cbuf_vaddr,
							Uint32 cbuf_size)
{
	Uint32 data;
	vpu_buffer_t vb;
	PhysicalAddress tempBuffer;
	PhysicalAddress paraBuffer;
	PhysicalAddress codeBuffer;

	if (vdi_init(coreIdx, reg_paddr, reg_vaddr, cbuf_paddr, cbuf_vaddr, cbuf_size) < 0)
		return RETCODE_FAILURE;

	InitCodecInstancePool(coreIdx);

	EnterLock(coreIdx);

	vdi_get_common_memory((unsigned long)coreIdx, &vb);

	codeBuffer = vb.phys_addr;
	tempBuffer = codeBuffer + CODE_BUF_SIZE;
	paraBuffer = tempBuffer + TEMP_BUF_SIZE;

	if (VpuReadReg(coreIdx, BIT_CUR_PC) != 0) {

		VpuWriteReg(coreIdx, BIT_CODE_BUF_ADDR, codeBuffer);
		VpuWriteReg(coreIdx, BIT_PARA_BUF_ADDR, paraBuffer);
		VpuWriteReg(coreIdx, BIT_TEMP_BUF_ADDR, tempBuffer);
		LeaveLock(coreIdx);
		return RETCODE_CALLED_BEFORE;
	}

	VPU_SWReset(coreIdx, SW_RESET_ON_BOOT, NULL);

	SetClockGate(coreIdx, 1);

	BitLoadFirmware(coreIdx, codeBuffer, code, size);

	VpuWriteReg(coreIdx, BIT_PARA_BUF_ADDR, paraBuffer);
	VpuWriteReg(coreIdx, BIT_CODE_BUF_ADDR, codeBuffer);
	VpuWriteReg(coreIdx, BIT_TEMP_BUF_ADDR, tempBuffer);

	VpuWriteReg(coreIdx, BIT_BIT_STREAM_CTRL, VPU_STREAM_ENDIAN);
	VpuWriteReg(coreIdx, BIT_FRAME_MEM_CTRL, CBCR_INTERLEAVE<<2|VPU_FRAME_ENDIAN);
	VpuWriteReg(coreIdx, BIT_BIT_STREAM_PARAM, 0);

	VpuWriteReg(coreIdx, BIT_AXI_SRAM_USE, 0);
	VpuWriteReg(coreIdx, BIT_INT_ENABLE, 0);
	VpuWriteReg(coreIdx, BIT_ROLLBACK_STATUS, 0);

	data = (1<<INT_BIT_BIT_BUF_FULL);
	data |= (1<<INT_BIT_BIT_BUF_EMPTY);
	data |= (1<<INT_BIT_DEC_MB_ROWS);
	data |= (1<<INT_BIT_SEQ_INIT);
	data |= (1<<INT_BIT_DEC_FIELD);
	data |= (1<<INT_BIT_PIC_RUN);
	data |= (1<<INT_BIT_DEC_BUF_FLUSH);



	VpuWriteReg(coreIdx, BIT_INT_ENABLE, data);
	VpuWriteReg(coreIdx, BIT_INT_CLEAR, 0x1);


	VpuWriteReg(coreIdx, BIT_BUSY_FLAG, 0x1);
	VpuWriteReg(coreIdx, BIT_CODE_RESET, 1);
	VpuWriteReg(coreIdx, BIT_CODE_RUN, 1);

	if (vdi_wait_vpu_busy(coreIdx, VPU_BUSY_CHECK_TIMEOUT, BIT_BUSY_FLAG) == -1) {
		LeaveLock(coreIdx);
		return RETCODE_VPU_RESPONSE_TIMEOUT;
	}

	LeaveLock(coreIdx);
	return RETCODE_SUCCESS;
}

RetCode VPU_Init(Uint32 coreIdx)
{
#ifdef BIT_CODE_FILE_PATH
	s_pusBitCode[coreIdx] = bit_code;
	s_bitCodeSize[coreIdx] = sizeof(bit_code)/sizeof(bit_code[0]);
#endif

	if (s_bitCodeSize[coreIdx] == 0)
		return RETCODE_NOT_FOUND_BITCODE_PATH;

	return InitializeVPU(coreIdx, s_pusBitCode[coreIdx], s_bitCodeSize[coreIdx], 0, 0, 0, 0, 0);
}

RetCode VPU_InitWithBitcode(Uint32 coreIdx, const Uint16* code, Uint32 size,
							Uint32 reg_paddr,
							Uint32 *reg_vaddr,
							Uint32 cbuf_paddr,
							Uint32 *cbuf_vaddr,
							Uint32 cbuf_size)
{
	if (coreIdx >= MAX_NUM_VPU_CORE)
		return RETCODE_INVALID_PARAM;
	if (code == NULL || size == 0)
		return RETCODE_INVALID_PARAM;

#ifdef BIT_CODE_FILE_PATH
	s_pusBitCode[coreIdx] = NULL;
	s_pusBitCode[coreIdx] = (Uint16 *)osal_malloc(size*sizeof(Uint16));
	if (!s_pusBitCode[coreIdx])
		return RETCODE_INSUFFICIENT_RESOURCE;
	osal_memcpy((void *)s_pusBitCode[coreIdx], (const void *)code, size*sizeof(Uint16));
	s_bitCodeSize[coreIdx] = size;
#endif

	return InitializeVPU(coreIdx, code, size, reg_paddr, reg_vaddr, cbuf_paddr, cbuf_vaddr, cbuf_size);
}

void VPU_DeInit(Uint32 coreIdx)
{
#ifdef BIT_CODE_FILE_PATH
	if (s_pusBitCode[coreIdx])
		osal_free((void *)s_pusBitCode[coreIdx]);
	s_pusBitCode[coreIdx] = NULL;
	s_bitCodeSize[coreIdx] = 0;
#endif
	vdi_release(coreIdx);
}

RetCode VPU_GetVersionInfo(Uint32 coreIdx, Uint32 *versionInfo, Uint32 *revision, Uint32 *productId)
{
	Uint32 ver;
	Uint32 rev;
	Uint32 pid;

	EnterLock(coreIdx);

	if (versionInfo && revision) {

		if (VpuReadReg(coreIdx, BIT_CUR_PC) == 0) {
			return RETCODE_NOT_INITIALIZED;
		}

		if (GetPendingInst(coreIdx)) {
			LeaveLock(coreIdx);
			return RETCODE_FRAME_NOT_COMPLETE;
		}

		VpuWriteReg(coreIdx, RET_FW_VER_NUM , 0);

		BitIssueCommand(coreIdx, NULL, FIRMWARE_GET);
		if (vdi_wait_vpu_busy(coreIdx, VPU_BUSY_CHECK_TIMEOUT, BIT_BUSY_FLAG) == -1) {
			LeaveLock(coreIdx);
			return RETCODE_VPU_RESPONSE_TIMEOUT;
		}


		ver = VpuReadReg(coreIdx, RET_FW_VER_NUM);
		rev = VpuReadReg(coreIdx, RET_FW_CODE_REV);

		if (ver == 0) {
			LeaveLock(coreIdx);
			return RETCODE_FAILURE;
		}

		*versionInfo = ver;
		*revision = rev;
	}

	pid = VpuReadReg(coreIdx, DBG_CONFIG_REPORT_1);
    if ((pid&0xff00) == 0x3200) pid = 0x3200;
	if (productId)
		*productId = pid;

	LeaveLock(coreIdx);
	return RETCODE_SUCCESS;
}

RetCode VPU_DecOpen(DecHandle * pHandle, DecOpenParam * pop)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	int val, i;
	RetCode ret;

	ret = CheckDecOpenParam(pop);
	if (ret != RETCODE_SUCCESS) {
		return ret;
	}

	EnterLock(pop->coreIdx);


	if (VpuReadReg(pop->coreIdx, BIT_CUR_PC) == 0) {
		LeaveLock(pop->coreIdx);
		return RETCODE_NOT_INITIALIZED;
	}


	ret = GetCodecInstance(pop->coreIdx, &pCodecInst);
	if (ret != RETCODE_SUCCESS) {
		*pHandle = 0;
		LeaveLock(pop->coreIdx);
		return ret;
	}

	*pHandle = pCodecInst;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	pDecInfo->openParam = *pop;

	pCodecInst->codecMode = 0;
	pCodecInst->codecModeAux = 0;
	if (pop->bitstreamFormat == STD_MPEG4) {
		pCodecInst->codecMode = MP4_DEC;
		pCodecInst->codecModeAux = MP4_AUX_MPEG4;
	}
	else if (pop->bitstreamFormat == STD_AVC) {
		pCodecInst->codecMode = AVC_DEC;
		pCodecInst->codecModeAux = pop->avcExtension;
	}
	else if (pop->bitstreamFormat == STD_VC1) {
		pCodecInst->codecMode = VC1_DEC;
	}
	else if (pop->bitstreamFormat == STD_MPEG2) {
		pCodecInst->codecMode = MP2_DEC;
	}
	else if (pop->bitstreamFormat == STD_H263) {
		pCodecInst->codecMode = MP4_DEC;
	}
	else if (pop->bitstreamFormat == STD_DIV3) {
		pCodecInst->codecMode = DV3_DEC;
		pCodecInst->codecModeAux = MP4_AUX_DIVX3;
	}
	else if (pop->bitstreamFormat == STD_RV) {
		pCodecInst->codecMode = RV_DEC;
	}
	else if (pop->bitstreamFormat == STD_AVS) {
		pCodecInst->codecMode = AVS_DEC;
	}
	else if (pop->bitstreamFormat == STD_THO) {
		pCodecInst->codecMode = VPX_DEC;
		pCodecInst->codecModeAux = VPX_AUX_THO;
	}
	else if (pop->bitstreamFormat == STD_VP3) {
		pCodecInst->codecMode = VPX_DEC;
		pCodecInst->codecModeAux = VPX_AUX_THO;
	}
	else if (pop->bitstreamFormat == STD_VP8) {
		pCodecInst->codecMode = VPX_DEC;
		pCodecInst->codecModeAux = VPX_AUX_VP8;
	}

	pDecInfo->wtlEnable = pop->wtlEnable;
	pDecInfo->wtlMode = pop->wtlMode;
	if (!pDecInfo->wtlEnable)
		pDecInfo->wtlMode = 0;
	pDecInfo->streamWrPtr = pop->bitstreamBuffer;
	pDecInfo->streamRdPtr = pop->bitstreamBuffer;

	pDecInfo->streamRdPtrRegAddr = BIT_RD_PTR;
	pDecInfo->streamWrPtrRegAddr = BIT_WR_PTR;
	pDecInfo->frameDisplayFlagRegAddr = BIT_FRM_DIS_FLG;

	pDecInfo->frameDisplayFlag = 0;
	pDecInfo->clearDisplayIndexes   = 0;
	pDecInfo->frameDelay = -1;

	pDecInfo->streamBufStartAddr = pop->bitstreamBuffer;
	pDecInfo->streamBufSize = pop->bitstreamBufferSize;
	pDecInfo->streamBufEndAddr = pop->bitstreamBuffer + pop->bitstreamBufferSize;

	pDecInfo->stride = 0;
	pDecInfo->vbFrame.size = 0;
	pDecInfo->vbWTL.size = 0;	//coda960 only
	pDecInfo->vbPPU.size = 0;
	pDecInfo->frameAllocExt = 0;
	pDecInfo->ppuAllocExt = 0;
	pDecInfo->reorderEnable = VPU_REORDER_ENABLE;
	pDecInfo->seqInitEscape = 0;
	pDecInfo->rotationEnable = 0;
	pDecInfo->mirrorEnable = 0;
	pDecInfo->mirrorDirection = MIRDIR_NONE;
	pDecInfo->rotationAngle = 0;
	pDecInfo->rotatorOutputValid = 0;
	pDecInfo->rotatorStride = 0;
	pDecInfo->deringEnable	= 0;
	pDecInfo->secAxiInfo.useBitEnable  = 0;
	pDecInfo->secAxiInfo.useIpEnable   = 0;
	pDecInfo->secAxiInfo.useDbkYEnable = 0;
	pDecInfo->secAxiInfo.useDbkCEnable = 0;
	pDecInfo->secAxiInfo.useOvlEnable  = 0;
	pDecInfo->secAxiInfo.useBtpEnable  = 0;
	pDecInfo->secAxiInfo.bufBitUse    = 0;
	pDecInfo->secAxiInfo.bufIpAcDcUse = 0;
	pDecInfo->secAxiInfo.bufDbkYUse   = 0;
	pDecInfo->secAxiInfo.bufDbkCUse   = 0;
	pDecInfo->secAxiInfo.bufOvlUse    = 0;
	pDecInfo->secAxiInfo.bufBtpUse    = 0;
	pDecInfo->initialInfoObtained = 0;
	pDecInfo->vc1BframeDisplayValid = 0;
	pDecInfo->userDataBufAddr = 0;
	pDecInfo->userDataEnable = 0;
	pDecInfo->userDataBufSize = 0;
	pDecInfo->vbUserData.size = 0;
	pDecInfo->dramCfg.rasBit = EM_RAS;		//coda960 only
	pDecInfo->dramCfg.casBit = EM_CAS;		//coda960 only
	pDecInfo->dramCfg.bankBit = EM_BANK;	//coda960 only
	pDecInfo->dramCfg.busBit = EM_WIDTH;	//coda960 only
	pDecInfo->avcErrorConcealMode = AVC_ERROR_CONCEAL_MODE_DEFAULT;

	pDecInfo->vbSlice.base = 0;
	pDecInfo->vbSlice.virt_addr = 0;
	pDecInfo->vbSlice.size = 0;

	if (pop->vbWork.size) {
		pDecInfo->vbWork = pop->vbWork;
		pDecInfo->workBufferAllocExt = 1;
	}
	else {
		pDecInfo->vbWork.size		= WORK_BUF_SIZE;
		if (pop->bitstreamFormat == STD_AVC) {
			pDecInfo->vbWork.size += PS_SAVE_SIZE;
		}
		if (vdi_allocate_dma_memory(pCodecInst->coreIdx, &pDecInfo->vbWork) < 0)
		{
			*pHandle = 0;
			LeaveLock(pCodecInst->coreIdx);
			return RETCODE_INSUFFICIENT_RESOURCE;
		}
		pop->vbWork = pDecInfo->vbWork;
		pDecInfo->workBufferAllocExt = 0;
	}

    VLOG(INFO,"\nOPEN inst %d_%d, cpb 0x%08x\n", pCodecInst->coreIdx, pCodecInst->instIndex, pDecInfo->streamBufStartAddr);
    VLOG(INFO,"[Work Buffer] 0x%08x\n", pDecInfo->vbWork.phys_addr);
    VLOG(INFO,"[Code buffer] 0x%08x\n",  VpuReadReg(pCodecInst->coreIdx, BIT_CODE_BUF_ADDR));
    VLOG(INFO,"[Para buffer] 0x%08x\n",  VpuReadReg(pCodecInst->coreIdx, BIT_PARA_BUF_ADDR));
    VLOG(INFO,"[Temp buffer] 0x%08x\n",  VpuReadReg(pCodecInst->coreIdx, BIT_TEMP_BUF_ADDR));
	pDecInfo->frameBufPool[0].bufMvCol = (PhysicalAddress)-1;

	pDecInfo->tiled2LinearEnable = pop->tiled2LinearEnable;
	pDecInfo->tiled2LinearMode = pop->tiled2LinearMode;
	if (!pDecInfo->tiled2LinearEnable)
		pDecInfo->tiled2LinearMode = 0;	//coda980 only

	if (!pDecInfo->wtlEnable)	//coda980 only
		pDecInfo->wtlMode = 0;
	pDecInfo->cacheConfig.CacheMode = 0;

	for (i=0; i<WPROT_DEC_MAX; i++)
		pDecInfo->writeMemProtectCfg.decRegion[i].enable = 0;
#ifdef SUPPORT_MEM_PROTECT
	ConfigDecWPROTRegion(pCodecInst->coreIdx, pDecInfo); // set common & PS or Work buffer memory protection
#endif
	VpuWriteReg(pCodecInst->coreIdx, pDecInfo->streamRdPtrRegAddr, pDecInfo->streamBufStartAddr);
	VpuWriteReg(pCodecInst->coreIdx, pDecInfo->streamWrPtrRegAddr, pDecInfo->streamWrPtr);

	pDecInfo->frameDisplayFlag = 0;
	pDecInfo->clearDisplayIndexes   = 0;


	val = VpuReadReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM);
	val &= ~(1 << 2);
	VpuWriteReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM, val);	// clear stream end flag at start


	pDecInfo->streamEndflag = val;
	LeaveLock(pCodecInst->coreIdx);
	return RETCODE_SUCCESS;
}

RetCode VPU_DecClose(DecHandle handle)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	RetCode ret;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;


	EnterLock(pCodecInst->coreIdx);

	if (pDecInfo->initialInfoObtained) {

		BitIssueCommand(pCodecInst->coreIdx, pCodecInst, SEQ_END);
		if (vdi_wait_vpu_busy(pCodecInst->coreIdx, VPU_BUSY_CHECK_TIMEOUT, BIT_BUSY_FLAG) == -1) {
			if (pCodecInst->loggingEnable)
				vdi_log(pCodecInst->coreIdx, SEQ_END, 2);
			LeaveLock(pCodecInst->coreIdx);
			return RETCODE_VPU_RESPONSE_TIMEOUT;
		}
		if (pCodecInst->loggingEnable)
			vdi_log(pCodecInst->coreIdx, SEQ_END, 0);
	}

/* @ modified
	if (pDecInfo->vbSlice.size) {
		  if (pDecInfo->sliceAllocExt == 0) {
			  vdi_free_dma_memory(pCodecInst->coreIdx, &pDecInfo->vbSlice);
		  }
		  else {
			  vdi_dettach_dma_memory(pCodecInst->coreIdx, &pDecInfo->vbSlice);
		  }
	  }
	  if (pDecInfo->vbWork.size) {
		   if (pDecInfo->workBufferAllocExt == 0) {
			   vdi_free_dma_memory(pCodecInst->coreIdx, &pDecInfo->vbWork);
		   }
		   else {
			   vdi_dettach_dma_memory(pCodecInst->coreIdx, &pDecInfo->vbWork);
		   }
	  }

	  if (pDecInfo->vbFrame.size){
		  if (pDecInfo->frameAllocExt == 0) {
			  vdi_free_dma_memory(pCodecInst->coreIdx, &pDecInfo->vbFrame);
		  }
	  }


	  if (pDecInfo->vbPPU.size) {
		  if (pDecInfo->ppuAllocExt == 0) {
			  vdi_free_dma_memory(pCodecInst->coreIdx, &pDecInfo->vbPPU);
		  }
	  }
	  if (pDecInfo->wtlEnable)	  //coda980 only
	  {
		  if (pDecInfo->vbWTL.size)
			  vdi_free_dma_memory(pCodecInst->coreIdx, &pDecInfo->vbWTL);
	  }


	  if (pDecInfo->vbUserData.size)
		  vdi_dettach_dma_memory(pCodecInst->coreIdx, &pDecInfo->vbUserData);

*/
	VLOG(INFO,"\nCLOSE inst %d_%d\n", pCodecInst->coreIdx, pCodecInst->instIndex);
	LeaveLock(pCodecInst->coreIdx);

	FreeCodecInstance(pCodecInst);

	return RETCODE_SUCCESS;
}



RetCode VPU_DecSetEscSeqInit(DecHandle handle, int escape)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	RetCode ret;
	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	if (pDecInfo->openParam.bitstreamMode != BS_MODE_INTERRUPT)
		return RETCODE_INVALID_PARAM;

	pDecInfo->seqInitEscape = escape;

	return RETCODE_SUCCESS;
}

RetCode VPU_DecGetInitialInfo(DecHandle handle,
	DecInitialInfo * info)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	Uint32 val, val2;
	RetCode ret;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if (info == 0) {
		return RETCODE_INVALID_PARAM;
	}


	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;
	ret = CheckDecProductCapability(pCodecInst);
	if (ret != RETCODE_SUCCESS)
		return ret;
	EnterLock(pCodecInst->coreIdx);

	if (GetPendingInst(pCodecInst->coreIdx)) {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	if (DecBitstreamBufEmpty(pDecInfo)) {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_WRONG_CALL_SEQUENCE;
	}


	VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_BB_START, pDecInfo->streamBufStartAddr);
	VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_BB_SIZE, (unsigned int)(pDecInfo->streamBufSize) >> 10); // size in KBytes


	if(pDecInfo->userDataEnable) {
		val  = 0;
		val |= (pDecInfo->userDataReportMode << 10);
		val |= (pDecInfo->userDataEnable << 5);
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_USER_DATA_OPTION, val);
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_USER_DATA_BASE_ADDR, pDecInfo->userDataBufAddr);
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_USER_DATA_BUF_SIZE, pDecInfo->userDataBufSize);
        VLOG(INFO, "[User Data] 0x%x \n", pDecInfo->userDataBufAddr);
	}
	else {
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_USER_DATA_OPTION, 0);
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_USER_DATA_BASE_ADDR, 0);
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_USER_DATA_BUF_SIZE, 0);
	}

	val  = 0;


	val |= (pDecInfo->reorderEnable<<1) & 0x2;
	val |= (pDecInfo->openParam.mp4DeblkEnable & 0x1);
	val |= (pDecInfo->avcErrorConcealMode << 2);
	VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_OPTION, val);

	switch(pCodecInst->codecMode) {
	case VC1_DEC:
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_VC1_STREAM_FMT, (0 << 3) & 0x08);
		break;
	case MP4_DEC:
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_MP4_ASP_CLASS, ((!VPU_GMC_PROCESS_METHOD)<<3)|pDecInfo->openParam.mp4Class);
		break;
	case AVC_DEC:
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_X264_MV_EN, VPU_AVC_X264_SUPPORT);
		break;
	}

	if( pCodecInst->codecMode == AVC_DEC )
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_SPP_CHUNK_SIZE, VPU_GBU_SIZE);

	VpuWriteReg(pCodecInst->coreIdx, pDecInfo->streamWrPtrRegAddr, pDecInfo->streamWrPtr);
	VpuWriteReg(pCodecInst->coreIdx, pDecInfo->streamRdPtrRegAddr, pDecInfo->streamRdPtr);


	pDecInfo->streamEndflag &= ~(3<<3);
	if (pDecInfo->openParam.bitstreamMode == BS_MODE_ROLLBACK)  //rollback mode
		pDecInfo->streamEndflag |= (1<<3);
	else if (pDecInfo->openParam.bitstreamMode == BS_MODE_PIC_END)
		pDecInfo->streamEndflag |= (2<<3);
	else {	// Interrupt Mode
		if (pDecInfo->seqInitEscape) {
			pDecInfo->streamEndflag |= (2<<3);
		}
	}

	VpuWriteReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM, pDecInfo->streamEndflag);

	val = pDecInfo->openParam.streamEndian;
	VpuWriteReg(pCodecInst->coreIdx, BIT_BIT_STREAM_CTRL, val);

	if (pCodecInst->productId == PRODUCT_ID_980) {
		val = 0;
		val |= (pDecInfo->openParam.bwbEnable<<15);
		val |= (pDecInfo->wtlMode<<17)|(pDecInfo->tiled2LinearMode<<13)|(FORMAT_420<<6);
		val |= ((pDecInfo->openParam.cbcrInterleave)<<2); // Interleave bit position is modified
		val |= pDecInfo->openParam.frameEndian;
		VpuWriteReg(pCodecInst->coreIdx, BIT_FRAME_MEM_CTRL, val);
	}
	else if (pCodecInst->productId == PRODUCT_ID_960) {
		val = 0;
		val |= (pDecInfo->wtlEnable<<17)|(pDecInfo->openParam.bwbEnable<<12);
		val |= ((pDecInfo->openParam.cbcrInterleave)<<2); // Interleave bit position is modified
		val |= pDecInfo->openParam.frameEndian;
		VpuWriteReg(pCodecInst->coreIdx, BIT_FRAME_MEM_CTRL, val);
	}
	else {
		return RETCODE_NOT_FOUND_VPU_DEVICE;
	}

	VpuWriteReg(pCodecInst->coreIdx, pDecInfo->frameDisplayFlagRegAddr, 0);

	SetPendingInst(pCodecInst->coreIdx, pCodecInst);
	BitIssueCommand(pCodecInst->coreIdx, pCodecInst, SEQ_INIT);
	if (vdi_wait_vpu_busy(pCodecInst->coreIdx, VPU_BUSY_CHECK_TIMEOUT, BIT_BUSY_FLAG) == -1) {
		if (pCodecInst->loggingEnable)
			vdi_log(pCodecInst->coreIdx, SEQ_INIT, 2);
		info->rdPtr = VpuReadReg(pCodecInst->coreIdx, pDecInfo->streamRdPtrRegAddr);
		info->wrPtr = VpuReadReg(pCodecInst->coreIdx, pDecInfo->streamWrPtrRegAddr);
		SetPendingInst(pCodecInst->coreIdx, 0);
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_VPU_RESPONSE_TIMEOUT;
	}

	if (pCodecInst->loggingEnable)
		vdi_log(pCodecInst->coreIdx, SEQ_INIT, 0);
	SetPendingInst(pCodecInst->coreIdx, 0);

	if (pDecInfo->openParam.bitstreamMode == BS_MODE_INTERRUPT &&
		pDecInfo->seqInitEscape) {
			pDecInfo->streamEndflag &= ~(3<<3);
			VpuWriteReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM, pDecInfo->streamEndflag);
			pDecInfo->seqInitEscape = 0;
	}

	pDecInfo->frameDisplayFlag = VpuReadReg(pCodecInst->coreIdx, pDecInfo->frameDisplayFlagRegAddr);
	pDecInfo->streamRdPtr = VpuReadReg(pCodecInst->coreIdx, pDecInfo->streamRdPtrRegAddr);
	pDecInfo->streamEndflag = VpuReadReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM);

	info->rdPtr = pDecInfo->streamRdPtr;
	info->wrPtr = VpuReadReg(pCodecInst->coreIdx, pDecInfo->streamWrPtrRegAddr);

    val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_SRC_SIZE);
    info->picWidth = ( (val >> 16) & 0xffff );
    info->picHeight = ( val & 0xffff );

	val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_SUCCESS);
	info->seqInitErrReason = 0;
#ifdef SUPPORT_MEM_PROTECT
	if (val & (1<<31)) {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_MEMORY_ACCESS_VIOLATION;
	}
#endif

	if ( pDecInfo->openParam.bitstreamMode == BS_MODE_ROLLBACK) {
		if (val & (1<<4)) {
			info->seqInitErrReason = (VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_SEQ_ERR_REASON) | (1<<20));
			SetPendingInst(pCodecInst->coreIdx, 0);
			LeaveLock(pCodecInst->coreIdx);
			return RETCODE_FAILURE;
		}
	}

	if (val == 0) {
		info->seqInitErrReason = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_SEQ_ERR_REASON);
        SetPendingInst(pCodecInst->coreIdx, 0);
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_FAILURE;
	}

	info->fRateNumerator    = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_FRATE_NR);
	info->fRateDenominator  = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_FRATE_DR);
	if (pCodecInst->codecMode == AVC_DEC && info->fRateDenominator > 0)
		info->fRateDenominator  *= 2;



	if (pCodecInst->codecMode  == MP4_DEC)
	{
		val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_INFO);
		info->mp4ShortVideoHeader = (val >> 2) & 1;
		info->mp4DataPartitionEnable = val & 1;
		info->mp4ReversibleVlcEnable =
			info->mp4DataPartitionEnable ?
			((val >> 1) & 1) : 0;
		info->h263AnnexJEnable = (val >> 3) & 1;
	}
	else if (pCodecInst->codecMode == VPX_DEC &&
		pCodecInst->codecModeAux == VPX_AUX_VP8)
	{
		// h_scale[31:30] v_scale[29:28] pic_width[27:14] pic_height[13:0]
		val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_VP8_SCALE_INFO);
		info->vp8ScaleInfo.hScaleFactor = (val >> 30) & 0x03;
		info->vp8ScaleInfo.vScaleFactor = (val >> 28) & 0x03;
		info->vp8ScaleInfo.picWidth = (val >> 14) & 0x3FFF;
		info->vp8ScaleInfo.picHeight = (val >> 0) & 0x3FFF;
	}

	info->minFrameBufferCount = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_FRAME_NEED);
	info->frameBufDelay = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_FRAME_DELAY);

	if (pCodecInst->codecMode == AVC_DEC)
	{
		val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_CROP_LEFT_RIGHT);
		val2 = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_CROP_TOP_BOTTOM);
		if( val == 0 && val2 == 0 )
		{
			info->picCropRect.left = 0;
			info->picCropRect.right = info->picWidth;
			info->picCropRect.top = 0;
			info->picCropRect.bottom = info->picHeight;
		}
		else
		{
			info->picCropRect.left = ( (val>>16) & 0xFFFF );
			info->picCropRect.right = info->picWidth - ( ( val & 0xFFFF ) );
			info->picCropRect.top = ( (val2>>16) & 0xFFFF );
			info->picCropRect.bottom = info->picHeight - ( ( val2 & 0xFFFF ) );
		}

		val = (info->picWidth * info->picHeight * 3 / 2) / 1024;
		info->normalSliceSize = val / 4;
		info->worstSliceSize = val / 2;
	}
	else
	{
		info->picCropRect.left = 0;
		info->picCropRect.right = info->picWidth;
		info->picCropRect.top = 0;
		info->picCropRect.bottom = info->picHeight;
	}

       if (pCodecInst->codecMode == MP2_DEC)
       {
             val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_MP2_BAR_LEFT_RIGHT);
             val2 = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_MP2_BAR_TOP_BOTTOM);

             info->mp2BardataInfo.barLeft = ((val>>16) & 0xFFFF);
             info->mp2BardataInfo.barRight = (val&0xFFFF);
             info->mp2BardataInfo.barTop = ((val2>>16) & 0xFFFF);
             info->mp2BardataInfo.barBottom = (val2&0xFFFF);
       }

	val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_HEADER_REPORT);
	info->profile =			(val >> 0) & 0xFF;
	info->level =			(val >> 8) & 0xFF;
	info->interlace =		(val >> 16) & 0x01;
	info->direct8x8Flag =		(val >> 17) & 0x01;
	info->vc1Psf =			(val >> 18) & 0x01;
	info->constraint_set_flag[0] = 	(val >> 19) & 0x01;
	info->constraint_set_flag[1] = 	(val >> 20) & 0x01;
	info->constraint_set_flag[2] = 	(val >> 21) & 0x01;
	info->constraint_set_flag[3] = 	(val >> 22) & 0x01;
	info->chromaFormat = ( val >> 23) & 0x03;
	info->avcIsExtSAR            =  (val >> 25) & 0x01;
	info->maxNumRefFrmFlag =  (val >> 31) & 0x01;

	val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_ASPECT);

	info->aspectRateInfo = val;

	val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_BIT_RATE);
	info->bitRate = val;

	if (pCodecInst->codecMode == MP2_DEC) {
		// seq_ext info
		val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_EXT_INFO);
		info->mp2LowDelay = val & 1;
		info->mp2DispVerSize = (val>>1) & 0x3fff;
		info->mp2DispHorSize = (val>>15) & 0x3fff;
	}

	if (pCodecInst->codecMode == AVC_DEC) {
		val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_VUI_INFO);
		info->avcVuiInfo.fixedFrameRateFlag    = val &1;
		info->avcVuiInfo.timingInfoPresent     = (val>>1) & 0x01;
		info->avcVuiInfo.chromaLocBotField     = (val>>2) & 0x07;
		info->avcVuiInfo.chromaLocTopField     = (val>>5) & 0x07;
		info->avcVuiInfo.chromaLocInfoPresent  = (val>>8) & 0x01;
		info->avcVuiInfo.colorPrimaries        = (val>>16) & 0xff;
		info->avcVuiInfo.colorDescPresent      = (val>>24) & 0x01;
		info->avcVuiInfo.isExtSAR              = (val>>25) & 0x01;
		info->avcVuiInfo.vidFullRange          = (val>>26) & 0x01;
		info->avcVuiInfo.vidFormat             = (val>>27) & 0x07;
		info->avcVuiInfo.vidSigTypePresent     = (val>>30) & 0x01;
		info->avcVuiInfo.vuiParamPresent       = (val>>31) & 0x01;

		val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_VUI_PIC_STRUCT);
		info->avcVuiInfo.vuiPicStructPresent = (val & 0x1);
		info->avcVuiInfo.vuiPicStruct = (val>>1);
	}

	if (pDecInfo->userDataEnable && pCodecInst->codecMode == MP2_DEC) {
		int userDataNum;
		int userDataSize;
		BYTE tempBuf[8] = {0,};

		VpuReadMem(pCodecInst->coreIdx, pDecInfo->userDataBufAddr, tempBuf, 8, VPU_USER_DATA_ENDIAN);

		val = ((tempBuf[0]<<24) & 0xFF000000) |
			((tempBuf[1]<<16) & 0x00FF0000) |
			((tempBuf[2]<< 8) & 0x0000FF00) |
			((tempBuf[3]<< 0) & 0x000000FF);

		userDataNum = (val >> 16) & 0xFFFF;
		userDataSize = (val >> 0) & 0xFFFF;
		if (userDataNum == 0)
			userDataSize = 0;

		info->userDataNum = userDataNum;
		info->userDataSize = userDataSize;

		val = ((tempBuf[4]<<24) & 0xFF000000) |
			((tempBuf[5]<<16) & 0x00FF0000) |
			((tempBuf[6]<< 8) & 0x0000FF00) |
			((tempBuf[7]<< 0) & 0x000000FF);

		if (userDataNum == 0)
			info->userDataBufFull = 0;
		else
			info->userDataBufFull = (val >> 16) & 0xFFFF;
	}

	pDecInfo->initialInfo = *info;
	pDecInfo->initialInfoObtained = 1;

	LeaveLock(pCodecInst->coreIdx);
	return RETCODE_SUCCESS;
}


RetCode VPU_DecIssueSeqInit(DecHandle handle)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	Uint32 val;
	RetCode ret;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;
	ret = CheckDecProductCapability(pCodecInst);
	if (ret != RETCODE_SUCCESS)
		return ret;
	EnterLock(pCodecInst->coreIdx);

	if (GetPendingInst(pCodecInst->coreIdx)) {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	if (DecBitstreamBufEmpty(pDecInfo)) {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_WRONG_CALL_SEQUENCE;
	}


	VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_BB_START, pDecInfo->streamBufStartAddr);
	VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_BB_SIZE, (unsigned int)(pDecInfo->streamBufSize) >> 10); // size in KBytes

	if(pDecInfo->userDataEnable) {
		val  = 0;
		val |= (pDecInfo->userDataReportMode << 10);
		val |= (pDecInfo->userDataEnable << 5);
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_USER_DATA_OPTION, val);
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_USER_DATA_BASE_ADDR, pDecInfo->userDataBufAddr);
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_USER_DATA_BUF_SIZE, pDecInfo->userDataBufSize);
	}
	else {
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_USER_DATA_OPTION, 0);
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_USER_DATA_BASE_ADDR, 0);
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_USER_DATA_BUF_SIZE, 0);
	}
	val  = 0;

	val |= (pDecInfo->reorderEnable<<1) & 0x2;

	val |= (pDecInfo->openParam.mp4DeblkEnable & 0x1);
	val |= (pDecInfo->avcErrorConcealMode << 2);
	VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_OPTION, val);

	switch(pCodecInst->codecMode) {
	case VC1_DEC:
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_VC1_STREAM_FMT, (0 << 3) & 0x08);
		break;
	case MP4_DEC:
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_MP4_ASP_CLASS, ((!VPU_GMC_PROCESS_METHOD)<<3)|pDecInfo->openParam.mp4Class);
		break;
	case AVC_DEC:
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_X264_MV_EN, VPU_AVC_X264_SUPPORT);
		break;
	}

	if( pCodecInst->codecMode == AVC_DEC )
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_SEQ_SPP_CHUNK_SIZE, VPU_GBU_SIZE);

	VpuWriteReg(pCodecInst->coreIdx, pDecInfo->streamWrPtrRegAddr, pDecInfo->streamWrPtr);
	VpuWriteReg(pCodecInst->coreIdx, pDecInfo->streamRdPtrRegAddr, pDecInfo->streamRdPtr);

	if (pCodecInst->productId == PRODUCT_ID_980 || pCodecInst->productId == PRODUCT_ID_960) {
		pDecInfo->streamEndflag &= ~(3<<3);
		if (pDecInfo->openParam.bitstreamMode == BS_MODE_ROLLBACK)  //rollback mode
			pDecInfo->streamEndflag |= (1<<3);
		else if (pDecInfo->openParam.bitstreamMode == BS_MODE_PIC_END)
			pDecInfo->streamEndflag |= (2<<3);
		else {	// Interrupt Mode
			if (pDecInfo->seqInitEscape) {
				pDecInfo->streamEndflag |= (2<<3);
			}
		}
	}

	VpuWriteReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM, pDecInfo->streamEndflag);

	val = pDecInfo->openParam.streamEndian;
	VpuWriteReg(pCodecInst->coreIdx, BIT_BIT_STREAM_CTRL, val);

	if (pCodecInst->productId == PRODUCT_ID_980) {
		val = 0;
		val |= (pDecInfo->openParam.bwbEnable<<15);
		val |= (pDecInfo->wtlMode<<17)|(pDecInfo->tiled2LinearMode<<13)|(FORMAT_420<<6);
		val |= ((pDecInfo->openParam.cbcrInterleave)<<2); // Interleave bit position is modified
		val |= pDecInfo->openParam.frameEndian;
		VpuWriteReg(pCodecInst->coreIdx, BIT_FRAME_MEM_CTRL, val);
	}
	else if (pCodecInst->productId == PRODUCT_ID_960) {
		val = 0;
    val |= (pDecInfo->wtlEnable<<17);
		val |= (pDecInfo->openParam.bwbEnable<<12);
		val |= ((pDecInfo->openParam.cbcrInterleave)<<2); // Interleave bit position is modified
		val |= pDecInfo->openParam.frameEndian;
		VpuWriteReg(pCodecInst->coreIdx, BIT_FRAME_MEM_CTRL, val);
	}
	else {
		return RETCODE_NOT_FOUND_VPU_DEVICE;
	}


	VpuWriteReg(pCodecInst->coreIdx, pDecInfo->frameDisplayFlagRegAddr, 0);
	BitIssueCommand(pCodecInst->coreIdx, pCodecInst, SEQ_INIT);

	SetPendingInst(pCodecInst->coreIdx, pCodecInst);

	return RETCODE_SUCCESS;
}

RetCode VPU_DecCompleteSeqInit(
	DecHandle handle,
	DecInitialInfo * info)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	Uint32 val, val2;
	RetCode ret;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS) {
		return ret;
	}

	if (info == 0) {
		return RETCODE_INVALID_PARAM;
	}

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;


	if (pCodecInst != GetPendingInst(pCodecInst->coreIdx)) {
		SetPendingInst(pCodecInst->coreIdx, 0);
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_WRONG_CALL_SEQUENCE;
	}


	if (pCodecInst->loggingEnable)
		vdi_log(pCodecInst->coreIdx, SEQ_INIT, 0);

	if (pDecInfo->openParam.bitstreamMode == BS_MODE_INTERRUPT &&
		pDecInfo->seqInitEscape) {
		pDecInfo->streamEndflag &= ~(3<<3);
		VpuWriteReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM, pDecInfo->streamEndflag);
		pDecInfo->seqInitEscape = 0;
	}


	pDecInfo->frameDisplayFlag = VpuReadReg(pCodecInst->coreIdx, pDecInfo->frameDisplayFlagRegAddr);
	pDecInfo->streamRdPtr = VpuReadReg(pCodecInst->coreIdx, pDecInfo->streamRdPtrRegAddr);
	pDecInfo->streamEndflag = VpuReadReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM);

	info->rdPtr = pDecInfo->streamRdPtr;
	info->wrPtr = VpuReadReg(pCodecInst->coreIdx, pDecInfo->streamWrPtrRegAddr);

	info->seqInitErrReason = 0;

    val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_SRC_SIZE);
    info->picWidth = ( (val >> 16) & 0xffff );
    info->picHeight = ( val & 0xffff );

	val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_SUCCESS);
#ifdef SUPPORT_MEM_PROTECT
	if (val & (1<<31)) {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_MEMORY_ACCESS_VIOLATION;
	}
#endif

	if ( pDecInfo->openParam.bitstreamMode == BS_MODE_PIC_END || pDecInfo->openParam.bitstreamMode == BS_MODE_ROLLBACK ) {
		if (val & (1<<4)) {
			info->seqInitErrReason = (VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_SEQ_ERR_REASON));
			SetPendingInst(pCodecInst->coreIdx, 0);
			LeaveLock(pCodecInst->coreIdx);
			return RETCODE_FAILURE;
		}
	}

	if (val == 0) {
		info->seqInitErrReason = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_SEQ_ERR_REASON);
        SetPendingInst(pCodecInst->coreIdx, 0);
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_FAILURE;
	}

	info->fRateNumerator    = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_FRATE_NR);
	info->fRateDenominator  = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_FRATE_DR);
	if (pCodecInst->codecMode == AVC_DEC && info->fRateDenominator > 0)
		info->fRateDenominator  *= 2;

	if (pCodecInst->codecMode  == MP4_DEC)
	{
		val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_INFO);
		info->mp4ShortVideoHeader = (val >> 2) & 1;
		info->mp4DataPartitionEnable = val & 1;
		info->mp4ReversibleVlcEnable =
			info->mp4DataPartitionEnable ?
			((val >> 1) & 1) : 0;
		info->h263AnnexJEnable = (val >> 3) & 1;
	}
	else if (pCodecInst->codecMode == VPX_DEC &&
		pCodecInst->codecModeAux == VPX_AUX_VP8)
	{
		// h_scale[31:30] v_scale[29:28] pic_width[27:14] pic_height[13:0]
		val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_VP8_SCALE_INFO);
		info->vp8ScaleInfo.hScaleFactor = (val >> 30) & 0x03;
		info->vp8ScaleInfo.vScaleFactor = (val >> 28) & 0x03;
		info->vp8ScaleInfo.picWidth = (val >> 14) & 0x3FFF;
		info->vp8ScaleInfo.picHeight = (val >> 0) & 0x3FFF;
	}

	info->minFrameBufferCount = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_FRAME_NEED);
	info->frameBufDelay = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_FRAME_DELAY);

	if (pCodecInst->codecMode == AVC_DEC)
	{
		val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_CROP_LEFT_RIGHT);
		val2 = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_CROP_TOP_BOTTOM);
		if( val == 0 && val2 == 0 )
		{
			info->picCropRect.left = 0;
			info->picCropRect.right = info->picWidth;
			info->picCropRect.top = 0;
			info->picCropRect.bottom = info->picHeight;
		}
		else
		{
			info->picCropRect.left = ( (val>>16) & 0xFFFF );
			info->picCropRect.right = info->picWidth - ( ( val & 0xFFFF ) );
			info->picCropRect.top = ( (val2>>16) & 0xFFFF );
			info->picCropRect.bottom = info->picHeight - ( ( val2 & 0xFFFF ) );
		}

		val = (info->picWidth * info->picHeight * 3 / 2) / 1024;
		info->normalSliceSize = val / 4;
		info->worstSliceSize = val / 2;
	}
	else
	{
		info->picCropRect.left = 0;
		info->picCropRect.right = info->picWidth;
		info->picCropRect.top = 0;
		info->picCropRect.bottom = info->picHeight;
	}

       if (pCodecInst->codecMode == MP2_DEC)
       {
             val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_MP2_BAR_LEFT_RIGHT);
             val2 = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_MP2_BAR_TOP_BOTTOM);

             info->mp2BardataInfo.barLeft = ((val>>16) & 0xFFFF);
             info->mp2BardataInfo.barRight = (val&0xFFFF);
             info->mp2BardataInfo.barTop = ((val2>>16) & 0xFFFF);
             info->mp2BardataInfo.barBottom = (val2&0xFFFF);
       }

	val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_HEADER_REPORT);
	info->profile                =	(val >> 0) & 0xFF;
	info->level                  =	(val >> 8) & 0xFF;
	info->interlace              =	(val >> 16) & 0x01;
	info->direct8x8Flag          =	(val >> 17) & 0x01;
	info->vc1Psf                 =	(val >> 18) & 0x01;
	info->constraint_set_flag[0] = 	(val >> 19) & 0x01;
	info->constraint_set_flag[1] = 	(val >> 20) & 0x01;
	info->constraint_set_flag[2] = 	(val >> 21) & 0x01;
	info->constraint_set_flag[3] = 	(val >> 22) & 0x01;
	info->chromaFormat		  = ( val >> 23) & 0x03;
	info->avcIsExtSAR            =  (val >> 25) & 0x01;
	info->maxNumRefFrmFlag =  (val >> 31) & 0x01;
	val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_ASPECT);
	info->aspectRateInfo = val;

	val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_BIT_RATE);
	info->bitRate = val;

	if (pCodecInst->codecMode == AVC_DEC) {
		val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_VUI_INFO);
		info->avcVuiInfo.fixedFrameRateFlag    = val &1;
		info->avcVuiInfo.timingInfoPresent     = (val>>1) & 0x01;
		info->avcVuiInfo.chromaLocBotField     = (val>>2) & 0x07;
		info->avcVuiInfo.chromaLocTopField     = (val>>5) & 0x07;
		info->avcVuiInfo.chromaLocInfoPresent  = (val>>8) & 0x01;
		info->avcVuiInfo.colorPrimaries        = (val>>16) & 0xff;
		info->avcVuiInfo.colorDescPresent      = (val>>24) & 0x01;
		info->avcVuiInfo.isExtSAR              = (val>>25) & 0x01;
		info->avcVuiInfo.vidFullRange          = (val>>26) & 0x01;
		info->avcVuiInfo.vidFormat             = (val>>27) & 0x07;
		info->avcVuiInfo.vidSigTypePresent     = (val>>30) & 0x01;
		info->avcVuiInfo.vuiParamPresent       = (val>>31) & 0x01;

		val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_VUI_PIC_STRUCT);
		info->avcVuiInfo.vuiPicStructPresent = (val & 0x1);
		info->avcVuiInfo.vuiPicStruct = (val>>1);
	}

	if (pCodecInst->codecMode == MP2_DEC) {
		// seq_ext info
		val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_SEQ_EXT_INFO);
		info->mp2LowDelay = val & 1;
		info->mp2DispVerSize = (val>>1) & 0x3fff;
		info->mp2DispHorSize = (val>>15) & 0x3fff;

		if (pDecInfo->userDataEnable)
		{
			int userDataNum;
			int userDataSize;
			BYTE tempBuf[8] = {0,};

			// user data
			VpuReadMem(pCodecInst->coreIdx, pDecInfo->userDataBufAddr, tempBuf, 8, VPU_USER_DATA_ENDIAN);

			val = ((tempBuf[0]<<24) & 0xFF000000) |
				((tempBuf[1]<<16) & 0x00FF0000) |
				((tempBuf[2]<< 8) & 0x0000FF00) |
				((tempBuf[3]<< 0) & 0x000000FF);

			userDataNum = (val >> 16) & 0xFFFF;
			userDataSize = (val >> 0) & 0xFFFF;
			if (userDataNum == 0)
				userDataSize = 0;

			info->userDataNum = userDataNum;
			info->userDataSize = userDataSize;

			val = ((tempBuf[4]<<24) & 0xFF000000) |
				((tempBuf[5]<<16) & 0x00FF0000) |
				((tempBuf[6]<< 8) & 0x0000FF00) |
				((tempBuf[7]<< 0) & 0x000000FF);

			if (userDataNum == 0)
				info->userDataBufFull = 0;
			else
				info->userDataBufFull = (val >> 16) & 0xFFFF;
		}


	}


	pDecInfo->initialInfo = *info;
	pDecInfo->initialInfoObtained = 1;

	SetPendingInst(pCodecInst->coreIdx, 0);
	LeaveLock(pCodecInst->coreIdx);

	return RETCODE_SUCCESS;
}
RetCode VPU_DecRegisterFrameBuffer(DecHandle handle, FrameBuffer *bufArray, int num, int stride, int height, int mapType)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	PhysicalAddress paraBuffer;
	vpu_buffer_t vb;
	Uint32 val;
	int i;
	RetCode ret;
	BYTE frameAddr[MAX_GDI_IDX][3][4];
	BYTE colMvAddr[MAX_GDI_IDX][4];
	int framebufFormat;
	int size_mvcolbuf;
#ifdef SUPPORT_MEM_PROTECT
	PhysicalAddress start;
	PhysicalAddress end;
#endif

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;


	pDecInfo->mapType = mapType;

	ret = CheckDecProductCapability(pCodecInst);
	if (ret != RETCODE_SUCCESS)
		return ret;
	if (!pDecInfo->initialInfoObtained) {
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

    if (!pDecInfo->thumbnailMode)
    {
        if (num < pDecInfo->initialInfo.minFrameBufferCount) {
            return RETCODE_INSUFFICIENT_FRAME_BUFFERS;
        }
    }

	if ( (stride < pDecInfo->initialInfo.picWidth) || (stride % 8 != 0) || (height<pDecInfo->initialInfo.picHeight) ) {
		return RETCODE_INVALID_STRIDE;
	}

	EnterLock(pCodecInst->coreIdx);
	if (GetPendingInst(pCodecInst->coreIdx)) {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	vdi_get_common_memory(pCodecInst->coreIdx, &vb);
	paraBuffer = vb.phys_addr + CODE_BUF_SIZE + TEMP_BUF_SIZE;

	pDecInfo->numFrameBuffers = num;
	pDecInfo->stride = stride;
	pDecInfo->frameBufferHeight = height;
	pDecInfo->mapType = mapType;
	pDecInfo->mapCfg.productId = pCodecInst->productId;

	if (pCodecInst->productId == PRODUCT_ID_980) {
		if (mapType != LINEAR_FRAME_MAP && mapType != LINEAR_FIELD_MAP) {
			val = SetTiledMapType(pCodecInst->coreIdx, &pDecInfo->mapCfg, mapType, (stride > height)?stride:height, pDecInfo->openParam.cbcrInterleave, &pDecInfo->dramCfg);

		} else {
			val = SetTiledMapType(pCodecInst->coreIdx, &pDecInfo->mapCfg, mapType,stride, pDecInfo->openParam.cbcrInterleave, &pDecInfo->dramCfg);
		}
	}
	else {
		val = SetTiledMapType(pCodecInst->coreIdx, &pDecInfo->mapCfg, mapType, stride, pDecInfo->openParam.cbcrInterleave, &pDecInfo->dramCfg);
	}

	if (val == 0) {
        LeaveLock(pCodecInst->coreIdx);
		return RETCODE_INVALID_PARAM;
	}

	//Allocate frame buffer
	framebufFormat = FORMAT_420;

	VLOG(INFO,"\ninst %d_%d, num %d, w 0x%x, h 0x%x\n", pCodecInst->coreIdx, pCodecInst->instIndex, num, stride, height);

	if (bufArray) {
		for(i=0; i<num; i++)
	        {
				pDecInfo->frameBufPool[i] = bufArray[i];
	            VLOG(INFO, "Frame Buffer Array [%d] = 0x%08x 0x%08x 0x%08x (0x%08x 0x%08x 0x%08x)\n", i,
	            		GetXY2AXIAddr(&pDecInfo->mapCfg, 0, 0, 0, stride, &pDecInfo->frameBufPool[i]),
	            		GetXY2AXIAddr(&pDecInfo->mapCfg, 2, 0, 0, stride, &pDecInfo->frameBufPool[i]),
			            GetXY2AXIAddr(&pDecInfo->mapCfg, 1, 0, 0, stride, &pDecInfo->frameBufPool[i]),
			            bufArray[i].bufY, bufArray[i].bufCb, bufArray[i].bufCr);

//	            VLOG(INFO, "Frame Buffer Array Bot [%d] = 0x%08x 0x%08x 0x%08x\n", i, bufArray[i].bufYBot, bufArray[i].bufCbBot, bufArray[i].bufCrBot);
	        }
//	        printk("GDI_TILEDBUF_BASE 0x%08x\n", VpuReadReg(pCodecInst->coreIdx, GDI_TILEDBUF_BASE));
	}
	else {
		if (pCodecInst->productId == PRODUCT_ID_980) {
			ret = AllocateFrameBufferArrayV20(pCodecInst->coreIdx, &pDecInfo->frameBufPool[0], &pDecInfo->vbFrame, mapType,
				pDecInfo->openParam.cbcrInterleave, framebufFormat, pDecInfo->numFrameBuffers, stride, height, pDecInfo->openParam.frameEndian, 0, FB_TYPE_CODEC);
		}
		else {
			ret = AllocateFrameBufferArrayV10(pCodecInst->coreIdx, &pDecInfo->frameBufPool[0], &pDecInfo->vbFrame, mapType,
				pDecInfo->openParam.cbcrInterleave, framebufFormat, pDecInfo->numFrameBuffers, stride, height, 0, FB_TYPE_CODEC, 0, &pDecInfo->dramCfg);
			pDecInfo->mapCfg.tiledBaseAddr = pDecInfo->vbFrame.phys_addr;
		}
	}
	if (ret != RETCODE_SUCCESS) {
		LeaveLock(pCodecInst->coreIdx);
		return ret;
	}



#ifdef SUPPORT_MEM_PROTECT
	pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].enable        = 1;
	pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].isSecondary  = 0;
	pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].startAddress = GetXY2AXIAddr(&pDecInfo->mapCfg, 0, 0, 0, stride, &pDecInfo->frameBufPool[0]);
	pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].endAddress   = GetXY2AXIAddr(&pDecInfo->mapCfg, 0, 0, 0, stride, &pDecInfo->frameBufPool[num-1]);
	pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].endAddress +=  VPU_GetFrameBufSize(pCodecInst->coreIdx, stride, height, pDecInfo->mapCfg.mapType, framebufFormat, pDecInfo->openParam.cbcrInterleave, &pDecInfo->dramCfg);
#endif // #ifdef SUPPORT_MEM_PROTECT

	for (i=0; i<pDecInfo->numFrameBuffers; i++) {
		frameAddr[i][0][0] = (pDecInfo->frameBufPool[i].bufY  >> 24) & 0xFF;
		frameAddr[i][0][1] = (pDecInfo->frameBufPool[i].bufY  >> 16) & 0xFF;
		frameAddr[i][0][2] = (pDecInfo->frameBufPool[i].bufY  >>  8) & 0xFF;
		frameAddr[i][0][3] = (pDecInfo->frameBufPool[i].bufY  >>  0) & 0xFF;
		if (pDecInfo->openParam.cbcrOrder == CBCR_ORDER_NORMAL) {
			frameAddr[i][1][0] = (pDecInfo->frameBufPool[i].bufCb >> 24) & 0xFF;
			frameAddr[i][1][1] = (pDecInfo->frameBufPool[i].bufCb >> 16) & 0xFF;
			frameAddr[i][1][2] = (pDecInfo->frameBufPool[i].bufCb >>  8) & 0xFF;
			frameAddr[i][1][3] = (pDecInfo->frameBufPool[i].bufCb >>  0) & 0xFF;
			frameAddr[i][2][0] = (pDecInfo->frameBufPool[i].bufCr >> 24) & 0xFF;
			frameAddr[i][2][1] = (pDecInfo->frameBufPool[i].bufCr >> 16) & 0xFF;
			frameAddr[i][2][2] = (pDecInfo->frameBufPool[i].bufCr >>  8) & 0xFF;
			frameAddr[i][2][3] = (pDecInfo->frameBufPool[i].bufCr >>  0) & 0xFF;
		}
		else {
			frameAddr[i][2][0] = (pDecInfo->frameBufPool[i].bufCb >> 24) & 0xFF;
			frameAddr[i][2][1] = (pDecInfo->frameBufPool[i].bufCb >> 16) & 0xFF;
			frameAddr[i][2][2] = (pDecInfo->frameBufPool[i].bufCb >>  8) & 0xFF;
			frameAddr[i][2][3] = (pDecInfo->frameBufPool[i].bufCb >>  0) & 0xFF;
			frameAddr[i][1][0] = (pDecInfo->frameBufPool[i].bufCr >> 24) & 0xFF;
			frameAddr[i][1][1] = (pDecInfo->frameBufPool[i].bufCr >> 16) & 0xFF;
			frameAddr[i][1][2] = (pDecInfo->frameBufPool[i].bufCr >>  8) & 0xFF;
			frameAddr[i][1][3] = (pDecInfo->frameBufPool[i].bufCr >>  0) & 0xFF;
		}
	}

	VpuWriteMem(pCodecInst->coreIdx, paraBuffer, (BYTE*)frameAddr, sizeof(frameAddr), VDI_BIG_ENDIAN);

	size_mvcolbuf = 0;
	// MV allocation and register
	if (pCodecInst->codecMode == AVC_DEC ||
		pCodecInst->codecMode == VC1_DEC ||
		pCodecInst->codecMode == MP4_DEC ||
		pCodecInst->codecMode == RV_DEC ||
		pCodecInst->codecMode == AVS_DEC)
	{
		Uint32 bufMvCol;


			size_mvcolbuf =  ((pDecInfo->initialInfo.picWidth+31)&~31)*((pDecInfo->initialInfo.picHeight+31)&~31);
			size_mvcolbuf = (size_mvcolbuf*3)/2;
			size_mvcolbuf = (size_mvcolbuf+4)/5;
			size_mvcolbuf = ((size_mvcolbuf+7)/8)*8;


		bufMvCol = pDecInfo->frameBufPool[0].bufMvCol;
        VLOG(INFO, "[MvColBuffer] 0x%x \n", bufMvCol);
		if (pCodecInst->codecMode == AVC_DEC) {
			for (i=0; i<pDecInfo->numFrameBuffers; i++) {
				colMvAddr[i][0] = (bufMvCol >> 24) & 0xFF;
				colMvAddr[i][1] = (bufMvCol >> 16) & 0xFF;
				colMvAddr[i][2] = (bufMvCol >>  8) & 0xFF;
				colMvAddr[i][3] = (bufMvCol >>  0) & 0xFF;
				bufMvCol += size_mvcolbuf;
			}
		}
		else {

			colMvAddr[0][0] = (bufMvCol >> 24) & 0xFF;
			colMvAddr[0][1] = (bufMvCol >> 16) & 0xFF;
			colMvAddr[0][2] = (bufMvCol >>  8) & 0xFF;
			colMvAddr[0][3] = (bufMvCol >>  0) & 0xFF;
			bufMvCol += size_mvcolbuf;
		}
#ifdef SUPPORT_MEM_PROTECT
		start = pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].startAddress;
		end   = pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].endAddress;

		start = (start<  pDecInfo->frameBufPool[0].bufMvCol) ? start :  pDecInfo->frameBufPool[0].bufMvCol;
		end   = (end  > bufMvCol) ? end : bufMvCol;

		pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].startAddress = start;
		pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].endAddress   = end;
#endif
 		VpuWriteMem(pCodecInst->coreIdx, paraBuffer+384, (BYTE*)colMvAddr, sizeof(colMvAddr), VDI_BIG_ENDIAN);
	}

	if (pCodecInst->productId == PRODUCT_ID_980) {
#ifdef SUPPORT_MEM_PROTECT
		PhysicalAddress frame_buffer_end;
#endif

		for (i=0; i<pDecInfo->numFrameBuffers; i++) {
			frameAddr[i][0][0] = (pDecInfo->frameBufPool[i].bufYBot  >> 24) & 0xFF;
			frameAddr[i][0][1] = (pDecInfo->frameBufPool[i].bufYBot  >> 16) & 0xFF;
			frameAddr[i][0][2] = (pDecInfo->frameBufPool[i].bufYBot  >>  8) & 0xFF;
			frameAddr[i][0][3] = (pDecInfo->frameBufPool[i].bufYBot  >>  0) & 0xFF;
			if (pDecInfo->openParam.cbcrOrder == CBCR_ORDER_NORMAL) {
				frameAddr[i][1][0] = (pDecInfo->frameBufPool[i].bufCbBot >> 24) & 0xFF;
				frameAddr[i][1][1] = (pDecInfo->frameBufPool[i].bufCbBot >> 16) & 0xFF;
				frameAddr[i][1][2] = (pDecInfo->frameBufPool[i].bufCbBot >>  8) & 0xFF;
				frameAddr[i][1][3] = (pDecInfo->frameBufPool[i].bufCbBot >>  0) & 0xFF;
				frameAddr[i][2][0] = (pDecInfo->frameBufPool[i].bufCrBot >> 24) & 0xFF;
				frameAddr[i][2][1] = (pDecInfo->frameBufPool[i].bufCrBot >> 16) & 0xFF;
				frameAddr[i][2][2] = (pDecInfo->frameBufPool[i].bufCrBot >>  8) & 0xFF;
				frameAddr[i][2][3] = (pDecInfo->frameBufPool[i].bufCrBot >>  0) & 0xFF;
			}
			else {
				frameAddr[i][2][0] = (pDecInfo->frameBufPool[i].bufCbBot >> 24) & 0xFF;
				frameAddr[i][2][1] = (pDecInfo->frameBufPool[i].bufCbBot >> 16) & 0xFF;
				frameAddr[i][2][2] = (pDecInfo->frameBufPool[i].bufCbBot >>  8) & 0xFF;
				frameAddr[i][2][3] = (pDecInfo->frameBufPool[i].bufCbBot >>  0) & 0xFF;
				frameAddr[i][1][0] = (pDecInfo->frameBufPool[i].bufCrBot >> 24) & 0xFF;
				frameAddr[i][1][1] = (pDecInfo->frameBufPool[i].bufCrBot >> 16) & 0xFF;
				frameAddr[i][1][2] = (pDecInfo->frameBufPool[i].bufCrBot >>  8) & 0xFF;
				frameAddr[i][1][3] = (pDecInfo->frameBufPool[i].bufCrBot >>  0) & 0xFF;
			}
		}

		VpuWriteMem(pCodecInst->coreIdx, paraBuffer+384+128, (BYTE*)frameAddr, sizeof(frameAddr), VDI_BIG_ENDIAN);

		if (pDecInfo->wtlEnable) {
			if (bufArray) {
				for(i=pDecInfo->numFrameBuffers; i<pDecInfo->numFrameBuffers*2; i++)
                {
					pDecInfo->frameBufPool[i] = bufArray[i];

			VLOG(INFO, "WTL Frame Buffer Array [%d] = 0x%08x 0x%08x 0x%08x\n", i, bufArray[i].bufY, bufArray[i].bufCb, bufArray[i].bufCr);
//			VLOG(INFO, "WTL Frame Buffer Array Bot [%d] = 0x%08x 0x%08x 0x%08x\n", i, bufArray[i].bufYBot, bufArray[i].bufCbBot, bufArray[i].bufCrBot);
                }
			}
			else {
				if (pDecInfo->wtlMode == FF_FRAME )
					ret = AllocateFrameBufferArrayV20(pCodecInst->coreIdx, &pDecInfo->frameBufPool[pDecInfo->numFrameBuffers], &pDecInfo->vbWTL, LINEAR_FRAME_MAP,
					pDecInfo->openParam.cbcrInterleave, framebufFormat, pDecInfo->numFrameBuffers, stride, height, pDecInfo->openParam.frameEndian, 0, FB_TYPE_PPU);
				else
					ret = AllocateFrameBufferArrayV20(pCodecInst->coreIdx, &pDecInfo->frameBufPool[pDecInfo->numFrameBuffers], &pDecInfo->vbWTL, LINEAR_FIELD_MAP,
					pDecInfo->openParam.cbcrInterleave, framebufFormat, pDecInfo->numFrameBuffers, stride, height, pDecInfo->openParam.frameEndian, 0, FB_TYPE_PPU);
			}
			if (ret != RETCODE_SUCCESS) {
                LeaveLock(pCodecInst->coreIdx);
                return ret;
            }
			for (i=pDecInfo->numFrameBuffers; i<pDecInfo->numFrameBuffers*2; i++) {
				frameAddr[i-pDecInfo->numFrameBuffers][0][0] = (pDecInfo->frameBufPool[i].bufY  >> 24) & 0xFF;
				frameAddr[i-pDecInfo->numFrameBuffers][0][1] = (pDecInfo->frameBufPool[i].bufY  >> 16) & 0xFF;
				frameAddr[i-pDecInfo->numFrameBuffers][0][2] = (pDecInfo->frameBufPool[i].bufY  >>  8) & 0xFF;
				frameAddr[i-pDecInfo->numFrameBuffers][0][3] = (pDecInfo->frameBufPool[i].bufY  >>  0) & 0xFF;
				if (pDecInfo->openParam.cbcrOrder == CBCR_ORDER_NORMAL) {
					frameAddr[i-pDecInfo->numFrameBuffers][1][0] = (pDecInfo->frameBufPool[i].bufCb >> 24) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][1][1] = (pDecInfo->frameBufPool[i].bufCb >> 16) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][1][2] = (pDecInfo->frameBufPool[i].bufCb >>  8) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][1][3] = (pDecInfo->frameBufPool[i].bufCb >>  0) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][2][0] = (pDecInfo->frameBufPool[i].bufCr >> 24) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][2][1] = (pDecInfo->frameBufPool[i].bufCr >> 16) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][2][2] = (pDecInfo->frameBufPool[i].bufCr >>  8) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][2][3] = (pDecInfo->frameBufPool[i].bufCr >>  0) & 0xFF;
				}
				else {
					frameAddr[i-pDecInfo->numFrameBuffers][2][0] = (pDecInfo->frameBufPool[i].bufCb >> 24) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][2][1] = (pDecInfo->frameBufPool[i].bufCb >> 16) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][2][2] = (pDecInfo->frameBufPool[i].bufCb >>  8) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][2][3] = (pDecInfo->frameBufPool[i].bufCb >>  0) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][1][0] = (pDecInfo->frameBufPool[i].bufCr >> 24) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][1][1] = (pDecInfo->frameBufPool[i].bufCr >> 16) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][1][2] = (pDecInfo->frameBufPool[i].bufCr >>  8) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][1][3] = (pDecInfo->frameBufPool[i].bufCr >>  0) & 0xFF;
				}
			}

			VpuWriteMem(pCodecInst->coreIdx, paraBuffer+384+128+384, (BYTE*)frameAddr, sizeof(frameAddr), VDI_BIG_ENDIAN);

			if (pDecInfo->wtlMode == FF_FIELD ) {
				for (i=pDecInfo->numFrameBuffers; i<pDecInfo->numFrameBuffers*2; i++) {
					frameAddr[i-pDecInfo->numFrameBuffers][0][0] = (pDecInfo->frameBufPool[i].bufYBot  >> 24) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][0][1] = (pDecInfo->frameBufPool[i].bufYBot  >> 16) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][0][2] = (pDecInfo->frameBufPool[i].bufYBot  >>  8) & 0xFF;
					frameAddr[i-pDecInfo->numFrameBuffers][0][3] = (pDecInfo->frameBufPool[i].bufYBot  >>  0) & 0xFF;
					if (pDecInfo->openParam.cbcrOrder == CBCR_ORDER_NORMAL) {
						frameAddr[i-pDecInfo->numFrameBuffers][1][0] = (pDecInfo->frameBufPool[i].bufCbBot >> 24) & 0xFF;
						frameAddr[i-pDecInfo->numFrameBuffers][1][1] = (pDecInfo->frameBufPool[i].bufCbBot >> 16) & 0xFF;
						frameAddr[i-pDecInfo->numFrameBuffers][1][2] = (pDecInfo->frameBufPool[i].bufCbBot >>  8) & 0xFF;
						frameAddr[i-pDecInfo->numFrameBuffers][1][3] = (pDecInfo->frameBufPool[i].bufCbBot >>  0) & 0xFF;
						frameAddr[i-pDecInfo->numFrameBuffers][2][0] = (pDecInfo->frameBufPool[i].bufCrBot >> 24) & 0xFF;
						frameAddr[i-pDecInfo->numFrameBuffers][2][1] = (pDecInfo->frameBufPool[i].bufCrBot >> 16) & 0xFF;
						frameAddr[i-pDecInfo->numFrameBuffers][2][2] = (pDecInfo->frameBufPool[i].bufCrBot >>  8) & 0xFF;
						frameAddr[i-pDecInfo->numFrameBuffers][2][3] = (pDecInfo->frameBufPool[i].bufCrBot >>  0) & 0xFF;
					}
					else {
						frameAddr[i-pDecInfo->numFrameBuffers][2][0] = (pDecInfo->frameBufPool[i].bufCbBot >> 24) & 0xFF;
						frameAddr[i-pDecInfo->numFrameBuffers][2][1] = (pDecInfo->frameBufPool[i].bufCbBot >> 16) & 0xFF;
						frameAddr[i-pDecInfo->numFrameBuffers][2][2] = (pDecInfo->frameBufPool[i].bufCbBot >>  8) & 0xFF;
						frameAddr[i-pDecInfo->numFrameBuffers][2][3] = (pDecInfo->frameBufPool[i].bufCbBot >>  0) & 0xFF;
						frameAddr[i-pDecInfo->numFrameBuffers][1][0] = (pDecInfo->frameBufPool[i].bufCrBot >> 24) & 0xFF;
						frameAddr[i-pDecInfo->numFrameBuffers][1][1] = (pDecInfo->frameBufPool[i].bufCrBot >> 16) & 0xFF;
						frameAddr[i-pDecInfo->numFrameBuffers][1][2] = (pDecInfo->frameBufPool[i].bufCrBot >>  8) & 0xFF;
						frameAddr[i-pDecInfo->numFrameBuffers][1][3] = (pDecInfo->frameBufPool[i].bufCrBot >>  0) & 0xFF;
					}
				}
				VpuWriteMem(pCodecInst->coreIdx, paraBuffer+384+128+384+384, (BYTE*)frameAddr, sizeof(frameAddr), VDI_BIG_ENDIAN);
			}


#ifdef SUPPORT_MEM_PROTECT
			start = pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].startAddress;
			end   = pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].endAddress;
			frame_buffer_end = pDecInfo->vbWTL.phys_addr + pDecInfo->vbWTL.size;
			start = (start< pDecInfo->vbWTL.phys_addr) ? start :pDecInfo->vbWTL.phys_addr;
			end   = (end  > frame_buffer_end) ? end : frame_buffer_end;

			pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].startAddress = start;
			pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].endAddress   = end;
#endif
			num *= 2;
		}
	}
  else{ // if (pCodecInst->productId == PRODUCT_ID_980)
#ifdef SUPPORT_MEM_PROTECT
        PhysicalAddress frame_buffer_end;
#endif
        if (pDecInfo->wtlEnable) {
            if (bufArray) {
                for(i=pDecInfo->numFrameBuffers; i<pDecInfo->numFrameBuffers*2; i++)
          {          pDecInfo->frameBufPool[i] = bufArray[i];

		VLOG(INFO, "WTL Frame Buffer Array [%d] = 0x%08x 0x%08x 0x%08x\n", i, bufArray[i].bufY, bufArray[i].bufCb, bufArray[i].bufCr);
//		VLOG(INFO, "WTL Frame Buffer Array Bot [%d] = 0x%08x 0x%08x 0x%08x\n", i, bufArray[i].bufYBot, bufArray[i].bufCbBot, bufArray[i].bufCrBot);
	}
            }
            else
            {
                ret = AllocateFrameBufferArrayV10(pCodecInst->coreIdx, &pDecInfo->frameBufPool[pDecInfo->numFrameBuffers], &pDecInfo->vbWTL, LINEAR_FRAME_MAP,
                    pDecInfo->openParam.cbcrInterleave, framebufFormat, pDecInfo->numFrameBuffers, stride, height, 0, FB_TYPE_CODEC, 0, &pDecInfo->dramCfg);
            }
            if (ret != RETCODE_SUCCESS) {
                LeaveLock(pCodecInst->coreIdx);
                return ret;
            }
            for (i=pDecInfo->numFrameBuffers; i<pDecInfo->numFrameBuffers*2; i++) {
                frameAddr[i-pDecInfo->numFrameBuffers][0][0] = (pDecInfo->frameBufPool[i].bufY  >> 24) & 0xFF;
                frameAddr[i-pDecInfo->numFrameBuffers][0][1] = (pDecInfo->frameBufPool[i].bufY  >> 16) & 0xFF;
                frameAddr[i-pDecInfo->numFrameBuffers][0][2] = (pDecInfo->frameBufPool[i].bufY  >>  8) & 0xFF;
                frameAddr[i-pDecInfo->numFrameBuffers][0][3] = (pDecInfo->frameBufPool[i].bufY  >>  0) & 0xFF;
                if (pDecInfo->openParam.cbcrOrder == CBCR_ORDER_NORMAL) {
                    frameAddr[i-pDecInfo->numFrameBuffers][1][0] = (pDecInfo->frameBufPool[i].bufCb >> 24) & 0xFF;
                    frameAddr[i-pDecInfo->numFrameBuffers][1][1] = (pDecInfo->frameBufPool[i].bufCb >> 16) & 0xFF;
                    frameAddr[i-pDecInfo->numFrameBuffers][1][2] = (pDecInfo->frameBufPool[i].bufCb >>  8) & 0xFF;
                    frameAddr[i-pDecInfo->numFrameBuffers][1][3] = (pDecInfo->frameBufPool[i].bufCb >>  0) & 0xFF;
                    frameAddr[i-pDecInfo->numFrameBuffers][2][0] = (pDecInfo->frameBufPool[i].bufCr >> 24) & 0xFF;
                    frameAddr[i-pDecInfo->numFrameBuffers][2][1] = (pDecInfo->frameBufPool[i].bufCr >> 16) & 0xFF;
                    frameAddr[i-pDecInfo->numFrameBuffers][2][2] = (pDecInfo->frameBufPool[i].bufCr >>  8) & 0xFF;
                    frameAddr[i-pDecInfo->numFrameBuffers][2][3] = (pDecInfo->frameBufPool[i].bufCr >>  0) & 0xFF;
                }
                else {
                    frameAddr[i-pDecInfo->numFrameBuffers][2][0] = (pDecInfo->frameBufPool[i].bufCb >> 24) & 0xFF;
                    frameAddr[i-pDecInfo->numFrameBuffers][2][1] = (pDecInfo->frameBufPool[i].bufCb >> 16) & 0xFF;
                    frameAddr[i-pDecInfo->numFrameBuffers][2][2] = (pDecInfo->frameBufPool[i].bufCb >>  8) & 0xFF;
                    frameAddr[i-pDecInfo->numFrameBuffers][2][3] = (pDecInfo->frameBufPool[i].bufCb >>  0) & 0xFF;
                    frameAddr[i-pDecInfo->numFrameBuffers][1][0] = (pDecInfo->frameBufPool[i].bufCr >> 24) & 0xFF;
                    frameAddr[i-pDecInfo->numFrameBuffers][1][1] = (pDecInfo->frameBufPool[i].bufCr >> 16) & 0xFF;
                    frameAddr[i-pDecInfo->numFrameBuffers][1][2] = (pDecInfo->frameBufPool[i].bufCr >>  8) & 0xFF;
                    frameAddr[i-pDecInfo->numFrameBuffers][1][3] = (pDecInfo->frameBufPool[i].bufCr >>  0) & 0xFF;
                }
            }

            VpuWriteMem(pCodecInst->coreIdx, paraBuffer+384+128+384, (BYTE*)frameAddr, sizeof(frameAddr), VDI_BIG_ENDIAN);

#ifdef SUPPORT_MEM_PROTECT
            start = pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].startAddress;
            end   = pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].endAddress;
            frame_buffer_end = pDecInfo->vbWTL.phys_addr + pDecInfo->vbWTL.size;

            start = (start< pDecInfo->vbWTL.phys_addr) ? start :pDecInfo->vbWTL.phys_addr;
            end   = (end  > frame_buffer_end) ? end : frame_buffer_end;

            pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].startAddress = start;
            pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].endAddress   = end;
#endif
            num *= 2;
        }
  }

	if (!ConfigSecAXI(pCodecInst->coreIdx, pDecInfo->openParam.bitstreamFormat, &pDecInfo->secAxiInfo, stride, height,
		pDecInfo->initialInfo.profile&0xff)) {
			LeaveLock(pCodecInst->coreIdx);
			return RETCODE_INSUFFICIENT_RESOURCE;
	}
#ifdef SUPPORT_MEM_PROTECT
	if (pDecInfo->secAxiInfo.bufSize) {
		pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_SEC_AXI].enable = 1;
		pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_SEC_AXI].isSecondary = 1;
		pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_SEC_AXI].startAddress = pDecInfo->secAxiInfo.bufBitUse;
		pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_SEC_AXI].endAddress = pDecInfo->secAxiInfo.bufBitUse + pDecInfo->secAxiInfo.bufSize;
	}
#endif

	// Tell the decoder how much frame buffers were allocated.
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_BUF_NUM, num);
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_BUF_STRIDE, stride);
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_AXI_BIT_ADDR, pDecInfo->secAxiInfo.bufBitUse);
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_AXI_IPACDC_ADDR, pDecInfo->secAxiInfo.bufIpAcDcUse);
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_AXI_DBKY_ADDR, pDecInfo->secAxiInfo.bufDbkYUse);
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_AXI_DBKC_ADDR, pDecInfo->secAxiInfo.bufDbkCUse);
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_AXI_OVL_ADDR, pDecInfo->secAxiInfo.bufOvlUse);
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_AXI_BTP_ADDR, pDecInfo->secAxiInfo.bufBtpUse);
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_DELAY, pDecInfo->frameDelay);


	// Maverick Cache Configuration
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_CACHE_CONFIG, pDecInfo->cacheConfig.CacheMode);


	if (pCodecInst->codecMode == VPX_DEC) {
		vpu_buffer_t	*pvbSlice = &pDecInfo->vbSlice;
#ifdef SUPPORT_MEM_PROTECT
		WriteMemProtectCfg *pCgf    = &pDecInfo->writeMemProtectCfg;
#endif
		if (pvbSlice->size == 0) {
			//pvbSlice->size     = 17*4*(pDecInfo->initialInfo.picWidth*pDecInfo->initialInfo.picHeight/256);	//(MB information + split MVs)*4*MbNumbyte
			pvbSlice->size = VP8_MB_SAVE_SIZE;
			if (vdi_allocate_dma_memory(pCodecInst->coreIdx, pvbSlice) < 0) {
				LeaveLock(pCodecInst->coreIdx);
				return RETCODE_INSUFFICIENT_RESOURCE;
			}
		}

#ifdef SUPPORT_MEM_PROTECT
		pCgf->decRegion[WPROT_DEC_PIC_SAVE].enable = 1;
		pCgf->decRegion[WPROT_DEC_PIC_SAVE].isSecondary = 0;
		pCgf->decRegion[WPROT_DEC_PIC_SAVE].startAddress = pvbSlice->phys_addr;
		pCgf->decRegion[WPROT_DEC_PIC_SAVE].endAddress =  pvbSlice->phys_addr + pvbSlice->size;
#endif
		VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_MB_BUF_BASE, pvbSlice->phys_addr);
	}

	if (pCodecInst->codecMode == AVC_DEC && pDecInfo->initialInfo.profile == 66) {
		vpu_buffer_t	*pvbSlice = &pDecInfo->vbSlice;
#ifdef SUPPORT_MEM_PROTECT
		WriteMemProtectCfg *pCgf    = &pDecInfo->writeMemProtectCfg;
#endif
		if (pvbSlice->size == 0) {
			//pvbSlice->size     = pDecInfo->initialInfo.picWidth*pDecInfo->initialInfo.picHeight*3/4;	// this buffer for ASO/FMO
			pvbSlice->size = SLICE_SAVE_SIZE;
			if (vdi_allocate_dma_memory(pCodecInst->coreIdx, pvbSlice) < 0) {
				LeaveLock(pCodecInst->coreIdx);
				return RETCODE_INSUFFICIENT_RESOURCE;
			}
		}
        VLOG(INFO, "[Slice Buffer] P:0x%x B:0x%x\n", pvbSlice->phys_addr, pvbSlice->base);

#ifdef SUPPORT_MEM_PROTECT
		pCgf->decRegion[WPROT_DEC_PIC_SAVE].enable = 1;
		pCgf->decRegion[WPROT_DEC_PIC_SAVE].isSecondary = 0;
		pCgf->decRegion[WPROT_DEC_PIC_SAVE].startAddress = pvbSlice->phys_addr;
		pCgf->decRegion[WPROT_DEC_PIC_SAVE].endAddress =  pvbSlice->phys_addr + pvbSlice->size;
#endif
		VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_SLICE_BB_START, pvbSlice->phys_addr);
		VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_SLICE_BB_SIZE, (pvbSlice->size/1024));
	}


	if (pCodecInst->productId == PRODUCT_ID_980) {
		val = 0;
		val |= (pDecInfo->openParam.bwbEnable<<15);
		val |= (pDecInfo->wtlMode<<17)|(pDecInfo->tiled2LinearMode<<13)|(pDecInfo->mapType<<9)|(FORMAT_420<<6);
		val |= ((pDecInfo->openParam.cbcrInterleave)<<2); // Interleave bit position is modified
		val |= pDecInfo->openParam.frameEndian;
		VpuWriteReg(pCodecInst->coreIdx, BIT_FRAME_MEM_CTRL, val);
	}
	else if (pCodecInst->productId == PRODUCT_ID_960) {
		val = 0;
        val |= (pDecInfo->wtlEnable<<17);
		val |= (pDecInfo->openParam.bwbEnable<<12);
		if (pDecInfo->mapType) {
			if (pDecInfo->mapType == TILED_FRAME_MB_RASTER_MAP || pDecInfo->mapType == TILED_FIELD_MB_RASTER_MAP)
				val |= (pDecInfo->tiled2LinearEnable<<11)|(0x03<<9)|(FORMAT_420<<6);
			else
				val |= (pDecInfo->tiled2LinearEnable<<11)|(0x02<<9)|(FORMAT_420<<6);
		}
		val |= ((pDecInfo->openParam.cbcrInterleave)<<2); // Interleave bit position is modified
		val |= pDecInfo->openParam.frameEndian;
		VpuWriteReg(pCodecInst->coreIdx, BIT_FRAME_MEM_CTRL, val);
	}
	else {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_NOT_FOUND_VPU_DEVICE;
	}


	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_MAX_DEC_SIZE, 0);  // Must set to zero at API 2.1.5 version

	SetPendingInst(pCodecInst->coreIdx, pCodecInst);
	BitIssueCommand(pCodecInst->coreIdx, pCodecInst, SET_FRAME_BUF);
	if (vdi_wait_vpu_busy(pCodecInst->coreIdx, VPU_BUSY_CHECK_TIMEOUT, BIT_BUSY_FLAG) == -1) {
		if (pCodecInst->loggingEnable)
			vdi_log(pCodecInst->coreIdx, SET_FRAME_BUF, 2);
		SetPendingInst(pCodecInst->coreIdx, 0);
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_VPU_RESPONSE_TIMEOUT;
	}
	if (pCodecInst->loggingEnable)
		vdi_log(pCodecInst->coreIdx, SET_FRAME_BUF, 0);

#ifdef SUPPORT_MEM_PROTECT
	if (VpuReadReg(pCodecInst->coreIdx, RET_SET_FRAME_SUCCESS) & (1<<31)) {
		SetPendingInst(pCodecInst->coreIdx, 0);
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_MEMORY_ACCESS_VIOLATION;
	}
#endif
	SetPendingInst(pCodecInst->coreIdx, 0);
	LeaveLock(pCodecInst->coreIdx);
	return RETCODE_SUCCESS;
}

RetCode VPU_DecGetFrameBuffer(
	DecHandle     handle,
	int           frameIdx,
	FrameBuffer * frameBuf)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	RetCode ret;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

    if (pDecInfo->wtlEnable)
    {
        if (frameIdx<0 || frameIdx>pDecInfo->numFrameBuffers*2)
            return RETCODE_INVALID_PARAM;
    }
    else
    {
        if (frameIdx<0 || frameIdx>pDecInfo->numFrameBuffers)
            return RETCODE_INVALID_PARAM;
    }

	if (frameBuf==0)
		return RETCODE_INVALID_PARAM;

	*frameBuf = pDecInfo->frameBufPool[frameIdx];

	return RETCODE_SUCCESS;

}

RetCode VPU_DecGetBitstreamBuffer( DecHandle handle,
	PhysicalAddress * prdPrt,
	PhysicalAddress * pwrPtr,
	int * size)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	PhysicalAddress rdPtr;
	PhysicalAddress wrPtr;
	int room;
	RetCode ret;


	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;


	if ( prdPrt == 0 || pwrPtr == 0 || size == 0) {
		return RETCODE_INVALID_PARAM;
	}

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	SetClockGate(pCodecInst->coreIdx, 1);

	if (GetPendingInst(pCodecInst->coreIdx) == pCodecInst)
		rdPtr = VpuReadReg(pCodecInst->coreIdx, pDecInfo->streamRdPtrRegAddr);
	else
		rdPtr = pDecInfo->streamRdPtr;

	SetClockGate(pCodecInst->coreIdx, 0);

	wrPtr = pDecInfo->streamWrPtr;

	if (wrPtr < rdPtr) {
		room = rdPtr - wrPtr - VPU_GBU_SIZE*2 - 1;
	}
	else {
		room = ( pDecInfo->streamBufEndAddr - wrPtr ) + ( rdPtr - pDecInfo->streamBufStartAddr ) - VPU_GBU_SIZE*2 - 1;
	}


	*prdPrt = rdPtr;
	*pwrPtr = wrPtr;
	room = ( ( room >> 9 ) << 9 ); // multiple of 512
	*size = room;

	return RETCODE_SUCCESS;
}


RetCode VPU_DecUpdateBitstreamBuffer(
	DecHandle handle,
	int size)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	PhysicalAddress wrPtr;
	PhysicalAddress rdPtr;
	RetCode ret;
	int		val = 0;
	int		room = 0;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;
	wrPtr = pDecInfo->streamWrPtr;

	SetClockGate(pCodecInst->coreIdx, 1);

	if (size == 0)
	{

		val = VpuReadReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM);
		val |= 1 << 2;
		pDecInfo->streamEndflag = val;
		if (GetPendingInst(pCodecInst->coreIdx) == pCodecInst)
			VpuWriteReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM, val);
		SetClockGate(pCodecInst->coreIdx, 0);
		return RETCODE_SUCCESS;
	}

	if (size == -1)
	{
		val = VpuReadReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM);
		val &= ~(1 << 2);
		pDecInfo->streamEndflag = val;
		if (GetPendingInst(pCodecInst->coreIdx) == pCodecInst)
			VpuWriteReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM, val);

		SetClockGate(pCodecInst->coreIdx, 0);
		return RETCODE_SUCCESS;
	}


	if (GetPendingInst(pCodecInst->coreIdx) == pCodecInst)
		rdPtr = VpuReadReg(pCodecInst->coreIdx, pDecInfo->streamRdPtrRegAddr);
	else
		rdPtr = pDecInfo->streamRdPtr;

	if (wrPtr < rdPtr) {
		if (rdPtr <= wrPtr + size) {
			SetClockGate(pCodecInst->coreIdx, 0);
			return RETCODE_INVALID_PARAM;
		}
	}

	wrPtr += size;

	if (wrPtr > pDecInfo->streamBufEndAddr) {
		room = wrPtr - pDecInfo->streamBufEndAddr;
		wrPtr = pDecInfo->streamBufStartAddr;
		wrPtr += room;
	}
	if (wrPtr == pDecInfo->streamBufEndAddr) {
		wrPtr = pDecInfo->streamBufStartAddr;
	}

	pDecInfo->streamWrPtr = wrPtr;
	pDecInfo->streamRdPtr = rdPtr;

	if (GetPendingInst(pCodecInst->coreIdx) == pCodecInst)
		VpuWriteReg(pCodecInst->coreIdx, pDecInfo->streamWrPtrRegAddr, wrPtr);

	SetClockGate(pCodecInst->coreIdx, 0);
	return RETCODE_SUCCESS;
}

RetCode VPU_HWReset(Uint32 coreIdx)
{
	if (vdi_hw_reset(coreIdx) < 0 )
		return RETCODE_FAILURE;

	if (GetPendingInst(coreIdx))
	{
		SetPendingInst(coreIdx, 0);
		LeaveLock(coreIdx);	//if vpu is in a lock state. release the state;
	}
	return RETCODE_SUCCESS;
}

/**
* VPU_SWReset
* IN
*    forcedReset : 1 if there is no need to waiting for BUS transaction,
*                  0 for otherwise
* OUT
*    RetCode : RETCODE_FAILURE if failed to reset,
*              RETCODE_SUCCESS for otherwise
*/
RetCode VPU_SWReset(Uint32 coreIdx, int resetMode, void *pendingInst)
{
	Uint32 cmd;
	CodecInst *pCodecInst = (CodecInst *)pendingInst;

	SetClockGate(coreIdx, 1);
	if (pCodecInst) {
		SetPendingInst(pCodecInst->coreIdx, 0);
		LeaveLock(coreIdx);
		SetClockGate(coreIdx, 1);
		if (pCodecInst->loggingEnable) {
			vdi_log(pCodecInst->coreIdx, 0x10, 1);
		}
	}

	if (resetMode == SW_RESET_SAFETY || resetMode == SW_RESET_ON_BOOT) {
		if (vdi_wait_vpu_busy(coreIdx, VPU_BUSY_CHECK_TIMEOUT, GDI_BWB_STATUS) == -1) {
			if (pCodecInst) {
				if (pCodecInst->loggingEnable) {
					vdi_log(pCodecInst->coreIdx, 0x10, 2);
				}
			}
			SetClockGate(coreIdx, 0);
			return RETCODE_VPU_RESPONSE_TIMEOUT;
		}
		// Waiting for completion of bus transaction
		// Step1 : No more request
		VpuWriteReg(coreIdx, GDI_BUS_CTRL, 0x11);	// no more request {3'b0,no_more_req_sec,3'b0,no_more_req}

		// Step2 : Waiting for completion of bus transaction
		// while (VpuReadReg(coreIdx, GDI_BUS_STATUS) != 0x77)
		// ;
		if (vdi_wait_bus_busy(coreIdx, VPU_BUSY_CHECK_TIMEOUT, GDI_BUS_STATUS) == -1) {
			if (pCodecInst) {
				if (pCodecInst->loggingEnable) {
					vdi_log(pCodecInst->coreIdx, 0x10, 2);
				}
			}

			VpuWriteReg(coreIdx, GDI_BUS_CTRL, 0x00);
			SetClockGate(coreIdx, 0);
			return RETCODE_VPU_RESPONSE_TIMEOUT;
		}

		// Step3 : clear GDI_BUS_CTRL
	}

	cmd = 0;
	// Software Reset Trigger
	if (resetMode != SW_RESET_ON_BOOT)
		cmd =  VPU_SW_RESET_BPU_CORE | VPU_SW_RESET_BPU_BUS;
	cmd |= VPU_SW_RESET_VCE_CORE | VPU_SW_RESET_VCE_BUS;
	if (resetMode == SW_RESET_ON_BOOT)
		cmd |= VPU_SW_RESET_GDI_CORE | VPU_SW_RESET_GDI_BUS;// If you reset GDI, tiled map should be reconfigured
	VpuWriteReg(coreIdx, BIT_SW_RESET, cmd);

	// wait until reset is done
	// while(VpuReadReg(coreIdx, BIT_SW_RESET_STATUS) != 0)
	//	;
	if (vdi_wait_vpu_busy(coreIdx, VPU_BUSY_CHECK_TIMEOUT, BIT_SW_RESET_STATUS) == -1) {
		if (pCodecInst) {
			if (pCodecInst->loggingEnable) {
				vdi_log(pCodecInst->coreIdx, 0x10, 2);
			}
		}
		VpuWriteReg(coreIdx, BIT_SW_RESET, 0x00);
		VpuWriteReg(coreIdx, GDI_BUS_CTRL, 0x00);
		SetClockGate(coreIdx, 0);
		return RETCODE_VPU_RESPONSE_TIMEOUT;
	}

	VpuWriteReg(coreIdx, BIT_SW_RESET, 0);
	VpuWriteReg(coreIdx, GDI_BUS_CTRL, 0x00);

	if (pCodecInst) {
		if (pCodecInst->loggingEnable) {
			vdi_log(pCodecInst->coreIdx, 0x10, 0);
		}
	}

	SetClockGate(coreIdx, 0);
	return RETCODE_SUCCESS;
}


//---- VPU_SLEEP/WAKE
RetCode VPU_SleepWake(Uint32 coreIdx, int iSleepWake)
{
	static unsigned int regBk[64];
	int i=0;
	Uint32 data;
	const Uint16* bit_code = NULL;
	if (s_pusBitCode[coreIdx] && s_bitCodeSize[coreIdx] > 0)
		bit_code = s_pusBitCode[coreIdx];

	if (!bit_code)
		return RETCODE_INVALID_PARAM;

	// do not use lock function in power management routine.
	SetClockGate(coreIdx, 1);

	if (vdi_wait_vpu_busy(coreIdx, VPU_BUSY_CHECK_TIMEOUT, BIT_BUSY_FLAG) == -1) {
		SetClockGate(coreIdx, 0);
		return RETCODE_VPU_RESPONSE_TIMEOUT;
	}

	if(iSleepWake==1)
	{
		for ( i = 0 ; i < 64 ; i++)
			regBk[i] = VpuReadReg(coreIdx, BIT_BASE + 0x100 + (i * 4));
	}
	else
	{
		VpuWriteReg(coreIdx, BIT_CODE_RUN, 0);

// 		//---- LOAD BOOT CODE
		for (i=0; i<512; i++)
		{
			data = bit_code[i] & 0xFFFF;
			VpuWriteReg(coreIdx, BIT_CODE_DOWN, (i<<16) | data);
		}

		for ( i = 0 ; i < 64 ; i++)
			VpuWriteReg(coreIdx, BIT_BASE + 0x100 + (i * 4), regBk[i]);

		VpuWriteReg(coreIdx, BIT_BUSY_FLAG, 1);
		VpuWriteReg(coreIdx, BIT_CODE_RESET, 1);
		VpuWriteReg(coreIdx, BIT_CODE_RUN, 1);

		if (vdi_wait_vpu_busy(coreIdx, VPU_BUSY_CHECK_TIMEOUT, BIT_BUSY_FLAG) == -1) {
			SetClockGate(coreIdx, 0);
			return RETCODE_VPU_RESPONSE_TIMEOUT;
		}
	}



	SetClockGate(coreIdx, 0);
	return RETCODE_SUCCESS;
}


RetCode VPU_DecStartOneFrame(DecHandle handle, DecParam *param)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	Uint32 rotMir;
	Uint32 val;
	RetCode ret;
	vpu_instance_pool_t *vip;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;


	if (pDecInfo->stride == 0) { // This means frame buffers have not been registered.
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

	rotMir = 0;
	if (pDecInfo->rotationEnable) {
		rotMir |= 0x10; // Enable rotator
		switch (pDecInfo->rotationAngle) {
		case 0:
			rotMir |= 0x0;
			break;

		case 90:
			rotMir |= 0x1;
			break;

		case 180:
			rotMir |= 0x2;
			break;

		case 270:
			rotMir |= 0x3;
			break;
		}
	}

	if (pDecInfo->mirrorEnable) {
		rotMir |= 0x10; // Enable rotator
		switch (pDecInfo->mirrorDirection) {
		case MIRDIR_NONE :
			rotMir |= 0x0;
			break;

		case MIRDIR_VER :
			rotMir |= 0x4;
			break;

		case MIRDIR_HOR :
			rotMir |= 0x8;
			break;

		case MIRDIR_HOR_VER :
			rotMir |= 0xc;
			break;

		}
	}

	if (pDecInfo->tiled2LinearEnable) {
		rotMir |= 0x10;
	}

	if (pDecInfo->deringEnable) {
		rotMir |= 0x20; // Enable Dering Filter
	}

	if (rotMir && !pDecInfo->rotatorOutputValid) {
		return RETCODE_ROTATOR_OUTPUT_NOT_SET;
	}

	EnterLock(pCodecInst->coreIdx);

	vip = (vpu_instance_pool_t *)vdi_get_instance_pool(pCodecInst->coreIdx);
	if (!vip) {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_INVALID_HANDLE;
	}

	if (GetPendingInst(pCodecInst->coreIdx)) {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	VpuWriteReg(pCodecInst->coreIdx, RET_DEC_PIC_CROP_LEFT_RIGHT, 0);				// frame crop information(left, right)
	VpuWriteReg(pCodecInst->coreIdx, RET_DEC_PIC_CROP_TOP_BOTTOM, 0);				// frame crop information(top, bottom)



	if (pCodecInst->productId == PRODUCT_ID_960) {
		if (pDecInfo->mapType>LINEAR_FRAME_MAP && pDecInfo->mapType<=TILED_MIXED_V_MAP) {
			SetTiledFrameBase(pCodecInst->coreIdx, pDecInfo->mapCfg.tiledBaseAddr);
		}
		else {
			SetTiledFrameBase(pCodecInst->coreIdx, 0);
		}

		val = SetTiledMapType(pCodecInst->coreIdx, &pDecInfo->mapCfg, pDecInfo->mapType, pDecInfo->stride, pDecInfo->openParam.cbcrInterleave, &pDecInfo->dramCfg);
		if (val == 0) {
			LeaveLock(pCodecInst->coreIdx);
			return RETCODE_INVALID_PARAM;
		}
	}

	if (pCodecInst->productId == PRODUCT_ID_980) {
		if (pDecInfo->mapType != LINEAR_FRAME_MAP && pDecInfo->mapType != LINEAR_FIELD_MAP) {
			val = SetTiledMapType(pCodecInst->coreIdx, &pDecInfo->mapCfg, pDecInfo->mapType,
				(pDecInfo->stride > pDecInfo->frameBufferHeight)?pDecInfo->stride: pDecInfo->frameBufferHeight,
				pDecInfo->openParam.cbcrInterleave, &pDecInfo->dramCfg);
		} else {
			val = SetTiledMapType(pCodecInst->coreIdx, &pDecInfo->mapCfg, pDecInfo->mapType,
				pDecInfo->stride, pDecInfo->openParam.cbcrInterleave, &pDecInfo->dramCfg);
		}
		if (val == 0) {
			LeaveLock(pCodecInst->coreIdx);
			return RETCODE_INVALID_PARAM;
		}
	}



	if (rotMir & 0x30) // rotator or dering or tiled2linear enabled
	{
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_ROT_MODE, rotMir);
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_ROT_INDEX, pDecInfo->rotatorOutput.myIndex);
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_ROT_ADDR_Y,  pDecInfo->rotatorOutput.bufY);
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_ROT_ADDR_CB, pDecInfo->rotatorOutput.bufCb);
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_ROT_ADDR_CR, pDecInfo->rotatorOutput.bufCr);
		if (pCodecInst->productId == PRODUCT_ID_980) {
			VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_ROT_BOTTOM_Y,  pDecInfo->rotatorOutput.bufYBot);
			VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_ROT_BOTTOM_CB, pDecInfo->rotatorOutput.bufCbBot);
			VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_ROT_BOTTOM_CR, pDecInfo->rotatorOutput.bufCrBot);
		}
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_ROT_STRIDE, pDecInfo->rotatorStride);
	}
	else
	{
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_ROT_MODE, rotMir);
	}
	if(pDecInfo->userDataEnable) {
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_USER_DATA_BASE_ADDR, pDecInfo->userDataBufAddr);
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_USER_DATA_BUF_SIZE, pDecInfo->userDataBufSize);
	}
	else {
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_USER_DATA_BASE_ADDR, 0);
		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_USER_DATA_BUF_SIZE, 0);
	}

	val = 0;
	if (param->iframeSearchEnable != 0) // if iframeSearch is Enable, other bit is ignore;
	{
		val |= (pDecInfo->userDataReportMode		<<10 );

		if (pCodecInst->codecMode == AVC_DEC || pCodecInst->codecMode == VC1_DEC)
		{
			if (param->iframeSearchEnable==1)
				val |= (1<< 11) | (1<<2);
			else if (param->iframeSearchEnable==2)
				val |= (1<<2);
		}
		else
			val |= (( param->iframeSearchEnable &0x1)   << 2 );
	}
	else
	{
		val |= (pDecInfo->userDataReportMode		<<10 );
			val |= (pDecInfo->userDataEnable		<< 5 );
		val |= (param->skipframeMode				<< 3 );
	}

	if (pCodecInst->codecMode == AVC_DEC && pDecInfo->lowDelayInfo.lowDelayEn)
		val |= (pDecInfo->lowDelayInfo.lowDelayEn <<18 );
	if (pCodecInst->codecMode == MP2_DEC)
		val |= ((param->DecStdParam.mp2PicFlush&1)<<15);
	if (pCodecInst->codecMode == RV_DEC)
		val |= ((param->DecStdParam.rvDbkMode&0x0f)<<16);


	VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_OPTION, val);

 	if (pDecInfo->lowDelayInfo.lowDelayEn)
 		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_NUM_ROWS, pDecInfo->lowDelayInfo.numRows);
    else
 		VpuWriteReg(pCodecInst->coreIdx, CMD_DEC_PIC_NUM_ROWS, 0);

	val = 0;
	val = (
		(pDecInfo->secAxiInfo.useBitEnable&0x01)<<0 |
		(pDecInfo->secAxiInfo.useIpEnable&0x01)<<1 |
		(pDecInfo->secAxiInfo.useDbkYEnable&0x01)<<2 |
		(pDecInfo->secAxiInfo.useDbkCEnable&0x01)<<3 |
		(pDecInfo->secAxiInfo.useOvlEnable&0x01)<<4 |
		(pDecInfo->secAxiInfo.useBtpEnable&0x01)<<5 |
		(pDecInfo->secAxiInfo.useBitEnable&0x01)<<8 |
		(pDecInfo->secAxiInfo.useIpEnable&0x01)<<9 |
		(pDecInfo->secAxiInfo.useDbkYEnable&0x01)<<10 |
		(pDecInfo->secAxiInfo.useDbkCEnable&0x01)<<11 |
		(pDecInfo->secAxiInfo.useOvlEnable&0x01)<<12
		|
		(pDecInfo->secAxiInfo.useBtpEnable&0x01)<<13
		);

	VpuWriteReg(pCodecInst->coreIdx, BIT_AXI_SRAM_USE, val);

	VpuWriteReg(pCodecInst->coreIdx, pDecInfo->streamWrPtrRegAddr, pDecInfo->streamWrPtr);
	VpuWriteReg(pCodecInst->coreIdx, pDecInfo->streamRdPtrRegAddr, pDecInfo->streamRdPtr);


	EnterDispFlagLock(pCodecInst->coreIdx);
	val = pDecInfo->frameDisplayFlag;
	val &= ~pDecInfo->clearDisplayIndexes;
    VpuWriteReg(pCodecInst->coreIdx, pDecInfo->frameDisplayFlagRegAddr, val);
    pDecInfo->clearDisplayIndexes = 0;
    LeaveDispFlagLock(pCodecInst->coreIdx);


	pDecInfo->streamEndflag &= ~(3<<3);
	if (pDecInfo->openParam.bitstreamMode == BS_MODE_ROLLBACK)  //rollback mode
		pDecInfo->streamEndflag |= (1<<3);
	else if (pDecInfo->openParam.bitstreamMode == BS_MODE_PIC_END)
		pDecInfo->streamEndflag |= (2<<3);

	VpuWriteReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM, pDecInfo->streamEndflag);


	if (pCodecInst->productId == PRODUCT_ID_980) {
		val = 0;
		val |= (pDecInfo->openParam.bwbEnable<<15);
		val |= (pDecInfo->wtlMode<<17)|(pDecInfo->tiled2LinearMode<<13)|(pDecInfo->mapType<<9)|(FORMAT_420<<6);
	}
	else if (pCodecInst->productId == PRODUCT_ID_960) {
		val = 0;
        val |= (pDecInfo->wtlEnable<<17);
		val |= (pDecInfo->openParam.bwbEnable<<12);
		if (pDecInfo->mapType) {
			if (pDecInfo->mapType == TILED_FRAME_MB_RASTER_MAP || pDecInfo->mapType == TILED_FIELD_MB_RASTER_MAP)
				val |= (pDecInfo->tiled2LinearEnable<<11)|(0x03<<9)|(FORMAT_420<<6);
			else
				val |= (pDecInfo->tiled2LinearEnable<<11)|(0x02<<9)|(FORMAT_420<<6);
		}
	}
	else {
        LeaveLock(pCodecInst->coreIdx);
		return RETCODE_NOT_FOUND_VPU_DEVICE;
	}
	val |= ((pDecInfo->openParam.cbcrInterleave)<<2);   // Interleave bit position is modified
	val |= pDecInfo->openParam.frameEndian;
	VpuWriteReg(pCodecInst->coreIdx, BIT_FRAME_MEM_CTRL, val);

	val = pDecInfo->openParam.streamEndian;
	VpuWriteReg(pCodecInst->coreIdx, BIT_BIT_STREAM_CTRL, val);

	pDecInfo->frameStartPos = pDecInfo->streamRdPtr;

	BitIssueCommand(pCodecInst->coreIdx, pCodecInst, PIC_RUN);

	SetPendingInst(pCodecInst->coreIdx, pCodecInst);

	return RETCODE_SUCCESS;
}


RetCode VPU_DecGetOutputInfo(
	DecHandle handle,
	DecOutputInfo * info)
{
	CodecInst * pCodecInst;
	DecInfo	  *	pDecInfo;
	RetCode		ret;
	Uint32		val  = 0;
	Uint32		val2 = 0;
	Rect        rectInfo;


	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS) {
		return ret;
	}

	if (info == 0) {
		return RETCODE_INVALID_PARAM;
	}

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	if (pCodecInst != GetPendingInst(pCodecInst->coreIdx)) {
		SetPendingInst(pCodecInst->coreIdx, 0);
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

	if (pCodecInst->loggingEnable)
		vdi_log(pCodecInst->coreIdx, PIC_RUN, 0);


	pDecInfo->streamRdPtr = VpuReadReg(pCodecInst->coreIdx, pDecInfo->streamRdPtrRegAddr);
	pDecInfo->frameDisplayFlag = VpuReadReg(pCodecInst->coreIdx, pDecInfo->frameDisplayFlagRegAddr);

	pDecInfo->frameEndPos = pDecInfo->streamRdPtr;

	if (pDecInfo->frameEndPos < pDecInfo->frameStartPos) {
		info->consumedByte = pDecInfo->frameEndPos + pDecInfo->streamBufSize - pDecInfo->frameStartPos;
	}
	else {
		info->consumedByte = pDecInfo->frameEndPos - pDecInfo->frameStartPos;
	}


	info->rdPtr = pDecInfo->streamRdPtr;
	info->wrPtr = pDecInfo->streamWrPtr;

	info->frameDisplayFlag = pDecInfo->frameDisplayFlag;

	info->frameCycle = VpuReadReg(pCodecInst->coreIdx, BIT_FRAME_CYCLE);


	val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_SUCCESS);
#ifdef SUPPORT_MEM_PROTECT
	if (val & (1<<31)) {
		info->wprotErrReason = VpuReadReg(pCodecInst->coreIdx, GDI_WPROT_ERR_RSN);
		info->wprotErrAddress = VpuReadReg(pCodecInst->coreIdx, GDI_WPROT_ERR_ADR);

		SetPendingInst(pCodecInst->coreIdx, 0);
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_MEMORY_ACCESS_VIOLATION;
	}
#endif

	if( pCodecInst->codecMode == AVC_DEC ) {
		info->notSufficientPsBuffer = (val >> 3) & 0x1;
		info->notSufficientSliceBuffer = (val >> 2) & 0x1;
	}

	info->chunkReuseRequired = 0;
	if (pDecInfo->openParam.bitstreamMode == BS_MODE_PIC_END) {
		if (pCodecInst->codecMode == AVC_DEC) {
			info->chunkReuseRequired = ((val >> 16) & 0x01);	// in case of NPF frame

			val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_DECODED_IDX);
			if (val == (Uint32)-1) {
				info->chunkReuseRequired = 1;
			}
		}

		if (pCodecInst->codecMode == MP4_DEC) {
			info->chunkReuseRequired = ((val >> 16) & 0x01); // in case of mpeg4 packed mode
		}
	}

	info->indexFrameDecoded	= VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_DECODED_IDX);
	info->indexFrameDisplay	= VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_DISPLAY_IDX);
	if (!pDecInfo->reorderEnable)
		info->indexFrameDisplay = info->indexFrameDecoded;

	val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_SIZE); // decoding picture size
	info->decPicWidth  = (val>>16) & 0xFFFF;
	info->decPicHeight = (val) & 0xFFFF;

	if (info->indexFrameDecoded >= 0 && info->indexFrameDecoded < MAX_GDI_IDX)
	{
		if( pCodecInst->codecMode == VPX_DEC ) {
			if ( pCodecInst->codecModeAux == VPX_AUX_VP8 ) {
				// VP8 specific header information
				// h_scale[31:30] v_scale[29:28] pic_width[27:14] pic_height[13:0]
				val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_VP8_SCALE_INFO);
				info->vp8ScaleInfo.hScaleFactor = (val >> 30) & 0x03;
				info->vp8ScaleInfo.vScaleFactor = (val >> 28) & 0x03;
				info->vp8ScaleInfo.picWidth = (val >> 14) & 0x3FFF;
				info->vp8ScaleInfo.picHeight = (val >> 0) & 0x3FFF;
				// ref_idx_gold[31:24], ref_idx_altr[23:16], ref_idx_last[15: 8],
				// version_number[3:1], show_frame[0]
				val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_VP8_PIC_REPORT);
				info->vp8PicInfo.refIdxGold = (val >> 24) & 0x0FF;
				info->vp8PicInfo.refIdxAltr = (val >> 16) & 0x0FF;
				info->vp8PicInfo.refIdxLast = (val >> 8) & 0x0FF;
				info->vp8PicInfo.versionNumber = (val >> 1) & 0x07;
				info->vp8PicInfo.showFrame = (val >> 0) & 0x01;
			}
		}

		//default value
		rectInfo.left   = 0;
		rectInfo.right  = info->decPicWidth;
		rectInfo.top    = 0;
		rectInfo.bottom = info->decPicHeight;

		if (pCodecInst->codecMode == AVC_DEC)
		{
			val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_CROP_LEFT_RIGHT);				// frame crop information(left, right)
			val2 = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_CROP_TOP_BOTTOM);			// frame crop information(top, bottom)

			if (val == (Uint32)-1 || val == 0)
			{
				rectInfo.left   = 0;
				rectInfo.right  = info->decPicWidth;
			}
			else
			{
				rectInfo.left    = ((val>>16) & 0xFFFF);
				rectInfo.right   = info->decPicWidth - (val&0xFFFF);
			}
			if (val2 == (Uint32)-1 || val2 == 0)
			{
				rectInfo.top    = 0;
				rectInfo.bottom = info->decPicHeight;
			}
			else
			{
				rectInfo.top     = ((val2>>16) & 0xFFFF);
				rectInfo.bottom	= info->decPicHeight - (val2&0xFFFF);
			}
		}


		info->rcDecoded.left   =  pDecInfo->decOutInfo[info->indexFrameDecoded].rcDecoded.left   = rectInfo.left;
		info->rcDecoded.right  =  pDecInfo->decOutInfo[info->indexFrameDecoded].rcDecoded.right  = rectInfo.right;
		info->rcDecoded.top    =  pDecInfo->decOutInfo[info->indexFrameDecoded].rcDecoded.top    = rectInfo.top;
		info->rcDecoded.bottom =  pDecInfo->decOutInfo[info->indexFrameDecoded].rcDecoded.bottom = rectInfo.bottom;
	}
	else
	{
		info->rcDecoded.left   = 0;
		info->rcDecoded.right  = info->decPicWidth;
		info->rcDecoded.top    = 0;
		info->rcDecoded.bottom = info->decPicHeight;
	}

       if (pCodecInst->codecMode == MP2_DEC)
       {
             val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_MP2_BAR_LEFT_RIGHT);
             val2 = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_MP2_BAR_TOP_BOTTOM);

             info->mp2BardataInfo.barLeft = ((val>>16) & 0xFFFF);
             info->mp2BardataInfo.barRight = (val&0xFFFF);
             info->mp2BardataInfo.barTop = ((val2>>16) & 0xFFFF);
             info->mp2BardataInfo.barBottom = (val2&0xFFFF);
       }

	if (info->indexFrameDisplay >= 0 && info->indexFrameDisplay < MAX_GDI_IDX)
	{
		if (pCodecInst->codecMode == VC1_DEC) // vc1 rotates decoded frame buffer region. the other std rotated whole frame buffer region.
		{
			if (pDecInfo->rotationEnable && (pDecInfo->rotationAngle==90 || pDecInfo->rotationAngle==270))
			{
				info->rcDisplay.left   = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.top;
				info->rcDisplay.right  = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.bottom;
				info->rcDisplay.top    = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.left;
				info->rcDisplay.bottom = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.right;
			}
			else
			{
				info->rcDisplay.left   = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.left;
				info->rcDisplay.right  = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.right;
				info->rcDisplay.top    = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.top;
				info->rcDisplay.bottom = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.bottom;
			}
		}
		else
		{
			if (pDecInfo->rotationEnable)
			{
				switch(pDecInfo->rotationAngle)
				{
				case 90:
				case 270:
					info->rcDisplay.left   = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.top;
					info->rcDisplay.right  = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.bottom;
                    if (pDecInfo->wtlEnable)
                    {
                        info->rcDisplay.top    = pDecInfo->decOutInfo[info->indexFrameDisplay].decPicWidth - pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.right;
                        info->rcDisplay.bottom = pDecInfo->decOutInfo[info->indexFrameDisplay].decPicWidth - pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.left;
                    }
                    else
                    {
					info->rcDisplay.top    = pDecInfo->rotatorOutput.height - pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.right;
					info->rcDisplay.bottom = pDecInfo->rotatorOutput.height - pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.left;
                    }
					break;
				default:
					info->rcDisplay.left   = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.left;
					info->rcDisplay.right  = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.right;
					info->rcDisplay.top    = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.top;
					info->rcDisplay.bottom = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.bottom;
					break;
				}

			}
			else
			{
				info->rcDisplay.left   = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.left;
				info->rcDisplay.right  = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.right;
				info->rcDisplay.top    = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.top;
				info->rcDisplay.bottom = pDecInfo->decOutInfo[info->indexFrameDisplay].rcDecoded.bottom;
			}
		}

        if (info->indexFrameDisplay == info->indexFrameDecoded)
        {
            info->dispPicWidth =  info->decPicWidth;
            info->dispPicHeight = info->decPicHeight;
        }
        else
        {
		    info->dispPicWidth = pDecInfo->decOutInfo[info->indexFrameDisplay].decPicWidth;
		    info->dispPicHeight = pDecInfo->decOutInfo[info->indexFrameDisplay].decPicHeight;
        }
	}
	else
	{
		info->rcDisplay.left   = 0;
		info->rcDisplay.right  = 0;
		info->rcDisplay.top    = 0;
		info->rcDisplay.bottom = 0;

		info->dispPicWidth = 0;
		info->dispPicHeight = 0;
	}

	val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_TYPE);
	info->interlacedFrame	= (val >> 18) & 0x1;
    info->topFieldFirst     = (val >> 21) & 0x0001;	// TopFieldFirst[21]
	if (info->interlacedFrame) {
		info->picTypeFirst      = (val & 0x38) >> 3;	  // pic_type of 1st field
		info->picType           = val & 7;              // pic_type of 2nd field

	}
	else {
		info->picTypeFirst   = PIC_TYPE_MAX;	// no meaning
		info->picType = val & 7;
	}

	info->pictureStructure  = (val >> 19) & 0x0003;	// MbAffFlag[17], FieldPicFlag[16]
	info->repeatFirstField  = (val >> 22) & 0x0001;
	info->progressiveFrame  = (val >> 23) & 0x0003;

	// ADD code default value for NPF info
	info->decFrameInfo = 1;
	info->mp2NpfFieldInfo = 0;

	if( pCodecInst->codecMode == AVC_DEC)
	{
		info->nalRefIdc = (val >> 7) & 0x03;
		info->decFrameInfo = (val >> 15) & 0x0001;
		info->picStrPresent = (val >> 27) & 0x0001;
		info->picTimingStruct = (val >> 28) & 0x000f;
		//update picture type when IDR frame
		if (val & 0x40) { // 6th bit
			if (info->interlacedFrame)
				info->picTypeFirst = PIC_TYPE_IDR;
			else
				info->picType = PIC_TYPE_IDR;

		}
		info->avcNpfFieldInfo  = (val >> 16) & 0x0003;
		val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_HRD_INFO);
		info->avcHrdInfo.cpbMinus1 = val>>2;
		info->avcHrdInfo.vclHrdParamFlag = (val>>1)&1;
		info->avcHrdInfo.nalHrdParamFlag = val&1;

		val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_VUI_INFO);
		info->avcVuiInfo.fixedFrameRateFlag    = val &1;
		info->avcVuiInfo.timingInfoPresent     = (val>>1) & 0x01;
		info->avcVuiInfo.chromaLocBotField     = (val>>2) & 0x07;
		info->avcVuiInfo.chromaLocTopField     = (val>>5) & 0x07;
		info->avcVuiInfo.chromaLocInfoPresent  = (val>>8) & 0x01;
		info->avcVuiInfo.colorPrimaries        = (val>>16) & 0xff;
		info->avcVuiInfo.colorDescPresent      = (val>>24) & 0x01;
		info->avcVuiInfo.isExtSAR              = (val>>25) & 0x01;
		info->avcVuiInfo.vidFullRange          = (val>>26) & 0x01;
		info->avcVuiInfo.vidFormat             = (val>>27) & 0x07;
		info->avcVuiInfo.vidSigTypePresent     = (val>>30) & 0x01;
		info->avcVuiInfo.vuiParamPresent       = (val>>31) & 0x01;
		val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_VUI_PIC_STRUCT);
		info->avcVuiInfo.vuiPicStructPresent = (val & 0x1);
		info->avcVuiInfo.vuiPicStruct = (val>>1);



	}

	if (pCodecInst->codecMode == MP2_DEC)
	{
		info->fieldSequence       = (val >> 25) & 0x0007;
		info->frameDct            = (val >> 28) & 0x0001;
		info->progressiveSequence = (val >> 29) & 0x0001;
		info->mp2NpfFieldInfo = (val >> 16) & 0x0003;
		val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_SEQ_EXT_INFO);
		info->mp2DispVerSize = (val>>1) & 0x3fff;
		info->mp2DispHorSize = (val>>15) & 0x3fff;

	}


	info->fRateNumerator    = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_FRATE_NR); //Frame rate, Aspect ratio can be changed frame by frame.
	info->fRateDenominator  = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_FRATE_DR);
	if (pCodecInst->codecMode == AVC_DEC && info->fRateDenominator > 0)
		info->fRateDenominator  *= 2;
	if (pCodecInst->codecMode == MP4_DEC)
	{
		info->mp4ModuloTimeBase = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_MODULO_TIME_BASE);
		info->mp4TimeIncrement  = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_VOP_TIME_INCREMENT);
	}

	if( pCodecInst->codecMode == VPX_DEC )
		info->aspectRateInfo = 0;
	else
		info->aspectRateInfo = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_ASPECT);


	// User Data
	if (pDecInfo->userDataEnable) {
		int userDataNum;
		int userDataSize;
		BYTE tempBuf[8] = {0,};

		VpuReadMem(pCodecInst->coreIdx, pDecInfo->userDataBufAddr + 0, tempBuf, 8,  VPU_USER_DATA_ENDIAN);

		val =	((tempBuf[0]<<24) & 0xFF000000) |
			((tempBuf[1]<<16) & 0x00FF0000) |
			((tempBuf[2]<< 8) & 0x0000FF00) |
			((tempBuf[3]<< 0) & 0x000000FF);

		userDataNum = (val >> 16) & 0xFFFF;
		userDataSize = (val >> 0) & 0xFFFF;
		if (userDataNum == 0)
			userDataSize = 0;

		info->decOutputExtData.userDataNum = userDataNum;
		info->decOutputExtData.userDataSize = userDataSize;

		val =	((tempBuf[4]<<24) & 0xFF000000) |
			((tempBuf[5]<<16) & 0x00FF0000) |
			((tempBuf[6]<< 8) & 0x0000FF00) |
			((tempBuf[7]<< 0) & 0x000000FF);

		if (userDataNum == 0)
			info->decOutputExtData.userDataBufFull = 0;
		else
			info->decOutputExtData.userDataBufFull = (val >> 16) & 0xFFFF;

		info->decOutputExtData.activeFormat = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_ATSC_USER_DATA_INFO)&0xf;
	}

	info->numOfErrMBs		= VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_ERR_MB);

	val                     = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_SUCCESS);
	info->decodingSuccess	= val;
	info->sequenceChanged = ((val>>20) & 0x1);
	info->streamEndFlag = ((pDecInfo->streamEndflag>>2) & 0x01);



	if (info->indexFrameDisplay >= 0 && info->indexFrameDisplay < pDecInfo->numFrameBuffers) {
		info->dispFrame = pDecInfo->frameBufPool[info->indexFrameDisplay];
		if (pDecInfo->openParam.wtlEnable) {
			info->dispFrame = pDecInfo->frameBufPool[info->indexFrameDisplay + pDecInfo->numFrameBuffers];
            if (pDecInfo->rotationEnable)
            {
                if (pDecInfo->rotationAngle == 90 || pDecInfo->rotationAngle == 270 )
                {
                    info->dispFrame.stride = pDecInfo->rotatorStride;
                    info->dispFrame.height = info->decPicWidth;
                }
            }
		}
	}

    if (!pDecInfo->wtlEnable)
    {
        if (pDecInfo->rotationEnable)
	if (pDecInfo->deringEnable || pDecInfo->mirrorEnable || pDecInfo->rotationEnable || pDecInfo->tiled2LinearEnable) {
		info->dispFrame = pDecInfo->rotatorOutput;
		info->dispFrame.stride = pDecInfo->rotatorStride;
	    }
	}

	if (pCodecInst->codecMode == VC1_DEC && info->indexFrameDisplay != -3) {
		if (pDecInfo->vc1BframeDisplayValid == 0) {
			if (info->picType == 2) {
				info->indexFrameDisplay = -3;
			} else {
				pDecInfo->vc1BframeDisplayValid = 1;
			}
		}
	}

	if (pCodecInst->codecMode == AVC_DEC && pCodecInst->codecModeAux == AVC_AUX_MVC)
	{
		int val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_MVC_REPORT);
		info->mvcPicInfo.viewIdxDisplay = (val>>0) & 1;
		info->mvcPicInfo.viewIdxDecoded = (val>>1) & 1;
	}

	if (pCodecInst->codecMode == AVC_DEC)
	{
		int val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_AVC_FPA_SEI0);

		if (val < 0) {
			info->avcFpaSei.exist = 0;
		}
		else {
			info->avcFpaSei.exist = 1;
			info->avcFpaSei.framePackingArrangementId = val;

			val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_AVC_FPA_SEI1);
			info->avcFpaSei.contentInterpretationType               = val&0x3F; // [5:0]
			info->avcFpaSei.framePackingArrangementType             = (val >> 6)&0x7F; // [12:6]
			info->avcFpaSei.framePackingArrangementExtensionFlag    = (val >> 13)&0x01; // [13]
			info->avcFpaSei.frame1SelfContainedFlag                 = (val >> 14)&0x01; // [14]
			info->avcFpaSei.frame0SelfContainedFlag                 = (val >> 15)&0x01; // [15]
			info->avcFpaSei.currentFrameIsFrame0Flag                = (val >> 16)&0x01; // [16]
			info->avcFpaSei.fieldViewsFlag                          = (val >> 17)&0x01; // [17]
			info->avcFpaSei.frame0FlippedFlag                       = (val >> 18)&0x01; // [18]
			info->avcFpaSei.spatialFlippingFlag                     = (val >> 19)&0x01; // [19]
			info->avcFpaSei.quincunxSamplingFlag                    = (val >> 20)&0x01; // [20]
			info->avcFpaSei.framePackingArrangementCancelFlag       = (val >> 21)&0x01; // [21]

			val = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_AVC_FPA_SEI2);
			info->avcFpaSei.framePackingArrangementRepetitionPeriod = val&0x7FFF;       // [14:0]
			info->avcFpaSei.frame1GridPositionY                     = (val >> 16)&0x0F; // [19:16]
			info->avcFpaSei.frame1GridPositionX                     = (val >> 20)&0x0F; // [23:20]
			info->avcFpaSei.frame0GridPositionY                     = (val >> 24)&0x0F; // [27:24]
			info->avcFpaSei.frame0GridPositionX                     = (val >> 28)&0x0F; // [31:28]
		}


		info->avcPocTop = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_POC_TOP);
		info->avcPocBot = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_POC_BOT);

        if (info->interlacedFrame)
        {
            if (info->avcPocTop > info->avcPocBot) {
                info->avcPocPic = info->avcPocBot;
            }
			else {
                info->avcPocPic = info->avcPocTop;
            }
        }
        else {
            info->avcPocPic = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_POC);
		}
	}

	info->bytePosFrameStart = VpuReadReg(pCodecInst->coreIdx, BIT_BYTE_POS_FRAME_START);
	info->bytePosFrameEnd   = VpuReadReg(pCodecInst->coreIdx, BIT_BYTE_POS_FRAME_END);

    if (info->sequenceChanged != 0) {
        pDecInfo->initialInfo.sequenceNo++;
    }

	if (info->indexFrameDecoded >= 0 && info->indexFrameDecoded < MAX_GDI_IDX)
		pDecInfo->decOutInfo[info->indexFrameDecoded] = *info;

	SetPendingInst(pCodecInst->coreIdx, 0);

	LeaveLock(pCodecInst->coreIdx);


	return RETCODE_SUCCESS;
}

RetCode VPU_DecFrameBufferFlush(DecHandle handle)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	RetCode ret;
	Uint32 val;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	EnterLock(pCodecInst->coreIdx);

	if (GetPendingInst(pCodecInst->coreIdx)) {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	EnterDispFlagLock(pCodecInst->coreIdx);
	val = pDecInfo->frameDisplayFlag;
	val &= ~pDecInfo->clearDisplayIndexes;
	VpuWriteReg(pCodecInst->coreIdx, pDecInfo->frameDisplayFlagRegAddr, val);
	pDecInfo->clearDisplayIndexes = 0;
	LeaveDispFlagLock(pCodecInst->coreIdx);

	BitIssueCommand(pCodecInst->coreIdx, pCodecInst, DEC_BUF_FLUSH);
	if (vdi_wait_vpu_busy(pCodecInst->coreIdx, VPU_BUSY_CHECK_TIMEOUT, BIT_BUSY_FLAG) == -1) {
		if (pCodecInst->loggingEnable)
			vdi_log(pCodecInst->coreIdx, DEC_BUF_FLUSH, 2);
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_VPU_RESPONSE_TIMEOUT;
	}

	pDecInfo->frameDisplayFlag = VpuReadReg(pCodecInst->coreIdx, pDecInfo->frameDisplayFlagRegAddr);

	if (pCodecInst->loggingEnable)
		vdi_log(pCodecInst->coreIdx, DEC_BUF_FLUSH, 0);

	LeaveLock(pCodecInst->coreIdx);
	return RETCODE_SUCCESS;
}



RetCode VPU_DecSetRdPtr(DecHandle handle, PhysicalAddress addr, int updateWrPtr)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	RetCode ret;
	CodecInst * pPendingInst;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	pPendingInst = GetPendingInst(pCodecInst->coreIdx);
	if (pCodecInst == pPendingInst) {
		SetClockGate(pCodecInst->coreIdx, 1);
		VpuWriteReg(pCodecInst->coreIdx, pDecInfo->streamRdPtrRegAddr, addr);
		SetClockGate(pCodecInst->coreIdx, 0);
	}
	else {
		if (pPendingInst) {
		return RETCODE_FRAME_NOT_COMPLETE;
	}
		EnterLock(pCodecInst->coreIdx);
	VpuWriteReg(pCodecInst->coreIdx, pDecInfo->streamRdPtrRegAddr, addr);
	LeaveLock(pCodecInst->coreIdx);
	}

	pDecInfo->streamRdPtr = addr;
	if (updateWrPtr)
		pDecInfo->streamWrPtr = addr;

	return RETCODE_SUCCESS;
}



RetCode VPU_DecClrDispFlag(DecHandle handle, int index)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	RetCode ret;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	if (pDecInfo->stride == 0) { // This means frame buffers have not been registered.
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

	if ((index < 0) || (index > (pDecInfo->numFrameBuffers - 1)))
		return	RETCODE_INVALID_PARAM;

	EnterDispFlagLock(pCodecInst->coreIdx);
	pDecInfo->clearDisplayIndexes |= (1<<index);
	LeaveDispFlagLock(pCodecInst->coreIdx);

	return RETCODE_SUCCESS;
}


RetCode VPU_DecGiveCommand(
	DecHandle handle,
	CodecCommand cmd,
	void * param)
{
	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	RetCode ret;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;


	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;
	switch (cmd)
	{
	case ENABLE_ROTATION :
		{
			if (pDecInfo->rotatorStride == 0) {
				return RETCODE_ROTATOR_STRIDE_NOT_SET;
			}
			pDecInfo->rotationEnable = 1;
            if (pDecInfo->wtlEnable)
                pDecInfo->rotatorOutputValid = 1; // use wtl linear frame buffer for rotate output
			break;
		}

	case DISABLE_ROTATION :
		{
			pDecInfo->rotationEnable = 0;
			break;
		}

	case ENABLE_MIRRORING :
		{
			if (pDecInfo->rotatorStride == 0) {
				return RETCODE_ROTATOR_STRIDE_NOT_SET;
			}
			pDecInfo->mirrorEnable = 1;
			break;
		}
	case DISABLE_MIRRORING :
		{
			pDecInfo->mirrorEnable = 0;
			break;
		}
	case SET_MIRROR_DIRECTION :
		{

			MirrorDirection mirDir;

			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}
			mirDir = *(MirrorDirection *)param;
			if (!(MIRDIR_NONE <= (int)mirDir && MIRDIR_HOR_VER >= (int)mirDir)) {
				return RETCODE_INVALID_PARAM;
			}
			pDecInfo->mirrorDirection = mirDir;

			break;
		}
	case SET_ROTATION_ANGLE :
		{
			int angle;

			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}
			angle = *(int *)param;
			if (angle != 0 && angle != 90 &&
				angle != 180 && angle != 270) {
					return RETCODE_INVALID_PARAM;
			}
			if (pDecInfo->rotatorStride != 0) {
				if (angle == 90 || angle ==270) {
					if (pDecInfo->initialInfo.picHeight > pDecInfo->rotatorStride) {
						return RETCODE_INVALID_PARAM;
					}
				} else {
					if (pDecInfo->initialInfo.picWidth > pDecInfo->rotatorStride) {
						return RETCODE_INVALID_PARAM;
					}
				}
			}

			pDecInfo->rotationAngle = angle;
			break;
		}
	case SET_ROTATOR_OUTPUT :
		{
#ifdef SUPPORT_MEM_PROTECT
			PhysicalAddress start, end, ppuAddr;
#endif
			FrameBuffer *frame;
			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}
#ifdef SUPPORT_MEM_PROTECT
			if (pDecInfo->rotatorStride == 0) {
				return RETCODE_ROTATOR_STRIDE_NOT_SET;
			}
#endif
			frame = (FrameBuffer *)param;

			pDecInfo->rotatorOutput = *frame;
			pDecInfo->rotatorOutputValid = 1;
#ifdef SUPPORT_MEM_PROTECT
			start   = pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].startAddress;
			end     = pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].endAddress;
			ppuAddr = GetXY2AXIAddr(&pDecInfo->mapCfg, 0, 0, 0, pDecInfo->rotatorStride, &pDecInfo->rotatorOutput);
			start = (start< ppuAddr) ? start : ppuAddr;
			ppuAddr +=  VPU_GetFrameBufSize(pCodecInst->coreIdx, pDecInfo->rotatorStride, pDecInfo->rotatorOutput.height, pDecInfo->rotatorOutput.mapType, FORMAT_420, pDecInfo->openParam.cbcrInterleave, &pDecInfo->dramCfg);
			end   = (end  > ppuAddr) ? end : ppuAddr;

			pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].startAddress = start;
			pDecInfo->writeMemProtectCfg.decRegion[WPROT_DEC_FRAME].endAddress   = end;
#endif
			break;
		}

	case SET_ROTATOR_STRIDE :
		{
			int stride;

			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}
			stride = *(int *)param;
			if (stride % 8 != 0 || stride == 0) {
				return RETCODE_INVALID_STRIDE;
			}

			if (pDecInfo->rotationAngle == 90 || pDecInfo->rotationAngle == 270) {
				if (pDecInfo->initialInfo.picHeight > stride) {
					return RETCODE_INVALID_STRIDE;
				}
			} else {
				if (pDecInfo->initialInfo.picWidth > stride) {
					return RETCODE_INVALID_STRIDE;
				}
			}

			pDecInfo->rotatorStride = stride;
			break;
		}
	case DEC_SET_SPS_RBSP:
		{
			if (pCodecInst->codecMode != AVC_DEC) {
				return RETCODE_INVALID_COMMAND;
			}
			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}

			return SetParaSet(handle, 0, (DecParamSet *)param);

		}

	case DEC_SET_PPS_RBSP:
		{
			if (pCodecInst->codecMode != AVC_DEC) {
				return RETCODE_INVALID_COMMAND;
			}
			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}

			return SetParaSet(handle, 1, (DecParamSet *)param);
		}
	case ENABLE_DERING :
		{
			if (pDecInfo->rotatorStride == 0) {
				return RETCODE_ROTATOR_STRIDE_NOT_SET;
			}
			pDecInfo->deringEnable = 1;
			break;
		}

	case DISABLE_DERING :
		{
			pDecInfo->deringEnable = 0;
			break;
		}
	case SET_SEC_AXI:
		{
			SecAxiUse secAxiUse;

			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}
			secAxiUse = *(SecAxiUse *)param;
			pDecInfo->secAxiInfo.useBitEnable = secAxiUse.useBitEnable;
			pDecInfo->secAxiInfo.useIpEnable = secAxiUse.useIpEnable;
			pDecInfo->secAxiInfo.useDbkYEnable = secAxiUse.useDbkYEnable;
			pDecInfo->secAxiInfo.useDbkCEnable = secAxiUse.useDbkCEnable;
			pDecInfo->secAxiInfo.useOvlEnable = secAxiUse.useOvlEnable;
			pDecInfo->secAxiInfo.useBtpEnable = secAxiUse.useBtpEnable;

			break;
		}
	case ENABLE_REP_USERDATA:
		{
#ifdef SUPPORT_MEM_PROTECT
			WriteMemProtectCfg *pCgf    = &pDecInfo->writeMemProtectCfg;
#endif
			if (!pDecInfo->userDataBufAddr) {
				return RETCODE_USERDATA_BUF_NOT_SET;
			}
			if (pDecInfo->userDataBufSize == 0) {
				return RETCODE_USERDATA_BUF_NOT_SET;
			}
			pDecInfo->userDataEnable = 1;
#ifdef SUPPORT_MEM_PROTECT
			pCgf->decRegion[WPROT_DEC_REPORT].enable = 1;
			pCgf->decRegion[WPROT_DEC_REPORT].isSecondary = 0;
			pCgf->decRegion[WPROT_DEC_REPORT].startAddress = pDecInfo->userDataBufAddr;
			pCgf->decRegion[WPROT_DEC_REPORT].endAddress = pDecInfo->userDataBufAddr + pDecInfo->userDataBufSize;
#endif
			break;
		}
	case DISABLE_REP_USERDATA:
		{
			pDecInfo->userDataEnable = 0;
			break;
		}
	case SET_ADDR_REP_USERDATA:
		{
			PhysicalAddress userDataBufAddr;

			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}
			userDataBufAddr = *(PhysicalAddress *)param;
			if (userDataBufAddr % 8 != 0 || userDataBufAddr == 0) {
				return RETCODE_INVALID_PARAM;
			}

			pDecInfo->userDataBufAddr = userDataBufAddr;
			break;
		}
	case SET_VIRT_ADDR_REP_USERDATA:
		{
/*			unsigned long userDataVirtAddr;

			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}

			if (!pDecInfo->userDataBufAddr) {
				return RETCODE_USERDATA_BUF_NOT_SET;
			}
			if (pDecInfo->userDataBufSize == 0) {
				return RETCODE_USERDATA_BUF_NOT_SET;
			}

			userDataVirtAddr = *(unsigned long *)param;
			if (!userDataVirtAddr) {
				return RETCODE_INVALID_PARAM;
			}


			pDecInfo->vbUserData.phys_addr = pDecInfo->userDataBufAddr;
			pDecInfo->vbUserData.size = pDecInfo->userDataBufSize;
			pDecInfo->vbUserData.virt_addr = (unsigned long)userDataVirtAddr;
			if (vdi_attach_dma_memory(pCodecInst->coreIdx, &pDecInfo->vbUserData) != 0) {
				return RETCODE_INSUFFICIENT_RESOURCE;
			}
*/			break;
		}
	case SET_SIZE_REP_USERDATA:
		{
			PhysicalAddress userDataBufSize;

			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}
			userDataBufSize = *(PhysicalAddress *)param;

			pDecInfo->userDataBufSize = userDataBufSize;
			break;
		}

	case SET_USERDATA_REPORT_MODE:
		{
			int userDataMode;

			userDataMode = *(int *)param;
			if (userDataMode != 1 && userDataMode != 0) {
				return RETCODE_INVALID_PARAM;
			}
			pDecInfo->userDataReportMode = userDataMode;
			break;
		}
	case SET_CACHE_CONFIG:
		{
			MaverickCacheConfig *mcCacheConfig;
			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}
			mcCacheConfig = (MaverickCacheConfig *)param;
			pDecInfo->cacheConfig.CacheMode = mcCacheConfig->CacheMode;
			break;
		}
	case SET_LOW_DELAY_CONFIG:
		{
			LowDelayInfo *lowDelayInfo;
			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}
			lowDelayInfo = (LowDelayInfo *)param;

			if (lowDelayInfo->lowDelayEn) {
				if (pCodecInst->codecMode != AVC_DEC ||
					pDecInfo->rotationEnable ||
					pDecInfo->mirrorEnable ||
					pDecInfo->tiled2LinearEnable ||
					pDecInfo->deringEnable) {
					return RETCODE_INVALID_PARAM;
				}
			}

			pDecInfo->lowDelayInfo.lowDelayEn = lowDelayInfo->lowDelayEn;
			pDecInfo->lowDelayInfo.numRows = lowDelayInfo->numRows;
			if (lowDelayInfo->lowDelayEn)
				pDecInfo->reorderEnable = 0;
			break;
		}

	case SET_DECODE_FLUSH:	// interrupt mode to pic_end
		{

			Uint32 val;

			if (pDecInfo->openParam.bitstreamMode != BS_MODE_INTERRUPT){
				return RETCODE_INVALID_COMMAND;
			}

			val = VpuReadReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM);
			val &= ~(3<<3);
			val |= (2<<3);	// set to pic_end mode
			VpuWriteReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM, val);
			break;
		}

	case DEC_SET_FRAME_DELAY:
		{
			pDecInfo->frameDelay = *(int *)param;
			break;
		}
	case DEC_ENABLE_REORDER:
		{
			if (pDecInfo->initialInfoObtained) {
				return RETCODE_WRONG_CALL_SEQUENCE;
			}
			pDecInfo->reorderEnable = 1;
			break;
		}
	case DEC_DISABLE_REORDER:
		{
			if (pDecInfo->initialInfoObtained) {
				return RETCODE_WRONG_CALL_SEQUENCE;
			}
			pDecInfo->reorderEnable = 0;
			break;
		}
	case DEC_SET_AVC_ERROR_CONCEAL_MODE:
		{
			if(pCodecInst->codecMode != AVC_DEC) {
				return RETCODE_INVALID_COMMAND;
			}

			if (pDecInfo->initialInfoObtained) {
				return RETCODE_WRONG_CALL_SEQUENCE;
			}

			pDecInfo->avcErrorConcealMode = *(AVCErrorConcealMode *)param;
			break;
		}
	case DEC_FREE_FRAME_BUFFER:
		{
			if (pDecInfo->vbSlice.size)
				vdi_free_dma_memory(pCodecInst->coreIdx, &pDecInfo->vbSlice);

			if (pDecInfo->vbFrame.size){
				if (pDecInfo->frameAllocExt == 0) {
					vdi_free_dma_memory(pCodecInst->coreIdx, &pDecInfo->vbFrame);
				}
			}


			if (pDecInfo->vbPPU.size) {
				if (pDecInfo->ppuAllocExt == 0) {
					vdi_free_dma_memory(pCodecInst->coreIdx, &pDecInfo->vbPPU);
				}
			}
			if (pDecInfo->wtlEnable)	//coda980 only
			{
				if (pDecInfo->vbWTL.size)
					vdi_free_dma_memory(pCodecInst->coreIdx, &pDecInfo->vbWTL);
			}
			break;
		}
	case ENABLE_DEC_THUMBNAIL_MODE:
		{
			if (pDecInfo->stride) {
				return RETCODE_WRONG_CALL_SEQUENCE;
			}
			pDecInfo->thumbnailMode = 1;
			break;
		}
	case DEC_GET_FIELD_PIC_TYPE:
		{
			SetClockGate(pCodecInst->coreIdx, 1);
			*((int *)param)  = VpuReadReg(pCodecInst->coreIdx, RET_DEC_PIC_TYPE) & 0x1f;
			SetClockGate(pCodecInst->coreIdx, 0);
			break;
		}
	case DEC_GET_DISPLAY_OUTPUT_INFO:
		{
			DecOutputInfo *pDecOutInfo = (DecOutputInfo *)param;
			*pDecOutInfo = pDecInfo->decOutInfo[pDecOutInfo->indexFrameDisplay];
			break;
		}
	case GET_TILEDMAP_CONFIG:
		{
			TiledMapConfig *pMapCfg = (TiledMapConfig *)param;
			if (!pMapCfg) {
				return RETCODE_INVALID_PARAM;
			}
			if (!pDecInfo->stride) {
				return RETCODE_WRONG_CALL_SEQUENCE;

			}
			*pMapCfg = pDecInfo->mapCfg;
			break;
		}
	case SET_DRAM_CONFIG:
		{
			DRAMConfig *cfg = (DRAMConfig *)param;

			if (!cfg) {
				return RETCODE_INVALID_PARAM;
			}

			pDecInfo->dramCfg = *cfg;
			break;
		}
	case GET_DRAM_CONFIG:
		{
			DRAMConfig *cfg = (DRAMConfig *)param;

			if (!cfg) {
				return RETCODE_INVALID_PARAM;
			}

			*cfg = pDecInfo->dramCfg;

			break;
		}
	case GET_LOW_DELAY_OUTPUT:
		{
			DecOutputInfo *lowDelayOutput;
			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}

			if (!pDecInfo->lowDelayInfo.lowDelayEn || pCodecInst->codecMode != AVC_DEC) {
				return RETCODE_INVALID_COMMAND;
			}

			if (pCodecInst != GetPendingInst(pCodecInst->coreIdx)) {
				return RETCODE_WRONG_CALL_SEQUENCE;
			}

			lowDelayOutput = (DecOutputInfo *)param;

			GetLowDelayOutput(pCodecInst, lowDelayOutput);

			break;
		}
	case ENABLE_LOGGING:
		{
			pCodecInst->loggingEnable = 1;
		}
		break;
	case DISABLE_LOGGING:
		{
			pCodecInst->loggingEnable = 0;
		}
		break;
	case DEC_SET_DISPLAY_FLAG:
		{
			int index;

			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}
			index = *(int *) param;

			pDecInfo->frameDisplayFlag |= (1 << index);
		}
		break;
    case DEC_SET_EXT_SLICE_BUFF:
        {
            vpu_buffer_t *pvbSlice = (vpu_buffer_t *)param;
            if (param == 0)
                return RETCODE_INVALID_PARAM;
/* @ modified
            if (pCodecInst->codecMode == VPX_DEC) {
                if (pvbSlice->size != VP8_MB_SAVE_SIZE) {
                    return RETCODE_INVALID_PARAM;
                }
            }
            else if (pCodecInst->codecMode == AVC_DEC) {
                if (pvbSlice->size != SLICE_SAVE_SIZE) {
                    return RETCODE_INVALID_PARAM;
                }
            }
            else
                return RETCODE_INVALID_PARAM;
*/
            pDecInfo->vbSlice.size = pvbSlice->size;
            pDecInfo->vbSlice.virt_addr = (unsigned long*)pvbSlice->base;
            pDecInfo->vbSlice.phys_addr = pvbSlice->phys_addr;
		}
		break;
	case DEC_GET_SEQ_INFO:
	{
		DecInitialInfo* seqInfo = (DecInitialInfo*)param;
		if (!seqInfo) {
			return RETCODE_INVALID_PARAM;
		}

		if (!pDecInfo->initialInfoObtained) {
			return RETCODE_WRONG_CALL_SEQUENCE;
		}

		*seqInfo = pDecInfo->initialInfo;
	}
		break;
	default:
		return RETCODE_INVALID_COMMAND;
	}
	return RETCODE_SUCCESS;
}

RetCode VPU_DecAllocateFrameBuffer(DecHandle handle, FrameBufferAllocInfo info, FrameBuffer *frameBuffer)
{

	CodecInst * pCodecInst;
	DecInfo * pDecInfo;
	RetCode ret;
	int gdiIndex;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pDecInfo = &pCodecInst->CodecInfo.decInfo;

	if (!frameBuffer) {
		return RETCODE_INVALID_PARAM;
	}

	if (info.type == FB_TYPE_PPU) {
		if (pDecInfo->numFrameBuffers == 0) {
			return RETCODE_WRONG_CALL_SEQUENCE;
		}
		pDecInfo->ppuAllocExt = 0;
		if (frameBuffer[0].bufCb == (PhysicalAddress)-1 && frameBuffer[0].bufCr == (PhysicalAddress)-1) {
			pDecInfo->ppuAllocExt = 1;
		}
		gdiIndex = pDecInfo->numFrameBuffers;
		if (pCodecInst->productId == PRODUCT_ID_980) {
			ret = AllocateFrameBufferArrayV20(pCodecInst->coreIdx, frameBuffer, &pDecInfo->vbPPU, info.mapType, info.cbcrInterleave,
				    info.format, info.num, info.stride, info.height, info.endian, gdiIndex, info.type);
		}
		else {
			ret = AllocateFrameBufferArrayV10(pCodecInst->coreIdx, frameBuffer, &pDecInfo->vbPPU, info.mapType, info.cbcrInterleave,
				    info.format, info.num, info.stride, info.height, gdiIndex, info.type, pDecInfo->mapCfg.tiledBaseAddr, &pDecInfo->dramCfg);
		}
	}
	else if (info.type == FB_TYPE_CODEC) {
		gdiIndex = 0;
		pDecInfo->frameAllocExt = 0;
		if (frameBuffer[0].bufCb == (PhysicalAddress)-1 && frameBuffer[0].bufCr == (PhysicalAddress)-1) {
			pDecInfo->frameAllocExt = 1;
		}
		else {
			frameBuffer[0].bufMvCol = (PhysicalAddress)-1;
		}
		if (pCodecInst->productId == PRODUCT_ID_980) {
			ret = AllocateFrameBufferArrayV20(pCodecInst->coreIdx, frameBuffer, &pDecInfo->vbFrame, info.mapType, info.cbcrInterleave,
				        info.format, info.num, info.stride, info.height, info.endian, gdiIndex, info.type);
		}
		else {
			ret = AllocateFrameBufferArrayV10(pCodecInst->coreIdx, frameBuffer, &pDecInfo->vbFrame, info.mapType, info.cbcrInterleave,
				        info.format, info.num, info.stride, info.height, gdiIndex, info.type, 0, &pDecInfo->dramCfg);
			pDecInfo->mapCfg.tiledBaseAddr = pDecInfo->vbFrame.phys_addr;
		}
	}
	return ret;
}





RetCode VPU_EncOpen(EncHandle * pHandle, EncOpenParam * pop)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	RetCode ret;
	int i;


	ret = CheckEncOpenParam(pop);
	if (ret != RETCODE_SUCCESS) {
		return ret;
	}

	EnterLock(pop->coreIdx);

	if (VpuReadReg(pop->coreIdx, BIT_CUR_PC) == 0) {
		LeaveLock(pop->coreIdx);
		return RETCODE_NOT_INITIALIZED;
	}


	ret = GetCodecInstance(pop->coreIdx, &pCodecInst);
	if (ret != RETCODE_SUCCESS) {
		*pHandle = 0;
		LeaveLock(pop->coreIdx);
		return ret;
	}

	*pHandle = pCodecInst;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;

	pEncInfo->openParam = *pop;


	if( pop->bitstreamFormat == STD_MPEG4 || pop->bitstreamFormat == STD_H263 )
		pCodecInst->codecMode = MP4_ENC;
	else if( pop->bitstreamFormat == STD_AVC )
		pCodecInst->codecMode = AVC_ENC;

	{
		pCodecInst->codecModeAux = 0;
	}

	pEncInfo->streamRdPtr = pop->bitstreamBuffer;
	pEncInfo->streamWrPtr = pop->bitstreamBuffer;
	pEncInfo->streamRdPtrRegAddr = BIT_RD_PTR;
	pEncInfo->streamWrPtrRegAddr = BIT_WR_PTR;
	pEncInfo->streamBufStartAddr = pop->bitstreamBuffer;
	pEncInfo->streamBufSize = pop->bitstreamBufferSize;
	pEncInfo->streamBufEndAddr = pop->bitstreamBuffer + pop->bitstreamBufferSize;
	pEncInfo->stride = 0;
	pEncInfo->vbFrame.size = 0;
	pEncInfo->vbPPU.size = 0;
	pEncInfo->frameAllocExt = 0;
	pEncInfo->ppuAllocExt = 0;
	pEncInfo->secAxiInfo.useBitEnable = 0;
	pEncInfo->secAxiInfo.useIpEnable = 0;
	pEncInfo->secAxiInfo.useDbkYEnable = 0;
	pEncInfo->secAxiInfo.useDbkCEnable = 0;
	pEncInfo->secAxiInfo.useOvlEnable = 0;

	pEncInfo->rotationEnable = 0;
	pEncInfo->mirrorEnable = 0;
	pEncInfo->mirrorDirection = MIRDIR_NONE;
	pEncInfo->rotationAngle = 0;
	pEncInfo->initialInfoObtained = 0;
	pEncInfo->ringBufferEnable = pop->ringBufferEnable;
	pEncInfo->linear2TiledEnable = pop->linear2TiledEnable;
		pEncInfo->linear2TiledMode = pop->linear2TiledMode;	// coda980 only
	if (!pEncInfo->linear2TiledEnable)
		pEncInfo->linear2TiledMode = 0;

	// coda960 only
	pEncInfo->dramCfg.rasBit = EM_RAS;
	pEncInfo->dramCfg.casBit = EM_CAS;
	pEncInfo->dramCfg.bankBit = EM_BANK;
	pEncInfo->dramCfg.busBit = EM_WIDTH;

	pEncInfo->vbScratch.base    = pEncInfo->vbScratch.virt_addr = pEncInfo->vbScratch.size = 0;

	if (pop->vbWork.size) {
	    pEncInfo->vbWork = pop->vbWork;
	    pEncInfo->workBufferAllocExt = 1;
	}
	else {
		pEncInfo->vbWork.size       = WORK_BUF_SIZE;
		if (vdi_allocate_dma_memory(pCodecInst->coreIdx, &pEncInfo->vbWork) < 0)
		{
			*pHandle = 0;
			LeaveLock(pCodecInst->coreIdx);
			return RETCODE_INSUFFICIENT_RESOURCE;
		}
		pop->vbWork = pEncInfo->vbWork;
		pEncInfo->workBufferAllocExt = 0;
	}

	pEncInfo->ActivePPSIdx = 0;
	pEncInfo->frameIdx     = 0;
	pEncInfo->fieldDone    = 0;

	for (i=0; i<6; i++)
		pEncInfo->writeMemProtectCfg.encRegion[i].enable = 0;


	pEncInfo->lineBufIntEn = pop->lineBufIntEn;


	LeaveLock(pCodecInst->coreIdx);
	return RETCODE_SUCCESS;
}

RetCode VPU_EncClose(EncHandle handle)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	RetCode ret;

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;

	EnterLock(pCodecInst->coreIdx);

	if (pEncInfo->initialInfoObtained) {

		VpuWriteReg(pCodecInst->coreIdx, pEncInfo->streamWrPtrRegAddr, pEncInfo->streamWrPtr);
		VpuWriteReg(pCodecInst->coreIdx, pEncInfo->streamRdPtrRegAddr, pEncInfo->streamRdPtr);

		BitIssueCommand(pCodecInst->coreIdx, pCodecInst, SEQ_END);
		if (vdi_wait_vpu_busy(pCodecInst->coreIdx, VPU_BUSY_CHECK_TIMEOUT, BIT_BUSY_FLAG) == -1) {
			if (pCodecInst->loggingEnable)
				vdi_log(pCodecInst->coreIdx, SEQ_END, 2);
			LeaveLock(pCodecInst->coreIdx);
			return RETCODE_VPU_RESPONSE_TIMEOUT;
		}
		if (pCodecInst->loggingEnable)
			vdi_log(pCodecInst->coreIdx, SEQ_END, 0);
		pEncInfo->streamWrPtr = VpuReadReg(pCodecInst->coreIdx, pEncInfo->streamWrPtrRegAddr);
	}

/* @ modified

	if (pEncInfo->vbScratch.size) {
	   if (pEncInfo->scratchAllocExt == 0) {
		   vdi_free_dma_memory(pCodecInst->coreIdx, &pEncInfo->vbScratch);
	   }
	   else {
		   vdi_dettach_dma_memory(pCodecInst->coreIdx, &pEncInfo->vbScratch);
	   }
	}
	if (pEncInfo->vbWork.size) {
	   if (pEncInfo->workBufferAllocExt == 0) {
		   vdi_free_dma_memory(pCodecInst->coreIdx, &pEncInfo->vbWork);
	   }
	   else {
		   vdi_dettach_dma_memory(pCodecInst->coreIdx, &pEncInfo->vbWork);
	   }
	}
	if (pEncInfo->vbFrame.size) {
	   if (pEncInfo->frameAllocExt == 0) {
		   vdi_free_dma_memory(pCodecInst->coreIdx, &pEncInfo->vbFrame);
	   }
	}

	if (pEncInfo->vbPPU.size) {
	   if (pEncInfo->ppuAllocExt == 0) {
		   vdi_free_dma_memory(pCodecInst->coreIdx, &pEncInfo->vbPPU);
	   }
	}
*/

	LeaveLock(pCodecInst->coreIdx);

	FreeCodecInstance(pCodecInst);

	return RETCODE_SUCCESS;
}


RetCode VPU_EncGetInitialInfo(EncHandle handle, EncInitialInfo * info)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	int picWidth;
	int picHeight;
	Uint32	data;
	RetCode ret;
	Uint32 val;

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if (info == 0) {
		return RETCODE_INVALID_PARAM;
	}

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;

	if (pEncInfo->initialInfoObtained) {
		return RETCODE_CALLED_BEFORE;
	}

	EnterLock(pCodecInst->coreIdx);

	if (GetPendingInst(pCodecInst->coreIdx)) {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_FRAME_NOT_COMPLETE;
	}


	picWidth = pEncInfo->openParam.picWidth;
	picHeight = pEncInfo->openParam.picHeight;
	VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_BB_START, pEncInfo->streamBufStartAddr);
	VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_BB_SIZE, pEncInfo->streamBufSize / 1024); // size in KB


	// Rotation Left 90 or 270 case : Swap XY resolution for VPU internal usage
	if (pEncInfo->rotationAngle == 90 || pEncInfo->rotationAngle == 270)
		data = (picHeight<< 16) | picWidth;
	else
		data = (picWidth << 16) | picHeight;

	VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_SRC_SIZE, data);
	VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_SRC_F_RATE, pEncInfo->openParam.frameRateInfo);

	if (pEncInfo->openParam.bitstreamFormat == STD_MPEG4) {
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_COD_STD, 3);
		data = pEncInfo->openParam.EncStdParam.mp4Param.mp4IntraDcVlcThr << 2 |
			pEncInfo->openParam.EncStdParam.mp4Param.mp4ReversibleVlcEnable << 1 |
			pEncInfo->openParam.EncStdParam.mp4Param.mp4DataPartitionEnable;

		data |= ((pEncInfo->openParam.EncStdParam.mp4Param.mp4HecEnable >0)? 1:0)<<5;
		data |= ((pEncInfo->openParam.EncStdParam.mp4Param.mp4Verid == 2)? 0:1) << 6;

		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_MP4_PARA, data);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_ME_OPTION, (pEncInfo->openParam.meBlkMode << 5) | (pEncInfo->openParam.MEUseZeroPmv << 4) | (pEncInfo->openParam.MESearchRangeY<<2) | pEncInfo->openParam.MESearchRangeX);
	}
	else if (pEncInfo->openParam.bitstreamFormat == STD_H263) {
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_COD_STD, 11);
		data = pEncInfo->openParam.EncStdParam.h263Param.h263AnnexIEnable << 3 |
			pEncInfo->openParam.EncStdParam.h263Param.h263AnnexJEnable << 2 |
			pEncInfo->openParam.EncStdParam.h263Param.h263AnnexKEnable << 1|
			pEncInfo->openParam.EncStdParam.h263Param.h263AnnexTEnable;
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_263_PARA, data);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_ME_OPTION, (pEncInfo->openParam.meBlkMode << 5) | (pEncInfo->openParam.MEUseZeroPmv << 4) | (pEncInfo->openParam.MESearchRangeY<<2) | pEncInfo->openParam.MESearchRangeX);
	}
	else if (pEncInfo->openParam.bitstreamFormat == STD_AVC) {
		int SliceNum = 0;
		AvcPpsParam *ActivePPS = &pEncInfo->openParam.EncStdParam.avcParam.ppsParam[pEncInfo->ActivePPSIdx];
		if (ActivePPS->transform8x8Mode == 1 || pEncInfo->openParam.EncStdParam.avcParam.chromaFormat400 == 1)
			pEncInfo->openParam.EncStdParam.avcParam.profile = 2;
		else if(ActivePPS->entropyCodingMode != 0 || pEncInfo->openParam.EncStdParam.avcParam.fieldFlag == 1)
			pEncInfo->openParam.EncStdParam.avcParam.profile = 1;
		else
			pEncInfo->openParam.EncStdParam.avcParam.profile = 0;

		if (pEncInfo->openParam.sliceMode.sliceMode==1 && pEncInfo->openParam.sliceMode.sliceSizeMode==1)
			SliceNum = pEncInfo->openParam.sliceMode.sliceSize;

		if (!pEncInfo->openParam.EncStdParam.avcParam.level)
		{
			if (pEncInfo->openParam.EncStdParam.avcParam.fieldFlag)
				pEncInfo->openParam.EncStdParam.avcParam.level = LevelCalculation(picWidth/16, (picHeight+31)/32, pEncInfo->openParam.frameRateInfo, 1, pEncInfo->openParam.bitRate, SliceNum);
			else
				pEncInfo->openParam.EncStdParam.avcParam.level = LevelCalculation(picWidth/16, picHeight/16, pEncInfo->openParam.frameRateInfo, 0, pEncInfo->openParam.bitRate, SliceNum);
			if (pEncInfo->openParam.EncStdParam.avcParam.level<0)
				return RETCODE_INVALID_PARAM;
		}

		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_COD_STD, (pEncInfo->openParam.EncStdParam.avcParam.profile<<4) | 0x0);
		data = (pEncInfo->openParam.EncStdParam.avcParam.deblkFilterOffsetBeta & 15) << 12 |
			(pEncInfo->openParam.EncStdParam.avcParam.deblkFilterOffsetAlpha & 15) << 8 |
			pEncInfo->openParam.EncStdParam.avcParam.disableDeblk << 6 |
			pEncInfo->openParam.EncStdParam.avcParam.constrainedIntraPredFlag << 5 |
			(pEncInfo->openParam.EncStdParam.avcParam.chromaQpOffset & 31);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_264_PARA, data);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_ME_OPTION, (pEncInfo->openParam.meBlkMode << 5) | (pEncInfo->openParam.MEUseZeroPmv << 4) | (pEncInfo->openParam.MESearchRangeY<<2) | pEncInfo->openParam.MESearchRangeX);
	}

	data = 0;
	if (pEncInfo->openParam.sliceMode.sliceMode!=0) {
		data = pEncInfo->openParam.sliceMode.sliceSize << 2 |
			(pEncInfo->openParam.sliceMode.sliceSizeMode+1); // encoding mode 0,1,2
	}
	VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_SLICE_MODE, data);

	if (pEncInfo->openParam.rcEnable) { // rate control enabled

		if (pEncInfo->openParam.bitstreamFormat == STD_AVC)
		{
			int MinDeltaQp, MaxDeltaQp, QpMin, QpMax;

            data = (pEncInfo->openParam.idrInterval << 21) | (pEncInfo->openParam.rcGopIQpOffsetEn<<20) | ((pEncInfo->openParam.rcGopIQpOffset & 0xF)<<16) | pEncInfo->openParam.gopSize ;
            VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_GOP_NUM, data);

			data = (!pEncInfo->openParam.enableAutoSkip) << 31 |
				pEncInfo->openParam.initialDelay << 16 | 0;
			VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_RC_PARA, data);
#ifdef STRICT_CBR
			data = (pEncInfo->openParam.StrictCbr <<22) |(pEncInfo->openParam.bitRate<<4)|(pEncInfo->openParam.rcEnable&0xf);
#else
            data = (pEncInfo->openParam.bitRate<<4)|(pEncInfo->openParam.rcEnable&0xf);
#endif
			VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_RC_PARA2, data);

			data = 0;
			if (pEncInfo->openParam.maxIntraSize>0)
				data = (1<<16) | (pEncInfo->openParam.maxIntraSize & 0xFFFF);
			VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_RC_MAX_INTRA_SIZE, data);


			if (pEncInfo->openParam.userMinDeltaQp<0)	MinDeltaQp = 0;
			else										MinDeltaQp = (1<<6)|pEncInfo->openParam.userMinDeltaQp;

			if  (pEncInfo->openParam.userMaxDeltaQp<0)  MaxDeltaQp = 0;
			else										MaxDeltaQp = (1<<6)|pEncInfo->openParam.userMaxDeltaQp;

			if  (pEncInfo->openParam.userQpMin<0)       QpMin = 0;
			else										QpMin = (1<<6)|pEncInfo->openParam.userQpMin;

			if  (pEncInfo->openParam.userQpMax<0)       QpMax = 0;
			else										QpMax = (1<<6)|pEncInfo->openParam.userQpMax;

			data = MinDeltaQp<<24 | MaxDeltaQp<<16 | QpMin<<8 | QpMax;
			VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_QP_RANGE_SET, data);
		}
		else // MP4
		{
            VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_GOP_NUM, pEncInfo->openParam.gopSize);

			data = (!pEncInfo->openParam.enableAutoSkip) << 31 |
				pEncInfo->openParam.initialDelay << 16 |
				pEncInfo->openParam.bitRate<<1 | 1;
			VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_RC_PARA, data);
			VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_QP_RANGE_SET, 0);
			VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_RC_PARA2, 0);
		}
	}
	else {
        if (pEncInfo->openParam.bitstreamFormat == STD_AVC)
            VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_GOP_NUM,  (pEncInfo->openParam.idrInterval << 21) | pEncInfo->openParam.gopSize);
        else
        VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_GOP_NUM, pEncInfo->openParam.gopSize);

		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_RC_PARA, 0);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_QP_RANGE_SET, 0);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_RC_PARA2, 0);
	}

	VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_RC_BUF_SIZE, pEncInfo->openParam.vbvBufferSize);

	data = pEncInfo->openParam.intraRefresh | pEncInfo->openParam.ConscIntraRefreshEnable<<16;
	VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_INTRA_REFRESH, data);

	data = 0;
	if(pEncInfo->openParam.rcIntraQp>=0) {
		data = (1 << 5);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_INTRA_QP, pEncInfo->openParam.rcIntraQp);
	}
	else {
		data = 0;
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_INTRA_QP, (Uint32)-1);
	}

	if (pCodecInst->codecMode == AVC_ENC) {
		data |= (pEncInfo->openParam.EncStdParam.avcParam.audEnable << 2);

		data |= pEncInfo->openParam.EncStdParam.avcParam.fieldFlag << 10;
		data |= pEncInfo->openParam.EncStdParam.avcParam.fieldRefMode << 11;
	}


	if(pEncInfo->openParam.userQpMax>=0) {
		data |= (1<<6);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_RC_QP_MAX, pEncInfo->openParam.userQpMax);
	}
	else {
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_RC_QP_MAX, 0);
	}

	if(pEncInfo->openParam.userGamma >= 0) {
		data |= (1<<7);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_RC_GAMMA, pEncInfo->openParam.userGamma);
	}
	else {
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_RC_GAMMA, 0);
	}

	VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_OPTION, data);

	VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_RC_INTERVAL_MODE, (pEncInfo->openParam.mbInterval<<2) | pEncInfo->openParam.rcIntervalMode);
	VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_INTRA_WEIGHT, pEncInfo->openParam.intraCostWeight);

	VpuWriteReg(pCodecInst->coreIdx, pEncInfo->streamWrPtrRegAddr, pEncInfo->streamWrPtr);
	VpuWriteReg(pCodecInst->coreIdx, pEncInfo->streamRdPtrRegAddr, pEncInfo->streamRdPtr);

	if (pCodecInst->productId == PRODUCT_ID_980) {
		val = 0;
		val |= (pEncInfo->openParam.bwbEnable<<15)|(pEncInfo->linear2TiledMode<<13)|(pEncInfo->mapType<<9);
		if (pEncInfo->openParam.EncStdParam.avcParam.chromaFormat400)
			val |= (FORMAT_400<<6);
		else
			val |= (FORMAT_420<<6);
		val |= ((pEncInfo->openParam.cbcrInterleave)<<2); // Interleave bit position is modified
		val |= pEncInfo->openParam.frameEndian;
		VpuWriteReg(pCodecInst->coreIdx, BIT_FRAME_MEM_CTRL, val);
	}
	else if (pCodecInst->productId == PRODUCT_ID_960) {
		val = 0;
		val |= (pEncInfo->openParam.bwbEnable<<12)|(pEncInfo->openParam.cbcrInterleave<<2);
		val |= pEncInfo->openParam.frameEndian;
		VpuWriteReg(pCodecInst->coreIdx, BIT_FRAME_MEM_CTRL, val);
	}
	else {
		return RETCODE_NOT_FOUND_VPU_DEVICE;
	}

	val = 0;
	if (pEncInfo->ringBufferEnable == 0) {

		if (pEncInfo->lineBufIntEn)
			val |= (0x1<<6);
		val |= (0x1<<5);
		val |= (0x1<<4);

	}
	else {
		val |= (0x1<<3);
	}
	val |= pEncInfo->openParam.streamEndian;
	VpuWriteReg(pCodecInst->coreIdx, BIT_BIT_STREAM_CTRL, val);


	BitIssueCommand(pCodecInst->coreIdx, pCodecInst, SEQ_INIT);
	if (vdi_wait_vpu_busy(pCodecInst->coreIdx, VPU_BUSY_CHECK_TIMEOUT, BIT_BUSY_FLAG) == -1) {
		if (pCodecInst->loggingEnable)
			vdi_log(pCodecInst->coreIdx, SEQ_INIT, 2);
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_VPU_RESPONSE_TIMEOUT;
	}

	if (pCodecInst->loggingEnable)
		vdi_log(pCodecInst->coreIdx, SEQ_INIT, 0);

	pEncInfo->streamWrPtr = VpuReadReg(pCodecInst->coreIdx, pEncInfo->streamWrPtrRegAddr);
	pEncInfo->streamEndflag = VpuReadReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM);
#ifdef SUPPORT_MEM_PROTECT
	if (VpuReadReg(pCodecInst->coreIdx, RET_ENC_SEQ_END_SUCCESS) & (1<<31)) {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_MEMORY_ACCESS_VIOLATION;
	}
#endif

	if (VpuReadReg(pCodecInst->coreIdx, RET_ENC_SEQ_END_SUCCESS) == 0) {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_FAILURE;
	}
	{
		info->minFrameBufferCount = 2; // reconstructed frame + reference frame
	}

	pEncInfo->initialInfo = *info;
	pEncInfo->initialInfoObtained = 1;



	LeaveLock(pCodecInst->coreIdx);

	return RETCODE_SUCCESS;
}
RetCode VPU_EncRegisterFrameBuffer(EncHandle handle, FrameBuffer * bufArray, int num, int stride, int height, int mapType)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	int i, val;
	RetCode ret;
	PhysicalAddress paraBuffer;
	BYTE frameAddr[22][3][4];
	int framebufFormat;


	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;

	pEncInfo->mapType = mapType;

	if (pEncInfo->stride) {
		return RETCODE_CALLED_BEFORE;
	}

	if (!pEncInfo->initialInfoObtained) {
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

	if (num < pEncInfo->initialInfo.minFrameBufferCount) {
		return RETCODE_INSUFFICIENT_FRAME_BUFFERS;
	}

	if (stride == 0 || (stride % 8 != 0)) {
		return RETCODE_INVALID_STRIDE;
	}

	EnterLock(pCodecInst->coreIdx);

	if (GetPendingInst(pCodecInst->coreIdx)) {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	pEncInfo->numFrameBuffers = num;
	pEncInfo->stride          = stride;
	pEncInfo->frameBufferHeight = height;
	pEncInfo->mapType = mapType;
	pEncInfo->mapCfg.productId = pCodecInst->productId;
	if (pCodecInst->codecMode == AVC_ENC)
	{
		framebufFormat = pEncInfo->openParam.EncStdParam.avcParam.chromaFormat400?FORMAT_400:FORMAT_420;
	}
	else
	{
		framebufFormat = FORMAT_420;
	}


	if (!ConfigSecAXI(pCodecInst->coreIdx, pEncInfo->openParam.bitstreamFormat, &pEncInfo->secAxiInfo, stride, height, 0)) {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_INSUFFICIENT_RESOURCE;
	}
#ifdef SUPPORT_MEM_PROTECT
	pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_SEC_AXI].enable = 1;
	pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_SEC_AXI].isSecondary = 1;
	pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_SEC_AXI].startAddress = pEncInfo->secAxiInfo.bufBitUse;
	pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_SEC_AXI].endAddress = pEncInfo->secAxiInfo.bufBitUse + pEncInfo->secAxiInfo.bufSize;
#endif
	if (pCodecInst->productId == PRODUCT_ID_980) {
		if (mapType != LINEAR_FRAME_MAP && mapType != LINEAR_FIELD_MAP) {
			val = SetTiledMapType(pCodecInst->coreIdx, &pEncInfo->mapCfg, mapType, (stride > height)?stride:height, pEncInfo->openParam.cbcrInterleave, &pEncInfo->dramCfg);

		} else {
			val = SetTiledMapType(pCodecInst->coreIdx, &pEncInfo->mapCfg, mapType,stride, pEncInfo->openParam.cbcrInterleave, &pEncInfo->dramCfg);
		}
	}
	else {
		val = SetTiledMapType(pCodecInst->coreIdx, &pEncInfo->mapCfg, mapType,stride, pEncInfo->openParam.cbcrInterleave, &pEncInfo->dramCfg);
	}
	if (val == 0) {
		return RETCODE_INVALID_PARAM;
	}

	if (pCodecInst->productId == PRODUCT_ID_980) {
		val = 0;
		val |= (pEncInfo->openParam.bwbEnable<<15)|(pEncInfo->linear2TiledMode<<13)|(pEncInfo->mapType<<9);
		if (pEncInfo->openParam.EncStdParam.avcParam.chromaFormat400)
			val |= (FORMAT_400<<6);
		else
			val |= (FORMAT_420<<6);

		val |= ((pEncInfo->openParam.cbcrInterleave)<<2); // Interleave bit position is modified
		val |= pEncInfo->openParam.frameEndian;
		VpuWriteReg(pCodecInst->coreIdx, BIT_FRAME_MEM_CTRL, val);
	}
	else if (pCodecInst->productId == PRODUCT_ID_960) {
		val = 0;
		if (pEncInfo->mapType) {
			if (pEncInfo->mapType == TILED_FRAME_MB_RASTER_MAP || pEncInfo->mapType == TILED_FIELD_MB_RASTER_MAP)
				val |= (pEncInfo->linear2TiledEnable<<11)|(0x03<<9)|(FORMAT_420<<6);
			else
				val |= (pEncInfo->linear2TiledEnable<<11)|(0x02<<9)|(FORMAT_420<<6);
		}
		val |= (pEncInfo->openParam.bwbEnable<<12)|(pEncInfo->openParam.cbcrInterleave<<2);
		val |= pEncInfo->openParam.frameEndian;
		VpuWriteReg(pCodecInst->coreIdx, BIT_FRAME_MEM_CTRL, val);
	}
	else {
		return RETCODE_NOT_FOUND_VPU_DEVICE;
	}

	//Allocate frame buffer
	if (bufArray) {
		for(i=0; i<num; i++)
			pEncInfo->frameBufPool[i] = bufArray[i];
	}
	else {
		if (pCodecInst->productId == PRODUCT_ID_980) {
			ret = AllocateFrameBufferArrayV20(pCodecInst->coreIdx, &pEncInfo->frameBufPool[0], &pEncInfo->vbFrame, pEncInfo->mapType,
				pEncInfo->openParam.cbcrInterleave, framebufFormat, num, stride, height, pEncInfo->openParam.frameEndian, 0, FB_TYPE_CODEC);
		}
		else {
			ret = AllocateFrameBufferArrayV10(pCodecInst->coreIdx, &pEncInfo->frameBufPool[0], &pEncInfo->vbFrame, pEncInfo->mapType,
				pEncInfo->openParam.cbcrInterleave, framebufFormat, num, stride, height, 0, FB_TYPE_CODEC, 0, &pEncInfo->dramCfg);
			pEncInfo->mapCfg.tiledBaseAddr = pEncInfo->vbFrame.phys_addr;
		}
	}
	if (ret != RETCODE_SUCCESS) {
		LeaveLock(pCodecInst->coreIdx);
		return ret;
	}

	paraBuffer = VpuReadReg(pCodecInst->coreIdx, BIT_PARA_BUF_ADDR);

	// Let the decoder know the addresses of the frame buffers.
	for (i=0; i<num; i++)
	{
		frameAddr[i][0][0] = (pEncInfo->frameBufPool[i].bufY  >> 24) & 0xFF;
		frameAddr[i][0][1] = (pEncInfo->frameBufPool[i].bufY  >> 16) & 0xFF;
		frameAddr[i][0][2] = (pEncInfo->frameBufPool[i].bufY  >>  8) & 0xFF;
		frameAddr[i][0][3] = (pEncInfo->frameBufPool[i].bufY  >>  0) & 0xFF;
		if (pEncInfo->openParam.cbcrOrder == CBCR_ORDER_NORMAL) {
			frameAddr[i][1][0] = (pEncInfo->frameBufPool[i].bufCb >> 24) & 0xFF;
			frameAddr[i][1][1] = (pEncInfo->frameBufPool[i].bufCb >> 16) & 0xFF;
			frameAddr[i][1][2] = (pEncInfo->frameBufPool[i].bufCb >>  8) & 0xFF;
			frameAddr[i][1][3] = (pEncInfo->frameBufPool[i].bufCb >>  0) & 0xFF;
			frameAddr[i][2][0] = (pEncInfo->frameBufPool[i].bufCr >> 24) & 0xFF;
			frameAddr[i][2][1] = (pEncInfo->frameBufPool[i].bufCr >> 16) & 0xFF;
			frameAddr[i][2][2] = (pEncInfo->frameBufPool[i].bufCr >>  8) & 0xFF;
			frameAddr[i][2][3] = (pEncInfo->frameBufPool[i].bufCr >>  0) & 0xFF;
		}
		else {
			frameAddr[i][2][0] = (pEncInfo->frameBufPool[i].bufCb >> 24) & 0xFF;
			frameAddr[i][2][1] = (pEncInfo->frameBufPool[i].bufCb >> 16) & 0xFF;
			frameAddr[i][2][2] = (pEncInfo->frameBufPool[i].bufCb >>  8) & 0xFF;
			frameAddr[i][2][3] = (pEncInfo->frameBufPool[i].bufCb >>  0) & 0xFF;
			frameAddr[i][1][0] = (pEncInfo->frameBufPool[i].bufCr >> 24) & 0xFF;
			frameAddr[i][1][1] = (pEncInfo->frameBufPool[i].bufCr >> 16) & 0xFF;
			frameAddr[i][1][2] = (pEncInfo->frameBufPool[i].bufCr >>  8) & 0xFF;
			frameAddr[i][1][3] = (pEncInfo->frameBufPool[i].bufCr >>  0) & 0xFF;
		}
	}
	VpuWriteMem(pCodecInst->coreIdx, paraBuffer, (BYTE *)frameAddr, sizeof(frameAddr), VDI_BIG_ENDIAN);
	if (pCodecInst->productId == PRODUCT_ID_980) {
		for (i=0; i<num; i++)
		{
			frameAddr[i][0][0] = (pEncInfo->frameBufPool[i].bufYBot  >> 24) & 0xFF;
			frameAddr[i][0][1] = (pEncInfo->frameBufPool[i].bufYBot  >> 16) & 0xFF;
			frameAddr[i][0][2] = (pEncInfo->frameBufPool[i].bufYBot  >>  8) & 0xFF;
			frameAddr[i][0][3] = (pEncInfo->frameBufPool[i].bufYBot  >>  0) & 0xFF;
			if (pEncInfo->openParam.cbcrOrder == CBCR_ORDER_NORMAL) {
				frameAddr[i][1][0] = (pEncInfo->frameBufPool[i].bufCbBot >> 24) & 0xFF;
				frameAddr[i][1][1] = (pEncInfo->frameBufPool[i].bufCbBot >> 16) & 0xFF;
				frameAddr[i][1][2] = (pEncInfo->frameBufPool[i].bufCbBot >>  8) & 0xFF;
				frameAddr[i][1][3] = (pEncInfo->frameBufPool[i].bufCbBot >>  0) & 0xFF;
				frameAddr[i][2][0] = (pEncInfo->frameBufPool[i].bufCrBot >> 24) & 0xFF;
				frameAddr[i][2][1] = (pEncInfo->frameBufPool[i].bufCrBot >> 16) & 0xFF;
				frameAddr[i][2][2] = (pEncInfo->frameBufPool[i].bufCrBot >>  8) & 0xFF;
				frameAddr[i][2][3] = (pEncInfo->frameBufPool[i].bufCrBot >>  0) & 0xFF;
			}
			else {
				frameAddr[i][2][0] = (pEncInfo->frameBufPool[i].bufCbBot >> 24) & 0xFF;
				frameAddr[i][2][1] = (pEncInfo->frameBufPool[i].bufCbBot >> 16) & 0xFF;
				frameAddr[i][2][2] = (pEncInfo->frameBufPool[i].bufCbBot >>  8) & 0xFF;
				frameAddr[i][2][3] = (pEncInfo->frameBufPool[i].bufCbBot >>  0) & 0xFF;
				frameAddr[i][1][0] = (pEncInfo->frameBufPool[i].bufCrBot >> 24) & 0xFF;
				frameAddr[i][1][1] = (pEncInfo->frameBufPool[i].bufCrBot >> 16) & 0xFF;
				frameAddr[i][1][2] = (pEncInfo->frameBufPool[i].bufCrBot >>  8) & 0xFF;
				frameAddr[i][1][3] = (pEncInfo->frameBufPool[i].bufCrBot >>  0) & 0xFF;
			}
		}
		VpuWriteMem(pCodecInst->coreIdx, paraBuffer+384+128, (BYTE *)frameAddr, sizeof(frameAddr), VDI_BIG_ENDIAN);
	}

	// Tell the codec how much frame buffers were allocated.
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_BUF_NUM, num);
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_BUF_STRIDE, stride);
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_AXI_BIT_ADDR, pEncInfo->secAxiInfo.bufBitUse);
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_AXI_IPACDC_ADDR, pEncInfo->secAxiInfo.bufIpAcDcUse);
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_AXI_DBKY_ADDR, pEncInfo->secAxiInfo.bufDbkYUse);
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_AXI_DBKC_ADDR, pEncInfo->secAxiInfo.bufDbkCUse);
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_AXI_OVL_ADDR, pEncInfo->secAxiInfo.bufOvlUse);

	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_CACHE_CONFIG, pEncInfo->cacheConfig.CacheMode);

	if (pCodecInst->productId == PRODUCT_ID_960) {
			ret = AllocateFrameBufferArrayV10(pCodecInst->coreIdx, &pEncInfo->frameBufPool[pEncInfo->numFrameBuffers], &pEncInfo->vbSubSampFrame, LINEAR_FRAME_MAP,
		pEncInfo->openParam.cbcrInterleave, FORMAT_420, 1, stride, height, 0, FB_TYPE_PPU, pEncInfo->mapCfg.tiledBaseAddr, &pEncInfo->dramCfg);
	if (ret != RETCODE_SUCCESS) {
		pEncInfo->vbSubSampFrame.size      = 0;
		pEncInfo->vbSubSampFrame.phys_addr = 0;
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_INSUFFICIENT_RESOURCE;
	}
	num++;

	// Set Sub-Sampling buffer for ME-Reference and DBK-Reconstruction
	// BPU will swap below two buffer internally every pic by pic
	val =  GetXY2AXIAddr(&pEncInfo->mapCfg, 0, 0, 0, stride, &pEncInfo->frameBufPool[pEncInfo->numFrameBuffers]);
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_SUBSAMP_A, val);
	VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_SUBSAMP_B, val+(stride*height/2));
	}

	if (pCodecInst->codecMode == MP4_ENC) {
		if (pEncInfo->vbScratch.size ==0)
		{
		// MPEG4 Encoder Data-Partitioned bitstream temporal buffer
		pEncInfo->vbScratch.size = SIZE_MP4ENC_DATA_PARTITION;
		if (vdi_allocate_dma_memory(pCodecInst->coreIdx, &pEncInfo->vbScratch)<0)
		{
			LeaveLock(pCodecInst->coreIdx);
			return RETCODE_INSUFFICIENT_RESOURCE;
			}
		}
		VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_DP_BUF_BASE, pEncInfo->vbScratch.phys_addr);
		VpuWriteReg(pCodecInst->coreIdx, CMD_SET_FRAME_DP_BUF_SIZE, pEncInfo->vbScratch.size>>10);
	}

#ifdef SUPPORT_MEM_PROTECT
	ConfigEncWPROTRegion(pCodecInst->coreIdx, pEncInfo, &pEncInfo->writeMemProtectCfg);

	pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_BITSTREAM].enable = 1;
	pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_BITSTREAM].isSecondary = 0;
	pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_BITSTREAM].startAddress = pEncInfo->streamBufStartAddr;
	pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_BITSTREAM].endAddress =  pEncInfo->streamBufStartAddr + pEncInfo->streamBufSize;

	pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_FRAME].enable        = 1;
	pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_FRAME].isSecondary  = 0;


	pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_FRAME].startAddress = GetXY2AXIAddr(&pEncInfo->mapCfg, 0, 0, 0, stride, &pEncInfo->frameBufPool[0]);
	pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_FRAME].endAddress   = GetXY2AXIAddr(&pEncInfo->mapCfg, 0, 0, 0, stride, &pEncInfo->frameBufPool[num-1]);
	pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_FRAME].endAddress += VPU_GetFrameBufSize(pCodecInst->coreIdx, stride, height, pEncInfo->mapCfg.mapType,
		framebufFormat, pEncInfo->openParam.cbcrInterleave, &pEncInfo->dramCfg);
#endif


	SetPendingInst(pCodecInst->coreIdx, pCodecInst);
	BitIssueCommand(pCodecInst->coreIdx, pCodecInst, SET_FRAME_BUF);
	if (vdi_wait_vpu_busy(pCodecInst->coreIdx, VPU_BUSY_CHECK_TIMEOUT, BIT_BUSY_FLAG) == -1) {
		if (pCodecInst->loggingEnable)
			vdi_log(pCodecInst->coreIdx, SET_FRAME_BUF, 2);
		SetPendingInst(pCodecInst->coreIdx, 0);
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_VPU_RESPONSE_TIMEOUT;
	}
	if (pCodecInst->loggingEnable)
		vdi_log(pCodecInst->coreIdx, SET_FRAME_BUF, 0);
#ifdef SUPPORT_MEM_PROTECT
	if (VpuReadReg(pCodecInst->coreIdx, RET_SET_FRAME_SUCCESS) & (1<<31)) {
		SetPendingInst(pCodecInst->coreIdx, 0);
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_MEMORY_ACCESS_VIOLATION;
	}
#endif
	SetPendingInst(pCodecInst->coreIdx, 0);
	LeaveLock(pCodecInst->coreIdx);
	return RETCODE_SUCCESS;
}


RetCode VPU_EncGetFrameBuffer(
	EncHandle     handle,
	int           frameIdx,
	FrameBuffer * frameBuf)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	RetCode ret;

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;

	if (frameIdx<0 || frameIdx>pEncInfo->numFrameBuffers)
		return RETCODE_INVALID_PARAM;

	if (frameBuf==0)
		return RETCODE_INVALID_PARAM;

	*frameBuf = pEncInfo->frameBufPool[frameIdx];

	return RETCODE_SUCCESS;

}
RetCode VPU_EncGetBitstreamBuffer( EncHandle handle,
	PhysicalAddress * prdPrt,
	PhysicalAddress * pwrPtr,
	int * size)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	PhysicalAddress rdPtr;
	PhysicalAddress wrPtr;
	Uint32 room;
	RetCode ret;

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	if ( prdPrt == 0 || pwrPtr == 0 || size == 0) {
		return RETCODE_INVALID_PARAM;
	}

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;

	rdPtr = pEncInfo->streamRdPtr;

	SetClockGate(pCodecInst->coreIdx, 1);

	if (GetPendingInst(pCodecInst->coreIdx) == pCodecInst)
		wrPtr = VpuReadReg(pCodecInst->coreIdx, pEncInfo->streamWrPtrRegAddr);
	else
		wrPtr = pEncInfo->streamWrPtr;

	SetClockGate(pCodecInst->coreIdx, 0);
	if( pEncInfo->ringBufferEnable == 1 ) {
		if ( wrPtr >= rdPtr ) {
			room = wrPtr - rdPtr;
		}
		else {
			room = ( pEncInfo->streamBufEndAddr - rdPtr ) + ( wrPtr - pEncInfo->streamBufStartAddr );
		}
	}
	else {
		if( rdPtr == pEncInfo->streamBufStartAddr && wrPtr >= rdPtr )
			room = wrPtr - rdPtr;
		else
			return RETCODE_INVALID_PARAM;
	}

	*prdPrt = rdPtr;
	*pwrPtr = wrPtr;
	*size = room;

	return RETCODE_SUCCESS;
}

RetCode VPU_EncUpdateBitstreamBuffer(
	EncHandle handle,
	int size)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	PhysicalAddress wrPtr;
	PhysicalAddress rdPtr;
	RetCode ret;
	int		room = 0;

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;

	rdPtr = pEncInfo->streamRdPtr;

	SetClockGate(pCodecInst->coreIdx, 1);

	if (GetPendingInst(pCodecInst->coreIdx) == pCodecInst)
		wrPtr = VpuReadReg(pCodecInst->coreIdx, pEncInfo->streamWrPtrRegAddr);
	else
		wrPtr = pEncInfo->streamWrPtr;

	if ( rdPtr < wrPtr ) {
		if ( rdPtr + size  > wrPtr ) {
			SetClockGate(pCodecInst->coreIdx, 0);
			return RETCODE_INVALID_PARAM;
		}
	}

	if( pEncInfo->ringBufferEnable == 1 ) {
		rdPtr += size;
		if (rdPtr > pEncInfo->streamBufEndAddr) {
			room = rdPtr - pEncInfo->streamBufEndAddr;
			rdPtr = pEncInfo->streamBufStartAddr;
			rdPtr += room;
		}
		if (rdPtr == pEncInfo->streamBufEndAddr) {
			rdPtr = pEncInfo->streamBufStartAddr;
		}
	}
	else {
		rdPtr = pEncInfo->streamBufStartAddr;
	}

	pEncInfo->streamRdPtr = rdPtr;
	pEncInfo->streamWrPtr = wrPtr;
	if (GetPendingInst(pCodecInst->coreIdx) == pCodecInst)
		VpuWriteReg(pCodecInst->coreIdx, pEncInfo->streamRdPtrRegAddr, rdPtr);

	SetClockGate(pCodecInst->coreIdx, 0);
	return RETCODE_SUCCESS;
}

RetCode VPU_EncStartOneFrame(
	EncHandle handle,
	EncParam * param )
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	FrameBuffer * pSrcFrame;
	Uint32 rotMirMode;
	Uint32 val;
	RetCode ret;
	vpu_instance_pool_t *vip;

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;


	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;
	vip = (vpu_instance_pool_t *)vdi_get_instance_pool(pCodecInst->coreIdx);
	if (!vip) {
		return RETCODE_INVALID_HANDLE;
	}

	if (pEncInfo->stride == 0) { // This means frame buffers have not been registered.
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

	ret = CheckEncParam(handle, param);
	if (ret != RETCODE_SUCCESS) {
		return ret;
	}

	pSrcFrame = param->sourceFrame;
	rotMirMode = 0;
	if (pEncInfo->rotationEnable) {
		switch (pEncInfo->rotationAngle) {
		case 0:
			rotMirMode |= 0x0;
			break;

		case 90:
			rotMirMode |= 0x1;
			break;

		case 180:
			rotMirMode |= 0x2;
			break;

		case 270:
			rotMirMode |= 0x3;
			break;
		}
	}
	if (pEncInfo->mirrorEnable) {
		switch (pEncInfo->mirrorDirection) {
		case MIRDIR_NONE :
			rotMirMode |= 0x0;
			break;

		case MIRDIR_VER :
			rotMirMode |= 0x4;
			break;

		case MIRDIR_HOR :
			rotMirMode |= 0x8;
			break;

		case MIRDIR_HOR_VER :
			rotMirMode |= 0xc;
			break;

		}
	}

	if (pCodecInst->productId == PRODUCT_ID_980) {
		rotMirMode |= ((pSrcFrame->endian&0x03)<<16);
		rotMirMode |= ((pSrcFrame->cbcrInterleave&0x01)<<18);
		rotMirMode |= ((pSrcFrame->sourceLBurstEn&0x01)<<4);
	}
	else {
		rotMirMode |= ((pSrcFrame->sourceLBurstEn&0x01)<<4);
	}


	EnterLock(pCodecInst->coreIdx);

	if (GetPendingInst(pCodecInst->coreIdx)) {
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_FRAME_NOT_COMPLETE;
	}

	if (pCodecInst->codecMode == AVC_ENC)
	{
		//ROI command
		if (param->setROI.mode)
		{
			int roi_number = param->setROI.number;
			int i;

			VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_ROI_MODE, 1); // currently, only mode 0 can be supported
			VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_ROI_NUM, roi_number);

			for (i=0; i<roi_number; i++)
			{
				Rect *rect = &param->setROI.region[i];
				int data;
				if (pEncInfo->openParam.EncStdParam.avcParam.fieldFlag)
					data = ((((rect->bottom+16)/32)&0xff)<<24)| ((((rect->top+16)/32)&0xff)<<16) | ((((rect->right+8)/16)&0xff)<<8) | (((rect->left+8)/16)&0xff);
				else
					data = ((((rect->bottom+8)/16)&0xff)<<24)| ((((rect->top+8)/16)&0xff)<<16) | ((((rect->right+8)/16)&0xff)<<8) | (((rect->left+8)/16)&0xff);

				VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_ROI_POS_0+i*8,  data);
				VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_ROI_QP_0+i*8, param->setROI.qp[i]);
			}
			BitIssueCommand(pCodecInst->coreIdx, pCodecInst, ENC_ROI_INIT);
			if (vdi_wait_vpu_busy(pCodecInst->coreIdx, VPU_BUSY_CHECK_TIMEOUT, BIT_BUSY_FLAG) == -1) {
				if (pCodecInst->loggingEnable)
					vdi_log(pCodecInst->coreIdx, ENC_ROI_INIT, 2);
				LeaveLock(pCodecInst->coreIdx);
				return RETCODE_VPU_RESPONSE_TIMEOUT;
			}
			if (pCodecInst->loggingEnable)
				vdi_log(pCodecInst->coreIdx, ENC_ROI_INIT, 0);
			if (!VpuReadReg(pCodecInst->coreIdx, RET_ENC_ROI_SUCCESS)) {
				LeaveLock(pCodecInst->coreIdx);
				return RETCODE_FAILURE;
			}
		} // if (param->setROI.mode)

		{
			int ActivePPSIdx = pEncInfo->ActivePPSIdx;
			AvcPpsParam *ActvePPS    =  &pEncInfo->openParam.EncStdParam.avcParam.ppsParam[ActivePPSIdx];
			if (ActvePPS->entropyCodingMode==2)
			{
				int pic_idx;
				int gop_num;
				int slice_type; // 0 = Intra, 1 =inter
				int mvc_second_view;

				pic_idx = pEncInfo->openParam.EncStdParam.avcParam.fieldFlag ? 2*pEncInfo->frameIdx+pEncInfo->fieldDone: pEncInfo->frameIdx;
				gop_num = pEncInfo->openParam.EncStdParam.avcParam.fieldFlag ? 2*pEncInfo->openParam.gopSize : pEncInfo->openParam.gopSize;
				mvc_second_view = 0;
				UNREFERENCED_PARAMETER(mvc_second_view);

				if (gop_num == 0)   // Only first I
				{
					if (pic_idx == 0 || pic_idx ==1)
						slice_type=0;
					else
						slice_type=1;

				}
				else if (gop_num == 1)   // All I
                {
					slice_type =0;
                }
				else
				{
					if ((pic_idx % gop_num) == 0 ||(pic_idx % gop_num) == 1)   // I frame
						slice_type=0;
					else  // P frame
						slice_type=1;
				}
				UNREFERENCED_PARAMETER(slice_type);
				{

					VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_PARA_CHANGE_ENABLE, 0x880);    // change_enable. ppsId, entropyCodingMode
					//default set by I (ID=0, mode=0)
					VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_PARA_CABAC_MODE, 0);
					VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_PARA_PPS_ID, 0);
					if (gop_num==0) // only first I
					{
						if (pic_idx!=0)
						{
							VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_PARA_CABAC_MODE, 1);
							VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_PARA_PPS_ID, 1);
						}

					}
					else if (gop_num!=1) // not All I
					{
						if ((pic_idx % gop_num)!=0) // P frame
						{
							VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_PARA_CABAC_MODE, 1);
							VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_SEQ_PARA_PPS_ID, 1);
						}
					}
				}
				BitIssueCommand(pCodecInst->coreIdx, pCodecInst, RC_CHANGE_PARAMETER);
				if (vdi_wait_vpu_busy(pCodecInst->coreIdx, VPU_BUSY_CHECK_TIMEOUT, BIT_BUSY_FLAG) == -1) {
					if (pCodecInst->loggingEnable)
						vdi_log(pCodecInst->coreIdx, RC_CHANGE_PARAMETER, 2);
					LeaveLock(pCodecInst->coreIdx);
					return RETCODE_VPU_RESPONSE_TIMEOUT;
				}
				if (pCodecInst->loggingEnable)
					vdi_log(pCodecInst->coreIdx, RC_CHANGE_PARAMETER, 0);
			}
		}
	}


	if (pCodecInst->productId == PRODUCT_ID_960) {
		if (pEncInfo->mapType > LINEAR_FRAME_MAP && pEncInfo->mapType <= TILED_MIXED_V_MAP) {
			SetTiledFrameBase(pCodecInst->coreIdx, pEncInfo->vbFrame.phys_addr);
		}
		else {
			SetTiledFrameBase(pCodecInst->coreIdx, 0);
		}

		val = SetTiledMapType(pCodecInst->coreIdx, &pEncInfo->mapCfg, pEncInfo->mapType, pEncInfo->stride, pEncInfo->openParam.cbcrInterleave, &pEncInfo->dramCfg);
		if (val == 0) {
			return RETCODE_INVALID_PARAM;
		}
	}
	if (pEncInfo->mapType != LINEAR_FRAME_MAP && pEncInfo->mapType != LINEAR_FIELD_MAP) {
		if (pEncInfo->stride > pEncInfo->frameBufferHeight)
			val = SetTiledMapType(pCodecInst->coreIdx, &pEncInfo->mapCfg, pEncInfo->mapType, pEncInfo->stride, pEncInfo->openParam.cbcrInterleave, &pEncInfo->dramCfg);
		else
			val = SetTiledMapType(pCodecInst->coreIdx, &pEncInfo->mapCfg, pEncInfo->mapType, pEncInfo->frameBufferHeight, pEncInfo->openParam.cbcrInterleave, &pEncInfo->dramCfg);
	} else {
		val = SetTiledMapType(pCodecInst->coreIdx, &pEncInfo->mapCfg, pEncInfo->mapType, pEncInfo->stride, pEncInfo->openParam.cbcrInterleave, &pEncInfo->dramCfg);
	}
	if (val == 0) {
		return RETCODE_INVALID_PARAM;
	}


	VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_PIC_ROT_MODE, rotMirMode);
	VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_PIC_QS, param->quantParam);

	if (param->skipPicture) {
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_PIC_OPTION, (param->fieldRun<<8) | 1);
	}
	else
	{
		// Registering Source Frame Buffer information
		// Hide GDI IF under FW level
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_PIC_SRC_INDEX, pSrcFrame->myIndex);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_PIC_SRC_STRIDE, pSrcFrame->stride);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_PIC_SRC_ADDR_Y, pSrcFrame->bufY);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_PIC_SRC_ADDR_CB, pSrcFrame->bufCb);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_PIC_SRC_ADDR_CR, pSrcFrame->bufCr);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_PIC_SRC_BOTTOM_Y, pSrcFrame->bufYBot);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_PIC_SRC_BOTTOM_CB, pSrcFrame->bufCbBot);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_PIC_SRC_BOTTOM_CR, pSrcFrame->bufCrBot);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_PIC_OPTION, (param->fieldRun<<8) | (param->forceIPicture << 1 & 0x2));
	}

	if (pEncInfo->ringBufferEnable == 0) {
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_PIC_BB_START, param->picStreamBufferAddr);
		VpuWriteReg(pCodecInst->coreIdx, CMD_ENC_PIC_BB_SIZE, param->picStreamBufferSize/1024); // size in KB
		VpuWriteReg(pCodecInst->coreIdx, pEncInfo->streamRdPtrRegAddr, param->picStreamBufferAddr);
		pEncInfo->streamRdPtr = param->picStreamBufferAddr;
#ifdef SUPPORT_MEM_PROTECT
		pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_BITSTREAM].enable = 1;
		pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_BITSTREAM].isSecondary = 0;
		pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_BITSTREAM].startAddress = param->picStreamBufferAddr;
		pEncInfo->writeMemProtectCfg.encRegion[WPROT_ENC_BITSTREAM].endAddress =  param->picStreamBufferAddr + param->picStreamBufferSize;
#endif
	}


	val = 0;
	val = (
		(pEncInfo->secAxiInfo.useBitEnable&0x01)<<0 |
		(pEncInfo->secAxiInfo.useIpEnable&0x01)<<1 |
		(pEncInfo->secAxiInfo.useDbkYEnable&0x01)<<2 |
		(pEncInfo->secAxiInfo.useDbkCEnable&0x01)<<3 |
		(pEncInfo->secAxiInfo.useOvlEnable&0x01)<<4 |
		(pEncInfo->secAxiInfo.useBtpEnable&0x01)<<5 |
		(pEncInfo->secAxiInfo.useBitEnable&0x01)<<8 |
		(pEncInfo->secAxiInfo.useIpEnable&0x01)<<9 |
		(pEncInfo->secAxiInfo.useDbkYEnable&0x01)<<10 |
		(pEncInfo->secAxiInfo.useDbkCEnable&0x01)<<11 |
		(pEncInfo->secAxiInfo.useOvlEnable&0x01)<<12
		| (pEncInfo->secAxiInfo.useBtpEnable&0x01)<<13
		);

	VpuWriteReg(pCodecInst->coreIdx, BIT_AXI_SRAM_USE, val);


	VpuWriteReg(pCodecInst->coreIdx, pEncInfo->streamWrPtrRegAddr, pEncInfo->streamWrPtr);
	VpuWriteReg(pCodecInst->coreIdx, pEncInfo->streamRdPtrRegAddr, pEncInfo->streamRdPtr);
	VpuWriteReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM, pEncInfo->streamEndflag);

	if (pCodecInst->productId == PRODUCT_ID_980) {
		val = 0;
		val |= (pEncInfo->openParam.bwbEnable<<15)|(pEncInfo->linear2TiledMode<<13)|(pEncInfo->mapType<<9);
		if (pEncInfo->openParam.EncStdParam.avcParam.chromaFormat400)
			val |= (FORMAT_400<<6);
		else
			val |= (FORMAT_420<<6);
		val |= ((pEncInfo->openParam.cbcrInterleave)<<2); // Interleave bit position is modified
		val |= pEncInfo->openParam.frameEndian;
		VpuWriteReg(pCodecInst->coreIdx, BIT_FRAME_MEM_CTRL, val);
	}
	else if (pCodecInst->productId == PRODUCT_ID_960) {
		val = 0;
		if (pEncInfo->mapType) {
			if (pEncInfo->mapType == TILED_FRAME_MB_RASTER_MAP || pEncInfo->mapType == TILED_FIELD_MB_RASTER_MAP)
				val |= (pEncInfo->linear2TiledEnable<<11)|(0x03<<9)|(FORMAT_420<<6);
			else
				val |= (pEncInfo->linear2TiledEnable<<11)|(0x02<<9)|(FORMAT_420<<6);
		}
		val |= (pEncInfo->openParam.bwbEnable<<12)|(pEncInfo->openParam.cbcrInterleave<<2);
		val |= pEncInfo->openParam.frameEndian;
		VpuWriteReg(pCodecInst->coreIdx, BIT_FRAME_MEM_CTRL, val);
	}
	val = 0;
	if (pEncInfo->ringBufferEnable == 0) {
		if (pEncInfo->lineBufIntEn)
			val |= (0x1<<6);
		val |= (0x1<<5);
		val |= (0x1<<4);
	}
	else {
		val |= (0x1<<3);
	}
	val |= pEncInfo->openParam.streamEndian;
	VpuWriteReg(pCodecInst->coreIdx, BIT_BIT_STREAM_CTRL, val);
	VpuWriteReg(pCodecInst->coreIdx, BIT_ME_LINEBUFFER_MODE, VPU_ME_LINEBUFFER_MODE);	// default


	BitIssueCommand(pCodecInst->coreIdx, pCodecInst, PIC_RUN);

	SetPendingInst(pCodecInst->coreIdx, pCodecInst);

	return RETCODE_SUCCESS;
}
RetCode VPU_EncGetOutputInfo(
	EncHandle handle,
	EncOutputInfo * info
	)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	RetCode ret;
	PhysicalAddress rdPtr;
	PhysicalAddress wrPtr;
	Uint32 val;

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS) {
		return ret;
	}

	if (info == 0) {
		return RETCODE_INVALID_PARAM;
	}

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;

	if (pCodecInst != GetPendingInst(pCodecInst->coreIdx)) {
		SetPendingInst(pCodecInst->coreIdx, 0);
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_WRONG_CALL_SEQUENCE;
	}

	if (pCodecInst->loggingEnable)
		vdi_log(pCodecInst->coreIdx, PIC_RUN, 0);

	pEncInfo->streamWrPtr = VpuReadReg(pCodecInst->coreIdx, pEncInfo->streamWrPtrRegAddr);
	pEncInfo->streamEndflag = VpuReadReg(pCodecInst->coreIdx, BIT_BIT_STREAM_PARAM);

	info->rdPtr = pEncInfo->streamRdPtr;
	info->wrPtr = pEncInfo->streamWrPtr;

	info->frameCycle = VpuReadReg(pCodecInst->coreIdx, BIT_FRAME_CYCLE);

	val = VpuReadReg(pCodecInst->coreIdx, RET_ENC_PIC_SUCCESS);
#ifdef SUPPORT_MEM_PROTECT
	if (val & (1<<31)) {
		SetPendingInst(pCodecInst->coreIdx, 0);
		LeaveLock(pCodecInst->coreIdx);
		return RETCODE_MEMORY_ACCESS_VIOLATION;
	}
#endif

	if (val & 2) { //top field coding done
		if (!pEncInfo->fieldDone)
			pEncInfo->fieldDone = 1;
	}
	else
	{
		pEncInfo->frameIdx = VpuReadReg(pCodecInst->coreIdx, RET_ENC_PIC_FRAME_NUM);
		pEncInfo->fieldDone = 0;
	}

	info->picType = VpuReadReg(pCodecInst->coreIdx, RET_ENC_PIC_TYPE);

	if (pEncInfo->ringBufferEnable == 0) {
		rdPtr = VpuReadReg(pCodecInst->coreIdx, pEncInfo->streamRdPtrRegAddr);
		wrPtr = VpuReadReg(pCodecInst->coreIdx, pEncInfo->streamWrPtrRegAddr);
		info->bitstreamBuffer = rdPtr;
		info->bitstreamSize = wrPtr - rdPtr;
	}

	info->numOfSlices = VpuReadReg(pCodecInst->coreIdx, RET_ENC_PIC_SLICE_NUM);
	info->bitstreamWrapAround = VpuReadReg(pCodecInst->coreIdx, RET_ENC_PIC_FLAG);
	info->reconFrameIndex = VpuReadReg(pCodecInst->coreIdx, RET_ENC_PIC_FRAME_IDX);
	info->reconFrame = pEncInfo->frameBufPool[info->reconFrameIndex];



	SetPendingInst(pCodecInst->coreIdx, 0);
	LeaveLock(pCodecInst->coreIdx);
	return RETCODE_SUCCESS;
}


RetCode VPU_EncGiveCommand(
	EncHandle handle,
	CodecCommand cmd,
	void * param)
{
	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	RetCode ret;

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;


	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;
	switch (cmd)
	{
	case ENABLE_ROTATION :
		{
			pEncInfo->rotationEnable = 1;
		}
		break;
	case DISABLE_ROTATION :
		{
			pEncInfo->rotationEnable = 0;
		}
		break;
	case ENABLE_MIRRORING :
		{
			pEncInfo->mirrorEnable = 1;
		}
		break;
	case DISABLE_MIRRORING :
		{
			pEncInfo->mirrorEnable = 0;
		}
		break;
	case SET_MIRROR_DIRECTION :
		{
			MirrorDirection mirDir;

			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}
			mirDir = *(MirrorDirection *)param;
			if (!(MIRDIR_NONE <= (int)mirDir && MIRDIR_HOR_VER >= (int)mirDir)) {
				return RETCODE_INVALID_PARAM;
			}
			pEncInfo->mirrorDirection = mirDir;
		}
		break;
	case SET_ROTATION_ANGLE :
		{
			int angle;

			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}
			angle = *(int *)param;
			if (angle != 0 && angle != 90 &&
				angle != 180 && angle != 270) {
					return RETCODE_INVALID_PARAM;
			}
			if (pEncInfo->initialInfoObtained && (angle == 90 || angle ==270)) {
				return RETCODE_INVALID_PARAM;
			}
			pEncInfo->rotationAngle = angle;
		}
		break;
	case SET_CACHE_CONFIG:
		{
			MaverickCacheConfig *mcCacheConfig;
			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}
			mcCacheConfig = (MaverickCacheConfig *)param;


			pEncInfo->cacheConfig.CacheMode = mcCacheConfig->CacheMode;

		}
		break;
	case ENC_PUT_MP4_HEADER:
	case ENC_PUT_AVC_HEADER:
	case ENC_PUT_VIDEO_HEADER:
		{
			EncHeaderParam *encHeaderParam;

			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}
			encHeaderParam = (EncHeaderParam *)param;
			if (pCodecInst->codecMode == MP4_ENC) {
				if (!( VOL_HEADER<=encHeaderParam->headerType && encHeaderParam->headerType <= VIS_HEADER)) {
					return RETCODE_INVALID_PARAM;
				}
			}
			else if  (pCodecInst->codecMode == AVC_ENC) {
				if (!( SPS_RBSP<=encHeaderParam->headerType && encHeaderParam->headerType <= PPS_RBSP_MVC)) {
					return RETCODE_INVALID_PARAM;
				}
			}
			else
				return RETCODE_INVALID_PARAM;

			if (pEncInfo->ringBufferEnable == 0 ) {
				if (encHeaderParam->buf % 8 || encHeaderParam->size == 0) {
					return RETCODE_INVALID_PARAM;
				}
			}
			return GetEncHeader(handle, encHeaderParam);
		}
	case ENC_SET_ACTIVE_PPS:
		{
			int ActivePPSIdx = (int)(*(int *)param);
			if (pCodecInst->codecMode != AVC_ENC)
				return RETCODE_INVALID_COMMAND;
			if (ActivePPSIdx <0 || ActivePPSIdx > pEncInfo->openParam.EncStdParam.avcParam.ppsNum)
				return RETCODE_INVALID_COMMAND;

			pEncInfo->ActivePPSIdx = ActivePPSIdx;
			return EncParaSet(handle, PPS_RBSP);
		}
		break;
	case ENC_GET_ACTIVE_PPS:
		if (pCodecInst->codecMode != AVC_ENC)
			return RETCODE_INVALID_COMMAND;
		*((int *)param) = pEncInfo->ActivePPSIdx;
		break;
	case ENC_SET_GOP_NUMBER:
		{
			int *pGopNumber =(int *)param;
			if (pCodecInst->codecMode != MP4_ENC && pCodecInst->codecMode != AVC_ENC ) {
				return RETCODE_INVALID_COMMAND;
			}
			if (*pGopNumber < 0 ||  *pGopNumber > 60)
				return RETCODE_INVALID_PARAM;
			pEncInfo->openParam.gopSize = *pGopNumber;

			SetGopNumber(handle, (Uint32 *)pGopNumber);
		}
		break;
	case ENC_SET_INTRA_QP:
		{
			int *pIntraQp =(int *)param;
			if (pCodecInst->codecMode != MP4_ENC && pCodecInst->codecMode != AVC_ENC ) {
				return RETCODE_INVALID_COMMAND;
			}
			if (pCodecInst->codecMode == MP4_ENC)
			{
				if(*pIntraQp<1 || *pIntraQp>31)
					return RETCODE_INVALID_PARAM;
			}
			if (pCodecInst->codecMode == AVC_ENC)
			{
				if(*pIntraQp<0 || *pIntraQp>51)
					return RETCODE_INVALID_PARAM;
			}
			SetIntraQp(handle, (Uint32 *)pIntraQp);
		}
		break;
	case ENC_SET_BITRATE:
		{
			int *pBitrate = (int *)param;
			if (pCodecInst->codecMode != MP4_ENC && pCodecInst->codecMode != AVC_ENC ) {
				return RETCODE_INVALID_COMMAND;
			}
			if (pCodecInst->codecMode == AVC_ENC)
			{
				if (*pBitrate < 0 || *pBitrate>524288) {
					return RETCODE_INVALID_PARAM;
				}

			}
			else // MP4_ENC
			{
				if (*pBitrate < 0 || *pBitrate> 32767) {
					return RETCODE_INVALID_PARAM;
				}
			}
			SetBitrate(handle, (Uint32 *)pBitrate);
		}
		break;
	case ENC_SET_FRAME_RATE:
		{
			int *pFramerate = (int *)param;

			if (pCodecInst->codecMode != MP4_ENC && pCodecInst->codecMode != AVC_ENC ) {
				return RETCODE_INVALID_COMMAND;
			}
			if (*pFramerate <= 0) {
				return RETCODE_INVALID_PARAM;
			}
			SetFramerate(handle, (Uint32 *)pFramerate);
		}
		break;
	case ENC_SET_INTRA_MB_REFRESH_NUMBER:
		{
			int *pIntraRefreshNum =(int *)param;
			SetIntraRefreshNum(handle, (Uint32 *)pIntraRefreshNum);
		}
		break;
	case ENC_SET_SLICE_INFO:
		{
			EncSliceMode *pSliceMode = (EncSliceMode *)param;
			if(pSliceMode->sliceMode<0 || pSliceMode->sliceMode>1)
			{
				return RETCODE_INVALID_PARAM;
			}
			if(pSliceMode->sliceSizeMode<0 || pSliceMode->sliceSizeMode>1)
			{
				return RETCODE_INVALID_PARAM;
			}

			SetSliceMode(handle, (EncSliceMode *)pSliceMode);
		}
		break;
	case ENC_ENABLE_HEC:
		{
			if (pCodecInst->codecMode != MP4_ENC) {
				return RETCODE_INVALID_COMMAND;
			}
			SetHecMode(handle, 1);
		}
		break;
	case ENC_DISABLE_HEC:
		{
			if (pCodecInst->codecMode != MP4_ENC) {
				return RETCODE_INVALID_COMMAND;
			}
			SetHecMode(handle, 0);
		}
		break;
	case SET_SEC_AXI:
		{
			SecAxiUse secAxiUse;

			if (param == 0) {
				return RETCODE_INVALID_PARAM;
			}
			secAxiUse = *(SecAxiUse *)param;
			pEncInfo->secAxiInfo.useBitEnable = secAxiUse.useBitEnable;
			pEncInfo->secAxiInfo.useIpEnable = secAxiUse.useIpEnable;
			pEncInfo->secAxiInfo.useDbkYEnable = secAxiUse.useDbkYEnable;
			pEncInfo->secAxiInfo.useDbkCEnable = secAxiUse.useDbkCEnable;
			pEncInfo->secAxiInfo.useOvlEnable = secAxiUse.useOvlEnable;
			pEncInfo->secAxiInfo.useBtpEnable = secAxiUse.useBtpEnable;
		}
		break;
	case GET_TILEDMAP_CONFIG:
		{
			TiledMapConfig *pMapCfg = (TiledMapConfig *)param;
			if (!pMapCfg) {
				return RETCODE_INVALID_PARAM;
			}
			*pMapCfg = pEncInfo->mapCfg;
			break;
		}
	case SET_DRAM_CONFIG:
		{
			DRAMConfig *cfg = (DRAMConfig *)param;

			if (!cfg) {
				return RETCODE_INVALID_PARAM;
			}

			pEncInfo->dramCfg = *cfg;
			break;
		}
	case GET_DRAM_CONFIG:
		{
			DRAMConfig *cfg = (DRAMConfig *)param;

			if (!cfg) {
				return RETCODE_INVALID_PARAM;
			}

			*cfg = pEncInfo->dramCfg;

			break;
		}
    case ENC_SET_EXT_SCRATCH_BUFF:
        {
            vpu_buffer_t *pvbScratch = (vpu_buffer_t *)param;
            if (param == 0)
                return RETCODE_INVALID_PARAM;
            if (pCodecInst->codecMode == MP4_ENC) {
                if (pvbScratch->size != SIZE_MP4ENC_DATA_PARTITION) {
                    return RETCODE_INVALID_PARAM;
                }
            }
            else
                return RETCODE_INVALID_PARAM;
            pEncInfo->vbScratch.size = pvbScratch->size;
            pEncInfo->vbScratch.virt_addr = (unsigned long *)pvbScratch->base;
            pEncInfo->vbScratch.phys_addr = pvbScratch->phys_addr;
        }
        break;
	case ENABLE_LOGGING:
		{
			pCodecInst->loggingEnable = 1;
		}
		break;
	case DISABLE_LOGGING:
		{
			pCodecInst->loggingEnable = 0;
		}
		break;
	default:
		return RETCODE_INVALID_COMMAND;
	}
	return RETCODE_SUCCESS;
}

RetCode VPU_EncAllocateFrameBuffer(EncHandle handle, FrameBufferAllocInfo info, FrameBuffer *frameBuffer)
{

	CodecInst * pCodecInst;
	EncInfo * pEncInfo;
	RetCode ret;
	int gdiIndex;

	ret = CheckEncInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	pEncInfo = &pCodecInst->CodecInfo.encInfo;

	if (info.type == FB_TYPE_PPU) {
		if (pEncInfo->numFrameBuffers == 0) {
			return RETCODE_WRONG_CALL_SEQUENCE;
		}
		pEncInfo->ppuAllocExt = 0;
		if (frameBuffer[0].bufCb == (PhysicalAddress)-1 && frameBuffer[0].bufCr == (PhysicalAddress)-1) {
			pEncInfo->ppuAllocExt = 1;
		}
		gdiIndex = pEncInfo->numFrameBuffers;

		if (pCodecInst->productId == PRODUCT_ID_980) {
			ret = AllocateFrameBufferArrayV20(pCodecInst->coreIdx, &frameBuffer[0], &pEncInfo->vbPPU, info.mapType, info.cbcrInterleave,
				        info.format, info.num, info.stride, info.height, info.endian, gdiIndex, info.type);
		}
		else {
			ret = AllocateFrameBufferArrayV10(pCodecInst->coreIdx, &frameBuffer[0], &pEncInfo->vbPPU, info.mapType, info.cbcrInterleave,
				        info.format, info.num, info.stride, info.height, gdiIndex, info.type, pEncInfo->mapCfg.tiledBaseAddr, &pEncInfo->dramCfg);
		}
	}
	else {
		gdiIndex = 0;
		pEncInfo->frameAllocExt = 0;
		if (frameBuffer[0].bufCb == (PhysicalAddress)-1 && frameBuffer[0].bufCr == (PhysicalAddress)-1) {
			pEncInfo->frameAllocExt = 1;
		}
		if (pCodecInst->productId == PRODUCT_ID_980) {
			ret = AllocateFrameBufferArrayV20(pCodecInst->coreIdx, &frameBuffer[0], &pEncInfo->vbFrame, info.mapType, info.cbcrInterleave,
				        info.format, info.num, info.stride, info.height, info.endian, gdiIndex, info.type);
		}
		else {
			ret = AllocateFrameBufferArrayV10(pCodecInst->coreIdx, &frameBuffer[0], &pEncInfo->vbFrame, info.mapType, info.cbcrInterleave,
				        info.format, info.num, info.stride, info.height, gdiIndex, info.type, 0, &pEncInfo->dramCfg);
			pEncInfo->mapCfg.tiledBaseAddr = pEncInfo->vbFrame.phys_addr;
		}
	}

	return ret;
}


RetCode VPU_PpuOpen(
	PpuHandle *pHandle,
	int coreIdx,
	vpu_buffer_t vbWork
	)
{
	CodecInst * pCodecInst;
	RetCode ret;
    DecInfo * pDecInfo;


	EnterLock(coreIdx);

	if (VpuReadReg(coreIdx, BIT_CUR_PC) == 0) {
		LeaveLock(coreIdx);
		return RETCODE_NOT_INITIALIZED;
	}


	ret = GetCodecInstance(coreIdx, &pCodecInst);
	if (ret == RETCODE_FAILURE) {
		*pHandle = 0;
		LeaveLock(coreIdx);
		return RETCODE_FAILURE;
	}

	*pHandle = pCodecInst;
	pCodecInst->codecMode = PPU_STAND_ALONE;
	pCodecInst->codecModeAux = 0;

    pDecInfo = &pCodecInst->CodecInfo.decInfo;

	if (vbWork.size) {
		pDecInfo->vbWork = vbWork;
		pDecInfo->workBufferAllocExt = 1;
	}
	else {
		pDecInfo->vbWork.size       = WORK_BUF_SIZE;
		if (vdi_allocate_dma_memory(pCodecInst->coreIdx, &pDecInfo->vbWork) < 0)
		{
		    *pHandle = 0;
		    LeaveLock(pCodecInst->coreIdx);
		    return RETCODE_INSUFFICIENT_RESOURCE;
		}
	}

	LeaveLock(pCodecInst->coreIdx);
	return RETCODE_SUCCESS;
}

RetCode VPU_PpuClose(PpuHandle handle)
{
	CodecInst * pCodecInst;
	RetCode ret;

	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;


	FreeCodecInstance(pCodecInst);

	return RETCODE_SUCCESS;
}

RetCode VPU_PpuStartOneFrame(
	PpuHandle handle,
	FrameBuffer *srcFrame,
	FrameBuffer *dstFrame,
	TiledMapType mapType,
	int cbcrInterleave,
	int endian,
	int width,
	int height,
	int rotAngle,
	int mirDir,
	FrameFlag tiled2LinearMode,
	int enableBWB)
{
	int val;
	RetCode ret;
	int coreIdx;
	CodecInst * pCodecInst;
	vpu_instance_pool_t *vip;


	ret = CheckDecInstanceValidity(handle);
	if (ret != RETCODE_SUCCESS)
		return ret;

	pCodecInst = handle;
	vip = (vpu_instance_pool_t *)vdi_get_instance_pool(pCodecInst->coreIdx);
	if (!vip) {
		return RETCODE_INVALID_HANDLE;
	}
	coreIdx = pCodecInst->coreIdx;


	if (srcFrame->myIndex == dstFrame->myIndex) {
		return RETCODE_INVALID_PARAM;
	}
	mapType = srcFrame->mapType;

	val = 0;
	val |= ((enableBWB&1)<<15);
	val |= (tiled2LinearMode<<13)|(mapType<<9)|(FORMAT_420<<6);
	val |= ((cbcrInterleave)<<2); // Interleave bit position is modified
	val |= endian&3;


    if (pCodecInst->productId == PRODUCT_ID_980) {
        val = 0;
        val |= (enableBWB<<15);
        val |= (tiled2LinearMode<<13)|(mapType<<9)|(FORMAT_420<<6);
        val |= ((cbcrInterleave)<<2); // Interleave bit position is modified
        val |= endian;
        VpuWriteReg(pCodecInst->coreIdx, BIT_FRAME_MEM_CTRL, val);
    }
    else if (pCodecInst->productId == PRODUCT_ID_960) {
        val = 0;
        val |= (enableBWB<<12);
        if (mapType) {
            if (mapType == TILED_FRAME_MB_RASTER_MAP || mapType == TILED_FIELD_MB_RASTER_MAP)
                val |= (tiled2LinearMode<<11)|(0x03<<9)|(FORMAT_420<<6);
            else
                val |= (tiled2LinearMode<<11)|(0x02<<9)|(FORMAT_420<<6);
        }
        val |= ((cbcrInterleave)<<2); // Interleave bit position is modified
        val |= endian;
        VpuWriteReg(pCodecInst->coreIdx, BIT_FRAME_MEM_CTRL, val);
    }
    else {
        return RETCODE_NOT_FOUND_VPU_DEVICE;
    }


	//RotMode[1:0] 01b:90, 10b:180, 11b:270,
	//RotMode[2] vert mirroring on/off
	//RotMode[3] hor mirroring on/off
	//RotMode[4] rotation enable
	val = 1<<4; //enable rotaion
	switch (rotAngle)
	{
	case 90:
		val |= 1;
		break;
	case 180:
		val |= 2;
		break;
	case 270:
		val |= 3;
		break;
	default:
		break;
	}
	switch (mirDir)
	{
	case MIRDIR_VER:
		val |= 4;
		break;
	case MIRDIR_HOR:
		val |= 8;
		break;
	case MIRDIR_HOR_VER:
		val |= 12;
		break;
	default:
		break;
	}

	VpuWriteReg(coreIdx, CMD_DEC_PIC_ROT_MODE, val);

	VpuWriteReg(coreIdx, CMD_PPU_SRC_INDEX, srcFrame->myIndex);
	VpuWriteReg(coreIdx, CMD_PPU_SRC_STRIDE, srcFrame->stride);
	VpuWriteReg(coreIdx, CMD_PPU_SRC_ADDR_Y, srcFrame->bufY);
	VpuWriteReg(coreIdx, CMD_PPU_SRC_ADDR_CB, srcFrame->bufCb);
	VpuWriteReg(coreIdx, CMD_PPU_SRC_ADDR_CR, srcFrame->bufCr);
	VpuWriteReg(coreIdx, CMD_PPU_SRC_BOTTOM_Y, srcFrame->bufYBot);
	VpuWriteReg(coreIdx, CMD_PPU_SRC_BOTTOM_CB, srcFrame->bufCbBot);
	VpuWriteReg(coreIdx, CMD_PPU_SRC_BOTTOM_CR, srcFrame->bufCrBot);

	VpuWriteReg(coreIdx, CMD_DEC_PIC_ROT_INDEX, dstFrame->myIndex);
	VpuWriteReg(coreIdx, CMD_DEC_PIC_ROT_STRIDE, dstFrame->stride);
	VpuWriteReg(coreIdx, CMD_DEC_PIC_ROT_ADDR_Y, dstFrame->bufY);
	VpuWriteReg(coreIdx, CMD_DEC_PIC_ROT_ADDR_CB, dstFrame->bufCb);
	VpuWriteReg(coreIdx, CMD_DEC_PIC_ROT_ADDR_CR, dstFrame->bufCr);
	VpuWriteReg(coreIdx, CMD_DEC_PIC_ROT_BOTTOM_Y, dstFrame->bufYBot);
	VpuWriteReg(coreIdx, CMD_DEC_PIC_ROT_BOTTOM_CB, dstFrame->bufCbBot);
	VpuWriteReg(coreIdx, CMD_DEC_PIC_ROT_BOTTOM_CR, dstFrame->bufCrBot);
	val = ((width&0xffff)<<16) | (height&0xffff);
	VpuWriteReg(coreIdx, CMD_PPU_SRC_PIC_SIZE, val);

    BitIssueCommand(coreIdx, pCodecInst, PIC_RUN);

	return RETCODE_SUCCESS;
}



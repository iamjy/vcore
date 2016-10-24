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

#include "jpuapifunc.h"
#include "jpuapi.h"
#include "../include/jpulog.h"

int JPU_IsBusy()
{
	Uint32 val;
	val = JpuReadReg(MJPEG_PIC_STATUS_REG);

	if ((val & (1<<INT_JPU_DONE)) ||
		(val & (1<<INT_JPU_ERROR)))
        return 0;

	return 1;
}
void JPU_ClrStatus(Uint32 val)
{
	if (val != 0)
		JpuWriteReg(MJPEG_PIC_STATUS_REG, val);
}
Uint32 JPU_GetStatus()
{
	return JpuReadReg(MJPEG_PIC_STATUS_REG);
}


Uint32 JPU_IsInit()
{
	jpu_instance_pool_t *pjip;

	pjip = (jpu_instance_pool_t *)jdi_get_instance_pool();

	if (!pjip)
		return 0;

	return 1;
}

Uint32	JPU_WaitInterrupt(int timeout)
{
	Uint32 reason = 0;

	reason = jdi_wait_interrupt(timeout);

	JpgSetClockGate(1);

	if (reason != (Uint32)-1)
		reason = JpuReadReg(MJPEG_PIC_STATUS_REG);


	JpgSetClockGate(0);

	return reason;
}

int JPU_GetOpenInstanceNum(void)
{
	jpu_instance_pool_t *pjip;

	pjip = (jpu_instance_pool_t *)jdi_get_instance_pool();
	if (!pjip)
		return -1;

	return pjip->jpu_instance_num;
}

JpgRet JPU_Init(unsigned long reg_base)
{
	jpu_instance_pool_t *pjip;

	if (jdi_init(reg_base) < 0)
		return JPG_RET_FAILURE;

	pjip = (jpu_instance_pool_t *)jdi_get_instance_pool();
	if (!pjip)
		return JPG_RET_FAILURE;

	InitJpgInstancePool();

	return JPG_RET_SUCCESS;
}
void JPU_DeInit()
{
	jdi_release();
}

JpgRet JPU_GetVersionInfo(Uint32 *versionInfo)
{

	if (JPU_IsInit() == 0) {
		return JPG_RET_NOT_INITIALIZED;
	}

	*versionInfo = API_VERSION;

	return JPG_RET_SUCCESS;
}

JpgRet JPU_DecOpen(JpgDecHandle * pHandle, JpgDecOpenParam * pop)
{
	JpgInst * pJpgInst;
	JpgDecInfo * pDecInfo;
	JpgRet ret;


	ret = CheckJpgDecOpenParam(pop);
	if (ret != JPG_RET_SUCCESS) {
		return ret;
	}


	ret = GetJpgInstance(&pJpgInst);
	if (ret == JPG_RET_FAILURE) {
		*pHandle = 0;
		JpgLeaveLock();
		return JPG_RET_FAILURE;
	}

	*pHandle = pJpgInst;

	pDecInfo = &pJpgInst->JpgInfo.decInfo;
	jdi_memset(pDecInfo, 0x00, sizeof(JpgDecInfo));

	pDecInfo->streamWrPtr = pop->bitstreamBuffer;
	pDecInfo->streamRdPtr = pop->bitstreamBuffer;

	pDecInfo->streamBufStartAddr = pop->bitstreamBuffer;
	pDecInfo->streamBufSize = pop->bitstreamBufferSize;
	pDecInfo->streamBufEndAddr = pop->bitstreamBuffer + pop->bitstreamBufferSize;
	JpuWriteReg(MJPEG_BBC_BAS_ADDR_REG, pDecInfo->streamBufStartAddr);
	JpuWriteReg(MJPEG_BBC_END_ADDR_REG, pDecInfo->streamBufEndAddr);
	JpuWriteReg(MJPEG_BBC_RD_PTR_REG, pDecInfo->streamRdPtr);
	JpuWriteReg(MJPEG_BBC_WR_PTR_REG, pDecInfo->streamWrPtr);
	JpuWriteReg(MJPEG_BBC_STRM_CTRL_REG, 0);


	pDecInfo->pBitStream = pop->pBitStream;

	pDecInfo->streamEndian = pop->streamEndian;
	pDecInfo->frameEndian = pop->frameEndian;
	pDecInfo->chromaInterleave = pop->chromaInterleave;



    return JPG_RET_SUCCESS;
}

JpgRet JPU_DecClose(JpgDecHandle handle)
{
	JpgInst * pJpgInst;
	JpgRet ret;

	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;

	pJpgInst = handle;

	JpgEnterLock();
	FreeJpgInstance(pJpgInst);
	JpgLeaveLock();

	return JPG_RET_SUCCESS;
}

JpgRet JPU_DecGetInitialInfo(JpgDecHandle handle, JpgDecInitialInfo * info)
{
	JpgInst * pJpgInst;
	JpgDecInfo * pDecInfo;
	JpgRet ret;


	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;

	if (info == 0) {
		return JPG_RET_INVALID_PARAM;
	}
	pJpgInst = handle;
	pDecInfo = &pJpgInst->JpgInfo.decInfo;

	if (JpegDecodeHeader(pDecInfo) <= 0)
		return JPG_RET_FAILURE;

	info->picWidth = pDecInfo->picWidth;
	info->picHeight = pDecInfo->picHeight;
	info->minFrameBufferCount = 1;
	info->sourceFormat = pDecInfo->format;
	info->ecsPtr = pDecInfo->ecsPtr;

	pDecInfo->initialInfoObtained = 1;
	pDecInfo->minFrameBufferNum = 1;

#ifdef MJPEG_ERROR_CONCEAL
    pDecInfo->curRstIdx = 0;
    pDecInfo->nextRstIdx = -1;
#endif


	info->colorComponents = pDecInfo->compNum;

	return JPG_RET_SUCCESS;
}


JpgRet JPU_DecRegisterFrameBuffer(JpgDecHandle handle, FrameBuffer * bufArray, int num, int stride)
{
	JpgInst * pJpgInst;
	JpgDecInfo * pDecInfo;
	JpgRet ret;


	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;

	pJpgInst = handle;
	pDecInfo = &pJpgInst->JpgInfo.decInfo;


	if (!pDecInfo->initialInfoObtained) {
		return JPG_RET_WRONG_CALL_SEQUENCE;
	}

	if (bufArray == 0) {
		return JPG_RET_INVALID_FRAME_BUFFER;
	}

	if (num < pDecInfo->minFrameBufferNum) {
		return JPG_RET_INSUFFICIENT_FRAME_BUFFERS;
	}

	pDecInfo->frameBufPool = bufArray;
	pDecInfo->numFrameBuffers = num;
	pDecInfo->stride = stride;

	return JPG_RET_SUCCESS;
}

JpgRet JPU_DecGetBitstreamBuffer(JpgDecHandle handle,
								 PhysicalAddress * prdPrt,
								 PhysicalAddress * pwrPtr,
								 int * size)
{
	JpgInst * pJpgInst;
	JpgDecInfo * pDecInfo;
	JpgRet ret;
	PhysicalAddress rdPtr;
	PhysicalAddress wrPtr;
	int room;

	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;

	if (prdPrt == 0 || pwrPtr == 0 || size == 0) {
		return JPG_RET_INVALID_PARAM;
	}

	pJpgInst = handle;
	pDecInfo = &pJpgInst->JpgInfo.decInfo;

	if (GetJpgPendingInst() == pJpgInst) {
		rdPtr = JpuReadReg(MJPEG_BBC_RD_PTR_REG);
	}
	else {
		rdPtr = pDecInfo->streamRdPtr;
	}

	wrPtr = pDecInfo->streamWrPtr;

	if (wrPtr == pDecInfo->streamBufStartAddr) {
		if (pDecInfo->frameOffset == 0) {
			room = (pDecInfo->streamBufEndAddr-pDecInfo->streamBufStartAddr);
		}
		else {
			room = (pDecInfo->frameOffset);
		}		
	}
	else {
		room = (pDecInfo->streamBufEndAddr - wrPtr);
	}

	room = ((room>>9)<<9); // multiple of 512

	*prdPrt = rdPtr;
	*pwrPtr = wrPtr;

	*size = room;


	return JPG_RET_SUCCESS;
}


JpgRet JPU_DecUpdateBitstreamBuffer(JpgDecHandle handle, int size)
{
	JpgInst * pJpgInst;
	JpgDecInfo * pDecInfo;
	PhysicalAddress wrPtr;
	PhysicalAddress rdPtr;
	JpgRet ret;
	int		val = 0;
	

	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;

	pJpgInst = handle;
	pDecInfo = &pJpgInst->JpgInfo.decInfo;
	wrPtr = pDecInfo->streamWrPtr;

	if (size == 0)
	{
 		val = (wrPtr-pDecInfo->streamBufStartAddr) / 256;
		if ((wrPtr-pDecInfo->streamBufStartAddr) % 256)
			val = val + 1;
		if (GetJpgPendingInst() == pJpgInst)
			JpuWriteReg(MJPEG_BBC_STRM_CTRL_REG, (1 << 31 | val));
		pDecInfo->streamEndflag = 1;
		return JPG_RET_SUCCESS;
	}



	JpgSetClockGate(1);

	wrPtr = pDecInfo->streamWrPtr;
	wrPtr += size;

	if (wrPtr == pDecInfo->streamBufEndAddr) {
		wrPtr = pDecInfo->streamBufStartAddr;
	}		

	pDecInfo->streamWrPtr = wrPtr;

	if (GetJpgPendingInst() == pJpgInst) {                                                                                                                                                                    
		rdPtr = JpuReadReg(MJPEG_BBC_RD_PTR_REG);                                                                                                                                                             

		if (rdPtr == pDecInfo->streamBufEndAddr) {                                                                                                                                                            
			JpuWriteReg(MJPEG_BBC_CUR_POS_REG, 0);                                                                                                                                                            
			JpuWriteReg(MJPEG_GBU_TT_CNT_REG, 0);                                                                                                                                                             
			JpuWriteReg(MJPEG_GBU_TT_CNT_REG+4, 0);                                                                                                                                                           
		}                                                                                                                                                                                                     

		JpuWriteReg(MJPEG_BBC_WR_PTR_REG, wrPtr);                                                                                                                                                             
		if (wrPtr == pDecInfo->streamBufStartAddr) {                                                                                                                                                          
			JpuWriteReg(MJPEG_BBC_END_ADDR_REG, pDecInfo->streamBufEndAddr);                                                                                                                                  
		}                                                                                                                                                                                                     
		else {                                                                                                                                                                                                
			JpuWriteReg(MJPEG_BBC_END_ADDR_REG, wrPtr);                                                                                                                                                       
		}                                                                                                                                                                                                     
	}                                                                                                                                                                                                         
	else {                                                                                                                                                                                                    
		rdPtr = pDecInfo->streamRdPtr;                                                                                                                                                                        
	}		                                                                                                                                                                                                  

	pDecInfo->streamRdPtr = rdPtr;         

	JpgSetClockGate(0);
	return JPG_RET_SUCCESS;

}

JpgRet JPU_HWReset()
{
	if (jdi_hw_reset() < 0 )
		return JPG_RET_FAILURE;

	return JPG_RET_SUCCESS;
}

#ifdef SUPPORT_STOP_COMMAND
JpgRet JPU_DecIssueStop(JpgDecHandle handle)
{
	JpgInst * pJpgInst;
	JpgDecInfo * pDecInfo;
	JpgRet ret;


	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;

	pJpgInst = handle;
	pDecInfo = &pJpgInst->JpgInfo.decInfo;

	if (pJpgInst != GetJpgPendingInst()) {
		return JPG_RET_WRONG_CALL_SEQUENCE;
	}

	JpgSetClockGate(1);
	JpuWriteReg(MJPEG_PIC_START_REG, 1<< JPG_START_STOP);
	JpgSetClockGate(0);
	return JPG_RET_SUCCESS;

}
JpgRet JPU_DecCompleteStop(JpgDecHandle handle)
{
	JpgInst * pJpgInst;
	JpgDecInfo * pDecInfo;
	JpgRet ret;
	Uint32 val;

	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;

	pJpgInst = handle;
	pDecInfo = &pJpgInst->JpgInfo.decInfo;

	if (pJpgInst != GetJpgPendingInst()) {
		return JPG_RET_WRONG_CALL_SEQUENCE;
	}

	JpgSetClockGate(1);
	val = JpuReadReg(MJPEG_PIC_STATUS_REG);

	if (val & (1<<INT_JPU_BIT_BUF_STOP)) {
		SetJpgPendingInst(0);
		JpgSetClockGate(0);
	}
	else {
		JpgSetClockGate(0);
		return JPG_RET_WRONG_CALL_SEQUENCE;
	}

	return JPG_RET_SUCCESS;
}
#endif
JpgRet JPU_DecStartOneFrame(JpgDecHandle handle, JpgDecParam *param)
{
	JpgInst * pJpgInst;
	JpgDecInfo * pDecInfo;
	Uint32 rotMir;
	JpgRet ret;
	Uint32 val;

	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;


	pJpgInst = handle;
	pDecInfo = &pJpgInst->JpgInfo.decInfo;

	if (pDecInfo->frameBufPool == 0) { // This means frame buffers have not been registered.
		return JPG_RET_WRONG_CALL_SEQUENCE;
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
	JpgEnterLock();

	if (GetJpgPendingInst()) {
		JpgLeaveLock();
		return JPG_RET_FRAME_NOT_COMPLETE;
	}

	if (pDecInfo->streamEndflag == 1) {
		val = JpuReadReg(MJPEG_BBC_STRM_CTRL_REG);
		if ((val & (1 << 31)) == 0 ) {
			val = (pDecInfo->streamWrPtr-pDecInfo->streamBufStartAddr) / 256;
			if ((pDecInfo->streamWrPtr-pDecInfo->streamBufStartAddr) % 256)
				val = val + 1;

			JpuWriteReg(MJPEG_BBC_STRM_CTRL_REG, (1 << 31 | val));
		}
	}

#ifdef MJPEG_ERROR_CONCEAL
	// Error Concealment
	if(pDecInfo->errInfo.bError)
	{
		// error conceal main function
		val = JpegDecodeConcealError(pDecInfo);
		if(val == -1)
		{
			// stream buffer wrap around in error cases.
			pDecInfo->frameOffset = 0;
			pDecInfo->nextOffset = 0;

			// end of stream
			if (pDecInfo->streamEndflag == 1) {
				pDecInfo->frameOffset = -1;
				SetJpgPendingInst(pJpgInst);
				return JPG_RET_EOS;

			}
			// request data
			JpgLeaveLock();
			return JPG_RET_BIT_EMPTY;
		}

		// init GBU
		JpuWriteReg(MJPEG_GBU_TT_CNT_REG, 0);
		JpuWriteReg(MJPEG_GBU_TT_CNT_REG+4, 0);

		JpuWriteReg(MJPEG_PIC_CTRL_REG, pDecInfo->huffAcIdx << 10 | pDecInfo->huffDcIdx << 7 | pDecInfo->userHuffTab << 6 | (JPU_CHECK_WRITE_RESPONSE_BVALID_SIGNAL << 2) | 0);
		JpuWriteReg(MJPEG_MCU_INFO_REG, pDecInfo->mcuBlockNum << 16 | pDecInfo->compNum << 12 | pDecInfo->compInfo[0] << 8 | pDecInfo->compInfo[1] << 4 | pDecInfo->compInfo[2]);
		JpuWriteReg(MJPEG_RST_INTVAL_REG, pDecInfo->rstIntval);

		JpgDecGramSetup(pDecInfo);

		JpuWriteReg(MJPEG_DPCM_DIFF_Y_REG, 0);
		JpuWriteReg(MJPEG_DPCM_DIFF_CB_REG, 0);
		JpuWriteReg(MJPEG_DPCM_DIFF_CR_REG, 0);

		JpuWriteReg(MJPEG_GBU_FF_RPTR_REG, pDecInfo->bitPtr);
		JpuWriteReg(MJPEG_GBU_CTRL_REG, 3);

		val = (pDecInfo->setPosX << 16) | (pDecInfo->setPosY);
		JpuWriteReg(MJPEG_PIC_SETMB_REG, val);
#ifdef SUPPORT_ERROR_CONCEAL_WORK_AROUND
		JpuWriteReg(MJPEG_RST_INTVAL_REG, pDecInfo->rstIntval + 1); // RST index for error concealment workaround code
#endif
		JpuWriteReg(MJPEG_PIC_START_REG, (1<<JPG_START_PIC));

		SetJpgPendingInst(pJpgInst);
		return JPG_RET_SUCCESS;
	}
#endif

	if (pDecInfo->frameOffset < 0) {
		SetJpgPendingInst(pJpgInst);
		return JPG_RET_EOS;
	}

	val = JpegDecodeHeader(pDecInfo);
	if (val == 0) {
		JpgLeaveLock();
		return JPG_RET_FAILURE;
	}

	if (val == -2) {	// wrap around case
		pDecInfo->frameOffset = 0;
		pDecInfo->ecsPtr = 0;
#ifdef MJPEG_ERROR_CONCEAL
		pDecInfo->nextOffset = 0;
#endif
		val = JpegDecodeHeader(pDecInfo);
		if (val == 0) {
			JpgLeaveLock();
			return JPG_RET_FAILURE;
		}
	}

	if (val == -1) {	//stream empty case
		if (pDecInfo->streamEndflag == 1) {
			SetJpgPendingInst(pJpgInst);
			pDecInfo->frameOffset = -1;
			return JPG_RET_EOS;
		}
		JpgLeaveLock();
		return JPG_RET_BIT_EMPTY;
	}

	if (pDecInfo->streamRdPtr == pDecInfo->streamBufEndAddr) {
		JpuWriteReg(MJPEG_BBC_CUR_POS_REG, 0);
		JpuWriteReg(MJPEG_GBU_TT_CNT_REG, 0);
		JpuWriteReg(MJPEG_GBU_TT_CNT_REG+4, 0);
	}

	JpuWriteReg(MJPEG_BBC_WR_PTR_REG, pDecInfo->streamWrPtr);
	if (pDecInfo->streamWrPtr == pDecInfo->streamBufStartAddr) {
		JpuWriteReg(MJPEG_BBC_END_ADDR_REG, pDecInfo->streamBufEndAddr);
	}
	else {
		JpuWriteReg(MJPEG_BBC_END_ADDR_REG, pDecInfo->streamWrPtr);
	}

	JpuWriteReg(MJPEG_BBC_BAS_ADDR_REG, pDecInfo->streamBufStartAddr);
	//JpuWriteReg(MJPEG_BBC_END_ADDR_REG, pDecInfo->streamWrPtr);
	JpuWriteReg(MJPEG_BBC_STRM_CTRL_REG, 0);
	JpuWriteReg(MJPEG_GBU_TT_CNT_REG, 0);
	JpuWriteReg(MJPEG_GBU_TT_CNT_REG+4, 0);
	JpuWriteReg(MJPEG_PIC_ERRMB_REG, 0);
	JpuWriteReg(MJPEG_PIC_CTRL_REG, pDecInfo->huffAcIdx << 10 | pDecInfo->huffDcIdx << 7 | pDecInfo->userHuffTab << 6 | (JPU_CHECK_WRITE_RESPONSE_BVALID_SIGNAL << 2) | pDecInfo->usePartial);

	JpuWriteReg(MJPEG_PIC_SIZE_REG, (pDecInfo->alignedWidth << 16) | pDecInfo->alignedHeight);
	JpuWriteReg(MJPEG_ROT_INFO_REG, 0);
	JpuWriteReg(MJPEG_OP_INFO_REG,  pDecInfo->lineNum << 16| pDecInfo->bufNum << 3 |pDecInfo->busReqNum);
		JpuWriteReg(MJPEG_MCU_INFO_REG, pDecInfo->mcuBlockNum << 16 | pDecInfo->compNum << 12 | pDecInfo->compInfo[0] << 8 | pDecInfo->compInfo[1] << 4 | pDecInfo->compInfo[2]);

	JpuWriteReg(MJPEG_DPB_CONFIG_REG, (pDecInfo->frameEndian << 1) | pDecInfo->chromaInterleave);
	JpuWriteReg(MJPEG_RST_INTVAL_REG, pDecInfo->rstIntval);

	if (param) {
		if (param->scaleDownRatioWidth > 0 )
			pDecInfo->iHorScaleMode = param->scaleDownRatioWidth;
		if (param->scaleDownRatioHeight > 0 )
			pDecInfo->iVerScaleMode = param->scaleDownRatioHeight;
	}
	if (pDecInfo->iHorScaleMode | pDecInfo->iVerScaleMode)
		val = ((pDecInfo->iHorScaleMode&0x3)<<2) | ((pDecInfo->iVerScaleMode&0x3)) | 0x10 ;
	else {
		val = 0;
	}
	JpuWriteReg(MJPEG_SCL_INFO_REG, val);

	if (pDecInfo->userHuffTab) {
		if (!JpgDecHuffTabSetUp(pDecInfo)) {
			JpgLeaveLock();
			return JPG_RET_INVALID_PARAM;
		}
	}

	if (!JpgDecQMatTabSetUp(pDecInfo)) {
		JpgLeaveLock();
		return JPG_RET_INVALID_PARAM;
	}

	JpgDecGramSetup(pDecInfo);

	JpuWriteReg(MJPEG_RST_INDEX_REG, 0);	// RST index at the beginning.
	JpuWriteReg(MJPEG_RST_COUNT_REG, 0);

	JpuWriteReg(MJPEG_DPCM_DIFF_Y_REG, 0);
	JpuWriteReg(MJPEG_DPCM_DIFF_CB_REG, 0);
	JpuWriteReg(MJPEG_DPCM_DIFF_CR_REG, 0);

	JpuWriteReg(MJPEG_GBU_FF_RPTR_REG, pDecInfo->bitPtr);
	JpuWriteReg(MJPEG_GBU_CTRL_REG, 3);

	JpuWriteReg(MJPEG_ROT_INFO_REG, rotMir);

	if (rotMir & 1) {
		pDecInfo->format = (pDecInfo->format==FORMAT_422) ? FORMAT_224 : (pDecInfo->format==FORMAT_224) ? FORMAT_422 : pDecInfo->format;
	}

	if (rotMir & 0x10) {
		JpuWriteReg(MJPEG_DPB_BASE00_REG, pDecInfo->rotatorOutput.bufY);
		JpuWriteReg(MJPEG_DPB_BASE01_REG, pDecInfo->rotatorOutput.bufCb);
		JpuWriteReg(MJPEG_DPB_BASE02_REG, pDecInfo->rotatorOutput.bufCr);
	}
	if (param->dpb_used_flag) {
		pDecInfo->dpb_user_flag = 1;
		pDecInfo->dpb_index = param->dpb_index;
		JpuWriteReg(MJPEG_DPB_BASE00_REG, pDecInfo->frameBufPool[param->dpb_index].bufY);
		JpuWriteReg(MJPEG_DPB_BASE01_REG, pDecInfo->frameBufPool[param->dpb_index].bufCb);
		JpuWriteReg(MJPEG_DPB_BASE02_REG, pDecInfo->frameBufPool[param->dpb_index].bufCr);
	} else {
		val = (pDecInfo->frameIdx%pDecInfo->numFrameBuffers);
		JpuWriteReg(MJPEG_DPB_BASE00_REG, pDecInfo->frameBufPool[val].bufY);
		JpuWriteReg(MJPEG_DPB_BASE01_REG, pDecInfo->frameBufPool[val].bufCb);
		JpuWriteReg(MJPEG_DPB_BASE02_REG, pDecInfo->frameBufPool[val].bufCr);
	}

	if(pDecInfo->rotationEnable) {
		JpuWriteReg(MJPEG_DPB_YSTRIDE_REG, pDecInfo->rotatorStride);
		val = (pDecInfo->format == FORMAT_420 || pDecInfo->format == FORMAT_422 || pDecInfo->format == FORMAT_400) ? 2 : 1;
		if (pDecInfo->chromaInterleave)
			JpuWriteReg(MJPEG_DPB_CSTRIDE_REG, (pDecInfo->rotatorStride/(int)val)*2);
		else
			JpuWriteReg(MJPEG_DPB_CSTRIDE_REG, pDecInfo->rotatorStride/(int)val);
	}
	else {
		JpuWriteReg(MJPEG_DPB_YSTRIDE_REG, pDecInfo->stride);
		val = (pDecInfo->format == FORMAT_420 || pDecInfo->format == FORMAT_422 || pDecInfo->format == FORMAT_400) ? 2 : 1;
		if (pDecInfo->chromaInterleave)
			JpuWriteReg(MJPEG_DPB_CSTRIDE_REG, (pDecInfo->stride/(int)val)*2);
		else
			JpuWriteReg(MJPEG_DPB_CSTRIDE_REG, pDecInfo->stride/(int)val);
	}
	JpuWriteReg(MJPEG_CLP_INFO_REG, 0);

	if (pJpgInst->loggingEnable)
		jdi_log(JDI_LOG_CMD_PICRUN, 1);

	JpuWriteReg(MJPEG_PIC_STATUS_REG, JpuReadReg(MJPEG_PIC_STATUS_REG));
	JpuWriteReg(MJPEG_PIC_START_REG, (1<<JPG_START_PIC));


	SetJpgPendingInst(pJpgInst);
	return JPG_RET_SUCCESS;
}

JpgRet JPU_DecGetOutputInfo(JpgDecHandle handle, JpgDecOutputInfo * info)
{
	JpgInst * pJpgInst;
	JpgDecInfo	  *	pDecInfo;
	JpgRet		ret;
	Uint32		val = 0;

	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS) {
		SetJpgPendingInst(0);
		JpgLeaveLock();
		return ret;
	}

	if (info == 0) {
		SetJpgPendingInst(0);
		JpgLeaveLock();
		return JPG_RET_INVALID_PARAM;
	}

	pJpgInst = handle;
	pDecInfo = &pJpgInst->JpgInfo.decInfo;


    val = JpuReadReg(MJPEG_PIC_STATUS_REG);

	if (pJpgInst != GetJpgPendingInst()) {
		SetJpgPendingInst(0);
		JpgLeaveLock();
		return JPG_RET_WRONG_CALL_SEQUENCE;
	}

	if (pDecInfo->frameOffset < 0)
	{
        info->numOfErrMBs = 0;
		info->decodingSuccess = 1;
		info->indexFrameDisplay = -1;
		SetJpgPendingInst(0);
		JpgLeaveLock();
		return JPG_RET_SUCCESS;
	}

	info->decPicWidth = pDecInfo->alignedWidth;
	info->decPicHeight = pDecInfo->alignedHeight;

    info->decPicWidth  >>= pDecInfo->iHorScaleMode;
    info->decPicHeight >>= pDecInfo->iVerScaleMode;

	if (pDecInfo->dpb_user_flag) {
		info->indexFrameDisplay = pDecInfo->dpb_index;
	}
	else {
		info->indexFrameDisplay = (pDecInfo->frameIdx%pDecInfo->numFrameBuffers);
	}

#ifdef MJPEG_ERROR_CONCEAL
	info->consumedByte = pDecInfo->gbuStartPtr + (JpuReadReg(MJPEG_GBU_TT_CNT_REG))/8;
#else
	info->consumedByte = (JpuReadReg(MJPEG_GBU_TT_CNT_REG))/8;
#endif
	pDecInfo->streamRdPtr = JpuReadReg(MJPEG_BBC_RD_PTR_REG);
	pDecInfo->consumeByte = info->consumedByte - 16 - pDecInfo->ecsPtr;
	info->bytePosFrameStart = pDecInfo->frameOffset;
	info->ecsPtr = pDecInfo->ecsPtr;

	pDecInfo->ecsPtr = 0;
	pDecInfo->frameIdx++;

	val = JpuReadReg(MJPEG_PIC_STATUS_REG);
	if (val & (1<<INT_JPU_DONE)) {
		info->decodingSuccess = 1;
        info->numOfErrMBs = 0;

#ifdef MJPEG_ERROR_CONCEAL
        pDecInfo->errInfo.bError = 0;
        pDecInfo->nextOffset = 0;
        pDecInfo->gbuStartPtr = 0;
#endif
	}
	else if (val & (1<<INT_JPU_ERROR)){

        info->numOfErrMBs = JpuReadReg(MJPEG_PIC_ERRMB_REG);
		info->decodingSuccess = 0;

#ifdef MJPEG_ERROR_CONCEAL
//			info->numOfErrMBs = JpuReadReg(MJPEG_PIC_ERRMB_REG);
        pDecInfo->errInfo.bError  = 1;
        pDecInfo->errInfo.errPosY = info->numOfErrMBs & 0xFFF;
        pDecInfo->errInfo.errPosX = (info->numOfErrMBs >> 12) & 0xFFF;

        // set search point to find next rstMarker from origin of frame buffer by host
        pDecInfo->nextOffset  = (info->consumedByte) & (~7);

        // prevent to find same position.
        if(pDecInfo->currOffset == pDecInfo->nextOffset)
            pDecInfo->nextOffset += JPU_GBU_SIZE;
#endif
    }

	if (val != 0)
    	JpuWriteReg(MJPEG_PIC_STATUS_REG, val&0xF);

	if (pJpgInst->loggingEnable)
		jdi_log(JDI_LOG_CMD_PICRUN, 0);

   	SetJpgPendingInst(0);
    JpgLeaveLock();

	return JPG_RET_SUCCESS;
}

JpgRet JPU_DecSetRdPtr(JpgDecHandle handle, PhysicalAddress addr, int updateWrPtr)
{
    JpgInst * pJpgInst;
    JpgDecInfo * pDecInfo;
    JpgRet ret;

	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;


    pJpgInst = handle;
    pDecInfo = &pJpgInst->JpgInfo.decInfo;


    JpgEnterLock();

    if (GetJpgPendingInst()) {
           JpgLeaveLock();
           return JPG_RET_FRAME_NOT_COMPLETE;
    }

    pDecInfo->streamRdPtr = addr;
    if (updateWrPtr)
           pDecInfo->streamWrPtr = addr;

    pDecInfo->frameOffset = addr - pDecInfo->streamBufStartAddr;
	pDecInfo->consumeByte = 0;
	pDecInfo->headerSize = 0;
	
    JpuWriteReg(MJPEG_BBC_RD_PTR_REG, pDecInfo->streamRdPtr);
	if (updateWrPtr)
		JpuWriteReg(MJPEG_BBC_WR_PTR_REG, pDecInfo->streamWrPtr);

    if (pDecInfo->streamRdPtr == pDecInfo->streamBufStartAddr) {                                                                                                                                                            
		JpuWriteReg(MJPEG_BBC_CUR_POS_REG, 0);
		JpuWriteReg(MJPEG_GBU_TT_CNT_REG, 0);
		JpuWriteReg(MJPEG_GBU_TT_CNT_REG+4, 0);
    }

    JpgLeaveLock();
	
//	JPU_DecGiveCommand(handle, ENABLE_LOGGING, NULL);

    return JPG_RET_SUCCESS;
}    


JpgRet JPU_DecGiveCommand(
						  JpgDecHandle handle,
						  JpgCommand cmd,
						  void * param)
{
	JpgInst * pJpgInst;
	JpgDecInfo * pDecInfo;
	JpgRet ret;

	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;


	pJpgInst = handle;
	pDecInfo = &pJpgInst->JpgInfo.decInfo;
	switch (cmd)
	{
	case ENABLE_JPG_ROTATION :
		{
			if (pDecInfo->rotatorStride == 0) {
				return JPG_RET_ROTATOR_STRIDE_NOT_SET;
			}
			pDecInfo->rotationEnable = 1;
			break;
		}

	case DISABLE_JPG_ROTATION :
		{
			pDecInfo->rotationEnable = 0;
			break;
		}

	case ENABLE_JPG_MIRRORING :
		{
			if (pDecInfo->rotatorStride == 0) {
				return JPG_RET_ROTATOR_STRIDE_NOT_SET;
			}
			pDecInfo->mirrorEnable = 1;
			break;
		}
	case DISABLE_JPG_MIRRORING :
		{
			pDecInfo->mirrorEnable = 0;
			break;
		}
	case SET_JPG_MIRROR_DIRECTION :
		{

			JpgMirrorDirection mirDir;

			if (param == 0) {
				return JPG_RET_INVALID_PARAM;
			}
			mirDir = *(JpgMirrorDirection *)param;
			if (!(MIRDIR_NONE <= mirDir && mirDir <= MIRDIR_HOR_VER)) {
				return JPG_RET_INVALID_PARAM;
			}
			pDecInfo->mirrorDirection = mirDir;

			break;
		}
	case SET_JPG_ROTATION_ANGLE :
		{
			int angle;

			if (param == 0) {
				return JPG_RET_INVALID_PARAM;
			}

			angle = *(int *)param;
			if (angle != 0 && angle != 90 &&
				angle != 180 && angle != 270) {
					return JPG_RET_INVALID_PARAM;
			}

			pDecInfo->rotationAngle = angle;
			break;
		}

	case SET_JPG_ROTATOR_OUTPUT :
		{
			FrameBuffer * frame;

			if (param == 0) {
				return JPG_RET_INVALID_PARAM;
			}
			frame = (FrameBuffer *)param;
			pDecInfo->rotatorOutput = *frame;
			pDecInfo->rotatorOutputValid = 1;
			break;
		}

	case SET_JPG_ROTATOR_STRIDE :
		{
			int stride;

			if (param == 0) {
				return JPG_RET_INVALID_PARAM;
			}
			stride = *(int *)param;
			if (stride % 8 != 0 || stride == 0) {
				return JPG_RET_INVALID_STRIDE;
			}

			if (pDecInfo->rotationAngle == 90 || pDecInfo->rotationAngle == 270) {
				if (pDecInfo->alignedHeight > stride) {
					return JPG_RET_INVALID_STRIDE;
				}
			} else {
				if (pDecInfo->alignedWidth > stride) {
					return JPG_RET_INVALID_STRIDE;
				}
			}


			pDecInfo->rotatorStride = stride;
			break;
		}
    case SET_JPG_SCALE_HOR:
        {
            int scale;
            scale = *(int *)param;
			if (pDecInfo->alignedWidth < 128 || pDecInfo->alignedHeight < 128) {
				if (scale) {
					return JPG_RET_INVALID_PARAM;
				}
			}

            pDecInfo->iHorScaleMode = scale;
            break;
        }
    case SET_JPG_SCALE_VER:
        {
            int scale;
            scale = *(int *)param;
			if (pDecInfo->alignedWidth < 128 || pDecInfo->alignedHeight < 128) {
				if (scale) {
					return JPG_RET_INVALID_PARAM;
				}
			}
            pDecInfo->iVerScaleMode = scale;
            break;
        }
	case ENABLE_LOGGING:
		{
			pJpgInst->loggingEnable = 1;
		}
		break;
	case DISABLE_LOGGING:
		{
			pJpgInst->loggingEnable = 0;
		}
		break;
	default:
		return JPG_RET_INVALID_COMMAND;
	}
	return JPG_RET_SUCCESS;
}

JpgRet JPU_EncOpen(JpgEncHandle * pHandle, JpgEncOpenParam * pop)
{
	JpgInst * pJpgInst;
	JpgEncInfo * pEncInfo;
	JpgRet ret;
	int i;

	ret = CheckJpgEncOpenParam(pop);
	if (ret != JPG_RET_SUCCESS) {
		return ret;
	}

	JpgEnterLock();
	ret = GetJpgInstance(&pJpgInst);
	if (ret == JPG_RET_FAILURE) {
		*pHandle = 0;
		JpgLeaveLock();
		return JPG_RET_FAILURE;
	}

	*pHandle = pJpgInst;
	pEncInfo = &pJpgInst->JpgInfo.encInfo;
	jdi_memset(pEncInfo, 0x00, sizeof(JpgEncInfo));
    pEncInfo->openParam = *pop;
	pEncInfo->streamRdPtr = pop->bitstreamBuffer;
	pEncInfo->streamWrPtr = pop->bitstreamBuffer;

	JpuWriteReg(MJPEG_BBC_WR_PTR_REG, pEncInfo->streamWrPtr);
	pEncInfo->streamBufStartAddr = pop->bitstreamBuffer;
	pEncInfo->streamBufSize = pop->bitstreamBufferSize;
	pEncInfo->streamBufEndAddr = pop->bitstreamBuffer + pop->bitstreamBufferSize;
	pEncInfo->streamEndian = pop->streamEndian;
	pEncInfo->frameEndian = pop->frameEndian;
	pEncInfo->chromaInterleave = pop->chromaInterleave;

	pEncInfo->format = pEncInfo->openParam.sourceFormat;
	pEncInfo->picWidth= pEncInfo->openParam.picWidth;
	pEncInfo->picHeight = pEncInfo->openParam.picHeight;
	// Picture size alignment
	if (pEncInfo->format == FORMAT_420 || pEncInfo->format == FORMAT_422)
		pEncInfo->alignedWidth = ((pEncInfo->picWidth+15)/16)*16;
	else
		pEncInfo->alignedWidth = ((pEncInfo->picWidth+7)/8)*8;

	if (pEncInfo->format == FORMAT_420 || pEncInfo->format == FORMAT_224)
		pEncInfo->alignedHeight = ((pEncInfo->picHeight+15)/16)*16;
	else
		pEncInfo->alignedHeight = ((pEncInfo->picHeight+7)/8)*8;
	pEncInfo->rstIntval = pEncInfo->openParam.restartInterval;

	for (i=0; i<4; i++)
		pEncInfo->pHuffVal[i] = pEncInfo->openParam.huffVal[i];
	for (i=0; i<4; i++)
		pEncInfo->pHuffBits[i] = pEncInfo->openParam.huffBits[i];
	for (i=0; i<4; i++)
		pEncInfo->pQMatTab[i] = pEncInfo->openParam.qMatTab[i];

	pEncInfo->pCInfoTab[0] = sJpuCompInfoTable[2/*pEncInfo->format*/];
	pEncInfo->pCInfoTab[1] = pEncInfo->pCInfoTab[0] + 6;
	pEncInfo->pCInfoTab[2] = pEncInfo->pCInfoTab[1] + 6;
	pEncInfo->pCInfoTab[3] = pEncInfo->pCInfoTab[2] + 6;


	JpgLeaveLock();

    return JPG_RET_SUCCESS;

}

JpgRet JPU_EncClose(JpgEncHandle handle)
{
	JpgInst * pJpgInst;
	JpgRet ret;

	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;

	JpgEnterLock();

	if (GetJpgPendingInst()) {
		JpgLeaveLock();
		return JPG_RET_FRAME_NOT_COMPLETE;
	}

	pJpgInst = handle;
	JpuWriteReg(MJPEG_BBC_FLUSH_CMD_REG, 0);
	FreeJpgInstance(pJpgInst);
	JpgLeaveLock();

	return JPG_RET_SUCCESS;
}


JpgRet JPU_EncGetInitialInfo(JpgEncHandle handle, JpgEncInitialInfo * info)
{
	JpgInst * pJpgInst;
	JpgEncInfo * pEncInfo;
	JpgRet ret;

	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;

	if (info == 0) {
		return JPG_RET_INVALID_PARAM;
	}

	pJpgInst = handle;
	pEncInfo = &pJpgInst->JpgInfo.encInfo;

	if (pEncInfo->initialInfoObtained) {
		return JPG_RET_CALLED_BEFORE;
	}


	JpgEnterLock();

	if (GetJpgPendingInst()) {
		JpgLeaveLock();
		return JPG_RET_FRAME_NOT_COMPLETE;
	}

	if (pEncInfo->format == FORMAT_400) {
		pEncInfo->compInfo[1] = 0;
		pEncInfo->compInfo[2] = 0;
	}
	else {
		pEncInfo->compInfo[1] = 5;
		pEncInfo->compInfo[2] = 5;
	}

	if (pEncInfo->format == FORMAT_400)
		pEncInfo->compNum = 1;
	else
		pEncInfo->compNum = 3;

	if (pEncInfo->format == FORMAT_420) {
		pEncInfo->mcuBlockNum = 6;
		pEncInfo->compInfo[0] = 10;
		pEncInfo->busReqNum = 2;
	}
	else if (pEncInfo->format == FORMAT_422) {
		pEncInfo->mcuBlockNum = 4;
		pEncInfo->busReqNum = 3;
		pEncInfo->compInfo[0] = 9;
	} else if (pEncInfo->format == FORMAT_224) {
		pEncInfo->mcuBlockNum = 4;
		pEncInfo->busReqNum  = 3;
		pEncInfo->compInfo[0] = 6;
	} else if (pEncInfo->format == FORMAT_444) {
		pEncInfo->mcuBlockNum = 3;
		pEncInfo->compInfo[0] = 5;
		pEncInfo->busReqNum = 4;
	} else if (pEncInfo->format == FORMAT_400) {
		pEncInfo->mcuBlockNum = 1;
		pEncInfo->busReqNum = 4;
		pEncInfo->compInfo[0] = 5;
	}

	info->minFrameBufferCount = 0;
	info->colorComponents = pEncInfo->compNum;

	pEncInfo->initialInfo = *info;
	pEncInfo->initialInfoObtained = 1;

    JpgLeaveLock();
	return JPG_RET_SUCCESS;
}



JpgRet JPU_EncGetBitstreamBuffer( JpgEncHandle handle,
								 PhysicalAddress * prdPtr,
								 PhysicalAddress * pwrPtr,
								 int * size)
{
	JpgInst * pJpgInst;
	JpgEncInfo * pEncInfo;
	JpgRet ret;
	int room;

	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;

	if ( prdPtr == 0 || pwrPtr == 0 || size == 0) {
		return JPG_RET_INVALID_PARAM;
	}

	pJpgInst = handle;
	pEncInfo = &pJpgInst->JpgInfo.encInfo;

	JpgSetClockGate(1);
	*prdPtr = pEncInfo->streamRdPtr;
	if (GetJpgPendingInst() == pJpgInst) {
		*pwrPtr = JpuReadReg(MJPEG_BBC_WR_PTR_REG);
	}
	else {
		*pwrPtr = pEncInfo->streamWrPtr;
	}

	if (*pwrPtr > pEncInfo->streamBufEndAddr) {
		room = *pwrPtr - pEncInfo->streamBufEndAddr;
		*pwrPtr = pEncInfo->streamBufStartAddr;
		*pwrPtr += room;
		pEncInfo->streamWrPtr = *pwrPtr;
		JpuWriteReg(MJPEG_BBC_WR_PTR_REG, pEncInfo->streamWrPtr);
	}

	if (*pwrPtr == pEncInfo->streamBufEndAddr) {
		pEncInfo->streamWrPtr = pEncInfo->streamBufStartAddr;
		JpuWriteReg(MJPEG_BBC_WR_PTR_REG, pEncInfo->streamWrPtr);
	}

	if (*pwrPtr >= *prdPtr) {
		*size = *pwrPtr - *prdPtr;
	} else {
		*size = (pEncInfo->streamBufEndAddr - *prdPtr) + (*pwrPtr - pEncInfo->streamBufStartAddr);
	}

	JpgSetClockGate(0);

	return JPG_RET_SUCCESS;

}

JpgRet JPU_EncUpdateBitstreamBuffer(
									JpgEncHandle handle,
									int size)
{
	JpgInst * pJpgInst;
	JpgEncInfo * pEncInfo;
	JpgRet ret;
	int room = 0;


	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;

	pJpgInst = handle;
	pEncInfo = &pJpgInst->JpgInfo.encInfo;


	JpgSetClockGate(1);

#if 1

	if (pEncInfo->streamBufEndAddr <= (unsigned int)size) {
		pEncInfo->streamRdPtr = pEncInfo->streamBufStartAddr;
		room = 0;
	}
	else {
		pEncInfo->streamRdPtr = (unsigned int)size;
		room = pEncInfo->streamRdPtr - pEncInfo->streamBufStartAddr;
	}
	
	if (GetJpgPendingInst() == pJpgInst) {
		JpuWriteReg(MJPEG_BBC_CUR_POS_REG, room);
		JpuWriteReg(MJPEG_BBC_EXT_ADDR_REG, pEncInfo->streamRdPtr);
		JpuWriteReg(MJPEG_BBC_RD_PTR_REG, pEncInfo->streamRdPtr);
		JpuWriteReg(MJPEG_BBC_WR_PTR_REG, pEncInfo->streamWrPtr);
	}
#else

	/* cnm original source */
	rdPtr = pEncInfo->streamRdPtr;
	rdPtr += size;

	if (rdPtr == pEncInfo->streamBufEndAddr) {
		rdPtr = pEncInfo->streamBufStartAddr;
	}

	pEncInfo->streamRdPtr = pEncInfo->streamBufStartAddr;

	if (GetJpgPendingInst() == pJpgInst) {
		pEncInfo->streamWrPtr = JpuReadReg(MJPEG_BBC_WR_PTR_REG);
		JpuWriteReg(MJPEG_BBC_CUR_POS_REG, 0);
		JpuWriteReg(MJPEG_BBC_EXT_ADDR_REG, pEncInfo->streamBufStartAddr);
		JpuWriteReg(MJPEG_BBC_RD_PTR_REG, pEncInfo->streamBufStartAddr);
		JpuWriteReg(MJPEG_BBC_WR_PTR_REG, pEncInfo->streamBufStartAddr);
	}
#endif
	
	JpgSetClockGate(0);
	return JPG_RET_SUCCESS;


}
#ifdef SUPPORT_STOP_COMMAND
JpgRet JPU_EncIssueStop(JpgEncHandle handle)
{
	JpgInst * pJpgInst;
	JpgEncInfo *pEncInfo;
	JpgRet ret;


	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;

	pJpgInst = handle;
	pEncInfo = &pJpgInst->JpgInfo.encInfo;

	if (pJpgInst != GetJpgPendingInst()) {
		return JPG_RET_WRONG_CALL_SEQUENCE;
	}

	JpgSetClockGate(1);
	JpuWriteReg(MJPEG_PIC_START_REG, 1<< JPG_START_STOP);
	JpgSetClockGate(0);
	return JPG_RET_SUCCESS;

}
JpgRet JPU_EncCompleteStop(JpgEncHandle handle)
{
	JpgInst * pJpgInst;
	JpgEncInfo *pEncInfo;
	JpgRet ret;
	Uint32 val;

	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;

	pJpgInst = handle;
	pEncInfo = &pJpgInst->JpgInfo.encInfo;

	if (pJpgInst != GetJpgPendingInst()) {
		return JPG_RET_WRONG_CALL_SEQUENCE;
	}

	JpgSetClockGate(1);
	val = JpuReadReg(MJPEG_PIC_STATUS_REG);

	if (val & (1<<INT_JPU_BIT_BUF_STOP)) {
		SetJpgPendingInst(0);
		JpgSetClockGate(0);
	}
	else {
		JpgSetClockGate(0);
		return JPG_RET_WRONG_CALL_SEQUENCE;
	}

	return JPG_RET_SUCCESS;
}
#endif

JpgRet JPU_EncStartOneFrame( JpgEncHandle handle, JpgEncParam * param )
{
	JpgInst * pJpgInst;
	JpgEncInfo * pEncInfo;
	FrameBuffer * pBasFrame;
	Uint32 rotMirEnable;
	Uint32 rotMirMode;
	JpgRet ret;
	Uint32 val;
    int room;

	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;


	pJpgInst = handle;
	pEncInfo = &pJpgInst->JpgInfo.encInfo;


	ret = CheckJpgEncParam(handle, param);
	if (ret != JPG_RET_SUCCESS) {
		return ret;
	}


	pBasFrame = param->sourceFrame;
	rotMirEnable = 0;
	rotMirMode = 0;
	if (pEncInfo->rotationEnable) {
		rotMirEnable = 0x10; // Enable rotator
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
		rotMirEnable = 0x10; // Enable rotator

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

	JpgEnterLock();

	if (GetJpgPendingInst()) {
		JpgLeaveLock();
		return JPG_RET_FRAME_NOT_COMPLETE;
	}
	JpuWriteReg(MJPEG_CLP_INFO_REG, 0);	//off ROI enable due to not supported feature for encoder.
    // JPU Init should be needed
	room = pEncInfo->streamWrPtr - pEncInfo->streamBufStartAddr;

	JpuWriteReg(MJPEG_BBC_BAS_ADDR_REG, pEncInfo->streamBufStartAddr + room);
	JpuWriteReg(MJPEG_BBC_END_ADDR_REG, pEncInfo->streamBufEndAddr);
	JpuWriteReg(MJPEG_BBC_WR_PTR_REG, pEncInfo->streamWrPtr);
	JpuWriteReg(MJPEG_BBC_RD_PTR_REG, pEncInfo->streamRdPtr);
	JpuWriteReg(MJPEG_BBC_CUR_POS_REG, 0);
	JpuWriteReg(MJPEG_BBC_DATA_CNT_REG, 256 / 4);	// 64 * 4 byte == 32 * 8 byte
	JpuWriteReg(MJPEG_BBC_EXT_ADDR_REG, pEncInfo->streamBufStartAddr + room);
	JpuWriteReg(MJPEG_BBC_INT_ADDR_REG, 0);

	JpuWriteReg(MJPEG_GBU_BT_PTR_REG, 0);
	JpuWriteReg(MJPEG_GBU_WD_PTR_REG, 0);
	JpuWriteReg(MJPEG_GBU_BBSR_REG, 0);
    JpuWriteReg(MJPEG_GBU_CTRL_REG, 0);

	JpuWriteReg(MJPEG_GBU_BBER_REG, ((256 / 4) * 2) - 1);
	JpuWriteReg(MJPEG_GBU_BBIR_REG, 256 / 4);	// 64 * 4 byte == 32 * 8 byte
	JpuWriteReg(MJPEG_GBU_BBHR_REG, 256 / 4);	// 64 * 4 byte == 32 * 8 byte
	JpuWriteReg(MJPEG_PIC_CTRL_REG, 0x18 | (JPU_CHECK_WRITE_RESPONSE_BVALID_SIGNAL<<2));
	JpuWriteReg(MJPEG_SCL_INFO_REG, 0);
	JpuWriteReg(MJPEG_DPB_CONFIG_REG, (pEncInfo->frameEndian << 1) | pEncInfo->chromaInterleave);

	JpuWriteReg(MJPEG_RST_INTVAL_REG, pEncInfo->rstIntval);
	JpuWriteReg(MJPEG_BBC_CTRL_REG, (pEncInfo->streamEndian << 1) | 1);
    JpuWriteReg(MJPEG_OP_INFO_REG, pEncInfo->busReqNum);

    // Load HUFFTab
	if (!JpgEncLoadHuffTab(pEncInfo)) {
		JpgLeaveLock();
		return JPG_RET_INVALID_PARAM;
	}

    // Load QMATTab
	if (!JpgEncLoadQMatTab(pEncInfo)) {
		JpgLeaveLock();
		return JPG_RET_INVALID_PARAM;
	}
    //although rotator is enable, this picture size must not be changed from widh to height.
    JpuWriteReg(MJPEG_PIC_SIZE_REG, pEncInfo->alignedWidth<<16 | pEncInfo->alignedHeight);
	JpuWriteReg(MJPEG_ROT_INFO_REG, (rotMirEnable|rotMirMode));

	JpuWriteReg(MJPEG_MCU_INFO_REG, pEncInfo->mcuBlockNum << 16 | pEncInfo->compNum << 12
		| pEncInfo->compInfo[0] << 8 | pEncInfo->compInfo[1] << 4 | pEncInfo->compInfo[2]);

    //JpgEncGbuResetReg
    JpuWriteReg(MJPEG_GBU_CTRL_REG, 0);

	    JpuWriteReg(MJPEG_DPB_BASE00_REG, pBasFrame->bufY);
	    JpuWriteReg(MJPEG_DPB_BASE01_REG, pBasFrame->bufCb);
	    JpuWriteReg(MJPEG_DPB_BASE02_REG, pBasFrame->bufCr);


	JpuWriteReg(MJPEG_DPB_YSTRIDE_REG, pBasFrame->stride);

	val = (pEncInfo->format == FORMAT_420 || pEncInfo->format == FORMAT_422 || pEncInfo->format == FORMAT_400) ? 2 : 1;

	if (pEncInfo->chromaInterleave)
		JpuWriteReg(MJPEG_DPB_CSTRIDE_REG, (pBasFrame->stride/(int)val)*2);
	else
		JpuWriteReg(MJPEG_DPB_CSTRIDE_REG, pBasFrame->stride/(int)val);


	if (pJpgInst->loggingEnable)
		jdi_log(JDI_LOG_CMD_PICRUN, 1);

	JpuWriteReg(MJPEG_PIC_STATUS_REG, JpuReadReg(MJPEG_PIC_STATUS_REG));
	JpuWriteReg(MJPEG_PIC_START_REG, (1<<JPG_START_PIC));

	SetJpgPendingInst(pJpgInst);
	return JPG_RET_SUCCESS;
}

JpgRet JPU_EncGetOutputInfo(
							JpgEncHandle handle,
							JpgEncOutputInfo * info
							)
{
	JpgInst * pJpgInst;
	JpgEncInfo * pEncInfo;
	Uint32 val;
	JpgRet ret;

	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS) {
		SetJpgPendingInst(0);
		JpgLeaveLock();
		return ret;
	}

	if (info == 0) {
		SetJpgPendingInst(0);
		JpgLeaveLock();
		return JPG_RET_INVALID_PARAM;
	}

	pJpgInst = handle;
	pEncInfo = &pJpgInst->JpgInfo.encInfo;

	if (pJpgInst != GetJpgPendingInst()) {
		SetJpgPendingInst(0);
		JpgLeaveLock();
		return JPG_RET_WRONG_CALL_SEQUENCE;
	}


	val = JpuReadReg(MJPEG_PIC_STATUS_REG);

	if ((val & 0x4) >> 2) {
		SetJpgPendingInst(0);
		JpgLeaveLock();
		return JPG_RET_WRONG_CALL_SEQUENCE;
	}

	if (val != 0)
		JpuWriteReg(MJPEG_PIC_STATUS_REG, val);

	info->bitstreamBuffer = pEncInfo->streamBufStartAddr;
	info->bitstreamSize = JpuReadReg(MJPEG_BBC_WR_PTR_REG) - pEncInfo->streamBufStartAddr;
	pEncInfo->streamWrPtr = JpuReadReg(MJPEG_BBC_WR_PTR_REG);

	JpuWriteReg(MJPEG_BBC_FLUSH_CMD_REG, 0);

	if (pJpgInst->loggingEnable)
		jdi_log(JDI_LOG_CMD_PICRUN, 0);

	SetJpgPendingInst(0);
	JpgLeaveLock();
	return JPG_RET_SUCCESS;

}


JpgRet JPU_EncGiveCommand(
						  JpgEncHandle handle,
						  JpgCommand cmd,
						  void * param)
{
	JpgInst * pJpgInst;
	JpgEncInfo * pEncInfo;
	JpgRet ret;

	ret = CheckJpgInstValidity(handle);
	if (ret != JPG_RET_SUCCESS)
		return ret;


	pJpgInst = handle;
	pEncInfo = &pJpgInst->JpgInfo.encInfo;
	switch (cmd)
	{
	case ENABLE_JPG_ROTATION :
		{
			pEncInfo->rotationEnable = 1;
		}
		break;
	case DISABLE_JPG_ROTATION :
		{
			pEncInfo->rotationEnable = 0;
		}
		break;
	case ENABLE_JPG_MIRRORING :
		{
			pEncInfo->mirrorEnable = 1;
		}
		break;
	case DISABLE_JPG_MIRRORING :
		{
			pEncInfo->mirrorEnable = 0;
		}
		break;
	case SET_JPG_MIRROR_DIRECTION :
		{
			JpgMirrorDirection mirDir;

			if (param == 0) {
				return JPG_RET_INVALID_PARAM;
			}
			mirDir = *(JpgMirrorDirection *)param;
			if (!(MIRDIR_NONE <= mirDir && mirDir <= MIRDIR_HOR_VER)) {
				return JPG_RET_INVALID_PARAM;
			}
			pEncInfo->mirrorDirection = mirDir;
		}
		break;
	case SET_JPG_ROTATION_ANGLE :
		{
			int angle;

			if (param == 0) {
				return JPG_RET_INVALID_PARAM;
			}
			angle = *(int *)param;
			if (angle != 0 && angle != 90 &&
				angle != 180 && angle != 270) {
					return JPG_RET_INVALID_PARAM;
			}
			if (pEncInfo->initialInfoObtained && (angle == 90 || angle ==270)) {
				return JPG_RET_INVALID_PARAM;
			}
			pEncInfo->rotationAngle = angle;
		}
		break;
	case ENC_JPG_GET_HEADER:
		{
			if (param == 0) {
				return JPG_RET_INVALID_PARAM;
			}

			if (!JpgEncEncodeHeader(handle, param)) {
				return JPG_RET_INVALID_PARAM;
			}
			break;
		}

	case ENABLE_LOGGING:
		{
			pJpgInst->loggingEnable = 1;
		}
		break;
	case DISABLE_LOGGING:
		{
			pJpgInst->loggingEnable = 0;
		}
		break;

	default:
		return JPG_RET_INVALID_COMMAND;
	}
	return JPG_RET_SUCCESS;
}



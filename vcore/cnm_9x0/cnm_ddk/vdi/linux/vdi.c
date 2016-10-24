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

#include <linux/delay.h>
#include <linux/io.h>
#include <asm/io.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/ion.h>
#include <linux/odin_iommu.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <media/odin/vcodec/vlog.h>
#include <asm/div64.h>

#if 1
#include "misc/vbuf/vbuf.h"
#include "vcore/cnm_9x0/cnm_ddk/vdi/vdi.h"
#include "vcore/cnm_9x0/cnm_ddk/vdi/vdi_osal.h"
#else
#include "../../../../../misc/vbuf/vbuf.h"
#include "../vdi.h"
#include "../vdi_osal.h"
#endif
#include "vcore/cnm_9x0/cnm/cnm.h"
#include "vcore/cnm_9x0/cnm_ddk/vpuapi/vpuapi.h"
#include "vcore/cnm_9x0/cnm_ddk/vpuapi/vpuapifunc.h"

#define VDI_SRAM_BASE_ADDR	0x0
#define VDI_CODA980_SRAM_SIZE		100*1024
#define VDI_BODA950_SRAM_SIZE		56*1024

#define VDI_SYSTEM_ENDIAN VDI_LITTLE_ENDIAN

struct  vdi_core_inst{
	unsigned long coreIdx;
	vpu_instance_pool_t pvip;

	unsigned long reg_paddr;
	unsigned long *reg_vaddr;

	bool  initialized;
	unsigned int task_num;

	wait_queue_head_t irq_wq;
};

struct  vdi_core_inst vdi_db[MAX_VPU_CORE_NUM] = {{0}};

unsigned long long _vdi_sched_clock(void)
{
	unsigned long long cval;
	asm volatile("mrrc p15, 0, %Q0, %R0, c14" : "=r" (cval));
	return cval * (NSEC_PER_SEC / 24000000);
}

int _vdi_swap_endian(unsigned char *data, int len, int endian)
{
	unsigned long *p;
	unsigned long v1, v2, v3;
	int i;
	int swap = 0;
	p = (unsigned long *)data;

	if(endian == VDI_SYSTEM_ENDIAN)
		swap = 0;
	else
		swap = 1;

	if (swap)
	{
		if (endian == VDI_LITTLE_ENDIAN ||endian == VDI_BIG_ENDIAN) {
			for (i=0; i<len/4; i+=2) {
				v1 = p[i];
				v2  = ( v1 >> 24) & 0xFF;
				v2 |= ((v1 >> 16) & 0xFF) <<  8;
				v2 |= ((v1 >>  8) & 0xFF) << 16;
				v2 |= ((v1 >>  0) & 0xFF) << 24;
				v3 =  v2;
				v1  = p[i+1];
				v2  = ( v1 >> 24) & 0xFF;
				v2 |= ((v1 >> 16) & 0xFF) <<  8;
				v2 |= ((v1 >>  8) & 0xFF) << 16;
				v2 |= ((v1 >>  0) & 0xFF) << 24;
				p[i]   =  v2;
				p[i+1] = v3;
			}
		}
		else {
			int swap4byte;
			//VDI_LITTLE_ENDIAN
			swap4byte = 1;
			swap = 1;

			if (swap) {
				for (i=0; i<len/4; i++) {
					v1 = p[i];
					v2  = ( v1 >> 24) & 0xFF;
					v2 |= ((v1 >> 16) & 0xFF) <<  8;
					v2 |= ((v1 >>  8) & 0xFF) << 16;
					v2 |= ((v1 >>  0) & 0xFF) << 24;
					p[i] = v2;
				}
			}

			if (swap4byte) {
				for (i=0; i<len/4; i+=2) {
					v1 = p[i];
					v2 = p[i+1];
					p[i]   = v2;
					p[i+1] = v1;
				}
			}
		}
	}

	return swap;
}

int vdi_init(unsigned long coreIdx,
		Uint32 reg_paddr,
		Uint32 *reg_vaddr,
		Uint32 cbuf_paddr,
		Uint32 *cbuf_vaddr,
		Uint32 cbuf_size)
{
	struct  vdi_core_inst *vdi;

	if (coreIdx >= MAX_NUM_VPU_CORE) {
		vlog_error("core_i %lu\n", coreIdx);
		return 0;
	}

	vdi = &vdi_db[coreIdx];

	if (vdi->initialized == true) {
		vdi->task_num++;
		return 0;
	}

	vdi->reg_paddr = (unsigned long)reg_paddr;
	vdi->reg_vaddr = (unsigned long *)reg_vaddr;

	vdi->pvip.vpu_common_buffer.phys_addr = cbuf_paddr;
	vdi->pvip.vpu_common_buffer.virt_addr = cbuf_vaddr;
	vdi->pvip.vpu_common_buffer.size = cbuf_size;

	vdi->coreIdx = coreIdx;
	vdi->task_num++;
	vdi->initialized = true;

	init_waitqueue_head(&vdi->irq_wq);
	vlog_info("vdi init OK. core_i %lu\n", coreIdx);

	return 0;
}

int vdi_release(unsigned long coreIdx)
{
	struct vdi_core_inst *vdi;

	vdi = &vdi_db[coreIdx];

	if (coreIdx >= MAX_NUM_VPU_CORE) {
		vlog_error("core_i %lu, initialized %d\n", coreIdx, (int)vdi->initialized);
		return 0;
	}

	if (vdi->task_num > 1) {
		vdi->task_num--;
		return 0;
	}

	wake_up_interruptible_sync(&vdi->irq_wq);
	osal_memset(vdi, 0x00, sizeof(struct vdi_core_inst));
	vlog_info("vdi release OK. core_i %lu\n", coreIdx);

	return 0;
}

int vdi_get_common_memory(unsigned long coreIdx, vpu_buffer_t *vb)
{
	memcpy(vb, &vdi_db[coreIdx].pvip.vpu_common_buffer, sizeof(vpu_buffer_t));

	return 0;
}

vpu_instance_pool_t *vdi_get_instance_pool(unsigned long coreIdx)
{
	if( !vdi_db[coreIdx].initialized) {
		vlog_error("not initialized\n");
		return NULL;
	}

	return (vpu_instance_pool_t *)&vdi_db[coreIdx].pvip;
}

void vdi_spin_lock_init(spinlock_t *lock)
{
	spin_lock_init(lock);
}

void vdi_spin_lock(spinlock_t *lock, unsigned long *flags)
{
	spin_lock_irqsave(lock, *flags);
}

void vdi_spin_unlock(spinlock_t *lock, unsigned long *flags)
{
	spin_unlock_irqrestore(lock , *flags);
}

void vdi_write_register(unsigned long coreIdx, unsigned long addr, unsigned int data)
{
	unsigned long *reg_addr;

	reg_addr = (unsigned long *)(addr + (unsigned long)vdi_db[coreIdx].reg_vaddr);
	*(volatile unsigned long *)reg_addr = data;
}

unsigned long vdi_read_register(unsigned long coreIdx, unsigned long addr)
{
	unsigned long *reg_addr;
	unsigned long reg_value;

	reg_addr = (unsigned long *)(addr + (unsigned long)vdi_db[coreIdx].reg_vaddr);
	reg_value = *(volatile unsigned long *)reg_addr;

	return reg_value;
}

int vdi_write_memory(unsigned long coreIdx, unsigned long addr, unsigned char *data, int len, int endian)
{
	unsigned long offset;
	struct vdi_core_inst *vdi;

	vdi = &vdi_db[coreIdx];

	if((addr < vdi->pvip.vpu_common_buffer.phys_addr) ||
		(addr > (vdi->pvip.vpu_common_buffer.phys_addr + vdi->pvip.vpu_common_buffer.size))){
		vlog_error("invalid address\n");
		BUG();
	}

	offset = addr - vdi->pvip.vpu_common_buffer.phys_addr;

	_vdi_swap_endian(data, len, endian);

	osal_memcpy((void *)((unsigned long)vdi->pvip.vpu_common_buffer.virt_addr+offset), data, len);

	return len;
}

int vdi_read_memory(unsigned long coreIdx, unsigned long addr, unsigned char *data, int len, int endian)
{
	unsigned long offset;
	struct vdi_core_inst *vdi;

	vdi = &vdi_db[coreIdx];

	if((addr < vdi->pvip.vpu_common_buffer.phys_addr) ||
		(addr > (vdi->pvip.vpu_common_buffer.phys_addr + vdi->pvip.vpu_common_buffer.size))){
		vlog_error("invalid address\n");
		BUG();
	}

	offset = addr - vdi->pvip.vpu_common_buffer.phys_addr;

	osal_memcpy(data, (const void *)((unsigned long)vdi->pvip.vpu_common_buffer.virt_addr+offset), len);
	_vdi_swap_endian(data, len, endian);

	return len;
}

int vdi_allocate_dma_memory(unsigned long coreIdx, vpu_buffer_t *vb)
{
	BUG();
	return 0;
}

void vdi_free_dma_memory(unsigned long coreIdx, vpu_buffer_t *vb)
{
	BUG();
}

int vdi_attach_dma_memory(unsigned long coreIdx, vpu_buffer_t *vb)
{
	return 0;
}

int vdi_dettach_dma_memory(unsigned long coreIdx, vpu_buffer_t *vb)
{
	return 0;
}

int vdi_get_sram_memory(unsigned long coreIdx, vpu_buffer_t *vb)
{
	if( !vb || coreIdx >= MAX_NUM_VPU_CORE) {
		vlog_error("vb 0x%08X, core_i %lu\n", (unsigned int)vb, coreIdx);
		return -1;
	}

	vb->phys_addr = VDI_SRAM_BASE_ADDR;

	if(coreIdx == CODA980_COREID)
		vb->size = VDI_CODA980_SRAM_SIZE;
	else if(coreIdx == BODA950_COREID)
		vb->size = VDI_BODA950_SRAM_SIZE;

	return 0;
}

void vdi_set_sdram(unsigned long coreIdx, unsigned long addr, int len, unsigned char data, int endian)
{
	unsigned char *buf;

	if( coreIdx >= MAX_NUM_VPU_CORE || len < 0) {
		vlog_error("core_i %lu, len %d\n", coreIdx, len);
		return;
	}

	buf = (unsigned char *)osal_malloc(len);
	if (IS_ERR_OR_NULL((const void*)buf)) {
		vlog_error("malloc fail\n");
		return;
	}
	osal_memset(buf, 0x00, len);
	vdi_write_memory(coreIdx, addr, buf, len, endian);
	osal_free(buf);
}

int vdi_wait_bus_busy(unsigned long coreIdx, int timeout_ms, unsigned long gdi_busy_flag)
{
	unsigned long long start, end;

	start = _vdi_sched_clock();
	end = _vdi_sched_clock() + ((unsigned long long)timeout_ms) * 1000000;

	while(vdi_read_register(coreIdx, gdi_busy_flag) != 0x77) {

		if(_vdi_sched_clock() > end) {
			int i;
			unsigned long long diff;

			diff = end - start;
			do_div(diff, 1000000000);

			vlog_error("%llds timeout,  reg 0x%lx\n", diff, gdi_busy_flag);
			for (i=0; i<5; i++)
				vlog_error("core %lu, busy %d, codec 0x%x, pc 0x%x, mbc 0x%x, bwb 0x%x, gdi 0x%x\n",
					coreIdx,
					(unsigned int)VpuReadReg(coreIdx, BIT_BUSY_FLAG),
					(unsigned int)VpuReadReg(coreIdx, BIT_RUN_COD_STD),
					(unsigned int)VpuReadReg(coreIdx, BIT_CUR_PC),
					(unsigned int)VpuReadReg(coreIdx, MBC_BASE+0x74),
					(unsigned int)VpuReadReg(coreIdx, GDI_BWB_STATUS),
					(unsigned int)VpuReadReg(coreIdx, 0x10f4));
			/*vdi_log(coreIdx, 3, 0);*/
			return -1;
		}

		cpu_relax();
	}
	return 0;
}

int vdi_wait_vpu_busy(unsigned long coreIdx, int timeout_ms, unsigned long addr_bit_busy_flag)
{
	unsigned long long start, end;

	start = _vdi_sched_clock();
	end = start + ((unsigned long long)timeout_ms) * 1000000;

	while(vdi_read_register(coreIdx, addr_bit_busy_flag)) {

		if(_vdi_sched_clock() > end) {
			int i;
			unsigned long long diff;

			diff = end - start;
			do_div(diff, 1000000000);

			vlog_error("%llds timeout,  reg 0x%lx\n", diff, addr_bit_busy_flag);
			for (i=0; i<5; i++)
				vlog_error("core %lu, busy %d, codec 0x%x, pc 0x%x, mbc 0x%x, bwb 0x%x, gdi 0x%x\n",
					coreIdx,
					(unsigned int)VpuReadReg(coreIdx, BIT_BUSY_FLAG),
					(unsigned int)VpuReadReg(coreIdx, BIT_RUN_COD_STD),
					(unsigned int)VpuReadReg(coreIdx, BIT_CUR_PC),
					(unsigned int)VpuReadReg(coreIdx, MBC_BASE+0x74),
					(unsigned int)VpuReadReg(coreIdx, GDI_BWB_STATUS),
					(unsigned int)VpuReadReg(coreIdx, 0x10f4));
			/*vdi_log(coreIdx, 3, 0);*/
			return -1;
		}

		cpu_relax();
	}
	return 0;
}

int vdi_wait_interrupt(unsigned long coreIdx, int timeout, unsigned long addr_bit_int_reason)
{
	unsigned long remain_timeout = 0;

	remain_timeout = wait_event_interruptible_timeout(vdi_db[coreIdx].irq_wq, 0, msecs_to_jiffies(timeout));

	if (remain_timeout <= 0) {
		vlog_error("interrupt timeouted\n");
		return false;
	}

	return true;
}

static int read_pinfo_buffer(int coreIdx, int addr)
{
	int ack;
	int rdata;
#define VDI_LOG_GDI_PINFO_ADDR  (0x1068)
#define VDI_LOG_GDI_PINFO_REQ   (0x1060)
#define VDI_LOG_GDI_PINFO_ACK   (0x1064)
#define VDI_LOG_GDI_PINFO_DATA  (0x106c)
	//------------------------------------------
	// read pinfo - indirect read
	// 1. set read addr     (GDI_PINFO_ADDR)
	// 2. send req          (GDI_PINFO_REQ)
	// 3. wait until ack==1 (GDI_PINFO_ACK)
	// 4. read data         (GDI_PINFO_DATA)
	//------------------------------------------
	vdi_write_register(coreIdx, VDI_LOG_GDI_PINFO_ADDR, addr);
	vdi_write_register(coreIdx, VDI_LOG_GDI_PINFO_REQ, 1);

	ack = 0;
	while (ack == 0)
	{
		ack = vdi_read_register(coreIdx, VDI_LOG_GDI_PINFO_ACK);
	}

	rdata = vdi_read_register(coreIdx, VDI_LOG_GDI_PINFO_DATA);

	//VLOG(INFO, "[READ PINFO] ADDR[%x], DATA[%x]", addr, rdata);
	return rdata;
}


enum {
	VDI_PRODUCT_ID_980,
	VDI_PRODUCT_ID_960
};

static void printf_gdi_info(int coreIdx, int num, int reset)
{
	int i;
	int bus_info_addr;
	int tmp;
	int val;
	int productId;

	val = vdi_read_register(coreIdx, DBG_CONFIG_REPORT_1);
    if ((val&0xff00) == 0x3200) val = 0x3200;
	if (val == CODA960_CODE || val == BODA950_CODE) {
		productId = VDI_PRODUCT_ID_960;
	} else if (val == CODA980_CODE || val == WAVE320_CODE) {
		productId = VDI_PRODUCT_ID_980;
	}

	if (productId == VDI_PRODUCT_ID_980)
		printk("\n**GDI information for GDI_20\n");
	else
		printk("\n**GDI information for GDI_10\n");

#define VDI_LOG_GDI_BUS_STATUS (0x10F4)
	printk("GDI_BUS_STATUS = %lx\n", vdi_read_register(coreIdx, VDI_LOG_GDI_BUS_STATUS));
	for (i=0; i < num; i++)
	{

#define VDI_LOG_GDI_INFO_CONTROL 0x1400
		if (productId == VDI_PRODUCT_ID_980)
			bus_info_addr = VDI_LOG_GDI_INFO_CONTROL + i*(0x20);
		else
			bus_info_addr = VDI_LOG_GDI_INFO_CONTROL + i*0x14;
		if (reset)
		{
			vdi_write_register(coreIdx, bus_info_addr, 0x00);
			bus_info_addr += 4;
			vdi_write_register(coreIdx, bus_info_addr, 0x00);
			bus_info_addr += 4;
			vdi_write_register(coreIdx, bus_info_addr, 0x00);
			bus_info_addr += 4;
			vdi_write_register(coreIdx, bus_info_addr, 0x00);
			bus_info_addr += 4;
			vdi_write_register(coreIdx, bus_info_addr, 0x00);

			if (productId == VDI_PRODUCT_ID_980)
			{
				bus_info_addr += 4;
				vdi_write_register(coreIdx, bus_info_addr, 0x00);

				bus_info_addr += 4;
				vdi_write_register(coreIdx, bus_info_addr, 0x00);

				bus_info_addr += 4;
				vdi_write_register(coreIdx, bus_info_addr, 0x00);
			}


		}
		else
		{
			printk("index = %02d", i);

			tmp = read_pinfo_buffer(coreIdx, bus_info_addr);	//TiledEn<<20 ,GdiFormat<<17,IntlvCbCr,<<16 GdiYuvBufStride
			printk(" control = 0x%08x", tmp);

			bus_info_addr += 4;
			tmp = read_pinfo_buffer(coreIdx, bus_info_addr);
			printk(" pic_size = 0x%08x", tmp);

			bus_info_addr += 4;
			tmp = read_pinfo_buffer(coreIdx, bus_info_addr);
			printk(" y-top = 0x%08x", tmp);

			bus_info_addr += 4;
			tmp = read_pinfo_buffer(coreIdx, bus_info_addr);
			printk(" cb-top = 0x%08x", tmp);

			bus_info_addr += 4;
			tmp = read_pinfo_buffer(coreIdx, bus_info_addr);
			printk(" cr-top = 0x%08x", tmp);
			if (productId == VDI_PRODUCT_ID_980)
			{
				bus_info_addr += 4;
				tmp = read_pinfo_buffer(coreIdx, bus_info_addr);
				printk(" y-bot = 0x%08x", tmp);

				bus_info_addr += 4;
				tmp = read_pinfo_buffer(coreIdx, bus_info_addr);
				printk(" cb-bot = 0x%08x", tmp);

				bus_info_addr += 4;
				tmp = read_pinfo_buffer(coreIdx, bus_info_addr);
				printk(" cr-bot = 0x%08x", tmp);
			}
			printk("\n");
		}

	}
}

void vdi_log(unsigned long coreIdx, int cmd, int step)
{
	// BIT_RUN command
	enum {
		SEQ_INIT = 1,
		SEQ_END = 2,
		PIC_RUN = 3,
		SET_FRAME_BUF = 4,
		ENCODE_HEADER = 5,
		ENC_PARA_SET = 6,
		DEC_PARA_SET = 7,
		DEC_BUF_FLUSH = 8,
		RC_CHANGE_PARAMETER	= 9,
		VPU_SLEEP = 10,
		VPU_WAKE = 11,
		ENC_ROI_INIT = 12,
		FIRMWARE_GET = 0xf,
		VPU_RESET = 0x10,
	};

	int i;

	switch(cmd)
	{
	case SEQ_INIT:
		if (step == 1) {
			printk( "\n**SEQ_INIT start\n");
		}
		else if (step == 2)	{
			printk( "\n**SEQ_INIT timeout\n");
		}
		else {
			printk( "\n**SEQ_INIT end \n");
		}
		break;
	case SEQ_END:
		if (step == 1) {
			printk( "\n**SEQ_END start\n");
		}
		else if (step == 2) {
			printk( "\n**SEQ_END timeout\n");
		}
		else {
			printk( "\n**SEQ_END end\n");
		}
		break;
	case PIC_RUN:
		if (step == 1) {
			printk( "\n**PIC_RUN start\n");
		}
		else if (step == 2) {
			printk( "\n**PIC_RUN timeout\n");
		}
		else  {
			printk( "\n**PIC_RUN end\n");
		}
		break;
	case SET_FRAME_BUF:
		if (step == 1) {
			printk( "\n**SET_FRAME_BUF start\n");
		}
		else if (step == 2) {
			printk( "\n**SET_FRAME_BUF timeout\n");
		}
		else  {
			printk( "\n**SET_FRAME_BUF end\n");
		}
		break;
	case ENCODE_HEADER:
		if (step == 1) {
			printk( "\n**ENCODE_HEADER start\n");
		}
		else if (step == 2) {
			printk( "\n**ENCODE_HEADER timeout\n");
		}
		else  {
			printk( "\n**ENCODE_HEADER end\n");
		}
		break;
	case RC_CHANGE_PARAMETER:
		if (step == 1) {
			printk( "\n**RC_CHANGE_PARAMETER start\n");
		}
		else if (step == 2) {
			printk( "\n**RC_CHANGE_PARAMETER timeout\n");
		}
		else {
			printk( "\n**RC_CHANGE_PARAMETER end\n");
		}
		break;

	case DEC_BUF_FLUSH:
		if (step == 1) {
			printk( "\n**DEC_BUF_FLUSH start\n");
		}
		else if (step == 2) {
			printk( "\n**DEC_BUF_FLUSH timeout\n");
		}
		else {
			printk( "\n**DEC_BUF_FLUSH end ");
		}
		break;
	case FIRMWARE_GET:
		if (step == 1) {
			printk( "\n**FIRMWARE_GET start\n");
		}
		else if (step == 2)  {
			printk( "\n**FIRMWARE_GET timeout\n");
		}
		else {
			printk( "\n**FIRMWARE_GET end\n");
		}
		break;
	case VPU_RESET:
		if (step == 1) {
			printk( "\n**VPU_RESET start\n");
		}
		else if (step == 2) {
			printk( "\n**VPU_RESET timeout\n");
		}
		else  {
			printk( "\n**VPU_RESET end\n");
		}
		break;
	case ENC_PARA_SET:
		if (step == 1)	//
			printk( "\n**ENC_PARA_SET start\n");
		else if (step == 2)
			printk( "\n**ENC_PARA_SET timeout\n");
		else
			printk( "\n**ENC_PARA_SET end\n");
		break;
	case DEC_PARA_SET:
		if (step == 1)	//
			printk( "\n**DEC_PARA_SET start\n");
		else if (step == 2)
			printk( "\n**DEC_PARA_SET timeout\n");
		else
			printk( "\n**DEC_PARA_SET end\n");
		break;
	default:
		if (step == 1) {
			printk( "\n**ANY CMD start\n");
		}
		else if (step == 2) {
			printk( "\n**ANY CMD timeout\n");
		}
		else {
			printk( "\n**ANY CMD end\n");
		}
		break;
	}

	for (i=0; i<0x200; i=i+16)
	{
		printk( "0x%04hx: 0x%08lx 0x%08lx 0x%08lx 0x%08lx\n", (unsigned int)i,
			vdi_read_register(coreIdx, i), vdi_read_register(coreIdx, i+4),
			vdi_read_register(coreIdx, i+8), vdi_read_register(coreIdx, i+0xc));
	}
#if 0
	if ((cmd == PIC_RUN && step== 0) || cmd == VPU_RESET)
	{
		printf_gdi_info(coreIdx, 32, 0);


#define VDI_LOG_MBC_BUSY 0x0440
#define VDI_LOG_MC_BASE	 0x0C00
#define VDI_LOG_MC_BUSY	 0x0C04
#define VDI_LOG_GDI_BUS_STATUS (0x10F4)

#define VDI_LOG_ROT_SRC_IDX	 (0x400 + 0x10C)
#define VDI_LOG_ROT_DST_IDX	 (0x400 + 0x110)

		printk( "GDI_BUS_STATUS = %x\n", vdi_read_register(coreIdx, VDI_LOG_GDI_BUS_STATUS));
		printk( "GDI_BWB_STATUS = %x\n", vdi_read_register(coreIdx, GDI_BWB_STATUS));

{
	BYTE frameAddr[MAX_GDI_IDX][3][4];
	PhysicalAddress paraBuffer = VpuReadReg(coreIdx, BIT_PARA_BUF_ADDR);
	unsigned int addr_y;
	int i;

        VpuReadMem(coreIdx, paraBuffer, (BYTE*)frameAddr, sizeof(frameAddr), VDI_BIG_ENDIAN);

        printk("Tiled \n");
        for(i=0; i<MAX_GDI_IDX; i++) {
            addr_y = frameAddr[i][0][3];
            addr_y |= frameAddr[i][0][2] << 8;
            addr_y |= frameAddr[i][0][1] << 16;
            addr_y |= frameAddr[i][0][0] << 24;
            printk("addr_y[%d]: 0x%08x \n",i, addr_y);
        }

        VpuReadMem(coreIdx, paraBuffer+384+128+384, (BYTE*)frameAddr, sizeof(frameAddr), VDI_BIG_ENDIAN);

        printk("Linear \n");
        for(i=0; i<MAX_GDI_IDX; i++) {
            addr_y = frameAddr[i][0][3];
            addr_y |= frameAddr[i][0][2] << 8;
            addr_y |= frameAddr[i][0][1] << 16;
            addr_y |= frameAddr[i][0][0] << 24;
            printk("addr_y[%d]: 0x%08x \n",i, addr_y);
        }
}

/*
		printk( "MBC_BUSY = %x\n", vdi_read_register(coreIdx, VDI_LOG_MBC_BUSY));
		printk( "MC_BUSY = %x\n", vdi_read_register(coreIdx, VDI_LOG_MC_BUSY));
		printk( "MC_MB_XY_DONE=(y:%d, x:%d)\n", (vdi_read_register(coreIdx, VDI_LOG_MC_BASE) >> 20) & 0x3F, (vdi_read_register(coreIdx, VDI_LOG_MC_BASE) >> 26) & 0x3F);

		printk( "ROT_SRC_IDX = %x\n", vdi_read_register(coreIdx, VDI_LOG_ROT_SRC_IDX));
		printk( "ROT_DST_IDX = %x\n", vdi_read_register(coreIdx, VDI_LOG_ROT_DST_IDX));

		printk( "P_MC_PIC_INDEX_0 = %x\n", vdi_read_register(coreIdx, MC_BASE+0x200));
		printk( "P_MC_PIC_INDEX_1 = %x\n", vdi_read_register(coreIdx, MC_BASE+0x20c));
		printk( "P_MC_PIC_INDEX_2 = %x\n", vdi_read_register(coreIdx, MC_BASE+0x218));
		printk( "P_MC_PIC_INDEX_3 = %x\n", vdi_read_register(coreIdx, MC_BASE+0x230));
		printk( "P_MC_PIC_INDEX_3 = %x\n", vdi_read_register(coreIdx, MC_BASE+0x23C));
		printk( "P_MC_PIC_INDEX_4 = %x\n", vdi_read_register(coreIdx, MC_BASE+0x248));
		printk( "P_MC_PIC_INDEX_5 = %x\n", vdi_read_register(coreIdx, MC_BASE+0x254));
		printk( "P_MC_PIC_INDEX_6 = %x\n", vdi_read_register(coreIdx, MC_BASE+0x260));
		printk( "P_MC_PIC_INDEX_7 = %x\n", vdi_read_register(coreIdx, MC_BASE+0x26C));
		printk( "P_MC_PIC_INDEX_8 = %x\n", vdi_read_register(coreIdx, MC_BASE+0x278));
		printk( "P_MC_PIC_INDEX_9 = %x\n", vdi_read_register(coreIdx, MC_BASE+0x284));
		printk( "P_MC_PIC_INDEX_a = %x\n", vdi_read_register(coreIdx, MC_BASE+0x290));
		printk( "P_MC_PIC_INDEX_b = %x\n", vdi_read_register(coreIdx, MC_BASE+0x29C));
		printk( "P_MC_PIC_INDEX_c = %x\n", vdi_read_register(coreIdx, MC_BASE+0x2A8));
		printk( "P_MC_PIC_INDEX_d = %x\n", vdi_read_register(coreIdx, MC_BASE+0x2B4));

		printk( "P_MC_PICIDX_0 = %x\n", vdi_read_register(coreIdx, MC_BASE+0x028));
		printk( "P_MC_PICIDX_1 = %x\n", vdi_read_register(coreIdx, MC_BASE+0x02C));
*/
	}
#endif
}

int vdi_open_instance(unsigned long coreIdx, unsigned long instIdx)
{
	return 0;
}

int vdi_close_instance(unsigned long coreIdx, unsigned long instIdx)
{
	return 0;
}

int vdi_lock(unsigned long coreIdx)
{
	return 0;
}

void vdi_unlock(unsigned long coreIdx)
{
	return;
}

int vdi_disp_lock(unsigned long coreIdx)
{
	return 0;
}

void vdi_disp_unlock(unsigned long coreIdx)
{
}

int vdi_set_bit_firmware_to_pm(unsigned long coreIdx, const unsigned short *code)
{
	return 0;
}

int vdi_set_clock_gate(unsigned long coreIdx, int enable)
{
	return 0;
}

int vdi_get_clock_gate(unsigned long coreIdx)
{
	return 0;
}

int vdi_hw_reset(unsigned long coreIdx) // DEVICE_ADDR_SW_RESET
{
	//TODO FIXME
	return 0;
}

int vdi_get_instance_num(unsigned long coreIdx)
{
	return 0;
}


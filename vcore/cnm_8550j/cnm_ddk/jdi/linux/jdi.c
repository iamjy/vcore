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

#if 1
#include "misc/vbuf/vbuf.h"
#include "vcore/cnm_8550j/cnm_ddk/jpuapi/jpuapifunc.h"
#include "vcore/cnm_8550j/cnm_ddk/jdi/jdi.h"
#else
#include "../../../../../misc/vbuf/vbuf.h"
#include "../../jpuapi/jpuapifunc.h"
#include "../jdi.h"
#endif

#define JDI_SYSTEM_ENDIAN JDI_LITTLE_ENDIAN

struct jdi_buf_node_t
{
	jpu_buffer_t buf;
	struct list_head list;
};

struct  jdi_db_t{
	jpu_instance_pool_t pvip;

	jpu_buffer_t register_base;

	bool  initialized;
	unsigned int task_num;

	struct list_head buf_list;
};

struct  jdi_db_t jdi_db = {{0}};

int _jpu_swap_endian(unsigned char *data, int len, int endian)
{
	unsigned long *p;
	unsigned long v1, v2, v3;
	int i;
	int swap = 0;
	p = (unsigned long *)data;

	if(endian == JDI_SYSTEM_ENDIAN)
		swap = 0;
	else
		swap = 1;

	if (swap) {
		if (endian == JDI_LITTLE_ENDIAN || endian == JDI_BIG_ENDIAN) {
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
		} else {
			int swap4byte = 0;
			swap = 0;

			if (endian != JDI_32BIT_LITTLE_ENDIAN) {
				swap4byte = 1;
				swap = 1;
			}

			if (swap) {
				for (i = 0; i < (len/4); i++) {
					v1 = p[i];
					v2  = ( v1 >> 24) & 0xFF;
					v2 |= ((v1 >> 16) & 0xFF) <<  8;
					v2 |= ((v1 >>  8) & 0xFF) << 16;
					v2 |= ((v1 >>  0) & 0xFF) << 24;
					p[i] = v2;
				}
			}

			if (swap4byte) {
				for (i = 0; i < (len/4); i+=2) {
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

int jdi_init(unsigned long reg_base)
{
	struct jdi_db_t *jdi = &jdi_db;

	if (jdi->initialized == true) {
		jdi->task_num++;
		return 0;
	}

	jdi->register_base.size = NPT_REG_SIZE;
	jdi->register_base.phys_addr = reg_base;
	jdi->register_base.virt_addr = (unsigned long *)ioremap(jdi->register_base.phys_addr, jdi->register_base.size);
	if( IS_ERR_OR_NULL((const void *)jdi->register_base.virt_addr)) {
		vlog_error("ioremap fail.\n");
		goto ERR_JDI_INIT;
	}

	jdi->task_num++;
	jdi->initialized = true;
	INIT_LIST_HEAD(&jdi->buf_list);

	return 0;

ERR_JDI_INIT:
	jdi_release();
	return -1;
}

int jdi_release()
{
	struct jdi_db_t *jdi = &jdi_db;
	struct jdi_buf_node_t *node, *tmp;

	if (jdi->task_num > 1) {
		jdi->task_num--;
		return 0;
	}

	if (jdi->register_base.virt_addr)
		iounmap((void *)jdi->register_base.virt_addr);

	/* all memory will free */
	list_for_each_entry_safe(node, tmp, &jdi->buf_list, list) {
		vbuf_free(node->buf.dma_buf);
		list_del(&node->list);
		jdi_free(node);
	}

	jdi_memset(jdi, 0x00, sizeof(struct jdi_db_t));

	return 0;
}

void *jdi_get_instance_pool()
{
	struct jdi_db_t *jdi = &jdi_db;

	if( jdi->initialized == false ) {
		vlog_error("not initialized\n");
		return NULL;
	}

	return &jdi->pvip;
}

void jdi_write_register(unsigned long addr, unsigned int data)
{
	unsigned long *reg_addr;
	struct jdi_db_t *jdi = &jdi_db;

	reg_addr = (unsigned long *)(addr + (unsigned long)jdi->register_base.virt_addr);
	*(volatile unsigned long *)reg_addr = data;
}

unsigned long jdi_read_register(unsigned long addr)
{
	unsigned long *reg_addr;
	unsigned long reg_value;
	struct jdi_db_t *jdi = &jdi_db;

	reg_addr = (unsigned long *)(addr + (unsigned long)jdi->register_base.virt_addr);
	reg_value = *(volatile unsigned long *)reg_addr;

	return reg_value;
}

int jdi_write_memory(unsigned long addr, unsigned char *data, int len, int endian)
{
	struct jdi_buf_node_t *node, *tmp;
	unsigned long offset;
	struct jdi_db_t *jdi = &jdi_db;

	list_for_each_entry_safe(node, tmp, &jdi->buf_list, list) {
		if((addr >= node->buf.phys_addr) && (addr < (node->buf.phys_addr + node->buf.size))){
			break;
		}
	}

	if (&node->list == &jdi->buf_list) {
		vlog_error("not exist in list 0x%08X\n",(unsigned int)addr);
		return -1;
	}

	offset = addr - (unsigned long)node->buf.phys_addr;

	_jpu_swap_endian(data, len, endian);
	if(IS_ERR_OR_NULL((const void*)node->buf.virt_addr) || IS_ERR_OR_NULL((const void*)node->buf.phys_addr)){
		vlog_error("node->buf.phys_addr = 0x%08X || node->buf.virt_addr = 0x%08X\n",(unsigned int)node->buf.phys_addr, (unsigned int)node->buf.virt_addr);
		return -1;
	}
	jdi_memcpy((void *)((unsigned long)node->buf.virt_addr+offset), data, len);

	return len;
}

int jdi_read_memory(unsigned long addr, unsigned char *data, int len, int endian)
{
	struct jdi_buf_node_t *node, *tmp;
	unsigned long offset;
	struct jdi_db_t *jdi = &jdi_db;

	list_for_each_entry_safe(node, tmp, &jdi->buf_list, list) {
		if((addr >= node->buf.phys_addr) && (addr < (node->buf.phys_addr + node->buf.size)))
			break;
	}

	if (&node->list == &jdi->buf_list) {
		vlog_error("not exist in list 0x%x\n", (unsigned int)addr);
		return -1;
	}

	offset = addr - (unsigned long)node->buf.phys_addr;

	jdi_memcpy(data, (const void *)((unsigned long)node->buf.virt_addr+offset), len);
	_jpu_swap_endian(data, len,  endian);

	return len;
}

int jdi_allocate_dma_memory(jpu_buffer_t *vb)
{
	struct jdi_buf_node_t* node;
	struct jdi_db_t *jdi = &jdi_db;

	if( vb == NULL) {
		vlog_error("vb NULL\n");
		return -1;
	}

	node = jdi_malloc(sizeof(struct jdi_buf_node_t));
	if (IS_ERR_OR_NULL((const void*)node)) {
		vlog_error("jdi_malloc fail ... \n");
		return -1;
	}
	node->buf.size = vb->size;

	node->buf.dma_buf = vbuf_malloc(node->buf.size, true);
	if( IS_ERR_OR_NULL((const void*)node->buf.dma_buf) ) {
		vlog_error("vbuf_malloc failed\n");
		jdi_free(node);
		return -1;
	}

	node->buf.phys_addr = node->buf.dma_buf->paddr;
	node->buf.virt_addr = node->buf.dma_buf->vaddr;

	if(IS_ERR_OR_NULL((const void*)node->buf.virt_addr) || IS_ERR_OR_NULL((const void*)node->buf.phys_addr)){
		vlog_error("node->buf.phys_addr = 0x%08X || node->buf.virt_addr = 0x%08X\n",(unsigned int)node->buf.phys_addr, (unsigned int)node->buf.virt_addr);
		jdi_free(node);
		return -1;
	}

	vb->phys_addr = node->buf.phys_addr;
	vb->virt_addr = node->buf.virt_addr;

	list_add(&node->list, &jdi->buf_list);

	return 0;
}

void jdi_free_dma_memory(jpu_buffer_t *vb)
{
	struct jdi_buf_node_t *node, *tmp;
	struct jdi_db_t *jdi = &jdi_db;

	if( vb == NULL) {
		vlog_error("vb 0x%08X\n", (unsigned int)vb);
		return;
	}

	if( vb->size == 0 ) {
		vlog_error("vb size %d\n", vb->size);
		return;
	}

	if (list_empty(&jdi->buf_list)) {
		vlog_error("list is empty \n");
		return;
	}

	list_for_each_entry_safe(node, tmp, &jdi->buf_list, list) {
		if(node->buf.phys_addr == vb->phys_addr)
			break;
	}

	if (&node->list == &jdi->buf_list) {
		vlog_error("not exist in list 0x%x\n", (unsigned int)vb->phys_addr);
		return;
	}

	vbuf_free(node->buf.dma_buf);
	list_del(&node->list);
	jdi_free(node);

	jdi_memset(vb, 0, sizeof(jpu_buffer_t));
}

void jdi_log(int cmd, int step)
{
	int i;

	switch (cmd) {
	case JDI_LOG_CMD_PICRUN:
		if (step == 1)
			vlog_error("\n**PIC_RUN start\n");
		else
			vlog_error("\n**PIC_RUN end \n");
		break;
	}

	for (i = 0; i <= 0x238; i = i + 16) {
		vlog_info("0x%04xh: 0x%08lx 0x%08lx 0x%08lx 0x%08lx\n", i,
		jdi_read_register(i), jdi_read_register(i+4),
		jdi_read_register(i+8), jdi_read_register(i+0xc));
	}
}

void * jdi_memcpy(void * dst, const void * src, int count)
{
	return memcpy(dst, src, count);
}

void * jdi_memset(void *dst, int val, int count)
{
	return memset(dst, val, count);
}

void * jdi_malloc(int size)
{
	return kzalloc(size, GFP_ATOMIC);
}

void jdi_free(void *p)
{
	return kfree(p);
}

void jdi_spin_lock_init(spinlock_t *lock)
{
	spin_lock_init(lock);
}

void jdi_spin_lock(spinlock_t *lock, unsigned long *flags)
{
	spin_lock_irqsave(lock, *flags);
}

void jdi_spin_unlock(spinlock_t *lock, unsigned long *flags)
{
	spin_unlock_irqrestore(lock , *flags);
}

int jdi_hw_reset()
{
	return 0;
}

int jdi_lock()
{
	return 0;
}
void jdi_unlock()
{
}

int jdi_set_clock_gate(int enable)
{
	return 0;
}

int jdi_get_clock_gate(void)
{
	return 0;
}

int jdi_get_instance_num(void)
{
	return 0;
}

unsigned long jdi_wait_interrupt(int timeout)
{
	return 0;
}

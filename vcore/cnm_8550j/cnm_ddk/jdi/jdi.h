//------------------------------------------------------------------------------
// File: jdi.h
//
// Copyright (c) 2006, Chips & Media.  All rights reserved.
//------------------------------------------------------------------------------

#ifndef _JDI_LINUX_H_
#define _JDI_LINUX_H_

#include <linux/spinlock.h>

#include "../jpuapi/jpuconfig.h"
#include "../jpuapi/regdefine.h"

#define JpuWriteReg( ADDR, DATA )	jdi_write_register( ADDR, DATA ) // system register write
#define JpuReadReg( ADDR )			jdi_read_register( ADDR )		   // system register write
#define JpuWriteMem( ADDR, DATA, LEN, ENDIAN )	jdi_write_memory( ADDR, DATA, LEN, ENDIAN ) // system memory write
#define JpuReadMem( ADDR, DATA, LEN, ENDIAN )	jdi_read_memory( ADDR, DATA, LEN, ENDIAN ) // system memory write

typedef struct jpu_buffer_t {
	int size;
	unsigned long phys_addr;
	unsigned long base;
	unsigned long *virt_addr;
	struct vbuf *dma_buf;
} jpu_buffer_t;

#ifdef SUPPORT_128BIT_BUS

typedef enum {
	JDI_128BIT_LITTLE_64BIT_LITTLE_ENDIAN = ((0<<2)+(0<<1)+(0<<0)),	//  128 bit little, 64 bit little
	JDI_128BIT_BIG_64BIT_LITTLE_ENDIAN = ((1<<2)+(0<<1)+(0<<0)),	//  128 bit big , 64 bit little
	JDI_128BIT_LITTLE_64BIT_BIG_ENDIAN = ((0<<2)+(0<<1)+(1<<0)),	//  128 bit little, 64 bit big
	JDI_128BIT_BIG_64BIT_BIG_ENDIAN = ((1<<2)+(0<<1)+(1<<0)),		//  128 bit big, 64 bit big
	JDI_128BIT_LITTLE_32BIT_LITTLE_ENDIAN = ((0<<2)+(1<<1)+(0<<0)),	//  128 bit little, 32 bit little
	JDI_128BIT_BIG_32BIT_LITTLE_ENDIAN = ((1<<2)+(1<<1)+(0<<0)),	//  128 bit big , 32 bit little
	JDI_128BIT_LITTLE_32BIT_BIG_ENDIAN = ((0<<2)+(1<<1)+(1<<0)),	//  128 bit little, 32 bit big
	JDI_128BIT_BIG_32BIT_BIG_ENDIAN = ((1<<2)+(1<<1)+(1<<0)),		//  128 bit big, 32 bit big
} EndianMode;
#define JDI_LITTLE_ENDIAN JDI_128BIT_LITTLE_64BIT_LITTLE_ENDIAN
#define JDI_BIG_ENDIAN JDI_128BIT_BIG_64BIT_BIG_ENDIAN
#define JDI_128BIT_ENDIAN_MASK (1<<2)
#define JDI_64BIT_ENDIAN_MASK  (1<<1)
#define JDI_ENDIAN_MASK  (1<<0)

#define JDI_32BIT_LITTLE_ENDIAN JDI_128BIT_LITTLE_32BIT_LITTLE_ENDIAN
#define JDI_32BIT_BIG_ENDIAN JDI_128BIT_LITTLE_32BIT_BIG_ENDIAN

#else

typedef enum {
	JDI_LITTLE_ENDIAN = 0,
	JDI_BIG_ENDIAN,
	JDI_32BIT_LITTLE_ENDIAN,
	JDI_32BIT_BIG_ENDIAN,
} EndianMode;
#endif

typedef enum {
	JDI_LOG_CMD_PICRUN  = 0,
	JDI_LOG_CMD_MAX
} jdi_log_cmd;


#if defined (__cplusplus)
extern "C" {
#endif
	int jdi_init(unsigned long reg_base);
	int jdi_release(void);			//this function may be called only at system off.
	void *jdi_get_instance_pool(void);
	int jdi_allocate_dma_memory(jpu_buffer_t *vb);
	void jdi_free_dma_memory(jpu_buffer_t *vb);

	unsigned long jdi_wait_interrupt(int timeout);
	int jdi_hw_reset(void);

	int jdi_set_clock_gate(int enable);
	int jdi_get_clock_gate(void);

	int jdi_get_instance_num(void);

	void jdi_write_register(unsigned long addr, unsigned int data);
	unsigned long jdi_read_register(unsigned long addr);

	int jdi_write_memory(unsigned long addr, unsigned char *data, int len, int endian);
	int jdi_read_memory(unsigned long addr, unsigned char *data, int len, int endian);

	int jdi_lock(void);
	void jdi_unlock(void);
	void jdi_log(int cmd, int step);

	void * jdi_memcpy(void * dst, const void * src, int count);
	void * jdi_memset(void *dst, int val, int count);
	void * jdi_malloc(int size);
	void jdi_free(void *p);
	void jdi_spin_lock_init(spinlock_t *lock);
	void jdi_spin_lock(spinlock_t *lock, unsigned long *flags);
	void jdi_spin_unlock(spinlock_t *lock, unsigned long *flags);

#ifdef CNM_FPGA_PLATFORM
#define HPI_SET_TIMING_MAX 1000
	int jdi_set_timing_opt();
	int jdi_set_clock_freg(int Device, int OutFreqMHz, int InFreqMHz);
#endif

#if defined (__cplusplus)
}
#endif


#endif //#ifndef _JDI_LINUX_H_

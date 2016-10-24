//------------------------------------------------------------------------------
// File: vdi_osal.c
//
// Copyright (c) 2006, Chips & Media.  All rights reserved.
//------------------------------------------------------------------------------
#include <linux/slab.h>
#include "../../vpuapi/vpuconfig.h"
#include "../vdi_osal.h"

int InitLog(void)
{
	return 1;
}

void DeInitLog(void)
{
}

void SetLogColor(int level, int color)
{
}

int GetLogColor(int level)
{
	return 0;
}

void SetLogDecor(int decor)
{
}

int GetLogDecor(void)
{
	return 0;
}

void SetMaxLogLevel(int level)
{
}

int GetMaxLogLevel(void)
{
	return MAX_LOG_LEVEL;
}

void LogMsg(int level, const char *format, ...)
{
}

void timer_init(void)
{
}

void timer_start(void)
{
}

void timer_stop(void)
{
}

double timer_elapsed_ms(void)
{
	return -1;
}

double timer_elapsed_us(void)
{
	return -1;
}

int timer_is_valid(void)
{
	return -1;
}

double timer_frequency(void)
{
	return 0;
}

void osal_init_keyboard(void)
{
}

void osal_close_keyboard(void)
{
}

int osal_kbhit(void)
{
    return 0;
}


int osal_getch(void)
{
    return 0;
}

int osal_flush_ch(void)
{
	return 0;
}

void * osal_memcpy(void * dst, const void * src, int count)
{
	return memcpy(dst, src, count);
}

void * osal_memset(void *dst, int val, int count)
{
	return memset(dst, val, count);
}

void * osal_malloc(int size)
{
	return kzalloc(size, GFP_ATOMIC);
}

void osal_free(void *p)
{
	return kfree(p);
}

int osal_fflush(osal_file_t fp)
{
	return 0;
}

int osal_feof(osal_file_t fp)
{
    return 0;
}

osal_file_t osal_fopen(const char * osal_file_tname, const char * mode)
{
	return 0;
}
int osal_fwrite(const void * p, int size, int count, osal_file_t fp)
{
	return 0;
}
int osal_fread(void *p, int size, int count, osal_file_t fp)
{
	return 0;
}
long osal_ftell(osal_file_t fp)
{
	return 0;
}

int osal_fseek(osal_file_t fp, long offset, int origin)
{
	return 0;
}
int osal_fclose(osal_file_t fp)
{
	return 0;
}

int osal_fscanf(osal_file_t fp, const char * _Format, ...)
{
	return 0;
}

int osal_fprintf(osal_file_t fp, const char * _Format, ...)
{
	return 0;
}

int math_div(int number, int denom)
{
    return (number / denom);
}

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

#ifndef _VCODEC_DEBUG_H_
#define _VCODEC_DEBUG_H_

#include <linux/kernel.h>

#define VCODEC_DEBUG
#ifdef VCODEC_DEBUG
#define IN() \
do{ \
	printk(KERN_ERR "[IN] %s\n", __func__); \
}while(0)

#define OUT() \
do{ \
	printk(KERN_ERR "[OUT] %s\n", __func__); \
}while(0)

#define debug( text, args... )	\
do{									\
	printk( KERN_ERR "[VCORE] %s\t" text, __func__, ##args); \	
} while(0)

#define debug_error( text, args... )						\
do{										\
	printk( KERN_ERR "### ERROR [VCORE] %s\t" text, __func__, ##args);\
} while(0)

#define debug_warning( text, args... )						\
do{										\
	printk( KERN_ERR "### WARNING [DRV][VCODEC] %s:%d\t" text, __FILE__, __LINE__, ##args);\
} while(0)

#define debug_trace()		\
do{				\
	printk(KERN_ERR "[TRACE]%s %s:%d\n", __FILE__, __func__, __LINE__); \
} while(0)

#else

#define IN()	
#define OUT()
#define debug(...)
#define debug_error( text, args... )						\
do{										\
	printk( KERN_ERR "### ERROR [DRV][VCODEC] %s:%d\t" text, __FILE__, __LINE__, ##args);\
} while(0)
#define debug_warning( fmt, args... )
#define debug_trace(...)
#endif


#endif //#ifndef _VCODEC_DEBUG_H_


//------------------------------------------------------------------------------
// File: config.h
//
// Copyright (c) 2006, Chips & Media.  All rights reserved.
// This file should be modified by some developers of C&M according to product version.
//------------------------------------------------------------------------------


#ifndef __CONFIG_H__
#define __CONFIG_H__


#if defined(_WIN32) || defined(__WIN32__) || defined(_WIN64) || defined(WIN32) || defined(__MINGW32__)
#	define PLATFORM_WIN32
#elif defined(linux) || defined(__linux) || defined(ANDROID)
#	define PLATFORM_LINUX
#else
#	define PLATFORM_NON_OS
#endif

#if defined(_MSC_VER)
#	include <windows.h>
#	include <conio.h>
#	define inline _inline
#	define VPU_DELAY_MS(X)		Sleep(X)
#	define VPU_DELAY_US(X)		Sleep(X)	// should change to delay function which can be delay a microsecond unut.
#	define kbhit _kbhit
#	define getch _getch
#elif defined(__GNUC__)
#ifdef	_KERNEL_
#	define VPU_DELAY_MS(X)		udelay(X*1000)
#	define VPU_DELAY_US(X)		udelay(X)
#else
#	define VPU_DELAY_MS(X)		usleep(X*1000)
#	define VPU_DELAY_US(X)		usleep(X)
#endif
#elif defined(__ARMCC__)
#else
#  error "Unknown compiler."
#endif

#define PROJECT_ROOT	"..\\..\\..\\"

#if defined(CNM_FPGA_PLATFORM)
#if defined(ANDROID) || defined(linux)
#else
#define SUPPORT_CONF_TEST
#endif
#endif

#define NIEUPORT_V10
#define NIEUPORT_V13			// should be on NIEUPORT_V10 as well
#define NIEUPORT_V14			// should be on NIEUPORT_V13 as well

#ifdef NIEUPORT_V16
#define API_VERSION 165
#else
#define API_VERSION 146
#endif


//#define SUPPORT_128BIT_BUS 
//#define MJPEG_ERROR_CONCEAL

#ifdef NIEUPORT_V10

#endif	//#ifdef NIEUPORT_V10

#endif	/* __CONFIG_H__ */


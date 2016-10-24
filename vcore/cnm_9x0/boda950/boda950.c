/*
 * SIC LABORATORY, LG ELECTRONICS INC., SEOUL, KOREA
 * Copyright(c) 2013 by LG Electronics Inc.
 * Youngki Lyu <youngki.lyu@lge.com>
 * Jungmin Park <jungmin016.park@lge.com>
 * Younghyun Jo <younghyun.jo@lge.com>
 * Seokhoon Kang <m4seokhoon.kang@lgepartner.com>
 * Inpyo Cho <inpyo.cho@lge.com>
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

#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/odin_iommu.h>
#include <linux/odin_mailbox.h>
#include <linux/odin_pm_domain.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spinlock.h>
#include <linux/kernel.h>
#include <linux/vmalloc.h>
#include <linux/io.h>
#include <asm/io.h>
#include <linux/clk.h>

#include <media/odin/vcodec/vlog.h>
#include <media/odin/vcodec/vcore/device.h>

#include "vcore/cnm_9x0/cnm/cnm.h"
#include "vcore/cnm_9x0/cnm_ddk/fw/blackbird.h"
#include "vcore/cnm_9x0/cnm_ddk/vdi/vdi.h"

#include "boda950_dec.h"
#include "boda950_ppu.h"

static void *boda950_core_instance;
static struct device *dev;
static struct clk *a_clk;
static struct clk *d_clk;

static int boda950_clk_prepare_cnt = 0;
static int boda950_clk_enable_cnt = 0;

static void _boda950_isr(void)
{
	cnm_isr(boda950_core_instance);
}

static void _boda950_resume(void)
{
	pm_runtime_get_sync(dev);
}

static void _boda950_suspend(void)
{
	pm_runtime_put_sync(dev);
}

static void boda950_report_reset(void)
{
	boda950_dec_report_reset();
	boda950_ppu_report_reset();
}

void* boda950_get_cnm_core_instance(void)
{
	return boda950_core_instance;
}

static unsigned int _boda950_get_running_weight(void)
{
       void *cnm_core_instance = boda950_get_cnm_core_instance();
       return cnm_get_running_weight(cnm_core_instance);
}

static unsigned long _boda950_get_utilization(void)
{
       void *cnm_core_instance = boda950_get_cnm_core_instance();
       return cnm_get_long_utilization(cnm_core_instance);
}

void  boda950_clock_on(void)
{
	boda950_clk_enable_cnt++;

	clk_enable(d_clk);
	clk_enable(a_clk);
}

void  boda950_clock_off(void)
{
	boda950_clk_enable_cnt--;

	clk_disable(a_clk);
	clk_disable(d_clk);
}

int boda950_runtime_resume(struct device *dev)
{
	boda950_clk_prepare_cnt++;

	vlog_info("prepare:%d, enable:%d\n",
			boda950_clk_prepare_cnt, boda950_clk_enable_cnt);

	clk_prepare(d_clk);
	clk_prepare(a_clk);

	boda950_clock_on();
	cnm_resume(boda950_core_instance);
	boda950_clock_off();

	return 0;
}

int boda950_runtime_suspend(struct device *dev)
{
	boda950_clk_prepare_cnt--;

	vlog_info("prepare:%d, enable:%d\n",
			boda950_clk_prepare_cnt, boda950_clk_enable_cnt);

	clk_unprepare(a_clk);
	clk_unprepare(d_clk);

	cnm_suspend(boda950_core_instance);

	return 0;
}

int boda950_resume(void)
{
	vlog_info("\n");

	return 0;
}

int boda950_suspend(void)
{
	vlog_info("\n");

	return 0;
}

void boda950_broadcast(void *vcore_id)
{
	boda950_ppu_broadcast(boda950_core_instance, vcore_id);
	boda950_dec_broadcast(boda950_core_instance, vcore_id);
}

void (*boda950_init(unsigned int reg_base, struct device *_dev))(void)
{
	struct vcore_entry entry =
	{
		.name = "boda950",
		.resume = _boda950_resume,
		.suspend = _boda950_suspend,
		.get_utilization = _boda950_get_utilization,
		.get_util_estimation = NULL,

		.dec.capability = (1 << VCORE_DEC_AVC) | (1 << VCORE_DEC_MPEG4) |
					(1 << VCORE_DEC_H263) | (1 << VCORE_DEC_VP8) |
					(1 << VCORE_DEC_VP8) | (1 << VCORE_DEC_MPEG2) |
				   	(1 << VCORE_DEC_DIVX3) | (1 << VCORE_DEC_DIVX) |
					(1 << VCORE_DEC_SORENSON) | (1 << VCORE_DEC_VC1) |
				   	(1 << VCORE_DEC_AVS) | (1 << VCORE_DEC_MVC) |
				   	(1 << VCORE_DEC_THO),
		.dec.init = boda950_dec_init,
		.enc.capability = 0x0,
		.enc.init = NULL,
		.ppu.init = boda950_ppu_init,
	};
	int fw_size;
	unsigned int default_running_weight = 0;
	/* unsigned int default_running_weight = 1920 * 1088 * 60; */

	dev = _dev;

	fw_size = sizeof(bit_code_blackbird)/sizeof(bit_code_blackbird[0]);

	boda950_core_instance = cnm_init(BODA950_COREID, reg_base,
			(unsigned short*)bit_code_blackbird, fw_size,
			default_running_weight, boda950_broadcast,
		   	boda950_report_reset, boda950_clock_on, boda950_clock_off);
	if (boda950_core_instance == NULL)
	{
		vlog_error("boda950 cnm_init failed\n");
		return NULL;
	}

	vcore_register(&entry);

	a_clk = clk_get(NULL, "vsp1_aclk_clk");
	d_clk = clk_get(NULL, "vsp1_dec_clk");
	if (IS_ERR_OR_NULL(a_clk) ||IS_ERR_OR_NULL(d_clk))
		vlog_error("clk get failed\n");


	return _boda950_isr;
}



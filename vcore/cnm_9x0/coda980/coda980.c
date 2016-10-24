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
#include <linux/clk-private.h>

#include <media/odin/vcodec/vlog.h>
#include <media/odin/vcodec/vcore/device.h>
#include <media/odin/vcodec/vcore/encoder.h>

#include "vcore/cnm_9x0/cnm/cnm.h"
#include "vcore/cnm_9x0/cnm_ddk/fw/harrier.h"
#include "coda980_dec.h"
#include "coda980_enc.h"
#include "coda980_ppu.h"


static void *coda980_core_instance;
static struct device *dev;
static struct clk *e_clk;
static struct clk *c_clk;
static struct clk *a_clk;

static int coda980_clk_prepare_cnt = 0;
static int coda980_clk_enable_cnt = 0;

static void _coda980_isr(void)
{
	cnm_isr(coda980_core_instance);
}

static void _coda980_resume(void)
{
	pm_runtime_get_sync(dev);
}

static void _coda980_suspend(void)
{
	pm_runtime_put_sync(dev);
}

static void coda980_report_reset(void)
{
	coda980_dec_report_reset();
	coda980_enc_report_reset();
	coda980_ppu_report_reset();
}

void* coda980_get_cnm_core_instance(void)
{
	return coda980_core_instance;
}

static unsigned int _coda980_get_running_weight(void)
{
       void *cnm_core_instance = coda980_get_cnm_core_instance();
       return cnm_get_running_weight(cnm_core_instance);
}

static unsigned long _coda980_get_utilization(void)
{
       void *cnm_core_instance = coda980_get_cnm_core_instance();
       return cnm_get_long_utilization(cnm_core_instance);
}

static unsigned int _coda980_calculate_utilization(
				unsigned int width, unsigned int height,
				unsigned int fr_residual, unsigned int fr_divider)
{
	unsigned int resolution_percetage_scaled;
	unsigned int full_capability_scaled;

	unsigned int ffffffff_boundary;

	resolution_percetage_scaled = width * height * 120;
	full_capability_scaled = 1920 * 1088 * 60;

	ffffffff_boundary = 0xFFFFFFFF / fr_residual;
	while (resolution_percetage_scaled) {
		if (resolution_percetage_scaled <= ffffffff_boundary)
			break;

		resolution_percetage_scaled >>= 1;
		full_capability_scaled >>= 1;
	}

	return fr_residual * resolution_percetage_scaled / fr_divider / full_capability_scaled;
}

static unsigned long _coda980_get_util_estimation(
							unsigned int width, unsigned int height,
							unsigned int fr_residual, unsigned int fr_divider)
{
	unsigned long util_estimation_all;
	unsigned long util_estimation;

	util_estimation = _coda980_calculate_utilization(width, height, fr_residual, fr_divider);
	util_estimation_all = _coda980_get_utilization() + util_estimation;
	if (util_estimation_all > 90) {
		vlog_error("full utilization: %lu\n", util_estimation);
		util_estimation_all = 0;
	}

	return util_estimation_all;
}


void  coda980_clock_on(void)
{
	coda980_clk_enable_cnt++;

	clk_enable(a_clk);
	clk_enable(c_clk);
	clk_enable(e_clk);
}

void  coda980_clock_off(void)
{
	coda980_clk_enable_cnt--;

	clk_disable(e_clk);
	clk_disable(c_clk);
	clk_disable(a_clk);
}

int coda980_runtime_resume(struct device *dev)
{
	coda980_clk_prepare_cnt++;

	vlog_info("prepare:%d, enable:%d\n",
			coda980_clk_prepare_cnt, coda980_clk_enable_cnt);

	clk_prepare(a_clk);
	clk_prepare(c_clk);
	clk_prepare(e_clk);

	coda980_clock_on();
	cnm_resume(coda980_core_instance);
	coda980_clock_off();

	return 0;
}

int coda980_runtime_suspend(struct device *dev)
{
	coda980_clk_prepare_cnt--;

	vlog_info("a_clk(%u %u) c_clk(%u %u) e_clk(%d %d), prepare:%d, enable:%d\n",
			a_clk->enable_count, a_clk->prepare_count,
			c_clk->enable_count, c_clk->prepare_count,
			e_clk->enable_count, e_clk->prepare_count,
			coda980_clk_prepare_cnt, coda980_clk_enable_cnt);

	clk_unprepare(e_clk);
	clk_unprepare(c_clk);
	clk_unprepare(a_clk);

	cnm_suspend(coda980_core_instance);

	return 0;
}

int coda980_resume(struct device *dev)
{
	vlog_info("\n");

	return 0;
}

int coda980_suspend(struct device *dev)
{
	vlog_info("\n");

	return 0;
}

void coda980_broadcast(void *last_vcore_id)
{
	coda980_dec_broadcast(coda980_core_instance, last_vcore_id);
	coda980_enc_broadcast(coda980_core_instance, last_vcore_id);
	coda980_ppu_broadcast(coda980_core_instance, last_vcore_id);
}

void (*coda980_init(unsigned int reg_base, struct device *_dev))(void)
{
	struct vcore_entry entry =
	{
		.name = "coda980",
		.resume = _coda980_resume,
		.suspend = _coda980_suspend,
		.get_utilization = _coda980_get_utilization,
		.get_util_estimation = _coda980_get_util_estimation,
		.dec.capability = (1 << VCORE_DEC_AVC) | (1 << VCORE_DEC_MPEG4) |
			(1 << VCORE_DEC_H263) | (1 << VCORE_DEC_VP8) | (1 << VCORE_DEC_VP8) |
			(1 << VCORE_DEC_MPEG2) | (1 << VCORE_DEC_DIVX3) | (1 << VCORE_DEC_DIVX) |
			(1 << VCORE_DEC_SORENSON) | (1 << VCORE_DEC_VC1) | (1 << VCORE_DEC_AVS) |
			(1 << VCORE_DEC_MVC) | (1 << VCORE_DEC_THO),
		.dec.init = coda980_dec_init,
		.enc.capability = (1 << VCORE_ENC_AVC) | (1 << VCORE_ENC_MPEG4) |
			(1 << VCORE_ENC_H263),
		.enc.init = coda980_enc_init,
		.ppu.init = coda980_ppu_init,
	};
	unsigned int default_running_weight = VCORE_ENCODER_DEFAULT_RUNNING_WEIGHT;
	int fw_size;

	dev = _dev;

	fw_size = sizeof(bit_code_harrier)/sizeof(bit_code_harrier[0]);

	coda980_core_instance = cnm_init(CODA980_COREID, reg_base,
							(unsigned short*)bit_code_harrier, fw_size,
							default_running_weight,
							coda980_broadcast,
							coda980_report_reset,
							coda980_clock_on, coda980_clock_off);
	if (coda980_core_instance == NULL)
	{
		vlog_error("coda980 cnm_init failed\n");

		return NULL;
	}

	vcore_register(&entry);

	e_clk = clk_get(NULL, "vsp0_e_codec");
	c_clk = clk_get(NULL, "vsp0_c_codec");
	a_clk = clk_get(NULL, "vsp0_aclk_clk");

	if (IS_ERR_OR_NULL(e_clk) ||IS_ERR_OR_NULL(c_clk) || IS_ERR_OR_NULL(a_clk))
		vlog_error("clk get failed\n");

	return _coda980_isr;
}

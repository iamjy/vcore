/*
 * SIC LABORATORY, LG ELECTRONICS INC., SEOUL, KOREA
 * Copyright(c) 2013 by LG Electronics Inc.
 * Youngki Lyu <youngki.lyu@lge.com>
 * Jungmin Park <jungmin016.park@lge.com>
 * Younghyun Jo <younghyun.jo@lge.com>
 * Seokhoon.Kang <m4seokhoon.kang@lgepartner.com>
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
#include <linux/clk.h>

#include <media/odin/vcodec/vlog.h>
#include <media/odin/vcodec/vcore/device.h>

#include "coda8550j_dec.h"
#include "coda8550j_enc.h"
#include "vcore/cnm_8550j/cnm_ddk/jpuapi/jpuapifunc.h"

#define	VCORE_UTILIZATION_LOG_CNT	0x100

static struct device *dev;
static struct clk *img_clk;

static spinlock_t lock;
static void *locked_ch;
static void (*locked_isr_func)(void *vcore_id, unsigned long reason);
static unsigned int running_weight = 0;
static int vcore_clock_state = 0;

struct
{
	unsigned long prev_time;
	unsigned long run_time;
	unsigned long total_time;
	unsigned int measure_cnt;
} usage;


int coda8550j_ch_lock(void *ch, void (*isr_func)(void*, unsigned long))
{
	unsigned long flags;

	jdi_spin_lock(&lock, &flags);

	if (locked_ch != NULL) {
		vlog_error("already locked ch(0x%X)\n", (unsigned int)locked_ch);
		jdi_spin_unlock(&lock , &flags);
		return -1;
	}

	locked_ch = ch;
	locked_isr_func = isr_func;

	jdi_spin_unlock(&lock , &flags);

	return 0;
}

int coda8550j_ch_unlock(void *ch)
{
	unsigned long flags;

	jdi_spin_lock(&lock, &flags);

	if (locked_ch != ch) {
		vlog_error("another locked ch(0x%X, cur 0x%X)\n", (unsigned int)locked_ch, (unsigned int)ch);
		jdi_spin_unlock(&lock , &flags);
		return -1;
	}

	locked_ch = NULL;
	locked_isr_func = NULL;

	jdi_spin_unlock(&lock , &flags);

	return 0;
}

int coda8550j_ch_is_locked(void *ch)
{
	unsigned long flags;

	jdi_spin_lock(&lock, &flags);

	if(locked_ch == NULL) { /* unlocked */
		jdi_spin_unlock(&lock , &flags);
		return 0;
	}
	else if(locked_ch == ch) { /* the ch is locking */
		jdi_spin_unlock(&lock , &flags);
		return 1;
	}

	/*another ch is locking */
	jdi_spin_unlock(&lock , &flags);
	return -1;
}

void coda8550j_update_run_time(int run)
{
	unsigned long curr_time;
	unsigned long prev_time;
	unsigned long elapse_time;
	unsigned long utilization = 0;
	unsigned long flags;
	struct timespec now;

	prev_time = usage.prev_time;
	getnstimeofday(&now);
	curr_time = (now.tv_sec*1000*1000 + now.tv_nsec/1000) & 0x7FFFFFFF;

	jdi_spin_lock(&lock, &flags);

	if (prev_time != 0x80000000) {
		elapse_time = (curr_time >= prev_time) ? curr_time - prev_time : curr_time + 0x80000000 - prev_time;
		if (!elapse_time)
			elapse_time = 1;

		if (run)
			usage.run_time += elapse_time;

		usage.total_time += elapse_time;

		usage.measure_cnt++;
		utilization = usage.run_time * 100 / usage.total_time;
	}
	usage.prev_time = curr_time;

	if (usage.measure_cnt > VCORE_UTILIZATION_LOG_CNT) {
		vlog_print(VLOG_VCORE_MONITOR, "utilization %lu%%\n", utilization);

		usage.measure_cnt = 0;
		usage.run_time = 0;
		usage.total_time = 0;
	}

	jdi_spin_unlock(&lock , &flags);
}

void __coda8550j_isr(void)
{
	unsigned long reason;

	reason = JpuReadReg(MJPEG_PIC_STATUS_REG);

	if(locked_ch == NULL) {
		vlog_error("no locked chnnel\n");
	}
	else {
		locked_isr_func(locked_ch, reason);
	}

	JpuWriteReg(MJPEG_PIC_STATUS_REG, (1 << reason));
}

static irqreturn_t _coda8550j_isr(int irq, void *isr)
{
	((void (*)(void))isr)();
	return IRQ_HANDLED;
}

void coda8550j_reserve_running_weight(unsigned int width, unsigned int height,
		unsigned int frame_rate_residual, unsigned int frame_rate_divider)
{
	running_weight += (width * height);
}

void coda8550j_unreserve_running_weight(unsigned int width, unsigned int height,
		unsigned int frame_rate_residual, unsigned int frame_rate_divider)
{
	running_weight -= (width * height);
}


static unsigned int _coda8550j_running_weight(void)
{
	return running_weight;
}

static unsigned long _coda8550j_utilization(void)
{
	return 0;
}

static void _coda8550j_resume(void)
{
	vlog_info("\n");

	pm_runtime_get_sync(dev);
}

static void _coda8550j_suspend(void)
{
	vlog_info("\n");

	pm_runtime_put_sync(dev);
}

void  coda8550j_clock_on(void)
{
	if (vcore_clock_state == 1) {
		vlog_warning("vcore clock already on\n");
	}
	else {
		clk_enable(img_clk);
		vcore_clock_state = 1;
	}
}

void  coda8550j_clock_off(void)
{
 	if (vcore_clock_state == 0) {
		vlog_warning("vcore clock already off\n");
	}
	else {
		clk_disable(img_clk);
		vcore_clock_state = 0;
	}
 }

void (*_coda8550j_init(unsigned int reg_base, struct device *_dev))(void)
{
	struct vcore_entry entry =
	{
		.name = "coda8550j",
		.resume = _coda8550j_resume,
		.suspend = _coda8550j_suspend,
		.get_utilization = _coda8550j_utilization,
		.get_util_estimation = NULL,
		.dec.capability = (1 << VCORE_DEC_JPEG),
		.dec.init = coda8550j_dec_init,
		.enc.capability = (1 << VCORE_ENC_JPEG) ,
		.enc.init = coda8550j_enc_init,
	};
	JpgRet ret = JPG_RET_SUCCESS;

	dev = _dev;

	vlog_info(" start \n");
	ret = JPU_Init(reg_base);
	if (ret != JPG_RET_SUCCESS)
	{
		vlog_error("JPU_Init failed Error code is 0x%x \n", ret );
	}
	jdi_spin_lock_init(&lock);

	vcore_register(&entry);

	img_clk = clk_get(NULL, "vsp1_img_clk");
	if(IS_ERR_OR_NULL(img_clk ))
		vlog_error("clk get failed\n");

	return __coda8550j_isr;
}

int coda8550j_probe(struct platform_device* pdev)
{
	struct resource res;
	int irq;
	unsigned int reg_base;
	void (*isr)(void);

	if(of_address_to_resource(pdev->dev.of_node, 0, &res) < 0)
		return -1;

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	reg_base = (unsigned int)res.start;

	isr = _coda8550j_init(reg_base, &pdev->dev);

	if (request_irq(irq, _coda8550j_isr, 0, pdev->dev.of_node->name, isr) <0)
		vlog_error("request_irq()failed. irq:%d,device_name:%s\n", irq, "coda8550j");

	if (odin_pd_register_dev(&pdev->dev, &odin_pd_vsp4_jpeg) < 0)
		return -1;

	pm_runtime_enable(&pdev->dev);

	vlog_info("vcore probed\n");

	return 0;
}

int coda8550j_remove(struct platform_device* pdev)
{
	vlog_info("\n");

	pm_runtime_suspend(&pdev->dev);
	JPU_DeInit();

	return 0;
}

int _coda8550j_runtime_resume(struct device *pdev)
{
	clk_prepare_enable(img_clk);

	return 0;
}

int _coda8550j_runtime_suspend(struct device *pdev)
{
	clk_disable_unprepare(img_clk);

	return 0;
}

static const struct dev_pm_ops  coda8550j_pm_ops =
{
	.runtime_resume = _coda8550j_runtime_resume,
	.runtime_suspend = _coda8550j_runtime_suspend,
};

static struct of_device_id coda8550j_match[] =
{
	{
		.name = "jpeg",
		.compatible = "LG,odin-jpeg"
	},
	{
	},
};

static struct platform_driver coda8550j_driver =
{
	.driver = {
		.name = "coda8550j",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr( coda8550j_match ),
		.pm = &coda8550j_pm_ops,
	},
	.probe = coda8550j_probe,
	.remove = coda8550j_remove,
};

static int __init coda8550j_init(void)
{
	int ret;

	ret = platform_driver_register(&coda8550j_driver);
	if( ret < 0 ) {
		platform_driver_unregister(&coda8550j_driver);
	}

	return ret;
}

static void __exit coda8550j_cleanup(void)
{
	platform_driver_unregister(&coda8550j_driver);
}

#ifdef CONFIG_VIDEO_ODIN_VCORE_COMPILE_AS_MODULE
module_init(coda8550j_init);
#else
late_initcall_sync(coda8550j_init);
#endif
module_exit(coda8550j_cleanup);

MODULE_AUTHOR("LGE");
MODULE_DESCRIPTION("coda8550j DRIVER");
MODULE_LICENSE("GPL");


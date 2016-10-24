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

#include <media/odin/vcodec/vlog.h>
#include "coda980/coda980.h"
#include "boda950/boda950.h"

static irqreturn_t _cnm_9x0_isr(int irq, void *isr)
{
	((void (*)(void))isr)();
	return IRQ_HANDLED;
}

static int _coda980_probe(struct platform_device* pdev)
{
	struct resource res;
	int irq;
	unsigned int reg_base;
	void (*isr)(void);

	if (of_address_to_resource(pdev->dev.of_node, 0, &res) < 0)
		return -1;

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	reg_base = (unsigned int)res.start;

	isr = coda980_init(reg_base, &pdev->dev);

	if (request_irq(irq, _cnm_9x0_isr, 0, pdev->dev.of_node->name, isr) <0)
		vlog_error("request_irq()failed. irq:%d,device_name:%s\n", irq, "coda980");

	if (odin_pd_register_dev(&pdev->dev, &odin_pd_vsp2) < 0) {
		vlog_error("odin_pd_register_dev failed\n");
		return -1;
	}

	pm_runtime_enable(&pdev->dev);

	vlog_info("vcore probed\n");

	return 0;
}

static int _coda980_remove(struct platform_device* pdev)
{
	pm_runtime_suspend(&pdev->dev);

	vlog_info("vcore removed\n");

	return 0;
}

static int _boda950_probe(struct platform_device* pdev)
{
	struct resource res;
	int irq;
	unsigned int reg_base;
	void (*isr)(void);

	if (of_address_to_resource(pdev->dev.of_node, 0, &res) < 0)
		return -1;

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	reg_base = (unsigned int)res.start;

	isr = boda950_init(reg_base, &pdev->dev);

	if (request_irq(irq, _cnm_9x0_isr, 0, pdev->dev.of_node->name, isr) <0)
		vlog_error("request_irq()failed. irq:%d,device_name:%s\n", irq, "boda950");

	if (odin_pd_register_dev(&pdev->dev, &odin_pd_vsp3) < 0) {
		vlog_error("odin_pd_register_dev failed\n");
		return -1;
	}

	pm_runtime_enable(&pdev->dev);

	vlog_info("vcore probed\n");

	return 0;
}

static int _boda950_remove(struct platform_device* pdev)
{
	pm_runtime_suspend(&pdev->dev);

	vlog_info("vcore removed\n");

	return 0;
}

static const struct dev_pm_ops _coda980_pm_ops =
{
	.runtime_resume = (int (*)(struct device *))coda980_runtime_resume,
	.runtime_suspend = (int (*)(struct device *))coda980_runtime_suspend,
	.suspend = (int (*)(struct device *))coda980_suspend,
	.resume = (int (*)(struct device *))coda980_resume,
};

static const struct dev_pm_ops _boda950_pm_ops =
{
	.runtime_resume = (int (*)(struct device *))boda950_runtime_resume,
	.runtime_suspend = (int (*)(struct device *))boda950_runtime_suspend,
	.suspend = (int (*)(struct device *))boda950_suspend,
	.resume = (int (*)(struct device *))boda950_resume,
};

static struct of_device_id _coda980_match[] =
{
	{
		.name = "codec",
		.compatible = "odin,coda980"
	},
	{
	},
};

static struct of_device_id _boda950_match[] =
{
	{
		.name = "dec",
		.compatible = "odin,boda950"
	},
	{
	},
};

static struct platform_driver _coda980_driver =
{
	.driver = {
		.name = "coda980",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr( _coda980_match ),
		.pm = &_coda980_pm_ops,
	},
	.probe = _coda980_probe,
	.remove = _coda980_remove,
};

static struct platform_driver _boda950_driver =
{
	.driver = {
		.name = "boda950",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr( _boda950_match ),
		.pm = &_boda950_pm_ops,
	},
	.probe = _boda950_probe,
	.remove = _boda950_remove,
};

static int __init cnm_9x0_init(void)
{
	int ret;

	ret = platform_driver_register(&_coda980_driver);
	if (ret < 0) {
		return ret;
	}

	ret = platform_driver_register(&_boda950_driver);
	if (ret < 0) {
		platform_driver_unregister(&_coda980_driver);
		return ret;
	}

	return ret;
}

static void __exit cnm_9x0_cleanup(void)
{
	platform_driver_unregister(&_coda980_driver);
	platform_driver_unregister(&_boda950_driver);
}

#ifdef CONFIG_VIDEO_ODIN_VCORE_COMPILE_AS_MODULE
module_init(cnm_9x0_init);
#else
late_initcall(cnm_9x0_init);
#endif
module_exit(cnm_9x0_cleanup);

MODULE_AUTHOR("LGE");
MODULE_DESCRIPTION("cnm_9x0 DRIVER");
MODULE_LICENSE("GPL");


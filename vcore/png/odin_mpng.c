
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

#include "odin_mpng.h"
#include "odin_mpng_dec.h"
#include "mpng.h"

#define	VCORE_UTILIZATION_LOG_CNT	0x100

static struct device *dev;
static struct clk *img_clk;

static int vcore_clock_state=0;
static spinlock_t lock;
static void *locked_ch;
static void (*locked_isr_func)(void *vcore_id, unsigned long reason);
static unsigned int png_running_weight = 0;
struct
{
	unsigned long prev_time;
	unsigned long run_time;
	unsigned long total_time;
	unsigned int measure_cnt;
} usage_png;


/******************************************************************************
 *
 *
 *************************************************************************** */
int odin_mpng_ch_lock(void *ch, void(*isr_func)(void *, unsigned long))
{
	unsigned long flags;

	mpng_spin_lock(&lock, &flags);

	if (locked_ch != NULL) {
		vlog_error("already locked ch(0x%X) \n", (unsigned int)locked_ch);
		mpng_spin_unlock(&lock, &flags);
		return -1;
	}

	locked_ch = ch;
	locked_isr_func = isr_func;

	mpng_spin_unlock(&lock, &flags);

	return 0;
}

int odin_mpng_ch_unlock(void *ch)
{
	unsigned long flags;

	mpng_spin_lock(&lock, &flags);

	if (locked_ch != ch) {
		vlog_error("another locked ch(0x%X, cur 0x%X) \n",
						(unsigned int)locked_ch, (unsigned int)ch);
		mpng_spin_unlock(&lock, &flags);
		return -1;
	}

	locked_ch = NULL;
	locked_isr_func = NULL;

	mpng_spin_unlock(&lock, &flags);

	return 0;
}

int odin_mpng_ch_is_locked(void *ch)
{
	unsigned long flags;

	mpng_spin_lock(&lock, &flags);

	if(locked_ch == NULL) { /* unlocked */
		mpng_spin_unlock(&lock , &flags);
		return 0;
	}
	else if(locked_ch == ch) { /* the ch is locking */
		mpng_spin_unlock(&lock , &flags);
		return 1;
	}

	/*another ch is locking */
	mpng_spin_unlock(&lock , &flags);
	return -1;
}

void odin_mpng_update_run_time(int run)
{
	unsigned long curr_time;
	unsigned long prev_time;
	unsigned long elapse_time;
	unsigned long utilization = 0;
	unsigned long flags;
	struct timespec now;

	prev_time = usage_png.prev_time;
	getnstimeofday(&now);
	curr_time = (now.tv_sec*1000*1000 + now.tv_nsec/1000) & 0x7FFFFFFF;

	mpng_spin_lock(&lock, &flags);

	if (prev_time != 0x80000000) {
		elapse_time = (curr_time >= prev_time) ? curr_time - prev_time :
										curr_time + 0x80000000 - prev_time;
		if (!elapse_time)
			elapse_time = 1;

		if (run)
			usage_png.run_time += elapse_time;

		usage_png.total_time += elapse_time;

		usage_png.measure_cnt++;
		utilization = (usage_png.run_time * 100) / usage_png.total_time;
	}
	usage_png.prev_time = curr_time;

	if (usage_png.measure_cnt > VCORE_UTILIZATION_LOG_CNT) {
		vlog_print(VLOG_VCORE_MONITOR, "utilization %lu%%\n", utilization);

		usage_png.measure_cnt = 0;
		usage_png.run_time = 0;
		usage_png.total_time = 0;
	}

	mpng_spin_unlock(&lock , &flags);
}

void __odin_mpng_isr(void)
{
	unsigned long reason;

	reason = mpng_read_register(PNG_INT_STATUS);

	if (locked_ch == NULL) {
		vlog_error("no locked channel \n");
	} else {
		reason = mpng_read_register(PNG_INT_STATUS);
		locked_isr_func(locked_ch, reason);
	}

	mpng_write_register(PNG_INT_CLEAR, 0xffffffff);
}

static irqreturn_t _odin_mpng_isr(int irq, void *isr)
{
	((void (*)(void))isr)();
	return IRQ_HANDLED;
}

void odin_mpng_reserve_running_weight(unsigned int width, unsigned int height,
	unsigned int frame_rate_residual, unsigned int frame_rate_divider)
{
	png_running_weight += (width * height);
}

void odin_mpng_unreserve_running_weight(unsigned int width, unsigned int height,
		unsigned int frame_rate_residual, unsigned int frame_rate_divider)
{
	png_running_weight -= (width * height);
}

static unsigned int _odin_mpng_running_weight(void)
{
	return png_running_weight;
}

static unsigned long _odin_mpng_utilization(void)
{
	return 0;
}

static void _odin_mpng_resume(void)
{
	pm_runtime_get_sync(dev);
}

static void _odin_mpng_suspend(void)
{
	pm_runtime_put_sync(dev);
}

void  odin_mpng_clock_on(void)
{
	if (vcore_clock_state == 1) {
		vlog_warning("vcore clock already on\n");
	}
	else {
		clk_enable(img_clk);
		vcore_clock_state = 1;
	}
}

void  odin_mpng_clock_off(void)
{
 	if (vcore_clock_state == 0) {
		vlog_warning("vcore clock already off\n");
	}
	else {
		clk_disable(img_clk);
		vcore_clock_state = 0;
	}
}

void (*_odin_mpng_init(unsigned int reg_base, struct device *_dev))(void)
{
	struct vcore_entry entry =
	{
		.name = "lge_mpng",
		.resume = _odin_mpng_resume,
		.suspend = _odin_mpng_suspend,
		.get_utilization = _odin_mpng_utilization,
		.get_util_estimation = NULL,

		.dec.capability = (1 << VCORE_DEC_PNG),
		.dec.init = odin_mpng_dec_init,
	};
	int ret = 0;

	dev = _dev;

	ret = mpng_init(reg_base);
	if (ret != 0) {
		vlog_error("lge_png_init failed Error code is 0x%x \n", ret );
	}
	mpng_spin_lock_init(&lock);

	vcore_register(&entry);

	img_clk = clk_get(NULL, "vsp1_img_clk");
	if(IS_ERR_OR_NULL(img_clk ))
		vlog_error("clk get failed\n");

	return __odin_mpng_isr;
}


int odin_mpng_probe(struct platform_device* pdev)
{
	struct resource res;
	int irq;
	unsigned int reg_base;
	void (*isr)(void);

	if(of_address_to_resource(pdev->dev.of_node, 0, &res) < 0)
		return -1;

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	reg_base = (unsigned int)res.start;

	isr = _odin_mpng_init(reg_base, &pdev->dev);

	if (request_irq(irq, _odin_mpng_isr, 0, pdev->dev.of_node->name, isr) <0)
		vlog_error("request_irq()failed. irq:%d,device_name:%s\n",
						irq, "lge_mpng");

	if (odin_pd_register_dev(&pdev->dev, &odin_pd_vsp4_png) < 0)
		return -1;

	pm_runtime_enable(&pdev->dev);

	vlog_info("lge mpng probed\n");

	return 0;
}

int odin_mpng_remove(struct platform_device* pdev)
{
	vlog_info("\n");

	pm_runtime_suspend(&pdev->dev);

	return 0;
}

int _odin_mpng_runtime_resume(struct device *pdev)
{
	vlog_info("\n");

	clk_prepare_enable(img_clk);

	return 0;
}

int _odin_mpng_runtime_suspend(struct device *pdev)
{
	vlog_info("\n");

	clk_disable_unprepare(img_clk);

	return 0;
}

static const struct dev_pm_ops  odin_mpng_pm_ops =
{
	.runtime_resume = _odin_mpng_runtime_resume,
	.runtime_suspend = _odin_mpng_runtime_suspend,
};


static struct of_device_id odin_mpng_match[] =
{
	{
		.name = "png",
		.compatible = "LG,odin-png"
	},
	{
	},
};

static struct platform_driver odin_mpng_driver =
{
	.driver = {
		.name = "odin_mpng",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr( odin_mpng_match ),
		.pm = &odin_mpng_pm_ops,
	},
	.probe = odin_mpng_probe,
	.remove = odin_mpng_remove,
};

static int __init odin_mpng_init(void)
{
	int ret;

	ret = platform_driver_register(&odin_mpng_driver);
	if( ret < 0 ) {
		platform_driver_unregister(&odin_mpng_driver);
	}

	return ret;
}

static void __exit odin_mpng_cleanup(void)
{
	platform_driver_unregister(&odin_mpng_driver);
}

#ifdef CONFIG_VIDEO_ODIN_VCORE_COMPILE_AS_MODULE
module_init(odin_mpng_init);
#else
late_initcall_sync(odin_mpng_init);
#endif
module_exit(odin_mpng_cleanup);

MODULE_AUTHOR("LGE");
MODULE_DESCRIPTION("odin_mpng DRIVER");
MODULE_LICENSE("GPL");


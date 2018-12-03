/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
 * Based on TDD v7.0 implemented by Mstar & ILITEK
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/input/mt.h>

#include "common.h"
#include "plat.h"
#include "probe.h"

#define DTS_INT_GPIO	"touch,irq-gpio"
#define DTS_RESET_GPIO	"touch,reset-gpio"
#define DTS_OF_NAME	"tchip,ilitek"

char *platform = "QCOM";

#ifdef REGULATOR_POWER_ON
void regulator_power_reg(struct ilitek_platform_data *ipd)
{
	const char *vdd_name = "vdd";
	const char *vcc_i2c_name = "vcc_i2c";

	ipd->vdd = regulator_get(&ipd->client->dev, vdd_name);

	ilitek_regulator_power_on(true);
}
#endif /* REGULATOR_POWER_ON */

/*Qcom platform dosen't need this*/
int prob_compelet_status_set(void)
{
	return 0;
}

int ilitek_gpio_to_irq(void)
{
	return gpio_to_irq(ipd->int_gpio);
}

void ilitek_get_device_gpio(void)
{
#ifdef CONFIG_OF
#if (INTERFACE == I2C_INTERFACE)
	struct device_node *dev_node = ipd->client->dev.of_node;
#else
	struct device_node *dev_node = ipd->spi->dev.of_node;
#endif
	uint32_t flag;

	ipd->int_gpio = of_get_named_gpio_flags(dev_node, DTS_INT_GPIO, 0, &flag);
	ipd->reset_gpio = of_get_named_gpio_flags(dev_node, DTS_RESET_GPIO, 0, &flag);
#endif /* CONFIG_OF */
}


#ifdef CONFIG_FB
static int ilitek_platform_notifier_fb(struct notifier_block *self, unsigned long event, void *data)
{
	int *blank;
	struct fb_event *evdata = data;

	ipio_info("Notifier's event = %ld\n", event);

	/*
	 *  FB_EVENT_BLANK(0x09): A hardware display blank change occurred.
	 *  FB_EARLY_EVENT_BLANK(0x10): A hardware display blank early change occurred.
	 */
	if (evdata && evdata->data && (event == FB_EVENT_BLANK)) {
		blank = evdata->data;

		if (*blank == FB_BLANK_POWERDOWN) {
			ipio_info("TP Suspend\n");

			if (!core_firmware->isUpgrading) {
				core_config_ic_suspend();
			}
		} else if (*blank == FB_BLANK_UNBLANK || *blank == FB_BLANK_NORMAL) {
			ipio_info("TP Resuem\n");

			if (!core_firmware->isUpgrading) {
				core_config_ic_resume();
			}
		}
	}

	return NOTIFY_OK;
}
#else /* CONFIG_HAS_EARLYSUSPEND */
static void ilitek_platform_early_suspend(struct early_suspend *h)
{
	ipio_info("TP Suspend\n");

	/* TODO: there is doing nothing if an upgrade firmware's processing. */

	core_fr_touch_release(0, 0, 0);

	input_sync(core_fr->input_device);

	core_fr->isEnableFR = false;

	core_config_ic_suspend();
}

static void ilitek_platform_late_resume(struct early_suspend *h)
{
	ipio_info("TP Resuem\n");

	core_fr->isEnableFR = true;
	core_config_ic_resume();
}
#endif

int ilitek_platform_tp_hw_reset(bool isEnable)
{
	int ret = 0;

	if (isEnable) {
		gpio_direction_output(ipd->reset_gpio, 1);
		mdelay(ipd->delay_time_high);
		gpio_set_value(ipd->reset_gpio, 0);
		mdelay(ipd->delay_time_low);
		gpio_set_value(ipd->reset_gpio, 1);
		mdelay(ipd->edge_delay);
	} else {
		gpio_set_value(ipd->reset_gpio, 0);
	}

	return ret;
}


/**
 * Register a callback function when the event of suspend and resume occurs.
 *
 * The default used to wake up the cb function comes from notifier block mechnaism.
 * If you'd rather liek to use early suspend, CONFIG_HAS_EARLYSUSPEND in kernel config
 * must be enabled.
 */
int ilitek_platform_reg_suspend(void)
{
	int ret = 0;

	ipio_info("Register suspend/resume callback function\n");
#ifdef CONFIG_FB
	ipd->notifier_fb.notifier_call = ilitek_platform_notifier_fb;
	ret = fb_register_client(&ipd->notifier_fb);
#else
	ipd->early_suspend->suspend = ilitek_platform_early_suspend;
	ipd->early_suspend->esume = ilitek_platform_late_resume;
	ipd->early_suspend->level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ret = register_early_suspend(ipd->early_suspend);

	return ret;
}

void core_fr_input_set_param(struct input_dev *input_device)
{
	int max_x = 0, max_y = 0, min_x = 0, min_y = 0;
	int max_tp = 0;

	core_fr->input_device = input_device;

	/* set the supported event type for input device */
	set_bit(EV_ABS, core_fr->input_device->evbit);
	set_bit(EV_SYN, core_fr->input_device->evbit);
	set_bit(EV_KEY, core_fr->input_device->evbit);
	set_bit(BTN_TOUCH, core_fr->input_device->keybit);
	set_bit(BTN_TOOL_FINGER, core_fr->input_device->keybit);
	set_bit(INPUT_PROP_DIRECT, core_fr->input_device->propbit);

	if (core_fr->isSetResolution) {
		max_x = core_config->tp_info->nMaxX;
		max_y = core_config->tp_info->nMaxY;
		min_x = core_config->tp_info->nMinX;
		min_y = core_config->tp_info->nMinY;
		max_tp = core_config->tp_info->nMaxTouchNum;
	} else {
		max_x = TOUCH_SCREEN_X_MAX;
		max_y = TOUCH_SCREEN_Y_MAX;
		min_x = TOUCH_SCREEN_X_MIN;
		min_y = TOUCH_SCREEN_Y_MIN;
		max_tp = MAX_TOUCH_NUM;
	}

	ipio_info("input resolution : max_x = %d, max_y = %d, min_x = %d, min_y = %d\n", max_x, max_y, min_x, min_y);
	ipio_info("input touch number: max_tp = %d\n", max_tp);

	if (core_fr->isEnablePressure)
		input_set_abs_params(core_fr->input_device, ABS_MT_PRESSURE, 0, 255, 0, 0);

#ifdef MT_B_TYPE
	#if KERNEL_VERSION(3, 7, 0) <= LINUX_VERSION_CODE
	input_mt_init_slots(core_fr->input_device, max_tp, INPUT_MT_DIRECT);
	#else
	input_mt_init_slots(core_fr->input_device, max_tp);
	#endif /* LINUX_VERSION_CODE */
#else
	input_set_abs_params(core_fr->input_device, ABS_MT_TRACKING_ID, 0, max_tp, 0, 0);
#endif /* MT_B_TYPE */

	/* Set up virtual key with gesture code */
	// core_gesture_set_key(core_fr);
}

int ilitek_platform_input_init(void)
{
	int ret = 0;

	ipd->input_device = input_allocate_device();

	if (ERR_ALLOC_MEM(ipd->input_device)) {
		ipio_err("Failed to allocate touch input device\n");
		ret = -ENOMEM;
		goto fail_alloc;
	}

#if (INTERFACE == I2C_INTERFACE)
	ipd->input_device->name = ipd->client->name;
	ipd->input_device->phys = "I2C";
	ipd->input_device->dev.parent = &ipd->client->dev;
	ipd->input_device->id.bustype = BUS_I2C;
#else
	ipd->input_device->name = DEVICE_ID;
	ipd->input_device->phys = "SPI";
	ipd->input_device->dev.parent = &ipd->spi->dev;
	ipd->input_device->id.bustype = BUS_SPI;
#endif

	core_fr_input_set_param(ipd->input_device);
	/* register the input device to input sub-system */
	ret = input_register_device(ipd->input_device);
	if (ret < 0) {
		ipio_err("Failed to register touch input device, ret = %d\n", ret);
		goto out;
	}

	return ret;

fail_alloc:
	input_free_device(core_fr->input_device);
	return ret;

out:
	input_unregister_device(ipd->input_device);
	input_free_device(core_fr->input_device);
	return ret;
}


/*
 * The name in the table must match the definiation
 * in a dts file.
 *
 */
static struct of_device_id tp_match_table[] = {
	{.compatible = DTS_OF_NAME},
	{},
};


static const struct i2c_device_id tp_device_id[] = {
	{DEVICE_ID, 0},
	{},			/* should not omitted */
};


#if (INTERFACE == I2C_INTERFACE)
static struct i2c_driver tp_i2c_driver = {
	.driver = {
		   .name = DEVICE_ID,
		   .owner = THIS_MODULE,
		   .of_match_table = tp_match_table,
		   },
	.probe = ilitek_platform_probe,
	.remove = ilitek_platform_remove,
	.id_table = tp_device_id,
};
#else
static struct spi_driver tp_spi_driver = {
	.driver = {
		.name	= DEVICE_ID,
		.owner = THIS_MODULE,
		.of_match_table = tp_match_table,
	},
	.probe = ilitek_platform_probe,
	.remove = ilitek_platform_remove,
};
#endif


static int __init ilitek_platform_init(void)
{

#if (INTERFACE == I2C_INTERFACE)
	ipio_info("TDDI TP driver add i2c interface\n");
	return i2c_add_driver(&tp_i2c_driver);
#else
	int ret = 0;
	ipio_info("TDDI TP driver add spi interface\n");
	ret = spi_register_driver(&tp_spi_driver);
	if (ret < 0) {
		ipio_err("Failed to add ilitek driver\n");
		spi_unregister_driver(&tp_spi_driver);
		return -ENODEV;
	}
#endif

	return 0;
}

static void __exit ilitek_platform_exit(void)
{
	ipio_info("I2C driver has been removed\n");

#if (INTERFACE == I2C_INTERFACE)
	i2c_del_driver(&tp_i2c_driver);
#else
	spi_unregister_driver(&tp_spi_driver);
#endif

}

MODULE_DEVICE_TABLE(i2c, tp_device_id);
module_init(ilitek_platform_init);
module_exit(ilitek_platform_exit);
MODULE_AUTHOR("ILITEK");
MODULE_LICENSE("GPL");


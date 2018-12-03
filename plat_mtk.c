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
#include "tpd.h"


// #include <linux/errno.h>
// #include <linux/types.h>
// #include <linux/kernel.h>
// #include <linux/slab.h>
// #include <linux/input.h>
// #include <linux/input/mt.h>
// #include <linux/i2c.h>
// #include <linux/list.h>

#define MTK_RST_GPIO 	GTP_RST_PORT
#define MTK_INT_GPIO 	GTP_INT_PORT
#define DTS_OF_NAME	"mediatek,cap_touch"

extern struct tpd_device *tpd;
char *platform = "MTK";

#ifdef REGULATOR_POWER_ON
void regulator_power_reg(struct ilitek_platform_data *ipd)
{
	const char *vdd_name = "vtouch";

	ipd->vdd = regulator_get(tpd->tpd_dev, vdd_name);
	tpd->reg = ipd->vdd;
}
#endif /* REGULATOR_POWER_ON */

/*MTK platform only*/
int prob_compelet_status_set(void)
{
	tpd_load_status = 1;

	return 0;
}

int ilitek_gpio_to_irq(void)
{
	struct device_node *node;
	node = of_find_matching_node(NULL, touch_of_match);
	if (node) {
		return irq_of_parse_and_map(node, 0);
	}

	return 0 ;
}

void ilitek_get_device_gpio(void)
{
	ipd->int_gpio = MTK_INT_GPIO;
	ipd->reset_gpio = MTK_RST_GPIO;
}

static void tpd_resume(struct device *h)
{
	ipio_info("TP Resume\n");

	// if (!core_firmware->isUpgrading) {
	// 	core_config_ic_resume();
	// }
}

static void tpd_suspend(struct device *h)
{
	ipio_info("TP Suspend\n");

	// if (!core_firmware->isUpgrading) {
	// 	core_config_ic_suspend();
	// }
}

int ilitek_platform_tp_hw_reset(bool isEnable)
{
	int ret = 0;

	if (isEnable) {
		tpd_gpio_output(ipd->reset_gpio, 1);
		mdelay(ipd->delay_time_high);
		tpd_gpio_output(ipd->reset_gpio, 0);
		mdelay(ipd->delay_time_low);
		tpd_gpio_output(ipd->reset_gpio, 1);
		mdelay(ipd->edge_delay);
	} else {
		tpd_gpio_output(ipd->reset_gpio, 0);
	}

	return ret;
}

/**
 * It does nothing if platform is MTK
 */
int ilitek_platform_reg_suspend(void)
{
	int ret = 0;

	ipio_info("It does nothing if platform is MTK\n");

	return ret;
}


// void core_fr_input_set_param(struct input_dev *input_device)
// {
// 	int max_x = 0, max_y = 0, min_x = 0, min_y = 0;
// 	int max_tp = 0;

// 	core_fr->input_device = input_device;

// 	/* set the supported event type for input device */
// 	set_bit(EV_ABS, core_fr->input_device->evbit);
// 	set_bit(EV_SYN, core_fr->input_device->evbit);
// 	set_bit(EV_KEY, core_fr->input_device->evbit);
// 	set_bit(BTN_TOUCH, core_fr->input_device->keybit);
// 	set_bit(BTN_TOOL_FINGER, core_fr->input_device->keybit);
// 	set_bit(INPUT_PROP_DIRECT, core_fr->input_device->propbit);

// 	if (core_fr->isSetResolution) {
// 		max_x = core_config->tp_info->nMaxX;
// 		max_y = core_config->tp_info->nMaxY;
// 		min_x = core_config->tp_info->nMinX;
// 		min_y = core_config->tp_info->nMinY;
// 		max_tp = core_config->tp_info->nMaxTouchNum;
// 	} else {
// 		max_x = TOUCH_SCREEN_X_MAX;
// 		max_y = TOUCH_SCREEN_Y_MAX;
// 		min_x = TOUCH_SCREEN_X_MIN;
// 		min_y = TOUCH_SCREEN_Y_MIN;
// 		max_tp = MAX_TOUCH_NUM;
// 	}

// 	ipio_info("input resolution : max_x = %d, max_y = %d, min_x = %d, min_y = %d\n", max_x, max_y, min_x, min_y);
// 	ipio_info("input touch number: max_tp = %d\n", max_tp);

// 	input_set_abs_params(core_fr->input_device, ABS_MT_POSITION_X, min_x, max_x - 1, 0, 0);
// 	input_set_abs_params(core_fr->input_device, ABS_MT_POSITION_Y, min_y, max_y - 1, 0, 0);

// 	input_set_abs_params(core_fr->input_device, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
// 	input_set_abs_params(core_fr->input_device, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);

// 	if (core_fr->isEnablePressure)
// 		input_set_abs_params(core_fr->input_device, ABS_MT_PRESSURE, 0, 255, 0, 0);

// #ifdef MT_B_TYPE
// 	#if KERNEL_VERSION(3, 7, 0) <= LINUX_VERSION_CODE
// 	input_mt_init_slots(core_fr->input_device, max_tp, INPUT_MT_DIRECT);
// 	#else
// 	input_mt_init_slots(core_fr->input_device, max_tp);
// 	#endif /* LINUX_VERSION_CODE */
// #else
// 	input_set_abs_params(core_fr->input_device, ABS_MT_TRACKING_ID, 0, max_tp, 0, 0);
// #endif /* MT_B_TYPE */

// 	/* Set up virtual key with gesture code */
// 	core_gesture_set_key(core_fr);
// }

/*
 * The name in the table must match the definiation
 * in a dts file.
 *
 */
static struct of_device_id tp_match_table[] = {
	{.compatible = DTS_OF_NAME},
	{},
};

#if (INTERFACE == I2C_INTERFACE)
static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	ipio_info("TPD detect i2c device\n");
	strcpy(info->type, TPD_DEVICE);
	return 0;
}
#endif


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
	.detect = tpd_detect,
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

static int tpd_local_init(void)
{
	ipio_info("TPD init device driver\n");

#if (INTERFACE == I2C_INTERFACE)
	if (i2c_add_driver(&tp_i2c_driver) != 0) {
		ipio_err("Unable to add i2c driver\n");
		return -1;
	}

	if (tpd_load_status == 0) {
		ipio_err("Add error touch panel driver\n");
		i2c_del_driver(&tp_i2c_driver);
		return -1;
	}
#else
	if (spi_register_driver(&tp_spi_driver) < 0) {
		ipio_err("Failed to add ilitek driver\n");
		spi_unregister_driver(&tp_spi_driver);
		return -ENODEV;
	}

	if (tpd_load_status == 0) {
		ipio_err("Add error touch panel driver\n");
		spi_unregister_driver(&tp_spi_drive_mtkr);
		return -1;
	}
#endif

	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
				   tpd_dts_data.tpd_key_dim_local);
	}

	tpd_type_cap = 1;

	return 0;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = DEVICE_ID,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
};

static int __init ilitek_platform_init(void)
{
	int ret = 0;
	ipio_info("TDDI TP driver add i2c interface for MTK\n");
	tpd_get_dts_info();
	ret = tpd_driver_add(&tpd_device_driver);
	if (ret < 0) {
		ipio_err("TPD add TP driver failed\n");
		tpd_driver_remove(&tpd_device_driver);
		return -ENODEV;
	}
	return 0;
}

static void __exit ilitek_platform_exit(void)
{
	ipio_info("ILITEK driver has been removed\n");

	tpd_driver_remove(&tpd_device_driver);
}

MODULE_DEVICE_TABLE(i2c, tp_device_id);
module_init(ilitek_platform_init);
module_exit(ilitek_platform_exit);
MODULE_AUTHOR("ILITEK");
MODULE_LICENSE("GPL");

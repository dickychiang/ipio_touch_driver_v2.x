/*
 * ILITEK Touch IC driver
 *
 * Copyright (C) 2011 ILI Technology Corporation.
 *
 * Author: Dicky Chiang <dicky_chiang@ilitek.com>
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
 */

#include "ilitek.h"
#include "tpd.h"

#define DTS_INT_GPIO	"touch,irq-gpio"
#define DTS_RESET_GPIO	"touch,reset-gpio"
#define DTS_OF_NAME		"mediatek,cap_touch"
#define MTK_RST_GPIO GTP_RST_PORT
#define MTK_INT_GPIO GTP_INT_PORT

extern struct tpd_device *tpd;

void ilitek_plat_tp_reset(struct ilitek_tddi_dev *idev)
{
    tpd_gpio_output(idev->tp_rst, 1);
    mdelay(idev->delay_time_high);
    tpd_gpio_output(idev->tp_rst, 0);
    mdelay(idev->delay_time_low);
    tpd_gpio_output(idev->tp_rst, 1);
    mdelay(idev->edge_delay);
}

static int ilitek_plat_input_register(struct ilitek_tddi_dev *idev)
{
	int i;

	ipio_info();

	idev->input = tpd->dev;

	if (tpd_dts_data.use_tpd_button) {
		for (i = 0; i < tpd_dts_data.tpd_key_num; i++) {
			input_set_capability(idev->input, EV_KEY, tpd_dts_data.tpd_key_local[i]);
		}
	}

	/* set the supported event type for input device */
	set_bit(EV_ABS, idev->input->evbit);
	set_bit(EV_SYN, idev->input->evbit);
	set_bit(EV_KEY, idev->input->evbit);
	set_bit(BTN_TOUCH, idev->input->keybit);
	set_bit(BTN_TOOL_FINGER, idev->input->keybit);
	set_bit(INPUT_PROP_DIRECT, idev->input->propbit);

#ifdef MT_PRESSURE
	input_set_abs_params(idev->input, ABS_MT_PRESSURE, 0, 255, 0, 0);
#endif

#ifdef MT_B_TYPE
#if KERNEL_VERSION(3, 7, 0) <= LINUX_VERSION_CODE
	input_mt_init_slots(idev->input, MAX_TOUCH_NUM, INPUT_MT_DIRECT);
#else
	input_mt_init_slots(idev->input, MAX_TOUCH_NUM);
#endif /* LINUX_VERSION_CODE */
#else
	input_set_abs_params(idev->input, ABS_MT_TRACKING_ID, 0, MAX_TOUCH_NUM, 0, 0);
#endif /* MT_B_TYPE */

	return 0;
}

static int ilitek_plat_gpio_register(struct ilitek_tddi_dev *idev)
{
	int ret = 0;

	idev->tp_int = MTK_INT_GPIO;
	idev->tp_rst = MTK_RST_GPIO;

	ipio_info("TP INT: %d\n", idev->tp_int);
	ipio_info("TP RESET: %d\n", idev->tp_rst);

	if (!gpio_is_valid(idev->tp_int)) {
		ipio_err("Invalid INT gpio: %d\n", idev->tp_int);
		return -EBADR;
	}

	if (!gpio_is_valid(idev->tp_rst)) {
		ipio_err("Invalid RESET gpio: %d\n", idev->tp_rst);
		return -EBADR;
	}

	ret = gpio_request(idev->tp_int, "TP_INT");
	if (ret < 0) {
		ipio_err("Request IRQ GPIO failed, ret = %d\n", ret);
		gpio_free(idev->tp_int);
		ret = gpio_request(idev->tp_int, "TP_INT");
		if (ret < 0) {
			ipio_err("Retrying request INT GPIO still failed , ret = %d\n", ret);
			goto out;
		}
	}

	ret = gpio_request(idev->tp_rst, "TP_RESET");
	if (ret < 0) {
		ipio_err("Request RESET GPIO failed, ret = %d\n", ret);
		gpio_free(idev->tp_rst);
		ret = gpio_request(idev->tp_rst, "TP_RESET");
		if (ret < 0) {
			ipio_err("Retrying request RESET GPIO still failed , ret = %d\n", ret);
			goto out;
		}
	}

out:
	gpio_direction_input(idev->tp_int);
	return ret;
}

void ilitek_plat_irq_disable(struct ilitek_tddi_dev *idev)
{
	unsigned long flag;

	spin_lock_irqsave(&idev->irq_spin, flag);

	if (!atomic_read(&idev->irq_status))
		goto out;

	if (!idev->irq_num) {
		ipio_err("gpio_to_irq (%d) is incorrect\n", idev->irq_num);
		goto out;
	}

	disable_irq_nosync(idev->irq_num);
	atomic_set(&idev->irq_status, IRQ_DISABLE);
	ipio_debug(DEBUG_IRQ, "Disable irq success\n");

out:
	spin_unlock_irqrestore(&idev->irq_spin, flag);
}

void ilitek_plat_irq_enable(struct ilitek_tddi_dev *idev)
{
	unsigned long flag;

	spin_lock_irqsave(&idev->irq_spin, flag);

	if (atomic_read(&idev->irq_status))
		goto out;

	if (!idev->irq_num) {
		ipio_err("gpio_to_irq (%d) is incorrect\n", idev->irq_num);
		goto out;
	}

	enable_irq(idev->irq_num);
	atomic_set(&idev->irq_status, IRQ_ENABLE);
	ipio_debug(DEBUG_IRQ, "Enable irq success\n");

out:
	spin_unlock_irqrestore(&idev->irq_spin, flag);
}

static irqreturn_t ilitek_plat_isr_top_half(int irq, void *dev_id)
{
	ipio_info();
	return IRQ_WAKE_THREAD;
}

static irqreturn_t ilitek_plat_isr_bottom_half(int irq, void *dev_id)
{
	ipio_info();

	mutex_lock(&idev->touch_mutex);

	ilitek_plat_irq_disable(idev);
	ilitek_tddi_report_handler(idev);
	ilitek_plat_irq_enable(idev);

	mutex_unlock(&idev->touch_mutex);
	return IRQ_HANDLED;
}

static int ilitek_plat_irq_register(struct ilitek_tddi_dev *idev)
{
	int ret = 0;

	struct device_node *node;

	node = of_find_matching_node(NULL, touch_of_match);
	if (node)
		idev->irq_num = irq_of_parse_and_map(node, 0);

	ipio_info("idev->irq_num = %d\n", idev->irq_num);

	ret = request_threaded_irq(idev->irq_num,
				   ilitek_plat_isr_top_half,
				   ilitek_plat_isr_bottom_half,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ilitek", NULL);

	if (ret != 0)
		ipio_err("Failed to register irq handler, irq = %d, ret = %d\n", idev->irq_num, ret);

	atomic_set(&idev->irq_status, IRQ_ENABLE);

	return ret;
}

static void tpd_resume(struct device *h)
{
	ipio_info("TP Resume\n");

	ilitek_tddi_reset_ctrl(idev, TP_RST_HW_ONLY);

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

static int ilitek_plat_probe(struct ilitek_tddi_dev *idev)
{
    ipio_info();

    ilitek_plat_gpio_register(idev);

    if (ilitek_tddi_init(idev) < 0) {
        ipio_err("Platform probe failed\n");
        return -ENODEV;
    }

    ilitek_plat_irq_register(idev);

	ilitek_plat_input_register(idev);

    /* TODO: */

 	tpd_load_status = 1;

    return 0;
}

static int ilitek_plat_remove(struct ilitek_tddi_dev *idev)
{
    ipio_info();

    ilitek_tddi_init(idev);

    return 0;
}

static struct of_device_id tp_match_table[] = {
	{.compatible = DTS_OF_NAME},
	{},
};

static struct ilitek_hwif_info hwif = {
    .bus_type = TP_BUS_I2C,
    .plat_type = TP_PLAT_MTK,
    .owner = THIS_MODULE,
    .name = TDDI_DEV_ID,
    .of_match_table = of_match_ptr(tp_match_table),
    .plat_probe = ilitek_plat_probe,
    .plat_remove = ilitek_plat_remove,
};

static int tpd_local_init(void)
{
	ipio_info("TPD init device driver\n");

    if (ilitek_tddi_dev_init(&hwif) < 0) {
        ipio_err("Failed to register i2c/spi bus driver\n");
        return -ENODEV;
    }

	if (tpd_load_status == 0) {
		ipio_err("Add error touch panel driver\n");
		// i2c_del_driver(&tp_i2c_driver);
		// spi_unregister_driver(&tp_spi_driver);
		return -1;
	}

	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
				   tpd_dts_data.tpd_key_dim_local);
	}

	tpd_type_cap = 1;

	return 0;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = TDDI_DEV_ID,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
};

static int __init ilitek_plat_dev_init(void)
{
	int ret = 0;

	ipio_info("ILITEK TP driver init for MTK\n");

	tpd_get_dts_info();

	ret = tpd_driver_add(&tpd_device_driver);
	if (ret < 0) {
		ipio_err("ILITEK add TP driver failed\n");
		tpd_driver_remove(&tpd_device_driver);
		return -ENODEV;
	}

	return 0;
}

static void __exit ilitek_plat_dev_exit(void)
{
	ipio_info("ilitek driver has been removed\n");

	tpd_driver_remove(&tpd_device_driver);
}

module_init(ilitek_plat_dev_init);
module_exit(ilitek_plat_dev_exit);
MODULE_AUTHOR("ILITEK");
MODULE_LICENSE("GPL");
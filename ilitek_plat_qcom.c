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

#define DTS_INT_GPIO	"touch,irq-gpio"
#define DTS_RESET_GPIO	"touch,reset-gpio"
#define DTS_OF_NAME		"tchip,ilitek"

void ilitek_plat_tp_reset(struct ilitek_tddi_dev *idev)
{
	gpio_direction_output(ipd->reset_gpio, 1);
	mdelay(ipd->delay_time_high);
	gpio_set_value(ipd->reset_gpio, 0);
	mdelay(ipd->delay_time_low);
	gpio_set_value(ipd->reset_gpio, 1);
	mdelay(ipd->edge_delay);
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

	if (atomic_read(&idev->irq_stat) == IRQ_DISABLE)
		goto out;

	if (!idev->irq_num) {
		ipio_err("gpio_to_irq (%d) is incorrect\n", idev->irq_num);
		goto out;
	}

	disable_irq_nosync(idev->irq_num);
	atomic_set(&idev->irq_stat, IRQ_DISABLE);
	ipio_debug(DEBUG_IRQ, "Disable irq success\n");

out:
	spin_unlock_irqrestore(&idev->irq_spin, flag);
}

void ilitek_plat_irq_enable(struct ilitek_tddi_dev *idev)
{
	unsigned long flag;

	spin_lock_irqsave(&idev->irq_spin, flag);

	if (atomic_read(&idev->irq_stat) == IRQ_ENABLE)
		goto out;

	if (!idev->irq_num) {
		ipio_err("gpio_to_irq (%d) is incorrect\n", idev->irq_num);
		goto out;
	}

	enable_irq(idev->irq_num);
	atomic_set(&idev->irq_stat, IRQ_ENABLE);
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
	//core_fr_handler();
	ilitek_plat_irq_enable(idev);

	mutex_unlock(&idev->touch_mutex);

	return IRQ_HANDLED;
}

static int ilitek_plat_irq_register(struct ilitek_tddi_dev *idev)
{
	int ret = 0;

	ipd->isr_gpio = gpio_to_irq(ipd->int_gpio);

	ipio_info("ipd->isr_gpio = %d\n", ipd->isr_gpio);

	ret = request_threaded_irq(ipd->isr_gpio,
				   ilitek_plat_isr_top_half,
				   ilitek_plat_isr_bottom_half, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ilitek", NULL);

	if (ret != 0) {
		ipio_err("Failed to register irq handler, irq = %d, ret = %d\n", ipd->isr_gpio, ret);
		goto out;
	}

	ipd->isEnableIRQ = true;

out:
	return ret;
}

static int ilitek_plat_probe(struct ilitek_tddi_dev *idev)
{
    ipio_info();

    ilitek_plat_gpio_register();

    ilitek_plat_irq_register();

    if (ilitek_tddi_init(idev) < 0) {
        ipio_err("Platform probe failed\n");
        return -ENODEV;
    }

    /* TODO: */

    return 0;
}

static int ilitek_plat_remove(struct ilitek_tddi_dev *idev)
{
    ipio_info();
    return 0;
}

static struct of_device_id tp_match_table[] = {
	{.compatible = DTS_OF_NAME},
	{},
};

static struct ilitek_hwif_info hwif = {
    .bus_type = TP_BUS_SPI,
    .plat_type = TP_PLAT_QCOM,
    .owner = THIS_MODULE,
    .name = TDDI_DEV_ID,
    .of_match_table = of_match_ptr(tp_match_table),
    .plat_probe = ilitek_plat_probe,
    .plat_remove = ilitek_plat_remove,
};

static int __init ilitek_plat_dev_init(void)
{
	int ret = 0;

	ipio_info("ILITEK TP driver init for MTK\n");

    if (ilitek_tddi_dev_init(&hwif) < 0) {
        ipio_err("Failed to register i2c/spi bus driver\n");
        return -ENODEV;
    }

	return 0;
}

static void __exit ilitek_plat_dev_exit(void)
{
	ipio_info("ilitek driver has been removed\n");
}

module_init(ilitek_plat_dev_init);
module_exit(ilitek_plat_dev_exit);
MODULE_AUTHOR("ILITEK");
MODULE_LICENSE("GPL");
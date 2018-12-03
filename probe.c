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
#include "common.h"
#include "probe.h"
#include "plat.h"



#ifdef USE_KTHREAD
static DECLARE_WAIT_QUEUE_HEAD(waiter);
#endif

/* Debug level */
uint32_t ipio_debug_level = DEBUG_ALL;
struct ilitek_platform_data *ipd = NULL;



#ifdef REGULATOR_POWER_ON
void ilitek_regulator_power_on(bool status)
{
	int ret = 0;

	ipio_info("%s\n", status ? "POWER ON" : "POWER OFF");

	if (status) {
		if (ipd->vdd) {
			ret = regulator_enable(ipd->vdd);
			if (ret < 0)
				ipio_err("regulator_enable vdd fail\n");
		}
		if (ipd->vdd_i2c) {
			ret = regulator_enable(ipd->vdd_i2c);
			if (ret < 0)
				ipio_err("regulator_enable vdd_i2c fail\n");
		}
	} else {
		if (ipd->vdd) {
			ret = regulator_disable(ipd->vdd);
			if (ret < 0)
				ipio_err("regulator_enable vdd fail\n");
		}
		if (ipd->vdd_i2c) {
			ret = regulator_disable(ipd->vdd_i2c);
			if (ret < 0)
				ipio_err("regulator_enable vdd_i2c fail\n");
		}
	}
	core_config->icemodeenable = false;
	mdelay(5);
}

static void ilitek_regulator_power_reg(struct ilitek_platform_data *ipd)
{

	regulator_power_reg(ipd);

	if (ERR_ALLOC_MEM(ipd->vdd)) {
		ipio_err("regulator_get vdd fail\n");
		ipd->vdd = NULL;
	} else {
		if (regulator_set_voltage(ipd->vdd, VDD_VOLTAGE, VDD_VOLTAGE) < 0)
			ipio_err("Failed to set vdd %d.\n", VDD_VOLTAGE);
	}

	ipd->vdd_i2c = regulator_get(ipd->dev, vcc_i2c_name);
	if (ERR_ALLOC_MEM(ipd->vdd_i2c)) {
		ipio_err("regulator_get vdd_i2c fail.\n");
		ipd->vdd_i2c = NULL;
	} else {
		if (regulator_set_voltage(ipd->vdd_i2c, VDD_I2C_VOLTAGE, VDD_I2C_VOLTAGE) < 0)
			ipio_err("Failed to set vdd_i2c %d\n", VDD_I2C_VOLTAGE);
	}
	ilitek_regulator_power_on(true);
}
#endif /* REGULATOR_POWER_ON */



static int ilitek_platform_gpio(void)
{
	int ret = 0;

	ilitek_get_device_gpio();

	ipio_info("GPIO INT: %d\n", ipd->int_gpio);
	ipio_info("GPIO RESET: %d\n", ipd->reset_gpio);

	if (!gpio_is_valid(ipd->int_gpio)) {
		ipio_err("Invalid INT gpio: %d\n", ipd->int_gpio);
		return -EBADR;
	}

	if (!gpio_is_valid(ipd->reset_gpio)) {
		ipio_err("Invalid RESET gpio: %d\n", ipd->reset_gpio);
		return -EBADR;
	}

	ret = gpio_request(ipd->int_gpio, "ILITEK_TP_IRQ");
	if (ret < 0) {
		ipio_err("Request IRQ GPIO failed, ret = %d\n", ret);
		gpio_free(ipd->int_gpio);
		ret = gpio_request(ipd->int_gpio, "ILITEK_TP_IRQ");
		if (ret < 0) {
			ipio_err("Retrying request INT GPIO still failed , ret = %d\n", ret);
			goto out;
		}
	}

	ret = gpio_request(ipd->reset_gpio, "ILITEK_TP_RESET");
	if (ret < 0) {
		ipio_err("Request RESET GPIO failed, ret = %d\n", ret);
		gpio_free(ipd->reset_gpio);
		ret = gpio_request(ipd->reset_gpio, "ILITEK_TP_RESET");
		if (ret < 0) {
			ipio_err("Retrying request RESET GPIO still failed , ret = %d\n", ret);
			goto out;
		}
	}

	gpio_direction_input(ipd->int_gpio);

out:
	return ret;
}


int ilitek_platform_reset_ctrl(bool rst, int mode)
{
	int ret = 0;

	atomic_set(&ipd->do_reset, true);

	switch (mode) {
		case SW_RST:
			ipio_info("SW RESET\n");
			// ret = core_config_ic_reset();
			break;
		case HW_RST:
			ipio_info("HW RESET\n");
			ilitek_platform_tp_hw_reset(rst);
			break;
		case HOST_DOWNLOAD_RST:

			break;
		case HOST_DOWNLOAD_BOOT_RST:

			break;
		default:
			ipio_err("Unknown RST mode (%d)\n", mode);
			ret = -1;
			break;
	}

	atomic_set(&ipd->do_reset, false);
	return ret;
}

#if (INTERFACE == I2C_INTERFACE)
int ilitek_platform_remove(struct i2c_client *client)
#else
int ilitek_platform_remove(struct spi_device *spi)
#endif
{
	ipio_info("Remove platform components\n");

	if (ipd->isEnableIRQ) {
		disable_irq_nosync(ipd->isr_gpio);
	}

	if (ipd->isr_gpio != 0 && ipd->int_gpio != 0 && ipd->reset_gpio != 0) {
		free_irq(ipd->isr_gpio, (void *)ipd->i2c_id);
		gpio_free(ipd->int_gpio);
		gpio_free(ipd->reset_gpio);
	}
#ifdef CONFIG_FB
	fb_unregister_client(&ipd->notifier_fb);
#else
	unregister_early_suspend(&ipd->early_suspend);
#endif /* CONFIG_FB */

#ifdef USE_KTHREAD
	if (ipd->irq_thread != NULL) {
		ipd->irq_trigger = true;
		ipd->free_irq_thread = true;
		wake_up_interruptible(&waiter);
		kthread_stop(ipd->irq_thread);
		ipd->irq_thread = NULL;
	}
#endif /* USE_KTHREAD */

	if (ipd->input_device != NULL) {
		input_unregister_device(ipd->input_device);
		input_free_device(ipd->input_device);
	}

	if (ipd->vpower_reg_nb) {
		cancel_delayed_work_sync(&ipd->check_power_status_work);
		destroy_workqueue(ipd->check_power_status_queue);
	}

	if (ipd->vesd_reg_nb) {
		cancel_delayed_work_sync(&ipd->check_esd_status_work);
		destroy_workqueue(ipd->check_esd_status_queue);
	}

	// ilitek_proc_remove();
	return 0;
}


#if (INTERFACE == I2C_INTERFACE)
static int ilitek_interface_check(struct i2c_client *client, const struct i2c_device_id *id)
{
	if (client == NULL) {
		ipio_err("i2c client is NULL\n");
		return -ENODEV;
	}

		/* Set i2c slave addr if it's not configured */
	ipio_info("I2C Slave address = 0x%x\n", client->addr);
	if (client->addr != ILITEK_I2C_ADDR) {
		client->addr = ILITEK_I2C_ADDR;
		ipio_err("I2C Slave addr doesn't be set up, use default : 0x%x\n", client->addr);
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ipio_err("I2C not supported\n");
		return -ENODEV;
	}

	ipd = devm_kzalloc(&client->dev, sizeof(struct ilitek_platform_data), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ipd)) {
		ipio_err("Failed to allocate ipd memory, %ld\n", PTR_ERR(ipd));
		return -ENOMEM;
	}

	ipd->client = client;
	ipd->i2c_id = id;
	ipd->dev = &client->dev;

	// if (core_i2c_init()) {
	// 	ipio_err("Failed to init I2C\n");
	// 	return -ENOMEM;
	// }

	return 0;
}
#else
static int ilitek_interface_check(struct spi_device *spi)
{
	if (spi == NULL) {
		ipio_err("spi device is NULL\n");
		return -ENODEV;
	}

	ipd = devm_kzalloc(&spi->dev, sizeof(struct ilitek_platform_data), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ipd)) {
		ipio_err("Failed to allocate ipd memory, %ld\n", PTR_ERR(ipd));
		return -ENOMEM;
	}

	ipd->spi = spi;
	ipd->dev = &spi->dev;

	return 0;
}
#endif

static int ilitek_configuration_set(void)
{
//HW RESET
	ilitek_platform_reset_ctrl(HW_RST);

//GET CHIP ID
	touch_get_chip_id();


}

/**
 * The probe func would be called after an i2c device was detected by kernel.
 *
 * It will still return zero even if it couldn't get a touch ic info.
 * The reason for why we allow it passing the process is because users/developers
 * might want to have access to ICE mode to upgrade a firwmare forcelly.
 */
#if (INTERFACE == I2C_INTERFACE)
int ilitek_platform_probe(struct i2c_client *client, const struct i2c_device_id *id)
#else
int ilitek_platform_probe(struct spi_device *spi)
#endif
{
#if (INTERFACE == I2C_INTERFACE)
	if(ilitek_interface_check(client, id) < 0)
		return -ENODEV;
#else
	if(ilitek_interface_check(spi) < 0)
		return -ENODEV;
#endif

	if (ilitek_platform_gpio() < 0)
		ipio_err("Failed to request gpios\n ");

#ifdef REGULATOR_POWER_ON
	ilitek_regulator_power_reg(ipd);
#endif

	if (ilitek_platform_reg_suspend() < 0)
		ipio_err("Failed to register suspend/resume function\n");

	prob_compelet_status_set();

	return 0;
}


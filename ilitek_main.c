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

#if (PLATFORM == MTK)
#define DTS_OF_NAME		"mediatek,cap_touch"
#include "tpd.h"
extern struct tpd_device *tpd;
#define MTK_RST_GPIO GTP_RST_PORT
#define MTK_INT_GPIO GTP_INT_PORT
#else
#define DTS_OF_NAME		"tchip,ilitek"
#endif /* MTK */

#define DEVICE_ID	"ILITEK_TDDI"

struct ilitek_tddi_dev
{
    struct i2c_client *client;
    struct spi_device *spi;
    struct input_dev *input;
    struct device *dev;

	struct mutex touch_mutex;
	struct mutex io_mutex;
	spinlock_t irq_spin;

	int irq_num;
	int tp_rst;
	int tp_int;

	atomic_t irq_enable;

    int (*i2c_write)(struct ilitek_tddi_data, void *buf, size_t size);
    int (*i2c_read)(struct ilitek_tddi_data, void *buf, size_t size);
    int (*spi_write)(struct ilitek_tddi_data, void *buf, size_t size);
    int (*spi_read)(struct ilitek_tddi_data, void *buf, size_t size);
    int (*suspend)(struct ilitek_tddi_data);
    int (*resume)(struct ilitek_tddi_data);
    int (*reset)(struct ilitek_tddi_data, int mode);
    int (*irq_enable)(struct ilitek_tddi_data, bool enable);


    irq_handler_t (*top_half)(int irq, void *dev_id);
    irq_handler_t (*bottom_half)(int irq, void *dev_id);
};

struct ilitek_tddi_dev *idev = NULL;

void ilitek_irq_disable(struct ilitek_tddi_data *idev)
{
	unsigned long flag;

	spin_lock_irqsave(&idev->irq_spin, flag);

	if (!atomic_read(&idev->irq_enable))
		goto out;

	if (!idev->irq_num) {
		ipio_err("gpio_to_irq (%d) is incorrect\n", idev->irq_num);
		goto out;
	}

	disable_irq_nosync(idev->irq_num);
	atomic_set(&idev->irq_enable, false);
	ipio_debug(DEBUG_IRQ, "Disable irq success\n");

out:
	spin_unlock_irqrestore(&idev->irq_spin, flag);
}

void ilitek_irq_enable(struct ilitek_tddi_data *idev)
{
	unsigned long flag;

	spin_lock_irqsave(&idev->irq_spin, flag);

	if (atomic_read(&idev->irq_enable))
		goto out;

	if (!idev->irq_num) {
		ipio_err("gpio_to_irq (%d) is incorrect\n", idev->irq_num);
		goto out;
	}

	enable_irq(idev->irq_num);
	atomic_set(&idev->irq_enable, true);
	ipio_debug(DEBUG_IRQ, "Enable irq success\n");

out:
	spin_unlock_irqrestore(&idev->irq_spin, flag);
}

irq_handler_t ilitek_isr_top_half(int irq, void *dev_id)
{
	ipio_info();
	return IRQ_WAKE_THREAD;
}

irq_handler_t ilitek_isr_bottom_half(int irq, void *dev_id)
{
	ipio_info();

	mutex_lock(&ipd->touch_mutex);

	ilitek_irq_disable(idev);
	//core_fr_handler();
	ilitek_irq_enable(idev);

	mutex_unlock(&ipd->touch_mutex);

	return IRQ_HANDLED;
}

int ilitek_i2c_write(struct ilitek_tddi_data *idev, void *buf, size_t len)
{
    int ret = 0;

    mutex_lock(&idev->io_lock);

    ret = core_i2c_write(idev->client, buf, len);

    mutex_unlock(&idev->io_lock);

    return ret;
}

int ilitek_i2c_read(struct ilitek_tddi_data *idev, void *buf, size_t len)
{
    int ret = 0;

    mutex_lock(&idev->io_lock);

    ret = core_i2c_read(idev->client, buf, len);

    mutex_unlock(&idev->io_lock);

    return ret;
}

int ilitek_spi_write(struct ilitek_tddi_data *idev, void *buf, size_t len)
{
    int ret = 0;

    mutex_lock(&idev->io_lock);

    ret = core_spi_write(idev->spi, buf, len);

    mutex_unlock(&idev->io_lock);

    return ret;
}

int ilitek_spi_read(struct ilitek_tddi_data *idev, void *buf, size_t len)
{
    int ret = 0;

    mutex_lock(&idev->io_lock);

    ret = core_spi_read(idev->spi, buf, len);

    mutex_unlock(&idev->io_lock);

    return ret;
}

static int ilitek_gpio_register(struct ilitek_tddi_data *idev)
{
	int ret = 0;

#if (PLATFORM == MTK)
	idev->tp_int = MTK_INT_GPIO;
	idev->tp_rst = MTK_RST_GPIO;
#else
	uint32_t flag;
	struct device_node *dev_node = NULL;

	if (!idev->spi)
		dev_node = idev->spi->dev.of_node;
	else
		dev_node = idev->client->dev.of_node;

	idev->tp_int = of_get_named_gpio_flags(dev_node, DTS_INT_GPIO, 0, &flag);
	idev->tp_rst = of_get_named_gpio_flags(dev_node, DTS_RESET_GPIO, 0, &flag);
#endif /* MTK */

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

	gpio_direction_input(idev->tp_int);
	return ret;
}

static int ilitek_irq_register(struct ilitek_tddi_data *idev)
{
	int ret = 0;

#if (PLATFORM == MTK)
	struct device_node *node;

	node = of_find_matching_node(NULL, touch_of_match);
	if (node)
		idev->irq_num = irq_of_parse_and_map(node, 0);
#else
	idev->irq_num = gpio_to_irq(idev->tp_int);
#endif

	ipio_info("idev->irq_num = %d\n", idev->irq_num);

	ret = request_threaded_irq(idev->irq_num,
				   ilitek_isr_top_half,
				   ilitek_isr_bottom_half, IRQF_TRIGGER_FALLING | IRQF_ONESHOT, "ilitek", NULL);

	if (ret != 0)
		ipio_err("Failed to register irq handler, irq = %d, ret = %d\n", idev->irq_num, ret);

	atomic_set(&idev->irq_enable, true);

	return ret;
}

static int ilitek_tddi_init(struct ilitek_tddi_data *idev)
{
	ipio_info();

	mutex_init(&idev->io_mutex);
	mutex_init(&idev->touch_mutex);
	spin_lock_init(&idev->irq_spin);

	irq_enable = ATOMIC_INIT(0);

    idev->spi_write = ilitek_spi_write;
    idev->spi_read = ilitek_spi_read;
    idev->i2c_write = ilitek_i2c_write;
    idev->i2c_read = ilitek_i2c_read;

	ilitek_gpio_register(idev);
	ilitek_irq_register(idev);
}

#if (INTERFACE == I2C)
static int ilitek_plat_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	if (client == NULL) {
		ipio_err("i2c client is NULL\n");
		return -ENODEV;
	}

	/* Set i2c slave addr if it's not configured */
	ipio_info("i2c addr = 0x%x\n", client->addr);
	if (client->addr != TDDI_I2C_ADDR) {
		client->addr = TDDI_I2C_ADDR;
		ipio_err("i2c addr doesn't be set up, use default : 0x%x\n", client->addr);
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		ipio_err("i2c functions are not supported!\n");
		return -ENODEV;
	}

	idev = devm_kzalloc(&client->dev, sizeof(struct ilitek_tddi_data), GFP_KERNEL);
	if (ERR_ALLOC_MEM(idev)) {
		ipio_err("Failed to allocate idev memory, %ld\n", PTR_ERR(idev));
		return -ENOMEM;
	}

	idev->client = client;
	idev->dev = &client->dev;

	ilitek_tddi_init(idev);

	return 0;
}
#else
static int ilitek_plat_probe(struct spi_device *spi)
{
	if (spi == NULL) {
		ipio_err("spi device is NULL\n");
		return -ENODEV;
	}

	idev = devm_kzalloc(&spi->dev, sizeof(struct ilitek_tddi_data), GFP_KERNEL);
	if (ERR_ALLOC_MEM(idev)) {
		ipio_err("Failed to allocate idev memory, %ld\n", PTR_ERR(idev));
		return -ENOMEM;
	}

	idev->spi = spi;
	idev->dev = &spi->dev;

	ilitek_tddi_init(idev);

	return 0;
}
#endif /* I2C */


static const struct i2c_device_id tp_device_id[] = {
	{DEVICE_ID, 0},
	{},	/* should not omitted */
};

MODULE_DEVICE_TABLE(i2c, tp_device_id);

static struct of_device_id tp_match_table[] = {
	{.compatible = DTS_OF_NAME},
	{},
};

#if (PLATFORM == MTK && INTERFACE == I2C)
static int tpd_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	ipio_info("TPD detect i2c device\n");
	strcpy(info->type, TPD_DEVICE);
	return 0;
}
#endif /* MTK */

#if (INTERFACE == I2C)
static struct i2c_driver tp_i2c_driver = {
	.driver = {
		   .name = DEVICE_ID,
		   .owner = THIS_MODULE,
		   .of_match_table = tp_match_table,
		   },
	.probe = ilitek_plat_probe,
	.remove = ilitek_plat_remove,
	.id_table = tp_device_id,
#if (PLATFORM == MTK)
	.detect = tpd_detect,
#endif /* MTK */
};
#else
static struct spi_driver tp_spi_driver = {
	.driver = {
		.name	= DEVICE_ID,
		.owner = THIS_MODULE,
		.of_match_table = tp_match_table,
	},
	.probe = ilitek_plat_probe,
	.remove = ilitek_platf_remove,
};
#endif

#if (PLATFORM == MTK)
static int tpd_local_init(void)
{
	ipio_info("TPD init device driver\n");

#if (INTERFACE == I2C)
	if (i2c_add_driver(&tp_i2c_driver) != 0) {
		ipio_err("Unable to add i2c driver\n");
		return -1;
	}
#else
	if (spi_register_driver(&tp_spi_driver) < 0) {
		ipio_err("Failed to add ilitek driver\n");
		spi_unregister_driver(&tp_spi_driver);
		return -ENODEV;
	}
#endif /* I2C */

	if (tpd_load_status == 0) {
		ipio_err("Add error touch panel driver\n");
#if (INTERFACE == I2C)
		i2c_del_driver(&tp_i2c_driver);
#else
		spi_unregister_driver(&tp_spi_driver);
#endif /* I2C */
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
	.tpd_device_name = DEVICE_ID,
	.tpd_local_init = tpd_local_init,
	.suspend = tpd_suspend,
	.resume = tpd_resume,
};
#endif /* MTK */


static int __init ilitek_plat_init(void)
{
#if (PLATFORM == MTK)
	int ret = 0;
	ipio_info("ILITEK TP driver init for MTK\n");
	tpd_get_dts_info();
	ret = tpd_driver_add(&tpd_device_driver);
	if (ret < 0) {
		ipio_err("ILITEK add TP driver failed\n");
		tpd_driver_remove(&tpd_device_driver);
		return -ENODEV;
	}
#else
#if (INTERFACE == I2C)
	ipio_info("ILITEK TP driver init with i2c\n");
	return i2c_add_driver(&tp_i2c_driver);
#else
	int ret = 0;
	ipio_info("ILITEK TP driver init with spi\n");
	ret = spi_register_driver(&tp_spi_driver);
	if (ret < 0) {
		ipio_err("ILITEK add TP driver failed\n");
		spi_unregister_driver(&tp_spi_driver);
		return -ENODEV;
	}
#endif
#endif /* MTK */
	return 0;
}

static void __exit ilitek_plat_init(void)
{
	ipio_info("ilitek driver has been removed\n");

#if (PLATFORM == MTK)
	tpd_driver_remove(&tpd_device_driver);
#else
#if (INTERFACE == I2C)
	i2c_del_driver(&tp_i2c_driver);
#else
	spi_unregister_driver(&tp_spi_driver);
#endif
#endif /* MTK */
}

module_init(ilitek_platform_init);
module_exit(ilitek_platform_exit);
MODULE_AUTHOR("ILITEK");
MODULE_LICENSE("GPL");
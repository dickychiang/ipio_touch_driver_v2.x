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

struct touch_bus_info {
	struct i2c_driver bus_driver;
	struct ilitek_hwif_info *hwif;
};

struct ilitek_tddi_dev *idev = NULL;

static int core_i2c_write(struct ilitek_tddi_dev *idev, void *buf, size_t len)
{
    u8 *txbuf = (u8 *)buf;

	struct i2c_msg msgs[] = {
		{
		 .addr = idev->i2c->addr,
		 .flags = 0,	/* write flag. */
		 .len = len,
		 .buf = txbuf,
		 },
	};

	if (i2c_transfer(idev->i2c->adapter, msgs, 1) < 0)
		return -EIO;

	return 0;
}

static int core_i2c_read(struct ilitek_tddi_dev *idev, void *buf, size_t len)
{
    u8 *rxbuf = (u8 *)buf;

	struct i2c_msg msgs[] = {
		{
		 .addr = idev->i2c->addr,
		 .flags = I2C_M_RD,	/* read flag. */
		 .len = len,
		 .buf = rxbuf,
		 },
	};

	if (i2c_transfer(idev->i2c->adapter, msgs, 1) < 0)
		return -EIO;

	return 0;
}

static int ilitek_i2c_write(struct ilitek_tddi_dev *idev, void *buf, size_t len)
{
    int ret = 0;

    mutex_lock(&idev->io_mutex);

    ret = core_i2c_write(idev, buf, len);
    if (ret < 0) {
		if (atomic_read(&idev->tp_reset) == TP_RST_START) {
			ret = 0;
			goto out;
		}
		ipio_err("i2c write error, ret = %d\n", ret);
	}

out:
    mutex_unlock(&idev->io_mutex);
    return ret;
}

static int ilitek_i2c_read(struct ilitek_tddi_dev *idev, void *buf, size_t len)
{
    int ret = 0;

    mutex_lock(&idev->io_mutex);

    ret = core_i2c_read(idev, buf, len);
    if (ret < 0) {
		if (atomic_read(&idev->tp_reset) == TP_RST_START) {
			ret = 0;
			goto out;
		}
		ipio_err("i2c read error, ret = %d\n", ret);
	}

out:
    mutex_unlock(&idev->io_mutex);
    return ret;
}

static int ilitek_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct touch_bus_info *info =
		container_of(to_i2c_driver(i2c->dev.driver),
			struct touch_bus_info, bus_driver);

    ipio_info();

	if (!i2c) {
		ipio_err("i2c client is NULL\n");
		return -ENODEV;
	}

    ipio_info("bus type = %d\n", info->hwif->bus_type);
    ipio_info("platform type = %d\n", info->hwif->plat_type);

	if (i2c->addr != TDDI_I2C_ADDR) {
		i2c->addr = TDDI_I2C_ADDR;
		ipio_info("i2c addr doesn't be set up, use default : 0x%x\n", i2c->addr);
	}

	if (!i2c_check_functionality(i2c->adapter, I2C_FUNC_I2C)) {
		ipio_err("i2c functions are not supported!\n");
		return -ENODEV;
	}

    idev = devm_kzalloc(&i2c->dev, sizeof(struct ilitek_tddi_dev), GFP_KERNEL);
    if (ERR_ALLOC_MEM(idev)) {
        ipio_err("Failed to allocate idev memory, %ld\n", PTR_ERR(idev));
        return -ENOMEM;
    }

	idev->i2c = i2c;
	idev->dev = &i2c->dev;

    idev->write = ilitek_i2c_write;
    idev->read = ilitek_i2c_read;

    idev->reset_mode = TP_RST_HW_ONLY;
    idev->fw_upgrade_mode = UPGRADE_FLASH;

	idev->delay_time_high = 10;
	idev->delay_time_low = 5;
	idev->edge_delay = 100;

    return info->hwif->plat_probe(idev);
}

static int ilitek_i2c_remove(struct i2c_client *i2c)
{
	struct touch_bus_info *info =
		container_of(to_i2c_driver(i2c->dev.driver),
			struct touch_bus_info, bus_driver);

    ipio_info();

    info->hwif->plat_remove(idev);

    return 0;
}

static const struct i2c_device_id tp_i2c_id[] = {
	{TDDI_DEV_ID, 0},
};

int ilitek_i2c_dev_init(struct ilitek_hwif_info *hwif)
{
    struct touch_bus_info *info;

    info = kzalloc(sizeof(*info), GFP_KERNEL);

    ipio_info();

	if (!info) {
		ipio_err("faied to allocate i2c_driver\n");
		return -ENOMEM;
	}

    hwif->info = info;

    info->bus_driver.driver.name = hwif->name;
    info->bus_driver.driver.owner = hwif->owner;
    info->bus_driver.driver.of_match_table = hwif->of_match_table;

	info->bus_driver.probe = ilitek_i2c_probe;
	info->bus_driver.remove = ilitek_i2c_remove;
	info->bus_driver.id_table = tp_i2c_id;

    info->hwif = hwif;

    return i2c_add_driver(&info->bus_driver);
}
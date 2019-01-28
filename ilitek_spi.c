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

#define SPI_WRITE 		0x82
#define SPI_READ 		0x83
#define SPI_CLK_HZ		(10 * 1024 * 1024)
#define SPI_RETRY		5
#define DMA_TRANSFER_MAX_TIMES 2
#define DMA_TRANSFER_MAX_SIZE 1024
#define SPI_WRITE_BUFF_MAXSIZE (1024 * DMA_TRANSFER_MAX_TIMES + 5)//plus 5 for IC Mode :(Head + Address) 0x82,0x25,Addr_L,Addr_M,Addr_H
#define SPI_READ_BUFF_MAXSIZE  (1024 * DMA_TRANSFER_MAX_TIMES)

#define CHECK_RECOVER 			-2

static int (*ilitek_spi_write_then_read)(struct spi_device *spi,
		const void *txbuf, unsigned n_tx,
			void *rxbuf, unsigned n_rx);

/* Declare dma buffer as 4 byte alignment */
__attribute__ ((section ("NONCACHEDRW"), aligned(4)))
uint8_t dma_txbuf[SPI_WRITE_BUFF_MAXSIZE] = {0};
__attribute__ ((section ("NONCACHEDRW"), aligned(4)))
uint8_t dma_rxbuf[SPI_READ_BUFF_MAXSIZE] = {0};

struct touch_bus_info {
	struct spi_driver bus_driver;
    struct ilitek_hwif_info *hwif;
};

struct ilitek_tddi_dev *idev = NULL;

/*
 * As spi_write_then_read() in kernel can't guarante the data we
 * want to send to or read from is always 4 bytes aligned via DMA transimission.
 *
 * This function works for that and limits the buffer of DMA is at 1024byte. You
 * can change it by request.
 */
#ifdef CONFIG_MTK_SPI
static int core_mtk_spi_write_then_read(struct spi_device *spi,
		const void *txbuf, unsigned n_tx,
		void *rxbuf, unsigned n_rx)
{
	static DEFINE_MUTEX(lock);

	int	status = -1;
	int xfercnt = 0, xferlen = 0, xferloop = 0;
	uint8_t cmd, temp1[1] = {0}, temp2[1] = {0};
	struct spi_message	message;
	struct spi_transfer	xfer[DMA_TRANSFER_MAX_TIMES + 1];

	if (n_tx > (SPI_WRITE_BUFF_MAXSIZE)) {
		ipio_err("[spi write] Exceeded transmission length, %d > Max length:%d\n", n_tx, SPI_WRITE_BUFF_MAXSIZE);
		goto out;
	}
	if (n_rx > (SPI_READ_BUFF_MAXSIZE)) {
		ipio_err("[spi read] Exceeded transmission length, %d > Max length:%d\n", n_rx, SPI_READ_BUFF_MAXSIZE);
		goto out;
	}

	mutex_trylock(&lock);

	spi_message_init(&message);
	memset(xfer, 0, sizeof(xfer));

	if ((n_tx == 1) && (n_rx == 1))
		cmd = SPI_READ;
	else
		cmd = *((u8 *)txbuf);

	switch(cmd) {
		case SPI_WRITE:
			if(n_tx % DMA_TRANSFER_MAX_SIZE)
				xferloop = (n_tx / DMA_TRANSFER_MAX_SIZE) + 1;
			else
				xferloop = n_tx / DMA_TRANSFER_MAX_SIZE ;

			xferlen = n_tx;
			memcpy(dma_txbuf, (u8 *)txbuf, xferlen);

			for(xfercnt = 0; xfercnt < xferloop; xfercnt++) {
				if(xferlen > DMA_TRANSFER_MAX_SIZE)
					xferlen = DMA_TRANSFER_MAX_SIZE;

				xfer[xfercnt].len = xferlen;
				xfer[xfercnt].tx_buf = dma_txbuf + xfercnt * DMA_TRANSFER_MAX_SIZE;
				spi_message_add_tail(&xfer[xfercnt], &message);

				xferlen = n_tx - (xfercnt+1) * DMA_TRANSFER_MAX_SIZE;
			}

			status = spi_sync(spi, &message);
			break;
		case SPI_READ:
			/* for write cmd and head */
			memcpy(dma_txbuf, (u8 *)txbuf, n_tx);
			xfer[0].len = n_tx;
			xfer[0].tx_buf = dma_txbuf;
			xfer[0].rx_buf = temp1;
			spi_message_add_tail(&xfer[0], &message);

			/* for read data */
			if (n_rx % DMA_TRANSFER_MAX_SIZE)
				xferloop = (n_rx / DMA_TRANSFER_MAX_SIZE) + 1;
			else
				xferloop = n_rx / DMA_TRANSFER_MAX_SIZE ;

			xferlen = n_rx;
			for(xfercnt = 0; xfercnt < xferloop; xfercnt++) {
				if (xferlen > DMA_TRANSFER_MAX_SIZE)
					xferlen = DMA_TRANSFER_MAX_SIZE;

				xfer[xfercnt+1].len = xferlen;
				xfer[xfercnt+1].tx_buf = temp2;
				xfer[xfercnt+1].rx_buf = dma_rxbuf + xfercnt * DMA_TRANSFER_MAX_SIZE;
				spi_message_add_tail(&xfer[xfercnt+1], &message);

				xferlen = n_rx - (xfercnt+1) * DMA_TRANSFER_MAX_SIZE;
			}
			status = spi_sync(spi, &message);
			if(status == 0)
				memcpy((u8 *)rxbuf, dma_rxbuf, n_rx);
			break;
		default:
			ipio_info("Unknown command 0x%x\n", cmd);
			break;
	}

	mutex_unlock(&lock);

out:
	return status;
}
#endif /* CONFIG_MTK_SPI */

static int core_rx_lock_check(struct ilitek_tddi_dev *idev, int *ret_size)
{
	int i, count = 10;
	u8 txbuf[5] = {SPI_WRITE, 0x25, 0x94, 0x0, 0x2};
	u8 rxbuf[4] = {0};
	u16 status = 0, lock = 0x5AA5;

	for (i = 0; i < count; i++) {
		txbuf[0] = SPI_WRITE;
		if (ilitek_spi_write_then_read(idev->spi, txbuf, 5, txbuf, 0) < 0) {
			ipio_err("spi Write Error\n");
			goto out;
		}

		txbuf[0] = SPI_READ;
		if (ilitek_spi_write_then_read(idev->spi, txbuf, 1, rxbuf, 4) < 0) {
			ipio_err("spi Read Error\n");
			goto out;
		}

		status = (rxbuf[2] << 8) + rxbuf[3];
		*ret_size = (rxbuf[0] << 8) + rxbuf[1];

		//ipio_debug(DEBUG_SPI, "Rx lock = 0x%x, size = %d\n", status, *ret_size);

		if (CHECK_EQUAL(status, lock) == 0) {
			ipio_info("Rx check lock free!!\n");
			return 0;
		}

		mdelay(1);
	}

out:
	ipio_err("Rx check lock error, lock = 0x%x, size = %d\n", status, *ret_size);
	return -EIO;
}

static int core_tx_unlock_check(struct ilitek_tddi_dev *idev)
{
	int i, count = 10;
	u8 txbuf[5] = {SPI_WRITE, 0x25, 0x0, 0x0, 0x2};
	u8 rxbuf[4] = {0};
	u16 status = 0, unlock = 0x9881;

	for (i = 0; i < count; i++) {
		txbuf[0] = SPI_WRITE;
		if (ilitek_spi_write_then_read(idev->spi, txbuf, 5, txbuf, 0) < 0) {
			ipio_err("spi Write Error\n");
			goto out;
		}

		txbuf[0] = SPI_READ;
		if (ilitek_spi_write_then_read(idev->spi, txbuf, 1, rxbuf, 4) < 0) {
			ipio_err("spi Read Error\n");
			goto out;
		}

		status = (rxbuf[2] << 8) + rxbuf[3];

		//ipio_debug(DEBUG_SPI, "Tx unlock = 0x%x\n", status);

		if (CHECK_EQUAL(status, unlock)) {
			ipio_info("Tx check unlock free!\n");
			return 0;
		}

		mdelay(1);
	}

out:
	ipio_err("Tx check unlock error, unlock = 0x%x\n", status);
	return -EIO;
}

static int core_spi_ice_mode_unlock_read(struct ilitek_tddi_dev *idev, u8 *data, size_t size)
{
	int ret = 0;
	uint8_t txbuf[64] = { 0 };

	/* set read address */
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x98;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;
	if (ilitek_spi_write_then_read(idev->spi, txbuf, 5, txbuf, 0) < 0) {
		ret = -EIO;
		return ret;
	}

	/* read data */
	txbuf[0] = SPI_READ;
	if (ilitek_spi_write_then_read(idev->spi, txbuf, 1, data, size) < 0) {
		ret = -EIO;
		return ret;
	}

	/* write data unlock */
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x94;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;
	txbuf[5] = (size & 0xFF00) >> 8;
	txbuf[6] = size & 0xFF;
	txbuf[7] = (char)0x98;
	txbuf[8] = (char)0x81;
	if (ilitek_spi_write_then_read(idev->spi, txbuf, 9, txbuf, 0) < 0) {
		ret = -EIO;
		ipio_err("spi Write data unlock error, ret = %d\n", ret);
	}

	return ret;
}

static int core_spi_ice_mode_lock_write(struct ilitek_tddi_dev *idev, u8 *data, size_t size)
{
	int ret = 0;
	int safe_size = size;
	uint8_t check_sum = 0,wsize = 0;
	uint8_t *txbuf = NULL;

    txbuf = kcalloc(size + 9, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(txbuf)) {
		ipio_err("Failed to allocate txbuf\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Write data */
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x4;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;

	/* Calcuate checsum and fill it in the last byte */
	check_sum = ilitek_calc_packet_checksum(data, size);
	ipio_memcpy(txbuf + 5, data, size, safe_size + 9);
	txbuf[5 + size] = check_sum;
	size++;
	wsize = size;
	if (wsize % 4 != 0)
		wsize += 4 - (wsize % 4);

	if (ilitek_spi_write_then_read(idev->spi, txbuf, wsize + 5, txbuf, 0) < 0) {
		ret = -EIO;
		ipio_err("spi Write Error, ret = %d\n", ret);
		goto out;
	}

	/* write data lock */
	txbuf[0] = SPI_WRITE;
	txbuf[1] = 0x25;
	txbuf[2] = 0x0;
	txbuf[3] = 0x0;
	txbuf[4] = 0x2;
	txbuf[5] = (size & 0xFF00) >> 8;
	txbuf[6] = size & 0xFF;
	txbuf[7] = (char)0x5A;
	txbuf[8] = (char)0xA5;
	if (ilitek_spi_write_then_read(idev->spi, txbuf, 9, txbuf, 0) < 0) {
		ret = -EIO;
		ipio_err("spi Write data lock Error, ret = %d\n", ret);
	}

out:
	ipio_kfree((void **)&txbuf);
	return ret;
}

static int core_spi_ice_mode_disable(struct ilitek_tddi_dev *idev)
{
	u8 txbuf[5] = {0x82, 0x1B, 0x62, 0x10, 0x18};

	if (ilitek_spi_write_then_read(idev->spi, txbuf, 5, txbuf, 0) < 0) {
		ipio_err("spi R/W Error\n");
		return -EIO;
	}

	return 0;
}

static int core_spi_ice_mode_enable(struct ilitek_tddi_dev *idev)
{
	u8 txbuf[5] = {0x82, 0x1F, 0x62, 0x10, 0x18};
	u8 rxbuf[2]= {0};

	if (ilitek_spi_write_then_read(idev->spi, txbuf, 1, rxbuf, 1) < 0) {
		ipio_err("spi R/W Error\n");
		return -EIO;
	}

	/* check recover data */
	if(rxbuf[0] != 0xA3){
		ipio_err("Check Recovery data failed (0x%x)\n", rxbuf[0]);
		return CHECK_RECOVER;
	}

	if (ilitek_spi_write_then_read(idev->spi, txbuf, 5, rxbuf, 0) < 0) {
		ipio_err("spi R/W Error\n");
		return -EIO;
	}

	return 0;
}

static int core_spi_ice_mode_write(struct ilitek_tddi_dev *idev, u8 *data, size_t len)
{
	int ret = 0;

	ret = core_spi_ice_mode_enable(idev);
	if (ret < 0) {
		ipio_err("spi ice mode enable failed\n");
		return ret;
	}

	ret = core_spi_ice_mode_lock_write(idev, data, len);
	if (ret < 0) {
		ipio_err("spi ice mode lock write failed\n");
		goto out;
	}

	ret = core_tx_unlock_check(idev);
	if (ret < 0) {
		ipio_err("tx unlock check error\n");
		goto out;
	}

out:
	if (core_spi_ice_mode_disable(idev) < 0) {
		ret = -EIO;
		ipio_err("spi ice mode disable failed\n");
	}

	return ret;
}

static int core_spi_ice_mode_read(struct ilitek_tddi_dev *idev, u8 *data)
{
	int ret = 0, size = 0;

	ret = core_spi_ice_mode_enable(idev);
	if (ret < 0) {
		ipio_err("spi ice mode enable failed\n");
		return ret;
	}

	ret = core_rx_lock_check(idev, &size);
	if (ret < 0) {
		ipio_err("Rx lock check error\n");
		goto out;
	}

	ret = core_spi_ice_mode_unlock_read(idev, data, size);
	if ( ret < 0) {
		ipio_err("spi ice mode unlock write failed\n");
		goto out;
	}

out:
	if (core_spi_ice_mode_disable(idev) < 0) {
		ret = -EIO;
		ipio_err("spi ice mode disable failed\n");
	}

	return ret;
}

static int core_spi_write(struct ilitek_tddi_dev *idev, u8 *data, size_t len)
{
	int ret = 0, count = 5;
	u8 *txbuf = NULL;
	size_t safe_size = len;

	if (atomic_read(&idev->ice_stat) == ICE_DISABLE) {
		do {
			ret = core_spi_ice_mode_write(idev, data, len);
			if (ret >= 0)
				break;

			ipio_err("spi ice mode write failed, retry = %d\n", count);
		} while(--count > 0);
		goto out;
	}

    txbuf = kcalloc(len + 1, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(txbuf)) {
		ipio_err("Failed to allocate txbuf\n");
		return -ENOMEM;
	}

	txbuf[0] = SPI_WRITE;
	ipio_memcpy(txbuf+1, data, len, safe_size + 1);

	if (ilitek_spi_write_then_read(idev->spi, txbuf, len+1, txbuf, 0) < 0) {
		ret = -EIO;
		goto out;
	}

out:
	ipio_kfree((void **)&txbuf);
	return ret;
}

static int core_spi_read(struct ilitek_tddi_dev *idev, u8 *rxbuf, size_t len)
{
	int ret = 0, count = 5;
	u8 txbuf[1] = {0};

	txbuf[0] = SPI_READ;

	if (atomic_read(&idev->ice_stat) == ICE_DISABLE) {
		do {
			ret = core_spi_ice_mode_read(idev, rxbuf);
			if (ret >= 0)
				break;

			ipio_err("spi ice mode write failed, retry = %d\n", count);
		} while(--count > 0);
		goto out;
	}

	if(ilitek_spi_write_then_read(idev->spi, txbuf, 1, rxbuf, len) < 0) {
		ret = -EIO;
		goto out;
	}

out:
	return ret;
}

static int core_spi_setup(struct ilitek_tddi_dev *idev, u32 freq)
{
#ifdef CONFIG_MTK_SPI
	struct mt_chip_conf *chip_config;
	u32 temp_pulse_width = 0;

	chip_config = (struct mt_chip_conf *)idev->spi->controller_data;
	if (!chip_config) {
		ipio_err("chip_config is NULL.\n");
		chip_config = kzalloc(sizeof(struct mt_chip_conf), GFP_KERNEL);
		if (!chip_config)
			return -ENOMEM;
	}

	temp_pulse_width = ((112 * 1000000) / freq);
	temp_pulse_width = temp_pulse_width / 2;

	chip_config->setuptime = temp_pulse_width * 2;// for CS
	chip_config->holdtime = temp_pulse_width * 2;// for CS
	chip_config->high_time = temp_pulse_width;// for CLK = 1M
	chip_config->low_time = temp_pulse_width;// for CLK= 1M
	chip_config->cs_idletime = temp_pulse_width * 2;// for CS
	chip_config->rx_mlsb = 1;
	chip_config->tx_mlsb = 1;
	chip_config->tx_endian = 0;
	chip_config->rx_endian = 0;
	chip_config->cpol = 0;
	chip_config->cpha = 0;
	chip_config->com_mod = DMA_TRANSFER;
	//chip_config->com_mod = FIFO_TRANSFER;
	chip_config->pause = 1;
	chip_config->finish_intr = 1;
	chip_config->deassert = 0;

	idev->spi->controller_data = chip_config;
#endif /* CONFIG_MTK_SPI */

	ipio_info("spi clock = %d\n", freq);

	idev->spi->mode = SPI_MODE_0;
	idev->spi->bits_per_word = 8;
	idev->spi->max_speed_hz = freq;

	if (spi_setup(idev->spi) < 0) {
		ipio_err("Failed to setup spi device\n");
		return -ENODEV;
	}

	return 0;
}

static int ilitek_spi_write(struct ilitek_tddi_dev *idev, void *buf, size_t len)
{
    int ret = 0;

    mutex_lock(&idev->io_mutex);

    ret = core_spi_write(idev, buf, len);
    if (ret < 0) {
		if (atomic_read(&idev->tp_reset) == TP_RST_START) {
			ret = 0;
			goto out;
		}
	}

	ipio_err("spi write Error, ret = %d\n", ret);

out:
    mutex_unlock(&idev->io_mutex);
    return ret;
}

static int ilitek_spi_read(struct ilitek_tddi_dev *idev, void *buf, size_t len)
{
    int ret = 0;

    mutex_lock(&idev->io_mutex);

    ret = core_spi_read(idev, buf, len);
    if (ret < 0) {
		if (atomic_read(&idev->tp_reset) == TP_RST_START) {
			ret = 0;
			goto out;
		}
	}

	ipio_err("spi read Error, ret = %d\n", ret);

out:
    mutex_unlock(&idev->io_mutex);
    return ret;
}

static int ilitek_spi_probe(struct spi_device *spi)
{
    ipio_info();

	if (spi == NULL) {
		ipio_err("spi device is NULL\n");
		return -ENODEV;
	}

	idev = devm_kzalloc(&spi->dev, sizeof(struct ilitek_tddi_dev), GFP_KERNEL);
	if (ERR_ALLOC_MEM(idev)) {
		ipio_err("Failed to allocate idev memory, %ld\n", PTR_ERR(idev));
		return -ENOMEM;
	}

	idev->spi = spi;
	idev->dev = &spi->dev;

    idev->write = ilitek_spi_write;
    idev->read = ilitek_spi_read;

#ifdef CONFIG_MTK_SPI
    ilitek_spi_write_then_read = core_mtk_spi_write_then_read;
#else
    ilitek_spi_write_then_read = spi_write_then_read;
#endif

    idev->reset_mode = TP_RST_HOST_DOWNLOAD;
    idev->fw_upgrade_mode = UPGRADE_IRAM;
	idev->mp_move_code = ilitek_tddi_mp_move_code_iram;
    return info->hwif->plat_probe(idev);
}

static int ilitek_spi_remove(struct spi_device *spi)
{
	struct touch_bus_info *info =
		container_of(to_i2c_driver(client->dev.driver),
			struct touch_bus_info, bus_driver);

    ipio_info();

    return info->hwif->plat_remove(idev);
}

static struct spi_device_id tp_spi_id[] = {
	{DEVICE_ID, 0},
};

int ilitek_spi_dev_init(struct ilitek_hwif_info *hwif)
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

	info->bus_driver.probe = ilitek_spi_probe;
	info->bus_driver.remove = ilitek_spi_remove;
	info->bus_driver.id_table = tp_spi_id;

    info->hwif = hwif;

    return spi_register_driver(&info->bus_driver);
}
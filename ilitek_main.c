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

/* Debug level */
u32 ipio_debug_level = DEBUG_ALL;
EXPORT_SYMBOL(ipio_debug_level);

int ilitek_tddi_mp_test_handler(struct ilitek_tddi_dev *idev, bool lcm_on)
{
	int ret = 0;

    ipio_info();

	if (atomic_read(&idev->fw_stat) == FW_RUNNING)
		return -1;

	mutex_lock(&idev->touch_mutex);
	atomic_set(&idev->mp_stat, ENABLE);

	ret = ilitek_tddi_mp_test_run(idev);

	mutex_unlock(&idev->touch_mutex);
	atomic_set(&idev->mp_stat, DISABLE);
    return ret;
}

int ilitek_tddi_esd_handler(struct ilitek_tddi_dev *idev)
{
	ipio_info();
	return 0;
}

int ilitek_tddi_fw_upgrade_handler(void *data)
{
	int ret = 0, fw_file = 0;
	struct ilitek_tddi_dev *idev = data;

	if (atomic_read(&idev->tp_suspend)) {
		ipio_info("TP is suspending, upgrade failed\n");
		return -1;
	}

	mutex_lock(&idev->touch_mutex);
	atomic_set(&idev->fw_stat, FW_RUNNING);

	if (idev->fw_boot)
		fw_file = ILI_FILE;
	else
		fw_file = HEX_FILE;

	ret = ilitek_tddi_fw_upgrade(idev, idev->fw_upgrade_mode, fw_file, idev->fw_open);

	mutex_unlock(&idev->touch_mutex);
	atomic_set(&idev->fw_stat, FW_IDLE);
	return ret;
}

void ilitek_tddi_report_handler(struct ilitek_tddi_dev *idev)
{
	int ret = 0, pid = 0;
	u8 *buf = NULL, checksum = 0;
	size_t rlen = 0;

	if (atomic_read(&idev->tp_reset) || atomic_read(&idev->fw_stat))
		return;

	switch (idev->actual_fw_mode) {
		case P5_X_FW_DEMO_MODE:
			rlen = P5_X_DEMO_MODE_PACKET_LENGTH;
			break;
		case P5_X_FW_DEBUG_MODE:
			break;
		case P5_X_FW_I2CUART_MODE:
			break;
		case P5_X_FW_GESTURE_MODE:
			break;
		default:
			ipio_err("Unknown fw mode, %d\n", idev->actual_fw_mode);
			rlen = 0;
			break;
	}

	if (!rlen) {
		ipio_err("Length of packet is invaild\n");
		goto out;
	}

	buf = kcalloc(rlen, sizeof(u8), GFP_ATOMIC);
	if (ERR_ALLOC_MEM(buf)) {
		ipio_err("Failed to allocate packet memory, %ld\n", PTR_ERR(buf));
		return;
	}

	ret = idev->read(idev, buf, rlen);
	if (ret < 0) {
		ipio_err("Read report packet buf failed\n");
		goto out;
	}

	ilitek_dump_data(buf, 8, rlen, 0, "finger report");

	checksum = ilitek_calc_packet_checksum(buf, rlen - 1);
	if (!CHECK_EQUAL(checksum, buf[rlen-1])) {
		ipio_err("Wrong checksum, checksum = %x, buf = %x\n", checksum, buf[rlen-1]);
		goto out;
	}

	pid = buf[0];
	ipio_info("Packet ID = %x\n", pid);

	switch (pid) {
		case P5_X_DEMO_PACKET_ID:
			ilitek_tddi_report_ap_mode(idev, buf);
			break;
		case P5_X_DEBUG_PACKET_ID:
			ilitek_tddi_report_debug_mode(idev);
			break;
		case P5_X_I2CUART_PACKET_ID:
			break;
		case P5_X_GESTURE_PACKET_ID:
			ilitek_tddi_report_gesture_mode(idev);
			break;
		default:
			ipio_err("Unknown packet id, %x\n", pid);
			break;
	}

out:
	ipio_kfree((void **)&buf);
}

int ilitek_tddi_reset_ctrl(struct ilitek_tddi_dev *idev, int mode)
{
	int ret = 0;

	atomic_set(&idev->tp_reset, TP_RST_START);

	switch (mode) {
		case TP_IC_CODE_RST:
			ipio_info("Doing TP IC Code RST \n");
			ret = ilitek_tddi_ic_code_reset(idev);
			break;
		case TP_IC_WHOLE_RST:
			ipio_info("Doing TP IC whole RST\n");
			ret = ilitek_tddi_ic_whole_reset(idev);
			break;
		case TP_RST_HW_ONLY:
			ipio_info("Doing TP RST only\n");
			ilitek_plat_tp_reset(idev);
			break;
		case TP_RST_HOST_DOWNLOAD:
			ipio_info("Doing TP RST with host download\n");
			ilitek_plat_tp_reset(idev);
			break;
		default:
			ipio_err("Unknown reset mode, %d\n", mode);
			ret = -EINVAL;
			break;
	}

	atomic_set(&idev->tp_reset, TP_RST_END);
	return ret;
}

int ilitek_tddi_init(struct ilitek_tddi_dev *idev)
{
	ipio_info();

	mutex_init(&idev->io_mutex);
	mutex_init(&idev->touch_mutex);
	spin_lock_init(&idev->irq_spin);

	atomic_set(&idev->irq_stat, IRQ_DISABLE);
	atomic_set(&idev->ice_stat, ICE_DISABLE);
	atomic_set(&idev->tp_reset, TP_RST_END);
	atomic_set(&idev->fw_stat, FW_IDLE);
	atomic_set(&idev->mp_stat, DISABLE);
	atomic_set(&idev->tp_suspend, DONE);
	atomic_set(&idev->tp_resume, DONE);

	idev->actual_fw_mode = P5_X_FW_DEMO_MODE;
    idev->suspend = ilitek_tddi_touch_suspend;
    idev->resume = ilitek_tddi_touch_resume;
	idev->fw_boot = DISABLE;
	idev->fw_open = FILP_OPEN;

	if (ilitek_tddi_ic_init(idev) < 0) {
		ipio_err("Init tddi ic info failed\n");
		return -ENOMEM;
	}

	ilitek_tddi_reset_ctrl(idev, idev->reset_mode);
	ilitek_tddi_ic_get_info(idev);
	ilitek_tddi_fw_read_flash_info(idev, idev->fw_upgrade_mode);

	if (idev->fw_boot == ENABLE) {
		idev->fw_boot_th = kthread_run(ilitek_tddi_fw_upgrade_handler, (void *)idev, "ili_fw_boot");
		if (idev->fw_boot_th == (struct task_struct *)ERR_PTR) {
			idev->fw_boot_th = NULL;
			ipio_err("Failed to create fw upgrade thread\n");
		}
	}

	ilitek_tddi_ic_get_protocl_ver(idev);
	ilitek_tddi_ic_get_fw_ver(idev);
	ilitek_tddi_ic_get_tp_info(idev);
	ilitek_tddi_ic_get_panel_info(idev);
	ilitek_plat_input_register(idev);
	ilitek_tddi_node_init(idev);
	return 0;
}

int ilitek_tddi_dev_init(struct ilitek_hwif_info *hwif)
{
	ipio_info();

	if (hwif->bus_type == TP_BUS_I2C)
		return ilitek_i2c_dev_init(hwif);
	// else
	// 	return ilitek_spi_dev_init(hwif);

	ipio_err("Unknown touch interface, %d\n", hwif->bus_type);
	return -ENODEV;
}
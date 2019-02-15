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

static struct workqueue_struct *esd_wq;
static struct workqueue_struct *bat_wq;
static struct delayed_work esd_work;
static struct delayed_work bat_work;

int ilitek_tddi_mp_test_handler(char *apk, bool lcm_on)
{
	int ret = 0;
	u8 tp_mode = P5_X_FW_TEST_MODE;

    ipio_info();

	if (atomic_read(&idev->fw_stat) == START)
		return 0;

	ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, DISABLE);

	mutex_lock(&idev->touch_mutex);
	atomic_set(&idev->mp_stat, ENABLE);

	if (ilitek_tddi_touch_switch_mode(&tp_mode) < 0)
		goto out;

	ret = ilitek_tddi_mp_test_main(apk, lcm_on);

out:
	tp_mode = P5_X_FW_DEMO_MODE;
	ilitek_tddi_touch_switch_mode(&tp_mode);
	mutex_unlock(&idev->touch_mutex);
	atomic_set(&idev->mp_stat, DISABLE);
	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
    return ret;
}

void ilitek_tddi_wq_esd_spi_check(void)
{
	ipio_info();
}

void ilitek_tddi_wq_esd_i2c_check(void)
{
	ipio_info();
}

static int read_power_status(u8 *buf)
{
	struct file *f = NULL;
	mm_segment_t old_fs;
	ssize_t byte = 0;

	old_fs = get_fs();
	set_fs(get_ds());

	f = filp_open(POWER_STATUS_PATH, O_RDONLY, 0);
	if (ERR_ALLOC_MEM(f)) {
		ipio_err("Failed to open %s\n", POWER_STATUS_PATH);
		return -1;
	}

	f->f_op->llseek(f, 0, SEEK_SET);
	byte = f->f_op->read(f, buf, 20, &f->f_pos);

	ipio_debug(DEBUG_BATTERY, "Read %d bytes\n", (int)byte);

	set_fs(old_fs);
	filp_close(f, NULL);
	return 0;
}

static void ilitek_tddi_wq_esd_check(struct work_struct *work)
{
	ipio_info();
	idev->esd_callabck();
	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
}

static void ilitek_tddi_wq_bat_check(struct work_struct *work)
{
	u8 str[20] = {0};
	static int charge_mode = 0;

	if (read_power_status(str) < 0)
		return;

	ipio_debug(DEBUG_BATTERY, "Batter Status: %s\n", str);

	if (strstr(str, "Charging") != NULL || strstr(str, "Full") != NULL
	    || strstr(str, "Fully charged") != NULL) {
		if (charge_mode != 1) {
			ipio_debug(DEBUG_BATTERY, "Charging mode\n");
			ilitek_tddi_ic_func_ctrl("plug", DISABLE);// plug in
			charge_mode = 1;
		}
	} else {
		if (charge_mode != 2) {
			ipio_debug(DEBUG_BATTERY, "Not charging mode\n");
			ilitek_tddi_ic_func_ctrl("plug", ENABLE);// plug out
			charge_mode = 2;
		}
	}
	ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
}

void ilitek_tddi_wq_ctrl(int type, int ctrl)
{
	mutex_lock(&idev->wq_mutex);

	switch (type) {
		case WQ_ESD:
#ifdef ENABLE_WQ_ESD
			if (!esd_wq)
				break;
			idev->wq_esd_ctrl = ctrl;
			if (ctrl == ENABLE) {
				ipio_info("Execute WQ ESD\n");
				if (!queue_delayed_work(esd_wq, &esd_work, msecs_to_jiffies(2000)))
					ipio_info("execute WQ ESD error\n");
			} else {
				cancel_delayed_work_sync(&esd_work);
				flush_workqueue(esd_wq);
				ipio_info("Cancel WQ ESD\n");
			}
#endif
			break;
		case WQ_BAT:
#ifdef ENABLE_WQ_BAT
			if (!bat_wq)
				break;
			idev->wq_bat_ctrl = ctrl;
			if (ctrl == ENABLE) {
				ipio_info("Execute WQ BAT\n");
				if (!queue_delayed_work(bat_wq, &bat_work, msecs_to_jiffies(4000)))
					ipio_info("execute WQ BAT error\n");
			} else {
				cancel_delayed_work_sync(&bat_work);
				flush_workqueue(bat_wq);
				ipio_info("Cancel WQ BAT\n");
			}
#endif
			break;
		case WQ_SUSPEND:
			ipio_err("Not implement yet\n");
			break;
		default:
			ipio_err("Unknown WQ type, %d\n", type);
			break;
	}
	mutex_unlock(&idev->wq_mutex);
}

static void ilitek_tddi_wq_init(void)
{
	ipio_info();

	esd_wq = alloc_workqueue("esd_check", WQ_MEM_RECLAIM, 0);
	bat_wq = alloc_workqueue("bat_check", WQ_MEM_RECLAIM, 0);

	WARN_ON(!esd_wq);
	WARN_ON(!bat_wq);

	INIT_DELAYED_WORK(&esd_work, ilitek_tddi_wq_esd_check);
	INIT_DELAYED_WORK(&bat_work, ilitek_tddi_wq_bat_check);

#ifdef ENABLE_WQ_ESD
	idev->wq_esd_ctrl = ENABLE;
	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
#endif
#ifdef ENABLE_WQ_BAT
	idev->wq_bat_ctrl = ENABLE;
	ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
#endif
}

int ilitek_tddi_sleep_handler(int mode)
{
	u8 tp_mode = P5_X_FW_DEMO_MODE;
	int ret = 0;

	ipio_info("Sleep Mode = %d\n", mode);

	if (atomic_read(&idev->fw_stat) == START ||
		atomic_read(&idev->mp_stat) == ENABLE)
		return 0;

	ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, DISABLE);

	atomic_set(&idev->tp_sleep, START);
	mutex_lock(&idev->sleep_mutex);

	ilitek_plat_irq_disable();

	switch (mode) {
		case TP_SUSPEND:
			ipio_info("TP normal suspend start\n");
			ilitek_tddi_ic_func_ctrl("sense", DISABLE);
			ilitek_tddi_ic_check_busy(50, 50);

			if (idev->gesture) {
				idev->gesture_move_code(idev->gesture_mode);
				enable_irq_wake(idev->irq_num);
				msleep(20); /* wait for isr queue completed. */
				ilitek_plat_irq_enable();
			} else {
				ilitek_tddi_ic_func_ctrl("sleep", SLEEP_IN);
			}
			ipio_info("TP normal suspend end\n");
			break;
		case TP_DEEP_SLEEP:
			ipio_info("TP deep suspend start\n");
			ilitek_tddi_ic_func_ctrl("sense", DISABLE);
			ilitek_tddi_ic_check_busy(50, 50);
			ilitek_tddi_ic_func_ctrl("sleep", DEEP_SLEEP_IN);
			ipio_info("TP deep suspend end\n");
			break;
		case TP_RESUME:
			ipio_info("TP resume start\n");
			if (idev->gesture)
				disable_irq_wake(idev->irq_num);

			ilitek_tddi_touch_switch_mode(&tp_mode);

			ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
			ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
			ilitek_plat_irq_enable();
			ipio_info("TP resume end\n");
			break;
		default:
			ipio_err("Unknown sleep mode, %d\n", mode);
			ret = -EINVAL;
			break;
	}

	ilitek_tddi_touch_release_all_point();
	mutex_unlock(&idev->sleep_mutex);
	atomic_set(&idev->tp_sleep, END);
	return ret;
}

int ilitek_tddi_fw_upgrade_handler(void *data)
{
	int ret = 0;

	if (atomic_read(&idev->tp_sleep)) {
		ipio_info("TP is suspending, upgrade failed\n");
		return -1;
	}

	ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, DISABLE);

	mutex_lock(&idev->touch_mutex);
	atomic_set(&idev->fw_stat, START);

	idev->fw_update_stat = 0;
	ret = ilitek_tddi_fw_upgrade(idev->fw_upgrade_mode, HEX_FILE, idev->fw_open);
	if (ret == 0)
		idev->fw_update_stat = 100;
	else
		idev->fw_update_stat = -1;

	mutex_unlock(&idev->touch_mutex);
	atomic_set(&idev->fw_stat, END);

	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
	return ret;
}

void ilitek_tddi_report_handler(void)
{
	int ret = 0, pid = 0;
	u8 *buf = NULL, checksum = 0;
	size_t rlen = 0;
	u16 self_key = 2;

	if (atomic_read(&idev->tp_sleep) == START)
		return;

	ilitek_tddi_wq_ctrl(WQ_ESD, DISABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, DISABLE);

	switch (idev->actual_fw_mode) {
		case P5_X_FW_DEMO_MODE:
			rlen = P5_X_DEMO_MODE_PACKET_LENGTH;
			break;
		case P5_X_FW_DEBUG_MODE:
			rlen = (2 * idev->xch_num * idev->ych_num) + (idev->stx * 2) + (idev->srx * 2);
			rlen += 2 * self_key + (8 * 2) + 1 + 35;
			break;
		case P5_X_FW_I2CUART_MODE:
			rlen = P5_X_DEMO_MODE_PACKET_LENGTH;
			break;
		case P5_X_FW_GESTURE_MODE:
			if (idev->gesture_mode == P5_X_FW_GESTURE_INFO_MODE)
				rlen = P5_X_GESTURE_INFO_LENGTH;
			else
				rlen = P5_X_GESTURE_NORMAL_LENGTH;
			break;
		default:
			ipio_err("Unknown fw mode, %d\n", idev->actual_fw_mode);
			rlen = 0;
			break;
	}

	ipio_info("Packget length = %ld\n", rlen);

	if (!rlen) {
		ipio_err("Length of packet is invaild\n");
		goto out;
	}

	buf = kcalloc(rlen, sizeof(u8), GFP_ATOMIC);
	if (ERR_ALLOC_MEM(buf)) {
		ipio_err("Failed to allocate packet memory, %ld\n", PTR_ERR(buf));
		return;
	}

	ret = idev->read(buf, rlen);
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
			ilitek_tddi_report_ap_mode(buf, rlen);
			break;
		case P5_X_DEBUG_PACKET_ID:
			ilitek_tddi_report_debug_mode(buf, rlen);
			break;
		case P5_X_I2CUART_PACKET_ID:
			ilitek_tddi_report_i2cuart_mode(buf, rlen);
			break;
		case P5_X_GESTURE_PACKET_ID:
			ilitek_tddi_report_gesture_mode(buf, rlen);
			break;
		default:
			ipio_err("Unknown packet id, %x\n", pid);
			break;
	}

out:
	ilitek_tddi_wq_ctrl(WQ_ESD, ENABLE);
	ilitek_tddi_wq_ctrl(WQ_BAT, ENABLE);
	ipio_kfree((void **)&buf);
}

int ilitek_tddi_reset_ctrl(int mode)
{
	int ret = 0;

	atomic_set(&idev->tp_reset, START);

	switch (mode) {
		case TP_IC_CODE_RST:
			ipio_info("Doing TP IC Code RST \n");
			ret = ilitek_tddi_ic_code_reset();
			break;
		case TP_IC_WHOLE_RST:
			ipio_info("Doing TP IC whole RST\n");
			ret = ilitek_tddi_ic_whole_reset();
			break;
		case TP_RST_HW_ONLY:
			ipio_info("Doing TP RST only\n");
			ilitek_plat_tp_reset();
			break;
		case TP_RST_HOST_DOWNLOAD:
			ipio_info("Doing TP RST with host download\n");
			ilitek_plat_tp_reset();
			break;
		default:
			ipio_err("Unknown reset mode, %d\n", mode);
			ret = -EINVAL;
			break;
	}

	atomic_set(&idev->tp_reset, END);
	atomic_set(&idev->ice_stat, DISABLE);
	idev->actual_fw_mode = P5_X_FW_DEMO_MODE;
	return ret;
}

#define TP_RST_BIND 1
int ilitek_tddi_init(void)
{
	struct task_struct *fw_boot_th;

	ipio_info();

	mutex_init(&idev->io_mutex);
	mutex_init(&idev->touch_mutex);
	mutex_init(&idev->wq_mutex);
	mutex_init(&idev->sleep_mutex);
	mutex_init(&idev->debug_mutex);
	mutex_init(&idev->debug_read_mutex);
	init_waitqueue_head(&(idev->inq));
	spin_lock_init(&idev->irq_spin);

	atomic_set(&idev->irq_stat, DISABLE);
	atomic_set(&idev->ice_stat, DISABLE);
	atomic_set(&idev->tp_reset, END);
	atomic_set(&idev->fw_stat, END);
	atomic_set(&idev->mp_stat, DISABLE);
	atomic_set(&idev->tp_sleep, END);
	atomic_set(&idev->mp_int_check, DISABLE);

	ilitek_tddi_ic_init();
	ilitek_tddi_wq_init();

	/* Must do hw reset once in first time for work normally */
	if (TP_RST_BIND)
		ilitek_tddi_reset_ctrl(TP_IC_WHOLE_RST);
	else
		ilitek_tddi_reset_ctrl(TP_RST_HW_ONLY);

	if (ilitek_tddi_ic_get_info() < 0) {
		ipio_err("Not found ilitek chipes\n");
		return -ENODEV;
	}

	ilitek_tddi_fw_read_flash_info(idev->fw_upgrade_mode);

	fw_boot_th = kthread_run(ilitek_tddi_fw_upgrade_handler, NULL, "ili_fw_boot");
	if (fw_boot_th == (struct task_struct *)ERR_PTR) {
		fw_boot_th = NULL;
		WARN_ON(!fw_boot_th);
		ipio_err("Failed to create fw upgrade thread\n");
	}

	ilitek_tddi_node_init();
	return 0;
}

void ilitek_tddi_dev_remove(void)
{
	ipio_info();

	if (!idev)
		return;

	gpio_free(idev->tp_int);
	gpio_free(idev->tp_rst);
	if (esd_wq) {
		cancel_delayed_work_sync(&esd_work);
		flush_workqueue(esd_wq);
		destroy_workqueue(esd_wq);
	}
	if (bat_wq) {
		cancel_delayed_work_sync(&bat_work);
		flush_workqueue(bat_wq);
		destroy_workqueue(bat_wq);
	}
}

int ilitek_tddi_dev_init(struct ilitek_hwif_info *hwif)
{
	ipio_info("TP interface = %d\n", hwif->bus_type);
	return ilitek_tddi_interface_dev_init(hwif);
}
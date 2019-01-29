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

void ilitek_dump_data(void *data, int type, int len, int row_len, const char *name)
{
	int i, row = 31;
	u8 *p8 = NULL;
	s32 *p32 = NULL;

	if (row_len > 0)
		row = row_len;

	if (ipio_debug_level & DEBUG_MP_TEST) {
		if (data == NULL) {
			ipio_err("The data going to dump is NULL\n");
			return;
		}

		printk(KERN_CONT "\n\n");
		printk(KERN_CONT "ILITEK: Dump %s data\n", name);
		printk(KERN_CONT "ILITEK: ");

		if (type == 8)
			p8 = (u8 *) data;
		if (type == 32 || type == 10)
			p32 = (s32 *) data;

		for (i = 0; i < len; i++) {
			if (type == 8)
				printk(KERN_CONT " %4x ", p8[i]);
			else if (type == 32)
				printk(KERN_CONT " %4x ", p32[i]);
			else if (type == 10)
				printk(KERN_CONT " %4d ", p32[i]);
			if ((i % row) == row - 1) {
				printk(KERN_CONT "\n");
				printk(KERN_CONT "ILITEK: ");
			}
		}
		printk(KERN_CONT "\n\n");
	}
}

u8 ilitek_calc_packet_checksum(u8 *packet, size_t len)
{
	int i;
	s32 sum = 0;

	for (i = 0; i < len; i++)
		sum += packet[i];

	return (u8) ((-sum) & 0xFF);
}

int ilitek_tddi_touch_switch_mode(struct ilitek_tddi_dev *idev, u8 *data)
{
	int ret = 0, mode, prev_mode;
	u8 cmd[4] = {0};

	if (!data) {
		ipio_err("data is null\n");
		return -EINVAL;
	}

	atomic_set(&idev->tp_sw_mode, START);

	mode = data[0];
	prev_mode = idev->actual_fw_mode;
	if (CHECK_EQUAL(mode, prev_mode)) {
		ipio_info("TP mode is the same, do nothing\n");
		goto out;
	}

	idev->actual_fw_mode = mode;

	switch(idev->actual_fw_mode) {
		case P5_X_FW_I2CUART_MODE:
			break;
		case P5_X_FW_DEMO_MODE:
			ipio_info("Switch to Demo mode");
			ilitek_tddi_reset_ctrl(idev, idev->reset_mode);
			break;
		case P5_X_FW_DEBUG_MODE:
			cmd[0] = P5_X_MODE_CONTROL;
			cmd[1] = mode;

			ipio_info("Switch to Debug mode\n");
			ret = idev->write(idev, cmd, 2);
			if (ret < 0)
				ipio_err("Failed to switch Debug mode\n");
			break;
		case P5_X_FW_GESTURE_MODE:
			ret = ilitek_tddi_ic_func_ctrl(idev, "lpwg", ON);
			break;
		case P5_X_FW_TEST_MODE:
			ipio_info("Switch to Test mode\n");
			ret = idev->mp_move_code(idev);
			break;
		default:
			ipio_err("Unknown firmware mode: %x\n", mode);
			ret = -1;
			break;
	}

	if (ret < 0) {
		idev->actual_fw_mode = prev_mode;
		ipio_err("switch mode failed, return to previous mode (%d)\n", idev->actual_fw_mode);
	}

out:
	ipio_info("Actual TP mode = %d\n", idev->actual_fw_mode);
	atomic_set(&idev->tp_sw_mode, DONE);
	return ret;
}

void ilitek_tddi_touch_suspend(struct ilitek_tddi_dev *idev)
{
	ipio_info("TP suspend start\n");
	atomic_set(&idev->tp_suspend, START);
	ilitek_plat_irq_disable(idev);

	ilitek_tddi_ic_func_ctrl(idev, "sense", DISABLE);

	ilitek_tddi_ic_check_busy(idev, 50, 50);

	ilitek_tddi_ic_func_ctrl(idev, "sleep", DISABLE);

	atomic_set(&idev->tp_suspend, DONE);
	ipio_info("TP suspend done\n");
}

void ilitek_tddi_touch_resume(struct ilitek_tddi_dev *idev)
{
	ipio_info("TP resume start\n");
	atomic_set(&idev->tp_resume, START);
	ilitek_plat_irq_disable(idev);

	ilitek_tddi_ic_func_ctrl(idev, "sleep", ENABLE);

	ilitek_tddi_ic_check_busy(idev, 50, 50);

	ilitek_tddi_ic_func_ctrl(idev, "sense", ENABLE);

	ilitek_plat_irq_enable(idev);
	atomic_set(&idev->tp_resume, DONE);
	ipio_info("TP resume done\n");
}

void ilitek_tddi_touch_press(struct ilitek_tddi_dev *idev, u16 x, u16 y, u16 pressure, u16 id)
{
	ipio_info("Touch Press: id = %d, x = %d, y = %d, p = %d\n", id, x, y, pressure);

#ifdef MT_B_TYPE
	input_mt_slot(idev->input, id);
	input_mt_report_slot_state(idev->input, MT_TOOL_FINGER, true);
	input_report_abs(idev->input, ABS_MT_POSITION_X, x);
	input_report_abs(idev->input, ABS_MT_POSITION_Y, y);
#ifdef MT_PRESSURE
	input_report_abs(idev->input, ABS_MT_PRESSURE, pressure);
#endif /* MT_PRESSURE */
#endif /* MT_B_TYPE */
}

void litek_tddi_touch_release(struct ilitek_tddi_dev *idev, u16 x, u16 y, u16 id)
{
	ipio_info("Touch Release: id = %d, x = %d, y = %d\n", id, x, y);

#ifdef MT_B_TYPE
	input_mt_slot(idev->input, id);
	input_mt_report_slot_state(idev->input, MT_TOOL_FINGER, false);
#else
	input_report_key(idev->input_device, BTN_TOUCH, 0);
	input_mt_sync(idev->input_device);
#endif /* MT_B_TYPE */
}

static struct ilitek_touch_info touch_info[MAX_TOUCH_NUM];

void ilitek_tddi_report_ap_mode(struct ilitek_tddi_dev *idev, u8 *buf)
{
	int i = 0;
	u32 xop = 0, yop = 0;

	memset(touch_info, 0x0, sizeof(touch_info) * MAX_TOUCH_NUM);

	idev->finger = 0;

	for (i = 0; i < MAX_TOUCH_NUM; i++) {
		if ((buf[(4 * i) + 1] == 0xFF) && (buf[(4 * i) + 2] && 0xFF)
			&& (buf[(4 * i) + 3] == 0xFF)) {
#ifdef MT_B_TYPE
			idev->curt_touch[i] = 0;
#endif
			continue;
		}

		xop = (((buf[(4 * i) + 1] & 0xF0) << 4) | (buf[(4 * i) + 2]));
		yop = (((buf[(4 * i) + 1] & 0x0F) << 8) | (buf[(4 * i) + 3]));

		touch_info[i].x = xop * idev->panel_wid / TPD_WIDTH;
		touch_info[i].y = yop * idev->panel_hei / TPD_HEIGHT;
		touch_info[i].id = i;
		touch_info[i].pressure = 1;

		ipio_info("original x = %d, y = %d\n", xop, yop);

		idev->finger++;

#ifdef MT_B_TYPE
		idev->curt_touch[i] = 1;
#endif
	}

	ipio_info("figner number = %d, LastTouch = %d\n", idev->finger, idev->last_touch);

	if (idev->finger) {
#ifdef MT_B_TYPE
		for (i = 0; i < idev->finger; i++) {
			input_report_key(idev->input, BTN_TOUCH, 1);
			ilitek_tddi_touch_press(idev, touch_info[i].x, touch_info[i].y,
						touch_info[i].pressure, touch_info[i].id);

			input_report_key(idev->input, BTN_TOOL_FINGER, 1);
		}

		for (i = 0; i < MAX_TOUCH_NUM; i++) {
			if (idev->curt_touch[i] == 0 && idev->prev_touch[i] == 1)
				litek_tddi_touch_release(idev, 0, 0, i);

			idev->prev_touch[i] = idev->curt_touch[i];
		}
#else
#endif
		input_sync(idev->input);
		idev->last_touch = idev->finger;
	} else {
		if (idev->last_touch) {
#ifdef MT_B_TYPE
			for (i = 0; i < MAX_TOUCH_NUM; i++) {
				if (idev->curt_touch[i] == 0 && idev->prev_touch[i] == 1)
					litek_tddi_touch_release(idev, 0, 0, i);

				idev->prev_touch[i] = idev->curt_touch[i];
			}

			input_report_key(idev->input, BTN_TOUCH, 0);
			input_report_key(idev->input, BTN_TOOL_FINGER, 0);
#else
			litek_tddi_touch_release(0, 0, 0);
#endif
			input_sync(idev->input);
			idev->last_touch = 0;
		}
	}
}

void ilitek_tddi_report_debug_mode(struct ilitek_tddi_dev *idev)
{
	ipio_info();
}

void ilitek_tddi_report_gesture_mode(struct ilitek_tddi_dev *idev)
{
	ipio_info();
}
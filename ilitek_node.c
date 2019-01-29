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

#define USER_STR_BUFF	PAGE_SIZE
#define IOCTL_I2C_BUFF	PAGE_SIZE

unsigned char g_user_buf[USER_STR_BUFF] = {0};

static int str2hex(char *str)
{
	int strlen, result, intermed, intermedtop;
	char *s = str;

	while (*s != 0x0) {
		s++;
	}

	strlen = (int)(s - str);
	s = str;
	if (*s != 0x30) {
		return -1;
	}

	s++;

	if (*s != 0x78 && *s != 0x58) {
		return -1;
	}
	s++;

	strlen = strlen - 3;
	result = 0;
	while (*s != 0x0) {
		intermed = *s & 0x0f;
		intermedtop = *s & 0xf0;
		if (intermedtop == 0x60 || intermedtop == 0x40) {
			intermed += 0x09;
		}
		intermed = intermed << (strlen << 2);
		result = result | intermed;
		strlen -= 1;
		s++;
	}
	return result;
}

static int dev_mkdir(char *name, umode_t mode)
{
    struct dentry *dentry;
    struct path path;
    int err;

	ipio_info("mkdir: %s\n", name);

    dentry = kern_path_create(AT_FDCWD, name, &path, LOOKUP_DIRECTORY);
    if (IS_ERR(dentry))
        return PTR_ERR(dentry);

    err = vfs_mkdir(path.dentry->d_inode, dentry, mode);
    done_path_create(&path, dentry);
    return err;
}

static ssize_t ilitek_node_mp_lcm_on_test_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int ret = 0;
	char apk_ret[100] = {0};

	ipio_info("Run MP test with LCM on\n");

	if (*pPos != 0)
		return 0;

	/* Create the directory for mp_test result */
	ret = dev_mkdir(CSV_LCM_ON_PATH, S_IRUGO | S_IWUSR);
    if (ret != 0)
        ipio_err("Failed to create directory for mp_test\n");

	ilitek_tddi_mp_test_handler(idev, apk_ret, ON);

	ret = copy_to_user((char *)buff, apk_ret, sizeof(apk_ret) * 100);
	if (ret < 0)
		ipio_err("Failed to copy data to user space\n");

	return 0;
}

static ssize_t ilitek_node_mp_lcm_off_test_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int ret = 0;
	char apk_ret[100] = {0};

	ipio_info("Run MP test with LCM off\n");

	if (*pPos != 0)
		return 0;

	/* Create the directory for mp_test result */
	ret = dev_mkdir(CSV_LCM_OFF_PATH, S_IRUGO | S_IWUSR);
    if (ret != 0)
        ipio_err("Failed to create directory for mp_test\n");

	ilitek_tddi_mp_test_handler(idev, apk_ret, OFF);

	ret = copy_to_user((char *)buff, apk_ret, sizeof(apk_ret) * 100);
	if (ret < 0)
		ipio_err("Failed to copy data to user space\n");

	return 0;
}

static ssize_t ilitek_node_fw_upgrade_read(struct file *filp, char __user *buff, size_t size, loff_t *pPos)
{
	int ret = 0;
	u32 len = 0;

	ipio_info("Preparing to upgarde firmware\n");

	if (*pPos != 0)
		return 0;

    memset(g_user_buf, 0, USER_STR_BUFF * sizeof(unsigned char));

    ret = ilitek_tddi_fw_upgrade_handler((void *)idev);
	len = sprintf(g_user_buf, "upgrade firwmare %s\n", (ret != 0) ? "failed" : "succeed");

	ret = copy_to_user((uint32_t *) buff, g_user_buf, len);
	if (ret < 0)
        ipio_err("Failed to copy data to user space\n");

	return 0;
}

static ssize_t ilitek_node_ioctl_write(struct file *filp, const char *buff, size_t size, loff_t *pPos)
{
	int ret = 0, count = 0;
	char cmd[512] = { 0 };
	char *token = NULL, *cur = NULL;
	//u8 temp[256] = { 0 };
	u8 *data = NULL;

	if (buff != NULL) {
		ret = copy_from_user(cmd, buff, size - 1);
		if (ret < 0) {
			ipio_info("copy data from user space, failed\n");
			return -1;
		}
	}

	ipio_info("size = %d, cmd = %s\n", (int)size, cmd);

	token = cur = cmd;

	data = kcalloc(512, sizeof(u8), GFP_KERNEL);

	while ((token = strsep(&cur, ",")) != NULL) {
		data[count] = str2hex(token);
		ipio_info("data[%d] = %x\n",count, data[count]);
		count++;
	}

	ipio_info("cmd = %s\n", cmd);

	if (strcmp(cmd, "hwreset") == 0) {
		ilitek_tddi_reset_ctrl(idev, TP_RST_HW_ONLY);
	} else if (strcmp(cmd, "icwholereset") == 0) {
		ilitek_ice_mode_ctrl(idev, ICE_ENABLE, MCU_STOP);
		ilitek_tddi_reset_ctrl(idev, TP_IC_WHOLE_RST);
		ilitek_ice_mode_ctrl(idev, ICE_DISABLE, MCU_STOP);
	} else if (strcmp(cmd, "iccodereset") == 0) {
		ilitek_ice_mode_ctrl(idev, ICE_ENABLE, MCU_STOP);
		ilitek_tddi_reset_ctrl(idev, TP_IC_CODE_RST);
		ilitek_ice_mode_ctrl(idev, ICE_DISABLE, MCU_STOP);
	} else if (strcmp(cmd, "hostdownloadreset") == 0) {
		ilitek_tddi_reset_ctrl(idev, TP_RST_HOST_DOWNLOAD);
	} else {
		ipio_err("Unknown command\n");
	}

	ipio_kfree((void **)&data);
	return size;
}

static long ilitek_node_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return 0;
}

struct proc_dir_entry *proc_dir_ilitek;

typedef struct {
	char *name;
	struct proc_dir_entry *node;
	struct file_operations *fops;
	bool isCreated;
} proc_node_t;

struct file_operations proc_mp_lcm_on_test_fops = {
	.read = ilitek_node_mp_lcm_on_test_read,
};

struct file_operations proc_mp_lcm_off_test_fops = {
	.read = ilitek_node_mp_lcm_off_test_read,
};

struct file_operations proc_ioctl_fops = {
	.unlocked_ioctl = ilitek_node_ioctl,
	.write = ilitek_node_ioctl_write,
};

struct file_operations proc_fw_upgrade_fops = {
	.read = ilitek_node_fw_upgrade_read,
};

proc_node_t proc_table[] = {
	{"ioctl", NULL, &proc_ioctl_fops, false},
	// {"fw_process", NULL, &proc_fw_process_fops, false},
	{"fw_upgrade", NULL, &proc_fw_upgrade_fops, false},
	// {"gesture", NULL, &proc_gesture_fops, false},
	// {"check_battery", NULL, &proc_check_battery_fops, false},
	// {"check_esd", NULL, &proc_check_esd_fops, false},
	// {"debug_level", NULL, &proc_debug_level_fops, false},
	{"mp_lcm_on_test", NULL, &proc_mp_lcm_on_test_fops, false},
	{"mp_lcm_off_test", NULL, &proc_mp_lcm_off_test_fops, false},
	// {"debug_message", NULL, &proc_debug_message_fops, false},
	// {"debug_message_switch", NULL, &proc_debug_message_switch_fops, false},
	// {"fw_pc_counter", NULL, &proc_fw_pc_counter_fops, false},
	// {"show_delta_data", NULL, &proc_get_delta_data_fops, false},
	// {"show_raw_data", NULL, &proc_get_raw_data_fops, false},
	// {"get_debug_mode_data", NULL, &proc_get_debug_mode_data_fops, false},
	// {"read_write_register", NULL, &proc_read_write_register_fops, false},
};

void ilitek_tddi_node_init(struct ilitek_tddi_dev *idev)
{
	int i = 0, ret = 0;

	proc_dir_ilitek = proc_mkdir("ilitek", NULL);

	for (; i < ARRAY_SIZE(proc_table); i++) {
		proc_table[i].node = proc_create(proc_table[i].name, 0666, proc_dir_ilitek, proc_table[i].fops);

		if (proc_table[i].node == NULL) {
			proc_table[i].isCreated = false;
			ipio_err("Failed to create %s under /proc\n", proc_table[i].name);
			ret = -ENODEV;
		} else {
			proc_table[i].isCreated = true;
			ipio_info("Succeed to create %s under /proc\n", proc_table[i].name);
		}
	}

	//netlink_init();
}
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
#define ILITEK_IOCTL_MAGIC	100
#define ILITEK_IOCTL_MAXNR	19

#define ILITEK_IOCTL_I2C_WRITE_DATA			_IOWR(ILITEK_IOCTL_MAGIC, 0, u8*)
#define ILITEK_IOCTL_I2C_SET_WRITE_LENGTH	_IOWR(ILITEK_IOCTL_MAGIC, 1, int)
#define ILITEK_IOCTL_I2C_READ_DATA			_IOWR(ILITEK_IOCTL_MAGIC, 2, u8*)
#define ILITEK_IOCTL_I2C_SET_READ_LENGTH	_IOWR(ILITEK_IOCTL_MAGIC, 3, int)

#define ILITEK_IOCTL_TP_HW_RESET			_IOWR(ILITEK_IOCTL_MAGIC, 4, int)
#define ILITEK_IOCTL_TP_POWER_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 5, int)
#define ILITEK_IOCTL_TP_REPORT_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 6, int)
#define ILITEK_IOCTL_TP_IRQ_SWITCH			_IOWR(ILITEK_IOCTL_MAGIC, 7, int)

#define ILITEK_IOCTL_TP_DEBUG_LEVEL			_IOWR(ILITEK_IOCTL_MAGIC, 8, int)
#define ILITEK_IOCTL_TP_FUNC_MODE			_IOWR(ILITEK_IOCTL_MAGIC, 9, int)

#define ILITEK_IOCTL_TP_FW_VER				_IOWR(ILITEK_IOCTL_MAGIC, 10, u8*)
#define ILITEK_IOCTL_TP_PL_VER				_IOWR(ILITEK_IOCTL_MAGIC, 11, u8*)
#define ILITEK_IOCTL_TP_CORE_VER			_IOWR(ILITEK_IOCTL_MAGIC, 12, u8*)
#define ILITEK_IOCTL_TP_DRV_VER				_IOWR(ILITEK_IOCTL_MAGIC, 13, u8*)
#define ILITEK_IOCTL_TP_CHIP_ID				_IOWR(ILITEK_IOCTL_MAGIC, 14, uint32_t*)

#define ILITEK_IOCTL_TP_NETLINK_CTRL		_IOWR(ILITEK_IOCTL_MAGIC, 15, int*)
#define ILITEK_IOCTL_TP_NETLINK_STATUS		_IOWR(ILITEK_IOCTL_MAGIC, 16, int*)

#define ILITEK_IOCTL_TP_MODE_CTRL			_IOWR(ILITEK_IOCTL_MAGIC, 17, u8*)
#define ILITEK_IOCTL_TP_MODE_STATUS			_IOWR(ILITEK_IOCTL_MAGIC, 18, int*)
#define ILITEK_IOCTL_ICE_MODE_SWITCH		_IOWR(ILITEK_IOCTL_MAGIC, 19, int)

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

	ilitek_tddi_mp_test_handler(apk_ret, ON);

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

	ilitek_tddi_mp_test_handler(apk_ret, OFF);

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

    ret = ilitek_tddi_fw_upgrade_handler(NULL);
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
	u8 *data = NULL, tp_mode;

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
		ilitek_tddi_reset_ctrl(TP_RST_HW_ONLY);
	} else if (strcmp(cmd, "icwholereset") == 0) {
		ilitek_ice_mode_ctrl(ENABLE, OFF);
		ilitek_tddi_reset_ctrl(TP_IC_WHOLE_RST);
		ilitek_ice_mode_ctrl(DISABLE, OFF);
	} else if (strcmp(cmd, "iccodereset") == 0) {
		ilitek_ice_mode_ctrl(ENABLE, OFF);
		ilitek_tddi_reset_ctrl(TP_IC_CODE_RST);
		ilitek_ice_mode_ctrl(DISABLE, OFF);
	} else if (strcmp(cmd, "hostdownloadreset") == 0) {
		ilitek_tddi_reset_ctrl(TP_RST_HOST_DOWNLOAD);
	} else if (strcmp(cmd, "getinfo") == 0) {
		ilitek_tddi_ic_get_info();
		ilitek_tddi_ic_get_protocl_ver();
		ilitek_tddi_ic_get_fw_ver();
		ilitek_tddi_ic_get_core_ver();
		ilitek_tddi_ic_get_tp_info();
		ilitek_tddi_ic_get_panel_info();
	} else if (strcmp(cmd, "enablewqesd") == 0) {
		ilitek_tddi_wq_ctrl(ESD, ENABLE);
	} else if (strcmp(cmd, "enablewqbat") == 0) {
		ilitek_tddi_wq_ctrl(BAT, ENABLE);
	} else if (strcmp(cmd, "disablewqesd") == 0) {
		ilitek_tddi_wq_ctrl(ESD, DISABLE);
	} else if (strcmp(cmd, "disablewqbat") == 0) {
		ilitek_tddi_wq_ctrl(BAT, DISABLE);
	} else if (strcmp(cmd, "enablenetlink") == 0) {
		idev->netlink = ENABLE;
	} else if (strcmp(cmd, "disablenetlink") == 0) {
		idev->netlink = DISABLE;
	} else if (strcmp(cmd, "switchtestmode") == 0) {
		tp_mode = P5_X_FW_TEST_MODE;
		ilitek_tddi_touch_switch_mode(&tp_mode);
	} else if (strcmp(cmd, "switchdebugmode") == 0) {
		tp_mode = P5_X_FW_DEBUG_MODE;
		ilitek_tddi_touch_switch_mode(&tp_mode);
	} else if (strcmp(cmd, "switchdemomode") == 0) {
		tp_mode = P5_X_FW_DEMO_MODE;
		ilitek_tddi_touch_switch_mode(&tp_mode);
	} else {
		ipio_err("Unknown command\n");
	}

	ipio_kfree((void **)&data);
	return size;
}

static long ilitek_node_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0, length = 0;
	u8 *szBuf = NULL;
	static u16 i2c_rw_length = 0;
	u32 id_to_user[3] = {0};
	char dbg[10] = { 0 };

	if (_IOC_TYPE(cmd) != ILITEK_IOCTL_MAGIC) {
		ipio_err("The Magic number doesn't match\n");
		return -ENOTTY;
	}

	if (_IOC_NR(cmd) > ILITEK_IOCTL_MAXNR) {
		ipio_err("The number of ioctl doesn't match\n");
		return -ENOTTY;
	}

	szBuf = kcalloc(IOCTL_I2C_BUFF, sizeof(u8), GFP_KERNEL);
	if (ERR_ALLOC_MEM(szBuf)) {
		ipio_err("Failed to allocate mem\n");
		return -ENOMEM;
	}

	switch (cmd) {
	case ILITEK_IOCTL_I2C_WRITE_DATA:
		ipio_info("ioctl: write len = %d\n", i2c_rw_length);
		ret = copy_from_user(szBuf, (u8 *) arg, i2c_rw_length);
		if (ret < 0) {
			ipio_err("Failed to copy data from user space\n");
			break;
		}
		ret = idev->write(&szBuf[0], i2c_rw_length);
		if (ret < 0)
			ipio_err("Failed to write data\n");
		break;
	case ILITEK_IOCTL_I2C_READ_DATA:
		ipio_info("ioctl: read len = %d\n", i2c_rw_length);
		ret = idev->read(szBuf, i2c_rw_length);
		if (ret < 0) {
			ipio_err("Failed to read data\n");
			break;
		}
		ret = copy_to_user((u8 *) arg, szBuf, i2c_rw_length);
		if (ret < 0)
			ipio_err("Failed to copy data to user space\n");
		break;
	case ILITEK_IOCTL_I2C_SET_WRITE_LENGTH:
	case ILITEK_IOCTL_I2C_SET_READ_LENGTH:
		i2c_rw_length = arg;
		break;
	case ILITEK_IOCTL_TP_HW_RESET:
		ipio_info("ioctl: hw reset\n");
		ilitek_tddi_reset_ctrl(idev->reset_mode);
		break;
	case ILITEK_IOCTL_TP_POWER_SWITCH:
		ipio_info("Not implemented yet\n");
		break;
	case ILITEK_IOCTL_TP_REPORT_SWITCH:
		ret = copy_from_user(szBuf, (u8 *) arg, 1);
		if (ret < 0) {
			ipio_err("Failed to copy data from user space\n");
			break;
		}
		ipio_info("ioctl: report switch = %d\n", szBuf[0]);
		if (szBuf[0]) {
			idev->report = ENABLE;
			ipio_info("report is enabled\n");
		} else {
			idev->report = DISABLE;
			ipio_info("report is disabled\n");
		}
		break;
	case ILITEK_IOCTL_TP_IRQ_SWITCH:
		ret = copy_from_user(szBuf, (u8 *) arg, 1);
		if (ret < 0) {
			ipio_err("Failed to copy data from user space\n");
			break;
		}
		ipio_info("ioctl: irq switch = %d\n", szBuf[0]);
		if (szBuf[0])
			ilitek_plat_irq_enable();
		else
			ilitek_plat_irq_disable();
		break;
	case ILITEK_IOCTL_TP_DEBUG_LEVEL:
		ret = copy_from_user(dbg, (uint32_t *) arg, sizeof(uint32_t));
		if (ret < 0) {
			ipio_err("Failed to copy data from user space\n");
			break;
		}
		ipio_debug_level = katoi(dbg);
		ipio_info("ipio_debug_level = %d", ipio_debug_level);
		break;
	case ILITEK_IOCTL_TP_FUNC_MODE:
		ret = copy_from_user(szBuf, (u8 *) arg, 3);
		if (ret < 0) {
			ipio_err("Failed to copy data from user space\n");
			break;
		}
		ipio_info("ioctl: set func mode = %x,%x,%x\n", szBuf[0],szBuf[1],szBuf[2]);
		idev->write(&szBuf[0], 3);
		break;
	case ILITEK_IOCTL_TP_FW_VER:
		ipio_info("ioctl: get fw version\n");
		ret = ilitek_tddi_ic_get_fw_ver();
		if (ret < 0) {
			ipio_err("Failed to get firmware version\n");
			break;
		}
		szBuf[3] = idev->chip->fw_ver & 0xFF;
		szBuf[2] = (idev->chip->fw_ver >> 8) & 0xFF;
		szBuf[1] = (idev->chip->fw_ver >> 16) & 0xFF;
		szBuf[0] = idev->chip->fw_ver >> 24;
		ipio_info("Firmware version = %d.%d.%d.%d\n", szBuf[0], szBuf[1], szBuf[2], szBuf[3]);
		ret = copy_to_user((u8 *) arg, szBuf, 4);
		if (ret < 0)
			ipio_err("Failed to copy firmware version to user space\n");
		break;
	case ILITEK_IOCTL_TP_PL_VER:
		ipio_info("ioctl: get protocl version\n");
		ret = ilitek_tddi_ic_get_protocl_ver();
		if (ret < 0) {
			ipio_err("Failed to get protocol version\n");
			break;
		}
		szBuf[2] = idev->protocol->ver & 0xFF;
		szBuf[1] = (idev->protocol->ver >> 8) & 0xFF;
		szBuf[0] = idev->protocol->ver >> 16;
		ipio_info("Protocol version = %d.%d.%d\n", szBuf[0], szBuf[1], szBuf[2]);
		ret = copy_to_user((u8 *) arg, szBuf, 3);
		if (ret < 0)
			ipio_err("Failed to copy protocol version to user space\n");
		break;
	case ILITEK_IOCTL_TP_CORE_VER:
		ipio_info("ioctl: get core version\n");
		ret = ilitek_tddi_ic_get_core_ver();
		if (ret < 0) {
			ipio_err("Failed to get core version\n");
			break;
		}
		szBuf[3] = idev->chip->core_ver & 0xFF;
		szBuf[2] = (idev->chip->core_ver >> 8) & 0xFF;
		szBuf[1] = (idev->chip->core_ver >> 16) & 0xFF;
		szBuf[0] = idev->chip->core_ver >> 24;
		ipio_info("Core version = %d.%d.%d.%d\n", szBuf[0], szBuf[1], szBuf[2], szBuf[3]);
		ret = copy_to_user((u8 *) arg, szBuf, 4);
		if (ret < 0)
			ipio_err("Failed to copy core version to user space\n");
		break;
	case ILITEK_IOCTL_TP_DRV_VER:
		ipio_info("ioctl: get driver version\n");
		length = sprintf(szBuf, "%s", DRIVER_VERSION);
	ret = copy_to_user((u8 *) arg, szBuf, length);
		if (ret < 0) {
			ipio_err("Failed to copy driver ver to user space\n");
		}
		break;
	case ILITEK_IOCTL_TP_CHIP_ID:
		ipio_info("ioctl: get chip id\n");
		ret = ilitek_tddi_ic_get_info();
		if (ret < 0) {
			ipio_err("Failed to get chip id\n");
			break;
		}
		id_to_user[0] = idev->chip->pid;
		id_to_user[1] = idev->chip->otp_id;
		id_to_user[2] = idev->chip->ana_id;
		ret = copy_to_user((uint32_t *) arg, id_to_user, sizeof(id_to_user));
		if (ret < 0)
			ipio_err("Failed to copy chip id to user space\n");
		break;
	case ILITEK_IOCTL_TP_NETLINK_CTRL:
		ret = copy_from_user(szBuf, (u8 *) arg, 1);
		if (ret < 0) {
			ipio_err("Failed to copy data from user space\n");
			break;
		}
		ipio_info("ioctl: netlink ctrl = %d\n", szBuf[0]);
		if (szBuf[0])
			idev->netlink = ENABLE;
		else
			idev->netlink = DISABLE;
		break;
	case ILITEK_IOCTL_TP_NETLINK_STATUS:
		ipio_info("ioctl: get netlink stat = %d\n", idev->netlink);
		ret = copy_to_user((int *)arg, &idev->netlink, sizeof(int));
		if (ret < 0)
			ipio_err("Failed to copy chip id to user space\n");
		break;
	case ILITEK_IOCTL_TP_MODE_CTRL:
		ret = copy_from_user(szBuf, (u8 *) arg, 4);
		if (ret < 0) {
			ipio_err("Failed to copy data from user space\n");
			break;
		}
		ipio_info("ioctl: switch fw mode = %d\n", szBuf[0]);
		ilitek_tddi_touch_switch_mode(szBuf);
		break;
	case ILITEK_IOCTL_TP_MODE_STATUS:
		ipio_info("ioctl: current firmware mode = %d", idev->actual_fw_mode);
		ret = copy_to_user((int *)arg, &idev->actual_fw_mode, sizeof(int));
		if (ret < 0)
			ipio_err("Failed to copy chip id to user space\n");
		break;
	/* It works for host downloado only */
	case ILITEK_IOCTL_ICE_MODE_SWITCH:
		ret = copy_from_user(szBuf, (u8 *) arg, 1);
		if (ret < 0) {
			ipio_err("Failed to copy data from user space\n");
			break;
		}
		ipio_info("ioctl: switch ice mode = %d", szBuf[0]);
		if (szBuf[0])
			atomic_set(&idev->ice_stat, ENABLE);
		else
			atomic_set(&idev->ice_stat, DISABLE);
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	ipio_kfree((void **)&szBuf);
	return ret;
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

#define NETLINK_USER 21
struct sock *netlink_skb;
struct nlmsghdr *netlink_head;
struct sk_buff *skb_out;
int netlink_pid;

void netlink_reply_msg(void *raw, int size)
{
	int ret;
	int msg_size = size;
	u8 *data = (u8 *) raw;

	ipio_debug(DEBUG_NETLINK, "The size of data being sent to user = %d\n", msg_size);
	ipio_debug(DEBUG_NETLINK, "pid = %d\n", netlink_pid);
	ipio_debug(DEBUG_NETLINK, "Netlink is enable = %d\n", idev->netlink);

	if (idev->netlink) {
		skb_out = nlmsg_new(msg_size, 0);

		if (!skb_out) {
			ipio_err("Failed to allocate new skb\n");
			return;
		}

		netlink_head = nlmsg_put(skb_out, 0, 0, NLMSG_DONE, msg_size, 0);
		NETLINK_CB(skb_out).dst_group = 0;	/* not in mcast group */

		/* strncpy(NLMSG_DATA(netlink_head), data, msg_size); */
		ipio_memcpy(nlmsg_data(netlink_head), data, msg_size, size);

		ret = nlmsg_unicast(netlink_skb, skb_out, netlink_pid);
		if (ret < 0)
			ipio_err("Failed to send data back to user\n");
	}
}

static void netlink_recv_msg(struct sk_buff *skb)
{
	netlink_pid = 0;

	ipio_debug(DEBUG_NETLINK, "Netlink = %d\n", idev->netlink);

	netlink_head = (struct nlmsghdr *)skb->data;

	ipio_debug(DEBUG_NETLINK, "Received a request from client: %s, %d\n",
	    (char *)NLMSG_DATA(netlink_head), (int)strlen((char *)NLMSG_DATA(netlink_head)));

	/* pid of sending process */
	netlink_pid = netlink_head->nlmsg_pid;

	ipio_debug(DEBUG_NETLINK, "the pid of sending process = %d\n", netlink_pid);

	/* TODO: may do something if there's not receiving msg from user. */
	if (netlink_pid != 0) {
		ipio_err("The channel of Netlink has been established successfully !\n");
		idev->netlink = ENABLE;
	} else {
		ipio_err("Failed to establish the channel between kernel and user space\n");
		idev->netlink = DISABLE;
	}
}

static int netlink_init(void)
{
	int ret = 0;

#if KERNEL_VERSION(3, 4, 0) > LINUX_VERSION_CODE
	netlink_skb = netlink_kernel_create(&init_net, NETLINK_USER, netlink_recv_msg, NULL, THIS_MODULE);
#else
	struct netlink_kernel_cfg cfg = {
		.input = netlink_recv_msg,
	};

	netlink_skb = netlink_kernel_create(&init_net, NETLINK_USER, &cfg);
#endif

	ipio_info("Initialise Netlink and create its socket\n");

	if (!netlink_skb) {
		ipio_err("Failed to create nelink socket\n");
		ret = -EFAULT;
	}
	return ret;
}

void ilitek_tddi_node_init(void)
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
	netlink_init();
}
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
#ifndef __ILITEK_H
#define __ILITEK_H

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/kobject.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/version.h>
#include <linux/regulator/consumer.h>
#include <linux/power_supply.h>
#include <linux/fs.h>
#include <asm/uaccess.h>

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/ctype.h>

#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <linux/socket.h>
#include <net/sock.h>

#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/time.h>

#include <linux/namei.h>
#include <linux/vmalloc.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>

#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <linux/security.h>
#include <linux/mount.h>

#ifdef CONFIG_OF
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#else
#include <linux/earlysuspend.h>
#endif

#ifdef CONFIG_MTK_SPI
#include "mt_spi.h"
#include "sync_write.h"
#endif

#define QCOM 1
#define MTK  2
#define PLATFORM MTK

/*  Debug messages */
#ifdef BIT
#undef BIT
#endif
#define BIT(x)	(1 << (x))

enum {
	DEBUG_NONE = 0,
	DEBUG_IRQ = BIT(0),
	DEBUG_FINGER_REPORT = BIT(1),
	DEBUG_FIRMWARE = BIT(2),
	DEBUG_CONFIG = BIT(3),
	DEBUG_I2C = BIT(4),
	DEBUG_BATTERY = BIT(5),
	DEBUG_MP_TEST = BIT(6),
	DEBUG_IOCTL = BIT(7),
	DEBUG_NETLINK = BIT(8),
	DEBUG_PARSER = BIT(9),
	DEBUG_GESTURE = BIT(10),
	DEBUG_SPI = BIT(11),
	DEBUG_ALL = ~0,
};

extern u32 ipio_debug_level;
#define ipio_info(fmt, arg...)	\
	pr_info("ILITEK: (%s, %d): " fmt, __func__, __LINE__, ##arg);

#define ipio_err(fmt, arg...)	\
	pr_err("ILITEK: (%s, %d): " fmt, __func__, __LINE__, ##arg);

#define ipio_debug(level, fmt, arg...)									\
	do {																\
		if (level & ipio_debug_level)									\
		pr_info("ILITEK: (%s, %d): " fmt, __func__, __LINE__, ##arg);	\
	} while (0)

#define CHECK_EQUAL(X, Y) 	((X == Y) ? 1 : 0)
#define ERR_ALLOC_MEM(X)	((IS_ERR(X) || X == NULL) ? 1 : 0)

enum IRQ_STATUS {
	IRQ_DISABLE = 0,
	IRQ_ENABLE
};

enum ICE_MODE_STATUS {
	ICE_DISABLE = 0,
	ICE_ENABLE
};

enum MCU_STATUS {
	MCU_STOP = 0,
	MCU_ON
};

enum TP_RST_METHOD{
	TP_RST_SW = 0,
	TP_RST_HW_ONLY,
	TP_RST_HOST_DOWNLOAD
};

enum TP_RST_STATUS {
	TP_RST_END = 0,
	TP_RST_START
};

enum TP_BUS_TYPE {
	TP_BUS_I2C = 0,
	TP_BUS_SPI
};

enum TP_PLAT_TYPE {
	TP_PLAT_MTK = 0,
	TP_PLAT_QCOM
};

enum TP_FW_UPGRADE_TYPE {
	UPGRADE_FLASH = 0,
	UPGRADE_IRAM
};

enum TP_FW_UPGRADE_STATUS {
	FW_IDLE = 0,
	FW_RUNNING
};

enum TP_SUSP_STATUS {
	DONE = 0,
	START,
};

enum TP_FUNC_CTRL_STATUS {
	DISABLE = 0,
	ENABLE = 1,
	OFF = 0,
	ON = 1
};

#define TDDI_I2C_ADDR	0x41
#define TDDI_DEV_ID	"ILITEK_TDDI"

 /* define the width and heigth of a screen. */
#define TOUCH_SCREEN_X_MIN 0
#define TOUCH_SCREEN_Y_MIN 0
#define TOUCH_SCREEN_X_MAX 720
#define TOUCH_SCREEN_Y_MAX 1440

/* define the range on panel */
#define TPD_HEIGHT 2048
#define TPD_WIDTH 2048

#define MAX_TOUCH_NUM	10

/* Linux multiple touch protocol, either B type or A type. */
#define MT_B_TYPE
/* Report points with pressule value */
#define MT_PRESSURE

struct ilitek_tddi_dev
{
    struct i2c_client *i2c;
    struct spi_device *spi;
    struct input_dev *input;
    struct device *dev;

	struct ilitek_ic_info *chip;
	struct ilitek_protocol_info *protocol;

	struct mutex touch_mutex;
	struct mutex io_mutex;
	spinlock_t irq_spin;

	u16 max_x;
	u16 max_y;
	u16 min_x;
	u16 min_y;
	u16 panel_wid;
	u16 panel_hei;
	u8 xch_num;
	u8 ych_num;

	int actual_fw_mode;

	int irq_num;
	int tp_rst;
	int tp_int;

	int finger;
	int curt_touch[MAX_TOUCH_NUM];
	int prev_touch[MAX_TOUCH_NUM];
	int last_touch;

	/* host download */
	int reset_mode;
	int fw_upgrade_mode;

	/* Timing for tp reset */
	int delay_time_high;
	int delay_time_low;
	int edge_delay;

	atomic_t irq_stat;
	atomic_t tp_reset;
	atomic_t ice_stat;
	atomic_t fw_stat;
	atomic_t tp_suspend;
	atomic_t tp_resume;
	atomic_t tp_sw_mode;

    int (*write)(struct ilitek_tddi_dev *, void *, size_t);
    int (*read)(struct ilitek_tddi_dev *, void *, size_t);
	void (*suspend)(struct ilitek_tddi_dev *);
	void (*resume)(struct ilitek_tddi_dev *);
	int (*mp_move_code)(struct ilitek_tddi_dev *);
};
extern struct ilitek_tddi_dev *idev;

struct ilitek_touch_info
{
	u16 id;
	u16 x;
	u16 y;
	u16 pressure;
};

struct ilitek_protocol_info
{
	u32 ver;
	int fw_ver_len;
	int pro_ver_len;
	int tp_info_len;
	int key_info_len;
	int panel_info_len;
	int core_ver_len;
	int func_ctrl_len;
	int window_len;
	int cdc_len;
	int mp_info_len;
};

struct ilitek_ic_func_ctrl
{
	const char *name;
	u8 cmd[3];
	int len;
};

struct ilitek_ic_info
{
	u32 pid_addr;
	u32 wdt_addr;
	u32 pc_counter_addr;
    u32 reset_addr;
	u32 otp_addr;
	u32 ana_addr;
	u32 pid;
	u16 id;
	u16 type_hi;
	u16 type_low;
	u32 otp_id;
	u32 ana_id;
    u32 fw_ver;
	int (*ice_mode_ctrl)(struct ilitek_tddi_dev *, bool, bool);
	int (*ice_write)(struct ilitek_tddi_dev *, u32, u32, size_t);
	int (*ice_read)(struct ilitek_tddi_dev *, u32, u32 *);
};

struct ilitek_hwif_info
{
	u8 bus_type;
	u8 plat_type;
	const char *name;
	struct module *owner;
	const struct of_device_id *of_match_table;
	int (*plat_probe)(struct ilitek_tddi_dev *);
	int (*plat_remove)(struct ilitek_tddi_dev *);
	void *info;
};

static inline void ipio_kfree(void **mem) {
	if (*mem != NULL) {
		kfree(*mem);
		*mem = NULL;
	}
}

static inline void ipio_vfree(void **mem) {
	if (*mem != NULL) {
		vfree(*mem);
		*mem = NULL;
	}
}

static inline void *ipio_memcpy(void *dest, const void *src, size_t n, size_t dest_size) {
    if (n > dest_size)
         n = dest_size;

    return memcpy(dest, src, n);
}

/* Protocol command definiation */
#define P5_X_READ_DATA_CTRL			    0xF6
#define P5_X_GET_TP_INFORMATION		    0x20
#define P5_X_GET_KEY_INFORMATION	    0x27
#define P5_X_GET_PANEL_INFORMATION	    0x29
#define P5_X_GET_FW_VERSION 			0x21
#define P5_X_GET_PROTOCOL_VERSION	    0x22
#define P5_X_GET_CORE_VERSION		    0x23
#define P5_X_MODE_CONTROL			    0xF0
#define P5_X_SET_CDC_INIT               0xF1
#define P5_X_GET_CDC_DATA               0xF2
#define P5_X_CDC_BUSY_STATE			    0xF3
#define P5_X_MP_TEST_MODE_INFO			0xFE
#define P5_X_I2C_UART				    0x40

#define P5_X_FW_UNKNOWN_MODE		0xFF
#define P5_X_FW_DEMO_MODE			0x00
#define P5_X_FW_TEST_MODE			0x01
#define P5_X_FW_DEBUG_MODE 			0x02
#define P5_X_FW_I2CUART_MODE		0x03
#define P5_X_FW_GESTURE_MODE		0x04

#define P5_X_DEMO_MODE_PACKET_LENGTH	43
#define P5_X_DEBUG_MODE_PACKET_LENGTH	1280
#define P5_X_TEST_MODE_PACKET_LENGTH	1180

#define P5_X_DEMO_PACKET_ID		        0x5A
#define P5_X_DEBUG_PACKET_ID	        0xA7
#define P5_X_TEST_PACKET_ID		        0xF2
#define P5_X_GESTURE_PACKET_ID	        0xAA
#define P5_X_I2CUART_PACKET_ID	        0x7A

/* Prototypes for tddi mp test */
extern int ilitek_tddi_mp_move_code_flash(struct ilitek_tddi_dev *idev);
extern int ilitek_tddi_mp_move_code_iram(struct ilitek_tddi_dev *idev);

/* Prototypes for tddi core functions */
extern void ilitek_tddi_report_ap_mode(struct ilitek_tddi_dev *, u8 *);
extern void ilitek_tddi_report_debug_mode(struct ilitek_tddi_dev *);
extern void ilitek_tddi_report_gesture_mode(struct ilitek_tddi_dev *);
extern int ilitek_tddi_ic_func_ctrl(struct ilitek_tddi_dev *, const char *, int);
extern u32 ilitek_tddi_ic_get_pc_counter(struct ilitek_tddi_dev *);
extern int ilitek_tddi_ic_check_busy(struct ilitek_tddi_dev *idev, int, int);
extern int ilitek_tddi_ic_get_panel_info(struct ilitek_tddi_dev *);
extern int ilitek_tddi_ic_get_tp_info(struct ilitek_tddi_dev *);
extern int ilitek_tddi_ic_get_protocl_ver(struct ilitek_tddi_dev *);
extern int ilitek_tddi_ic_get_fw_ver(struct ilitek_tddi_dev *);
extern int ilitek_tddi_ic_get_info(struct ilitek_tddi_dev *);
extern int ilitek_ice_mode_ctrl(struct ilitek_tddi_dev *, bool, bool);
extern int ilitek_tddi_ic_init(struct ilitek_tddi_dev *);

/* Prototypes for tddi events */
extern int ilitek_tddi_esd_handler(struct ilitek_tddi_dev *);
extern int ilitek_tddi_mp_test_handler(struct ilitek_tddi_dev *, bool);
extern void ilitek_tddi_report_handler(struct ilitek_tddi_dev *);
extern int ilitek_tddi_touch_suspend(struct ilitek_tddi_dev *);
extern int ilitek_tddi_touch_resume(struct ilitek_tddi_dev *);
extern int ilitek_tddi_reset_ctrl(struct ilitek_tddi_dev *, int);
extern int ilitek_tddi_init(struct ilitek_tddi_dev *);
extern int ilitek_tddi_dev_init(struct ilitek_hwif_info *);

/* Prototypes for i2c interface */
extern int ilitek_i2c_dev_init(struct ilitek_hwif_info *);

/* Prototypes for spi interface */
//extern int ilitek_spi_dev_init(struct ilitek_hwif_info *);

/* Prototypes for platform level */
extern void ilitek_plat_irq_disable(struct ilitek_tddi_dev *);
extern void ilitek_plat_irq_enable(struct ilitek_tddi_dev *);
extern void ilitek_plat_tp_reset(struct ilitek_tddi_dev *);

/* Prototypes for miscs */
extern void ilitek_dump_data(void *, int, int, int, const char *);
extern u8 ilitek_calc_packet_checksum(u8 *, size_t);
#endif /* __ILITEK_H */
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
#include <linux/firmware.h>

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

#define DRIVER_VERSION "2.0.0.0"

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
#define K (1024)
#define M (K * K)

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
	TP_IC_WHOLE_RST = 0,
	TP_IC_CODE_RST,
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

enum TP_FW_UPGRADE_TARGET {
	ILI_FILE = 0,
	HEX_FILE
};

enum TP_FW_OPEN_METHOD {
	REQUEST_FIRMWARE = 0,
	FILP_OPEN
};

enum TP_FW_UPGRADE_STATUS {
	FW_IDLE = 0,
	FW_RUNNING
};

enum TP_FW_BLOCK_NUM {
	AP = 1,
	DATA = 2,
	TUNING = 3,
	GESTURE = 4,
	MP = 5,
	DDI = 6
};

/* FW block info tag */
enum TP_FW_BLOCK_TAG {
	BLOCK_TAG_AE = 0xAE,
	BLOCK_TAG_AF = 0xAF,
	BLOCK_TAG_B0 = 0xB0
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
#define MAX_TOUCH_NUM	10

/* define the range on panel */
#define TPD_HEIGHT 2048
#define TPD_WIDTH 2048

/* Firmware upgrade */
#define MAX_HEX_FILE_SIZE			(160*1024)
#define MAX_FLASH_FIRMWARE_SIZE		(256*1024)
#define MAX_IRAM_FIRMWARE_SIZE		(60*1024)
#define ILI_FILE_HEADER				64
#define MAX_AP_FIRMWARE_SIZE		(64*1024)
#define MAX_DLM_FIRMWARE_SIZE		(8*1024)
#define MAX_MP_FIRMWARE_SIZE		(64*1024)
#define MAX_GESTURE_FIRMWARE_SIZE	(8*1024)
#define MAX_TUNING_FIRMWARE_SIZE	(4*1024)
#define MAX_DDI_FIRMWARE_SIZE		(4*1024)
#define DLM_START_ADDRESS           0x20610
#define DLM_HEX_ADDRESS             0x10000
#define MP_HEX_ADDRESS              0x13000
#define RESERVE_BLOCK_START_ADDR	0x1D000
#define RESERVE_BLOCK_END_ADDR		0x1DFFF
#define FW_VER_ADDR					0xFFE0
#define SPI_UPGRADE_LEN				2048
#define SPI_READ_LEN				2048
#define FW_BLOCK_INFO_NUM			7

/* DMA Control Registers */
#define DMA_BASED_ADDR 						0x72000
#define DMA48_ADDR  						(DMA_BASED_ADDR + 0xC0)
#define DMA48_reg_dma_ch0_busy_flag         DMA48_ADDR
#define DMA48_reserved_0                    0xFFFE
#define DMA48_reg_dma_ch0_trigger_sel       BIT(16)|BIT(17)|BIT(18)|BIT(19)
#define DMA48_reserved_1                    BIT(20)|BIT(21)|BIT(22)|BIT(23)
#define DMA48_reg_dma_ch0_start_set         BIT(24)
#define DMA48_reg_dma_ch0_start_clear       BIT(25)
#define DMA48_reg_dma_ch0_trigger_src_mask  BIT(26)
#define DMA48_reserved_2                    BIT(27)

#define DMA49_ADDR  						(DMA_BASED_ADDR + 0xC4)
#define DMA49_reg_dma_ch0_src1_addr         DMA49_ADDR
#define DMA49_reserved_0                    BIT(20)

#define DMA50_ADDR  						(DMA_BASED_ADDR + 0xC8)
#define DMA50_reg_dma_ch0_src1_step_inc     DMA50_ADDR
#define DMA50_reserved_0                    (DMA50_ADDR + 0x01)
#define DMA50_reg_dma_ch0_src1_format       BIT(24)|BIT(25)
#define DMA50_reserved_1                    BIT(26)|BIT(27)|BIT(28)|BIT(29)|BIT(30)
#define DMA50_reg_dma_ch0_src1_en           BIT(31)

#define DMA52_ADDR  						(DMA_BASED_ADDR + 0xD0)
#define DMA52_reg_dma_ch0_src2_step_inc     DMA52_ADDR
#define DMA52_reserved_0                    (DMA52_ADDR + 0x01)
#define DMA52_reg_dma_ch0_src2_format       BIT(24)|BIT(25)
#define DMA52_reserved_1                    BIT(26)|BIT(27)|BIT(28)|BIT(29)|BIT(30)
#define DMA52_reg_dma_ch0_src2_en           BIT(31) // [RESET] h0

#define DMA53_ADDR  						(DMA_BASED_ADDR + 0xD4)
#define DMA53_reg_dma_ch0_dest_addr         DMA53_ADDR
#define DMA53_reserved_0                    BIT(20)

#define DMA54_ADDR  						(DMA_BASED_ADDR + 0xD8)
#define DMA54_reg_dma_ch0_dest_step_inc     DMA54_ADDR
#define DMA54_reserved_0                    (DMA54_ADDR + 0x01)
#define DMA54_reg_dma_ch0_dest_format       BIT(24)|BIT(25)
#define DMA54_reserved_1                    BIT(26)|BIT(27)|BIT(28)|BIT(29)|BIT(30)
#define DMA54_reg_dma_ch0_dest_en           BIT(31)

#define DMA55_ADDR  						(DMA_BASED_ADDR + 0xDC)
#define DMA55_reg_dma_ch0_trafer_counts     DMA55_ADDR
#define DMA55_reserved_0                    BIT(17)|BIT(18)|BIT(19)|BIT(20)|BIT(21)|BIT(22)|BIT(23)
#define DMA55_reg_dma_ch0_trafer_mode       BIT(24)|BIT(25)|BIT(26)|BIT(27)
#define DMA55_reserved_1                    BIT(28)|BIT(29)|BIT(30)|BIT(31)

/* INT Function Registers */
#define INTR_BASED_ADDR     					0x48000
#define INTR1_ADDR								(INTR_BASED_ADDR + 0x4)
#define INTR1_reg_uart_tx_int_flag            	INTR1_ADDR
#define INTR1_reserved_0                      	BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)
#define INTR1_reg_wdt_alarm_int_flag          	BIT(8)
#define INTR1_reserved_1                      	BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15)
#define INTR1_reg_dma_ch1_int_flag            	BIT(16)
#define INTR1_reg_dma_ch0_int_flag            	BIT(17)
#define INTR1_reg_dma_frame_done_int_flag     	BIT(18)
#define INTR1_reg_dma_tdi_done_int_flag       	BIT(19)
#define INTR1_reserved_2                      	BIT(20)|BIT(21)|BIT(22)|BIT(23)
#define INTR1_reg_flash_error_flag           	BIT(24)
#define INTR1_reg_flash_int_flag              	BIT(25)
#define INTR1_reserved_3                      	BIT(26)

#define INTR2_ADDR  							   (INTR_BASED_ADDR + 0x8)
#define INTR2_td_int_flag_clear                    INTR2_ADDR
#define INTR2_td_timeout_int_flag_clear            BIT(1)
#define INTR2_td_debug_frame_done_int_flag_clear   BIT(2)
#define INTR2_td_frame_start_scan_int_flag_clear   BIT(3)
#define INTR2_log_int_flag_clear                   BIT(4)
#define INTR2_d2t_crc_err_int_flag_clear           BIT(8)
#define INTR2_d2t_flash_req_int_flag_clear         BIT(9)
#define INTR2_d2t_ddi_int_flag_clear               BIT(10)
#define INTR2_wr_done_int_flag_clear               BIT(16)
#define INTR2_rd_done_int_flag_clear               BIT(17)
#define INTR2_tdi_err_int_flag_clear               BIT(18)
#define INTR2_d2t_slpout_rise_flag_clear           BIT(24)
#define INTR2_d2t_slpout_fall_flag_clear           BIT(25)
#define INTR2_d2t_dstby_flag_clear                 BIT(26)
#define INTR2_ddi_pwr_rdy_flag_clear               BIT(27)

#define INTR32_ADDR  						  (INTR_BASED_ADDR + 0x80)
#define INTR32_reg_ice_sw_int_en 			  INTR32_ADDR
#define INTR32_reg_ice_apb_conflict_int_en	  BIT(1)
#define INTR32_reg_ice_ilm_conflict_int_en	  BIT(2)
#define INTR32_reg_ice_dlm_conflict_int_en	  BIT(3)
#define INTR32_reserved_0 	  				  BIT(4)|BIT(5)|BIT(6)|BIT(7)
#define INTR32_reg_spi_sr_int_en			  BIT(8)
#define INTR32_reg_spi_sp_int_en			  BIT(9)
#define INTR32_reg_spi_trx_int_en			  BIT(10)
#define INTR32_reg_spi_cmd_int_en			  BIT(11)
#define INTR32_reg_spi_rw_int_en			  BIT(12)
#define INTR32_reserved_1					  BIT(13)|BIT(14)|BIT(15)
#define INTR32_reg_i2c_start_int_en			  BIT(16)
#define INTR32_reg_i2c_addr_match_int_en	  BIT(17)
#define INTR32_reg_i2c_cmd_int_en			  BIT(18)
#define INTR32_reg_i2c_sr_int_en			  BIT(19)
#define INTR32_reg_i2c_trx_int_en			  BIT(20)
#define INTR32_reg_i2c_rx_stop_int_en		  BIT(21)
#define INTR32_reg_i2c_tx_stop_int_en		  BIT(22)
#define INTR32_reserved_2					  BIT(23)
#define INTR32_reg_t0_int_en				  BIT(24)
#define INTR32_reg_t1_int_en				  BIT(25)
#define INTR32_reserved_3					  BIT(26)|BIT(27)|BIT(28)|BIT(29)|BIT(30)|BIT(31)

#define INTR33_ADDR							  (INTR_BASED_ADDR + 0x84)
#define INTR33_reg_uart_tx_int_en             INTR33_ADDR
#define INTR33_reserved_0                     BIT(1)|BIT(2)|BIT(3)|BIT(4)|BIT(5)|BIT(6)|BIT(7)
#define INTR33_reg_wdt_alarm_int_en           BIT(8)
#define INTR33_reserved_1                     BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15)
#define INTR33_reg_dma_ch1_int_en             BIT(16)
#define INTR33_reg_dma_ch0_int_en             BIT(17)
#define INTR33_reg_dma_frame_done_int_en      BIT(18)
#define INTR33_reg_dma_tdi_done_int_en        BIT(19)
#define INTR33_reserved_2                     BIT(20)|BIT(21)|BIT(22)|BIT(23)
#define INTR33_reg_flash_error_en             BIT(24)
#define INTR33_reg_flash_int_en               BIT(25) 
#define INTR33_reserved_3                     BIT(26)|BIT(27)|BIT(28)|BIT(29)|BIT(30)|BIT(31)

/* Flash */
#define FLASH_BASED_ADDR					0x41000
#define FLASH0_ADDR							(FLASH_BASED_ADDR + 0x0)
#define FLASH0_reg_flash_csb				FLASH0_ADDR
#define FLASH0_reserved_0					BIT(8)|BIT(9)|BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15)
#define FLASH0_reg_preclk_sel		        BIT(16)|BIT(17)|BIT(18)|BIT(19)|BIT(20)|BIT(21)|BIT(22)|BIT(23)
#define FLASH0_reg_rx_dual                  BIT(24)
#define FLASH0_reg_tx_dual                  BIT(25)
#define FLASH0_reserved_26                  BIT(26)|BIT(27)|BIT(28)|BIT(29)|BIT(30)|BIT(31)
#define FLASH1_ADDR							(FLASH_BASED_ADDR + 0x4)
#define FLASH1_reg_flash_key1               FLASH1_ADDR
#define FLASH1_reg_flash_key2               (FLASH1_ADDR + 0x01)
#define FLASH1_reg_flash_key3               (FLASH1_ADDR + 0x02)
#define FLASH1_reserved_0                   (FLASH1_ADDR + 0x03)
#define FLASH2_ADDR  						(FLASH_BASED_ADDR + 0x8)
#define FLASH2_reg_tx_data                  FLASH2_ADDR
#define FLASH3_ADDR  						(FLASH_BASED_ADDR + 0xC)
#define FLASH3_reg_rcv_cnt                  FLASH3_ADDR
#define FLASH4_ADDR  						(FLASH_BASED_ADDR + 0x10)
#define FLASH4_reg_rcv_data 				FLASH4_ADDR
#define FLASH4_reg_rcv_dly					BIT(8)
#define FLASH4_reg_sutrg_en					BIT(9)
#define FLASH4_reserved_1					BIT(10)|BIT(11)|BIT(12)|BIT(13)|BIT(14)|BIT(15)
#define FLASH4_reg_rcv_data_valid_state     BIT(16)
#define FLASH4_reg_flash_rd_finish_state	BIT(17);
#define FLASH4_reserved_2					BIT(18)|BIT(19)|BIT(20)|BIT(21)|BIT(22)|BIT(23)
#define FLASH4_reg_flash_dma_trigger_en		BIT(24)|BIT(25)|BIT(26)|BIT(27)|BIT(28)|BIT(29)|BIT(30)|BIT(31)

/* Protocol */
#define PROTOCOL_VER_500    			0x050000
#define PROTOCOL_VER_510    			0x050100
#define PROTOCOL_VER_520    			0x050200
#define PROTOCOL_VER_530    			0x050300
#define PROTOCOL_VER_540    			0x050400
#define PROTOCOL_VER_550    			0x050500
#define PROTOCOL_VER_560    			0x050600
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
#define P5_X_FW_UNKNOWN_MODE			0xFF
#define P5_X_FW_DEMO_MODE				0x00
#define P5_X_FW_TEST_MODE				0x01
#define P5_X_FW_DEBUG_MODE 				0x02
#define P5_X_FW_I2CUART_MODE			0x03
#define P5_X_FW_GESTURE_MODE			0x04
#define P5_X_DEMO_MODE_PACKET_LENGTH	43
#define P5_X_DEBUG_MODE_PACKET_LENGTH	1280
#define P5_X_TEST_MODE_PACKET_LENGTH	1180
#define P5_X_DEMO_PACKET_ID		        0x5A
#define P5_X_DEBUG_PACKET_ID	        0xA7
#define P5_X_TEST_PACKET_ID		        0xF2
#define P5_X_GESTURE_PACKET_ID	        0xAA
#define P5_X_I2CUART_PACKET_ID	        0x7A

/* Chipes */
#define ILI9881H_CHIP       			0x9881
#define ILI9881_H						0x11
#define ILI9881_F						0x0F
#define ILI9881H_AE_CHIP    			0x98811103
#define ILI7807G_CHIP       			0x7807
#define ILI7807G_AA_CHIP    			0x78071100
#define RAWDATA_NO_BK_SHIFT_9881H 		8192
#define RAWDATA_NO_BK_SHIFT_9881F 		4096

/* Path */
#define CSV_LCM_ON_PATH     "/sdcard/ilitek_mp_lcm_on_log"
#define CSV_LCM_OFF_PATH	"/sdcard/ilitek_mp_lcm_off_log"
#define INI_NAME_PATH		"/sdcard/mp.ini"
#define UPDATE_FW_PATH		"/sdcard/ILITEK_FW"

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

	struct task_struct *fw_boot_th;

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

	int fw_retry;
	int fw_update_stat;
	int fw_boot;
	int fw_open;

	u16 flash_mid;
	u16 flash_devid;
	int program_page;
	int flash_sector;

	/* host download */
	int reset_mode;
	int fw_upgrade_mode;

	atomic_t irq_stat;
	atomic_t tp_reset;
	atomic_t ice_stat;
	atomic_t fw_stat;
	atomic_t mp_stat;
	atomic_t tp_suspend;
	atomic_t tp_resume;
	atomic_t tp_sw_mode;
	atomic_t mp_int_check;

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
	u32 max_count;
	u32 reset_key;
	int no_bk_shift;
	s32 (*open_sp_formula)(int dac, int raw);
	s32 (*open_c_formula)(int dac, int raw, int tvch, int gain);
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

static inline s32 open_sp_formula_ili9881h(int dac, int raw)
{
	return (int)((int)(dac * 2 * 10000 * 161 / 100) - (int)(16384 / 2 - (int)raw) * 20000 * 7 / 16384 * 36 / 10) / 31 / 2;
}

static inline s32 open_sp_formula_ili7807g(int dac, int raw)
{
	return (int)((int)(dac * 2 * 10000 * 131 / 100) - (int)(16384 / 2 - (int)raw) * 20000 * 7 / 16384 * 36 / 10) / 31 / 2;
}

static inline s32 open_c_formula(int dac, int raw, int tvch, int gain)
{
	return (int)((int)(dac * 414 * 39 / 2) + (int)(((int)raw - 8192) * 36 * (7 * 100 - 22) * 10 / 16384)) /
						tvch / 100 / gain;
}

/* Prototypes for tddi firmware/flash functions */
extern void ilitek_tddi_flash_dma_write(struct ilitek_tddi_dev *, u32, u32, u32);
extern void ilitek_tddi_flash_clear_dma(struct ilitek_tddi_dev *);
extern void ilitek_tddi_fw_read_flash_info(struct ilitek_tddi_dev *, bool);
extern u32 ilitek_tddi_fw_read_hw_crc(struct ilitek_tddi_dev *, u32, u32);
extern int ilitek_tddi_fw_read_flash(struct ilitek_tddi_dev *, u32, u32, u8 *, size_t);
extern int ilitek_tddi_fw_upgrade(struct ilitek_tddi_dev *, int, int, int);

/* Prototypes for tddi mp test */
extern int ilitek_tddi_mp_test_main(struct ilitek_tddi_dev *, bool);
extern int ilitek_tddi_mp_move_code_flash(struct ilitek_tddi_dev *);
extern int ilitek_tddi_mp_move_code_iram(struct ilitek_tddi_dev *);

/* Prototypes for tddi core functions */
extern int ilitek_tddi_touch_switch_mode(struct ilitek_tddi_dev *, u8 *);
extern void ilitek_tddi_report_ap_mode(struct ilitek_tddi_dev *, u8 *);
extern void ilitek_tddi_report_debug_mode(struct ilitek_tddi_dev *);
extern void ilitek_tddi_report_gesture_mode(struct ilitek_tddi_dev *);
extern int ilitek_tddi_ic_whole_reset(struct ilitek_tddi_dev *);
extern int ilitek_tddi_ic_code_reset(struct ilitek_tddi_dev *);
extern int ilitek_tddi_ic_func_ctrl(struct ilitek_tddi_dev *, const char *, int);
extern u32 ilitek_tddi_ic_get_pc_counter(struct ilitek_tddi_dev *);
extern int ilitek_tddi_ic_check_int_stat(bool high);
extern int ilitek_tddi_ic_check_busy(struct ilitek_tddi_dev *, int, int);
extern int ilitek_tddi_ic_get_panel_info(struct ilitek_tddi_dev *);
extern int ilitek_tddi_ic_get_tp_info(struct ilitek_tddi_dev *);
extern int ilitek_tddi_ic_get_protocl_ver(struct ilitek_tddi_dev *);
extern int ilitek_tddi_ic_get_fw_ver(struct ilitek_tddi_dev *);
extern int ilitek_tddi_ic_get_info(struct ilitek_tddi_dev *);
extern int ilitek_ice_mode_bit_mask_write(struct ilitek_tddi_dev *, u32, u32, u32);
extern int ilitek_ice_mode_write(struct ilitek_tddi_dev *, u32 , u32 , size_t);
extern u32 ilitek_ice_mode_read(struct ilitek_tddi_dev *, u32, size_t);
extern int ilitek_ice_mode_ctrl(struct ilitek_tddi_dev *, bool, bool);
extern int ilitek_tddi_ic_init(struct ilitek_tddi_dev *);

/* Prototypes for tddi events */
extern int ilitek_tddi_fw_upgrade_handler(void *);
extern int ilitek_tddi_esd_handler(struct ilitek_tddi_dev *);
extern int ilitek_tddi_mp_test_handler(struct ilitek_tddi_dev *, bool);
extern void ilitek_tddi_report_handler(struct ilitek_tddi_dev *);
extern void ilitek_tddi_touch_suspend(struct ilitek_tddi_dev *);
extern void ilitek_tddi_touch_resume(struct ilitek_tddi_dev *);
extern int ilitek_tddi_reset_ctrl(struct ilitek_tddi_dev *, int);
extern int ilitek_tddi_init(struct ilitek_tddi_dev *);
extern int ilitek_tddi_dev_init(struct ilitek_hwif_info *);

/* Prototypes for i2c interface */
extern int ilitek_i2c_dev_init(struct ilitek_hwif_info *);

/* Prototypes for spi interface */
//extern int ilitek_spi_dev_init(struct ilitek_hwif_info *);

/* Prototypes for platform level */
extern void ilitek_plat_input_register(struct ilitek_tddi_dev *);
extern void ilitek_plat_irq_disable(struct ilitek_tddi_dev *);
extern void ilitek_plat_irq_enable(struct ilitek_tddi_dev *);
extern void ilitek_plat_tp_reset(struct ilitek_tddi_dev *);

/* Prototypes for miscs */
extern void ilitek_tddi_node_init(struct ilitek_tddi_dev *);
extern void ilitek_dump_data(void *, int, int, int, const char *);
extern u8 ilitek_calc_packet_checksum(u8 *, size_t);
extern int katoi(char *);
#endif /* __ILITEK_H */
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
/* Firmware data with static array */
#include "ilitek_fw.h"

#define UPDATE_PASS 0
#define UPDATE_FAIL -1
#define TIMEOUT_SECTOR	 500
#define TIMEOUT_PAGE	 3500

struct touch_fw_data {
	u8 block_number;
	u32 start_addr;
	u32 end_addr;
	u32 new_fw_cb;
	int delay_after_upgrade;
	bool isCRC;
	bool isboot;
	int hex_tag;
} tfd;

struct flash_block_info {
	char *name;
	u32 start;
	u32 end;
	u32 len;
	u32 mem_start;
	u32 fix_mem_start;
	u8 mode;
} fbi[FW_BLOCK_INFO_NUM];

u8 gestrue_fw[(10 * K)];

static u32 HexToDec(char *phex, s32 len)
{
	u32 ret = 0, temp = 0, i;
	s32 shift = (len - 1) * 4;

	for (i = 0; i < len; shift -= 4, i++) {
		if ((phex[i] >= '0') && (phex[i] <= '9')) {
			temp = phex[i] - '0';
		} else if ((phex[i] >= 'a') && (phex[i] <= 'f')) {
			temp = (phex[i] - 'a') + 10;
		} else if ((phex[i] >= 'A') && (phex[i] <= 'F')) {
			temp = (phex[i] - 'A') + 10;
		} else {
			return -1;
		}

		ret |= (temp << shift);
	}

	return ret;
}

static u32 CalculateCRC32(u32 start_addr, u32 len, u8 *pfw)
{
	u32 i = 0, j = 0;
	u32 crc_poly = 0x04C11DB7;
	u32 tmp_crc = 0xFFFFFFFF;

	for (i = start_addr; i < start_addr + len; i++) {
		tmp_crc ^= (pfw[i] << 24);

		for (j = 0; j < 8; j++) {
			if ((tmp_crc & 0x80000000) != 0)
				tmp_crc = tmp_crc << 1 ^ crc_poly;
			else
				tmp_crc = tmp_crc << 1;
		}
	}
	return tmp_crc;
}

static int ilitek_tddi_fw_check_hex_hw_crc(u8 *pfw)
{
	u32 i = 0, len = 0;
	u32 hex_crc = 0, hw_crc;

	for (i = 0; i < ARRAY_SIZE(fbi); i++) {
		if (fbi[i].end == 0)
			continue;

		len = fbi[i].end - fbi[i].start + 1 - 4;

		hex_crc = CalculateCRC32(fbi[i].start, len, pfw);
		hw_crc = ilitek_tddi_fw_read_hw_crc(fbi[i].start, len);

		ipio_info("Block = %d, Hex CRC = %x, HW CRC = %x\n", i, hex_crc, hw_crc);

		if(!CHECK_EQUAL(hex_crc, hw_crc)) {
			ipio_info("Hex and HW CRC are incorrect!\n");
			return UPDATE_FAIL;
		}
	}

	ipio_info("Hex and HW CRC are correct !\n");
	return UPDATE_PASS;
}

static int ilitek_tddi_flash_poll_busy(int timer)
{
	int ret = UPDATE_PASS, retry = timer;
	u8 cmd = 0x5, temp = 0;

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write(FLASH2_ADDR, cmd, 1);

	do {
		ilitek_ice_mode_write(FLASH2_ADDR, 0xFF, 1); /* Dummy */
		mdelay(1);
		temp = ilitek_ice_mode_read(FLASH4_ADDR, sizeof(u8));
		if ((temp & 0x3) == 0)
			break;
	} while (--retry >= 0);

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

	if (retry <= 0) {
		ipio_err("Flash polling busy timeout ! tmp = %x\n", temp);
		ret = UPDATE_FAIL;
	}

	return ret;
}

void ilitek_tddi_flash_clear_dma(void)
{
	ilitek_ice_mode_bit_mask_write(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_bit_mask_write(FLASH0_ADDR, FLASH0_reg_preclk_sel, (2 << 16));
	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x01, 1);	/* CS high */

	ilitek_ice_mode_bit_mask_write(FLASH4_ADDR, FLASH4_reg_flash_dma_trigger_en, (0 << 24));
	ilitek_ice_mode_bit_mask_write(FLASH0_ADDR, FLASH0_reg_rx_dual, (0 << 24));

	ilitek_ice_mode_write(FLASH3_reg_rcv_cnt, 0x00, 1);
	ilitek_ice_mode_write(FLASH4_reg_rcv_data, 0xFF, 1);
}

void ilitek_tddi_flash_dma_write(u32 start, u32 end, u32 len)
{
	ilitek_ice_mode_bit_mask_write(FLASH0_ADDR, FLASH0_reg_preclk_sel, 1 << 16);

	ilitek_ice_mode_write(FLASH0_reg_flash_csb, 0x00, 1);	/* CS low */
	ilitek_ice_mode_write(FLASH1_reg_flash_key1, 0x66aa55, 3);	/* Key */

	ilitek_ice_mode_write(FLASH2_reg_tx_data, 0x0b, 1);
	while(!(ilitek_ice_mode_read(INTR1_ADDR & BIT(25), sizeof(u32))));
	ilitek_ice_mode_bit_mask_write(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_write(FLASH2_reg_tx_data, (start & 0xFF0000) >> 16, 1);
	while(!(ilitek_ice_mode_read(INTR1_ADDR & BIT(25), sizeof(u32))));
	ilitek_ice_mode_bit_mask_write(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_write(FLASH2_reg_tx_data, (start & 0x00FF00) >> 8, 1);
	while(!(ilitek_ice_mode_read(INTR1_ADDR & BIT(25), sizeof(u32))));
	ilitek_ice_mode_bit_mask_write(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_write(FLASH2_reg_tx_data, (start & 0x0000FF), 1);
	while(!(ilitek_ice_mode_read(INTR1_ADDR & BIT(25), sizeof(u32))));
	ilitek_ice_mode_bit_mask_write(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_bit_mask_write(FLASH0_ADDR, FLASH0_reg_rx_dual, 0 << 24);

	ilitek_ice_mode_write(FLASH2_reg_tx_data, 0x00, 1);	/* Dummy */
	while(!(ilitek_ice_mode_read(INTR1_ADDR & BIT(25), sizeof(u32))));
	ilitek_ice_mode_bit_mask_write(INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ilitek_ice_mode_write(FLASH3_reg_rcv_cnt, len, 4);	/* Write Length */
}

static void ilitek_tddi_flash_write_enable(void)
{
	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write(FLASH2_ADDR, 0x6, 1);
	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */
}

u32 ilitek_tddi_fw_read_hw_crc(u32 start, u32 end)
{
	int retry = 500;
	u8 busy = 0;
	u32 write_len = end;
	u32 iram_check = 0;

	if (write_len > idev->chip->max_count) {
		ipio_err("The length (%x) written into firmware is greater than max count (%x)\n",
			write_len, idev->chip->max_count);
		return -1;
	}

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write(FLASH2_ADDR, 0x3b, 1);
	ilitek_ice_mode_write(FLASH2_ADDR, (start & 0xFF0000) >> 16, 1);
	ilitek_ice_mode_write(FLASH2_ADDR, (start & 0x00FF00) >> 8, 1);
	ilitek_ice_mode_write(FLASH2_ADDR, (start & 0x0000FF), 1);
	ilitek_ice_mode_write(0x041003, 0x01, 1); /* Enable Dio_Rx_dual */
	ilitek_ice_mode_write(FLASH2_ADDR, 0xFF, 1); /* Dummy */
	ilitek_ice_mode_write(0x04100C, write_len, 3); /* Set Receive count */
	ilitek_ice_mode_write(0x048007, 0x02, 1);/* Clear Int Flag */
	ilitek_ice_mode_write(0x041016, 0x00, 1);
	ilitek_ice_mode_write(0x041016, 0x01, 1);	/* Checksum_En */

	ilitek_ice_mode_write(FLASH4_ADDR, 0xFF, 1); /* Start to receive */

	do {
		busy = ilitek_ice_mode_read(0x048007, sizeof(u8));
		if (((busy >> 1) & 0x01) == 0x01)
			break;
	} while (--retry >= 0);

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

	if (retry <= 0) {
		ipio_err("Read HW CRC timeout !, busy = 0x%x\n", busy);
		return -1;
	}

	ilitek_ice_mode_write(0x041003, 0x0, 1); /* Disable dio_Rx_dual */
	iram_check = ilitek_ice_mode_read(0x04101C, sizeof(u32));
	return iram_check;
}

int ilitek_tddi_fw_read_flash_data(u32 start, u32 end,
								u8 *data, size_t len)
{
	u32 i, index = 0;

	if (end - start > len) {
		ipio_err("the length (%d) reading crc is over than len(%d)\n", end - start, (int)len);
		return -1;
	}

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write(FLASH2_ADDR, 0x03, 1);
	ilitek_ice_mode_write(FLASH2_ADDR, (start & 0xFF0000) >> 16, 1);
	ilitek_ice_mode_write(FLASH2_ADDR, (start & 0x00FF00) >> 8, 1);
	ilitek_ice_mode_write(FLASH2_ADDR, (start & 0x0000FF), 1);

	for (i = start; i <= end; i++) {
		ilitek_ice_mode_write(FLASH2_ADDR, 0xFF, 1); /* Dummy */
		data[index] = ilitek_ice_mode_read(FLASH4_ADDR, sizeof(u8));
		//ipio_info("flash_data[%d] = %x\n", index, data[index]);
		index++;
	}

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */
	return 0;
}

static void ilitek_tddi_flash_protect(bool enable)
{
	ipio_info("%s flash protection\n", enable ? "Enable" : "Disable");

	ilitek_tddi_flash_write_enable();

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write(FLASH2_ADDR, 0x1, 1);
	ilitek_ice_mode_write(FLASH2_ADDR, 0x0, 1);

	switch (idev->flash_mid) {
	case 0xEF:
		if (idev->flash_devid == 0x6012 || idev->flash_devid == 0x6011) {
			if (enable)
				ilitek_ice_mode_write(FLASH2_ADDR, 0x7E, 1);
			else
				ilitek_ice_mode_write(FLASH2_ADDR, 0x0, 1);
		}
		break;
	case 0xC8:
		if (idev->flash_devid == 0x6012 || idev->flash_devid == 0x6013) {
			if (enable)
				ilitek_ice_mode_write(FLASH2_ADDR, 0x7A, 1);
			else
				ilitek_ice_mode_write(FLASH2_ADDR, 0x0, 1);
		}
		break;
	default:
		ipio_err("Can't find flash id(0x%x), ignore protection\n", idev->flash_mid);
		break;
	}

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */
}

static int ilitek_tddi_fw_check_ver(u8 *pfw)
{
	int i, crc_byte_len = 4;
	u8 flash_crc[4] = {0};
	u32 start_addr = 0, end_addr = 0;
	u32 block_crc, flash_crc_cb;

	/* Check FW version */
	ipio_info("New FW ver = 0x%x, Current FW ver = 0x%x\n", tfd.new_fw_cb, idev->chip->fw_ver);
	if (tfd.new_fw_cb != idev->chip->fw_ver) {
		ipio_info("FW version is different, do upgrade\n");
		return UPDATE_FAIL;
	}

	ipio_info("FW version is the same, check Flash and HW CRC if there's corruption.\n");

	/* Check Flash and HW CRC with last 4 bytes in each block */
	for (i = 0; i < ARRAY_SIZE(fbi); i++) {
		start_addr = fbi[i].start;
		end_addr = fbi[i].end;

		/* Invaild end address */
		if (end_addr == 0)
			continue;

		if (ilitek_tddi_fw_read_flash_data(end_addr - crc_byte_len + 1, end_addr,
					flash_crc, sizeof(flash_crc)) < 0) {
			ipio_err("Read Flash failed\n");
			return UPDATE_FAIL;
		}

		flash_crc_cb = flash_crc[0] << 24 | flash_crc[1] << 16 | flash_crc[2] << 8 | flash_crc[3];

		block_crc = ilitek_tddi_fw_read_hw_crc(start_addr, end_addr - start_addr - crc_byte_len + 1);

		ipio_info("Block = %d, HW CRC = 0x%06x, Flash CRC = 0x%06x\n", i, block_crc, flash_crc_cb);

		/* Compare Flash CRC with HW CRC */
		if (flash_crc_cb != block_crc) {
			ipio_info("Both are different, do update\n");
			return UPDATE_FAIL;
		}
		memset(flash_crc, 0, sizeof(flash_crc));
	}

	ipio_info("Both are the same, no need to update\n");
	return UPDATE_PASS;
}

static int ilitek_tddi_fw_iram_upgrade(u8 *pfw)
{
	//  int ret = UPDATE_OK;
	int ret = 0;
	// uint32_t mode, crc, dma;
	u8 *fw_ptr = NULL;

	ipio_info();

	/* Reset before load AP and MP code*/
	// if (!core_gesture->entry) summer
	// 	ilitek_platform_tp_hw_reset(true);

    ret = ilitek_ice_mode_ctrl(ENABLE, OFF);
	if (ret < 0)
		return ret;

	if (ilitek_set_watch_dog(false) < 0) {
		ipio_err("Failed to disable watch dog\n");
		ret = -EINVAL;
		goto out;
	}

	fw_ptr = pfw;
	// if (core_fr->actual_fw_mode == protocol->test_mode) {
	// 	mode = MP;
	// } else if (core_gesture->entry) {
	// 	mode = GESTURE;
	// 	fw_ptr = gestrue_fw;
	// } else {
	// 	mode = AP;
	// }

	// /* Program data to iram acorrding to each block */
	// for (i = 0; i < ARRAY_SIZE(fbi); i++) {
	// 	if (fbi[i].mode == mode && fbi[i].len != 0) {
	// 		ipio_info("Download %s code from hex 0x%x to IRAN 0x%x len = 0x%x\n", fbi[i].name, fbi[i].start, fbi[i].mem_start, fbi[i].len);
	// 		write_download(fbi[i].mem_start, fbi[i].len, (fw_ptr + fbi[i].start) , SPI_UPGRADE_LEN);

	// 		crc = calc_crc32(fbi[i].start, fbi[i].len - 4 , fw_ptr);
	// 		dma = host_download_dma_check(fbi[i].mem_start, fbi[i].len - 4);

	// 		ipio_info("%s CRC is %s (%x) : (%x)\n",fbi[i].name, (crc != dma ? "Invalid !" : "Correct !"), crc, dma);

	// 		if (CHECK_EQUAL(crc, dma) == UPDATE_FAIL)
	// 			ret = UPDATE_FAIL;
	// 	}
	// }

out:
	// if (!core_gesture->entry) {
	// 	/* ice mode code reset */
	// 	ipio_info("Doing code reset ...\n");
	// 	ilitek_ice_mode_write(0x40040, 0xAE, 1);
	// }

	ilitek_ice_mode_ctrl(DISABLE, OFF);
	mdelay(10);
	return ret;
}

static int ilitek_tddi_fw_do_program(u8 *pfw)
{
	u8 buf[512] = {0};
	u32 i = 0, addr = 0, k = 0, recv_addr = 0;
	bool skip = true;

	for (i = 0; i < FW_BLOCK_INFO_NUM; i++) {
		if (fbi[i].end == 0)
			continue;

		if (fbi[i].start >= RESERVE_BLOCK_START_ADDR &&
			fbi[i].end <= RESERVE_BLOCK_END_ADDR)
			continue;

		ipio_info("Block[%d]: Programing from (0x%x) to (0x%x)\n", i, fbi[i].start, fbi[i].end);

		for (addr = fbi[i].start; addr < fbi[i].end; addr += idev->program_page) {
			buf[0] = 0x25;
			buf[3] = 0x04;
			buf[2] = 0x10;
			buf[1] = 0x08;

			for (k = 0; k < idev->program_page; k++) {
				if (addr + k <= tfd.end_addr)
					buf[4 + k] = pfw[addr + k];
				else
					buf[4 + k] = 0xFF;

				if (buf[4 + k] != 0xFF)
					skip = false;
			}

			if (skip) {
				ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */
				return UPDATE_FAIL;
			}

			ilitek_tddi_flash_write_enable();

			ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
			ilitek_ice_mode_write(FLASH1_ADDR, 0x66aa55, 3); /* Key */
			ilitek_ice_mode_write(FLASH2_ADDR, 0x2, 1);
			recv_addr = ((addr & 0xFF0000) >> 16) | (addr & 0x00FF00) | ((addr & 0x0000FF) << 16);
			ilitek_ice_mode_write(FLASH2_ADDR, recv_addr, 3);

			if (idev->write(buf, idev->program_page + 4) < 0) {
				ipio_err("Failed to program data at start_addr = 0x%X, k = 0x%X, addr = 0x%x\n",
				addr, k, addr + k);
				ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */
				return UPDATE_FAIL;
			}

			ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

			if (idev->flash_mid == 0x85)
				mdelay(2);
			else
				mdelay(1);

			idev->fw_update_stat = (addr * 101) / tfd.end_addr;

			/* holding the status until finish this upgrade. */
			if (idev->fw_update_stat > 90)
				idev->fw_update_stat = 90;
		}
	}
	return UPDATE_PASS;
}

static int ilitek_tddi_fw_do_erase(void)
{
	int ret = 0;
	u32 i = 0, addr = 0, recv_addr = 0;

	for (i = 0; i < FW_BLOCK_INFO_NUM; i++) {
		if (fbi[i].end == 0)
			continue;

		if (fbi[i].start >= RESERVE_BLOCK_START_ADDR &&
			fbi[i].end <= RESERVE_BLOCK_END_ADDR)
			continue;

		ipio_info("Block[%d]: Erasing from (0x%x) to (0x%x) \n", i, fbi[i].start, fbi[i].end);

		for(addr = fbi[i].start; addr <= fbi[i].end; addr += idev->flash_sector) {
			ilitek_tddi_flash_write_enable();

			ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
			ilitek_ice_mode_write(FLASH1_ADDR, 0x66aa55, 3); /* Key */

			if (CHECK_EQUAL(addr, fbi[AP].start))
				ilitek_ice_mode_write(FLASH2_ADDR, 0xD8, 1);
			else
				ilitek_ice_mode_write(FLASH2_ADDR, 0x20, 1);

			recv_addr = ((addr & 0xFF0000) >> 16) | (addr & 0x00FF00) | ((addr & 0x0000FF) << 16);
			ilitek_ice_mode_write(FLASH2_ADDR, recv_addr, 3);

			ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

			mdelay(1);

			if (CHECK_EQUAL(addr, fbi[AP].start))
				ret = ilitek_tddi_flash_poll_busy(TIMEOUT_PAGE);
			else
				ret = ilitek_tddi_flash_poll_busy(TIMEOUT_SECTOR);

			if (ret < 0)
				return UPDATE_FAIL;

			ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

			if (fbi[i].start == fbi[AP].start)
				break;
		}
	}
	return UPDATE_PASS;
}

static int ilitek_tddi_fw_flash_upgrade(u8 *pfw)
{
	int ret = UPDATE_PASS;

	ilitek_tddi_reset_ctrl(idev->reset_mode);

	ret = ilitek_ice_mode_ctrl(ENABLE, OFF);
	if (ret < 0)
		return UPDATE_FAIL;

	ret = ilitek_tddi_fw_check_ver(pfw);
	if (ret == UPDATE_PASS)
		goto out;

	ret = ilitek_tddi_fw_do_erase();
	if (ret == UPDATE_FAIL)
		goto out;

	mdelay(1);

	ret = ilitek_tddi_fw_do_program(pfw);
	if (ret == UPDATE_FAIL)
		goto out;

	/* We do have to reset chip in order to move new code from flash to iram. */
	ilitek_tddi_reset_ctrl(idev->reset_mode);

	/* the delay time moving code depends on what the touch IC you're using. */
	mdelay(200);

	ret = ilitek_ice_mode_ctrl(ENABLE, OFF);
	if (ret < 0)
		goto out;

	ret = ilitek_tddi_fw_check_hex_hw_crc(pfw);

out:
	ilitek_ice_mode_ctrl(DISABLE, OFF);
	return ret;
}

static void ilitek_tddi_fw_update_block_info(u8 *pfw, u8 type)
{
	u32 ges_info_addr, ges_fw_start, ges_fw_end;

    ipio_info("Upgarde = %s, Tag = %x\n", type ? "IRAM" : "Flash", tfd.hex_tag);

	if (type == UPGRADE_IRAM) {
		if (tfd.hex_tag == BLOCK_TAG_AF) {
			fbi[AP].mem_start = (fbi[AP].fix_mem_start != INT_MAX) ? fbi[AP].fix_mem_start : 0;
			fbi[DATA].mem_start = (fbi[DATA].fix_mem_start != INT_MAX) ? fbi[DATA].fix_mem_start : DLM_START_ADDRESS;
			fbi[TUNING].mem_start = (fbi[TUNING].fix_mem_start != INT_MAX) ? fbi[TUNING].fix_mem_start :  fbi[DATA].mem_start + fbi[DATA].len;
			fbi[MP].mem_start = (fbi[MP].fix_mem_start != INT_MAX) ? fbi[MP].fix_mem_start :  0;
			fbi[GESTURE].mem_start = (fbi[GESTURE].fix_mem_start != INT_MAX) ? fbi[GESTURE].fix_mem_start :  0;

			/* Parsing gesture info form AP code */
			ges_info_addr = (fbi[AP].end + 1 - 60);
			//core_gesture->area_section = (pfw[ges_info_addr + 3] << 24) + (pfw[ges_info_addr + 2] << 16) + (pfw[ges_info_addr + 1] << 8) + pfw[ges_info_addr];
			fbi[GESTURE].mem_start = (pfw[ges_info_addr + 7] << 24) + (pfw[ges_info_addr + 6] << 16) + (pfw[ges_info_addr + 5] << 8) + pfw[ges_info_addr + 4];
			fbi[GESTURE].len = MAX_GESTURE_FIRMWARE_SIZE;
			ges_fw_start = (pfw[ges_info_addr + 15] << 24) + (pfw[ges_info_addr + 14] << 16) + (pfw[ges_info_addr + 13] << 8) + pfw[ges_info_addr + 12];
			ges_fw_end = fbi[GESTURE].end;
			fbi[GESTURE].start = 0;
		} else {
			fbi[AP].start = 0;
			fbi[AP].mem_start = 0;
			fbi[AP].len = MAX_AP_FIRMWARE_SIZE;

			fbi[DATA].start = DLM_HEX_ADDRESS;
			fbi[DATA].mem_start = DLM_START_ADDRESS;
			fbi[DATA].len = MAX_DLM_FIRMWARE_SIZE;

			fbi[MP].start = MP_HEX_ADDRESS;
			fbi[MP].mem_start = 0;
			fbi[MP].len = MAX_MP_FIRMWARE_SIZE;

			//core_gesture->area_section = (pfw[0xFFCF] << 24) + (pfw[0xFFCE] << 16) + (pfw[0xFFCD] << 8) + pfw[0xFFCC];;
			fbi[GESTURE].mem_start = (pfw[0xFFD3] << 24) + (pfw[0xFFD2] << 16) + (pfw[0xFFD1] << 8) + pfw[0xFFD0];;
			fbi[GESTURE].len = MAX_GESTURE_FIRMWARE_SIZE;
			ges_fw_start = (pfw[0xFFDB] << 24) + (pfw[0xFFDA] << 16) + (pfw[0xFFD9] << 8) + pfw[0xFFD8];
			ges_fw_end = (pfw[0xFFDB] << 24) + (pfw[0xFFDA] << 16) + (pfw[0xFFD9] << 8) + pfw[0xFFD8];
			fbi[GESTURE].start = 0;
		}

		memset(gestrue_fw, 0xff, sizeof(gestrue_fw));

#ifdef GESTURE_ENABLE
		/* Parsing gesture info and code */
		if (fbi[GESTURE].mem_start != 0xffffffff && ges_fw_start != 0xffffffff && fbi[GESTURE].mem_start != 0 && ges_fw_start != 0)
			ipio_memcpy(gestrue_fw, (pfw + ges_fw_start), fbi[GESTURE].len, sizeof(gestrue_fw));
		else
			ipio_err("There is no gesture data inside fw\n");

		core_gesture->ap_length = MAX_GESTURE_FIRMWARE_SIZE;

		ipio_info("GESTURE memory start = 0x%x, upgrade lenth = 0x%x",
					fbi[GESTURE].mem_start, MAX_GESTURE_FIRMWARE_SIZE);
		ipio_info("hex area = %d, ap_start_addr = 0x%x, ap_end_addr = 0x%x\n",
					core_gesture->area_section, ges_fw_start, ges_fw_end);
#endif
		fbi[AP].name ="AP";
		fbi[DATA].name ="DATA";
		fbi[TUNING].name ="TUNING";
		fbi[MP].name ="MP";
		fbi[GESTURE].name ="GESTURE";

		/* upgrade mode define */
		fbi[DATA].mode = fbi[AP].mode = fbi[TUNING].mode = AP;
		fbi[MP].mode = MP;
		fbi[GESTURE].mode = GESTURE;
	}

	/* Get hex fw vers */
	tfd.new_fw_cb = (pfw[FW_VER_ADDR] << 24) | (pfw[FW_VER_ADDR + 1] << 16) |
			(pfw[FW_VER_ADDR + 2] << 8) | (pfw[FW_VER_ADDR + 3]);

	/* Calculate update adress  */
	ipio_info("New FW ver = 0x%x\n", tfd.new_fw_cb);
	ipio_info("star_addr = 0x%06X, end_addr = 0x%06X, Block Num = %d\n", tfd.start_addr, tfd.end_addr, tfd.block_number);
}

static void ilitek_tdd_fw_ili_convert(u8 *pfw)
{
	int i = 0, block_enable = 0, num = 0;

	ipio_info("Start to parser ILI file, type = %d, block_count = %d\n", CTPM_FW[32], CTPM_FW[33]);

	memset(fbi, 0x0, sizeof(struct flash_block_info) * FW_BLOCK_INFO_NUM);

	tfd.start_addr = 0;
	tfd.end_addr = 0;
	tfd.hex_tag = 0;

	block_enable = CTPM_FW[32];

	if (block_enable == 0) {
		tfd.hex_tag = BLOCK_TAG_AE;
		goto out;
	}

	tfd.hex_tag = BLOCK_TAG_AF;
	for (i = 0; i < FW_BLOCK_INFO_NUM; i++) {
		if (((block_enable >> i) & 0x01) == 0x01) {
			num = i + 1;
			if ((num) == 6) {
				fbi[num].start = (CTPM_FW[0] << 16) + (CTPM_FW[1] << 8) + (CTPM_FW[2]);
				fbi[num].end = (CTPM_FW[3] << 16) + (CTPM_FW[4] << 8) + (CTPM_FW[5]);
				fbi[num].fix_mem_start = INT_MAX;
			} else {
				fbi[num].start = (CTPM_FW[34 + i * 6] << 16) + (CTPM_FW[35 + i * 6] << 8) + (CTPM_FW[36 + i * 6]);
				fbi[num].end = (CTPM_FW[37 + i * 6] << 16) + (CTPM_FW[38 + i * 6] << 8) + (CTPM_FW[39 + i * 6]);
				fbi[num].fix_mem_start = INT_MAX;
			}
			fbi[num].len = fbi[num].end - fbi[num].start + 1;
			ipio_info("Block[%d]: start_addr = %x, end = %x\n",
					num, fbi[num].start, fbi[num].end);
		}
	}

out:
	tfd.block_number = CTPM_FW[33];

	memcpy(pfw, CTPM_FW + ILI_FILE_HEADER, (sizeof(CTPM_FW) - ILI_FILE_HEADER));
	tfd.end_addr = (sizeof(CTPM_FW) - ILI_FILE_HEADER);
}

static int ilitek_tddi_fw_hex_convert(u8 *phex, size_t size, u8 *pfw)
{
	int block = 0;
	u32 i = 0, j = 0, k = 0, num = 0;
	u32 len = 0, addr = 0, type = 0;
	u32 start_addr = 0x0, end_addr = 0x0, ex_addr = 0;
	u32 offset;

	memset(fbi, 0x0, sizeof(struct flash_block_info) * FW_BLOCK_INFO_NUM);

	/* Parsing HEX file */
	for (; i < size;) {
		len = HexToDec(&phex[i + 1], 2);
		addr = HexToDec(&phex[i + 3], 4);
		type = HexToDec(&phex[i + 7], 2);

		if (type == 0x04) {
			ex_addr = HexToDec(&phex[i + 9], 4);
		} else if (type == 0x02) {
			ex_addr = HexToDec(&phex[i + 9], 4);
			ex_addr = ex_addr >> 12;
		} else if (type == BLOCK_TAG_AE || type == BLOCK_TAG_AF) {
			/* insert block info extracted from hex */
			tfd.hex_tag = type;
			if (tfd.hex_tag == BLOCK_TAG_AF)
				num = HexToDec(&phex[i + 9 + 6 + 6], 2);
			else
				num = block;

			fbi[num].start = HexToDec(&phex[i + 9], 6);
			fbi[num].end = HexToDec(&phex[i + 9 + 6], 6);
			fbi[num].fix_mem_start = INT_MAX;
			fbi[num].len = fbi[num].end - fbi[num].start + 1;
			ipio_info("Block[%d]: start_addr = %x, end = %x", num, fbi[num].start, fbi[num].end);

			block++;
		} else if (type == BLOCK_TAG_B0 && tfd.hex_tag == BLOCK_TAG_AF) {
			num = HexToDec(&phex[i + 9 + 6], 2);
			fbi[num].fix_mem_start = HexToDec(&phex[i + 9], 6);
			ipio_info("number = 0x%x, fix_mem_start = 0x%x",num, fbi[num].fix_mem_start);
		}

		addr = addr + (ex_addr << 16);

		if (phex[i + 1 + 2 + 4 + 2 + (len * 2) + 2] == 0x0D)
			offset = 2;
		else
			offset = 1;

		if (addr > MAX_HEX_FILE_SIZE) {
			ipio_err("Invalid hex format %d\n", addr);
			return -1;
		}

		if (type == 0x00) {
			end_addr = addr + len;
			if (addr < start_addr)
				start_addr = addr;
			/* fill data */
			for (j = 0, k = 0; j < (len * 2); j += 2, k++)
				pfw[addr + k] = HexToDec(&phex[i + 9 + j], 2);
		}
		i += 1 + 2 + 4 + 2 + (len * 2) + 2 + offset;
	}

	tfd.start_addr = start_addr;
	tfd.end_addr = end_addr;
	tfd.block_number = block;
	return 0;
}

static int ilitek_tdd_fw_hex_open(u8 open_file_method, u8 *pfw)
{
	int fsize = 1;
	u8 *hex_buffer = NULL;
	const struct firmware *fw = NULL;
	struct file *f = NULL;
	mm_segment_t old_fs;
	loff_t pos = 0;

    ipio_info("Open file method = %s, path = %s\n",
		open_file_method ? "FILP_OPEN" : "REQUEST_FIRMWARE", UPDATE_FW_PATH);

	switch (open_file_method) {
		case REQUEST_FIRMWARE:
            if (request_firmware(&fw, UPDATE_FW_PATH, idev->dev) < 0) {
                ipio_err("Rquest firmware failed\n");
                return -ENOMEM;
            }

            fsize = fw->size;
            ipio_info("fsize = %d\n", fsize);
            if (fsize <= 0) {
                ipio_err("The size of file is zero\n");
                release_firmware(fw);
                return -ENOMEM;
            }

			hex_buffer = vmalloc(fsize * sizeof(u8));
			if (ERR_ALLOC_MEM(hex_buffer)) {
				ipio_err("Failed to allocate hex_buffer memory, %ld\n", PTR_ERR(hex_buffer));
				release_firmware(fw);
				return -ENOMEM;
			}

			ipio_memcpy(hex_buffer, fw->data, fsize * sizeof(*fw->data), fsize);
			release_firmware(fw);
            break;
        case FILP_OPEN:
            f = filp_open(UPDATE_FW_PATH, O_RDONLY, 0644);
            if (ERR_ALLOC_MEM(f)) {
                ipio_err("Failed to open the file at %ld.\n", PTR_ERR(f));
                return -ENOMEM;
            }

            fsize = f->f_inode->i_size;
            ipio_info("fsize = %d\n", fsize);
            if (fsize <= 0) {
                ipio_err("The size of file is invaild\n");
                filp_close(f, NULL);
                return -ENOMEM;
            }

            hex_buffer = vmalloc(fsize * sizeof(u8));
            if (ERR_ALLOC_MEM(hex_buffer)) {
                ipio_err("Failed to allocate hex_buffer memory, %ld\n", PTR_ERR(hex_buffer));
                filp_close(f, NULL);
                return -ENOMEM;
            }

            /* ready to map user's memory to obtain data by reading files */
            old_fs = get_fs();
            set_fs(get_ds());
            set_fs(KERNEL_DS);
            pos = 0;
            vfs_read(f, hex_buffer, fsize, &pos);
            set_fs(old_fs);
            filp_close(f, NULL);
            break;
        default:
            ipio_err("Unknown open file method, %d\n", open_file_method);
            break;
    }

	/* Convert hex and copy data from hex_buffer to pfw */
	ilitek_tddi_fw_hex_convert(hex_buffer, fsize, pfw);

	ipio_vfree((void **)&hex_buffer);
    return 0;
}

static void ilitek_tddi_fw_file_handle(int target, u8 *pfw, int open_file_method)
{
	ipio_info("Convert fw data from %s\n", (target == ILI_FILE ? "ILI_FILE" : "HEX_FILE"));

    if (target == HEX_FILE) {
        /* Feed ili file if can't find hex file from filesystem. */
        if (ilitek_tdd_fw_hex_open(open_file_method, pfw) < 0) {
            ipio_err("Open hex file fail, try ili file upgrade");
            ilitek_tdd_fw_ili_convert(pfw);
        }
		return;
    }

    ilitek_tdd_fw_ili_convert(pfw);
}

static void ilitek_tddi_fw_update_tp_info(int ret)
{
	ipio_info("FW upgrade %s\n", (ret == UPDATE_PASS ? "PASS" : "FAIL"));

	if (ret == UPDATE_FAIL) {
		ipio_info("Erase all fw data\n");
		ilitek_ice_mode_ctrl(ENABLE, OFF);
		ilitek_tddi_fw_do_erase();
		ilitek_ice_mode_ctrl(DISABLE, OFF);
		ilitek_tddi_reset_ctrl(TP_RST_HW_ONLY);
	}
	ilitek_tddi_ic_get_protocl_ver();
	ilitek_tddi_ic_get_fw_ver();
	ilitek_tddi_ic_get_tp_info();
	ilitek_tddi_ic_get_panel_info();
	ilitek_plat_input_register();
}

int ilitek_tddi_fw_upgrade(int upgrade_type, int file_type, int open_file_method)
{
    int ret = 0, retry = 3;
    u8 *pfw = NULL;

	idev->fw_update_stat = 0;

	pfw = vmalloc(MAX_HEX_FILE_SIZE * sizeof(u8));
	if (ERR_ALLOC_MEM(pfw)) {
		ipio_err("Failed to allocate pfw memory, %ld\n", PTR_ERR(pfw));
		ret = -ENOMEM;
		goto out;
	}

	memset(pfw, 0xFF, MAX_HEX_FILE_SIZE * sizeof(u8));

    ilitek_tddi_fw_file_handle(file_type, pfw, open_file_method);
	ilitek_tddi_fw_update_block_info(pfw, upgrade_type);

	/* Get firmware version from chip for comparison in after */
	ilitek_tddi_ic_get_protocl_ver();
	ilitek_tddi_ic_get_fw_ver();

	do {
		if (upgrade_type == UPGRADE_FLASH)
			ret = ilitek_tddi_fw_flash_upgrade(pfw);
		else
			ret = ilitek_tddi_fw_iram_upgrade(pfw);

		if (ret == 0)
			break;
		idev->fw_update_stat = 0;
	} while(--retry >= 0);

	if (retry <= 0) {
		ipio_err("Upgrade firmware failed after retry 3 times\n");
		ret = UPDATE_FAIL;
	}

	ilitek_tddi_fw_update_tp_info(ret);

out:
	if (ret == UPDATE_PASS)
		idev->fw_update_stat = 100;
	else
		idev->fw_update_stat = UPDATE_FAIL;
	ipio_vfree((void **)&pfw);
    return ret;
}

struct flash_table {
	u16 mid;
	u16 dev_id;
	int mem_size;
	int program_page;
	int sector;
} flashtab[] = {
	[0] = {0x00, 0x0000, (256 * K), 256, (4 * K)}, /* Default */
	[1] = {0xEF, 0x6011, (128 * K), 256, (4 * K)}, /* W25Q10EW  */
	[2] = {0xEF, 0x6012, (256 * K), 256, (4 * K)}, /* W25Q20EW  */
	[3] = {0xC8, 0x6012, (256 * K), 256, (4 * K)}, /* GD25LQ20B */
	[4] = {0xC8, 0x6013, (512 * K), 256, (4 * K)}, /* GD25LQ40 */
	[5] = {0x85, 0x6013, (4 * M), 256, (4 * K)},
	[6] = {0xC2, 0x2812, (256 * K), 256, (4 * K)},
	[7] = {0x1C, 0x3812, (256 * K), 256, (4 * K)},
};

void ilitek_tddi_fw_read_flash_info(bool mode)
{
	int i = 0;
	u8 buf[4] = {0};
	u8 cmd = 0x9F;
	u16 flash_id = 0, flash_mid = 0;

	if (mode == UPGRADE_IRAM)
		return;

	ilitek_ice_mode_ctrl(ENABLE, OFF);

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x0, 1); /* CS low */
	ilitek_ice_mode_write(FLASH1_ADDR, 0x66aa55, 3); /* Key */
	ilitek_ice_mode_write(FLASH2_ADDR, cmd, 1);

	for (i = 0; i < ARRAY_SIZE(buf); i++) {
		ilitek_ice_mode_write(FLASH2_ADDR, 0xFF, 1);
		buf[i] = ilitek_ice_mode_read(FLASH4_ADDR, sizeof(u8));
		//ipio_info("buf[%d] = %x\n", i, buf[i]);
	}

	ilitek_ice_mode_write(FLASH_BASED_ADDR, 0x1, 1); /* CS high */

	flash_mid = buf[0];
	flash_id = buf[1] << 8 | buf[2];

	for (i = 0; i < ARRAY_SIZE(flashtab); i++) {
		if (flash_mid == flashtab[i].mid && flash_id == flashtab[i].dev_id) {
			idev->flash_mid = flashtab[i].mid;
			idev->flash_devid = flashtab[i].dev_id;
			idev->program_page = flashtab[i].program_page;
			idev->flash_sector = flashtab[i].sector;
			break;
		}
	}

	if (i >= ARRAY_SIZE(flashtab)) {
		ipio_info("Not found flash id in tab, use default\n");
		idev->flash_mid = flashtab[0].mid;
		idev->flash_devid = flashtab[0].dev_id;
		idev->program_page = flashtab[0].program_page;
		idev->flash_sector = flashtab[0].sector;
	}

	ipio_info("Flash MID = %x, Flash DEV_ID = %x\n", idev->flash_mid, idev->flash_devid);
	ipio_info("Flash program page = %d\n", idev->program_page);
	ipio_info("Flash sector = %d\n", idev->flash_sector);

	ilitek_tddi_flash_protect(DISABLE);
	ilitek_ice_mode_ctrl(DISABLE, OFF);
}
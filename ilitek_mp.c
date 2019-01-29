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

#define MP_PASS              0
#define MP_FAIL             -1
#define CSV_FILE_SIZE       (500 * K)
#define BENCHMARK_KEY_NAME "benchmark_data"
#define NODE_TYPE_KEY_NAME "node type"
#define VALUE               0
#define RETRY_COUNT         3
#define INT_CHECK           0
#define POLL_CHECK          1
#define DELAY_CHECK         2

#define NORMAL_CSV_PASS_NAME		"mp_pass"
#define NORMAL_CSV_FAIL_NAME		"mp_fail"

#define PARSER_MAX_CFG_BUF          (512 * 3)
#define PARSER_MAX_KEY_NUM	        (600 * 3)
#define PARSER_MAX_KEY_NAME_LEN	    100
#define PARSER_MAX_KEY_VALUE_LEN	2000
#define INI_ERR_OUT_OF_LINE         -1

#define CMD_MUTUAL_DAC              0x1
#define CMD_MUTUAL_BG               0x2
#define CMD_MUTUAL_SIGNAL           0x3
#define CMD_MUTUAL_NO_BK            0x5
#define CMD_MUTUAL_HAVE_BK          0x8
#define CMD_MUTUAL_BK_DAC           0x10
#define CMD_SELF_DAC                0xC
#define CMD_SELF_BG                 0xF
#define CMD_SELF_SIGNAL             0xD
#define CMD_SELF_NO_BK              0xE
#define CMD_SELF_HAVE_BK            0xB
#define CMD_SELF_BK_DAC             0x11
#define CMD_KEY_DAC                 0x14
#define CMD_KEY_BG                  0x16
#define CMD_KEY_NO_BK               0x7
#define CMD_KEY_HAVE_BK             0x15
#define CMD_KEY_OPEN                0x12
#define CMD_KEY_SHORT               0x13
#define CMD_ST_DAC                  0x1A
#define CMD_ST_BG                   0x1C
#define CMD_ST_NO_BK                0x17
#define CMD_ST_HAVE_BK              0x1B
#define CMD_ST_OPEN                 0x18
#define CMD_TX_SHORT                0x19
#define CMD_RX_SHORT                0x4
#define CMD_RX_OPEN                 0x6
#define CMD_TX_RX_DELTA             0x1E
#define CMD_CM_DATA                 0x9
#define CMD_CS_DATA                 0xA
#define CMD_TRCRQ_PIN               0x20
#define CMD_RESX2_PIN               0x21
#define CMD_MUTUAL_INTEGRA_TIME     0x22
#define CMD_SELF_INTEGRA_TIME       0x23
#define CMD_KEY_INTERGRA_TIME       0x24
#define CMD_ST_INTERGRA_TIME        0x25
#define CMD_PEAK_TO_PEAK            0x1D
#define CMD_GET_TIMING_INFO         0x30
#define CMD_DOZE_P2P                0x32
#define CMD_DOZE_RAW                0x33

struct ini_file_data {
	char pSectionName[PARSER_MAX_KEY_NAME_LEN];
	char pKeyName[PARSER_MAX_KEY_NAME_LEN];
	char pKeyValue[PARSER_MAX_KEY_VALUE_LEN];
	int iSectionNameLen;
	int iKeyNameLen;
	int iKeyValueLen;
} ilitek_ini_file_data[PARSER_MAX_KEY_NUM];

enum mp_test_catalog {
	MUTUAL_TEST = 0,
	SELF_TEST = 1,
	KEY_TEST = 2,
	ST_TEST = 3,
	TX_RX_DELTA = 4,
	UNTOUCH_P2P = 5,
	PIXEL = 6,
	OPEN_TEST = 7,
	PEAK_TO_PEAK_TEST = 8,
	SHORT_TEST = 9,
};

struct core_mp_test_data {
	bool retry;
	bool m_signal;
	bool m_dac;
	bool s_signal;
	bool s_dac;
	bool key_dac;
	bool st_dac;
	bool p_no_bk;
	bool p_has_bk;
	bool open_integ;
	bool open_cap;
	bool isLongV;

	int xch_len;
	int ych_len;
	int stx_len;
	int srx_len;
	int key_len;
	int st_len;
	int frame_len;
	int mp_items;
	int final_result;

	u32 overlay_start_addr;
	u32 overlay_end_addr;
	u32 mp_flash_addr;
	u32 mp_size;
	u8 dma_trigger_enable;

	/* Tx/Rx threshold & buffer */
	int TxDeltaMax;
	int TxDeltaMin;
	int RxDeltaMax;
	int RxDeltaMin;
	s32 *tx_delta_buf;
	s32 *rx_delta_buf;
	s32 *tx_max_buf;
	s32 *tx_min_buf;
	s32 *rx_max_buf;
	s32 *rx_min_buf;

	int tdf;
	bool busy_cdc;
	bool ctrl_lcm;
} core_mp = {0};

struct mp_test_items {
	char *name;
	/* The description must be the same as ini's section name */
	char *desp;
	char *result;
	int catalog;
	u8 cmd;
	u8 spec_option;
	u8 type_option;
	bool run;
	int max;
	int max_res;
	int item_result;
	int min;
	int min_res;
	int frame_count;
	int trimmed_mean;
	int lowest_percentage;
	int highest_percentage;
	int v_tdf_1;
	int v_tdf_2;
	int h_tdf_1;
	int h_tdf_2;
	s32 *result_buf;
	s32 *buf;
	s32 *max_buf;
	s32 *min_buf;
	s32 *bench_mark_max;
	s32 *bench_mark_min;
	s32 *node_type;
	int (*do_test)(int index);
};

#define MP_TEST_ITEM    47
static struct mp_test_items tItems[MP_TEST_ITEM] = {
	{.name = "mutual_dac", .desp = "calibration data(dac)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_bg", .desp = "baseline data(bg)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_signal", .desp = "untouch signal data(bg-raw-4096) - mutual", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_no_bk", .desp = "raw data(no bk)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_has_bk", .desp = "raw data(have bk)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_bk_dac", .desp = "manual bk data(mutual)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "self_dac", .desp = "calibration data(dac) - self", .result = "FAIL", .catalog = SELF_TEST},
	{.name = "self_bg", .desp = "baselin data(bg,self_tx,self_r)", .result = "FAIL", .catalog = SELF_TEST},
	{.name = "self_signal", .desp = "untouch signal data(bg-raw-4096) - self", .result = "FAIL", .catalog = SELF_TEST},
	{.name = "self_no_bk", .desp = "raw data(no bk) - self", .result = "FAIL", .catalog = SELF_TEST},
	{.name = "self_has_bk", .desp = "raw data(have bk) - self", .result = "FAIL", .catalog = SELF_TEST},
	{.name = "self_bk_dac", .desp = "manual bk dac data(self_tx,self_rx)", .result = "FAIL", .catalog = SELF_TEST},
	{.name = "key_dac", .desp = "calibration data(dac/icon)", .result = "FAIL", .catalog = KEY_TEST},
	{.name = "key_bg", .desp = "key baseline data", .result = "FAIL", .catalog = KEY_TEST},
	{.name = "key_no_bk", .desp = "key raw data", .result = "FAIL", .catalog = KEY_TEST},
	{.name = "key_has_bk", .desp = "key raw bk dac", .result = "FAIL", .catalog = KEY_TEST},
	{.name = "key_open", .desp = "key raw open test", .result = "FAIL", .catalog = KEY_TEST},
	{.name = "key_short", .desp = "key raw short test", .result = "FAIL", .catalog = KEY_TEST},
	{.name = "st_dac", .desp = "st calibration data(dac)", .result = "FAIL", .catalog = ST_TEST},
	{.name = "st_bg", .desp = "st baseline data(bg)", .result = "FAIL", .catalog = ST_TEST},
	{.name = "st_no_bk", .desp = "st raw data(no bk)", .result = "FAIL", .catalog = ST_TEST},
	{.name = "st_has_bk", .desp = "st raw(have bk)", .result = "FAIL", .catalog = ST_TEST},
	{.name = "st_open", .desp = "st open data", .result = "FAIL", .catalog = ST_TEST},
	{.name = "tx_short", .desp = "tx short test", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "rx_short", .desp = "short test -ili9881", .result = "FAIL", .catalog = SHORT_TEST},
	{.name = "rx_open", .desp = "rx open", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "cm_data", .desp = "untouch cm data", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "cs_data", .desp = "untouch cs data", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "tx_rx_delta", .desp = "tx/rx delta", .result = "FAIL", .catalog = TX_RX_DELTA},
	{.name = "p2p", .desp = "untouch peak to peak", .result = "FAIL", .catalog = UNTOUCH_P2P},
	{.name = "pixel_no_bk", .desp = "pixel raw (no bk)", .result = "FAIL", .catalog = PIXEL},
	{.name = "pixel_has_bk", .desp = "pixel raw (have bk)", .result = "FAIL", .catalog = PIXEL},
	{.name = "open_integration", .desp = "open test(integration)", .result = "FAIL", .catalog = OPEN_TEST},
	{.name = "open_cap", .desp = "open test(cap)", .result = "FAIL", .catalog = OPEN_TEST},
	/* New test items for protocol 5.4.0 as below */
	{.name = "noise_peak_to_peak_ic", .desp = "noise peak to peak(ic only)", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},
	{.name = "noise_peak_to_peak_panel", .desp = "noise peak to peak(with panel)", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},
	{.name = "noise_peak_to_peak_ic_lcm_off", .desp = "noise peak to peak(ic only) (lcm off)", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},
	{.name = "noise_peak_to_peak_panel_lcm_off", .desp = "noise peak to peak(with panel) (lcm off)", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},
	{.name = "mutual_no_bk_lcm_off", .desp = "raw data(no bk) (lcm off)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "mutual_has_bk_lcm_off", .desp = "raw data(have bk) (lcm off)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "open_integration_sp", .desp = "open test(integration)_sp", .result = "FAIL", .catalog = OPEN_TEST},
	{.name = "doze_raw", .desp = "doze raw data", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "doze_p2p", .desp = "doze peak to peak", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},
	{.name = "doze_raw_td_lcm_off", .desp = "raw data_td (lcm off)", .result = "FAIL", .catalog = MUTUAL_TEST},
	{.name = "doze_p2p_td_lcm_off", .desp = "peak to peak_td (lcm off)", .result = "FAIL", .catalog = PEAK_TO_PEAK_TEST},
	{.name = "rx_short", .desp = "short test", .result = "FAIL", .catalog = SHORT_TEST},
	{.name = "open test_c", .desp = "open test_c", .result = "FAIL", .catalog = OPEN_TEST},
};

int g_ini_items = 0;

static void dma_clear_reg_setting(struct ilitek_tddi_dev *idev)
{
	ipio_info("[Clear register setting]\n");

	ipio_info("interrupt t0/t1 enable flag\n");
	ilitek_ice_mode_bit_mask_write(idev, INTR32_ADDR, INTR32_reg_t0_int_en, (0 << 24));
	ilitek_ice_mode_bit_mask_write(idev, INTR32_ADDR, INTR32_reg_t1_int_en, (0 << 25));

	ipio_info("clear tdi_err_int_flag\n");
	ilitek_ice_mode_bit_mask_write(idev, INTR2_ADDR, INTR2_tdi_err_int_flag_clear, (1 << 18));

	ipio_info("clear dma channel 0 src1 info\n");
	ilitek_ice_mode_write(idev, DMA49_reg_dma_ch0_src1_addr, 0x00000000, 4);
	ilitek_ice_mode_write(idev, DMA50_reg_dma_ch0_src1_step_inc, 0x00, 1);
	ilitek_ice_mode_bit_mask_write(idev, DMA50_ADDR, DMA50_reg_dma_ch0_src1_format, (0 << 24));
	ilitek_ice_mode_bit_mask_write(idev, DMA50_ADDR, DMA50_reg_dma_ch0_src1_en, (1 << 31));

	ipio_info("clear dma channel 0 src2 info\n");
	ilitek_ice_mode_bit_mask_write(idev, DMA52_ADDR, DMA52_reg_dma_ch0_src2_en, (0 << 31));

	ipio_info("clear dma channel 0 trafer info\n");
	ilitek_ice_mode_write(idev, DMA55_reg_dma_ch0_trafer_counts, 0x00000000, 4);
	ilitek_ice_mode_bit_mask_write(idev, DMA55_ADDR, DMA55_reg_dma_ch0_trafer_mode, (0 << 24));

	ipio_info("clear dma channel 0 trigger select\n");
	ilitek_ice_mode_bit_mask_write(idev, DMA48_ADDR, DMA48_reg_dma_ch0_trigger_sel, (0 << 16));

	ilitek_ice_mode_bit_mask_write(idev, INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));

	ipio_info("clear dma flash setting\n");
	ilitek_tddi_flash_clear_dma(idev);
}

static void dma_trigger_reg_setting(uint32_t reg_dest_addr, uint32_t flash_start_addr, uint32_t copy_size)
{
	ipio_info("set dma channel 0 clear\n");
	ilitek_ice_mode_bit_mask_write(idev, DMA48_ADDR, DMA48_reg_dma_ch0_start_clear, (1 << 25));

	ipio_info("set dma channel 0 src1 info\n");
	ilitek_ice_mode_write(idev, DMA49_reg_dma_ch0_src1_addr, 0x00041010, 4);
	ilitek_ice_mode_write(idev, DMA50_reg_dma_ch0_src1_step_inc, 0x00, 1);
	ilitek_ice_mode_bit_mask_write(idev, DMA50_ADDR, DMA50_reg_dma_ch0_src1_format, (0 << 24));
	ilitek_ice_mode_bit_mask_write(idev, DMA50_ADDR, DMA50_reg_dma_ch0_src1_en, (1 << 31));

	ipio_info("set dma channel 0 src2 info\n");
	ilitek_ice_mode_bit_mask_write(idev, DMA52_ADDR, DMA52_reg_dma_ch0_src2_en, (0 << 31));

	ipio_info("set dma channel 0 dest info\n");
	ilitek_ice_mode_write(idev, DMA53_reg_dma_ch0_dest_addr, reg_dest_addr, 3);
	ilitek_ice_mode_write(idev, DMA54_reg_dma_ch0_dest_step_inc, 0x01, 1);
	ilitek_ice_mode_bit_mask_write(idev, DMA54_ADDR, DMA54_reg_dma_ch0_dest_format, (0 << 24));
	ilitek_ice_mode_bit_mask_write(idev, DMA54_ADDR, DMA54_reg_dma_ch0_dest_en, (1 << 31));

	ipio_info("set dma channel 0 trafer info\n");
	ilitek_ice_mode_write(idev, DMA55_reg_dma_ch0_trafer_counts, copy_size, 4);
	ilitek_ice_mode_bit_mask_write(idev, DMA55_ADDR, DMA55_reg_dma_ch0_trafer_mode, (0 << 24));

	ipio_info("set dma channel 0 int info\n");
	ilitek_ice_mode_bit_mask_write(idev, INTR33_ADDR, INTR33_reg_dma_ch0_int_en, (1 << 17));

	ipio_info("set dma channel 0 trigger select\n");
	ilitek_ice_mode_bit_mask_write(idev, DMA48_ADDR, DMA48_reg_dma_ch0_trigger_sel, (1 << 16));

	ipio_info("set dma flash setting, FlashAddr = 0x%x\n",flash_start_addr);
	ilitek_tddi_flash_dma_write(idev, flash_start_addr,(flash_start_addr+copy_size), copy_size);

	ipio_info("clear flash and dma ch0 int flag\n");
	ilitek_ice_mode_bit_mask_write(idev, INTR1_ADDR, INTR1_reg_flash_int_flag, (1 << 25));
	ilitek_ice_mode_bit_mask_write(idev, INTR1_ADDR, INTR1_reg_dma_ch0_int_flag, (1 << 17));
	ilitek_ice_mode_bit_mask_write(idev, 0x041013, BIT(0), 1); //patch

	/* DMA Trigger */
	ilitek_ice_mode_write(idev, FLASH4_reg_rcv_data, 0xFF, 1);
	mdelay(30);

	/* CS High */
	ilitek_ice_mode_write(idev, FLASH0_reg_flash_csb, 0x1, 1);
	mdelay(60);
}

static int isspace_t(int x)
{
    if (x==' '||x=='\t'||x=='\n'||x=='\f'||x=='\b'||x=='\r')
        return 1;
    else
        return 0;
}

static int parser_get_ini_key_value(char *section, char *key, char *value)
{
	int i = 0;
	int ret = -2;
	int len = 0;

	len = strlen(key);

	for (i = 0; i < g_ini_items; i++) {
		if (strcmp(section, ilitek_ini_file_data[i].pSectionName) != 0)
			continue;

		if (strcmp(key, ilitek_ini_file_data[i].pKeyName) == 0) {
			ipio_memcpy(value, ilitek_ini_file_data[i].pKeyValue, ilitek_ini_file_data[i].iKeyValueLen, PARSER_MAX_KEY_VALUE_LEN);
			ipio_debug(DEBUG_PARSER, " value:%s , pKeyValue: %s\n", value, ilitek_ini_file_data[i].pKeyValue);
			ret = 0;
			break;
		}
	}
	return ret;
}

static int parser_get_u8_array(char *key, uint8_t *buf, uint16_t base, size_t len)
{
	char *s = key;
	char *pToken;
	int ret, conut = 0;
    long s_to_long = 0;

	if (strlen(s) == 0 || len <= 0) {
		ipio_err("Can't find any characters inside buffer\n");
		return -1;
	}

	/*
	*  @base: The number base to use. The maximum supported base is 16. If base is
	*  given as 0, then the base of the string is automatically detected with the
	*  conventional semantics - If it begins with 0x the number will be parsed as a
	*  hexadecimal (case insensitive), if it otherwise begins with 0, it will be
	*  parsed as an octal number. Otherwise it will be parsed as a decimal.
	*/
	if (isspace_t((int)(unsigned char)*s) == 0)
	{
		while((pToken = strsep(&s, ",")) != NULL) {
			ret = kstrtol(pToken, base, &s_to_long);
			if (ret == 0)
				buf[conut] = s_to_long;
			else
				ipio_info("convert string too long, ret = %d\n", ret);
			conut++;

			if (conut >= len)
				break;
		}
	}

	return conut;
}

static int parser_get_int_data(char *section, char *keyname, char *rv)
{
	int len = 0;
	char value[512] = { 0 };

	if (rv == NULL || section == NULL || keyname == NULL) {
		ipio_err("Parameters are invalid\n");
		return -EINVAL;
	}

	/* return a white-space string if get nothing */
	if (parser_get_ini_key_value(section, keyname, value) < 0) {
		sprintf(rv, "%s", value);
		return 0;
	}

	len = sprintf(rv, "%s", value);
	return len;
}

/* Count the number of each line and assign the content to tmp buffer */
static int parser_get_ini_phy_line(char *data, char *buffer, int maxlen)
{
	int i = 0;
	int j = 0;
	int iRetNum = -1;
	char ch1 = '\0';

	for (i = 0, j = 0; i < maxlen; j++) {
		ch1 = data[j];
		iRetNum = j + 1;
		if (ch1 == '\n' || ch1 == '\r') {	/* line end */
			ch1 = data[j + 1];
			if (ch1 == '\n' || ch1 == '\r') {
				iRetNum++;
			}

			break;
		} else if (ch1 == 0x00) {
			//iRetNum = -1;
			break;	/* file end */
		}

		buffer[i++] = ch1;
	}

	buffer[i] = '\0';
	return iRetNum;
}

static char *parser_ini_str_trim_r(char *buf)
{
	int len, i;
	char tmp[512] = { 0 };

	len = strlen(buf);

	for (i = 0; i < len; i++) {
		if (buf[i] != ' ')
			break;
	}

	if (i < len)
		strncpy(tmp, (buf + i), (len - i));

	strncpy(buf, tmp, len);
	return buf;
}

static int parser_get_ini_phy_data(char *data, int fsize)
{
	int i, n = 0, ret = 0 , banchmark_flag = 0, empty_section, nodetype_flag = 0;
	int offset = 0, isEqualSign = 0;
	char *ini_buf = NULL, *tmpSectionName = NULL;
	char M_CFG_SSL = '[';
	char M_CFG_SSR = ']';
/* char M_CFG_NIS = ':'; */
	char M_CFG_NTS = '#';
	char M_CFG_EQS = '=';

	if (data == NULL) {
		ipio_err("INI data is NULL\n");
		ret = -EINVAL;
		goto out;
	}

	ini_buf = kzalloc((PARSER_MAX_CFG_BUF + 1) * sizeof(char), GFP_KERNEL);
	if (ERR_ALLOC_MEM(ini_buf)) {
		ipio_err("Failed to allocate ini_buf memory, %ld\n", PTR_ERR(ini_buf));
		ret = -ENOMEM;
		goto out;
	}

	tmpSectionName = kzalloc((PARSER_MAX_CFG_BUF + 1) * sizeof(char), GFP_KERNEL);
	if (ERR_ALLOC_MEM(tmpSectionName)) {
		ipio_err("Failed to allocate tmpSectionName memory, %ld\n", PTR_ERR(tmpSectionName));
		ret = -ENOMEM;
		goto out;
	}

    while (true) {
		banchmark_flag = 0;
		empty_section = 0;
		nodetype_flag = 0;
		if (g_ini_items > PARSER_MAX_KEY_NUM) {
			ipio_err("MAX_KEY_NUM: Out of length\n");
			goto out;
		}

		if (offset >= fsize)
			goto out;/*over size*/

		n = parser_get_ini_phy_line(data + offset, ini_buf, PARSER_MAX_CFG_BUF);
		if (n < 0) {
			ipio_err("End of Line\n");
			goto out;
		}

		offset += n;

		n = strlen(parser_ini_str_trim_r(ini_buf));

		if (n == 0 || ini_buf[0] == M_CFG_NTS)
			continue;

		/* Get section names */
		if (n > 2 && ((ini_buf[0] == M_CFG_SSL && ini_buf[n - 1] != M_CFG_SSR))) {
			ipio_err("Bad Section: %s\n", ini_buf);
			ret = -EINVAL;
			goto out;
		} else {
			if (ini_buf[0] == M_CFG_SSL) {
				ilitek_ini_file_data[g_ini_items].iSectionNameLen = n - 2;
				if (ilitek_ini_file_data[g_ini_items].iSectionNameLen > PARSER_MAX_KEY_NAME_LEN) {
					ipio_err("MAX_KEY_NAME_LEN: Out Of Length\n");
					ret = INI_ERR_OUT_OF_LINE;
					goto out;
				}

				ini_buf[n - 1] = 0x00;
				strcpy((char *)tmpSectionName, ini_buf + 1);
				banchmark_flag = 0;
				nodetype_flag = 0;
				ipio_debug(DEBUG_PARSER, "Section Name: %s, Len: %d, offset = %d\n", tmpSectionName, n - 2, offset);
				continue;
			}
		}

		/* copy section's name without square brackets to its real buffer */
		strcpy(ilitek_ini_file_data[g_ini_items].pSectionName, tmpSectionName);
		ilitek_ini_file_data[g_ini_items].iSectionNameLen = strlen(tmpSectionName);

		isEqualSign = 0;
		for (i = 0; i < n; i++) {
			if (ini_buf[i] == M_CFG_EQS) {
				isEqualSign = i;
				break;
			}
			if (ini_buf[i] == M_CFG_SSL || ini_buf[i] == M_CFG_SSR) {
				empty_section = 1;
				break;
			}
		}

		if (isEqualSign == 0) {
			if (empty_section)
				continue;

			if (strstr(ilitek_ini_file_data[g_ini_items].pSectionName,BENCHMARK_KEY_NAME) > 0) {
				banchmark_flag = 1;
				isEqualSign =-1;
			} else if (strstr(ilitek_ini_file_data[g_ini_items].pSectionName,NODE_TYPE_KEY_NAME) > 0) {
				nodetype_flag = 1;
				isEqualSign =-1;
			} else {
                continue;
            }
		}

		if (banchmark_flag) {
			ilitek_ini_file_data[g_ini_items].iKeyNameLen = strlen(BENCHMARK_KEY_NAME);
			strcpy(ilitek_ini_file_data[g_ini_items].pKeyName, BENCHMARK_KEY_NAME);
			ilitek_ini_file_data[g_ini_items].iKeyValueLen = n;
		} else if (nodetype_flag) {
			ilitek_ini_file_data[g_ini_items].iKeyNameLen = strlen(NODE_TYPE_KEY_NAME);
			strcpy(ilitek_ini_file_data[g_ini_items].pKeyName, NODE_TYPE_KEY_NAME);
			ilitek_ini_file_data[g_ini_items].iKeyValueLen = n;
		} else{
			ilitek_ini_file_data[g_ini_items].iKeyNameLen = isEqualSign;
			if (ilitek_ini_file_data[g_ini_items].iKeyNameLen > PARSER_MAX_KEY_NAME_LEN) {
				/* ret = CFG_ERR_OUT_OF_LEN; */
				ipio_err("MAX_KEY_NAME_LEN: Out Of Length\n");
				ret = INI_ERR_OUT_OF_LINE;
				goto out;
			}

			ipio_memcpy(ilitek_ini_file_data[g_ini_items].pKeyName, ini_buf,
						ilitek_ini_file_data[g_ini_items].iKeyNameLen, PARSER_MAX_KEY_NAME_LEN);
			ilitek_ini_file_data[g_ini_items].iKeyValueLen = n - isEqualSign - 1;
		}

		if (ilitek_ini_file_data[g_ini_items].iKeyValueLen > PARSER_MAX_KEY_VALUE_LEN) {
			ipio_err("MAX_KEY_VALUE_LEN: Out Of Length\n");
			ret = INI_ERR_OUT_OF_LINE;
			goto out;
		}

		ipio_memcpy(ilitek_ini_file_data[g_ini_items].pKeyValue,
		       ini_buf + isEqualSign + 1, ilitek_ini_file_data[g_ini_items].iKeyValueLen, PARSER_MAX_KEY_VALUE_LEN);

		ipio_debug(DEBUG_PARSER, "%s = %s\n", ilitek_ini_file_data[g_ini_items].pKeyName,
		    ilitek_ini_file_data[g_ini_items].pKeyValue);

		g_ini_items++;
    }
out:
	ipio_kfree((void **)&ini_buf);
	ipio_kfree((void **)&tmpSectionName);
	return ret;
}

static int ilitek_tddi_mp_ini_parser(const char *path)
{
	int i, ret = 0, fsize = 0;
	char *tmp = NULL;
	struct file *f = NULL;
	struct inode *inode;
	mm_segment_t old_fs;
	loff_t pos = 0;

	ipio_info("ini file path = %s\n", path);
	f = filp_open(path, O_RDONLY, 644);
	if (ERR_ALLOC_MEM(f)) {
		ipio_err("Failed to open ini file at %ld.\n", PTR_ERR(f));
		return -ENOENT;
	}

#if KERNEL_VERSION(3, 18, 0) >= LINUX_VERSION_CODE
	inode = f->f_dentry->d_inode;
#else
	inode = f->f_path.dentry->d_inode;
#endif

	fsize = inode->i_size;
	ipio_info("ini file size = %d\n", fsize);
	if (fsize <= 0) {
		ipio_err("The size of file is invaild\n");
		ret = -EINVAL;
		goto out;
	}

	tmp = vmalloc(fsize+1);
	if (ERR_ALLOC_MEM(tmp)) {
		ipio_err("Failed to allocate tmp memory, %ld\n", PTR_ERR(tmp));
		ret = -ENOMEM;
		goto out;
	}

	old_fs = get_fs();
	set_fs(get_ds());
	vfs_read(f, tmp, fsize, &pos);
	set_fs(old_fs);
	tmp[fsize] = 0x00;

	g_ini_items = 0;

	/* Initialise ini strcture */
	for (i = 0; i < PARSER_MAX_KEY_NUM; i++) {
		memset(ilitek_ini_file_data[i].pSectionName, 0, PARSER_MAX_KEY_NAME_LEN);
		memset(ilitek_ini_file_data[i].pKeyName, 0, PARSER_MAX_KEY_NAME_LEN);
		memset(ilitek_ini_file_data[i].pKeyValue, 0, PARSER_MAX_KEY_VALUE_LEN);
		ilitek_ini_file_data[i].iSectionNameLen = 0;
		ilitek_ini_file_data[i].iKeyNameLen = 0;
		ilitek_ini_file_data[i].iKeyValueLen = 0;
	}

	/* change all characters to lower case */
	for (i = 0; i < strlen(tmp); i++)
		tmp[i] = tolower(tmp[i]);

	ret = parser_get_ini_phy_data(tmp, fsize);
	if (ret < 0) {
		ipio_err("Failed to get physical ini data, ret = %d\n", ret);
		goto out;
	}

	ipio_info("Parsed INI file done\n");
out:
	ipio_vfree((void **)&tmp);
	filp_close(f, NULL);
	return ret;
}

static int mutual_test(int index)
{
    return 0;
}

static int open_test_sp(int index)
{
    return 0;
}

static int open_test_cap(int index)
{
    return 0;
}

static int key_test(int index)
{
    ipio_info();
    return 0;
}

static int self_test(int index)
{
	ipio_err("TDDI has no self to be tested currently\n");
	return -1;
}

static int st_test(int index)
{
	ipio_err("ST Test is not supported by the driver\n");
	return -1;
}

static int mp_get_timing_info(void)
{
	int slen = 0;
	char str[256] = {0};
	uint8_t info[64] = {0};
	char *key = "timing_info_raw";

	core_mp.isLongV = 0;

	slen = parser_get_int_data("pv5_4 command", key, str);
	if (slen < 0)
		return -1;

	if (parser_get_u8_array(str, info, 16, slen) < 0)
		return -1;

	core_mp.isLongV = info[6];

	ipio_info("DDI Mode = %s\n", (core_mp.isLongV ? "Long V" : "Long H"));

	return 0;
}

static void ilitek_tddi_mp_init_item(struct ilitek_tddi_dev *idev)
{
    int i = 0;

    ipio_info();

    memset(&core_mp, 0, sizeof(core_mp));

    core_mp.xch_len = idev->xch_num;
    core_mp.ych_len = idev->ych_num;
    core_mp.stx_len = 0;
    core_mp.srx_len = 0;
    core_mp.key_len = 0;
    core_mp.st_len = 0;
    core_mp.tdf = 240;
    core_mp.busy_cdc = POLL_CHECK;
    core_mp.retry = false;
    core_mp.final_result = MP_FAIL;

    for (i = 0; i < MP_TEST_ITEM; i++) {
		tItems[i].spec_option = 0;
		tItems[i].type_option = 0;
		tItems[i].run = false;
		tItems[i].max = 0;
		tItems[i].max_res = MP_FAIL;
		tItems[i].item_result = MP_PASS;
		tItems[i].min = 0;
		tItems[i].min_res = MP_FAIL;
		tItems[i].frame_count = 0;
		tItems[i].trimmed_mean = 0;
		tItems[i].lowest_percentage = 0;
		tItems[i].highest_percentage = 0;
		tItems[i].v_tdf_1 = 0;
		tItems[i].v_tdf_2 = 0;
		tItems[i].h_tdf_1 = 0;
		tItems[i].h_tdf_2 = 0;
		tItems[i].result_buf = NULL;
		tItems[i].buf = NULL;
		tItems[i].max_buf = NULL;
		tItems[i].min_buf = NULL;
		tItems[i].bench_mark_max = NULL;
		tItems[i].bench_mark_min = NULL;
		tItems[i].node_type = NULL;

		if (tItems[i].catalog == MUTUAL_TEST) {
			tItems[i].do_test = mutual_test;
		} else if (tItems[i].catalog == TX_RX_DELTA) {
			tItems[i].do_test = mutual_test;
		} else if (tItems[i].catalog == UNTOUCH_P2P) {
			tItems[i].do_test = mutual_test;
		} else if (tItems[i].catalog == PIXEL) {
			tItems[i].do_test = mutual_test;
		} else if (tItems[i].catalog == OPEN_TEST) {
			if (strcmp(tItems[i].name, "open_integration_sp") == 0)
				tItems[i].do_test = open_test_sp;
			else if (strcmp(tItems[i].name, "open test_c") == 0)
				tItems[i].do_test = open_test_cap;
			else
				tItems[i].do_test = mutual_test;
		} else if (tItems[i].catalog == KEY_TEST) {
			tItems[i].do_test = key_test;
		} else if (tItems[i].catalog == SELF_TEST) {
			tItems[i].do_test = self_test;
		} else if (tItems[i].catalog == ST_TEST) {
			tItems[i].do_test = st_test;
		} else if (tItems[i].catalog == PEAK_TO_PEAK_TEST) {
			tItems[i].do_test = mutual_test;
		} else if (tItems[i].catalog == SHORT_TEST) {
			tItems[i].do_test = mutual_test;
		}

		tItems[i].result = kmalloc(16, GFP_KERNEL);
        sprintf(tItems[i].result, "%s", "FAIL");
    }

    tItems[0].cmd = CMD_MUTUAL_DAC;
    tItems[1].cmd = CMD_MUTUAL_BG;
    tItems[2].cmd = CMD_MUTUAL_SIGNAL;
    tItems[3].cmd = CMD_MUTUAL_NO_BK;
    tItems[4].cmd = CMD_MUTUAL_HAVE_BK;
    tItems[5].cmd = CMD_MUTUAL_BK_DAC;
    tItems[6].cmd = CMD_SELF_DAC;
    tItems[7].cmd = CMD_SELF_BG;
    tItems[8].cmd = CMD_SELF_SIGNAL;
    tItems[9].cmd = CMD_SELF_NO_BK;
    tItems[10].cmd = CMD_SELF_HAVE_BK;
    tItems[11].cmd = CMD_SELF_BK_DAC;
    tItems[12].cmd = CMD_KEY_DAC;
    tItems[13].cmd = CMD_KEY_BG;
    tItems[14].cmd = CMD_KEY_NO_BK;
    tItems[15].cmd = CMD_KEY_HAVE_BK;
    tItems[16].cmd = CMD_KEY_OPEN;
    tItems[17].cmd = CMD_KEY_SHORT;
    tItems[18].cmd = CMD_ST_DAC;
    tItems[19].cmd = CMD_ST_BG;
    tItems[20].cmd = CMD_ST_NO_BK;
    tItems[21].cmd = CMD_ST_HAVE_BK;
    tItems[22].cmd = CMD_ST_OPEN;
    tItems[23].cmd = CMD_TX_SHORT;
    tItems[24].cmd = CMD_RX_SHORT;
    tItems[25].cmd = CMD_RX_OPEN;
    tItems[26].cmd = CMD_CM_DATA;
    tItems[27].cmd = CMD_CS_DATA;
    tItems[28].cmd = CMD_TX_RX_DELTA;
    tItems[29].cmd = CMD_MUTUAL_SIGNAL;
    tItems[30].cmd = CMD_MUTUAL_NO_BK;
    tItems[31].cmd = CMD_MUTUAL_HAVE_BK;
    tItems[32].cmd = CMD_RX_SHORT;
    tItems[33].cmd = CMD_RX_SHORT;
    tItems[34].cmd = CMD_PEAK_TO_PEAK;
}

int ilitek_tddi_mp_test_run(struct ilitek_tddi_dev *idev)
{
    int ret = 0;
	u8 tp_mode;

    ilitek_tddi_mp_init_item(idev);

	ret = ilitek_tddi_mp_ini_parser(INI_NAME_PATH);
	if (ret < 0) {
		ipio_err("Failed to parsing INI file\n");
		goto out;
	}

	/* Switch to Test mode nad move mp code into iram */
	tp_mode = P5_X_FW_TEST_MODE;
	ret = ilitek_tddi_touch_switch_mode(idev, &tp_mode);
	if (ret < 0) {
		ipio_err("Switch to test mode failed\n");
		goto out;
	}

	/* Read timing info from ini file */
	ret = mp_get_timing_info();
	if (ret < 0) {
		ipio_err("Failed to get timing info from ini\n");
		goto out;
	}

out:
	/* Switch to Demo mode */
	tp_mode = P5_X_FW_DEMO_MODE;
	ret = ilitek_tddi_touch_switch_mode(idev, &tp_mode);
	if (ret < 0) {
		ipio_err("Switch to test mode failed\n");
		goto out;
	}

    return ret;
};

int ilitek_tddi_mp_move_code_flash(struct ilitek_tddi_dev *idev)
{
	int ret = 0;
	u32 mp_text_size = 0, mp_andes_init_size = 0;
	u8 cmd[16] = {0};

	cmd[0] = P5_X_MODE_CONTROL;
	cmd[1] = P5_X_FW_TEST_MODE;
	ret = idev->write(idev, cmd, 2);
	if (ret < 0)
		goto out;

	cmd[0] = P5_X_MP_TEST_MODE_INFO;
	ret = idev->write(idev, cmd, 1);
	if (ret < 0)
		goto out;

	memset(cmd, 0, sizeof(cmd));

	ipio_info("read mp info length = %d\n", idev->protocol->mp_info_len);
	ret = idev->read(idev, cmd, idev->protocol->mp_info_len);
	if (ret < 0)
		goto out;

	ilitek_dump_data(cmd, 8, idev->protocol->mp_info_len, 0, "MP overlay info");

	core_mp.dma_trigger_enable = 0;

	core_mp.mp_flash_addr = cmd[3] + (cmd[2] << 8) + (cmd[1] << 16);
	core_mp.mp_size = cmd[6] + (cmd[5] << 8) + (cmd[4] << 16);
	core_mp.overlay_start_addr = cmd[9] + (cmd[8] << 8) + (cmd[7] << 16);
	core_mp.overlay_end_addr = cmd[12] + (cmd[11] << 8) + (cmd[10] << 16);

	if (core_mp.overlay_start_addr != 0x0 && core_mp.overlay_end_addr != 0x0
		&& cmd[0] == P5_X_MP_TEST_MODE_INFO)
		core_mp.dma_trigger_enable = 1;

	ipio_info("MP info Overlay: Enable = %d, addr = 0x%x ~ 0x%x, flash addr = 0x%x, mp size = 0x%x\n",
		core_mp.dma_trigger_enable, core_mp.overlay_start_addr,
		core_mp.overlay_end_addr, core_mp.mp_flash_addr, core_mp.mp_size);

	/* Check if ic is ready switching test mode from demo mode */
	idev->actual_fw_mode = P5_X_FW_DEMO_MODE;
	ret = ilitek_tddi_ic_check_busy(idev, 50, 50); /* Set busy as 0x41 */
	if (ret < 0)
		goto out;

	ret = ilitek_ice_mode_ctrl(idev, ICE_ENABLE, MCU_STOP);
	if (ret < 0)
		goto out;

	if (core_mp.dma_trigger_enable) {
		mp_andes_init_size = core_mp.overlay_start_addr;
		mp_text_size = (core_mp.mp_size - core_mp.overlay_end_addr) + 1;
		ipio_info("MP andes init size = %d , MP text size = %d\n",mp_andes_init_size, mp_text_size);

		dma_clear_reg_setting(idev);

		ipio_info("[Move ANDES.INIT to DRAM]\n");
		dma_trigger_reg_setting(0, core_mp.mp_flash_addr, mp_andes_init_size);   /* DMA ANDES.INIT */

		dma_clear_reg_setting(idev);

		ipio_info("[Move MP.TEXT to DRAM]\n");
		dma_trigger_reg_setting(core_mp.overlay_end_addr, (core_mp.mp_flash_addr + core_mp.overlay_start_addr), mp_text_size);
	} else {
		/* DMA Trigger */
		ilitek_ice_mode_write(idev, FLASH4_reg_rcv_data, 0xFF, 1);
		mdelay(30);

		/* CS High */
		ilitek_ice_mode_write(idev, FLASH0_reg_flash_csb, 0x1, 1);
		mdelay(60);
	}

	ilitek_tddi_reset_ctrl(idev, TP_IC_CODE_RST);

	ret = ilitek_ice_mode_ctrl(idev, ICE_DISABLE, MCU_STOP);
	if (ret < 0)
		goto out;

	/* Check if ic is already in test mode */
	idev->actual_fw_mode = P5_X_FW_TEST_MODE; /* set busy as 0x51 */
	ret = ilitek_tddi_ic_check_busy(idev, 300, 50);

out:
    return ret;
}

int ilitek_tddi_mp_move_code_iram(struct ilitek_tddi_dev *idev)
{
    ipio_info();
    return 0;
}
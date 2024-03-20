/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include <string.h>
#include <ssd1306.h>
#include <font8x8_basic.h>
#include <uit_logo.h>

static const char *TAG = "LAB02";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */

static esp_err_t i2c_master_init(void);
// static void disp_buf(uint8_t *buf, int len);

void ssd1306_init();
void task_ssd1306_display_text(const void *arg_text);
void task_ssd1306_display_clear();
void task_ssd1306_display_img (uint8_t img[]);

uint8_t shift_right(uint8_t left_opr, int8_t right_opr);
void convert_img(uint8_t img_arr[], uint8_t converted_img_arr[1024]);


void app_main(void)
{
	ESP_ERROR_CHECK(i2c_master_init());
	ssd1306_init();
	
	while (1)
	{
		task_ssd1306_display_img(uit_logo);
		vTaskDelay(5000/portTICK_PERIOD_MS);

		task_ssd1306_display_clear();

		task_ssd1306_display_text((void *)"21521048\n21521535\n21521548\n                ERROR\n\n\n\n\nERROR");
		vTaskDelay(5000/portTICK_PERIOD_MS);
	}
}


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        // .clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
    };
    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        return err;
    }
    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

// /**
//  * @brief test function to show buffer
//  */
// static void disp_buf(uint8_t *buf, int len)
// {
//     int i;
//     for (i = 0; i < len; i++) {
//         printf("%02x ", buf[i]);
//         if ((i + 1) % 16 == 0) {
//             printf("\n");
//         }
//     }
//     printf("\n");
// }

void ssd1306_init() {
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_X_FLIPPED, true); // reverse left-right mapping
	i2c_master_write_byte(cmd, OLED_CMD_SET_Y_FLIPPED, true); // reverse up-bottom mapping
	
	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_NORMAL, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
	i2c_master_write_byte(cmd, 0x14, true);

	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	if (espRc == ESP_OK) {
		ESP_LOGI(TAG, "OLED configured successfully");
	} else {
		ESP_LOGE(TAG, "OLED configuration failed. code: 0x%.2X", espRc);
	}
}

void task_ssd1306_display_text(const void *arg_text) {
	esp_err_t ret = ESP_OK;
	char *text = (char*)arg_text;
	uint8_t text_len = strlen(text);

	i2c_cmd_handle_t cmd;

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	i2c_master_write_byte(cmd, OLED_CMD_SET_MEMORY_ADDR_MODE, true); // switch to page addressing
	i2c_master_write_byte(cmd, 0x02, true);
	i2c_master_write_byte(cmd, OLED_CMD_SET_COL_ADDR_LOW, true); // reset column - choose column --> 0
	i2c_master_write_byte(cmd, OLED_CMD_SET_COL_ADDR_HIGH, true);
	i2c_master_write_byte(cmd, OLED_CMD_SET_PAGE_ADDRESS, true); // reset page

	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	for (uint8_t i = 0, ch_count = 0, cur_page = 0; i < text_len && ret == ESP_OK; i++) {
		if (text[i] == '\n')
		{
			if (cur_page > 6)
				break;
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
			i2c_master_write_byte(cmd, OLED_CMD_SET_COL_ADDR_LOW, true); // reset column
			i2c_master_write_byte(cmd, OLED_CMD_SET_COL_ADDR_HIGH, true);
			i2c_master_write_byte(cmd, OLED_CMD_SET_PAGE_ADDRESS | ++cur_page, true); // increment page

			i2c_master_stop(cmd);
			ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10/portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);

			ch_count = 0;
		}
		else if (ch_count < 16)
		{
			cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

			i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
			i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);

			i2c_master_stop(cmd);
			ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10/portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);

			ch_count++;
		}
	}

	if (ret == ESP_OK)
		ESP_LOGI(TAG, "Written: %s", text);
	else if (ret == ESP_ERR_TIMEOUT)
		ESP_LOGE(TAG, "I2C Timeout");
	else
		ESP_LOGW(TAG, "Failed to write text - %s", esp_err_to_name(ret));

	// vTaskDelete(NULL);
}

void task_ssd1306_display_clear() {
	esp_err_t ret = ESP_OK;
	i2c_cmd_handle_t cmd;

	uint8_t clear[128] = {0};

	for (uint8_t i = 0; i < 8 && ret == ESP_OK; i++) {
		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
		i2c_master_write_byte(cmd, OLED_CMD_SET_PAGE_ADDRESS | i, true);

		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
		i2c_master_write(cmd, clear, 128, true);
		i2c_master_stop(cmd);
		ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
	}

	if (ret == ESP_OK)
		ESP_LOGI(TAG, "Cleaned");
	else if (ret == ESP_ERR_TIMEOUT)
		ESP_LOGE(TAG, "I2C Timeout");
	else
		ESP_LOGW(TAG, "Failed to clear screen - %s", esp_err_to_name(ret));

	// vTaskDelete(NULL);
}

void task_ssd1306_display_img (uint8_t img[])
{
	esp_err_t ret = ESP_OK;
	i2c_cmd_handle_t cmd;
	
	uint8_t converted_image[1024];
	convert_img(img, converted_image);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

	// Change to Horizontal Addressing mode
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
	i2c_master_write_byte(cmd, OLED_CMD_SET_MEMORY_ADDR_MODE, true);
	i2c_master_write_byte(cmd, 0x00, true);

	// Set Page and Col range
	i2c_master_write_byte(cmd, OLED_CMD_SET_COLUMN_RANGE, true);
	i2c_master_write_byte(cmd, 0x00, true);
	i2c_master_write_byte(cmd, 0x7F, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_PAGE_RANGE, true);
	i2c_master_write_byte(cmd, 0x00, true);
	i2c_master_write_byte(cmd, 0x07, true);
	
	i2c_master_stop(cmd);
	ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	if (ret == ESP_OK)
	{
		// Write image
		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
		
		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
		i2c_master_write(cmd, converted_image, 1024, true);

		i2c_master_stop(cmd);
		ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 10/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
	}

	if (ret == ESP_OK)
		ESP_LOGI(TAG, "Image displayed");
	else if (ret == ESP_ERR_TIMEOUT)
		ESP_LOGE(TAG, "I2C Timeout");
	else
		ESP_LOGW(TAG, "Failed to display image - %s", esp_err_to_name(ret));
}


uint8_t shift_right(uint8_t left_opr, int8_t right_opr)
{
	return (right_opr > 0? left_opr >> right_opr : left_opr << (0 - right_opr));
}

void convert_img(uint8_t img_arr[], uint8_t converted_img_arr[1024])
{
	uint16_t index = 0;
	// loop, scan 8 rows at a time
	for (uint8_t page_index = 0; page_index < 8; page_index++)
	{
		// loop, scan 8 bytes, each from every rows
		for (uint8_t byte_index = 0; byte_index < 16; byte_index++)
		{
			uint8_t byte_stack[] = {img_arr[(page_index * 8 + 0) * 16 + byte_index],
															img_arr[(page_index * 8 + 1) * 16 + byte_index],
															img_arr[(page_index * 8 + 2) * 16 + byte_index],
															img_arr[(page_index * 8 + 3) * 16 + byte_index],
															img_arr[(page_index * 8 + 4) * 16 + byte_index],
															img_arr[(page_index * 8 + 5) * 16 + byte_index],
															img_arr[(page_index * 8 + 6) * 16 + byte_index],
															img_arr[(page_index * 8 + 7) * 16 + byte_index],
															};
			// loop, scan each bit, combine them into a byte
			for (int8_t bit_index = 0; bit_index < 8; bit_index++)
			{
				converted_img_arr[index] = (((byte_stack[0] << bit_index) & 0x80) >> 7)
																 | (((byte_stack[1] << bit_index) & 0x80) >> 6)
																 | (((byte_stack[2] << bit_index) & 0x80) >> 5)
																 | (((byte_stack[3] << bit_index) & 0x80) >> 4)
																 | (((byte_stack[4] << bit_index) & 0x80) >> 3)
																 | (((byte_stack[5] << bit_index) & 0x80) >> 2)
																 | (((byte_stack[6] << bit_index) & 0x80) >> 1)
																 | (((byte_stack[7] << bit_index) & 0x80) >> 0);
				index++;
			}
		}
	}
}
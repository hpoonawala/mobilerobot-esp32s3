/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rmt.h"

// LED control using RMT example
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM      (35)

#define EXAMPLE_LED_NUMBERS         24
#define EXAMPLE_CHASE_SPEED_MS      100

#define RMT_TASK_STACK_SIZE (4096)
static uint8_t led_strip_pixels[EXAMPLE_LED_NUMBERS * 3];

static void rmt_task(void *arg)
{

	rmt_channel_handle_t led_chan = NULL;
	rmt_tx_channel_config_t tx_chan_config = {
		.clk_src = RMT_CLK_SRC_DEFAULT, // select source clock
		.gpio_num = RMT_LED_STRIP_GPIO_NUM,
		.mem_block_symbols = 64, // increase the block size can make the LED less flickering
		.resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
		.trans_queue_depth = 4, // set the number of transactions that can be pending in the background
	};
	ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

	rmt_encoder_handle_t led_encoder = NULL;
	led_strip_encoder_config_t encoder_config = {
		.resolution = RMT_LED_STRIP_RESOLUTION_HZ,
	};
	ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

	ESP_ERROR_CHECK(rmt_enable(led_chan));

	rmt_transmit_config_t tx_config = {
		.loop_count = 0, // no transfer loop
	};
	uint32_t red = 0;
	uint32_t green = 0;
	uint32_t blue = 0;
	uint32_t rmtTaskValue=0;
	while (1) {

		xTaskNotifyAndQuery(xTaskGetCurrentTaskHandle() ,0x00,eNoAction,&rmtTaskValue);
		if (rmtTaskValue % 4) {
			led_strip_pixels[0] = 64; // green	
			led_strip_pixels[1] = 0; // red
			led_strip_pixels[2] = 0; // blue	
		}
		else{
			led_strip_pixels[0] = 0; // green	
			led_strip_pixels[1] = 64; // red
			led_strip_pixels[2] = 0; // green
		}
		ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
		ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
		vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS));
		/* led_strip_pixels[0] = 0; // green */	
		/* led_strip_pixels[1] = 0; // green */	
		/* led_strip_pixels[2] = 0; // green */	
		/* ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config)); */
		/* ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY)); */
		/* vTaskDelay(pdMS_TO_TICKS(EXAMPLE_CHASE_SPEED_MS)); */
	}
}


void initRMT(void)
{
	xTaskCreate(rmt_task, "RMT_task", RMT_TASK_STACK_SIZE, NULL, 1, NULL);
}


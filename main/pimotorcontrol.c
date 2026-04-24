/* 
 * Code for companion controller (AtomS3lite, based on esp32s3) that converts velocity commands into wheel speed controllers
 * Main task watches for data over USB and parses message, then sends computed duty cycles to a PWM task, also displays on WLED
*/ 

//
//	 Used to be ledcontrol.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "esp_now.h"
#include <string.h>

// Communication
#include "driver/usb_serial_jtag.h"
#include "driver/i2c_master.h"

// LEDs
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

// My code
#include "my_espnow.h"
#include "ledc_fade.h"
#include "control_loop.h"
#include "myi2c.h"

#define PMC_TASK_STACK_SIZE (4096)
#define CONTROL_TASK_STACK_SIZE (1024)

// USB config
#define BUF_SIZE (1024)

static const char *TAG = "espnow_example";

// I2C
#include "myi2c.h"

// LED and RMT config
// LED control using RMT example
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution)
#define RMT_LED_STRIP_GPIO_NUM      (35) // On AtomS3lite

#define EXAMPLE_LED_NUMBERS         1
#define EXAMPLE_CHASE_SPEED_MS      100

#define RMT_TASK_STACK_SIZE (2048)
static uint8_t led_strip_pixels[3];


// Watchdog timer 
TimerHandle_t watchdog_timer;
bool is_stopped = true;

// Timer callback - called when timeout expires
void watchdogCallback(TimerHandle_t xTimer) {
	if (!is_stopped) {
		// Send zero motor speed command and set stopped flag
		xTaskNotify(xTaskGetHandle("CONTROL_task"),0,eSetValueWithOverwrite);
		is_stopped = true;
	}
}

// Parses the string and puts the result in the data location
// Used to process commands from Raspberry Pi
void myatoi(const char * passedStr, uint8_t *dataLocation)
{
	*dataLocation = 0;
	for (int i = 1; passedStr[i] != '\0'; ++i)
		*dataLocation = *dataLocation * 10 + passedStr[i] - '0'; // last part uses ASCII math 
}

// For sending commands from esp32s3 to the Serial Monitor for debugging 
void write_int_to_usb(const uint8_t *data_ptr, int len, char *buffer) {
	int a = *data_ptr;
	if (itoa(a, buffer, 10) != NULL) {
		usb_serial_jtag_write_bytes(buffer, len, 20 / portTICK_PERIOD_MS);
	}
}

// Separate initialization code to declutter main a little bit
static void initUSB(usb_serial_jtag_driver_config_t* usb_serial_jtag_config)
{
	usb_serial_jtag_config->rx_buffer_size = BUF_SIZE;
	usb_serial_jtag_config->tx_buffer_size = BUF_SIZE;
	ESP_ERROR_CHECK(usb_serial_jtag_driver_install(usb_serial_jtag_config));
	ESP_LOGI("usb_serial_jtag echo", "USB_SERIAL_JTAG init done");
}

static void initRMT(rmt_channel_handle_t* led_chan, 
		rmt_tx_channel_config_t *tx_chan_config,
		rmt_encoder_handle_t *led_encoder,
		led_strip_encoder_config_t *encoder_config
		)
{	
	tx_chan_config->clk_src = RMT_CLK_SRC_DEFAULT; // select source clock
	tx_chan_config->gpio_num = RMT_LED_STRIP_GPIO_NUM;
	tx_chan_config->mem_block_symbols = 64; // increase the block size can make the LED less flickering
	tx_chan_config->resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ;
	tx_chan_config->trans_queue_depth = 4; // set the number of transactions that can be pending in the background
	encoder_config->resolution = RMT_LED_STRIP_RESOLUTION_HZ;

	rmt_new_tx_channel(tx_chan_config, led_chan);
	rmt_new_led_strip_encoder(encoder_config, led_encoder);
}

// Primary task. the main function calls this task and an LED task which is really the PWM task.
static void pmc_task(void *arg)
{
	// Configure USB SERIAL JTAG
	usb_serial_jtag_driver_config_t usb_serial_jtag_config;
	initUSB(&usb_serial_jtag_config);

    ESP_LOGI("PMC", "Stack HWM after child spawn: %u bytes free", uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t));
	// Configure a temporary buffer for the incoming data
	uint8_t data[64];

	// Config the RMT channel. 
	rmt_channel_handle_t led_chan = NULL;
	rmt_tx_channel_config_t tx_chan_config;
	rmt_encoder_handle_t led_encoder = NULL;
	led_strip_encoder_config_t encoder_config;
	initRMT(&led_chan, &tx_chan_config, &led_encoder, &encoder_config);
	ESP_ERROR_CHECK(rmt_enable(led_chan));
	rmt_transmit_config_t tx_config = {
		.loop_count = 0, // no transfer loop
	};
	// Done with RMT config

	int len = 2;
	uint8_t *data_v = malloc(len); // forward velocity magnitude
	uint8_t *data_w = malloc(len); // angular velocity magnitude
	uint8_t *velSigns = malloc(len); // velocity signs
	uint32_t pmcTaskValue=0; // initialize a store of information from ESPNOW
	*data_v = 0;
	*data_w = 0;
	// Initialize LED to zero
	led_strip_pixels[0]=255; //G
	led_strip_pixels[1]=0; //R
	led_strip_pixels[2]=0; //B
	ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
	// Send 0 LED intensities (blink off)
	vTaskDelay(100);
	led_strip_pixels[0]=0;
	led_strip_pixels[1]=255;
	ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
	char buffer_t[8];
	// We have a state toggle
	bool motor_running = false;
	while (1) {
		// Read the buffer
		int len = usb_serial_jtag_read_bytes(data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
		if (len) { // if buffer has non-zero bytes, enter loop
			data[len] = '\0'; // creates an end bit in the 1024-size buffer
							  // check for stop command
			if (*(data) == 'C' && len > 13 && *(data+1)== '.' && *(data+5)=='.' && *(data+9) == '.' && *(data+13) == '.' ){
				// C.GGG.RRR.BBB
				// 0123456789012
				*(data+1) = '\0'; *(data+5) = '\0'; *(data+9) = '\0';*(data+13) = '\0';
				// Note that myatoi was starting from second character for the prior case where there was a + or - in front of 3-digit number
				myatoi((const char *)(data+1),led_strip_pixels);
				myatoi((const char *)(data+5),led_strip_pixels+1);
				myatoi((const char *)(data+9),led_strip_pixels+2);
				usb_serial_jtag_write_bytes((const char *) (data), 1, 20 / portTICK_PERIOD_MS); // prints C
				char buffer[8];
				// Print to console
				usb_serial_jtag_write_bytes(" ", 1, 20 / portTICK_PERIOD_MS);
				write_int_to_usb(led_strip_pixels, 4,buffer);
				usb_serial_jtag_write_bytes(" ", 1, 20 / portTICK_PERIOD_MS);
				write_int_to_usb(led_strip_pixels+1, 4,buffer);
				usb_serial_jtag_write_bytes(" ", 1, 20 / portTICK_PERIOD_MS);
				write_int_to_usb(led_strip_pixels+2, 4,buffer);
				usb_serial_jtag_write_bytes("\n", 1, 20 / portTICK_PERIOD_MS);
			}
			else if (*(data) == 'I' && len > 2 && *(data+1)== '.' ){
				// In pmc_task loop, when you check pmcTaskValue:
				if (pmcTaskValue & IMU_READY_BIT) {
					imu_data_t imu_snapshot;
					xSemaphoreTake(imu_mutex, portMAX_DELAY);
					memcpy(&imu_snapshot, &latest_imu, sizeof(imu_data_t));
					xSemaphoreGive(imu_mutex);

					// use imu_snapshot.yaw, imu_snapshot.yaw_rate etc.
					char buffer[8];
					ulTaskNotifyValueClear(pmc_handle, IMU_READY_BIT);
				led_strip_pixels[0]= imu_snapshot.yaw >= 0? (uint8_t) imu_snapshot.yaw : 0;
				led_strip_pixels[1]= imu_snapshot.yaw < 0? (uint8_t) (-imu_snapshot.yaw) : 0;

				}
			}
			else if (*(data) == 'G' && len > 13 && *(data+1)== '.' && *(data+5)=='.' && *(data+9) == '.' && *(data+13) == '.' ){
				// Debug mode, for direct command of wheel speeds
				// Unclear where to put this with new control code 4/22/2026
				// C.RRR.GGG.BBB
				// 0123456789012
				*(data+1) = '\0'; *(data+5) = '\0'; *(data+9) = '\0';*(data+13) = '\0';
				// Note that myatoi was starting from second character for the prior case where there was a + or - in front of 3-digit number
				myatoi((const char *)(data+1),led_strip_pixels);
				myatoi((const char *)(data+5),led_strip_pixels+1);
				myatoi((const char *)(data+9),led_strip_pixels+2);
				usb_serial_jtag_write_bytes((const char *) (data), 1, 20 / portTICK_PERIOD_MS); // prints C
				*data_v = led_strip_pixels[0];
				*data_w = led_strip_pixels[1];
				*velSigns = 0; // lowest bit is sign of omega, 2nd lowest is sign of v
				if (*(data+2) == '+') *velSigns = *velSigns | (1 << 1); 
				if (*(data+7) == '+') *velSigns = *velSigns | 1; 
				char buffer[8];
				// Print to console
				usb_serial_jtag_write_bytes(" ", 1, 20 / portTICK_PERIOD_MS);
				write_int_to_usb(led_strip_pixels, 4,buffer);
				usb_serial_jtag_write_bytes(" ", 1, 20 / portTICK_PERIOD_MS);
				write_int_to_usb(led_strip_pixels+1, 4,buffer);
				usb_serial_jtag_write_bytes(" ", 1, 20 / portTICK_PERIOD_MS);
				write_int_to_usb(led_strip_pixels+2, 4,buffer);
				usb_serial_jtag_write_bytes("\n", 1, 20 / portTICK_PERIOD_MS);
			}
			else if (*(data) == 'S' && !motor_running){ // Not that this !motor_running becomes blocking
				led_strip_pixels[0]=255; // Green
				usb_serial_jtag_write_bytes((const char *) (data), 1, 20 / portTICK_PERIOD_MS);
				usb_serial_jtag_write_bytes("\n", 1, 20 / portTICK_PERIOD_MS);
				*(data_v) = 0;
				*(data_w) = 0;
				*velSigns = 1;
				led_strip_pixels[1]=0;
				led_strip_pixels[2]=0;
				motor_running = false; // this step just uses messages to enable and disable motor control 
			}
			// Check for differential speed command
			else if (*(data) == 'D' && len > 13 && *(data+1)== '.' && *(data+6)=='.' && *(data+11) == '.' && motor_running){
				//motor_running = true; // this step just uses messages to enable and disable motor control 
				// define substrings using end character symbol: M.+SSS.+SSS.0 ('.' -> '\0')
				led_strip_pixels[0]=0;
				*(data+1) = '\0'; *(data+6) = '\0'; *(data+11) = '\0';

				// print the substrings on the console
				usb_serial_jtag_write_bytes((const char *) (data), 1, 20 / portTICK_PERIOD_MS); // 'D'
				// convert strings containing |v|,|omega| into integers 
				myatoi((const char *) (data+2),data_v); 
				myatoi((const char *) (data+7),data_w);
				*velSigns = 0; // lowest bit is sign of omega, 2nd lowest is sign of v
				if (*(data+2) == '+') *velSigns = *velSigns | (1 << 1); 
				if (*(data+7) == '+') *velSigns = *velSigns | 1; 

				// Print received values to console
				char buffer[8];  
				write_int_to_usb(data_v, 4,buffer);
				usb_serial_jtag_write_bytes(" ", 1, 20 / portTICK_PERIOD_MS);
				write_int_to_usb(data_w, 4,buffer);
				usb_serial_jtag_write_bytes(" ", 1, 20 / portTICK_PERIOD_MS);


				// Print converted values to console
				write_int_to_usb(data_v, 4,buffer);
				usb_serial_jtag_write_bytes(" ", 1, 20 / portTICK_PERIOD_MS);
				write_int_to_usb(data_w, 4,buffer);
				usb_serial_jtag_write_bytes(" ", 1, 20 / portTICK_PERIOD_MS);
				usb_serial_jtag_write_bytes(" w:", 3, 20 / portTICK_PERIOD_MS);
				write_int_to_usb(velSigns, 8,buffer);
				usb_serial_jtag_write_bytes("\n", 1, 20 / portTICK_PERIOD_MS);
				// Set the LED values to be 
				led_strip_pixels[1]=*(data_v);
				led_strip_pixels[2]=*(data_w);

			}

			// Update LEDs 
			ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
			ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
			// Update the motors
			// reset watchdog timer
			// Send (v , w ,signs) as 8-8-8 chunk
			xTimerReset(watchdog_timer, 0);
			is_stopped = false;

		} 
		xTaskNotifyAndQuery(xTaskGetCurrentTaskHandle() ,0x00,eNoAction,&pmcTaskValue);
		if (pmcTaskValue & 1 ) {
			motor_running = true; // run under CA control
		} else {
				led_strip_pixels[0]=0;
				led_strip_pixels[1]=0;
				led_strip_pixels[2]=0;
			motor_running = false; // We'll give the command
			if (( (pmcTaskValue >> 1) & 1) ) { // Right / Left ?
				led_strip_pixels[0]=255;
				*(data_v) = 00;
				*(data_w) = 200;
				*(velSigns) = 0;
			} else if (( (pmcTaskValue >> 2) & 1) ) { // Forward
				led_strip_pixels[1]=255;
				*(data_v) = 255;
				*(data_w) = 0;
				*(velSigns) = 2;
			} else if (( (pmcTaskValue >> 3) & 1) ) { // Right / Left?
				led_strip_pixels[2]=255;
				*(data_v) = 0;
				*(data_w) = 200;
				*(velSigns) = 1;
			}   else { // stop */
				*(data_v) = 0;
				*(data_w) = 0;
				*(velSigns) = 0;
			}
			xTimerReset(watchdog_timer, 0);
			is_stopped = false;
			// Update LEDs 
			ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
			ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
		}
		xTaskNotify(xTaskGetHandle("CONTROL_task"),(*(data_w) << 8) | (*(data_v) << 16) | *velSigns,eSetValueWithOverwrite);
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}


void app_main(void)
{
	// Set up ESPNOW 
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK( nvs_flash_erase() );
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK( ret );

	set_custom_mac_addr();
	wifi_init_sta();

	print_mac_addr();
	xTaskCreate(control_task, "CONTROL_task", 2048, NULL, 1, NULL);
	xTaskCreate(ledc_task, "LEDC_task", 3076, NULL, 1, NULL); // In reality, the PWM task
	xTaskNotify(xTaskGetHandle("LEDC_task"),0,eSetValueWithOverwrite); // Wheel speeds are zero
	// Create one-shot timer (200ms). Seems reasonable that LEDC_task should exist first
	watchdog_timer = xTimerCreate(
			"Watchdog",           // Name
			pdMS_TO_TICKS(200),   // Period (200ms)
			pdFALSE,              // Auto-reload (false = one-shot)
			(void*)0,             // Timer ID
			watchdogCallback      // Callback function
			);
	// Pi Motor Controller task. 
	TaskHandle_t pmc_handle;
	xTaskCreate(pmc_task, "PMC_task", 6144, NULL, 1, &pmc_handle);
	// Has to be after?: 
	espnow_set_pmc_handle(pmc_handle);
	example_espnow_init();  
	i2c_set_pmc_handle(pmc_handle);
	initI2Ctask();
	ESP_LOGI("MEM", "Free heap after I2C with IMU: %lu", esp_get_free_heap_size());

}

//	 Used to be ledcontrol.c
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "nvs_flash.h"
#include "esp_now.h"

// Communication
#include "driver/usb_serial_jtag.h"
#include "driver/i2c_master.h"

// LEDs
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"

// My code
/* #include "my_espnow.h" */
#include "ledc_fade.h"
#include "pimotorcontrol.h"

#define PMC_TASK_STACK_SIZE (4096)

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
		xTaskNotify(xTaskGetHandle("LEDC_task"),0,eSetValueWithOverwrite);
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

// Converts the body-velocity commands into  +/- wheel speeds through PWM duty cycle and polarity change for DRV8833
void convertToWheels(const char * passedStr, uint8_t *dataV, uint8_t *dataW, uint8_t *dataR, uint8_t *dataL, uint8_t *wheelSigns)
{
	// set the wheel speeds using magnitudes in *dataV, *dataW and their signs in passedStr

	uint8_t delta;
	delta = 255-*dataW; // By design, dataW <= 255
	if (delta > *dataV){
		delta = *dataV; 
	} 
	// delta = min (v, 255-omega) is the above code is uncommented. LiDAR worked with uncommented, RS with commented
	*wheelSigns=0;
	if (passedStr[7] == '+'){
		*dataR = *dataW+delta; // w + v, both positive, sum <= 255 by design
		*wheelSigns = *wheelSigns | (1 << 1); 
		if (*dataW > delta){ // except left wheel to be negative
			*dataL = *dataW-delta; // -(w - v) = v-w
		} else {
			*dataL = delta-*dataW; // -(w - v) = v-w
			*wheelSigns = *wheelSigns | 1; 
		}
	}
	else{
		*dataL = *dataW+delta; // w + v, both positive, sum <= 255 by design
		*wheelSigns = *wheelSigns | 1; 
		if (*dataW > delta){
			*dataR = *dataW-delta; // -(w - v) = v-w
		} else {
			*dataR = delta-*dataW; // -(w - v) = v-w
			*wheelSigns = *wheelSigns | (1 << 1); 
		}
	}
	// Remove deadzone
	/* *dataL = ((*dataL*6)/10)+100; */
	/* *dataR = ((*dataR*6)/10)+100; */
	*dataL = (*dataL/2)+120;
	*dataR = (*dataR/2)+120;

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

	// Configure a temporary buffer for the incoming data
	uint8_t data[64];

	// Configure the master i2c bus
	//i2c_master_bus_config_t i2c_bus_config;
	//i2c_master_bus_handle_t bus_handle;
	//i2c_device_config_t i2c_dev_cfg;
	//i2c_master_dev_handle_t dev_handle;
	//initI2C(&i2c_bus_config, &bus_handle, &i2c_dev_cfg ,&dev_handle);
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


	//ESP_ERROR_CHECK(i2c_master_probe(bus_handle, 0x38, 1000));
	//ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, data_wr, len, 100));
	//ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, data_wr, len, data_rd,4,-1));

	int len = 2;
	uint8_t *data_rw = malloc(len + 1); // right wheel
	uint8_t *data_lw = malloc(len + 1); // left wheel
	uint8_t *data_v = malloc(len); // right wheel
	uint8_t *data_w = malloc(len); // left wheel
	uint8_t *wheelSigns = malloc(len); // left wheel
	uint32_t pmcTaskValue=0; // initialize a store of information
							 // This 0x21 is outdated, from when we used I2C commands instead of a direct PWM
	data_lw[0] = 0x21; // left wheel connects to motor 2 on AtomicMotionBase. 
	data_rw[0] = 0x20; // right wheel connects to motor 1 on AtomicMotionBase
	data_rw[1] = 0x00;
	data_lw[1] = 0x00;
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
	// We have a state toggle
	char buffer_t[8];
	while (1) {
		int len = usb_serial_jtag_read_bytes(data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
		/* xTaskNotifyAndQuery(xTaskGetCurrentTaskHandle() ,0x00,eNoAction,&pmcTaskValue); */
		/* led_strip_pixels[0] = (pmcTaskValue & 255); */
		/* led_strip_pixels[1] = ((pmcTaskValue >> 8) & 255); */
		/* led_strip_pixels[2] = ((pmcTaskValue >> 16) & 255); */
		/* usb_serial_jtag_write_bytes("R: ", 3, 20 / portTICK_PERIOD_MS); */
		/* write_int_to_usb(led_strip_pixels, 4,buffer_t); */
		/* usb_serial_jtag_write_bytes(" ", 1, 20 / portTICK_PERIOD_MS); */
		/* write_int_to_usb(led_strip_pixels+1, 4,buffer_t); */
		/* usb_serial_jtag_write_bytes(" ", 1, 20 / portTICK_PERIOD_MS); */
		/* write_int_to_usb(led_strip_pixels+2, 4,buffer_t); */
		/* usb_serial_jtag_write_bytes("\n", 1, 20 / portTICK_PERIOD_MS); */
		/* ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config)); */
		/* ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY)); */
		if (len) {
			data[len] = '\0'; // creates an end bit in the 1024-size buffer
							  // check for stop command
			if (*(data) == 'C' && len > 13 && *(data+1)== '.' && *(data+5)=='.' && *(data+9) == '.' && *(data+13) == '.' ){
				// C.RRR.GGG.BBB
				// 0123456789012
				*(data+1) = '\0'; *(data+5) = '\0'; *(data+9) = '\0';*(data+13) = '\0';
				// Note that myatoi was starting from second character for the prior case where there was a + or - in front of 3-digit number
				myatoi((const char *)(data+1),led_strip_pixels);
				myatoi((const char *)(data+5),led_strip_pixels+1);
				myatoi((const char *)(data+9),led_strip_pixels+2);
				usb_serial_jtag_write_bytes((const char *) (data), 1, 20 / portTICK_PERIOD_MS); // prints C
				*(data_rw+1) = 0;
				*(data_lw+1) = 0;
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
			if (*(data) == 'S'){
				led_strip_pixels[0]=255;
				usb_serial_jtag_write_bytes((const char *) (data), 1, 20 / portTICK_PERIOD_MS);
				usb_serial_jtag_write_bytes("\n", 1, 20 / portTICK_PERIOD_MS);
				*(data_rw+1) = 0;
				*(data_lw+1) = 0;
				led_strip_pixels[1]=0;
				led_strip_pixels[2]=0;
			}
			// Check for differential speed command
			if (*(data) == 'D' && len > 13 && *(data+1)== '.' && *(data+6)=='.' && *(data+11) == '.'){
				// define substrings using end character symbol: M.+SSS.+SSS.0 ('.' -> '\0')
				led_strip_pixels[0]=0;
				*(data+1) = '\0'; *(data+6) = '\0'; *(data+11) = '\0';

				// print the substrings on the console
				usb_serial_jtag_write_bytes((const char *) (data), 1, 20 / portTICK_PERIOD_MS); // 'D'
																								// convert strings containing |v|,|omega| into integers 
				myatoi((const char *) (data+2),data_v); 
				myatoi((const char *) (data+7),data_w);

				// Print received values to console
				char buffer[8];  
				write_int_to_usb(data_v, 4,buffer);
				usb_serial_jtag_write_bytes(" ", 1, 20 / portTICK_PERIOD_MS);
				write_int_to_usb(data_w, 4,buffer);
				usb_serial_jtag_write_bytes(" ", 1, 20 / portTICK_PERIOD_MS);

				// Now we convert v,w into phiR, phiL
				convertToWheels((const char *) data,data_v,data_w,data_rw+1,data_lw+1,wheelSigns);

				// Print converted values to console
				write_int_to_usb(data_rw+1, 4,buffer);
				usb_serial_jtag_write_bytes(" ", 1, 20 / portTICK_PERIOD_MS);
				write_int_to_usb(data_lw+1, 4,buffer);
				usb_serial_jtag_write_bytes(" ", 1, 20 / portTICK_PERIOD_MS);
				usb_serial_jtag_write_bytes(" w:", 3, 20 / portTICK_PERIOD_MS);
				write_int_to_usb(wheelSigns, 8,buffer);
				usb_serial_jtag_write_bytes("\n", 1, 20 / portTICK_PERIOD_MS);
				// Set the LED values to be 
				led_strip_pixels[1]=*(data_rw+1);
				led_strip_pixels[2]=*(data_lw+1);

			}

			// Update LEDs 
			ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
			ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
			// Update the motors
			// NO MECHANISM RIGHT NOW
			// Perhaps you want to notify the ledc fade task
			xTaskNotify(xTaskGetHandle("LEDC_task"),(*(data_rw+1) << 8) | (*(data_lw+1) << 16) | *wheelSigns,eSetValueWithOverwrite);
			// reset watchdog timer
			xTimerReset(watchdog_timer, 0);
			is_stopped = false;

		}
		/* xTaskNotifyAndQuery(xTaskGetCurrentTaskHandle() ,0x00,eNoAction,&pmcTaskValue); */
	}
}


void app_main(void)
{

	xTaskCreate(ledc_task, "LEDC_task", 3072, NULL, 1, NULL); // In reality, the PWM task
	xTaskNotify(xTaskGetHandle("LEDC_task"),0,eSetValueWithOverwrite);
	// Create one-shot timer (100ms). Seems reasonable that LEDC_task should exist first
	watchdog_timer = xTimerCreate(
			"Watchdog",           // Name
			pdMS_TO_TICKS(200),   // Period (100ms)
			pdFALSE,              // Auto-reload (false = one-shot)
			(void*)0,             // Timer ID
			watchdogCallback      // Callback function
			);
	// Pi Motor Controller task. 
	xTaskCreate(pmc_task, "PMC_task", 3072, NULL, 1, NULL);
}

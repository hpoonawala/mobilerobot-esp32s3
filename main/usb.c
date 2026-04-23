#include "driver/usb_serial_jtag.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "usb.h"
#define BUF_SIZE (1024)
#define USB_TASK_STACK_SIZE (4096)

static void usb_task(void *arg)
{
	// Configure USB SERIAL JTAG
	usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
		.rx_buffer_size = BUF_SIZE,
		.tx_buffer_size = BUF_SIZE,
	};

	ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
	ESP_LOGI("usb_serial_jtag echo", "USB_SERIAL_JTAG init done");

	// Configure a temporary buffer for the incoming data
	uint8_t *data = (uint8_t *) malloc(BUF_SIZE);
	if (data == NULL) {
		ESP_LOGE("usb_serial_jtag echo", "no memory for data");
		return;
	}
	while (1) {

		int len = usb_serial_jtag_read_bytes(data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
		// Write data back to the USB SERIAL JTAG
		if (len) {
			/* xTaskNotify(xTaskGetHandle("RMT_task"),0x00,eIncrement); */
			data[len] = '\0'; // creates an end bit in the 1024-size buffer
			usb_serial_jtag_write_bytes((const char *) data, len, 20 / portTICK_PERIOD_MS);
			if (*(data) == 'D'){
				xTaskNotify(xTaskGetHandle("RMT_task"),0x00,eIncrement); // affect RGBLED parameters
				xTaskNotify(xTaskGetHandle("RMT_task"),0x00,eIncrement); // affect RGBLED parameters
			}
		}
	}
}

void initUSB(void)
{
	xTaskCreate(usb_task, "USB_task", USB_TASK_STACK_SIZE, NULL, 10, NULL);
}

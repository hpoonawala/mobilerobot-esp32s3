#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "ledc_fade.h"
#include "control_loop.h"


QueueHandle_t wheel_cmd_queue;

void convertToWheels(uint8_t *dataV, uint8_t *dataW, uint8_t *velSigns,
		int16_t *signedR, int16_t *signedL)
{
	if ((*dataV==0) && (*dataW == 0)) {
		*signedR = 0;
		*signedL = 0;
		return;
	}
	// project V into the "nullspace" of omega: Without this step, saturation of wheel velocities prevents rotation
    uint8_t delta = 255 - *dataW;
    if (delta > *dataV) delta = *dataV;

    int16_t v = (*velSigns & 2) ? (int16_t) delta : -(int16_t) delta;
    int16_t w = (*velSigns & 1) ? (int16_t) *dataW : -(int16_t) *dataW;


    *signedR = v + w;
    *signedL = v - w ;

}

void control_task(void *arg)
{
    wheel_cmd_queue = xQueueCreate(1, sizeof(wheel_command_t));

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    uint8_t *data_v   = malloc(2);
    uint8_t *data_w   = malloc(2);
    uint8_t *velSigns = malloc(2);

    int16_t signed_r = 0, signed_l = 0;
    int16_t smoothed_r = 0, smoothed_l = 0;

    uint32_t controlTaskValue = 0;

    while (1) {
        xTaskNotifyAndQuery(xTaskGetCurrentTaskHandle(), 0x00, eNoAction, &controlTaskValue);

        *data_v   = (controlTaskValue >> 16) & 255;
        *data_w   = (controlTaskValue >> 8)  & 255;
        *velSigns =  controlTaskValue        & 255;

        convertToWheels(data_v, data_w, velSigns, &signed_r, &signed_l);

        // Filter on signed values — direction reversals ramp through zero
        smoothed_l = (signed_l + 7 * smoothed_l) >> 3;
        smoothed_r = (signed_r + 7 * smoothed_r) >> 3;

        wheel_command_t cmd = {
            .left  = smoothed_l,
            .right = smoothed_r,
        };
        xQueueOverwrite(wheel_cmd_queue, &cmd);

        vTaskDelay(15 / portTICK_PERIOD_MS);
    }
}

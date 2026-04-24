#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "ledc_fade.h"
#include "control_loop.h"
#include "imu_data.h"


QueueHandle_t wheel_cmd_queue;

void convertToWheels(uint8_t *dataV, uint8_t *dataW, uint8_t *velSigns,
		int16_t *signedR, int16_t *signedL, int16_t *gz_error, float *gz)
{
	if ((*dataV==0) && (*dataW == 0)) {
		*signedR = 0;
		*signedL = 0;
		*gz_error = 0;
		return;
	}
    int16_t w = (*velSigns & 1) ? (int16_t) *dataW : -(int16_t) *dataW;
	// By adding gz_error, we may be breaking things.
	int16_t error = (int16_t)(0.001f*(10.0f* *gz - (float)w)); // 0.1 * (converted yaw rate - target)
	// Integrate 
	*gz_error+= error;
	*gz_error = *gz_error >  255 ?  255 : *gz_error;
	*gz_error = *gz_error < -255 ? -255 : *gz_error;
	// Add integral term tp command
	w += *gz_error;
	w = w >= 180 ? 180 : w;
	w = w <= -180 ? -180 : w;

	// w is now the commanded value plus an adapted term based on the error from measured ang vel
	// Saturated to lie between +- 255, making it like *dataW. 

	// Saturate the angular velocity command
	// project V into the "nullspace" of omega: Without this step, saturation of wheel velocities prevents rotation
	uint8_t cmdW = w > 0 ? w : -w;
    uint8_t delta = 255 - cmdW;
    if (delta > *dataV) delta = *dataV;
	// So far we were guaranteed that   |v+w| and |v-w| were between 0 and 255. This may not be true anymore

    int16_t v = (*velSigns & 2) ? (int16_t) delta : -(int16_t) delta;
	// If v can only be positive, we are likely still ensuring that |v+w| and |v-w| were between 0 and 255.


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
	int16_t gz_error = 0;

    while (1) {
		float gz = 0.0f;
		if (imu_mutex != NULL) {
			xSemaphoreTake(imu_mutex, portMAX_DELAY);
			gz = latest_imu.yaw_rate;
			xSemaphoreGive(imu_mutex);
		}

		// The error in angular velocity is K*gz - *dataW. We assume that when *dataW = 150, we want, say, 30 degrees per second. 
		// Then, K maps actual reading gz = (30 deg per second in units) -> 150, meaning no error. 
		// Now we want to boost the PWM difference to increase the torque until the error is zero? What's the model here? Sounds like an integral control
		// Nominrally, *cmdW = *dataW, but we won't get the rate we want, so then *cmdW += error, so when error = 0 we have found the command that produces gz
		//
		*data_v   = (controlTaskValue >> 16) & 255;
		*data_w   = (controlTaskValue >> 8)  & 255;
		*velSigns =  controlTaskValue        & 255;

        convertToWheels(data_v, data_w, velSigns, &signed_r, &signed_l, &gz_error, &gz);

		xTaskNotifyAndQuery(xTaskGetCurrentTaskHandle(), 0x00, eNoAction, &controlTaskValue);
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

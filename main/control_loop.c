//New control task 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

// My code
#include "ledc_fade.h"
#include "control_loop.h"



// Converts the body-velocity commands into  +/- wheel speeds through PWM duty cycle and polarity change for DRV8833
void convertToWheels( uint8_t *dataV, uint8_t *dataW,  uint8_t *velSigns,uint8_t *dataR, uint8_t *dataL, uint8_t *wheelSigns)
{
	// set the wheel speeds using magnitudes in *dataV, *dataW and their signs in passedStr

	uint8_t delta;
	delta = 255-*dataW; // By design, dataW <= 255
	if (delta > *dataV){
		delta = *dataV; 
	} 
	// delta = min (v, 255-omega) is the above code is uncommented. LiDAR worked with uncommented, RS with commented
	*wheelSigns=0;
	if ( *velSigns & 1){ // positive omega when true
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
	// Deadzone handling done in LEDC FADE task 
	// Can do it here if we switch to Queue-based comms and send more than 32 bits
}


void control_task(void *arg){
	vTaskDelay(100/ portTICK_PERIOD_MS);
	int len=2;
	uint8_t *data_v = malloc(len); // forward velocity magnitude
	uint8_t *data_w = malloc(len); // angular velocity magnitude
	uint8_t *velSigns = malloc(len); // velocity signs
	uint8_t *data_rw = malloc(len); // right wheel
	uint8_t *data_lw = malloc(len); // left wheel
	uint8_t *wheelSigns = malloc(len); // left wheel
	uint8_t smoothed_left_wheel_speed = 0;
	uint8_t smoothed_right_wheel_speed = 0;
	uint32_t controlTaskValue=0; // initialize
	while(1){
		xTaskNotifyAndQuery(xTaskGetCurrentTaskHandle() ,0x00,eNoAction,&controlTaskValue);
		*data_v = (controlTaskValue >> 16) & 255;
		*data_w = (controlTaskValue >> 8) & 255;
		*velSigns = controlTaskValue & 255;
		// Now we convert v,w into phiR, phiL
		convertToWheels(data_v,data_w,velSigns,data_rw,data_lw,wheelSigns);
		smoothed_left_wheel_speed  =  ( (*(data_lw)  + 7 * (uint16_t)smoothed_left_wheel_speed)  >> 3 );
		smoothed_right_wheel_speed  = (  (*(data_rw)  + 7 * (uint16_t)smoothed_right_wheel_speed)  >> 3);

		xTaskNotify(xTaskGetHandle("LEDC_task"),(smoothed_right_wheel_speed << 8) | (smoothed_left_wheel_speed << 16) | *wheelSigns,eSetValueWithOverwrite);

		vTaskDelay(10/ portTICK_PERIOD_MS);
	}
}

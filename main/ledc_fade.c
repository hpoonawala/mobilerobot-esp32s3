/* LEDC (LED Controller) fade example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_err.h"

/* PLAN: recreate gpio task loop here, using fade functions instead of gpio_set functions */
/* #include "ledc_fade.h" */
/* #include "gpio.h" */
/*
 * About this example
 *
 * 1. Start with initializing LEDC module:
 *    a. Set the timer of LEDC first, this determines the frequency
 *       and resolution of PWM.
 *    b. Then set the LEDC channel you want to use,
 *       and bind with one of the timers.
 *
 * 2. You need first to install a default fade function,
 *    then you can use fade APIs.
 *
 * 3. You can also set a target duty directly without fading.
 *
 * 4. On ESP32, GPIO18/19/4/5 are used as the LEDC outputs:
 *              GPIO18/19 are from the high speed channel group
 *              GPIO4/5 are from the low speed channel group
 *
 *    On other targets, GPIO8/9/4/5 are used as the LEDC outputs,
 *    and they are all from the low speed channel group.
 *
 * 5. All the LEDC outputs change the duty repeatedly.
 *
 */
#if CONFIG_IDF_TARGET_ESP32
#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (6)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_HS_CH1_GPIO       (5)
#define LEDC_HS_CH1_CHANNEL    LEDC_CHANNEL_1
#endif
#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#if !CONFIG_IDF_TARGET_ESP32
#define LEDC_LS_CH0_GPIO       (6)
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LEDC_LS_CH1_GPIO       (5)
#define LEDC_LS_CH1_CHANNEL    LEDC_CHANNEL_1
#endif
#define LEDC_LS_CH2_GPIO       (7)
#define LEDC_LS_CH2_CHANNEL    LEDC_CHANNEL_2
#define LEDC_LS_CH3_GPIO       (8)
#define LEDC_LS_CH3_CHANNEL    LEDC_CHANNEL_3

#define LEDC_TEST_CH_NUM       (4)
#define LEDC_TEST_DUTY         (1000)
#define LEDC_TEST_FADE_TIME    (50)


#define GPIO_OUTPUT_IO_3     GPIO_NUM_39 // Sleep
#define GPIO_OUTPUT_IO_6     GPIO_NUM_35 // LED
#define GPIO_OUTPUT_PIN_SEL   ((1ULL<<GPIO_OUTPUT_IO_3) | (1ULL << GPIO_OUTPUT_IO_6))
/*
 * This callback function will be called when fade operation has ended
 * Use callback only if you are aware it is being called inside an ISR
 * Otherwise, you can use a semaphore to unblock tasks
 */
static IRAM_ATTR bool cb_ledc_fade_end_event(const ledc_cb_param_t *param, void *user_arg)
{
    BaseType_t taskAwoken = pdFALSE;

    if (param->event == LEDC_FADE_END_EVT) {
        SemaphoreHandle_t counting_sem = (SemaphoreHandle_t) user_arg;
        xSemaphoreGiveFromISR(counting_sem, &taskAwoken);
    }

    return (taskAwoken == pdTRUE);
}

void ledc_task(void)
{
    int ch;

	//zero-initialize the config structure.
	gpio_config_t io_conf = {};

	//disable interrupt
	io_conf.intr_type = GPIO_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set,e.g.GPIO18/19
	io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
	//disable pull-down mode
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf);
	gpio_set_level(GPIO_OUTPUT_IO_3, 0); // Sleep
										 //

	for(int jj=0;jj<3;jj++){																	//
		gpio_set_level(GPIO_NUM_35, 1); // Bin1
		vTaskDelay(300 / portTICK_PERIOD_MS);
		gpio_set_level(GPIO_NUM_35, 0); // Bin1
		vTaskDelay(300 / portTICK_PERIOD_MS);
	}
    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT, // resolution of PWM duty
        .freq_hz = 60000,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);
#ifdef CONFIG_IDF_TARGET_ESP32
    // Prepare and set configuration of timer1 for low speed channels
    ledc_timer.speed_mode = LEDC_HS_MODE;
    ledc_timer.timer_num = LEDC_HS_TIMER;
    ledc_timer_config(&ledc_timer);
#endif
    /*
     * Prepare individual configuration
     * for each channel of LED Controller
     * by selecting:
     * - controller's channel number
     * - output duty cycle, set initially to 0
     * - GPIO number where LED is connected to
     * - speed mode, either high or low
     * - timer servicing selected channel
     *   Note: if different channels use one timer,
     *         then frequency and bit_num of these channels
     *         will be the same
     */
    ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
#if CONFIG_IDF_TARGET_ESP32
        {
            .channel    = LEDC_HS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH0_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER,
            .flags.output_invert = 0
        },
        {
            .channel    = LEDC_HS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_HS_CH1_GPIO,
            .speed_mode = LEDC_HS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_HS_TIMER,
            .flags.output_invert = 0
        },
#else
        {
            .channel    = LEDC_LS_CH0_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH0_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
        {
            .channel    = LEDC_LS_CH1_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH1_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
#endif
        {
            .channel    = LEDC_LS_CH2_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH2_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
        {
            .channel    = LEDC_LS_CH3_CHANNEL,
            .duty       = 0,
            .gpio_num   = LEDC_LS_CH3_GPIO,
            .speed_mode = LEDC_LS_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_LS_TIMER,
            .flags.output_invert = 0
        },
    };

    // Set LED Controller with previously prepared configuration
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Initialize fade service.
    ledc_fade_func_install(0);
    ledc_cbs_t callbacks = {
        .fade_cb = cb_ledc_fade_end_event
    };
	// Create a counting semaphore
    SemaphoreHandle_t counting_sem = xSemaphoreCreateCounting(LEDC_TEST_CH_NUM, 0);
	// Send that semaphore to the callback for each channel
    for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_cb_register(ledc_channel[ch].speed_mode, ledc_channel[ch].channel, &callbacks, (void *) counting_sem);
    }
	uint32_t ledcTaskValue;
    while (1) {
		xTaskNotifyAndQuery(xTaskGetCurrentTaskHandle() ,0x00,eNoAction,&ledcTaskValue);
		// Expecting to have the 24-17th bit as left wheel, 16-9th as right wheel, and 8th-1st bit as mode or sign information
		// You could make the code far more compact by changing the channel index using the sign bit
		// You need a time out condition to stop motors
		
		gpio_set_level(GPIO_OUTPUT_IO_3, 1); // Sleep
		// Set Bin 2
		ledc_set_duty(ledc_channel[1-(ledcTaskValue & 1)].speed_mode, ledc_channel[1-(ledcTaskValue & 1)].channel, 0);
		ledc_update_duty(ledc_channel[1-(ledcTaskValue & 1)].speed_mode, ledc_channel[1-(ledcTaskValue & 1)].channel);
		// Set Bin 1
		ledc_set_fade_with_time(ledc_channel[ledcTaskValue & 1].speed_mode, ledc_channel[ledcTaskValue & 1].channel,( (ledcTaskValue >> 16) & 255)+500, LEDC_TEST_FADE_TIME);
		ledc_fade_start(ledc_channel[ledcTaskValue & 1].speed_mode, ledc_channel[ledcTaskValue & 1].channel, LEDC_FADE_NO_WAIT);

		// Set Ain 2
		ledc_set_duty(ledc_channel[3-((ledcTaskValue >> 1) & 1)].speed_mode, ledc_channel[3-((ledcTaskValue >> 1) & 1)].channel, 0);
		ledc_update_duty(ledc_channel[3-((ledcTaskValue >> 1) & 1)].speed_mode, ledc_channel[3-((ledcTaskValue >> 1) & 1)].channel);
		// Set Ain 1
		ledc_set_fade_with_time(ledc_channel[2+((ledcTaskValue >> 1) & 1)].speed_mode, ledc_channel[2+((ledcTaskValue >> 1) & 1)].channel,( (ledcTaskValue >> 8) & 255)+500, LEDC_TEST_FADE_TIME);
		ledc_fade_start(ledc_channel[2+((ledcTaskValue >> 1) & 1)].speed_mode, ledc_channel[2+((ledcTaskValue >> 1) & 1)].channel, LEDC_FADE_NO_WAIT);

        for (int i = 0; i < 2; i++) {
            xSemaphoreTake(counting_sem, portMAX_DELAY);
        }

    }
}

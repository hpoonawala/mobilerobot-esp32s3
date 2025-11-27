#ifndef LED_H
#define LED_H
#include "driver/ledc.h"

IRAM_ATTR bool cb_ledc_fade_end_event(const ledc_cb_param_t *param, void *user_arg);
void ledc_task(void);
#endif

#ifndef CONTROL_LOOP_H
#define CONTROL_LOOP_H

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <stdint.h>

typedef struct {
    int16_t left;   // signed, negative = reverse
    int16_t right;  // signed, negative = reverse
} wheel_command_t;

extern QueueHandle_t wheel_cmd_queue;

void control_task(void *arg);

#endif


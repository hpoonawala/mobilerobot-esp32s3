#ifndef IMU_DATA_H
#define IMU_DATA_H
#include "freertos/semphr.h"

typedef struct {
    float yaw;
    float yaw_rate;  // gz in deg/s
    float roll;
    float pitch;
} imu_data_t;

extern imu_data_t latest_imu;
extern SemaphoreHandle_t imu_mutex;

#define IMU_READY_BIT (1 << 8)  //
								//
#endif


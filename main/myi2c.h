#ifndef MYI2C_H
#define MYI2C_H
void initI2Ctask(void);
void myi2c_task(void *arg);
void i2c_set_pmc_handle(TaskHandle_t pmc_handle);
void initI2C(i2c_master_bus_config_t*, 
		i2c_master_bus_handle_t *,
		i2c_device_config_t*,
		i2c_master_dev_handle_t*
		);
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

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
#endif

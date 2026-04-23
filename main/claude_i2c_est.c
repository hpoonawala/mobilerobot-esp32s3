#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "myi2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include <math.h>

// I2C config
#define I2C_TOOL_TIMEOUT_VALUE_MS (50)
#define I2C_TASK_STACK_SIZE (4096)
#define MPU6050_ADDR 0x68
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1 0x6B

// Sensor scaling factors
#define ACCEL_SCALE 16384.0f  // LSB/g for ±2g range
#define GYRO_SCALE 131.0f     // LSB/(deg/s) for ±250deg/s range
#define DEG_TO_RAD 0.017453292519943295f

static uint32_t i2c_frequency = 100 * 1000;
static const char *TAG = "i2c-tools";
static gpio_num_t i2c_gpio_sda = 2;
static gpio_num_t i2c_gpio_scl = 1;
static i2c_port_t i2c_port = I2C_NUM_0;

// Complementary filter state
typedef struct {
    float roll;   // Rotation around X-axis (degrees)
    float pitch;  // Rotation around Y-axis (degrees)
    float yaw;    // Rotation around Z-axis (degrees)
} orientation_t;

static orientation_t orientation = {0.0f, 0.0f, 0.0f};
static const float ALPHA = 0.98f;  // Complementary filter coefficient (0.96-0.98 typical)

void initI2C(i2c_master_bus_config_t* i2c_bus_config, 
        i2c_master_bus_handle_t *bus_handle,
        i2c_device_config_t* i2c_dev_cfg,
        i2c_master_dev_handle_t* dev_handle)
{
    i2c_bus_config->clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_bus_config->i2c_port = i2c_port;
    i2c_bus_config->scl_io_num = i2c_gpio_scl;
    i2c_bus_config->sda_io_num = i2c_gpio_sda;
    i2c_bus_config->glitch_ignore_cnt = 7;
    i2c_bus_config->flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(i2c_bus_config, bus_handle));
    
    i2c_dev_cfg->dev_addr_length = I2C_ADDR_BIT_LEN_7;
    i2c_dev_cfg->scl_speed_hz = i2c_frequency;
    i2c_dev_cfg->device_address = MPU6050_ADDR;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, i2c_dev_cfg, dev_handle));
}


// Complementary filter for orientation estimation
void update_orientation(float ax, float ay, float az, 
                       float gx, float gy, float gz, 
                       float dt)
{
    // Calculate roll and pitch from accelerometer (in degrees)
    float accel_roll = atan2f(ay, sqrtf(ax*ax + az*az)) * 180.0f / M_PI;
    float accel_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / M_PI;
    
    // Integrate gyroscope data (gyro is in deg/s, multiply by dt to get degrees)
    float gyro_roll = orientation.roll + gx * dt;
    float gyro_pitch = orientation.pitch + gy * dt;
    float gyro_yaw = orientation.yaw + gz * dt;
    
    // Complementary filter: trust gyro for short term, accel for long term
    orientation.roll = ALPHA * gyro_roll + (1.0f - ALPHA) * accel_roll;
    orientation.pitch = ALPHA * gyro_pitch + (1.0f - ALPHA) * accel_pitch;
    orientation.yaw = gyro_yaw;  // Yaw comes only from gyro (no magnetometer)
    
    // Keep yaw in 0-360 range
    if (orientation.yaw > 180.0f) orientation.yaw -= 360.0f;
    if (orientation.yaw < -180.0f) orientation.yaw += 360.0f;
}

void myi2c_task(void *arg)
{
	uint8_t data_rd[14];
	// Configure the master bus
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port,
        .scl_io_num = i2c_gpio_scl,
        .sda_io_num = i2c_gpio_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));
    
    // Probe for MPU6050
    esp_err_t ret = i2c_master_probe(bus_handle, MPU6050_ADDR, 1000);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 not found at address 0x%02X", MPU6050_ADDR);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "MPU6050 found at address 0x%02X", MPU6050_ADDR);
    
    // Configure device
    i2c_device_config_t i2c_dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz = i2c_frequency,
        .device_address = MPU6050_ADDR,
    };
    
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &i2c_dev_cfg, &dev_handle));
    
    // Wake up MPU6050
    uint8_t wake_cmd[2] = {MPU6050_REG_PWR_MGMT_1, 0x00};
    ret = i2c_master_transmit(dev_handle, wake_cmd, 2, 1000 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake MPU6050: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "MPU6050 woken up");
    }
    
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    uint8_t reg_addr = MPU6050_REG_ACCEL_XOUT_H;
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t sample_period = pdMS_TO_TICKS(10);  // 100 Hz sampling
    float dt = 0.01f;  // 10ms in seconds
	TickType_t now = xTaskGetTickCount();
    
    while(1) {
        // Read 14 bytes starting from ACCEL_XOUT_H register
        ret = i2c_master_transmit_receive(dev_handle, 
                                          &reg_addr, 1,
                                          data_rd, 14,
                                          1000 / portTICK_PERIOD_MS);
        
        if (ret == ESP_OK) {
            // Extract raw sensor readings
            int16_t accel_x_raw = (data_rd[0] << 8) | data_rd[1];
            int16_t accel_y_raw = (data_rd[2] << 8) | data_rd[3];
            int16_t accel_z_raw = (data_rd[4] << 8) | data_rd[5];
            int16_t temp_raw = (data_rd[6] << 8) | data_rd[7];
            int16_t gyro_x_raw = (data_rd[8] << 8) | data_rd[9];
            int16_t gyro_y_raw = (data_rd[10] << 8) | data_rd[11];
            int16_t gyro_z_raw = (data_rd[12] << 8) | data_rd[13];
            
            // Convert to physical units
            float ax = accel_x_raw / ACCEL_SCALE;  // g
            float ay = accel_y_raw / ACCEL_SCALE;
            float az = accel_z_raw / ACCEL_SCALE;
            float gx = gyro_x_raw / GYRO_SCALE;    // deg/s
            float gy = gyro_y_raw / GYRO_SCALE;
            float gz = gyro_z_raw / GYRO_SCALE;
            float temperature = (temp_raw / 340.0f) + 36.53f;
			now = xTaskGetTickCount();
			dt = (now - last_wake_time) / (float)configTICK_RATE_HZ;
			last_wake_time = now;
            // Update orientation estimate
            update_orientation(ax, ay, az, gx, gy, gz, dt);
            
            // Log orientation every 10 samples (1 Hz)
            static int log_counter = 0;
            if (++log_counter >= 10) {
                ESP_LOGI(TAG, "Orientation: Roll=%.1f° Pitch=%.1f° Yaw=%.1f° | Temp=%.1f°C", 
                         orientation.roll, orientation.pitch, orientation.yaw, temperature);
                log_counter = 0;
            }
            
            // Send notification to PMC task (optional - include orientation data)
            TaskHandle_t pmc_task = xTaskGetHandle("PMC_task");
            if (pmc_task != NULL) {
                // Pack roll and pitch into notification (scaled to fit in uint32_t)
                int16_t roll_int = (int16_t)(orientation.roll * 10);
                int16_t pitch_int = (int16_t)(orientation.pitch * 10);
                uint32_t notify_value = (roll_int & 0xFFFF) | ((pitch_int & 0xFFFF) << 16);
                xTaskNotify(pmc_task, notify_value, eSetValueWithOverwrite);
            }
        } else {
            ESP_LOGE(TAG, "Failed to read from MPU6050: %s", esp_err_to_name(ret));
        }
        
        vTaskDelayUntil(&last_wake_time, sample_period);
    }
}

void initI2Ctask(void)
{
    xTaskCreate(myi2c_task, "myi2c_task", I2C_TASK_STACK_SIZE, NULL, 3, NULL);
}

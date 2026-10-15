#pragma once
#include "driver/i2c_master.h"
#include "esp_err.h"

typedef struct {
  float accel_x, accel_y, accel_z; // in g
  float gyro_x, gyro_y, gyro_z;    // in deg/s
  float mag_x, mag_y, mag_z;       // in mG
} imu_data_t;

typedef struct {
  i2c_port_num_t port;
  gpio_num_t sda_pin;
  gpio_num_t scl_pin;
} imu_config_t;

typedef struct {
  i2c_master_bus_handle_t bus_handle;
  i2c_master_dev_handle_t mpu6050_handle;
  i2c_master_dev_handle_t mmc5603_handle;
} imu_handle_t;

esp_err_t imu_driver_setup(imu_handle_t *imu_handle,
                           const imu_config_t *imu_cfg);
esp_err_t imu_driver_read_accel_gyro(imu_handle_t *imu_handle, imu_data_t *out);
esp_err_t imu_driver_read_mag(imu_handle_t *imu_handle, imu_data_t *out);
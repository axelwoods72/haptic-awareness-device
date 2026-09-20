#pragma once
#include "esp_err.h"

typedef struct {
  float accel_x, accel_y, accel_z; // in g
  float gyro_x, gyro_y, gyro_z;    // in deg/s
  float mag_x, mag_y, mag_z;       // in mG
} imu_data_t;

esp_err_t imu_driver_init(void);
esp_err_t imu_driver_read(imu_data_t *out);
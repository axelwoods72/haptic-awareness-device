#pragma once
#include "imu_driver.h"

typedef struct {
  float pitch;
  float roll;
  float yaw; // all in deg
} orientation_data_t;

void complementary_filter_init(void);
void complementary_filter_update_gyro(const imu_data_t *imu,
                                      orientation_data_t *out);
void complementary_filter_update_mag(const imu_data_t *imu,
                                     orientation_data_t *out);

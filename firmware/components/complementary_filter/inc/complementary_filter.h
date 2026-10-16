#pragma once
#include "imu_driver.h"

typedef struct {
  float pitch;
  float roll;
  float yaw; // all in deg
} orientation_data_t;

typedef struct {
  float roll, pitch, yaw;
  int64_t last_time_us;
  bool initialized;
  bool yaw_initialized;
} comp_filter_t;

void complementary_filter_init(comp_filter_t *f);
void complementary_filter_update_gyro(comp_filter_t *f, const imu_data_t *imu,
                                      orientation_data_t *out);
void complementary_filter_update_mag(comp_filter_t *f, const imu_data_t *imu,
                                     orientation_data_t *out);
float wrap_angle(float diff);

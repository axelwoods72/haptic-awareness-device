#include "complementary_filter.h"
#include "esp_err.h"
#include "esp_timer.h"
#include <math.h>
#include <stdbool.h>

#define ALPHA 0.98f
#define RAD_TO_DEG (180.0f / (float)M_PI)
#define DEG_TO_RAD ((float)M_PI / 180.0f)

float wrap_angle(float diff) {
  while (diff > 180.0f)
    diff -= 360.0f;
  while (diff < -180.0f)
    diff += 360.0f;
  return diff;
}

void complementary_filter_init(comp_filter_t *f) {
  f->roll = f->pitch = f->yaw = 0.0f;
  f->last_time_us = 0;
  f->initialized = false;
  f->yaw_initialized = false;
}

void complementary_filter_update_gyro(comp_filter_t *f, const imu_data_t *imu,
                                      orientation_data_t *out) {
  // Calculate roll and pitch from accelerometer
  float roll_accel = atan2f(imu->accel_y, imu->accel_z) * RAD_TO_DEG;
  float pitch_accel =
      atan2f(-imu->accel_x,
             sqrtf(imu->accel_y * imu->accel_y + imu->accel_z * imu->accel_z)) *
      RAD_TO_DEG;

  // First readings - use readings without gyro integration
  if (!f->initialized) {
    f->roll = roll_accel;
    f->pitch = pitch_accel;
    f->yaw = 0.0f; // corrected on first mag update

    f->last_time_us = esp_timer_get_time();
    f->initialized = true;

    out->roll = f->roll;
    out->pitch = f->pitch;
    out->yaw = f->yaw;
    return;
  }

  // Find dt for rate * dt gyro integration
  int64_t now_us = esp_timer_get_time();
  float dt = (now_us - f->last_time_us) / 1000000.0f;
  f->last_time_us = now_us;

  // Calculate pitch and roll from gyro
  float roll_gyro = f->roll + imu->gyro_x * dt;
  float pitch_gyro = f->pitch + imu->gyro_y * dt;
  // Blend pitch and roll from gyro and accel
  f->roll = ALPHA * roll_gyro + (1.0f - ALPHA) * roll_accel;
  f->pitch = ALPHA * pitch_gyro + (1.0f - ALPHA) * pitch_accel;

  // Calculate yaw from gyro
  f->yaw =
      f->yaw + imu->gyro_z * dt; // free runs once before next mag correction

  out->roll = f->roll;
  out->pitch = f->pitch;
  out->yaw = f->yaw;
}

void complementary_filter_update_mag(comp_filter_t *f, const imu_data_t *imu,
                                     orientation_data_t *out) {
  // Calculate yaw from pitch, roll and mag readings
  float phi_rad = f->roll * DEG_TO_RAD;
  float theta_rad = f->pitch * DEG_TO_RAD;
  float mx = imu->mag_x * cosf(theta_rad) + imu->mag_z * sinf(theta_rad);
  float my = imu->mag_x * sinf(phi_rad) * sinf(theta_rad) +
             imu->mag_y * cosf(phi_rad) -
             imu->mag_z * sinf(phi_rad) * cosf(theta_rad);
  float yaw_mag = atan2f(-my, mx) * RAD_TO_DEG;

  // First reading - use without gyro integration
  if (!f->yaw_initialized) {
    f->yaw = yaw_mag;
    f->yaw_initialized = true;
    out->roll = f->roll;
    out->pitch = f->pitch;
    out->yaw = f->yaw;
    return;
  }

  // Blend the wrapped difference, not raw values since yaw from gyro and yaw
  // from mag can straddle the +-180 wrap point for the same heading
  float diff = wrap_angle(yaw_mag - f->yaw); // yaw is actually yaw from gyro
  f->yaw = wrap_angle(f->yaw + (1.0f - ALPHA) * diff);

  out->roll = f->roll;
  out->pitch = f->pitch;
  out->yaw = f->yaw;
}
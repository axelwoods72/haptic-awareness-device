#include "complementary_filter.h"
#include "esp_timer.h"
#include <math.h>
#include <stdbool.h>

#define ALPHA 0.98f
#define RAD_TO_DEG (180.0f / (float)M_PI)
#define DEG_TO_RAD ((float)M_PI / 180.0f)

static float roll, pitch, yaw;
static int64_t last_time_us;
static bool initialized = false;

static float wrap_angle_diff(float diff) {
  while (diff > 180.0f)
    diff -= 360.0f;
  while (diff < -180.0f)
    diff += 360.0f;
  return diff;
}

void orientation_filter_init(void) {
  roll = pitch = yaw = 0.0f;
  last_time_us = 0;
  initialized = false;
}

void orientation_filter_update(const imu_data_t *imu, orientation_data_t *out) {
  float roll_accel = atan2f(imu->accel_y, imu->accel_z) * RAD_TO_DEG;
  float pitch_accel =
      atan2f(-imu->accel_x,
             sqrtf(imu->accel_y * imu->accel_y + imu->accel_z * imu->accel_z)) *
      RAD_TO_DEG;

  if (!initialized) {
    roll = roll_accel;
    pitch = pitch_accel;

    float phi_rad = roll * DEG_TO_RAD;
    float theta_rad = pitch * DEG_TO_RAD;
    float mx = imu->mag_x * cosf(theta_rad) + imu->mag_z * sinf(theta_rad);
    float my = imu->mag_x * sinf(phi_rad) * sinf(theta_rad) +
               imu->mag_y * cosf(phi_rad) -
               imu->mag_z * sinf(phi_rad) * cosf(theta_rad);
    yaw = atan2f(-my, mx) * RAD_TO_DEG;

    last_time_us = esp_timer_get_time();
    initialized = true;

    out->roll = roll;
    out->pitch = pitch;
    out->yaw = yaw;
    return;
  }

  int64_t now_us = esp_timer_get_time();
  float dt = (now_us - last_time_us) / 1000000.0f;
  last_time_us = now_us;

  float roll_gyro = roll + imu->gyro_x * dt;
  float pitch_gyro = pitch + imu->gyro_y * dt;
  roll = ALPHA * roll_gyro + (1.0f - ALPHA) * roll_accel;
  pitch = ALPHA * pitch_gyro + (1.0f - ALPHA) * pitch_accel;

  float phi_rad = roll * DEG_TO_RAD;
  float theta_rad = pitch * DEG_TO_RAD;
  float mx = imu->mag_x * cosf(theta_rad) + imu->mag_z * sinf(theta_rad);
  float my = imu->mag_x * sinf(phi_rad) * sinf(theta_rad) +
             imu->mag_y * cosf(phi_rad) -
             imu->mag_z * sinf(phi_rad) * cosf(theta_rad);
  float yaw_mag = atan2f(-my, mx) * RAD_TO_DEG;

  float yaw_gyro = yaw + imu->gyro_z * dt;
  float diff = wrap_angle_diff(yaw_mag - yaw_gyro);
  yaw = yaw_gyro + (1.0f - ALPHA) * diff;

  while (yaw > 180.0f)
    yaw -= 360.0f;
  while (yaw < -180.0f)
    yaw += 360.0f;

  out->roll = roll;
  out->pitch = pitch;
  out->yaw = yaw;
}

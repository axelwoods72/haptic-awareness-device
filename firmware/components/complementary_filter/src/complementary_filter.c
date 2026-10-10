#include "complementary_filter.h"
#include "esp_err.h"
#include "esp_timer.h"
#include <math.h>
#include <stdbool.h>

#define ALPHA 0.98f
#define RAD_TO_DEG (180.0f / (float)M_PI)
#define DEG_TO_RAD ((float)M_PI / 180.0f)

static float roll, pitch, yaw;
static int64_t last_time_us;
static bool initialized = false;
static bool yaw_initialized = false;

static float wrap_angle(float diff) {
  while (diff > 180.0f)
    diff -= 360.0f;
  while (diff < -180.0f)
    diff += 360.0f;
  return diff;
}

void complementary_filter_init(void) {
  roll = pitch = yaw = 0.0f;
  last_time_us = 0;
  initialized = false;
  yaw_initialized = false;
}

void complementary_filter_update_gyro(const imu_data_t *imu,
                                      orientation_data_t *out) {
  // Calculate roll and pitch from accelerometer
  float roll_accel = atan2f(imu->accel_y, imu->accel_z) * RAD_TO_DEG;
  float pitch_accel =
      atan2f(-imu->accel_x,
             sqrtf(imu->accel_y * imu->accel_y + imu->accel_z * imu->accel_z)) *
      RAD_TO_DEG;

  // First readings - use readings without gyro integration
  if (!initialized) {
    roll = roll_accel;
    pitch = pitch_accel;
    yaw = 0.0f; // corrected on first mag update

    last_time_us = esp_timer_get_time();
    initialized = true;

    out->roll = roll;
    out->pitch = pitch;
    out->yaw = yaw;
    return;
  }

  // Find dt for rate * dt gyro integration
  int64_t now_us = esp_timer_get_time();
  float dt = (now_us - last_time_us) / 1000000.0f;
  last_time_us = now_us;

  // Calculate pitch and roll from gyro
  float roll_gyro = roll + imu->gyro_x * dt;
  float pitch_gyro = pitch + imu->gyro_y * dt;
  // Blend pitch and roll from gyro and accel
  roll = ALPHA * roll_gyro + (1.0f - ALPHA) * roll_accel;
  pitch = ALPHA * pitch_gyro + (1.0f - ALPHA) * pitch_accel;

  // Calculate yaw from gyro
  yaw = yaw + imu->gyro_z * dt; // free runs once before next mag correction

  out->roll = roll;
  out->pitch = pitch;
  out->yaw = yaw;
}

void complementary_filter_update_mag(const imu_data_t *imu,
                                     orientation_data_t *out) {
  // Calculate yaw from pitch, roll and mag readings
  float phi_rad = roll * DEG_TO_RAD;
  float theta_rad = pitch * DEG_TO_RAD;
  float mx = imu->mag_x * cosf(theta_rad) + imu->mag_z * sinf(theta_rad);
  float my = imu->mag_x * sinf(phi_rad) * sinf(theta_rad) +
             imu->mag_y * cosf(phi_rad) -
             imu->mag_z * sinf(phi_rad) * cosf(theta_rad);
  float yaw_mag = atan2f(-my, mx) * RAD_TO_DEG;

  // First reading - use without gyro integration
  if (!yaw_initialized) {
    yaw = yaw_mag;
    yaw_initialized = true;
    return;
  }

  // Blend the wrapped difference, not raw values since yaw from gyro and yaw
  // from mag can straddle the +-180 wrap point for the same heading
  float diff = wrap_angle(yaw_mag - yaw); // yaw is actually yaw from gyro
  yaw = wrap_angle(yaw + (1.0f - ALPHA) * diff);

  out->roll = roll;
  out->pitch = pitch;
  out->yaw = yaw;
}
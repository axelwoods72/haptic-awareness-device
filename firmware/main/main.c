#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "complementary_filter.h"
#include "imu_driver.h"

static const char *TAG = "main";

void app_main(void) {
  esp_err_t err = imu_driver_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "imu_driver_init failed: %s", esp_err_to_name(err));
    return;
  }

  imu_data_t imu_data;
  orientation_data_t orientation;

  complementary_filter_init();

  while (1) {
    err = imu_driver_read(&imu_data);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "imu_driver_read failed: %s", esp_err_to_name(err));
    } else {
      ESP_LOGI(TAG,
               "accel: %.3f %.3f %.3f g | gyro: %.2f %.2f %.2f deg/s | mag: "
               "%.1f %.1f %.1f mG ",
               imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
               imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
               imu_data.mag_x, imu_data.mag_y, imu_data.mag_z);
      complementary_filter_update(&imu_data, &orientation);
      ESP_LOGI(TAG, "pitch: %.2f deg | roll: %.2f deg | yaw: %.2f deg ",
               orientation.pitch, orientation.roll, orientation.yaw);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#include "complementary_filter.h"
#include "imu_driver.h"
#include "portmacro.h"
#include <stdint.h>

#define IMU_LOOP_PERIOD_MS 10    // 100 Hz
#define N_LOOPS_PER_MAG_UPDATE 2 // update every 2nd loop - 50Hz

static const char *TAG = "main";

void app_main(void) {
  vTaskDelay(pdMS_TO_TICKS(2000));

  esp_err_t err = imu_driver_setup();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "imu_driver_setup failed: %s\n", esp_err_to_name(err));
    return;
  }
  complementary_filter_init();

  imu_data_t imu_data;
  orientation_data_t orientation;
  uint32_t loop_cnt = 0;

  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t period_ticks = pdMS_TO_TICKS(IMU_LOOP_PERIOD_MS);

  while (1) {
    // Read and integrate accel/gyro every loop
    err = imu_driver_read_accel_gyro(&imu_data);
    if (err == ESP_OK) {
      ESP_LOGE(TAG, "imu_driver_read_mag failed: %s\n", esp_err_to_name(err));
    }
    complementary_filter_update_gyro(&imu_data, &orientation);

    // Read and integrate mag every second loop
    if (loop_cnt % N_LOOPS_PER_MAG_UPDATE == 0) {
      err = imu_driver_read_mag(&imu_data);
      if (err == ESP_OK) {
        ESP_LOGE(TAG, "imu_driver_read_mag failed: %s\n", esp_err_to_name(err));
      }
      complementary_filter_update_mag(&imu_data, &orientation);
    }

    loop_cnt++;
    // visualize_imu.py greps this from serial
    printf("ORIENT,%.2f,%.2f,%.2f\n", orientation.roll, orientation.pitch,
           orientation.yaw);
    vTaskDelayUntil(&last_wake_time, period_ticks);
  }
}

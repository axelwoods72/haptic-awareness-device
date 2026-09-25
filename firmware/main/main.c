#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "imu_driver.h"
#include "mic_driver.h"

void mic_task (void* pv);

static const char *TAG = "main";

void app_main(void) {

  // mic_task runs in parallel with imu driver, large stack memory allocation
  xTaskCreatePinnedToCore(mic_task, "mic_task", 8192, NULL, 5, NULL, 1);

  esp_err_t err = imu_driver_init();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "imu_driver_init failed: %s", esp_err_to_name(err));
    return;
  }

  imu_data_t imu_data;

  while (1) {
    err = imu_driver_read(&imu_data);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "imu_driver_read failed: %s", esp_err_to_name(err));
    } else {
      ESP_LOGI(TAG,
               "accel: %.3f %.3f %.3f g | gyro: %.2f %.2f %.2f deg/s | mag: "
               "%.1f %.1f %.1f mG",
               imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
               imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
               imu_data.mag_x, imu_data.mag_y, imu_data.mag_z);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void mic_task (void* pv) {
  esp_err_t err = mic_driver_init();
  ESP_ERROR_CHECK(err);

  while (1) {
    int32_t buf0[MIC_FRAME_SAMPLES], buf1[MIC_FRAME_SAMPLES], buf2[MIC_FRAME_SAMPLES];
    size_t bytes_read_stereo, bytes_read_mono;

    err = mic_driver_read_stereo(buf0, buf1, &bytes_read_stereo);
    err = mic_driver_read_mono(buf2, &bytes_read_mono);

    size_t expected_bytes = MIC_FRAME_SAMPLES * sizeof(int32_t);

    if (bytes_read_mono != expected_bytes || bytes_read_stereo != expected_bytes * 2) {
      ESP_LOGW(TAG, "Actual bytes read smaller than expected (%u bytes):\nmic0: %u\nmic1: %u\nmic2: %u", 
        expected_bytes, bytes_read_stereo / 2, bytes_read_stereo / 2, bytes_read_mono);
    }

    int32_t min0 = INT32_MAX, max0 = INT32_MIN;
    int64_t sum0 = 0;
    int32_t min1 = INT32_MAX, max1 = INT32_MIN;
    int64_t sum1 = 0;
    int32_t min2 = INT32_MAX, max2 = INT32_MIN;
    int64_t sum2 = 0;

    for (int i = 0; i < MIC_FRAME_SAMPLES; i++) {
      int32_t s0 = buf0[i] >> 8;
      if (s0 < min0) min0 = s0;
      if (s0 > max0) max0 = s0;
      sum0 += s0;
      
      int32_t s1 = buf1[i] >> 8;
      if (s1 < min1) min1 = s1;
      if (s1 > max1) max1 = s1;
      sum1 += s1;

      int32_t s2 = buf2[i] >> 8;
      if (s2 < min2) min2 = s2;
      if (s2 > max2) max2 = s2;
      sum2 += s2;
    }
    ESP_LOGI(TAG, "mic0: min=%ld max=%ld mean=%lld", (long)min0, (long)max0, sum0 / MIC_FRAME_SAMPLES);
    ESP_LOGI(TAG, "mic1: min=%ld max=%ld mean=%lld", (long)min1, (long)max1, sum1 / MIC_FRAME_SAMPLES);
    ESP_LOGI(TAG, "mic2: min=%ld max=%ld mean=%lld", (long)min2, (long)max2, sum2 / MIC_FRAME_SAMPLES);
  }
}
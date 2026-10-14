#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "imu_driver.h"
#include "mic_driver.h"

void mic_task (void* pv);

static const char *TAG = "main";

QueueHandle_t theta_queue;

void app_main(void) {

  // queue length of 1
  theta_queue = xQueueCreate(1, sizeof(float)); 
  if (theta_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create theta queue");
    return;
  }

  // mic_task runs in parallel with imu driver, large stack memory allocation
  xTaskCreatePinnedToCore(mic_task, "mic_task", 16384, NULL, 5, NULL, 1);

  // esp_err_t err = imu_driver_init();
  // if (err != ESP_OK) {
  //   ESP_LOGE(TAG, "imu_driver_init failed: %s", esp_err_to_name(err));
  //   return;
  // }

  // imu_data_t imu_data;

  while (1) {
    // err = imu_driver_read(&imu_data);
    // if (err != ESP_OK) {
    //   ESP_LOGE(TAG, "imu_driver_read failed: %s", esp_err_to_name(err));
    // } else {
    //   ESP_LOGI(TAG,
    //            "accel: %.3f %.3f %.3f g | gyro: %.2f %.2f %.2f deg/s | mag: "
    //            "%.1f %.1f %.1f mG",
    //            imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
    //            imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
    //            imu_data.mag_x, imu_data.mag_y, imu_data.mag_z);
    // }

  float theta;
  if (xQueueReceive(theta_queue, &theta, pdMS_TO_TICKS(0)) == pdTRUE) {
    ESP_LOGI(TAG, "Theta: %f", theta);
  }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void mic_task (void* pv) {
  esp_err_t err = mic_driver_init();
  ESP_ERROR_CHECK(err);
  ESP_LOGI(TAG, "mic_driver initialised, threshold=%d", AMP_THRES);

  // once-per-second status
  const int frames_per_sec = MIC_SAMPLE_RATE / MIC_FRAME_SAMPLES;
  int frame_count = 0, accepted_count = 0;
  float peak_max[N_MICS] = {0};

  // smoothed direction as a unit vector, so -179 and +179 average to 180 rather than 0
  float sin_avg = 0.0f, cos_avg = 1.0f;
  int quiet_frames = QUIET_RESET_FRAMES;

  // for ensuring theta is accurate at beginning of new sound event
  int theta_fresh = 1;

  while (1) {
    int32_t buf0[MIC_FRAME_SAMPLES], buf1[MIC_FRAME_SAMPLES], buf2[MIC_FRAME_SAMPLES];
    size_t bytes_read_stereo, bytes_read_mono;

    err = mic_driver_read_stereo(buf0, buf1, &bytes_read_stereo);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "stereo read failed: %s", esp_err_to_name(err));
      continue;
    }
    err = mic_driver_read_mono(buf2, &bytes_read_mono);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mono read failed: %s", esp_err_to_name(err));
      continue;
    }

    size_t expected_bytes = MIC_FRAME_SAMPLES * sizeof(int32_t);

    if (bytes_read_mono != expected_bytes || bytes_read_stereo != expected_bytes) {
      ESP_LOGW(TAG, "Actual bytes read smaller than expected (%u bytes):\nmic0: %u\nmic1: %u\nmic2: %u", 
        expected_bytes, bytes_read_stereo / 2, bytes_read_stereo / 2, bytes_read_mono);
      continue;
    }

    // hpf each signal
    float clean0[MIC_FRAME_SAMPLES], clean1[MIC_FRAME_SAMPLES], clean2[MIC_FRAME_SAMPLES];
    hpf(buf0, 0, clean0);
    hpf(buf1, 1, clean1);
    hpf(buf2, 2, clean2);

    // track loudest frame per mic for the status log
    float peak[N_MICS] = {frame_peak(clean0), frame_peak(clean1), frame_peak(clean2)};
    for (int m = 0; m < N_MICS; m++) {
      if (peak[m] > peak_max[m]) peak_max[m] = peak[m];
    }

    int has_signal = peak[0] > AMP_THRES && peak[1] > AMP_THRES && peak[2] > AMP_THRES;
    accepted_count += has_signal;

    if (++frame_count >= frames_per_sec) {
      ESP_LOGI(TAG, "status: %d/%d frames above threshold, peak mic0=%.0f mic1=%.0f mic2=%.0f",
        accepted_count, frame_count, peak_max[0], peak_max[1], peak_max[2]);
      frame_count = accepted_count = 0;
      peak_max[0] = peak_max[1] = peak_max[2] = 0.0f;
    }

    // ignore background noise
    if (!has_signal) {
      quiet_frames++;
      continue;
    }

    // reset running avg for new sound
    int new_event = quiet_frames >= QUIET_RESET_FRAMES;
    quiet_frames = 0;
    if (new_event) {
      tdoa_reset();
      theta_fresh = 1;
    }

#if USE_SRP_PHAT
    float score;
    float rad = srp_phat(clean0, clean1, clean2, &score);

    ESP_LOGI(TAG, "srp theta: %.1f, score: %.3g", rad * 180.0f / (float)PI, score);
#else
    // GCC-PHAT TDOA
    float tau[3];
    tdoa(tau, clean0, clean1, clean2);

    ESP_LOGI(TAG, "tau01: %f, tau02: %f, tau12: %f", tau[0], tau[1], tau[2]);

    // ensure same sound signal being evaluated for each cross-correlation
    if (fabsf(tau[0] + tau[2] - tau[1]) > 3.0f / MIC_SAMPLE_RATE) {
      ESP_LOGI(TAG, "Tau sum check failed, rejecting reading.");
      continue;
    }

    float rad = tau_to_angle(tau);
#endif

    if (theta_fresh) {
      sin_avg = sinf(rad);
      cos_avg = cosf(rad);
      theta_fresh = 0;
    } else {
      sin_avg = (1.0f - THETA_ALPHA) * sin_avg + THETA_ALPHA * sinf(rad);
      cos_avg = (1.0f - THETA_ALPHA) * cos_avg + THETA_ALPHA * cosf(rad);
    }

    float theta = atan2f(sin_avg, cos_avg) * 180.0f / (float)PI;
    xQueueOverwrite(theta_queue, &theta);

    int32_t min0 = INT32_MAX, max0 = INT32_MIN;
    int64_t sum0 = 0;
    int32_t min1 = INT32_MAX, max1 = INT32_MIN;
    int64_t sum1 = 0;
    int32_t min2 = INT32_MAX, max2 = INT32_MIN;
    int64_t sum2 = 0;

    for (int i = 0; i < MIC_FRAME_SAMPLES; i++) {
      int32_t s0 = (int32_t)clean0[i] >> 8;
      if (s0 < min0) min0 = s0;
      if (s0 > max0) max0 = s0;
      sum0 += s0;
      
      int32_t s1 = (int32_t)clean1[i] >> 8;
      if (s1 < min1) min1 = s1;
      if (s1 > max1) max1 = s1;
      sum1 += s1;

      int32_t s2 = (int32_t)clean2[i] >> 8;
      if (s2 < min2) min2 = s2;
      if (s2 > max2) max2 = s2;
      sum2 += s2;
    }
    ESP_LOGI(TAG, "mic0: min=%ld max=%ld mean=%lld", (long)min0, (long)max0, sum0 / MIC_FRAME_SAMPLES);
    ESP_LOGI(TAG, "mic1: min=%ld max=%ld mean=%lld", (long)min1, (long)max1, sum1 / MIC_FRAME_SAMPLES);
    ESP_LOGI(TAG, "mic2: min=%ld max=%ld mean=%lld", (long)min2, (long)max2, sum2 / MIC_FRAME_SAMPLES);
  }
}
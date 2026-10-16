#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#include "complementary_filter.h"
#include "freertos/queue.h"
#include "gpio.h"
#include "haptic_control.h"
#include "imu_driver.h"
#include "mic_driver.h"
#include "portmacro.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#define IMU_LOOP_PERIOD_MS 10    // 100 Hz
#define N_LOOPS_PER_MAG_UPDATE 2 // update every 2nd loop - 50Hz
#define ZERO_IMUS_AFTER_LOOPS 100
#define SOUND_HOLD_MS 250
#define HAPTIC_STRENGTH 0.4f
#define HAPTIC_LOOP_PERIOD_MS 10

static const char *TAG = "main";

static imu_handle_t imu_head, imu_body;
static comp_filter_t f_head, f_body;

void mic_task(void *pv);
void imu_update(const char *name, imu_handle_t *imu, comp_filter_t *f,
                orientation_data_t *orient, bool do_mag);

static QueueHandle_t theta_queue;
static QueueHandle_t haptic_queue;
static void haptic_task(void *pv);
static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }

/* ================================================================== */
/* Main                                                               */
/* ================================================================== */

void app_main(void) {
  /* Set all motor outputs LOW before waiting for sensors. */
  const haptic_config_t motor_cfg = {.ia = {HAPTIC_FRONT_PIN, HAPTIC_RIGHT_PIN,
                                            HAPTIC_BACK_PIN, HAPTIC_LEFT_PIN}};
  ESP_ERROR_CHECK(haptic_init(&motor_cfg));
  vTaskDelay(pdMS_TO_TICKS(2000));

  /* ---- IMU setup ---- */
  const imu_config_t head_cfg = {.port = I2C_NUM_0,
                                 .sda_pin = IMU_HEAD_SDA_PIN,
                                 .scl_pin = IMU_HEAD_SCL_PIN};
  const imu_config_t body_cfg = {.port = I2C_NUM_1,
                                 .sda_pin = IMU_BODY_SDA_PIN,
                                 .scl_pin = IMU_BODY_SCL_PIN};

  esp_err_t err = imu_driver_setup(&imu_head, &head_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "head imu setup failed: %s", esp_err_to_name(err));
    return;
  }
  err = imu_driver_setup(&imu_body, &body_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "body imu setup failed: %s", esp_err_to_name(err));
    return;
  }
  complementary_filter_init(&f_head);
  complementary_filter_init(&f_body);
  /* --------------------- */

  // queue length of 1
  theta_queue = xQueueCreate(1, sizeof(mic_output_t));
  if (theta_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create theta queue");
    return;
  }

  haptic_queue = xQueueCreate(1, sizeof(haptic_command_t));
  if (haptic_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create haptic queue");
    vQueueDelete(theta_queue);
    return; // outputs remain LOW
  }
  /* Only this task writes PWM after init. It expires stale commands even
   * while the sensor loop is waiting for I2C. */
  if (xTaskCreatePinnedToCore(haptic_task, "haptic_task", 4096, NULL, 4, NULL,
                              0) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create haptic task");
    vQueueDelete(haptic_queue);
    vQueueDelete(theta_queue);
    return;
  }
  if (xTaskCreatePinnedToCore(mic_task, "mic_task", 16384, NULL, 5, NULL, 1) !=
      pdPASS) {
    ESP_LOGE(TAG, "Failed to create mic task");
    return; // haptic task has no active command and stays stopped
  }

  orientation_data_t head_orient = {0}, body_orient = {0};
  uint32_t loop_cnt = 0;
  float head_yaw0 = 0.0f, body_yaw0 = 0.0f;
  bool imus_zeroed = false;

  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t period_ticks = pdMS_TO_TICKS(IMU_LOOP_PERIOD_MS);
  mic_output_t mic_output = {0};
  bool have_sound_sample = false;

  /* ---- loop ---- */
  while (1) {
    /* ---- IMU ---- */
    bool do_mag = (loop_cnt % N_LOOPS_PER_MAG_UPDATE == 0);
    imu_update("head", &imu_head, &f_head, &head_orient, do_mag);
    imu_update("body", &imu_body, &f_body, &body_orient, do_mag);
    loop_cnt++;

    // Capture the startup heading once, after the filters have settled
    if (!imus_zeroed && loop_cnt >= ZERO_IMUS_AFTER_LOOPS &&
        f_head.yaw_initialized && f_body.yaw_initialized) {
      head_yaw0 = head_orient.yaw;
      body_yaw0 = body_orient.yaw;
      imus_zeroed = true;
      ESP_LOGI(TAG, "IMUs zeroed (head %.1f, body %.1f)", head_yaw0, body_yaw0);
    }
    float head_yaw = wrap_angle(head_orient.yaw - head_yaw0);
    float body_yaw = wrap_angle(body_orient.yaw - body_yaw0);

    float axels_theta = wrap_angle(head_yaw - body_yaw);

    // visualize_imu.py greps this from serial
    printf("ORIENT,%.2f,%.2f,%.2f\n", head_orient.roll, head_orient.pitch,
           head_yaw);
    printf("NECK,%.2f\n", axels_theta);
    /* ---------------- */

    if (xQueueReceive(theta_queue, &mic_output, 0) == pdTRUE)
      have_sound_sample = true;
    bool sound_active =
        have_sound_sample &&
        (TickType_t)(xTaskGetTickCount() - mic_output.last_heard) <
            pdMS_TO_TICKS(SOUND_HOLD_MS);
    if (!sound_active)
      have_sound_sample = false;

    haptic_command_t command = {.updated_at_ms = now_ms()};
    if (sound_active && imus_zeroed) {
      float sams_theta;
      if (haptic_body_angle(mic_output.theta, head_yaw, body_yaw,
                            &sams_theta) == ESP_OK) {
        command.theta_deg = sams_theta;
        command.strength = HAPTIC_STRENGTH;
        command.active = true;
        /* Preserve the mic visualiser's log and expose the corrected angle. */
        ESP_LOGI(TAG, "Theta: %f", mic_output.theta);
        ESP_LOGD(TAG, "Belt theta: %.2f", sams_theta);
      }
    }
    /* Disabled commands stop every channel; no blocking one-second demo. */
    xQueueOverwrite(haptic_queue, &command);

    vTaskDelayUntil(&last_wake_time, period_ticks);
  }
}

/* ================================================================== */
/* Helper functions                                                   */
/* ================================================================== */

static void haptic_task(void *pv) {
  (void)pv;
  haptic_command_t latest = {0};
  TickType_t wake = xTaskGetTickCount();
  while (1) {
    xQueueReceive(haptic_queue, &latest, 0);
    uint32_t now = now_ms();
    if ((uint32_t)(now - latest.updated_at_ms) >= HAPTIC_COMMAND_TIMEOUT_MS)
      latest.active = false; // latch expiry until a new command arrives
    esp_err_t err = haptic_apply_command(&latest, now);
    if (err != ESP_OK) {
      /* Invalid inputs and write failures already stop all configured motors.
       */
      ESP_LOGE(TAG, "Haptic output failed: %s", esp_err_to_name(err));
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(HAPTIC_LOOP_PERIOD_MS));
  }
}

void mic_task(void *pv) {
  (void)pv;
  esp_err_t err = mic_driver_init();
  ESP_ERROR_CHECK(err);
  ESP_LOGI(TAG, "mic_driver initialised, threshold=%d", AMP_THRES);

  // once-per-second status
  const int frames_per_sec = MIC_SAMPLE_RATE / MIC_FRAME_SAMPLES;
  int frame_count = 0, accepted_count = 0;
  float peak_max[N_MICS] = {0};

  // smoothed direction as a unit vector, so -179 and +179 average to 180 rather
  // than 0
  float sin_avg = 0.0f, cos_avg = 1.0f;
  int quiet_frames = QUIET_RESET_FRAMES;

  // for ensuring theta is accurate at beginning of new sound event
  int theta_fresh = 1;

  while (1) {
    int32_t buf0[MIC_FRAME_SAMPLES], buf1[MIC_FRAME_SAMPLES],
        buf2[MIC_FRAME_SAMPLES];
    size_t bytes_read_stereo, bytes_read_mono;
    mic_output_t mic_out;

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

    if (bytes_read_mono != expected_bytes ||
        bytes_read_stereo != expected_bytes) {
      ESP_LOGW(TAG,
               "Actual bytes read smaller than expected (%u bytes):\nmic0: "
               "%u\nmic1: %u\nmic2: %u",
               (unsigned)expected_bytes, (unsigned)bytes_read_stereo,
               (unsigned)bytes_read_stereo, (unsigned)bytes_read_mono);
      continue;
    }

    // hpf each signal
    float clean0[MIC_FRAME_SAMPLES], clean1[MIC_FRAME_SAMPLES],
        clean2[MIC_FRAME_SAMPLES];
    hpf(buf0, 0, clean0);
    hpf(buf1, 1, clean1);
    hpf(buf2, 2, clean2);

    // track loudest frame per mic for the status log
    float peak[N_MICS] = {frame_peak(clean0), frame_peak(clean1),
                          frame_peak(clean2)};
    for (int m = 0; m < N_MICS; m++) {
      if (peak[m] > peak_max[m])
        peak_max[m] = peak[m];
    }

    int has_signal =
        peak[0] > AMP_THRES && peak[1] > AMP_THRES && peak[2] > AMP_THRES;
    accepted_count += has_signal;

    if (++frame_count >= frames_per_sec) {
      ESP_LOGI(TAG,
               "status: %d/%d frames above threshold, peak mic0=%.0f mic1=%.0f "
               "mic2=%.0f",
               accepted_count, frame_count, peak_max[0], peak_max[1],
               peak_max[2]);
      frame_count = accepted_count = 0;
      peak_max[0] = peak_max[1] = peak_max[2] = 0.0f;
    }

    // ignore background noise
    if (!has_signal) {
      if (quiet_frames < QUIET_RESET_FRAMES)
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

    ESP_LOGI(TAG, "srp theta: %.1f, score: %.3g", rad * 180.0f / (float)PI,
             score);
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
    mic_out.theta = theta;
    mic_out.last_heard = xTaskGetTickCount();
    xQueueOverwrite(theta_queue, &mic_out);

    int32_t min0 = INT32_MAX, max0 = INT32_MIN;
    int64_t sum0 = 0;
    int32_t min1 = INT32_MAX, max1 = INT32_MIN;
    int64_t sum1 = 0;
    int32_t min2 = INT32_MAX, max2 = INT32_MIN;
    int64_t sum2 = 0;

    for (int i = 0; i < MIC_FRAME_SAMPLES; i++) {
      int32_t s0 = (int32_t)clean0[i] >> 8;
      if (s0 < min0)
        min0 = s0;
      if (s0 > max0)
        max0 = s0;
      sum0 += s0;

      int32_t s1 = (int32_t)clean1[i] >> 8;
      if (s1 < min1)
        min1 = s1;
      if (s1 > max1)
        max1 = s1;
      sum1 += s1;

      int32_t s2 = (int32_t)clean2[i] >> 8;
      if (s2 < min2)
        min2 = s2;
      if (s2 > max2)
        max2 = s2;
      sum2 += s2;
    }
    ESP_LOGI(TAG, "mic0: min=%ld max=%ld mean=%lld", (long)min0, (long)max0,
             sum0 / MIC_FRAME_SAMPLES);
    ESP_LOGI(TAG, "mic1: min=%ld max=%ld mean=%lld", (long)min1, (long)max1,
             sum1 / MIC_FRAME_SAMPLES);
    ESP_LOGI(TAG, "mic2: min=%ld max=%ld mean=%lld", (long)min2, (long)max2,
             sum2 / MIC_FRAME_SAMPLES);
  }
}

void imu_update(const char *name, imu_handle_t *imu, comp_filter_t *f,
                orientation_data_t *orient, bool do_mag) {
  imu_data_t data;
  esp_err_t err = imu_driver_read_accel_gyro(imu, &data);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "%s accel/gyro read failed: %s", name, esp_err_to_name(err));
    return;
  }
  complementary_filter_update_gyro(f, &data, orient);

  if (do_mag) {
    err = imu_driver_read_mag(imu, &data);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "%s mag read failed: %s", name, esp_err_to_name(err));
      return;
    }
    complementary_filter_update_mag(f, &data, orient);
  }
}

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/projdefs.h"
#include "freertos/task.h"

#include "complementary_filter.h"
#include "driver/ledc.h"
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
#define HAPTIC_STRENGTH 1.0f // PWM duty 0..1, 1.0 = full power
#define HAPTIC_LOOP_PERIOD_MS 10
#define MOTOR_LOG_PERIOD_MS                                                    \
  200 // how often the motor debug line repeats while a sound is active

// set to 0 to build without that hardware, e.g. to test the mic array alone
#define ENABLE_IMU 1
#define ENABLE_HAPTICS 1

static const char *TAG = "main";

#if ENABLE_IMU
static imu_handle_t imu_head, imu_body;
static comp_filter_t f_head, f_body;
#endif

void mic_task(void *pv);
#if ENABLE_IMU
void imu_update(const char *name, imu_handle_t *imu, comp_filter_t *f,
                orientation_data_t *orient, bool do_mag);
#endif

static QueueHandle_t theta_queue;
#if ENABLE_HAPTICS
static QueueHandle_t haptic_queue;
static void haptic_task(void *pv);
static uint32_t now_ms(void) { return (uint32_t)(esp_timer_get_time() / 1000); }
#endif
static void log_motors(bool active, float mic_deg, float neck_deg,
                       float theta_deg, float strength);

/* ================================================================== */
/* Main                                                               */
/* ================================================================== */

void app_main(void) {
#if ENABLE_HAPTICS
  /* Set all motor outputs LOW before waiting for sensors. */
  const haptic_config_t motor_cfg = {.ia = {HAPTIC_FRONT_PIN, HAPTIC_RIGHT_PIN,
                                            HAPTIC_BACK_PIN, HAPTIC_LEFT_PIN}};
  ESP_ERROR_CHECK(haptic_init(&motor_cfg));
  ESP_LOGI(TAG, "motors: FRONT=GPIO%d RIGHT=GPIO%d BACK=GPIO%d LEFT=GPIO%d",
           HAPTIC_FRONT_PIN, HAPTIC_RIGHT_PIN, HAPTIC_BACK_PIN,
           HAPTIC_LEFT_PIN);

  // boot test: all four motors on for 5 s. haptic_init() puts motor i on LEDC
  // channel i
  ESP_LOGI(TAG, "MOTOR boot test: all motors on for 5 s");
  for (int i = 0; i < HAPTIC_COUNT; ++i) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i,
                  (uint32_t)(HAPTIC_STRENGTH * 1023.0f + 0.5f));
    ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
  }
  vTaskDelay(pdMS_TO_TICKS(5000));
  haptic_stop();
  ESP_LOGI(TAG, "MOTOR boot test done");
#endif
  vTaskDelay(pdMS_TO_TICKS(2000));

#if ENABLE_IMU
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
#endif

  // queue length of 1
  theta_queue = xQueueCreate(1, sizeof(mic_output_t));
  if (theta_queue == NULL) {
    ESP_LOGE(TAG, "Failed to create theta queue");
    return;
  }

#if ENABLE_HAPTICS
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
#endif
  if (xTaskCreatePinnedToCore(mic_task, "mic_task", 16384, NULL, 5, NULL, 1) !=
      pdPASS) {
    ESP_LOGE(TAG, "Failed to create mic task");
    return; // haptic task has no active command and stays stopped
  }

  // with the IMUs disabled, the belt angle is just the mic angle
  float head_yaw = 0.0f, body_yaw = 0.0f;
#if ENABLE_IMU
  orientation_data_t head_orient = {0}, body_orient = {0};
  uint32_t loop_cnt = 0;
  float head_yaw0 = 0.0f, body_yaw0 = 0.0f;
  bool imus_zeroed = false;
#else
  bool imus_zeroed = true;
#endif

  TickType_t last_wake_time = xTaskGetTickCount();
  const TickType_t period_ticks = pdMS_TO_TICKS(IMU_LOOP_PERIOD_MS);
  mic_output_t mic_output = {0};
  bool have_sound_sample = false;

  /* ---- loop ---- */
  while (1) {
#if ENABLE_IMU
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
    head_yaw = wrap_angle(head_orient.yaw - head_yaw0);
    body_yaw = wrap_angle(body_orient.yaw - body_yaw0);

    // for visualization tools (visualize_imu.py / visualize_neck.py), commented
    // out so the 100 Hz prints don't bury the MIC/MOTOR logs
    // printf("HEAD,%.2f,%.2f,%.2f\n", head_orient.roll, head_orient.pitch,
    //        head_yaw);
    // printf("BODY,%.2f,%.2f,%.2f\n", body_orient.roll, body_orient.pitch,
    //        body_yaw);
    // printf("NECK,%.2f\n", wrap_angle(head_yaw - body_yaw));
    /* ---------------- */
#endif
    // head yaw relative to body yaw, 0 with the IMUs disabled
    float axels_theta = wrap_angle(head_yaw - body_yaw);

    if (xQueueReceive(theta_queue, &mic_output, 0) == pdTRUE)
      have_sound_sample = true;
    bool sound_active =
        have_sound_sample &&
        (TickType_t)(xTaskGetTickCount() - mic_output.last_heard) <
            pdMS_TO_TICKS(SOUND_HOLD_MS);
    if (!sound_active)
      have_sound_sample = false;

#if ENABLE_HAPTICS
    haptic_command_t command = {.updated_at_ms = now_ms()};
#endif
    bool motors_on = false;
    float sams_theta = 0.0f;
    if (sound_active && imus_zeroed &&
        haptic_body_angle(mic_output.theta, head_yaw, body_yaw, &sams_theta) ==
            ESP_OK) {
      motors_on = true;
#if ENABLE_HAPTICS
      command.theta_deg = sams_theta;
      command.strength = HAPTIC_STRENGTH;
      command.active = true;
#endif
    }
    log_motors(motors_on, mic_output.theta, axels_theta, sams_theta,
               HAPTIC_STRENGTH);
#if ENABLE_HAPTICS
    /* Disabled commands stop every channel; no blocking one-second demo. */
    xQueueOverwrite(haptic_queue, &command);
#endif

    vTaskDelayUntil(&last_wake_time, period_ticks);
  }
}

/* ================================================================== */
/* Helper functions                                                   */
/* ================================================================== */

// shows which motors should be buzzing: once when a sound starts or stops, and
// every MOTOR_LOG_PERIOD_MS while it continues. Percent is the PWM duty.
// mic = angle from the mic array, neck = head yaw - body yaw from the IMUs,
// belt = the angle the motors are driven at
static void log_motors(bool active, float mic_deg, float neck_deg,
                       float theta_deg, float strength) {
  static bool was_active = false;
  static TickType_t last_log = 0;
  TickType_t now = xTaskGetTickCount();

  if (!active) {
    if (was_active)
      ESP_LOGI(TAG, "MOTOR off");
    was_active = false;
    return;
  }
  if (was_active &&
      (TickType_t)(now - last_log) < pdMS_TO_TICKS(MOTOR_LOG_PERIOD_MS))
    return;
  was_active = true;
  last_log = now;

  float duty[HAPTIC_COUNT];
  if (haptic_map(theta_deg, strength, duty) != ESP_OK) {
    ESP_LOGW(TAG, "MOTOR invalid command: belt theta %.1f", theta_deg);
    return;
  }
  ESP_LOGI(TAG,
           "MOTOR mic %+6.1f  neck %+6.1f  belt %+6.1f deg -> FRONT %3.0f%%  "
           "RIGHT %3.0f%%  BACK %3.0f%%  LEFT %3.0f%%",
           mic_deg, neck_deg, theta_deg, duty[HAPTIC_FRONT] * 100.0f,
           duty[HAPTIC_RIGHT] * 100.0f, duty[HAPTIC_BACK] * 100.0f,
           duty[HAPTIC_LEFT] * 100.0f);
}

#if ENABLE_HAPTICS
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
#endif

void mic_task(void *pv) {
  (void)pv;
  esp_err_t err = mic_driver_init();
  ESP_ERROR_CHECK(err);
  ESP_LOGI(TAG, "mic_driver initialised");

  // once-per-second status
  const int frames_per_sec = MIC_SAMPLE_RATE / MIC_FRAME_SAMPLES;
  int frame_count = 0, loud_count = 0, rejected_count = 0, accepted_count = 0;
  float peak_max[N_MICS] = {0};

  // smoothed direction as a unit vector, so -179 and +179 average to 180 rather
  // than 0
  float sin_avg = 0.0f, cos_avg = 1.0f;
  int quiet_frames = QUIET_RESET_FRAMES;

  // for ensuring theta is accurate at beginning of new sound event
  int theta_fresh = 1;

  float background_vol = 10000;

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

    float amp_thres = AMP_THRES_MULT * background_vol;
    int has_signal =
        peak[0] > amp_thres && peak[1] > amp_thres && peak[2] > amp_thres;
    loud_count += has_signal;

    // peaks and threshold are printed in the same units they are compared in
    if (++frame_count >= frames_per_sec) {
      ESP_LOGI(TAG,
               "MIC status: %d frames, %d loud, %d accepted, %d rejected | "
               "peak mic0=%.0f mic1=%.0f mic2=%.0f | thres=%.0f",
               frame_count, loud_count, accepted_count, rejected_count,
               peak_max[0], peak_max[1], peak_max[2], amp_thres);
      frame_count = loud_count = rejected_count = accepted_count = 0;
      peak_max[0] = peak_max[1] = peak_max[2] = 0.0f;
    }

    // ignore background noise
    if (!has_signal) {
      // adjust background_vol
      float new_back_vol =
          (1 - AMP_THRES_ALPHA) * background_vol +
          AMP_THRES_ALPHA * avg_buffers(clean0, clean1, clean2);
      if (new_back_vol >= MIN_BACK_VOL)
        background_vol = new_back_vol;

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
    float peak_ratio;
    float rad = srp_phat(clean0, clean1, clean2, &score, &peak_ratio);

    // reject frames reading if another sound is similarly confident or low
    // confidence in general
    if (peak_ratio < SRP_PEAK_RATIO_MIN || score < SRP_CUTOFF) {
      rejected_count++;
      continue;
    }
#else
    // GCC-PHAT TDOA
    float tau[3];
    tdoa(tau, clean0, clean1, clean2);

    // ensure same sound signal being evaluated for each cross-correlation
    if (fabsf(tau[0] + tau[2] - tau[1]) > 3.0f / MIC_SAMPLE_RATE) {
      rejected_count++;
      continue;
    }

    float rad = tau_to_angle(tau);
#endif
    accepted_count++;
    int first_frame = theta_fresh;

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

    // one line per accepted frame. visualize_mics.py greps "srp theta:" (this
    // frame) and "Theta:" (smoothed, what the motors get), so keep those
    // tokens.
#if USE_SRP_PHAT
    ESP_LOGI(TAG,
             "MIC srp theta: %+6.1f  Theta: %+6.1f  score %.2f  ratio %.2f%s",
             rad * 180.0f / (float)PI, theta, score, peak_ratio,
             first_frame ? "  [new sound]" : "");
#else
    ESP_LOGI(TAG, "MIC srp theta: %+6.1f  Theta: %+6.1f%s",
             rad * 180.0f / (float)PI, theta,
             first_frame ? "  [new sound]" : "");
#endif
  }
}

#if ENABLE_IMU
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
#endif

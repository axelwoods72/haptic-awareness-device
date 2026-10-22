#pragma once

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#define HPF_CUTOFF_HZ 300.0f
#define HPF_Q 0.7071f           // butterworth
#define GCC_MIN_HZ 300.0f       // cross-spectrum lower cutoff
#define GCC_MAX_HZ 6000.0f      // cross-spectrum upper cutoff
#define CSD_ALPHA 0.2f          // weight of the newest frame in the running cross-spectrum average
#define AMP_THRES_ALPHA 0.0001f // weight of the newest mic frame's buffer for running amp_thres buffer
#define AMP_THRES_MULT 3.5
#define MIN_BACK_VOL 3500
#define PHAT_WEIGHTING 1.0f
#define N_MICS 3
#define MIC_FRAME_SAMPLES 256
#define MIC_SAMPLE_RATE 48000
#define EPSILON 1e-10f
#define MIC_RADIUS 0.093521f       // left/right mics, distance from centre
#define MIC_RADIUS_FRONT 0.113518f // front mic, distance from centre
#define V 343                      // speed of sound
#define PI 3.14159265358979323846
// largest physically possible delay between two mics (widest pair is left-right = 2*R) in samples, +2 margin for placement error
#define MAX_LAG ((int)(2.0f * MIC_RADIUS / V * MIC_SAMPLE_RATE) + 2)

#define USE_SRP_PHAT 1 // 1: SRP-PHAT angle search, 0: per-pair GCC-PHAT peaks + least squares
#define SRP_STEP_DEG 2
#define SRP_N_ANGLES (360 / SRP_STEP_DEG)
#define SRP_PEAK_SEP_DEG 30     // a second peak must be at least this far from the best to count as a competing direction
#define SRP_PEAK_RATIO_MIN 1.3f // best peak must be this many times the second peak, otherwise the frame is rejected
#define SRP_CUTOFF 0.25f        // minimum accepted SRP correlation

#define THETA_ALPHA 0.3f                                             // weighting for new theta running avg
#define QUIET_RESET_FRAMES (MIC_SAMPLE_RATE / MIC_FRAME_SAMPLES / 2) // 0.5s

typedef struct
{
    float theta;
    TickType_t last_heard;
} mic_output_t;

esp_err_t mic_driver_init(void);
esp_err_t mic_driver_read_stereo(int32_t *out_buf_left, int32_t *out_buf_right, size_t *bytes_read);
esp_err_t mic_driver_read_mono(int32_t *out_buf, size_t *bytes_read);
void get_mic_positions(float pos[N_MICS][2]);
void hpf(int32_t *buf, int chan_i, float *out);
void ifft(float *G);
float get_time_delay(float *G);
float tau_to_angle(float *tau);
void tdoa(float *tau, float *clean0, float *clean1, float *clean2);
float frame_peak(const float *clean);
void tdoa_reset(void);
float srp_phat(float *clean0, float *clean1, float *clean2, float *score, float *peak_ratio);
float avg_buffers(float buf0[], float buf1[], float buf2[]);
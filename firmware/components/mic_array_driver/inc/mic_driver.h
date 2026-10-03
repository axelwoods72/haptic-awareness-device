#pragma once

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "esp_err.h"

#define AMP_THRES 15000
#define DC_CUTOFF_HZ 10.0f
#define N_MICS 3
#define MIC_FRAME_SAMPLES 256
#define MIC_SAMPLE_RATE 48000
#define EPSILON 1e-10f
#define MIC_RADIUS 0.1
#define V 330 // speed of sound
#define PI 3.14159265358979323846
// largest physically possible delay between two mics (spacing = R*sqrt(3)) in samples, +2 margin for placement error
#define MAX_LAG ((int)(MIC_RADIUS * 1.7320508 / V * MIC_SAMPLE_RATE) + 2)

esp_err_t mic_driver_init(void);
esp_err_t mic_driver_read_stereo(int32_t* out_buf_left, int32_t* out_buf_right, size_t* bytes_read);
esp_err_t mic_driver_read_mono(int32_t* out_buf, size_t* bytes_read);
void get_mic_positions(float pos[N_MICS][2]);
void hpf(int32_t *buf, int chan_i, float *out);
void ifft(float* G);
float get_time_delay(float* G);
float tau_to_angle(float* tau);
void tdoa(float* tau, float* clean0, float* clean1, float* clean2);
float frame_peak(const float* clean);
int frame_has_signal(const float* clean);

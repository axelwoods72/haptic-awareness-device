#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#define MIC_FRAME_SAMPLES 256

esp_err_t mic_driver_init(void);
esp_err_t mic_driver_read_stereo(int32_t* out_buf_left, int32_t* out_buf_right, size_t* bytes_read);
esp_err_t mic_driver_read_mono(int32_t* out_buf, size_t* bytes_read);

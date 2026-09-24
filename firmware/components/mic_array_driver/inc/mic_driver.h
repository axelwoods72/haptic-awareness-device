#include <stdint.h>

#define MIC_FRAME_SAMPLES 256

esp_err_t mic_driver_init();
esp_err_t mic_driver_read(int32_t *out_buf, size_t len);
esp_err_t mic_driver_read_stereo(int32_t* out_buf_left, int32_t* out_buf_right, size_t* bytes_read);
esp_err_t mic_driver_read_mono(int32_t* out_buf, size_t* bytes_read);

// in main.c
void mic_task (void* pv);
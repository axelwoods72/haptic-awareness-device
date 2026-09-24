#include "mic_driver.h"
#include "esp_err.h"
#include "driver/i2s_std.h"
#include "gpio.h"

static i2s_chan_handle_t rx0_handle;
static i2s_chan_handle_t rx1_handle;

// create worker classes and set up i2s buses
esp_err_t mic_driver_init() {

    esp_err_t err;
    
    // reserve the stereo bus
    i2s_chan_config_t chan_cfg0 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    err = i2s_new_channel(&chan_cfg0, NULL, &rx0_handle);
    if (err != ESP_OK) {
        return err;
    }

    // configure the stereo bus
    i2s_std_config_t std_cfg0 = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),   // sample rate
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT, // bit width of each sample
            I2S_SLOT_MODE_STEREO
        ),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,      
            .bclk = BCLK_PIN,
            .ws   = WS_PIN,
            .dout = I2S_GPIO_UNUSED,
            .din  = SD0_PIN,
        },
    };
    err = i2s_channel_init_std_mode(rx0_handle, &std_cfg0);
    if (err != ESP_OK) {
        return err;
    }

    // reserve the mono bus
    i2s_chan_config_t chan_cfg1 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
    err = i2s_new_channel(&chan_cfg1, NULL, &rx1_handle);
    if (err != ESP_OK) {
        return err;
    }

    // configure the mono bus
    i2s_std_config_t std_cfg1 = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),   // sample rate
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_32BIT, // bit width of each sample
            I2S_SLOT_MODE_MONO
        ),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,      
            .bclk = BCLK_PIN,
            .ws   = WS_PIN,
            .dout = I2S_GPIO_UNUSED,
            .din  = SD1_PIN,
        },
    };
    err = i2s_channel_init_std_mode(rx1_handle, &std_cfg1);
    if (err != ESP_OK) {
        return err;
    }

    // start WS and BCLK at same time
    err = i2s_channel_enable(rx0_handle);
    if (err != ESP_OK) {
        return err;
    }
    err = i2s_channel_enable(rx1_handle);

    return err;
}

// read in MIC_FRAME_SAMPLES from stereo channel
esp_err_t mic_driver_read_stereo(int32_t* out_buf_left, int32_t* out_buf_right, size_t* bytes_read) {
    int32_t raw_buf[MIC_FRAME_SAMPLES * 2];  // stereo interleaved: [L0, R0, L1, R1, ...]
    esp_err_t err = i2s_channel_read(rx0_handle, raw_buf, MIC_FRAME_SAMPLES * 2 * sizeof(int32_t), bytes_read, pdMS_TO_TICKS(1000));

    for (int i = 0; i < *bytes_read / (2 * sizeof(int32_t)); i++) {
        out_buf_left[i] = raw_buf[2 * i];
        out_buf_right[i] = raw_buf[2 * i + 1];
    }
    return err;
}

// read in MIC_FRAME_SAMPLES from mono channel
esp_err_t mic_driver_read_mono(int32_t* out_buf, size_t* bytes_read) {
    esp_err_t err = i2s_channel_read(rx1_handle, out_buf, MIC_FRAME_SAMPLES * sizeof(int32_t), bytes_read, pdMS_TO_TICKS(1000));
    return err;
}
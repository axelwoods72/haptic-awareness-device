#include "driver/i2s_std.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "mic_driver.h"
#include "gpio.h"
#include "esp_dsp.h"

static i2s_chan_handle_t rx0_handle;
static i2s_chan_handle_t rx1_handle;

// hpf
static float dc_x_prev[3] = {0}; // previous raw input, per channel (0=mic0,1=mic1,2=mic2)
static float dc_y_prev[3] = {0}; // previous filtered output, per channel

// for calculating tau
static float wind[MIC_FRAME_SAMPLES] __attribute__((aligned(16)));
static float win0[MIC_FRAME_SAMPLES * 2] __attribute__((aligned(16)));
static float win1[MIC_FRAME_SAMPLES * 2] __attribute__((aligned(16)));
static float win2[MIC_FRAME_SAMPLES * 2] __attribute__((aligned(16)));
static float G01[MIC_FRAME_SAMPLES * 2] __attribute__((aligned(16)));
static float G02[MIC_FRAME_SAMPLES * 2] __attribute__((aligned(16)));
static float G12[MIC_FRAME_SAMPLES * 2] __attribute__((aligned(16)));


// for calcualting theta
static float a_coef[N_MICS]; // a_p for each mic pair
static float b_coef[N_MICS]; // b_p for each mic pair
static float S_aa, S_ab, S_bb; // sums of each a^2, b^2 and a*b
static float inv_det;          // 1/det

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
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),   // sample rate
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

    // mono bus intentionally shares BCLK/WS with the stereo bus, silence the "already reserved" warnings
    esp_log_level_set("i2s_common", ESP_LOG_ERROR);

    // reserve the mono bus
    i2s_chan_config_t chan_cfg1 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
    err = i2s_new_channel(&chan_cfg1, NULL, &rx1_handle);
    if (err != ESP_OK) {
        return err;
    }

    // configure the mono bus
    i2s_std_config_t std_cfg1 = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),   // sample rate
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

    // start both i2s buses at same time
    err = i2s_channel_enable(rx1_handle);
    if (err != ESP_OK) {
        return err;
    }
    err = i2s_channel_enable(rx0_handle);
    if (err != ESP_OK) {
        return err;
    }

    // initialise digital signal processing tool
    err = dsps_fft2r_init_fc32(NULL, CONFIG_DSP_MAX_FFT_SIZE);
    if (err  != ESP_OK) {
        return err;
    }

    // Generate hann window coefficients
    dsps_wind_hann_f32(wind, MIC_FRAME_SAMPLES);

    // calculate least squares coefficients
    float pos[N_MICS][2];
    get_mic_positions(pos);

    int pair_i[N_MICS] = {0, 0, 1};
    int pair_j[N_MICS] = {1, 2, 2};

    S_aa = S_ab = S_bb = 0.0f;
    for (int p = 0; p < N_MICS; p++) {
        int i = pair_i[p], j = pair_j[p];
        a_coef[p] = (pos[j][0] - pos[i][0]) / V;
        b_coef[p] = (pos[j][1] - pos[i][1]) / V;
        S_aa += a_coef[p] * a_coef[p];
        S_ab += a_coef[p] * b_coef[p];
        S_bb += b_coef[p] * b_coef[p];
    }

    float det = S_aa * S_bb - S_ab * S_ab;
    inv_det = 1.0f / det;

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

    *bytes_read = *bytes_read / 2;
    return err;
}

// read in MIC_FRAME_SAMPLES from mono channel
esp_err_t mic_driver_read_mono(int32_t* out_buf, size_t* bytes_read) {
    esp_err_t err = i2s_channel_read(rx1_handle, out_buf, MIC_FRAME_SAMPLES * sizeof(int32_t), bytes_read, pdMS_TO_TICKS(1000));
    return err;
}

// peak absolute amplitude of a filtered frame, in 24-bit counts
float frame_peak(const float* clean) {
    float peak = 0.0f;
    for (int i = 0; i < MIC_FRAME_SAMPLES; i++) {
        float a = fabsf(clean[i]);
        if (a > peak) {
            peak = a;
        }
    }
    return peak / 256.0f;
}

// return 1 if the frame's buffer amplitude reaches threshold
int frame_has_signal(const float* clean) {
    return frame_peak(clean) > AMP_THRES;
}

void ifft(float* G) {
    // conjugate
    for (int i = 0; i < MIC_FRAME_SAMPLES; i++) {
        G[2*i+1] = -G[2*i+1];
    }

    // fft
    dsps_fft2r_fc32(G, MIC_FRAME_SAMPLES);
    dsps_bit_rev_fc32(G, MIC_FRAME_SAMPLES);

    // conjugate and scale
    for (int i = 0; i < MIC_FRAME_SAMPLES; i++) {
        G[2*i] = G[2*i] / MIC_FRAME_SAMPLES;
        G[2*i+1] = -G[2*i+1] / MIC_FRAME_SAMPLES; 
    }
}

float get_time_delay(float* G) {
    // Find peak magnitude in r[n], handling wraparound for negative lags
    float max_val = -1.0f;
    int max_idx = 0;
    for (int n = 0; n < MIC_FRAME_SAMPLES; n++) {
        float re = G[2*n] / MIC_FRAME_SAMPLES;   // conj+scale: real part sign doesn't matter for magnitude
        float im = -G[2*n+1] / MIC_FRAME_SAMPLES;
        float mag = sqrtf(re*re + im*im);
        if (mag > max_val) {
            max_val = mag;
            max_idx = n;
        }
    }

    // Handle wraparound: lags > N/2 represent negative delays
    int lag = (max_idx > MIC_FRAME_SAMPLES/2) ? (max_idx - MIC_FRAME_SAMPLES) : max_idx;
    return (float) lag / (float) MIC_SAMPLE_RATE; // seconds
}

void hpf(int32_t *buf, int chan_i, float *out) {
    const float alpha = 1.0f - (2.0f * PI * DC_CUTOFF_HZ / MIC_SAMPLE_RATE);
    for (int i = 0; i < MIC_FRAME_SAMPLES; i++) {
        float x = (float)buf[i];
        float y = x - dc_x_prev[chan_i] + alpha * dc_y_prev[chan_i];
        out[i] = y;
        dc_x_prev[chan_i] = x;
        dc_y_prev[chan_i] = y;
    }
}

void tdoa(float* tau, float* clean0, float* clean1, float* clean2) {
    // apply window and make interleaved complex with 0 imaginary part
    for (int i = 0; i < MIC_FRAME_SAMPLES; i++) {
        win0[i * 2] = clean0[i] * wind[i];
        win1[i * 2] = clean1[i] * wind[i];
        win2[i * 2] = clean2[i] * wind[i];

        win0[i * 2 + 1] = 0.0f;
        win1[i * 2 + 1] = 0.0f;
        win2[i * 2 + 1] = 0.0f;
    }

    // fft each signal (bin values reversed)
    dsps_fft2r_fc32(win0, MIC_FRAME_SAMPLES);
    dsps_fft2r_fc32(win1, MIC_FRAME_SAMPLES);
    dsps_fft2r_fc32(win2, MIC_FRAME_SAMPLES);
    // reverse bits
    dsps_bit_rev_fc32(win0, MIC_FRAME_SAMPLES);
    dsps_bit_rev_fc32(win1, MIC_FRAME_SAMPLES);
    dsps_bit_rev_fc32(win2, MIC_FRAME_SAMPLES);

    // cross-power spectrum and normalise for each pair
    for (int i = 0; i < MIC_FRAME_SAMPLES; i++) {
        float r0 = win0[2 * i], r1 = win1[2 * i], r2 = win2[2 * i];
        float c0 = win0[2 * i + 1], c1 = win1[2 * i + 1], c2 = win2[2 * i + 1];

        float r01 = r0 * r1 + c0 * c1, r02 = r0 * r2 + c0 * c2, r12 = r1 * r2 + c1 * c2;
        float c01 = c0 * r1 - r0 * c1, c02 = c0 * r2 - r0 * c2, c12 = c1 * r2 - r1 * c2;

        float mag = sqrtf(r01 * r01 + c01 * c01) + EPSILON;
        G01[2 * i] = r01 / mag;
        G01[2 * i + 1] = c01 / mag;

        mag = sqrtf(r02 * r02 + c02 * c02) + EPSILON;
        G02[2 * i] = r02 / mag;
        G02[2 * i + 1] = c02 / mag;

        mag = sqrtf(r12 * r12 + c12 * c12) + EPSILON;
        G12[2 * i] = r12 / mag;
        G12[2 * i + 1] = c12 / mag;
    }

    // ifft = (1/N) * conj(FFT(conj(X)))
    ifft(G01);
    ifft(G02);
    ifft(G12);

    tau[0] = get_time_delay(G01);
    tau[1] = get_time_delay(G02);
    tau[2] = get_time_delay(G12);
}

void get_mic_positions (float pos[N_MICS][2]) {
    pos[0][0] = MIC_RADIUS * sinf(0 * (PI / 180));
    pos[0][1] = MIC_RADIUS * cosf(0 * (PI / 180));
    pos[1][0] = MIC_RADIUS * sinf(-120 * (PI / 180));
    pos[1][1] = MIC_RADIUS * cosf(-120 * (PI / 180));
    pos[2][0] = MIC_RADIUS * sinf(120 * (PI / 180));
    pos[2][1] = MIC_RADIUS * cosf(120 * (PI / 180));
}

// 0 degrees is forward, negative to the left, positive to the right
float tau_to_angle(float* tau) {
    float S_at = 0.0f, S_bt = 0.0f;
    for (int i = 0; i < N_MICS; i++) {
        S_at += a_coef[i] * tau[i];
        S_bt += b_coef[i] * tau[i];
    }

    float ux = (S_at * S_bb - S_bt * S_ab) * inv_det;
    float uy = (S_bt * S_aa - S_at * S_ab) * inv_det;

    return atan2f(ux, uy) * (180.0f / (float) PI);
}
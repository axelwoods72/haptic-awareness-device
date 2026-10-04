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
static float hpf_coef[5];          // b0, b1, b2, a1, a2
static float hpf_w[N_MICS][2] = {0}; // filter state per mic, carried across frames

// for calculating tau
static float wind[MIC_FRAME_SAMPLES] __attribute__((aligned(16)));
static float win0[MIC_FRAME_SAMPLES * 2] __attribute__((aligned(16)));
static float win1[MIC_FRAME_SAMPLES * 2] __attribute__((aligned(16)));
static float win2[MIC_FRAME_SAMPLES * 2] __attribute__((aligned(16)));
static float G01[MIC_FRAME_SAMPLES * 2] __attribute__((aligned(16)));
static float G02[MIC_FRAME_SAMPLES * 2] __attribute__((aligned(16)));
static float G12[MIC_FRAME_SAMPLES * 2] __attribute__((aligned(16)));

// running average of each pair's cross-power spectrum
static float avg01[MIC_FRAME_SAMPLES * 2], avg02[MIC_FRAME_SAMPLES * 2], avg12[MIC_FRAME_SAMPLES * 2];

// for calcualting theta
static float a_coef[N_MICS]; // a_p for each mic pair
static float b_coef[N_MICS]; // b_p for each mic pair
static float S_aa, S_ab, S_bb; // sums of each a^2, b^2 and a*b
static float inv_det;          // 1/det

// expected lag in samples for each candidate angle and mic pair
static float srp_lag[SRP_N_ANGLES][N_MICS];

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

    err = dsps_biquad_gen_hpf_f32(hpf_coef, HPF_CUTOFF_HZ / MIC_SAMPLE_RATE, HPF_Q);
    if (err != ESP_OK) {
        return err;
    }

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

    // tau = a*sin(theta) + b*cos(theta)
    for (int a = 0; a < SRP_N_ANGLES; a++) {
        float th = a * SRP_STEP_DEG * (float) PI / 180.0f;
        for (int p = 0; p < N_MICS; p++) {
            srp_lag[a][p] = (a_coef[p] * sinf(th) + b_coef[p] * cosf(th)) * MIC_SAMPLE_RATE;
        }
    }

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

// value of the cross-correlation at a signed lag, negative lags wrap to the end of the circular buffer
static float corr_at(float* G, int lag) {
    int n = (lag + MIC_FRAME_SAMPLES) % MIC_FRAME_SAMPLES;
    return G[2 * n]; // cross-correlation of two real signals is real
}

float get_time_delay(float* G) {
    // only search lags the mic spacing can physically produce
    float max_val = -INFINITY;
    int best_lag = 0;
    for (int lag = -MAX_LAG; lag <= MAX_LAG; lag++) {
        float r = corr_at(G, lag);
        if (r > max_val) {
            max_val = r;
            best_lag = lag;
        }
    }

    // parabolic interpolation through the peak and its neighbours for sub-sample delay
    float y0 = corr_at(G, best_lag - 1), y1 = max_val, y2 = corr_at(G, best_lag + 1);
    float denom = y0 - 2.0f * y1 + y2;
    float offset = (fabsf(denom) > EPSILON) ? 0.5f * (y0 - y2) / denom : 0.0f;

    return ((float) best_lag + offset) / (float) MIC_SAMPLE_RATE; // seconds
}

void hpf(int32_t *buf, int chan_i, float *out) {
    for (int i = 0; i < MIC_FRAME_SAMPLES; i++) {
        out[i] = (float)buf[i];
    }
    dsps_biquad_f32(out, out, MIC_FRAME_SAMPLES, hpf_coef, hpf_w[chan_i]);
}

// PHAT-normalised copy of an averaged cross-spectrum bin
static void phat_bin(float* G, const float* avg, int i) {
    float re = avg[2 * i], im = avg[2 * i + 1];
    float mag = powf(hypotf(re, im) + EPSILON, 0.75f);
    G[2 * i] = re / mag;
    G[2 * i + 1] = im / mag;
}

void tdoa_reset(void) {
    memset(avg01, 0, sizeof(avg01));
    memset(avg02, 0, sizeof(avg02));
    memset(avg12, 0, sizeof(avg12));
}

static void avg_bin(float* avg, int i, float re, float im) {
    avg[2 * i] = (1.0f - CSD_ALPHA) * avg[2 * i] + CSD_ALPHA * re;
    avg[2 * i + 1] = (1.0f - CSD_ALPHA) * avg[2 * i + 1] + CSD_ALPHA * im;
}

// fills G01/G02/G12 with each pair's GCC-PHAT cross-correlation
static void gcc_phat(float* clean0, float* clean1, float* clean2) {
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

    // bin k holds frequency k * fs / N, bins above N/2 mirror the negative frequencies
    const int k_min = (int)ceilf(GCC_MIN_HZ * MIC_FRAME_SAMPLES / MIC_SAMPLE_RATE);
    const int k_max = (int)floorf(GCC_MAX_HZ * MIC_FRAME_SAMPLES / MIC_SAMPLE_RATE);

    // cross-power spectrum, averaged over frames, then normalise for each pair
    for (int i = 0; i < MIC_FRAME_SAMPLES; i++) {
        int k = (i <= MIC_FRAME_SAMPLES / 2) ? i : MIC_FRAME_SAMPLES - i;
        if (k < k_min || k > k_max) {
            G01[2 * i] = G01[2 * i + 1] = 0.0f;
            G02[2 * i] = G02[2 * i + 1] = 0.0f;
            G12[2 * i] = G12[2 * i + 1] = 0.0f;
            continue;
        }

        float r0 = win0[2 * i], r1 = win1[2 * i], r2 = win2[2 * i];
        float c0 = win0[2 * i + 1], c1 = win1[2 * i + 1], c2 = win2[2 * i + 1];

        float r01 = r0 * r1 + c0 * c1, r02 = r0 * r2 + c0 * c2, r12 = r1 * r2 + c1 * c2;
        float c01 = c0 * r1 - r0 * c1, c02 = c0 * r2 - r0 * c2, c12 = c1 * r2 - r1 * c2;

        avg_bin(avg01, i, r01, c01);
        avg_bin(avg02, i, r02, c02);
        avg_bin(avg12, i, r12, c12);

        phat_bin(G01, avg01, i);
        phat_bin(G02, avg02, i);
        phat_bin(G12, avg12, i);
    }

    // ifft = (1/N) * conj(FFT(conj(X)))
    ifft(G01);
    ifft(G02);
    ifft(G12);
}

void tdoa(float* tau, float* clean0, float* clean1, float* clean2) {
    gcc_phat(clean0, clean1, clean2);

    tau[0] = get_time_delay(G01);
    tau[1] = get_time_delay(G02);
    tau[2] = get_time_delay(G12);
}

// linear interpolation of the cross-correlation at a fractional lag
static float corr_interp(float* G, float lag) {
    int l0 = (int) floorf(lag);
    float frac = lag - (float) l0;
    return (1.0f - frac) * corr_at(G, l0) + frac * corr_at(G, l0 + 1);
}

// steered response power: score every candidate angle by summing all pairs' correlation at the lags that angle predicts
float srp_phat(float* clean0, float* clean1, float* clean2, float* score) {
    gcc_phat(clean0, clean1, clean2);

    float* G[N_MICS] = {G01, G02, G12};
    float srp[SRP_N_ANGLES];
    int best = 0;
    for (int a = 0; a < SRP_N_ANGLES; a++) {
        srp[a] = 0.0f;
        for (int p = 0; p < N_MICS; p++) {
            srp[a] += corr_interp(G[p], srp_lag[a][p]);
        }
        if (srp[a] > srp[best]) {
            best = a;
        }
    }

    // parabolic interpolation across neighbouring angles, wrapping at 360
    float y0 = srp[(best - 1 + SRP_N_ANGLES) % SRP_N_ANGLES];
    float y1 = srp[best];
    float y2 = srp[(best + 1) % SRP_N_ANGLES];
    float denom = y0 - 2.0f * y1 + y2;
    float offset = (fabsf(denom) > EPSILON) ? 0.5f * (y0 - y2) / denom : 0.0f;

    *score = y1;

    float deg = ((float) best + offset) * SRP_STEP_DEG;
    if (deg > 180.0f) {
        deg -= 360.0f;
    }
    return deg * (float) PI / 180.0f;
}

void get_mic_positions (float pos[N_MICS][2]) {
    pos[0][0] = MIC_RADIUS * sinf(0 * (PI / 180));
    pos[0][1] = MIC_RADIUS * cosf(0 * (PI / 180));
    pos[1][0] = MIC_RADIUS * sinf(-120 * (PI / 180));
    pos[1][1] = MIC_RADIUS * cosf(-120 * (PI / 180));
    pos[2][0] = MIC_RADIUS * sinf(120 * (PI / 180));
    pos[2][1] = MIC_RADIUS * cosf(120 * (PI / 180));
}

// 0 rad is forward, negative to the left, positive to the right
float tau_to_angle(float* tau) {
    float S_at = 0.0f, S_bt = 0.0f;
    for (int i = 0; i < N_MICS; i++) {
        S_at += a_coef[i] * tau[i];
        S_bt += b_coef[i] * tau[i];
    }

    float ux = (S_at * S_bb - S_bt * S_ab) * inv_det;
    float uy = (S_bt * S_aa - S_at * S_ab) * inv_det;

    return atan2f(ux, uy);
}
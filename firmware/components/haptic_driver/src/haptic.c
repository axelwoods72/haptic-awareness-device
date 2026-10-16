#include "haptic.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <stdbool.h>
#include <stddef.h>

static bool ready = false;
static unsigned configured = 0;

// Stop any channels that were successfully configured.
static void stop_configured(void)
{
    for (unsigned i = 0; i < configured; ++i) {
        (void)ledc_stop(
            LEDC_LOW_SPEED_MODE,
            (ledc_channel_t)i,
            0
        );
    }
}

esp_err_t haptic_init(const haptic_config_t *config)
{
    if (ready)
        return ESP_ERR_INVALID_STATE;

    if (config == NULL)
        return ESP_ERR_INVALID_ARG;

    // Validate the four IA GPIOs. IB inputs are grounded.
    for (int i = 0; i < HAPTIC_COUNT; ++i) {
        if (!GPIO_IS_VALID_OUTPUT_GPIO(config->ia[i]))
            return ESP_ERR_INVALID_ARG;

        for (int j = 0; j < i; ++j) {
            if (config->ia[i] == config->ia[j])
                return ESP_ERR_INVALID_ARG;
        }
    }

    // Configure the four GPIOs as outputs.
    gpio_config_t gpio = {
        .mode = GPIO_MODE_OUTPUT
    };

    for (int i = 0; i < HAPTIC_COUNT; ++i)
        gpio.pin_bit_mask |= 1ULL << config->ia[i];

    esp_err_t err = gpio_config(&gpio);
    if (err != ESP_OK)
        return err;

    // Start with all outputs LOW.
    for (int i = 0; i < HAPTIC_COUNT; ++i) {
        err = gpio_set_level(config->ia[i], 0);
        if (err != ESP_OK)
            return err;
    }

    // Shared PWM timer: 20 kHz, 10-bit resolution.
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK
    };

    err = ledc_timer_config(&timer);
    if (err != ESP_OK)
        return err;

    // Connect one PWM channel to each motor's IA pin.
    configured = 0;

    for (int i = 0; i < HAPTIC_COUNT; ++i) {
        ledc_channel_config_t channel = {
            .gpio_num = config->ia[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .timer_sel = LEDC_TIMER_0,
            .intr_type = LEDC_INTR_DISABLE,
            .duty = 0,
            .hpoint = 0
        };

        err = ledc_channel_config(&channel);

        if (err != ESP_OK) {
            stop_configured();
            return err;
        }

        ++configured;
    }

    ready = true;
    return ESP_OK;
}

esp_err_t haptic_stop(void)
{
    if (!ready)
        return ESP_ERR_INVALID_STATE;

    esp_err_t result = ESP_OK;

    for (int i = 0; i < HAPTIC_COUNT; ++i) {
        esp_err_t err = ledc_stop(
            LEDC_LOW_SPEED_MODE,
            (ledc_channel_t)i,
            0
        );

        if (err != ESP_OK && result == ESP_OK)
            result = err;
    }

    return result;
}

esp_err_t haptic_set(float theta_deg, float strength)
{
    if (!ready)
        return ESP_ERR_INVALID_STATE;

    float duties[HAPTIC_COUNT];

    esp_err_t err = haptic_map(theta_deg, strength, duties);

    if (err != ESP_OK) {
        (void)haptic_stop();
        return err;
    }

    for (int i = 0; i < HAPTIC_COUNT; ++i) {
        unsigned duty =
            (unsigned)(duties[i] * 1023.0f + 0.5f);

        err = ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            (ledc_channel_t)i,
            duty
        );

        if (err == ESP_OK) {
            err = ledc_update_duty(
                LEDC_LOW_SPEED_MODE,
                (ledc_channel_t)i
            );
        }

        if (err != ESP_OK) {
            (void)haptic_stop();
            return err;
        }
    }

    return ESP_OK;
}
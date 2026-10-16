#pragma once

#include "esp_err.h"

// Motor order.
enum {
    HAPTIC_FRONT = 0,
    HAPTIC_RIGHT,
    HAPTIC_BACK,
    HAPTIC_LEFT,
    HAPTIC_COUNT
};

// Four IA GPIOs. IB inputs are physically grounded.
typedef struct {
    int ia[HAPTIC_COUNT];
} haptic_config_t;

// Calculate four motor strengths from angle and strength.
esp_err_t haptic_map(
    float theta_deg,
    float strength,
    float output[HAPTIC_COUNT]
);

// Prepare the motor outputs.
esp_err_t haptic_init(const haptic_config_t *config);

// Stop all motors.
esp_err_t haptic_stop(void);

// Drive the motors using amplitude panning.
esp_err_t haptic_set(float theta_deg, float strength);
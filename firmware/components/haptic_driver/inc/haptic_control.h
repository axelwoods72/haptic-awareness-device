#pragma once
#include "haptic.h"
#include <stdbool.h>
#include <stdint.h>

#define HAPTIC_COMMAND_TIMEOUT_MS 250U

typedef struct {
    float theta_deg;
    float strength;
    bool active;
    uint32_t updated_at_ms;
} haptic_command_t;

/* Called by the single motor task. Missing, disabled or expired input stops
 * every motor. Unsigned subtraction supports the millisecond counter wrap. */
esp_err_t haptic_apply_command(const haptic_command_t *command, uint32_t now_ms);

/* Microphones: 0 front, +90 right, +/-180 back, -90 left.
 * Preserve the team's mic - (head yaw - body yaw) transform. The sign and
 * sensor mounting must be checked using a fixed sound source on hardware. */
esp_err_t haptic_body_angle(float mic_deg, float head_deg, float body_deg,
                            float *theta_deg);

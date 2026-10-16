#include "haptic_control.h"
#include <math.h>
#include <stddef.h>

esp_err_t haptic_apply_command(const haptic_command_t *command,
                               uint32_t now_ms) {
  if (command == NULL || !command->active ||
      (uint32_t)(now_ms - command->updated_at_ms) >= HAPTIC_COMMAND_TIMEOUT_MS)
    return haptic_stop();
  return haptic_set(command->theta_deg, command->strength);
}

esp_err_t haptic_body_angle(float mic_deg, float head_deg, float body_deg,
                            float *theta_deg) {
  if (theta_deg == NULL)
    return ESP_ERR_INVALID_ARG;
  *theta_deg = 0.0f;
  if (!isfinite(mic_deg) || !isfinite(head_deg) || !isfinite(body_deg))
    return ESP_ERR_INVALID_ARG;
  /* Reduce each input before subtraction to avoid overflow for large floats. */
  float angle = fmodf(mic_deg, 360.0f) - fmodf(head_deg, 360.0f) +
                fmodf(body_deg, 360.0f);
  angle = fmodf(angle, 360.0f);
  if (angle > 180.0f)
    angle -= 360.0f;
  if (angle < -180.0f)
    angle += 360.0f;
  *theta_deg = angle;
  return ESP_OK;
}

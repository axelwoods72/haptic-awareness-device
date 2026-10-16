#include "haptic.h"
#include <math.h>

esp_err_t haptic_map(float theta, float strength, float out[HAPTIC_COUNT]) {
    if (!out) return ESP_ERR_INVALID_ARG;
    for (int i = 0; i < HAPTIC_COUNT; ++i) out[i] = 0;
    if (!isfinite(theta) || !isfinite(strength) || strength < 0 || strength > 1)
        return ESP_ERR_INVALID_ARG;
    theta = fmodf(theta, 360.0f);
    if (theta < 0) theta += 360.0f;
    float sector = theta / 90.0f;
    int first = (int)sector;
    // Guard rounding at the wrap boundary.
    if (first == HAPTIC_COUNT) first = 0;
    float fraction = sector - floorf(sector);
    out[first] = strength * (1.0f - fraction);
    out[(first + 1) % HAPTIC_COUNT] = strength * fraction;
    return ESP_OK;
}

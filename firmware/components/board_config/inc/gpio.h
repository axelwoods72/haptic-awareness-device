#pragma once

#include "sdkconfig.h"

#if !CONFIG_IDF_TARGET_ESP32S3
#error "This pin map is for Waveshare ESP32-S3-Zero. Run idf.py set-target esp32s3."
#endif

/* Microphones: retain the team's existing wiring. */
#define BCLK_PIN 13
#define WS_PIN 2
#define SD0_PIN 11
#define SD1_PIN 10

/* IMUs: both buses use accessible edge pins on ESP32-S3-Zero.
 * REWIRE head from 21/22 to 8/9, body from 32/33 to 12/1.
 * The old assignments were not suitable for this board. */
#define IMU_HEAD_SDA_PIN 8
#define IMU_HEAD_SCL_PIN 9
#define IMU_BODY_SDA_PIN 12
#define IMU_BODY_SCL_PIN 1

/* Motors: front, right, back, left; one PWM wire to each L9110 IA.
 * All four IB inputs are physically grounded. */
#define HAPTIC_FRONT_PIN 4
#define HAPTIC_RIGHT_PIN 5
#define HAPTIC_BACK_PIN 6
#define HAPTIC_LEFT_PIN 7

/* These are external, non-strapping edge pins, excluding USB, RGB LED
 * and flash/PSRAM. Compile-time checks catch reserved or overlapping pins. */
#define BOARD_PIN_OK(p) (((p) == 1) || ((p) == 2) || \
                        ((p) >= 4 && (p) <= 13) || \
                        ((p) == 17) || ((p) == 18) || \
                        ((p) == 43) || ((p) == 44))
#define BOARD_CHECK_PIN(p) _Static_assert(BOARD_PIN_OK(p), #p " is not an allowed S3-Zero pin")
BOARD_CHECK_PIN(BCLK_PIN);
BOARD_CHECK_PIN(WS_PIN);
BOARD_CHECK_PIN(SD0_PIN);
BOARD_CHECK_PIN(SD1_PIN);
BOARD_CHECK_PIN(IMU_HEAD_SDA_PIN);
BOARD_CHECK_PIN(IMU_HEAD_SCL_PIN);
BOARD_CHECK_PIN(IMU_BODY_SDA_PIN);
BOARD_CHECK_PIN(IMU_BODY_SCL_PIN);
BOARD_CHECK_PIN(HAPTIC_FRONT_PIN);
BOARD_CHECK_PIN(HAPTIC_RIGHT_PIN);
BOARD_CHECK_PIN(HAPTIC_BACK_PIN);
BOARD_CHECK_PIN(HAPTIC_LEFT_PIN);
#define BOARD_USED_PINS ((1ULL << BCLK_PIN) | (1ULL << WS_PIN) | \
    (1ULL << SD0_PIN) | (1ULL << SD1_PIN) | \
    (1ULL << IMU_HEAD_SDA_PIN) | (1ULL << IMU_HEAD_SCL_PIN) | \
    (1ULL << IMU_BODY_SDA_PIN) | (1ULL << IMU_BODY_SCL_PIN) | \
    (1ULL << HAPTIC_FRONT_PIN) | (1ULL << HAPTIC_RIGHT_PIN) | \
    (1ULL << HAPTIC_BACK_PIN) | (1ULL << HAPTIC_LEFT_PIN))
_Static_assert(__builtin_popcountll(BOARD_USED_PINS) == 12,
               "GPIO conflict between microphones, IMUs or motors");

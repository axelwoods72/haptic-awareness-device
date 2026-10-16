#pragma once

#include "sdkconfig.h"

#if CONFIG_IDF_TARGET_ESP32S3 // Waveshare ESP32-S3-Zero

#define BCLK_PIN 13
#define WS_PIN 2
#define SD0_PIN 11
#define SD1_PIN 10

#define IMU_HEAD_SDA_PIN 8
#define IMU_HEAD_SCL_PIN 9
#define IMU_BODY_SDA_PIN 12
#define IMU_BODY_SCL_PIN 1

#define HAPTIC_FRONT_PIN 4
#define HAPTIC_RIGHT_PIN 5
#define HAPTIC_BACK_PIN 6
#define HAPTIC_LEFT_PIN 7

#else // Classic ESP32 bench board

#define BCLK_PIN 26
#define WS_PIN 25
#define SD0_PIN 27
#define SD1_PIN 14

#define IMU_HEAD_SDA_PIN 21
#define IMU_HEAD_SCL_PIN 22
#define IMU_BODY_SDA_PIN 18
#define IMU_BODY_SCL_PIN 19

#define HAPTIC_FRONT_PIN 32
#define HAPTIC_RIGHT_PIN 33
#define HAPTIC_BACK_PIN 23
#define HAPTIC_LEFT_PIN 4

#endif
#include "imu_driver.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/i2c_types.h"

#define MPU6050_ADDR 0x68
#define MMC5603_ADDR 0x30

#define OUTPUT_DATA_RATE 50 // in Hz (max = 150)

// MMC5603 registers
#define MMC5603_REG_XOUT0 0x00
#define MMC5603_REG_ODR 0x1A
#define MMC5603_REG_CTRL0 0x1B
#define MMC5603_REG_CTRL2 0x1D

#define MMC5603_CMM_FREQ_EN (1 << 7) // bit 7 of Internal Control 0
#define MMC5603_CMM_EN (1 << 4)      // bit 4 of Internal Control 2
#define MMC5603_AUTO_SR_EN (1 << 5)  // bit 5 of Internal Control 0

esp_err_t imu_driver_setup(imu_handle_t *imu_handle,
                           const imu_config_t *imu_cfg) {

  // Configure bus
  i2c_master_bus_config_t bus_config = {
      .i2c_port = imu_cfg->port,
      .sda_io_num = imu_cfg->sda_pin,
      .scl_io_num = imu_cfg->scl_pin,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  esp_err_t err = i2c_new_master_bus(&bus_config, &imu_handle->bus_handle);
  if (err != ESP_OK)
    return err;

  ESP_LOGI("imu", "port %d: MPU 0x68 %s, MMC 0x30 %s", imu_cfg->port,
           i2c_master_probe(imu_handle->bus_handle, 0x68, 100) == ESP_OK
               ? "OK"
               : "missing",
           i2c_master_probe(imu_handle->bus_handle, 0x30, 100) == ESP_OK
               ? "OK"
               : "missing");

  // Add device - MPU6050 IMU
  i2c_device_config_t dev1_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = MPU6050_ADDR,
      .scl_speed_hz = 400000,
  };
  err = i2c_master_bus_add_device(imu_handle->bus_handle, &dev1_config,
                                  &imu_handle->mpu6050_handle);
  if (err != ESP_OK)
    return err;

  // Reset MPU6050 - it stays powered across ESP32 resets, so clear any
  // state left over from a previous run
  uint8_t reset_cmd[2] = {0x6B, 0x80}; // PWR_MGMT_1: DEVICE_RESET
  err = i2c_master_transmit(imu_handle->mpu6050_handle, reset_cmd,
                            sizeof(reset_cmd), 100);
  if (err != ESP_OK)
    return err;
  vTaskDelay(pdMS_TO_TICKS(100)); // wait for reset to complete

  // Wake up MPU6050 - it powers on in sleep mode.
  // Write 0x00 to PWR_MGMT_1 (register 0x6B) to clear the SLEEP bit.
  uint8_t wake_cmd[2] = {0x6B, 0x00};
  err = i2c_master_transmit(imu_handle->mpu6050_handle, wake_cmd,
                            sizeof(wake_cmd), 100);
  if (err != ESP_OK)
    return err;

  // Add device - MMC5603 magnetometer
  i2c_device_config_t dev2_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = MMC5603_ADDR,
      .scl_speed_hz = 400000,
  };
  err = i2c_master_bus_add_device(imu_handle->bus_handle, &dev2_config,
                                  &imu_handle->mmc5603_handle);
  if (err != ESP_OK)
    return err;

  // Configure MMC5603 to continuous mode
  // Set output data rate
  uint8_t odr_cmd[2] = {MMC5603_REG_ODR, OUTPUT_DATA_RATE};
  err = i2c_master_transmit(imu_handle->mmc5603_handle, odr_cmd,
                            sizeof(odr_cmd), 100);
  if (err != ESP_OK)
    return err;

  // Calculate measurement period from ODR + enable automatic set/reset
  uint8_t cmm_freq_cmd[2] = {MMC5603_REG_CTRL0,
                             MMC5603_CMM_FREQ_EN | MMC5603_AUTO_SR_EN};
  err = i2c_master_transmit(imu_handle->mmc5603_handle, cmm_freq_cmd,
                            sizeof(cmm_freq_cmd), 100);
  if (err != ESP_OK)
    return err;

  // Start continuous mode
  uint8_t cmm_en_cmd[2] = {MMC5603_REG_CTRL2, MMC5603_CMM_EN};
  err = i2c_master_transmit(imu_handle->mmc5603_handle, cmm_en_cmd,
                            sizeof(cmm_en_cmd), 100);
  if (err != ESP_OK)
    return err;

  return ESP_OK;
}

esp_err_t imu_driver_read_accel_gyro(imu_handle_t *imu_handle,
                                     imu_data_t *out) {

  // Relevant registers of MPU6050
  uint8_t mpu_reg = 0x3B; // ACCEL_XOUT_H — start of the 14-byte burst
  uint8_t mpu_buf[14];

  // Read data into buffer
  esp_err_t err = i2c_master_transmit_receive(
      imu_handle->mpu6050_handle, &mpu_reg, 1, mpu_buf, sizeof(mpu_buf), 100);
  if (err != ESP_OK)
    return err;

  // Get data from buffer
  int16_t raw_ax = (mpu_buf[0] << 8) | mpu_buf[1];
  int16_t raw_ay = (mpu_buf[2] << 8) | mpu_buf[3];
  int16_t raw_az = (mpu_buf[4] << 8) | mpu_buf[5];
  // mpu_buf[6],mpu_buf[7] = temperature — don't need
  int16_t raw_gx = (mpu_buf[8] << 8) | mpu_buf[9];
  int16_t raw_gy = (mpu_buf[10] << 8) | mpu_buf[11];
  int16_t raw_gz = (mpu_buf[12] << 8) | mpu_buf[13];

  // Convert to g (range = +/- 2g)
  out->accel_x = raw_ax / 16384.0f;
  out->accel_y = raw_ay / 16384.0f;
  out->accel_z = raw_az / 16384.0f;
  // Convert to deg/s (range = +/- 250deg)
  out->gyro_x = raw_gx / 131.0f;
  out->gyro_y = raw_gy / 131.0f;
  out->gyro_z = raw_gz / 131.0f;

  return ESP_OK;
}

esp_err_t imu_driver_read_mag(imu_handle_t *imu_handle, imu_data_t *out) {
  uint8_t mmc_reg = MMC5603_REG_XOUT0;
  uint8_t mmc_buf[9];

  // Read data into buffer
  esp_err_t err = i2c_master_transmit_receive(
      imu_handle->mmc5603_handle, &mmc_reg, 1, mmc_buf, sizeof(mmc_buf), 100);
  if (err != ESP_OK)
    return err;

  // Get data from buffer
  uint32_t raw_mx = (mmc_buf[0] << 12) | (mmc_buf[1] << 4) | (mmc_buf[6] >> 4);
  uint32_t raw_my = (mmc_buf[2] << 12) | (mmc_buf[3] << 4) | (mmc_buf[7] >> 4);
  uint32_t raw_mz = (mmc_buf[4] << 12) | (mmc_buf[5] << 4) | (mmc_buf[8] >> 4);

  // Convert ADC to mG
  out->mag_x = ((int32_t)raw_mx - 524288) * 0.0625f;
  out->mag_y = ((int32_t)raw_my - 524288) * 0.0625f;
  out->mag_z = ((int32_t)raw_mz - 524288) * 0.0625f;

  return ESP_OK;
}
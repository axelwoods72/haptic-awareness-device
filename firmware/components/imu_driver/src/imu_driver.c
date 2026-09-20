#include "imu_driver.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "hal/i2c_types.h"

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define MPU6050_ADDR 0x68
#define MMC5603_ADDR 0x30

#define OUTPUT_DATA_RATE 50 // in Hz

// MMC5603 registers
#define MMC5603_REG_XOUT0 0x00
#define MMC5603_REG_ODR 0x1A
#define MMC5603_REG_CTRL0 0x1B
#define MMC5603_REG_CTRL2 0x1D

#define MMC5603_CMM_FREQ_EN (1 << 7) // bit 7 of Internal Control 0
#define MMC5603_CMM_EN (1 << 4)      // bit 4 of Internal Control 2

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t mpu6050_handle;
static i2c_master_dev_handle_t mmc5603_handle;

esp_err_t imu_driver_init(void) {

  // Configure bus
  i2c_master_bus_config_t bus_config = {
      .i2c_port = I2C_NUM_0,
      .sda_io_num = I2C_SDA_PIN,
      .scl_io_num = I2C_SCL_PIN,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };
  esp_err_t err = i2c_new_master_bus(&bus_config, &bus_handle);
  if (err != ESP_OK)
    return err;

  // Add device - MPU6050 IMU
  i2c_device_config_t dev1_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = MPU6050_ADDR,
      .scl_speed_hz = 400000,
  };
  err = i2c_master_bus_add_device(bus_handle, &dev1_config, &mpu6050_handle);
  if (err != ESP_OK)
    return err;

  // Wake up MPU6050 - it powers on in sleep mode.
  // Write 0x00 to PWR_MGMT_1 (register 0x6B) to clear the SLEEP bit.
  uint8_t wake_cmd[2] = {0x68, 0x00};
  return i2c_master_transmit(mpu6050_handle, wake_cmd, sizeof(wake_cmd), 100);

  // Add device - MMC5603 magnetometer
  i2c_device_config_t dev2_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = MMC5603_ADDR,
      .scl_speed_hz = 400000,
  };
  err = i2c_master_bus_add_device(bus_handle, &dev2_config, &mmc5603_handle);
  if (err != ESP_OK)
    return err;

  // Configure MMC5603 to continuous mode
  // Set output data rate
  uint8_t odr_cmd[2] = {MMC5603_REG_ODR, OUTPUT_DATA_RATE};
  err = i2c_master_transmit(mmc5603_handle, odr_cmd, sizeof(odr_cmd), 100);
  if (err != ESP_OK)
    return err;

  // Tell chip to calculate its internal measurement period from ODR
  uint8_t cmm_freq_cmd[2] = {MMC5603_REG_CTRL0, MMC5603_CMM_FREQ_EN};
  err = i2c_master_transmit(mmc5603_handle, cmm_freq_cmd, sizeof(cmm_freq_cmd),
                            100);
  if (err != ESP_OK)
    return err;

  // Start continuous mode
  uint8_t cmm_en_cmd[2] = {MMC5603_REG_CTRL2, MMC5603_CMM_EN};
  err =
      i2c_master_transmit(mmc5603_handle, cmm_en_cmd, sizeof(cmm_en_cmd), 100);
  if (err != ESP_OK)
    return err;
}

esp_err_t imu_driver_read(imu_data_t *out) {

  // Read MPU6050 first
  uint8_t mpu_reg = 0x3B; // ACCEL_XOUT_H — start of the 14-byte burst
  uint8_t mpu_buf[14];

  // Read data into buffer
  esp_err_t err = i2c_master_transmit_receive(mpu6050_handle, &mpu_reg, 1,
                                              mpu_buf, sizeof(mpu_buf), 100);
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

  // Read MMC5603 second
  uint8_t mmc_reg = MMC5603_REG_XOUT0;
  uint8_t mmc_buf[9];

  // Read data into buffer
  esp_err_t err2 = i2c_master_transmit_receive(mmc5603_handle, &mmc_reg, 1,
                                               mmc_buf, sizeof(mmc_buf), 100);
  if (err2 != ESP_OK)
    return err2;

  // Get data from buffer
  uint32_t raw_mx = (mmc_buf[0] << 12) | (mmc_buf[1] << 4) | (mmc_buf[6] >> 4);
  uint32_t raw_my = ((uint32_t)mmc_buf[2] << 12) | ((uint32_t)mmc_buf[3] << 4) |
                    (mmc_buf[7] >> 4);
  uint32_t raw_mz = ((uint32_t)mmc_buf[4] << 12) | ((uint32_t)mmc_buf[5] << 4) |
                    (mmc_buf[8] >> 4);

  // Convert ADC to mG
  out->mag_x = ((int32_t)raw_mx - 524288) * 0.0625f;
  out->mag_y = ((int32_t)raw_my - 524288) * 0.0625f;
  out->mag_z = ((int32_t)raw_mz - 524288) * 0.0625f;

  return ESP_OK;
}
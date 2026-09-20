#include "imu_driver.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "hal/i2c_types.h"

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define MPU6050_ADDR 0x68
#define MMC5603_ADDR 0x30

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

  // Wake the MPU6050 up - it powers on in sleep mode.
  // Write 0x00 to PWR_MGMT_1 (register 0x6B) to clear the SLEEP bit.
  uint8_t wake_cmd[2] = {0x68, 0x00};
  return i2c_master_transmit(mpu6050_handle, wake_cmd, sizeof(wake_cmd), 100);
}

esp_err_t imu_driver_read(imu_data_t *out) {

  uint8_t reg = 0x3B; // ACCEL_XOUT_H — start of the 14-byte burst
  uint8_t buf[14];

  // Read data from MPU6050
  esp_err_t err = i2c_master_transmit_receive(mpu6050_handle, &reg, 1, buf,
                                              sizeof(buf), 100);
  if (err != ESP_OK)
    return err;

  int16_t raw_ax = (buf[0] << 8) | buf[1];
  int16_t raw_ay = (buf[2] << 8) | buf[3];
  int16_t raw_az = (buf[4] << 8) | buf[5];
  // buf[6],buf[7] = temperature — don't need
  int16_t raw_gx = (buf[8] << 8) | buf[9];
  int16_t raw_gy = (buf[10] << 8) | buf[11];
  int16_t raw_gz = (buf[12] << 8) | buf[13];

  // Convert ADC to g (range = +/- 2g)
  out->accel_x = raw_ax / 16384.0f;
  out->accel_y = raw_ay / 16384.0f;
  out->accel_z = raw_az / 16384.0f;
  // Convert ADC to deg/s (range = +/- 250deg)
  out->gyro_x = raw_gx / 131.0f;
  out->gyro_y = raw_gy / 131.0f;
  out->gyro_z = raw_gz / 131.0f;

  return ESP_OK;
}
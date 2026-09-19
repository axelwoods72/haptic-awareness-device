#include "imu_driver.h"
#include "driver/i2c_master.h"

#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9
#define MPU6050_ADDR 0x68

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t mpu6050_handle;

esp_err_t imu_driver_init(void) {
  // Check for errors
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

  i2c_device_config_t dev_config = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = MPU6050_ADDR,
      .scl_speed_hz = 400000,
  };
  err = i2c_master_bus_add_device(bus_handle, &dev_config, &mpu6050_handle);
  if (err != ESP_OK)
    return err;

  // Wake the chip up
  uint8_t wake_cmd[2] = {0x68, 0x00};
  return i2c_master_transmit(mpu6050_handle, wake_cmd, sizeof(wake_cmd), 100);
}

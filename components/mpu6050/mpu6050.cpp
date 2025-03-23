#include "mpu6050.h"

#include <string.h>

#include <iostream>

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "math.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const uint8_t MPU6050_SELF_TEST_REG = 0x0D;
static const uint8_t MPU6050_ADDR = 0x68;
static const uint8_t MPU6050_WHO_AM_I = 0x75;
static const uint8_t MPU6050_ACCEL_REG = 0x3B;
static const uint8_t MPU6050_GYRO_REG = 0x43;
static const uint8_t MPU6050_GYRO_CONFIG = 0x1B;
static const uint8_t MPU6050_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU6050_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU6050_SIGNAL_PATH_RESET = 0x68;
static const uint8_t MPU6050_USER_CTRL = 0x6A;
static const float EARTH_GRAVITY = 9.80665f;

MPU6050::MPU6050() {
}

void MPU6050::init(i2c_master_bus_handle_t &bus_handle) {
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0x68,
        .scl_speed_hz = 400000,
        .scl_wait_us = 1000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    reset();
    get_acceleration_scale_range();
    get_gyro_scale_range();
}

MPU6050_data MPU6050::read() {
    MPU6050_data data;
    uint8_t raw_data[14];
    uint8_t reg_addr = MPU6050_ACCEL_REG;
    i2c_master_transmit_receive(dev_handle, &reg_addr, 1, raw_data, 14, 1000);
    // Don't attempt to memcpy directly because ESP32 is little-endian
    int16_t accel_x = raw_data[0] << 8 | raw_data[1];
    int16_t accel_y = raw_data[2] << 8 | raw_data[3];
    int16_t accel_z = raw_data[4] << 8 | raw_data[5];
    data.accelerometer.x = accel_x / acceleration_scale_factor * EARTH_GRAVITY;
    data.accelerometer.y = accel_y / acceleration_scale_factor * EARTH_GRAVITY;
    data.accelerometer.z = accel_z / acceleration_scale_factor * EARTH_GRAVITY;
    int16_t gyro_x = raw_data[8] << 8 | raw_data[9];
    int16_t gyro_y = raw_data[10] << 8 | raw_data[11];
    int16_t gyro_z = raw_data[12] << 8 | raw_data[13];
    data.gyroscope.x = gyro_x / gyro_scale_factor;
    data.gyroscope.y = gyro_y / gyro_scale_factor;
    data.gyroscope.z = gyro_z / gyro_scale_factor;
    return data;
}

uint8_t MPU6050::get_acceleration_scale_range() {
    acceleration_scale_range = _get_acceleration_scale_range();
    acceleration_scale_factor = 2048.0f * (1 << (3 - acceleration_scale_range));
    return acceleration_scale_range;
}

uint8_t MPU6050::_get_acceleration_scale_range() {
    uint8_t data;
    uint8_t reg_addr = MPU6050_ACCEL_CONFIG;
    i2c_master_transmit_receive(dev_handle, &reg_addr, 1, &data, 1, 1000);
    return (data & 0x18) >> 3;
}

uint8_t MPU6050::get_gyro_scale_range() {
    uint8_t data;
    uint8_t reg_addr = MPU6050_GYRO_CONFIG;
    i2c_master_transmit_receive(dev_handle, &reg_addr, 1, &data, 1, 1000);
    gyro_scale_range = (data & 0x18) >> 3;
    gyro_scale_factor = 16.4f * (1 << (3 - gyro_scale_range));
    return gyro_scale_range;
}

void MPU6050::set_acceleration_scale_range(uint8_t range) {
    uint8_t data[2] = {MPU6050_ACCEL_CONFIG, 0x00};
    i2c_master_transmit_receive(dev_handle, data, 1, data + 1, 1, 1000);
    data[1] = (data[1] & 0xE7) | (range << 3);
    i2c_master_transmit(dev_handle, data, 2, 1000);
    acceleration_scale_range = range;
    acceleration_scale_factor = 2048.0f * (1 << (3 - acceleration_scale_range));
}

void MPU6050::set_gyro_scale_range(uint8_t range) {
    uint8_t data[2] = {MPU6050_GYRO_CONFIG, 0x00};
    i2c_master_transmit_receive(dev_handle, data, 1, data + 1, 1, 1000);
    data[1] = (data[1] & 0xE7) | (range << 3);
    i2c_master_transmit(dev_handle, data, 2, 1000);
    gyro_scale_range = range;
    gyro_scale_factor = 16.4f * (1 << (3 - gyro_scale_range));
}

void MPU6050::raw_read(uint8_t reg_addr, uint8_t *data,
                       uint8_t len) {
    i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, 1000);
}

void MPU6050::raw_write(uint8_t reg_addr, uint8_t &data,
                        uint8_t len) {
    uint8_t buffer[len + 1];
    buffer[0] = reg_addr;
    memcpy(&buffer[1], &data, len);
    i2c_master_transmit(dev_handle, buffer, len + 1, 1000);
}

void MPU6050::reset() {
    uint8_t transmit_data[2] = {MPU6050_PWR_MGMT_1, 0x80};
    i2c_master_transmit(dev_handle, transmit_data, 2, 1000);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    transmit_data[0] = MPU6050_SIGNAL_PATH_RESET;
    transmit_data[1] = 0x07;
    i2c_master_transmit(dev_handle, transmit_data, 2, 1000);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    transmit_data[0] = MPU6050_PWR_MGMT_1;
    transmit_data[1] = 0x09;
    i2c_master_transmit(dev_handle, transmit_data, 2, 1000);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    transmit_data[0] = MPU6050_USER_CTRL;
    transmit_data[1] = 0x07;
    i2c_master_transmit(dev_handle, transmit_data, 2, 1000);
    vTaskDelay(50 / portTICK_PERIOD_MS);
}

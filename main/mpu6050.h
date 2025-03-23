#pragma once

#include <stdint.h>

#include "driver/i2c_master.h"

struct MPU6050_data {
    struct accelerometer {
        float x;
        float y;
        float z;
    } accelerometer;
    struct gyroscope {
        float x;
        float y;
        float z;
    } gyroscope;

    MPU6050_data operator-(const MPU6050_data &other) {
        MPU6050_data result;
        result.accelerometer.x = accelerometer.x - other.accelerometer.x;
        result.accelerometer.y = accelerometer.y - other.accelerometer.y;
        result.accelerometer.z = accelerometer.z - other.accelerometer.z;
        result.gyroscope.x = gyroscope.x - other.gyroscope.x;
        result.gyroscope.y = gyroscope.y - other.gyroscope.y;
        result.gyroscope.z = gyroscope.z - other.gyroscope.z;
        return result;
    }
};

class MPU6050 {
  private:
    const char *TAG = "MPU6050";
    i2c_master_dev_handle_t dev_handle;
    uint8_t acceleration_scale_range;
    float acceleration_scale_factor;
    uint8_t gyro_scale_range;
    float gyro_scale_factor;
    uint8_t _get_acceleration_scale_range();
    uint8_t _get_gyro_scale_range();
    void _set_acceleration_scale_range(uint8_t range);
    void _set_gyro_scale_range(uint8_t range);

  public:
    MPU6050();
    void init();
    void raw_read(uint8_t reg_addr, uint8_t *data, uint8_t len);
    void raw_write(uint8_t reg_addr, uint8_t &data, uint8_t len);
    MPU6050_data read();
    void reset();
    uint8_t get_acceleration_scale_range();
    uint8_t get_gyro_scale_range();
    void set_acceleration_scale_range(uint8_t range);
    void set_gyro_scale_range(uint8_t range);
};

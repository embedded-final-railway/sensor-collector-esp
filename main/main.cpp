#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "mpu6050.h"
#include "utils.h"
#include "wifi_station.h"
#include <string.h>

WifiStation station;
MPU6050 mpu;

extern "C" void app_main(void) {
    print_chip_info();
    esp_log_level_set("MPU6050", ESP_LOG_DEBUG);
    // station.init();
    station.install_uart();
    // station.connect();
    mpu.init();
    mpu.set_acceleration_scale_range(2);
    while (true) {
        MPU6050_data data = mpu.read();
        // printf("Accel: %f %f %f\n", data.accelerometer.x, data.accelerometer.y, data.accelerometer.z);
        // printf("Gyro: %f %f %f\n", data.gyroscope.x, data.gyroscope.y, data.gyroscope.z);
        printf("Current time: %lld\n", esp_timer_get_time());
    }
    // bool self_test_result = mpu.self_test_accelerometer();
    // printf("Self-test result: %s\n", self_test_result ? "PASSED" : "FAILED");
}
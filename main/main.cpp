#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gy_neo6mv2.h"
#include "mpu6050.h"
#include "utils.h"
#include "wifi_station.h"
#include <string.h>

WifiStation station;
MPU6050 mpu;
GY_NEO6MV2 gps;

struct Data {
    MPU6050_data mpu_data;
    GY_NEO6MV2_data gps_data;
};

Data data;

void vReadMPU6050(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    while (true) {
        Data data;
        data.mpu_data = mpu.read();
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void vReadGPS(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    while (true) {
        Data data;
        data.gps_data = gps.read();
        // xQueueSend(data_queue, &data, portMAX_DELAY);
        if (data.gps_data.position.latitude.has_value() && data.gps_data.position.longitude.has_value()) {
            ESP_LOGI("Got GPS data", "Latitude: %f, Longitude: %f",
                     data.gps_data.position.latitude.value(),
                     data.gps_data.position.longitude.value());
        } else {
            ESP_LOGI("Got GPS data", "GPS data not available");
        }
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

uint8_t data_queue_array[3000 * sizeof(Data)];

extern "C" void app_main(void) {
    print_chip_info();

    int uart_buffer_size = 2048;
    uart_driver_install(UART_NUM_0, uart_buffer_size, uart_buffer_size, 10, NULL, 0);
    station.init();
    // station.connect(UART_NUM_0);

    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_config_t i2c_mst_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true,
        }};
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    mpu.init(bus_handle);
    mpu.set_acceleration_scale_range(2);
    uart_config_t gps_uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &gps_uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, GPIO_NUM_18, GPIO_NUM_19, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    uart_driver_install(UART_NUM_1, uart_buffer_size, uart_buffer_size, 10, NULL, 0);
    gps.init(UART_NUM_1);
    ESP_LOGI("Size of Data", "%d", sizeof(Data));
    ESP_LOGW("app_main", "RAM left %lu", esp_get_free_heap_size());
    xTaskCreatePinnedToCore(vReadMPU6050, "ReadMPU6050", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(vReadGPS, "ReadGPS", 4096, NULL, 6, NULL, 1);
}
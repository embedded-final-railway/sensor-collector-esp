#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "mpu6050.h"
#include "soc/gpio_sig_map.h"
#include "utils.h"
#include "wifi_station.h"
#include <string.h>
#include "gy_neo6mv2.h"

WifiStation station;
MPU6050 mpu;
GY_NEO6MV2 gps;

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
    // gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);
    gps.init(UART_NUM_1);
    while (true) {
        char buffer[1000];
        uart_write_bytes(UART_NUM_0, "Enter command: ", sizeof("Enter command: "));
        get_string_from_uart(UART_NUM_0, buffer, sizeof(buffer), true);
        gps.send_command((uint8_t *)buffer, strlen(buffer));
        int read_bytes = uart_read_bytes(UART_NUM_1, (uint8_t *)buffer, sizeof(buffer), 100 / portTICK_PERIOD_MS);
        if (read_bytes > 0) {
            buffer[read_bytes] = '\0';
            if (buffer[0] == '$') {
                ESP_LOGI("GPS", "NMEA: %s", buffer);
            } else if (buffer[0] == 0xB5 && buffer[1] == 0x62) {
                char str[100];
                gps.bytes_array_to_hex_string((uint8_t *)buffer, read_bytes, str);
                ESP_LOGI("GPS", "UBX: %s", str);
            } else {
                ESP_LOGI("GPS", "Unknown: %s", buffer);
            }
        }
    }
}
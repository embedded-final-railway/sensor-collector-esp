#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_sntp.h"
#include "esp_spiffs.h"
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
static portMUX_TYPE gps_spinlock = portMUX_INITIALIZER_UNLOCKED;
esp_vfs_spiffs_conf_t spiffs_conf;

QueueHandle_t data_queue;

struct Data {
    MPU6050_data mpu_data;
    GY_NEO6MV2_data gps_data;
};

Data data;

extern const uint8_t pem_start[] asm("_binary_fullchain_pem_start");
extern const uint8_t pem_end[] asm("_binary_fullchain_pem_end");

void vReadMPU6050(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(2);
    struct timeval tv;

    while (true) {
        char *str = (char *)malloc(100 * 500);
        if (str == NULL) {
            ESP_LOGE("vReadMPU6050", "Failed to allocate memory for string");
            ESP_LOGI("vReadMPU6050", "Free heap size: %u", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        } else {
            ESP_LOGD("vReadMPU6050", "Allocated memory for string, free heap size: %u", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
        }
        uint64_t pos = 0;
        long long int start = esp_timer_get_time();
        for (int i = 0; i < 500; i++) {
            gettimeofday(&tv, NULL);
            data.mpu_data = mpu.read();
            std::string latStr = data.gps_data.position.latitude.has_value() ? std::to_string(data.gps_data.position.latitude.value()) : "";
            std::string lonStr =
                data.gps_data.position.longitude.has_value() ? std::to_string(data.gps_data.position.longitude.value()) : "";
            pos += sprintf(str + pos, "%lld.%ld,%f,%f,%f,%s,%s\n", tv.tv_sec, tv.tv_usec, data.mpu_data.accelerometer.x,
                           data.mpu_data.accelerometer.y, data.mpu_data.accelerometer.z, latStr.c_str(), lonStr.c_str());
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
        xQueueSend(data_queue, &str, portMAX_DELAY);
        long long int end = esp_timer_get_time();
        long long int diff = end - start;
        if (diff > 1001000) {
            ESP_LOGW("vReadMPU6050", "Time taken %lld", diff);
        }
    }
}

void vReadGPS(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);
    while (true) {
        GY_NEO6MV2_data gps_data = gps.read();
        taskENTER_CRITICAL(&gps_spinlock);
        data.gps_data = gps_data;
        taskEXIT_CRITICAL(&gps_spinlock);
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void vUploadFile(void *pvParameter) {
    // const char *const url = "https://linux-vm-southeastasia-2.southeastasia.cloudapp.azure.com/upload";
    const char *const url = "http://192.168.1.102:8080/upload";
    static esp_http_client_config_t config = {
        .url = url,
        .method = HTTP_METHOD_POST,
        // .timeout_ms = 800,
        .event_handler = NULL,
    };
    while (true) {
        char *str = NULL;
        xQueueReceive(data_queue, &str, portMAX_DELAY);
        int len = strlen(str);
        esp_http_client_handle_t client = esp_http_client_init(&config);
        esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
        esp_http_client_open(client, len);
        esp_http_client_write(client, str, len);
        esp_err_t err = esp_http_client_perform(client);
        esp_http_client_cleanup(client);
        free(str);
    }
}

void wait_for_time_sync(void) {
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 10;

    while (timeinfo.tm_year < (2016 - 1900) && ++retry < retry_count) {
        ESP_LOGI("app_main", "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
}

extern "C" void app_main(void) {
    // esp_log_level_set("*", ESP_LOG_DEBUG);
    print_chip_info();

    int uart_buffer_size = 2048;
    uart_driver_install(UART_NUM_0, uart_buffer_size, uart_buffer_size, 10, NULL, 0);
    station.init();
    station.connect(UART_NUM_0);

    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_config_t i2c_mst_config = {.i2c_port = I2C_NUM_0,
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
    data.gps_data = gps.read();
    data_queue = xQueueCreate(1000, sizeof(char *));
    if (data_queue == NULL) {
        ESP_LOGE("app_main", "Failed to create queue");
    }
    // spiffs_conf = {
    //     .base_path = "/spiffs",
    //     .partition_label = NULL,
    //     .max_files = 5,
    //     .format_if_mount_failed = true,
    // };
    // esp_err_t spiffs_ret = esp_vfs_spiffs_register(&spiffs_conf);
    // if (spiffs_ret != ESP_OK) {
    //     if (spiffs_ret == ESP_FAIL) {
    //         ESP_LOGE("app_main", "Failed to mount or format filesystem");
    //     } else if (spiffs_ret == ESP_ERR_NOT_FOUND) {
    //         ESP_LOGE("app_main", "Failed to find SPIFFS partition");
    //     } else {
    //         ESP_LOGE("app_main", "Failed to initialize SPIFFS (%s)", esp_err_to_name(spiffs_ret));
    //     }
    //     return;
    // }
    // esp_spiffs_format(spiffs_conf.partition_label);
    // ESP_LOGI("app_main", "SPIFFS mounted");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_setservername(1, "time.google.com");
    esp_sntp_init();
    wait_for_time_sync();
    ESP_LOGW("app_main", "RAM left %lu", esp_get_free_heap_size());
    static StaticTask_t xTaskBuffer1, xTaskBuffer2, xTaskBuffer3;
    static StackType_t xStack1[4096], xStack2[4096], xStack3[4096];

    xTaskCreateStaticPinnedToCore(vReadMPU6050, "ReadMPU6050", 4096, NULL, 5, xStack1, &xTaskBuffer1, 1);
    xTaskCreateStaticPinnedToCore(vReadGPS, "ReadGPS", 4096, NULL, 4, xStack2, &xTaskBuffer2, 1);
    xTaskCreateStaticPinnedToCore(vUploadFile, "UploadFile", 4096, NULL, 5, xStack3, &xTaskBuffer3, 0);
}
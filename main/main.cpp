#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "esp_http_client.h"
#include "esp_log.h"
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

QueueHandle_t filename_data_queue;

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
    while (true) {
        long long int start = esp_timer_get_time();
        char *filepath = (char *)malloc(32 * sizeof(char));
        sprintf(filepath, "/spiffs/%lld.csv", esp_timer_get_time() / 100);
        FILE *file = fopen(filepath, "w");
        if (file == NULL) {
            ESP_LOGE("vReadMPU6050", "Failed to open file");
            return;
        }
        for (int i = 0; i < 500; i++) {
            data.mpu_data = mpu.read();
            std::string latStr = data.gps_data.position.latitude.has_value() ? std::to_string(data.gps_data.position.latitude.value()) : "";
            std::string lonStr =
                data.gps_data.position.longitude.has_value() ? std::to_string(data.gps_data.position.longitude.value()) : "";
            fprintf(file, "%lld,%f,%f,%f,%s,%s\n", esp_timer_get_time(), data.mpu_data.accelerometer.x, data.mpu_data.accelerometer.y,
                    data.mpu_data.accelerometer.z, latStr.c_str(), lonStr.c_str());
            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }
        fclose(file);
        ESP_LOGD("vReadMPU6050", "Finished writing to file %s", filepath);
        xQueueSend(filename_data_queue, &filepath, portMAX_DELAY);
        long long int end = esp_timer_get_time();
        ESP_LOGI("Time", "%lld", end - start);
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
    while (true) {
        char *filepath;
        xQueueReceive(filename_data_queue, &filepath, portMAX_DELAY);
        if (filepath == NULL) {
            ESP_LOGE("vUploadFile", "Failed to receive filename");
            continue;
        } else {
            ESP_LOGD("vUploadFile", "Received filename %s", filepath);
        }
        FILE *file = fopen(filepath, "r");
        fseek(file, 0, SEEK_END);
        // Get the file size
        long file_size = ftell(file);
        fseek(file, 0, SEEK_SET);
        const char *const url = "http://192.168.1.102:8080/upload";
        static esp_http_client_config_t config = {
            .url = url,
            .method = HTTP_METHOD_POST,
            .event_handler = NULL,
        };
        esp_http_client_handle_t client = esp_http_client_init(&config);
        esp_http_client_set_header(client, "Content-Type", "application/octet-stream");
        esp_http_client_open(client, file_size);
        char buffer[1024];
        size_t read_len;
        while ((read_len = fread(buffer, 1, sizeof(buffer), file)) > 0) {
            esp_http_client_write(client, buffer, read_len);
        }
        esp_err_t err = esp_http_client_perform(client);
        if (err == ESP_OK) {
            ESP_LOGD("vUploadFile", "Uploaded file %s", filepath);
        } else {
            ESP_LOGE("vUploadFile", "Failed to upload file %s", filepath);
        }
        fclose(file);
        if (remove(filepath) != 0) {
            ESP_LOGE("vUploadFile", "Failed to delete file %s", filepath);
        } else {
            ESP_LOGD("vUploadFile", "Deleted file %s", filepath);
        }
        esp_http_client_cleanup(client);
        free(filepath);
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
    filename_data_queue = xQueueCreate(10, sizeof(char *));
    if (filename_data_queue == NULL) {
        ESP_LOGE("app_main", "Failed to create queue");
    }
    spiffs_conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true,
    };
    esp_err_t spiffs_ret = esp_vfs_spiffs_register(&spiffs_conf);
    if (spiffs_ret != ESP_OK) {
        if (spiffs_ret == ESP_FAIL) {
            ESP_LOGE("app_main", "Failed to mount or format filesystem");
        } else if (spiffs_ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE("app_main", "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE("app_main", "Failed to initialize SPIFFS (%s)", esp_err_to_name(spiffs_ret));
        }
        return;
    }
    esp_spiffs_format(spiffs_conf.partition_label);
    ESP_LOGI("app_main", "SPIFFS mounted");
    ESP_LOGW("app_main", "RAM left %lu", esp_get_free_heap_size());
    xTaskCreatePinnedToCore(vReadMPU6050, "ReadMPU6050", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(vReadGPS, "ReadGPS", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(vUploadFile, "UploadFile", 4096, NULL, 5, NULL, 0);
}
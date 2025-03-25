#include "wifi_station.h"

#include <esp_log.h>
#include <nvs_flash.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "utils.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

WifiStation::WifiStation() {
}

void WifiStation::init() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to create event loop: %d", ret);
        return;
    }
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, this, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, this, &instance_got_ip));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
}

esp_err_t WifiStation::connect(const char *ssid, const char *password) {
    wifi_config_t wifi_config;
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK;
    memcpy((char *)wifi_config.sta.ssid, ssid, strlen(ssid));
    memcpy((char *)wifi_config.sta.password, password, strlen(password));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Connecting to AP");

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to %s", ssid);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to %s", ssid);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    return ESP_OK;
}

esp_err_t WifiStation::connect(uart_port_t uart_num) {
    wifi_config_t wifi_cfg;
    esp_wifi_get_config(WIFI_IF_STA, &wifi_cfg);
    bool reconfigure = false;
    if (strlen((char *)wifi_cfg.sta.ssid) != 0) {
        uart_write_bytes(uart_num, "Press any key in 3 seconds to reconfigure wifi...",
                         strlen("Press any key in 3 seconds to reconfigure wifi..."));
        char c;
        if (uart_read_bytes(uart_num, &c, 1, 3000 / portTICK_PERIOD_MS) > 0) {
            reconfigure = true;
        }
    } else {
        reconfigure = true;
    }
    uart_write_bytes(uart_num, "\n", 1);
    if (reconfigure || strlen((char *)wifi_cfg.sta.ssid) == 0) {
        // printf("Enter SSID: ");
        uart_write_bytes(uart_num, "Enter SSID: ", strlen("Enter SSID: "));
        get_string_from_uart(uart_num, (char *)wifi_cfg.sta.ssid, 32, true);
        uart_write_bytes(uart_num, "Enter Password: ", strlen("Enter Password: "));
        get_string_from_uart(uart_num, (char *)wifi_cfg.sta.password, 64, false);
        ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    } else {
        uart_write_bytes(uart_num, "\n", 1);
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Connecting to %s", (char *)wifi_cfg.sta.ssid);

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to ap SSID: %s", (char *)wifi_cfg.sta.ssid);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to SSID: %s", (char *)wifi_cfg.sta.ssid);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
    return ESP_OK;
}

void WifiStation::event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    WifiStation *self = (WifiStation *)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (self->retry_count < MAX_RETRY) {
            esp_wifi_connect();
            self->retry_count++;
            ESP_LOGI(self->TAG, "Retrying connection to AP");
        } else {
            xEventGroupSetBits(self->wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGE(self->TAG, "Connection to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(self->TAG, "Got ip: " IPSTR, IP2STR(&event->ip_info.ip));
        self->retry_count = 0;
        xEventGroupSetBits(self->wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

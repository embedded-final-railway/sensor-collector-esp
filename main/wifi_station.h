#pragma once

#include "driver/uart.h"
#include "esp_err.h"
#include "esp_event.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

class WifiStation {
  private:
    const char *TAG = "WifiStation";
    EventGroupHandle_t wifi_event_group;
    static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

    uint8_t retry_count = 0;
    static const uint8_t MAX_RETRY = 5;

  public:
    WifiStation();
    void init();
    esp_err_t connect(uart_port_t uart_num);
    esp_err_t connect(const char *ssid, const char *password);
};
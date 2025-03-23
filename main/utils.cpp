#include "utils.h"

#include <inttypes.h>
#include <sdkconfig.h>
#include <stdio.h>

#include "driver/uart.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void print_chip_info() {
  esp_chip_info_t chip_info;
  uint32_t flash_size;
  esp_chip_info(&chip_info);
  printf("%s chip, %d CPU core(s), %s%s%s%s, ", CONFIG_IDF_TARGET,
         chip_info.cores,
         (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
         (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
         (chip_info.features & CHIP_FEATURE_IEEE802154)
             ? ", 802.15.4 (Zigbee/Thread)"
             : "");

  unsigned major_rev = chip_info.revision / 100;
  unsigned minor_rev = chip_info.revision % 100;
  printf("silicon revision v%d.%d, ", major_rev, minor_rev);
  if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
    printf("Get flash size failed");
    return;
  }

  printf(
      "%" PRIu32 "MB %s flash\n", flash_size / (uint32_t)(1024 * 1024),
      (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

  printf("Minimum free heap size: %" PRIu32 " bytes\n",
         esp_get_minimum_free_heap_size());
}

void get_string_from_uart(uart_port_t uart_num, char *buf,
                                     size_t len, bool echo) {
  int index = 0;
  buf[0] = '\0';  // Initialize the buffer

  while (true) {
    // Read data from UART
    uint8_t data;
    int read_bytes = uart_read_bytes(
        uart_num, &data, 1, 100 / portTICK_PERIOD_MS);  // 100 ms timeout

    if (read_bytes > 0) {
      if (data == '\n' || data == '\r') {   // Enter key pressed
        buf[index] = '\0';  // Null terminate the string
        uart_write_bytes(uart_num, "\n", 1);
        break;
      } else if (data == '\b' || data == 127) {  // Backspace key
        if (index > 0) {
          index--;            // Move pointer back
          buf[index] = '\0';  // Null terminate the string
          // Optionally echo back to the console for visual feedback
          if (echo) uart_write_bytes(uart_num, "\b \b", 3);
        }
      } else {
        if (index < len - 1) {  // Prevent buffer overflow
          buf[index++] = data;  // Append new char to buffer
          buf[index] = '\0';    // Null terminate the string
          // Optionally echo back character to the console
          if (echo) uart_write_bytes(uart_num, (const char *)&data, 1);
        }
      }
    }
  }
}

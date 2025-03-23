#pragma once
#include "driver/uart.h"
#define LOG_LOCAL_LEVEL ESP_LOG_INFO

struct GY_NEO6MV2_data {
    struct position {
        float latitude;
        float longitude;
    } position;
    struct time {
        uint8_t hours;
        uint8_t minutes;
        uint8_t seconds;
    } time;
};

class GY_NEO6MV2 {
  private:
    const char *TAG = "GY_NEO6MV2";
    uart_port_t uart_num;
    int hex_string_to_bytes(const char *hex_string, uint8_t *bytes);
    public:
    GY_NEO6MV2();
    int bytes_array_to_hex_string(uint8_t *bytes, int len, char *hex_string);
    void init(uart_port_t uart_num);
    void read(char *buf, size_t len);
    void send_command(uint8_t *cmd, size_t len);
};

namespace NMEA {
    int add_checksum(uint8_t *data, int len);
    bool verify_checksum(uint8_t *data, int len);
    uint8_t calculate_checksum(uint8_t *data, int len);
}

namespace UBX {
    int add_checksum(uint8_t *data, int len);
    bool verify_checksum(uint8_t *data, int len);
    uint16_t calculate_checksum(uint8_t *data, int len);
}
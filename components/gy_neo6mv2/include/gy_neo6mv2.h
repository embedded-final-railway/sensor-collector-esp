#pragma once
#include "driver/uart.h"
#include <optional>
#include <string>
#include <vector>

struct GY_NEO6MV2_data {
    struct position {
        std::optional<double> latitude;
        std::optional<double> longitude;
    } position;
    struct time {
        std::optional<uint8_t> hours;
        std::optional<uint8_t> minutes;
        std::optional<uint8_t> seconds;
    } time;
};

class GY_NEO6MV2 {
  private:
    const char *TAG = "GY_NEO6MV2";
    uart_port_t uart_num;
    int hex_string_to_bytes(const char *hex_string, uint8_t *bytes);
    double parseCoordinate(const char *coordStr);
    uint8_t obtain_payload(uint8_t *buffer, int len);
    GY_NEO6MV2_data parse_GPGLL(const char *buffer);
    static std::vector<std::string> split(const std::string &s, char delimiter);

  public:
    GY_NEO6MV2();
    int bytes_array_to_hex_string(uint8_t *bytes, int len, char *hex_string);
    void init(uart_port_t uart_num);
    GY_NEO6MV2_data read();
    void send_command(uint8_t *cmd, size_t len);
};

namespace NMEA {
int add_checksum(uint8_t *data, int len);
bool verify_checksum(uint8_t *data, int len);
uint8_t calculate_checksum(uint8_t *data, int len);
} // namespace NMEA

namespace UBX {
int add_checksum(uint8_t *data, int len);
bool verify_checksum(uint8_t *data, int len);
uint16_t calculate_checksum(uint8_t *data, int len);
} // namespace UBX
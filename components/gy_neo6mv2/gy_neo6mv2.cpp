#include "gy_neo6mv2.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <charconv>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <stdio.h>
#include <string>
#include <optional>
#include <string>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <cstring>

GY_NEO6MV2::GY_NEO6MV2() {
}

void GY_NEO6MV2::init(uart_port_t uart_num) {
    this->uart_num = uart_num;
    const char *commands[] = {
        "$PUBX,40,GLL,0,1,0,0,0,0",
        "$PUBX,40,GSV,0,0,0,0,0,0",
        "$PUBX,40,GSA,0,0,0,0,0,0",
        "$PUBX,40,GGA,0,0,0,0,0,0",
        "$PUBX,40,VTG,0,0,0,0,0,0",
        "$PUBX,40,RMC,0,0,0,0,0,0"};

    for (auto &cmd : commands) {
        send_command((uint8_t *)cmd, strlen(cmd));
    }

    uart_flush(this->uart_num);
}

void GY_NEO6MV2::send_command(uint8_t *cmd, size_t len) {
    uint8_t payload[100];
    memcpy(payload, cmd, len);
    if (cmd[0] == '$' && len > 1) {
        // NMEA protocol
        len = NMEA::add_checksum(payload, len);
        if (esp_log_level_get(TAG) >= ESP_LOG_INFO) {
            char hex_string[100];
            memcpy(hex_string, payload, len);
            hex_string[len - 2] = '\0';
            ESP_LOGI(TAG, "Sending NMEA command: %s", hex_string);
        }
        uart_write_bytes(this->uart_num, (const char *)payload, len);
    } else if (strncasecmp((const char *)cmd, "B562", 4) == 0 && len > 6) {
        // UBX protocol
        memset(&payload[len], 0, 2);
        len = hex_string_to_bytes((const char *)cmd, payload);
        len = UBX::add_checksum(payload, len);
        if (esp_log_level_get(TAG) >= ESP_LOG_INFO) {
            char hex_string[100];
            bytes_array_to_hex_string(payload, len, hex_string);
            ESP_LOGI(TAG, "Sending UBX command: %s", hex_string);
        }
        uart_write_bytes(this->uart_num, (const char *)payload, len);
    } else {
        ESP_LOGE(TAG, "Invalid command");
    }
}

int GY_NEO6MV2::hex_string_to_bytes(const char *hex_string, uint8_t *bytes) {
    int len = strlen(hex_string);
    if (len % 2 != 0) return -1; // Invalid hex string

    for (int i = 0; i < len; i += 2) {
        char hex_byte[3] = {hex_string[i], hex_string[i + 1], '\0'};
        bytes[i / 2] = strtol(hex_byte, NULL, 16);
    }
    return len / 2;
}

int GY_NEO6MV2::bytes_array_to_hex_string(uint8_t *bytes, int len, char *hex_string) {
    for (int i = 0; i < len; i++) {
        sprintf(&hex_string[i * 2], "%02X", bytes[i]);
    }
    return len * 2;
}

uint8_t GY_NEO6MV2::obtain_payload(uint8_t *buffer, int len) {
    uint8_t pos = 0;
    while (true) {
        uart_read_bytes(this->uart_num, buffer + pos, 1, portMAX_DELAY);
        if (buffer[pos] == '$') {
            pos++;
            break;
        } else if (pos == 0 && buffer[pos] == 0xB5) {
            pos++;
            continue;
        } else if (pos == 1 && buffer[pos] == 0x62) {
            pos++;
            break;
        }
        pos = 0;
    }
    if (buffer[0] == '$') {
        while (true) {
            uart_read_bytes(this->uart_num, buffer + pos, 1, portMAX_DELAY);
            if (buffer[pos++] == '\n') {
                buffer[pos] = '\0';
                break;
            }
        }
    } else {
        uart_read_bytes(this->uart_num, buffer + pos, 4, portMAX_DELAY);
        pos += 4;
        uint16_t payload_len = buffer[5] << 8 | buffer[4];
        uart_read_bytes(this->uart_num, buffer + pos, payload_len + 2, portMAX_DELAY);
        pos += payload_len + 2;
    }
    return pos;
}

double parseCoordinate(const std::string &raw, char direction) {
    double value = 0.0;
    auto dotPos = raw.find('.');
    if (dotPos != std::string::npos) {
        int degrees = std::stoi(raw.substr(0, dotPos - 2));
        double minutes = std::stod(raw.substr(dotPos - 2));
        value = degrees + minutes / 60.0;
    }
    return (direction == 'S' || direction == 'W') ? -value : value;
}

GY_NEO6MV2_data GY_NEO6MV2::read() {
    uint8_t buffer[100];
    GY_NEO6MV2_data data;
    while (true) {
        obtain_payload(buffer, sizeof(buffer));
        ESP_LOGI(TAG, "Received: %s", buffer);
        if (strncmp((const char *)buffer, "$GPGLL", 6) == 0) {
            data = parse_GPGLL((const char *)buffer);
            break;
        }
    }
    return data;
}

GY_NEO6MV2_data GY_NEO6MV2::parse_GPGLL(const char *buffer) {
    GY_NEO6MV2_data data;

    // The expected NMEA sentence format is:
    // $GPGLL,<lat>,<N/S>,<lon>,<E/W>,<hhmmss.ss>,<status>,<mode>*<checksum>
    //
    // Example: "$GPGLL,4717.11634,N,00833.91297,E,124923.00,A,A*6E"
    //
    // We'll parse the latitude string, latitude direction,
    // longitude string, longitude direction, and the time string.

    char lat_str[16]     = {0};
    char lon_str[16]     = {0};
    char time_str[16]    = {0};
    char lat_dir        = 0;
    char lon_dir        = 0;
    
    // Use sscanf to extract the required fields:
    // We discard other fields since they are not needed.
    // The format string reads the sentence parts by splitting on commas.
    sscanf(buffer, "$GPGLL,%15[^,],%c,%15[^,],%c,%15[^,]", 
           lat_str, &lat_dir, lon_str, &lon_dir, time_str);

    // Convert latitude:
    // NMEA format for latitude is ddmm.mmmm. So first two digits are degrees,
    // and the remaining are minutes.
    double lat_val = static_cast<double>(atof(lat_str));
    int lat_degrees = static_cast<int>(lat_val / 100);
    double lat_minutes = lat_val - (lat_degrees * 100);
    data.position.latitude = lat_degrees + (lat_minutes / 60.0f);
    if(lat_dir == 'S') {
        data.position.latitude = -data.position.latitude;
    }

    // Convert longitude:
    // NMEA format for longitude is dddmm.mmmm. So first three digits are degrees.
    double lon_val = static_cast<double>(atof(lon_str));
    int lon_degrees = static_cast<int>(lon_val / 100);
    double lon_minutes = lon_val - (lon_degrees * 100);
    data.position.longitude = lon_degrees + (lon_minutes / 60.0f);
    if(lon_dir == 'W') {
        data.position.longitude = -data.position.longitude;
    }

    // Extract time fields from hhmmss.ss (we ignore the fractional part)
    char hours_str[3]   = {0};
    char minutes_str[3] = {0};
    char seconds_str[3] = {0};

    // Ensure that the time string length is at least 6 characters.
    if(strlen(time_str) >= 6) {
        strncpy(hours_str, time_str, 2);
        strncpy(minutes_str, time_str + 2, 2);
        strncpy(seconds_str, time_str + 4, 2);
    }
    data.time.hours   = static_cast<uint8_t>(atoi(hours_str));
    data.time.minutes = static_cast<uint8_t>(atoi(minutes_str));
    data.time.seconds = static_cast<uint8_t>(atoi(seconds_str));

    return data;
}

namespace NMEA {
int add_checksum(uint8_t *data, int len) {
    data[len] = '*';
    snprintf((char *)&data[len + 1], 3, "%02X", calculate_checksum(data, len));
    data[len + 3] = '\r';
    data[len + 4] = '\n';
    return len + 5;
}

bool verify_checksum(uint8_t *data, int len) {
    if (len < 7) return -1; // Invalid NMEA message
    uint8_t checksum = calculate_checksum(data, len - 5);
    uint8_t actual_checksum = strtol((const char *)&data[len - 4], NULL, 16);
    return checksum == actual_checksum;
}

uint8_t calculate_checksum(uint8_t *data, int len) {
    uint8_t checksum = 0;
    for (int i = 1; i < len; i++) {
        checksum ^= data[i];
    }
    return checksum;
}
} // namespace NMEA

namespace UBX {
int add_checksum(uint8_t *data, int len) {
    uint16_t checksum = calculate_checksum(data, len);
    data[len] = checksum >> 8;
    data[len + 1] = checksum & 0xFF;
    return len + 2;
}

bool verify_checksum(uint8_t *data, int len) {
    if (len < 8) return -1; // Invalid UBX message
    uint8_t checksum = calculate_checksum(data, len - 2);
    uint8_t actual_checksum = data[len - 2] << 8 | data[len - 1];
    return checksum == actual_checksum;
}

uint16_t calculate_checksum(uint8_t *data, int len) {
    uint8_t a = 0, b = 0;
    for (int i = 2; i < len; i++) {
        a += data[i];
        b += a;
    }
    return (uint16_t)a << 8 | b;
}
} // namespace UBX

std::vector<std::string> GY_NEO6MV2::split(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::istringstream tokenStream(s);
    std::string token;
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}
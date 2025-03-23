#include <stdio.h>
#include "gy_neo6mv2.h"
#include "string.h"
#include "esp_log.h"
#include "driver/uart.h"

GY_NEO6MV2::GY_NEO6MV2() {
}

void GY_NEO6MV2::init(uart_port_t uart_num) {
    this->uart_num = uart_num;
    const char* commands[] = {
        "$PUBX,40,GLL,0,0,0,0,0,0",
        "$PUBX,40,GSV,0,0,0,0,0,0",
        "$PUBX,40,GSA,0,0,0,0,0,0",
        "$PUBX,40,GGA,0,0,0,0,0,0",
        "$PUBX,40,VTG,0,0,0,0,0,0",
        "$PUBX,40,RMC,0,0,0,0,0,0"
    };

    for (auto &cmd : commands) {
        send_command((uint8_t*)cmd, strlen(cmd));
    }

    // uart_flush(this->uart_num);
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
}

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
}
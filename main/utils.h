#pragma once

#include "driver/uart.h"

void print_chip_info();
void get_string_from_uart(uart_port_t uart_num, char *buf, size_t len, bool echo);
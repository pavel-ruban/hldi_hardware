//
// Created by root on 23/09/16.
//
#pragma once

#include <stdint.h>
#include <stm32f10x_conf.h>
#include <queue.h>

#define RECV_STRING_MAX_SIZE 200

#define UART1 1
#define UART2 2
#define UART3 3

class Uart {
public:
    Queue<uint8_t, 200> cyclo_buffer;
    Uart(uint8_t uart_number, uint32_t speed);
    ~Uart();
    uint8_t current_uart_number;
    void send_byte(uint8_t data);
    void send(char *string);
    uint8_t last_byte;
    char last_string[RECV_STRING_MAX_SIZE];
    uint8_t last_string_empty;
    uint16_t last_char;
    uint8_t last_string_ready;
    uint8_t last_string_parsed;
private:
    void init_uart(uint32_t speed, uint8_t uart = UART1);
};

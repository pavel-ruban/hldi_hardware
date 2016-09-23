//
// Created by root on 23/09/16.
//
#pragma once

#include <cstdint>
#include <stm32f10x_conf.h>

#define UART1 1
#define UART2 2
#define UART3 3

class Uart {
public:
    Uart(uint8_t uart_number, uint32_t speed);
    ~Uart();
    uint8_t current_uart_number;
    void send_byte(uint8_t data);
    void send(char *string);
    uint8_t last_byte;
private:
    void init_uart_1(uint32_t speed);
    void init_uart_2(uint32_t speed);
    void init_uart_3(uint32_t speed);
};

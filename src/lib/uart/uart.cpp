//
// Created by root on 23/09/16.
//

#include "uart.h"

void Uart::init_uart(uint32_t speed, uint8_t uart)
{
    GPIO_InitTypeDef Tx_init;

    uint32_t uart_rcc_apb2 = 0;
    uint32_t uart_rcc_apb2_gpio = 0;
    uint16_t uart_pin_rx = 0;
    uint16_t uart_pin_tx = 0;
    USART_TypeDef *usartx = 0;
    GPIO_TypeDef *uart_pins_port = 0;

    switch (uart)
    {
        case UART1:
            uart_rcc_apb2 = RCC_APB2Periph_USART1;
            uart_rcc_apb2_gpio = RCC_APB2Periph_GPIOA;
            uart_pin_rx = GPIO_Pin_10;
            uart_pin_tx = GPIO_Pin_9;
            uart_pins_port = GPIOA;
            usartx = USART1;
            break;

        case UART2:
            uart_rcc_apb2 = RCC_APB1Periph_USART2;
            uart_rcc_apb2_gpio = RCC_APB2Periph_GPIOA;
            uart_pin_rx = GPIO_Pin_3;
            uart_pin_tx = GPIO_Pin_2;
            uart_pins_port = GPIOA;
            usartx = USART2;
            break;


        case UART3:
            uart_rcc_apb2 = RCC_APB1Periph_USART3;
            uart_rcc_apb2_gpio = RCC_APB2Periph_GPIOB;
            uart_pin_rx = GPIO_Pin_11;
            uart_pin_tx = GPIO_Pin_10;
            uart_pins_port = GPIOB;
            usartx = USART3;
            break;
    }

    // Включаем тактирование порта А и USART1
    RCC_APB2PeriphClockCmd(uart_rcc_apb2_gpio | uart_rcc_apb2, ENABLE);

    // Настраиваем ногу TxD (PA9) как выход push-pull c альтернативной функцией
    Tx_init.GPIO_Pin = uart_pin_tx;
    Tx_init.GPIO_Speed = GPIO_Speed_50MHz;
    Tx_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(uart_pins_port, &Tx_init);

    GPIO_InitTypeDef Rx_init;
    Rx_init.GPIO_Pin = uart_pin_rx;
    Rx_init.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(uart_pins_port, &Rx_init);

    USART_InitTypeDef uart_struct;

    uart_struct.USART_BaudRate = speed;
    uart_struct.USART_WordLength = USART_WordLength_8b;
    uart_struct.USART_StopBits = USART_StopBits_1;
    uart_struct.USART_Parity = USART_Parity_No ;
    uart_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    //Инициализируем UART
    USART_Init(usartx, &uart_struct);

    //Включаем UART
    USART_Cmd(usartx, ENABLE);
}

Uart::Uart(uint8_t uart_number, uint32_t speed)
{
    for (uint16_t i = 0; i < RECV_STRING_MAX_SIZE; ++i) {
        last_string[i] = 0;
    }

    last_string_empty = 1;
    last_char = 0;
    last_string_ready = 0;
    last_string_parsed = 0;

    init_uart(speed, uart_number);
    current_uart_number = uart_number;
}

Uart::~Uart() {}

void Uart::send_byte(uint8_t data)
{
    switch (current_uart_number)
    {
        case UART1:
            while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
            USART_SendData(USART1, data);
            break;
        case UART2:
            while(!(USART1->SR & USART_SR_TC));
            USART2->DR = data;
            break;
        case UART3:
            while(!(USART1->SR & USART_SR_TC));
            USART3->DR = data;
            break;
    }
}

void Uart::send(char *string)
{
    for (uint16_t i = 0; string[i] != '\0'; i++) {
        this->send_byte(string[i]);
    }
}
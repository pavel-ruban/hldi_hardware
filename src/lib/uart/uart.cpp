//
// Created by root on 23/09/16.
//

#include "uart.h"

void Uart::init_uart_1(uint32_t speed) {
    GPIO_InitTypeDef Tx_init;
    // Включаем тактирование порта А и USART1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);
    // Настраиваем ногу TxD (PA9) как выход push-pull c альтернативной функцией
    Tx_init.GPIO_Pin = GPIO_Pin_9;
    Tx_init.GPIO_Speed = GPIO_Speed_50MHz;
    Tx_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &Tx_init);

    GPIO_InitTypeDef Rx_init;
    Rx_init.GPIO_Pin   = GPIO_Pin_10;
    Rx_init.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &Rx_init);

    USART_InitTypeDef uart_struct;
    uart_struct.USART_BaudRate            = speed;
    uart_struct.USART_WordLength          = USART_WordLength_8b;
    uart_struct.USART_StopBits            = USART_StopBits_1;
    uart_struct.USART_Parity              = USART_Parity_No ;
    uart_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart_struct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    //Инициализируем UART
    USART_Init(USART1, &uart_struct);
    //Включаем UART
    USART_Cmd(USART1, ENABLE);
}

void Uart::init_uart_2(uint32_t speed) {
    GPIO_InitTypeDef Tx_init;
    // Включаем тактирование порта А и USART2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB1Periph_USART2, ENABLE);
    // Настраиваем ногу TxD (PA3) как выход push-pull c альтернативной функцией
    Tx_init.GPIO_Pin = GPIO_Pin_2;
    Tx_init.GPIO_Speed = GPIO_Speed_50MHz;
    Tx_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &Tx_init);

    GPIO_InitTypeDef Rx_init;
    Rx_init.GPIO_Pin   = GPIO_Pin_3;
    Rx_init.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &Rx_init);

    USART_InitTypeDef uart_struct;
    uart_struct.USART_BaudRate            = speed;
    uart_struct.USART_WordLength          = USART_WordLength_8b;
    uart_struct.USART_StopBits            = USART_StopBits_1;
    uart_struct.USART_Parity              = USART_Parity_No ;
    uart_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart_struct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    //Инициализируем UART
    USART_Init(USART2, &uart_struct);
    //Включаем UART
    USART_Cmd(USART2, ENABLE);
}

void Uart::init_uart_3(uint32_t speed) {
    GPIO_InitTypeDef Tx_init;
    // Включаем тактирование порта А и USART2
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB1Periph_USART3, ENABLE);
    // Настраиваем ногу TxD (PA3) как выход push-pull c альтернативной функцией
    Tx_init.GPIO_Pin = GPIO_Pin_10;
    Tx_init.GPIO_Speed = GPIO_Speed_50MHz;
    Tx_init.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOB, &Tx_init);

    GPIO_InitTypeDef Rx_init;
    Rx_init.GPIO_Pin   = GPIO_Pin_11;
    Rx_init.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &Rx_init);

    USART_InitTypeDef uart_struct;
    uart_struct.USART_BaudRate            = speed;
    uart_struct.USART_WordLength          = USART_WordLength_8b;
    uart_struct.USART_StopBits            = USART_StopBits_1;
    uart_struct.USART_Parity              = USART_Parity_No ;
    uart_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    uart_struct.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    //Инициализируем UART
    USART_Init(USART3, &uart_struct);
    //Включаем UART
    USART_Cmd(USART3, ENABLE);
}

Uart::Uart(uint8_t uart_number, uint32_t speed) {
    switch (uart_number){
        case UART1:
            init_uart_1(speed);
            current_uart_number = UART1;
            break;
        case UART2:
            init_uart_2(speed);
            current_uart_number = UART2;
            break;
        case UART3:
            init_uart_3(speed);
            current_uart_number = UART3;
            break;
    }
}

Uart::~Uart() {

}

void Uart::send_byte(uint8_t data) {
    switch (current_uart_number){
        case UART1:
            while(!(USART1->SR & USART_SR_TC));
            USART1->DR=data;
            break;
        case UART2:
            while(!(USART1->SR & USART_SR_TC));
            USART2->DR=data;
            break;
        case UART3:
            while(!(USART1->SR & USART_SR_TC));
            USART3->DR=data;
            break;
    }
}

void Uart::send(char *string) {
    for (uint16_t i = 0; string[i] != '\0'; i++) {
        this->send_byte(string[i]);
    }
}
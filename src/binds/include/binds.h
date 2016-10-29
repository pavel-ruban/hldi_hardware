#pragma once
#pragma pack(1)

#include <stdint.h>

#include "spi_binds.h"
#include "rc522_binds.h"
#include "ethernet_binds.h"
#include <stm32f10x_conf.h>

#define LED_INDICATOR_PORT GPIOA
#define LED_INDICATOR_PIN_RED GPIO_Pin_1
#define LED_INDICATOR_PIN_GREEN GPIO_Pin_2
#define LED_INDICATOR_PIN_BLUE GPIO_Pin_3
#define BTN_CALL_PORT GPIOB
#define BTN_CALL_PIN GPIO_Pin_1
#define BTN_OPEN_PORT GPIOB
#define BTN_OPEN_PIN GPIO_Pin_0
#define EM_LOCK_PORT GPIOB
#define EM_LOCK_PIN GPIO_Pin_3

extern volatile uint32_t ticks;

void *memcpy(void *s1, const void *s2, size_t n);
void *memset(void *dst, int val, size_t count);
size_t strlen(const char* str);
int strncmp(const char *str1, const char *str2, size_t len);
//void strcpy(char* str1, char* str2);

void __enable_enc28j60_irq();
void __disable_enc28j60_irq();

void Delay(vu32 nCount);

//
// Created by root on 13/09/16.
//
#pragma once
#define PINS_OFF(PORT, PINS)                                \
GPIO_InitStructure.GPIO_Pin = PINS;                         \
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;           \
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;               \
GPIO_Init(PORT, &GPIO_InitStructure);                       \
GPIO_ResetBits(PORT, PINS)

#define PINS_ON(PORT, PINS)                                 \
GPIO_InitStructure.GPIO_Pin = PINS;                         \
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;           \
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;             \
GPIO_Init(PORT, &GPIO_InitStructure)

#define LED_BLINK_ON 1
#define LED BLINK_OFF 0

/*******************************************************************************
 * Function Name  : Delay
 * Description    : Inserts a delay time.
 * Input          : nCount: specifies the delay time length.
 * Output         : None
 * Return         : None
 *******************************************************************************/
void Delay(uint32_t nCount)
{
    for(; nCount != 0; nCount--);
}


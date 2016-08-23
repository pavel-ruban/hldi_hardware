#include <stdint.h>
#include <stm32f10x_conf.h>
#include <assert.h>
#include "led.hpp"

//using namespace std;

led::led(uint8_t led_type, GPIO_TypeDef *led_port, uint16_t led_pin)
{

	led::type = led_type;
	color = 0xFFFFFF;

	/* Enable GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure GPIO
	GPIO_InitTypeDef led_init;
	GPIO_InitStructure.GPIO_Pin = led_pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(led_port, &led_init);

	// Enable TIM clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	// Configure TIM2 to PWM
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_Prescaler = TIM_PRESCALER;
	TIM_TimeBaseInitStructure.TIM_Period = TIM_PERIOD;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

	// Configure TIM output channel
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = (color >> (8 * type)) & 0xFF;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	switch(led_pin)
	{
		case GPIO_Pin_1:
			TIM_OC2Init(TIM2, &TIM_OCInitStructure);
			break;
		case GPIO_Pin_2:
			TIM_OC3Init(TIM2, &TIM_OCInitStructure);
			break;
		case GPIO_Pin_3:
			TIM_OC4Init(TIM2, &TIM_OCInitStructure);
			break;
	}

	TIM_Cmd(TIM2, ENABLE);
}

led::led(uint8_t led_type, GPIO_TypeDef *led_port, uint16_t led_pin_r, uint16_t led_pin_g, uint16_t led_pin_b)
{

	led::type = led_type;
	led::color = 0xFFFFFF;

	/* Enable GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;

	// Configure GPIO
	GPIO_InitTypeDef led_init;
	GPIO_InitStructure.GPIO_Pin = led_pin_r;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(led_port, &led_init);

	GPIO_InitStructure.GPIO_Pin = led_pin_g;
	GPIO_Init(led_port, &led_init);

	GPIO_InitStructure.GPIO_Pin = led_pin_b;
	GPIO_Init(led_port, &led_init);

	// Enable TIM clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	// Configure TIM2 to PWM
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_Prescaler = 720;
	TIM_TimeBaseInitStructure.TIM_Period = 100;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

	// Configure TIM output channel for red
	TIM_OCInitTypeDef TIM_OCInitStructure;

	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_Pulse = ((color >> (8 * 2)) & 0xFF) / 0xFF * 100;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	// need to set color
}

void led::set_color(uint32_t color)
{


}

void led::_init_output_channel(uint32_t intensity, uint16_t led_pin)
{
	// Configure TIM output channel for specified intensity

	TIM_OCInitStructure.TIM_Pulse = (intensity / 0xFF) * TIM_PERIOD;

	switch(led_pin)
	{
		case GPIO_Pin_1:
			TIM_OC2Init(TIM2, &TIM_OCInitStructure);
			break;
		case GPIO_Pin_2:
			TIM_OC3Init(TIM2, &TIM_OCInitStructure);
			break;
		case GPIO_Pin_3:
			TIM_OC4Init(TIM2, &TIM_OCInitStructure);
			break;
	}


}

void led::on()
{

}

void led::off()
{

}

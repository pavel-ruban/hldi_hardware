#include <stdint.h>
#include <stm32f10x_conf.h>
#include <assert.h>
#include "led.hpp"

//using namespace std;


void test_f(uint32_t tp32,uint8_t tp8, uint32_t tp32_1, uint32_t tp32_2, uint32_t tp32_3, uint32_t tp32_4, uint32_t tp32_5, uint32_t tp32_6)
{
	uint32_t loc = tp32+tp8;
}

led::led(uint8_t led_type, GPIO_TypeDef *led_port, uint16_t led_pin, uint8_t led_intensity)
{

	led::type = led_type;

	/* Enable GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// Configure GPIO
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = led_pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(led_port, &GPIO_InitStructure);

	// Enable TIM clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	// Configure TIM2 to PWM
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_Prescaler = TIM_PRESCALER;
	TIM_TimeBaseInitStructure.TIM_Period = TIM_PERIOD;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

	switch(led::type)
	{
	case LED_TYPE_RED:
		led::color = led_intensity << 16;
		led::_init_output_channel( (led::color >> 16) & 0xFF, led_pin);
		led::pin_r = led_pin;
		break;
	case LED_TYPE_GREEN:
		led::color = led_intensity << 8;
		led::_init_output_channel( (led::color >> 8) & 0xFF, led_pin);
		led::pin_g = led_pin;
		break;
	case LED_TYPE_BLUE:
		led::color = led_intensity << 0;
		led::_init_output_channel( (led::color >> 0) & 0xFF, led_pin);
		led::pin_b = led_pin;
		break;
	}

	TIM_Cmd(TIM2, ENABLE);
}

//led::led(uint8_t led_type, GPIO_TypeDef *led_port, uint16_t led_pin_r, uint16_t led_pin_g, uint16_t led_pin_b, uint32_t led_color)
//{
//
//	led::type = led_type;
//	led::color = led_color;
//	led::pin_r = led_pin_r;
//	led::pin_g = led_pin_g;
//	led::pin_b = led_pin_b;
//
//	/* Enable GPIOA clock */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//
//
//	// Configure GPIO
//	GPIO_InitTypeDef GPIO_InitStructure;
//
//	GPIO_InitStructure.GPIO_Pin = led_pin_r;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//	GPIO_Init(led_port, &GPIO_InitStructure);
//
//	GPIO_InitStructure.GPIO_Pin = led_pin_g;
//	GPIO_Init(led_port, &GPIO_InitStructure);
//
//	GPIO_InitStructure.GPIO_Pin = led_pin_b;
//	GPIO_Init(led_port, &GPIO_InitStructure);
//
//	// Enable TIM clock
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
//
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
//
//	// Configure TIM2 to PWM
//	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
//	TIM_TimeBaseInitStructure.TIM_Prescaler = TIM_PRESCALER;
//	TIM_TimeBaseInitStructure.TIM_Period = TIM_PERIOD;
//	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);
//
//	led::_init_output_channel( (led::color >> 16) & 0xFF, led_pin_r);
//	led::_init_output_channel( (led::color >>  8) & 0xFF, led_pin_g);
//	led::_init_output_channel( (led::color >>  0) & 0xFF, led_pin_b);
//}

void led::set_color(uint32_t color)
{
	led::color = color;

	switch(led::type)
	{
	case LED_TYPE_RGB:

		led::_init_output_channel( (led::color >> 16) & 0xFF, led::pin_r);
		led::_init_output_channel( (led::color >>  8) & 0xFF, led::pin_g);
		led::_init_output_channel( (led::color >>  0) & 0xFF, led::pin_b);
		break;
	}

}

void led::_init_output_channel(uint32_t intensity, uint16_t led_pin)
{
	// Configure TIM output channel for specified intensity
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
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

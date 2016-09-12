#include <stdint.h>
#include <stm32f10x_conf.h>
#include <assert.h>
//#include "system/include/cmsis/stm32f10x.h"
#include "led.hpp"




led::led(uint8_t led_type, GPIO_TypeDef *led_port, uint16_t led_pin, uint8_t led_intensity)
{
	//Color color;
	led::type = led_type;
    led::pin_s = led_pin;
    led::s_led_intensity = led_intensity;

	/* Enable GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	// Configure GPIO
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = led_pin;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
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
		led::color.Uncolored = led_intensity << 16;
		led::_init_output_channel( (color.Uncolored) & 0xFF, led_pin);
		led::pin_r = led_pin;
		break;
	case LED_TYPE_GREEN:
		led::color.Uncolored = led_intensity << 8;
		led::_init_output_channel( (color.Uncolored) & 0xFF, led_pin);
		led::pin_g = led_pin;
		break;
	case LED_TYPE_BLUE:
		led::color.Uncolored = led_intensity << 0;
		led::_init_output_channel( (color.Uncolored) & 0xFF, led_pin);
		led::pin_b = led_pin;
		break;
	}

	TIM_Cmd(TIM2, ENABLE);
}

led::led(uint8_t led_type, GPIO_TypeDef *led_port, uint16_t led_pin_r, uint16_t led_pin_g, uint16_t led_pin_b, uint32_t led_color)
{

	//Color color;
	led::type = led_type;
	//led::color = led_color;
	led::pin_r = led_pin_r;
	led::pin_g = led_pin_g;
	led::pin_b = led_pin_b;
	led::color.Red = led_color >> 16;
	led::color.Green = led_color >> 8;
	led::color.Blue = led_color >> 0;

	/* Enable GPIOA clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);


	// Configure GPIO
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin = led_pin_r;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(led_port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = led_pin_g;
	GPIO_Init(led_port, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = led_pin_b;
	GPIO_Init(led_port, &GPIO_InitStructure);

	// Enable TIM clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	// Configure TIM2 to PWM
	TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	TIM_TimeBaseInitStructure.TIM_Prescaler = TIM_PRESCALER;
	TIM_TimeBaseInitStructure.TIM_Period = TIM_PERIOD;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

	led::_init_output_channel( led::color.Red & 0xFF, led_pin_r);
	led::_init_output_channel( led::color.Green & 0xFF, led_pin_g);
	led::_init_output_channel( led::color.Blue & 0xFF, led_pin_b);

	TIM_Cmd(TIM2, ENABLE);

}

void led::set_color(Color  _color)
{
	color = _color;

	if (led::type == LED_TYPE_RGB) {
		led::_init_output_channel(color.Red & 0xFF, led::pin_r);
		led::_init_output_channel(color.Green & 0xFF, led::pin_g);
		led::_init_output_channel(color.Blue & 0xFF, led::pin_b);
	} else {
		switch (led::type) {
			case LED_TYPE_RED:
				TIM_SetCompare2(TIM2,color.Uncolored * TIM_PERIOD / 0xFF);
				break;
			case LED_TYPE_GREEN:
				TIM_SetCompare2(TIM2,color.Uncolored * TIM_PERIOD / 0xFF);
				break;
			case LED_TYPE_BLUE:
				TIM_SetCompare2(TIM2,color.Uncolored * TIM_PERIOD / 0xFF);
				break;


		}
	}

}

void led::set_color(uint32_t _color)
{
	led::color.Uncolored = (_color >> 24) & 0xFF;
	led::color.Red = (_color >> 16) & 0xFF;
	led::color.Green = (_color >> 8) & 0xFF;
	led::color.Blue = (_color >> 0) & 0xFF;

	if (led::type == LED_TYPE_RGB) {
		led::_init_output_channel(color.Red & 0xFF, led::pin_r);
		led::_init_output_channel(color.Green & 0xFF, led::pin_g);
		led::_init_output_channel(color.Blue & 0xFF, led::pin_b);
	} else {
		switch (led::type) {
			case LED_TYPE_RED:
				TIM_SetCompare2(TIM2,color.Uncolored * TIM_PERIOD / 0xFF);
				break;
			case LED_TYPE_GREEN:
				TIM_SetCompare2(TIM2,color.Uncolored * TIM_PERIOD / 0xFF);
				break;
			case LED_TYPE_BLUE:
				TIM_SetCompare2(TIM2,color.Uncolored * TIM_PERIOD / 0xFF);
				break;


		}
	}

}


void led::set_intensity(uint8_t intensity){
	led::color.Uncolored = intensity;
	TIM_SetCompare2(TIM2,intensity * TIM_PERIOD / 0xFF);
}

void led::_init_output_channel(uint32_t intensity, uint16_t led_pin)
{
	// Configure TIM output channel for specified intensity
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OCInitStructure.TIM_Pulse = intensity * TIM_PERIOD / 0xFF;

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
	if (led::type == LED_TYPE_RGB) {
		TIM_SetCompare2(TIM2, color.Red * TIM_PERIOD / 0xFF - 1);
		TIM_SetCompare3(TIM2, color.Green * TIM_PERIOD / 0xFF - 1);
		TIM_SetCompare4(TIM2, color.Blue * TIM_PERIOD / 0xFF - 1);

	} else
	{
		TIM_SetCompare2(TIM2, color.Uncolored * TIM_PERIOD / 0xFF - 1);
		TIM_SetCompare3(TIM2, color.Uncolored * TIM_PERIOD / 0xFF - 1);
		TIM_SetCompare4(TIM2, color.Uncolored * TIM_PERIOD / 0xFF - 1);
	}

}

void led::off()
{
	TIM_SetCompare2(TIM2, 0);
	TIM_SetCompare3(TIM2, 0);
	TIM_SetCompare4(TIM2, 0);
}

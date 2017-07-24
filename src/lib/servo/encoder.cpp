#include "servo.h"
#include "utils.h"

namespace Servo
{
	GPIO_InitTypeDef GPIO_InitStructure;

	static uint8_t hardware_initialized = 0;

	Encoder::Encoder()
	{
		if (hardware_initialized) return;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

		GPIO_InitStructure.GPIO_Pin = ENCODER_A_CH1_PIN | ENCODER_A_CH2_PIN;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
		GPIO_Init(ENCODER_A_PORT, &GPIO_InitStructure);

		// Enable TIM clock.
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

		TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

		// Configure TIM3 to Encoder.
		TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
		TIM_TimeBaseInitStructure.TIM_Prescaler = DEFAULT_ENCODER_TIM_PRESCALER;
		TIM_TimeBaseInitStructure.TIM_Period = DEFAULT_ENCODER_TIM_PERIOD;
		TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
		TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);

		TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);

		TIM_ICInitTypeDef TIM_ICInitStructure;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1 | TIM_Channel_2;
		TIM_ICStructInit(&TIM_ICInitStructure);

		TIM_ICInitStructure.TIM_ICFilter = 6; //ICx_FILTER;

		TIM_ICInit(TIM3, &TIM_ICInitStructure);
		TIM_Cmd(TIM3, ENABLE);
		// Configure TIM output channel for specified pwm.
//		TIM_OCInitTypeDef TIM_OCInitStructure;
//
//		// TIM_OCStructInit(&TIM_OCInitStructure).
//		TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
//		TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//		TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//
//		// PB[10, 11] pins use TIM2 counter, but each pin is mapped to own OC -
//		// timer channel, it's value determines the PWM duty cyle alongside with
//		// other settings (PWM mode, polarity etc).
//		TIM_OC4Init(TIM2, &TIM_OCInitStructure);
//		TIM_OC3Init(TIM2, &TIM_OCInitStructure);
//
//		TIM_Cmd(TIM2, ENABLE);

		hardware_initialized = 1;
	}
}
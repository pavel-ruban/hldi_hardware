#include "servo.h"
#include "utils.h"

namespace Servo
{
	GPIO_InitTypeDef GPIO_InitStructure;

	static uint8_t hardware_initialized = 0;

	Encoder::Encoder(uint16_t ppr) {
		this->ppr = ppr;

		Encoder();
	}

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

		TIM_ICInitTypeDef TIM_ICInitStructure;
		TIM_ICInitStructure.TIM_Channel = TIM_Channel_1 | TIM_Channel_2;
		TIM_ICStructInit(&TIM_ICInitStructure);

		// ICx_FILTER.
//		TIM_ICInitStructure.TIM_ICFilter = 6;
		TIM_ICInit(TIM3, &TIM_ICInitStructure);

		TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);

		TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

		TIM_Cmd(TIM3, ENABLE);

		hardware_initialized = 1;
	}

	void Encoder::overflow(uint8_t negative)
	{
		overflows += negative ? -1 : 1;
	}

	int32_t Encoder::pos()
	{
		return (int32_t) TIM_GetCounter(TIM3) + (int32_t) (overflows * 0xFFFF);
	}
}

#include "servo.h"
#include "utils.h"

namespace Servo {

    extern GPIO_InitTypeDef GPIO_InitStructure;

	class Servo;

	static uint8_t hardware_initialized = 0;

    Motor::Motor(Servo *servo) {
        // Init pins if we create the instance of class per current MCU session.
        if (!hardware_initialized) {
            // Ensure bus is tact. Bind GPIOA, GPIOB to APB2 bus.
            RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

            // Please be careful this logic controls H-bridge done on N & P channel mosfets. So
            // you shouldn't have situation when these combinations of pins are active:
            // a) DC2 = 1, DC3 = 0 WHILE DC0 = 1, DC1 = 0 - SHORT CIRCUIT!!!
            // b) DC2 = 0, DC3 = 1 WHILE DC0 = 0, DC1 = 1 - SHORT CIRCUIT!!!
            // If will burn out you circuit out. As there would be short circuirt through N & P.
            GPIO_InitStructure.GPIO_Pin = HBRIDGE_DC2 | HBRIDGE_DC3;
            GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
            GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
            GPIO_Init(HBRIDGE_PORT, &GPIO_InitStructure);

	        // Pull all hbridge pins to GND.
	        GPIO_ResetBits(HBRIDGE_PORT, HBRIDGE_DC2 | HBRIDGE_DC3);

			// Configure PWM pins.
	        GPIO_InitStructure.GPIO_Pin = HBRIDGE_DC0 | HBRIDGE_DC1;
			// Disable PWM pins by default to avoid short circuit, as if timer would be activated
	        // it can pulse & cause N & P channels be opened together at once on the same side.
	        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	        GPIO_Init(HBRIDGE_PORT, &GPIO_InitStructure);

	        // Activate timers on PB10 & PB11.
	        GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);
	        GPIO_PinRemapConfig(GPIO_PartialRemap2_TIM2, ENABLE);

	        // Enable TIM clock.
	        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	        TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

	        // Configure TIM2 to PWM.
	        TIM_TimeBaseStructInit(&TIM_TimeBaseInitStructure);
	        TIM_TimeBaseInitStructure.TIM_Prescaler = DEFAULT_PWM_TIM_PRESCALER;
	        TIM_TimeBaseInitStructure.TIM_Period = DEFAULT_PWM_TIM_PERIOD;
	        TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

	        // Configure TIM output channel for specified pwm.
	        TIM_OCInitTypeDef TIM_OCInitStructure;

	        // TIM_OCStructInit(&TIM_OCInitStructure).
	        TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	        TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	        TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	        TIM_OCInitStructure.TIM_Pulse = _pwm;

	        // PB[10, 11] pins use TIM2 counter, but each pin is mapped to own OC -
	        // timer channel, it's value determines the PWM duty cyle alongside with
	        // other settings (PWM mode, polarity etc).
	        TIM_OC4Init(TIM2, &TIM_OCInitStructure);
	        TIM_OC3Init(TIM2, &TIM_OCInitStructure);

	        TIM_Cmd(TIM2, ENABLE);

	        this->servo = servo;
	        _on = 0;

            hardware_initialized = 1;
        }

    }

    Motor::Motor(uint8_t _pwm, Servo *servo) {
	    this->_pwm = _pwm;

	    this->servo = servo;

        Motor(this->servo);
    }

    void Motor::set_pins() {
        // Disable all pins to avoid unexpected short circuit.
        GPIO_ResetBits(HBRIDGE_PORT, HBRIDGE_DC2 | HBRIDGE_DC3);
	    PINS_OUT_PD(HBRIDGE_PORT, HBRIDGE_DC0 | HBRIDGE_DC1);

        // Consider previously set direction.
	    switch (this->_cw)
	    {
		    case CLOCKWISE:
			    GPIO_SetBits(HBRIDGE_PORT, HBRIDGE_DC2);
			    PINS_ON(HBRIDGE_PORT, HBRIDGE_DC1);
			    break;

		    case COUNTER_CLOCKWISE:
			    GPIO_SetBits(HBRIDGE_PORT, HBRIDGE_DC3);
			    PINS_ON(HBRIDGE_PORT, HBRIDGE_DC0);
			    break;
	    }
    }

    void Motor::on() {
        set_pins();
	    _on = 1;
    }

    void Motor::off() {
	    // Disable all pins to avoid unexpected short circuit.
        GPIO_ResetBits(HBRIDGE_PORT, HBRIDGE_DC2 | HBRIDGE_DC3);
	    PINS_OUT_PD(HBRIDGE_PORT, HBRIDGE_DC0 | HBRIDGE_DC1);

	    _on = 0;
    }

    void Motor::dir_cw() {
        if (!_cw) {
            dir_toggle();
        }
    }

    void Motor::dir_ccw() {
        if (_cw) {
            dir_toggle();
        }
    }

    void Motor::dir_toggle() {
        _cw ^= 1;

	    servo->position_pid.reset_integral();

        set_pins();

	    servo->position_pid.reset_ignore_error_level();
    }

    void Motor::pwm(uint8_t pwm) {
	    _pwm = pwm;

	    TIM_SetCompare3(TIM2, _pwm);
	    TIM_SetCompare4(TIM2, _pwm);
    }
}

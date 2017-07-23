#pragma once
#pragma pack(1)

/**
 * This is software to control DC brushhed motor over H-bridge on N & P mosftes.
 *
 * @author Pavel Ruban http://pavelruban.org
 * @email apps.pavelruban@gmail.com
 */
#include <stdint.h>
#include <stm32f10x_conf.h>

#define HBRIDGE_PORT GPIOB
// DC0 & DC1 pins control N channel mosfet of H-bridge over PWM HCPL3120 driver.
#define HBRIDGE_DC0 GPIO_Pin_10
#define HBRIDGE_DC1 GPIO_Pin_11
// DC2 & DC3 pins control P channel mosfet of H-bridge over PC-817 optocoupler.
#define HBRIDGE_DC2 GPIO_Pin_12
#define HBRIDGE_DC3 GPIO_Pin_13

// Caution!!! Chage this value only understanding what you doing, if you set it
// too low MOSFETs won't have time to open fully & due to the POWER on the chip
// it will be burned out. Actually current software was writted for HCPL3120, so
// please don't go ahead 10Khz, 0.5-4Khz should work fine.
#define DEFAULT_PWM_TIM_PRESCALER 280
#define DEFAULT_PWM_TIM_PERIOD 255

#define COUNTER_CLOCKWISE 0
#define CLOCKWISE 1

namespace Servo {
    extern int hardware_initialized;

    class Servo {

    public:
        Servo();

        // Allows to set initial pwm value.
        Servo(uint8_t pwm);

        // Enables motor current. Direct is that was before.
        void on();

        // Disables motor current.
        void off();

        // Clockwise, counter cw direction control.
        void dir_cw();

        void dir_ccw();

        // Toggles direction.
        void dir_toggle();

        // Apllies set direction.
        void set_pins();

        // Sets PWM between 0-255, 255 100% duty cycle.
        void pwm(uint8_t duty);

    private:
        // Direction, used when motor is enabled / disabled to keep the same direction.
        uint8_t _cw;

	    // Set motor power by default to 100% to avoid cases when user invokes ::on() but
	    // motor doesn't rotate.
	    // 0-255 Stores set pwm value.
	    uint8_t _pwm = 255;
    };
}

/**
 * This is software to control DC brushhed motor over H-bridge on N & P mosftes.
 *
 * @author Pavel Ruban http://pavelruban.org
 * @email apps.pavelruban@gmail.com
 */

#pragma once
#pragma pack(1)

#include <stdint.h>
#include <stm32f10x_conf.h>

#define HBRIDGE_PORT GPIOB
// DC0 & DC1 pins control N channel mosfet of H-bridge over PWM HCPL3120 driver.
#define HBRIDGE_DC0 GPIO_Pin_10
#define HBRIDGE_DC1 GPIO_Pin_11
// DC2 & DC3 pins control P channel mosfet of H-bridge over PC-817 optocoupler.
#define HBRIDGE_DC2 GPIO_Pin_12
#define HBRIDGE_DC3 GPIO_Pin_13

#define ENCODER_A_PORT GPIOA
#define ENCODER_A_CH1_PIN GPIO_Pin_6
#define ENCODER_A_CH2_PIN GPIO_Pin_7
#define DEFAULT_ENCODER_TIM_PRESCALER 0
#define DEFAULT_ENCODER_TIM_PERIOD 0xffff

// Caution!!! Chage this value only understanding what you doing, if you set it
// too low MOSFETs won't have time to open fully & due to the POWER on the chip
// it will be burned out. Actually current software was writted for HCPL3120, so
// please don't go ahead 10Khz, 0.5-4Khz should work fine.
#define DEFAULT_PWM_TIM_PRESCALER 280
#define DEFAULT_PWM_TIM_PERIOD 255

#define COUNTER_CLOCKWISE 0
#define CLOCKWISE 1

namespace Servo {
	class Servo;

	class PID {

	public:
		PID(float Kp, float Ki, float Kd, Servo *servo);

		// Sets desired setpoint.
		void set_setpoint(float point);

		// CV, see PID theory for reference.
		int16_t get_ctrl_var();

		// Sets proportional, integral, derivative terms.
		void set_terms(float Kp, float Ki, float Kd);

		void set_kp_term(float Kp);
		void set_ki_term(float Ki);
		void set_kd_term(float Kd);

		// Desired value that PID attempts to reach.
		float get_setpoint();

	private:
		// Servo system current PID belongs to.
		Servo *servo;

		// Last calculation time for dt capability.
		uint32_t last_ticks;

		// Proportional, integral, derivative terms.
		float Kp, Ki, Kd;
		float setpoint;

		// Internal data used for calucaltions.
		float integral, previous_error;
	};

	class Encoder {
		friend class Servo;

	public:
		Encoder();
		Encoder(uint16_t ppr);

		// Return current encoder position.
		int32_t pos();

		// Trigger overflow buffer logic.
		void overflow(uint8_t negative);

	private:
		// Encoder overflows. Encoder is implemented with hardware timer encoder interface.
		// Timer is 16 bits so if encoder overflows up/down it increases/decreases this
		// overflows counter accordingly so logic can use TIMER_CNT + overflows * 0xFFFF
		// formula to determine how much timer ticker have been occurred since system boot.
		int overflows;

		// Pulses per revolution.
		uint16_t ppr;
	};

    class Motor {
		friend class Servo;

    public:
	    Motor(Servo *servo);

        // Allows to set initial pwm value.
        Motor(uint8_t pwm, Servo *servo);

	    // Enables motor current. Direct is that was before.
        void on();

        // Disables motor current.
        void off();

        // Clockwise, counter cw direction control.
        void dir_cw();

        void dir_ccw();

        // Toggles direction.
        void dir_toggle();

        // Applies set direction.
        void set_pins();

        // Sets PWM between 0-255, 255 100% duty cycle.
        void pwm(uint8_t duty);

    private:
	    // Servo system current motor belongs to.
	    Servo *servo;

        // Direction, used when motor is enabled / disabled to keep the same direction.
        uint8_t _cw;

	    // Set motor power by default to 100% to avoid cases when user invokes ::on() but
	    // motor doesn't rotate.
	    // 0-255 Stores set pwm value.
	    uint8_t _pwm = 255;

    };

    class Servo {
	    // Share encoder to PID & some other private vars.
		friend class PID;
	    friend class Encoder;

    public:
        Servo();
        Servo(uint16_t ppr, float pitch);

	    // PID controllers.
	    PID speed_pid;
	    PID position_pid;
	    PID acceleration_pid;

	    // Enables motor current. Direct is that was before.
	    void on();

	    // Disables motor current.
	    void off();

	    // Clockwise, counter cw direction control.
	    void dir_cw();

	    void dir_ccw();

	    // Get current direction.
	    uint8_t get_dir();

	    // Toggles direction.
	    void dir_toggle();
	    // Sets pwm duty cycle = [0, 255].
	    void pwm(uint8_t pwm);

	    // Get current positin in mm.
	    float position();

	    // Quadrature incremental hardware timer encoder initialization.
	    Encoder encoder;

    private:
	    // DC brushed 24V motor.
	    Motor motor;

	    // Pitch is how much linear path done  by one ratation of motor shaft.
	    float pitch;

    };

}

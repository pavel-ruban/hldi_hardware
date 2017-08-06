#include "servo.h"
#include "utils.h"

#define INIT_DEFAULT_PIDS                           \
speed_pid(10, 0, 0, this),                          \
	position_pid(0.05, 0.001, 0, this),             \
	acceleration_pid(10, 0, 0, this)

namespace Servo
{
	Servo::Servo() : INIT_DEFAULT_PIDS, motor(this)	{}
	Servo::Servo(uint16_t ppr, double pitch) : INIT_DEFAULT_PIDS, motor(this), encoder(ppr) {
		this->pitch = pitch;
	}

	// Enables motor current. Direct is that was before.
	void Servo::on()
	{
		motor.on();
	}

	// Disables motor current.
	void Servo::off()
	{
		motor.off();
	}

	// Clockwise, counter cw direction control.
	void Servo::dir_cw()
	{
		motor.dir_cw();
	}

	void Servo::dir_ccw()
	{
		motor.dir_ccw();
	}

	// Toggles direction.
	void Servo::dir_toggle()
	{
		motor.dir_toggle();
	}

	uint8_t Servo::get_dir()
	{
		return motor._cw;
	}

	double Servo::position()
	{
		// Get distance / position in mm eg 1.003 mm. E.g. ppr 2000 it for 2 channel quadrature
		// encoder it means 2000 * 2 (channels number) * 2 (raise, fall edges) = 8000 ticks per
		// revolution, pitch is path length per revolution, so picth / 8000 = length of 1 tick.
		// ticks * length of 1 tick is total distance or position that motor passed.
		return encoder.pos() * (pitch / (encoder.ppr * 4));
	}

	void Servo::pwm(uint8_t pwm) {
		motor.pwm(pwm);
	}

	uint8_t Servo::get_pwm() {
		return motor._pwm;
	}

}

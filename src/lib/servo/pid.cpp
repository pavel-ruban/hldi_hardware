#include "servo.h"

extern uint32_t ticks;

namespace Servo
{
	PID::PID(float Kp, float Ki, float Kd, Servo *servo)
	{
		this->servo = servo;

		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
	}

	void PID::set_terms(float Kp, float Ki, float Kd)
	{
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
	}

	void PID::set_kp_term(float Kp)
	{
		this->Kp = Kp;
	}

	void PID::set_ki_term(float Ki)
	{
		this->Ki = Ki;
	}

	void PID::set_kd_term(float Kd)
	{
		this->Kd = Kd;
	}

	void PID::set_setpoint(float point)
	{
		setpoint = point;
	}

	float PID::get_setpoint()
	{
		return setpoint;
	}

	int16_t PID::get_ctrl_var()
	{
		float error = setpoint - servo->position();

		float dt = (double) (ticks - last_ticks) / 1000;

		integral += error * dt;
		float derivative = (error - previous_error) / dt;

		float f_cv = Kp * error + Ki * integral + Kd * derivative;

		if (f_cv > 255) {
			f_cv = 255;
		} else if (f_cv < -255) {
			f_cv = -255;
		}

		// Our implementation uses values between -255 to 255 for CW CCW pwm control.
		previous_error = error;

		return (int16_t) f_cv;
	}
}

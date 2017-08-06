#include "servo.h"
#include <stdlib.h>
#include <stdio.h>
#include "uart/uart.h"

extern uint32_t ticks;
extern Uart *puart;

extern "C" {
#include <string.h>
}

namespace Servo
{
	PID::PID(double Kp, double Ki, double Kd, Servo *servo)
	{
		this->servo = servo;

		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
	}

	void PID::set_terms(double Kp, double Ki, double Kd)
	{
		this->Kp = Kp;
		this->Ki = Ki;
		this->Kd = Kd;
	}

	void PID::set_kp_term(double Kp)
	{
		this->Kp = Kp;
	}

	void PID::set_ki_term(double Ki)
	{
		this->Ki = Ki;
	}

	void PID::set_kd_term(double Kd)
	{
		this->Kd = Kd;
	}

	void PID::set_setpoint(double point)
	{
		setpoint = point;
	}

	double PID::get_setpoint()
	{
		return setpoint;
	}

	void PID::reset_integral() {
		integral = 0;
	}

	void PID::reset_ignore_error_level() {
		ignore_error_level = 0;
	}

	double PID::get_integral() {
		return integral;
	}

	double PID::get_prev_error() {
		return previous_error;
	}

	int16_t PID::get_ctrl_var()
	{
		if (!servo->motor._on)
			return 0;

		double error = setpoint - servo->position();

		// Set accuracy level, if error below required accuracy no need to adjust position.
		if ((error > 0 && error <= 0.1) || (error < 0 && error >= -0.1)) {
			return 0;
		}

		double dt = (double) (ticks - last_ticks) / 1000.;

		bool is_low_error_threshold = !(error > 2. || error < -2.);

		// Use integral only for low level errors.
		if (is_low_error_threshold) {
			integral += error * dt;
		}

		double derivative = (error - previous_error) / dt;

		double d_cv;
		int16_t f_cv;

		if (is_low_error_threshold) {
			d_cv = Kp * error + Ki * integral + Kd * derivative;
		} else {
			d_cv = Kp * error + Kd * derivative;
		}

		if (d_cv >= 0) {
			f_cv = d_cv < 1. && d_cv > 0. ? (int16_t) 1 : (int16_t) d_cv;
		} else {
			f_cv = d_cv > -1. ? (int16_t) -1 : (int16_t) d_cv;
		}

		if (f_cv > 255) {
			f_cv = 255;
		} else if (f_cv < -255) {
			f_cv = -255;
		}

		// Motor has threshold of needed pwm to start turning.
		if (ignore_error_level || !is_low_error_threshold) {
			ignore_error_level = 1;

			if (f_cv > 0 && f_cv < 16) {
				f_cv = 16;
			} else if (f_cv < 0 && f_cv > -14) {
				f_cv = -14;
			}
		}

		// Our implementation uses values between -255 to 255 for CW CCW pwm control.
		previous_error = error;

		return f_cv;
	}
}

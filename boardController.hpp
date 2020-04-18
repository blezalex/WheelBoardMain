#pragma once
#include <math.h>

#include "global.h"
#include "pid.hpp"
#include "imu/imu.hpp"
#include "stateTracker.hpp"
#include "lpf.hpp"
#include "drv/vesc/vesc.hpp"
#include "io/genericOut.hpp"
#include "io/pwm_out.hpp"

#define BRAKE_VIA_USART

class BoardController  : public UpdateListener  {
public:
	BoardController(Config* settings, IMU& imu, PwmOut& motor_out, GenericOut& status_led,
			GenericOut& beeper, Guard** guards, int guards_count, GenericOut& green_led, VescComm* vesc)
	  : settings_(settings),
		imu_(imu),
		state_(guards, guards_count),
		balancer_(settings_),
		ppm_motor_out_(motor_out),
		status_led_(status_led),
		beeper_(beeper),
		avg_running_motor_out_(&settings->misc.throttle_rc),
		green_led_(green_led),
		motor_out_lpf_(&settings->balance_settings.output_lpf_hz),
		vesc_(vesc) {
	}

	float filterMotorCommand(float cmd) {
		cmd = constrain(cmd, -1.0f, 1.0f);
		float max_update = settings_->balance_settings.max_update_limiter / 512.0f;
		float new_out = constrain(cmd, prev_out_ - max_update, prev_out_ + max_update);
		new_out = motor_out_lpf_.compute(new_out);

		prev_out_ = new_out;
		return new_out;
	}

	// Send BRAKE_MOTOR_CMD after a short delay after controller goes into Stopped state.
	void processBrakes() {
		if (brakes_on_) {
			setMotorOutput(BRAKE_MOTOR_CMD);
			first_stopped_to_brake_iteration_ = true;
			return;
		}

		setMotorOutput(0);
		if (first_stopped_to_brake_iteration_) {
			first_stopped_to_brake_iteration_ = false;
			stopped_since_ts_ = millis();
		}
		else {
			if (millis() - stopped_since_ts_ > 600u) {
				brakes_on_ = true;
			}
		}
	}

	// Main control loop. Runs at 1000hz Must finish in less than 1ms otherwise controller will freeze.
	void processUpdate(const MpuUpdate& update) {
		imu_.compute(update);
		State current_state = state_.update();

		switch (current_state) {
		case State::Stopped:
			processBrakes();
			status_led_.setState(0);
			beeper_.setState(0);
			break;

		case State::FirstIteration:
			brakes_on_ = false;
			balancer_.reset();
			motor_out_lpf_.reset(0);
			prev_out_ = 0;
			avg_running_motor_out_.reset();
			status_led_.setState(1);
			// intentional fall through
		case State::Starting:
			setMotorOutput(filterMotorCommand(balancer_.computeStarting((float*)imu_.rates, (float*)imu_.angles, state_.start_progress())));
			break;

		case State::Running:
			float out = balancer_.compute((float*)imu_.rates, (float*)imu_.angles, balance_angle_)
				+ vesc_->mc_values_.erpm_smoothed * settings_->misc.speed_input_mixin * 0.002f; // 0.002 to keep old config value. TODO: update cfg and get rid of multiplier

			setMotorOutput(filterMotorCommand(out));

			bool warning_requested = shouldWarn(out);
			beeper_.setState(warning_requested);

			// updateBalancingSetPoint(warning_requested && abs(vesc_->mc_values_.erpm_smoothed) > settings_->pushback.min_speed_erpm, vesc_->mc_values_.erpm_smoothed > 0);
			break;
		}

		if (vesc_update_cycle_ctr_++ >= 50) {
			// request a stats update every 50 cycles => 20hz
			vesc_update_cycle_ctr_ = 0;
			vesc_->requestStats();
		}
	}

	bool shouldWarn(float current_throttle) {
		float smoothed_throttle = avg_running_motor_out_.compute(current_throttle);

		bool warning_requested = false;
		warning_requested |= fabsf(smoothed_throttle) >= settings_->misc.throttle_threshold;
		warning_requested |= fabsf(vesc_->mc_values_.duty_smoothed) > settings_->misc.duty_threshold;
		warning_requested |= abs(vesc_->mc_values_.erpm_smoothed) > settings_->misc.erpm_threshold;
		warning_requested |= vesc_->mc_values_.v_in_smoothed < settings_->misc.low_volt_threshold;
		return warning_requested;
	}

	void updateBalancingSetPoint(bool shouldPushback, bool forward) {
		if (!shouldPushback) {
			if (balance_angle_ < 0) {
				balance_angle_ += settings_->pushback.push_release_speed_deg_sec / 1000;
				if (balance_angle_ > 0) {
					balance_angle_ = 0;
				}
			}
			else {
				balance_angle_ -= settings_->pushback.push_release_speed_deg_sec  / 1000;
				if (balance_angle_ < 0) {
					balance_angle_ = 0;
				}
			}
			return;
		}

		const int32_t max_push_angle = abs(settings_->pushback.push_angle);
		if (forward && settings_->pushback.push_angle > 0) {
			balance_angle_ += settings_->pushback.push_raise_speed_deg_sec / 1000;
			if (balance_angle_ > max_push_angle) {
				balance_angle_ = max_push_angle;
			}
		}
		else {
			balance_angle_ -= settings_->pushback.push_raise_speed_deg_sec  / 1000;
			if (balance_angle_ < -max_push_angle) {
				balance_angle_ = -max_push_angle;
			}
		}
	}

	void setMotorOutput(float cmd) {
		float usart_scaling = settings_->balance_settings.usart_control_scaling;
		if (fabs(usart_scaling) > 0) {
			// Using usart for control
			ppm_motor_out_.set(0);

			if (cmd == BRAKE_MOTOR_CMD) {
				vesc_->setCurrentBrake(20);
			}
			else {
				vesc_->setCurrent(cmd * usart_scaling);
			}
		}
		else {
			if (cmd == BRAKE_MOTOR_CMD) {
				ppm_motor_out_.set(0);

#ifdef BRAKE_VIA_USART
				vesc_->setCurrentBrake(20);
#endif
			}
			else {
				ppm_motor_out_.set(fmap(cmd, -1.0f, 1.0f, MIN_MOTOR_CMD, MAX_MOTOR_CMD));
			}
		}
	}

	float getLastOut() {
		return prev_out_;
	}

private:
	Config* settings_;
	IMU& imu_;
	StateTracker state_;
	BalanceController balancer_;
	PwmOut& ppm_motor_out_;
	GenericOut& status_led_;
	GenericOut& beeper_;
	// avg value for zero-center motor out. range [-1:1]
	LPF avg_running_motor_out_;
	float prev_out_;
	GenericOut& green_led_;
	BiQuadLpf motor_out_lpf_;
	uint16_t stopped_since_ts_;
	bool brakes_on_ = false;
	bool first_stopped_to_brake_iteration_ = true;

	VescComm* vesc_;
	int vesc_update_cycle_ctr_ = 0;

	float balance_angle_ = 0;
};

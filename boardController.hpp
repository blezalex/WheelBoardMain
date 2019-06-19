#pragma once
#include <math.h>

#include "global.h"
#include "pid.hpp"
#include "imu/imu.hpp"
#include "lpf.hpp"


class BoardController  : public UpdateListener  {
public:
	BoardController(Config* settings, IMU& imu, PwmOut& motor_out, GenericOut& status_led,
			GenericOut& beeper, Guard** guards, int guards_count, GenericOut& green_led)
	  : settings_(settings),
		imu_(imu),
		state_(guards, guards_count),
		balancer_(settings_),
		motor_out_(motor_out),
		status_led_(status_led),
		beeper_(beeper),
		avg_running_motor_out_(&settings->misc.beep_rc),
		green_led_(green_led),
		motor_out_lpf_(&settings->balance_settings.output_lpf_rc){
	}

	uint16_t mapOutToPwm(int32_t balancer_out) {
		uint16_t out = constrain(balancer_out + NEUTRAL_MOTOR_CMD, MIN_MOTOR_CMD, MAX_MOTOR_CMD);
		uint16_t max_update = settings_->balance_settings.max_update_limiter;
		uint16_t new_out = constrain(out, prev_out_ - max_update, prev_out_ + max_update);
		new_out = (uint16_t)motor_out_lpf_.compute(new_out);

		prev_out_ = new_out;
		return new_out;
	}

	// Main control loop. Runs at 1000hz Must finish in less than 1ms otherwise controller will freeze.
	void processUpdate(const MpuUpdate& update) {
		imu_.compute(update);
		State current_state = state_.update();

		switch (current_state) {
		case State::Stopped:
			motor_out_.set(NEUTRAL_MOTOR_CMD);
			status_led_.setState(0);
			beeper_.setState(0);
			break;

		case State::FirstIteration:
			balancer_.reset();
			motor_out_lpf_.reset(NEUTRAL_MOTOR_CMD);
			prev_out_ = NEUTRAL_MOTOR_CMD;
			avg_running_motor_out_.reset();
			status_led_.setState(1);
			// intentional fall through
		case State::Starting:
			motor_out_.set(mapOutToPwm(balancer_.computeStarting(update.gyro, (float*)imu_.angles, state_.start_progress())));
			break;

		case State::Running:
			int16_t out = balancer_.compute(update.gyro, (float*)imu_.angles);
			motor_out_.set(mapOutToPwm(out));
			float smoothed_out = avg_running_motor_out_.compute(constrain(out, -MOTOR_CMD_RANGE, MOTOR_CMD_RANGE));

			bool warning_requested = abs(smoothed_out) >= MOTOR_CMD_RANGE * settings_->misc.beep_threshold;
			beeper_.setState(warning_requested);
			break;
		}
		have_update_ = true;
	}

	volatile bool have_update_;

private:
	Config* settings_;
	IMU& imu_;
	StateTracker state_;
	BalanceController balancer_;
	PwmOut& motor_out_;
	GenericOut& status_led_;
	GenericOut& beeper_;
	LPF avg_running_motor_out_;
	uint16_t prev_out_;
	GenericOut& green_led_;
	LPF motor_out_lpf_;
};

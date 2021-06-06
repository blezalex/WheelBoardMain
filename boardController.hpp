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
		vesc_(vesc),
		load_lift_current_lpf_(&settings->load_lift.filter_rc),
		load_lift_ramp_(&settings->load_lift.ramp_deg_sec, &settings->load_lift.ramp_deg_sec, &settings->load_lift.max_angle),
		pushback_ramp_(&settings->pushback.push_raise_speed_deg_sec, &settings->pushback.push_release_speed_deg_sec,  &settings->pushback.push_angle),
		motor_acceleration_lpf_(&settings->keep_roll.filter_rc){
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
	void enableBrakes() {
		if (first_stopped_to_brake_iteration_) {
			// Remember the stop time on the first iteration. Don't overwrite it until run again.
			first_stopped_to_brake_iteration_ = false;
			stopped_since_ts32_ = millis32();
			return;
		}

		uint32_t stopped_millis = millis32() - stopped_since_ts32_;
		if (stopped_millis > 1000ul*60ul*1ul) {
			// Timeout after 1 minute, release brakes to allow controller got to sleep.
			return;
		}

		// Wait 400ms, then apply brakes to avoid very hard stop
		if (stopped_millis > 400ul) {
			setMotorOutput(BRAKE_MOTOR_CMD);
			return;
		}

		// Freewheel the motor.
		setMotorOutput(0);
	}

	// Main control loop. Runs at 1000hz Must finish in less than 1ms otherwise controller will freeze.
	void processUpdate(const MpuUpdate& update) {
		imu_.compute(update);
		current_state_ = state_.update();

		switch (current_state_) {
		case State::Stopped:
			enableBrakes();
			status_led_.setState(0);
			beeper_.setState(0);
			break;

		case State::FirstIteration:
			first_stopped_to_brake_iteration_ = true;
			last_current_ = 0;
			balancer_.reset();
			motor_out_lpf_.reset(0);
			prev_out_ = 0;
			avg_running_motor_out_.reset();
			status_led_.setState(1);
			load_lift_ramp_.Reset();
			pushback_ramp_.Reset();
			prev_motor_erpm_ = 0;
			motor_acceleration_lpf_.reset();
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


			balance_angle_ = computeLoadLiftOffset(last_current_, &settings_->load_lift);

			balance_angle_+= computePushbackOffset(warning_requested && fabsf(vesc_->mc_values_.erpm_smoothed) > settings_->pushback.min_speed_erpm, vesc_->mc_values_.erpm_smoothed > 0);

			balance_angle_ += computeKeepRollOffset();
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

	float computeLoadLiftOffset(float unfiltered_motor_current, const Config_LoadLift* lift_settings) {
		const float motor_current = load_lift_current_lpf_.compute(unfiltered_motor_current);
		if (fabs(motor_current) > lift_settings->start_current) {
			float current_with_start_offset = motor_current > 0
					? motor_current - lift_settings->start_current
					: motor_current + lift_settings->start_current;
			return load_lift_ramp_.Compute(current_with_start_offset * lift_settings->multiplier);
		}
		else {
			return load_lift_ramp_.Compute(0);
		}
	}


	float computePushbackOffset(bool shouldPushback, bool forward) {
		if (!shouldPushback) {
			return pushback_ramp_.Compute(0);
		}

		float push_angle = forward ? settings_->pushback.push_angle : -settings_->pushback.push_angle;
		return pushback_ramp_.Compute(push_angle);
	}

	float computeKeepRollOffset() {
		float erpm_acc = motor_acceleration_lpf_.getVal();

		return constrain(erpm_acc * settings_->keep_roll.multiplier,  -settings_->keep_roll.max_angle, settings_->keep_roll.max_angle);
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
				float current = cmd * usart_scaling;
				last_current_ = current;
				vesc_->setCurrent(current);
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

	State current_state() {
		return current_state_;
	}

	void UpdateMotorERPM(float new_val) {
		motor_acceleration_lpf_.compute(new_val - prev_motor_erpm_);
		prev_motor_erpm_ = new_val;
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
	uint32_t stopped_since_ts32_;
	bool first_stopped_to_brake_iteration_ = true;

	VescComm* vesc_;
	int vesc_update_cycle_ctr_ = 0;

	float balance_angle_ = 0;
	State current_state_;

	LPF load_lift_current_lpf_ = 0;
	Ramp load_lift_ramp_;
	Ramp pushback_ramp_;
	float last_current_ = 0;

	float prev_motor_erpm_ = 0;
	LPF motor_acceleration_lpf_;
};

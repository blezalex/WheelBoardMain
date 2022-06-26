#pragma once


#include "global.h"
#include "pid.hpp"
#include "lpf.hpp"


class BalanceController  {
public:
	BalanceController(const Config* settings) :
		settings_(settings), d_lpf_(&settings->balance_settings.balance_d_param_lpf_hz), angle_pid_(&settings->angle_pid), rate_pid_(&settings->rate_pid) {
		reset();
	}

	void reset() {
		angle_pid_.reset();
		rate_pid_.reset();
		d_lpf_.reset();
		max_D_multiplier_so_far_ = 0;
		prev_error_ = 0;
	}

	float getPIInput(float* angles, float balance_angle) {
		float raw_input = (balance_angle - angles[ANGLE_DRIVE]) / settings_->balance_settings.balance_angle_scaling;
		return constrain(raw_input, -1, 1);
	}

	float getPInput(float* angles, float balance_angle) {
		float p_input = getPIInput(angles, balance_angle);
		switch (settings_->balance_settings.expo_type) {
		case 0: return applyExpoReal(p_input, settings_->balance_settings.balance_expo);
		case 1: return applyExpoNatural(p_input, settings_->balance_settings.balance_expo);
		case 2: return applyExpoPoly(p_input, settings_->balance_settings.balance_expo);
		default: return p_input;
		}
	}

	float calcRatePid(float rateRequest,  float rate) {
		float error = rateRequest * 400 - rate;
		float d_term  = error - prev_error_;
		d_term = constrain(d_term, -settings_->balance_settings.balance_d_param_limiter, settings_->balance_settings.balance_d_param_limiter);
		prev_error_ = error;
		d_term = d_lpf_.compute(d_term);

		return rate_pid_.compute(error, d_term);
	}

	// Compute torque needed while board in normal mode.
	// Returns torque request based on current imu and gyro readings. Expected range is [-1:1],
	// but not constrained to that range.
	float compute(const float* gyro_rates, float* angles, float balance_angle) {
		float rateRequest = angle_pid_.compute(getPInput(angles, balance_angle));
		return calcRatePid(rateRequest, -gyro_rates[ANGLE_DRIVE]);
	}

	// Compute torque needed while board in starting up phase (coming from one side to balanced state).
	// Returns torque request based on current imu and gyro readings. Expected range is [-1:1],
	// but not constrained to that range.
	float computeStarting(const float* gyro_rates, float* angles, float pid_P_multiplier) {
		rate_pid_.resetI();
		angle_pid_.resetI();
		float rateRequest = angle_pid_.compute(getPInput(angles, 0));
		rateRequest *= pid_P_multiplier;
		float pid_out = calcRatePid(rateRequest, -gyro_rates[ANGLE_DRIVE]) * pid_P_multiplier;

		return constrain(pid_out, -START_MAX_POWER, START_MAX_POWER);
	}

private:
	const Config* settings_;
	float max_D_multiplier_so_far_ = 0;
	BiQuadLpf d_lpf_;
	PidController angle_pid_;
	PidController rate_pid_;
	float prev_error_ = 0;
};

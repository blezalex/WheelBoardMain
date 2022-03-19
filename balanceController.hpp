#pragma once


#include "global.h"
#include "pid.hpp"
#include "lpf.hpp"


class BalanceController  {
public:
	BalanceController(const Config* settings) :
		settings_(settings), balance_pid_(&settings->balance_pid) {
		reset();
	}

	void reset() {
		balance_pid_.reset();
		max_D_multiplier_so_far_ = 0;
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

	// Compute torque needed while board in normal mode.
	// Returns torque request based on current imu and gyro readings. Expected range is [-1:1],
	// but not constrained to that range.
	float compute(const float* gyro_rates, float* angles, float balance_angle) {
		return balance_pid_.compute(getPInput(angles, balance_angle));
	}

	// Compute torque needed while board in starting up phase (coming from one side to balanced state).
	// Returns torque request based on current imu and gyro readings. Expected range is [-1:1],
	// but not constrained to that range.
	float computeStarting(const float* gyro_rates, float* angles, float pid_P_multiplier) {

		float pid_out = balance_pid_.compute(
			getPInput(angles, 0) * pid_P_multiplier);

		return constrain(pid_out, -START_MAX_POWER, START_MAX_POWER);
	}

private:
	const Config* settings_;
	float max_D_multiplier_so_far_ = 0;
	PidController balance_pid_;
};

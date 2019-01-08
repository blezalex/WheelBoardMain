#pragma once
#include <math.h>

#include "global.h"
#include "pid.hpp"
#include "imu/imu.hpp"

inline int sgn(float val) { return (0 < val) - (val < 0); }

inline float applyExpoReal(float x, float k) { return sgn(x) * pow(abs(x), k); }

// TODO: chck if gyro needs to be negated!!!!!!!!!!!!!!!!!!!!!!

class BalanceController  {
public:
	BalanceController(const Config* settings, const IMU& imu) :
		settings_(settings), imu_(imu), balance_pid_(&settings->balance_pid) {
		reset();
	}

	void reset() {
		balance_pid_.reset();
		max_D_multiplier_so_far_ = 0;
		prev_gyro_update_ = 0;
	}

	float getPInput() {
		return applyExpoReal(-imu_.angles[ANGLE_DRIVE] / settings_->balance_settings.balance_angle_scaling, settings_->balance_settings.balance_expo);
	}

	// Compute torque needed while board in normal mode.
	// Returns torque request based on current imu and gyro readings. Expected range is -MOTOR_CMD_RANGE:MOTOR_CMD_RANGE,
	// but not limited here to that range.
	int16_t compute(const int16_t* gyro_update) {
		int32_t avg_gyro_upd = (gyro_update[ANGLE_DRIVE] + prev_gyro_update_) / 2; // ESC refresh rate is 300 hz, controller is 1000hz. Avg last 2 readings to get closer to ctrl refresh rate
		prev_gyro_update_ = gyro_update[ANGLE_DRIVE];
		return balance_pid_.compute(getPInput(), avg_gyro_upd);
	}

	// Compute torque needed while board in starting up phase (coming from one side to balanced state).
	// Returns torque request based on current imu and gyro readings. Expected range is -MOTOR_CMD_RANGE:MOTOR_CMD_RANGE,
	// but not limited here to that range.
	int16_t computeStarting(const int16_t* gyro_update, float pid_P_multiplier) {
		float pid_D_multiplier =  START_D_MAX_MULTIPLIER *
				min((START_ANGLE_DRIVE - fabs(imu_.angles[ANGLE_DRIVE])) / (START_ANGLE_DRIVE - START_ANGLE_DRIVE_FULL), 1);

		// increase angle compensation as board approaches balance point. Never reduce it until reset
		if (pid_D_multiplier > max_D_multiplier_so_far_)
			max_D_multiplier_so_far_ = pid_D_multiplier;

		int16_t pid_out = balance_pid_.compute(getPInput() * pid_P_multiplier, gyro_update[ANGLE_DRIVE] * max_D_multiplier_so_far_);
		return constrain(pid_out, -START_MAX_POWER, START_MAX_POWER);
	}

private:
	const Config* settings_;
	const IMU& imu_;
	int16_t prev_gyro_update_ = 0;
	float max_D_multiplier_so_far_ = 0;
	PidController balance_pid_;
};

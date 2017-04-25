#pragma once
#include <math.h>

#include "global.h"
#include "pid.hpp"
#include "imu.hpp"

class BalanceController  {
public:
	BalanceController(const IMU& imu, const PidSettings& pid_settings) :
		imu_(imu), balance_pid_(pid_settings) {
		reset();
	}

	void reset() {
		balance_pid_.reset();
		max_D_multiplier_so_far_ = 0;
	}

	// Compute torque needed while board in normal mode.
	// Returns torque request based on current imu and gyro readings. Expected range is -MOTOR_CMD_RANGE:MOTOR_CMD_RANGE,
	// but not limited here to that range.
	int16_t compute(const int16_t* gyro_update) {
		return balance_pid_.compute( - imu_.angles[ANGLE_DRIVE], gyro_update[ANGLE_DRIVE]);
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

		int16_t pid_out = balance_pid_.compute(-imu_.angles[ANGLE_DRIVE] * pid_P_multiplier, gyro_update[ANGLE_DRIVE] * max_D_multiplier_so_far_);
		return constrain(pid_out, -START_MAX_POWER, START_MAX_POWER);
	}

private:
	const IMU& imu_;
	float max_D_multiplier_so_far_ = 0;
	PidController balance_pid_;
};

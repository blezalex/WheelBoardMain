#pragma once

#include "global.h"
#include "guard.hpp"
#include "imu/imu.hpp"

class AngleGuard : public Guard {
public:
	AngleGuard(const IMU& imu, const Config_BalancingConfig* balance_settings)
		: imu_(imu), balance_settings_(balance_settings) {
	}

	bool CanStart() {
		return abs(imu_.angles[ANGLE_STEER]) < balance_settings_->max_start_angle_steer && abs(imu_.angles[ANGLE_DRIVE]) < START_ANGLE_DRIVE;
	}

	bool MustStop() {
		return abs(imu_.angles[ANGLE_DRIVE]) > balance_settings_->shutoff_angle_drive  // back/forward tilt too high (accelerate/stop angle)
						|| abs(imu_.angles[ANGLE_STEER]) > balance_settings_->shutoff_angle_steer; // left/right tilt is too high
	}

private:
	const IMU& imu_;
	const Config_BalancingConfig* balance_settings_;

	DISALLOW_COPY_AND_ASSIGN(AngleGuard);
};

#pragma once

#include "global.h"
#include "guard.hpp"
#include "imu/imu.hpp"

class AngleGuard : public Guard {
public:
	AngleGuard(const IMU& imu)
		: imu_(imu) {
	}

	bool CanStart() {
		return abs(imu_.angles[ANGLE_STEER]) < START_ANGLE_STEER && abs(imu_.angles[ANGLE_DRIVE]) < START_ANGLE_DRIVE;
	}

	bool MustStop() {
		return abs(imu_.angles[ANGLE_DRIVE]) > STOP_ANGLE_DRIVE  // back/forward tilt too high (accelerate/stop angle)
						|| abs(imu_.angles[ANGLE_STEER]) > STOP_ANGLE_STEER; // left/right tilt is too high
	}

private:
	const IMU& imu_;

	DISALLOW_COPY_AND_ASSIGN(AngleGuard);
};

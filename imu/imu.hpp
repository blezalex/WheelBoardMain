
#ifndef IMU_H
#define IMU_H

#include "Arduino.h"
#include "global.h"
#include "drv/mpu6050/mpu.hpp"
#include "MadgwickAHRS.hpp"
#include "drv/comms/protocol.pb.h"

#define MADGWICK

class IMU {
public:
	IMU(const Config* config)
#ifndef MADGWICK
	: accCompensatedVector_{ 0, 0, ACC_1G }, config_(config) {
#else
		:mw_(&config_->balance_settings.imu_beta), config_(config) {
		mw_.begin(1000);
#endif
	}
	void compute(const MpuUpdate& update, bool init = false);

	volatile float angles[2];
private:

#ifdef MADGWICK
	Madgwick mw_;
#else
	float accCompensatedVector_[3];
#endif

	const Config* config_;
	DISALLOW_COPY_AND_ASSIGN(IMU);
};

#endif

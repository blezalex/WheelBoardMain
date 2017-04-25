
#ifndef IMU_H
#define IMU_H

#include "Arduino.h"
#include "global.h"
#include "mpu.hpp"

class IMU {
public:
	IMU()
	: accCompensatedVector_{ 0, 0, ACC_1G } {}
	void compute(const MpuUpdate& update);
	void updateGravityVector(const MpuUpdate& update);
	volatile float angles[2];
private:
	float accCompensatedVector_[3];

	DISALLOW_COPY_AND_ASSIGN(IMU);
};

#endif

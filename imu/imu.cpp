#include <math.h>
#include "imu.hpp"
#include "stm32f10x_gpio.h"


#define sq(v) ((v)*(v))
#define PI 3.141593

#define fcos(angle) (1 - sq(angle)/2)
#define fsin(angle) (angle)


#define GYRO_SCALE (4 / 65.5 * PI / 180.0 / 1000000.0)   //MPU6050 and MPU3050   16.4 LSB/(deg/s) and we ignore the last 2 bits

#define compFilterAccWeight 0.0004
#define compFilterAccWeightCalib 0.005
#define GRAVITY_TOLERANCE 0.15

#ifdef MADGWICK


// Madgwick takes about 220us to compute
void IMU::compute(const MpuUpdate& update, bool init) {
//	GPIOA->BSRR = GPIO_Pin_11;

	const float MW_GYRO_SCALE = (4 / 65.5);   //MPU6050 and MPU3050   65.5 LSB/(deg/s) and we ignore the last 2 bits
	if (init) {
		// While gyro is getting initialized its data is invalid - ignore gyro.
		// Take all data from ACC with much higher weight - assuming board is stationary (otherwise gyro would not calibrate)
		mw_.updateIMU(0, 0, 0, update.acc[0] / (float)ACC_1G, update.acc[1] / (float)ACC_1G, update.acc[2] / (float)ACC_1G, true);
	}
	else {
		mw_.updateIMU(update.gyro[0] * MW_GYRO_SCALE, update.gyro[1] * MW_GYRO_SCALE, update.gyro[2] * MW_GYRO_SCALE, update.acc[0] / (float)ACC_1G, update.acc[1] / (float)ACC_1G, update.acc[2] / (float)ACC_1G, false);
	}

	angles[0] = mw_.getRoll() + config_->callibration.x_offset; // TODO: replace with proper vector rotation in MpuUpdate (so pid controler sees rotated gyro input too)
	angles[1] = - mw_.getPitch() + config_->callibration.y_offset;

//	GPIOA->BRR = GPIO_Pin_11;
}

#else

static void RotateVector(float vector[], float roll, float pitch, float yaw)
{
    float cosR = fcos(roll);
    float sinR = fsin(roll);

    float cosP = fcos(pitch);
    float sinP = fsin(pitch);

    float cosY = fcos(yaw);
    float sinY = fsin(yaw);

    float tmpCopy[3];

    tmpCopy[0] = vector[0] * cosP * cosY    + vector[1] * (cosR * sinY + sinR * sinP * cosY) + vector[2] * (sinR * sinY - cosR * sinP * cosY);
    tmpCopy[1] = -vector[0] * cosP * sinY   + vector[1] * (cosR * cosY - sinR * sinP * sinY) + vector[2] * (sinR * cosY + cosR * sinP * sinY);
    tmpCopy[2] = vector[0] * sinP           - vector[1] * (sinR * cosP)                      + vector[2] * (cosR * cosP);

    for (int i = 0; i < 3; i++) {
        vector[i] = tmpCopy[i];
    }
}

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)& y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)& i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void IMU::compute(const MpuUpdate& update, bool init) {
	if (init) {
		// While gyro is getting initialized its data is invalid - ignore gyro.
		// Take all data from ACC with much higher weight - assuming board is stationary (otherwise gyro would not calibrate)
	  for (int i = 0; i < 3; i++) {
		  accCompensatedVector_[i] = (1 - compFilterAccWeightCalib) * accCompensatedVector_[i] + compFilterAccWeightCalib * update.acc[i];
	  }
	  return;
	}

  const float scale =  1000 * GYRO_SCALE; // TODO: use actual time
  RotateVector(accCompensatedVector_, update.gyro[0] * scale, update.gyro[1] * scale, update.gyro[2] * scale);

  int32_t sumAccSq = 0;
  for (int i = 0; i < 3; i++) {
		sumAccSq += sq((int32_t)update.acc[i]);
  }

  float currRatio = atan2(accCompensatedVector_[1], accCompensatedVector_[2]);
  float accRatio = atan2(update.acc[1], update.acc[2]);

  if (sumAccSq > sq(ACC_1G * (1 - GRAVITY_TOLERANCE)) && sumAccSq < sq(ACC_1G * (1 + GRAVITY_TOLERANCE)) && fabs(currRatio - accRatio) < 0.5) {
      for (int i = 0; i < 3; i++) {
          accCompensatedVector_[i] = (1 - compFilterAccWeight) * accCompensatedVector_[i] + compFilterAccWeight * update.acc[i];
      }                        
  }

	float inv_sqrt = invSqrt(sq(accCompensatedVector_[0]) + sq(accCompensatedVector_[1]) + sq(accCompensatedVector_[2])) * ACC_1G;
	accCompensatedVector_[0] *= inv_sqrt;
	accCompensatedVector_[1] *= inv_sqrt;
	accCompensatedVector_[2] *= inv_sqrt;

  angles[0] = atan2(accCompensatedVector_[1], accCompensatedVector_[2]) * 180 / PI + config_->callibration.x_offset;
  angles[1] = atan2(accCompensatedVector_[0], sqrt(sq(accCompensatedVector_[2]) + sq(accCompensatedVector_[1]))) * 180 / PI + config_->callibration.y_offset;
}

#endif

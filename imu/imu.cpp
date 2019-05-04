#include <math.h>
#include "imu.hpp"

#define sq(v) ((v)*(v))
#define PI 3.141593

#define fcos(angle) (1 - sq(angle)/2)
#define fsin(angle) (angle)


#define GYRO_SCALE (4 / 16.4 * PI / 180.0 / 1000000.0)   //MPU6050 and MPU3050   16.4 LSB/(deg/s) and we ignore the last 2 bits

#define compFilterAccWeight 0.00167
#define compFilterAccWeightCalib 0.01

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

// IMU::compute updates gravity vector with highly filtered ACC values. It takes a lot of time for ACC values to propagate on startup
// This method is using much higher update rate
void IMU::updateGravityVector(const MpuUpdate& update) {
  int16_t calibratedAccVector[3] = { update.acc[0], update.acc[1], update.acc[2] };
  for (int i = 0; i < 3; i++) {
	  accCompensatedVector_[i] = (1 - compFilterAccWeightCalib) * accCompensatedVector_[i] + compFilterAccWeightCalib * calibratedAccVector[i];
  }
}

void IMU::compute(const MpuUpdate& update) {
  float scale =  1000 * GYRO_SCALE; // TODO: use actual time
  RotateVector(accCompensatedVector_, update.gyro[0] * scale, update.gyro[1] * scale, update.gyro[2] * scale);

  int32_t sumAcc = 0;
  for (int i = 0; i < 3; i++) {
      sumAcc += sq((int32_t)update.acc[i]);
  }

  float currentGByAcc = sqrt(sumAcc) / ACC_1G;

  if (currentGByAcc > 0.85 && currentGByAcc < 1.15) {
      for (int i = 0; i < 3; i++) {
          accCompensatedVector_[i] = (1 - compFilterAccWeight) * accCompensatedVector_[i] + compFilterAccWeight * update.acc[i];
      }                        
  }

  angles[0] = atan2(accCompensatedVector_[1], accCompensatedVector_[2]) * 180 / PI;
  angles[1] = atan2(accCompensatedVector_[0], sqrt(sq(accCompensatedVector_[2]) + sq(accCompensatedVector_[1]))) * 180 / PI;
}

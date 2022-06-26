#ifndef IMU_UTILS_H_
#define IMU_UTILS_H_

#include <cstdint>
#include <math.h>

inline void ComputeRotationMatrix(float matrix[][3], float roll, float pitch, float yaw) {
  float cosR = cos(roll);
  float sinR = sin(roll);

  float cosP = cos(pitch);
  float sinP = sin(pitch);

  float cosY = cos(yaw);
  float sinY = sin(yaw);

  matrix[0][0] = cosP * cosY;
  matrix[0][1] = cosP * sinY;
  matrix[0][2] = -sinP;

  matrix[1][0] = sinR * sinP * cosY - cosR * sinY ;
  matrix[1][1] = cosR * cosY + sinR * sinP * sinY;
  matrix[1][2] = sinR * cosP;

  matrix[2][0] = sinR * sinY + cosR * sinP * cosY;
  matrix[2][1] = -sinR * cosY + cosR * sinP * sinY;
  matrix[2][2] = cosR * cosP;
}



inline void RotateVectorUsingMatrix(float vector[], float matrix[][3]) {
  float tmpCopy[3];

  tmpCopy[0] =  vector[0]  * matrix[0][0] + vector[1] * matrix[1][0] + vector[2] * matrix[2][0];
  tmpCopy[1] =  vector[0]  * matrix[0][1] + vector[1] * matrix[1][1] + vector[2] * matrix[2][1];
  tmpCopy[2] =  vector[0]  * matrix[0][2] + vector[1] * matrix[1][2] + vector[2] * matrix[2][2];

  for (int i = 0; i < 3; i++) {
    vector[i] = tmpCopy[i];
  }
}

inline void RotateVectorUsingMatrix(int16_t vector[], float matrix[][3]) {
  float tmpCopy[3];

  tmpCopy[0] =  vector[0]  * matrix[0][0] + vector[1] * matrix[1][0] + vector[2] * matrix[2][0];
  tmpCopy[1] =  vector[0]  * matrix[0][1] + vector[1] * matrix[1][1] + vector[2] * matrix[2][1];
  tmpCopy[2] =  vector[0]  * matrix[0][2] + vector[1] * matrix[1][2] + vector[2] * matrix[2][2];

  for (int i = 0; i < 3; i++) {
    vector[i] = tmpCopy[i];
  }
}

#endif /* IMU_UTILS_H_ */

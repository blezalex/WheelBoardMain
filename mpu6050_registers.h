#pragma once

#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_CONFIG 0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_SMPLRT_DIV       0x19
#define MPU6050_INT_PIN_CFG      0x37
#define MPU6050_INT_ENABLE       0x38
#define MPU6050_ACCEL_XOUT_H     0x3Bu

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03
#define MPU6050_CLOCK_PLL_EXT32K        0x04
#define MPU6050_CLOCK_PLL_EXT19M        0x05
#define MPU6050_CLOCK_KEEP_RESET        0x07

#define MPU6050_GYRO_FS_250         0x00<<3
#define MPU6050_GYRO_FS_500         0x01<<3
#define MPU6050_GYRO_FS_1000        0x02<<3
#define MPU6050_GYRO_FS_2000        0x03<<3

#define MPU6050_ACCEL_FS_2          0x00<<3
#define MPU6050_ACCEL_FS_4          0x01<<3
#define MPU6050_ACCEL_FS_8          0x02<<3
#define MPU6050_ACCEL_FS_16         0x03<<3

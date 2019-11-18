#pragma once

#define MPU6050_INT_Exti EXTI_Line13

#define MPU6050_LPF_256HZ 0
#define MPU6050_LPF_188HZ 1
#define MPU6050_LPF_98HZ 2
#define MPU6050_LPF_42HZ 3
#define MPU6050_LPF_20HZ 4
#define MPU6050_LPF_10HZ 5
#define MPU6050_LPF_5HZ  6

void mpuHandleDataReady();

struct MpuUpdate {
	int16_t gyro[3];
	int16_t acc[3];
};

class UpdateListener {
public:
	virtual void processUpdate(const MpuUpdate& update) = 0;
};


#define ACC_1G 2048

#define GYRO_CALIBRATION_ITERATIONS_REQUIRED 2048
#define GYRO_CALIBRATION_MAX_DIFF 16

class Mpu {
public:
	Mpu()
	: listener_(nullptr),
	  accZeroOffsets_{0,0,0},
	  gyroZeroOffsets_{0,0,0}
	  {}

	void init(uint8_t accLowPassFilterValue);
	inline void handleRawData(uint8_t* data);

	void setListener(UpdateListener* listener) {
		listener_ = listener;
	}

	inline bool calibrationComplete() {
		return gyroCalibrationIterationsLeft_ <= 0;
	}

	inline bool accCalibrationComplete() {
		return accCalibrationIterationsLeft_ <= 0;
	}

	void callibrateAcc();

	void applyAccZeroOffsets(int16_t* offsets_x_y_z) {
		for (int i = 0; i < 3; i++) {
			accZeroOffsets_[i] = offsets_x_y_z[i];
		}
	}

	const int16_t* getAccOffsets() {
		return accZeroOffsets_;
	}

private:
	inline void handleGyroData(int16_t* gyro, uint8_t* rawData);
	inline void handleAccData(int16_t* acc, int16_t* gyro,  uint8_t* rawData);

	UpdateListener* listener_;

	int16_t accZeroOffsets_[3];

	volatile int32_t gyroCalibrationIterationsLeft_;
	volatile int32_t accCalibrationIterationsLeft_;

	int32_t gyroCalibrationAccumulator_[3];
	int16_t gyroZeroOffsets_[3]; // calculated internally
};

extern Mpu accGyro;

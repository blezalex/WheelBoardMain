#include <stdlib.h>
#include <stdio.h>

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "misc.h"
#include "mpu.hpp"
#include "mpu6050_registers.hpp"
#include "io/i2c.hpp"
#include "global.h"


#define MPU6050_ADDRESS     0x68u

#define MPU6050_DMA_Channel DMA1_Channel5
#define MPU6050_DMA_ChannelIRQn DMA1_Channel5_IRQn
#define MPU6050_DMA_TC_FLAG DMA1_FLAG_TC5
#define MPU6050_I2C I2C2

#ifdef REV5
#define MPU6050_INT_ExtiPin GPIO_Pin_13;
#define MPU6050_INT_ExtiPort GPIOC
#define MPU6050_INT_ExtiPeriph RCC_APB2Periph_GPIOC
#define MPU6050_INT_ExtiPinSource GPIO_PortSourceGPIOC
#else
#define MPU6050_INT_ExtiPin GPIO_Pin_13;
#define MPU6050_INT_ExtiPort GPIOB
#define MPU6050_INT_ExtiPeriph RCC_APB2Periph_GPIOB
#define MPU6050_INT_ExtiPinSource GPIO_PortSourceGPIOB
#endif

unsigned char MPU6050_Rx_Buffer[6+2+6];

#ifndef BOARD_ROTATION_MACRO
#define BOARD_ROTATION_MACRO(XYZ)  {}
#endif

volatile bool data_processed = true;

void mpuHandleDataReady() {
	if (!data_processed)
		return;

	data_processed = false;
	i2c_DmaRead(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, MPU6050_DMA_Channel, 14);
}

void MPU_DataReadyIntInit() {
	RCC_APB2PeriphClockCmd(MPU6050_INT_ExtiPeriph, ENABLE);

	/* GPIO configuration */
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  MPU6050_INT_ExtiPin;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(MPU6050_INT_ExtiPort, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource13);

    EXTI_InitTypeDef EXTI_InitStructure;
    EXTI_InitStructure.EXTI_Line = EXTI_Line13;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn; //I2C2 connect to channel 5 of DMA1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void MPU_DmaInit() {
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;

	DMA_DeInit(MPU6050_DMA_Channel); //reset DMA1 channe1 to default values;

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&MPU6050_I2C->DR; //address of data reading register of I2C1
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)MPU6050_Rx_Buffer; //variable to store data
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //channel will be used for peripheral to memory transfer
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;	// (non circular)
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	//Location assigned to peripheral register will be source
	DMA_InitStructure.DMA_BufferSize = 14;	//number of data to be transfered
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; //automatic memory increment disable for peripheral
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	//automatic memory increment enable for memory
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//source peripheral data size = 8bit
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	//destination memory data size = 8bit
	DMA_Init(MPU6050_DMA_Channel, &DMA_InitStructure);
	DMA_ITConfig(MPU6050_DMA_Channel, DMA_IT_TC, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = MPU6050_DMA_ChannelIRQn; //I2C2 connect to channel 5 of DMA1
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

Mpu accGyro;

void Mpu::callibrateAcc() {
	accCalibrationIterationsLeft_ = GYRO_CALIBRATION_ITERATIONS_REQUIRED;
}

void Mpu::handleRawData(uint8_t* rawData) {
	MpuUpdate update;
	handleGyroData(update.gyro, rawData+8); // first 6 are ACC data, then 2 temperature and next 6 are gyro data
	handleAccData(update.acc, update.gyro, rawData);

	if (listener_ != nullptr) {
		listener_->processUpdate(update);
	}

	data_processed = true;
}

inline void applyZeroOffset(int16_t* data, int16_t* offsets) {
	for (int i = 0; i < 3; i++) {
		data[i] -= offsets[i];
	}
}

inline int16_t getAccVal(uint8_t* rawData, uint8_t axis) {
	return ((int16_t)((rawData[axis*2]<<8) | rawData[axis*2 + 1]))>>2;
}

void Mpu::handleAccData(int16_t* acc, int16_t* gyro, uint8_t* rawData) {
	acc[0] = -getAccVal(rawData, 1);
	acc[1] = -getAccVal(rawData, 0);
	acc[2] =  getAccVal(rawData, 2);
	BOARD_ROTATION_MACRO(acc);
	if (accCalibrationIterationsLeft_ > 0) {
		if (--accCalibrationIterationsLeft_ == 0) {
			accZeroOffsets_[0] = acc[0];
			accZeroOffsets_[1] = acc[1];
			accZeroOffsets_[2] = acc[2] - ACC_1G;
		}
		if (gyro[0] > GYRO_CALIBRATION_MAX_DIFF || gyro[1] > GYRO_CALIBRATION_MAX_DIFF || gyro[2] > GYRO_CALIBRATION_MAX_DIFF)
			accCalibrationIterationsLeft_ = GYRO_CALIBRATION_ITERATIONS_REQUIRED;
	}
	applyZeroOffset(acc, accZeroOffsets_);
}

inline int16_t getGyroVal(uint8_t* rawData, uint8_t axis) {
	return ((int16_t)((rawData[axis * 2] << 8) | rawData[axis * 2 + 1])) >> 2;  // range: +/- 8192; +/- 500 deg/sec
}

void Mpu::handleGyroData(int16_t* gyro,  uint8_t* rawData) {
	gyro[0] =  getGyroVal(rawData, 1);
	gyro[1] =  getGyroVal(rawData, 0);
	gyro[2] = -getGyroVal(rawData, 2);
	BOARD_ROTATION_MACRO(gyro);
	if (calibrationComplete()) {
		applyZeroOffset(gyro, gyroZeroOffsets_);
	}
	else { // run one calibration step
		if (gyroCalibrationIterationsLeft_ == GYRO_CALIBRATION_ITERATIONS_REQUIRED) { // init at first iteration
			for (int axis = 0; axis < 3; axis++) {
				gyroCalibrationAccumulator_[axis] = 0;
			}
		}

		bool allAxisHaveSmallDiff = true;
		const int calibrationIterationNumber = GYRO_CALIBRATION_ITERATIONS_REQUIRED - gyroCalibrationIterationsLeft_;
		for (int axis = 0; axis < 3; axis++) {
			gyroCalibrationAccumulator_[axis] += gyro[axis];
			const int axisGyroAvg = gyroCalibrationAccumulator_[axis] / (calibrationIterationNumber + 1);
			allAxisHaveSmallDiff = allAxisHaveSmallDiff && abs(axisGyroAvg - gyro[axis]) < GYRO_CALIBRATION_MAX_DIFF;
		}

		if (allAxisHaveSmallDiff) { // successful iteration
			gyroCalibrationIterationsLeft_--;
		}
		else { // device has moved during calibration, restart
			gyroCalibrationIterationsLeft_ = GYRO_CALIBRATION_ITERATIONS_REQUIRED;
		}

		if (gyroCalibrationIterationsLeft_ == 0) { // last iteration, save calibration values
			for (int axis = 0; axis < 3; axis++) {
				gyroZeroOffsets_[axis] = gyroCalibrationAccumulator_[axis] / GYRO_CALIBRATION_ITERATIONS_REQUIRED;
			}
		}
	}
}

extern "C" void DMA1_Channel5_IRQHandler(void)
{
	//GPIOA->BSRR = GPIO_Pin_11;
	if (DMA_GetFlagStatus(MPU6050_DMA_TC_FLAG)) // 172us
	{
		DMA_ClearFlag(MPU6050_DMA_TC_FLAG);

		I2C_DMACmd(MPU6050_I2C, DISABLE);
		/* Send I2C1 STOP Condition */
		I2C_GenerateSTOP(MPU6050_I2C, ENABLE);
		/* Disable DMA channel*/
		DMA_Cmd(MPU6050_DMA_Channel, DISABLE);

		accGyro.handleRawData(MPU6050_Rx_Buffer);
	}
	//GPIOA->BRR = GPIO_Pin_11;
}

void Mpu::init(uint8_t lpfSettings) {
	i2c_writeReg(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1, MPU6050_CLOCK_PLL_ZGYRO);
	i2c_writeReg(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV, 0);
	i2c_writeReg(MPU6050_ADDRESS, MPU6050_CONFIG, lpfSettings);
	i2c_writeReg(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG, MPU6050_GYRO_FS_500);
	i2c_writeReg(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS_4);
	i2c_writeReg(MPU6050_ADDRESS, MPU6050_INT_PIN_CFG, 1<<4);
	i2c_writeReg(MPU6050_ADDRESS, MPU6050_INT_ENABLE, 1);

	MPU_DataReadyIntInit();
	MPU_DmaInit();

	gyroCalibrationIterationsLeft_ = GYRO_CALIBRATION_ITERATIONS_REQUIRED;
}

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "misc.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "arduino.h"
#include "drv/mpu6050/mpu.hpp"
#include "io/usart.hpp"
#include "io/pwm_out.hpp"
#include "lpf.hpp"
#include "pid.hpp"
#include "io/genericOut.hpp"
#include "guards/angleGuard.hpp"
#include "guards/footpadGuard.hpp"
#include "global.h"
#include "io/i2c.hpp"
#include "imu/imu.hpp"
#include "stateTracker.hpp"
#include "balanceController.hpp"
#include "boardController.hpp"
#include "stm32f10x_adc.h"
#include "drv/esc_status/escStatusReader.hpp"


extern "C" void EXTI15_10_IRQHandler(void)
{
	if (EXTI_GetITStatus(MPU6050_INT_Exti))			//MPU6050_INT
	{
		EXTI_ClearITPendingBit(MPU6050_INT_Exti);
		mpuHandleDataReady();
	}
}


class InitWaiter : public UpdateListener {
public:
	InitWaiter(GenericOut* status_led, IMU* imu, Guard* angle_guard)
	 :status_led_(status_led), imu_(imu), angle_guard_(angle_guard) {
	}

	void processUpdate(const MpuUpdate& update) {
		if (!accGyro.calibrationComplete()) {
			imu_->updateGravityVector(update);
		}
		else {
			imu_->compute(update);
		}
		angle_guard_->Update();
	}

	void waitForAccGyroCalibration() {
		uint16_t last_check_time = 0;
		while(!accGyro.calibrationComplete() || angle_guard_->CanStart()) {
			if ((uint16_t)(millis() - last_check_time) > 200u) {
				last_check_time = millis();
				status_led_->toggle();
			}
		}
	}
private:
	GenericOut* status_led_;
	IMU* imu_;
	Guard* angle_guard_;
};



#define PID_SCALE_FACTOR 2.0

int main(void)
{
	//PidSettings balance_pid_settings(286*PID_SCALE_FACTOR, 0.825*PID_SCALE_FACTOR, 0.1*PID_SCALE_FACTOR, 4000);
	PidSettings balance_pid_settings(190, 0.55, 0.1, 4000);
	SystemInit();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	initArduino();
	Serial1.Init(USART1, 115200);
	Serial2.Init(USART2, 115200);

	PwmOut motor_out;
	motor_out.init(NEUTRAL_MOTOR_CMD);

	const char msg[] = "Hello world!\n";
	Serial1.Send((uint8_t*)msg, sizeof(msg));

	i2c_init();

	GenericOut status_led(RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_4, 1); // red
	status_led.init();

	status_led.setState(0);
	status_led.setState(1);

	status_led.toggle();
	status_led.toggle();

	// accGyro.applyAccZeroOffsets(0, 0, 0); // TODO: Implement ACC calibration function and save values to EEPROM
	accGyro.init(MPU6050_LPF_98HZ);

	IMU imu;
	AngleGuard angle_guard(imu);
	InitWaiter waiter(&status_led, &imu, &angle_guard); 	// wait for angle. Wait for pads too?
	accGyro.setListener(&waiter);
	waiter.waitForAccGyroCalibration();

	GenericOut green_led(RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_3, 1);
	green_led.init();

	GenericOut beeper(RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_12, 0);
	beeper.init();

	FootpadGuard foot_pad_guard;
	Guard* guards[] = { &angle_guard, &foot_pad_guard };
	int guards_count = sizeof(guards) / sizeof(Guard*);

	BoardController main_ctrl(imu, motor_out, status_led, beeper, balance_pid_settings, MOTOR_OUT_AVG_RC, guards, guards_count, green_led, -25, 0.05);
	accGyro.setListener(&main_ctrl);

	EscStatusReader esc_status_reader(&Serial2);

	uint16_t last_check_time = 0;
    while(1) { // background work
    	// read status update from esc
    	if (esc_status_reader.update()) {
    		main_ctrl.process_esc_update(esc_status_reader.esc_status);
    	}

    	// print debug info every 100ms
    	if ((uint16_t)(millis() - last_check_time) > 200u) {
			last_check_time = millis();

			char buff[50];
			int size = sprintf(buff, "%d\t%d\n", (int16_t)imu.angles[0], (int16_t)imu.angles[1]);
			Serial1.Send((uint8_t*)buff, size);
		}
    }
}

#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_exti.h"
#include "stm32f10x_flash.h"
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
#include "drv/settings/settings.hpp"


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

#include "drv/comms/communicator.hpp"

int main(void)
{
	SystemInit();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_CRC, ENABLE);

	PwmOut motor_out;
	motor_out.init(NEUTRAL_MOTOR_CMD);

	initArduino();
	Serial1.Init(USART1, 115200);
	Serial2.Init(USART2, 115200);

	Config cfg = Config_init_default;
	if (readSettingsFromFlash(&cfg)) {
		const char msg[] = "Config OK\n";
		Serial1.Send((uint8_t*)msg, sizeof(msg));
	}
	else {
		Config cfg = Config_init_default;
		const char msg[] = "Config DEFAULT\n";
	}

	i2c_init();

	GenericOut status_led(RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_4, 1); // red
	status_led.init();
	status_led.setState(0);

	//accGyro.applyAccZeroOffsets(0, 0, 0); // TODO: Implement ACC calibration function and save values to EEPROM
	accGyro.init(MPU6050_LPF_98HZ);

	Communicator comms(&Serial1);
    while(1) { // background work

    	uint8_t comms_msg = comms.update();
    	switch (comms_msg) {
    	case RequestId_READ_CONFIG:
    	{
    		uint8_t data[256];
    		int16_t data_len = saveProtoToBuffer(data, sizeof(data), Config_fields,  &cfg);
    		if (data_len != -1) {
    			comms.SendMsg(ReplyId_CONFIG, data, data_len);
    		}
    		else {
    			comms.SendMsg(ReplyId_GENERIC_FAIL);
    		}
    		break;
    	}

    	case RequestId_WRITE_CONFIG:
    	{
    		bool good = readSettingsFromBuffer(&cfg, comms.data(), comms.data_len());
    		if (good)
    			comms.SendMsg(ReplyId_GENERIC_OK);
    		else
    			comms.SendMsg(ReplyId_GENERIC_FAIL);
    		break;
    	}
    	case RequestId_GET_STATS:
    	{
    		Stats stats = Stats_init_default;
//    		stats.drive_angle = imu.angles[0];
//    		stats.stear_angle = imu.angles[1];

    		uint8_t data[256];
			int16_t data_len = saveProtoToBuffer(data, sizeof(data), Stats_fields,  &stats);
    		if (data_len != -1) {
    			comms.SendMsg(ReplyId_STATS, data, data_len);
    		}
    		else {
    			comms.SendMsg(ReplyId_GENERIC_FAIL);
    		}
    	}

    	case RequestId_SAVE_CONFIG:
    		saveSettingsToFlash(cfg);
    		comms.SendMsg(ReplyId_GENERIC_OK);
    		break;
    	}
    }

	IMU imu;
	AngleGuard angle_guard(imu);
	InitWaiter waiter(&status_led, &imu, &angle_guard); 	// wait for angle. Wait for pads too?
	accGyro.setListener(&waiter);
	waiter.waitForAccGyroCalibration();

	GenericOut green_led(RCC_APB2Periph_GPIOB, GPIOB, GPIO_Pin_3, true);
	green_led.init();

	GenericOut beeper(RCC_APB2Periph_GPIOA, GPIOA, GPIO_Pin_12, true);
	beeper.init();

	FootpadGuard foot_pad_guard;
	Guard* guards[] = { &angle_guard, &foot_pad_guard };
	int guards_count = sizeof(guards) / sizeof(Guard*);

	BoardController main_ctrl(imu, motor_out, status_led, beeper, &cfg.balance_pid, MOTOR_OUT_AVG_RC, guards, guards_count, green_led, -25, 0.05);
	accGyro.setListener(&main_ctrl);



}

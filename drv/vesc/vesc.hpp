#pragma once
#include "io/usart.hpp"
#include <string.h>

typedef struct {
	float v_in;
	float temp_mos_filtered;
	float temp_motor_filtered;
	float avg_motor_current;
	float avg_input_current;
	float rpm;
	float duty_now;
	float amp_hours;
	float amp_hours_charged;
	int32_t tachometer;
	int32_t tachometer_abs;

	float v_in_smoothed;
	float duty_smoothed;
	float erpm_smoothed;
} mc_values;

class VescComm {
public:
	VescComm(Usart* serial) : serial_(serial) {
		memset(&mc_values_, 0, sizeof(mc_values));
	}

	void requestStats();

	void setCurrent(float current);

	void setCurrentBrake(float current);

	int update();


	// Communication commands
	enum class COMM_PACKET_ID {
		COMM_FW_VERSION = 0,
		COMM_JUMP_TO_BOOTLOADER,
		COMM_ERASE_NEW_APP,
		COMM_WRITE_NEW_APP_DATA,
		COMM_GET_VALUES,
		COMM_SET_DUTY,
		COMM_SET_CURRENT,
		COMM_SET_CURRENT_BRAKE,
		COMM_SET_RPM,
		COMM_SET_POS,
		COMM_SET_HANDBRAKE,
		COMM_SET_DETECT,
		COMM_SET_SERVO_POS,
		COMM_SET_MCCONF,
		COMM_GET_MCCONF,
		COMM_GET_MCCONF_DEFAULT,
		COMM_SET_APPCONF,
		COMM_GET_APPCONF,
		COMM_GET_APPCONF_DEFAULT,
		COMM_SAMPLE_PRINT,
		COMM_TERMINAL_CMD,
		COMM_PRINT,
		COMM_ROTOR_POSITION,
		COMM_EXPERIMENT_SAMPLE,
		COMM_DETECT_MOTOR_PARAM,
		COMM_DETECT_MOTOR_R_L,
		COMM_DETECT_MOTOR_FLUX_LINKAGE,
		COMM_DETECT_ENCODER,
		COMM_DETECT_HALL_FOC,
		COMM_REBOOT,
		COMM_ALIVE,
		COMM_GET_DECODED_PPM,
		COMM_GET_DECODED_ADC,
		COMM_GET_DECODED_CHUK,
		COMM_FORWARD_CAN,
		COMM_SET_CHUCK_DATA,
		COMM_CUSTOM_APP_DATA,
		COMM_NRF_START_PAIRING,
		COMM_GPD_SET_FSW,
		COMM_GPD_BUFFER_NOTIFY,
		COMM_GPD_BUFFER_SIZE_LEFT,
		COMM_GPD_FILL_BUFFER,
		COMM_GPD_OUTPUT_SAMPLE,
		COMM_GPD_SET_MODE,
		COMM_GPD_FILL_BUFFER_INT8,
		COMM_GPD_FILL_BUFFER_INT16,
		COMM_GPD_SET_BUFFER_INT_SCALE,
		COMM_GET_VALUES_SETUP,
		COMM_SET_MCCONF_TEMP,
		COMM_SET_MCCONF_TEMP_SETUP,
		COMM_GET_VALUES_SELECTIVE,
		COMM_GET_VALUES_SETUP_SELECTIVE,
		COMM_EXT_NRF_PRESENT,
		COMM_EXT_NRF_ESB_SET_CH_ADDR,
		COMM_EXT_NRF_ESB_SEND_DATA,
		COMM_EXT_NRF_ESB_RX_DATA,
		COMM_EXT_NRF_SET_ENABLED,
		COMM_DETECT_MOTOR_FLUX_LINKAGE_OPENLOOP,
		COMM_DETECT_APPLY_ALL_FOC,
		COMM_JUMP_TO_BOOTLOADER_ALL_CAN,
		COMM_ERASE_NEW_APP_ALL_CAN,
		COMM_WRITE_NEW_APP_DATA_ALL_CAN,
		COMM_PING_CAN,
		COMM_APP_DISABLE_OUTPUT,
		COMM_TERMINAL_CMD_SYNC,
		COMM_GET_IMU_DATA,
		COMM_BM_CONNECT,
		COMM_BM_ERASE_FLASH_ALL,
		COMM_BM_WRITE_FLASH,
		COMM_BM_REBOOT,
		COMM_BM_DISCONNECT
	} COMM_PACKET_ID;


public:
	mc_values mc_values_;

private:
	void sendRequest(const uint8_t* payload, int len);

	int expected_msg_len() {
		return rx_data_[0] == 2 ? rx_data_[1] : ( rx_data_[1] << 8 | rx_data_[2]);
	}

	int actual_header_size() {
		return rx_data_[0];
	}

	Usart* serial_;

	// buffer_pos_ always points  to non-written yet memory. bufferpos = 1 means have only one byte with index 0
	int32_t buffer_pos_ = 0;

	uint8_t rx_data_[256];

	uint16_t last_uart_data_time_ = 0;
};

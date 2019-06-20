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
} mc_values;

class VescComm {
public:
	VescComm(Usart* serial) : serial_(serial) {
		memset(&mc_values_, 0, sizeof(mc_values));
	}

	void requestStats();

	int update();

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

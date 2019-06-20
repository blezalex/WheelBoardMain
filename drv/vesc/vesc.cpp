#include "vesc.hpp"
#include "crc.h"
#include <string.h>

// Communication commands
typedef enum {
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

void VescComm::sendRequest(const uint8_t* payload, int payload_len) {
	uint16_t crc_payload = crc16(payload, payload_len);

	if (payload_len <= 256) {
		uint8_t header[] = {0x2, (uint8_t)payload_len};
		serial_->Send(header, sizeof(header));
	}
	else {
		uint8_t header[] = {0x3, (uint8_t)(payload_len >> 8), (uint8_t)payload_len};
		serial_->Send(header, sizeof(header));
	}

	serial_->Send(payload, payload_len);
	uint8_t footer[] = {(uint8_t)(crc_payload >> 8), (uint8_t)crc_payload, 0x3};
	serial_->Send(footer, sizeof(footer));
}

void VescComm::requestStats() {
	uint8_t request[] = {COMM_GET_VALUES};
	sendRequest(request , sizeof(request));
}

#define kMsgTimeoutMs 100u
#define kHeaderSize 3
#define kFooterSize 3


static int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index) {
	int16_t res =	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

static uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index) {
	uint16_t res = 	((uint16_t) buffer[*index]) << 8 |
					((uint16_t) buffer[*index + 1]);
	*index += 2;
	return res;
}

static int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index) {
	int32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

static uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index) {
	uint32_t res =	((uint32_t) buffer[*index]) << 24 |
					((uint32_t) buffer[*index + 1]) << 16 |
					((uint32_t) buffer[*index + 2]) << 8 |
					((uint32_t) buffer[*index + 3]);
	*index += 4;
	return res;
}

static float buffer_get_float16(const uint8_t *buffer, float scale, int32_t *index) {
    return (float)buffer_get_int16(buffer, index) / scale;
}

static  float buffer_get_float32(const uint8_t *buffer, float scale, int32_t *index) {
    return (float)buffer_get_int32(buffer, index) / scale;
}


int VescComm::update() {
	if (!serial_->HasData())
		return 0;

	uint16_t time = millis();
	if ((uint16_t)(time - last_uart_data_time_) > kMsgTimeoutMs) {
		buffer_pos_ = 0;
	}

	last_uart_data_time_ = time;

	int32_t received_bytes = serial_->Read(rx_data_ + buffer_pos_, sizeof(rx_data_) - buffer_pos_);
	buffer_pos_+= received_bytes;
	if (buffer_pos_ > kHeaderSize) {
		if (buffer_pos_ >= sizeof(rx_data_)) {
			buffer_pos_ = 0; // too long/invalid
		}

		if (buffer_pos_ >= actual_header_size() + expected_msg_len() + kFooterSize) {
			uint16_t crc_payload = crc16(rx_data_ + actual_header_size(), expected_msg_len());

			int footer_start_idx = actual_header_size() + expected_msg_len();

			if (crc_payload >> 8 != rx_data_[footer_start_idx] || crc_payload & 0xFF != rx_data_[footer_start_idx + 1]) {
				buffer_pos_ = 0;
				return -1; // Bad CRC
			}

			int msg_id = rx_data_[actual_header_size()];
			int32_t idx = actual_header_size() + 1;

			if (msg_id == COMM_GET_VALUES) {
				mc_values_.temp_mos_filtered = buffer_get_float16(rx_data_, 1e1, &idx);
				mc_values_.temp_motor_filtered = buffer_get_float16(rx_data_, 1e1, &idx);
				mc_values_.avg_motor_current = buffer_get_float32(rx_data_, 1e2, &idx);
				mc_values_.avg_input_current = buffer_get_float32(rx_data_, 1e2, &idx);
				idx+=8; // i_current, q_curent
				mc_values_.duty_now = buffer_get_float16(rx_data_, 1e3, &idx);
				mc_values_.rpm = buffer_get_float32(rx_data_, 1e0, &idx);
				mc_values_.v_in = buffer_get_float16(rx_data_, 1e1, &idx);
				mc_values_.amp_hours = buffer_get_float32(rx_data_, 1e4, &idx);
				mc_values_.amp_hours_charged = buffer_get_float32(rx_data_, 1e4, &idx);
				idx+=8; // watt hours used, charged
				mc_values_.tachometer =  buffer_get_int32(rx_data_, &idx);
				mc_values_.tachometer_abs =  buffer_get_int32(rx_data_, &idx);
			}

			int bytes_to_move = buffer_pos_ - (footer_start_idx + kFooterSize);
			if (bytes_to_move > 0) {
				memmove(rx_data_, rx_data_ + footer_start_idx + kFooterSize, bytes_to_move);
			}
			buffer_pos_ = bytes_to_move;

			return msg_id;
		}
	}

		return 0;
}

#pragma once
#include "global.h"
#include "stm32f10x_crc.h"

#define kHeaderPrefixSize 2 // Id + 8 byte len
#define kSuffixSize 2 // CRC
#define kHeaderSize (kHeaderPrefixSize + kSuffixSize)
#define kMsgTimeoutMs 500u

class Communicator {
public:
	Communicator(Usart* comms_channel) : comms_(comms_channel) { }

	// data must always be padded to 4 bytes. Header is extended to 4
	void SendMsg(uint8_t msg_id, const uint8_t* data, uint8_t data_len) {
		uint8_t header[4];
		header[0] = msg_id;
		header[1] = data_len + kHeaderSize + 2;
		header[2] = 0;
		header[3] = 0;
		CRC_ResetDR();
		uint32_t crc32 = CRC_CalcCRC(*(uint32_t*)header);

		uint8_t aligned_blocks = data_len / 4;
		if (data_len > 0) {
			crc32 = CRC_CalcBlockCRC((uint32_t*)data,  aligned_blocks);
		}
		uint8_t last_block[4] = {0};
		int remaining_offset = aligned_blocks * 4;
		for (int i = 0; i < data_len - remaining_offset; i++) {
			last_block[i] = data[remaining_offset + i];
		}
		if (remaining_offset != data_len) {
			crc32 = CRC_CalcCRC(*(uint32_t*)last_block);
		}

		comms_->Send(header, 4);
		comms_->Send(data, data_len);
		comms_->Send((uint8_t*)&crc32, 2);
	}

	void SendMsg(uint8_t msg_id) {
		uint8_t header[4] = { 0 };
		header[0] = msg_id;
		header[1] = kHeaderSize;

		CRC_ResetDR();
		uint32_t crc32 = CRC_CalcCRC(*(uint32_t*)header);
		comms_->Send(header, 2);
		comms_->Send((uint8_t*)&crc32, 2);
	}

	// returns msg_id
	int update() {
		if (!comms_->HasData())
			return false;

		uint16_t time = millis();
		if ((uint16_t)(time - last_uart_data_time_) > kMsgTimeoutMs) {
			buffer_pos_ = 0;
		}
		last_uart_data_time_ = time;

    	uint8_t received_bytes = comms_->Read(rx_data + buffer_pos_, sizeof(rx_data) - buffer_pos_);
    	buffer_pos_+= received_bytes;
    	if (buffer_pos_ > kHeaderPrefixSize) {
    		if (buffer_pos_ >= 255)
    			buffer_pos_ = 0; // too long/invalid

    		if (buffer_pos_ >= expected_msg_len()) {
    		//	CRC_ResetDR();
    		//    uint32_t crc32 = CRC_CalcBlockCRC(&rx_data, sizeof(rx_data));
    			buffer_pos_ = 0;
    			return expected_msg_id();
    		}
    	}

    	return 0;
	}

	const uint8_t* data() {
		return rx_data + kHeaderPrefixSize;
	}

	const uint8_t data_len() {
		return expected_msg_len() - kHeaderSize;
	}

private:
	uint8_t buffer_pos_ = 0;

	uint8_t expected_msg_id() {
		return rx_data[0];
	}

	uint8_t expected_msg_len() {
		return rx_data[1];
	}

	uint8_t expected_msg_crc_lo() {
		return rx_data[expected_msg_len()];
	}

	uint8_t rx_data[255];

	uint16_t last_uart_data_time_ = 0;

	Usart* comms_;
};

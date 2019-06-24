#pragma once
#include "global.h"
#include "stm32f10x_crc.h"

#define kHeaderSize 4 // Id + 1 byte msg len, 2 bytes padding
#define kSuffixSize 2 // CRC
#define kMetadataSize (kHeaderSize + kSuffixSize)
#define kMsgTimeoutMs 1000u // 1 second

class Communicator {
public:
	Communicator(Usart* comms_channel) : comms_(comms_channel) { }

	void SendMsg(uint8_t msg_id, const uint8_t* data, uint32_t data_len) {
		uint8_t header[kHeaderSize];
		header[0] = msg_id;
		header[1] = data_len + kMetadataSize;
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

		comms_->Send(header, kHeaderSize);
		comms_->Send(data, data_len);
		comms_->Send((uint8_t*)&crc32, kSuffixSize);
	}

	void SendMsg(uint8_t msg_id) {
		uint8_t header[kHeaderSize] = { 0 };
		header[0] = msg_id;
		header[1] = kMetadataSize;

		CRC_ResetDR();
		uint32_t crc32 = CRC_CalcCRC(*(uint32_t*)header);
		comms_->Send(header, kHeaderSize);
		comms_->Send((uint8_t*)&crc32, kSuffixSize);
	}

	// returns msg_id
	int update() {
		if (!comms_->HasData())
			return false;

		uint16_t time = millis();
		if ((uint16_t)(time - last_uart_data_time_) > kMsgTimeoutMs) {
			buffer_pos_ = 0;
			move_message_ = false;
	//		comms_->Send("Timeout\n", 8);
		}
		last_uart_data_time_ = time;

		if (move_message_) {
			move_message_ = false;
			int bytes_to_move = buffer_pos_ - (int)expected_msg_len();

			if (bytes_to_move > 0) {
				memmove(rx_data, rx_data + expected_msg_len(), bytes_to_move);
				buffer_pos_ = bytes_to_move;
			}
			else {
				buffer_pos_ = 0;
			}
		}

    	int32_t received_bytes = comms_->Read(rx_data + buffer_pos_, sizeof(rx_data) - buffer_pos_);
    	buffer_pos_+= received_bytes;
    	if (buffer_pos_ > kHeaderSize) {
    		if (buffer_pos_ >= sizeof(rx_data))
    			buffer_pos_ = 0; // too long/invalid

    		if (buffer_pos_ >= expected_msg_len()) {
    			int32_t msg_len = expected_msg_len() - kSuffixSize;
    			CRC_ResetDR();
    			int32_t aligned_blocks = msg_len / 4;
    			uint32_t crc32;
				if (msg_len > 0) {
					crc32 = CRC_CalcBlockCRC((uint32_t*)rx_data,  aligned_blocks);
				}
				uint8_t last_block[4] = {0};
				int remaining_offset = aligned_blocks * 4;
				for (int i = 0; i < msg_len - remaining_offset; i++) {
					last_block[i] = rx_data[remaining_offset + i];
				}
				if (remaining_offset != msg_len) {
					crc32 = CRC_CalcCRC(*(uint32_t*)last_block);
				}

				uint8_t* crc_bytes = (uint8_t*)&crc32;
				int msg_id = expected_msg_id();
				if (crc_bytes[0] != expected_msg_crc_lo() || crc_bytes[1] != expected_msg_crc_hi())
				{
					msg_id = RequestId_MSG_NONE;
					SendMsg(ReplyId_CRC_MISMATCH);
				}

    			move_message_ = true;
    			return msg_id;
    		}
    	}

    	return 0;
	}

	const uint8_t* data() {
		return rx_data + kHeaderSize;
	}

	const uint8_t data_len() {
		return expected_msg_len() - kMetadataSize;
	}

private:
	// buffer_pos_ always points  to non-written yet memory. bufferpos = 1 means have only one byte with index 0
	int32_t buffer_pos_ = 0;

	uint8_t expected_msg_id() {
		return rx_data[0];
	}

	uint8_t expected_msg_len() {
		return rx_data[1];
	}

	uint8_t expected_msg_crc_lo() {
		return rx_data[expected_msg_len() - 2];
	}

	uint8_t expected_msg_crc_hi() {
		return rx_data[expected_msg_len() - 1];
	}

	uint8_t rx_data[255];

	uint16_t last_uart_data_time_ = 0;

	// previous update processed a message. buffer_pos_ still poits to its end or some bytes past it. move data if past.
	bool move_message_ = false;

	Usart* comms_;
};

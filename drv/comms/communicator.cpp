#include "communicator.hpp"

	void Communicator::SendMsg(uint8_t msg_id, const uint8_t* data, uint32_t data_len) {
		uint8_t header[kHeaderSize];
		const uint32_t total_len = data_len + kMetadataSize;
		header[0] = msg_id;
		header[1] = total_len;
		header[2] = total_len >> 8;
		header[3] = 0;
		CRC_ResetDR();
		uint32_t crc32 = CRC_CalcCRC(*(uint32_t*)header);

		uint32_t aligned_blocks = data_len / 4;
		if (data_len > 0) {
			crc32 = CRC_CalcBlockCRC((uint32_t*)data,  aligned_blocks);
		}
		uint8_t last_block[4] = {0};
		uint32_t remaining_offset = aligned_blocks * 4;
		for (uint32_t i = 0; i < data_len - remaining_offset; i++) {
			last_block[i] = data[remaining_offset + i];
		}
		if (remaining_offset != data_len) {
			crc32 = CRC_CalcCRC(*(uint32_t*)last_block);
		}

		comms_->Send(header, kHeaderSize);
    	comms_->SendWithWait(data, data_len);
		comms_->SendWithWait((uint8_t*)&crc32, kSuffixSize);
	}

	void Communicator::SendMsg(uint8_t msg_id) {
		uint8_t header[kHeaderSize] = { 0 };
		header[0] = msg_id;
		header[1] = kMetadataSize;

		CRC_ResetDR();
		uint32_t crc32 = CRC_CalcCRC(*(uint32_t*)header);
		comms_->Send(header, kHeaderSize);
		comms_->Send((uint8_t*)&crc32, kSuffixSize);
	}

  int Communicator::update() {
		if (!comms_->HasData())
			return false;

		uint16_t time = half_millis();
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
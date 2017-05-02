#pragma once
#include "global.h"
#include "escStatus.h"

class EscStatusReader {
public:
	EscStatusReader(Usart* escChannel) : escChannel_(escChannel) { }

	void update() {
    	uint8_t usart2rx_data[10];
    	uint8_t received_bytes = escChannel_->Read(usart2rx_data, sizeof(usart2rx_data));
    	if (received_bytes != 0) {
    		uint16_t time = millis();
			if ((uint16_t)(time - last_uart_data_time_) > STATUS_MESSAGE_MIN_GAP) {
			  tmp_message_idx_ = 0;
			}
			last_uart_data_time_ = time;

    		for (int i = 0; i < received_bytes; i++) {
    			tmp_message_raw_[tmp_message_idx_++] = usart2rx_data[i];
    			if (tmp_message_idx_ == sizeof(ESCMessage)) {
    				tmp_message_idx_ = 0;

    	  			if (31 ^ tmp_message_raw_[0] ^ tmp_message_raw_[1] ^ tmp_message_raw_[2] == tmp_message_raw_[3]) {
    	  				esc_status = tmp_message_;
    	  			}
    			}
    		}
    	}
	}

	ESCMessage esc_status{0, 0, 0, 0};

private:
	uint8_t tmp_message_idx_ = 0;
	ESCMessage tmp_message_{0, 0, 0, 0};
	uint8_t* tmp_message_raw_ = (uint8_t*)&tmp_message_;
	uint16_t last_uart_data_time_ = 0;

	Usart* escChannel_;
};

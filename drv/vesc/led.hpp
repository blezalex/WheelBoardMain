#pragma once

class LedPort {

	void setColor(uint32_t color) {
		for (int n = 0; n < 16; n++) {
			uint32_t send_color = color;
			for (int i = 0; i < 24; i++) {
				if (send_color & 1) {
					GPIOA->BSRR = GPIO_Pin_11;
					delay_cycles(5);
					GPIOA->BRR = GPIO_Pin_11;
					delay_cycles(1);
				}
				else {
					GPIOA->BSRR = GPIO_Pin_11;
					delay_cycles(1);
					GPIOA->BRR = GPIO_Pin_11;
					delay_cycles(5);
				}
				send_color = send_color >> 1;
			}
		}
	}

};

void delay_cycles(volatile long cycles) {
	while(cycles--);
}

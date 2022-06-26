#pragma once
#include <stdint.h>
#include <stm32f10x_gpio.h>
#include "global.h"

class GenericOut {
public:
	GenericOut(uint32_t RCC_APB2Periph, GPIO_TypeDef* port, uint16_t pin, bool inverted);
	void init(bool open_drain = false);
	bool getState();
	void setState(bool enabled);
	void toggle();

private:
	uint32_t RCC_APB2Periph_;
	GPIO_TypeDef* port_;
	uint16_t pin_;
	bool inverted_;

	DISALLOW_COPY_AND_ASSIGN(GenericOut);
};

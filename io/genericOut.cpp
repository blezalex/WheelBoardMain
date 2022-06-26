#include "genericOut.hpp"
#include "stm_lib/inc/stm32f10x_rcc.h"
#include "global.h"


GenericOut::GenericOut(uint32_t RCC_APB2Periph, GPIO_TypeDef* port, uint16_t pin, bool inverted)
	: RCC_APB2Periph_(RCC_APB2Periph), port_(port), pin_(pin), inverted_(inverted) {

}

void GenericOut::init(bool open_drain) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_, ENABLE);

	setState(0);

	/* GPIO configuration */
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  pin_;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = open_drain ? GPIO_Mode_Out_OD : GPIO_Mode_Out_PP;
    GPIO_Init(port_, &GPIO_InitStructure);
}

void GenericOut::setState(bool enabled) {
	if (enabled ^ inverted_) {
		port_->BSRR = pin_;
	}
	else {
		port_->BRR = pin_;
	}
}

void GenericOut::toggle() {
	setState(!getState());
}

bool GenericOut::getState() {
	return GPIO_ReadOutputDataBit(port_, pin_) ^ inverted_;
}

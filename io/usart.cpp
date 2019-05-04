#include "usart.hpp"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"

void init(uint16_t rx_pin, uint16_t tx_pin, uint32_t baud, USART_TypeDef* usart, uint8_t IRQ_Channel) {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	/* Configure USART Tx alternate function  */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

	GPIO_InitStructure.GPIO_Pin = tx_pin;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// RX
	GPIO_InitStructure.GPIO_Pin = rx_pin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = baud;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(usart, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(usart, ENABLE);


	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = IRQ_Channel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(usart, USART_IT_RXNE, ENABLE);
	USART_ITConfig(usart, USART_IT_ORE, ENABLE);
}

bool Usart::Init(USART_TypeDef * device, uint32_t baud) {
	device_  = device;
	rxEmpty = true;
	rxBufferReadIdx = 0;
	rxBufferWriteIdx = 0;

	txBufferReadIdx = 0;
	txBufferWriteIdx = 0;
	txStarted = false;

	if (device_ == USART1) {
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
		init(GPIO_Pin_10, GPIO_Pin_9, baud, USART1, USART1_IRQn);
		return true;
	}

	if (device_ == USART2) {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		init(GPIO_Pin_3, GPIO_Pin_2, baud, USART2, USART2_IRQn);
		return true;
	}

	return false;
}

int32_t Usart::Read(uint8_t* data, int32_t max_size) {
	if (rxEmpty)
		return 0;

	int bytes_available = (RX_BUFFER_SIZE + rxBufferWriteIdx - rxBufferReadIdx) % RX_BUFFER_SIZE;
	int bytes_to_copy = min(max_size, bytes_available);
	for (int i = 0; i < bytes_to_copy; i++) {
		data[i] = rxBuffer[rxBufferReadIdx++];
		if (rxBufferReadIdx >= RX_BUFFER_SIZE)
			rxBufferReadIdx = 0;
	}
	if (rxBufferReadIdx == rxBufferWriteIdx)
		rxEmpty = true;

	return bytes_to_copy;
}

bool Usart::HasData() {
	return !rxEmpty;
}

void Usart::SendCurrentByteFromBuffer() {
	 USART_SendData(device_, txBuffer[txBufferReadIdx++]);
	 if (txBufferReadIdx >= TX_BUFFER_SIZE) {
		 txBufferReadIdx = 0;
	 }
}

// TODO: this function is not thread safe, but works OK in practice :)

// TODO: Implement block TX, such that caller blocks is if there is no room in the buffer left and waits until all data is buffered.
void Usart::Send(const uint8_t* data, int32_t size) {
	// TODO: Check if buffer has enough room, if not block or error ?
	for (int i = 0; i < size; i++) {
		txBuffer[txBufferWriteIdx] = data[i];

		if (txBufferWriteIdx < TX_BUFFER_SIZE - 1)
			txBufferWriteIdx++;
		else
			txBufferWriteIdx = 0;
	}

	if (!txStarted) {
		txStarted = 1;

		SendCurrentByteFromBuffer();
		USART_ITConfig(device_, USART_IT_TXE, ENABLE);
	}
}

void Usart::handleIRQ(){
	if(USART_GetITStatus(device_, USART_IT_TXE) != RESET) {
	 if (txBufferReadIdx != txBufferWriteIdx) {
		 SendCurrentByteFromBuffer();
	 }
	 else {
		 USART_ITConfig(device_, USART_IT_TXE, DISABLE);
		 txStarted = 0;
	 }
	}
	if(USART_GetITStatus(device_, USART_IT_RXNE) != RESET) {
		rxBuffer[rxBufferWriteIdx++] = USART_ReceiveData(device_);
		rxEmpty = false;
		if (rxBufferWriteIdx >= RX_BUFFER_SIZE) {
			rxBufferWriteIdx = 0;
		}
	}
	if(USART_GetITStatus(device_, USART_IT_ORE) != RESET) {
		device_->SR;
		USART_ReceiveData(device_);
	}
}

extern "C" void USART1_IRQHandler(void) {
	Serial1.handleIRQ();
}

extern "C" void USART2_IRQHandler(void) {
	Serial2.handleIRQ();
}

Usart Serial1(USART1);
Usart Serial2(USART2);


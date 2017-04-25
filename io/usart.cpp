#include "usart.hpp"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "misc.h"

#define TX_BUFFER_SIZE 50
uint8_t USART1_txBuffer[TX_BUFFER_SIZE];
uint8_t USART1_txBufferReadIdx = 0;
uint8_t USART1_txBufferWriteIdx = 0;
uint8_t USART1_txStarted = 0;

void USART1_Init() {
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* Enable GPIO clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	/* Enable UART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Configure USART Tx alternate function  */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// RX
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(USART1, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);


	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 15;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


static void USART1_SendCurrentByteFromBuffer() {
	 USART_SendData(USART1, USART1_txBuffer[USART1_txBufferReadIdx++]);
	 if (USART1_txBufferReadIdx >= TX_BUFFER_SIZE) {
		 USART1_txBufferReadIdx = 0;
	 }
}

// TODO: this function is not thread safe, but works OK in practice :)
void USART1_Send(uint8_t* data, uint8_t size) {
	// TODO: Check if buffer has enough room, if not block or error ?
	for (int i = 0; i < size; i++) {
		USART1_txBuffer[USART1_txBufferWriteIdx] = data[i];

		if (USART1_txBufferWriteIdx < TX_BUFFER_SIZE - 1)
			USART1_txBufferWriteIdx++;
		else
			USART1_txBufferWriteIdx = 0;
	}

	if (!USART1_txStarted)
	{
		USART1_txStarted = 1;

		USART1_SendCurrentByteFromBuffer();
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	}
}

extern "C" void USART1_IRQHandler(void) {
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET) {
	 if (USART1_txBufferReadIdx != USART1_txBufferWriteIdx) {
		 USART1_SendCurrentByteFromBuffer();
	 }
	 else {
		 USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		 USART1_txStarted = 0;
	 }
	}
}

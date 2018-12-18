#pragma once
#include <stdint.h>
#include "global.h"

#define TX_BUFFER_SIZE 50
#define RX_BUFFER_SIZE 50

class Usart  {
public:
	Usart(USART_TypeDef * device) : device_(device), rxEmpty(true) {

	}
	bool Init(USART_TypeDef * device, uint32_t baud);
	void Send(const uint8_t* data, uint8_t size);
	void Send(const char* data, uint8_t size) { Send((const uint8_t*)data, size); }
	uint8_t Read(uint8_t* data, uint8_t max_size);

	bool HasData();
	void handleIRQ();

private:
	void SendCurrentByteFromBuffer();

private:
	USART_TypeDef * device_;

	uint8_t txBuffer[TX_BUFFER_SIZE];
	uint8_t txBufferReadIdx = 0;
	uint8_t txBufferWriteIdx = 0;
	uint8_t txStarted = 0;

	uint8_t rxBuffer[RX_BUFFER_SIZE];
	uint8_t rxBufferReadIdx = 0;
	uint8_t rxBufferWriteIdx = 0;
	uint8_t rxEmpty = 0;

	DISALLOW_COPY_AND_ASSIGN(Usart);
};

extern Usart Serial1;
extern Usart Serial2;

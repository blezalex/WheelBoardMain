#pragma once
#include <stdint.h>
#include "global.h"

#define TX_BUFFER_SIZE 256
#define RX_BUFFER_SIZE 256

class Usart  {
public:
	Usart(USART_TypeDef * device) : device_(device), rxEmpty(true) {

	}
	bool Init(USART_TypeDef * device, uint32_t baud);
	void Send(const uint8_t* data, int32_t size);
	void Send(const char* data, int32_t size) { Send((const uint8_t*)data, size); }
	int32_t Read(uint8_t* data, int32_t max_size);

	bool HasData();
	void handleIRQ();

private:
	void SendCurrentByteFromBuffer();

private:
	USART_TypeDef * device_;

	uint8_t txBuffer[TX_BUFFER_SIZE];
	int32_t txBufferReadIdx = 0;
	int32_t txBufferWriteIdx = 0;
	bool txStarted = 0;

	uint8_t rxBuffer[RX_BUFFER_SIZE];
	int32_t rxBufferReadIdx = 0;
	int32_t rxBufferWriteIdx = 0;
	bool rxEmpty = 0;

	DISALLOW_COPY_AND_ASSIGN(Usart);
};

extern Usart Serial1;
extern Usart Serial2;

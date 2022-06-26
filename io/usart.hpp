#pragma once
#include <stdint.h>
#include "../global.h"

#define TX_BUFFER_SIZE 512
#define RX_BUFFER_SIZE 256

class Usart  {
public:
	Usart(USART_TypeDef * device) : device_(device), rxEmpty(true) {

	}
	bool Init(USART_TypeDef * device, uint32_t baud);
	int32_t Send(const uint8_t* data, int32_t size);
	int32_t Send(const char* data, int32_t size) { return Send((const uint8_t*)data, size); }

	void SendWithWait(const uint8_t* data, int32_t size);

	int32_t TxBufferFreeCapacity();
	int32_t Read(uint8_t* data, int32_t max_size);

	bool HasData();
	void handleIRQ();

private:
	void SendCurrentByteFromBuffer();

private:
	USART_TypeDef * device_;

	uint8_t txBuffer[TX_BUFFER_SIZE];
	volatile int32_t txBufferReadIdx = 0;
	volatile int32_t txBufferWriteIdx = 0;
	volatile bool txStarted = 0;

	uint8_t rxBuffer[RX_BUFFER_SIZE];
	volatile int32_t rxBufferReadIdx = 0;
	volatile int32_t rxBufferWriteIdx = 0;
	volatile bool rxEmpty = 0;

	DISALLOW_COPY_AND_ASSIGN(Usart);
};

extern Usart Serial1;
extern Usart Serial2;

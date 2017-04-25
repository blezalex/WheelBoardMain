#pragma once
#include <stdint.h>

void USART1_Init();

void USART1_Send(uint8_t* data, uint8_t size);

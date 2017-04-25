#pragma once
#include "stm32f10x.h"

void i2c_init();

uint8_t i2c_readReg(uint8_t hwAddr, uint8_t rAddr);

void i2c_read_reg_to_buf(uint8_t hwAddr, uint8_t rAddr, uint8_t* buf, uint8_t size);

void i2c_writeReg(uint8_t hwAddr, uint8_t wAddr, uint8_t value);

void i2c_DmaRead(uint8_t hwAddr, uint8_t rAddr, DMA_Channel_TypeDef* DMAy_Channelx, uint16_t dataNumber);

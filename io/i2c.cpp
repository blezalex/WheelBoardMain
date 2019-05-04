#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_i2c.h"
#include "misc.h"
#include "arduino.h"


#define PinSCL GPIO_Pin_10
#define PinSDA GPIO_Pin_11

// TODO: try replacing all while waits with interrupts

void i2c_init() {
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);

	/* GPIO configuration, drive pins manually at first to get i2c unstuck */
	GPIO_WriteBit(GPIOB, PinSCL, BitAction::Bit_SET);
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  PinSCL;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  PinSDA;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    while (!GPIO_ReadInputDataBit(GPIOB, PinSDA)) {
    	GPIO_WriteBit(GPIOB, PinSCL, BitAction::Bit_RESET);
    	delay(4);
    	GPIO_WriteBit(GPIOB, PinSCL, BitAction::Bit_SET);
    	delay(4);
    }


	/* GPIO configuration for I2C */
    GPIO_InitStructure.GPIO_Pin =  PinSCL | PinSDA;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    delay(4);

    I2C_DeInit(I2C2);
    delay(4);
    I2C_Cmd(I2C2, ENABLE);
    delay(4);

	/* I2C configuration */
    I2C_InitTypeDef  I2C_InitStructure;
    I2C_StructInit(&I2C_InitStructure);

    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x4A;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 400000;

    I2C_Init(I2C2, &I2C_InitStructure);
    I2C_Cmd(I2C2, ENABLE);
}

void start() {
	I2C_GenerateSTART(I2C2, ENABLE);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
}

void sendAddr(uint8_t hwAddr, uint8_t tx) {
	if (tx) {
		I2C_Send7bitAddress(I2C2, hwAddr << 1, I2C_Direction_Transmitter);
		while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else {
		I2C_Send7bitAddress(I2C2, hwAddr << 1, I2C_Direction_Receiver);
		while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

void sendData(uint8_t data) {
	I2C_SendData(I2C2, data);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

void stop() {
	I2C_GenerateSTOP(I2C2, ENABLE);
	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));
}



uint8_t i2c_readReg(uint8_t hwAddr, uint8_t rAddr) {
    start();
    sendAddr(hwAddr, 1);
    sendData(rAddr);

	I2C_AcknowledgeConfig(I2C2, ENABLE);
    start();
    sendAddr(hwAddr, 0);

	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
	uint8_t data = I2C_ReceiveData(I2C2);

	I2C_AcknowledgeConfig(I2C2, DISABLE);

	stop();
	return data;
}

void i2c_read_reg_to_buf(uint8_t hwAddr, uint8_t rAddr, uint8_t* buf, uint8_t size) {
    start();

    sendAddr(hwAddr, 1);

	I2C_SendData(I2C2, rAddr);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    start();

    sendAddr(hwAddr, 0);

	I2C_AcknowledgeConfig(I2C2, ENABLE);

	for (int i = 0; i < size; i++) {
		while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
		buf[i] = I2C_ReceiveData(I2C2);

		if (i >= size - 2)
			I2C_AcknowledgeConfig(I2C2, DISABLE);
	}

	stop();
}

void i2c_writeReg(uint8_t hwAddr, uint8_t wAddr, uint8_t value) {
    start();
    sendAddr(hwAddr, 1);

	sendData(wAddr);
	sendData(value);

	stop();
}

void i2c_DmaRead(uint8_t hwAddr, uint8_t rAddr, DMA_Channel_TypeDef* DMAy_Channelx, uint16_t dataNumber) // 72 us
{
	// GPIOA->BSRR = GPIO_Pin_11;

	DMA_Cmd(DMAy_Channelx, DISABLE);
	/* Set current data number again to 14 for MPu6050, only possible after disabling the DMA channel */
	DMA_SetCurrDataCounter(DMAy_Channelx, dataNumber);
	DMA_Cmd(DMAy_Channelx, ENABLE);

//	/* While the bus is busy */
//	while(I2C_GetFlagStatus(I2C2, I2C_FLAG_BUSY));

	/* Enable DMA NACK automatic generation */
	I2C_DMALastTransferCmd(I2C2, ENABLE);					//Note this one, very important

    start();
    sendAddr(hwAddr, 1);
    sendData(rAddr);

    start();
    sendAddr(hwAddr, 0);

	/* Start DMA to receive data from I2C */
	I2C_DMACmd(I2C2, ENABLE);

	//GPIOA->BRR = GPIO_Pin_11;

	// When the data transmission is complete, it will automatically jump to DMA interrupt routine to finish the rest.
}

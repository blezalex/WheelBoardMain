#include "led.hpp"
#include <string.h>

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_tim.h"

#define BUS_SPEED 800000 // hz

#define TICKS_PER_BIT (SystemCoreClock / BUS_SPEED - 1)

#define BIT_ONE (TICKS_PER_BIT*0.8)
#define BIT_ZERO (TICKS_PER_BIT*0.2)
#define RESET_BITS
uint8_t pwm_out_buffer[LED_COUNT*24 + 50];

void led_init()  {
	memset(pwm_out_buffer, 0, sizeof(pwm_out_buffer));


	for (int i = 0; i < LED_COUNT; i++) {
		led_set_color(i, 0x00A000);
	}

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	TIM_TimeBaseInitTypeDef TimerBaseInit;
	TIM_TimeBaseStructInit(&TimerBaseInit);

	TimerBaseInit.TIM_Prescaler = 0;
	TimerBaseInit.TIM_Period = TICKS_PER_BIT;
	TimerBaseInit.TIM_CounterMode = TIM_CounterMode_Up;
	TimerBaseInit.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM3,&TimerBaseInit);

	TIM_OCInitTypeDef OC_Config;
	TIM_OCStructInit(&OC_Config);
	OC_Config.TIM_OCMode = TIM_OCMode_PWM1;
	OC_Config.TIM_Pulse = 0;
	OC_Config.TIM_OutputState = TIM_OutputState_Enable;
	OC_Config.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM3, &OC_Config);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);


	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel6);

	DMA_InitTypeDef DMA_InitStructure;
	DMA_StructInit(&DMA_InitStructure);

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&TIM3->CCR1;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)pwm_out_buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = sizeof(pwm_out_buffer);
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);

	DMA_Cmd(DMA1_Channel6, ENABLE);
	TIM_DMACmd(TIM3, TIM_DMA_CC1, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}


void led_set_color(int led_idx, uint32_t color) {
	int led_offset = (led_idx+1)*24; // we set the bits backwards, thus + 1
	for (int i = 0; i < 24; i++) {
		pwm_out_buffer[led_offset - i] = (color & 1) ? BIT_ONE : BIT_ZERO;
		color = color >> 1;
	}
}

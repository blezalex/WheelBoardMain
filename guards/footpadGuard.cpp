#include "footpadGuard.hpp"
#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_dma.h"

volatile uint16_t ADC1_Buffer[2] = { 0 };

void initADCs() {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1 ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	ADC_InitTypeDef ADC_InitStructure;
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 2;
	ADC_Init ( ADC1, &ADC_InitStructure);

	ADC_RegularChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_239Cycles5);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_239Cycles5);

	ADC_Cmd (ADC1, ENABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	DMA_DeInit(DMA1_Channel1);

	DMA_InitTypeDef DMA_InitStructure;
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC1_Buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 2;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	DMA_Cmd(DMA1_Channel1, ENABLE);

	ADC_DMACmd(ADC1, ENABLE);

	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void FootpadGuard::Update() {
	for (int i = 0; i < 2; i++) {
		padLevelFilter[0].compute(ADC1_Buffer[0]);
		padLevelFilter[1].compute(ADC1_Buffer[1]);
	}

	if (!seen_booth_off) {
		// check if both off now
		if (padLevelFilter[0].getVal() < settings_->min_level_to_continue && padLevelFilter[1].getVal() < settings_->min_level_to_continue) {
			seen_booth_off = true;
		}
	}
}

FootpadGuard::FootpadGuard(const Config_FootPadSettings* settings)
 : padLevelFilter {&settings_->filter_rc, &settings_->filter_rc}, settings_(settings) {
	initADCs();
}

bool FootpadGuard::CanStart() {
	if (!seen_booth_off) {
		// sensor failure or still pressed since start.
		return false;
	}
	return padLevelFilter[0].getVal() > settings_->min_level_to_start && padLevelFilter[1].getVal() > settings_->min_level_to_start;
}

// Stop if one of the footpads below the threshold for at least shutoff_delay_ms
bool FootpadGuard::MustStop() {
	bool stop_condition = padLevelFilter[0].getVal() < settings_->min_level_to_continue || padLevelFilter[1].getVal() < settings_->min_level_to_continue;

	if (!stop_condition) {
		stop_requested_ = false;
		return false;
	}

	if (!stop_requested_) {
		stop_requested_ = true;
		stop_request_timestamp_ = millis();
	}

	return (uint16_t)(millis() - stop_request_timestamp_) > settings_->shutoff_delay_ms;
}

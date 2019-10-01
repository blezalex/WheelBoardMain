#include "pwm_out.hpp"
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"

void PwmOut::init(uint16_t val) {
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_InitTypeDef  GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseInitTypeDef TimerBaseInit;
	TIM_TimeBaseStructInit(&TimerBaseInit);

	TimerBaseInit.TIM_Prescaler =  SystemCoreClock / 1000000 - 1; // 1us tick ;
	TimerBaseInit.TIM_Period = 2500; // 400hz
	TimerBaseInit.TIM_CounterMode = TIM_CounterMode_Up;
	TimerBaseInit.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1,&TimerBaseInit);
	TIM_Cmd(TIM1, ENABLE);

	TIM_OCInitTypeDef OC_Config;
	TIM_OCStructInit(&OC_Config);
	OC_Config.TIM_OCMode = TIM_OCMode_PWM1;
	OC_Config.TIM_Pulse = val;
	OC_Config.TIM_OutputState = TIM_OutputState_Enable;
	OC_Config.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(TIM1, &OC_Config);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}

void PwmOut::set(uint16_t val) {
	TIM1->CCR1 = val;
}

uint16_t PwmOut::get() {
	return TIM1->CCR1;
}

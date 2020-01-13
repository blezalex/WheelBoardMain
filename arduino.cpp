#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"

#define MILLIS_TIMER TIM2
#define MILLIS_TIMER_PERIPH RCC_APB1Periph_TIM2

void initArduino() {
	RCC_APB1PeriphClockCmd(MILLIS_TIMER_PERIPH, ENABLE);

	TIM_TimeBaseInitTypeDef TimerBaseInit;
	TIM_TimeBaseStructInit(&TimerBaseInit);

	TimerBaseInit.TIM_Prescaler =  SystemCoreClock / 2 / 1000 - 1; // 1ms tick ;
	TimerBaseInit.TIM_Period = 0xFFFF;
	TimerBaseInit.TIM_CounterMode = TIM_CounterMode_Up;
	TimerBaseInit.TIM_ClockDivision = TIM_CKD_DIV2;
	TIM_TimeBaseInit(MILLIS_TIMER,&TimerBaseInit);

	TIM_Cmd(MILLIS_TIMER, ENABLE);
}

uint16_t millis() {
	return TIM2->CNT;
}
void delay(uint16_t time) {
	uint16_t start_time = millis();

	while ((uint16_t)(millis() - start_time) < time);
}

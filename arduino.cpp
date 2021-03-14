#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "misc.h"


#define MILLIS_TIMER TIM2
#define MILLIS_TIMER_PERIPH RCC_APB1Periph_TIM2


static volatile uint32_t millis_time_;

void initArduino() {
	millis_time_ = 0;

	RCC_APB1PeriphClockCmd(MILLIS_TIMER_PERIPH, ENABLE);

	TIM_TimeBaseInitTypeDef TimerBaseInit;
	TIM_TimeBaseStructInit(&TimerBaseInit);

	TimerBaseInit.TIM_Prescaler =  SystemCoreClock / 2 / 1000 - 1; // 1ms tick ;

	//TimerBaseInit.TIM_Prescaler =  SystemCoreClock / 2 / 1000000 - 1; // 0.001ms tick ;

	TimerBaseInit.TIM_Period = 0xFFFF;
	TimerBaseInit.TIM_CounterMode = TIM_CounterMode_Up;
	TimerBaseInit.TIM_ClockDivision = TIM_CKD_DIV2;
	TIM_TimeBaseInit(MILLIS_TIMER,&TimerBaseInit);

	TIM_Cmd(MILLIS_TIMER, ENABLE);

  NVIC_InitTypeDef nvicStructure;
  nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
  nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
  nvicStructure.NVIC_IRQChannelSubPriority = 0;
  nvicStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvicStructure);

  TIM_ClearITPendingBit(MILLIS_TIMER, TIM_IT_Update);
	TIM_ITConfig(MILLIS_TIMER, TIM_IT_Update, ENABLE );
}


extern "C" void TIM2_IRQHandler() {
	if (TIM_GetITStatus(MILLIS_TIMER, TIM_IT_Update) != RESET) {
		millis_time_++;
		TIM_ClearITPendingBit(MILLIS_TIMER, TIM_IT_Update);
	}
}

uint16_t millis() {
	return TIM2->CNT;
}


// Returns uint32 version of millis time.
// Assuming TIM2_IRQHandler never gets preempted.
uint32_t millis32() {
	uint16_t enter_cnt;
	uint32_t high_bytes;
	do {
		enter_cnt = TIM2->CNT;
		bool it_pending = TIM_GetITStatus(MILLIS_TIMER, TIM_IT_Update) != RESET;
		// it_pending may be set to true only if this code is called from an interrupt of the same priority as TIM2_IRQHandler, otherwise this code would have been stopped
		// prior to calling it_pending.
		high_bytes = millis_time_ + it_pending;
	} while (enter_cnt > TIM2->CNT); // overflow occurred during the loop execution.
	return (high_bytes << 16) | enter_cnt;
}

void delay(uint16_t time) {
	uint16_t start_time = millis();

	while ((uint16_t)(millis() - start_time) < time);
}

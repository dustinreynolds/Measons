/*
 * timer.c
 *
 *  Created on: Jul 5, 2015
 *      Author: dustin
 *
 * Copyright (c) 2015, Dustin Reynolds
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of [project] nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "stm32l1xx.h"

void timer_TIM2_Configuration(void){
	TIM_TimeBaseInitTypeDef timerInitStructure;
	NVIC_InitTypeDef nvicStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	//Period between interrupts is (Period-1)/(32000000/(Prescaler-1))
	timerInitStructure.TIM_Prescaler = 31;
	timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	timerInitStructure.TIM_Period = 1000; //results in 10us on, 10us off
	timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2, &timerInitStructure);
	TIM_Cmd(TIM2, ENABLE);
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	nvicStructure.NVIC_IRQChannel = TIM2_IRQn;
	nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
	nvicStructure.NVIC_IRQChannelSubPriority = 1;
	nvicStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvicStructure);

}

volatile static TIM2_flag = 0;
void TIM2_IRQHandler()
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		TIM2_flag = 1;
	}
}

void __inline__ init_TIM2_Change_Period(uint16_t period){
	//By disabling events, we can modify the Period register safely.
	//Once updated, re-enabling UEV events clears the existing counters,
	//giving us a clean slate to start from.
	TIM2->CNT = 0;
	TIM2->CR1 |= TIM_CR1_UDIS; //Disable UEV events
	TIM2_flag = 0;
	TIM2->ARR = period;
	TIM2->CR1 &= ~TIM_CR1_UDIS; //Enable UEV events
}

void delayms(uint32_t msec){
	while (msec-- > 0){
		delayus(1000);
	}
}

void delayus(uint16_t usec){
	//TIM2->CR1 |= TIM_CR1_CEN;  //Enable TIM2
	if (usec <= 2){
		return;
	}else if (usec < 10){
		uint16_t counter = usec * 2 - 1;
		while(counter-- > 0)
			asm("nop");
		return;
	}
	init_TIM2_Change_Period(usec);
	while(TIM2_flag == 0){
		//__WFI();  //can cause debugger to think it has disconnected, explore http://nuttx.org/doku.php?id=wiki:howtos:jtag-debugging
	};
	//TIM2->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN)); //DISABLE TIM2
}

void startDelayus(uint16_t usec){
	init_TIM2_Change_Period(usec);
}

uint32_t checkDelayus(void){
	return TIM_GetCounter(TIM2);
}

void waitSpecificCount(uint16_t usec){
	while(TIM_GetCounter(TIM2) < usec){};
}
void waitStartedDelay(void){
	while(TIM2_flag == 0){
		//__WFI();  //can cause debugger to think it has disconnected, explore http://nuttx.org/doku.php?id=wiki:howtos:jtag-debugging
	};
}

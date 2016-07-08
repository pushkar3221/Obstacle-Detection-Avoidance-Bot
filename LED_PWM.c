#include "LED_PWM.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_rcc.h"


// initialize LEDs as timer-based (PWM) pins
void init_LED_PWM()
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* Clock for GPIOD */
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	/* Alternate functions for pins -- as timer-based (PWM) */
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource15, GPIO_AF_TIM4);

	/* Set pins */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	TIM_TimeBaseInitTypeDef TIM_BaseStruct;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); // enable clock
	TIM_BaseStruct.TIM_Prescaler = 84-1;
	TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_BaseStruct.TIM_Period = 19999; //Converting to 50hz frequency
  TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_BaseStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM4, &TIM_BaseStruct);
	TIM_Cmd(TIM4, ENABLE);
	
	TIM_OCInitTypeDef TIM_OCStruct;
	TIM_OCStruct.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCStruct.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCStruct.TIM_Pulse = 0; 
	TIM_OC1Init(TIM4, &TIM_OCStruct);
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OCStruct.TIM_Pulse = 0; 
	TIM_OC4Init(TIM4, &TIM_OCStruct);
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
}

// set dutycycle of LED i (i=0,1,2,3) to dutycycle (specified as percentage -- 0 to 100)
void set_LED_dutycycle(uint32_t i , uint32_t duty_cycle)
{
	uint32_t arr = TIM4->ARR;
	uint32_t ccr = ((float)(arr+1) * (duty_cycle / 100.0));
  if (ccr < 0) ccr = 0;
  if (ccr > arr) ccr = arr;
	if (i == 0) { TIM4->CCR1 = ccr; }
  if (i == 3) { TIM4->CCR4 = ccr; }
}

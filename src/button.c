/*
 * button.c
 *
 *  Created on: 13. 1. 2017
 *      Author: Laci
 */

#include "button.h"


void init_NVIC(void){


	//ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC || ADC_FLAG_OVR);
}
void init_button(void){


	 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC,ENABLE);
	 /* Enable clock for SYSCFG */
	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	 GPIO_InitTypeDef gpioInitStruc;
	 gpioInitStruc.GPIO_Mode= GPIO_Mode_IN;
	 gpioInitStruc.GPIO_PuPd = GPIO_PuPd_UP;
	 gpioInitStruc.GPIO_Pin = GPIO_Pin_13;
	 gpioInitStruc.GPIO_Speed=GPIO_Speed_40MHz;
	 GPIO_Init(GPIOC,&gpioInitStruc);


	 //NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	 	NVIC_InitTypeDef NVIC_InitStructure;
	 	EXTI_InitTypeDef EXTI_InitStruct;

	  /* Tell system that you will use PD0 for EXTI_Line0 */
	 	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);


	  /* PD0 is connected to EXTI_Line0 */
	 	 EXTI_InitStruct.EXTI_Line = EXTI_Line13;
	 /* Enable interrupt */
	 	 EXTI_InitStruct.EXTI_LineCmd = ENABLE;
      /* Interrupt mode */
	 	 EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	  /* Triggers on rising and falling edge */
	 	 EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	   /* Add to EXTI */
	 	  EXTI_Init(&EXTI_InitStruct);


	 		 	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
	 		 	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	 		 	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 		 	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	 		 	NVIC_Init(&NVIC_InitStructure);

}


void EXTI15_10_IRQHandler(void){
	uint8_t button;
	if (EXTI_GetITStatus(EXTI_Line13) != RESET){

	button= GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13);
	if(button==0){
		//TODO

	}
	EXTI_ClearITPendingBit(EXTI_Line13);
	}
}


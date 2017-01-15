/*
 * button.h
 *
 *  Created on: 13. 1. 2017
 *      Author: Laci
 */



#ifndef BUTTON_H_
#define BUTTON_H_

#include "stm32l1xx.h"
#include <stddef.h>
#include "bmp180.h"

/*
 * Function:  init_button 
 * --------------------
 *  Inicializacia tlacidla (GPIO periferia, externe prerusenie na tlacidlo)
 *    
 */
void init_button(void);

/*
 * Function:  init_button 
 * --------------------
 *  Interrupt handler pre tlacidlo
 *    
 */ 
 void EXTI15_10_IRQHandler(void);
#endif /* BUTTON_H_ */

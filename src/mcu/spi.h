/**
  ******************************************************************************
  * @file    firmware/src/mcu/spi.h
  * @author  Jozef Rodina
  * @version V1.0.0
  * @date    24-January-2012
  * @brief   SPI configuration file.
  ******************************************************************************
  *
  * <h2><center>&copy; COPYRIGHT 2012 Jozef Rodina</center></h2>
  ******************************************************************************
  */

#ifndef __SPI_H
#define __SPI_H

/*
 * Function:  initSPI2 
 * --------------------
 * Inicializacia SPI zbernice
 *    
 */
void initSPI2(void);

/*
 * Function:  readWriteSPI2 
 * --------------------
 *  Citanie a zapisovanie zo zbernice SPI
 *    
 */
unsigned char readWriteSPI2(unsigned char txData);

//Example of CS use

/*
 * Function:  initCS_Pin 
 * --------------------
 * Inicializacia chip select pinu
 *    
 */
void initCS_Pin(void);

void device_Select(void);

void device_Unselect(void);

void initCD_Pin(void);

void cd_set(void);

void cd_reset(void);

/*
 * Function:  res_set 
 * --------------------
 * Inicializacia restat pinu (PA9)
 *    
 */
void initRES_Pin(void);

void res_set(void);

void res_reset(void);
#endif

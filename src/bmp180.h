/*
 * bmp180.h
 *
 * Author: Ladislav Tar
 *
 */
#include "stm32l1xx.h"
#include "stm32l1xx_i2c.h"
#include "stm32l1xx_rcc.h"
#include<stdlib.h>
#include <stdio.h>
#include <math.h>

#ifndef BMP180_H_
#define BMP180_H_

/* I2C to use for communications with BMP180 */


	#define I2C_PORT1         I2C1
	#define I2C_SCL_PIN1      GPIO_Pin_8     // PB8
	#define I2C_SDA_PIN1      GPIO_Pin_9     // PB9
	#define I2C_GPIO_PORT     GPIOB
	#define I2C_CLOCK1        RCC_APB1Periph_I2C1

	#define I2C_PORT2         I2C2
	#define I2C_SCL_PIN2      GPIO_Pin_10    // PB10
	#define I2C_SDA_PIN2      GPIO_Pin_11    // PB11
	#define I2C_CLOCK2        RCC_APB1Periph_I2C2


/* BMP180 defines */
#define BMP180_ADDR                     0xEE // BMP180 address
/* BMP180 registers */
#define BMP180_PROM_START_ADDR          0xAA // E2PROM calibration data start register
#define BMP180_PROM_DATA_LEN            22   // E2PROM length
#define BMP180_CHIP_ID_REG              0xD0 // Chip ID
#define BMP180_VERSION_REG              0xD1 // Version
#define BMP180_CTRL_MEAS_REG            0xF4 // Measurements control (OSS[7.6], SCO[5], CTL[4.0]
#define BMP180_ADC_OUT_MSB_REG          0xF6 // ADC out MSB  [7:0]
#define BMP180_ADC_OUT_LSB_REG          0xF7 // ADC out LSB  [7:0]
#define BMP180_ADC_OUT_XLSB_REG         0xF8 // ADC out XLSB [7:3]
#define BMP180_SOFT_RESET_REG           0xE0 // Soft reset control
/* BMP180 control values */
#define BMP180_T_MEASURE                0x2E // temperature measurement
#define BMP180_P0_MEASURE               0x34 // pressure measurement (OSS=0, 4.5ms)
#define BMP180_P1_MEASURE               0x74 // pressure measurement (OSS=1, 7.5ms)
#define BMP180_P2_MEASURE               0xB4 // pressure measurement (OSS=2, 13.5ms)
#define BMP180_P3_MEASURE               0xF4 // pressure measurement (OSS=3, 25.5ms)
/* BMP180 Pressure calculation constants */
#define BMP180_PARAM_MG                 3038
#define BMP180_PARAM_MH                -7357
#define BMP180_PARAM_MI                 3791
#define PRINT_SIZE						20


/* Calibration parameters structure */
typedef struct {
	int16_t AC1;
	int16_t AC2;
	int16_t AC3;
	uint16_t AC4;
	uint16_t AC5;
	uint16_t AC6;
	int16_t B1;
	int16_t B2;
	int16_t MB;
	int16_t MC;
	int16_t MD;
	int32_t B5;
} BMP180_Calibration_TypeDef;


/* Calibration parameters from E2PROM of BMP180 */
BMP180_Calibration_TypeDef BMP180_Calibration1, BMP180_Calibration2;

uint32_t pressure1,pressure2,temperature1,temperature2;
float delta,altitude;

/*
 * Function:  BMP180_Init 
 * --------------------
 * Pouziva sa na inicializaciu BMP senzora(inicializacia GPIO periferie a I2C zbernice)
 *    
 */
uint8_t BMP180_Init(uint32_t SPI_Clock_Speed);

/*
 * Function:  BMP180_Reset 
 * --------------------
 *  Vykonanie softveroveho resetu
 *    
 */
void BMP180_Reset(I2C_TypeDef * I2C_PORT);

/*
 * Function:  BMP180_WriteReg 
 * --------------------
 *  Zapis na I2C zbernicu
 *    
 */
uint8_t BMP180_WriteReg(uint8_t reg, uint8_t value, I2C_TypeDef * I2C_PORT);

/*
 * Function:  BMP180_ReadReg 
 * --------------------
 *  Citanie z I2C zbernice
 *    
 */
uint8_t BMP180_ReadReg(uint8_t reg,I2C_TypeDef * I2C_PORT);

/*
 * Function:  BMP180_ReadCalibration 
 * --------------------
 *  Pouziva sa na kalibraciu raw udajov zo senzora
 *    
 */
void BMP180_ReadCalibration(I2C_TypeDef * I2C_PORT,BMP180_Calibration_TypeDef * BMP180_Calibration);

/*
 * Function:  BMP180_Read_UT 
 * --------------------
 *  Vycitavanie raw teploty
 *    
 */
uint16_t BMP180_Read_UT(I2C_TypeDef * I2C_PORT);

/*
 * Function:  BMP180_Read_PT 
 * --------------------
 *  Vycitavanie raw tlaku
 *    
 */
uint32_t BMP180_Read_PT(uint8_t oss, I2C_TypeDef * I2C_PORT);

/*
 * Function:  BMP180_Calc_RT 
 * --------------------
 *  Prepocet na skutocnu teplotu z raw dat
 *    
 */
int16_t BMP180_Calc_RT(uint16_t UT,BMP180_Calibration_TypeDef * BMP180_Calibration);

/*
 * Function:  BMP180_Calc_RP 
 * --------------------
 *  Prepocet na skutocny tlak z raw dat
 *    
 */
int32_t BMP180_Calc_RP(uint32_t UP, uint8_t oss,BMP180_Calibration_TypeDef * BMP180_Calibration);

/*
 * Function:  calculateAltitud 
 * --------------------
 *  Vypocet vyskoveho rozdielu z dvoch tlakov
 *    
 */
float calculateAltitude(int32_t presMove, int32_t presBase, float temp);

/*
 * Function:  float2String 
 * --------------------
 *  Prepocet float premennej na string (kvoli vypisovaniu na displej)
 *    
 */
void float2String(float number, char *res);

/*
 * Function:  readAveragePressure 
 * --------------------
 *  Vycitavanie tlaku a nasledne spriemerovanie desiatich vzoriek
 *    
 */
void readAveragePressure(uint8_t oss);

#endif /* BMP180_H_ */

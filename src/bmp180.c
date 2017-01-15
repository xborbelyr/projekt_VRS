
/*
 * bmp180.c
 *
 * Author: Ladislav Tar
 *
 */

#include "bmp180.h"


	// Init I2C
uint8_t BMP180_Init(uint32_t SPI_Clock_Speed) {
	GPIO_InitTypeDef PORT;
	I2C_InitTypeDef I2CInit;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
	PORT.GPIO_Pin = I2C_SCL_PIN1;
	PORT.GPIO_OType = GPIO_OType_OD;
	PORT.GPIO_PuPd = GPIO_PuPd_UP;//
	PORT.GPIO_Mode = GPIO_Mode_AF;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOB,&PORT);

	PORT.GPIO_Pin = I2C_SDA_PIN1;
	PORT.GPIO_OType = GPIO_OType_OD;
	PORT.GPIO_PuPd = GPIO_PuPd_UP;
	PORT.GPIO_Mode = GPIO_Mode_AF;
	PORT.GPIO_Speed = GPIO_Speed_40MHz;
	GPIO_Init(GPIOB,&PORT);

	RCC_APB1PeriphClockCmd(I2C_CLOCK1,ENABLE); // Enable I2C clock

	  //choosing peripherals for selected pins
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_I2C1);//
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_I2C1);//

	I2C_DeInit(I2C_PORT1); // I2C reset to initial state
	I2CInit.I2C_Mode = I2C_Mode_I2C; // I2C mode is I2C
	I2CInit.I2C_DutyCycle = I2C_DutyCycle_2; // I2C fast mode duty cycle (WTF is this?)
	I2CInit.I2C_OwnAddress1 = 1; // This device address (7-bit or 10-bit)
	I2CInit.I2C_Ack = I2C_Ack_Enable; // Acknowledgment enable
	I2CInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // choose 7-bit address for acknowledgment
	I2CInit.I2C_ClockSpeed = SPI_Clock_Speed;
	I2C_Cmd(I2C_PORT1,ENABLE); // Enable I2C
	I2C_Init(I2C_PORT1,&I2CInit); // Configure I2C

	while (I2C_GetFlagStatus(I2C_PORT1,I2C_FLAG_BUSY)); // Wait until I2C free



	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB,ENABLE);
		PORT.GPIO_Pin = I2C_SCL_PIN2;
		PORT.GPIO_OType = GPIO_OType_OD;
		PORT.GPIO_PuPd = GPIO_PuPd_UP;//
		PORT.GPIO_Mode = GPIO_Mode_AF;
		PORT.GPIO_Speed = GPIO_Speed_40MHz;
		GPIO_Init(GPIOB,&PORT);

		PORT.GPIO_Pin = I2C_SDA_PIN2;
		PORT.GPIO_OType = GPIO_OType_OD;
		PORT.GPIO_PuPd = GPIO_PuPd_UP;
		PORT.GPIO_Mode = GPIO_Mode_AF;
		PORT.GPIO_Speed = GPIO_Speed_40MHz;
		GPIO_Init(GPIOB,&PORT);

		RCC_APB1PeriphClockCmd(I2C_CLOCK2,ENABLE); // Enable I2C clock

		  //choosing peripherals for selected pins
		  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_I2C2);//
		  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_I2C2);//

		I2C_DeInit(I2C_PORT2); // I2C reset to initial state
		I2CInit.I2C_Mode = I2C_Mode_I2C; // I2C mode is I2C
		I2CInit.I2C_DutyCycle = I2C_DutyCycle_2; // I2C fast mode duty cycle (WTF is this?)
		I2CInit.I2C_OwnAddress1 = 1; // This device address (7-bit or 10-bit)
		I2CInit.I2C_Ack = I2C_Ack_Enable; // Acknowledgment enable
		I2CInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // choose 7-bit address for acknowledgment
		I2CInit.I2C_ClockSpeed = SPI_Clock_Speed;
		I2C_Cmd(I2C_PORT2,ENABLE); // Enable I2C
		I2C_Init(I2C_PORT2,&I2CInit); // Configure I2C

		while (I2C_GetFlagStatus(I2C_PORT2,I2C_FLAG_BUSY)); // Wait until I2C free

	return 0;
}

void BMP180_Reset(I2C_TypeDef * I2C_PORT) {
	BMP180_WriteReg(BMP180_SOFT_RESET_REG,0xb6,I2C_PORT); // Do software reset
}

uint8_t BMP180_WriteReg(uint8_t reg, uint8_t value, I2C_TypeDef * I2C_PORT) {
	I2C_GenerateSTART(I2C_PORT,ENABLE);
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C_PORT,reg); // Send register address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_SendData(I2C_PORT,value); // Send register value
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTOP(I2C_PORT,ENABLE);

	return value;
}

uint8_t BMP180_ReadReg(uint8_t reg,I2C_TypeDef * I2C_PORT) {

	uint8_t value;

	I2C_GenerateSTART(I2C_PORT,ENABLE);
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C_PORT,reg); // Send register address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	value = I2C_ReceiveData(I2C_PORT); // Receive ChipID
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)

	return value;
}

void BMP180_ReadCalibration(I2C_TypeDef * I2C_PORT, BMP180_Calibration_TypeDef * BMP180_Calibration) {
	uint8_t i;
	uint8_t buffer[BMP180_PROM_DATA_LEN];

	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C_PORT,BMP180_PROM_START_ADDR); // Send calibration first register address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	for (i = 0; i < BMP180_PROM_DATA_LEN-1; i++) {
		while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
		buffer[i] = I2C_ReceiveData(I2C_PORT); // Receive byte
	}
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	buffer[i] = I2C_ReceiveData(I2C_PORT); // Receive last byte

	BMP180_Calibration->AC1 = (buffer[0]  << 8) | buffer[1];
	BMP180_Calibration->AC2 = (buffer[2]  << 8) | buffer[3];
	BMP180_Calibration->AC3 = (buffer[4]  << 8) | buffer[5];
	BMP180_Calibration->AC4 = (buffer[6]  << 8) | buffer[7];
	BMP180_Calibration->AC5 = (buffer[8]  << 8) | buffer[9];
	BMP180_Calibration->AC6 = (buffer[10] << 8) | buffer[11];
	BMP180_Calibration->B1  = (buffer[12] << 8) | buffer[13];
	BMP180_Calibration->B2  = (buffer[14] << 8) | buffer[15];
	BMP180_Calibration->MB  = (buffer[16] << 8) | buffer[17];
	BMP180_Calibration->MC  = (buffer[18] << 8) | buffer[19];
	BMP180_Calibration->MD  = (buffer[20] << 8) | buffer[21];
}

uint32_t BMP180_Read_PT(uint8_t oss, I2C_TypeDef * I2C_PORT) {
	uint32_t PT;
	uint8_t cmd,d;

	switch(oss) {
	case 0:
		cmd = BMP180_P0_MEASURE;
		d  = 6;
		break;
	case 1:
		cmd = BMP180_P1_MEASURE;
		d   = 9;
		break;
	case 2:
		cmd = BMP180_P2_MEASURE;
		d  = 15;
		break;
	case 3:
		cmd = BMP180_P3_MEASURE;
		d   = 27;
		break;
	}

	BMP180_WriteReg(BMP180_CTRL_MEAS_REG,cmd,I2C_PORT);
	delay_us(100*d+400);

	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C_PORT,BMP180_ADC_OUT_MSB_REG); // Send ADC MSB register address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	PT = (uint32_t)I2C_ReceiveData(I2C_PORT) << 16; // Receive MSB
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	PT |= (uint32_t)I2C_ReceiveData(I2C_PORT) << 8; // Receive LSB
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	PT |= (uint32_t)I2C_ReceiveData(I2C_PORT); // Receive XLSB

	return PT >> (8 - oss);
}

uint16_t BMP180_Read_UT(I2C_TypeDef * I2C_PORT) {
	uint16_t UT;

	BMP180_WriteReg(BMP180_CTRL_MEAS_REG,BMP180_T_MEASURE,I2C_PORT);
	delay_us(100*10); // Wait for 4.5ms by datasheet


	I2C_AcknowledgeConfig(I2C_PORT,ENABLE); // Enable I2C acknowledge
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send START condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Transmitter); // Send slave address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); // Wait for EV6
	I2C_SendData(I2C_PORT,BMP180_ADC_OUT_MSB_REG); // Send ADC MSB register address
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // Wait for EV8
	I2C_GenerateSTART(I2C_PORT,ENABLE); // Send repeated START condition (aka Re-START)
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_MODE_SELECT)); // Wait for EV5
	I2C_Send7bitAddress(I2C_PORT,BMP180_ADDR,I2C_Direction_Receiver); // Send slave address for READ
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); // Wait for EV6
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	UT = (uint16_t)I2C_ReceiveData(I2C_PORT) << 8; // Receive MSB
	I2C_AcknowledgeConfig(I2C_PORT,DISABLE); // Disable I2C acknowledgment
	I2C_GenerateSTOP(I2C_PORT,ENABLE); // Send STOP condition
	while (!I2C_CheckEvent(I2C_PORT,I2C_EVENT_MASTER_BYTE_RECEIVED)); // Wait for EV7 (Byte received from slave)
	UT |= I2C_ReceiveData(I2C_PORT); // Receive LSB

	return UT;
}

int16_t BMP180_Calc_RT(uint16_t UT,BMP180_Calibration_TypeDef * BMP180_Calibration) {
	BMP180_Calibration->B5  = (((int32_t)UT - (int32_t)BMP180_Calibration->AC6) * (int32_t)BMP180_Calibration->AC5) >> 15;
	BMP180_Calibration->B5 += ((int32_t)BMP180_Calibration->MC << 11) / (BMP180_Calibration->B5 + BMP180_Calibration->MD);

	return (BMP180_Calibration->B5 + 8) >> 4;
}

int32_t BMP180_Calc_RP(uint32_t UP, uint8_t oss, BMP180_Calibration_TypeDef * BMP180_Calibration) {
	int32_t B3,B6,X3,p;
	uint32_t B4,B7;

	B6 = BMP180_Calibration->B5 - 4000;
	X3 = ((BMP180_Calibration->B2 * ((B6 * B6) >> 12)) >> 11) + ((BMP180_Calibration->AC2 * B6) >> 11);
	B3 = (((((int32_t)BMP180_Calibration->AC1) * 4 + X3) << oss) + 2) >> 2;
	X3 = (((BMP180_Calibration->AC3 * B6) >> 13) + ((BMP180_Calibration->B1 * ((B6 * B6) >> 12)) >> 16) + 2) >> 2;
	B4 = (BMP180_Calibration->AC4 * (uint32_t)(X3 + 32768)) >> 15;
	B7 = ((uint32_t)UP - B3) * (50000 >> oss);
	if (B7 < 0x80000000) p = (B7 << 1) / B4; else p = (B7 / B4) << 1;
	p += ((((p >> 8) * (p >> 8) * BMP180_PARAM_MG) >> 16) + ((BMP180_PARAM_MH * p) >> 16) + BMP180_PARAM_MI) >> 4;

	return p;
}

void readAveragePressure(uint8_t oss){
	uint32_t p1=0,p2=0,temp1=0,temp2=0,p_temp=0,t_temp=0;
	int numberOfMes=10;
	for(int i=0;i<numberOfMes;i++){
		 t_temp = BMP180_Read_UT(I2C_PORT1);
		 temp1 = BMP180_Calc_RT(t_temp,&BMP180_Calibration1);
		 p_temp = BMP180_Read_PT(oss,I2C_PORT1);
		 p1+= BMP180_Calc_RP(p_temp,oss,&BMP180_Calibration1);

		 t_temp = BMP180_Read_UT(I2C_PORT2);
		 temp2 = BMP180_Calc_RT(t_temp,&BMP180_Calibration2);
		 p_temp = BMP180_Read_PT(oss,I2C_PORT2);
		 p2+= BMP180_Calc_RP(p_temp,oss,&BMP180_Calibration2);
	}
	pressure1 = (float)p1/numberOfMes;
	pressure2 = (float)p2/numberOfMes;
	temperature1 = (float)temp1/numberOfMes;
	temperature2 = (float)temp2/numberOfMes;
}

float calculateAltitude(float presMove, float presBase, float temp){

	float alt, x1, x2;
	x1 = presBase-presMove;
	x2 = presMove+presBase;

	alt = 16000*(1+0.004*temp)*(x1/x2);
	return alt;
}

float calculateAltitude2(float presMove, float presBase, float temp){

	float alt, x1, x2;
	int barSuc = 18464;
	float koefRozVzduchu = 0.00366;

	x1 = presBase-presMove;
	x2 = presMove+presBase;

	alt = barSuc * log10(presBase/presMove)*(1+koefRozVzduchu*temp);
	return alt;
}

float calculateAltitude3(float presMove, float presBase, float temp){

	float alt, x1, x2;

	x1=presBase - presMove;
	x2=x1/3;
	alt=x2*0.25;

	return alt;
}

void float2string(float* number, char *res)
{
	char padding[2]={0};
	int full=(int)*number;
	int decimal= (int) ((*number-full)*100);
	sprintf(padding,"%d",abs(decimal));
	if(abs(decimal)<10)
		sprintf(padding,"0%d m  ",abs(decimal));
	else
		sprintf(padding,"%d m  ",abs(decimal));
	if (full==0 && decimal<0)
		sprintf(res,"-%d.%s",full,padding);
	else
		sprintf(res,"%d.%s",full,padding);
}

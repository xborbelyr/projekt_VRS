#include <bmp180.h>
#include <stddef.h>
#include "stm32l1xx.h"

#include "mcu/spi.h"
#include "ssd1306.h"
#include "ili9163.h"
#include "button.h"

#include <time.h>
#include <stdlib.h>

int main(void)
{
	char s_pres1[PRINT_SIZE],s_pres2[PRINT_SIZE],s_alt[PRINT_SIZE];

	init_button();

	BMP180_Init(400000);
	BMP180_ReadCalibration(I2C_PORT1,&BMP180_Calibration1);
	BMP180_ReadCalibration(I2C_PORT2,&BMP180_Calibration2);

	initSPI2();
	initCD_Pin();
	initCS_Pin();
	initRES_Pin();

	lcdInitialise(LCD_ORIENTATION0);

	lcdClearDisplay(decodeRgbValue(0, 0, 0));

	lcdPutS("PRESSURE 1", lcdTextX(2), lcdTextY(2), decodeRgbValue(0, 0, 0), decodeRgbValue(255,255,255));

	lcdPutS("PRESSURE 2", lcdTextX(2), lcdTextY(6), decodeRgbValue(0, 0, 0), decodeRgbValue(255, 255, 255));

	lcdPutS("HEIGHT", lcdTextX(2), lcdTextY(10), decodeRgbValue(0, 0, 0), decodeRgbValue(255, 255, 255));


	delta = 0;

    while(1)
    {
    	int oss=3;

    	readAveragePressure(oss);

    	altitude = calculateAltitude(pressure1, pressure2+delta, (float)temperature2/10);

		itoa(pressure1,s_pres1,10);
		strcat(s_pres1," Pa  ");
		lcdPutS(s_pres1,lcdTextX(5), lcdTextY(4), decodeRgbValue(255, 255, 255), decodeRgbValue(0, 0, 0));

		itoa(pressure2+delta,s_pres2,10);
		strcat(s_pres2," Pa  ");
		lcdPutS(s_pres2, lcdTextX(5), lcdTextY(8), decodeRgbValue(255, 255, 255), decodeRgbValue(0, 0, 0));

		format_3V(altitude, s_alt);
		lcdPutS(s_alt, lcdTextX(5), lcdTextY(12), decodeRgbValue(255, 255, 255), decodeRgbValue(0, 0, 0));

		delay_us(200000);

    }

	return 0;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/*
 * Minimal __assert_func used by the assert() macro
 * */
void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
  while(1)
  {}
}

/*
 * Minimal __assert() uses __assert__func()
 * */
void __assert(const char *file, int line, const char *failedexpr)
{
   __assert_func (file, line, NULL, failedexpr);
}

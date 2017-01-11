#include <bmp180.h>
#include <stddef.h>
#include "stm32l1xx.h"

#include "mcu/spi.h"
#include "ssd1306.h"
#include "ili9163.h"

#include <time.h>
#include <stdlib.h>

int main(void)
{
	uint32_t u_pres;
	int32_t rp;
	char s_pres[20];

	BMP180_Init(400000);
	BMP180_ReadCalibration();

	initSPI2();
	initCD_Pin();
	initCS_Pin();
	initRES_Pin();


	lcdInitialise(LCD_ORIENTATION0);

	lcdClearDisplay(decodeRgbValue(0, 0, 0));

	lcdPutS("PRESSURE 1", lcdTextX(2), lcdTextY(2), decodeRgbValue(0, 0, 0), decodeRgbValue(255,255,255));

	lcdPutS("PRESSURE 2", lcdTextX(2), lcdTextY(6), decodeRgbValue(0, 0, 0), decodeRgbValue(255, 255, 255));
	lcdPutS("88 888 Pa", lcdTextX(5), lcdTextY(8), decodeRgbValue(255, 255, 255), decodeRgbValue(0, 0, 0));
	lcdPutS("HEIGHT", lcdTextX(2), lcdTextY(10), decodeRgbValue(0, 0, 0), decodeRgbValue(255, 255, 255));
	lcdPutS("20 cm", lcdTextX(5), lcdTextY(12), decodeRgbValue(255, 255, 255), decodeRgbValue(0, 0, 0));

    while(1)
    {
		u_pres = BMP180_Read_PT(0);
		rp = BMP180_Calc_RP(u_pres,0); // press
		itoa(rp,s_pres,10);
		strcat(s_pres," Pa");
		lcdPutS(s_pres,lcdTextX(5), lcdTextY(4), decodeRgbValue(255, 255, 255), decodeRgbValue(0, 0, 0));
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

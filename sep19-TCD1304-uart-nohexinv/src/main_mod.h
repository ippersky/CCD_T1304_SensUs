#ifndef MAINH
#define MAINH

#include "stm32f4xx.h"  // Located in ../Libraries/CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h

/* Data definitions */
#define CCDSize 3694
#define RxDataSize 12

/* CCD master clock in Hz */
/* The values presented here are the prescalable frequencies between
   1-2MHz with the STM32F401RE running at 84 MHz. Other frequencies
   are achievable when prescaling APB1 differently. 1 MHz does not
   seem to work well. */
//#define CCD_fm 1000000
//#define CCD_fm 1400000
//#define CCD_fm 1500000
//define CCD_fm 1680000
//#define CCD_fm 1750000
#define CCD_fm 2000000

/*  Comply with the CCD's timing requirements:
	The delay is dependent on the CCD_fM. These values appear to work:
		For 1.4-2.0 MHz use: ICG_delay = 11 and SH_delay = 12
		For 1.0 MHz use: ICG_delay = 12 and SH_delay = 12   */
#define SH_delay 12
#define ICG_delay 11
#define fm_delay 3



/* UART definitions */
#define BAUDRATE 115200


#include "stm32f4xx_it.h"
#include "timer_conf.h"
#include "ADC_conf.h"
#include "UART_conf.h"

// LED
// void led_on(uint8_t led, uint16_t buffer);

// LCD
void lcd_init(void);   // initialize lcd
void lcd_send_cmd(char cmd);  // send command to the lcd
void lcd_send_data(char data);  // send data to the lcd
void lcd_send_string(char* str);  // send string to the lcd
void lcd_put_cur(int row, int col);  // put cursor at the entered position row (0 or 1), col (0-15);
void lcd_clear(void);


#endif

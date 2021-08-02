/*-
 * Copyright (c) 2017 Esben Rossel
 * All rights reserved.
 *
 * Author: Esben Rossel <esbenrossel@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */


#include "main.h"

void virtual_GND(void);
void flush_CCD(void);
void NVIC_conf(void);
void ledBTN_conf(void);
void led_on(uint8_t led);
void led_off(uint8_t led);

// LED pins
#define LED1_pin = GPIO_Pin_6; // GPIO_Pin_6 in GPIOB
#define LED2_pin = GPIO_Pin_7;
#define LED3_pin = GPIO_Pin_8;

// Buttons pins
#define BTNmove_pin = GPIO_Pin_6; // GPIO_Pin6 in GPIOA
#define BTNselect_pin = GPIO_Pin_7;
#define BTNreturn_pin = GPIO_Pin_8;

// LCD pins
#define LCD_D4 = GPIO_Pin_10;
#define LCD_D5 = GPIO_Pin_11;
#define LCD_D6 = GPIO_Pin_12;
#define LCD_D7 = GPIO_Pin_13;
#define LCD_E = GPIO_Pin_14;	// must have PWM?
#define LCD_RS = GPIO_Pin_15; 
// LCD V0 : enter value for contrast, no need of potentiometer
// How to control arduino LCD with STM32? Is there an existing library?


__IO uint32_t SH_period = 20;			// Shift gate period	
__IO uint32_t ICG_period = 500000;		// Integration Clear Gate period

__IO uint16_t aTxBuffer[CCDSize];		// CCD data sent to UART
__IO uint16_t avgBuffer[CCDSize] = {0};		// Averaged CCD data sent to UART
__IO uint8_t aRxBuffer[RxDataSize] = {0};	// Received commands by PC (12 bytes) : SH period, ICH period, number of average
__IO uint8_t nRxBuffer[RxDataSize] = {0};	// Difference between nRX and aRx? UART = 8 bits

// changes to make with new buffers
__IO uint16_t LED1avgBuffer[CCDSize];		// CCD data sent to UART
__IO uint16_t LED2avgBuffer[CCDSize];		// CCD data sent to UART
__IO uint16_t LED3avgBuffer[CCDSize];		// CCD data sent to UART
__IO uint16_t LED1Buffer1[CCDSize];		// CCD data sent to UART
__IO uint16_t LED2Buffer1[CCDSize];		// CCD data sent to UART
__IO uint16_t LED3Buffer1[CCDSize];		// CCD data sent to UART
__IO uint16_t LED1Buffer2[CCDSize];		// CCD data sent to UART
__IO uint16_t LED2Buffer2[CCDSize];		// CCD data sent to UART
__IO uint16_t LED3Buffer2[CCDSize];		// CCD data sent to UART

__IO uint8_t change_exposure_flag = 0;	// When we want to change SH and ICG, exposure_flag = 1;
__IO uint8_t data_flag = 0;		// value 0 to 4 
__IO uint8_t pulse_counter = 0;		// No idea what it means/does
__IO uint8_t CCD_flushed = 0;		// CCD_flushed = 0 when CCD is not completly flushed?
__IO uint8_t avg_exps = 0;		// Number of average of exposures
__IO uint8_t exps_left = 0;		// Number of exposures left to mesure
__IO uint8_t coll_mode = 0;		// collection mode : single vs continuous mode

/* LED control */
__IO uint8_t led = 0;		// with which led we are working : none (0), 1, 2, and 3.
__IO uint8_t blink_time = 25;	// integration time is 15 ms
__IO uint8_t blink_delay = 5;	// delay before next LED ON



/* TIM Configuration
	TIM2/5 are 32 bit and will serve the ICG and SH pulses that may require very long periods.
	TIM3/4 are 16 bit and will serve the fM (master clock) and ADC clock. 

	fM (TIM3)	PB0	(Ch3)
	SH (TIM2)	PA1	(Ch2)
	ICG (TIM5)	PA0	(Ch1)
	CCD-output	PC0	(ADC-in10) */

/* UART Configuration
	Tx on PA3
	Rx on PA2 */

/* Other GPIOs
	PA0, PB2 and PC2 are driven low
	PA5 (LED) is enabled (but driven low by default) */


int main(void)
{
	int i = 0;

	/* virtual_GND() enables GPIOA, GPIOB and GPIOC clocks */
	virtual_GND();		// To keep noise low
	NVIC_conf();		// Related to interrupts

	/* Setup CCD_fM (TIM3) and ADC-timer (TIM4) */
	get_Timer_clocks();	// Get frequences of clocks : MCU, AHBP1, AHBP2...
	TIM_CCD_fM_conf();	// Configure master TIM3 clock
	TIM_ADC_conf();		// Congifure ADC TIM4 clock

	/* Setup UART */
	USART2_conf();

	/* Setup ADC + ADC-DMA */	
	ADC1_conf();

	/* Setup ICG (TIM5) and SH (TIM2) */
	TIM_ICG_SH_conf();


	//flush_CCD();

	// We are continually getting data from CCD. Must look into other functions. 




	while(1)
	{
		if (change_exposure_flag == 1)		// When want to change SH, ICG periods. What about the number of average?
		{
			/* reset flag */
			change_exposure_flag = 0;
			
			flush_CCD();

			/* set new integration time */		// 32 bits to 8 bits, shifts because of little/big-endian
			ICG_period = nRxBuffer[6]<<24|nRxBuffer[7]<<16|nRxBuffer[8]<<8|nRxBuffer[9];
			SH_period = nRxBuffer[2]<<24|nRxBuffer[3]<<16|nRxBuffer[4]<<8|nRxBuffer[5];

			/*	Disable ICG (TIM5) and SH (TIM2) before reconfiguring*/
			TIM_Cmd(TIM2, DISABLE);
			TIM_Cmd(TIM5, DISABLE);

			/* 	Reconfigure TIM2 and TIM5 */
			TIM_ICG_SH_conf();
		}

		// switch (data_flag) for every led

		// first lecture with average for sampling
		// Setup, only once
		led_on(LED1_pin, LED1Buffer1); // switch (data_flag) is in led_on function
		led_on(LED2_pin, LED2Buffer1);
		led_on(LED3_pin, LED3Buffer1);

		// in loop
		// second lecture with average --> worth to increase time to increase precision?
		led_on(LED1_pin, LED1Buffer2); // switch (data_flag) is in led_on function
		led_on(LED2_pin, LED2Buffer2);
		led_on(LED3_pin, LED3Buffer2);

		/* 

		analysis(buffer1, buffer2)
		analysis : 
		- find peak in both buffer
		- calculate displacement difference
		- separated by 1 second 
		- calculate rate of displacement / speed / slope
		- what does it returns? rate? position of pixel with lowest intensity?

		*/

		LED1Buffer1 = LED1Buffer2;
		LED2Buffer1 = LED2Buffer2;
		LED3Buffer1 = LED3Buffer2;


		/*

		rate buffer[60*15] : calibration curve for 15 minutes
		Real thing = rate buffer[16*3]
		rate buffer for led1, led2 and led3

		*/


		/* 
		For calibration curve
		If saturated : no change in pixel position, y = const
		stop everything and outputs concentration and graph of rate of change. 
		IS the rate of change linear? I think so.
		How to deal with slow rate of change (slower than 1 sec)
		condition : rate == 0 for 30 seconds? 20 seconds? 
		if(rate == 0)


		OR

		Only need to look at slope rate for 2-3 minutes. 
		After having constant rate changes : outputs the results, which is the 
		concentration associated with the slope.

		*/




		/* GPIO(LED1, 1);
		GPIO(LED1, 0);
		delay(5);

		*/

		/* GPIO(LED2, 1);

		*/

		// GPIO(LED3, 1);
		switch (data_flag){		// data_flag = when press collect in GUI?
		case 1:
			/* reset flags */
			data_flag = 0;
            		if (coll_mode == 1)	// single mode, average number = 1?
				pulse_counter=6;

			/* Transmit data in aTxBuffer */
			UART2_Tx_DMA();
			break;		

		case 2:
			/* reset flags */
			data_flag = 0;

			/* This is the first integration of several so overwrite avgBuffer */
			for (i=0; i<CCDSize; i++)
				avgBuffer[i] = aTxBuffer[i];	
			break;

		case 3:
			/* reset flags */
			data_flag = 0;

			/* Add new to previous integrations.
			   This loop takes 3-4ms to complete. */		
			for (i=0; i<CCDSize; i++)
				avgBuffer[i] = avgBuffer[i] + aTxBuffer[i];		
			break;

		case 4:
			/* reset flags */
			data_flag = 0;

			/* Add new to previous integrations.
			   This loop takes 3-4ms to complete. */		
			for (i=0; i<CCDSize; i++)
				avgBuffer[i] = avgBuffer[i] + aTxBuffer[i];		

			/* Store average values in aTxBuffer */
			for (i=0; i<CCDSize; i++)
				aTxBuffer[i] = avgBuffer[i]/avg_exps;

            if (coll_mode == 1){
				exps_left = avg_exps;
				pulse_counter=6;	
			}			

			// conserve aTxBuffer for each leds for at least 3 different measure --> 9 buffers of 3964
			// Then calculate the slope / speed / rate of displacement of the curve.

			/* Transmit data in aTxBuffer */
			UART2_Tx_DMA();
			break;
		}
	}
}



/* 	To keep noise-level on ADC-in down, the following GPIO's are
	set as output, driven low and physically connected to GND:
		PC1 and PC4 which are physically close to PC0 (ADC-in)
		PB1 and PC5 which are physically close to PB0 - the most busy GPIO (fM) */
void virtual_GND(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

	/* 	Clock the GPIOs */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  

	/* PC2 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* PB1 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Setup LED (PA5) */ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/* Run this function prior to datacollection */
void flush_CCD()
{
	/* Set exposure very low */
	ICG_period = 15000;
	SH_period = 20;

	/*	Disable ICG (TIM5) and SH (TIM2) before reconfiguring*/
	TIM_Cmd(TIM2, DISABLE);
	TIM_Cmd(TIM5, DISABLE);

	/*	Reset flags and counters */
	CCD_flushed = 0;
	pulse_counter = 0;

	/* 	Reconfigure TIM2 and TIM5 */
	TIM_ICG_SH_conf();

	/*	Block until CCD is properly flushed */
	while(CCD_flushed == 0);
}


/* Configure interrupts */
void NVIC_conf(void)
{
	NVIC_InitTypeDef		NVIC_InitStructure;

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* ICG (TIM5) IRQ */
	/* The TIM5 update interrupts starts TIM4 and ADC */
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* ADC-DMA IRQ */
	/* DMA1 Transfer complete interrupt stops TIM4 and ADC */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* USART2-DMA-Rx IRQ */
	/* DMA1 Transfer complete interrupt checks incoming data */
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void ledBTN_conf(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	/* 	Clock the GPIOs */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);  
	// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);  

	/* LED1 to LED3 */
	GPIO_InitStructure.GPIO_Pin = LED1_pin;
    uint8_t led1 = GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LED2_pin;
    uint8_t led2 = GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LED3_pin;
    uint8_t led3 = GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* LCD D4 to D7, E and RS */
	GPIO_InitStructure.GPIO_Pin = LCD_D4;
    uint8_t lcdD4 = GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LCD_D5;
    uint8_t lcdD5 = GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LCD_D6;
    uint8_t lcdD6 = GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LCD_D7;
    uint8_t lcdD7 = GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LCD_E;
    uint8_t lcdE = GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LCD_RS;
    uint8_t lcdRS = GPIO_Init(GPIOB, &GPIO_InitStructure);

	// change mode for BTN : IN
	GPIO_InitStructure.GPIO_mode = GPIO_Mode_IN;
	
	/* BTN move, select and return */
	GPIO_InitStructure.GPIO_Pin = BTNmove_pin;
    uint8_t BTNmove = GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = BTNselect_pin;
    uint8_t BTNselect = GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = BTNreturn_pin;
    uint8_t BTNreturn = GPIO_Init(GPIOA, &GPIO_InitStructure);

	



}

void led_on(uint8_t led){
	GPIO_Write(led, 1);	// led on
	// switch (data_flag)
	switch (data_flag){		// data_flag = when press collect in GUI?
		case 1:
			/* reset flags */
			data_flag = 0;
            		if (coll_mode == 1)	// single mode, average number = 1?
				pulse_counter=6;

			/* Transmit data in aTxBuffer */
			UART2_Tx_DMA();
			break;		

		case 2:
			/* reset flags */
			data_flag = 0;

			/* This is the first integration of several so overwrite avgBuffer */
			for (i=0; i<CCDSize; i++)
				avgBuffer[i] = aTxBuffer[i];	
			break;

		case 3:
			/* reset flags */
			data_flag = 0;

			/* Add new to previous integrations.
			   This loop takes 3-4ms to complete. */		
			for (i=0; i<CCDSize; i++)
				avgBuffer[i] = avgBuffer[i] + aTxBuffer[i];		
			break;

		case 4:
			/* reset flags */
			data_flag = 0;

			/* Add new to previous integrations.
			   This loop takes 3-4ms to complete. */		
			for (i=0; i<CCDSize; i++)
				avgBuffer[i] = avgBuffer[i] + aTxBuffer[i];		

			/* Store average values in aTxBuffer */
			for (i=0; i<CCDSize; i++)
				aTxBuffer[i] = avgBuffer[i]/avg_exps;

            if (coll_mode == 1){
				exps_left = avg_exps;
				pulse_counter=6;	
			}			

			// conserve aTxBuffer for each leds for at least 3 different measure --> 9 buffers of 3964
			// Then calculate the slope / speed / rate of displacement of the curve.

			/* Transmit data in aTxBuffer */
			UART2_Tx_DMA();
			break;
		}
	GPIO_Write(led, 0);	// led off
	Delay(5);
}


}
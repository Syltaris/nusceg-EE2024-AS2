#include "stdio.h"

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_uart.h"

#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "led7seg.h"
#include "light.h"
#include "temp.h"

#include<lpc17xx.h>
#include "lcd.h"        //ExploreEmbedded LCD library which contains the lcd routines
#include "delay.h"      //ExploreEmbedded delay library which contains the delay routines
/* Bit positions of ADCR and ADGR registers */
#define SBIT_BURST      16u
#define SBIT_START      24u
#define SBIT_PDN        21u
#define SBIT_EDGE       27u
#define SBIT_DONE       31u
#define SBIT_RESULT     4u
#define SBIT_CLCKDIV    8u
#define SBIT_START		24u
#define SBIT_EDGE		27u

void main() {

	/*
	 * TS0, TS1 jumpers removed to pull pins HIGH
	 * 640 uS/Kelvin
	 */

	uint16_t adc_result;
	SystemInit();                              //Clock and PLL configuration

	/* Specify the LCD type(2x16) for initialization*/
	LCD_Init(2, 16);

	LPC_SC ->PCONP |= (1 << 12); /* Enable CLOCK for internal ADC controller */

	//Set the clock and Power ON ADC module
	//Start ad conversion when rising edge on P2.10
	LPC_ADC ->ADCR = ((1 << SBIT_PDN) | (10 << SBIT_CLCKDIV) | (2 << SBIT_START) | (0 << SBIT_EDGE));

	//Use AD0.7
	LPC_ADC ->ADCR |= (1 << 7);

	LPC_PINCON ->PINSEL1 |= 0x01 << 14; /* Select the P0_23 AD0[0] for ADC function */

	while (1) {

		LPC_ADC ->ADCR |= 0x01; /* Select Channel 0 by setting 0th bit of ADCR */

		util_BitSet(LPC_ADC ->ADCR, SBIT_START); /*Start ADC conversion*/

		while (util_GetBitStatus(LPC_ADC ->ADGDR, SBIT_DONE) == 0)
			; /* wait till conversion completes */

		adc_result = (LPC_ADC ->ADGDR >> SBIT_RESULT) & 0xfff; /*Read the 12bit adc result*/

		LCD_GoToLine(0); /* Go to First line of 2x16 LCD */
		LCD_Printf("Adc0: %4d", adc_result); /* Display 4-digit adc result */
	}
}

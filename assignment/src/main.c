/*****************************************************************************
 *   A demo example using several of the peripherals on the base board
 *
 *   Copyright(C) 2011, EE2024
 *   All rights reserved.
 *
 ******************************************************************************/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_timer.h"

#include "joystick.h"
#include "pca9532.h"
#include "acc.h"
#include "oled.h"
#include "rgb.h"
#include "led7seg.h"

static void init_i2c(void)
{
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_timer1(){

	// default value of PCLK = CCLK/4
	// CCLK is derived from PLL0 output signal / (CCLKSEL + 1) [CCLKCFG register]

    LPC_SC->PCONP |= (1<<2);					/* Power ON Timer1 */

    LPC_TIM1->MCR  = (1<<0) | (1<<1);     		/* Clear COUNT on MR0 match and Generate Interrupt */
    LPC_TIM1->PR   = 0;      					/* Update COUNT every (value + 1) of PCLK  */
    LPC_TIM1->MR0  = 40000000;                 	/* Value of COUNT that triggers interrupts */
    LPC_TIM1->TCR  = (1 << 0);                 	/* Start timer by setting the Counter Enable */

    NVIC_EnableIRQ(TIMER1_IRQn);				/* Enable Timer1 interrupt */

}

uint8_t set = 0;

volatile uint8_t led_mask = 0;

void TIMER1_IRQHandler(void)
{
	unsigned int isrMask;

    isrMask = LPC_TIM1->IR;
    LPC_TIM1->IR = isrMask;         /* Clear the Interrupt Bit by writing to the register */

    rgb_setLeds(led_mask & set); 	// bitwise and
    set = ~set; 					// bitwise not

}

int main (void) {

	init_i2c();
	pca9532_init();

	init_timer1();
	rgb_init();

	pca9532_setLeds(0x0000, 0xFFFF);

}




/*****************************************************************************
 *   GPIO Speaker & Tone Example
 *
 *   CK Tham, EE2024
 *
 ******************************************************************************/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_i2c.h"
#include "acc.h"
#include "stdio.h"

//i2c enabler
static void init_I2C2(void) {
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C2 peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C2 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

void init_interrupts() {

	// pio1_8, P0.3
	LPC_GPIOINT ->IO0IntClr |= 1 << 3;
	LPC_GPIOINT ->IO0IntEnR |= 1 << 3;

	// P2.5
	LPC_GPIOINT ->IO2IntClr |= 1 << 5;
	LPC_GPIOINT ->IO2IntEnR |= 1 << 5;

	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn); // Enable EINT3 interrupt
}

volatile int flag = 0;

void EINT3_IRQHandler(void) {
	// PIO1_8
	if ((LPC_GPIOINT ->IO0IntStatR >> 3) & 0x1) {
		printf("HELLO1\n");
	}

//	if ((LPC_GPIOINT ->IO2IntStatR >> 5) & 0x1) {
//		printf("HELLO2\n");
//	}

//	printf("HELLO3\n");

}

int main(void) {

	init_I2C2();
	init_interrupts();

	// if can't access, forced access i2c bus

	acc_setMode(2);
	acc_setRange(ACC_RANGE_2G);
	acc_config_mode_LEVEL();

	while (1) {
		printf("%d\n", flag);
	}
}

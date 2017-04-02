/*****************************************************************************
 *   GPIO Speaker & Tone Example
 *
 *   CK Tham, EE2024
 *
 ******************************************************************************/

#include "lpc17xx_pinsel.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_timer.h"
#include "lpc17xx_ssp.h"
#include "oled.h"

typedef struct menu {
	struct menu * first;
	struct menu * second;
	struct menu * prev;
	int num;

} menu;

static menu myMenu;
static menu first;
static menu second;

volatile int updated = 0;
volatile int x = 20, y = 20;
volatile int menu_counter = 1;

static void init_ssp(void) {
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_gpio(void) {

	// for SW2
	GPIO_SetDir(0, 1 << 15, 0); //input
	GPIO_SetDir(0, 1 << 16, 0); //input
	GPIO_SetDir(0, 1 << 17, 0); //input
	GPIO_SetDir(2, 1 << 3, 0); //input
	GPIO_SetDir(2, 1 << 4, 0); //input
}

static void init_interrupt(void) {
	LPC_GPIOINT ->IO0IntEnF |= 1 << 15;
	LPC_GPIOINT ->IO0IntEnF |= 1 << 16;
	LPC_GPIOINT ->IO0IntEnF |= 1 << 17;
	LPC_GPIOINT ->IO2IntEnF |= 1 << 3;
	LPC_GPIOINT ->IO2IntEnF |= 1 << 4;
}

void EINT3_IRQHandler(void) {
	// Determine whether GPIO Interrupt P2.10 has occurred
	if ((LPC_GPIOINT ->IO0IntStatF >> 15) & 0x1) {
//		y++;
		menu_counter = (menu_counter + 1) % 2;
		updated = 1;

		LPC_GPIOINT ->IO0IntClr = 1 << 15;
	}
	if ((LPC_GPIOINT ->IO0IntStatF >> 16) & 0x1) {
//		x++;
		LPC_GPIOINT ->IO0IntClr = 1 << 16;
	}
	if ((LPC_GPIOINT ->IO0IntStatF >> 17) & 0x1) {
		// centre button

		LPC_GPIOINT ->IO0IntClr = 1 << 17;
	}
	if ((LPC_GPIOINT ->IO2IntStatF >> 3) & 0x1) {
//		y--;

		menu_counter--;
		if (menu_counter < 0)
			menu_counter = 0;
		updated = 1;

		LPC_GPIOINT ->IO2IntClr = 1 << 3;
	}
	if ((LPC_GPIOINT ->IO2IntStatF >> 4) & 0x1) {
//		x--;
		LPC_GPIOINT ->IO2IntClr = 1 << 4;
	}
}

static void draw_menu(menu * current_menu) {
	char temp[10];
	sprintf(temp, "%d", current_menu->first->num);
	oled_putString(10, 0, temp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	sprintf(temp, "%d", current_menu->second->num);
	oled_putString(10, 10, temp, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

static void draw_cursor(void) {
	oled_putString(0, menu_counter * 10, "->", OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
}

static void init_tree(void) {
	myMenu.first = &first;
	myMenu.second = &second;
	myMenu.prev = NULL;
	myMenu.num = 0;

	first.first = NULL;
	first.second = NULL;
	first.prev = &myMenu;
	first.num = 1;

	second.first = NULL;
	second.second = NULL;
	second.prev = &myMenu;
	second.num = 2;

}

int main(void) {

	init_ssp();
	init_gpio();
	oled_init();
	oled_clearScreen(OLED_COLOR_BLACK);

	init_interrupt();
	NVIC_EnableIRQ(EINT3_IRQn);

	init_tree();

	draw_cursor();
	draw_menu(&myMenu);

	while (1) {
		if (updated) {
			oled_clearScreen(OLED_COLOR_BLACK);
			draw_cursor();
			draw_menu(&myMenu);
			updated = 0;
		}
	}

}


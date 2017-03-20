#include "stdio.h"

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
#include "light.h"
#include "temp.h"



/* LED params */
static uint8_t set = 0;
static uint8_t ledRed_mask = RGB_RED;
static uint32_t led_set = 0x0001;

static volatile int RED_LED_FLAG = 0;
static volatile int LED_ARRAY_FLAG = 0;
static int LED_MOV_DIR = 0; // 0 for <<, 1 for >>

/* timer params */
volatile uint32_t msTicks = 0; // counter for 1ms SysTicks


/* 7-segment display params */
volatile int SSEG_FLAG = 0;
unsigned int sseg_count = 0;
int monitor_symbols[]  = {'0', '1', '2', '3', '4', '5', '6' ,'7' ,'8' ,'9' ,'A', 'B', 'C', 'D', 'E', 'F'};



/* ISL290003 light sensor params */
uint32_t light_reading = 0;


/* MMA7455 accelerometer sensor params */
volatile int8_t accX, accY, accZ;
int8_t accInitX, accInitY, accInitZ;

/* temperature sensor */
int32_t temperature_reading = 0;


/* OLED params */
volatile int SET_MONITOR_OLED_FLAG = 1;



/* stable, monitor mode flag */
volatile int MODE_FLAG = 0; //1 - monitor, 0 - passive

/* flag to sample light and accelerometer sensors */
volatile int SAMPLE_SENSORS_FLAG = 0;
char tempStr[80];



static void init_I2C2(void)
{
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

//ssp enabler
static void init_SSP(void)
{
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

//timer1 w/ 2s period
static void init_timer1(){

	// default value of PCLK = CCLK/4
	// CCLK is derived from PLL0 output signal / (CCLKSEL + 1) [CCLKCFG register]

    LPC_SC->PCONP |= (1<<2);					/* Power ON Timer1 */

    LPC_TIM1->MCR  = (1<<0) | (1<<1);     		/* Clear COUNT on MR0 match and Generate Interrupt */
    LPC_TIM1->PR   = 0;      					/* Update COUNT every (value + 1) of PCLK  */
    LPC_TIM1->MR0  = 20000000;                 	/* Value of COUNT that triggers interrupts */
    LPC_TIM1->TCR  = (1 << 0);                 	/* Start timer by setting the Counter Enable */

    NVIC_EnableIRQ(TIMER1_IRQn);				/* Enable Timer1 interrupt */

}

//timer2 w/ 4s period
static void init_timer2(){

	// default value of PCLK = CCLK/4
	// CCLK is derived from PLL0 output signal / (CCLKSEL + 1) [CCLKCFG register]

    LPC_SC->PCONP |= (1<<22);					/* Power ON Timer2 */

    LPC_TIM2->MCR  = (1<<0) | (1<<1);     		/* Clear COUNT on MR0 match and Generate Interrupt */
    LPC_TIM2->PR   = 0;      					/* Update COUNT every (value + 1) of PCLK  */
    LPC_TIM2->MR0  = 26666666;                 	/* Value of COUNT that triggers interrupts */
    LPC_TIM2->TCR  = (1 << 0);                 	/* Start timer by setting the Counter Enable */

    NVIC_EnableIRQ(TIMER2_IRQn);				/* Enable Timer2 interrupt */

}

void TIMER1_IRQHandler(void)
{
	unsigned int isrMask;

    isrMask = LPC_TIM1->IR;
    LPC_TIM1->IR = isrMask;         /* Clear the Interrupt Bit by writing to the register */					// bitwise not

    RED_LED_FLAG = 1;
    LED_ARRAY_FLAG = 1;
}

void TIMER2_IRQHandler(void)
{
	unsigned int isrMask;

    isrMask = LPC_TIM2->IR;
    LPC_TIM2->IR = isrMask;         /* Clear the Interrupt Bit by writing to the register */					// bitwise not

    SSEG_FLAG = 1;

    if(sseg_count == 5 || sseg_count == 10 || sseg_count == 15) {
    	SAMPLE_SENSORS_FLAG = 1;
    }
}

// EINT3 Interrupt Handler
void EINT3_IRQHandler(void)
{
	// Determine whether GPIO Interrupt P2.10 has occurred
	if ((LPC_GPIOINT->IO2IntStatF>>10)& 0x1)
	{
		MODE_FLAG = !MODE_FLAG; // ready to put device into monitor mode

        // Clear GPIO Interrupt P2.10
        LPC_GPIOINT->IO2IntClr = 1<<10;
	}
}



/* SysTick helper functions */
void SysTick_Handler(void) {
	msTicks++;
}

uint32_t getTicks(void) {
	return msTicks;
}

int main (void) {
	//SysTick init
	SysTick_Config(SystemCoreClock/1000);

	//protocol init
	init_I2C2();
	init_SSP();

	//GPIO devices init
	pca9532_init(); //port expander for led array
	rgb_init(); 	//rgb led
	temp_init(getTicks); //temperature sensor

	//SSP/GPIO devices init
	led7seg_init(); //seven-segment display
	oled_init(); //OLED display module

	//I2C sensors init
	light_init(); //light sensor module
	acc_init(); //accelerometer sensor

	//interrupts init
	init_timer1(); //2s period clock
	init_timer2(); //4s period clock
    LPC_GPIOINT->IO2IntEnF |= 1<<10;     // Enable GPIO Interrupt P2.10 (SW3)

    NVIC_EnableIRQ(EINT3_IRQn);     // Enable EINT3 interrupt


	//hardware setup
	light_enable(); //enable light sensor
	oled_clearScreen(OLED_COLOR_BLACK); //clear oled
	acc_read(&accInitX, &accInitY, &accInitZ); //initialize base acc params


	//round robin w/interrupts for RGB, LED array, SSEG
	while(1) {
		//stable,stable mode
		if(MODE_FLAG == 0) {

			//off everything
			oled_clearScreen(OLED_COLOR_BLACK);

			led7seg_setChar(0x00 , 0);
		    sseg_count = 0;

			rgb_setLeds(ledRed_mask & 0x00);

			//reset clocks
			LPC_TIM1->TCR = (1 << 1);
			LPC_TIM2->TCR = (1 << 1);
			//disable timers
			LPC_TIM1->TCR = (0 << 0);
			LPC_TIM2->TCR = (0 << 0);

			SET_MONITOR_OLED_FLAG = 1; //write MONITOR to OLED later



			while(MODE_FLAG == 0); //wait for MONITOR to be enabled



			//un-reset clocks
			LPC_TIM1->TCR = (0 << 1);
			LPC_TIM2->TCR = (0 << 1);
			//re-enable
			init_timer1();
			init_timer2();
		}
		//init: write MONITOR to OLED once, write LUX:
		if(SET_MONITOR_OLED_FLAG) {
			oled_putString(1, 1, "MODE: MONITOR", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			oled_putString(1, 10, "LUX : ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			oled_putString(1, 20, "TEMP: ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			oled_putString(1, 30, "ACC : ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);

			SET_MONITOR_OLED_FLAG = 0;
		}



		//ISR tasks
		if(RED_LED_FLAG && MODE_FLAG) {
		    rgb_setLeds(ledRed_mask & set); 	// bitwise and
		    set = ~set;

		    RED_LED_FLAG = 0;
		}

		if(LED_ARRAY_FLAG && MODE_FLAG) {
			pca9532_setLeds(led_set, 0xFFFF);  //moves onLed down array
			led_set = LED_MOV_DIR ? led_set >> 1 : led_set << 1;

			//changes direction when at the ends
			if(led_set == 0x0001) {
				LED_MOV_DIR = 0;
			} else if(led_set == 0x8000) {
				LED_MOV_DIR = 1;
			}
			LED_ARRAY_FLAG = 0;
		}

		if(SSEG_FLAG && MODE_FLAG) {
		    led7seg_setChar(monitor_symbols[sseg_count++], 0);
		    sseg_count %= 16;

		    SSEG_FLAG = 0;
		}

		//main tasks
		if(SAMPLE_SENSORS_FLAG) {
			//poll light sensor
			light_reading = light_read();

			//poll acc sensor
			acc_read(&accX, &accY, &accZ);

			//poll temp sensor
			temperature_reading = temp_read();



			//update OLED
			sprintf(tempStr, "LUX :%lu", light_reading);
			oled_putString(1, 10, tempStr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			sprintf(tempStr, "TEMP:%.2f", temperature_reading/10.0);
			oled_putString(1, 20, tempStr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
			sprintf(tempStr, "ACC :%d|%d|%d     ", accX-accInitX, accY-accInitY, accZ-accInitZ);
			oled_putString(1, 30, tempStr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);

			SAMPLE_SENSORS_FLAG = 0;
		}
	}

	return 0;
}





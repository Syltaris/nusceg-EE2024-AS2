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

#define light_interrupt 17

void rgbLED_controller(void);
void monitor_oled_init(void);

/*** LED params ***/
static uint8_t ledRed_set = 0;
static uint8_t ledRed_mask = RGB_RED;
static uint8_t ledBlue_set = 0;
static uint8_t ledBlue_mask = RGB_BLUE;

static uint32_t led_set = 0x0001; //for array

static volatile int red_led_flag = 0;
static volatile int led_array_flag = 0;
static int led_mov_dir = 0; // 0 for <<, 1 for >>

/*** timer params ***/
volatile uint32_t msTicks = 0; // counter for 1ms SysTicks

/*** 7-segment display params ***/
volatile int sseg_flag = 0;
unsigned int timer2count = 0;
int monitor_symbols[]  = {'0', '1', '2', '3', '4', '5', '6' ,'7' ,'8' ,'9' ,'A', 'B', 'C', 'D', 'E', 'F'};

/*** ISL290003 light sensor params ***/
uint32_t light_reading = 0;
volatile int light_low_flag = 0;

/*** MMA7455 accelerometer sensor params ***/
int8_t accInitX, accInitY, accInitZ; //for offsetting
int8_t accX, accY, accZ;
int8_t accOldX, accOldY, accOldZ;

/*** temperature sensor ***/
int32_t temperature_reading = 0;
const int32_t TEMP_HIGH_WARNING = 450;
int temp_high_flag = 0;

/*** stable, monitor mode flag ***/
volatile int mode_flag = 0; //1 - monitor, 0 - passive

/*** flag to sample light and accelerometer sensors ***/
volatile int sample_sensors_flag = 0;

/*** OLED params ***/
char tempStr[80];

<<<<<<< HEAD
/*** UART params ***/
volatile int send_message_flag = 0;





=======
>>>>>>> f37298b583189b5600f2650777a426b6e4c5f9a6
/*** protocols initialisers ***/

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

//uart1 pincfg
void pinsel_uart1(void){
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 0;
    PinCfg.Portnum = 15;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 16;
    PINSEL_ConfigPin(&PinCfg);
}

//uart3 pincfg
void pinsel_uart3(void){
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 2;
    PinCfg.Pinnum = 0;
    PinCfg.Portnum = 0;
    PINSEL_ConfigPin(&PinCfg);
    PinCfg.Pinnum = 1;
    PINSEL_ConfigPin(&PinCfg);
}

//uart enabler
void init_uart(void){
    UART_CFG_Type uartCfg;
    uartCfg.Baud_rate = 115200;
    uartCfg.Databits = UART_DATABIT_8;
    uartCfg.Parity = UART_PARITY_NONE;
    uartCfg.Stopbits = UART_STOPBIT_1;
    //pin select for uart1;
    pinsel_uart1();
    //supply power & setup working parameters for uart1
    UART_Init(LPC_UART1, &uartCfg);
    //enable transmit for uart1
    UART_TxCmd(LPC_UART1, ENABLE);

    //init uart3
    pinsel_uart3();
    UART_Init(LPC_UART3, &uartCfg);
    UART_TxCmd(LPC_UART3, ENABLE);
}

//timer1 w/ 0.33s~ period
static void init_timer1(){

	// default value of PCLK = CCLK/4
	// CCLK is derived from PLL0 output signal / (CCLKSEL + 1) [CCLKCFG register]

    LPC_SC->PCONP |= (1<<2);					/* Power ON Timer1 */

    LPC_TIM1->MCR  = (1<<0) | (1<<1);     		/* Clear COUNT on MR0 match and Generate Interrupt */
    LPC_TIM1->PR   = 0;      					/* Update COUNT every (value + 1) of PCLK  */
    LPC_TIM1->MR0  = 8888888;                 	/* Value of COUNT that triggers interrupts */
    LPC_TIM1->TCR  = (1 << 0);                 	/* Start timer by setting the Counter Enable */

    NVIC_EnableIRQ(TIMER1_IRQn);				/* Enable Timer1 interrupt */

}

//timer2 w/ 1s~ period
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

void rgbLED_controller(void);
void TIMER1_IRQHandler(void)
{
	unsigned int isrMask;

    isrMask = LPC_TIM1->IR;
    LPC_TIM1->IR = isrMask;         /* Clear the Interrupt Bit by writing to the register */					// bitwise not

    rgbLED_controller();

    led_array_flag = 1;
}

void sseg_controller(void);
void TIMER2_IRQHandler(void)
{
	unsigned int isrMask;

    isrMask = LPC_TIM2->IR;
    LPC_TIM2->IR = isrMask;         /* Clear the Interrupt Bit by writing to the register */					// bitwise not

    sseg_flag = 1;

    if(timer2count == 5 || timer2count == 10 || timer2count == 15) {
    	sample_sensors_flag = 1;
    }

    if(timer2count == 15) {
    	send_message_flag = 1;
    }

	sseg_controller(); // ! may be slow
}

// EINT3 Interrupt Handler
void EINT3_IRQHandler(void)
{
	// Determine whether GPIO Interrupt P2.10 has occurred
	if ((LPC_GPIOINT->IO2IntStatF>>10)& 0x1)
	{
		mode_flag = !mode_flag; // ready to put device into monitor mode

        // Clear GPIO Interrupt P2.10
        LPC_GPIOINT->IO2IntClr = 1<<10;
	}
}



/*** SysTick helper functions ***/
void SysTick_Handler(void) {
	msTicks++;
}

uint32_t getTicks(void) {
	return msTicks;
}

//protocol init
void init_protocols() {
	//protocol init
	init_I2C2();
	init_SSP();
	init_uart();
}

//sensors, peripherals init
void init_peripherals() {
	//GPIO devices init
	pca9532_init(); //port expander for led array
	rgb_init(); //rgb led
	temp_init(getTicks); //temperature sensor
	//SSP/GPIO devices init
	led7seg_init(); //seven-segment display
	oled_init(); //OLED display module
	//I2C sensors init
	light_init(); //light sensor module
	acc_init(); //accelerometer sensor
}

//interrupts init
void init_interrupts() {
	//interrupts init
	init_timer1(); //2s period clock
	init_timer2(); //4s period clock

//	LPC_GPIOINT ->IO2IntEnF |= 1 << 10;

	// config pin 2.10 for EINT0
	PINSEL_CFG_Type PinCfg;
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 10;
	PINSEL_ConfigPin(&PinCfg);

	// EINT0
	int * EXT_INT_Mode_Register = (int *)0x400fc148;
	* EXT_INT_Mode_Register |= 1 << 0; // edge sensitive
	int * EXT_INT_Polarity_Register = (int *)0x400fc14c;
	* EXT_INT_Polarity_Register |= 0 << 0; // falling edge

	NVIC_EnableIRQ(EINT0_IRQn); // Enable EINT0 interrupt

	LPC_GPIOINT ->IO0IntEnF |= 1 << light_interrupt;

	light_enable();
	light_setLoThreshold(100);
	light_setIrqInCycles(LIGHT_CYCLE_1);

	NVIC_EnableIRQ(EINT3_IRQn); // Enable EINT3 interrupt

}

//reset devices and disable timers
void prep_passiveMode(void) {
	//off everything
	oled_clearScreen(OLED_COLOR_BLACK); //clear OLED
	led7seg_setChar(0x00, 0);			//off 7 segment
	timer2count = 0;						//reset 7 segment counter
	rgb_setLeds(ledRed_mask & 0x00);	//off RGB led

	//reset clocks
	LPC_TIM1 ->TCR = (1 << 1);
	LPC_TIM2 ->TCR = (1 << 1);
	//disable timers
	LPC_TIM1 ->TCR = (0 << 0);
	LPC_TIM2 ->TCR = (0 << 0);

	//reset flags
	temp_high_flag = 0;
	light_low_flag = 0;
}

//re-enable timers
void monitor_oled_init(void);
void prep_monitorMode(void) {
	//un-reset clocks
	LPC_TIM1 ->TCR = (0 << 1);
	LPC_TIM2 ->TCR = (0 << 1);
	//re-enable
	init_timer1();
	init_timer2();

	monitor_oled_init();
}

//sets the sseg to the corresponding symbol
void sseg_controller(void) {
	led7seg_setChar(monitor_symbols[timer2count++], 0);
	timer2count %= 16;
}

//toggles LEDs
void rgbLED_controller(void) {
	if(temp_high_flag) {
		ledRed_set = ~ledRed_set;
	} else {
		ledRed_set = 0;
	}

	if(light_low_flag) {
		ledBlue_set = ~ledBlue_set;
	} else {
		ledBlue_set = 0;
	}

	rgb_setLeds((ledRed_mask & ledRed_set)
				| (ledBlue_mask & ledBlue_set));
}

//sample the accelerometer, light, temperature sensors
void sample_sensors(void) {
	//poll light sensor
	light_reading = light_read();
	//poll acc sensor
	acc_read(&accX, &accY, &accZ);
	//poll temp sensor
	temperature_reading = temp_read();
}

/*** OLED functions for monitor mode ***/

//prints empty labels and 'monitor' on oled
void monitor_oled_init(void) {
	oled_putString(1, 1, (uint8_t *) "MODE: MONITOR", OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	oled_putString(1, 10, (uint8_t *) "LUX : ", OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	oled_putString(1, 20, (uint8_t *) "TEMP: ", OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	oled_putString(1, 30, (uint8_t *) "ACC : ", OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
}

//update sampled data on oled
void monitor_oled_sensors(void) {
	//update OLED
	sprintf(tempStr, "LUX :%lu", light_reading);
	oled_putString(1, 10, (uint8_t *) tempStr, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	sprintf(tempStr, "TEMP:%.2f", temperature_reading / 10.0);
	oled_putString(1, 20, (uint8_t *) tempStr, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
	sprintf(tempStr, "ACC :%d|%d|%d     ", accX - accInitX, accY - accInitY,
			accZ - accInitZ);
	oled_putString(1, 30, (uint8_t *) tempStr, OLED_COLOR_WHITE,
			OLED_COLOR_BLACK);
}

<<<<<<< HEAD
//transmit message through UART
void transmitData(char* msg) {
    uint8_t data = 0;
    uint32_t len = 0;
    uint8_t line[64];

    //test sending message
    UART_Send(LPC_UART3, (uint8_t *)msg , strlen(msg), BLOCKING);
    //test receiving a letter and sending back to port
    UART_Receive(LPC_UART3, &data, 1, BLOCKING);
    UART_Send(LPC_UART3, &data, 1, BLOCKING);
    //test receiving message without knowing message length
    len = 0;
    do
    {   UART_Receive(LPC_UART3, &data, 1, BLOCKING);

        if (data != '\r')
        {
            len++;
            line[len-1] = data;
        }
    } while ((len<64) && (data != '\r'));
    line[len]=0;
    UART_SendString(LPC_UART3, &line);
    printf("--%s--\n", line);
}

int main (void) {
=======
int main(void) {
>>>>>>> f37298b583189b5600f2650777a426b6e4c5f9a6
	//SysTick init
	SysTick_Config(SystemCoreClock / 1000);

	init_protocols();
	init_peripherals();
	init_interrupts();

	//hardware setup
	oled_clearScreen(OLED_COLOR_BLACK); //clear oled
	acc_read(&accInitX, &accInitY, &accInitZ); //initialize base acc params

	//main execution loop
	while (1) {
		uint32_t reading = light_read();
		printf("%d\n", (int) reading);

		if (flag) {
			printf("hello\n");
		}

		//stable, passive mode
		if (mode_flag == 0) {
			prep_passiveMode();
			while (mode_flag == 0)
				; //wait for MONITOR to be enabled
			prep_monitorMode();
		}

		if (led_array_flag && mode_flag) {
			pca9532_setLeds(led_set, 0xFFFF);  //moves onLed down array
			led_set = led_mov_dir ? led_set >> 1 : led_set << 1;

			//changes direction when at the ends
			if (led_set == 0x0001) {
				led_mov_dir = 0;
			} else if (led_set == 0x8000) {
				led_mov_dir = 1;
			}
			led_array_flag = 0;
		}

		//main tasks
		if (sample_sensors_flag) {
			sample_sensors();
			monitor_oled_sensors();
			sample_sensors_flag = 0;
		}

		//if MOVEMENT_DETECTED //if LOW_LIGHT_WARNING
		if (1) {
			light_low_flag = 1;
		}

		//if high temperature is detected
		if (temperature_reading >= TEMP_HIGH_WARNING) {
			temp_high_flag = 1;
		}

		//if need to transmit
		if(send_message_flag) {
			char* msg = "HELLCOME \n";
			transmitData(msg);

			send_message_flag = 0;
		}
	}

	return 0;
}


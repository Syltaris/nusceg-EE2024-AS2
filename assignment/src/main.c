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

#define DEBUG

#ifdef DEBUG
#define DEBUG_HEAT_OFFSET 200
#endif
#ifndef DEBUG
#define DEBUG_HEAT_OFFSET 0
#endif

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
int movement_lowLight_flag = 0;
volatile int detect_darkness_flag = 1; // 0 also means that darkness is detected

/*** MMA7455 accelerometer sensor params ***/
int8_t accInitX, accInitY, accInitZ; //for offsetting
volatile int8_t accX, accY, accZ;
int8_t accOldX, accOldY, accOldZ;
volatile int movement_detected_flag = 0;
volatile uint32_t lastMotionDetectedTicks = 0;

/*** temperature sensor ***/
int32_t temperature_reading = 0;
const int32_t TEMP_HIGH_WARNING = 450;
int temp_high_flag = 0;

/*** stable, monitor mode flag ***/
volatile int mode_flag = 0; //1 - monitor, 0 - passive

/*** flag to sample light and accelerometer sensors ***/
volatile int sample_sensors_flag = 0;

/*** OLED params ***/
uint8_t tempStr[80];

/*** UART params ***/
volatile int send_message_flag = 0;





/*** protocols initialisers ***/
static void init_GPIO(void) {
	PINSEL_CFG_Type PinCfg;

	/* Initialize SW3 pin connect */
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 2;
	PinCfg.Pinmode = 0;
	PINSEL_ConfigPin(&PinCfg);
}


//i2c enabler
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

//uart1 pincfg
void pinsel_uart1(void){
    PINSEL_CFG_Type PinCfg;
    PinCfg.Funcnum = 1;
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



/*** SysTick helper functions ***/
void SysTick_Handler(void) {
	msTicks++;
}

uint32_t getTicks(void) {
	return msTicks;
}


/** light_sensor helper functions **/\

//configure light sensors thresholds to detect any lighting
void lightSensor_detectLight() {
	light_setLoThreshold(0);
	light_setHiThreshold(50);
}

//configures light sensor's HI and LO thresholds to detect low light conditions
void lightSensor_detectDarkness() {
	light_setLoThreshold(50);
	light_setHiThreshold(972);
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
	init_timer1(); //0.33s period clock
	init_timer2(); //1s period clock

	//switches
//	LPC_GPIOINT ->IO2IntEnF |= 1 << 10; // enable SW3

	//light sensor
	LPC_GPIOINT ->IO2IntClr |= 1 << 5;
	LPC_GPIOINT ->IO2IntEnF |= 1 << 5; // enable light interrupt
	light_clearIrqStatus();
	//configure default light threshold
	lightSensor_detectDarkness();

	// accelerometer pio1_8, P0.3
	LPC_GPIOINT ->IO0IntClr |= 1 << 3;
	LPC_GPIOINT ->IO0IntEnR |= 1 << 3;

	NVIC_ClearPendingIRQ(EINT3_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn); // Enable EINT3 interrupt


	// EINT0
	int * EXT_INT_Mode_Register = (int *)0x400fc148;
	* EXT_INT_Mode_Register |= 1 << 0; // edge sensitive
	int * EXT_INT_Polarity_Register = (int *)0x400fc14c;
	* EXT_INT_Polarity_Register |= 0 << 0; // falling edge

	NVIC_ClearPendingIRQ(EINT0_IRQn);
	NVIC_EnableIRQ(EINT0_IRQn); // Enable EINT0 interrupt

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

	if(movement_lowLight_flag) {
		ledBlue_set = ~ledBlue_set;
	} else {
		ledBlue_set = 0;
	}

	rgb_setLeds((ledRed_mask & ledRed_set)
				| (ledBlue_mask & ledBlue_set));
}

//void EINT0_IRQHandler(void) {
////	// Determine whether GPIO Interrupt P2.10 has occurred (SW3)
////	if ((LPC_GPIOINT->IO2IntStatF>>10)& 0x1) {
//		mode_flag = !mode_flag; // ready to put device into monitor mode
//
////        // Clear GPIO Interrupt P2.10
////        LPC_GPIOINT->IO2IntClr = 1<<10;
////	}
//}

void EINT0_IRQHandler(void) {
	printf("hi\n");

	mode_flag = !mode_flag;

	NVIC_ClearPendingIRQ(EINT0_IRQn);
	LPC_SC ->EXTINT = (1 << 0); /* Clear Interrupt Flag */
}

// EINT3 Interrupt Handler
void EINT3_IRQHandler(void)
{
	// Determine if GPIO Interrupt P2.5 has occurred (ISL2900023)
	if ((LPC_GPIOINT->IO2IntStatF>>5) & 0x1) {
		//clear interrupts
		LPC_GPIOINT->IO2IntClr = 1<<5; //clear GPIO interrupt
		light_clearIrqStatus(); //clear peripheral interrupt

		//darkness detected
		if(detect_darkness_flag == 1) {

			lightSensor_detectLight();
			detect_darkness_flag = 0;
		} else if(detect_darkness_flag == 0) {

			lightSensor_detectDarkness();
			detect_darkness_flag = 1;
		}

		printf("light triggered\n");
	}
	// Determine if GPIO Interrupt P0.3 has occurred (MMA7455 Accelerometer)
	if ((LPC_GPIOINT ->IO0IntStatR >> 3) & 0x1) {
	  movement_detected_flag = 1;

	  lastMotionDetectedTicks = getTicks();

	  LPC_GPIOINT->IO0IntClr = 1<<3;
	  acc_intClr();
	  printf("acc triggered\n");
	}

}


/*** OLED functions for monitor mode ***/

//prints empty labels and 'monitor' on oled
void monitor_oled_init(void) {
	oled_putString(1, 1, "MODE: MONITOR", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(1, 10, "LUX : ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(1, 20, "TEMP: ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	oled_putString(1, 30, "ACC : ", OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

//update sampled data on oled
void displaySampledData_oled(void) {
	//update OLED
	sprintf(tempStr, "LUX :%u     ", light_reading);
	oled_putString(1, 10, tempStr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	sprintf(tempStr, "TEMP:%.2f     ", temperature_reading / 10.0);
	oled_putString(1, 20, tempStr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
	sprintf(tempStr, "ACC :%d|%d|%d     ", accX - accInitX, accY - accInitY,
			accZ - accInitZ);
	oled_putString(1, 30, tempStr, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
}

void prep_monitorMode(void) {
	//un-reset clocks
	LPC_TIM1 ->TCR = (0 << 1);
	LPC_TIM2 ->TCR = (0 << 1);
	//re-enable
	init_timer1();
	init_timer2();

	monitor_oled_init();
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
	detect_darkness_flag = 1;
	movement_lowLight_flag = 0;
}




//sample the accelerometer, light, temperature sensors
void sample_sensors(void) {
	//poll light sensor
	light_reading = light_read();
	//poll acc sensor
	acc_setMode(ACC_MODE_MEASURE);
	acc_read(&accX, &accY, &accZ);
	acc_setMode(ACC_MODE_LEVEL);

	//poll temp sensor
	temperature_reading = temp_read();
}




/*** Data transmitting functions ***/

int format_string(char * string, char * title, float value){
 return sprintf(string, "%s_%s%.2f", string, title, value);
}
//transmit message through UART
void transmitData(char* msg) {
	if(temp_high_flag == 1) {
		UART_SendString(LPC_UART3, "Fire was Detected.\r\n");
	}

	if(movement_lowLight_flag == 1) {
		UART_SendString(LPC_UART3, "Movement in darkness was Detected.\r\n");
	}

	static uint8_t transmitCount = 0;

    char string[50];
	sprintf(string, "%d_-", transmitCount++);

	format_string(string, "T", temperature_reading/10.0);
	format_string(string, "L", light_reading);
	format_string(string, "AX", accX-accInitX);
	format_string(string, "AY", accY-accInitY);
	format_string(string, "AZ", accZ-accInitZ);

	sprintf(string, "%s\r\n", string);

    UART_SendString(LPC_UART3, &string);
}




int main (void) {
	//SysTick init
	SysTick_Config(SystemCoreClock/1000);

	init_protocols();
	init_peripherals();
	init_GPIO();
	init_interrupts();


	//hardware setup
	light_setIrqInCycles(LIGHT_CYCLE_1);
	light_enable(); //enable light sensor
	oled_clearScreen(OLED_COLOR_BLACK); //clear oled
	acc_setMode(ACC_MODE_LEVEL); //configure for interrupt
	acc_setRange(ACC_RANGE_8G);
	acc_config_mode_LEVEL();
	acc_read(&accInitX, &accInitY, &accInitZ); //initialize base acc params

	//main execution loop
	while(1) {
		//stable, passive mode
		if(mode_flag == 0) {
			prep_passiveMode();
			while(mode_flag == 0); //wait for MONITOR to be enabled
			prep_monitorMode();
		}

		if(led_array_flag && mode_flag) {
			pca9532_setLeds(led_set, 0xFFFF);  //moves onLed down array
			led_set = led_mov_dir ? led_set >> 1 : led_set << 1;

			//changes direction when at the ends
			if(led_set == 0x0001) {
				led_mov_dir = 0;
			} else if(led_set == 0x8000) {
				led_mov_dir = 1;
			}
			led_array_flag = 0;
		}

		//main tasks
		if(sample_sensors_flag) {
			sample_sensors();
			displaySampledData_oled();
			sample_sensors_flag = 0;
		}

		//if high temperature is detected
		if(temperature_reading >= (TEMP_HIGH_WARNING - DEBUG_HEAT_OFFSET)) {
			temp_high_flag = 1;
		}

		//if MOVEMENT_DETECTED
		if(movement_detected_flag) {
			//if no movement in darkness after set duration, disable movement flag
			if(getTicks() > lastMotionDetectedTicks + 20) {
				//check for prolonged movement detection, if so, set flag
				if(!detect_darkness_flag) {
					movement_lowLight_flag = 1;
				} else {
					movement_detected_flag = 0;
				}
			}
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





/**
*
* @author Ram Bhattarai (ram9@pdx.edu)
* @copyright Portland State University, 2016-2019
* This file implements the colorwheel application by taking the switches and rotary encoder input
*  and displaying the color according to the HSV scale.
*  Once the values are calcuated, it will be replicated to RGB leds
*  Those values will be detected again in the software and hardware pulse width detection
* The peripherals provides access to the Nexys4 pushbuttons
* and slide switches, the LEDs, the RGB LEDs, and the Seven Segment display
* on the Digilent Nexys4 DDR board and the PmodOLEDrgb (94 x 64 RGB graphics display)
* and the PmodENC (rotary encoder + slide switch + pushbutton).
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  Date     Changes
* ----- ---- -------- -----------------------------------------------
* 1.00a	rhk	02-Jul-2016		First release of test program.  Builds on the ece544 peripheral test used
*							to check the functionality of Nexys4IO and PMod544IOR2
* 2.00a sy  14-Oct-2016		Modified the code to include different initialize function for other peripherals
*							connected to the system.
* 3.00	rk	05-Apr-2018		Modified for Digilent PmodENC and PmodOLEDrgb.  Replaced MB_Sleep() w/ usleep.
* 4.00  Ram 4/18/2019     Modified for Colorwheel, Pulse width detection, writing to seven segment
******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include "platform.h"
#include "xparameters.h"
#include "xstatus.h"
#include "microblaze_sleep.h"
#include "nexys4IO.h"
#include "PmodOLEDrgb.h"
#include "PmodENC.h"
#include "xgpio.h"
#include "xintc.h"
#include "xtmrctr.h"

/************************** Constant Definitions ****************************/

// Clock frequencies
#define CPU_CLOCK_FREQ_HZ		XPAR_CPU_CORE_CLOCK_FREQ_HZ
#define AXI_CLOCK_FREQ_HZ		XPAR_CPU_M_AXI_DP_FREQ_HZ

// AXI timer parameters
#define AXI_TIMER_DEVICE_ID		XPAR_AXI_TIMER_0_DEVICE_ID
#define AXI_TIMER_BASEADDR		XPAR_AXI_TIMER_0_BASEADDR
#define AXI_TIMER_HIGHADDR		XPAR_AXI_TIMER_0_HIGHADDR
#define TmrCtrNumber			0


// Definitions for peripheral NEXYS4IO
#define NX4IO_DEVICE_ID		XPAR_NEXYS4IO_0_DEVICE_ID
#define NX4IO_BASEADDR		XPAR_NEXYS4IO_0_S00_AXI_BASEADDR
#define NX4IO_HIGHADDR		XPAR_NEXYS4IO_0_S00_AXI_HIGHADDR

// Definitions for peripheral PMODOLEDRGB
#define RGBDSPLY_DEVICE_ID		XPAR_PMODOLEDRGB_0_DEVICE_ID
#define RGBDSPLY_GPIO_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_BASEADDR
#define RGBDSPLY_GPIO_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_GPIO_HIGHADD
#define RGBDSPLY_SPI_BASEADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_BASEADDR
#define RGBDSPLY_SPI_HIGHADDR	XPAR_PMODOLEDRGB_0_AXI_LITE_SPI_HIGHADDR

// Definitions for peripheral PMODENC
#define PMODENC_DEVICE_ID		XPAR_PMODENC_0_DEVICE_ID
#define PMODENC_BASEADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_BASEADDR
#define PMODENC_HIGHADDR		XPAR_PMODENC_0_AXI_LITE_GPIO_HIGHADDR

// Fixed Interval timer - 100 MHz input clock, 40KHz output clock
// FIT_COUNT_1MSEC = FIT_CLOCK_FREQ_HZ * .001
#define FIT_IN_CLOCK_FREQ_HZ	CPU_CLOCK_FREQ_HZ
#define FIT_CLOCK_FREQ_HZ		40000
#define FIT_COUNT				(FIT_IN_CLOCK_FREQ_HZ / FIT_CLOCK_FREQ_HZ)
#define FIT_COUNT_1MSEC			40

// GPIO parameters
#define GPIO_0_DEVICE_ID					XPAR_AXI_GPIO_0_DEVICE_ID
#define GPIO_0_INPUT_0_CHANNEL				1
#define GPIO_0_OUTPUT_0_CHANNEL				2

//GPIO 1 for the RED LEDs
//Used to detect the pulse width
#define GPIO_1_DEVICE_ID					XPAR_AXI_GPIO_1_DEVICE_ID
#define GPIO_1_INPUT_0_CHANNEL				2
#define GPIO_1_INPUT_1_CHANNEL				1

//GPIO 2 for the Green LEDs
//Used to detect the pulse width
#define GPIO_2_DEVICE_ID					XPAR_AXI_GPIO_2_DEVICE_ID
#define GPIO_2_INPUT_0_CHANNEL				2
#define GPIO_2_INPUT_1_CHANNEL				1

//GPIO 3 for the Blue LEDs
//Used to detect the pulse width
#define GPIO_3_DEVICE_ID					XPAR_AXI_GPIO_3_DEVICE_ID
#define GPIO_3_INPUT_0_CHANNEL				2
#define GPIO_3_INPUT_1_CHANNEL				1

// Interrupt Controller parameters
#define INTC_DEVICE_ID			XPAR_INTC_0_DEVICE_ID
#define FIT_INTERRUPT_ID		XPAR_MICROBLAZE_0_AXI_INTC_FIT_TIMER_0_INTERRUPT_INTR

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Variable Definitions ****************************/
// Microblaze peripheral instances
uint64_t 	timestamp = 0L;
PmodOLEDrgb	pmodOLEDrgb_inst;
PmodENC 	pmodENC_inst;
XGpio		GPIOInst0;					// GPIO instance 0
XGpio		GPIOInst1;					// GPIO instance 1 (RED)
XGpio		GPIOInst2;					// GPIO instance 2 (GREEN)
XGpio		GPIOInst3;					// GPIO instance 3 (BLUE)
XIntc 		IntrptCtlrInst;				// Interrupt Controller instance
XTmrCtr		AXITimerInst;			// PWM timer instance


// The following variables are shared between non-interrupt processing and
// interrupt processing such that they must be global(and declared volatile)
// These variables are controlled by the FIT timer interrupt handler


volatile u32		  gpio_in;				// GPIO input port

volatile u8           SAT=0;                //Saturation Value ---- Increments/Decrements & Stores value when button Right and Left is pressed
volatile u8           HUE=0;                //Hue Value-------------Increments/Decrements & Stores value when rotatary encoder is rotated
volatile u8           VAL=0;                //Value-----------------Increments/Decrements & Stores value when button UP and Down is pressed

volatile u16          RGBVAL=0;            //Stores the converted 565 RGB value from HSV
volatile u16          OLDRGBVAL=0;         //Store the old RGB values

//Keep track of the old values to properly update the values on OLEDRGB
volatile u32          OLD_SAT = 0;
volatile u32          OLD_HUE = 0;
volatile u32          OLD_VAL = 0;

//Store the temporary rotatary encoder values
volatile u32         ROTENC_CNT = 0;
volatile u32         OLD_ROTENC_CNT=0;

//Store the temporrary Saturation Values
volatile u32		SAT_CNT = 0;
volatile u32        OLD_SAT_CNT = 0;

//Store the temporrary Saturation Values
volatile u32		VAL_CNT = 0;
volatile u32        OLD_VAL_CNT=0;

//PULSE width for LOW and HIGH Counts for the RED, Green and Blue LEDS
volatile u32       PWDET_RED_H = 0;
volatile u32       PWDET_RED_L = 0;

volatile u32       PWDET_GREEN_H = 0;
volatile u32       PWDET_GREEN_L = 0;

volatile u32       PWDET_BLUE_H = 0;
volatile u32       PWDET_BLUE_L = 0;



//PULSE width for LOW and HIGH Counts for the RED, Green and Blue LEDS Software Detection
volatile u32       SW_PWDET_RED_H = 0;
volatile u32       SW_PWDET_RED_L = 0;

volatile u32       SW_PWDET_GREEN_H = 0;
volatile u32       SW_PWDET_GREEN_L = 0;

volatile u32       SW_PWDET_BLUE_H = 0;
volatile u32       SW_PWDET_BLUE_L = 0;

//PWM signals for R,G,B software detection
volatile u32       SW_RED = 0;
volatile u32       SW_GREEN = 0;
volatile u32       SW_BLUE = 0;


//PWM signals for R,G,B software detection
volatile u32       OLD_SW_RED = 0;
volatile u32       OLD_SW_GREEN = 0;
volatile u32       OLD_SW_BLUE = 0;



//Count values signals for R,G,B software detection
volatile u32       CNT_SW_RED_H = 0;
volatile u32       CNT_SW_RED_L = 0;
volatile u32       CNT_SW_GREEN_H = 0;
volatile u32       CNT_SW_GREEN_L = 0;
volatile u32       CNT_SW_BLUE_H = 0;
volatile u32       CNT_SW_BLUE_L = 0;


//Values to store the Red, Green and Blue Duty Cycle
volatile u8 RED_DUTY_CYCLE = 0;
volatile u8 GREEN_DUTY_CYCLE = 0;
volatile u8 BLUE_DUTY_CYCLE = 0;


/************************** Function Prototypes *****************************/

void PMDIO_itoa(int32_t value, char *string, int32_t radix);
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num);
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix);
int	 do_init(void);											// initialize system
void FIT_Handler(void);										// fixed interval timer interrupt handler
int AXI_Timer_initialize(void);

//Helper Functions
void Initialize_OLED(void);
void colorwheel(void);
void UPDATE_OLED_DISPLAY(void);
u8 Extract_R(u16 wRGB);
u8 Extract_G(u16 wRGB);
u8 Extract_B(u16 wRGB);
void PWDET_HDWARE(void);
void PWDET_SFTWARE(void);
u8 DUTY_CYCLE(u32 PWDET_HIGH, u32 PWDET_LOW);
u32 Gpio_DiscreteRead(XGpio *InstancePtr, unsigned Channel);
void WRITETOSEVENSEGMENT(uint8_t RED_CYCLE,uint8_t GREEN_CYCLE,uint8_t BLUE_CYCLE);
void RESET_SEVEN_SEG();

/* Initialize the OLED Display with all the Label and rectangles
 * Display Hue, Saturation, Value on the left side of display
 * Display Rectangle box showing the color for the corresponding value of the Hue, Saturation, Value
 * values in HSV scale
 */
void Initialize_OLED(void)
{
	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);
	// Set up the display output
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildRGB(200, 12, 44));
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"H:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 3, 1);
	PMDIO_putnum(&pmodOLEDrgb_inst, ROTENC_CNT, 10);

	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"S:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 3, 3);
	PMDIO_putnum(&pmodOLEDrgb_inst, SAT_CNT/10, 10);
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
	PMDIO_putnum(&pmodOLEDrgb_inst, SAT_CNT%10, 10);

	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 5);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"V:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 3, 5);
	PMDIO_putnum(&pmodOLEDrgb_inst, VAL_CNT/10, 10);
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 5);
	PMDIO_putnum(&pmodOLEDrgb_inst, VAL_CNT%10, 10);

	OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,70,70,90,90,0,true,0);
}

//Calculate the duty cycle for the RGB leds
u8 DUTY_CYCLE(u32 PWDET_HIGH, u32 PWDET_LOW)
{
	return ((PWDET_HIGH*100) /(PWDET_HIGH+PWDET_LOW));
}

//Read the Current GPIO
//Calls provide GPIO function
u32 Gpio_DiscreteRead(XGpio *InstancePtr, unsigned Channel)
{
	return XGpio_DiscreteRead(InstancePtr,Channel);
}

//Hardware Pulse width detection
//Uses the RED, Green, and Blue registers available to detect
// the high and low pulse and calculates the duty cycle
//Duty cycle calculated is supplied to the Seven segment display
void PWDET_HDWARE(void)
{
	PWDET_RED_H = Gpio_DiscreteRead(&GPIOInst1,GPIO_1_INPUT_0_CHANNEL);
	PWDET_RED_L = Gpio_DiscreteRead(&GPIOInst1,GPIO_1_INPUT_1_CHANNEL);

	PWDET_GREEN_H = Gpio_DiscreteRead(&GPIOInst2,GPIO_2_INPUT_0_CHANNEL);
	PWDET_GREEN_L = Gpio_DiscreteRead(&GPIOInst2,GPIO_2_INPUT_1_CHANNEL);

	PWDET_BLUE_H = Gpio_DiscreteRead(&GPIOInst3,GPIO_3_INPUT_0_CHANNEL);
	PWDET_BLUE_L = Gpio_DiscreteRead(&GPIOInst3,GPIO_3_INPUT_1_CHANNEL);


	u8 RED_CYCLE = DUTY_CYCLE(PWDET_RED_H,PWDET_RED_L);
	u8 GREEN_CYCLE = DUTY_CYCLE(PWDET_GREEN_H,PWDET_GREEN_L);
	u8 BLUE_CYCLE = DUTY_CYCLE(PWDET_BLUE_H,PWDET_BLUE_L);

	xil_printf("FROM HDDET RED %d\n",RED_CYCLE);
	xil_printf("FROM HDDET RED HIGH VAL %d\n",PWDET_RED_H);
	xil_printf("FROM HDDET RED LOW VAL %d\n",PWDET_RED_L);
	xil_printf("FROM HDDET GREEN %d\n",GREEN_CYCLE);
	xil_printf("FROM HDDET GREEN HIGH VAL %d\n",PWDET_GREEN_H);
	xil_printf("FROM HDDET GREEN LOW VAL %d\n",PWDET_GREEN_L);
	xil_printf("FROM HDDET BLUE %d\n",BLUE_CYCLE);
	xil_printf("FROM HDDET BLUE HIGH VAL %d\n",PWDET_BLUE_H);
	xil_printf("FROM HDDET BLUE LOW VAL %d\n",PWDET_BLUE_L);

	//Write to seven segment
	WRITETOSEVENSEGMENT(RED_CYCLE,GREEN_CYCLE,BLUE_CYCLE);

}


/* Update the OLED Display values based on the
 * Display Hue, Saturation, Value on the left side of display
 * Display Rectangle box showing the color for the corresponding value of the Hue, Saturation, Value
 * values in HSV scale
 */

void UPDATE_OLED_DISPLAY(void)
{

	// Set up the display output
	OLEDrgb_SetFontColor(&pmodOLEDrgb_inst,OLEDrgb_BuildRGB(200, 12, 44));
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 1);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"H:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 3, 1);
	PMDIO_putnum(&pmodOLEDrgb_inst, ROTENC_CNT, 10);

	OLD_HUE = HUE;
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 3);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"S:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 3, 3);
	PMDIO_putnum(&pmodOLEDrgb_inst, SAT_CNT/10, 10);
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 3);
	PMDIO_putnum(&pmodOLEDrgb_inst, SAT_CNT%10, 10);
	OLD_SAT = SAT;

	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 0, 5);
	OLEDrgb_PutString(&pmodOLEDrgb_inst,"V:");
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 3, 5);
	PMDIO_putnum(&pmodOLEDrgb_inst, VAL_CNT/10, 10);
	OLEDrgb_SetCursor(&pmodOLEDrgb_inst, 4, 5);
	PMDIO_putnum(&pmodOLEDrgb_inst, VAL_CNT%10, 10);
	OLD_VAL = VAL;
	OLEDrgb_DrawRectangle(&pmodOLEDrgb_inst,70,70,90,90,RGBVAL,true,RGBVAL);

}

/************************** MAIN PROGRAM ************************************/
int main(void)
{
    init_platform();

	uint32_t sts;

	//Initialize everything
	sts = do_init();
	if (XST_SUCCESS != sts)
	{
		exit(1);
	}

	//Set the interrupts
	microblaze_enable_interrupts();

	//Reset the seven segment
	RESET_SEVEN_SEG();

	//Turn off all the LEDs
	NX4IO_setLEDs(0x00);


	xil_printf("ECE 544 Project 1\n\r");
	xil_printf("By Ram Bhattarai. 18-April-2019\n\n\r");

	//Initialze the PMOD Encoder
	ENC_begin(&pmodENC_inst, PMODENC_BASEADDR);

	//Initialize the PMO_OLED Display
	Initialize_OLED();

	/*Color wheel function that uses the switches and rotatary encoder input
	 * and displaying the color according to the HSV Scale. Then it replicates the obtained
	 * RGB values to RGB LEDs
	 * It also detects the pulse width from the software and the hardware
	 * If the LED[0] is lit, it sends to the Hardware detection
	 * Otherwise just does the software detection
	 * After detecting the pulse width values, the values are sent off to seven segment
	 * Display
	 */
	colorwheel();


	timestamp = 0;

	// clear the displays and power down the pmodOLEDrbg
	NX410_SSEG_setAllDigits(SSEGHI, CC_BLANK, CC_B, CC_LCY, CC_E, DP_NONE);
	NX410_SSEG_setAllDigits(SSEGLO, CC_B, CC_LCY, CC_E, CC_BLANK, DP_NONE);
	OLEDrgb_Clear(&pmodOLEDrgb_inst);
	OLEDrgb_end(&pmodOLEDrgb_inst);

	//Turn of RGB LEDs after the use
	NX4IO_RGBLED_setChnlEn(RGB1, false,false, false);
	NX4IO_RGBLED_setChnlEn(RGB2,false, false, false);
	NX4IO_RGBLED_setDutyCycle(RGB1,0,0, 0);
	NX4IO_RGBLED_setDutyCycle(RGB2,0,0,0);

	// cleanup and exit
    cleanup_platform();
    exit(0);
}

/**************************** HELPER FUNCTIONS ******************************/

/****************************************************************************/
/**
* initialize the system
*
* This function is executed once at start-up and after resets.  It initializes
* the peripherals and registers the interrupt handler(s)
*****************************************************************************/

int	 do_init(void)
{
	uint32_t status;				// status from Xilinx Lib calls

	// initialize the Nexys4 driver and (some of)the devices
	status = (uint32_t) NX4IO_initialize(NX4IO_BASEADDR);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// set all of the display digits to blanks and turn off
	// the decimal points using the "raw" set functions.
	// These registers are formatted according to the spec
	// and should remain unchanged when written to Nexys4IO...
	// something else to check w/ the debugger when we bring the
	// drivers up for the first time
	NX4IO_SSEG_setSSEG_DATA(SSEGHI, 0xFFFFFFFF);
	NX4IO_SSEG_setSSEG_DATA(SSEGLO, 0xFFFFFFFF);

	OLEDrgb_begin(&pmodOLEDrgb_inst, RGBDSPLY_GPIO_BASEADDR, RGBDSPLY_SPI_BASEADDR);

	// initialize the pmodENC and hardware
	ENC_begin(&pmodENC_inst, PMODENC_BASEADDR);

	// initialize the GPIO instances
	status = XGpio_Initialize(&GPIOInst0, GPIO_0_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// initialize the GPIO instances 1
	status = XGpio_Initialize(&GPIOInst1, GPIO_1_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// initialize the GPIO instances 1
		status = XGpio_Initialize(&GPIOInst2, GPIO_2_DEVICE_ID);
		if (status != XST_SUCCESS)
		{
			return XST_FAILURE;
		}

		// initialize the GPIO instances 1
		status = XGpio_Initialize(&GPIOInst3, GPIO_3_DEVICE_ID);
		if (status != XST_SUCCESS)
		{
			return XST_FAILURE;
		}
	// GPIO0 channel 1 is an 8-bit input port.
	// GPIO0 channel 2 is an 8-bit output port.
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL, 0xFF);
	XGpio_SetDataDirection(&GPIOInst0, GPIO_0_OUTPUT_0_CHANNEL, 0x00);

	// GPIO1 channel 1 is an 32-bit input port
	// GPIO1 channel 2 is an 32-bit input port
	// Used to detect the High and Low values for RED Leds of RGB LED
	XGpio_SetDataDirection(&GPIOInst1, GPIO_1_INPUT_0_CHANNEL, 0xFFFFFFFF);
	XGpio_SetDataDirection(&GPIOInst1, GPIO_1_INPUT_1_CHANNEL, 0xFFFFFFFF);

	// GPIO2 channel 1 is an 32-bit input port
	// GPIO2 channel 2 is an 32-bit input port
	// Used to detect the High and Low values for GREEN Leds of RGB LED
	XGpio_SetDataDirection(&GPIOInst2, GPIO_2_INPUT_0_CHANNEL, 0xFFFFFFFF);
	XGpio_SetDataDirection(&GPIOInst2, GPIO_2_INPUT_1_CHANNEL, 0xFFFFFFFF);


	// GPIO2 channel 1 is an 32-bit input port
	// GPIO2 channel 2 is an 32-bit input port
	// Used to detect the High and Low values for BLUE Leds of RGB LED
	XGpio_SetDataDirection(&GPIOInst3, GPIO_3_INPUT_0_CHANNEL, 0xFFFFFFFF);
	XGpio_SetDataDirection(&GPIOInst3, GPIO_3_INPUT_1_CHANNEL, 0xFFFFFFFF);


	status = AXI_Timer_initialize();
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// initialize the interrupt controller
	status = XIntc_Initialize(&IntrptCtlrInst, INTC_DEVICE_ID);
	if (status != XST_SUCCESS)
	{
	   return XST_FAILURE;
	}
	
	// connect the fixed interval timer (FIT) handler to the interrupt
	status = XIntc_Connect(&IntrptCtlrInst, FIT_INTERRUPT_ID,
						   (XInterruptHandler)FIT_Handler,
						   (void *)0);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;

	}

	// start the interrupt controller such that interrupts are enabled for
	// all devices that cause interrupts.
	status = XIntc_Start(&IntrptCtlrInst, XIN_REAL_MODE);
	if (status != XST_SUCCESS)
	{
		return XST_FAILURE;
	}

	// enable the FIT interrupt
	XIntc_Enable(&IntrptCtlrInst, FIT_INTERRUPT_ID);
	return XST_SUCCESS;
}
/*
 * AXI timer initializes it to generate out a 4Khz signal, Which is given to the Nexys4IO module as clock input.
 * DO NOT MODIFY
 */
int AXI_Timer_initialize(void){

	uint32_t status;				// status from Xilinx Lib calls
	u32		ctlsts;		// control/status register or mask

	status = XTmrCtr_Initialize(&AXITimerInst,AXI_TIMER_DEVICE_ID);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	status = XTmrCtr_SelfTest(&AXITimerInst, TmrCtrNumber);
		if (status != XST_SUCCESS) {
			return XST_FAILURE;
		}
	ctlsts = XTC_CSR_AUTO_RELOAD_MASK | XTC_CSR_EXT_GENERATE_MASK | XTC_CSR_LOAD_MASK |XTC_CSR_DOWN_COUNT_MASK ;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber,ctlsts);

	//Set the value that is loaded into the timer counter and cause it to be loaded into the timer counter
	XTmrCtr_SetLoadReg(AXI_TIMER_BASEADDR, TmrCtrNumber, 24998);
	XTmrCtr_LoadTimerCounterReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts &= (~XTC_CSR_LOAD_MASK);
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	ctlsts = XTmrCtr_GetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber);
	ctlsts |= XTC_CSR_ENABLE_TMR_MASK;
	XTmrCtr_SetControlStatusReg(AXI_TIMER_BASEADDR, TmrCtrNumber, ctlsts);

	XTmrCtr_Enable(AXI_TIMER_BASEADDR, TmrCtrNumber);
	return XST_SUCCESS;

}

/*********************** DISPLAY-RELATED FUNCTIONS ***********************************/

/****************************************************************************/
/**
* Converts an integer to ASCII characters
*
* algorithm borrowed from ReactOS system libraries
*
* Converts an integer to ASCII in the specified base.  Assumes string[] is
* long enough to hold the result plus the terminating null
*
* @param 	value is the integer to convert
* @param 	*string is a pointer to a buffer large enough to hold the converted number plus
*  			the terminating null
* @param	radix is the base to use in conversion, 
*
* @return  *NONE*
*
* @note
* No size check is done on the return string size.  Make sure you leave room
* for the full string plus the terminating null in string
*****************************************************************************/
void PMDIO_itoa(int32_t value, char *string, int32_t radix)
{
	char tmp[33];
	char *tp = tmp;
	int32_t i;
	uint32_t v;
	int32_t  sign;
	char *sp;

	if (radix > 36 || radix <= 1)
	{
		return;
	}

	sign = ((10 == radix) && (value < 0));
	if (sign)
	{
		v = -value;
	}
	else
	{
		v = (uint32_t) value;
	}
	
  	while (v || tp == tmp)
  	{
		i = v % radix;
		v = v / radix;
		if (i < 10)
		{
			*tp++ = i+'0';
		}
		else
		{
			*tp++ = i + 'a' - 10;
		}
	}
	sp = string;
	
	if (sign)
		*sp++ = '-';

	while (tp > tmp)
		*sp++ = *--tp;
	*sp = 0;
	
  	return;
}


/****************************************************************************/
/**
* Write a 32-bit unsigned hex number to PmodOLEDrgb in Hex
*       
* Writes  32-bit unsigned number to the pmodOLEDrgb display starting at the current
* cursor position.
*
* @param num is the number to display as a hex value
*
* @return  *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/ 
void PMDIO_puthex(PmodOLEDrgb* InstancePtr, uint32_t num)
{
  char  buf[9];
  int32_t   cnt;
  char  *ptr;
  int32_t  digit;
  
  ptr = buf;
  for (cnt = 7; cnt >= 0; cnt--) {
    digit = (num >> (cnt * 4)) & 0xF;
    
    if (digit <= 9)
	{
      *ptr++ = (char) ('0' + digit);
	}
    else
	{
      *ptr++ = (char) ('a' - 10 + digit);
	}
  }

  *ptr = (char) 0;
  OLEDrgb_PutString(InstancePtr,buf);
  
  return;
}



/****************************************************************************/
/**
* Write a 32-bit number in Radix "radix" to LCD display
*
* Writes a 32-bit number to the LCD display starting at the current
* cursor position. "radix" is the base to output the number in.
*
* @param num is the number to display
*
* @param radix is the radix to display number in
*
* @return *NONE*
*
* @note
* No size checking is done to make sure the string will fit into a single line,
* or the entire display, for that matter.  Watch your string sizes.
*****************************************************************************/ 
void PMDIO_putnum(PmodOLEDrgb* InstancePtr, int32_t num, int32_t radix)
{
  char  buf[16];
  
  PMDIO_itoa(num, buf, radix);
  OLEDrgb_PutString(InstancePtr,buf);
  
  return;
}


/**************************** INTERRUPT HANDLERS ******************************/

/****************************************************************************/
/**
* Fixed interval timer interrupt handler
*
* Reads the GPIO port which reads back the hardware generated PWM wave for the RGB Leds
*
* @note
* ECE 544 students - When you implement your software solution for pulse width detection in
* Project 1 this could be a reasonable place to do that processing.
 *****************************************************************************/

void FIT_Handler(void)
{
	// Read the GPIO port to read back the generated PWM signal for RGB led's
	gpio_in = XGpio_DiscreteRead(&GPIOInst0, GPIO_0_INPUT_0_CHANNEL);


	SW_RED = (gpio_in & 0x0004)>>2;                                              //Shift to get to the SW_RED_register
	SW_BLUE = (gpio_in & 0x0002)>>1;                                            //Shift to get to the SW_Blue Register
	SW_GREEN =(gpio_in & 0x0001)>>0;                                         //Shift to get to the SW_BLUE register

	//if the red pulse is high and old_red_pulse is low
	//set the red low value
	if(SW_RED==1 && OLD_SW_RED==0)
	{
		SW_PWDET_RED_L = CNT_SW_RED_L;
		CNT_SW_RED_L = 0;
	}

	//If red pulse is low and old_red pulse is high
	//set the red high value
	else if (SW_RED==0 && OLD_SW_RED==1)
	{
		SW_PWDET_RED_H = CNT_SW_RED_H;
		CNT_SW_RED_H = 0;
	}

	//If red is high and the old value is one, increment the low counter value
	if (SW_RED==1 && OLD_SW_RED==1)
	{
		++CNT_SW_RED_L;

	}

	//If the red is low and the old_red value is low, increment the high value
	else if (SW_RED==0 && OLD_SW_RED==0)
	{
		++CNT_SW_RED_H;

	}

	//Set the old value to the current value for further comparision
	OLD_SW_RED = SW_RED;


	//if the green pulse is high and old_green_pulse is low
	//set the green low value
	if(SW_GREEN==1 && OLD_SW_GREEN==0)
	{
		SW_PWDET_GREEN_L = CNT_SW_GREEN_L;
		CNT_SW_GREEN_L = 0;
	}

	//If green pulse is low and old_red pulse is high
	//set the green high value
	else if (SW_GREEN==0 && OLD_SW_GREEN==1)
	{
		SW_PWDET_GREEN_H = CNT_SW_GREEN_H;
		CNT_SW_GREEN_H = 0;

	}

	//If green is high and the old value is high, increment the low counter value
	else if (SW_GREEN==1 && OLD_SW_GREEN==1)
	{
		++CNT_SW_GREEN_L;

	}

	//If the green is low and the old_green value is low, increment the high value
	else if (SW_GREEN==0 && OLD_SW_GREEN==0)
	{
		++CNT_SW_GREEN_H;

	}

	//Set the old value to the current value for further comparision
	OLD_SW_GREEN = SW_GREEN;


	//if the blue pulse is high and old_blue_pulse is low
	//set the blue low value
	if(SW_BLUE==1 && OLD_SW_BLUE==0)
	{
		SW_PWDET_BLUE_L = CNT_SW_BLUE_L;
		CNT_SW_BLUE_L = 0;
	}

	//If blue pulse is low and old_blue pulse is high
	//set the blue high value
	else if (SW_BLUE==0 && OLD_SW_BLUE==1)
	{
		SW_PWDET_BLUE_H = CNT_SW_BLUE_H;
		CNT_SW_BLUE_H = 0;
	}

	//If blue is high and the old value is one, increment the low counter value
	else if (SW_BLUE==1 && OLD_SW_BLUE==1)
	{
		++CNT_SW_BLUE_L;
	}

	//If the blue is low and the old_blue value is low, increment the high value
	else if (SW_BLUE==0 && OLD_SW_BLUE==0)
	{
		++CNT_SW_BLUE_H;

	}

	//Set the old value to the current value for further comparision
	OLD_SW_BLUE = SW_BLUE;

}

//Software Detection, Depends on the Fit timer module to determine Red, Green, Blue Duty cycle
//Uses Software pulse width low and high values to determine the Duty cycle
void PWDET_SFTWARE(void)
{
	RED_DUTY_CYCLE = DUTY_CYCLE(SW_PWDET_RED_H,SW_PWDET_RED_L);
	GREEN_DUTY_CYCLE = DUTY_CYCLE(SW_PWDET_GREEN_H,SW_PWDET_GREEN_L);
	BLUE_DUTY_CYCLE = DUTY_CYCLE(SW_PWDET_BLUE_H,SW_PWDET_BLUE_L);

	xil_printf("FROM SDDET RED %d\n",RED_DUTY_CYCLE);
	xil_printf("FROM SDDET RED HIGH VAL %d\n",SW_PWDET_RED_H);
	xil_printf("FROM SDDET RED LOW VAL %d\n",SW_PWDET_RED_L);
	xil_printf("FROM SDDET RED HIGH VAL %d\n",SW_PWDET_GREEN_H);
	xil_printf("FROM SDDET RED LOW VAL %d\n",SW_PWDET_GREEN_L);
	xil_printf("FROM SDDET RED HIGH VAL %d\n",SW_PWDET_BLUE_H);
	xil_printf("FROM SDDET RED LOW VAL %d\n",SW_PWDET_BLUE_L);
	xil_printf("FROM SDDET GREEN %d\n",RED_DUTY_CYCLE);
	xil_printf("FROM SDDET GREEN %d\n",GREEN_DUTY_CYCLE);
	xil_printf("FROM SDDET BLUE %d\n",BLUE_DUTY_CYCLE);

	//Write to Seven segment display
	WRITETOSEVENSEGMENT(RED_DUTY_CYCLE,GREEN_DUTY_CYCLE,BLUE_DUTY_CYCLE);

}

/*
 * Color wheel application that uses the input from switches and rotatry encoder input and displaying the color according
 * to the HSV scale
 */
void colorwheel(void)
{
	u32 state, laststate; //comparing current and previous state to detect edges on GPIO pins.
	bool btnrstate,btnlstate,btnustate,btndstate,btnr_laststate,btnl_laststate,btnu_laststate,btnd_laststate;

	laststate = ENC_getState(&pmodENC_inst);

	//Signals to check for the push buttons Right and LEFT, UP and DOWN
	//These signals are used to keep track of the push button press
	// It will only take the one count for the push button
	btnl_laststate= false;
	btnr_laststate=false;
	btnu_laststate= false;
	btnd_laststate=false;

	btnlstate= true;
	btnrstate=true;
	btnustate= true;
	btndstate=true;

	while(1) {
			// get the PmodENC state
			state = ENC_getState(&pmodENC_inst);

			// check if the rotary encoder pushbutton or BTNC is pressed
			// exit the loop if either one is pressed.
			if (ENC_buttonPressed(state) && !ENC_buttonPressed(laststate))//only check on button posedge
			{
				break;
			}

			if (NX4IO_isPressed(BTNC))
			{
				break;
			}

			else
			{
				//Get the rotation count from the rotary encoder
				ROTENC_CNT += ENC_getRotation(state, laststate);
			}


			//Check if old and current count is same
			if(ROTENC_CNT!=OLD_ROTENC_CNT)
			{
				if(ROTENC_CNT>359)						//IF count is > 359, reset it to 0
				{
					ROTENC_CNT = 0;
				}
				else if(ROTENC_CNT<0)				//IF count is <0, reset it to 359
				{
					ROTENC_CNT = 359;
				}

				//Normalize the HUE value to be in range for 0-255
				HUE= ROTENC_CNT*255/360;
			}

			//Set the old value to the current value
			OLD_ROTENC_CNT = ROTENC_CNT;
			laststate=state;

			//Button Right is pressed and Saturation is less than 99
			if (NX4IO_isPressed(BTNR) && SAT_CNT<99)
			{
					if(btnrstate && !btnr_laststate)
					{
						++SAT_CNT;
						btnrstate=false;
					}
			}

			//Button Right is pressed and Saturation is greater than equal to 99
			else if (NX4IO_isPressed(BTNR) && SAT_CNT>=99)
			{
				if(btnrstate && !btnr_laststate)
				{
					SAT_CNT = 0;
					btnrstate=false;
				}
			}

			//Button Left is pressed and Saturation is greater than 0, decrement the saturation count
			else if (NX4IO_isPressed(BTNL) && SAT_CNT>0)
			{
				if(btnlstate && !btnl_laststate)
				{
					--SAT_CNT;
					btnlstate = false;

				}

			}

			//Button Left is pressed and Saturation is equal 0, set the saturation count to 99
			else if (NX4IO_isPressed(BTNL) && SAT_CNT==0)
			{
					if(btnlstate && !btnl_laststate)
					{
						SAT_CNT = 99;
						btnlstate = false;
					}
			}

			//Otherwise no change, set the flag to false so that only one button press is kept
			else
			{
				SAT_CNT = SAT_CNT;
				btnlstate=true;
				btnrstate=true;

			}

			//Normalize the value to be in range for 0-255
			SAT = SAT_CNT * 255/99;


			//Button UP is pressed and the value is less than 99, increment the value count and set the button to false state
			if (NX4IO_isPressed(BTNU) && VAL_CNT<99)
			{
				if(btnustate && !btnu_laststate)
				{
					++VAL_CNT;
					btnustate = false;
				}

			}

			//Button is up and the value is greater than equal to 99, set the value to 0
			else if (NX4IO_isPressed(BTNU) && VAL_CNT>=99)
			{
				if(btnustate && !btnu_laststate)
				{
					VAL_CNT = 0;
					btnustate = false;
				}
			}

			//Button is down and the value is greater than 0, decrement the saturation count
			else if (NX4IO_isPressed(BTND) && VAL_CNT>0)
			{
				if(btndstate && !btnd_laststate)
				{
					--VAL_CNT;
					btndstate=false;
				}
			}

			//Button is Down and the value count is equal to 0, set the value back to 99
			else if (NX4IO_isPressed(BTND) && VAL_CNT==0)
			{
				if(btndstate && !btnd_laststate)
				{
					VAL_CNT = 99;
					btndstate = false;
				}
			}

			//Otherwise no change, but change the flag to false so only one press is kept
			else
			{
				VAL_CNT = VAL_CNT;
				btndstate = true;
				btnustate = true;

			}

			//Normalize the value to be in range for 0-255
			VAL = VAL_CNT * 255/99;

		//Now Check if there is a change in HUE, Saturation, and Value.
		// If there is a change, then call the UPDATE_OLED_DISPLAY to update the change
		// If there is no change do nothing
		// ALSO keep blinking RGB LED if there is no change
		// Detect the SOFTWARE OR Hardware pulse width detection

		if((HUE!=OLD_HUE) || (SAT!=OLD_SAT) || (VAL!=OLD_VAL))
		{
			RGBVAL = OLEDrgb_BuildHSV(HUE, SAT, VAL);

			// For RGB1 use the RGB Value to get the dutycycle for R, G, B
			NX4IO_RGBLED_setChnlEn(RGB1, true, true, true);
			NX4IO_RGBLED_setDutyCycle(RGB1,Extract_R(RGBVAL),Extract_G(RGBVAL), Extract_B(RGBVAL));

			// For RGB2 use the RGB Value to get the dutycycle for R, G, B
			NX4IO_RGBLED_setChnlEn(RGB2, true, true, true);
			NX4IO_RGBLED_setDutyCycle(RGB2, Extract_R(RGBVAL),Extract_G(RGBVAL), Extract_B(RGBVAL));

			usleep(5000);
			//Update the OledRgb display
			UPDATE_OLED_DISPLAY();

			xil_printf("RGB VAL %d\n",RGBVAL);
			xil_printf("FROM LED RED %d\n",Extract_R(RGBVAL));
			xil_printf("FROM LED GREEN %d\n",Extract_G(RGBVAL));
			xil_printf("FROM LED BLUE %d\n",Extract_B(RGBVAL));

			uint16_t ledval;
			//Get the value of switch
			ledval = NX4IO_getSwitches();

			//Set the switch values to the LED
			NX4IO_setLEDs(ledval);
			if(ledval==1)
			{
				PWDET_HDWARE();            //Led is lit
			}
			else
			{
				PWDET_SFTWARE();			//Led is off
			}
		}
	}
}

//Extract the RED Value from the RGBVAL using OLEDRGB Extract function
u8 Extract_R(u16 wRGB)
{
	return OLEDrgb_ExtractRFromRGB(wRGB<<3);
}

//Extract the GREEN Value from the RGBVAL using OLEDRGB Extract function
u8 Extract_G(u16 wRGB)
{
	return OLEDrgb_ExtractGFromRGB(wRGB<<2);
}

//Extract the BLUE Value from the RGBVAL using OLEDRGB Extract function
u8 Extract_B(u16 wRGB)
{
	return OLEDrgb_ExtractBFromRGB(wRGB<<3);
}

//Write the Duty Cycle for R, G, B to Seven segment Display
void WRITETOSEVENSEGMENT(uint8_t RED_CYCLE,uint8_t GREEN_CYCLE,uint8_t BLUE_CYCLE)
{
	NX4IO_SSEG_setDigit(SSEGHI,DIGIT7,(RED_CYCLE/10));
	NX4IO_SSEG_setDigit(SSEGHI,DIGIT6,(RED_CYCLE%10));

	NX4IO_SSEG_setDigit(SSEGHI,DIGIT4,(GREEN_CYCLE/10));
	NX4IO_SSEG_setDigit(SSEGLO,DIGIT3,(GREEN_CYCLE%10));

	NX4IO_SSEG_setDigit(SSEGLO,DIGIT1,(BLUE_CYCLE/10));
	NX4IO_SSEG_setDigit(SSEGLO,DIGIT0,(BLUE_CYCLE%10));
	usleep(5000);
}

//Reset the Seven Segment Display Decimal points
void RESET_SEVEN_SEG()
{
	NX4IO_SSEG_setDecPt(SSEGLO,DIGIT7,false);
	NX4IO_SSEG_setDecPt(SSEGLO,DIGIT6,false);
	NX4IO_SSEG_setDecPt(SSEGLO,DIGIT5,false);
	NX4IO_SSEG_setDecPt(SSEGLO,DIGIT4,false);
	NX4IO_SSEG_setDecPt(SSEGHI,DIGIT3,false);
	NX4IO_SSEG_setDecPt(SSEGHI,DIGIT2,false);
	NX4IO_SSEG_setDecPt(SSEGHI,DIGIT1,false);
	NX4IO_SSEG_setDecPt(SSEGHI,DIGIT0,false);

}











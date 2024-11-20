/* Increase the size of configMINIMAL_STACK_SIZE to avoid stack overflow. */

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* TI includes. */
#include <msp430.h>
#include "driverlib.h"

/* Arducam includes. */
#include "Arducam.h"
#include "spi.h"
#include "uart.h"

uint8_t temp = 0xff;
uint8_t commandBuff[20] = { 0 };
uint8_t commandLength = 0;
uint32_t readImageLength = 0;
uint8_t jpegHeadFlag = 0;
ArducamCamera myCAM;

#ifndef USING_ARDUCAM_UI
#pragma PERSISTENT (YUVarr)
uint8_t YUVarr[96 * 96] = { 0 };
#endif

/* The heap is allocated here so the "persistent" qualifier can be used.  This
 requires configAPPLICATION_ALLOCATED_HEAP to be set to 1 in FreeRTOSConfig.h.
 See http://www.freertos.org/a00111.html for more information. */
#ifdef __ICC430__
    __persistent                    /* IAR version. */
#else
#pragma PERSISTENT( ucHeap )    /* CCS version. */
#endif
uint8_t ucHeap[configTOTAL_HEAP_SIZE] = { 0 };

/* Used for maintaining a 32-bit run time stats counter from a 16-bit timer. */
volatile uint32_t ulRunTimeCounterOverflows = 0;

void initGPIO(void);
void initClockTo16MHz(void);

static void sem_task(void *pvParameters);
uint8_t read_buf_callback(uint8_t *imagebuf, uint8_t length);
void stop_preview_callback();

TaskHandle_t xHandle = NULL;

int main(void)
{
	/* Stop Watchdog timer. */
	WDT_A_hold( __MSP430_BASEADDRESS_WDT_A__);
	initGPIO();
	initClockTo16MHz();
	initUART();

	EUSCI_A_UART_enable(EUSCI_A0_BASE);
	EUSCI_A_UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);

	myCAM = createArducamCamera(CS_PIN);

	begin(&myCAM);
	registerCallback(&myCAM, read_buf_callback, 50, stop_preview_callback);
	__enable_interrupt();

	/*
	 #ifndef USING_ARDUCAM_UI_VIDEO
	 lowPowerOn(&myCAM);
	 LPM0;
	 __no_operation();
	 lowPowerOff(&myCAM);
	 //printf("Preparing camera from LPM...\n");
	 arducamDelayMs(1000);
	 #endif
	 */

//    uint8_t CameraResolution = myCAM.currentPictureMode;
//    uint8_t CameraFarmat = myCAM.currentPixelFormat;
//    CameraResolution = 0x3A & 0x0f;     // Set picture resolution (0x3A, 3:YUYV format, A: 96*96)
//    CameraFarmat = (0x3A & 0x70) >> 4;
//    arducamDelayMs(1000);
//    takePicture(&myCAM,(CAM_IMAGE_MODE)CameraResolution, (CAM_IMAGE_PIX_FMT)CameraFarmat);
//    cameraGetPicture(&myCAM);

	xTaskCreate(sem_task, /* The function that implements the task. */
				"t", /* The text name assigned to the task - for debug only as it is not used by the kernel. */
				200, /* The size of the stack to allocate to the task. */
				NULL, /* The parameter passed to the task - not used in this case. */
				1, /* The priority assigned to the task. */
				&xHandle); /* Used to pass out the created task's handle. */

	vTaskStartScheduler(); /* Start the tasks and timer running. */
	return 0;
}

static void sem_task(void *pvParameters)
{
	while (1)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		arducamDelayMs(5);
		while (arducamUartAvailable())
		{
			commandBuff[commandLength] = arducamUartRead();
			if (commandBuff[commandLength] == 0xAA)
			{
				break;
			}
			commandLength++;
		}
		arducamFlush();
		commandProcessing(&myCAM, commandBuff, commandLength);
		commandLength = 0;
	}
}

/* Arducam register callback functions. */

uint8_t read_buf_callback(uint8_t *imagebuf, uint8_t length)
{
	if (imagebuf[0] == 0xff && imagebuf[1] == 0xd8)
	{
		jpegHeadFlag = 1;
		readImageLength = 0;
		arducamUartWrite(0xff);
		arducamUartWrite(0xAA);
		arducamUartWrite(0x01);

		arducamUartWrite((uint8_t) (myCAM.totalLength & 0xff));
		arducamUartWrite((uint8_t) ((myCAM.totalLength >> 8) & 0xff));
		arducamUartWrite((uint8_t) ((myCAM.totalLength >> 16) & 0xff));
		arducamUartWrite((uint8_t) ((myCAM.receivedLength >> 24) & 0xff));
		arducamUartWrite(((CAM_IMAGE_PIX_FMT_JPG & 0x0f) << 4) | 0x01);
	}

	if (jpegHeadFlag == 1)
	{
		readImageLength += length;
		for (uint8_t i = 0; i < length; i++)
		{
			arducamUartWrite(imagebuf[i]);
		}
	}

	if (readImageLength == myCAM.totalLength)
	{
		jpegHeadFlag = 0;
		arducamUartWrite(0xff);
		arducamUartWrite(0xBB);
	}
	return 1;
}

void stop_preview_callback()
{
	readImageLength = 0;
	jpegHeadFlag = 0;
	uint32_t len = 9;

	arducamUartWrite(0xff);
	arducamUartWrite(0xBB);
	arducamUartWrite(0xff);
	arducamUartWrite(0xAA);
	arducamUartWrite(0x06);
	uartWriteBuffer((uint8_t*) &len, 4);
	//printf("streamoff");
	arducamUartWrite(0xff);
	arducamUartWrite(0xBB);
}

/* Board initialize function. */

void initGPIO(void)
{
	/* Set all GPIO pins to output and low. */
	P1DIR = 0xFF;
	P1OUT = 0x00;
	P2DIR = 0xFF;
	P2OUT = 0x00;
	P3DIR = 0xFF;
	P3OUT = 0x00;
	P4DIR = 0xFF;
	P4OUT = 0x00;
	P5DIR = 0xFF;
	P5OUT = 0x00;
	P6DIR = 0xFF;
	P6OUT = 0x00;
	P7DIR = 0xFF;
	P7OUT = 0x00;
	P8DIR = 0xFF;
	P8OUT = 0x00;
	P9DIR = 0xFF;
	P9OUT = 0x00;
	PADIR = 0xFF;
	PAOUT = 0x00;
	PBDIR = 0xFF;
	PBOUT = 0x00;
	PCDIR = 0xFF;
	PCOUT = 0x00;
	PDDIR = 0xFF;
	PDOUT = 0x00;
	PEDIR = 0xFF;
	PEOUT = 0x00;
	PJDIR = 0xFF;
	PJOUT = 0x00;

	/* Configure eUSCI_A0 UART operation. */
	GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2, GPIO_PIN0,
	GPIO_SECONDARY_MODULE_FUNCTION);
	GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN1,
	GPIO_SECONDARY_MODULE_FUNCTION);

	/* Configure SPI and CS GPIO. */
	P5SEL1 &= ~(BIT0 | BIT1 | BIT2);        // USCI_B1 SCLK, MOSI,
	P5SEL0 |= (BIT0 | BIT1 | BIT2);         // and MISO pin (primary_module)

	GPIO_setAsOutputPin(GPIO_PORT_P5, GPIO_PIN3); // CS P5.3

	/* Disable the GPIO power-on default high-impedance mode. */
	PMM_unlockLPM5();
}

void initClockTo16MHz(void) // SMCLK = MCLK = DCO, ACLK = VLOCLK
{
	// Configure one FRAM waitstate as required by the device datasheet for MCLK
	// operation beyond 8MHz _before_ configuring the clock system.
	FRCTL0 = FRCTLPW | NWAITS_1;

	// Clock System Setup
	CSCTL0_H = CSKEY_H;                     // Unlock CS registers
	CSCTL1 = DCOFSEL_0;                     // Set DCO to 1MHz

	// Set SMCLK = MCLK = DCO, ACLK = VLOCLK
	CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;

	// Per Device Errata set divider to 4 before changing frequency to
	// prevent out of spec operation from overshoot transient
	CSCTL3 = DIVA__4 | DIVS__4 | DIVM__4; // Set all corresponding clk sources to divide by 4 for errata
	CSCTL1 = DCOFSEL_4 | DCORSEL;           // Set DCO to 16MHz

	// Delay by ~10us to let DCO settle. 60 cycles = 20 cycles buffer + (10us / (1/4MHz))
	__delay_cycles(60);
	CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1; // Set all dividers to 1 for 16MHz operation
	CSCTL0_H = 0;                           // Lock CS registers
}

/* freeRTOS hook functions. */

void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	 free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	 internally by FreeRTOS API functions that create tasks, queues, software
	 timers, and semaphores.  The size of the FreeRTOS heap is set by the
	 configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	printf("Malloc Failed.\n");
	configASSERT(( volatile void * ) NULL);
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
	(void) pcTaskName;
	(void) pxTask;

	/* Run time stack overflow checking is performed if
	 configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	 function is called if a stack overflow is detected.
	 See http://www.freertos.org/Stacks-and-stack-overflow-checking.html */

	/* Force an assert. */
	printf("Stack Overflowed.\n");
	configASSERT(( volatile void * ) NULL);
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook(void)
{
	__bis_SR_register( LPM0_bits + GIE);
	__no_operation();
}

/*-----------------------------------------------------------*/

/* The MSP430X port uses this callback function to configure its tick interrupt.
 This allows the application to choose the tick interrupt source.
 configTICK_VECTOR must also be set in FreeRTOSConfig.h to the correct
 interrupt vector for the chosen tick interrupt source.  This implementation of
 vApplicationSetupTimerInterrupt() generates the tick from timer A0, so in this
 case configTICK_VECTOR is set to TIMER0_A0_VECTOR. */
void vApplicationSetupTimerInterrupt(void)
{
//	const uint32_t SMCLK_Frequency_Hz = 16000000;
//
//	/* Ensure the timer is stopped. */
//	TA0CTL = 0;
//
//	/* Run the timer from the SMCLK. */
//	TA0CTL = TASSEL_2;
//
//	/* Clear everything to start with. */
//	TA0CTL |= TACLR;
//
//	/* Set the compare match value according to the tick rate we want. */
//	TA0CCR0 = SMCLK_Frequency_Hz / configTICK_RATE_HZ;
//
//	/* Enable the interrupts. */
//	TA0CCTL0 = CCIE;
//
//	/* Start up clean. */
//	TA0CTL |= TACLR;
//
//	/* Up mode. */
//	TA0CTL |= MC_1;

	const unsigned short usACLK_Frequency_Hz = 32768;

		/* Ensure the timer is stopped. */
		TA0CTL = 0;

		/* Run the timer from the ACLK. */
		TA0CTL = TASSEL_1;

		/* Clear everything to start with. */
		TA0CTL |= TACLR;

		/* Set the compare match value according to the tick rate we want. */
		TA0CCR0 = usACLK_Frequency_Hz / configTICK_RATE_HZ;

		/* Enable the interrupts. */
		TA0CCTL0 = CCIE;

		/* Start up clean. */
		TA0CTL |= TACLR;

		/* Up mode. */
		TA0CTL |= MC_1;
}
/*-----------------------------------------------------------*/

void vConfigureTimerForRunTimeStats(void)
{
	/* Configure a timer that is used as the time base for run time stats.  See
	 http://www.freertos.org/rtos-run-time-stats.html */

	/* Ensure the timer is stopped. */
	TA0CTL = 0;

	/* Start up clean. */
	TA0CTL |= TACLR;

	/* Run the timer from the ACLK/8, continuous mode, interrupt enable. */
	TA0CTL = TASSEL_1 | ID__8 | MC__CONTINUOUS | TAIE;
}

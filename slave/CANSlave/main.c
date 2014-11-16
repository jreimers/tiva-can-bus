/*
 * CAN LED Slave
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"


volatile unsigned long g_ulMsgCount = 0; // msg count

volatile bool g_bRXFlag = 0; // msg recieved flag

volatile bool g_bErrFlag = 0; // error flag

#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3


// CAN interrupt handler
void CANIntHandler(void)
{
    unsigned long ulStatus = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);

    // If the cause is a controller status interrupt, then get the status
    if(ulStatus == CAN_INT_INTID_STATUS)
    {
        ulStatus = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        UARTprintf("CAN Int, Status: %u\n", ulStatus);

        g_bErrFlag = 1;
    }
    else if(ulStatus == 1) // msg object 1
    {
        CANIntClear(CAN0_BASE, 1); // clear interrupt

        g_ulMsgCount++;
        g_bRXFlag = 1;
        g_bErrFlag = 0;
    }
}

int main(void)
{
	tCANMsgObject sCANMessage;
	unsigned char ucMsgData[8];

    // Set the clocking to run directly from the crystal.
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Set up debugging UART
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTStdioConfig(0, 115200, SysCtlClockGet());

   	// Set up onboard LEDs
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);

    // Set up CAN0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinConfigure(GPIO_PE4_CAN0RX);
	GPIOPinConfigure(GPIO_PE5_CAN0TX);
	GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);
	CANInit(CAN0_BASE);
	CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 500000);
	CANIntRegister(CAN0_BASE, CANIntHandler); // use dynamic vector table allocation
	CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
	IntEnable(INT_CAN0);
	CANEnable(CAN0_BASE);

	//
	// Initialize a message object to be used for receiving CAN messages with
	// any CAN ID.  In order to receive any CAN ID, the ID and mask must both
	// be set to 0, and the ID filter enabled.
	//
	sCANMessage.ui32MsgID = 0;                        // CAN msg ID - 0 for any
	sCANMessage.ui32MsgIDMask = 0;                    // mask is 0 for any ID
	sCANMessage.ui32Flags = MSG_OBJ_RX_INT_ENABLE | MSG_OBJ_USE_ID_FILTER;
	sCANMessage.ui32MsgLen = 8;                       // allow up to 8 bytes

	//
	// Now load the message object into the CAN peripheral.  Once loaded the
	// CAN will receive any message on the bus, and an interrupt will occur.
	// Use message object 1 for receiving messages (this is not the same as
	// the CAN ID which can be any value in this example).
	//
	CANMessageSet(CAN0_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_RX);

	UARTprintf("Clock: %u\nListening for messages...\n", SysCtlClockGet());

	while(1)
	{
		unsigned int uIdx;

		if(g_bRXFlag)
		{

			sCANMessage.pui8MsgData = ucMsgData;
			CANMessageGet(CAN0_BASE, 1, &sCANMessage, 0);

			g_bRXFlag = 0;

			if(sCANMessage.ui32Flags & MSG_OBJ_DATA_LOST)
			{
				UARTprintf("CAN message loss detected\n");
			}

			long out = 0;
			int dataIdx;
			for(dataIdx=0;dataIdx<4;dataIdx++) {
				out |= (((long)ucMsgData[dataIdx])<<(dataIdx*8));
			}
			UARTprintf("Msg ID=0x%08X len=%u data=%d\n", sCANMessage.ui32MsgID, sCANMessage.ui32MsgLen, out);
			UARTprintf("total count=%u\n", g_ulMsgCount);
			if(out % 2 == 0) {
				GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, RED_LED|BLUE_LED);
			} else {
				GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, BLUE_LED|GREEN_LED);
			}
		}
	}

	return 0;
}

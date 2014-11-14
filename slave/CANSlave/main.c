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

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Hello World (hello)</h1>
//!
//! A very simple ``hello world'' example.  It simply displays ``Hello World!''
//! on the UART and is a starting point for more complicated applications.
//!
//! UART0, connected to the Virtual Serial Port and running at
//! 115,200, 8-N-1, is used to display messages from this application.
//
//*****************************************************************************

//*****************************************************************************
//
// A counter that keeps track of the number of times the RX interrupt has
// occurred, which should match the number of messages that were received.
//
//*****************************************************************************
volatile unsigned long g_ulMsgCount = 0;

//*****************************************************************************
//
// A flag for the interrupt handler to indicate that a message was received.
//
//*****************************************************************************
volatile unsigned long g_bRXFlag = 0;

//*****************************************************************************
//
// A flag to indicate that some reception error occurred.
//
//*****************************************************************************
volatile unsigned long g_bErrFlag = 0;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
InitConsole(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

//*****************************************************************************
//
// This function is the interrupt handler for the CAN peripheral.  It checks
// for the cause of the interrupt, and maintains a count of all messages that
// have been received.
//
//*****************************************************************************
void
CANIntHandler(void)
{
    unsigned long ulStatus;


    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ulStatus = CANIntStatus(CAN0_BASE, CAN_INT_STS_CAUSE);



    //
    // If the cause is a controller status interrupt, then get the status
    //
    if(ulStatus == CAN_INT_INTID_STATUS)
    {
        //
        // Read the controller status.  This will return a field of status
        // error bits that can indicate various errors.  Error processing
        // is not done in this example for simplicity.  Refer to the
        // API documentation for details about the error status bits.
        // The act of reading this status will clear the interrupt.
        //
        ulStatus = CANStatusGet(CAN0_BASE, CAN_STS_CONTROL);

        UARTprintf("CAN Int, Status: %u\n", ulStatus);
        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // receiving messages.
    //
    else if(ulStatus == 1)
    {
        //
        // Getting to this point means that the RX interrupt occurred on
        // message object 1, and the message reception is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN0_BASE, 1);

        //
        // Increment a counter to keep track of how many messages have been
        // received.  In a real application this could be used to set flags to
        // indicate when a message is received.
        //
        g_ulMsgCount++;

        //
        // Set flag to indicate received message is pending.
        //
        g_bRXFlag = 1;

        //
        // Since a message was received, clear any error flags.
        //
        g_bErrFlag = 0;
    }

    //
    // Otherwise, something unexpected caused the interrupt.  This should
    // never happen.
    //
    else
    {
        //
        // Spurious interrupt handling can go here.
        //
    }
}

//*****************************************************************************
//
// Print "Hello World!" to the UART on the evaluation board.
//
//*****************************************************************************
int
main(void)
{
	 tCANMsgObject sCANMessage;
	 unsigned char ucMsgData[8];

    //volatile uint32_t ui32Loop;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                           SYSCTL_OSC_MAIN);
    //
    // Initialize the UART.
    //
    InitConsole();

    //
	// For this example CAN0 is used with RX and TX pins on port D0 and D1.
	// The actual port and pins used may be different on your part, consult
	// the data sheet for more information.
	// GPIO port D needs to be enabled so these pins can be used.
	// TODO: change this to whichever GPIO port you are using
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	// LEDs

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED);

	//
	// Configure the GPIO pin muxing to select CAN0 functions for these pins.
	// This step selects which alternate function is available for these pins.
	// This is necessary if your part supports GPIO pin function muxing.
	// Consult the data sheet to see which functions are allocated per pin.
	// TODO: change this to select the port/pin you are using
	//
	GPIOPinConfigure(GPIO_PE4_CAN0RX);
	GPIOPinConfigure(GPIO_PE5_CAN0TX);

	//
	// Enable the alternate function on the GPIO pins.  The above step selects
	// which alternate function is available.  This step actually enables the
	// alternate function instead of GPIO for these pins.
	// TODO: change this to match the port/pin you are using
	//
	GPIOPinTypeCAN(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

	//
	// The GPIO port and pins have been set up for CAN.  The CAN peripheral
	// must be enabled.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);

	//
	// Initialize the CAN controller
	//
	CANInit(CAN0_BASE);

	//
	// Set up the bit rate for the CAN bus.  This function sets up the CAN
	// bus timing for a nominal configuration.  You can achieve more control
	// over the CAN bus timing by using the function CANBitTimingSet() instead
	// of this one, if needed.
	// In this example, the CAN bus is set to 500 kHz.  In the function below,
	// the call to SysCtlClockGet() is used to determine the clock rate that
	// is used for clocking the CAN peripheral.  This can be replaced with a
	// fixed value if you know the value of the system clock, saving the extra
	// function call.  For some parts, the CAN peripheral is clocked by a fixed
	// 8 MHz regardless of the system clock in which case the call to
	// SysCtlClockGet() should be replaced with 8000000.  Consult the data
	// sheet for more information about CAN peripheral clocking.
	//
	CANBitRateSet(CAN0_BASE, ROM_SysCtlClockGet(), 500000);

	//
	// Enable interrupts on the CAN peripheral.  This example uses static
	// allocation of interrupt handlers which means the name of the handler
	// is in the vector table of startup code.  If you want to use dynamic
	// allocation of the vector table, then you must also call CANIntRegister()
	// here.
	//
	CANIntRegister(CAN0_BASE, CANIntHandler); // if using dynamic vectors
	//
	CANIntEnable(CAN0_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

	//
	// Enable the CAN interrupt on the processor (NVIC).
	//
	IntEnable(INT_CAN0);

	//
	// Enable the CAN for operation.
	//
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

	UARTprintf("Clock: %u\nListening for messages...\n", ROM_SysCtlClockGet());

	//
	// Enter loop to process received messages.  This loop just checks a flag
	// that is set by the interrupt handler, and if set it reads out the
	// message and displays the contents.  This is not a robust method for
	// processing incoming CAN data and can only handle one messages at a time.
	// If many messages are being received close together, then some messages
	// may be dropped.  In a real application, some other method should be used
	// for queuing received messages in a way to ensure they are not lost.  You
	// can also make use of CAN FIFO mode which will allow messages to be
	// buffered before they are processed.
	//
	for(;;)
	{
		unsigned int uIdx;

		//
		// If the flag is set, that means that the RX interrupt occurred and
		// there is a message ready to be read from the CAN
		//
		if(g_bRXFlag)
		{
			//
			// Reuse the same message object that was used earlier to configure
			// the CAN for receiving messages.  A buffer for storing the
			// received data must also be provided, so set the buffer pointer
			// within the message object.
			//
			sCANMessage.pui8MsgData = ucMsgData;

			//
			// Read the message from the CAN.  Message object number 1 is used
			// (which is not the same thing as CAN ID).  The interrupt clearing
			// flag is not set because this interrupt was already cleared in
			// the interrupt handler.
			//
			CANMessageGet(CAN0_BASE, 1, &sCANMessage, 0);

			//
			// Clear the pending message flag so that the interrupt handler can
			// set it again when the next message arrives.
			//
			g_bRXFlag = 0;

			//
			// Check to see if there is an indication that some messages were
			// lost.
			//
			if(sCANMessage.ui32Flags & MSG_OBJ_DATA_LOST)
			{
				UARTprintf("CAN message loss detected\n");
			}

			long out = 0;
			int dataIdx;
			for(dataIdx=0;dataIdx<4;dataIdx++) {
				out |= (((long)ucMsgData[dataIdx])<<(dataIdx*8));
			}

			//
			// Print out the contents of the message that was received.
			//
			UARTprintf("Msg ID=0x%08X len=%u data=%d\n", sCANMessage.ui32MsgID, sCANMessage.ui32MsgLen, out);
			UARTprintf("total count=%u\n", g_ulMsgCount);

			if(out % 2 == 0) {
				GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, RED_LED|BLUE_LED);
			} else {
				GPIOPinWrite(GPIO_PORTF_BASE, RED_LED|BLUE_LED|GREEN_LED, BLUE_LED|GREEN_LED);
			}
		}
	}

	//
	// Return no errors
	//
	return(0);
}

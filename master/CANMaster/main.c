#include <stdbool.h>
#include <stdint.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_can.h"
#include "inc/hw_ints.h"
#include "driverlib/can.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "utils/uartstdio.h"

//*****************************************************************************
//
// A counter that keeps track of the number of times the TX interrupt has
// occurred, which should match the number of TX messages that were sent.
//
//*****************************************************************************
volatile uint32_t g_ui32MsgCount = 0;

//*****************************************************************************
//
// A flag to indicate that some transmission error occurred.
//
//*****************************************************************************
volatile bool g_bErrFlag = 0;


//*****************************************************************************
//
// System clock rate in Hz.
//
//*****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
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
	// Initialize the UART for console I/O.
	//
	UARTStdioConfig(0, 115200, g_ui32SysClock);
}

//*****************************************************************************
//
// This function provides a 1 second delay using a simple polling method.
//
//*****************************************************************************
void
SimpleDelay(void)
{
    //
    // Delay cycles for 1 second
    //
    SysCtlDelay(g_ui32SysClock / 3);
}

//*****************************************************************************
//
// This function is the interrupt handler for the CAN peripheral.  It checks
// for the cause of the interrupt, and maintains a count of all messages that
// have been transmitted.
//
//*****************************************************************************
void
CANIntHandler(void)
{
	UARTprintf("CANIntHandler\n");
    unsigned long ulStatus;

    //
    // Read the CAN interrupt status to find the cause of the interrupt
    //
    ulStatus = CANIntStatus(CAN1_BASE, CAN_INT_STS_CAUSE);

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
        // The act of reading this status will clear the interrupt.  If the
        // CAN peripheral is not connected to a CAN bus with other CAN devices
        // present, then errors will occur and will be indicated in the
        // controller status.
        //
        ulStatus = CANStatusGet(CAN1_BASE, CAN_STS_CONTROL);

        //
        // Set a flag to indicate some errors may have occurred.
        //
        g_bErrFlag = 1;
    }

    //
    // Check if the cause is message object 1, which what we are using for
    // sending messages.
    //
    else if(ulStatus == 1)
    {
        //
        // Getting to this point means that the TX interrupt occurred on
        // message object 1, and the message TX is complete.  Clear the
        // message object interrupt.
        //
        CANIntClear(CAN1_BASE, 1);

        //
        // Increment a counter to keep track of how many messages have been
        // sent.  In a real application this could be used to set flags to
        // indicate when a message is sent.
        //
        g_ui32MsgCount++;

        //
        // Since the message was sent, clear any error flags.
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
    	UARTprintf("Unexpected CAN bus interrupt\n");
    }
}

//*****************************************************************************
//
// Configure the CAN and enter a loop to transmit periodic CAN messages.
//
//*****************************************************************************
int
main(void)
{
	tCANMsgObject sCANMessage;
	uint32_t ui32MsgData;
	uint8_t *pui8MsgData;

	pui8MsgData = (uint8_t *)&ui32MsgData;

    //
    // Set the clocking to run directly from the external crystal/oscillator.
    // TODO: The SYSCTL_XTAL_ value must be changed to match the value of the
    // crystal on your board.
    //
//    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
//                   SYSCTL_XTAL_16MHZ);

    //
       // Run from the PLL at 120 MHz.
       //
       g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                   SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                   SYSCTL_CFG_VCO_480), 120000000);

       //
       // Configure the device pins.
       //
       //PinoutSet(false, false);

    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for CAN operation.
    //
    InitConsole();

    //
    // For this example CAN0 is used with RX and TX pins on port D0 and D1.
    // The actual port and pins used may be different on your part, consult
    // the data sheet for more information.
    // GPIO port D needs to be enabled so these pins can be used.
    // TODO: change this to whichever GPIO port you are using
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    //
    // Configure the GPIO pin muxing to select CAN0 functions for these pins.
    // This step selects which alternate function is available for these pins.
    // This is necessary if your part supports GPIO pin function muxing.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using
    //
    GPIOPinConfigure(GPIO_PB0_CAN1RX);
    GPIOPinConfigure(GPIO_PB1_CAN1TX);

    //
    // Enable the alternate function on the GPIO pins.  The above step selects
    // which alternate function is available.  This step actually enables the
    // alternate function instead of GPIO for these pins.
    // TODO: change this to match the port/pin you are using
    //
    GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // The GPIO port and pins have been set up for CAN.  The CAN peripheral
    // must be enabled.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);

    //
    // Initialize the CAN controller
    //
    CANInit(CAN1_BASE);

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
    CANBitRateSet(CAN1_BASE, g_ui32SysClock, 500000);

    //
    // Enable interrupts on the CAN peripheral.  This example uses static
    // allocation of interrupt handlers which means the name of the handler
    // is in the vector table of startup code.  If you want to use dynamic
    // allocation of the vector table, then you must also call CANIntRegister()
    // here.
    //
    CANIntRegister(CAN1_BASE, CANIntHandler); // if using dynamic vectors
    //
    CANIntEnable(CAN1_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);

    //
    // Enable the CAN interrupt on the processor (NVIC).
    //
    IntEnable(INT_CAN1);

    //
    // Enable the CAN for operation.
    //
    CANEnable(CAN1_BASE);

    ui32MsgData = 0;
	sCANMessage.ui32MsgID = 1;
	sCANMessage.ui32MsgIDMask = 0;
	sCANMessage.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
	sCANMessage.ui32MsgLen = sizeof(pui8MsgData);
	sCANMessage.pui8MsgData = pui8MsgData;

	//
	// Enter loop to send messages.  A new message will be sent once per
	// second.  The 4 bytes of message content will be treated as an uint32_t
	// and incremented by one each time.
	//
	while(1)
	{
		UARTprintf("%u\n", g_ui32SysClock);
		//
		// Print a message to the console showing the message count and the
		// contents of the message being sent.
		//
		UARTprintf("Sending msg: 0x%02X %02X %02X %02X\n",
				   pui8MsgData[0], pui8MsgData[1], pui8MsgData[2],
				   pui8MsgData[3]);

		//
		// Send the CAN message using object number 1 (not the same thing as
		// CAN ID, which is also 1 in this example).  This function will cause
		// the message to be transmitted right away.
		//
		CANMessageSet(CAN1_BASE, 1, &sCANMessage, MSG_OBJ_TYPE_TX);

		//
		// Now wait 1 second before continuing
		//
		SimpleDelay();

		//
		// Check the error flag to see if errors occurred
		//
		if(g_bErrFlag)
		{
			UARTprintf(" error - cable connected?\n");
		}
		else
		{
			//
			// If no errors then print the count of message sent
			//
			UARTprintf(" total count = %u\n", g_ui32MsgCount);
		}

		//
		// Increment the value in the message data.
		//
		ui32MsgData++;
	}

    //
    // Return no errors
    //
    return(0);
}


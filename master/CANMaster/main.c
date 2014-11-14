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

volatile bool errFlag = 0; // transmission error flag
unsigned int sysClock; // clockspeed in hz

void Delay(unsigned int seconds) {
	SysCtlDelay((sysClock / 3) * seconds);
}

// CAN interrupt handler
void CANIntHandler(void) {

	unsigned long status = CANIntStatus(CAN1_BASE, CAN_INT_STS_CAUSE); // read interrupt status

	if(status == CAN_INT_INTID_STATUS) { // controller status interrupt
		status = CANStatusGet(CAN1_BASE, CAN_STS_CONTROL); // read back error bits, do something with them?
		errFlag = 1;
	} else if(status == 1) { // message object 1
		CANIntClear(CAN1_BASE, 1); // clear interrupt
		errFlag = 0; // clear any error flags
	} else { // should never happen
		UARTprintf("Unexpected CAN bus interrupt\n");
	}
}

int main(void) {

	tCANMsgObject msg; // the CAN message object
	unsigned int msgData; // the message data
	unsigned char *msgDataPtr;

	msgDataPtr = (unsigned char *)&msgData;

	// Run from the PLL at 120 MHz.
	sysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480), 120000000);

	// Set up debugging UART
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // enable UART0 GPIO peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTStdioConfig(0, 115200, sysClock); // 115200 baud

	// Set up CAN1
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB); // enable CAN1 GPIO peripheral
	GPIOPinConfigure(GPIO_PB0_CAN1RX);
	GPIOPinConfigure(GPIO_PB1_CAN1TX);
	GPIOPinTypeCAN(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN1);
	CANInit(CAN1_BASE);
	CANBitRateSet(CAN1_BASE, sysClock, 500000);
	CANIntRegister(CAN1_BASE, CANIntHandler); // use dynamic vector table allocation
	CANIntEnable(CAN1_BASE, CAN_INT_MASTER | CAN_INT_ERROR | CAN_INT_STATUS);
	IntEnable(INT_CAN1);
	CANEnable(CAN1_BASE);


	// Set up msg object
	msgData = 0;
	msg.ui32MsgID = 1;
	msg.ui32MsgIDMask = 0;
	msg.ui32Flags = MSG_OBJ_TX_INT_ENABLE;
	msg.ui32MsgLen = sizeof(msgDataPtr);
	msg.pui8MsgData = msgDataPtr;

	while(1) {
		
		UARTprintf("Sending message: %d\n", msgData); // write the msg to UART for debugging
		CANMessageSet(CAN1_BASE, 1, &msg, MSG_OBJ_TYPE_TX); // send as msg object 1

		Delay(1); // wait 1 second

		if(errFlag) { // check for errors
			UARTprintf("CAN Bus Error\n");
		}

		msgData++; // increment msg data
	}

	return 0;
}


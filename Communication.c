/*
 * File Communication.c
 * version 1.0
 *
 * NAME         : CHETHAN NARAYANASWAMY
 * STUDENT ID#     : 1001103300
 * To build a node capable of operations on an RS 485 based bidirectional  peer to peer communications system.
 *
 */
/*===================================================================================================================
                                            HARDWARE DESCRIPTION
====================================================================================================================*/

/*Target Platform: EK-TM4C123GXL Evaluation Board
 * Target uC:       TM4C123GH6PM
 * System Clock:    40 MHz
 *
 *
 * Hardware configuration:
 * Red LED: PF1 drives an NPN transistor that powers the red LED
 * Blue LED: PF2 drives an NPN transistor that powers the blue LED
 * Green LED: PF3 drives an NPN transistor that powers the green LED
 *
 *
 * UART Interface:
 * U0TX (PA1) and U0RX (PA0) are connected to the 2nd controller
 * The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port
 * Configured to 115,200 baud, 8N1
 */
/*===================================================================================================================
                                    DEVICE INCLUDES AND ASSEMBLER DIRECTIVES
====================================================================================================================*/
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include "inc\tm4c123gh6pm.h"
#include "driverlib\eeprom.h"
#include "driverlib\sysctl.h"
/*===================================================================================================================
                                                DEFINES
====================================================================================================================*/

#define RED_LED         (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4)))
#define BLUE_LED        (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4)))
#define GREEN_LED       (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))
#define ERROR           (uint16_t)-1
#define DEN 			(*((volatile uint32_t *)(0x42000000 + (0x400063FC-0x40000000)*32 + 6*4)))
#define PUSH_BUTTON  	(*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4)))
#define MAX 			5
/*===================================================================================================================
                                                 GLOBAL VARIABLES
=====================================================================================================================*/
char Rx_Str_Buffer[81];
uint16_t Rx_Str_Count = 0;
uint16_t Rx_Str_Arguments_Count = 48;
char Rx_Str_type[10];
uint16_t Rx_Str_Pos[10];
int Tx_Seq_ID = -1;
uint8_t Src_Add = 0;
int Phase = -1;
uint8_t Command = 0, Dest_Add = 0, Channel = 0, Size = 0, Data = 0;
uint16_t Calculate_CheckSum = 0x0;
uint8_t Rx_CheckSum = 0;
uint8_t Rx_Data[15];
bool Ack_Received = false;
uint8_t Red_Timeout = 0;
uint8_t Green_Timeout = 0;
uint8_t Dest_Add_Tx[10], TxSequenceID_Tx[10], Command_Tx[10], Channel_Tx[10], Size_Tx[10], Data_Tx[10][10];
//uint16_t Calculate_CheckSum_Tx[10];
uint8_t Tx_Requested[10], Tx_Valid[10], Tx_Ack_Required[10], Tx_Timeout[10], Tx_Retry_Count[10];
uint8_t WriteIndex = 0, ReadIndex = 0;
uint8_t MsgPhase = 0;
int MsgInProgress = -1;
uint8_t Tx_Seq_Rx_Data = 0;
uint8_t Push_Button_Pressed;
uint32_t EEPROM_Write_Value_Data[4] = {0x00,0x00,0x00,0x00};
uint32_t EEPROM_Read_Value_Data[4];
uint32_t EEPROM_Value_Address = 0x400af080;
uint32_t EEPROM_Return_Value = EEPROM_INIT_ERROR;
/*==================================================================================================================
 *                                               GLOBAL FUNCTIONS
====================================================================================================================*/


/*==================================================================================================================
 *                                               LOCAL FUNCTIONS
====================================================================================================================*/
/*==================================================================================================================
 * INITIALIZE THE HARDWARE
 *
 * 1. Configure the system to work with 16MHz crystal, PLL enabled and system clock of 40MHZ.
 * 2. Configure GPIO ports to use APB.
 * 3. Enable the GPIO ports.
 * 4. Configure the PORT F to make LED work.
 * 5. Configure the UART0 with 115200 baud and  8N1 format (must be 3 clocks from clock enable and config writes).
 * 6. Configure UART1 to 38400 baud, 8x1 format (must be 3 clocks from clock enable and config writes)
 * 7. Configure Timer 1 as the time base.
====================================================================================================================*/
void initHw()
{
	/* Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz */
	SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

	/* Set GPIO ports to use APB (not needed since default configuration -- for clarity) */
	/* Note UART on port A must use APB */
	SYSCTL_GPIOHBCTL_R = 0;

	/* Enable GPIO port A, C and F peripherals */
	SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOC | SYSCTL_RCGC2_GPIOF;

	/* Configure LED and pushbutton pins */
	GPIO_PORTF_DIR_R = 0x0E;  /* bits 1, 2 and 3 are outputs, other pins are inputs */
	GPIO_PORTF_DR2R_R = 0x0A; /* set drive strength to 2mA (not needed since default configuration -- for clarity) */
	GPIO_PORTF_DR4R_R = 0x04; /* set drive strength to 4mA */
	GPIO_PORTF_DEN_R = 0x1E;  /* enable LEDs and pushbuttons */
	GPIO_PORTF_PUR_R = 0x10;  /* enable internal pull-up for push button */

	/* Configure UART0 pins */
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         /* turn-on UART0, leave other uarts in same status */
	GPIO_PORTA_DEN_R |= 3;                           /* default, added for clarity */
	GPIO_PORTA_AFSEL_R |= 3;                         /* default, added for clarity */
	GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

	/* Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes) */
	UART0_CTL_R = 0;                                 /* turn-off UART0 to allow safe programming */
	UART0_CC_R = UART_CC_CS_SYSCLK;                  /* use system clock (40 MHz) */
	UART0_IBRD_R = 21;                               /* r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16 */
	UART0_FBRD_R = 45;                               /* round(fract(r)*64)=45 */
	UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; /* configure for 8N1 w/ 16-level FIFO */
	UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; /* enable TX, RX, and module */

	/* Configure UART1 pins */
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R1;         /* turn-on UART1, leave other uarts in same status */
	GPIO_PORTC_DEN_R |= 0x70;                        /* Receive, transmit and DEN is turned on */
	GPIO_PORTC_DIR_R = 0x40;  						/* DEN bit is made output */
	GPIO_PORTC_AFSEL_R |= 0x30;                      /* Select alternate functionality */
	GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC5_M;
	GPIO_PORTC_PCTL_R &= ~GPIO_PCTL_PC4_M;
	GPIO_PORTC_PCTL_R |= GPIO_PCTL_PC5_U1TX | GPIO_PCTL_PC4_U1RX;

	/* Configure UART1 to 38400 baud, 8x1 format (must be 3 clocks from clock enable and config writes) */
	UART1_CTL_R = 0;                                 /* turn-off UART0 to allow safe programming */
	UART1_CC_R = UART_CC_CS_SYSCLK;                  /* use system clock (40 MHz) */
	UART1_IBRD_R = 65;                               /* r = 40 MHz / (Nx38400), set floor(r)=65, where N=16 */
	UART1_FBRD_R = 7;                               /* round(fract(r)*64)=7 */
	UART1_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_SPS | UART_LCRH_PEN; 	/* configure for 8x1, enable SPS and PEN */
	UART1_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN | UART_CTL_EOT; /* enable TX, RX, and module */
	UART1_IM_R = UART_IM_RXIM | UART_IM_TXIM;						/* Set interrupt mask bit for reception and transmission */
	NVIC_EN0_R = 1 << (22-16);						/* Enable UART 1 interrupt */

	/* Configure Timer 1 as the time base */
	SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;       /* turn-on timer */
	TIMER1_CTL_R &= ~TIMER_CTL_TAEN;                 /* turn-off timer before reconfiguring */
	TIMER1_CFG_R = TIMER_CFG_32_BIT_TIMER;           /* configure as 32-bit timer (A+B) */
	TIMER1_TAMR_R = TIMER_TAMR_TAMR_PERIOD;          /* configure for periodic mode (count down) */
	TIMER1_TAILR_R = 0x00061A80;                     /* set load value to 40e6 for 1 Hz interrupt rate */
	TIMER1_IMR_R = TIMER_IMR_TATOIM;                 /* turn-on interrupts */
	NVIC_EN0_R |= 1 << (INT_TIMER1A-16);             /* turn-on interrupt 37 (TIMER1A)*/
	TIMER1_CTL_R |= TIMER_CTL_TAEN;                  /* turn-on timer */

	SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
}

/*====================================================================================================================
 * Blocking function which waits till the push button is pressed.
======================================================================================================================*/
void waitPbPress()
{
	while(PUSH_BUTTON);
}

/*====================================================================================================================
 * Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
======================================================================================================================*/

void waitMicrosecond(uint32_t us)
{
	// Approx clocks per us
	__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
	__asm("WMS_LOOP1:   SUB  R1, #1");          // 6
	__asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
	__asm("             NOP");                  // 5
	__asm("             B    WMS_LOOP1");       // 5*3
	__asm("WMS_DONE1:   SUB  R0, #1");          // 1
	__asm("             CBZ  R0, WMS_DONE0");   // 1
	__asm("             B    WMS_LOOP0");       // 1*3
	__asm("WMS_DONE0:");                        // ---
	// 40 clocks/us + error
}

/*====================================================================================================================
 * Blocking function that writes a serial character when the UART buffer is not full
====================================================================================================================*/
void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}

/*===================================================================================================================
 * Blocking function that writes a string when the UART buffer is not full
====================================================================================================================*/
void putsUart0(char* str)
{
	int i;
	for (i = 0; i < strlen(str); i++)
		putcUart0(str[i]);
}

/*===================================================================================================================
 * Blocking function that returns with serial data once the buffer is not empty
====================================================================================================================*/
char getcUart0()
{
	while (UART0_FR_R & UART_FR_RXFE);
	return UART0_DR_R;
}

/*===================================================================================================================
 * Function returns the number of arguments after parsing the string.
 * Example: If command SET 25 45 96 is passed, the total number arguments in this command is 4.
 * This function returns 4.
====================================================================================================================*/
uint16_t Get_Rx_Str_Count()
{
	return Rx_Str_Count;
}

/*===================================================================================================================
 * Function returns the string based the position value passed to the function.
 * Example: If command SET 25 45 96 is passed, the string "SET" starts at position "0".
 * When Get_String(0) is called, this function returns SET.
====================================================================================================================*/

char *Get_String(uint16_t arg)
{
	return (char *) &Rx_Str_Buffer[Rx_Str_Pos[arg]];

}

/*===================================================================================================================
 * Function returns whether the command passed is valid or not.
 * Example: If command SET 25 45 96 is passed, the valid command is "SET" and number of arguments is 4.
 * When isCommand_valid("SET",4) is called, this function checks if command passed by the user and valid command is same
 * and whether the same number of arguments passed by user and valid command are same.
 * If both conditions are valid then functions returns true otherwise false.
====================================================================================================================*/
bool isCommand_valid(char *command, int arg)
{
	int temp = (strcmp(command, Get_String(0)));
	if(( temp == 0) && (Rx_Str_Arguments_Count == arg))
	{
		return true;
	}
	else
	{
		return false;
	}
}

/*===================================================================================================================
 * Function returns the number based the position value passed to the function.
 * When Get_Number(1) is called, this function first checks whether the type of the string is number.
 * If its number then it converts the string to number and returns the number.
====================================================================================================================*/
uint16_t Get_Number(int arg)
{
	if(Rx_Str_type[arg] == 'N')
	{
		return atoi(Get_String(arg));
	}
	else
	{
		return ERROR;
	}
}

/*===================================================================================================================
 * Function stores the character entered by the user after passing through some conditions.
 * If user enters "backspace" and if its first character entered then its ignored otherwise the count is decremented
 * so the array gets decremented.
 * If user enters "carriage return(enter)" then a null character is added to buffer and exit out of function.
 * If user enters the any character which is greater than or equal to 32(ASCII value), add the character to buffer.
 * If user entered lower case alphabets then convert it into upper case alphabets and store it.
 * If user entered any other character ignore it.
====================================================================================================================*/
bool Receive_String()
{
	bool isValid = true;

	char Rx_Char = getcUart0();

	if(Rx_Char == '\b')
	{
		if(Rx_Str_Count > 0)
		{
			Rx_Str_Count--;
		}
	}

	else if(Rx_Char == '\r')
	{
		Rx_Str_Buffer[Rx_Str_Count] = '\0';
		isValid = false;
	}

	else if(Rx_Char >= ' ')
	{
		if((Rx_Char >= 'a') && (Rx_Char <= 'z'))
		{
			Rx_Char = Rx_Char - 32;
		}
		Rx_Str_Buffer[Rx_Str_Count] = Rx_Char;
		Rx_Str_Count++;
	}

	else
	{

	}

	return isValid;
}

/*=====================================================================================================================
 * Function parses the string entered by the user and categorizes into three types:
 * (1)Position of string, (2)type of string (3)number of arguments.
 * When Parse_String() is called, the buffer is parsed completely and if any character other than alphabets or numbers
 * are present, it is replaced by null characters.
 * When we encounter alphabet of string the type of string is updated as "A" or number is encounter then type of string
 * is updated as "N", the position of the string is updated and number of arguments is incremented by 1 for one string.
=======================================================================================================================*/
void Parse_String()
{
	uint16_t i = 0;
	uint16_t j = 0;

	bool isChar = true;
	bool isNum  = true;

	int temp = strlen(Rx_Str_Buffer);

	for(i=0; i <= temp; i++ )
	{
		if(((Rx_Str_Buffer[i] >= 'A')  && (Rx_Str_Buffer[i] <= 'Z')) || ((Rx_Str_Buffer[i] >= '0') && (Rx_Str_Buffer[i] <=  '9')))
		{

		}
		else
		{
			Rx_Str_Buffer[i] = 0;
		}
	}

	for( i = 0; i <= temp; i++)
	{
		if(Rx_Str_Buffer[i] != 0)
		{
			if(((Rx_Str_Buffer[i] >= 'A')  && (Rx_Str_Buffer[i] <= 'Z')) && (isChar == true))
			{
				Rx_Str_Pos[j] = i;
				Rx_Str_Arguments_Count++;
				Rx_Str_type[j] = 'A';
				j++;
				isChar = false;
			}
			else if(((Rx_Str_Buffer[i] >= '0')  && (Rx_Str_Buffer[i] <= '9')) && (isNum == true))
			{
				Rx_Str_Pos[j] = i;
				Rx_Str_Arguments_Count++;
				Rx_Str_type[j] = 'N';
				j++;
				isNum = false;
			}
			else
			{

			}
		}
		else
		{
			isChar = true;
			isNum  = true;
		}
	}
}

/*===================================================================================================================
 * Function sends the Destination Address when Send_RS485 function is called.
 * Wait until the UART 1 is not busy
 * Clears the EPS bit to make parity as '1'.(Setting PEN and SPS, clearing EPS bit makes parity '1').
 * Write the Destination address to UART 1 data register and add to checksum variable
====================================================================================================================*/
void Send_Address(uint16_t arg)
{
	UART1_LCRH_R &= ~UART_LCRH_EPS;
	UART1_DR_R = arg;
	Calculate_CheckSum += arg;
}

/*===================================================================================================================
 * Function sends the data when Send_RS485 function is called.
 * Wait until the UART 1 is not busy
 * Set the EPS bit to make parity as '0'.(Setting PEN,SPS and EPS bit makes parity '0'.)
 * Write the data to UART 1 data register and add to checksum variable
====================================================================================================================*/
void Send_Data(uint16_t arg)
{
	UART1_LCRH_R |= UART_LCRH_EPS;
	UART1_DR_R = arg;
	Calculate_CheckSum += arg;
}

/*===================================================================================================================
 * Function stored the data to be sent in an circular buffer
 * Scan to find out which position in the buffer can be written with new data
 * If the message present in buffer is already transmitted then update it with new address
 * Make Tx_Valid and Tx_Requested equal to 1
 * Manually trigger the UART1Isr for first time.
 * Turn on RED LED and set Red timeout to 20 which is turned off in Timer1Isr.
====================================================================================================================*/
void Send_RS485(uint8_t Dest_Add, uint8_t Command, uint8_t Channel, uint8_t Size, uint8_t Data[])
{
	uint8_t k = 0;
	bool full = true;
	for(WriteIndex = 0; WriteIndex < 10; WriteIndex++)
	{
		if(WriteIndex == 10)
		{
			WriteIndex = 0;
		}
		if ((Tx_Valid[WriteIndex] == 0) && (full == true))
		{
			Tx_Valid[WriteIndex] = 0x01;
			Dest_Add_Tx[WriteIndex] = Dest_Add;
			Command_Tx[WriteIndex] = Command;
			Channel_Tx[WriteIndex] = Channel;
			Size_Tx[WriteIndex] = Size;
			if((Command_Tx[WriteIndex] == 0x70) || (Command_Tx[WriteIndex] == 0x21))
			{
				TxSequenceID_Tx[WriteIndex] = Tx_Seq_Rx_Data;
			}
			else
			{
				Tx_Seq_ID++;
				TxSequenceID_Tx[WriteIndex] = Tx_Seq_ID;
			}
			for(k=0; k < Size; k++)
			{
				Data_Tx[WriteIndex][k] = Data[k];
			}
			if((Command_Tx[WriteIndex] & 0x80) == 0x80)
			{
				Tx_Ack_Required[WriteIndex] = 0x01;

				Tx_Retry_Count[WriteIndex] = 0;
				Tx_Timeout[WriteIndex] = 10;
			}
			Tx_Requested[WriteIndex] = 0x01;
			full = false;
		}
	}
	full = true;
	RED_LED = 1;
	Red_Timeout = 20;
	NVIC_SW_TRIG_R=0x06;
}
/*===================================================================================================================
 * Function checks whether the command entered by user and total number of arguments are correct based on the predefined
 * command.
 * If command entered by user matches the predefined command then the system command, address, channel and data value
 * is updated according the command.
====================================================================================================================*/
void Process_Command()
{
	if(isCommand_valid("RESET",'2') == true)
	{
		Command = 0x7F;
		Dest_Add = Get_Number(1);
		Send_RS485(Dest_Add, Command, 0x0, 0x0, &Data);
	}

	else if(isCommand_valid("SET",'4') == true)
	{
		if (Ack_Received == true)
		{
			Command = 0x80;
		}
		else
		{
			Command = 0x00;
		}
		Dest_Add = Get_Number(1);
		Channel = Get_Number(2);
		Data = Get_Number(3);
		Send_RS485(Dest_Add, Command, Channel, 0x1, &Data);
	}

	else if(isCommand_valid("GET",'3') == true)
	{
		if (Ack_Received == true)
		{
			Command = 0xA0;
		}
		else
		{
			Command = 0x20;
		}
		Dest_Add = Get_Number(1);
		Channel = Get_Number(2);
		Send_RS485(Dest_Add, Command, Channel, 0x0, 0x0);
	}

	else if(isCommand_valid("POLL",'1') == true)
	{
		Command = 0x78;
		Send_RS485(0xFF, Command, 0x0, 0x0, 0x0);
	}

	else if(isCommand_valid("SA",'3') == true)
	{
		if (Ack_Received == true)
		{
			Command = 0xFA;
		}
		else
		{
			Command = 0x7A;
		}
		Dest_Add = Get_Number(1);
		Data = Get_Number(2);
		Send_RS485(Dest_Add, Command, 0x0, 0x1, &Data);
	}
	else if(isCommand_valid("ACK",'2') == true)
	{
		if(strcmp("ON", Get_String(1)) == 0)
		{
			Ack_Received = true;
		}
		else if(strcmp("OFF", Get_String(1)) == 0)
		{
			Ack_Received = false;
		}
		else
		{
			putsUart0("ERROR : The command entered is not valid\r\n");
		}
	}
	else
	{
		putsUart0("ERROR : The command entered is not valid\r\n");
	}
}

/*===================================================================================================================
 * Function process the message received through Uart1
 * If SET command is received with Channel number 3 and data 1 then BLUE LED is turned on.
 * If SET command is received with Channel number 3 and data 0 then BLUE LED is turned off.
 * If GET command is received with Channel number 9, Data response message is sent back with status of push button.
 * If RESET command is received, the Micro-controller is reset.
 * If SA command is received then it address of the controller is changed.
 * If ACK command is received then for following commands the acknowledgment message is sent back.
 * If POLL command is received then POLL response message is sent back.
====================================================================================================================*/
void Process_Message()
{
	uint8_t data_step9 = 0;
	if((Rx_Data[0] == Src_Add) && (Rx_Data[3] == 0x80) && (Rx_Data[4] == 3) && (Rx_Data[6] == 1))
	{
		BLUE_LED = 1;
		Tx_Seq_Rx_Data = Rx_Data[2];
		Send_RS485(Rx_Data[1], 0x70, 0x0, 0x1, &Tx_Seq_Rx_Data);
	}
	else if((Rx_Data[0] == Src_Add) && (Rx_Data[3] == 0x80) && (Rx_Data[4] == 3) && (Rx_Data[6] == 0))
	{
		BLUE_LED = 0;
		Tx_Seq_Rx_Data = Rx_Data[2];
		Send_RS485(Rx_Data[1], 0x70, 0x0, 0x1, &Tx_Seq_Rx_Data);
	}
	else if((Rx_Data[0] == Src_Add) && (Rx_Data[3] == 0x00) && (Rx_Data[4] == 3) && (Rx_Data[6] == 1))
	{
		BLUE_LED = 1;
	}
	else if((Rx_Data[0] == Src_Add) && (Rx_Data[3] == 0x00) && (Rx_Data[4] == 3) && (Rx_Data[6] == 0))
	{
		BLUE_LED = 0;
	}
	else if((Rx_Data[3] == 0x7F) && (Rx_Data[1] == Src_Add))
	{
		NVIC_APINT_R = (NVIC_APINT_SYSRESETREQ | NVIC_APINT_VECTKEY);
	}
	else if((Rx_Data[3] == 0x20)  && (Rx_Data[4] == 9))
	{

		if(PUSH_BUTTON == 0)
		{
			data_step9 = 1;
			//Tx_Seq_Rx_Data = Rx_Data[2];
			Send_RS485(Rx_Data[1], 0x21, 0x0, 0x1, &data_step9);
		}
		else
		{
			data_step9 = 0;
			//Tx_Seq_Rx_Data = Rx_Data[2];
			Send_RS485(Rx_Data[1], 0x21, 0x0, 0x1, &data_step9);
		}
	}
	else if((Rx_Data[3] == 0xA0)  && (Rx_Data[4] == 9))
	{
		if(PUSH_BUTTON == 0)
		{
			data_step9 = 1;
			Tx_Seq_Rx_Data = Rx_Data[2];
			Send_RS485(Rx_Data[1], 0x70, 0x0, 0x1, &Tx_Seq_Rx_Data);
			Send_RS485(Rx_Data[1], 0x21, 0x0, 0x1, &data_step9);
		}
		else
		{
			data_step9 = 0;
			Tx_Seq_Rx_Data = Rx_Data[2];
			Send_RS485(Rx_Data[1], 0x70, 0x0, 0x1, &Tx_Seq_Rx_Data);
			Send_RS485(Rx_Data[1], 0x21, 0x0, 0x1, &data_step9);
		}
	}
	else if(Rx_Data[3] == 0x70)
	{
		char str;
		bool find = true;
		uint8_t k = 0;
		for(k = 0; k < 10; k++)
		{
			if((TxSequenceID_Tx[k] == Rx_Data[2]) && (find == true))
			{
				Tx_Ack_Required[k] = 0;
				putsUart0("Acknowledgement received\r\n");
				str = '0' + TxSequenceID_Tx[k];
				putsUart0("The transmitted sequence ID is\r\n");
				putcUart0(str);
				putsUart0("\r\n");
				//sprintf(str, "The transmitted sequence ID is = %u", Rx_Data[6]);
				Tx_Valid[k] = 0;
				find = false;
			}
		}
		find = true;
	}
	else if(Rx_Data[3] == 0x70)
	{
		char str;
		putsUart0("Acknowledgement received\r\n");
		str = '0' + Rx_Data[0];
		putsUart0("The response received from the address\r\n");
		putcUart0(str);
		putsUart0("\r\n");
		str = '0' + Rx_Data[2];
		putsUart0("The transmitted sequence ID is\r\n");
		putcUart0(str);
		putsUart0("\r\n");
		str = '0' + Rx_Data[6];
		putsUart0("The data received is\r\n");
		putcUart0(str);
		putsUart0("\r\n");
		//sprintf(str, "The transmitted sequence ID is = %u", Rx_Data[6]);
	}
	else if(Rx_Data[3] == 0x21)
	{
		char str;
		putsUart0("Data response received\r\n");
		str = '0' + Rx_Data[0];
		putsUart0("The response received from the address\r\n");
		putcUart0(str);
		putsUart0("\r\n");
		str = '0' + Rx_Data[2];
		putsUart0("The transmitted sequence ID is\r\n");
		putcUart0(str);
		putsUart0("\r\n");
		str = '0' + Rx_Data[6];
		putsUart0("The data received is\r\n");
		putcUart0(str);
		putsUart0("\r\n");
		//sprintf(str, "The transmitted sequence ID is = %u", Rx_Data[6]);
	}
	else if(Rx_Data[3] == 0x78)
	{
		data_step9 = Src_Add;
		Send_RS485(Rx_Data[1], 0x79, 0x0, 0x1, &data_step9);
	}
	else if(Rx_Data[3] == 0x79)
	{
		char str;
		putsUart0("Poll response received\r\n");
		str = '0' + Rx_Data[6];
		putsUart0("The Data received from poll response is\r\n");
		putcUart0(str);
		putsUart0("\r\n");
	}
	else if(Rx_Data[3] == 0x7A)
	{
		EEPROM_Write_Value_Data[0] = (uint32_t)Rx_Data[6];
		EEPROM_Return_Value = EEPROMInit();
		if(EEPROM_Return_Value == EEPROM_INIT_OK)
		{
			EEPROMProgram(EEPROM_Write_Value_Data, EEPROM_Value_Address, sizeof(EEPROM_Write_Value_Data));
			__asm("             NOP");
			__asm("             NOP");
			__asm("             NOP");
			__asm("             NOP");
			__asm("             NOP");
			__asm("             NOP");
			__asm("             NOP");
			Src_Add = EEPROM_Write_Value_Data[0];
		}
	}
	else if(Rx_Data[3] == 0xFA)
	{
		EEPROM_Write_Value_Data[0] = (uint32_t)Rx_Data[6];
		EEPROM_Return_Value = EEPROMInit();
		if(EEPROM_Return_Value == EEPROM_INIT_OK)
		{
			EEPROMProgram(EEPROM_Write_Value_Data, EEPROM_Value_Address, sizeof(EEPROM_Write_Value_Data));
			__asm("             NOP");
			__asm("             NOP");
			__asm("             NOP");
			__asm("             NOP");
			__asm("             NOP");
			__asm("             NOP");
			__asm("             NOP");

		}
		Tx_Seq_Rx_Data = Rx_Data[2];
		Src_Add = EEPROM_Write_Value_Data[0];
		Send_RS485(Src_Add, 0x70, 0x0, 0x1, &Tx_Seq_Rx_Data);

	}
	else
	{

	}
}

/*===================================================================================================================
 * Function process the data to be written to UART1 data register and data received in UART1 data register when interrupt occurs.
 * If Data should be transmitted, finds the valid message and starts writing to the UART1 data register.
 * When all bytes in data packets has been transmitted, the transmit interrupt flag is cleared.
 * If Data should be received and it is intended for this controller or broadcast address then controller starts processing message.
 * The controller starts storing the received message bytes in a buffer and calculate the checksum.
 * After all message bytes are received, controllers validates the message by using checksum
 * when received checksum and calculated checksum are added, if its equal to 0xFF then message is valid.
 * The valid message is passed to process the message and GREEN LED is turned and timeout is loaded.
 * If received message is not valid then RED LED is turned on.
====================================================================================================================*/
void Uart1Isr()
{
	bool First = true;
	if((((UART1_MIS_R & UART_MIS_TXMIS) || (First == true)) && (!(UART1_MIS_R & UART_MIS_RXMIS))))
	{
		DEN = 1;
		uint8_t k = 0;
		char str;
		char * str1 = 0;
		for(k = 0; k < 10; k++)
		{
			if((Tx_Valid[k] == 1) && (Tx_Requested[k] == 1))
			{
				MsgInProgress = k;
				break;
			}
		}
		if(MsgInProgress >= 0)
		{
			if(MsgPhase == 0)
			{
				UART1_LCRH_R &= ~UART_LCRH_EPS;
				UART1_DR_R = Dest_Add_Tx[MsgInProgress];
				Calculate_CheckSum += Dest_Add_Tx[MsgInProgress];
				waitMicrosecond(25000);
				MsgPhase++;
				str = '0' + Dest_Add_Tx[MsgInProgress];
				putcUart0(str);
				putsUart0("  ");
			}
			else if(MsgPhase == 1)
			{
				UART1_LCRH_R |= UART_LCRH_EPS;
				UART1_DR_R = Src_Add;
				Calculate_CheckSum += Src_Add;
				waitMicrosecond(25000);
				MsgPhase++;
				str = '0' + Src_Add;
				putcUart0(str);
				putsUart0("  ");
			}
			else if(MsgPhase == 2)
			{
				UART1_LCRH_R |= UART_LCRH_EPS;
				UART1_DR_R = TxSequenceID_Tx[MsgInProgress];
				Calculate_CheckSum += TxSequenceID_Tx[MsgInProgress];
				waitMicrosecond(25000);
				MsgPhase++;
				str = '0' + TxSequenceID_Tx[MsgInProgress];
				putcUart0(str);
				putsUart0("  ");
			}
			else if(MsgPhase == 3)
			{
				UART1_LCRH_R |= UART_LCRH_EPS;
				UART1_DR_R = Command_Tx[MsgInProgress];
				Calculate_CheckSum += Command_Tx[MsgInProgress];
				waitMicrosecond(25000);
				MsgPhase++;
				str = '0' + Command_Tx[MsgInProgress];
				str1 = &str;
				putsUart0(str1);
				putsUart0("  ");
			}
			else if(MsgPhase == 4)
			{
				UART1_LCRH_R |= UART_LCRH_EPS;
				UART1_DR_R = Channel_Tx[MsgInProgress];
				Calculate_CheckSum += Channel_Tx[MsgInProgress];
				waitMicrosecond(25000);
				MsgPhase++;
				str = '0' + Channel_Tx[MsgInProgress];
				putcUart0(str);
				putsUart0("  ");
			}
			else if(MsgPhase == 5)
			{
				UART1_LCRH_R |= UART_LCRH_EPS;
				UART1_DR_R = Size_Tx[MsgInProgress];
				Calculate_CheckSum += Size_Tx[MsgInProgress];
				waitMicrosecond(25000);
				MsgPhase++;
				str = '0' + Size_Tx[MsgInProgress];
				putcUart0(str);
				putsUart0("  ");
			}
			else if((MsgPhase == 6) && (Size_Tx[MsgInProgress] > 0))
			{
				UART1_LCRH_R |= UART_LCRH_EPS;
				for(k=0; k < Size_Tx[MsgInProgress]; k++)
				{
					UART1_DR_R = Data_Tx[MsgInProgress][k];
					Calculate_CheckSum += Data_Tx[MsgInProgress][k];
					str = '0' + Data_Tx[MsgInProgress][k];
					putcUart0(str);
					putsUart0("  ");
				}
				waitMicrosecond(25000);
				MsgPhase++;
			}
			else
			{
				UART1_LCRH_R |= UART_LCRH_EPS;
				UART1_DR_R = ~Calculate_CheckSum;
				waitMicrosecond(25000);
				MsgPhase = 0;
				str = '0' + (~Calculate_CheckSum);
				putcUart0(str);
				putsUart0("\r\n");
				Tx_Requested[MsgInProgress] = 0;
				Calculate_CheckSum = 0;
				if(Tx_Ack_Required[MsgInProgress] == 0)
				{
					Tx_Valid[MsgInProgress] = 0;
				}
				MsgInProgress = -1;
			}
		}
		UART1_ICR_R |= UART_ICR_TXIC;
		DEN = 0;
	}
	if(UART1_MIS_R & UART_MIS_RXMIS)
	{
		uint16_t CheckSum_Offset;
		uint16_t Data_Temp = UART1_DR_R;
		uint16_t Data = Data_Temp & 0xFF;
		uint16_t ParityError = (!(((UART1_LCRH_R & UART_LCRH_EPS) >> 2) ^ ((UART1_DR_R & UART_DR_PE) >> 9)));

		if((ParityError == 1) && (Phase == -1))
		{
			if(Data == 255 || Data == Src_Add)
			{
				Phase = 0;
				Rx_CheckSum = 0;
			}
		}
		if(Phase >= 0)
		{
			Rx_Data[Phase] = Data;
			Rx_CheckSum += Data;

			if(Phase > 5)
			{
				CheckSum_Offset = (uint8_t)6 + Rx_Data[5];
				if(Phase == CheckSum_Offset)
				{
					Phase = -1;
					if(((Rx_CheckSum | Rx_Data[CheckSum_Offset])!= 0xFF) || (((Rx_CheckSum | Rx_Data[CheckSum_Offset])!= 0x1FF) && (Rx_Data[4] == 0x78)))
					{
						RED_LED = 1;
					}
					else
					{
						GREEN_LED = 1;
						Green_Timeout = 20;
						Process_Message();
					}
				}
				else
				{
					Phase++;
				}
			}
			else
			{
				Phase++;
			}
		}
	}
}

/*===================================================================================================================
 * Timer 1 interrupt occurs every 10ms
 * If red_timeout and/or Green_timeout is greater than zero, decrement there value.
 * when red_timeout and/or green_timeout is equal to zero then turn off the RED and/or GREEN LED.
 * If acknowledgement is not received for certain time and then request for retransmission of data upto 4 times.
 * if PUSH BUTTON is pressed continuously for 30ms then send POLL response message to all devices in the network
 * Clear the Timer1 interrupt.
====================================================================================================================*/
void Timer1Isr()
{
	uint8_t k = 0;

	uint8_t data_temp = 0;
	if(Red_Timeout > 0)
	{
		Red_Timeout--;
	}
	if(Green_Timeout > 0)
	{
		Green_Timeout--;
	}
	if(Red_Timeout == 0)
	{
		RED_LED = 0;
	}
	if(Green_Timeout == 0)
	{
		GREEN_LED = 0;
	}
	for(k = 0; k < 10; k++)
	{
		if((Tx_Valid[k] == 1) && (Tx_Ack_Required[k] == 1) && (Tx_Timeout[k] > 0))
		{
			Tx_Timeout[k]--;

			if(Tx_Timeout[k] == 0)
			{
				Tx_Retry_Count[k]++;
				Tx_Timeout[k] = (10 + 10 * (rand() % 2^(Tx_Retry_Count[k])));

				if(Tx_Retry_Count[k] == MAX)
				{
					putsUart0("ERROR : Acknowledgement not received\r\n");
					RED_LED = 1;
					Tx_Valid[k] = 0;
					Tx_Requested[k] = 0;
					Tx_Ack_Required[k] = 0;
					Tx_Timeout[k] = 0;
				}
				else
				{
					Tx_Valid[k] = 1;
					Tx_Requested[k] = 1;
					NVIC_SW_TRIG_R=0x06;
				}
			}
		}

	}
	if(PUSH_BUTTON == 0)
	{
		Push_Button_Pressed++;
	}
	if(Push_Button_Pressed == 3)
	{
		Push_Button_Pressed = 0;
		data_temp = 1;
		Send_RS485(0xFF, 0x79, 0x0, 0x1, &data_temp);
	}

	TIMER1_ICR_R = TIMER_ICR_TATOCINT;               // clear interrupt flag
}

/*===================================================================================================================
 * Main Function which is the starting point of program execution
====================================================================================================================*/
int main(void)
{
	char str_src_add;
	/* Calls initHw() function to Initializes the TM4C123GH6PM hardware */
	initHw();

	/* Display the below lines on Tera Term window */
	putsUart0("Welcome To EE5314 Project Based On ARM Processor\r\n");

	/* Turn ON the GREEN LED then wait for quarter second and switch OFF the GREEN LED. */
	GREEN_LED = 1;

	waitMicrosecond(250000);

	GREEN_LED = 0;

	EEPROMRead(EEPROM_Read_Value_Data, EEPROM_Value_Address, sizeof(EEPROM_Read_Value_Data));

	if(EEPROM_Read_Value_Data[0] >= (uint32_t)Src_Add)
	{
		Src_Add = EEPROM_Read_Value_Data[0];
	}
	else
	{
		Src_Add = 0;
	}
	str_src_add = '0' + Src_Add;
	putsUart0("The Source address for this controller is :\r\n");
	putcUart0(str_src_add);
	putsUart0("\r\n");
	while(1)
	{
		/* Make the string buffer count and number of arguments counter to 0 */
		Rx_Str_Count = 0;
		Rx_Str_Arguments_Count = 48;

		putsUart0("Enter the command\r\n");

		/* Receive the characters entered by user till count reaches 80 or carriage return is pressed */
		while(Rx_Str_Count != 80)
		{
			bool isValid = Receive_String();
			if(!isValid){
				break;
			}
		}

		/* Display the string entered by user back on Tera term */
		putsUart0("The Command you entered is :\r\n");
		putsUart0(Rx_Str_Buffer);
		putsUart0("\r\n");

		/* Call Parse_String() function to update position of string, type of string and number of arguments */
		Parse_String();

		/*
		 * Call Process_Command() function to check whether command and number of arguments entered by user is
		 * valid and update the command value, address, channel and data value based on the command.
		 */
		Process_Command();
	}
}

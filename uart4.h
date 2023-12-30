// UART4 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL
// Target uC:       TM4C123GH6PM
// System Clock:    -

// Hardware configuration:
// UART Interface:
//   U4TX (PC5) and U4RX (PC4) are connected to the 2nd controller
//   The USB on the 2nd controller enumerates to an ICDI interface and a virtual COM port

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#ifndef UART4_H_
#define UART4_H_

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

void initUart4();
void setUart4BaudRate(uint32_t baudRate, uint32_t fcyc);
void putcUart4(char c);
void putsUart4(char* str);
char getcUart4();
bool kbhitUart4();
void Uart4TxIsr();

#endif

// UART0 Library
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

#include <stdint.h>
#include <stdbool.h>
#include "tm4c123gh6pm.h"
#include "uart4.h"
#include "wait.h"
#include "rgb_led.h"

// UART4 TX PC5 and RX PC4

// Port C masks
#define UART_TX_MASK 32
#define UART_RX_MASK 16

// Bit band aliases
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4))) // PF1 Activity LED
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // PF2 Transmit LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4))) // PF3

// PortF masks
#define GREEN_LED_MASK 8
#define RED_LED_MASK   2
#define BLUE_LED_MASK  4

//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

extern uint32_t global[512];
extern uint32_t max;

extern uint32_t j;

bool bf = 0;
bool sf = 0;

uint32_t flag1 = 0;


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize UART0
void initUart4()
{
    // Enable clocks
    SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R4;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
    _delay_cycles(3);

    // Configure UART4 pins
    GPIO_PORTC_DR2R_R  |= UART_TX_MASK;                            // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTC_DEN_R   |= UART_TX_MASK | UART_RX_MASK;             // enable digital on UART0 pins
    GPIO_PORTC_AFSEL_R |= UART_TX_MASK | UART_RX_MASK;             // use peripheral to drive PA0, PA1
    GPIO_PORTC_PCTL_R  &= ~(GPIO_PCTL_PC4_M | GPIO_PCTL_PC5_M);    // clear bits 0-7
    GPIO_PORTC_PCTL_R  |= GPIO_PCTL_PC5_U4TX | GPIO_PCTL_PC4_U4RX; // select UART0 to drive pins PA0 and PA1: default, added for clarity

    // Configure UART4 to 256000 baud, 8N1 format
    UART4_CTL_R  = 0;                                              // turn-off UART0 to allow safe programming
    UART4_CC_R   = UART_CC_CS_SYSCLK;                              // use system clock (40 MHz)
    UART4_IBRD_R = 10;                                             // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
    UART4_FBRD_R = 0;                                              // round(fract(r)*64)=45
    // 10 & 0
    UART4_IM_R   = UART_IM_RXIM;                                     // UART Interrupt Mask Register, UART Receive Interrupt Mask
    UART4_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_STP2;               // configure for 8N1 w/ 16-level FIFO
    UART4_CTL_R  =  UART_CTL_UARTEN | UART_CTL_EOT ;  // enable TX, RX, and module, End of Transmission UART_CTL_EOT
    NVIC_EN1_R   = 1 <<  (INT_UART4-16-32);                             // turn-on interrupt 76 (UART4) in NVIC


}

// Set baud rate as function of instruction cycle frequency

//void setUart4BaudRate(uint32_t baudRate, uint32_t fcyc)
//{
//    uint32_t divisorTimes128 = (fcyc * 8) / baudRate;   // calculate divisor (r) in units of 1/128,
//                                                        // where r = fcyc / 16 * baudRate
//    divisorTimes128 += 1;                               // add 1/128 to allow rounding
//    UART0_CTL_R = 0;                                    // turn-off UART0 to allow safe programming
//    UART0_IBRD_R = divisorTimes128 >> 7;                // set integer value to floor(r)
//    UART0_FBRD_R = ((divisorTimes128) >> 1) & 63;       // set fractional value to round(fract(r)*64)
//    UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN;    // configure for 8N1 w/ 16-level FIFO
//    UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN;
//                                                        // turn-on UART0
//}

// Blocking function that writes a serial character when the UART buffer is not full
void putcUart4(char c)
{
    while (UART4_FR_R & UART_FR_TXFF);               // wait if uart0 tx fifo full
    UART4_DR_R = c;                                  // write character to fifo
}

// Blocking function that writes a string when the UART buffer is not full
void putsUart4(char* str)
{
    uint8_t i = 0;
    while (str[i] != '\0')
        putcUart4(str[i++]);
}

// Blocking function that returns with serial data once the buffer is not empty
char getcUart4()
{
    while (UART4_FR_R & UART_FR_RXFE);               // wait if uart0 rx fifo empty
    return UART4_DR_R & 0xFF;                        // get character from fifo
}

// Returns the status of the receive buffer
bool kbhitUart4()
{
    return !(UART4_FR_R & UART_FR_RXFE);
}




void Uart4TxIsr()
{
    if (UART4_DR_R & UART_DR_BE) // if Break Error is set
    {
        bf = 1;
    }

    if ((bf == 1) && (sf == 0) )
    {
     if ( (UART4_DR_R & 0xFF) == 0) // if Null start code received
     {

         sf = 1;
     }
    }

    if( (bf) && (sf) )
    {

     if (j<512) // No for loop here
    {
      //while (UART4_FR_R & UART_FR_BUSY);    // wait if uart4 tx fifo full; Same logic as putcUart4
      global[j] = UART4_DR_R & 0xFF;               // transfer data from the UART_DR_R to global array. create a mask here
      j++;
      setRgbColor(global[j] * 4, 0, 0);
    }

     else
     {
         bf = 0;
         sf = 0;
              j = 0;
     }


    }




    //clear UART interrupt flag
    UART4_ICR_R = UART_ICR_RXIC; // Receive Interrupt Clear

}

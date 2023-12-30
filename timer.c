/*
 * timer.c
 *
 *  Created on: 23-Oct-2023
 *      Author: padma
 */

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "timer.h"
#include "tm4c123gh6pm.h"
#include "wait.h"

#define MAX_CHARS 80
#define MAX_FIELDS 5

extern uint32_t global2[512];
extern uint32_t max;
extern bool timerisr_flag;
extern uint32_t i;

typedef struct _USER_DATA
{
 char buffer[MAX_CHARS+1];
 uint8_t fieldCount;
 uint8_t fieldPosition[MAX_FIELDS];
 char fieldType[MAX_FIELDS];
} USER_DATA;

// Pin bitbands
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4)))

// PortF masks
#define GREEN_LED_MASK 8


//-----------------------------------------------------------------------------
// Global variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initTimer1()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R1;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure LED pin
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK;
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK;

    // Configure Timer 1 as the time base
    TIMER1_CTL_R  &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER1_CFG_R   = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER1_TAMR_R  = TIMER_TAMR_TAMR_1_SHOT;          // configure for periodic mode (count down)
    TIMER1_TAILR_R = 3680;                       // 92us
    TIMER1_IMR_R   = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    //TIMER1_CTL_R  |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R     = 1 << (INT_TIMER1A-16);              // turn-on interrupt 37 (TIMER1A) in NVIC
}

void timer1_BRK()
{
    // BREAK is outside the main timer ISR else it won't trigger the actual timer1Isr() ISR

 UART4_LCRH_R  |= UART_LCRH_BRK   ;  // UART Send Break. When set to '1', A Low level is continually output on the UnTx signal.
 TIMER1_CTL_R  |= TIMER_CTL_TAEN ;  // GPTM Timer A Enable for 92us. Timer started here.

}

// timer ISR

void timer1Isr()
{


    i = 0;

    // MAB - MARK AFTER BREAK

    UART4_LCRH_R  &=   ~(UART_LCRH_BRK) ; // UART Send Break set to '0' so the UART line is HIGH.
    waitMicrosecond(12);

    while (UART4_FR_R & UART_FR_BUSY);    // wait if uart4 tx fifo full; Same logic as putcUart4

    // STARTCODE & DATA
    UART4_DR_R = 0x00; // send hardcoded 5 bytes

    for (i=0;i<max;i++)
    {
     while (UART4_FR_R & UART_FR_BUSY);    // wait if uart4 tx fifo full; Same logic as putcUart4
     UART4_DR_R = global2[i]; // transfer data from the global array to UART_DR_R
    }

    while (UART4_FR_R & UART_FR_BUSY);    // wait if uart4 tx fifo full; Same logic as putcUart4

    // clear interrupt flag end of the timer timer1Isr()

    TIMER1_ICR_R = TIMER_ICR_TATOCINT;

    //  timer1_BRK() must get called only when we type "on" else it should not

    if (i>=max)
        timer1_BRK();


}



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
#include "timer2.h"
#include "tm4c123gh6pm.h"
#include "wait.h"
#include "rgb_led.h"

#define MAX_CHARS 80
#define MAX_FIELDS 5

extern uint32_t global2[512];
extern uint32_t max;
extern bool timerisr_flag;
extern uint32_t i;

extern uint32_t address_ramp;
extern uint32_t time_ramp;
extern uint32_t start_ramp;
extern uint32_t stop_ramp;
extern uint32_t tmp_ramp;
extern uint32_t increment;





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
void initTimer2()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCTIMER_R |= SYSCTL_RCGCTIMER_R2;
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5;
    _delay_cycles(3);

    // Configure LED pin
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK;
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK;

    // Configure Timer 2 as the time base
    TIMER2_CTL_R  &= ~TIMER_CTL_TAEN;                 // turn-off timer before reconfiguring
    TIMER2_CFG_R   = TIMER_CFG_32_BIT_TIMER;           // configure as 32-bit timer (A+B)
    TIMER2_TAMR_R  = TIMER_TAMR_TAMR_PERIOD;          // configure for one shot (count down)
    //TIMER2_TAILR_R = 40000 * time_ramp ;                   // load value
    TIMER2_IMR_R   = TIMER_IMR_TATOIM;                 // turn-on interrupts for timeout in timer module
    //TIMER1_CTL_R  |= TIMER_CTL_TAEN;                  // turn-on timer
    NVIC_EN0_R     = 1 << (INT_TIMER2A-16);              // turn-on interrupt 37 (TIMER1A) in NVIC
}

// timer ISR
/*

void timer2Isr()
{

    increment = (float) (stop_ramp - start_ramp) / (float)  (time_ramp);
    increment = increment + 1;

    if (global2[address_ramp] >= stop_ramp)
    {
     global2[address_ramp] = start_ramp;
     tmp_ramp = start_ramp;

    }

    else
    {
     global2[address_ramp] = global2[address_ramp] + increment; // do the ramp increment
    }

    TIMER2_ICR_R = TIMER_ICR_TATOCINT;     // Clear timer2 interrupt flag

}
*/


void timer2Isr()
{
    //global2[address_ramp] = start_ramp;

    if (global2[address_ramp] < stop_ramp)
    {
     global2[address_ramp] = global2[address_ramp] + increment;
     //setRgbColor(global2[address_ramp] * 4, 0, 0);
    }

    else
    {
        global2[address_ramp] = start_ramp;

    }

    TIMER2_ICR_R = TIMER_ICR_TATOCINT;     // Clear timer2 interrupt flag

}






#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <inttypes.h>
#include "clock.h"

#include <stdlib.h>
#include <stdio.h>

#include "uart0.h"
#include "uart4.h"
#include "timer.h"
#include "timer2.h"
#include "timer0.h"
#include "rgb_led.h"
#include "wait.h"

#include "eeprom.h"

#include "tm4c123gh6pm.h"

#define DEBUG

// PortA masks
#define UART_TX_MASK 2
#define UART_RX_MASK 1

#define MAX_CHARS 80
#define MAX_FIELDS 5

#define MAX 512

uint32_t global[512];
uint32_t global2[512];
uint32_t max;

bool timerisr_flag =0;

uint32_t i = 0;
uint32_t j = 0;

uint16_t address1;
uint32_t address_ramp;
uint32_t time_ramp;
uint32_t start_ramp;
uint32_t stop_ramp;
uint32_t tmp_ramp;
uint32_t global_time;
uint32_t increment = 0;

uint32_t address_pulse;
uint32_t time_pulse;
uint32_t first_pulse;
uint32_t last_pulse;
uint32_t tmp_pulse=0;


uint32_t tmp;



typedef struct _USER_DATA
{
 char buffer[MAX_CHARS+1];
 uint8_t fieldCount;
 uint8_t fieldPosition[MAX_FIELDS];
 char fieldType[MAX_FIELDS];
} USER_DATA;


// Bit band aliases
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 1*4))) // PF1 DEvice LED
#define BLUE_LED     (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 2*4))) // PF2 Transmit LED
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 3*4))) // PF3 Activity LED for UART0

#define DE_PIN       (*((volatile uint32_t *) (0x42000000 + (0x400063FC-0x40000000)*32 + 6*4))) // DE PC6

// PortF masks
#define GREEN_LED_MASK 8
#define RED_LED_MASK   2
#define BLUE_LED_MASK  4

#define DE_PIN_MASK 64 // DE PC6


//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();

    // Enable clocks
    SYSCTL_RCGCGPIO_R = SYSCTL_RCGCGPIO_R5 | SYSCTL_RCGCGPIO_R2;
    _delay_cycles(3);

    // Configure LED pins
    GPIO_PORTF_DIR_R |= GREEN_LED_MASK | RED_LED_MASK;  // bits 1 and 3 are outputs
    GPIO_PORTF_DR2R_R |= GREEN_LED_MASK | RED_LED_MASK; // set drive strength to 2mA (not needed since default configuration -- for clarity)
    GPIO_PORTF_DEN_R |= GREEN_LED_MASK | RED_LED_MASK;  // enable LEDs

    GPIO_PORTC_DIR_R |= DE_PIN_MASK;   // bits 0,1,2,3 are outputs (LEDs), other pins are inputs

    GPIO_PORTC_DEN_R |= DE_PIN_MASK;  // enable LEDs
}

// Blocking function that writes a serial character when the UART buffer is not full
extern void putcUart0(char c);

// Blocking function that writes a string when the UART buffer is not full
extern void putsUart0(char* str);

// Blocking function that returns with serial data once the buffer is not empty
extern char getcUart0();

// Blocking function that returns with string data once the buffer is not empty
extern void getsUart0(USER_DATA *data);

extern void parseFields(USER_DATA *data);

extern char* getFieldString (USER_DATA* data, uint8_t fieldNumber);

extern int getFieldInteger(USER_DATA* data, uint8_t fieldNumber);

extern bool isCommand(USER_DATA* data, const char strCommand[], uint8_t minArguments);

//extern void timer1_BRK();




//----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    // intUart0(); intUart4(); intTimer(); TimerISR(); Global variable?

    USER_DATA data;

    initHw();
    initRgb();

    initUart0();
    initUart4();
    initEeprom();
    initTimer1();
    initTimer2();
    initTimer0();

    max = MAX;
    address1 = 1;
    char str[50];

    uint32_t addr_eeprom1 = (uint16_t) 1;
    uint32_t data_eeprom1 = (uint32_t) 1;

    uint32_t addr_eeprom2 = (uint16_t) 1;
    //uint32_t data_eeprom2 = (uint32_t) 1;

    writeEeprom(addr_eeprom1,data_eeprom1); // forcing the default mode to be controller mode

    // DE_PIN = 1; // DE_PIN is required for transmitting the data not for receiving. Receiving is through RE_PIN

    // timer1_BRK();



    while(1)
    {

        if (kbhitUart0())   // only if keyboard hit  else getsUart0() is blocking function that blocks it
        {
         getsUart0(&data);
         parseFields(&data);

         if (isCommand(&data, "controller", 0)) // if typed controller it should remain in controller mode. done through EEPROM.
         {
             DE_PIN = 1;

             UART4_CTL_R  &=  ~(UART_CTL_RXE);
             UART4_CTL_R  |=  (UART_CTL_TXE );

             putsUart0("Inside 'controller' block \r\n");

          // configure EEPROM logic

          writeEeprom(addr_eeprom1,data_eeprom1); // address of eeprom, address of device uint16_t add, uint32_t data
          GREEN_LED = 1;
          setRgbColor(0, 255, 0);
          RED_LED = 0;



         }

         else if (isCommand(&data, "on", 0))
         {
             putsUart0("Inside 'on' block \r\n");
          // DMX stream sent continuously - MAB, STARTCODE, DATA etc. copy the stuffs from timerisr() ? timer1_BRK();

         //    if( DE_PIN)
         //    {

             timerisr_flag = 1;

             timer1_BRK();

             GREEN_LED = 1;
             RED_LED = 0;

          //   }

          //   else
          //       putsUart0("NOT Inside 'on' block \r\n");

         }

         else if (isCommand(&data, "clear", 0))
          {

                          putsUart0("Inside clear block \r\n");

                          for (i=0;i<max;i++)
                          {
                           global2[i] = 0;
                          }

                          GREEN_LED = 1;
                                      RED_LED = 0;

           //  else
            //     putsUart0("NOT Inside 'clear' block \r\n");

           }

           else if (isCommand(&data, "set", 2))

           {
               GREEN_LED = 1;
               RED_LED = 0;
                          putsUart0("Inside set block\r\n");

                          int32_t address       = getFieldInteger(&data, 1);
                          int32_t value         = getFieldInteger(&data, 2);



                          global2[address] = value;



            //   else
            //      putsUart0("NOT Inside 'set' block \r\n");

           }

           else if (isCommand(&data, "get", 1))          // return the value being sent to the address
           {

                          putsUart0("Inside get block\r\n");

                          int32_t address       = getFieldInteger(&data, 1);

                          //putsUart0( intToString ( (global[address]) ) );

                          snprintf(str, sizeof(str), "address:  %7"PRIu32"  \t\n\r",  global[address]);
                          putsUart0(str);
                          //GREEN_LED ^= 1;

            //   else
            //     putsUart0("NOT Inside 'get' block \r\n");
            }

            else if (isCommand(&data, "max", 1))
            {

                             putsUart0("Inside max block\r\n");

                             int32_t  MAX1 = getFieldInteger(&data, 1);
                             max = MAX1;
                             //GREEN_LED ^= 1;

             //   else
             //      putsUart0("NOT Inside 'max' block \r\n");
            }

         // lab 8 commands put them here as else if

            else if (isCommand(&data, "ramp", 4))
            {
                DE_PIN = 1;
                putsUart0("Inside ramp block\r\n");

                address_ramp       = getFieldInteger(&data, 1);
                time_ramp          = getFieldInteger(&data, 2);
                start_ramp         = getFieldInteger(&data, 3);
                stop_ramp          = getFieldInteger(&data, 4);

                increment = (float) (stop_ramp - start_ramp) / (float)  (time_ramp);


                increment = increment + 20;
                global2[address_ramp] = start_ramp;

                TIMER2_TAILR_R = 40000 * time_ramp ;                   // load value
                TIMER2_CTL_R  |= TIMER_CTL_TAEN; // enable timer 2a

            }

            else if (isCommand(&data, "pulse", 4))
            {
               putsUart0("Inside pulse block\r\n");

                address_pulse       = getFieldInteger(&data, 1);
                time_pulse          = getFieldInteger(&data, 2); //make sure it is in 50, 100 etc.
                first_pulse         = getFieldInteger(&data, 3);
                last_pulse          = getFieldInteger(&data, 4);

                TIMER0_TAILR_R = 40000 * time_pulse;                   // load value
                TIMER0_CTL_R  |= TIMER_CTL_TAEN; // enable timer 0a

            }

         else if (isCommand(&data, "off", 0))
         {
           // stop the DMX stream by turning off the timer

             putsUart0("Inside 'off' block \r\n");

             // uart here ?? UART4_CTL_R  &= ~UART_CTL_TXE ??

             timerisr_flag = 0;

             TIMER1_CTL_R  &= ~TIMER_CTL_TAEN ;

             //BLUE_LED = 0;
             //GREEN_LED = 0;
             setRgbColor(0, 0, 0);


         }

         else if (isCommand(&data, "device", 1))
        {
          DE_PIN = 0;
          UART4_CTL_R  &=  ~(UART_CTL_TXE);
                  UART4_CTL_R  |=  (UART_CTL_RXE );
          //device_flag = 1;
          BLUE_LED = 0;
          putsUart0("Inside 'device' block \r\n");

          int16_t address       = getFieldInteger(&data, 1);
          address1 = address;

          writeEeprom(addr_eeprom2,address); // uint16_t add, uint32_t data
          //GREEN_LED = 0;
          //RED_LED = 1;


         }

      //    else

      //    putsUart0("INVALID COMMAND FINAL");
        }

     }

}




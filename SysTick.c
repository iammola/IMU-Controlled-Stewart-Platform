/**
* Ademola Adedeji
* February 14, 2023
*
* Added SysTick_Countdown function and removed NVIC_ prefix from register names
*/

// SysTick.h
// Runs on LM4F120/TM4C123
// Provide functions that initialize the SysTick module, wait at least a
// designated number of clock cycles, and wait approximately a multiple
// of 10 milliseconds using busy wait.  After a power-on-reset, the
// LM4F120 gets its clock from the 16 MHz precision internal oscillator,
// which can vary by +/- 1% at room temperature and +/- 3% across all
// temperature ranges.  If you are using this module, you may need more
// precise timing, so it is assumed that you are using the PLL to set
// the system clock to 50 MHz.  This matters for the function
// SysTick_Wait10ms(), which will wait longer than 10 ms if the clock is
// slower.
// Daniel Valvano
// September 11, 2013

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2014
   Program 2.11, Section 2.6

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */
#include <stdint.h>
#include <stdbool.h>

#include "SysTick.h"
#include "tm4c123gh6pm.h"

 // Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(void) {
  ST_CTRL_R = 0;                   // disable SysTick during setup
  ST_RELOAD_R = ST_RELOAD_M;  // maximum reload value
  ST_CURRENT_R = 0;                // any write to current clears it
  // enable SysTick with core clock
  ST_CTRL_R = ST_CTRL_ENABLE + ST_CTRL_CLK_SRC;
}

// Time delay using busy wait.
// The delay parameter is in units of the core clock. (units of 12.5 nsec for 80 MHz clock)
void SysTick_Wait(uint32_t delay) {
  volatile uint32_t elapsedTime;
  uint32_t startTime = ST_CURRENT_R;

  do {
    elapsedTime = (startTime - ST_CURRENT_R) & ST_CURRENT_M;
  } while (elapsedTime <= delay);
}

// Time delay using busy wait.
// This assumes 80 MHz system clock.
void SysTick_Wait10ms(uint32_t delay) {
  uint32_t i;
  for (i = 0; i < delay; i++) {
    SysTick_Wait(800000);  // wait 10ms (assumes 80 MHz clock)
  }
}

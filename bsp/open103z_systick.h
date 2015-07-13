//--------------------------------------------------------------
// File     : stm32_ub_systick.h
//--------------------------------------------------------------

//--------------------------------------------------------------
#ifndef __OPEN103Z_SYSTICK_H
#define __OPEN103Z_SYSTICK_H

//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"


//--------------------------------------------------------------
// Resolution of the SysTick
// (Setting either 1us or 1000us as resolution)
//--------------------------------------------------------------
//#define  SYSTICK_RESOLUTION   1    // 1us resolution
#define  SYSTICK_RESOLUTION   1000   // 1ms resolution

//--------------------------------------------------------------
// Status of a Timer
//--------------------------------------------------------------
#if SYSTICK_RESOLUTION==1
  typedef enum {
    TIMER_STOP =0,  // Stop and reset timer
    TIMER_START_us, // Start timer in the us-Mode
    TIMER_START_ms, // Start timer in the ms-Mode
    TIMER_START_s,  // Start timer in thes-Mode
    TIMER_CHECK     // Test whether the timer expired
  }TIMER_STATUS_t;
#else
  typedef enum {
    TIMER_STOP =0,  // Stop and reset timer
    TIMER_START_ms, // Start timer in the ms-Mode
    TIMER_START_s,  // Start timer in the s-Mode
    TIMER_CHECK     // Test whether the timer expired
  }TIMER_STATUS_t;
#endif

typedef enum {
  TIMER_HOLD =0,  // Timer has expired
  TIMER_RUN       // Timer is still running
}TIMER_CHECK_t;


//--------------------------------------------------------------
// Status of a counter
//--------------------------------------------------------------
#if SYSTICK_RESOLUTION==1
  typedef enum {
    COUNTER_STOP =0,  // stop Counter
    COUNTER_START_us, // Counter start in the us-Mode
    COUNTER_START_ms, // Counter start in the ms-Mode
    COUNTER_START_s,  // Counter start in the s-Mode
    COUNTER_CHECK     // Test how much time has passed
  }COUNTER_STATUS_t;
#else
  typedef enum {
    COUNTER_STOP =0,  // stop Counter
    COUNTER_START_ms, // Counter start in the ms-Mode
    COUNTER_START_s,  // Counter start in the s-Mode
    COUNTER_CHECK     // Test how much time has passed
  }COUNTER_STATUS_t;
#endif

typedef struct {
  uint32_t factor;
  uint32_t value;
}COUNTER_t;


//--------------------------------------------------------------
// Global Pause-Functions
//--------------------------------------------------------------
void systick_init(void);
#if SYSTICK_RESOLUTION==1
  void systick_delayUs(volatile uint32_t pause);
#endif
void systick_delayMs(volatile uint32_t pause);
void systick_delayS(volatile uint32_t pause);


//--------------------------------------------------------------
// Global Timer-Functions
//--------------------------------------------------------------
TIMER_CHECK_t systick_timer1(TIMER_STATUS_t status, uint32_t wert);
TIMER_CHECK_t systick_timer2(TIMER_STATUS_t status, uint32_t wert);


//--------------------------------------------------------------
// Global Counter-Functions
//--------------------------------------------------------------
uint32_t systick_counter1(COUNTER_STATUS_t status);
uint32_t systick_counter2(COUNTER_STATUS_t status);

//--------------------------------------------------------------
#endif // __OPEN103Z_SYSTICK_H



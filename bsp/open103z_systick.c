//--------------------------------------------------------------
// File     : open103z_systick.c
// Datum    : 23.1.2014
// Version  : 1.0
// Autor    : Aytac Dilek
// EMail    : aytacdilek@gmail.com
// Web      :
// CPU      : STM32F1
// IDE      : CooCox CoIDE 1.7.7
// Module   : Open103z
// Function : Pause timer and counter functions in [us, ms, s] scale
//--------------------------------------------------------------

//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "open103z_systick.h"

#if ((SYSTICK_RESOLUTION!=1) && (SYSTICK_RESOLUTION!=1000))
  #error print WRONG SYSTICK RESOLUTION !
#endif


//--------------------------------------------------------------
// Global Pause-Functions
//--------------------------------------------------------------
static volatile uint32_t Systick_Delay;  // Global pauses count

//--------------------------------------------------------------
// Global Timer-Variables
//--------------------------------------------------------------
static volatile uint32_t Systick_T1;     // Global Timer1
static volatile uint32_t Systick_T2;     // Global Timer2

//--------------------------------------------------------------
// Global Counter-Variables
//--------------------------------------------------------------
static COUNTER_t Systick_C1;             // Global Counter1
static COUNTER_t Systick_C2;             // Global Counter2

//--------------------------------------------------------------
// Initialize System-Counter
// either in 1us clock or 1ms clock
//--------------------------------------------------------------
void systick_init(void) {
	RCC_ClocksTypeDef RCC_Clocks;

	Systick_Delay=0;
	Systick_T1=0;        	// Timer1 STOP
	Systick_T2=0;        	// Timer2 STOP
	Systick_C1.factor=0; 	// Counter1 STOP
	Systick_C1.value=0;
	Systick_C2.factor=0; 	// Counter2 STOP
	Systick_C2.value=0;


	#if SYSTICK_RESOLUTION==1
		// Setting the timer to 1us
		RCC_GetClocksFreq(&RCC_Clocks);
		SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000000);
	#else
		//Setting the timer to 1ms
		RCC_GetClocksFreq(&RCC_Clocks);
		SysTick_Config(RCC_Clocks.HCLK_Frequency / 1000);
	#endif
}

#if SYSTICK_RESOLUTION==1
//--------------------------------------------------------------
// Pause function (in us)
// the CPU waits until the time has expired
//--------------------------------------------------------------
void systick_delayUs(volatile uint32_t pause)
{

	Systick_Delay = pause;

	while(Systick_Delay != 0);
}
#endif


//--------------------------------------------------------------
// Pause function (in ms)
// the CPU waits until the time has expired
//--------------------------------------------------------------
void systick_delayMs(volatile uint32_t pause)
{
	#if SYSTICK_RESOLUTION==1
		uint32_t ms;

		for(ms=0;ms<pause;ms++) {
			UB_Systick_Pause_us(1000);
		}
	#else
		Systick_Delay = pause;

		while(Systick_Delay != 0);
	#endif
}


//--------------------------------------------------------------
// Delay function (in s)
// the CPU waits until the time has expired
//--------------------------------------------------------------
void systick_delayS(volatile uint32_t pause)
{
	uint32_t s;

	for(s=0;s<pause;s++) {
		systick_delayMs(1000);
	}
}


//--------------------------------------------------------------
// Timer1
// Timer can be started with a time
// and can then be queried cyclically whether it has expired
// status : [TIMER_STOP, TIMER_START_us/ms/s, TIMER_CHECK]
// Value : Start value in [us, ms, sec]
// Return value at TIMER_CHECK :
//  -> if timer expired 				= TIMER_HOLD
//  -> if the timer is still running	= TIMER_RUN
//--------------------------------------------------------------
TIMER_CHECK_t systick_timer1(TIMER_STATUS_t status, uint32_t value)
{
	TIMER_CHECK_t ret_value = TIMER_RUN;

	if(status == TIMER_STOP) {
		// Stop Timer1 quickly
		Systick_T1 = 1;
		ret_value = TIMER_HOLD;
	}
	#if SYSTICK_RESOLUTION==1
		else if(status==TIMER_START_us) {
			// Start Timer1 in the us-Mode
			Systick_T1=value;
		}
		else if(status==TIMER_START_ms) {
			// Start Timer1 in the ms-Mode
			Systick_T1=1000*value;
		}
		else if(status==TIMER_START_s) {
			// Start Timer1 in the s-Mode
			if(value>4293) value=4293;
			Systick_T1=1000*1000*value;
			}
	#else
		else if(status == TIMER_START_ms) {
			// Start Timer1 in the ms-Mode
			Systick_T1 = value;
		}
		else if(status == TIMER_START_s) {
			// Start Timer1 in the s-Mode
			Systick_T1 = 1000*value;
		}
	#endif

	else {
		// return status
		if(Systick_T1!=0x00) {
			// Timer is still running
			ret_value=TIMER_RUN;
		}
		else {
			// Timer has expired
			ret_value=TIMER_HOLD;
		}
	}

	return(ret_value);
}


//--------------------------------------------------------------
// Timer2
// Timer can be started with a time
// and can then be queried cyclically whether it has expired
// status : [TIMER_STOP, TIMER_START_us/ms/s, TIMER_CHECK]
// data: initial value in [us, ms, sec]
// Return Value at TIMER_CHECK :
//  -> if timer expired 				= TIMER_HOLD
//  -> when the timer is still running 	= TIMER_RUN
//--------------------------------------------------------------
TIMER_CHECK_t systick_timer2(TIMER_STATUS_t status, uint32_t data)
{
	TIMER_CHECK_t ret_value=TIMER_RUN;

	if(status==TIMER_STOP) {
		// Stop Timer2 quickly
		Systick_T2 = 1;
		ret_value = TIMER_HOLD;
	}
	#if SYSTICK_RESOLUTION==1
    	else if(status == TIMER_START_us) {
    		// Start Timer2 in the us-Mode
    		Systick_T2=data;
    	}
    	else if(status==TIMER_START_ms) {
    		// Start Timer2 in the ms-Mode
    		Systick_T2=1000*data;
    	}
    	else if(status==TIMER_START_s) {
    		// Start Timer2 in the s-Mode
    		if(data>4293) data=4293;
    		Systick_T2=1000*1000*data;
    	}
	#else
	else if(status==TIMER_START_ms) {
		// Start Timer2 in the ms-Mode
		Systick_T2=data;
	}
	else if(status==TIMER_START_s) {
		// Start Timer2 in the s-Mode
		Systick_T2=1000*data;
	}
	#endif
	else {
		// return status
		if(Systick_T2!=0x00) {
			// Timer is still running
			ret_value=TIMER_RUN;
		}
		else {
			// Timer has expired
			ret_value=TIMER_HOLD;
		}
	}

	return(ret_value);
}


//--------------------------------------------------------------
// Counter1
// Counter can be started at 0 and at a second time,
// the elapsed time can be read
// status : [COUNTER_STOP, COUNTER_START_us/ms/s, COUNTER_CHECK]
// Return value at COUNTER_CHECK :
//  -> elapsed time in [us, ms, sec]
//--------------------------------------------------------------
uint32_t systick_counter1(COUNTER_STATUS_t status)
{
	uint32_t ret_value = 0;

	if(status==COUNTER_STOP) {
		// stop Counter1
		Systick_C1.factor = 0;
	}
	#if SYSTICK_RESOLUTION==1
	else if(status == COUNTER_START_us) {
		// Start Counter1 in the us-Mode
		Systick_C1.factor = 1;
		Systick_C1.value = 0;
    }
    else if(status == COUNTER_START_ms) {
    	// Start Counter1 in the ms-Mode
    	Systick_C1.factor = 1000;
    	Systick_C1.value = 0;
    }
    else if(status == COUNTER_START_s) {
    	// Start Counter1 in the s-Mode
    	Systick_C1.factor = 1000 * 1000;
    	Systick_C1.value = 0;
    }
	#else
    else if(status==COUNTER_START_ms) {
    	// Start Counter1 in the ms-Mode
    	Systick_C1.factor=1;
    	Systick_C1.value=0;
    }
    else if(status==COUNTER_START_s) {
    	// Start Counter1 in the s-Mode
    	Systick_C1.factor=1000;
    	Systick_C1.value=0;
    }
	#endif
    else if(status==COUNTER_CHECK) {
    	// Return elapsed time
    	ret_value = Systick_C1.value / Systick_C1.factor;
    }

	return(ret_value);
}


//--------------------------------------------------------------
// Counter2
// Counter can be started at 0 and at a second time,
// the elapsed time can be read
// status : [COUNTER_STOP, COUNTER_START_us/ms/s, COUNTER_CHECK]
// Return value at COUNTER_CHECK:
//  -> elapsed time in [us, ms, sec]
//--------------------------------------------------------------
uint32_t systick_counter2(COUNTER_STATUS_t status)
{
	uint32_t ret_value=0;

	if(status == COUNTER_STOP) {
		// stop Counter2
		Systick_C2.factor=0;
	}
	#if SYSTICK_RESOLUTION==1
	else if(status==COUNTER_START_us) {
		// Start Counter2 in the us-Mode
		Systick_C2.factor=1;
		Systick_C2.value=0;
    }
    else if(status==COUNTER_START_ms) {
    	// Start Counter2 in the ms-Mode
    	Systick_C2.factor=1000;
    	Systick_C2.value=0;
    }
    else if(status==COUNTER_START_s) {
    	// Start Counter2 in the s-Mode
    	Systick_C2.factor=1000*1000;
    	Systick_C2.value=0;
    }
	#else
    else if(status==COUNTER_START_ms) {
    	// Start Counter2 in the ms-Mode
    	Systick_C2.factor=1;
      	Systick_C2.value=0;
    }
    else if(status==COUNTER_START_s) {
    	// Start Counter2 in the s-Mode
    	Systick_C2.factor=1000;
    	Systick_C2.value=0;
    }
	#endif
    else if(status==COUNTER_CHECK) {
    	// return elapsed time
    	ret_value=Systick_C2.value/Systick_C2.factor;
    }

	return(ret_value);
}


//--------------------------------------------------------------
// System-Interrupt
//--------------------------------------------------------------
void SysTick_Handler(void)
{
	// Tick to pause
	if(Systick_Delay != 0x00) {
		Systick_Delay--;
	}
	// Tick to Timer1
	if(Systick_T1 != 0x00) {
		Systick_T1--;
	}
	// Tick to Timer2
	if(Systick_T2 != 0x00) {
		Systick_T2--;
	}
	// Tick to Counter1
	if(Systick_C1.factor != 0x00) {
		Systick_C1.value++;
	}
	// Tick to Counter2
	if(Systick_C2.factor != 0x00) {
		Systick_C2.value++;
	}
}



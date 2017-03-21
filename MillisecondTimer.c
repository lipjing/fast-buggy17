#include "plib/timers.h"
#include "global_defines.h"

volatile unsigned int millisecond_COUNT;

//Returns value of millisecond counter
unsigned int ReadMillis(void) {

    return (millisecond_COUNT);

}

//Resets millisecond counter and re-loads Timer0. 
//Be careful when using this whilst system clock is in use, resetting millisecond count may cause inaccurate timekeeping by system clock
void ResetMillis(void) {

    WriteTimer0(TIMER0_VALUE);
    millisecond_COUNT = 0;

}

inline void MillisecondTimerISR(void) {

    WriteTimer0(TIMER0_VALUE);  //Re-load Timer0 for next delay
    millisecond_COUNT++;        //Increment millisecond counter
}

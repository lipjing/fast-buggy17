#include "plib/timers.h"
#include "global_defines.h"

volatile unsigned int millisecond_COUNT0, millisecond_COUNT1, millisecond_COUNT2, millisecond_COUNT3;

//Returns value of millisecond counter
unsigned int ReadMillis0(void) {

    return (millisecond_COUNT0);

}

unsigned int ReadMillis1(void) {

    return (millisecond_COUNT1);

}

unsigned int ReadMillis2(void) {

    return (millisecond_COUNT2);

}

unsigned int ReadMillis3(void) {

    return (millisecond_COUNT3);

}

//Resets millisecond counter and re-loads Timer0. 
//Be careful when using this whilst system clock is in use, resetting millisecond count may cause inaccurate timekeeping by system clock
void ResetMillis0(void) {

    millisecond_COUNT0 = 0;

}

void ResetMillis1(void) {

    millisecond_COUNT1 = 0;

}

void ResetMillis2(void) {
    
    millisecond_COUNT2 = 0;
}

void ResetMillis3(void) {
    
    millisecond_COUNT3 = 0;
}

inline void MillisecondTimerISR(void) {

    WriteTimer0(TIMER0_VALUE);  //Re-load Timer0 for next delay
    millisecond_COUNT0++;        //Increment millisecond counter
    millisecond_COUNT1++;
    millisecond_COUNT2++;
    millisecond_COUNT3++;
}

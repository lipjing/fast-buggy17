typedef struct {
    unsigned int milliseconds;
    unsigned char seconds;
    unsigned char minutes;
    unsigned int hours;
} SYSTEMTIME;

volatile SYSTEMTIME SystemClock;
volatile unsigned char second_rollover_flag; 
unsigned char system_clock_enable;

//Resets System Clock back to zero
void ResetSystemClock(void) {
    
    SystemClock.hours = 0;
    SystemClock.minutes = 0;
    SystemClock.seconds = 0;
    SystemClock.milliseconds = 0;
    
}

//Enables System Clock; ISR and time calculation
void EnableSystemClock(void) {
    
    system_clock_enable = 1;
    
}

//Disables System Clock: ISR and time calculation
void DisableSystemClock(void) {
    
    system_clock_enable = 0;
    
}

//ISR routine for System Clock; place this within the TimerX ISR which generates the millisecond count
inline void SystemClockISR(void) {
    
    if(system_clock_enable == 1) {
        if(SystemClock.milliseconds < 999) {
            SystemClock.milliseconds++;
        }
        else {
            SystemClock.milliseconds = 0;
            second_rollover_flag = 1;
        }
    }
    
}

//Calculate the system clock time; ensure this function is called at least once per second within main() for accurate timekeeping
void SystemClockCalcTime(void) {
    
    if(system_clock_enable == 1) {
        if(second_rollover_flag == 1) {
            second_rollover_flag = 0;
            if(SystemClock.seconds < 59) {
                SystemClock.seconds++;
            }
            else {
                SystemClock.seconds = 0;
                if(SystemClock.minutes < 59) {
                    SystemClock.minutes++;
                }
                else {
                    SystemClock.minutes = 0;
                    SystemClock.hours++;
                }
            }
        }
    }
    
}

unsigned int SystemClockGetMillis(void) {
    
    return(SystemClock.milliseconds);
    
}

unsigned char SystemClockGetSecs(void) {
    
    return(SystemClock.seconds);
    
}

unsigned char SystemClockGetMins(void) {
    
    return(SystemClock.minutes);
    
}

unsigned int SystemClockGetHrs(void) {
    
    return(SystemClock.hours);
    
}

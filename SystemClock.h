/* 
 * File:   SystemClock.h
 * Author: Jack
 *
 * Created on 20 March 2017, 11:47
 */

#ifndef SYSTEMCLOCK_H
#define	SYSTEMCLOCK_H

#ifdef	__cplusplus
extern "C" {
#endif

    void ResetSystemClock(void);
    
    void EnableSystemClock(void);
    
    void DisableSystemClock(void);
    
    inline void SystemClockISR(void);
    
    void SystemClockCalcTime(void);
    
    unsigned int SystemClockGetMillis(void);
    
    unsigned char SystemClockGetSecs(void);
    
    unsigned char SystemClockGetMins(void);
    
    unsigned int SystemClockGetHrs(void);


#ifdef	__cplusplus
}
#endif

#endif	/* SYSTEMCLOCK_H */


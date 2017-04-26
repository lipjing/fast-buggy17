/* 
 * File:   MillisecondTimer.h
 * Author: Jack
 *
 * Created on 21 March 2017, 18:36
 */

#ifndef MILLISECONDTIMER_H
#define	MILLISECONDTIMER_H

#ifdef	__cplusplus
extern "C" {
#endif

    unsigned int ReadMillis(void);
    
    void ResetMillis0(void);

    void ResetMillis1(void);
    
    void ResetMillis2(void);
    
    void ResetMillis3(void);
    
    inline void MillisecondTimerISR(void);
    
    unsigned int ReadMillis0(void);
    
    unsigned int ReadMillis1(void);
    
    unsigned int ReadMillis2(void);
    
    unsigned int ReadMillis3(void);




#ifdef	__cplusplus
}
#endif

#endif	/* MILLISECONDTIMER_H */


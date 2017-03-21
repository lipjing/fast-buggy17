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
    
    void ResetMillis(void);
    
    inline void MillisecondTimerISR(void);



#ifdef	__cplusplus
}
#endif

#endif	/* MILLISECONDTIMER_H */


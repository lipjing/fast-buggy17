/* 
 * File:   global_defines.h
 * Author: Jack
 *
 * Created on 21 March 2017, 16:39
 */

#ifndef GLOBAL_DEFINES_H
#define	GLOBAL_DEFINES_H

#ifdef	__cplusplus
extern "C" {
#endif

    //Motor control pin connections
#define MOTOR_L_BI  LATDbits.LATD0
#define MOTOR_L_DIR LATDbits.LATD1
#define MOTOR_R_BI  LATDbits.LATD2
#define MOTOR_R_DIR LATDbits.LATD3
#define MOTOR_EN    LATDbits.LATD4

    //Motor duty cycle defines - duty cycles are in reverse - lower numbers equal higher motor speeds
#define DC_MAX_SPEED 425
#define DC_STOP 500

#define TIMER0_VALUE    63036    //Value written to Timer0 to generate ~1ms delay


#ifdef	__cplusplus
}
#endif

#endif	/* GLOBAL_DEFINES_H */


/* 
 * File:   Motors.h
 * Author: Jack
 *
 * Created on 21 March 2017, 16:24
 */

#ifndef MOTORS_H
#define	MOTORS_H

#ifdef	__cplusplus
extern "C" {
#endif

    void SetDCMotorPID(int PIDoutput);

    void SetDCMotorL(unsigned int duty_cycle);

    void SetDCMotorR(unsigned int duty_cycle);
    
    unsigned int ReadDCMotorL(void);
    
    unsigned int ReadDCMotorR(void);

    void SetDirectionForward(void);

    void SetDirectionReverse(void);

    void SetForwardsMotorR(void);

    void SetForwardsMotorL(void);

    void SetReverseMotorR(void);

    void SetReverseMotorL(void);

    void EnableMotors(void);

    void DisableMotors(void);
    
    void SetBipolar(void);
    
    void SetUnipolar(void);

    void StopMotors(void);


#ifdef	__cplusplus
}
#endif

#endif	/* MOTORS_H */


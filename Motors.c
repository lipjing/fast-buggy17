#include "xc.h"
#include "global_defines.h"

void SetDCMotorL(unsigned int duty_cycle) {
    
    SetDCPWM5(duty_cycle);
    
}

void SetDCMotorR(unsigned int duty_cycle) {
    
    SetDCPWM4(duty_cycle);
    
}

void SetDCMotorPID(int PIDoutput) {
    
    if(PIDoutput < 0) {
        SetDCMotorL(DC_MAX_SPEED - PIDoutput);
        SetDCMotorR(DC_MAX_SPEED);
    }
    else {
        SetDCMotorL(DC_MAX_SPEED);
        SetDCMotorR(DC_MAX_SPEED + PIDoutput);
    }
    
}

void SetDirectionForward(void) {
    
    MOTOR_L_DIR = 1;
    MOTOR_R_DIR = 1;
    
}

void SetDirectionReverse(void) {
    
    MOTOR_L_DIR = 0;
    MOTOR_R_DIR = 0;
    
}

void SetForwardsMotorR(void) {
    
    MOTOR_R_DIR = 1;
    
}

void SetForwardsMotorL(void) {
    
    MOTOR_L_DIR = 1;
    
}

void SetReverseMotorR(void) {
    
    MOTOR_R_DIR = 0;
    
}

void SetReverseMotorL(void) {
    
    MOTOR_L_DIR = 0;
    
}

void EnableMotors(void) {
    
    MOTOR_EN = 1;
    
}

void DisableMotors(void) {
    
    MOTOR_EN = 0;
    
}

void SetBipolar(void) {
    
    MOTOR_R_BI = 1;
    MOTOR_L_BI = 1;
    
}

void SetUnipolar(void) {
    
    MOTOR_R_BI = 0;
    MOTOR_L_BI = 0;
    
}

void StopMotors(void) {
    
    SetDCMotorR(DC_STOP);
    SetDCMotorL(DC_STOP);
    
}

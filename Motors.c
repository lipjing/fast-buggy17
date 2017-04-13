#include "xc.h"
#include "global_defines.h"

unsigned int DCmotorL, DCmotorR;

void SetDCMotorL(unsigned int duty_cycle) {
    
    SetDCPWM5(duty_cycle);
//    if(duty_cycle >= DC_STOP) {
//        SetDCPWM5(DC_STOP);
//        
//    } else {
//        SetDCPWM5(duty_cycle);        
//    }

    
}

void SetDCMotorR(unsigned int duty_cycle) {
    
    SetDCPWM4(duty_cycle);
//    if(duty_cycle >= DC_STOP) {
//        SetDCPWM4(DC_STOP);
//        
//    } else {
//        SetDCPWM4(duty_cycle);        
//    }
    
}

unsigned int ReadDCMotorL(void) {
    unsigned int DCreg = 0;
    
    DCreg ^= 0x03FF & ((CCPR5L << 2) | (CCP5CON >> 4));
    
    return(DCreg);
}

unsigned int ReadDCMotorR(void) {
    unsigned int DCreg = 0;
    
    DCreg ^= 0x03FF & ((CCPR4L << 2) | (CCP4CON >> 4));
    
    return(DCreg);
}

void SetDCMotorPID(int PIDoutput) {
    
    if(PIDoutput <= 0) {
        SetDCMotorL(DC_MAX_SPEED - PIDoutput);
        SetDCMotorR(DC_MAX_SPEED);
    }
    else if(PIDoutput > 0) {
        SetDCMotorL(DC_MAX_SPEED);
        SetDCMotorR(DC_MAX_SPEED + PIDoutput);
    }
    else if(PIDoutput == 0) {
        SetDCMotorL(DC_MAX_SPEED);
        SetDCMotorR(DC_MAX_SPEED);
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

//void Ramp_Motors_Down(void){
//    unsigned int stepL, stepR;
//    unsigned char index;
//
//    
//    stepL = ReadDCMotorL() / NO_OF_STEPS;
//    stepR = ReadDCMotorR() / NO_OF_STEPS;
//
//    for(index = NO_OF_STEPS; index >= 0; index--) {
//        SetDCMotorL(ReadDCMotorL() - stepL);
//        SetDCMotorR(ReadDCMotorR() - stepR);
//    }
//}

//void Ramp_Motors_Up(void){
//    
//
//}
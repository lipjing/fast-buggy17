/*
 * Filename: PWM_ramp_test.c
 * Author:
 * Date: 14/02/2017
 * Description: This program repeatedly ramps up the PWM duty cycle from 0 to its maximum value, whilst allowing the
 * user to set the motor drive board configuration (Bipolar/Direction) using toggle switches on the School IO board.
 * Using the School Microcontroller Board (10MHz) oscillator results in a PWM
 * Frequency of approximately 19kHz. The PWM resolution with this configuration is approximately 9 bits.  
*/

//Pre-processor include directives
#include "xc.h"
#include "config_18f8722.h"
#include "plib\pwm.h"
#include "plib\adc.h"

//Pre-processor define directives
#define PERIOD_REG 130      //PWM Period Register (PR2) value. This equates to a PWM Frequency of approx. 19kHz
#define MAX_PWM_DC 526      //Maximum PWM Duty Cycle register value (based on no. of bits resolution of PWM module)

//Define statements for Left & Right Motor Bipolar/Direction/Enable bits
#define MOTOR_L_BI  LATDbits.LATD0
#define MOTOR_L_DIR LATDbits.LATD1
#define MOTOR_R_BI  LATDbits.LATD2
#define MOTOR_R_DIR LATDbits.LATD3
#define MOTOR_EN    LATDbits.LATD4

//Function prototypes
void StartTimer2(void);     //Configures Timer2 for PWM operation and loads register PR2 with value required for desired PWM period
void ConfigPWM(void);       //Configures CCP4/5 modules for PWM operation


void main(void) {

    unsigned int duty_cycle = 0;        //Declare and initialise variables to be used in the program

    //TRIS configuration for use with motor drive board
    TRISD = 0xE0;           //Bits RD0-RD4 of PORTD as outputs to control Bipolar/Direction/Enable signals
    TRISGbits.RG3 = 0;      //Bits RG3 and RG4 as outputs for PWM output
    TRISGbits.RG4 = 0;

    //TRIS configuration for use with School IO Board
    TRISF = 0x00;           //All bits of PORTF as outputs for LEDs
    TRISA = 0xEF;           //Bit RA4 as output to control LED enable transistor
    TRISH = 0xFC;           //Bits RH4-RH7 as inputs for toggle switches, RH0-RH1 as outputs for 7-segment display enable transistors
    TRISC = 0xFF;           //All bits of PORTC as inputs for toggle switches
    TRISJ = 0xBF;           //RJ5 as input for PB1, RJ6 as output for buzzer

    LATF = 0x00;            //Clear LATF to turn off LEDs
    LATAbits.LATA4 = 0;     //Disable LEDs
    LATHbits.LATH0 = 1;     //Disable both 7-segment displays
    LATHbits.LATH1 = 1;

    LATDbits.LATD4 = 0;     //Disable motor drive outputs
    LATAbits.LATA4 = 1;     //Enable LEDs

    StartTimer2();
    ConfigPWM();

    while (1) {
        
        //Control motor drive board inputs with toggle switches by reading switches and outputting them to LATD
        MOTOR_EN     = PORTHbits.RH7;
        MOTOR_L_BI   = PORTCbits.RC2;
        MOTOR_L_DIR  = PORTCbits.RC3;
        MOTOR_R_BI   = PORTCbits.RC5;
        MOTOR_R_DIR  = PORTHbits.RH4;

        LATF = (PORTH & 0xF0) | ((PORTC >> 2) & 0x0F);  //Display switch configuration on LEDs

        for (duty_cycle = 0; duty_cycle <= MAX_PWM_DC; duty_cycle++) {     //Ramp up 
            SetDCPWM4(duty_cycle);
            SetDCPWM5(duty_cycle);
            Delay1KTCYx(20);
        }

    }


}

void StartTimer2(void) {
    PIE1bits.TMR2IE = 0;        //Disable Timer2 interrupts
    T2CONbits.T2CKPS1 = 0;      //Set Timer2 pre-scaler to 1:1
    T2CONbits.T2CKPS0 = 0;
    T2CONbits.TMR2ON = 1;       //Turn on Timer2
    PR2 = PERIOD_REG;           //Write PWM Period value to period register
}

void ConfigPWM(void) {
    CCP4CONbits.CCP4M3 = 1;     //Configure CCP4 module for PWM operation
    CCP4CONbits.CCP4M2 = 1;
    CCP4CONbits.CCP4M1 = 0;
    CCP4CONbits.CCP4M0 = 0;

    CCP5CONbits.CCP5M3 = 1;     //Configure CCP5 module for PWM operation
    CCP5CONbits.CCP5M2 = 1;
    CCP5CONbits.CCP5M1 = 0;
    CCP5CONbits.CCP5M0 = 0;
}
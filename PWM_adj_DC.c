/*
 * Filename: PWM_adj_DC.c
 * Author:
 * Date: 14/02/2017
 * Description: This program allows the user to adjust the duty cycle of the PWM waveform sent to the motors using the
 * potentiometer fitted to the School IO Board. Using the School Microcontroller Board (10MHz) oscillator results in a PWM
 * Frequency of approximately 19kHz. The PWM resolution with this configuration is approximately 9 bits. 
 *  
*/


#include "xc.h"
#include "config_18f8722.h"
#include "plib\pwm.h"
#include "plib\adc.h"

//Pre-processor define directives
#define PERIOD_REG 130      //PWM Period Register (PR2) value. This equates to a PWM Frequency of approx. 19kHz

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

    unsigned int ADC_val, duty_cycle;       //Declare and initialise variables to be used in the program

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

    //Configure ADC to read from potentiometer
    OpenADC(ADC_FOSC_8 &        //ADC freq. Fosc/8   
            ADC_RIGHT_JUST &    //ADC results are right-justified in result register pair
            ADC_0_TAD,          //0TAD acquisition delay as ADC channel remains the same throughout
            ADC_CH0 &           //ADC input taken from CH0
            ADC_INT_OFF &       //ADC does not issue interrupts
            ADC_VREFPLUS_VDD,   //+/-Vref comes from +5V & 0V
            14                  //ADCON1 configuration to make pin RA0 analogue input
            );

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

        ConvertADC();       //Start an ADC conversion
        
        while(BusyADC());   //Loop until conversion is complete
        
        ADC_val = ReadADC() & 0x03FF;       //Read ADC value into ADC_val and mask off upper 6 bits to produce 10 bit value
        duty_cycle = ((ADC_val * 2) >> 2) & 0x03FF;     //Scale ADC input to duty cycle for PWM. This operation is equivalent to: duty_cycle = ADC_val * 0.51
        
        //Set duty cycles of both PWM4/5 to control motor speed
        SetDCPWM4(duty_cycle);
        SetDCPWM5(duty_cycle);

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
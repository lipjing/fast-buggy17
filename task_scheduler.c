/*
 * Filename: task_scheduler.c
 * Author:
 * Date: 
 * Description: 
*/

//Pre-processor include directives
#include "config_18f8722.h"
#include "plib/adc.h"
#include "plib/timers.h"
#include "plib/delays.h"
#include "plib/usart.h"
#include "stdio.h"

#define TIMER0_VALUE    63036    //Value written to Timer0 to generate ~1ms delay
#define PERIOD_REG      130      //Value written to PR2 register to set PWM frequency (approx. 19kHz)
#define ECHO_TO_DIST_CM 0.0137   //Multiplier for echo pulse length to convert to distance in CM
#define ECHO_TO_DIST_IN 0.054    //Multiplier for echo pulse length to convert to distance in INCHES
#define NO_OF_SENSORS   5        //Number of sensors in use

volatile unsigned int ms_count, echo_time;
volatile unsigned char echo_acq_done, sensor_acq_done, sensor_acq_index;
volatile int (*sensor_readings)[5];

//High-priority ISR
void interrupt high_priority isrHP(void) {
    
    //Timer0 ISR
    if(INTCONbits.TMR0IF == 1) {
        INTCONbits.TMR0IF = 0;
        ms_count++;      
        WriteTimer0(TIMER0_VALUE);
    }
    
    //PORTB ISR
    if(INTCONbits.RBIF == 1) {         //Check to see if interrupt has come from PORTB (ultrasonic sensor return pulse)
        INTCONbits.RBIE = 0;            //If it has, temporarily disable PORTB interrups
        if(PORTBbits.RB4 == 1) {       //If RB4 edge is a rising edge (start of echo pulse)
            T1CONbits.TMR1ON = 1;       //Enable Timer0 to start timing pulse length
            WriteTimer1(0);             //Then set Timer0 to 0 to reset it
        }
        if(PORTBbits.RB4 == 0) {       //If RB4 edge is falling edge (end of echo pulse)
            T1CONbits.TMR1ON = 0;       //Turn off Timer0 to stop timing pulse length
            echo_acq_done = 1;             //Set echo flag to indicate that echo reading is ready for processing
        }
        INTCONbits.RBIE = 1;            //Re-enable PORTB interrupts
        INTCONbits.RBIF = 0;            //Clear the interrupt flag
    }
    
    
}

//Low-priority ISR
void interrupt low_priority isrLP(void) {
    
    //INT1 (PB1) ISR
    if(INTCON3bits.INT1IF == 1) {
        
        INTCON3bits.INT1IF = 0;
    }
    
    //INT2 (PB2) ISR
    if(INTCON3bits.INT2IF == 1) {
        
        INTCON3bits.INT2IF = 0;
    }
    
    //ADC ISR
    if(PIR1bits.ADIF == 1) {
        PIR1bits.ADIF = 0;
        if(sensor_acq_index < NO_OF_SENSORS) {
            *(sensor_readings + sensor_acq_index) = ReadADC();
            sensor_acq_index++;
            SetADCChannel(sensor_acq_index);
            ConvertADC();
        }
        else {
            sensor_acq_done = 1;
            PIE1bits.ADIE = 0;
        }   
    }
    
}

//Configure interrupt sources
void ConfigureInterrupts(void) {
    
    INTCONbits.GIE = 1;         //Enable global interrupts
    INTCONbits.PEIE = 1;        //Enable peripheral interrupts
    RCONbits.IPEN = 1;          //Prioritised mode enabled
    
    //INT1 (PB1) Interrupt
    INTCON3bits.INT1E = 1;      //Enable INT1 (PB1) interrupt
    INTCON3bits.INT1IF = 0;     //Clear flag
    INTCON2bits.INTEDG1 = 1;    //Rising-edge triggered
    INTCON3bits.INT1IP = 0;     //Low priority when priority mode enabled

    
    //INT1 (PB2) Interrupt
    INTCON3bits.INT2E = 1;      //Enable INT2 (PB2) interrupt
    INTCON3bits.INT2IF = 0;     //Clear flag
    INTCON2bits.INTEDG2 = 1;    //Rising-edge triggered
    INTCON3bits.INT2IP = 0;     //Low priority when prioritised mode enabled

       
    //PORTB Interrupt
    INTCONbits.RBIE = 0;        //Disable PORTB interrupt for now
    INTCONbits.RBIF = 0;        //Clear flag
    INTCON2bits.RBIP = 1;       //High priority when prioritised mode enabled

    
    //Timer0 Interrupt
    INTCONbits.TMR0IE = 1;      //Enable Timer0 interrupt
    INTCONbits.TMR0IF = 0;      //Clear flag
    INTCON2bits.TMR0IP = 1;     //High priority when prioritised mode enabled
    
    //Timer1 Interrupt
    PIE1bits.TMR1IE = 0;        //Disable Timer1 interrupt
    
    //Timer2 Interrupt
    PIE1bits.TMR2IE = 0;        //Disable Timer2 interrupt
    
    //ADC Interrupt
    PIE1bits.ADIE = 0;          //Disable ADC interrupt for now
    PIR1bits.ADIF = 0;          //Clear flag
    IPR1bits.ADIP = 0;          //Low priority when prioritised mode enabled

    //USART1 Interrupt
    PIE1bits.RC1IE = 0;         //Disable USART1 Rx interrupt for now
    PIE1bits.TX1IE = 0;         //Disable USART1 Tx interrupt for now
    PIR1bits.RC1IF = 0;         //Clear Rx flag
    PIR1bits.TX1IF = 0;         //Clear Tx flag
    IPR1bits.RC1IP = 0;         //Low priority (Rx) when prioritised mode enabled
    IPR1bits.TX1IP = 0;         //Low priority (Tx) when prioritised mode enabled
}

//Configure Timer0
void ConfigureTimer0(void) {
    
    OpenTimer0(TIMER_INT_ON     //Interrupts enabled
            & T0_16BIT          //16-bit mode
            & T0_SOURCE_INT     //Increment on internal instruction clock
            & T0_PS_1_1);       //Prescaler 1:1
    
    //T0CON = 0xFF;             //Achieves same as OpenTimer0 function, uncomment if OpenTimerX doesn't work on some versions of XC8
    
    WriteTimer0(TIMER0_VALUE);

}

//Configure Timer1
void ConfigureTimer1(void) {
    
    OpenTimer1(TIMER_INT_OFF    //Interrupts disabled
            & T1_16BIT_RW       //16-bit mode
            & T1_SOURCE_INT     //Increment on internal instruction clock
            & T1_PS_1_2         //Prescaler 1:2
            & T1_SYNC_EXT_OFF); //External clock synchronisation off
    
    //T1CON = 0x9D;              //Achieves same as OpenTimer1 function, uncomment if OpenTimerX doesn't work on some versions of XC8
    
    T1CONbits.TMR1ON = 0;       //Turn off Timer1 for now, will be enabled when ultrasonic distance is requested
    
    WriteTimer1(0);

}

//Configure Timer2
void ConfigureTimer2(void) {
    
    OpenTimer2(TIMER_INT_OFF    //Interrupts disabled
            & T2_PS_1_1         //Prescaler 1:1
            & T2_POST_1_1);     //Postscaler 1:1
    
    //T2CON = 0x04;               //Achieves same as OpenTimer2 function, uncomment if OpenTimerX doesn't work on some versions of XC8
    
}

//Configure PWM
void ConfigurePWM(void) {
    
    OpenPWM4(PERIOD_REG);       //Set up CCP4 in PWM mode
    OpenPWM5(PERIOD_REG);       //Set up CCP5 in PWM mode
    
//    //Uncomment lines below if OpenPWMX functions are not working on the version of XC8 used
//    //Configure CCP4 module for PWM operation
//    CCP4CONbits.CCP4M3 = 1;     
//    CCP4CONbits.CCP4M2 = 1;
//    CCP4CONbits.CCP4M1 = 0;
//    CCP4CONbits.CCP4M0 = 0;
//
//    //Configure CCP5 module for PWM operation
//    CCP5CONbits.CCP5M3 = 1;     
//    CCP5CONbits.CCP5M2 = 1;
//    CCP5CONbits.CCP5M1 = 0;
//    CCP5CONbits.CCP5M0 = 0;
//    
//    PR2 = PERIOD_REG;
    
}

//Configures IO for buggy
void ConfigureBuggyIO(void) {
    
    //TRISA
    TRISA = 0xFF;
    
    //TRISB
    TRISB = 0xFF;
    
    //TRISC
    TRISC = 0xBF;
    
    //TRISD
    TRISD = 0xFF;
    
    //TRISE
    TRISE = 0x80;
    
    //TRISG
    TRISG = 0xE7;
    
    //TRISJ
    TRISJ = 0xEF;
    
}

//Configures ADC for sensor readings
void ConfigureADC(void) {
    
    OpenADC(ADC_FOSC_8              //ADC clock of FOSC/8
            & ADC_RIGHT_JUST        //Results right-justified in results registers
            & ADC_4_TAD,           //Acquisition delay as we are changing the ADC channel; to give time for the hold capacitor to charge/discharge
            ADC_CH0                 //Start off with Channel 0 selected
            & ADC_INT_OFF           //Disable interrupts for now
            & ADC_VREFPLUS_VDD      //Vref comes from PIC power rails (+5V/0V)
            & ADC_VREFMINUS_VSS,
            10);                    //Enables AN0-AN4 as analogue inputs, rest are standard port pins (digital)
    
}

//Configure USART1 for serial port
void ConfigureSerial(void) {
    
    Open1USART(USART_TX_INT_OFF     //Disable Tx and Rx interrupts
            & USART_RX_INT_OFF
            & USART_ASYNCH_MODE     //Asynchronous mode (no clock)
            & USART_EIGHT_BIT       //8-bit mode
            & USART_CONT_RX         //Continuous Rx
            & USART_BRGH_HIGH,      //BRG in high-speed mode
            64);                    //SPBRG register value, produces baud rate of 9600bps
      
}

//Start a distance acquisition from ultrasonic sensor
void GetDistance(void) {
    
    LATEbits.LATE5 = 1;         //Send a trigger pulse to the ultrasound module (minimum 10uS)
    Delay1TCYx(25);
    LATEbits.LATE5 = 0;
    INTCONbits.RBIE = 1;        //Enable PORTB interrupts to detect the echo pulse
    echo_acq_done = 0;         //Clear flag to show that measurement is not complete

}

//Checks if a distance acquisition is ongoing
unsigned char BusyDistanceAcq(void) {
    
    return (echo_acq_done);        //Return the value of the echo_ready flag
    
}

//Reads raw timer value upon completion of distance acquisition, assumes distance acquisition is complete
unsigned int ReadEchoLength(void) {
    
    return(ReadTimer1());
    
}

//Converts echo length to distance in CM, assumes distance acquisition is complete
unsigned int ConvertDistanceCM(void) {
    
    return (echo_time * ECHO_TO_DIST_CM);
    
}

//Converts echo length to distance in INCHES, assumes distance acquisition is complete
unsigned int ConvertDistanceIN(void) {
    
    return (echo_time * ECHO_TO_DIST_IN);
    
}

//Gets an ADC reading from all sensors and stores them in an array
void GetSensorReadings(void) {
    
    sensor_acq_index = 0;               //Set sensor index to zero
    sensor_acq_done = 0;                //Clear done flag until conversion is complete
    SetADCChannel(sensor_acq_index);    //Set ADC channel to start acquisition from
    ConvertADC();                       //Start a conversion
    PIE1bits.ADIE = 1;                  //Enable ADC interrupts
    
}

//Checks if a sensor array acquisition is ongoing
unsigned char BusySensorAcq(void) {
    
    return(sensor_acq_done);
    
}

//Sets ADC channel based upon decimal value passed in
void SetADCChannel(unsigned char channel) {
    
    switch (channel) {              //Switch case based upon which channel is to be sampled
        case 0:
            SetChanADC(ADC_CH0);    //Set ADC channel
            break;
        case 1:
            SetChanADC(ADC_CH1);
            break;
        case 2:
            SetChanADC(ADC_CH2);
            break;
        case 3:
            SetChanADC(ADC_CH3);
            break;
        case 4:
            SetChanADC(ADC_CH4);
            break;
        case 5:
            SetChanADC(ADC_CH5);
            break;
        case 6:
            SetChanADC(ADC_CH6);
            break;
        case 7:
            SetChanADC(ADC_CH7);
            break;
        case 8:
            SetChanADC(ADC_CH8);
            break;
        case 9:
            SetChanADC(ADC_CH9);
            break;
        case 10:
            SetChanADC(ADC_CH10);
            break;
        case 11:
            SetChanADC(ADC_CH11);
            break;
        case 12:
            SetChanADC(ADC_CH12);
            break;
        case 13:
            SetChanADC(ADC_CH13);
            break;
        case 14:
            SetChanADC(ADC_CH14);
            break;
        case 15:
            SetChanADC(ADC_CH15);
            break;
        default:
            return 0;
    }
    
}

//Main Function
void main(void) {

    
    
    
    
}
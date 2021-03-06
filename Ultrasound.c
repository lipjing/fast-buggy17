#include "plib/timers.h"
#include "plib/delays.h"

float convert_cm, convert_in;
unsigned char echo_acq_done;
volatile unsigned int echo_time;

//Configure ultrasound distance conversion
void ConfigureUltrasound(float echo_to_cm, float echo_to_in) {
    convert_cm = echo_to_cm;
    convert_in = echo_to_in;
    WriteTimer1(0);

}

//ISR for ultrasound sensor
inline void UltrasoundISR(void) {
    
//    INTCONbits.RBIE = 0;        //If it has, temporarily disable PORTB interrupts
//    if (PORTBbits.RB4 == 1) {   //If RB4 edge is a rising edge (start of echo pulse)
//        T1CONbits.TMR1ON = 1;   //Enable Timer1 to start timing pulse length
//        WriteTimer1(0);         //Then set Timer1 to 0 to reset it
//    }
//    if (PORTBbits.RB4 == 0) {   //If RB4 edge is falling edge (end of echo pulse)
//        T1CONbits.TMR1ON = 0;   //Turn off Timer1 to stop timing pulse length
//        echo_time = ReadTimer1();
//        echo_acq_done = 1;      //Set echo flag to indicate that echo reading is ready for processing
//    }
//    INTCONbits.RBIE = 1;        //Re-enable PORTB interrupts
//    INTCONbits.RBIF = 0;        //Clear the interrupt flag
    
    INTCON3bits.INT3IE = 0;
    if(T1CONbits.TMR1ON == 0) {
        T1CONbits.TMR1ON = 1;
        WriteTimer1(0);
        INTCON2bits.INTEDG3 = 0;
    }
    else {
        T1CONbits.TMR1ON = 0;
        echo_time = ReadTimer1();
        echo_acq_done = 1;            
    }
    INTCON3bits.INT3IF = 0;
    INTCON3bits.INT3IE = 1;
}

//Start a distance acquisition from ultrasonic sensor
void GetDistance(void) {

    LATEbits.LATE5 = 1; //Send a trigger pulse to the ultrasound module (minimum 10uS)
    Delay1TCYx(25);
    LATEbits.LATE5 = 0;
    INTCON2bits.INTEDG3 = 1;
    INTCON3bits.INT3IE = 1;    
    //INTCONbits.RBIE = 1; //Enable PORTB interrupts to detect the echo pulse
    echo_acq_done = 0; //Clear flag to show that measurement is not complete

}

//Checks if a distance acquisition is ongoing
unsigned char BusyDistanceAcq(void) {

    return (!echo_acq_done); //Return the value of the echo_ready flag

}

//Reads raw timer value upon completion of distance acquisition, assumes distance acquisition is complete
unsigned int ReadEchoLength(void) {

    return (echo_time);

}

void ResetEchoLength(void) {
    
    echo_time = 0;
    echo_acq_done = 0;
    
}

//Converts echo length to distance in CM, assumes distance acquisition is complete
float ConvertDistanceCM(unsigned int echo_time) {

    return (echo_time * convert_cm);

}

//Converts echo length to distance in INCHES, assumes distance acquisition is complete
float ConvertDistanceIN(unsigned int echo_time) {

    return (echo_time * convert_in);

}
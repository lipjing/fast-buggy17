#include "xc.h"
#include "config_18f8722.h"
#include "plib/adc.h"
#include "plib/timers.h"
#include "plib/delays.h"

#define THRESHOLD 40

volatile unsigned int echo;
volatile unsigned char echo_ready;

float dist_cm;
unsigned char dist_cm_hundreds, dist_cm_tens, dist_cm_units, dist_cm_tenths;

void interrupt isr() {
    if(INTCONbits.RBIF == 1) {
        INTCONbits.RBIF = 0;
        if(PORTBbits.RB4 == 1) {
            WriteTimer0(0);
            T0CONbits.TMR0ON = 1;
            echo_ready = 0;
        }
        if(PORTBbits.RB4 == 0) {
            T0CONbits.TMR0ON = 0;
            echo = ReadTimer0();
            echo_ready = 1;
            INTCONbits.RBIE = 0;
        }  
    }    
}

void ConfigIO(void) {
    TRISE = 0xC0; 
    LATE = 0x1F;

    TRISA = 0xFF;
    
    TRISB = 0xFF;  

    TRISJ = 0x00;
    LATJ = 0x00;

}

void ConfigTimer0(void) {
    T0CON = 0x06;
    WriteTimer0(0);
}

void ConfigInterrupts(void) {
    RCONbits.IPEN = 0;
    INTCONbits.GIE = 1;
    INTCONbits.RBIE = 0;
    INTCONbits.RBIF = 0;
    
}

void GetDistance(void) {
    INTCONbits.RBIE = 1;
    LATEbits.LATE5 = 1;
    Delay1TCYx(37);
    LATEbits.LATE5 = 0;
}

void main(void) {
    
    ConfigIO();
    ConfigTimer0();
    ConfigInterrupts();
    
    while(1) {
        
        GetDistance();
        
        LATJbits.LATJ0 = 1;
        Delay1KTCYx(100);
        LATJbits.LATJ0 = 0;
        
        while(echo_ready != 1);
        
        dist_cm = 0.883 * echo;
        echo_ready = 0;
        
        Delay10KTCYx(100);  

    }  
    
}

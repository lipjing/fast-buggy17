#include "xc.h"
#include "config_18f8722.h"
#include "plib/adc.h"
#include "plib/timers.h"

#define THRESHOLD 40

volatile int echo;

unsigned char i;
unsigned int sensors[5], threshold = 0;

void interrupt isr() {
    if(INTCONbits.RBIF == 1) {
        INTCONbits.RBIF = 0;
        if(PORTBbits.RB4 == 1) {
            WriteTimer0(0);
            T0CONbits.TMR0ON = 1;
        }
        if(PORTBbits.RB4 == 0) {
            T0CONbits.TMR0ON = 0;
            echo = ReadTimer0();
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

void ConfigADC(void) {
    OpenADC(ADC_FOSC_16 
            & ADC_RIGHT_JUST 
            & ADC_12_TAD, 
            ADC_CH0 
            & ADC_INT_OFF 
            & ADC_VREFPLUS_VDD 
            & ADC_VREFMINUS_VSS, 
            10);
}

void ConfigTimer0(void) {
    T0CON = 0x00;
    WriteTimer0(0);
}

void ConfigInterrupts(void) {
    RCONbits.IPEN = 0;
    INTCONbits.GIE = 1;
    INTCONbits.RBIE = 0;
    INTCONbits.RBIF = 0;
    
}

unsigned int TakeReading(unsigned char channel) { //returns the converted voltage
    switch (channel) {
        case 0:
            SetChanADC(ADC_CH0);
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
        default :
            return 0;
    }
    
    ConvertADC();
    
    while (BusyADC());
    return ReadADC() & 0x03FF;
}

void SelectLED(unsigned char sel) { //returns the value of the LED to light
    switch (sel) {
        case 0:
            LATJ = 0x01;
            break;
        case 1:
            LATJ = 0x02;
            break;
        case 2:
            LATJ = 0x04;
            break;
        case 3:
            LATJ = 0x08;
            break;
        case 4:
            LATJ = 0x10;
            break;
    }
}

void main(void) {

    ConfigIO();
    ConfigADC();

    while (1) {

        LATJ = 0x00; //clears LEDs

        for (i = 0; i < 6; i++) {

            if (i < 5) {
                sensors[i] = TakeReading(i); //samples sensor
                
                if (sensors[i] > threshold) { //If converted sample is greater than 40, illuminate LED
                    SelectLED(i);
                }
                
            }
            
            else {
                threshold = TakeReading(5); 
            }

            Delay1KTCYx(10);

        }

    }
    
}
#include "xc.h"
#include "plib/adc.h"

unsigned char reading = 0, i;
unsigned char sensors[4];

void configure_IO_pins(void){
    ADCON1 = 0x0F;
    TRISE = 0xE0;
    LATE = 0x1F;
    TRISB = 0x07;
}
void configure_ADC (void){
    OpenADC(ADC_FOSC_16 & ADC_RIGHT_JUST & ADC_12_TAD, ADC_CH0 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, 10);  //allows channels a0-a5 for analog input
}

unsigned char take_reading(void){   //returns the converted voltage
    ConvertADC();
    while(BusyADC());
    return ReadADC();
}
unsigned char LED_select(unsigned char sel){                        //returns the value of the LED to light
    unsigned char LEDS[] = {0x01,0x02,0x04,0x08,0x10};
    LATB = LEDS[sel] << 3;
}
void main (void){
    
    configure_IO_pins();
    configure_ADC();
    
    
    
    while(1){
        
        LATB = 0x00;        //clears LEDs
    
        for(i=0;i<5;i++){
        
            SetChanADC(i);              //Selects which sensor to sample
       
            sensors[i] = take_reading();        //samples sensor
        
            if (sensors[i]>40){                 //If coverted sample is greater than 40, illuminate LED
            
                LED_select(i);
        
            }
        }
        
        
        
    }
}
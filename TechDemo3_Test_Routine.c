/*
 * Filename: TechDemo3_Test_Routine.c
 * Author:
 * Date: 10/03/2017
 * Description:  This program implements a series of tests to demonstrate the functionality required by tech demo 2.
 * >Detection and indication of white line using TCRT5000 line sensors, with adjustable threshold
 * >Measurement and display of distance from ultrasonic distance sensor
 * 
 * The program uses the UART on the PIC to output values to a serial terminal.
*/

//Pre-processor include directives
#include "config_18f8722.h"
#include "plib/adc.h"
#include "plib/timers.h"
#include "plib/delays.h"
#include "stdio.h"

//Define directives 
#define DIST_CM_MULTIPLIER 0.883    //Multiplier for Timer0 raw value to convert pulse length to CM
#define DELAY 110       //Delay between printing of sensor threshold value on serial terminal

//Volatile variable declarations, these are modified in the ISR
volatile unsigned int echo_time;    //Value of Timer0 register after ultrasonic echo pulse received
volatile unsigned char echo_ready;  //Flag to indicate that the echo pulse has been received and distance is ready to be calculated
volatile unsigned char mode = 0;    //Mode flag to switch between line sensing/ultrasonic distance modes

//Variable declarations
float dist_cm;      //Stores calculated distance in CM
unsigned char i, i2;    //Loop counters
unsigned int sensors[5], threshold = 0;     //Stores ADC values from all 5 sensors and ADC value of threshold potentiometer
char buffer[16];        //Array to store string to be written to serial port

//Interrupt Service Routine
void interrupt isr() {  
    if (INTCON3bits.INT1IF == 1) {      //Check to see if interrupt has come from INT1 (PB1)
        INTCON3bits.INT1IF = 0;         //If it has, clear interrupt flag
        mode ^= 0x01;                   //and change mode by toggling mode flag
    }

    if (INTCONbits.RBIF == 1) {         //Check to see if interrupt has come from PORTB (ultrasonic sensor return pulse)
        INTCONbits.RBIE = 0;            //If it has, temporarily disable PORTB interrups
        if (PORTBbits.RB4 == 1) {       //If RB4 edge is a rising edge (start of echo pulse)
            T0CONbits.TMR0ON = 1;       //Enable Timer0 to start timing pulse length
            WriteTimer0(0);             //Then set Timer0 to 0 to reset it
        }
        if (PORTBbits.RB4 == 0) {       //If RB4 edge is falling edge (end of echo pulse)
            T0CONbits.TMR0ON = 0;       //Turn off Timer0 to stop timing pulse length
            echo_time = ReadTimer0();   //Load Timer0 value into echo_time
            echo_ready = 1;             //Set echo flag to indicate that echo reading is ready for processing
        }
        INTCONbits.RBIE = 1;            //Re-enable PORTB interrupts
        INTCONbits.RBIF = 0;            //Clear the interrupt flag
    }
}

//Configure IO ports - this is done as per the pin mapping, pretty self explanatory
void ConfigIO(void) {
    TRISE = 0xC0;
    LATE = 0x00;

    TRISA = 0xFF;

    TRISB = 0xFF;
    
    TRISC = 0x10;
    LATC = 0x20; 

    TRISJ = 0x00;
    LATJ = 0x00;

}

//Configure the ADC to take readings from the sensors and threshold potentiometer
void ConfigADC(void) {
    OpenADC(ADC_FOSC_16             //ADC clock of Fosc/16
            & ADC_RIGHT_JUST        //Results right-justified in results registers
            & ADC_12_TAD,           //Acquisition delay as we are changing the ADC channel; to give time for the hold capacitor to charge/discharge
            ADC_CH0                 //Start off with channel 0 selected
            & ADC_INT_OFF           //No ADC interrupts
            & ADC_VREFPLUS_VDD      //Vref comes from PIC power rails (+5V/0V)
            & ADC_VREFMINUS_VSS,
            10);                    //Enables AN0-AN5 as analogue inputs, rest are standard port pins (digital)
}

//Configure Timer0 to measure pulse lengths
void ConfigTimer0(void) {
    T0CON = 0x00;       //Disable Timer0 for now, pre-scaler assigned, pre-scaler = 1:2
    WriteTimer0(0);     //Clear Timer0
}

//Configure interrupt sources
void ConfigInterrupts(void) {
    RCONbits.IPEN = 0;          //Prioritised mode disabled
    INTCONbits.GIE = 1;         //Enable global interrupts
    INTCONbits.RBIE = 0;        //Disable PORTB interrupt for now
    INTCON3bits.INT1IE = 1;     //Enable INT1 (PB1) interrupt
    INTCON2bits.INTEDG1 = 1;    //Set INT1 (PB1) to occur on rising edge
    INTCON3bits.INT1IF = 0;     //Clear interrupt flags
    INTCONbits.RBIF = 0;

}

//Configure USART
void ConfigUSART(void) {
    Open1USART(USART_TX_INT_OFF     //Disable Tx and Rx interrupts
            & USART_RX_INT_OFF
            & USART_ASYNCH_MODE     //Asynchronous mode (no clock)
            & USART_EIGHT_BIT       //8-bit mode
            & USART_CONT_RX         //Continuous Rx
            & USART_BRGH_HIGH,      //BRG in high-speed mode
            64);                    //SPBRG register value, produces baud rate of 9600bps

    putrs1USART("USART Configured\r\n");        //Send a string to show that USART is up and running

}

//Function to get an ADC reading from a particular channel
unsigned int GetADCReading(unsigned char channel) { 
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
        default:
            return 0;
    }

    ConvertADC();       //Start an A-D conversion
    
    while (BusyADC());  //Wait until the conversion has finished
    return ReadADC() & 0x03FF;  //Return the ADC value AND'ed to mask off the most significant 6 bits
}

//Illuminate a specific LED as determined by index passed into function
void IlluminateLED(unsigned char sel) { 
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

//Initiate a distance measurement
void GetDistance(void) {
    LATEbits.LATE5 = 1;     //Send a trigger pulse to the ultrasound module (minimum 10uS)
    Delay1TCYx(25);
    LATEbits.LATE5 = 0;
    INTCONbits.RBIE = 1;    //Enable PORTB interrupts to detect the echo pulse
    echo_ready = 0;         //Clear flag to show that measurement is not complete
}

//Flash routine for LEDs to indicate changing of mode
void ChangeMode(void) {
    LATJ = 0x00;
    LATJ = 0x1F;
    Delay10KTCYx(125);
    LATJ = 0x00;
    Delay10KTCYx(125);
    LATJ = 0x1F;
    Delay10KTCYx(125);
    LATJ = 0x00;
}

//Main function
void main(void) {

    ConfigIO();
    ConfigADC();
    ConfigTimer0();
    ConfigUSART();
    ConfigInterrupts();
    
    putrs1USART("ESP Group 16\r\nTech Demo 3 Test Program\r\n");    //Send string over serial to show program is starting

    while (1) {     //Sit in this loop forever

        putrs1USART("Mode: LINE SENSING\r\n");      //Send string over serial to indicate what mode the program is in    
        putrs1USART("THRESHOLD\r\n");
        i2 = 0;

        while (mode == 0) {     //Line sensing mode

            LATJ = 0x00;        //Clears stripboard LEDs
            LATE = 0xFF;        //Enables all TCRT5000 emitter LEDs

            for (i = 0; i < 6; i++) {       //Loop to cycle through the sensors and threshold potentiometer and get ADC values from each
                if (i < 5) {                            //If sampling sensors, store result in sensors array
                    sensors[i] = GetADCReading(i);      //Samples sensor indicated by index 
                    if (sensors[i] > threshold) {       //If converted sample is greater than threshold, illuminate LED
                        IlluminateLED(i);
                    }
                } else {                                //If sampling threshold potentiometer, store result in different variable
                    threshold = GetADCReading(5);
                }
                Delay1KTCYx(10);                        //Delay between iterations of loop
                if(i2 >= DELAY) {                       //If statement to slow down sending of threshold value over serial
                    i2 = 0;
                    sprintf(buffer, "%d\r\n", threshold);   //Format threshold value into a string, store it in the buffer, ready for sending
                    puts1USART(buffer);                     //Send buffer contents over serial port
                }
                else {
                    i2++;
                }
            }
        }

        ChangeMode();       //Flash LEDs to indicate changing of mode
        putrs1USART("Mode: ULTRASONIC DISTANCE\r\n");   //Send string over serial to indicate what mode the program is in    
        putrs1USART("DISTANCE (CM)\tTMR0 VALUE\r\n");

        while (mode == 1) {     //Ultrasonic distance sensing mode
            GetDistance();      //Initiate distance measurement

            LATJbits.LATJ0 = 1;     //Pulse LED0 on stripboard to indicate distance conversion has been initiated
            Delay1KTCYx(100);
            LATJbits.LATJ0 = 0;

            while (!(echo_ready == 1));     //Wait until conversion is complete

            dist_cm = 0.0137 * echo_time;   //Convert raw echo time stored in Timer0 register to distance in CM

            sprintf(buffer, "%.2f cm\t%d\r\n", dist_cm, echo_time);     //Format distance in CM and Timer0 value into a string, store it in buffer, ready for sending
            puts1USART(buffer);             //Send buffer contents over serial port (print distance and Timer0 value)

            Delay10KTCYx(200);              //Delay of approx. 400ms until next distance measurement

        }

        ChangeMode();       //Flash LEDs to indicate changing of mode

    }

}

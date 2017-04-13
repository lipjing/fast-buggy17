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
#include "plib/pwm.h"
#include "stdio.h"
#include "math.h"
#include "Ultrasound.h"
#include "SystemClock.h"
#include "Motors.h"
#include "MillisecondTimer.h"
#include "EEPROM.h"
#include "Wireless.h"
#include "global_defines.h"     //Some global pre-processor definitions for things like motor drive board pin connections etc.

#define PERIOD_REG      130      //Value written to PR2 register to set PWM frequency (approx. 19kHz)
#define ECHO_TO_DIST_CM 0.0137   //Multiplier for echo pulse length to convert to distance in CM
#define ECHO_TO_DIST_IN 0.054    //Multiplier for echo pulse length to convert to distance in INCHES
#define THRESHOLD_MARGIN 10      //Margin subtracted from sensor threshold
#define BLACK_ON_WHITE  0       //Line mode black on white
#define WHITE_ON_BLACK  1       //Line mode white on black
#define DEBOUNCE_DELAY  10      //Debounce delay in milliseconds
#define LED_FLASH_DELAY 200     //LED flash delay for waiting for user input

//PID defines
#define PID_KP  13  //Proportional constant
#define PID_KD  1   //Derivative constant
#define PID_KI  1   //Integral constant

//Sensor weighting defines - expand the code fold below for a description
/* Sensor weightings - adjusting these WILL have an effect on the control algorithm
 * Sensors are weighted with values as shown in the diagram below:
 *      ---------------------------------
 *     |   [0]   [1]   [2]   [3]   [4]   |
 *      ---------------------------------
 *          |_____|_____|_____|_____|
 *          |  |  |  |  |  |  |  |  |
 *          |  W1 |  W3 |  W5 |  W7 |
 *          W0    W2    W4   W6     W8
 * Upper row indicates weighting when line is under two sensors
 * Lower row indicates weighting when line is under a single sensor
 * 
 * Weightings influence the contribution that each sensor will have to the overall sensor error
 * therefore they must be tuned WITH the PID variables
 */
#define W0  -64
#define W1  -27
#define W2  -8
#define W3  -1
#define W4  0
#define W5  1
#define W6  8
#define W7  27
#define W8  64

//Global volatile variables
volatile unsigned char sensor_acq_done, sensor_acq_index;
volatile int sensor_readings_raw[NO_OF_SENSORS];

//Global variables
unsigned char sensor_status[NO_OF_SENSORS], line_mode;
int sensor_offsets[NO_OF_SENSORS], sensor_readings_normalised[NO_OF_SENSORS];
int sensor_threshold, PID_error, PID_output;

//Global pointer variables
int *sensor_threshold_ptr;

//Configure interrupt sources
void ConfigureInterrupts(void) {

    INTCONbits.GIE = 1; //Enable global interrupts
    INTCONbits.PEIE = 1; //Enable peripheral interrupts
    RCONbits.IPEN = 1; //Prioritised mode enabled

//    //INT1 (PB1) Interrupt
//    INTCON3bits.INT1E = 1; //Enable INT1 (PB1) interrupt
//    INTCON3bits.INT1IF = 0; //Clear flag
//    INTCON2bits.INTEDG1 = 1; //Rising-edge triggered
//    INTCON3bits.INT1IP = 0; //Low priority when priority mode enabled


//    //INT2 (PB2) Interrupt
//    INTCON3bits.INT2E = 1; //Enable INT2 (PB2) interrupt
//    INTCON3bits.INT2IF = 0; //Clear flag
//    INTCON2bits.INTEDG2 = 1; //Rising-edge triggered
//    INTCON3bits.INT2IP = 0; //Low priority when prioritised mode enabled


    //PORTB Interrupt
//    INTCONbits.RBIE = 0; //Disable PORTB interrupt for now
//    INTCONbits.RBIF = 0; //Clear flag
//    INTCON2bits.RBIP = 1; //High priority when prioritised mode enabled
    
    //INT3 (Ultrasound) Interrupt
    INTCON3bits.INT3IE = 0;     //Disable INT3 interrupt for now
    INTCON3bits.INT3IF = 0;     //Clear flag
    INTCON2bits.INTEDG3 = 1;    //Rising-edge triggered
    INTCON2bits.INT3IP = 1;     //High priority when prioritised mode enabled

    //Timer0 Interrupt
    INTCONbits.TMR0IE = 1; //Enable Timer0 interrupt
    INTCONbits.TMR0IF = 0; //Clear flag
    INTCON2bits.TMR0IP = 1; //High priority when prioritised mode enabled

//    //Timer1 Interrupt
//    PIE1bits.TMR1IE = 0; //Disable Timer1 interrupt

//    //Timer2 Interrupt
//    PIE1bits.TMR2IE = 0; //Disable Timer2 interrupt

    //ADC Interrupt
    PIE1bits.ADIE = 0; //Disable ADC interrupt for now
    PIR1bits.ADIF = 0; //Clear flag
    IPR1bits.ADIP = 0; //Low priority when prioritised mode enabled

    //USART1 Interrupt
    PIE1bits.RC1IE = 0; //Disable USART1 Rx interrupt for now
    PIE1bits.TX1IE = 0; //Disable USART1 Tx interrupt for now
    PIR1bits.RC1IF = 0; //Clear Rx flag
    PIR1bits.TX1IF = 0; //Clear Tx flag
    IPR1bits.RC1IP = 0; //Low priority (Rx) when prioritised mode enabled
    IPR1bits.TX1IP = 0; //Low priority (Tx) when prioritised mode enabled
}

//Configure Timer0
void ConfigureTimer0(void) {

    OpenTimer0(TIMER_INT_ON //Interrupts enabled
            & T0_16BIT //16-bit mode
            & T0_SOURCE_INT //Increment on internal instruction clock
            & T0_PS_1_1); //Prescaler 1:1

    //T0CON = 0xFF;             //Achieves same as OpenTimer0 function, uncomment if OpenTimerX doesn't work on some versions of XC8

    WriteTimer0(TIMER0_VALUE);

}

//Configure Timer1
void ConfigureTimer1(void) {

    OpenTimer1(TIMER_INT_OFF //Interrupts disabled
            & T1_16BIT_RW //16-bit mode
            & T1_SOURCE_INT //Increment on internal instruction clock
            & T1_PS_1_2 //Prescaler 1:2
            & T1_OSC1EN_OFF //Disable oscillator attached to OSC1 pins
            & T1_SYNC_EXT_OFF); //External clock synchronisation off

    //T1CON = 0x9D;              //Achieves same as OpenTimer1 function, uncomment if OpenTimerX doesn't work on some versions of XC8

    T1CONbits.TMR1ON = 0; //Turn off Timer1 for now, will be enabled when ultrasonic distance is requested

    WriteTimer1(0);

}

//Configure Timer2
void ConfigureTimer2(void) {

    OpenTimer2(TIMER_INT_OFF //Interrupts disabled
            & T2_PS_1_1 //Prescaler 1:1
            & T2_POST_1_1 //Postscaler 1:1
            & T12_SOURCE_CCP);
    
//    PIE1bits.TMR2IE = 0;        //Disable Timer2 interrupts
//    T2CONbits.T2CKPS1 = 0;      //Set Timer2 pre-scaler to 1:1
//    T2CONbits.T2CKPS0 = 0;
//    T2CONbits.TMR2ON = 1;       //Turn on Timer2
//    PR2 = PERIOD_REG;           //Write PWM Period value to period register

    //T2CON = 0x04;               //Achieves same as OpenTimer2 function, uncomment if OpenTimerX doesn't work on some versions of XC8

}

//Configure PWM
void ConfigurePWM(void) {

    OpenPWM4(PERIOD_REG); //Set up CCP4 in PWM mode
    OpenPWM5(PERIOD_REG); //Set up CCP5 in PWM mode

//        //Uncomment lines below if OpenPWMX functions are not working on the version of XC8 used
//        //Configure CCP4 module for PWM operation
//        CCP4CONbits.CCP4M3 = 1;     
//        CCP4CONbits.CCP4M2 = 1;
//        CCP4CONbits.CCP4M1 = 0;
//        CCP4CONbits.CCP4M0 = 0;
//    
//        //Configure CCP5 module for PWM operation
//        CCP5CONbits.CCP5M3 = 1;     
//        CCP5CONbits.CCP5M2 = 1;
//        CCP5CONbits.CCP5M1 = 0;
//        CCP5CONbits.CCP5M0 = 0;
//        
//        PR2 = PERIOD_REG;

}

//Configures IO for buggy
void ConfigureBuggyIO(void) {

    //TRISA
    TRISA = 0xFF;

    //TRISB
    TRISB = 0xFF;

    //TRISC
    TRISC = 0x9F;

    //TRISD
    TRISD = 0xE0;

    //TRISE
    TRISE = 0x00;

    //TRISG
    TRISG = 0xE7;

    //TRISJ
    TRISJ = 0x00;

}

//Configures ADC for sensor readings
void ConfigureADC(void) {

    OpenADC(ADC_FOSC_8 //ADC clock of FOSC/8
            & ADC_RIGHT_JUST //Results right-justified in results registers
            & ADC_4_TAD, //Acquisition delay as we are changing the ADC channel; to give time for the hold capacitor to charge/discharge
            ADC_CH0 //Start off with Channel 0 selected
            & ADC_INT_OFF //Disable interrupts for now
            & ADC_VREFPLUS_VDD //Vref comes from PIC power rails (+5V/0V)
            & ADC_VREFMINUS_VSS,
            9); //Enables AN0-AN4 as analogue inputs, rest are standard port pins (digital)

}

//Configure USART1 for serial port
void ConfigureSerial(void) {

    Open1USART(USART_TX_INT_OFF //Disable Tx and Rx interrupts
            & USART_RX_INT_OFF
            & USART_ASYNCH_MODE //Asynchronous mode (no clock)
            & USART_EIGHT_BIT //8-bit mode
            & USART_CONT_RX //Continuous Rx
            & USART_BRGH_HIGH, //BRG in high-speed mode
            64); //SPBRG register value, produces baud rate of 9600bps

}

//Sets ADC channel based upon decimal value passed in
void SetADCChannel(unsigned char channel) {

    switch (channel) { //Switch case based upon which channel is to be sampled
        case 0:
            SetChanADC(ADC_CH0); //Set ADC channel
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
            break;
    }

}

//Gets an ADC reading from all sensors and stores them in an array
void GetSensorReadings(void) {

    sensor_acq_index = 0; //Set sensor index to zero
    sensor_acq_done = 0; //Clear done flag until conversion is complete
    SetADCChannel(sensor_acq_index); //Set ADC channel to start acquisition from
    ConvertADC(); //Start a conversion
    PIE1bits.ADIE = 1; //Enable ADC interrupts

}

//Checks if a sensor array acquisition is ongoing
unsigned char BusySensorAcq(void) {

    return (!sensor_acq_done);

}

//Normalises sensor readings using offsets stored in array
void NormaliseSensorReadings(void) {
    unsigned char index;
    
    for(index = 0; index < NO_OF_SENSORS; index++) {
        
        sensor_readings_normalised[index] = sensor_readings_raw[index] + sensor_offsets[index];
        
    }    
    
}

//Calculates if a sensor is above threshold and therefore above line
void CalculateSensorStatuses(void) {
    unsigned char index;
    
    //WHITE LINE ON BLACK DETECTION
    if(line_mode == WHITE_ON_BLACK) {
        for(index = 0; index < NO_OF_SENSORS; index++) {

            if(sensor_readings_normalised[index] > sensor_threshold) {
                sensor_status[index] = 1;
            }
            else {
                sensor_status[index] = 0;
            }
        }
    }
    //BLACK LINE ON WHITE DETECTION
    else if(line_mode == BLACK_ON_WHITE) {

        for(index = 0; index < NO_OF_SENSORS; index++) {

            if(sensor_readings_normalised[index] < sensor_threshold) {
                sensor_status[index] = 1;
            }
            else {
                sensor_status[index] = 0;
            }

        }
    }

     
}

//Calculate sensor sums
unsigned char CalculateSensorSums(void) {
    unsigned char index, sum = 0;
    
    for(index = 0; index < NO_OF_SENSORS; index++) {
        
        sum += (unsigned char) (sensor_status[index] * pow(2, index)); 
        
    }
    
    return(sum);
    
}

//Calculate discrete sensor error
int CalculateSensorError(const unsigned char *sum) {
    
    switch(*sum) {
        case(16)    :
            return(W8);
        case(24) :
            return(W7);
        case(8) :
            return(W6);
        case(12) :
            return(W5);
        case(4) : 
            return(W4);
        case(6) :
            return(W3);
        case(2) :
            return(W2);
        case(3) :
            return(W1);
        case(1) :
            return(W0);
        default :
            break;
    }
    
    return(0);
    
}

//Displays statuses of sensors on LEDs
void DisplaySensorStatuses(const unsigned char *status_array) {
    unsigned char index;
    
    for (index = 0; index < NO_OF_SENSORS; index++) {
            LATJ ^= *(status_array + index) << index;
        }
    
}

//Turn on all TCRT5000 IR LEDs
void EnableSensorLEDS(void) {
    
    LATE = 0x1F;
    
}

//Turn off all TCRT5000 IR LEDs
void DisableSensorLEDS(void) {
    
    LATE &= 0xE0;       //AND used to avoid disturbing other IO connected to PORTE
    
}

//Flashes LEDs to indicate changing of mode
void FlashLEDS(void) {
    LATJ = 0x00;
    LATJ = 0x1F;
    Delay10KTCYx(75);
    LATJ = 0x00;
    Delay10KTCYx(75);
    LATJ = 0x1F;
    Delay10KTCYx(75);
    LATJ = 0x00;
}

//Returns the value of PB1 with a debounce delay
unsigned char PB1pressed(void) {
    if(PORTBbits.RB1 == 0) {
        return(0);
    }
    else if(PORTBbits.RB1 == 1) {
        ResetMillis2();
        while(ReadMillis2() < DEBOUNCE_DELAY);
        if(PORTBbits.RB1 == 1) {
            return(1);
        }
        else {
            return(0);
        }
    }
    
    return(0);
    
}

//Returns the value of PB2 with a debounce delay
unsigned char PB2pressed(void) {
    if(PORTBbits.RB2 == 0) {
        return(0);
    }
    else if(PORTBbits.RB2 == 1) {
        ResetMillis2();
        while(ReadMillis2() < DEBOUNCE_DELAY);
        if(PORTBbits.RB2 == 1) {
            return(1);
        }
        else {
            return(0);
        }
    }
    
    return(0);
}

//Generates offsets for each sensor
void CalibrateOffsets(void) {
    unsigned char index;
    
    GetSensorReadings();
    
    while(BusySensorAcq());
    
    for(index = 0; index < NO_OF_SENSORS; index++) {
        
        sensor_offsets[index] = sensor_readings_raw[2] - sensor_readings_raw[index];
        
    }
    
}

//Calibrates sensor threshold value
void CalibrateThreshold(void) {
    unsigned char index, sum;
    
    LATJ = 0x00;
    
    //Get threshold potentiometer reading
    SetADCChannel(5);
    ConvertADC();
    while(BusyADC());
    sensor_threshold = ReadADC();

    //Get sensor readings
    GetSensorReadings();

    while(BusySensorAcq());

    NormaliseSensorReadings();
    
    CalculateSensorStatuses();
    
    DisplaySensorStatuses(sensor_status);
    
    //Possible automatic calibration routine below
//    sum = CalculateSensorSums();
//    
//    while(sum != 4) {
//        sensor_threshold--;
//    
//        CalculateSensorStatuses();
//        
//        sum = CalculateSensorSums();
//        
//        DisplaySensorStatuses(sensor_status);
//        
//    }
//    
//    if(line_mode == WHITE_ON_BLACK) {
//        sensor_threshold =- THRESHOLD_MARGIN;
//    }
//    else if(line_mode == BLACK_ON_WHITE) {
//        sensor_threshold += THRESHOLD_MARGIN;
//    }

}

//Generates all calibration constants
void GenerateCalibration(void) {
    unsigned char index;
    
    //GENERATE SENSOR OFFSETS
    SendStatus(STATUS_3);
    LATJ = 0x03;
    
    while(PB1pressed() == 0) {
        
        CalibrateOffsets();
        Delay10KTCYx(10);
        
    }
    
    for(index = 0; index < NO_OF_SENSORS; index++) {
        WriteIntEEPROM(0x03 + (2 * index), sensor_offsets[index]);
    }
    
    SendOffsets(sensor_offsets);
    
    FlashLEDS();
    
    //SET SENSOR THRESHOLD
    SendStatus(STATUS_4);
    
    while(PB1pressed() == 0) {
        
        CalibrateThreshold();
        Delay10KTCYx(10);
        
    }
    
    WriteIntEEPROM(0x01, sensor_threshold);
    WriteCharEEPROM(0x00, 1);
    
    SendThreshold(sensor_threshold_ptr);
    
}

//Loads all calibration constants from EEPROM
void LoadCalibration(void) {
    unsigned char index;
    
    if(ReadCharEEPROM(0x00) == 0) {
        FlashLEDS();
        GenerateCalibration();        
    }
    
    else {
        sensor_threshold = ReadIntEEPROM(0x01);
        for(index = 0; index < NO_OF_SENSORS; index++) {
            sensor_offsets[index] = ReadIntEEPROM(0x03 + (2 * index));
        }
    }
    
}

//High-priority ISR
void interrupt high_priority isrHP(void) {

    //Timer0 ISR
    if (INTCONbits.TMR0IF == 1) {
        INTCONbits.TMR0IF = 0;      //Clear interrupt flag                
        
        MillisecondTimerISR();      //ISR for millisecond timer functionality
        
        //SystemClockISR();           //Uncomment this line if system clock functionality is needed - there will be a slight performance slowdown          
        
    }

    //PORTB ISR (for ultrasound)
//    else if (INTCONbits.RBIF == 1) {                 //Check to see if interrupt has come from PORTB (ultrasonic sensor return pulse)
//        UltrasoundISR();
//    }
    
    //INT3 ISR (for ultrasound)
    else if (INTCON3bits.INT3IF == 1) {
        INTCON3bits.INT3IF = 0;
        
        UltrasoundISR();
    }

}

//Low-priority ISR
void interrupt low_priority isrLP(void) {

//    //INT1 (PB1) ISR
//    if (INTCON3bits.INT1IF == 1) {
//
//        INTCON3bits.INT1IF = 0;
//    }
//
//    //INT2 (PB2) ISR
//    if (INTCON3bits.INT2IF == 1) {
//
//        INTCON3bits.INT2IF = 0;
//    }

    //ADC ISR
    if (PIR1bits.ADIF == 1) {
        PIR1bits.ADIF = 0;
        if (sensor_acq_index < NO_OF_SENSORS) {
            *(sensor_readings_raw + sensor_acq_index) = ReadADC();
            sensor_acq_index++;
            SetADCChannel(sensor_acq_index);
            ConvertADC();
        } else {
            sensor_acq_done = 1;
            PIE1bits.ADIE = 0;
        }
    }
    
    else if(PIR1bits.TX1IF == 1) {
        PIR1bits.TX1IF = 0;
        WirelessTx_ISR();        
    }

}

//Main Function
void main(void) {

    //Variable declarations
    unsigned char sensor_sum, loop_count = 0;
    unsigned int echo_length;
    
    //Pointer declarations
    unsigned char *sensor_sum_ptr, *line_mode_ptr;
    sensor_sum_ptr = &sensor_sum;
    line_mode_ptr = &line_mode;
    
    sensor_threshold_ptr = &sensor_threshold;    
    
    ConfigureUltrasound(ECHO_TO_DIST_CM, ECHO_TO_DIST_IN);
    ConfigureBuggyIO();
    ConfigureInterrupts();
    ConfigureTimer0();
    ConfigureTimer1();
    ConfigureTimer2();
    ConfigurePWM();
    ConfigureADC();
    ConfigureWireless();

    DisableMotors();
    
    Delay10KTCYx(100);

    SendStatus(STATUS_0);
    
    //STARTUP ROUTINE
    
    //LINE MODE SETTING
    LATJ = 0x0F;
    SendStatus(STATUS_1);
    
    while(1) {
        if(PB1pressed() == 1) {
            line_mode = BLACK_ON_WHITE;
            break;
        }
        else if(PB2pressed() == 1) {
            line_mode = WHITE_ON_BLACK;
            break;
        }
    }
    
    SendLineMode(line_mode_ptr);    
    FlashLEDS();
    
    EnableSensorLEDS();
   
    //GENERATE / LOAD CALIBRATION
    LATJ = 0x07;
    SendStatus(STATUS_2);
    
    while(1) {
        if(PB1pressed() == 1) {
            FlashLEDS();
            GenerateCalibration();
            break;
        }
        else if(PB2pressed() == 1) {
            FlashLEDS();
            LoadCalibration();
            SendOffsets(sensor_offsets);
            SendThreshold(sensor_threshold_ptr);
            break;
        }

    }
    
    FlashLEDS();
    
    //READY TO RACE
    LATJ = 0x01;
    SendStatus(STATUS_6);
    
    while(PB1pressed() == 0);    
    
    //RACE INITIALISATION
    PID_error = 0;
    PID_output = 0;
    
    LATJ = 0x00;
    
    ResetMillis0();
    
    FlashLEDS();
    SendStatus(STATUS_7);
    
    SetUnipolar();
    StopMotors();
    SetDirectionForward();
    EnableMotors();
    
    //RACE ROUTINE
    while(1) {
        
        //GET SENSOR READINGS AND STATUSES
        GetSensorReadings();
        
        while(BusySensorAcq());
        
        NormaliseSensorReadings();
        
        CalculateSensorStatuses();
        sensor_sum = CalculateSensorSums();
                
        //PID LOOP
        PID_error = CalculateSensorError(sensor_sum_ptr);
        PID_output = PID_KP * PID_error;
        
        SetDCMotorPID(PID_output);
                
        //DISPLAY SENSOR STATUSES
        DisplaySensorStatuses(sensor_status);
        
        //END OF LINE DETECTION
//        if(sensor_sum == 0) {
//            ResetMillis0();
//            while(ReadMillis0() < 10);
//            
//            GetSensorReadings();
//
//            while(BusySensorAcq());
//
//            NormaliseSensorReadings();
//
//            CalculateSensorStatuses();
//            sensor_sum = CalculateSensorSums();
//            
//            if(sensor_sum == 0) {
//                StopMotors();
//                DisableMotors();
//                while(1);                
//            }
//        }
        
        
        //ULTRASOUND DISTANCE
        if(BusyDistanceAcq() == 0) {
            echo_length = ReadEchoLength();
            if(echo_length < 1232) {
                                
                DisableMotors();
                
                SetForwardsMotorR();
                SetReverseMotorL();                
                
                SetDCMotorL(DC_MAX_SPEED_REV_L);
                SetDCMotorR(DC_MAX_SPEED_REV_R);
                
                EnableMotors();
                
                ResetMillis0();
                while(ReadMillis0() < 30);
                sensor_sum = 0;
               
                while(sensor_sum == 0) {
                    GetSensorReadings();
                    while(BusySensorAcq());
                    NormaliseSensorReadings();
                    CalculateSensorStatuses();
                    sensor_sum = CalculateSensorSums();
                    DisplaySensorStatuses(sensor_status);
                    
                }
                               
                DisableMotors();
                SetDirectionForward();
                EnableMotors();
                
                //PID_error = 0;
                
                GetDistance();
            }
            else {
                GetDistance();
            }   
        }

        
        //ITERATE LOOP EVERY X MILLISECONDS
        ResetMillis0();
        while(ReadMillis0() < 5);
        LATJ = 0x00;
        loop_count++;
       
    }

}
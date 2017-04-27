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
#include "BatteryLib/ds2781.h"
#include "global_defines.h"     //Some global pre-processor definitions for things like motor drive board pin connections etc.

#define PERIOD_REG      130      //Value written to PR2 register to set PWM frequency (approx. 19kHz)
#define ECHO_TO_DIST_CM 0.0137   //Multiplier for echo pulse length to convert to distance in CM
#define ECHO_TO_DIST_IN 0.054    //Multiplier for echo pulse length to convert to distance in INCHES
#define BLACK_ON_WHITE  0       //Line mode black on white
#define WHITE_ON_BLACK  1       //Line mode white on black
#define DEBOUNCE_DELAY  10      //Debounce delay in milliseconds
#define LED_FLASH_DELAY 200     //LED flash delay for waiting for user input
#define ULTRASOUND_POLLING_DELAY 30 //Rate at which ultrasound sensor is polled in milliseconds
#define END_OF_LINE_DELAY 180   //Delay in ms until a sensor reading of 0 is classed as the end of the line
#define STOP_DELAY_1 300      //Delay in ms to allow buggy to stop before turning around
#define STOP_DELAY_2 200      //Delay in ms to allow buggy to stop before carrying on after turn around
#define TURN_AROUND_DELAY 900  //Delay before buggy tries to detect line again after starting turn around routine
#define CONTROL_LOOP_DELAY 0   //Delay between control loop iterations (dt in integral/differential terms)
#define BATTERY_STATS_DELAY 800 //Rate at which battery stats are send in milliseconds
#define ECHO_LENGTH_FOR_WALL 1670   //1232 //Raw timer register value for ultrasound echo length for distance to wall

//PID defines - stored in program memory
#define DEFAULT_PID_KP  20  //Proportional
#define DEFAULT_PID_KD  0   //Derivative
#define DEFAULT_PID_KI  0   //Integral

//Global volatile variables - accessed in ISRs
volatile unsigned char sensor_acq_done, sensor_acq_index;
volatile int sensor_readings_raw[NO_OF_SENSORS];

//Global variables
unsigned char line_mode, sensor_statuses[NO_OF_SENSORS];
int sensor_offsets[NO_OF_SENSORS], sensor_readings_normalised[NO_OF_SENSORS], sensor_threshold; 
int PID_set_point, PID_output, PID_error, PID_Kp, PID_Kd, PID_Ki;

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

//Initiates an ADC reading from all sensors and stores them in an array
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

//Normalises raw sensor readings using stored offsets
void NormaliseSensorReadings(volatile const int *sensor_readings_raw, const int *sensor_offsets) {
    unsigned char index;
    
    for(index = 0; index < NO_OF_SENSORS; index++) {
        sensor_readings_normalised[index] = *(sensor_readings_raw + index) + *(sensor_offsets + index);        
    }
    
}

//Populate sensor status array with TRUE/FALSE based upon whether a sensor is over the line
void CalculateSensorStatuses(const int *sensor_readings_normalised, const int *sensor_threshold) {
    unsigned char index;
    
    if(line_mode == BLACK_ON_WHITE) {
        for(index = 0; index < NO_OF_SENSORS; index++) {
            if(*(sensor_readings_normalised + index) < *sensor_threshold) {
                sensor_statuses[index] = 1;
            }
            else {
                sensor_statuses[index] = 0;
            } 
        }
    }
    else if(line_mode == WHITE_ON_BLACK) {
        for(index = 0; index < NO_OF_SENSORS; index++) {
            if(*(sensor_readings_normalised + index) > *sensor_threshold) {
                sensor_statuses[index] = 1;
            }
            else {
                sensor_statuses[index] = 0;
            } 
        }
    }
    
}

//Calculate weighted sum of all sensor readings
int CalculateSensorSum(const int *sensor_readings_normalised) {

    int sum = 0;
    
    sum += *(sensor_readings_normalised + 0) * 3;
    sum += *(sensor_readings_normalised + 1) * 2;
    sum += *(sensor_readings_normalised + 2) * 1;
    sum += *(sensor_readings_normalised + 3) * -2;
    sum += *(sensor_readings_normalised + 4) * -3;
    
    return(sum);
    
}

//Calculate discrete sum of all sensor values; equals zero when no sensors are on line
unsigned char CalculateSensorSumDiscrete(const unsigned char *sensor_statuses) {
    unsigned char index, sum = 0;
    
    for(index = 0; index < NO_OF_SENSORS; index++) {
        
        sum += *(sensor_statuses + index); 
        
    }
    
    return(sum);
    
}

//Displays statuses of sensors on LEDs
void DisplaySensorStatuses(const unsigned char *sensor_statuses) {
    unsigned char index;
    
    LATJ = 0x00;
    
    for(index = 0; index < NO_OF_SENSORS; index++) {
        LATJ ^= *(sensor_statuses + index) << index;        
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
    if(CommandAvailable() == 1) {
        if(GetCommand() == RX_MSG_PB1) {
            return(1);
        }
        else {
            return(0);
        }
    }
    else if(PORTBbits.RB1 == 0) {
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
    if(CommandAvailable() == 1) {
        if(GetCommand() == RX_MSG_PB2) {
            return(1);
        }
        else {
            return(0);
        }
    }
    else if(PORTBbits.RB2 == 0) {
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

//Generate offsets for each sensor to eliminate effect of mis-matches
void CalibrateOffsets(void) {
    unsigned char index;
    
    SendStatus(STATUS_3);
    LATJ = 0x07;
    
    while(PB1pressed() == 0);
    
    GetSensorReadings();
    while(BusySensorAcq());
    
    for(index = 0; index < NO_OF_SENSORS; index++) {
        
        sensor_offsets[index] = sensor_readings_raw[2] - sensor_readings_raw[index];
        
    }
    
}

//Generate the set point for the PID algorithm
void CalibrateOnLine(void) {
    SendStatus(STATUS_4);
    LATJ = 0x07;
    
    while(PB1pressed() == 0);
    
    GetSensorReadings();
    while(BusySensorAcq() == 1);
    NormaliseSensorReadings(sensor_readings_raw, sensor_offsets);
    
    sensor_threshold = sensor_readings_normalised[2];
    PID_set_point = CalculateSensorSum(sensor_readings_normalised);
    
}

//Generate the threshold for determining if a sensor is over the line
void CalibrateThreshold(void) {
    int sensor_threshold_temp;
    int *sensor_threshold_temp_ptr;
    
    sensor_threshold_temp_ptr = &sensor_threshold_temp;
    
    SendStatus(STATUS_5);
    LATJ = 0x07;
    
    while(PB1pressed() == 0) {
    
        SetADCChannel(5);
        ConvertADC();
        while(BusyADC());
        sensor_threshold_temp = ReadADC();
        
        GetSensorReadings();
        while(BusySensorAcq() == 1);
        NormaliseSensorReadings(sensor_readings_raw, sensor_offsets);
        CalculateSensorStatuses(sensor_readings_normalised, sensor_threshold_temp_ptr);

        DisplaySensorStatuses(sensor_statuses);
    
    }
    
    sensor_threshold = sensor_threshold_temp;
    
}

//Calibration generation routine; sequences each calibration step
void GenerateCalibration(void) {
    unsigned char index;
    
    CalibrateOffsets();
    
    for(index = 0; index < NO_OF_SENSORS; index++) {
        WriteIntEEPROM(ADDR_S0_OFFSET + (2* index), sensor_offsets[index]);
    }
    
    FlashLEDS();
    
    CalibrateOnLine();
    
    WriteIntEEPROM(ADDR_PID_SET_POINT, PID_set_point);
    
    FlashLEDS();
    
    CalibrateThreshold();
    
    WriteIntEEPROM(ADDR_SENS_THRESHOLD, sensor_threshold);
    WriteCharEEPROM(ADDR_CAL_VALID, 1);
    
}

//Load calibration constants from EEPROM
void LoadCalibration(void) {
    unsigned char index;
    
    if(ReadCharEEPROM(ADDR_CAL_VALID) == 0xFF) {
        GenerateCalibration();
    }
    
    else if(ReadCharEEPROM(ADDR_CAL_VALID) == 1) {    
        for(index = 0; index < NO_OF_SENSORS; index++) {
            sensor_offsets[index] = ReadIntEEPROM(ADDR_S0_OFFSET + (2 * index));
        }
        
        PID_set_point = ReadIntEEPROM(ADDR_PID_SET_POINT);
        
        sensor_threshold = ReadIntEEPROM(ADDR_SENS_THRESHOLD);
    }
    
}

//Set PID values remotely via serial/LabVIEW interface
void SetPIDValues(void) {
    SendStatus(STATUS_8);
    
    unsigned char char_index;
    
    FlushRxBuf();
    
    while(DataRdy1USART() == 0);
    
    for(char_index = 0; char_index <= 8; char_index++) {
        while(!DataRdy1USART());
        
        PutCharRxBuf(Read1USART()); 
    }
    
    while(GetCharRxBuf() != RX_MSG_START);
    while(GetCharRxBuf() != RX_MSG_PID_VALUES);

    PID_Kp = (GetCharRxBuf() << 8) & 0xFF00;
    PID_Kp |= (GetCharRxBuf() & 0x00FF);

    PID_Kd = (GetCharRxBuf() << 8) & 0xFF00;
    PID_Kd |= (GetCharRxBuf() & 0x00FF);

    PID_Ki = (GetCharRxBuf() << 8) & 0xFF00;
    PID_Ki |= (GetCharRxBuf() & 0x00FF);

    WriteIntEEPROM(ADDR_PID_KP, PID_Kp);
    WriteIntEEPROM(ADDR_PID_KD, PID_Kd);
    WriteIntEEPROM(ADDR_PID_KI, PID_Ki);

    WriteCharEEPROM(ADDR_CAL_VALID_PID, 1);
    
}

//Load PID values from EEPROM
void LoadPIDValues(void) {
    SendStatus(STATUS_7);
    
    while(1) {
        if(PB1pressed() == 1) {
            PID_Kp = DEFAULT_PID_KP;
            PID_Kd = DEFAULT_PID_KD;
            PID_Ki = DEFAULT_PID_KI;
            WriteIntEEPROM(ADDR_PID_KP, PID_Kp);
            WriteIntEEPROM(ADDR_PID_KD, PID_Kd);
            WriteIntEEPROM(ADDR_PID_KI, PID_Ki);
            
            WriteCharEEPROM(ADDR_CAL_VALID_PID, 1);
            break;
        }
        else if(PB2pressed() == 1) {
            if(ReadCharEEPROM(ADDR_CAL_VALID_PID) == 0xFF) {
                SetPIDValues();
            }
            else {
                PID_Kp = ReadIntEEPROM(ADDR_PID_KP);
                PID_Kd = ReadIntEEPROM(ADDR_PID_KD);
                PID_Ki = ReadIntEEPROM(ADDR_PID_KI);
            }
            break;
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

    //INT3 ISR (for ultrasound)
    else if (INTCON3bits.INT3IF == 1) {
        INTCON3bits.INT3IF = 0;
        
        UltrasoundISR();
    }

}

//Low-priority ISR
void interrupt low_priority isrLP(void) {

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
    unsigned char stop_flag, find_line_flag, line_end_flag, wall_detected_flag, send_battery_stats_flag, command_msg, sensor_sum_discrete;
    unsigned int echo_length, battery_voltage, battery_current;
    unsigned long int battery_current_acc;
    int sensor_sum, D_temp, I_temp, I_max, I_min, Kp_stored, Kd_stored, Ki_stored;
    
    //Pointer declarations and assignments
    unsigned char *line_mode_ptr;
    int *sensor_threshold_ptr, *sensor_sum_ptr, *PID_error_ptr, *PID_set_point_ptr;
    
    line_mode_ptr = &line_mode;
    sensor_threshold_ptr = &sensor_threshold;
    sensor_sum_ptr = &sensor_sum;
    PID_error_ptr = &PID_error;
    PID_set_point_ptr = &PID_set_point;
    
    unsigned int *battery_voltage_ptr, *battery_current_ptr;
    unsigned long int *battery_current_acc_ptr;
    
    battery_voltage_ptr = &battery_voltage;
    battery_current_ptr = &battery_current;
    battery_current_acc_ptr = &battery_current_acc;
    
    //CONFIGURE BUGGY (one-time on power-up)
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
    
    Delay10KTCYx(100);      //Delay to allow battery monitor IC to initialise before getting a reading
    
    battery_voltage = ReadVoltage();

    SendStatus(STATUS_0);
    
    SendBattVoltageInitial(battery_voltage_ptr);    //Send battery voltage upon power-up to provide indication of level of charge
        
    //LINE MODE SETTING
    LATJ = 0x1F;
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
    LATJ = 0x0F;
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
            break;
        }

    }
    
    SendOffsets(sensor_offsets);
    SendThreshold(sensor_threshold_ptr);
    SendPIDSetPointOnLine(PID_set_point_ptr);
    
    FlashLEDS();
    
    //SET / LOAD PID VALUES
    LATJ = 0x03;
    SendStatus(STATUS_6);
    
    SendDefaultPIDValues(DEFAULT_PID_KP, DEFAULT_PID_KD, DEFAULT_PID_KI);
    
    Kp_stored = ReadIntEEPROM(ADDR_PID_KP);
    Kd_stored = ReadIntEEPROM(ADDR_PID_KD);
    Ki_stored = ReadIntEEPROM(ADDR_PID_KI);
    
    SendStoredPIDValues(Kp_stored, Kd_stored, Ki_stored);
    
    while(1) {
        if(PB1pressed() == 1) {
            FlashLEDS();
            LATJ = 0x03;
            SetPIDValues();
            break;
        }
        else if(PB2pressed() == 1) {
            FlashLEDS();
            LATJ = 0x03;
            LoadPIDValues();
            break;            
        }       
    }
    
    SendCurrentPIDValues(PID_Kp, PID_Kd, PID_Ki);
    
    //READY TO RACE / RACE MODE
    while(1) {
        
        //READY TO RACE
        FlashLEDS();
        
        LATJ = 0x01;
        SendStatus(STATUS_9);

        while(1) {
            if(PB1pressed() == 1) {
                break;
            }
            if(CommandAvailable() == 1 && GetCommand() == RX_MSG_START_RACE) {
                break;
            }
        }    

        //RACE INITIALISATION
        PID_error = 0;
        PID_output = 0;
        D_temp = 0;
        I_temp = 0;
        
        stop_flag = 0;
        find_line_flag = 0;
        line_end_flag = 0;
        wall_detected_flag = 0;
        send_battery_stats_flag = 0;
        
        LATJ = 0x00;

        FlashLEDS();
        SendStatus(STATUS_10);
        
        ResetAccumulatedCurrent();
        
        ResetMillis0();
        ResetMillis1();
        ResetMillis2();
        ResetMillis3();

        GetDistance();
        
        SetUnipolar();
        StopMotors();
        SetDirectionForward();
        EnableMotors();

        //RACE ROUTINE
        while(1) {
            
            ResetMillis1();
            
            //GET SENSOR READINGS
            GetSensorReadings();

            while(BusySensorAcq());
            NormaliseSensorReadings(sensor_readings_raw, sensor_offsets);
            CalculateSensorStatuses(sensor_readings_normalised, sensor_threshold_ptr);

            sensor_sum = CalculateSensorSum(sensor_readings_normalised);
            sensor_sum_discrete = CalculateSensorSumDiscrete(sensor_statuses);

            //PID LOOP
            PID_error = (PID_set_point - sensor_sum) / 100;
            PID_output = PID_Kp * PID_error + PID_Kd * (D_temp - PID_error) + PID_Ki * I_temp;
            D_temp = PID_error;
            I_temp += PID_error;
            
            SetDCMotorPID(PID_output);
            DisplaySensorStatuses(sensor_statuses);

            //WALL DETECTION
            if(BusyDistanceAcq() == 0 && wall_detected_flag == 0  && ReadMillis0() >= ULTRASOUND_POLLING_DELAY) {
                
                ResetMillis0();
                
                if(ReadEchoLength() < ECHO_LENGTH_FOR_WALL) {  //1232
                    wall_detected_flag = 1;
                                    
                    //TURN AROUND ROUTINE
                    DisableMotors();
                    
                    ResetMillis0();
                    while(ReadMillis0() < STOP_DELAY_1);
                                        
                    PID_error = 0;
                    PID_output = 0;
                    D_temp = 0;
                    I_temp = 0;
                    
                    SetForwardsMotorR();
                    SetReverseMotorL();                
                    
                    SetDCMotorL(DC_MAX_SPEED_REV_L);
                    SetDCMotorR(DC_MAX_SPEED_REV_R);
                    
                    EnableMotors();
                    
                    sensor_sum_discrete = 0;
                    
                    ResetMillis0();
                    while(ReadMillis0() < TURN_AROUND_DELAY);
                    
                    GetSensorReadings();
                    while(BusySensorAcq());
                    NormaliseSensorReadings(sensor_readings_raw, sensor_offsets);
                    CalculateSensorStatuses(sensor_readings_normalised, sensor_threshold_ptr);
                   
                    while(sensor_statuses[2] != 1) {
                        
                        GetSensorReadings();
                        while(BusySensorAcq());
                        NormaliseSensorReadings(sensor_readings_raw, sensor_offsets);
                        CalculateSensorStatuses(sensor_readings_normalised, sensor_threshold_ptr);
                        
                        //sensor_sum_discrete = CalculateSensorSumDiscrete(sensor_statuses);
                        
                        DisplaySensorStatuses(sensor_statuses);
                    }
                    
                    DisableMotors();
                    SetDCMotorL(DC_STOP);
                    SetDCMotorR(DC_STOP);
                    SetDirectionForward();
                    EnableMotors();
                    
                    ResetMillis0();
                    while(ReadMillis0() < STOP_DELAY_2);

                }
                else {
                    GetDistance();
                }   
            }
            
            //END OF LINE DETECTION
            else if(sensor_sum_discrete == 0) {
                ResetMillis2();
                while(ReadMillis2() <= END_OF_LINE_DELAY) {
                
                    GetSensorReadings();
                    while(BusySensorAcq());
                    NormaliseSensorReadings(sensor_readings_raw, sensor_offsets);
                    CalculateSensorStatuses(sensor_readings_normalised, sensor_threshold_ptr);

                    sensor_sum_discrete = CalculateSensorSumDiscrete(sensor_statuses);
                
                }
                
                if(sensor_sum_discrete == 0) {
                    StopMotors();
                    break;
                }
                                
            }
            
            //WIRELESS COMMAND HANDLING
            else if(CommandAvailable() == 1) {
                command_msg = GetCommand();
                if(command_msg == RX_MSG_FIND_LINE) {
                    find_line_flag = 1;
                }
                else if(command_msg == RX_MSG_STOP_BUGGY) {
                    stop_flag = 1;
                }
                
            }
            
            //PB1/PB2 STOP FUNCTIONALITY
            else if(PB1pressed() == 1 | PB2pressed() == 1) {
                stop_flag = 1;
            }
            
//            //BUGGY FIND LINE COMMAND
//            else if(find_line_flag == 1) {
//                //Find line routine
//                
//            }
            
                        //BATTERY MONITORING
//            else if(ReadMillis1() >= BATTERY_STATS_DELAY && send_battery_stats_flag == 0) {
//                ResetMillis1();
//                battery_voltage = ReadVoltage();
//                SendBattVoltage(battery_voltage_ptr);
//                send_battery_stats_flag++;
//            }
//            
//            else if(ReadMillis1() >= BATTERY_STATS_DELAY && send_battery_stats_flag == 1) {
//                ResetMillis1();
//                battery_current = ReadCurrent();
//                SendBattCurrent(battery_current_ptr);
//                send_battery_stats_flag++;
//            }
//            
//            else if(ReadMillis1() >= BATTERY_STATS_DELAY && send_battery_stats_flag == 2) {
//                ResetMillis1();
//                battery_current_acc = ReadAccumulatedCurrent();
//                SendBattCurrentAcc(battery_current_acc_ptr);
//                send_battery_stats_flag = 0;
//            }
            
            //BUGGY STOP COMMAND
            else if(stop_flag == 1) {
                StopMotors();
                break;
            }
            
            //ITERATE LOOP EVERY X MILLISECONDS
            while(ReadMillis1() <= CONTROL_LOOP_DELAY);

        }       
        
        //SET PID VALUES FOR NEXT LOOP - FOR PID TUNING
        FlashLEDS();
        LATJ = 0x03;
        SendStatus(STATUS_5);

        Kp_stored = ReadIntEEPROM(ADDR_PID_KP);
        Kd_stored = ReadIntEEPROM(ADDR_PID_KD);
        Ki_stored = ReadIntEEPROM(ADDR_PID_KI);

        SendStoredPIDValues(Kp_stored, Kd_stored, Ki_stored);
        
        SetPIDValues();

        SendCurrentPIDValues(PID_Kp, PID_Kd, PID_Ki);  
        
    }

}

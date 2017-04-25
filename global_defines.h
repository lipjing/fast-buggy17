/* 
 * File:   global_defines.h
 * Author: Jack
 *
 * Created on 21 March 2017, 16:39
 */

#ifndef GLOBAL_DEFINES_H
#define	GLOBAL_DEFINES_H

#ifdef	__cplusplus
extern "C" {
#endif

    #define NO_OF_SENSORS   5        //Number of sensors in use
    
    //Motor control pin connections
#define MOTOR_R_BI  LATDbits.LATD0
#define MOTOR_R_DIR LATDbits.LATD1
#define MOTOR_L_BI  LATDbits.LATD2
#define MOTOR_L_DIR LATDbits.LATD3
#define MOTOR_EN    LATDbits.LATD4

    //Motor duty cycle defines - duty cycles are in reverse - lower numbers equal higher motor speeds
#define DC_MAX_SPEED 240
#define DC_MAX_SPEED_REV_L 250
#define DC_MAX_SPEED_REV_R 340 
#define DC_STOP 526
    
#define NO_OF_STEPS 10

#define TIMER0_VALUE    63036    //Value written to Timer0 to generate ~1ms delay
    
#define BUFFER_SIZE 25      //RX and TX buffer sizes in bytes
    
#define TX_MSG_START 0xAA
#define RX_MSG_START 0xAA
    
#define TX_MSG_STATUS 0x01
#define TX_MSG_LINE_MODE 0x11
#define TX_MSG_SENS_OFFSETS 0x21
#define TX_MSG_SENS_THRESHOLD 0x31
#define TX_MSG_DEFAULT_PID_VALUES 0x41
#define TX_MSG_STORED_PID_VALUES 0x42
#define TX_MSG_CURRENT_PID_VALUES 0x43
#define TX_MSG_BATT_VOLT 0x03
#define TX_MSG_BATT_CURR 0x04
#define TX_MSG_BATT_CURR_ACC 0x05
#define TX_MSG_BATT_VOLT_INITIAL 0x06
    
#define RX_MSG_PID_VALUES 0x01
#define RX_MSG_START_RACE 0x02
#define RX_MSG_FIND_LINE 0x03
#define RX_MSG_STOP_BUGGY 0x04
#define RX_MSG_PB1 0x05
#define RX_MSG_PB2 0x06
    
#define STATUS_0 0x00
#define STATUS_1 0x01
#define STATUS_2 0x02
#define STATUS_3 0x03
#define STATUS_4 0x04
#define STATUS_5 0x05
#define STATUS_6 0x06
#define STATUS_7 0x07
#define STATUS_8 0x08
#define STATUS_9 0x09
#define STATUS_10 0x0A
#define STATUS_ERR 0x0E
    
#define CR 0x0D
#define LF 0x0A


#ifdef	__cplusplus
}
#endif

#endif	/* GLOBAL_DEFINES_H */


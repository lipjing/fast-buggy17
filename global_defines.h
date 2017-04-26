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
#define DC_MAX_SPEED 100 //240  //Maximum speed
#define DC_MAX_SPEED_REV_L 250  //L motor reverse speed
#define DC_MAX_SPEED_REV_R 340  //R motor reverse speed
#define DC_STOP 526     //Stop

#define TIMER0_VALUE    63036    //Value written to Timer0 to generate ~1ms delay
    
#define BUFFER_SIZE 25      //RX and TX buffer sizes in bytes
    
    //Transmit/receive message headers
#define TX_MSG_START 0xAA
#define RX_MSG_START 0xAA
  
    //Transmit message codes
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
#define TX_MSG_PID_SET_POINT_ON_LINE 0x07
#define TX_MSG_PID_SET_POINT_OFF_LINE 0x08
#define TX_MSG_PID_ERROR 0x09
    
    //Receive message codes
#define RX_MSG_PID_VALUES 0x01
#define RX_MSG_START_RACE 0x02
#define RX_MSG_FIND_LINE 0x03
#define RX_MSG_STOP_BUGGY 0x04
#define RX_MSG_PB1 0x05
#define RX_MSG_PB2 0x06
    
    //Status message codes
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

    //Address definitions for storage of calibration constants in EEPROM
#define ADDR_CAL_VALID 0x0000
#define ADDR_SENS_THRESHOLD 0x0001
#define ADDR_S0_OFFSET 0x0003
#define ADDR_S1_OFFSET 0x0005
#define ADDR_S2_OFFSET 0x0007
#define ADDR_S3_OFFSET 0x0009
#define ADDR_S4_OFFSET 0x000B
#define ADDR_CAL_VALID_PID 0x000D
#define ADDR_PID_SET_POINT 0x000E
#define ADDR_PID_KP 0x0010
#define ADDR_PID_KD 0x0012
#define ADDR_PID_KI 0x0014


#ifdef	__cplusplus
}
#endif

#endif	/* GLOBAL_DEFINES_H */


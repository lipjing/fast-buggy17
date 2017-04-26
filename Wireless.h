/* 
 * File:   Wireless.h
 * Author: Jack
 *
 * Created on 12 April 2017, 15:17
 */

#ifndef WIRELESS_H
#define	WIRELESS_H

#ifdef	__cplusplus
extern "C" {
#endif

    void PutCharTxBuf(unsigned char ch);
    
    unsigned char GetCharTxBuf(void);
    
    void PutCharRxBuf(unsigned char ch);
    
    unsigned char GetCharRxBuf(void);
    
    void StartTx(void);
    
    void StopTx(void);
    
    void StartRx(void);
    
    void StopRx(void);
    
    inline void WirelessTx_ISR(void);
    
    inline void WirelessRx_ISR(void);
    
    void FlushTxBuf(void);
    
    void FlushRxBuf(void);
    
    void ConfigureWireless(void);
    
    unsigned char BusyTx(void);
    
    unsigned char BusyRx(void);
    
    void SendStatus(const unsigned char status_code);
    
    void SendLineMode(const unsigned char *line_mode);
    
    void SendOffsets(const int *offsets_array);
    
    void SendThreshold(const int *sensor_threshold);
    
    void SendDefaultPIDValues(int Kp, int Kd, int Ki);

    void SendStoredPIDValues(int Kp, int Kd, int Ki);
    
    void SendCurrentPIDValues(int Kp, int Kd, int Ki);
    
    void SendBattVoltageInitial(const unsigned int *reading);
    
    void SendBattVoltage(const unsigned int *reading);
    
    void SendBattCurrent(const unsigned int *reading);
    
    void SendBattCurrentAcc(const unsigned long int *reading);
    
    void SendPIDSetPointOnLine(const int *set_point);
    
    void SendPIDSetPointOffLine(const int *set_point);
    
    void SendPIDError(const int error);
    
    void ReceiveCommandsEnable(void);
    
    void ReceiveCommandsDisable(void);
    
    unsigned char CommandAvailable(void);
    
    unsigned char GetCommand(void);

#ifdef	__cplusplus
}
#endif

#endif	/* WIRELESS_H */


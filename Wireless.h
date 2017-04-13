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
    
    void StartTx(void);
    
    void StopTx(void);
    
    inline void WirelessTx_ISR(void);
    
    inline void WirelessRX_ISR(void);
    
    void ClearBufTx(void);
    
    void ClearBufRX(void);
    
    void ConfigureWireless(void);
    
    unsigned char BusyTx(void);
    
    void SendStatus(const unsigned char status_code);
    
    void SendLineMode(const unsigned char *line_mode);
    
    void SendOffsets(const int *offsets_array);
    
    void SendThreshold(const int *sensor_threshold);
    
    



#ifdef	__cplusplus
}
#endif

#endif	/* WIRELESS_H */


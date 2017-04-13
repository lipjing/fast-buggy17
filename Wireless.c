#include "plib/usart.h"
#include "plib/delays.h"
#include "global_defines.h"

unsigned char Tx_buffer[BUFFER_SIZE], Rx_buffer[BUFFER_SIZE], put_Tx_index, get_Tx_index, Tx_complete;

volatile unsigned char character;

void PutCharTxBuf(unsigned char ch) {
    Tx_buffer[put_Tx_index] = ch;
    put_Tx_index = (unsigned char)(put_Tx_index + 1) % BUFFER_SIZE;
    
}

unsigned char GetCharTxBuf(void) {
    unsigned char ch;
    
    ch = Tx_buffer[get_Tx_index];
    get_Tx_index = (unsigned char)(get_Tx_index + 1) % BUFFER_SIZE;
    
    return(ch);
    
}

void StartTx(void) {
    Tx_complete = 0;
    PIE1bits.TX1IE = 1;
}

void StopTx(void) {
    PIE1bits.TX1IE = 0;
    PIR1bits.TX1IF = 0;
}

inline void WirelessTx_ISR(void) {
    character = GetCharTxBuf();
    if(character == 0x0A) {
        Write1USART(character);
        Tx_complete = 1;
        PIE1bits.TX1IE = 0;
    }
    else {
        Write1USART(character);       
    }
}

inline void WirelessRx_ISR(void) {
    
    
    
    
}

void FlushTxBuf(void) {
    unsigned char index;
    
    for(index = 0; index < BUFFER_SIZE; index++) {
        Tx_buffer[index] = 0;
    }
    
    put_Tx_index = 0;
    get_Tx_index = 0;
    
}

void FlushRxBuf(void) {
    unsigned char index;
    
    for(index = 0; index < BUFFER_SIZE; index++) {
        Rx_buffer[index] = 0;
    }
    
}

void ConfigureWireless(void) {
    
    Open1USART(USART_TX_INT_OFF     //Disable Tx and Rx interrupts
        & USART_RX_INT_OFF
        & USART_ASYNCH_MODE         //Asynchronous mode (no clock)
        & USART_EIGHT_BIT           //8-bit mode
        & USART_CONT_RX             //Continuous Rx
        & USART_BRGH_HIGH,          //BRG in high-speed mode
        64);                        //SPBRG register value, produces baud rate of 9600bps
    
    FlushTxBuf();
    FlushRxBuf();
    
    get_Tx_index = 0;
    put_Tx_index = 0;
    
    Tx_complete = 1;
    
}

unsigned char BusyTx(void) {
    return(!Tx_complete);    
}

void SendStatus(const unsigned char status_code) {
    while(BusyTx());
    FlushTxBuf();
    
    PutCharTxBuf(0xAA);
    PutCharTxBuf(MSG_STATUS);
    PutCharTxBuf(status_code);
    PutCharTxBuf('\r');
    PutCharTxBuf('\n');

    StartTx();   
}

void SendLineMode(const unsigned char *line_mode) {
    while(BusyTx());
    FlushTxBuf();
    
    PutCharTxBuf(0xAA);
    PutCharTxBuf(MSG_LINE_MODE);
    PutCharTxBuf(*line_mode);
    PutCharTxBuf('\r');
    PutCharTxBuf('\n');

    StartTx();   
}

void SendOffsets(const int *offsets_array) {
    unsigned char index;
    
    while(BusyTx());
    FlushTxBuf();
    
    PutCharTxBuf(0xAA);
    PutCharTxBuf(MSG_SENS_OFFSETS);
    
    for(index = 0; index < NO_OF_SENSORS; index++) {
        PutCharTxBuf((unsigned char)(*(offsets_array + index) >> 8 & 0x00FF));
        PutCharTxBuf((unsigned char)(*(offsets_array + index) * 0x00FF));        
    }
    
    PutCharTxBuf('\r');
    PutCharTxBuf('\n');
    while(BusyTx());
    StartTx();
    
}

void SendThreshold(const int *sensor_threshold) {
    while(BusyTx());
    FlushTxBuf();
    
    PutCharTxBuf(0xAA);
    PutCharTxBuf(MSG_SENS_THRESHOLD);
    
    PutCharTxBuf((unsigned char)(*sensor_threshold >> 8 & 0x00FF));
    PutCharTxBuf((unsigned char)(*sensor_threshold & 0x00FF));

    PutCharTxBuf('\r');
    PutCharTxBuf('\n');
    while (BusyTx());
    StartTx();  
    
}

//void SendPIDValues(void) {
//    
//    
//    
//}
//
//void SendBattVoltage(void) {
//    
//    
//    
//}
//
//void SendCurrent(void) {
//    
//    
//    
//}
//
//void SendAccCurrent(void) {
//    
//    
//    
//}



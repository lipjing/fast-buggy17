#include "plib/usart.h"
#include "plib/delays.h"
#include "global_defines.h"

unsigned char Tx_buffer[BUFFER_SIZE], Rx_buffer[BUFFER_SIZE], put_Tx_index, get_Tx_index, put_Rx_index, get_Rx_index, Tx_complete;

volatile unsigned char TxCharacter, RxCharacter, Rx_chars_received, Rx_msg_length, Rx_complete;

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

void PutCharRxBuf(unsigned char ch) {
    Rx_buffer[put_Rx_index] = ch;
    put_Rx_index = (unsigned char)(put_Rx_index + 1) % BUFFER_SIZE;
    
}

unsigned char GetCharRxBuf(void) {
    unsigned char ch;
    
    ch = Rx_buffer[get_Rx_index];
    get_Rx_index = (unsigned char)(get_Rx_index + 1) % BUFFER_SIZE;
    
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

void StartRx(void) {
    Rx_complete = 0;
    PIE1bits.RC1IE = 1;
}

void StopRx(void) {
    PIR1bits.RC1IF = 0;
    PIE1bits.RC1IE = 0;    
}

inline void WirelessTx_ISR(void) {
    TxCharacter = GetCharTxBuf();
    if(TxCharacter == 0x0A) {
        Write1USART(TxCharacter);
        Tx_complete = 1;
        PIE1bits.TX1IE = 0;
    }
    else {
        Write1USART(TxCharacter);       
    }
}

inline void WirelessRx_ISR(void) {
    RxCharacter = Read1USART();
    Rx_chars_received++;
    if(RxCharacter == 0x0A && Rx_chars_received == Rx_msg_length) {
        PutCharRxBuf(RxCharacter);
        Rx_complete = 1;
        Rx_chars_received = 0;
    }
    else {
        PutCharRxBuf(RxCharacter);
        Rx_complete = 0;
    }
    
    
    
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
    
    put_Rx_index = 0;
    get_Rx_index = 0;
    
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
    
    Tx_complete = 1;
    
}

unsigned char BusyTx(void) {
    return(!Tx_complete);    
}

unsigned char BusyRx(void) {
    return(!Rx_complete);
}

void SendStatus(const unsigned char status_code) {
    while(BusyTx());
    FlushTxBuf();
    
    PutCharTxBuf(0xAA);
    PutCharTxBuf(TX_MSG_STATUS);
    PutCharTxBuf((unsigned char)status_code);
    PutCharTxBuf('\r');
    PutCharTxBuf('\n');

    StartTx();   
}

void SendLineMode(const unsigned char *line_mode) {
    while(BusyTx());
    FlushTxBuf();
    
    PutCharTxBuf(0xAA);
    PutCharTxBuf(TX_MSG_LINE_MODE);
    PutCharTxBuf((unsigned char)*line_mode);
    PutCharTxBuf('\r');
    PutCharTxBuf('\n');

    StartTx();   
}

void SendOffsets(const int *offsets_array) {
    unsigned char index;
    
    while(BusyTx());
    FlushTxBuf();
    
    PutCharTxBuf(0xAA);
    PutCharTxBuf(TX_MSG_SENS_OFFSETS);
    
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
    PutCharTxBuf(TX_MSG_SENS_THRESHOLD);
    
    PutCharTxBuf((unsigned char)(*sensor_threshold >> 8 & 0x00FF));
    PutCharTxBuf((unsigned char)(*sensor_threshold & 0x00FF));

    PutCharTxBuf('\r');
    PutCharTxBuf('\n');
    while (BusyTx());
    StartTx();  
    
}

void SendDefaultPIDValues(int Kp, int Kd, int Ki) {
    while(BusyTx());
    FlushTxBuf();
    
    PutCharTxBuf(0xAA);
    PutCharTxBuf(TX_MSG_DEFAULT_PID_VALUES);
    
    PutCharTxBuf((unsigned char)(Kp >> 8 & 0x00FF));
    PutCharTxBuf((unsigned char)(Kp & 0x00FF));
    PutCharTxBuf((unsigned char)(Kd >> 8 & 0x00FF));
    PutCharTxBuf((unsigned char)(Kd & 0x00FF));
    PutCharTxBuf((unsigned char)(Ki >> 8 & 0x00FF));
    PutCharTxBuf((unsigned char)(Ki & 0x00FF));
    
    PutCharTxBuf('\r');
    PutCharTxBuf('\n');
    while (BusyTx());
    StartTx(); 
}

void SendStoredPIDValues(int Kp, int Kd, int Ki) {
    while(BusyTx());
    FlushTxBuf();
    
    PutCharTxBuf(0xAA);
    PutCharTxBuf(TX_MSG_STORED_PID_VALUES);
    
    PutCharTxBuf((unsigned char)(Kp >> 8 & 0x00FF));
    PutCharTxBuf((unsigned char)(Kp & 0x00FF));
    PutCharTxBuf((unsigned char)(Kd >> 8 & 0x00FF));
    PutCharTxBuf((unsigned char)(Kd & 0x00FF));
    PutCharTxBuf((unsigned char)(Ki >> 8 & 0x00FF));
    PutCharTxBuf((unsigned char)(Ki & 0x00FF));
    
    PutCharTxBuf('\r');
    PutCharTxBuf('\n');
    while (BusyTx());
    StartTx(); 
}

void SendCurrentPIDValues(int Kp, int Kd, int Ki) {
    while(BusyTx());
    FlushTxBuf();
    
    PutCharTxBuf(0xAA);
    PutCharTxBuf(TX_MSG_CURRENT_PID_VALUES);
    
    PutCharTxBuf((unsigned char)(Kp >> 8 & 0x00FF));
    PutCharTxBuf((unsigned char)(Kp & 0x00FF));
    PutCharTxBuf((unsigned char)(Kd >> 8 & 0x00FF));
    PutCharTxBuf((unsigned char)(Kd & 0x00FF));
    PutCharTxBuf((unsigned char)(Ki >> 8 & 0x00FF));
    PutCharTxBuf((unsigned char)(Ki & 0x00FF));
    
    PutCharTxBuf('\r');
    PutCharTxBuf('\n');
    while (BusyTx());
    StartTx(); 
}



void SendBattVoltageInitial(const unsigned int *reading) {
    while(BusyTx());
    FlushTxBuf();
    
    PutCharTxBuf(0xAA);
    PutCharTxBuf(TX_MSG_BATT_VOLT_INITIAL);
    
    PutCharTxBuf((unsigned char)(*reading >> 8 & 0x00FF));
    PutCharTxBuf((unsigned char)(*reading & 0x00FF));
    
    PutCharTxBuf('\r');
    PutCharTxBuf('\n');
    while (BusyTx());
    StartTx(); 
     
}

void SendBattVoltage(const unsigned int *reading) {
    while(BusyTx());
    FlushTxBuf();
    
    PutCharTxBuf(0xAA);
    PutCharTxBuf(TX_MSG_BATT_VOLT);
    
    PutCharTxBuf((unsigned char)(*reading >> 8 & 0x00FF));
    PutCharTxBuf((unsigned char)(*reading & 0x00FF));
    
    PutCharTxBuf('\r');
    PutCharTxBuf('\n');
    while (BusyTx());
    StartTx(); 
    
}

void SendBattCurrent(const unsigned int *reading) {
    while(BusyTx());
    FlushTxBuf();
    
    PutCharTxBuf(0xAA);
    PutCharTxBuf(TX_MSG_BATT_CURR);
    
    PutCharTxBuf((unsigned char)(*reading >> 8 & 0x00FF));
    PutCharTxBuf((unsigned char)(*reading & 0x00FF));
    
    PutCharTxBuf('\r');
    PutCharTxBuf('\n');
    while (BusyTx());
    StartTx(); 
    
    
}

void SendBattCurrentAcc(const unsigned long *reading) {
    while(BusyTx());
    FlushTxBuf();
    
    PutCharTxBuf(0xAA);
    PutCharTxBuf(TX_MSG_BATT_CURR_ACC);
    
    PutCharTxBuf((unsigned char)(*reading >> 24 & 0x000000FF));
    PutCharTxBuf((unsigned char)(*reading >> 16 & 0x000000FF));
    PutCharTxBuf((unsigned char)(*reading >> 8 & 0x000000FF));
    PutCharTxBuf((unsigned char)(*reading & 0x000000FF));
    
    PutCharTxBuf('\r');
    PutCharTxBuf('\n');
    while (BusyTx());
    StartTx();
    
    
}

void ReceiveCommandsEnable(void) {    
    FlushRxBuf();
    
    Rx_chars_received = 0;
    Rx_msg_length = 4; 

    StartRx();
    
}

void ReceiveCommandsDisable(void) {
    StopRx();
}

unsigned char CommandAvailable(void) {
    return(DataRdy1USART());
}

unsigned char GetCommand(void) {
   
    return(Read1USART());
}

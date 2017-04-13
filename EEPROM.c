#include "config_18f8722.h"

unsigned char WriteEEPROM(unsigned int address, unsigned char data) {
    if(address > 0x03FF) {
        return(1);
    }
    else {
        EEADR = (address & 0x00FF);
        EEADRH = ((address >> 8) & 0x0003);
        EEDATA = data;
        
        EECON1bits.EEPGD = 0;
        EECON1bits.CFGS = 0;
        EECON1bits.WREN = 1;
        
        INTCONbits.GIE = 0;
        EECON2 = 0x55;
        EECON2 = 0x0AA;
        EECON1bits.WR = 1;
        
        while(EECON1bits.WR == 1);
        
        EECON1bits.WREN = 0;        
        INTCONbits.GIE = 1;
        
        if(EECON1bits.WRERR == 0) {
            return(0);
        }
        else {
            return(1);
        }
        
    }  
    
}

unsigned char ReadEEPROM(unsigned int address) {
    if(address > 0x3FF) {
        return(0);
    }
    else {
        EEADR = address & 0x0F;
        EEADRH = (address >> 8) & 0x03;
        EECON1bits.EEPGD = 0;
        EECON1bits.CFGS = 0;
        EECON1bits.RD = 1;
        
        while(EECON1bits.RD == 1);
        
        return(EEDATA);
        
    }
       
}

void WriteIntEEPROM(unsigned int address, unsigned int data) {
    WriteEEPROM(address, ((data >> 8) & 0x00FF)); 
    WriteEEPROM(address + 1, (data & 0x00FF));
       
}

void WriteCharEEPROM(unsigned int address, unsigned char data) {
    WriteEEPROM(address, data);    
}

unsigned int ReadIntEEPROM(unsigned int address) {
    unsigned int data;
    
    data = (ReadEEPROM(address) << 8) & 0xFF00;
    data |= ReadEEPROM(address + 1) & 0x00FF;
        
    return(data);
    
}

unsigned char ReadCharEEPROM(unsigned int address) {
    return(ReadEEPROM(address));
    
}
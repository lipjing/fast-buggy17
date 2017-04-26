/* 
 * File:   eeprom.h
 * Author: Jack
 *
 * Created on 10 April 2017, 18:15
 */

#ifndef EEPROM_H
#define	EEPROM_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    unsigned char WriteEEPROM(unsigned int address, unsigned char data);
    
    unsigned char ReadEEPROM(unsigned int address);
    
//    void WriteFloat24EEPROM(unsigned int address, float data);
    
    void WriteIntEEPROM(unsigned int address, unsigned int data);
    
    void WriteCharEEPROM(unsigned int address, unsigned char data);
    
//    float ReadFloat24EEPROM(unsigned int address);
    
    unsigned int ReadIntEEPROM(unsigned int address);
    
    unsigned char ReadCharEEPROM(unsigned int address);

#ifdef	__cplusplus
}
#endif

#endif	/* EEPROM_H */


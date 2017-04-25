#define _XTAL_FREQ 10000000 //Setting HS(osc) as 10MHz-used in delays etc

#include <xc.h>
#include "OneWire.h"

/* --------------------------------------------------------------------------
   This file includes the basic functions needed to communicate with a 1-Wire
   device. 1-Wire is a device communications bus system designed by Dallas 
   Semiconductor Corp. that provides low-speed data, signaling and power over
   a single signal [1]. The source code developed is based on the example
   provided by Maxim in application note 126 [2].
   --------------------------------------------------------------------------
   [1] http://en.wikipedia.org/wiki/1-Wire
   [2] http://www.maxim-ic.com/app-notes/index.mvp/id/126
   --------------------------------------------------------------------------
   Project Supervisor: Dr. J. Apsley
   Date Completed: 06/08/2010
   --------------------------------------------------------------------------
   NOTE_1: The functions are defined to run on a PICmicro at a clock speed of
   10MHz. If the clock speed is 40 MHz, the user must #define __FREQ40MHz__.
   To use the functions at other frequencies the delay values must be changed
   in the header file.
   NOTE_2: The pin used for 1-Wire data I/O is set by default as RC3. To
   modify that, the user must change the definition of WIRE and WIRE_TRIS in
   the header file.  
   -------------------------------------------------------------------------- */




/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : OWReset													   *
 * Overview : Implements 1-Wire initialization sequence by sending a reset	   *
 *			  pulse and detecting a presence pulse from the 1-Wire device.     *
 *			  Returns TRUE if device present and FALSE if not				   *
 * Return type : Boolean													   *
 * Parameters : None														   *
 * Time : < 963us															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

BOOL OWReset (void)
{
	BOOL result = FALSE;
    
    WIRE = 0; 
    WIRE_TRIS = 0; 		// Drives DQ low
	
    Delay_H();

    WIRE_TRIS = INPUT; 			// Releases the bus
    Delay_I();

    if(WIRE == 0) 				// Sample for presence pulse from slave
	{
		result = TRUE;
	}
    Delay_J(); 					// Complete the reset sequence recovery

    return result; 				// Return sample presence pulse result

}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : OWWriteBit												   *
 * Overview : Writes a bit to the 1-Wire device as supplied by the parameter   *
 * Return type : Void										  				   *
 * Parameters : Logic level of bit to be written to the device, non-zero 	   *
 * 				value for logic-high and zero for logic-low					   *
 * Time : < 73us															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void OWWriteBit (BYTE b){
	
	if (b)
    {
    	// Write '1' bit
        WIRE = 0; 
		WIRE_TRIS = OUTPUT; 	// Drives DQ low
        //LATEbits.LATE0 = ~ LATEbits.LATE0; //just for checking timing
        Delay_A();
        //LATEbits.LATE0 = ~ LATEbits.LATE0; //just for checking timing

        WIRE_TRIS = INPUT; 		// Releases the bus
        Delay_B(); 				// Complete the time slot and recovery
        //LATEbits.LATE0 = ~ LATEbits.LATE0; //just for checking timing
    }
    else
    {
        // Write '0' bit
    	WIRE = 0; 
		WIRE_TRIS = OUTPUT; 	// Drives DQ low
        //LATEbits.LATE0 = ~ LATEbits.LATE0; //just for checking timing
        Delay_C();
        //LATEbits.LATE0 = ~ LATEbits.LATE0; //just for checking timing

        WIRE_TRIS = INPUT; 		// Releases the bus
        Delay_D(); 				// Complete the time slot and recovery
        //LATEbits.LATE0 = ~ LATEbits.LATE0; //just for checking timing
    }

}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : OWReadBit												   *
 * Overview : Reads a bit from the 1-Wire device. Returns the value one for	   *
 * 			  logic-high and zero for logic-low								   *
 * Return type : 8-bit unsigned char										   *
 * Parameters : None														   *
 * Time : < 74us															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

BYTE OWReadBit (void){
	
	BYTE result;

    WIRE = 0; 
	WIRE_TRIS = OUTPUT; 		// Drives DQ low
    //LATEbits.LATE0 = ~ LATEbits.LATE0; //just for checking timing
    Delay_A();
    //__delay_us(4);
    //LATEbits.LATE0 = ~ LATEbits.LATE0; //just for checking timing

    WIRE_TRIS = INPUT; 			// Releases the bus
    Delay_E();
    //LATEbits.LATE0 = ~ LATEbits.LATE0; //just for checking timing

    result = WIRE_READ; 				// Sample the bit value from the slave
    Delay_F(); 					// Complete the time slot and recovery
    //LATEbits.LATE0 = ~ LATEbits.LATE0; //just for checking timing

    return result & 0x01;

}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : OWWriteByte												   *
 * Overview : Writes a byte to the 1-Wire device as supplied by the parameter  *
 * Return type : Void														   *
 * Parameters : Byte to be written											   *
 * Time : < 640us															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void OWWriteByte (BYTE byte){

	BYTE i;
    
    for (i = 0; i < 8; i++)		// Loop to write each bit in the byte, LS-bit first
    {
    	OWWriteBit(byte & 0x01);        
        byte >>= 1;				// shift the data byte for the next bit
    }

}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : OWReadByte												   *
 * Overview : Reads a byte from the 1-Wire device							   *
 * Return type : 8-bit unsigned char										   *
 * Parameters : None														   *
 * Time : < 720us															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

BYTE OWReadByte (void){

	BYTE i, result=0;

    for (i = 0; i < 8; i++)
    {    
        result >>= 1;			// shift the result to get it ready for the next bit
        
        if (OWReadBit())		// if result is one, then set MS bit
			result |= 0x80;		
    }
    return result;

}

/* EOF */
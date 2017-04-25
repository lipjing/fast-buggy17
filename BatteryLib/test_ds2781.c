#define _XTAL_FREQ 10000000 //Setting HS(osc) as 10MHz-used in delays etc

#include <xc.h>
#include "ds2781.h"

/* --------------------------------------------------------------------------
   This file includes a test routine for the DS2781. It continuously polls
   the device for voltage, current and accumulated current. The PICmicro must
   be connected to a breakout board and the UoM I/O board. The code uses
   toggle switches on RC2, RC3, RC4 to select the parameter to diplay on the
   7 segment displays. The different configurations are:
   RC4 = 0 & RC3 = 0 & RC2 = 1 > Display voltage in V
   RC4 = 0 & RC3 = 1 & RC2 = 0 > Display current in A
   RC4 = 1 & RC3 = 0 & RC2 = 0 > Display accumulated current in As
   RC4 = 1 & RC3 = 1 & RC2 = 0 > Reset accumulated current register
   --------------------------------------------------------------------------
   Project Supervisor: Dr. J. Apsley
   Date Completed: 06/08/2010
 
   Ported to PIC18F8722, MPLABX and XC8 Compiler (MPLABX v3.15, XC8 v1.35) 
   by Samuel Walsh <samuel.walsh@manchester.ac.uk> 01/02/16
   --------------------------------------------------------------------------
   NOTE_1: The data pin and clock frequency must be checked as stated in 
   ds2781.c and OneWire.c.
   NOTE_2: The links on the breakout board must be checked. The data pin must
   be disconnected from the I/O board.
   NOTE_3: The current calculations are based on a 10mOhm resistor.   
   -------------------------------------------------------------------------- */

/* ***** variables ***** */
BYTE display[ ] = {0x84, 0xF5, 0x4C, 0x64, 0x35, 0x26, 0x06, 0xB4,
        0x04, 0x24, 0x14, 0x07, 0x8E, 0x45, 0x0E, 0x1E};

BYTE u1=0, u2=0;
WORD y =0;
DWORD x =0;
BYTE n = 0;
BYTE switch_value;

/* ***** test routine ***** */
void main (void){

	TRISF = 0;						//7 segment LEDS
	TRISHbits.TRISH0 = 0;
    TRISHbits.TRISH1 = 0;			//7 segment on/off pins (RC5:U2	RC4:U1) (RH1:U2 RH0:U1))
    
    TRISEbits.TRISE0 = 0;           //Used for debugging
    
    ADCON1 = 0x0F;                  //Set all digital IO
	
	while(1){
        
        switch_value = PORTC & 0b00011100;
        switch_value = switch_value >> 2; 
        
		if(LATHbits.LATH1 == 0){						//Multiplex displays
			LATHbits.LATH1 = 1;
			LATF = u1;
			LATHbits.LATH0 = 0;
		}else{
			LATHbits.LATH0 = 1;
			LATF = u2;
			LATHbits.LATH1 = 0;
		}

		if(switch_value == 0x01)                        //First Switch Config
		{
			y = ReadVoltage();							//Read voltage register
			y /= 10;									//Scale: x 9.67m x 10
           // y = 43;
			u1 = display [y%10];						//Convert to BCD
			u2 = display [y/10];						//Display in V
			u2 &= 0xFB;									//Add decimal point
		}
		else if(switch_value == 0x02)
		{
			y = ReadCurrent();							//Read current register
			y /= 640;									//Scale: x 1.5625u x 1/0.01 x 10
			u1 = display [y%10];						//Display in A
			u2 = display [y/10];
			u2 &= 0xFB;
		}
		else if(switch_value == 0x04)
		{
			x = ReadAccumulatedCurrent();				//Read acc current register
			x /= 1820;									//Scale: x 1.526n x 1/0.01 x 3600
			u1 = display [x%10];						//Display in As
			u2 = display [x/10];
		}
		else if(switch_value == 0x06)                   //Reset acc current
		{
			ResetAccumulatedCurrent ();
		}
	}
}

/* EOF */
//		if( (PORTC & 0b00000111) == 0x01)				//First Switch Config
//		{
//			y = ReadVoltage();							//Read voltage register
//			y /= 10;									//Scale: x 9.67m x 10
//			u1 = display [y%10];						//Convert to BCD
//			u2 = display [y/10];						//Display in V
//			u2 &= 0xFB;									//Add deciman point
//		}
//		else if( (PORTC & 0b00000111) == 0x02)
//		{
//			y = ReadCurrent();							//Read current register
//			y /= 640;									//Scale: x 1.5625u x 1/0.01 x 10
//			u1 = display [y%10];						//Display in A
//			u2 = display [y/10];
//			u2 &= 0xFB;
//		}
//		else if( (PORTC & 0b00000111) == 0x04)
//		{
//			x = ReadAccumulatedCurrent();				//Read acc current register
//			x /= 1820;									//Scale: x 1.526n x 1/0.01 x 3600
//			u1 = display [x%10];						//Display in As
//			u2 = display [x/10];
//		}
//		else if( (PORTC & 0b00000111) == 0x06)			//Reset acc current
//		{
//			ResetAccumulatedCurrent ();
//		}
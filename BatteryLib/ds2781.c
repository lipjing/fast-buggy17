#define _XTAL_FREQ 10000000

#include "ds2781.h"

/* --------------------------------------------------------------------------
   This file includes the functions needed to access and modify the registers
   in a DS2781 using the 1-Wire protocol. The DS2781 is an IC that measures
   voltage, current, accumulated current and temperature. It implements
   capacity estimation algorithms for rechargeable batteries. However, this
   file only includes routines to access the electrical parameters and not
   the age-estimation registers.
   --------------------------------------------------------------------------
   Project Supervisor: Dr. J. Apsley
   Date Completed: 06/08/2010
   --------------------------------------------------------------------------
   NOTE_1: The functions that return parameters, do so in the units reported
   in the description of each function. The user should implement the scaling
   on his/her own.  
   -------------------------------------------------------------------------- */




/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : ReadVoltage												   *
 * Overview : Returns the voltage measured at the VIN input of the DS2781	   *
 * 			  in units of 9.76mV											   *
 * Return type : 16-bit unsigned int										   *
 * Parameters : None														   *
 * Time : < 4.3ms															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

WORD ReadVoltage (void)
{
	WORD result = 0;
	if( OWReset() == TRUE)
	{		
		OWWriteByte( SKIP_NETADDRESS );
		OWWriteByte( READ_DATA );
		OWWriteByte( 0x0C );						//Register Address
		result  = ((WORD)OWReadByte() ) << 8;		//MSB	
		result |= ((WORD)OWReadByte() );			//LSB
	}
	return (result >> 5);
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : ReadCurrent												   *
 * Overview : Returns the current measured through Rsns external to DS2781 in  *
 *			  units of 1.5625uV/Rsns. Positive current indicates discharge	   *
 * Return type : 16-bit unsigned int										   *
 * Parameters : None														   *
 * Time : < 4.3ms															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

WORD ReadCurrent (void)
{
	WORD result = 0;
	if( OWReset() == TRUE)
	{		
		OWWriteByte( SKIP_NETADDRESS );
		OWWriteByte( READ_DATA );
		OWWriteByte( 0x0E );						//Register Address
		result  = ((WORD)OWReadByte() ) << 8;		//MSB	
		result |= ((WORD)OWReadByte() );			//LSB
	}
	return result;
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : ReadAccumulatedCurrent									   *
 * Overview : Returns the accumulated current at the DS2781 in units of		   *
 *			  1.526nVhr/Rsns												   *
 * Return type : 32-bit unsigned long										   *
 * Parameters : None														   *
 * Time : < 5.8ms															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

DWORD ReadAccumulatedCurrent (void)
{
	DWORD result = 0;
	if( OWReset() == TRUE)
	{		
		OWWriteByte( SKIP_NETADDRESS );
		OWWriteByte( READ_DATA );
		OWWriteByte( 0x10 );						//Register Address
		result  = ((DWORD)OWReadByte() ) << 24;		//MSB
		result |= ((DWORD)OWReadByte() ) << 16;
		result |= ((DWORD)OWReadByte() ) << 8;	
		result |= ((DWORD)OWReadByte() );			//LSB
	}
	return (result >>  4);
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : ResetAccumulatedCurrent									   *
 * Overview : Resets the accumulated current register at the DS2781	  		   *
 * Return type : Void														   *
 * Parameters : None														   *
 * Time : < 4.2ms															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void ResetAccumulatedCurrent (void)
{
	if( OWReset() == TRUE)
	{		
		OWWriteByte( SKIP_NETADDRESS );
		OWWriteByte( WRITE_DATA );
		OWWriteByte( 0x10 );						//Register Address
		OWWriteByte( 0x00 );						//MSB
		OWWriteByte( 0x00 );						//LSB
	}
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : ReadNetAddress 											   *
 * Overview : Returns the net address of the DS2781	  						   *
 * Return type : 64-bit unsigned long long									   *
 * Parameters : None														   *
 * Time : < 7.3ms															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

QWORD ReadNetAddress (void)
{
	QWORD result = 0;
	if( OWReset() == TRUE)
	{		
		OWWriteByte( READ_NETADDRESS );
		//result  = ((QWORD)OWReadByte() );		//MSB
		//result |= ((QWORD)OWReadByte() ) << 8;
		//result |= ((QWORD)OWReadByte() ) << 16;
		//result |= ((QWORD)OWReadByte() ) << 24;
		//result |= ((QWORD)OWReadByte() ) << 32;
		//result |= ((QWORD)OWReadByte() ) << 40;
		//result |= ((QWORD)OWReadByte() ) << 48;	
		//result |= ((QWORD)OWReadByte() ) <<56;			//LSB
	}
	return result;
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : ReadTemperature											   *
 * Overview : Returns the temperature measured by the DS2781 in units of 	   *
 * 			  0.125°C														   *
 * Return type : 16-bit unsigned int										   *
 * Parameters : None														   *
 * Time : < 4.3ms															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

WORD ReadTemperature (void)
{
	WORD result = 0;
	if( OWReset() == TRUE)
	{		
		OWWriteByte( SKIP_NETADDRESS );
		OWWriteByte( READ_DATA );
		OWWriteByte( 0x0A );						//Register Address
		result  = ((WORD)OWReadByte() ) << 8;		//MSB	
		result |= ((WORD)OWReadByte() );			//LSB
	}
	return (result >> 5);
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : ReadCurrentOffset										   *
 * Overview : Returns the value of the current offset register of the DS2781   *
 * 			  in units of 1.56uV/Rsns										   *
 * Return type : 8-bit unsigned char										   *
 * Parameters : None														   *
 * Time : < 3.6ms															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

BYTE ReadCurrentOffset (void)
{
	BYTE result = 0;
	if( OWReset() == TRUE)
	{		
		OWWriteByte( SKIP_NETADDRESS );
		OWWriteByte( READ_DATA );
		OWWriteByte( 0x7B );						//Register Address
		result  = OWReadByte();
	}
	return result;
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : WriteCurrentOffset										   *
 * Overview : Writes to the current offset register of the DS2781 in units of  *
 * 			  1.56uV/Rsns													   *
 * Return type : Void														   *
 * Parameters : Byte to be written to the register in 2's complement		   *
 * Time : < 3.6ms															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void  WriteCurrentOffset (CHAR offset)
{
	if( OWReset() == TRUE)
	{		
		OWWriteByte( SKIP_NETADDRESS );
		OWWriteByte( WRITE_DATA );
		OWWriteByte( 0x7B );						//Register Address
		OWWriteByte( offset );
	}
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : AdjustCurrentOffset										   *
 * Overview : Adjusts the value of the current offset register of the DS2781   *
 * 			  by taking into account the offset at no current. Should only	   *
 *			  be called when the battery is supplying no current			   *
 * Return type : Void														   *
 * Parameters : None														   *
 * Time : < 3.62s															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void AdjustCurrentOffset (void)
{
	CHAR offset = 0;
	
	WriteCurrentOffset ( 0x0 );						//Reset Current Offset Register

	Delay100MSx(36);								//Wait 3.6s for current register to update

	if( OWReset() == TRUE)
	{		
		OWWriteByte( SKIP_NETADDRESS );
		OWWriteByte( READ_DATA );
		OWWriteByte( 0x0F );						//Current Register LSB
		offset  = OWReadByte();	
	}

	offset = 256 - offset;							//2's complement Negating

	if( OWReset() == TRUE)
	{		
		OWWriteByte( SKIP_NETADDRESS );
		OWWriteByte( WRITE_DATA );
		OWWriteByte( 0x7B );						//Current Offset Register
		OWWriteByte( offset );	
	}
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : UpdateControlRegister									   *
 * Overview : Writes to the Control register of the DS2781 using the values	   *
 * 			  supplied as a byte parameter. Writes to EEPROM addresses are	   *
 * 			  ignored for up to 15ms after this function is called.			   *
 * Return type : Void														   *
 * Parameters : None														   *
 * Time : < 6.4ms															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void UpdateControlRegister (BYTE control)
{
	if( OWReset() == TRUE )
	{		
		OWWriteByte( SKIP_NETADDRESS );
		OWWriteByte( WRITE_DATA );
		OWWriteByte( 0x60 );						//Register Address
		OWWriteByte( control );
	}

	if( OWReset() == TRUE )
	{		
		OWWriteByte( SKIP_NETADDRESS );
		OWWriteByte( COPY_DATA );
		OWWriteByte( 0x60 );						//Register Address
	}
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : ReadRAM													   *
 * Overview : Reads a byte from the shadow RAM of the DS2781 at the given 	   *
 * 			  memory address												   *
 * Return type : 8-bit unsigned char										   *
 * Parameters : Address of register to be read								   *
 * Time : < 3.6ms															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

BYTE ReadRAM (BYTE addr)
{
	BYTE result = 0;
	if( OWReset() == TRUE)
	{		
		OWWriteByte( SKIP_NETADDRESS );
		OWWriteByte( READ_DATA );
		OWWriteByte( addr );						//Register Address
		result  = OWReadByte();
	}
	return result;
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : WriteRAM													   *
 * Overview : Writes the given byte to the shadow RAM of the DS2781 at the 	   *
 * 			  given memory address											   *
 * Return type : Void														   *
 * Parameters : Byte to be written, address of register						   *
 * Time : < 3.6ms															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void WriteRAM (BYTE byte, BYTE addr)
{
	if( OWReset() == TRUE)
	{		
		OWWriteByte( SKIP_NETADDRESS );
		OWWriteByte( WRITE_DATA );
		OWWriteByte( addr );						//Register Address
		OWWriteByte( byte );
	}
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : CopyEEPROM												   *
 * Overview : This function copies the contents of the EEPROM shadow RAM to    *
 * 			  EEPROM cells for the EEPROM block containing thr given address.  *
 *			  Writes to EEPROM addresses are ignored for up to 15ms after this *
 *			  function is called.											   *
 * Return type : Void														   *
 * Parameters : Memory address of shadow RAM to be copied					   *
 * Time : < 2.9ms															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void CopyEEPROM (BYTE addr)
{
	if( OWReset() == TRUE)
	{		
		OWWriteByte( SKIP_NETADDRESS );
		OWWriteByte( COPY_DATA );
		OWWriteByte( addr );
	}
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : RecallEEPROM												   *
 * Overview : This function copies the contents of the EEPROM cells to the     *
 * 			  shadow RAM for the EEPROM block containing the given address.    *										   *
 * Return type : Void														   *
 * Parameters : Memory address of EEPROM to be copied						   *
 * Time : < 2.9ms															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void RecallEEPROM (BYTE addr)
{
	if( OWReset() == TRUE)
	{		
		OWWriteByte( SKIP_NETADDRESS );
		OWWriteByte( RECALL_DATA );
		OWWriteByte( addr );
	}
}

/*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*
 * Function name : Delay100MSx												   *
 * Overview : Delays in units of 100ms as supplied by the parameter. To use	   *
 * 			  the funtion at a clock speed of 40MHz #define __FREQ40MHz__ in   *
 *			  the header file. For other frequencies change the definitions    *
 * Return type : Void														   *
 * Parameters : Number of 100ms delays										   *
 * Time : Varies															   *
 *-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*/

void Delay100MSx(BYTE counts)
{
	BYTE i,j;
	for(i = 0; i < counts; i++)
	{
        for(j=0;j<10;j++)
        {
            __delay_ms(1);
        }			
	}
}


/* EOF */
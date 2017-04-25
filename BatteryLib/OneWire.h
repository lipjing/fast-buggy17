#define _XTAL_FREQ 10000000 //Setting HS(osc) as 10MHz-used in delays etc

#include <xc.h>
#include "GenericTypeDefs.h"

/* ***** general definitions ***** */
#define INPUT 1
#define OUTPUT 0


/* ***** 1-Wire data pin ***** */
#define WIRE_TRIS 	TRISHbits.TRISH3
#define WIRE 		LATHbits.LATH3
#define WIRE_READ   PORTHbits.RH3


/* ***** function prototypes ***** */
/* Function details can be found in the .c file */
BOOL OWReset (void);
void OWWriteBit (BYTE);
BYTE OWReadBit (void);
void OWWriteByte (BYTE byte);
BYTE OWReadByte (void);


#define Delay_A()	__delay_us(5);
#define Delay_B()	__delay_us(64);
#define Delay_C()	__delay_us(60);
#define Delay_D()	__delay_us(10);

#define Delay_E()	__delay_us(10);
#define Delay_F()	__delay_us(55);

//Delays Used for OWReset (Confirmed Working)
#define Delay_H()	__delay_us(480);
#define Delay_I()	__delay_us(68);
#define Delay_J()	__delay_us(405);



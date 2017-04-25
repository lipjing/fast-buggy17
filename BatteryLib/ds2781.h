#include "OneWire.h"



/* ***** net address commands ***** */
#define READ_NETADDRESS 0x33
#define SKIP_NETADDRESS 0xCC


/* ***** function commands ***** */
#define READ_DATA 0x69
#define WRITE_DATA 0x6C
#define	COPY_DATA 0x48
#define	RECALL_DATA 0xB8
#define LOCK_EEPROM 0x6A	//DO NOT USE


/* ***** function prototypes ***** */
/* Function details can be found in the .c file */
WORD ReadVoltage (void);
WORD ReadCurrent (void);
DWORD ReadAccumulatedCurrent (void);
void ResetAccumulatedCurrent (void);
QWORD ReadNetAddress (void);
WORD ReadTemperature (void);
BYTE ReadCurrentOffset (void);
void  WriteCurrentOffset (CHAR offset);
void AdjustCurrentOffset (void);
void UpdateControlRegister (BYTE control);
BYTE ReadRAM (BYTE addr);
void WriteRAM (BYTE byte, BYTE addr);
void CopyEEPROM (BYTE addr);
void RecallEEPROM (BYTE addr);
void Delay100MSx(BYTE counts);


#define Delay_100MS()	__delay_ms(100);


#include "xc.h"
#include "timers.h"
#include "delays.h"

volatile int a;

void interrupt isr(){
    if (INTCONbits.RBIF == 1){
      INTCONbits.RBIE = 0;
      if (PORTBbits.RB4 ==1){
          TMR1ON = 1;}
      if(PORTBbits.RB4 == 0){
          TMR1ON = 0;
          a = (TMR1L | (TMR1H<<8))/73.53;
          
      } 
    }
    INTCONbits.RBIF = 0;
    INTCONbits.RBIE = 1;
}




void main (void){
  TRISBbits.RB0 = 0x00;  
  TRISBbits.RB4 = 0x01;
  //TRISF = 0x00;
  //TRISA = 0xEF;
  
  //LATAbits.LATA4 = 0;    
  //LATAbits.LATA4 = 1;
  INTCONbits.GIE = 1;
  //INTCONbits.PEIE=1;
  INTCONbits.RBIF = 0;
  INTCONbits.RBIE = 1;
  

  T1CON = 0x10; 
  
  //OpenTimer1 (TIMER_INT_OFF & 
            //T1_16BIT_RW &
            //T1_SOURCE_INT & 
            //T1_PS_1_2);
  
  while (1){
   TMR1L = TMR1H = 0;
   LATBbits.LATB0 = 1;
   Delay1TCYx(25);
   LATBbits.LATB0 = 0;
   
   
   Delay1KTCYx(250);
   //a = a + 1;
   
   
   if(a>=2 && a<=400){
       ADCON1=0x0F;
       TRISF = 0x00;
       TRISAbits.RA4 = 0;
       LATAbits.LATA4 = 1;
       LATF = a;
   }
   
   else{
       ADCON1=0x0F;
       TRISF = 0x00;
       TRISAbits.RA4 = 0;
       LATAbits.LATA4 = 1;
       LATF = 0x00;
   }
      
      
  Delay10KTCYx(100);    
  }
  
  
 
}

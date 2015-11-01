/*
 * File:		sLCD_freedom.c
 * Purpose:		Main process
 *
 */

#include "common.h"
#include "lcd.h"


#ifdef CMSIS
#include "start.h"
#endif



char chBufDisp[10];

/********************************************************************/
int main (void)
{
    	int  delay;
        int  count;
          
        
#ifdef CMSIS  // If we are conforming to CMSIS, we need to call start here
    start();
#endif
        
  	printf("\n\rRunning the sLCD_freedom project.\n\r");
        vfnLCD_Init();
        vfnLCD_Write_Msg("1234");
        

	while(1)
	{
                count++;
                if (count>9999) count=0;
                for (delay=0; delay<500000; delay++);
                sprintf(chBufDisp,"%04i",count);
                vfnLCD_Write_Msg((uint8 *)chBufDisp);
                
                
	} 
}
/********************************************************************/

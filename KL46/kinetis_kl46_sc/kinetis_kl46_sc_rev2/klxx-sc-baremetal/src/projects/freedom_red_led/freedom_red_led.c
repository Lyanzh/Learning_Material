/*
 * File:		freedom_red_led.c
 * Purpose:		Main process
 *
 */

#include "common.h"
#include "gpio.h"


#ifdef CMSIS
#include "start.h"
#endif


/********************************************************************/
int main (void)
{
    	int32 delay;
        
#ifdef CMSIS  // If we are conforming to CMSIS, we need to call start here
    start();
#endif
        
        gpio_init();
        LED1_ON;   // Green LED
        LED1_OFF;
        
        LED2_ON;   // Red LED
        LED2_OFF;

        
        
        
  	printf("\n\rRunning the freedom_red_led project.\n\r");

	while(1)
	{
	  LED2_TOGGLE;
          for (delay=0; delay<1000000; delay++);
          
	} 
}
/********************************************************************/

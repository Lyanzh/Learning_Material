/* ###################################################################
**     Filename    : ProcessorExpert.c
**     Project     : ProcessorExpert
**     Processor   : MKL46Z256VMC4
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2013-07-24, 17:43, # CodeGen: 0
**     Abstract    :
**         Main module.   TWRPI-sLCD + TWR-KL46Z48M demo code
**                   sLCD configured by by PEx,
**                   codification on bare metal code   
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file ProcessorExpert.c
** @version 01.01
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup ProcessorExpert_module ProcessorExpert module documentation
**  @{
*/         
/* MODULE ProcessorExpert */


/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "Events.h"
#include "CsIO1.h"
#include "IO1.h"
#include "SegLCD1.h"
#include "LED_GREEN.h"
#include "BitIoLdd1.h"
#include "LED_RED.h"
#include "BitIoLdd2.h"
#include "LED_YELLOW.h"
#include "BitIoLdd3.h"
#include "LED_ORANGE.h"
#include "BitIoLdd4.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "LCD.h"

#define CW
/* variables */
LDD_TError Error;
LDD_TDeviceData *MySegLCDPtr;

char InpData[16];
char sLCDBuffer[16];  //temporal buffer for the sLCD,
int main_counter;

/* User includes (#include below this line is not maintained by Processor Expert) */

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */


	  
  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/
  LED_GREEN_NegVal();
  LED_RED_NegVal();
  LED_YELLOW_NegVal();
  LED_ORANGE_NegVal();
  
  MySegLCDPtr = SegLCD1_Init(NULL);  //initialize sLCD according to PEx 
  vfnLCD_Write_Msg("1234");
  
  /*Turn on all special symbols*/
  _FREESCALE_ON();   
  _ONE_ON();         
  _COL_ON();         
  _GRADE_ON();       
  _PERCENTAGE_ON();  
  _AM_ON();          
  _PM_ON();          
         
   for (main_counter=1000000 ; main_counter>0 ;main_counter--){}; //delay to show all symbol ON
  
   /*Turn off all special symbols*/
  _FREESCALE_OFF();   
  _ONE_OFF();         
  _COL_OFF();         
  _GRADE_OFF();       
  _PERCENTAGE_OFF();  
  _AM_OFF();          
  _PM_OFF();          
   	
#ifdef CW
  // If we are using CodeWarrior, we must call this function to change
  // the buffer behavior such that a new line character, "\n", is not 
  // required to print characters to the terminal.  
  setvbuf(stdout, NULL, _IONBF, 0);
#endif
  
  printf("\n\r KL46 sLCD test \n");
  printf("\n\r press a=all symbols on, 0=all symbols Off,  1 to 6 symbols");
  _FREESCALE_ON();   
  for(;;) 
   {
	  /*echoes all received character to terminal */
	  InpData[0] = getchar();
      printf("%c",InpData[0]);
      main_counter++;
      sprintf(sLCDBuffer,"%04i",main_counter%1000);
      vfnLCD_Write_Msg(sLCDBuffer);
      
      LED_GREEN_NegVal();
      LED_RED_NegVal();
      LED_YELLOW_NegVal();
      LED_ORANGE_NegVal();
      
      switch ( InpData[0])
      {
      case 'a': 
    	  _FREESCALE_ON();   
    	  _ONE_ON();         
    	  _COL_ON();         
    	  _GRADE_ON();       
    	  _PERCENTAGE_ON();  
    	  _AM_ON();          
    	  _PM_ON();  
      break;	  

      case '0': 
    	  _FREESCALE_OFF();   
    	  _ONE_OFF();         
    	  _COL_OFF();         
    	  _GRADE_OFF();       
    	  _PERCENTAGE_OFF();  
    	  _AM_OFF();          
    	  _PM_OFF();   
      break;	  
      
      case '1':
    	 _FREESCALE_ON();
    	 break;

      case '2':
    	  _COL_ON();         
    	 break;

      case '3':
    	 _GRADE_ON();       
    	 break;

      case '4':
    	 _PERCENTAGE_ON();
    	 break;

      case '5':
    	 _AM_ON();    
    	 break;
    	 
      case '6':
    	 _PM_ON();    
      break;
      

      default:
    	  _FREESCALE_ON();   
    	  _ONE_OFF();         
    	  _COL_OFF();         
    	  _GRADE_OFF();       
    	  _PERCENTAGE_OFF();  
    	  _AM_OFF();          
    	  _PM_OFF();
      }
   }
  
  
  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END ProcessorExpert */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.2 [05.06]
**     for the Freescale Kinetis series of microcontrollers.
**
** ###################################################################
*/

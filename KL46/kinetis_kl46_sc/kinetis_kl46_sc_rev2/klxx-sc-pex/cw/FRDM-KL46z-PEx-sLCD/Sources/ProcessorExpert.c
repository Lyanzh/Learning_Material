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

  
  MySegLCDPtr = SegLCD1_Init(NULL);  //initialize sLCD according to PEx 
  
  vfnLCD_Write_Msg((uint8 *)"8888");
  
  _LCD_DP1_ON();
  _LCD_DP2_ON();
  _LCD_DP3_ON();
  _LCD_COL_ON();

     
         
   for (main_counter=1000000 ; main_counter>0 ;main_counter--){}; //delay to show all symbol ON

     _LCD_DP1_OFF();
     _LCD_DP2_OFF();
     _LCD_DP3_OFF();
     _LCD_COL_OFF();

   
#ifdef CW
  // If we are using CodeWarrior, we must call this function to change
  // the buffer behavior such that a new line character, "\n", is not 
  // required to print characters to the terminal.  
  setvbuf(stdout, NULL, _IONBF, 0);
#endif
  
  printf("\n\r KL46 sLCD FRDM-KL46 test \n");
  printf("\n\r press a=all symbols on, 0=all symbols Off,  1 to 5 symbols");
  
  for(;;) 
   {
	  /*echoes all received character to terminal */
	  InpData[0] = getchar();
      printf("%c",InpData[0]);
      main_counter++;
      sprintf(sLCDBuffer,"%04i",main_counter%1000);
      vfnLCD_Write_Msg((uint8 *)sLCDBuffer);
      
      LED_GREEN_NegVal();
      LED_RED_NegVal();
    
      switch (InpData[0])
      {      
      case 'a':        	 
    	  _LCD_DP1_ON();
    	  _LCD_DP2_ON();
    	  _LCD_DP3_ON();
    	  _LCD_COL_ON();     	  
      break;
      
      case '0':        	 
          	  _LCD_DP1_OFF();
          	  _LCD_DP2_OFF();
          	  _LCD_DP3_OFF();
          	  _LCD_COL_OFF();     	  
            break;
      
        case '1':
    	  _LCD_DP1_ON();
    	  break;
    	  
        case '2':
      	  _LCD_DP2_ON();
      	  break;
    
        case '3':
      	  _LCD_DP3_ON();
      	  break;
    
        case '4':
      	  _LCD_COL_ON();
      	  break;
    
      	  
        default: 
    	  
        	  _LCD_DP1_OFF();
        	  _LCD_DP2_OFF();
        	  _LCD_DP3_OFF();
        	  _LCD_COL_OFF(); 
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

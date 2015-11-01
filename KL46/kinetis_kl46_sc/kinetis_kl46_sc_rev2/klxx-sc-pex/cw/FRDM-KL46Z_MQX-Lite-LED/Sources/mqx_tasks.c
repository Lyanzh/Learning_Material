/* ###################################################################
**     Filename    : mqx_tasks.c
**     Project     : ProcessorExpert
**     Processor   : MKL46Z256VMC4
**     Component   : Events
**     Version     : Driver 01.00
**     Compiler    : GNU C Compiler
**     Date/Time   : 2013-07-26, 09:24, # CodeGen: 0
**     Abstract    :
**         This is user's event module.
**         Put your event handler code here.
**     Settings    :
**     Contents    :
**         RED_LED_task - void RED_LED_task(uint32_t task_init_data);
**
** ###################################################################*/
/*!
** @file mqx_tasks.c
** @version 01.00
** @brief
**         This is user's event module.
**         Put your event handler code here.
*/         
/*!
**  @addtogroup mqx_tasks_module mqx_tasks module documentation
**  @{
*/         
/* MODULE mqx_tasks */

#include "Cpu.h"
#include "Events.h"
#include "mqx_tasks.h"

#ifdef __cplusplus
extern "C" {
#endif 


/* User includes (#include below this line is not maintained by Processor Expert) */

/*
** ===================================================================
**     Event       :  RED_LED_task (module mqx_tasks)
**
**     Component   :  Task1 [MQXLite_task]
**     Description :
**         MQX task routine. The routine is generated into mqx_tasks.c
**         file.
**     Parameters  :
**         NAME            - DESCRIPTION
**         task_init_data  - 
**     Returns     : Nothing
** ===================================================================
*/
void RED_LED_task(uint32_t task_init_data)
{
  int counter = 0;
  _task_id    task_id;
  
  printf("Hello World!\r\n");
  
  /* Requires to enable C99 extensions */
  printf(" - %s is running\n", __func__);
  /* Create YEL_LED_task with higher priority and switch context to it */
  printf(" - Creating GREEN_LED_task from RED_LED_task\n");
  task_id = _task_create_at(0, LED_TASK2_TASK, 0, LED_Task2_task_stack, LED_TASK2_TASK_STACK_SIZE);
  if (task_id == MQX_NULL_TASK_ID) {
	printf("...failed\n");
  }

  while(1) {
    counter++;
    LED2_NegVal(NULL);
    _time_delay_ticks(10);
    _sched_yield();
  }
}

/*
** ===================================================================
**     Event       :  GREEN_LED_task (module mqx_tasks)
**
**     Component   :  Task2 [MQXLite_task]
**     Description :
**         MQX task routine. The routine is generated into mqx_tasks.c
**         file.
**     Parameters  :
**         NAME            - DESCRIPTION
**         task_init_data  - 
**     Returns     : Nothing
** ===================================================================
*/
void GREEN_LED_task(uint32_t task_init_data)
{
  int counter = 0;
  /* Requires to enable C99 extensions */
  printf(" - %s is running\n", __func__);
  while(1) {
    counter++;
    LED1_NegVal(NULL);
    _time_delay_ticks(10);
    _sched_yield();
  }
}

/*
** ===================================================================
**     Event       :  SW1_task (module mqx_tasks)
**
**     Component   :  Task3 [MQXLite_task]
**     Description :
**         MQX task routine. The routine is generated into mqx_tasks.c
**         file.
**     Parameters  :
**         NAME            - DESCRIPTION
**         task_init_data  - 
**     Returns     : Nothing
** ===================================================================
*/
void SW1_task(uint32_t task_init_data)
{
  int counter = 0;
  int prss = 0;
  /* Requires to enable C99 extensions */
  printf(" - %s is running\n", __func__);
  while(1) {
	counter++;
	if(!SW1_GetVal(NULL))
	{
		prss = 1;
	}
	if(SW1_GetVal(NULL) && prss)
	{
		printf("SW1 was pressed\n");
		prss = 0;
	}
	_sched_yield();
  }
}

/*
** ===================================================================
**     Event       :  SW2_task (module mqx_tasks)
**
**     Component   :  Task4 [MQXLite_task]
**     Description :
**         MQX task routine. The routine is generated into mqx_tasks.c
**         file.
**     Parameters  :
**         NAME            - DESCRIPTION
**         task_init_data  - 
**     Returns     : Nothing
** ===================================================================
*/
void SW2_task(uint32_t task_init_data)
{
	int counter = 0;
	int prss = 0;
	/* Requires to enable C99 extensions */
	printf(" - %s is running\n", __func__);
	while(1) {
	counter++;
	if(!SW2_GetVal(NULL))
	{
		prss = 1;
	}
	if(SW2_GetVal(NULL) && prss)
	{
		printf("SW2 was pressed\n");
		prss = 0;
	}
	_sched_yield();
	}
}

/* END mqx_tasks */

#ifdef __cplusplus
}  /* extern "C" */
#endif 

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

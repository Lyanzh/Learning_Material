/* ###################################################################
**     THIS COMPONENT MODULE IS GENERATED BY THE TOOL. DO NOT MODIFY IT.
**     Filename    : Bit1.c
**     Project     : ProcessorExpert
**     Processor   : MKL46Z256VMC4
**     Component   : BitIO_LDD
**     Version     : Component 01.033, Driver 01.00, CPU db: 3.00.000
**     Compiler    : GNU C Compiler
**     Date/Time   : 2013-07-26, 06:07, # CodeGen: 17
**     Abstract    :
**         The HAL BitIO component provides a low level API for unified
**         access to general purpose digital input/output pins across
**         various device designs.
**
**         RTOS drivers using HAL BitIO API are simpler and more
**         portable to various microprocessors.
**     Settings    :
**          Component name                                 : Bit1
**          Pin for I/O                                    : PTA17/SPI0_MISO/SPI0_MOSI/I2S0_MCLK
**          Pin signal                                     : 
**          Direction                                      : Input/Output
**          Initialization                                 : 
**            Init. direction                              : Output
**            Init. value                                  : 0
**            Auto initialization                          : no
**          Safe mode                                      : no
**     Contents    :
**         Init     - LDD_TDeviceData* Bit1_Init(LDD_TUserData *UserDataPtr);
**         SetDir   - void Bit1_SetDir(LDD_TDeviceData *DeviceDataPtr, bool Dir);
**         SetInput - void Bit1_SetInput(LDD_TDeviceData *DeviceDataPtr);
**         GetVal   - bool Bit1_GetVal(LDD_TDeviceData *DeviceDataPtr);
**         PutVal   - void Bit1_PutVal(LDD_TDeviceData *DeviceDataPtr, bool Val);
**         ClrVal   - void Bit1_ClrVal(LDD_TDeviceData *DeviceDataPtr);
**         SetVal   - void Bit1_SetVal(LDD_TDeviceData *DeviceDataPtr);
**         NegVal   - void Bit1_NegVal(LDD_TDeviceData *DeviceDataPtr);
**
**     Copyright : 1997 - 2013 Freescale Semiconductor, Inc. All Rights Reserved.
**     SOURCE DISTRIBUTION PERMISSIBLE as directed in End User License Agreement.
**     
**     http      : www.freescale.com
**     mail      : support@freescale.com
** ###################################################################*/
/*!
** @file Bit1.c
** @version 01.00
** @brief
**         The HAL BitIO component provides a low level API for unified
**         access to general purpose digital input/output pins across
**         various device designs.
**
**         RTOS drivers using HAL BitIO API are simpler and more
**         portable to various microprocessors.
*/         
/*!
**  @addtogroup Bit1_module Bit1 module documentation
**  @{
*/         

/* MODULE Bit1. */

/* {Default RTOS Adapter} No RTOS includes */
#include "Bit1.h"

#ifdef __cplusplus
extern "C" {
#endif 

typedef struct {
  LDD_TUserData *UserDataPtr;          /* Pointer to user data */
} Bit1_TDeviceData;                    /* Device data structure type */

typedef Bit1_TDeviceData *Bit1_TDeviceDataPtr ; /* Pointer to the device data structure. */

/* {Default RTOS Adapter} Static object used for simulation of dynamic driver memory allocation */
static Bit1_TDeviceData DeviceDataPrv__DEFAULT_RTOS_ALLOC;
/*
** ===================================================================
**     Method      :  Bit1_Init (component BitIO_LDD)
*/
/*!
**     @brief
**         Initializes the device. Allocates memory for the device data
**         structure, allocates interrupt vectors and sets interrupt
**         priority, sets pin routing, sets timing, etc. If the "Enable
**         in init. code" is set to "yes" value then the device is also
**         enabled(see the description of the Enable() method). In this
**         case the Enable() method is not necessary and needn't to be
**         generated. 
**     @param
**         UserDataPtr     - Pointer to the user or
**                           RTOS specific data. This pointer will be
**                           passed as an event or callback parameter.
**     @return
**                         - Pointer to the dynamically allocated private
**                           structure or NULL if there was an error.
*/
/* ===================================================================*/
LDD_TDeviceData* Bit1_Init(LDD_TUserData *UserDataPtr)
{
  /* Allocate device structure */
  Bit1_TDeviceDataPtr DeviceDataPrv;
  /* {Default RTOS Adapter} Driver memory allocation: Dynamic allocation is simulated by a pointer to the static object */
  DeviceDataPrv = &DeviceDataPrv__DEFAULT_RTOS_ALLOC;

  DeviceDataPrv->UserDataPtr = UserDataPtr; /* Store the RTOS device structure */

  /* Configure pin as output */
  /* GPIOA_PDDR: PDD|=0x00020000 */
  GPIOA_PDDR |= GPIO_PDDR_PDD(0x00020000);                                   

  /* Set initialization value */
  /* GPIOA_PDOR: PDO&=~0x00020000 */
  GPIOA_PDOR &= (uint32_t)~(uint32_t)(GPIO_PDOR_PDO(0x00020000));                                   

  /* Initialization of Port Control register */
  /* PORTA_PCR17: ISF=0,MUX=1 */
  PORTA_PCR17 = (uint32_t)((PORTA_PCR17 & (uint32_t)~(uint32_t)(
                 PORT_PCR_ISF_MASK |
                 PORT_PCR_MUX(0x06)
                )) | (uint32_t)(
                 PORT_PCR_MUX(0x01)
                ));                                  
  /* Registration of the device structure */
  PE_LDD_RegisterDeviceStructure(PE_LDD_COMPONENT_Bit1_ID,DeviceDataPrv);
  return ((LDD_TDeviceData *)DeviceDataPrv);
}
/*
** ===================================================================
**     Method      :  Bit1_SetDir (component BitIO_LDD)
*/
/*!
**     @brief
**         Sets a pin direction (available only if the direction =
**         _[input/output]_).
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by <Init> method.
**     @param
**         Dir             - Direction to set. Possible values:
**                           <false> - Input
**                           <true> - Output
*/
/* ===================================================================*/
void Bit1_SetDir(LDD_TDeviceData *DeviceDataPtr, bool Dir)
{
  (void)DeviceDataPtr;                 /* Parameter is not used, suppress unused argument warning */

  if (Dir) {
    /* Output */
    GPIO_PDD_SetPortOutputDirectionMask(Bit1_MODULE_BASE_ADDRESS, Bit1_PORT_MASK);
  } else {
    /* Input */
    GPIO_PDD_SetPortInputDirectionMask(Bit1_MODULE_BASE_ADDRESS, Bit1_PORT_MASK);
  }
}

/*
** ===================================================================
**     Method      :  Bit1_SetInput (component BitIO_LDD)
*/
/*!
**     @brief
**         Sets a pin direction to input (available only if the
**         direction = _[input/output]_).
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by <Init> method.
*/
/* ===================================================================*/
void Bit1_SetInput(LDD_TDeviceData *DeviceDataPtr)
{
  (void)DeviceDataPtr;                 /* Parameter is not used, suppress unused argument warning */

  GPIO_PDD_SetPortInputDirectionMask(Bit1_MODULE_BASE_ADDRESS, Bit1_PORT_MASK);
}

/*
** ===================================================================
**     Method      :  Bit1_GetVal (component BitIO_LDD)
*/
/*!
**     @brief
**         Returns the input/output value. If the direction is [input]
**         then the input value of the pin is read and returned. If the
**         direction is [output] then the last written value is read
**         and returned (see <Safe mode> property for limitations).
**         This method cannot be disabled if direction is [input].
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by <Init> method.
**     @return
**                         - Input or output value. Possible values:
**                           <false> - logical "0" (Low level)
**                           <true> - logical "1" (High level)
*/
/* ===================================================================*/
bool Bit1_GetVal(LDD_TDeviceData *DeviceDataPtr)
{
  uint32_t portData;                   /* Port data masked according to the bit used */

  (void)DeviceDataPtr;                 /* Parameter is not used, suppress unused argument warning */

  if ((GPIO_PDD_GetPortDirection(Bit1_MODULE_BASE_ADDRESS) & Bit1_PORT_MASK) == 0U) {
    /* port is configured as input */
    portData = GPIO_PDD_GetPortDataInput(Bit1_MODULE_BASE_ADDRESS) & Bit1_PORT_MASK;
  } else {
    /* port is configured as output */
    portData = GPIO_PDD_GetPortDataOutput(Bit1_MODULE_BASE_ADDRESS) & Bit1_PORT_MASK;
  }

  return (portData != 0U) ? (bool)TRUE : (bool)FALSE;
}

/*
** ===================================================================
**     Method      :  Bit1_PutVal (component BitIO_LDD)
*/
/*!
**     @brief
**         The specified output value is set. If the direction is <b>
**         input</b>, the component saves the value to a memory or a
**         register and this value will be written to the pin after
**         switching to the output mode (using <tt>SetDir(TRUE)</tt>;
**         see <a href="BitIOProperties.html#SafeMode">Safe mode</a>
**         property for limitations). If the direction is <b>output</b>,
**         it writes the value to the pin. (Method is available only if
**         the direction = <u><tt>output</tt></u> or <u><tt>
**         input/output</tt></u>).
**     @param
**         DeviceDataPtr   - Device data structure
**                           pointer returned by <Init> method.
**     @param
**         Val             - Output value. Possible values:
**                           <false> - logical "0" (Low level)
**                           <true> - logical "1" (High level)
*/
/* ===================================================================*/
void Bit1_PutVal(LDD_TDeviceData *DeviceDataPtr, bool Val)
{
  (void)DeviceDataPtr;                 /* Parameter is not used, suppress unused argument warning */

  if (Val) {
    GPIO_PDD_SetPortDataOutputMask(Bit1_MODULE_BASE_ADDRESS, Bit1_PORT_MASK);
  } else { /* !Val */
    GPIO_PDD_ClearPortDataOutputMask(Bit1_MODULE_BASE_ADDRESS, Bit1_PORT_MASK);
  } /* !Val */
}

/*
** ===================================================================
**     Method      :  Bit1_ClrVal (component BitIO_LDD)
*/
/*!
**     @brief
**         Clears (set to zero) the output value. It is equivalent to
**         the [PutVal(FALSE)]. This method is available only if the
**         direction = _[output]_ or _[input/output]_.
**     @param
**         DeviceDataPtr   - Pointer to device data
**                           structure returned by <Init> method.
*/
/* ===================================================================*/
void Bit1_ClrVal(LDD_TDeviceData *DeviceDataPtr)
{
  (void)DeviceDataPtr;                 /* Parameter is not used, suppress unused argument warning */

  GPIO_PDD_ClearPortDataOutputMask(Bit1_MODULE_BASE_ADDRESS, Bit1_PORT_MASK);
}

/*
** ===================================================================
**     Method      :  Bit1_SetVal (component BitIO_LDD)
*/
/*!
**     @brief
**         Sets (to one) the output value. It is equivalent to the
**         [PutVal(TRUE)]. This method is available only if the
**         direction = _[output]_ or _[input/output]_.
**     @param
**         DeviceDataPtr   - Pointer to device data
**                           structure returned by <Init> method.
*/
/* ===================================================================*/
void Bit1_SetVal(LDD_TDeviceData *DeviceDataPtr)
{
  (void)DeviceDataPtr;                 /* Parameter is not used, suppress unused argument warning */

  GPIO_PDD_SetPortDataOutputMask(Bit1_MODULE_BASE_ADDRESS, Bit1_PORT_MASK);
}

/*
** ===================================================================
**     Method      :  Bit1_NegVal (component BitIO_LDD)
*/
/*!
**     @brief
**         Negates (inverts) the output value. It is equivalent to the
**         [PutVal(!GetVal())]. This method is available only if the
**         direction = _[output]_ or _[input/output]_.
**     @param
**         DeviceDataPtr   - Pointer to device data
**                           structure returned by <Init> method.
*/
/* ===================================================================*/
void Bit1_NegVal(LDD_TDeviceData *DeviceDataPtr)
{
  (void)DeviceDataPtr;                 /* Parameter is not used, suppress unused argument warning */

  GPIO_PDD_TogglePortDataOutputMask(Bit1_MODULE_BASE_ADDRESS, Bit1_PORT_MASK);
}

/* END Bit1. */

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

/*
*********************************************************************************************************
*                                     MICIRUM BOARD SUPPORT PACKAGE
*
*                             (c) Copyright 2013; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                             I2C Driver
*
*                                        Freescale Kinetis KL40
*                                               on the
*                                        Freescale FRDM-KL46Z
*                                          Evaluation Board
*
* Filename      : bsp_i2c.h
* Version       : V1.00
* Programmer(s) : LYZ
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                 MODULE
*
* Note(s) : (1) This header file is protected from multiple pre-processor inclusion through use of the
*               BSP present pre-processor macro definition.
*********************************************************************************************************
*/

#ifndef  BSP_I2C_PRESENT
#define  BSP_I2C_PRESENT


/*
*********************************************************************************************************
*                                                 EXTERNS
*********************************************************************************************************
*/

#ifdef   BSP_I2C_MODULE
#define  BSP_I2C_EXT
#else
#define  BSP_I2C_EXT  extern
#endif

#if 0
/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/

#include  <bsp.h>


/*
*********************************************************************************************************
*                                         LCD CTL OPTION DEFINES
*********************************************************************************************************
*/

#define  BSP_LCD_OPT_NONE       DEF_NULL
#define  BSP_LCD_OPT_DP1        DEF_BIT_01
#define  BSP_LCD_OPT_DP2        DEF_BIT_02
#define  BSP_LCD_OPT_DP3        DEF_BIT_03
#define  BSP_LCD_OPT_COL        DEF_BIT_07


/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void         BSP_I2C_Init   (void);

CPU_BOOLEAN  BSP_I2C_Write  (CPU_INT08U  dig1,
                             CPU_INT08U  dig2,
                             CPU_INT08U  dig3,
                             CPU_INT08U  dig4,
                             CPU_INT08U  opt);

void        BSP_LCD_Clear         (void);


#endif

#endif                                                          /* End of module include.                               */

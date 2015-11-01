/*
*********************************************************************************************************
*                                     MICIRUM BOARD SUPPORT PACKAGE
*
*                             (c) Copyright 2014; Micrium, Inc.; Weston, FL
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
*                                                UART
*                                          UART Panel Driver
*
*                                        Freescale Kinetis KL40
*                                               on the
*                                        Freescale FRDM-KL46Z
*                                          Evaluation Board
*
* Filename      : bsp_uart.c
* Version       : V1.00
* Programmer(s) : Lin Yanzhou
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  BSP_UART_MODULE
#include  <bsp_uart.h>
#include  <MKL46Z4.h>


/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/
                                                                /* -------------------- UART PINS --------------------- */
#define  BSP_UART0_TXD                  PORTA_PCR14
#define  BSP_UART0_RXD                  PORTA_PCR15
#define  BSP_UART1_TXD                  PORTE_PCR0
#define  BSP_UART1_RXD                  PORTE_PCR1
#define  BSP_UART2_TXD                  PORTE_PCR22
#define  BSP_UART3_RXD                  PORTE_PCR23
                                                                /* --------------------- LCD PINS --------------------- */

                                                                /* Mappings from SLCD module pins to LCD panel inputs   */
#define  BSP_LCD_PIN_01                            40u          /* COM 0.                                               */
#define  BSP_LCD_PIN_02                            52u          /* COM 1.                                               */
#define  BSP_LCD_PIN_03                            19u          /* COM 2.                                               */
#define  BSP_LCD_PIN_04                            18u          /* COM 3.                                               */
#define  BSP_LCD_PIN_05                            37u
#define  BSP_LCD_PIN_06                            17u
#define  BSP_LCD_PIN_07                             7u
#define  BSP_LCD_PIN_08                             8u
#define  BSP_LCD_PIN_09                            53u
#define  BSP_LCD_PIN_10                            38u
#define  BSP_LCD_PIN_11                            10u
#define  BSP_LCD_PIN_12                            11u

                                                                /* Each digit is associated with two distinct pins.     */
                                                                /* DP and COL segments each share a pin with one digit. */
#define  BSP_LCD_DIG1_PIN0              BSP_LCD_PIN_05
#define  BSP_LCD_DIG1_PIN1              BSP_LCD_PIN_06
#define  BSP_LCD_DIG2_PIN0              BSP_LCD_PIN_07
#define  BSP_LCD_DIG2_PIN1              BSP_LCD_PIN_08
#define  BSP_LCD_DIG3_PIN0              BSP_LCD_PIN_09
#define  BSP_LCD_DIG3_PIN1              BSP_LCD_PIN_10
#define  BSP_LCD_DIG4_PIN0              BSP_LCD_PIN_11
#define  BSP_LCD_DIG4_PIN1              BSP_LCD_PIN_12

#define  BSP_LCD_DP1_PIN0               BSP_LCD_PIN_06
#define  BSP_LCD_DP2_PIN0               BSP_LCD_PIN_08
#define  BSP_LCD_DP3_PIN0               BSP_LCD_PIN_10
#define  BSP_LCD_COL_PIN0               BSP_LCD_PIN_12


                                                                /* ------------- WAVEFORM REGISTER DEFINES ------------ */

#define  BSP_LCD_WFR_BASE               0x40053020u
#define  BSP_LCD_WFR(x)                (*(CPU_REG32 *)(BSP_LCD_WFR_BASE + ((x) - ((x) % 4u))))

                                                                /* Write y to the byte in the WFR for x.                */
#define  BSP_LCD_WFR_WRT(x, y)         (BSP_LCD_WFR(x) = (BSP_LCD_WFR(x) & ~(0xFFu << (8u * ((x) % 4u)))) \
                                                                         |  ((y)   << (8u * ((x) % 4u))))

                                                                /*  Use bitfield y to set the bits in the WFR for x.    */
#define  BSP_LCD_WFR_SET(x, y)         (BSP_LCD_WFR(x) = (BSP_LCD_WFR(x) |  ((y)   << (8u * ((x) % 4u)))))


#define  BSP_LCD_WFR_BIT_A              DEF_BIT_00
#define  BSP_LCD_WFR_BIT_B              DEF_BIT_01
#define  BSP_LCD_WFR_BIT_C              DEF_BIT_02
#define  BSP_LCD_WFR_BIT_D              DEF_BIT_03
#define  BSP_LCD_WFR_BIT_E              DEF_BIT_04
#define  BSP_LCD_WFR_BIT_F              DEF_BIT_05
#define  BSP_LCD_WFR_BIT_G              DEF_BIT_06
#define  BSP_LCD_WFR_BIT_H              DEF_BIT_07

                                                                /* ---------------- LCD PANEL SEGMENTS ---------------- */

                                                                /* Map from WFR segments to physical segments.          */
#define  BSP_LCD_SEG_A                  BSP_LCD_WFR_BIT_D
#define  BSP_LCD_SEG_B                  BSP_LCD_WFR_BIT_C
#define  BSP_LCD_SEG_C                  BSP_LCD_WFR_BIT_B
#define  BSP_LCD_SEG_D                  BSP_LCD_WFR_BIT_A
#define  BSP_LCD_SEG_E                  BSP_LCD_WFR_BIT_B
#define  BSP_LCD_SEG_F                  BSP_LCD_WFR_BIT_D
#define  BSP_LCD_SEG_G                  BSP_LCD_WFR_BIT_C
#define  BSP_LCD_SEG_OPT                BSP_LCD_WFR_BIT_A
#define  BSP_LCD_SEG_NONE               DEF_NULL


                                                                /* -------------------- BIT MASKS --------------------- */

                                                                /* Used for validating arguments to BSP_LCD_Write().    */
#define  BSP_LCD_DIG_MASK           (0xFu)

#define  BSP_LCD_OPT_MASK           (BSP_LCD_OPT_NONE | \
                                     BSP_LCD_OPT_DP1  | \
                                     BSP_LCD_OPT_DP2  | \
                                     BSP_LCD_OPT_DP3  | \
                                     BSP_LCD_OPT_COL)


/*
*********************************************************************************************************
*                                           LOCAL CONSTANTS
*********************************************************************************************************
*/

                                                                /* WFR values to display Hex Digits.                    */
#define  BSP_LCD_WFRL_0                (BSP_LCD_SEG_D | BSP_LCD_SEG_E | BSP_LCD_SEG_F)
#define  BSP_LCD_WFRH_0                (BSP_LCD_SEG_A | BSP_LCD_SEG_B | BSP_LCD_SEG_C)

#define  BSP_LCD_WFRL_1                (BSP_LCD_SEG_NONE)
#define  BSP_LCD_WFRH_1                (BSP_LCD_SEG_B | BSP_LCD_SEG_C)

#define  BSP_LCD_WFRL_2                (BSP_LCD_SEG_D | BSP_LCD_SEG_E | BSP_LCD_SEG_G)
#define  BSP_LCD_WFRH_2                (BSP_LCD_SEG_A | BSP_LCD_SEG_B)

#define  BSP_LCD_WFRL_3                (BSP_LCD_SEG_D | BSP_LCD_SEG_G)
#define  BSP_LCD_WFRH_3                (BSP_LCD_SEG_A | BSP_LCD_SEG_B | BSP_LCD_SEG_C)

#define  BSP_LCD_WFRL_4                (BSP_LCD_SEG_F | BSP_LCD_SEG_G)
#define  BSP_LCD_WFRH_4                (BSP_LCD_SEG_B | BSP_LCD_SEG_C)

#define  BSP_LCD_WFRL_5                (BSP_LCD_SEG_D | BSP_LCD_SEG_F | BSP_LCD_SEG_G)
#define  BSP_LCD_WFRH_5                (BSP_LCD_SEG_A | BSP_LCD_SEG_C)

#define  BSP_LCD_WFRL_6                (BSP_LCD_SEG_D | BSP_LCD_SEG_E | BSP_LCD_SEG_F | BSP_LCD_SEG_G)
#define  BSP_LCD_WFRH_6                (BSP_LCD_SEG_A | BSP_LCD_SEG_C)

#define  BSP_LCD_WFRL_7                (BSP_LCD_SEG_F)
#define  BSP_LCD_WFRH_7                (BSP_LCD_SEG_A | BSP_LCD_SEG_B | BSP_LCD_SEG_C)

#define  BSP_LCD_WFRL_8                (BSP_LCD_SEG_D | BSP_LCD_SEG_E | BSP_LCD_SEG_F | BSP_LCD_SEG_G)
#define  BSP_LCD_WFRH_8                (BSP_LCD_SEG_A | BSP_LCD_SEG_B | BSP_LCD_SEG_C)

#define  BSP_LCD_WFRL_9                (BSP_LCD_SEG_D | BSP_LCD_SEG_F | BSP_LCD_SEG_G)
#define  BSP_LCD_WFRH_9                (BSP_LCD_SEG_A | BSP_LCD_SEG_B | BSP_LCD_SEG_C)

#define  BSP_LCD_WFRL_A                (BSP_LCD_SEG_E | BSP_LCD_SEG_F | BSP_LCD_SEG_G)
#define  BSP_LCD_WFRH_A                (BSP_LCD_SEG_A | BSP_LCD_SEG_B | BSP_LCD_SEG_C)

#define  BSP_LCD_WFRL_B                (BSP_LCD_SEG_D | BSP_LCD_SEG_E | BSP_LCD_SEG_F | BSP_LCD_SEG_G)
#define  BSP_LCD_WFRH_B                (BSP_LCD_SEG_C)

#define  BSP_LCD_WFRL_C                (BSP_LCD_SEG_D | BSP_LCD_SEG_E | BSP_LCD_SEG_F)
#define  BSP_LCD_WFRH_C                (BSP_LCD_SEG_A)

#define  BSP_LCD_WFRL_D                (BSP_LCD_SEG_D | BSP_LCD_SEG_E | BSP_LCD_SEG_G)
#define  BSP_LCD_WFRH_D                (BSP_LCD_SEG_B | BSP_LCD_SEG_C)

#define  BSP_LCD_WFRL_E                (BSP_LCD_SEG_D | BSP_LCD_SEG_E | BSP_LCD_SEG_F | BSP_LCD_SEG_G)
#define  BSP_LCD_WFRH_E                (BSP_LCD_SEG_A)

#define  BSP_LCD_WFRL_F                (BSP_LCD_SEG_E | BSP_LCD_SEG_F | BSP_LCD_SEG_G)
#define  BSP_LCD_WFRH_F                (BSP_LCD_SEG_A)


/*
*********************************************************************************************************
*                                          LOCAL DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            LOCAL TABLES
*********************************************************************************************************
*/

                                                                /* ------------- UART1,UART2 ADDRESS MAP -------------*/
static const  UART_MemMapPtr UART_ARR[]    = {UART1_BASE_PTR, UART2_BASE_PTR};

                                                                /* ----------------- UART IRQ TABLE ------------------*/
static        CPU_INT08U table_irq_uart[3] = {12, 13, 14};

/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                        MCGIRCLK_OutEnable()
*
* Description : Enable clock MCGIRCLK,which frequency is 31.25Khz ~ 4Mhz.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/
static void MCGIRCLK_OutEnable(void)
{
		MCG_C1 |= MCG_C1_IRCLKEN_MASK|MCG_C1_IREFSTEN_MASK;
		MCG_SC |= MCG_SC_FCRDIV(0x0);  //??4M??????????,
																	//???????31.25Khz ~ 4Mhz
		MCG_C4 |= MCG_C4_FCTRIM(0xA);
		MCG_C2 |= MCG_C2_IRCS_MASK;
		while (!(MCG_S & MCG_S_IRCST_MASK));
	}

/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                           BSP_UART_Init()
*
* Description : Initialize the UART module.
*
* Argument(s) : uartNo        UART channel : BSP_UART_0
	                                           BSP_UART_1
	                                           BSP_UART_2
*
*	              sel_clk       Clock of UART : BSP_MCGIRCLK  (4000Khz)
*                                             BSP_MCGPLL    (48000Khz)
*                                             BSP_BUSCLK    (24000khz)
*
*               baud_rate     Baud rate of UART : 
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/
void  BSP_UART_Init (CPU_INT08U uartNo,
	                   CPU_INT32U sel_clk,
                     CPU_INT32U baud_rate) 
{	  
    register CPU_INT16U sbr;
    CPU_INT08U temp;  
    CPU_INT32U clk_Khz;
    UARTLP_MemMapPtr uartch1 = UART0_BASE_PTR;//??uartch1?UARTLP_MemMapPtr??????? 
    UART_MemMapPtr uartch2;       //??uartch2?UART_MemMapPtr??????? 
    uartch2 = UART_ARR[uartNo-1]; /* Get UART1,UART2 modules base address */
	
    //??????uartNo,?????uartch1??
	
    if (uartNo == 0) {
				switch (sel_clk) {
				case BSP_MCGIRCLK:
						//?????UART0???MCGIRCLK
						MCGIRCLK_OutEnable();      //????????	
						SIM_SOPT2 |= SIM_SOPT2_UART0SRC(0x3);  //UART0??MCGIRCLK=4000Khz???
						break;
				 
				case BSP_MCGPLL:
						SIM_SOPT2 |= SIM_SOPT2_UART0SRC(1);   //UART0??MCGFLLCLK_khz=48000Khz???
						SIM_SOPT2 |= SIM_SOPT2_PLLFLLSEL_MASK;
						break;
					 
				default:
					  break;
				}
				
				//??????????????????  	   
				PORTA_PCR14 = PORT_PCR_MUX(0x3);      //??UART0_TXD
				PORTA_PCR15 = PORT_PCR_MUX(0x3);      //??UART0_RXD	
				SIM_SCGC4  |= SIM_SCGC4_UART0_MASK;    //????0??
				
				//??????0???????  
				uartch1->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK); 
			
				//???????????
				if(BSP_MCGIRCLK == sel_clk)
					clk_Khz = MCGIR_CLK_KHZ;
				else if (BSP_MCGPLL == sel_clk)
					clk_Khz = MCGPLL_CLK_KHZ;
			 
				//????????:8??????
				sbr = (CPU_INT16U)((clk_Khz*1000)/(baud_rate * 16));
				temp = UART_BDH_REG(uartch1) & ~(UART_BDH_SBR(0x1F));
				UART_BDH_REG(uartch1) = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
				UART_BDL_REG(uartch1) = (CPU_INT08U)(sbr & UART_BDL_SBR_MASK);

				//?????????????
				UART0_C4 = 0x0F;
				UART0_C1 = 0x00;
				UART0_C3 = 0x00;
				UART0_MA1 = 0x00;
				UART0_MA2 = 0x00;
				UART0_S1 |= 0x1F;
				UART0_S2 |= 0xC0;    
					
				//??????
				uartch1->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);    
	}
	else
	{
		 switch (uartNo)
		       {
		       case BSP_UART_1:
		           PORTE_PCR0 = PORT_PCR_MUX(0x3);    //??UART1_TXD
		           PORTE_PCR1 = PORT_PCR_MUX(0x3);    //??UART1_RXD
		           SIM_SCGC4 |= SIM_SCGC4_UART1_MASK; //????1??
		           break;
		       case BSP_UART_2:
		           PORTE_PCR22 = PORT_PCR_MUX(0x4); //??UART2_TXD
		           PORTE_PCR23 = PORT_PCR_MUX(0x4); //??UART2_RXD
		           SIM_SCGC4  |= SIM_SCGC4_UART2_MASK;//????2??
		           break;
		       default:
		    	   break;  //????,??
		       }
		       
		 //??????1?2???????  
		uartch2->C2 &= ~(UART_C2_TE_MASK | UART_C2_RE_MASK);
		   	  
		//?????,KL25??1?2???????? Bus clock = 24M ????
		//????????,8??????
		uartch2->C1 = 0;          
		sbr = (CPU_INT16U)((BUS_CLK_KHZ*1000)/(baud_rate * 16));
		temp = UART_BDH_REG(uartch2) & ~(UART_BDH_SBR(0x1F));
		UART_BDH_REG(uartch2) = temp |  UART_BDH_SBR(((sbr & 0x1F00) >> 8));
		UART_BDL_REG(uartch2) = (CPU_INT08U)(sbr & UART_BDL_SBR_MASK);
		           
		//?????????????
		uartch2->C1 = 0x00;
		uartch2->C3 = 0x00;
		uartch2->S1 = 0x1F;
		uartch2->S2 = 0x00;   	               
		//??????
		uartch2->C2 |= (UART_C2_TE_MASK | UART_C2_RE_MASK);                           	
	}
 }

/*
*********************************************************************************************************
*                                            BSP_LCD_Init()
*
* Description : Initialize the Four-Digit LCD Panel.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_LCD_Init (void)
{
    SIM_SCGC5   |=  SIM_SCGC5_SLCD_MASK;                        /* Enable clock for the SLCD module.                    */

                                                                /* Configure pins for SLCD module.                      */
    PORTD_PCR0  |=  PORT_PCR_MUX(0u);                           /* SLCD Pin 40.                                         */
    PORTE_PCR4  |=  PORT_PCR_MUX(0u);                           /* SLCD Pin 52.                                         */
    PORTB_PCR23 |=  PORT_PCR_MUX(0u);                           /* SLCD Pin 19.                                         */
    PORTB_PCR22 |=  PORT_PCR_MUX(0u);                           /* SLCD Pin 18.                                         */
    PORTC_PCR17 |=  PORT_PCR_MUX(0u);                           /* SLCD Pin 37.                                         */
    PORTB_PCR21 |=  PORT_PCR_MUX(0u);                           /* SLCD Pin 17.                                         */
    PORTB_PCR7  |=  PORT_PCR_MUX(0u);                           /* SLCD Pin 07.                                         */
    PORTB_PCR8  |=  PORT_PCR_MUX(0u);                           /* SLCD Pin 08.                                         */
    PORTE_PCR5  |=  PORT_PCR_MUX(0u);                           /* SLCD Pin 53.                                         */
    PORTC_PCR18 |=  PORT_PCR_MUX(0u);                           /* SLCD Pin 38.                                         */
    PORTB_PCR10 |=  PORT_PCR_MUX(0u);                           /* SLCD Pin 10.                                         */
    PORTB_PCR11 |=  PORT_PCR_MUX(0u);                           /* SLCD Pin 11.                                         */

                                                                /* ------------ SLCD Module Configuration ------------- */
    LCD_GCR      = (LCD_GCR_RVEN_MASK    |                      /* Regulated voltage enabled.                           */
                    LCD_GCR_RVTRIM(0xFu) |                      /* Set Regulated Voltage Trim (1.5% for each bit).      */
                    LCD_GCR_CPSEL_MASK   |                      /* LCD charge pump selected.                            */
                    LCD_GCR_LADJ(0x0u)   |                      /* Fastest clock source for charge pump.                */
                    LCD_GCR_VSUPPLY_MASK |                      /* Drive VLL from internal regulated voltage (VIREG).   */
                    LCD_GCR_ALTDIV(0x2u) |                      /* LCD Base Clock : (4 MHz / 64) = 62.5 kHz             */
                    LCD_GCR_SOURCE_MASK  |                      /* Select the alternate clock source (MCGIRCLK)         */
                    LCD_GCR_LCLK(0x4u)   |                      /* Select LCD Clock Prescaler.                          */
                    LCD_GCR_DUTY(0x3u));                        /* Use 4 back planes (1/4 duty cycle).                  */

                                                                /* Enable LCD operation for the pins.                   */
    LCD_PENL     = ((1u <<  BSP_LCD_PIN_03)  |
                    (1u <<  BSP_LCD_PIN_04)  |
                    (1u <<  BSP_LCD_PIN_06)  |
                    (1u <<  BSP_LCD_PIN_07)  |
                    (1u <<  BSP_LCD_PIN_08)  |
                    (1u <<  BSP_LCD_PIN_11)  |
                    (1u <<  BSP_LCD_PIN_12));

    LCD_PENH     = ((1u << (BSP_LCD_PIN_01 - 32u)) |
                    (1u << (BSP_LCD_PIN_02 - 32u)) |
                    (1u << (BSP_LCD_PIN_05 - 32u)) |
                    (1u << (BSP_LCD_PIN_09 - 32u)) |
                    (1u << (BSP_LCD_PIN_10 - 32u)));

                                                                /* Enable back plane operation for the COM pins.        */
    LCD_BPENL    = ((1u <<  BSP_LCD_PIN_03)  |
                    (1u <<  BSP_LCD_PIN_04));

    LCD_BPENH    = ((1u << (BSP_LCD_PIN_01 - 32u)) |
                    (1u << (BSP_LCD_PIN_02 - 32u)));

                                                                /* Select phases for back plane pins.                   */
    BSP_LCD_WFR_WRT(BSP_LCD_PIN_01, BSP_LCD_WFR_BIT_A);         /* COM0 : Phase A.                                      */
    BSP_LCD_WFR_WRT(BSP_LCD_PIN_02, BSP_LCD_WFR_BIT_B);         /* COM1 : Phase B.                                      */
    BSP_LCD_WFR_WRT(BSP_LCD_PIN_03, BSP_LCD_WFR_BIT_C);         /* COM2 : Phase C.                                      */
    BSP_LCD_WFR_WRT(BSP_LCD_PIN_04, BSP_LCD_WFR_BIT_D);         /* COM3 : Phase D.                                      */

    LCD_GCR     |=  LCD_GCR_LCDEN_MASK;                         /* Enable LCD driver.                                   */

    BSP_LCD_Clear();
}


/*
*********************************************************************************************************
*                                           BSP_LCD_Write()
*
* Description : Write digits to the LCD display.
*
* Argument(s) : dig1            Hex value for the first digit (Most Significant Digit).
*
*               dig2            Hex value for the second digit.
*
*               dig3            Hex value for the third digit.
*
*               dig4            Hex value for the fourth digit (Least Significant Digit).
*
*               opt             Used to optionally turn on the colon or decimal point segments.
*                               The following options are available (See Notes #1a and #1b):
*
*                               BSP_LCD_OPT_NONE    If the decimal points and colon are not used.
*
*                               BSP_LCD_OPT_DP1     Left   decimal point segment.
*
*                               BSP_LCD_OPT_DP2     Middle decimal point segment.
*
*                               BSP_LCD_OPT_DP3     Right  decimal point segment.
*
*                               BSP_LCD_OPT_COL     Colon  segment.
*
*
* Return(s)   : DEF_OK          if dig1 - dig4 are between 0x0 and 0xF inclusive,
*                               and if opt has a legal value (See Note #1b).
*
*               DEF_FAIL        otherwise.
*
*
* Caller(s)   : Application.
*
* Note(s)     : (1) (a) The LCD panel contains four digits (DIG1 - DIG4), three decimal points (DP1 - DP3),
*                       and a colon (COL). The display is layed out as shown below.
*
*                               ====         ====   COL   ====         ====
*                             ||    ||     ||    ||  _  ||    ||     ||    ||
*                             ||    ||     ||    || |_| ||    ||     ||    ||
*                             ||    ||     ||    ||  _  ||    ||     ||    ||
*                               ====         ====   |_|   ====         ====
*                             ||    ||     ||    ||     ||    ||     ||    ||
*                             ||    ||  _  ||    ||  _  ||    ||  _  ||    ||
*                             ||    || |_| ||    || |_| ||    || |_| ||    ||
*                               ====         ====         ====         ====
*                               DIG1   DP1   DIG2   DP2   DIG3   DP3   DIG4
*
*
*                   (b) The DP1-DP3 and COL segments can be turned on using the opt parameter.
*                       opt is formed by performing a bitwise OR of the BSP_LCD_OPT_XXX constants.
*********************************************************************************************************
*/

CPU_BOOLEAN  BSP_LCD_Write (CPU_INT08U  dig1,
                            CPU_INT08U  dig2,
                            CPU_INT08U  dig3,
                            CPU_INT08U  dig4,
                            CPU_INT08U  opt)
{
                                                                /* Validate arguments.                                  */
    if (((dig1 & ~BSP_LCD_DIG_MASK) != 0u) ||
        ((dig2 & ~BSP_LCD_DIG_MASK) != 0u) ||
        ((dig3 & ~BSP_LCD_DIG_MASK) != 0u) ||
        ((dig4 & ~BSP_LCD_DIG_MASK) != 0u) ||
        ((opt  & ~BSP_LCD_OPT_MASK) != 0u)) {

        return (DEF_FAIL);
    }

                                                                /* Set the WFR register pair values for each digit.     */
    BSP_LCD_WFR_WRT(BSP_LCD_DIG1_PIN0, BSP_LCD_WFRL[dig1]);
    BSP_LCD_WFR_WRT(BSP_LCD_DIG1_PIN1, BSP_LCD_WFRH[dig1]);

    BSP_LCD_WFR_WRT(BSP_LCD_DIG2_PIN0, BSP_LCD_WFRL[dig2]);
    BSP_LCD_WFR_WRT(BSP_LCD_DIG2_PIN1, BSP_LCD_WFRH[dig2]);

    BSP_LCD_WFR_WRT(BSP_LCD_DIG3_PIN0, BSP_LCD_WFRL[dig3]);
    BSP_LCD_WFR_WRT(BSP_LCD_DIG3_PIN1, BSP_LCD_WFRH[dig3]);

    BSP_LCD_WFR_WRT(BSP_LCD_DIG4_PIN0, BSP_LCD_WFRL[dig4]);
    BSP_LCD_WFR_WRT(BSP_LCD_DIG4_PIN1, BSP_LCD_WFRH[dig4]);

                                                                /* Turn on Optional Segments (DPs or COL).              */
    if (DEF_BIT_IS_SET(opt, BSP_LCD_OPT_DP1)) {
        BSP_LCD_WFR_SET(BSP_LCD_DP1_PIN0, BSP_LCD_SEG_OPT);
    }

    if (DEF_BIT_IS_SET(opt, BSP_LCD_OPT_DP2)) {
        BSP_LCD_WFR_SET(BSP_LCD_DP2_PIN0, BSP_LCD_SEG_OPT);
    }

    if (DEF_BIT_IS_SET(opt, BSP_LCD_OPT_DP3)) {
        BSP_LCD_WFR_SET(BSP_LCD_DP3_PIN0, BSP_LCD_SEG_OPT);
    }

    if (DEF_BIT_IS_SET(opt, BSP_LCD_OPT_COL)) {
        BSP_LCD_WFR_SET(BSP_LCD_COL_PIN0, BSP_LCD_SEG_OPT);
    }

    return (DEF_OK);
}

/*
*********************************************************************************************************
*                                           BSP_LCD_Clear()
*
* Description : Clear the LCD Display.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_LCD_Clear (void)
{
    BSP_LCD_WFR_WRT(BSP_LCD_DIG1_PIN0, BSP_LCD_SEG_NONE);
    BSP_LCD_WFR_WRT(BSP_LCD_DIG1_PIN1, BSP_LCD_SEG_NONE);

    BSP_LCD_WFR_WRT(BSP_LCD_DIG2_PIN0, BSP_LCD_SEG_NONE);
    BSP_LCD_WFR_WRT(BSP_LCD_DIG2_PIN1, BSP_LCD_SEG_NONE);

    BSP_LCD_WFR_WRT(BSP_LCD_DIG3_PIN0, BSP_LCD_SEG_NONE);
    BSP_LCD_WFR_WRT(BSP_LCD_DIG3_PIN1, BSP_LCD_SEG_NONE);

    BSP_LCD_WFR_WRT(BSP_LCD_DIG4_PIN0, BSP_LCD_SEG_NONE);
    BSP_LCD_WFR_WRT(BSP_LCD_DIG4_PIN1, BSP_LCD_SEG_NONE);
}

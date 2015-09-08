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
*                                     BOARD SUPPORT PACKAGE (BSP)
*
*                                       IAR Development Kits
*                                              on the
*
*                                    STM32F429II-SK KICKSTART KIT
*
* Filename      : bsp.h
* Version       : V1.00
* Programmer(s) : FF
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                 MODULE
*
* Note(s) : (1) This header file is protected from multiple pre-processor inclusion through use of the
*               BSP present pre-processor macro definition.
*
*           (2) This file and its dependencies requires IAR v6.20 or later to be compiled.
*
*********************************************************************************************************
*/

#ifndef  BSP_PRESENT
#define  BSP_PRESENT


/*
*********************************************************************************************************
*                                                 EXTERNS
*********************************************************************************************************
*/

#ifdef   BSP_MODULE
#define  BSP_EXT
#else
#define  BSP_EXT  extern
#endif


/*
*********************************************************************************************************
*                                              INCLUDE FILES
*********************************************************************************************************
*/

#include  <stdio.h>
#include  <stdarg.h>

#include  <cpu.h>
#include  <cpu_core.h>

#include  <lib_def.h>
#include  <lib_ascii.h>


#include  "stm32f4xx.h"

#include  <bsp_ser.h>
#include  <bsp_os.h>


/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             PERIPH DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               DATA TYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                            GLOBAL VARIABLES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                                 MACRO'S
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                               INT DEFINES
*********************************************************************************************************
*/

#define  BSP_INT_ID_WWDG                                   0    /* Window WatchDog Interrupt                            */
#define  BSP_INT_ID_PVD                                    1    /* PVD through EXTI Line detection Interrupt            */
#define  BSP_INT_ID_TAMP_STAMP                             2    /* Tamper and TimeStamp Interrupt                       */
#define  BSP_INT_ID_RTC_WKUP                               3    /* RTC wakeup Interrupt through the EXTI line           */
#define  BSP_INT_ID_FLASH                                  4    /* FLASH global Interrupt                               */
#define  BSP_INT_ID_RCC                                    5    /* RCC global Interrupt                                 */
#define  BSP_INT_ID_EXTI0                                  6    /* EXTI Line0 Interrupt                                 */
#define  BSP_INT_ID_EXTI1                                  7    /* EXTI Line1 Interrupt                                 */
#define  BSP_INT_ID_EXTI2                                  8    /* EXTI Line2 Interrupt                                 */
#define  BSP_INT_ID_EXTI3                                  9    /* EXTI Line3 Interrupt                                 */
#define  BSP_INT_ID_EXTI4                                 10    /* EXTI Line4 Interrupt                                 */
#define  BSP_INT_ID_DMA1_CH0                              11    /* DMA1 Channel 0 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH1                              12    /* DMA1 Channel 1 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH2                              13    /* DMA1 Channel 2 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH3                              14    /* DMA1 Channel 3 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH4                              15    /* DMA1 Channel 4 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH5                              16    /* DMA1 Channel 5 global Interrupt                      */
#define  BSP_INT_ID_DMA1_CH6                              17    /* DMA1 Channel 6 global Interrupt                      */
#define  BSP_INT_ID_ADC                                   18    /* ADC1, ADC2 and ADC3 global Interrupt                 */
#define  BSP_INT_ID_CAN1_TX                               19    /* CAN1 TX Interrupts                                   */
#define  BSP_INT_ID_CAN1_RX0                              20    /* CAN1 RX0 Interrupts                                  */
#define  BSP_INT_ID_CAN1_RX1                              21    /* CAN1 RX1 Interrupt                                   */
#define  BSP_INT_ID_CAN1_SCE                              22    /* CAN1 SCE Interrupt                                   */
#define  BSP_INT_ID_EXTI9_5                               23    /* External Line[9:5] Interrupts                        */
#define  BSP_INT_ID_TIM1_BRK_TIM9                         24    /* TIM1 Break Interrupt and TIM9 global interrupt       */
#define  BSP_INT_ID_TIM1_UP_TIM10                         25    /* TIM1 Update Interrupt and TIM10 global interrupt     */
#define  BSP_INT_ID_TIM1_TRG_COM_TIM11                    26    /* TIM1 Trigger & Commutation Int. & TIM11 global Int.  */
#define  BSP_INT_ID_TIM1_CC                               27    /* TIM1 Capture Compare Interrupt                       */
#define  BSP_INT_ID_TIM2                                  28    /* TIM2 global Interrupt                                */
#define  BSP_INT_ID_TIM3                                  29    /* TIM3 global Interrupt                                */
#define  BSP_INT_ID_TIM4                                  30    /* TIM4 global Interrupt                                */
#define  BSP_INT_ID_I2C1_EV                               31    /* I2C1 Event Interrupt                                 */
#define  BSP_INT_ID_I2C1_ER                               32    /* I2C1 Error Interrupt                                 */
#define  BSP_INT_ID_I2C2_EV                               33    /* I2C2 Event Interrupt                                 */
#define  BSP_INT_ID_I2C2_ER                               34    /* I2C2 Error Interrupt                                 */
#define  BSP_INT_ID_SPI1                                  35    /* SPI1 global Interrupt                                */
#define  BSP_INT_ID_SPI2                                  36    /* SPI2 global Interrupt                                */
#define  BSP_INT_ID_USART1                                37    /* USART1 global Interrupt                              */
#define  BSP_INT_ID_USART2                                38    /* USART2 global Interrupt                              */
#define  BSP_INT_ID_USART3                                39    /* USART3 global Interrupt                              */
#define  BSP_INT_ID_EXTI15_10                             40    /* External Line[15:10] Interrupts                      */
#define  BSP_INT_ID_RTC_ALARM                             41    /* RTC Alarms (A and B) through EXTI Line Interrupt     */
#define  BSP_INT_ID_OTG_FS_WKUP                           42    /* USB WakeUp from suspend through EXTI Line Interrupt  */

#define  BSP_INT_ID_TIM8_BRK_TIM12                        43    /* TIM8 Break Interrupt and TIM12 global Interrupt      */
#define  BSP_INT_ID_TIM8_UP_TIM13                         44    /* TIM8 Update Interrupt and TIM13 global Interrupt     */
#define  BSP_INT_ID_TIM8_TRG_COM_TIM14                    45    /* TIM8 Trigger/Commutation and TIM14 global Interrupt  */
#define  BSP_INT_ID_TIM8_CC                               46    /* TIM8 Capture Compare Interrupt                       */
#define  BSP_INT_ID_DMA1_CH7                          47    /* DMA1 Stream7 Interrupt                               */
#define  BSP_INT_ID_FSMC                                  48    /* FSMC global Interrupt                                */
#define  BSP_INT_ID_SDIO                                  49    /* SDIO global Interrupt                                */

#define  BSP_INT_ID_TIM5                                  50    /* TIM5 global Interrupt                                */
#define  BSP_INT_ID_SPI3                                  51    /* SPI3 global Interrupt                                */
#define  BSP_INT_ID_USART4                                52    /* USART4 global Interrupt                              */
#define  BSP_INT_ID_USART5                                53    /* USART5 global Interrupt                              */
#define  BSP_INT_ID_TIM6_DAC                              54    /* TIM6 global Interrupt, DAC1 & DAC2 underrun err int. */
#define  BSP_INT_ID_TIM7                                  55    /* TIM7 global Interrupt                                */
#define  BSP_INT_ID_DMA2_CH0                              56    /* DMA2 Channel 0 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH1                              57    /* DMA2 Channel 1 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH2                              58    /* DMA2 Channel 2 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH3                              59    /* DMA2 Channel 3 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH4                              60    /* DMA2 Channel 4 global Interrupt                      */

#define  BSP_INT_ID_ETH                                   61    /* ETH  global Interrupt                                */
#define  BSP_INT_ID_ETH_WKUP                              62    /* ETH  WakeUp from EXTI line interrupt                 */
#define  BSP_INT_ID_CAN2_TX                               63    /* CAN2 TX Interrupts                                   */
#define  BSP_INT_ID_CAN2_RX0                              64    /* CAN2 RX0 Interrupts                                  */
#define  BSP_INT_ID_CAN2_RX1                              65    /* CAN2 RX1 Interrupt                                   */
#define  BSP_INT_ID_CAN2_SCE                              66    /* CAN2 SCE Interrupt                                   */
#define  BSP_INT_ID_OTG_FS                                67    /* OTG global Interrupt                                 */

#define  BSP_INT_ID_DMA2_CH5                              68    /* DMA2 Channel 5 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH6                              69    /* DMA2 Channel 6 global Interrupt                      */
#define  BSP_INT_ID_DMA2_CH7                              70    /* DMA2 Channel 7 global Interrupt                      */
#define  BSP_INT_ID_USART6                                71    /* USART6 global Interrupt                              */
#define  BSP_INT_ID_I2C3_EV                               72    /* I2C3 Event  Interrupt                                */
#define  BSP_INT_ID_I2C3_ER                               73    /* I2C3 Error  Interrupt                                */
#define  BSP_INT_ID_OTG_HS_EP1_OUT                        74    /* OTG HS EP1 OUT global Interrupt                      */
#define  BSP_INT_ID_OTG_HS_EP1_IN                         75    /* OTG HS EP1 IN global Interrupt                       */
#define  BSP_INT_ID_OTG_HS_WKUP                           76    /* OTG HS Wakeup Interrupt                              */
#define  BSP_INT_ID_OTG_HS                                77    /* OTG HS global Interrupt                              */
#define  BSP_INT_ID_DCMI                                  78    /* DCMI global Interrupt                                */
#define  BSP_INT_ID_CRYP                                  79    /* CRYPT crypto global Interrupt                        */
#define  BSP_INT_ID_HASH_RNG                              80    /* HASH and RNG global Interrupt                        */
#define  BSP_INT_ID_FPU                                   81    /* FPU global Interrupt                                 */
#define  BSP_INT_ID_UART7                                 82    /* USART7 global Interrupt                              */
#define  BSP_INT_ID_UART8                                 83    /* USART8 global Interrupt                              */
#define  BSP_INT_ID_SPI4                                  84    /* SPI4   global Interrupt                              */
#define  BSP_INT_ID_SPI5                                  85    /* SPI6   global Interrupt                              */
#define  BSP_INT_ID_SPI6                                  86    /* SPI7   global Interrupt                              */
#define  BSP_INT_ID_SAI1                                  87    /* SAI1   Interrupt                                     */
#define  BSP_INT_ID_LTDC                                  88    /* LTDC   Interrupt                                     */
#define  BSP_INT_ID_LTDC_ER                               89    /* LTDC   Error Interrupt                               */
#define  BSP_INT_ID_DMA2D                                 90    /* DMA2D  Interrupt                                     */



/*
*********************************************************************************************************
*                                             PERIPH DEFINES
*********************************************************************************************************
*/
                                                                /* -                  AHB1 BUS                        - */
#define  BSP_PERIPH_ID_GPIOA                               0
#define  BSP_PERIPH_ID_GPIOB                               1
#define  BSP_PERIPH_ID_GPIOC                               2
#define  BSP_PERIPH_ID_GPIOD                               3
#define  BSP_PERIPH_ID_GPIOE                               4
#define  BSP_PERIPH_ID_GPIOF                               5
#define  BSP_PERIPH_ID_GPIOG                               6
#define  BSP_PERIPH_ID_GPIOH                               7
#define  BSP_PERIPH_ID_GPIOI                               8
#define  BSP_PERIPH_ID_GPIOJ                               9
#define  BSP_PERIPH_ID_GPIOK                              10
#define  BSP_PERIPH_ID_CRC                                12
#define  BSP_PERIPH_ID_BKPSRAM                            18
#define  BSP_PERIPH_ID_CCMDATARAM                         20
#define  BSP_PERIPH_ID_DMA1                               21
#define  BSP_PERIPH_ID_DMA2                               22
#define  BSP_PERIPH_ID_DMA2D                              23
#define  BSP_PERIPH_ID_ETHMAC                             25
#define  BSP_PERIPH_ID_ETHMACTX                           26
#define  BSP_PERIPH_ID_ETHMACRX                           27
#define  BSP_PERIPH_ID_ETHMACPTP                          28
#define  BSP_PERIPH_ID_OTGHS                              29
#define  BSP_PERIPH_ID_OTGHSULPI                          30
                                                                /* -                  AHB2 BUS                        - */
#define  BSP_PERIPH_ID_DCMI                               32
#define  BSP_PERIPH_ID_CRYP                               36
#define  BSP_PERIPH_ID_HASH                               37
#define  BSP_PERIPH_ID_RNG                                38
#define  BSP_PERIPH_ID_OTGFS                              39
                                                                /* -                  AHB3 BUS                        - */
#define  BSP_PERIPH_ID_FSMC                               64
                                                                /* -                  APB1 BUS                        - */
#define  BSP_PERIPH_ID_TIM2                               96
#define  BSP_PERIPH_ID_TIM3                               97
#define  BSP_PERIPH_ID_TIM4                               98
#define  BSP_PERIPH_ID_TIM5                               99
#define  BSP_PERIPH_ID_TIM6                              100
#define  BSP_PERIPH_ID_TIM7                              101
#define  BSP_PERIPH_ID_TIM12                             102
#define  BSP_PERIPH_ID_TIM13                             103
#define  BSP_PERIPH_ID_TIM14                             104
#define  BSP_PERIPH_ID_WWDG                              107
#define  BSP_PERIPH_ID_SPI2                              110
#define  BSP_PERIPH_ID_SPI3                              111
#define  BSP_PERIPH_ID_USART2                            113
#define  BSP_PERIPH_ID_USART3                            114
#define  BSP_PERIPH_ID_UART4                             115
#define  BSP_PERIPH_ID_UART5                             116
#define  BSP_PERIPH_ID_I2C1                              117
#define  BSP_PERIPH_ID_I2C2                              118
#define  BSP_PERIPH_ID_I2C3                              119
#define  BSP_PERIPH_ID_CAN1                              121
#define  BSP_PERIPH_ID_CAN2                              122
#define  BSP_PERIPH_ID_PWR                               124
#define  BSP_PERIPH_ID_DAC                               125
#define  BSP_PERIPH_ID_UART7                             126
#define  BSP_PERIPH_ID_UART8                             127
                                                                /* -                  APB2 BUS                        - */
#define  BSP_PERIPH_ID_TIM1                              128
#define  BSP_PERIPH_ID_TIM8                              129
#define  BSP_PERIPH_ID_USART1                            132
#define  BSP_PERIPH_ID_USART6                            133
#define  BSP_PERIPH_ID_ADC1                              136
#define  BSP_PERIPH_ID_ADC2                              137
#define  BSP_PERIPH_ID_ADC3                              138
#define  BSP_PERIPH_ID_SDIO                              139
#define  BSP_PERIPH_ID_SPI1                              140
#define  BSP_PERIPH_ID_SPI4                              141
#define  BSP_PERIPH_ID_SYSCFG                            142
#define  BSP_PERIPH_ID_TIM9                              144
#define  BSP_PERIPH_ID_TIM10                             145
#define  BSP_PERIPH_ID_TIM11                             146
#define  BSP_PERIPH_ID_SPI5                              148
#define  BSP_PERIPH_ID_SPI6                              149
#define  BSP_PERIPH_ID_SAI1                              150
#define  BSP_PERIPH_ID_LTDC                              154

/*
*********************************************************************************************************
*                                           FUNCTION PROTOTYPES
*********************************************************************************************************
*/

void        BSP_Init                          (void);

void        BSP_IntDisAll                     (void);

CPU_INT32U  BSP_CPU_ClkFreq                   (void);

/*
*********************************************************************************************************
*                                           INTERRUPT SERVICES
*********************************************************************************************************
*/

void        BSP_IntInit                       (void);

void        BSP_IntEn                         (CPU_DATA       int_id);

void        BSP_IntDis                        (CPU_DATA       int_id);

void        BSP_IntClr                        (CPU_DATA       int_id);
                    

#define BSP_IntVectSet(int_id,isr)  BSP_IntVectSetEx(int_id, (CPU_FNCT_PTR)isr, NULL)

void        BSP_IntVectSetEx                    (CPU_DATA       int_id,
                                               CPU_FNCT_PTR  isr,
                                                void *p_arg);

void        BSP_IntPrioSet                    (CPU_DATA       int_id,
                                               CPU_INT08U     prio);

void         BSP_IntHandlerWWDG               (void);
void         BSP_IntHandlerPVD                (void);
void         BSP_IntHandlerTAMP_STAMP         (void);
void         BSP_IntHandlerRTC_WKUP           (void);
void         BSP_IntHandlerFLASH              (void);
void         BSP_IntHandlerRCC                (void);
void         BSP_IntHandlerEXTI0              (void);
void         BSP_IntHandlerEXTI1              (void);
void         BSP_IntHandlerEXTI2              (void);
void         BSP_IntHandlerEXTI3              (void);
void         BSP_IntHandlerEXTI4              (void);
void         BSP_IntHandlerDMA1_CH0           (void);
void         BSP_IntHandlerDMA1_CH1           (void);
void         BSP_IntHandlerDMA1_CH2           (void);
void         BSP_IntHandlerDMA1_CH3           (void);
void         BSP_IntHandlerDMA1_CH4           (void);
void         BSP_IntHandlerDMA1_CH5           (void);
void         BSP_IntHandlerDMA1_CH6           (void);
void         BSP_IntHandlerADC                (void);
void         BSP_IntHandlerCAN1_TX            (void);
void         BSP_IntHandlerCAN1_RX0           (void);
void         BSP_IntHandlerCAN1_RX1           (void);
void         BSP_IntHandlerCAN1_SCE           (void);
void         BSP_IntHandlerEXTI9_5            (void);
void         BSP_IntHandlerTIM1_BRK_TIM9      (void);
void         BSP_IntHandlerTIM1_UP_TIM10      (void);
void         BSP_IntHandlerTIM1_TRG_COM_TIM11 (void);
void         BSP_IntHandlerTIM1_CC            (void);
void         BSP_IntHandlerTIM2               (void);
void         BSP_IntHandlerTIM3               (void);
void         BSP_IntHandlerTIM4               (void);
void         BSP_IntHandlerI2C1_EV            (void);
void         BSP_IntHandlerI2C1_ER            (void);
void         BSP_IntHandlerI2C2_EV            (void);
void         BSP_IntHandlerI2C2_ER            (void);
void         BSP_IntHandlerSPI1               (void);
void         BSP_IntHandlerSPI2               (void);
void         BSP_IntHandlerUSART1             (void);
void         BSP_IntHandlerUSART2             (void);
void         BSP_IntHandlerUSART3             (void);
void         BSP_IntHandlerEXTI15_10          (void);
void         BSP_IntHandlerRTCAlarm           (void);
void         BSP_IntHandlerOTG_FS_WKUP        (void);
void         BSP_IntHandlerTIM8_BRK_TIM12     (void);
void         BSP_IntHandlerTIM8_UP_TIM13      (void);
void         BSP_IntHandlerTIM8_TRG_COM_TIM14 (void);
void         BSP_IntHandlerTIM8_CC            (void);
void         BSP_IntHandlerDMA1_STREAM7       (void);
void         BSP_IntHandlerFSMC               (void);
void         BSP_IntHandlerSDIO               (void);
void         BSP_IntHandlerTIM5               (void);
void         BSP_IntHandlerSPI3               (void);
void         BSP_IntHandlerUSART4             (void);
void         BSP_IntHandlerUSART5             (void);
void         BSP_IntHandlerTIM6_DAC           (void);
void         BSP_IntHandlerTIM7               (void);
void         BSP_IntHandlerDMA2_CH0           (void);
void         BSP_IntHandlerDMA2_CH1           (void);
void         BSP_IntHandlerDMA2_CH2           (void);
void         BSP_IntHandlerDMA2_CH3           (void);
void         BSP_IntHandlerDMA2_CH4           (void);
void         BSP_IntHandlerETH                (void);
void         BSP_IntHandlerETHWakeup          (void);
void         BSP_IntHandlerCAN2_TX            (void);
void         BSP_IntHandlerCAN2_RX0           (void);
void         BSP_IntHandlerCAN2_RX1           (void);
void         BSP_IntHandlerCAN2_SCE           (void);
void         BSP_IntHandlerOTG_FS             (void);
void         BSP_IntHandlerDMA2_CH5           (void);
void         BSP_IntHandlerDMA2_CH6           (void);
void         BSP_IntHandlerDMA2_CH7           (void);
void         BSP_IntHandlerUSART6             (void);
void         BSP_IntHandlerI2C3_EV            (void);
void         BSP_IntHandlerI2C3_ER            (void);
void         BSP_IntHandlerOTG_HS_EP1_OUT     (void);
void         BSP_IntHandlerOTG_HS_EP1_IN      (void);
void         BSP_IntHandlerOTG_HS_WKUP        (void);
void         BSP_IntHandlerOTG_HS             (void);
void         BSP_IntHandlerDCMI               (void);
void         BSP_IntHandlerCRYP               (void);
void         BSP_IntHandlerHASH_RNG           (void);
void         BSP_IntHandlerFPU                (void);
void         BSP_IntHandlerUART7              (void);
void         BSP_IntHandlerUART8              (void);
void         BSP_IntHandlerSPI4               (void);
void         BSP_IntHandlerSPI5               (void);
void         BSP_IntHandlerSPI6               (void);
void         BSP_IntHandlerSAI1               (void);
void         BSP_IntHandlerLTDC               (void);
void         BSP_IntHandlerLTDC_ER            (void);
void         BSP_IntHandlerDMA2D              (void);


/*
*********************************************************************************************************
*                                     PERIPHERAL POWER/CLOCK SERVICES
*********************************************************************************************************
*/

CPU_INT32U   BSP_PeriphClkFreqGet        (CPU_DATA       pwr_clk_id);

void         BSP_PeriphEn                (CPU_DATA       pwr_clk_id);

void         BSP_PeriphDis               (CPU_DATA       pwr_clk_id);

/*
*********************************************************************************************************
*                                              LED SERVICES
*********************************************************************************************************
*/

void        BSP_LED_On                        (CPU_INT08U     led);

void        BSP_LED_Off                       (CPU_INT08U     led);

void        BSP_LED_Toggle                    (CPU_INT08U     led);

/*
*********************************************************************************************************
*                                             MODULE END
*********************************************************************************************************
*/


#endif                                                          /* End of module include.                               */


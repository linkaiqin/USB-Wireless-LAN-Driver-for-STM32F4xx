/**
  ******************************************************************************
  * @file    usb_bsp.c
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    19-March-2012
  * @brief   This file implements the board support package for the USB host library
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#include "bsp.h"
#include "usb_bsp.h"


/** @addtogroup USBH_USER
* @{
*/

/** @defgroup USB_BSP
  * @brief This file is responsible to offer board support package
  * @{
  */

/** @defgroup USB_BSP_Private_Defines
  * @{
  */


#ifdef USE_USB_OTG_FS
#define HOST_POWERSW_PORT_RCC            RCC_AHB1Periph_GPIOC
#define HOST_POWERSW_PORT                GPIOC
#define HOST_POWERSW_VBUS                GPIO_Pin_0
#endif

/**
  * @}
  */


/** @defgroup USB_BSP_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */



/** @defgroup USB_BSP_Private_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup USBH_BSP_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup USBH_BSP_Private_FunctionPrototypes
  * @{
  */


/**
  * @}
  */

/** @defgroup USB_BSP_Private_Functions
  * @{
  */

/**
  * @brief  USB_OTG_BSP_Init
  *         Initilizes BSP configurations
  * @param  None
  * @retval None
  */

void USB_OTG_BSP_Init(int coreID)
{
    GPIO_InitTypeDef GPIO_InitStructure;

#ifdef USB_OTG_FS_CORE  
    if(coreID == USB_OTG_FS_CORE_ID)
    {

        RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA , ENABLE);

        //PA.11-->D-
        //PA.12-->D+
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 |
                                      GPIO_Pin_12;

        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
        GPIO_Init(GPIOA, &GPIO_InitStructure);

        GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_OTG1_FS) ;
        GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_OTG1_FS) ;

        RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
        RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE) ;
    }
#endif
    
#ifdef USB_OTG_HS_CORE       
    if(coreID == USB_OTG_HS_CORE_ID) 
    {
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
        
        //PB.14-->D-
        //PB.15-->D+
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 |    
                                      GPIO_Pin_15;

        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//new add,prevent assert faild
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ; //new add,prevent assert faild
        GPIO_Init(GPIOB, &GPIO_InitStructure);


        GPIO_PinAFConfig(GPIOB,GPIO_PinSource14,GPIO_AF_OTG2_FS) ;
        GPIO_PinAFConfig(GPIOB,GPIO_PinSource15,GPIO_AF_OTG2_FS) ;
        RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_OTG_HS, ENABLE) ;
    }
#endif
    /* Intialize Timer for delay function */
//   USB_OTG_BSP_TimeInit();
}

void OTG_ISR_Handler(void *p_obj)
{
    USBH_OTG_ISR_Handler(p_obj);
}

/**
  * @brief  USB_OTG_BSP_EnableInterrupt
  *         Configures USB Global interrupt
  * @param  None
  * @retval None
  */
void USB_OTG_BSP_EnableInterrupt(void *otg_handle, int coreID)
{

#ifdef USB_OTG_FS_CORE     
    if(coreID == USB_OTG_FS_CORE_ID)
    {
        BSP_IntVectSetEx(BSP_INT_ID_OTG_FS,OTG_ISR_Handler, otg_handle);
        BSP_IntEn(BSP_INT_ID_OTG_FS);
    }
#endif
    
#ifdef USB_OTG_HS_CORE  
    if(coreID == USB_OTG_HS_CORE_ID)
    {
        BSP_IntVectSetEx(BSP_INT_ID_OTG_HS,OTG_ISR_Handler, otg_handle);
        BSP_IntEn(BSP_INT_ID_OTG_HS);
    }
#endif
}

/**
  * @brief  BSP_Drive_VBUS
  *         Drives the Vbus signal through IO
  * @param  state : VBUS states
  * @retval None
  */

void USB_OTG_BSP_DriveVBUS(uint8_t state, int coreID)
{
    /*
    On-chip 5 V VBUS generation is not supported. For this reason, a charge pump
    or, if 5 V are available on the application board, a basic power switch, must
    be added externally to drive the 5 V VBUS line. The external charge pump can
    be driven by any GPIO output. When the application decides to power on VBUS
    using the chosen GPIO, it must also set the port power bit in the host port
    control and status register (PPWR bit in OTG_FS_HPRT).

    Bit 12 PPWR: Port power
    The application uses this field to control power to this port, and the core
    clears this bit on an overcurrent condition.
    */
#ifdef USB_OTG_FS_CORE 
    if(coreID == USB_OTG_FS_CORE_ID)
    {
        if (0 == state)
        {
            /* DISABLE is needed on output of the Power Switch */
            GPIO_SetBits(HOST_POWERSW_PORT, HOST_POWERSW_VBUS);
        }
        else
        {
            /*ENABLE the Power Switch by driving the Enable LOW */
            GPIO_ResetBits(HOST_POWERSW_PORT, HOST_POWERSW_VBUS);
        }
    }
#endif     
}

/**
  * @brief  USB_OTG_BSP_ConfigVBUS
  *         Configures the IO for the Vbus and OverCurrent
  * @param  None
  * @retval None
  */

void  USB_OTG_BSP_ConfigVBUS(int coreID)
{
#ifdef USB_OTG_FS_CORE        
    if(coreID == USB_OTG_FS_CORE_ID)
    {
        GPIO_InitTypeDef GPIO_InitStructure;

        RCC_AHB1PeriphClockCmd( HOST_POWERSW_PORT_RCC , ENABLE);

        GPIO_InitStructure.GPIO_Pin = HOST_POWERSW_VBUS;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
        GPIO_Init(HOST_POWERSW_PORT,&GPIO_InitStructure);


        /* By Default, DISABLE is needed on output of the Power Switch */
        GPIO_SetBits(HOST_POWERSW_PORT, HOST_POWERSW_VBUS);

        USB_OTG_BSP_mDelay(200);   /* Delay is need for stabilising the Vbus Low
         in Reset Condition, when Vbus=1 and Reset-button is pressed by user */

    }
#endif     
}



void USB_OTG_BSP_uDelay (const uint32_t usec)
{


    __IO uint32_t count = 0;
    const uint32_t utime = (120 * usec / 7);
    do
    {
        if ( ++count > utime )
        {
            return ;
        }
    }
    while (1);


}

#include "delay.h"

/**
  * @brief  USB_OTG_BSP_mDelay
  *          This function provides delay time in milli sec
  * @param  msec : Value of delay required in milli sec
  * @retval None
  */
void USB_OTG_BSP_mDelay (const uint32_t msec)
{
    delay_ms(msec);
//    BSP_OS_TimeDlyMs(msec);
}



/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

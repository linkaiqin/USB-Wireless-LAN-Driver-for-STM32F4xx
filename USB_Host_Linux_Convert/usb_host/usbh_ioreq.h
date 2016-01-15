/**
  ******************************************************************************
  * @file    usbh_ioreq.h
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    19-March-2012
  * @brief   Header file for usbh_ioreq.c
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

/* Define to prevent recursive  ----------------------------------------------*/
#ifndef __USBH_IOREQ_H
#define __USBH_IOREQ_H

/* Includes ------------------------------------------------------------------*/
#include "usb_conf.h"
#include "usbh_core.h"
#include "usbh_def.h"

/** @addtogroup USBH_LIB
  * @{
  */

/** @addtogroup USBH_LIB_CORE
* @{
*/
  
/** @defgroup USBH_IOREQ
  * @brief This file is the header file for usbh_ioreq.c
  * @{
  */ 


/** @defgroup USBH_IOREQ_Exported_Defines
  * @{
  */

/**
  * @}
  */ 


/** @defgroup USBH_IOREQ_Exported_Types
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBH_IOREQ_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBH_IOREQ_Exported_Variables
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBH_IOREQ_Exported_FunctionsPrototype
  * @{
  */

struct usb_device;

int USBH_CtlSendData (struct usb_device *dev, 
                                uint8_t *buff, 
                                uint16_t length,
                                struct usb_host_channel *ch);

int USBH_CtlReceiveData(struct usb_device *dev, 
                                uint8_t *buff, 
                                uint16_t length,
                                struct usb_host_channel *ch);

int USBH_BulkIntReceiveData(struct usb_device *dev, 
                                uint8_t *buff, 
                                uint16_t length,
                                struct usb_host_channel *ch);


int USBH_BulkIntSendData (struct usb_device *dev, 
                                uint8_t *buff, 
                                uint16_t length,
                                struct usb_host_channel *ch);


int USBH_IsocReceiveData(struct usb_device *dev, 
                            uint8_t *buff, 
                            uint32_t length,
                            struct usb_host_channel *ch);



int USBH_IsocSendData(struct usb_device *dev, 
                        uint8_t *buff, 
                        uint32_t length,
                        struct usb_host_channel *ch);


/**
  * @}
  */ 

#endif /* __USBH_IOREQ_H */

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



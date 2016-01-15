/** 
  ******************************************************************************
  * @file    usbh_ioreq.c 
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    19-March-2012
  * @brief   This file handles the issuing of the USB transactions
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
#include "os.h"
#include "usb_hcd.h"
#include "usbh_debug.h"
#include "usbh_linux.h"
#include "usbh_ioreq.h"
/** @addtogroup USBH_LIB
  * @{
  */

/** @addtogroup USBH_LIB_CORE
* @{
*/
  
/** @defgroup USBH_IOREQ 
  * @brief This file handles the standard protocol processing (USB v2.0)
  * @{
  */


/** @defgroup USBH_IOREQ_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 
 

/** @defgroup USBH_IOREQ_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 



/** @defgroup USBH_IOREQ_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBH_IOREQ_Private_Variables
  * @{
  */ 
/**
  * @}
  */ 


/**
  * @brief  USBH_CtlSendData
  *         Sends a data Packet to the Device
  * @param  pdev: Selected device
  * @param  buff: Buffer pointer from which the Data will be sent to Device
  * @param  length: Length of the data to be sent
  * @param  hc_num: Host channel Number
  * @retval Status
  */
int USBH_CtlSendData (struct usb_device *dev, 
                                uint8_t *buff, 
                                uint16_t length,
                                struct usb_host_channel *ch)
{

//    printf("USBH_CtlSendData length:%d\r\n",length);
    if(ch->hc_num >= USBH_CHANNEL_MAX)
    {
        usb_halt(dev,"USBH_CtlSendData hc_num:%d out of range\r\n",ch->hc_num);
    }

    if(dev->USB_OTG_Core->host.hc[ch->hc_num].ep_is_in != 0)
    {
        dev->USB_OTG_Core->host.hc[ch->hc_num].ep_is_in = 0;
        USB_OTG_HC_Init(dev->USB_OTG_Core, ch->hc_num);  
    }
    
  dev->USB_OTG_Core->host.hc[ch->hc_num].xfer_buff = buff;
  dev->USB_OTG_Core->host.hc[ch->hc_num].xfer_len = length;
 
  if ( length == 0 )
  { /* For Status OUT stage, Length==0, Status Out PID = 1 */
    usb_settoggle(dev, dev->USB_OTG_Core->host.hc[ch->hc_num].ep_num, 1, 1);
  }
 
 /* Set the Data Toggle bit as per the Flag */
  if (usb_gettoggle(dev, dev->USB_OTG_Core->host.hc[ch->hc_num].ep_num, 1) == 0)
  { /* Put the PID 0 */
      dev->USB_OTG_Core->host.hc[ch->hc_num].data_pid = HC_PID_DATA0;    
  }
 else
 { /* Put the PID 1 */
      dev->USB_OTG_Core->host.hc[ch->hc_num].data_pid = HC_PID_DATA1 ;
 }

  return HCD_SubmitRequest (dev->USB_OTG_Core , ch);   
}


/**
  * @brief  USBH_CtlReceiveData
  *         Receives the Device Response to the Setup Packet
  * @param  pdev: Selected device
  * @param  buff: Buffer pointer in which the response needs to be copied
  * @param  length: Length of the data to be received
  * @param  hc_num: Host channel Number
  * @retval Status. 
  */
int USBH_CtlReceiveData(struct usb_device *dev, 
                                uint8_t* buff, 
                                uint16_t length,
                                struct usb_host_channel *ch)
{
//    printf("USBH_CtlReceiveData length:%d\r\n",length);
    if(ch->hc_num >= USBH_CHANNEL_MAX)
    {
        usb_halt(dev,"hc_num:%d out of range\r\n",ch->hc_num);
    }
    
    if(dev->USB_OTG_Core->host.hc[ch->hc_num].ep_is_in != 1)
    {
        dev->USB_OTG_Core->host.hc[ch->hc_num].ep_is_in = 1;
        USB_OTG_HC_Init(dev->USB_OTG_Core, ch->hc_num);  
    }

  if (length == 0 )
  { /* For Status IN stage, Length==0, Status IN PID = 1 */
    usb_settoggle(dev, dev->USB_OTG_Core->host.hc[ch->hc_num].ep_num, 0, 1);
  }


  if (usb_gettoggle(dev, dev->USB_OTG_Core->host.hc[ch->hc_num].ep_num, 0) == 0)
  { /* Put the PID 0 */
       dev->USB_OTG_Core->host.hc[ch->hc_num].data_pid = HC_PID_DATA0;    
  }
  else
  { /* Put the PID 1 */
       dev->USB_OTG_Core->host.hc[ch->hc_num].data_pid = HC_PID_DATA1 ;
  }

  dev->USB_OTG_Core->host.hc[ch->hc_num].xfer_buff = buff;
  dev->USB_OTG_Core->host.hc[ch->hc_num].xfer_len = length;  

    
  
  return HCD_SubmitRequest (dev->USB_OTG_Core , ch);
  
}



/**
  * @brief  USBH_BulkSendData
  *         Sends the Bulk Packet to the device
  * @param  pdev: Selected device
  * @param  buff: Buffer pointer from which the Data will be sent to Device
  * @param  length: Length of the data to be sent
  * @param  hc_num: Host channel Number
  * @retval Status
  */
int USBH_BulkIntSendData (struct usb_device *dev, 
                                uint8_t *buff, 
                                uint16_t length,
                                struct usb_host_channel *ch)
{ 
    if(ch->hc_num >= USBH_CHANNEL_MAX)
    {
        usb_halt(dev,"hc_num:%d out of range\r\n",ch->hc_num);
    }
  dev->USB_OTG_Core->host.hc[ch->hc_num].ep_is_in = 0;
  dev->USB_OTG_Core->host.hc[ch->hc_num].xfer_buff = buff;
  dev->USB_OTG_Core->host.hc[ch->hc_num].xfer_len = length;  
  if(dev->USB_OTG_Core->host.hc[ch->hc_num].ep_type == EP_TYPE_INTR)
  {
    int max = dev->USB_OTG_Core->host.hc[ch->hc_num].max_packet;
    dev->USB_OTG_Core->host.hc[ch->hc_num].multi_count = (length + max - 1)/max;
  }

  
 /* Set the Data Toggle bit as per the Flag */
  if (usb_gettoggle(dev, dev->USB_OTG_Core->host.hc[ch->hc_num].ep_num, 1) == 0)
  { /* Put the PID 0 */
      dev->USB_OTG_Core->host.hc[ch->hc_num].data_pid = HC_PID_DATA0;    
  }
 else
 { /* Put the PID 1 */
      dev->USB_OTG_Core->host.hc[ch->hc_num].data_pid = HC_PID_DATA1 ;
 }

//    printf("USBH_BulkIntSendData data_pid:%d\r\n",pdev->host.hc[hc_num].data_pid);
    
  return HCD_SubmitRequest (dev->USB_OTG_Core, ch);   
}


/**
  * @brief  USBH_BulkReceiveData
  *         Receives IN bulk packet from device
  * @param  pdev: Selected device
  * @param  buff: Buffer pointer in which the received data packet to be copied
  * @param  length: Length of the data to be received
  * @param  hc_num: Host channel Number
  * @retval Status. 
  */
int USBH_BulkIntReceiveData(struct usb_device *dev, 
                                uint8_t *buff, 
                                uint16_t length,
                                struct usb_host_channel *ch)
{
    if(ch->hc_num >= USBH_CHANNEL_MAX)
    {
        USBH_DBG("USBH_BulkReceiveData hc_num:%d out of range\r\n",ch->hc_num);
        return -1;
    }    
  dev->USB_OTG_Core->host.hc[ch->hc_num].ep_is_in = 1;   
  dev->USB_OTG_Core->host.hc[ch->hc_num].xfer_buff = buff;
  dev->USB_OTG_Core->host.hc[ch->hc_num].xfer_len = length;
  if(dev->USB_OTG_Core->host.hc[ch->hc_num].ep_type == EP_TYPE_INTR)
  {
    int max = dev->USB_OTG_Core->host.hc[ch->hc_num].max_packet;
    dev->USB_OTG_Core->host.hc[ch->hc_num].multi_count = (length + max - 1)/max;
  }  

  if(usb_gettoggle(dev, dev->USB_OTG_Core->host.hc[ch->hc_num].ep_num, 0) == 0)
  {
    dev->USB_OTG_Core->host.hc[ch->hc_num].data_pid = HC_PID_DATA0;
  }
  else
  {
    dev->USB_OTG_Core->host.hc[ch->hc_num].data_pid = HC_PID_DATA1;
  }

  return HCD_SubmitRequest (dev->USB_OTG_Core, ch);
}


int USBH_IsocSendData(struct usb_device *dev, 
                                uint8_t *buff, 
                                uint32_t length,
                                struct usb_host_channel *ch)
{ 
  int multi = (length + dev->USB_OTG_Core->host.hc[ch->hc_num].max_packet - 1) \
                    /dev->USB_OTG_Core->host.hc[ch->hc_num].max_packet;  
  
  dev->USB_OTG_Core->host.hc[ch->hc_num].ep_is_in = 0;  
  dev->USB_OTG_Core->host.hc[ch->hc_num].xfer_buff = buff;
  dev->USB_OTG_Core->host.hc[ch->hc_num].xfer_len = length;
  dev->USB_OTG_Core->host.hc[ch->hc_num].multi_count = multi;
  //if mult is more than one, we can sure usb device is in DMA mode.See usb_submit_urb()
  if(multi == 1)
    dev->USB_OTG_Core->host.hc[ch->hc_num].data_pid = HC_PID_DATA0;
  else
    dev->USB_OTG_Core->host.hc[ch->hc_num].data_pid = HC_PID_MDATA;    
  
  return HCD_SubmitRequest (dev->USB_OTG_Core, ch);  
}



/**
  * @brief  USBH_IsocReceiveData
  *         Receives the Device Response to the Isochronous IN token
  * @param  pdev: Selected device
  * @param  buff: Buffer pointer in which the response needs to be copied
  * @param  length: Length of the data to be received
  * @param  hc_num: Host channel Number
  * @retval Status. 
  */
int USBH_IsocReceiveData(struct usb_device *dev, 
                                uint8_t *buff, 
                                uint32_t length,
                                struct usb_host_channel *ch)
{    
  int multi = (length + dev->USB_OTG_Core->host.hc[ch->hc_num].max_packet - 1) \
                    /dev->USB_OTG_Core->host.hc[ch->hc_num].max_packet;  
  
  dev->USB_OTG_Core->host.hc[ch->hc_num].ep_is_in = 1;  
  dev->USB_OTG_Core->host.hc[ch->hc_num].xfer_buff = buff;
  dev->USB_OTG_Core->host.hc[ch->hc_num].xfer_len = length;
  dev->USB_OTG_Core->host.hc[ch->hc_num].multi_count = multi;
  if(multi == 1)
    dev->USB_OTG_Core->host.hc[ch->hc_num].data_pid = HC_PID_DATA0;
  else if(multi == 2)
    dev->USB_OTG_Core->host.hc[ch->hc_num].data_pid = HC_PID_DATA1;  
  else //multi == 3
    dev->USB_OTG_Core->host.hc[ch->hc_num].data_pid = HC_PID_DATA2;  

  return HCD_SubmitRequest (dev->USB_OTG_Core, ch);  
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

/**
* @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/




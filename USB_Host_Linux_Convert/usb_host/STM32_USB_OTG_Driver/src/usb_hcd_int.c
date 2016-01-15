/**
  ******************************************************************************
  * @file    usb_hcd_int.c
  * @author  MCD Application Team
  * @version V2.1.0
  * @date    19-March-2012
  * @brief   Host driver interrupt subroutines
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
#include "usb_core.h"
#include "usbh_core.h"
#include "usbh_debug.h"
#include "usbh_linux.h"
#include "usb_defines.h"
#include "usb_hcd_int.h"
#include "errno.h"





#if defined   (__CC_ARM) /*!< ARM Compiler */
#pragma O0
#elif defined (__GNUC__) /*!< GNU Compiler */
#pragma GCC optimize ("O0")
#elif defined  (__TASKING__) /*!< TASKING Compiler */ 
#pragma optimize=0                          

#endif /* __CC_ARM */

/** @addtogroup USB_OTG_DRIVER
* @{
*/

/** @defgroup USB_HCD_INT 
* @brief This file contains the interrupt subroutines for the Host mode.
* @{
*/


/** @defgroup USB_HCD_INT_Private_Defines
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USB_HCD_INT_Private_TypesDefinitions
* @{
*/ 
/**
* @}
*/ 



/** @defgroup USB_HCD_INT_Private_Macros
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USB_HCD_INT_Private_Variables
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USB_HCD_INT_Private_FunctionPrototypes
* @{
*/ 

static uint32_t USB_OTG_USBH_handle_sof_ISR(USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_port_ISR(USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_hc_ISR (USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_hc_n_In_ISR (USB_OTG_CORE_HANDLE *pdev ,
                                                 uint32_t num);
static uint32_t USB_OTG_USBH_handle_hc_n_Out_ISR (USB_OTG_CORE_HANDLE *pdev , 
                                                  uint32_t num);
static uint32_t USB_OTG_USBH_handle_rx_qlvl_ISR (USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_nptxfempty_ISR (USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_ptxfempty_ISR (USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_Disconnect_ISR (USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_IncompletePeriodicXfer_ISR (USB_OTG_CORE_HANDLE *pdev);

/**
* @}
*/ 


/** @defgroup USB_HCD_INT_Private_Functions
* @{
*/ 

/**
* @brief  HOST_Handle_ISR 
*         This function handles all USB Host Interrupts
* @param  pdev: Selected device
* @retval status 
*/

int USBH_OTG_ISR_Handler (void *otg_handle)
{
  USB_OTG_GINTSTS_TypeDef  gintsts;
  USB_OTG_CORE_HANDLE *pdev = otg_handle;
  uint32_t retval = 0;
  
  gintsts.d32 = 0;
  
  /* Check if HOST Mode */
  if (USB_OTG_IsHostMode(pdev))
  {
    gintsts.d32 = USB_OTG_ReadCoreItr(pdev);
    if (!gintsts.d32)
    {
      return 0;
    }
    
    if (gintsts.b.sofintr)
    {
      retval |= USB_OTG_USBH_handle_sof_ISR (pdev);
    }
    
    if (gintsts.b.rxstsqlvl)
    {
      retval |= USB_OTG_USBH_handle_rx_qlvl_ISR (pdev);
    }
    
    if (gintsts.b.nptxfempty)
    {
      retval |= USB_OTG_USBH_handle_nptxfempty_ISR (pdev);
    }
    
    if (gintsts.b.ptxfempty)
    {
      retval |= USB_OTG_USBH_handle_ptxfempty_ISR (pdev);
    }    
    
    if (gintsts.b.hcintr)
    {
      retval |= USB_OTG_USBH_handle_hc_ISR (pdev);
    }
    
    if (gintsts.b.portintr)
    {
      retval |= USB_OTG_USBH_handle_port_ISR (pdev);
    }
    
    if (gintsts.b.disconnect)
    {
      retval |= USB_OTG_USBH_handle_Disconnect_ISR (pdev);  
      
    }
    
    if (gintsts.b.incomplisoout)
    {
      retval |= USB_OTG_USBH_handle_IncompletePeriodicXfer_ISR (pdev);
    }
    
    
  }
  return retval;
}

/**
* @brief  USB_OTG_USBH_handle_hc_ISR 
*         This function indicates that one or more host channels has a pending
* @param  pdev: Selected device
* @retval status 
*/
static uint32_t USB_OTG_USBH_handle_hc_ISR (USB_OTG_CORE_HANDLE *pdev)
{
  USB_OTG_HAINT_TypeDef        haint;
  USB_OTG_HCCHAR_TypeDef       hcchar;
  uint32_t i = 0;
  uint32_t retval = 0;
  
  /* Clear appropriate bits in HCINTn to clear the interrupt bit in
  * GINTSTS */
  
  haint.d32 = USB_OTG_ReadHostAllChannels_intr(pdev);
  
  for (i = 0; i < pdev->cfg.host_channels ; i++)
  {
    if (haint.b.chint & (1 << i))
    {
      hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[i]->HCCHAR);
      
      if (hcchar.b.epdir)
      {
        retval |= USB_OTG_USBH_handle_hc_n_In_ISR (pdev, i);
      }
      else
      {
        retval |=  USB_OTG_USBH_handle_hc_n_Out_ISR (pdev, i);
      }
    }
  }
  
  return retval;
}

/**
* @brief  USB_OTG_otg_hcd_handle_sof_intr 
*         Handles the start-of-frame interrupt in host mode.
* @param  pdev: Selected device
* @retval status 
*/
static uint32_t USB_OTG_USBH_handle_sof_ISR (USB_OTG_CORE_HANDLE *pdev)
{
  struct usb_host_endpoint *ep,*next;
  USB_OTG_GINTSTS_TypeDef      gintsts;
  struct usb_device *usb_dev = pdev->parent;
  OS_ERR err;
  struct urb *urb;

  gintsts.d32 = 0;

  /* Clear interrupt */
  gintsts.b.sofintr = 1;
  USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GINTSTS, gintsts.d32);
  
  usb_get_current_frame_number(usb_dev);

  if(list_empty(&usb_dev->period_xfer_mgt.frame_wait_ep_list))
    return 1;
  
  list_for_each_entry_safe(ep, next, &usb_dev->period_xfer_mgt.frame_wait_ep_list, frame_wait_ep_list)
  {
      if(list_empty(&ep->urb_list))
        usb_halt(usb_dev, "ep->urb_list is empty\r\n");
      
      urb = list_first_entry(&ep->urb_list, struct urb, urb_list);
      if(urb->start_frame - 1 <= usb_dev->frame_number)
      {
       OSTaskQPost(&USBHTaskTCB, ep, USBH_PROCESS_ISOC_SOF, OS_OPT_POST_FIFO, &err);      
        if(err != OS_ERR_NONE)
        {
            usb_dbg(usb_dev, "%s OSTaskQPost failed:%d\r\n",__func__, err);
            break;
        }
        
        list_del_init(&ep->frame_wait_ep_list);
      }
  }

  if(list_empty(&usb_dev->period_xfer_mgt.frame_wait_ep_list))
  {
    USB_OTG_GINTMSK_TypeDef  intmsk;

    intmsk.d32 = 0;
    intmsk.b.sofintr    = 1;  
    USB_OTG_MODIFY_REG32(&pdev->regs.GREGS->GINTMSK, intmsk.d32, 0);      
  }

  
  return 1;
}

/**
* @brief  USB_OTG_USBH_handle_Disconnect_ISR 
*         Handles disconnect event.
* @param  pdev: Selected device
* @retval status 
*/
static uint32_t USB_OTG_USBH_handle_Disconnect_ISR (USB_OTG_CORE_HANDLE *pdev)
{
  USB_OTG_GINTSTS_TypeDef      gintsts;
  struct usb_device *usb_dev;
//   OS_ERR err;
    
  gintsts.d32 = 0;

  usb_dev = pdev->parent;

  if(usb_dev->state != USB_STATE_NOTATTACHED)
  {
      pdev->host.ConnSts = USB_STATE_NOTATTACHED;
  }

  /* Clear interrupt */
  gintsts.b.disconnect = 1;
  USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GINTSTS, gintsts.d32);
  
  return 1;
}
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize = none
#endif /* __CC_ARM */
/**
* @brief  USB_OTG_USBH_handle_nptxfempty_ISR 
*         Handles non periodic tx fifo empty.
* @param  pdev: Selected device
* @retval status 
*/
#if 0 

void Notify_FiFoSpaceAvail(struct urbs_xfer_mgt *xfer_mgt)
{
    OS_ERR err;
    struct usbh_task_req *task_req;    
    struct usb_host_channel *ch;

    if(!list_empty(&xfer_mgt->out_hc_wait_list)
      && list_empty(&xfer_mgt->out_hc_process_list))
    {
        ch = list_first_entry(&xfer_mgt->out_hc_wait_list, struct usb_host_channel, hc_list);
        list_move_tail(&ch->hc_list, &xfer_mgt->out_hc_process_list);
        
        task_req = OSMemGet(&ReqQueueMem,&err);
        if((err != OS_ERR_NONE) || (task_req == NULL))
        {
            USBH_DBG("Notify_FiFoSpaceAvail OSMemGet ReqQueueMem Faild %d\r\n",err);
            return;
        }
        
        task_req->req_type = USBH_TASK_REQ_CH_PROCESS;
        task_req->req_msg = USBH_TASK_REQ_FROM_INT;
        INIT_LIST_HEAD(&task_req->list);
        list_add_tail(&task_req->list, &ch->urb_task_req_list);
        
        usb_get_channel(ch);
        OSTaskQPost(&USBHTaskTCB, ch, USBH_PROCESS_CHANNEL, OS_OPT_POST_FIFO, &err);
        if(err != OS_ERR_NONE)
        {
            USBH_DBG("Notify_FiFoSpaceAvail OSTaskQPost ReqQueueMem Faild %d\r\n",err);
            usb_put_channel(ch);
            list_del(&task_req->list);
            OSMemPut(&ReqQueueMem,task_req, &err);
            list_move(&ch->hc_list, &xfer_mgt->out_hc_wait_list);
        }
    }
    else
    {
//         USBH_DBG("Notify_FiFoSpaceAvail do none\r\n");
    }

}
#endif

static uint32_t USB_OTG_USBH_handle_nptxfempty_ISR (USB_OTG_CORE_HANDLE *pdev)
{
#if 0     
  USB_OTG_GINTMSK_TypeDef      intmsk;
  USB_OTG_HNPTXSTS_TypeDef     hnptxsts; 
  USB_OTG_FSIZ_TypeDef nptxfifosize;
//  uint16_t                     len_words , len; 
  struct urbs_xfer_mgt *xfer_mgt;
  
  
  nptxfifosize.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->DIEPTXF0_HNPTXFSIZ); 
  hnptxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->HNPTXSTS);

  intmsk.d32 = 0;
  intmsk.b.nptxfempty = 1;
  USB_OTG_MODIFY_REG32( &pdev->regs.GREGS->GINTMSK, intmsk.d32, 0);    

  xfer_mgt = &pdev->parent->nperiod_xfer_mgt;
  
  if(hnptxsts.b.nptxfspcavail >= nptxfifosize.b.depth/2)
  {
      Notify_FiFoSpaceAvail(xfer_mgt);
  }
  

 
  len_words = (pdev->host.hc[hnptxsts.b.nptxqtop.chnum].xfer_len + 3) / 4;
  
  while ((hnptxsts.b.nptxfspcavail > len_words)&&
         (pdev->host.hc[hnptxsts.b.nptxqtop.chnum].xfer_len != 0))
  {
    
    len = hnptxsts.b.nptxfspcavail * 4;
    
    if (len > pdev->host.hc[hnptxsts.b.nptxqtop.chnum].xfer_len)
    {
      /* Last packet */
      len = pdev->host.hc[hnptxsts.b.nptxqtop.chnum].xfer_len;
      
      intmsk.d32 = 0;
      intmsk.b.nptxfempty = 1;
      USB_OTG_MODIFY_REG32( &pdev->regs.GREGS->GINTMSK, intmsk.d32, 0);       
    }
    
    len_words = (pdev->host.hc[hnptxsts.b.nptxqtop.chnum].xfer_len + 3) / 4;
    
    USB_OTG_WritePacket (pdev , pdev->host.hc[hnptxsts.b.nptxqtop.chnum].xfer_buff, hnptxsts.b.nptxqtop.chnum, len);
    
    pdev->host.hc[hnptxsts.b.nptxqtop.chnum].xfer_buff  += len;
    pdev->host.hc[hnptxsts.b.nptxqtop.chnum].xfer_len   -= len;
    pdev->host.hc[hnptxsts.b.nptxqtop.chnum].xfer_count  += len; 
    
    hnptxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->HNPTXSTS);
  }  
#endif  
  return 1;
}
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize = none
#endif /* __CC_ARM */
/**
* @brief  USB_OTG_USBH_handle_ptxfempty_ISR 
*         Handles periodic tx fifo empty
* @param  pdev: Selected device
* @retval status 
*/
static uint32_t USB_OTG_USBH_handle_ptxfempty_ISR (USB_OTG_CORE_HANDLE *pdev)
{
#if 0     
  USB_OTG_GINTMSK_TypeDef      intmsk;
  USB_OTG_HPTXSTS_TypeDef      hptxsts; 
//  uint16_t                     len_words , len; 
  USB_OTG_FSIZ_TypeDef ptxfifosize;
  struct urbs_xfer_mgt *xfer_mgt;


  hptxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.HREGS->HPTXSTS);
  ptxfifosize.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->HPTXFSIZ);  
    
  intmsk.d32 = 0;
  intmsk.b.ptxfempty = 1;
  USB_OTG_MODIFY_REG32( &pdev->regs.GREGS->GINTMSK, intmsk.d32, 0); 

  xfer_mgt = &pdev->parent->period_xfer_mgt;

  if(hptxsts.b.ptxfspcavail >= ptxfifosize.b.depth/2)
  {
     Notify_FiFoSpaceAvail(xfer_mgt);
  } 


 
  len_words = (pdev->host.hc[hptxsts.b.ptxqtop.chnum].xfer_len + 3) / 4;
  
  while ((hptxsts.b.ptxfspcavail > len_words)&&
         (pdev->host.hc[hptxsts.b.ptxqtop.chnum].xfer_len != 0))    
  {
    
    len = hptxsts.b.ptxfspcavail * 4;
    
    if (len > pdev->host.hc[hptxsts.b.ptxqtop.chnum].xfer_len)
    {
      len = pdev->host.hc[hptxsts.b.ptxqtop.chnum].xfer_len;
      /* Last packet */
      intmsk.d32 = 0;
      intmsk.b.ptxfempty = 1;
      USB_OTG_MODIFY_REG32( &pdev->regs.GREGS->GINTMSK, intmsk.d32, 0); 
    }
    
    len_words = (pdev->host.hc[hptxsts.b.ptxqtop.chnum].xfer_len + 3) / 4;
    
    USB_OTG_WritePacket (pdev , pdev->host.hc[hptxsts.b.ptxqtop.chnum].xfer_buff, hptxsts.b.ptxqtop.chnum, len);
    
    pdev->host.hc[hptxsts.b.ptxqtop.chnum].xfer_buff  += len;
    pdev->host.hc[hptxsts.b.ptxqtop.chnum].xfer_len   -= len;
    pdev->host.hc[hptxsts.b.ptxqtop.chnum].xfer_count  += len; 
    
    hptxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.HREGS->HPTXSTS);
  }  
#endif  
  return 1;
}

/**
* @brief  USB_OTG_USBH_handle_port_ISR 
*         This function determines which interrupt conditions have occurred
* @param  pdev: Selected device
* @retval status 
*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize = none
#endif /* __CC_ARM */
static uint32_t USB_OTG_USBH_handle_port_ISR (USB_OTG_CORE_HANDLE *pdev)
{
  USB_OTG_HPRT0_TypeDef  hprt0;
  USB_OTG_HPRT0_TypeDef  hprt0_dup;
  USB_OTG_HCFG_TypeDef   hcfg;    
  uint32_t retval = 0;
  
  hcfg.d32 = 0;
  hprt0.d32 = 0;
  hprt0_dup.d32 = 0;
  
  hprt0.d32 = USB_OTG_READ_REG32(pdev->regs.HPRT0);
  hprt0_dup.d32 = USB_OTG_READ_REG32(pdev->regs.HPRT0);
  
  /* Clear the interrupt bits in GINTSTS */
  
  hprt0_dup.b.prtena = 0;
  hprt0_dup.b.prtconndet = 0;
  hprt0_dup.b.prtenchng = 0;
  hprt0_dup.b.prtovrcurrchng = 0;
  
  /* Port Connect Detected */
  if (hprt0.b.prtconndet)
  {
    struct usb_device *usb_dev;

    usb_dev = pdev->parent;

    printf("hprt0.b.prtconndet\r\n");
    hprt0_dup.b.prtconndet = 1;

    if(usb_dev->state == USB_STATE_NOTATTACHED)
        pdev->host.ConnSts = USB_STATE_ATTACHED;
    
    retval |= 1;
  }
  
  /* Port Enable Changed */
  if (hprt0.b.prtenchng)
  {
    hprt0_dup.b.prtenchng = 1;
    
    if (hprt0.b.prtena == 1)
    {      
      if ((hprt0.b.prtspd == HPRT0_PRTSPD_LOW_SPEED) ||
          (hprt0.b.prtspd == HPRT0_PRTSPD_FULL_SPEED))
      {
        
        hcfg.d32 = USB_OTG_READ_REG32(&pdev->regs.HREGS->HCFG);
        
        if (hprt0.b.prtspd == HPRT0_PRTSPD_LOW_SPEED)
        {
          USB_OTG_WRITE_REG32(&pdev->regs.HREGS->HFIR, 6000 );
          if (hcfg.b.fslspclksel != HCFG_6_MHZ)
          {
            if(pdev->cfg.phy_itface  == USB_OTG_EMBEDDED_PHY)
            {
              USB_OTG_InitFSLSPClkSel(pdev ,HCFG_6_MHZ );
            }
          }
        }
        else
        {
          
          USB_OTG_WRITE_REG32(&pdev->regs.HREGS->HFIR, 48000 );            
          if (hcfg.b.fslspclksel != HCFG_48_MHZ)
          {
            USB_OTG_InitFSLSPClkSel(pdev ,HCFG_48_MHZ );
          }
        }
      }
    }
  }
  /* Overcurrent Change Interrupt */
  if (hprt0.b.prtovrcurrchng)
  {
    hprt0_dup.b.prtovrcurrchng = 1;
    retval |= 1;
  }

  /* Clear Port Interrupts */
  USB_OTG_WRITE_REG32(pdev->regs.HPRT0, hprt0_dup.d32);
  
  return retval;
}
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize = none
#endif /* __CC_ARM */
/**
* @brief  USB_OTG_USBH_handle_hc_n_Out_ISR 
*         Handles interrupt for a specific Host Channel
* @param  pdev: Selected device
* @param  hc_num: Channel number
* @retval status 
*/


void USB_OTG_USBH_SendIsocPacket(USB_OTG_CORE_HANDLE *pdev, uint32_t num, unsigned char *buffer, int length)
{
  USB_OTG_HCCHAR_TypeDef   hcchar;
  USB_OTG_HCTSIZn_TypeDef  hctsiz;
  
  int max = pdev->host.hc[num].max_packet;
  int mult = (length + max - 1)/max;

  hctsiz.d32 = 0;
  hcchar.d32 = 0;

  pdev->host.hc[num].xfer_buff = buffer;
  /* Initialize the HCTSIZn register */
  pdev->host.hc[num].xfer_len = length;
  hctsiz.b.xfersize = length;
  hctsiz.b.pktcnt = mult;
  if(mult == 1)
    hctsiz.b.pid = HC_PID_DATA0;
  else
    hctsiz.b.pid = HC_PID_MDATA;

  USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[num]->HCTSIZ, hctsiz.d32);

  if (pdev->cfg.dma_enable == 1)
  {
    USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[num]->HCDMA, (unsigned int)pdev->host.hc[num].xfer_buff);
  }
  
  hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[num]->HCCHAR);
  hcchar.b.oddfrm = USB_OTG_IsEvenFrame(pdev);
  hcchar.b.multicnt = mult;
    
  /* Set host channel enable */
  hcchar.b.chen = 1;
  hcchar.b.chdis = 0; 
  USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[num]->HCCHAR, hcchar.d32);

  if(pdev->cfg.dma_enable == 0) /* Slave mode */
  {  
    USB_OTG_WritePacket(pdev, 
                          buffer, 
                          num, 
                          length);
  }
}


uint32_t USB_OTG_USBH_handle_hc_n_Out_ISR (USB_OTG_CORE_HANDLE *pdev , uint32_t num)
{
  
  USB_OTG_HCINTn_TypeDef     hcint;
  USB_OTG_HCINTMSK_TypeDef  hcintmsk;
  USB_OTG_HC_REGS *hcreg;
  USB_OTG_HCCHAR_TypeDef     hcchar; 
  struct usbh_task_req *task_req;
  struct usb_host_channel *ch;
  OS_ERR err;


    
  hcreg = pdev->regs.HC_REGS[num];
  hcint.d32 = USB_OTG_READ_REG32(&hcreg->HCINT);
  hcintmsk.d32 = USB_OTG_READ_REG32(&hcreg->HCINTMSK);
  hcint.d32 = hcint.d32 & hcintmsk.d32;
  
  hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[num]->HCCHAR);

  
  if (hcint.b.ahberr)
  {
    printf("out_isr:hcint.b.ahberr at 0x%x\r\n",USB_OTG_READ_REG32(&pdev->regs.HC_REGS[num]->HCDMA));
      
    CLEAR_HC_INT(hcreg ,ahberr);
    UNMASK_HOST_INT_CHH (num);

    USB_OTG_HC_Halt(pdev, num);
    pdev->host.HC_Status[num] = HC_XACTERR;
  } 
  else if (hcint.b.ack)
  {
    USB_OTG_HCTSIZn_TypeDef  hctsiz;
    int remain_length;
    
    hctsiz.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[num]->HCTSIZ);
    remain_length = hctsiz.b.xfersize;
    
//    printf("hcint.b.ack remain_length:%d  hctsiz.b.pktcnt:%d\r\n",remain_length,hctsiz.b.pktcnt);
   
    /* Write packet into the Tx FIFO. */
    USB_OTG_WritePacket(pdev, 
                        pdev->host.hc[num].xfer_buff + pdev->host.hc[num].xfer_len - remain_length, 
                        num, 
                        DEF_MIN(remain_length, pdev->host.hc[num].max_packet));
    
    //last packet, mask ack interrupt.
    if(remain_length <= pdev->host.hc[num].max_packet)
    {
        MASK_HOST_INT_ACK(num);
    }
    
    CLEAR_HC_INT(hcreg , ack);
  }
  else if (hcint.b.frmovrun)
  {
      printf("out_isr:hcint.b.frmovrun \r\n");
    UNMASK_HOST_INT_CHH (num);
    if(pdev->cfg.dma_enable == 0)
        USB_OTG_HC_Halt(pdev, num);
    
    CLEAR_HC_INT(hcreg ,frmovrun);
  }
  else if (hcint.b.xfercompl)
  {
//     printf("out_isr:hcint.b.xfercompl \r\n");

    UNMASK_HOST_INT_CHH (num);
    if(pdev->cfg.dma_enable == 0)
       USB_OTG_HC_Halt(pdev, num);
    
    CLEAR_HC_INT(hcreg , xfercompl);
    pdev->host.HC_Status[num] = HC_XFRC;            
  }
  
  else if (hcint.b.stall)
  {
     printf("out_isr:hcint.b.stall \r\n");
    CLEAR_HC_INT(hcreg , stall);
    UNMASK_HOST_INT_CHH (num);
    if(pdev->cfg.dma_enable == 0)
        USB_OTG_HC_Halt(pdev, num);
    
    pdev->host.HC_Status[num] = HC_STALL;      
  }
  
  else if (hcint.b.nak)
  { 

//    printf("out_isr:hcint.b.nak\r\n");
    
    if(pdev->host.hc[num].do_ping == 1)
    {
        USB_OTG_HCTSIZn_TypeDef  hctsiz;
        hctsiz.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[num]->HCTSIZ);
        printf("out_isr:hctsiz.b.dopng:%d\r\n",hctsiz.b.dopng);

        //only for high speed device
        USB_OTG_HC_DoPing(pdev, num);    
    }
    else
    {    
        UNMASK_HOST_INT_CHH (num);
        USB_OTG_HC_Halt(pdev, num);
        CLEAR_HC_INT(hcreg , nak);
        pdev->host.HC_Status[num] = HC_NAK; 
    }
  }
  
  else if (hcint.b.xacterr)
  {
      printf("out_isr:hcint.b.xacterr \r\n");
    UNMASK_HOST_INT_CHH (num);
    if(pdev->cfg.dma_enable == 0)
        USB_OTG_HC_Halt(pdev, num);
    
    pdev->host.HC_Status[num] = HC_XACTERR;
    CLEAR_HC_INT(hcreg , xacterr);
  }
  else if (hcint.b.nyet)
  {
     printf("out_isr:hcint.b.nyet \r\n");

     USB_OTG_HC_DoPing(pdev, num);
     
//    pdev->host.ErrCnt[num] = 0;
//    UNMASK_HOST_INT_CHH (num);
//    USB_OTG_HC_Halt(pdev, num);
//    CLEAR_HC_INT(hcreg , nyet);
//    pdev->host.HC_Status[num] = HC_NYET;    
  }
  else if (hcint.b.datatglerr)
  { 
        printf("out_isr:hcint.b.datatglerr \r\n");
    UNMASK_HOST_INT_CHH (num);
    if(pdev->cfg.dma_enable == 0)
        USB_OTG_HC_Halt(pdev, num);
    
    CLEAR_HC_INT(hcreg , nak);   
    pdev->host.HC_Status[num] = HC_DATATGLERR;
    
    CLEAR_HC_INT(hcreg , datatglerr);
  }  
  else if (hcint.b.chhltd)
  {
    struct usb_device *usb_dev = pdev->parent;
  
    MASK_HOST_INT_CHH (num);

//      printf("out_isr:hcint.b.chhltd \r\n");
    
    if(pdev->host.HC_Status[num] == HC_XFRC)
    {    
      pdev->host.URB_State[num] = URB_DONE;  
    }
    else if(pdev->host.HC_Status[num] == HC_NAK)
    {
      pdev->host.URB_State[num] = URB_NOTREADY;         
    }          
    else if(pdev->host.HC_Status[num] == HC_STALL)
    {
      pdev->host.URB_State[num] = URB_STALL;     
    }  
    else if(pdev->host.HC_Status[num] == HC_XACTERR)
    {
        pdev->host.URB_State[num] = URB_ERROR;  
    }
    else
    {
        usb_dbg(usb_dev, "hc_n_Out_ISR unknown HC_Status %d\r\n",pdev->host.HC_Status[num]);
        return 0;
    }
    CLEAR_HC_INT(hcreg , chhltd); 

    pdev->host.HC_Status[num] = HC_IDLE; 

    
    if(num >= USBH_CHANNEL_MAX)
    {
        usb_halt(usb_dev, " num error num:%d  HC_MAX:%d\r\n",num,USBH_CHANNEL_MAX);
    }

    ch = &usb_dev->usb_ch_pool[num];
    if(ch->state == USBH_CHANNEL_STATE_IDLE)
    {
        usb_dbg(usb_dev, "hc_n_Out_ISR line:%d channel:%d is idle\r\n",__LINE__, ch->hc_num);
        return 1;
    }
    USB_OTG_HCTSIZn_TypeDef  hctsiz;
    hctsiz.d32 = USB_OTG_READ_REG32(&hcreg->HCTSIZ);
    usb_settoggle(usb_dev, pdev->host.hc[num].ep_num, 1, hctsiz.b.pid?1:0); //add by LKQ 2015.11.6
    //must check hctsiz.b.pktcnt.
    //when hctsiz.b.pktcnt is zero, hctsiz.b.xfersize may not be zero.
    if(hctsiz.b.pktcnt)
        pdev->host.XferCnt[num] = pdev->host.hc[num].xfer_len - hctsiz.b.xfersize;
    else
        pdev->host.XferCnt[num] = pdev->host.hc[num].xfer_len;


    if((hcchar.b.eptype == EP_TYPE_ISOC) 
        || (hcchar.b.eptype == EP_TYPE_INTR))
    {
        struct urb *urb = usb_dev->usb_ch_pool[num].urb;

        usb_get_current_frame_number(urb->dev);
        urb->start_frame = urb->dev->frame_number + urb->interval;             
    }
     
    if(hcchar.b.eptype == EP_TYPE_ISOC)
    {
        struct urb *urb = usb_dev->usb_ch_pool[num].urb;

        if(pdev->host.URB_State[num] == URB_DONE)
        {
            urb->iso_frame_desc[urb->iso_frame_index].status = 0;
            urb->iso_frame_desc[urb->iso_frame_index].actual_length = pdev->host.XferCnt[num];
            urb->iso_frame_desc[urb->iso_frame_index].frame_number = USB_OTG_READ_REG32(&pdev->regs.HREGS->HFNUM)&0xffff;
            urb->actual_length += urb->iso_frame_desc[urb->iso_frame_index].actual_length;
                 
            if((urb->interval == 1)
                && (urb->iso_frame_index != urb->number_of_packets - 1))
            {
                urb->iso_frame_index++;
                USB_OTG_USBH_SendIsocPacket(pdev, num, (u8 *)urb->transfer_buffer + urb->iso_frame_desc[urb->iso_frame_index].offset,
                                                urb->iso_frame_desc[urb->iso_frame_index].length);
                return 1;
            }
        }
        else //URB_ERROR
        {
            urb->iso_frame_desc[urb->iso_frame_index].status = -EILSEQ;
        }
    }

    
    task_req = OSMemGet(&ReqQueueMem,&err);
    if(err != OS_ERR_NONE)
    {
        usb_halt(usb_dev, " OSMemGet ReqQueueMem Faild %d\r\n",err);
    }
    task_req->req_type = USBH_TASK_REQ_CH_PROCESS;
    task_req->req_msg = USBH_TASK_REQ_FROM_INT;
    INIT_LIST_HEAD(&task_req->list);
    list_add_tail(&task_req->list, &ch->urb_task_req_list);

    usb_get_channel(ch);
    OSTaskQPost(&USBHTaskTCB, ch, USBH_PROCESS_CHANNEL, OS_OPT_POST_FIFO, &err);        
    if(err != OS_ERR_NONE){
        OS_ERR err2;
        usb_put_channel(ch);
        /*if failed, this channel may not be processed forever in task*/
        list_del(&task_req->list);
        OSMemPut(&ReqQueueMem, task_req, &err2);
        
        usb_halt(usb_dev, "OSTaskQPost Failed %d should not occur stop!!\r\n",err);
    }    
  }
  
  
  return 1;
}
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize = none
#endif /* __CC_ARM */
/**
* @brief  USB_OTG_USBH_handle_hc_n_In_ISR 
*         Handles interrupt for a specific Host Channel
* @param  pdev: Selected device
* @param  hc_num: Channel number
* @retval status 
*/
static void USB_OTG_USBH_ReceiveIsocPacket(USB_OTG_CORE_HANDLE *pdev, uint32_t num, unsigned char *buffer, int length)
{
  USB_OTG_HCCHAR_TypeDef   hcchar;
  USB_OTG_HCTSIZn_TypeDef  hctsiz;
  
  int max = pdev->host.hc[num].max_packet;
  int mult = (length + max - 1)/max;

  hctsiz.d32 = 0;
  hcchar.d32 = 0;


  pdev->host.hc[num].xfer_buff = buffer;
  /* Initialize the HCTSIZn register */
  pdev->host.hc[num].xfer_len = length;
  hctsiz.b.xfersize = length;
  hctsiz.b.pktcnt = mult;
  if(mult == 1)
    hctsiz.b.pid = HC_PID_DATA0;
  else if(mult == 2)
    hctsiz.b.pid = HC_PID_DATA1;  
  else //multi == 3 
    hctsiz.b.pid = HC_PID_DATA2;  
  
  USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[num]->HCTSIZ, hctsiz.d32);

  if (pdev->cfg.dma_enable == 1)
  {
    USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[num]->HCDMA, (unsigned int)pdev->host.hc[num].xfer_buff);
  }
  
  hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[num]->HCCHAR);
  hcchar.b.oddfrm = USB_OTG_IsEvenFrame(pdev);
  hcchar.b.multicnt = mult;
    
  /* Set host channel enable */
  hcchar.b.chen = 1;
  hcchar.b.chdis = 0; 
  USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[num]->HCCHAR, hcchar.d32);
}



uint32_t USB_OTG_USBH_handle_hc_n_In_ISR (USB_OTG_CORE_HANDLE *pdev , uint32_t num)
{
  USB_OTG_HCINTn_TypeDef     hcint;
  USB_OTG_HCINTMSK_TypeDef  hcintmsk;
  USB_OTG_HCCHAR_TypeDef     hcchar; 
  USB_OTG_HC_REGS *hcreg;
  struct usbh_task_req *task_req;
  struct usb_host_channel *ch;
  OS_ERR err;
  struct usb_device *usb_dev;


  usb_dev = pdev->parent;

  
  hcreg = pdev->regs.HC_REGS[num];
  hcint.d32 = USB_OTG_READ_REG32(&hcreg->HCINT);
  hcintmsk.d32 = USB_OTG_READ_REG32(&hcreg->HCINTMSK);
  hcint.d32 = hcint.d32 & hcintmsk.d32;
  hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[num]->HCCHAR);
  hcintmsk.d32 = 0;
  
  
  if (hcint.b.ahberr)
  {
    printf("in_isr:hcint.b.ahberr at 0x%x\r\n",USB_OTG_READ_REG32(&pdev->regs.HC_REGS[num]->HCDMA));
    CLEAR_HC_INT(hcreg ,ahberr);
    UNMASK_HOST_INT_CHH (num);
    USB_OTG_HC_Halt(pdev, num);
    pdev->host.HC_Status[num] = HC_XACTERR;

  }  
  else if (hcint.b.ack)
  {
    CLEAR_HC_INT(hcreg ,ack);
  }
  
  else if (hcint.b.stall)  
  {
     printf("in_isr:hcint.b.stall \r\n");
    UNMASK_HOST_INT_CHH (num);
    pdev->host.HC_Status[num] = HC_STALL; 
    CLEAR_HC_INT(hcreg , nak);   /* Clear the NAK Condition */
    CLEAR_HC_INT(hcreg , stall); /* Clear the STALL Condition */
    hcint.b.nak = 0;           /* NOTE: When there is a 'stall', reset also nak, 
                                        else, the pdev->host.HC_Status = HC_STALL
                                        will be overwritten by 'nak' in code below */
    if(pdev->cfg.dma_enable == 0)
        USB_OTG_HC_Halt(pdev, num);    
  }
  else if (hcint.b.datatglerr)
  {
    printf("in_isr:hcint.b.datatglerr \r\n");
    UNMASK_HOST_INT_CHH (num);
    if(pdev->cfg.dma_enable == 0)
        USB_OTG_HC_Halt(pdev, num);
    
    CLEAR_HC_INT(hcreg , nak);   
    pdev->host.HC_Status[num] = HC_DATATGLERR; 
    CLEAR_HC_INT(hcreg , datatglerr);
  }    


  if(hcint.b.bblerr)
  {
      printf("in_isr:hcint.b.bblerr \r\n");
  }
  else if (hcint.b.frmovrun)
  {
      printf("in_isr:hcint.b.frmovrun \r\n");
    UNMASK_HOST_INT_CHH (num);
    if(pdev->cfg.dma_enable == 0)
        USB_OTG_HC_Halt(pdev, num);
    
    CLEAR_HC_INT(hcreg ,frmovrun);
  }
  else if (hcint.b.xfercompl)
  {
    if ((hcchar.b.eptype == EP_TYPE_ISOC)||
        (hcchar.b.eptype == EP_TYPE_INTR))
    {
      hcchar.b.oddfrm  = 1;
      USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[num]->HCCHAR, hcchar.d32);         
    }
    
    pdev->host.HC_Status[num] = HC_XFRC;     
    CLEAR_HC_INT(hcreg , xfercompl);


    UNMASK_HOST_INT_CHH (num);
    //In DMA mode, USB hard will auto generates channel halt request.
    if(pdev->cfg.dma_enable == 0)
      USB_OTG_HC_Halt(pdev, num);
    
    CLEAR_HC_INT(hcreg , nak);   

  }
  else if (hcint.b.chhltd)
  {

    MASK_HOST_INT_CHH (num);

//    printf("in_isr:hcint.b.chhltd \r\n");
    
    if(pdev->host.HC_Status[num] == HC_XFRC)
    {
      pdev->host.URB_State[num] = URB_DONE;      
    }
    
    else if (pdev->host.HC_Status[num] == HC_STALL) 
    {
      pdev->host.URB_State[num] = URB_STALL;
    }   
    else if(pdev->host.HC_Status[num] == HC_NAK)//add by LKQ
    {
      pdev->host.URB_State[num] = URB_NAK;
    }
    else if((pdev->host.HC_Status[num] == HC_XACTERR) ||
            (pdev->host.HC_Status[num] == HC_DATATGLERR))
    {
      pdev->host.URB_State[num] = URB_ERROR;  
    }
    else
    {
        usb_dbg(usb_dev, "hc_n_In_ISR unknown HC_Status %d  channel num:%d\r\n",pdev->host.HC_Status[num],num);
        return 0;
    }
    
    CLEAR_HC_INT(hcreg , chhltd); 

    pdev->host.HC_Status[num] = HC_IDLE; 
       

    if(num >= USBH_CHANNEL_MAX)
    {
        usb_halt(usb_dev, " num error num:%d  HC_MAX:%d\r\n",num,USBH_CHANNEL_MAX);
    }

  
    ch = &usb_dev->usb_ch_pool[num];
    if(ch->state == USBH_CHANNEL_STATE_IDLE)
    {
        usb_dbg(usb_dev, "hc_n_In_ISR line:%d channel:%d is idle\r\n",__LINE__, ch->hc_num);
        return 1;
    }
    USB_OTG_HCTSIZn_TypeDef  hctsiz;
    hctsiz.d32 = USB_OTG_READ_REG32(&hcreg->HCTSIZ);
    usb_settoggle(usb_dev, pdev->host.hc[num].ep_num, 0, hctsiz.b.pid?1:0); //add by LKQ 2015.11.6
    pdev->host.XferCnt[num] =  pdev->host.hc[num].xfer_len - hctsiz.b.xfersize;

//    printf("XferCnt[num]:%d  pkcnt:%d len:%d xfersize:%d after dec:%d\r\n",pdev->host.XferCnt[num],hctsiz.b.pktcnt, pdev->host.hc[num].xfer_len, hctsiz.b.xfersize, pdev->host.hc[num].xfer_len - hctsiz.b.xfersize);
        
    if((hcchar.b.eptype == EP_TYPE_ISOC) 
        || (hcchar.b.eptype == EP_TYPE_INTR))
    {
        struct urb *urb = usb_dev->usb_ch_pool[num].urb;

        usb_get_current_frame_number(urb->dev);
        urb->start_frame = urb->dev->frame_number + urb->interval;             
    }
     
    if(hcchar.b.eptype == EP_TYPE_ISOC)
    {
        struct urb *urb = usb_dev->usb_ch_pool[num].urb;

        if(pdev->host.URB_State[num] == URB_DONE)
        {
            urb->iso_frame_desc[urb->iso_frame_index].status = 0;
            urb->iso_frame_desc[urb->iso_frame_index].actual_length = pdev->host.XferCnt[num];
            urb->iso_frame_desc[urb->iso_frame_index].frame_number = USB_OTG_READ_REG32(&pdev->regs.HREGS->HFNUM)&0xffff;
            urb->actual_length += urb->iso_frame_desc[urb->iso_frame_index].actual_length;
                 
            if((urb->interval == 1)
                && (urb->iso_frame_index != urb->number_of_packets - 1))
            {
                urb->iso_frame_index++;
                USB_OTG_USBH_ReceiveIsocPacket(pdev, num, (u8 *)urb->transfer_buffer + urb->iso_frame_desc[urb->iso_frame_index].offset,
                                                urb->iso_frame_desc[urb->iso_frame_index].length);
                return 1;
            }
        }
        else //URB_ERROR
        {
            urb->iso_frame_desc[urb->iso_frame_index].status = -EILSEQ;
        }
    }


    task_req = OSMemGet(&ReqQueueMem,&err);
    if(err != OS_ERR_NONE)
    {
        usb_halt(usb_dev, "OSMemGet ReqQueueMem Faild %d  HC_Status:%d\r\n",err,pdev->host.HC_Status[num]);
    }
    task_req->req_type = USBH_TASK_REQ_CH_PROCESS;
    task_req->req_msg = USBH_TASK_REQ_FROM_INT;    
    INIT_LIST_HEAD(&task_req->list);
    list_add_tail(&task_req->list, &ch->urb_task_req_list);

    usb_get_channel(ch);
    OSTaskQPost(&USBHTaskTCB, ch, USBH_PROCESS_CHANNEL, OS_OPT_POST_FIFO, &err);      
    if(err != OS_ERR_NONE){
        OS_ERR err2;
        usb_put_channel(ch);
        /*if failed, this channel may not be processed forever in task*/
        list_del(&task_req->list);
        OSMemPut(&ReqQueueMem, task_req, &err2);
        
        usb_halt(usb_dev, "OSTaskQPost Failed %d should not occur stop!!\r\n",err); 
    }   
  }    
  else if (hcint.b.xacterr)
  {
       printf("in_isr:hcint.b.xacterr \r\n");
    UNMASK_HOST_INT_CHH (num);
    pdev->host.HC_Status[num] = HC_XACTERR;
    USB_OTG_HC_Halt(pdev, num);
    CLEAR_HC_INT(hcreg , xacterr);    
    
  }
  else if (hcint.b.nak)  
  {  
//    printf("in_isr:hcint.b.nak \r\n");
    if(hcchar.b.eptype == EP_TYPE_INTR)
    {
      UNMASK_HOST_INT_CHH (num);
      USB_OTG_HC_Halt(pdev, num);
    }
    else if  ((hcchar.b.eptype == EP_TYPE_CTRL)||
              (hcchar.b.eptype == EP_TYPE_BULK))
    {
        /*In DMA mode, USB hard will auto re-activate the channel*/
        if(pdev->cfg.dma_enable == 0)
        {
            ch = &usb_dev->usb_ch_pool[num];
            if(ch->state == USBH_CHANNEL_STATE_IDLE)
            {
                usb_halt(ch->dev, "line:%d channel:%d is idle\r\n",__LINE__, ch->hc_num);
            }

            if(ch->urb->transfer_flags & URB_NO_RECV_NAK_REACTIVE_IN_INTTERUPT)
            {
               UNMASK_HOST_INT_CHH (num);
               USB_OTG_HC_Halt(pdev, num);                 
            }
            else if(ch->urb->unlinked || pdev->host.XferCnt[num])
            {
               if(pdev->host.XferCnt[num])
                    usb_dbg(ch->dev, "recive nak but XferCnt:%d > 0\r\n",pdev->host.XferCnt[num]);    
               
               
               UNMASK_HOST_INT_CHH (num);
               USB_OTG_HC_Halt(pdev, num);        
            }
            else
            {
                /* re-activate the channel  */
                hcchar.b.chen = 1;
                hcchar.b.chdis = 0;
                USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[num]->HCCHAR, hcchar.d32); 
            }
        }
    }
    pdev->host.HC_Status[num] = HC_NAK;
    CLEAR_HC_INT(hcreg , nak);   
  }


  return 1;
  
}

/**
* @brief  USB_OTG_USBH_handle_rx_qlvl_ISR 
*         Handles the Rx Status Queue Level Interrupt
* @param  pdev: Selected device
* @retval status 
*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize = none
#endif /* __CC_ARM */
static uint32_t USB_OTG_USBH_handle_rx_qlvl_ISR (USB_OTG_CORE_HANDLE *pdev)
{
  USB_OTG_GRXFSTS_TypeDef       grxsts;
  USB_OTG_GINTMSK_TypeDef       intmsk;
  USB_OTG_HCTSIZn_TypeDef       hctsiz; 
  USB_OTG_HCCHAR_TypeDef        hcchar;
  __IO uint8_t                  channelnum =0;  

  
  /* Disable the Rx Status Queue Level interrupt */
  intmsk.d32 = 0;
  intmsk.b.rxstsqlvl = 1;
  USB_OTG_MODIFY_REG32( &pdev->regs.GREGS->GINTMSK, intmsk.d32, 0);
  
  grxsts.d32 = USB_OTG_READ_REG32(&pdev->regs.GREGS->GRXSTSP);
  channelnum = grxsts.b.chnum;  
  hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[channelnum]->HCCHAR);


  switch (grxsts.b.pktsts)
  {
  case GRXSTS_PKTSTS_IN:
    /* Read the data into the host buffer. */
    if ((grxsts.b.bcnt > 0) && (pdev->host.hc[channelnum].xfer_buff != (void  *)0))
    {          
//       if(hcchar.b.eptype == EP_TYPE_BULK)         
//         printf("GRXSTS_PKTSTS_IN grxsts.b.bcnt:%d\r\n",grxsts.b.bcnt);
      USB_OTG_ReadPacket(pdev, pdev->host.hc[channelnum].xfer_buff, grxsts.b.bcnt);
      /*manage multiple Xfer */
      pdev->host.hc[grxsts.b.chnum].xfer_buff += grxsts.b.bcnt;           
      
      
      hctsiz.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[channelnum]->HCTSIZ);
      if((hctsiz.b.pktcnt > 0) && (hcchar.b.eptype != EP_TYPE_ISOC))
      {
        if((grxsts.b.bcnt == 0) || (grxsts.b.bcnt < pdev->host.hc[channelnum].max_packet))
        {
//            printf("rx_qlvl_ISR grxsts.b.bcnt:%d  total_count:%d\r\n",grxsts.b.bcnt,count);
            pdev->host.HC_Status[channelnum] = HC_XFRC;     
            /*transfer complete*/
            UNMASK_HOST_INT_CHH (channelnum);
            USB_OTG_HC_Halt(pdev, channelnum);
        }
        else
        {
            /* re-activate the channel when more packets are expected */
            hcchar.b.chen = 1;
            hcchar.b.chdis = 0;
            USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[channelnum]->HCCHAR, hcchar.d32);
        }
      }
    }
    break;
    
  case GRXSTS_PKTSTS_IN_XFER_COMP:
//      if(hcchar.b.eptype == EP_TYPE_BULK) 
//            printf("USB_OTG_USBH_handle_rx_qlvl_ISR grxsts.b.pktsts:%d\r\n",grxsts.b.pktsts);    
  case GRXSTS_PKTSTS_DATA_TOGGLE_ERR:
  case GRXSTS_PKTSTS_CH_HALTED:
  default:
    break;
  }
  
  /* Enable the Rx Status Queue Level interrupt */
  intmsk.b.rxstsqlvl = 1;
  USB_OTG_MODIFY_REG32(&pdev->regs.GREGS->GINTMSK, 0, intmsk.d32);
  return 1;
}

/**
* @brief  USB_OTG_USBH_handle_IncompletePeriodicXfer_ISR 
*         Handles the incomplete Periodic transfer Interrupt
* @param  pdev: Selected device
* @retval status 
*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize = none
#endif /* __CC_ARM */
static uint32_t USB_OTG_USBH_handle_IncompletePeriodicXfer_ISR (USB_OTG_CORE_HANDLE *pdev)
{
  
  USB_OTG_GINTSTS_TypeDef       gintsts;
  USB_OTG_HCCHAR_TypeDef        hcchar; 
  
  
  
  
  hcchar.d32 = USB_OTG_READ_REG32(&pdev->regs.HC_REGS[0]->HCCHAR);
  hcchar.b.chen = 1;
  hcchar.b.chdis = 1;
  USB_OTG_WRITE_REG32(&pdev->regs.HC_REGS[0]->HCCHAR, hcchar.d32);  
  
  gintsts.d32 = 0;
  /* Clear interrupt */
  gintsts.b.incomplisoout = 1;
  USB_OTG_WRITE_REG32(&pdev->regs.GREGS->GINTSTS, gintsts.d32);
  
  return 1;
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


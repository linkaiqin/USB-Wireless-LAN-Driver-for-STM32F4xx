/*
 *************************************************************************
 * Ralink Tech Inc.
 * 5F., No.36, Taiyuan St., Jhubei City,
 * Hsinchu County 302,
 * Taiwan, R.O.C.
 *
 * (c) Copyright 2002-2010, Ralink Technology, Inc.
 *
 * This program is free software; you can redistribute it and/or modify  *
 * it under the terms of the GNU General Public License as published by  *
 * the Free Software Foundation; either version 2 of the License, or     *
 * (at your option) any later version.                                   *
 *                                                                       *
 * This program is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 * GNU General Public License for more details.                          *
 *                                                                       *
 * You should have received a copy of the GNU General Public License     *
 * along with this program; if not, write to the                         *
 * Free Software Foundation, Inc.,                                       *
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 *                                                                       *
 *************************************************************************/


#include "rt_config.h"

#ifdef CONFIG_MULTI_CHANNEL
/* 
	==========================================================================
	Description:
		Send out a NULL frame to a specified STA at a higher TX rate. The 
		purpose is to ensure the designated client is okay to received at this
		rate.
	==========================================================================
 */
VOID RtmpEnqueueLastNullFrame(
	IN PRTMP_ADAPTER pAd,
	IN PUCHAR pAddr,
	IN UCHAR TxRate,
	IN UCHAR PID,
	IN UCHAR apidx,
    IN BOOLEAN bQosNull,
    IN BOOLEAN bEOSP,
    IN UCHAR OldUP,
    IN UCHAR PwrMgmt,
	IN UCHAR OpMode)
{
	UCHAR	NullFrame[48];
	ULONG	Length;
	PHEADER_802_11	pHeader_802_11;
	MAC_TABLE_ENTRY *pEntry = NULL;
	PAPCLI_STRUCT pApCliEntry = NULL;

	pEntry = MacTableLookup(pAd, pAddr);

	if (pEntry == NULL)
	{
		return;
	}

	NdisZeroMemory(NullFrame, 48);
	Length = sizeof(HEADER_802_11);

	pHeader_802_11 = (PHEADER_802_11) NullFrame;
	
	pHeader_802_11->FC.Type = BTYPE_DATA;
	pHeader_802_11->FC.SubType = SUBTYPE_NULL_FUNC;
	pHeader_802_11->FC.ToDs = 1;

	COPY_MAC_ADDR(pHeader_802_11->Addr1, pEntry->Addr);
	{
		COPY_MAC_ADDR(pHeader_802_11->Addr2, pAd->CurrentAddress);
		COPY_MAC_ADDR(pHeader_802_11->Addr3, pAd->CommonCfg.Bssid);
	}

	pHeader_802_11->FC.PwrMgmt = PwrMgmt;
	
	pHeader_802_11->Duration = pAd->CommonCfg.Dsifs + RTMPCalcDuration(pAd, TxRate, 14);

	/* sequence is increased in MlmeHardTx */
	pHeader_802_11->Sequence = pAd->Sequence;
	pAd->Sequence = (pAd->Sequence+1) & MAXSEQ; /* next sequence  */

	/* Prepare QosNull function frame */
	if (bQosNull)
	{
		pHeader_802_11->FC.SubType = SUBTYPE_QOS_NULL;
		
		/* copy QOS control bytes */
		NullFrame[Length]	=  0;
		NullFrame[Length+1] =  0;
		Length += 2;/* if pad with 2 bytes for alignment, APSD will fail */
	}

		HAL_KickOutNullFrameTx(pAd, MGMT_USE_QUEUE_FLAG | MGMT_USE_SPECIFIC_FLAG, NullFrame, Length);
}


VOID EnableMACTxPacket(
	IN PRTMP_ADAPTER pAd,
	IN PMAC_TABLE_ENTRY pEntry,
	IN UCHAR PwrMgmt,
	IN BOOLEAN bTxNullFrame,
	IN UCHAR QSel)
{
	UINT32 Data = 0;
	UCHAR OpMode = OPMODE_STA;



	
	RtmpEnqueueLastNullFrame(pAd,
							pEntry->Addr,
							pAd->CommonCfg.TxRate,
							pEntry->Aid,
							pEntry->apidx,
							TRUE,
							FALSE,
							0,
							PWR_ACTIVE,
							OpMode);

	RTMP_IO_READ32(pAd, PBF_CFG, &Data);

	if (QSel == FIFO_HCCA)	
	{
		Data |= 0x18;
		Data &= ~(1 << 11);
	}
	else if (QSel == FIFO_EDCA)
	{
		Data |=0x14;
		Data &= ~(1 << 10);
	}

	RTMP_IO_WRITE32(pAd, PBF_CFG, Data);

	RTMP_CLEAR_FLAG(pAd, fRTMP_ADAPTER_DISABLE_DEQUEUEPACKET);
#ifdef CONFIG_STA_SUPPORT
	if (QSel == FIFO_EDCA)
		RTMPSetTimer(&pAd->Mlme.EDCAToHCCATimer, 400);
#endif /* CONFIG_STA_SUPPORT */
}


VOID DisableMACTxPacket(
	IN PRTMP_ADAPTER pAd,
	IN PMAC_TABLE_ENTRY pEntry,
	IN UCHAR PwrMgmt,
	IN BOOLEAN bWaitACK,
	IN UCHAR QSel)
{
	UINT32 Data = 0;
	UINT32 macStatus;
	UINT32 MTxCycle;
	UCHAR OpMode = OPMODE_STA;

	RTMP_SET_FLAG(pAd, fRTMP_ADAPTER_DISABLE_DEQUEUEPACKET);


	/* Set HCCA/EDCA Q mode to manual-mode  */
	RTMP_IO_READ32(pAd, PBF_CFG, &Data);
	Data |= ((1 << 10) | (1 << 11));
	RTMP_IO_WRITE32(pAd, PBF_CFG, Data);

	if (QSel == FIFO_HCCA)
	{
		/* Polling HCCA Out-Q until empty  */
		for (MTxCycle = 0; MTxCycle < 2000; MTxCycle++)
		{
			RTMP_IO_READ32(pAd, TXRXQ_STA, &Data);

			if (((Data >> 11) & 0x1f) == 0)
				break;
			else
				RTMPusecDelay(50);
		}
	}
	else if (QSel == FIFO_EDCA)
	{
		/* Polling EDCA Out-Q until empty  */
		for (MTxCycle = 0; MTxCycle < 2000; MTxCycle++)
		{
			RTMP_IO_READ32(pAd, TXRXQ_STA, &Data);

			if (((Data >> 19) & 0x1f) == 0)	
				break;
			else
				RTMPusecDelay(50);
		}
	}

	if (MTxCycle >= 2000)
	{
		printk("Polling HCCA Out-Q max\n");
	}

	/* Kick H/W Null frame with PS=0/1 */
	if (QSel == FIFO_EDCA)
	{
		RTMP_IO_WRITE32(pAd, PBF_CTRL, 0x80);

	}
	else
	{
		RTMP_IO_WRITE32(pAd, PBF_CTRL, 0x40);

	}

	RTMPusecDelay(10000);

	//Disable PBF HCCA/EDCA
	RTMP_IO_READ32(pAd, PBF_CFG, &Data);

	Data &= (~0xC);

	RTMP_IO_WRITE32(pAd, PBF_CFG, Data);
}


VOID InitMultiChannelRelatedValue(
	IN PRTMP_ADAPTER pAd,
	IN UCHAR Channel,
	IN UCHAR CentralChannel)
{
	UCHAR Value = 0;
	UINT32 Data = 0;
	unsigned long IrqFlags;
	INT ret;

	RTMP_SEM_EVENT_WAIT(&pAd->MultiChannelLock, ret);

#ifdef DOT11_N_SUPPORT
	/* Change to AP channel */
	if (CentralChannel > Channel) {
		/* Must using 40MHz. */
		pAd->CommonCfg.BBPCurrentBW = BW_40;
		AsicSwitchChannel(pAd, CentralChannel, FALSE);

		RTMP_BBP_IO_READ8_BY_REG_ID(pAd, BBP_R4, &Value);
		Value &= (~0x18);
		Value |= 0x10;
		RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R4, Value);

		/*  RX : control channel at lower */
		RTMP_BBP_IO_READ8_BY_REG_ID(pAd, BBP_R3, &Value);
		Value &= (~0x20);
		RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R3, Value);

		RTMP_IO_READ32(pAd, TX_BAND_CFG, &Data);
		Data &= 0xfffffffe;
		RTMP_IO_WRITE32(pAd, TX_BAND_CFG, Data);

		DBGPRINT(RT_DEBUG_TRACE,
			 ("!!!40MHz Lower !!! Control Channel at Below. Central = %d \n", CentralChannel));
	} 
	else if (CentralChannel < Channel) {
		/* Must using 40MHz. */
		pAd->CommonCfg.BBPCurrentBW = BW_40;
		AsicSwitchChannel(pAd, CentralChannel, FALSE);
		AsicLockChannel(pAd, CentralChannel);

		RTMP_BBP_IO_READ8_BY_REG_ID(pAd, BBP_R4, &Value);
		Value &= (~0x18);
		Value |= 0x10;
		RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R4, Value);

		RTMP_IO_READ32(pAd, TX_BAND_CFG, &Data);
		Data |= 0x1;
		RTMP_IO_WRITE32(pAd, TX_BAND_CFG, Data);

		RTMP_BBP_IO_READ8_BY_REG_ID(pAd, BBP_R3, &Value);
		Value |= (0x20);
		RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R3, Value);

		DBGPRINT(RT_DEBUG_TRACE,
			 ("!!! 40MHz Upper !!! Control Channel at UpperCentral = %d \n", CentralChannel));
	} else
#endif /* DOT11_N_SUPPORT */
	{
		pAd->CommonCfg.BBPCurrentBW = BW_20;
		AsicSwitchChannel(pAd, Channel, FALSE);

		RTMP_BBP_IO_READ8_BY_REG_ID(pAd, BBP_R4, &Value);
		Value &= (~0x18);
		RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R4, Value);

		RTMP_IO_READ32(pAd, TX_BAND_CFG, &Data);
		Data &= 0xfffffffe;
		RTMP_IO_WRITE32(pAd, TX_BAND_CFG, Data);

		RTMP_BBP_IO_READ8_BY_REG_ID(pAd, BBP_R3, &Value);
		Value &= (~0x20);
		RTMP_BBP_IO_WRITE8_BY_REG_ID(pAd, BBP_R3, Value);


		DBGPRINT(RT_DEBUG_TRACE, ("!!! 20MHz !!! \n"));
	}

	RTMPSetAGCInitValue(pAd, pAd->CommonCfg.BBPCurrentBW);

	/* Save BBP_R66 value, it will be used in RTUSBResumeMsduTransmission */
	AsicBBPReadWithRxChain(pAd, BBP_R66, &pAd->BbpTuning.R66CurrentValue, RX_CHAIN_0);

	RTMP_SEM_EVENT_UP(&pAd->MultiChannelLock);
}

VOID EDCA_ActionTimeout(
    IN PVOID SystemSpecific1, 
    IN PVOID FunctionContext, 
    IN PVOID SystemSpecific2, 
    IN PVOID SystemSpecific3) 
{
	RTMP_ADAPTER *pAd = (PRTMP_ADAPTER)FunctionContext;

	pAd->MultiChannelAction = EDCA_TO_HCCA;
	RtmpOsTaskWakeUp(&(pAd->MultiChannelTask));
}

VOID HCCA_ActionTimeout(
    IN PVOID SystemSpecific1, 
    IN PVOID FunctionContext, 
    IN PVOID SystemSpecific2, 
    IN PVOID SystemSpecific3) 
{
	RTMP_ADAPTER *pAd = (PRTMP_ADAPTER)FunctionContext;

	pAd->MultiChannelAction = HCCA_TO_EDCA;
	RtmpOsTaskWakeUp(&(pAd->MultiChannelTask));
}

static VOID ProcessEDCAToHCCA(
    RTMP_ADAPTER *pAd) 
{
	MAC_TABLE_ENTRY *pEntry = NULL;
	PAPCLI_STRUCT pApCliEntry = NULL;
	UINT i = 0;
	UINT32 MacValue, Data;
	INT ret;
	UCHAR Value = 0;
	pApCliEntry = &pAd->ApCfg.ApCliTab[BSS0];

	if ((pApCliEntry->Valid) && INFRA_ON(pAd))
	{
		UINT32 MTxCycle;
		RTMP_SEM_EVENT_WAIT(&pAd->reg_atomic, ret);

		RTMP_OS_NETDEV_STOP_QUEUE(pAd->net_dev);
		RTMP_SET_FLAG(pAd, fRTMP_ADAPTER_DISABLE_DEQUEUEPACKET);

		/* Disable EDCA AC0 dequeue */
		pAd->MultiChannelFlowCtl |= EDCA_AC0_DEQUEUE_DISABLE;
		
		/* Set HCCA/EDCA Q mode to manual-mode  */
		RTMP_IO_READ32(pAd, PBF_CFG, &Data);
		Data |= ((1 << 10) | (1 << 11));
		RTMP_IO_WRITE32(pAd, PBF_CFG, Data);
		
		/* Polling EDCA Out-Q until empty  */
		for (MTxCycle = 0; MTxCycle < 2000; MTxCycle++)
		{

			RTMP_IO_READ32(pAd, TXRXQ_STA, &Data);
			if (((Data >> 19) & 0x1f) == 0)	
				break;
			else
				RTMPusecDelay(50);
		}
	
		if (MTxCycle >= 2000)
		{
			DBGPRINT(RT_DEBUG_ERROR, ("Polling EDCA Out-Q max(%x)\n", Data));
		}

		RT5592_ChannelParamsInit(pAd, 1, pAd->ApCliMlmeAux.Channel, pAd->ApCliMlmeAux.CentralChannel);
		AsicSendCommandToMcu(pAd, CHANNEL_SWITCH_OFFLOAD, 0xff, MUL_CHANNEL_ENABLE, EDCA_TO_HCCA, TRUE);
		//printk("ProcessEDCAToHCCA pAd->ApCliMlmeAux.HtCapability.HtCapInfo.ChannelWidth=%d\n",
		//	pAd->ApCliMlmeAux.HtCapability.HtCapInfo.ChannelWidth);
		/* Check MCU if complete multi channel siwtch */
		do
		{
			RTMP_IO_READ32(pAd, CHANNEL_MCU_READY, &MacValue);

			if ((MacValue & 0x000000ff) == 0x00000078)
			{
				/* Clear CHANNEL_MCU_READY to 0x00 */ 
				MacValue = (MacValue &~ 0x000000ff);
				RTMP_IO_WRITE32(pAd, CHANNEL_MCU_READY, MacValue);
				break;
			}

			i++;
			RTMPusecDelay(1000);
		}while ((i < 10 && (!RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_NIC_NOT_EXIST))));

		if ((i == 10) || (RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_NIC_NOT_EXIST)))
		{
			DBGPRINT(RT_DEBUG_ERROR, ("Multi Channel Switch Retry count exhausted\n"));
		}

		/* Enable HCCA dequeue */
		pAd->MultiChannelFlowCtl &= ~HCCA_DEQUEUE_DISABLE;

		RTMP_CLEAR_FLAG(pAd, fRTMP_ADAPTER_DISABLE_DEQUEUEPACKET);

		RTMP_OS_NETDEV_WAKE_QUEUE(pApCliEntry->dev);
		RTMP_SEM_EVENT_UP(&pAd->reg_atomic);	

		RTMPSetTimer(&pAd->Mlme.HCCAToEDCATimer, HCCA_TIMEOUT);
	}
}


static VOID ProcessHCCAToEDCA(
    PRTMP_ADAPTER pAd)
{
	UINT32 MacValue;
	UINT32 i = 0;
	INT ret;
	UCHAR Value;
	{
		MAC_TABLE_ENTRY *pEntry = NULL;
		PAPCLI_STRUCT pApCliEntry = NULL;

#ifdef CONFIG_STA_SUPPORT
		pApCliEntry = &pAd->ApCfg.ApCliTab[BSS0]; 
		if ((pApCliEntry->Valid) && INFRA_ON(pAd))
		{
			UINT32 MTxCycle, Data;
			RTMP_SEM_EVENT_WAIT(&pAd->reg_atomic, ret);

			RTMP_OS_NETDEV_STOP_QUEUE(pApCliEntry->dev);
			RTMP_SET_FLAG(pAd, fRTMP_ADAPTER_DISABLE_DEQUEUEPACKET);
			
			/* Disable HCCA dequeue */
			pAd->MultiChannelFlowCtl |= HCCA_DEQUEUE_DISABLE;
			
			/* Set HCCA/EDCA Q mode to manual-mode  */
			RTMP_IO_READ32(pAd, PBF_CFG, &Data);
			Data |= ((1 << 10) | (1 << 11));
			RTMP_IO_WRITE32(pAd, PBF_CFG, Data);

			/* Polling HCCA Out-Q until empty  */
			for (MTxCycle = 0; MTxCycle < 2000; MTxCycle++)
			{
				RTMP_IO_READ32(pAd, TXRXQ_STA, &Data);

				if (((Data >> 11) & 0x1f) == 0)
				break;
				else
					RTMPusecDelay(50);
			}

			if (MTxCycle >= 2000)
			{
				DBGPRINT(RT_DEBUG_ERROR, ("Polling HCCA Out-Q max\n"));
			}			

			RT5592_ChannelParamsInit(pAd, 1, pAd->CommonCfg.Channel, pAd->CommonCfg.Channel);
			AsicSendCommandToMcu(pAd, CHANNEL_SWITCH_OFFLOAD, 0xff, MUL_CHANNEL_ENABLE, HCCA_TO_EDCA, TRUE);

		//printk("ProcessHCCAToEDCA pAd->MlmeAux.HtCapability.HtCapInfo.ChannelWidth=%d\n",
			//pAd->MlmeAux.HtCapability.HtCapInfo.ChannelWidth);

			/* Check MCU if complete multi channel siwtch */
			do
			{
				RTMP_IO_READ32(pAd, CHANNEL_MCU_READY, &MacValue);

				if ((MacValue & 0x000000ff) == 0x00000078)
				{
					/* Clear CHANNEL_MCU_READY to 0x00 */ 
					MacValue = (MacValue &~ 0x000000ff);
					RTMP_IO_WRITE32(pAd, CHANNEL_MCU_READY, MacValue);
					break;
				}

				i++;
				RTMPusecDelay(1000);
			}while ((i < 10 && (!RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_NIC_NOT_EXIST))));

			if ((i == 10) || (RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_NIC_NOT_EXIST)))
			{
				DBGPRINT(RT_DEBUG_ERROR, ("Multi Channel Switch Retry count exhausted\n"));
			}

			/* Enable EDCA AC0 dequeue */
			pAd->MultiChannelFlowCtl &= ~EDCA_AC0_DEQUEUE_DISABLE;

			RTMP_CLEAR_FLAG(pAd, fRTMP_ADAPTER_DISABLE_DEQUEUEPACKET);

			RTMP_OS_NETDEV_WAKE_QUEUE(pAd->net_dev);
			RTMP_SEM_EVENT_UP(&pAd->reg_atomic);	

			RTMPSetTimer(&pAd->Mlme.EDCAToHCCATimer, EDCA_TIMEOUT);
		}
#endif /* CONFIG_STA_SUPPORT */
	}
}

static INT MultiChannelTaskThread(
    IN  ULONG Context)
{
	RTMP_OS_TASK *pTask;
	RTMP_ADAPTER *pAd;
	INT	Status = 0;

	pTask = (RTMP_OS_TASK *)Context;
	pAd = (PRTMP_ADAPTER)RTMP_OS_TASK_DATA_GET(pTask);

	if (pAd == NULL)
		return 0;

	RtmpOSTaskCustomize(pTask);

	while (pTask && !RTMP_OS_TASK_IS_KILLED(pTask))
	{
		RTMPusecDelay(2000);

		if (RtmpOSTaskWait(pAd, pTask, &Status) == FALSE)
		{
			RTMP_SET_FLAG(pAd, fRTMP_ADAPTER_HALT_IN_PROGRESS);
			break;
		}

		if (Status != 0)
			break;

#ifdef RTMP_MAC_USB		
		/* device had been closed */
		if (RTMP_TEST_FLAG(pAd, fRTMP_ADAPTER_NIC_NOT_EXIST))
			break;
#endif /* RTMP_MAC_USB */
		
		if (pAd->MultiChannelAction == HCCA_TO_EDCA)
			ProcessHCCAToEDCA(pAd);
		else if (pAd->MultiChannelAction == EDCA_TO_HCCA)
			ProcessEDCAToHCCA(pAd);
		else
			DBGPRINT(RT_DEBUG_TRACE, ("%s: Unkown Action(=%d)\n", __FUNCTION__, pAd->MultiChannelAction));
	}

	if (pTask)
		RtmpOSTaskNotifyToExit(pTask);
	
	return 0;
}

NDIS_STATUS MultiChannelThreadInit(
	IN  PRTMP_ADAPTER pAd)
{
	NDIS_STATUS status = NDIS_STATUS_FAILURE;
	RTMP_OS_TASK *pTask;
	
	DBGPRINT(RT_DEBUG_TRACE, ("--> %s\n", __FUNCTION__));

	pTask = &pAd->MultiChannelTask;
	pAd->MultiChannelAction = 0xFF;
	pAd->Mlme.HCCAToEDCATimerValue = HCCA_TIMEOUT;
	pAd->Mlme.EDCAToHCCATimerValue = EDCA_TIMEOUT;
	pAd->Mlme.HCCAToEDCATimerRunning = FALSE;
	pAd->Mlme.EDCAToHCCATimerRunning = FALSE;
	pAd->MultiChannelFlowCtl = 0;
	RTMPInitTimer(pAd, &pAd->Mlme.EDCAToHCCATimer, GET_TIMER_FUNCTION(EDCA_ActionTimeout), pAd, FALSE);
	RTMPInitTimer(pAd, &pAd->Mlme.HCCAToEDCATimer, GET_TIMER_FUNCTION(HCCA_ActionTimeout), pAd, FALSE);

	RTMP_OS_TASK_INIT(pTask, "MultiChannelTask", pAd);
	status = RtmpOSTaskAttach(pTask, MultiChannelTaskThread, (ULONG)&pAd->MultiChannelTask);
	DBGPRINT(RT_DEBUG_TRACE, ("<-- %s, status=%d!\n", __FUNCTION__, status));

	return status;
}

BOOLEAN MultiChannelThreadExit(
	IN  PRTMP_ADAPTER pAd)
{	
	INT ret;

	MultiChannelTimerStop(pAd);
	
	ret = RtmpOSTaskKill(&pAd->MultiChannelTask);
	if (ret == NDIS_STATUS_FAILURE)
	{
		DBGPRINT(RT_DEBUG_ERROR, ("%s: kill multi-channel task failed!\n", __FUNCTION__));
	}
	return TRUE;
}

VOID MultiChannelTimerStop(
	IN  PRTMP_ADAPTER pAd)
{
	BOOLEAN bCancelled;

	pAd->MultiChannelAction = 0xFF;	
	if (pAd->Mlme.EDCAToHCCATimerRunning)
	{
		RTMPCancelTimer(&pAd->Mlme.EDCAToHCCATimer, &bCancelled);
		pAd->Mlme.EDCAToHCCATimerRunning = FALSE;
	}
	
	if (pAd->Mlme.HCCAToEDCATimerRunning)
	{
		RTMPCancelTimer(&pAd->Mlme.HCCAToEDCATimer, &bCancelled);
		pAd->Mlme.HCCAToEDCATimerRunning = FALSE;
	}

	OS_WAIT(200);
}

VOID MultiChannelTimerStart(
	IN  PRTMP_ADAPTER pAd,
	IN MAC_TABLE_ENTRY  *pEntry)
{
		if (IS_P2P_CLI_ENTRY(pEntry))
		{
			MultiChannelTimerStop(pAd);	
			if (INFRA_ON(pAd))
			{
				RTMPSetTimer(&pAd->Mlme.HCCAToEDCATimer, pAd->Mlme.HCCAToEDCATimerValue);
				pAd->Mlme.EDCAToHCCATimerRunning = TRUE;
			}
			pAd->Mlme.P2pStayTick = 0;
		}
		else if (IS_ENTRY_CLIENT(pEntry))
		{
			MultiChannelTimerStop(pAd);
			if (P2P_CLI_ON(pAd))
			{
				RTMPSetTimer(&pAd->Mlme.EDCAToHCCATimer, pAd->Mlme.EDCAToHCCATimerValue);
				pAd->Mlme.EDCAToHCCATimerRunning = TRUE;		
			}
			pAd->Mlme.StaStayTick = 0;
		}
}
#endif /* CONFIG_MULTI_CHANNEL */


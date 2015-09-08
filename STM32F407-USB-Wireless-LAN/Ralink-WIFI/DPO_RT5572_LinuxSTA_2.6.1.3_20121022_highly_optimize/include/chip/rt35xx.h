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


#ifndef __RT35XX_H__
#define __RT35XX_H__

#ifdef RT35xx

struct _RTMP_ADAPTER;

#ifndef RTMP_RF_RW_SUPPORT
#error "For RT3062/3562/3572/3592, you should define the compile flag -DRTMP_RF_RW_SUPPORT"
#endif

#include "chip/rt30xx.h"

extern REG_PAIR   RF3572_RFRegTable[];

/* */
/* Device ID & Vendor ID, these values should match EEPROM value */
/* */
#define NIC3062_PCI_DEVICE_ID	0x3062		/* 2T/2R miniCard */
#define NIC3562_PCI_DEVICE_ID	0x3562		/* 2T/2R miniCard */
#define NIC3060_PCI_DEVICE_ID	0x3060		/* 1T/1R miniCard */

#define EDIMAX_PCI_VENDOR_ID	0x1432

/* use CHIPSET = 3562 compile */
#define NIC3592_PCIe_DEVICE_ID	0x3592		/* 2T/2R miniCard */

VOID RT35xx_Init(
	IN struct _RTMP_ADAPTER		*pAd);

VOID NICInitRT35xxMacRegisters(
	IN	struct _RTMP_ADAPTER	*pAd);

VOID RT35xx_ChipSwitchChannel(
	IN struct _RTMP_ADAPTER		*pAd,
	IN UCHAR					Channel,
	IN BOOLEAN					bScan);

VOID RT35xx_RxSensitivityTuning(
	IN struct _RTMP_ADAPTER		*pAd);

VOID NICInitRT3572RFRegisters(
	IN struct _RTMP_ADAPTER		*pAd);

VOID RT3572ReverseRFSleepModeSetup(
	IN struct _RTMP_ADAPTER		*pAd,
	IN BOOLEAN					FlgIsInitState);

VOID RT35xx_NICInitAsicFromEEPROM(
	IN struct _RTMP_ADAPTER		*pAd);

UCHAR RT35xx_ChipAGCAdjust(
	IN	struct _RTMP_ADAPTER	*pAd,
	IN  CHAR					Rssi,
	IN  UCHAR					R66);

VOID RT35xx_ChipBBPAdjust(
	IN	struct _RTMP_ADAPTER	*pAd);

#endif /* RT35xx */
#endif /*__RT35XX_H__ */


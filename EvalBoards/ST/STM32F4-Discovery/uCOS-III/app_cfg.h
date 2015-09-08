/*
*********************************************************************************************************
*                                              uC/OS-II
*                                        The Real-Time Kernel
*
*                             (c) Copyright 2012; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*
*               uC/OS-II is provided in source form for FREE evaluation, for educational
*               use or peaceful research.  If you plan on using uC/OS-II in a commercial
*               product you need to contact Micrium to properly license its use in your
*               product.  We provide ALL the source code for your convenience and to
*               help you experience uC/OS-II.  The fact that the source code is provided
*               does NOT mean that you can use it without paying a licensing fee.
*
*               Knowledge of the source code may NOT be used to develop a similar product.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                       APPLICATION CONFIGURATION
*
*                                       IAR Development Kits
*                                              on the
*
*                                    STM32F429II-SK KICKSTART KIT
*
* Filename      : app_cfg.h
* Version       : V1.00
* Programmer(s) : FT
*********************************************************************************************************
*/

#ifndef  APP_CFG_MODULE_PRESENT
#define  APP_CFG_MODULE_PRESENT


/*
*********************************************************************************************************
*                                       ADDITIONAL uC/MODULE ENABLES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            TASK PRIORITIES
*********************************************************************************************************
*/



#define  APP_TASK_START_PRIO                        15u 

#define  FTPD_TASK_PRIO                             15u 
#define  USBH_TASK_PRIO                             17u 
#define  USBH_PROBE_TASK_PRIO                      16u

//#define  TTCP_OS_CFG_TASK_PRIO                      4
/*
*********************************************************************************************************
*                                            TASK STACK SIZES
*********************************************************************************************************
*/

#define  APP_TASK_START_STK_SIZE                        512u

#define  USBH_TASK_STK_SIZE                              256u
#define  USBH_PROBE_TASK_STK_SIZE                        512u

#define FTPD_TASK_STK_SIZE  768

//#define  TTCP_OS_CFG_TASK_STK_SIZE                       768


/*
*********************************************************************************************************
*                                                 TTCP
*********************************************************************************************************
*/

// #define  TTCP_CFG_MAX_ACCEPT_TIMEOUT_MS                 5000    /* Maximum inactivity time (ms) on ACCEPT.              */
// #define  TTCP_CFG_MAX_CONN_TIMEOUT_MS                   5000    /* Maximum inactivity time (ms) on CONNECT.             */
// #define  TTCP_CFG_MAX_RX_TIMEOUT_MS                     5000    /* Maximum inactivity time (ms) on RX.                  */
// #define  TTCP_CFG_MAX_TX_TIMEOUT_MS                     5000    /* Maximum inactivity time (ms) on TX.                  */

// #define  TTCP_CFG_MAX_ACCEPT_RETRY                         3    /* Maximum number of retries on ACCEPT.                 */
// #define  TTCP_CFG_MAX_CONN_RETRY                           3    /* Maximum number of retries on CONNECT.                */
// #define  TTCP_CFG_MAX_RX_RETRY                             3    /* Maximum number of retries on RX.                     */
// #define  TTCP_CFG_MAX_TX_RETRY                             3    /* Maximum number of retries on TX.                     */

// #define  TTCP_CFG_BUF_LEN                               2048    /* Buffer length.                                       */




/*
*********************************************************************************************************
*                                       TRACE / DEBUG CONFIGURATION
*********************************************************************************************************
*/

#ifndef  TRACE_LEVEL_OFF
#define  TRACE_LEVEL_OFF                0
#endif

#ifndef  TRACE_LEVEL_INFO
#define  TRACE_LEVEL_INFO               1
#endif

#ifndef  TRACE_LEVEL_DBG
#define  TRACE_LEVEL_DBG                2
#endif

#define  APP_CFG_TRACE_LEVEL             TRACE_LEVEL_DBG
#define  APP_CFG_TRACE                   printf

#define  BSP_CFG_TRACE_LEVEL             TRACE_LEVEL_DBG
#define  BSP_CFG_TRACE                   printf

#define  APP_TRACE_INFO(x)               ((APP_CFG_TRACE_LEVEL >= TRACE_LEVEL_INFO)  ? (void)(APP_CFG_TRACE x) : (void)0)
#define  APP_TRACE_DBG(x)                ((APP_CFG_TRACE_LEVEL >= TRACE_LEVEL_DBG)   ? (void)(APP_CFG_TRACE x) : (void)0)

#define  BSP_TRACE_INFO(x)               ((BSP_CFG_TRACE_LEVEL  >= TRACE_LEVEL_INFO) ? (void)(BSP_CFG_TRACE x) : (void)0)
#define  BSP_TRACE_DBG(x)                ((BSP_CFG_TRACE_LEVEL  >= TRACE_LEVEL_DBG)  ? (void)(BSP_CFG_TRACE x) : (void)0)

#endif

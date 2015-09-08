/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                             (c) Copyright 2013; Micrium, Inc.; Weston, FL
*
*                   All rights reserved.  Protected by international copyright laws.
*                   Knowledge of the source code may not be used to write a similar
*                   product.  This file may only be used in accordance with a license
*                   and should not be redistributed in any way.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                            EXAMPLE CODE
*
*                                       IAR Development Kits
*                                              on the
*
*                                    STM32F429II-SK KICKSTART KIT
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : YS
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#include  <includes.h>
#include  <os_app_hooks.h>
/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

/* ----------------- APPLICATION GLOBALS -------------- */
static  OS_TCB   AppTaskStartTCB;
static  CPU_STK  AppTaskStartStk[APP_TASK_START_STK_SIZE];


/*
*********************************************************************************************************
*                                         FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart          (void     *p_arg);


/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Arguments   : none
*
* Returns     : none
*********************************************************************************************************
*/

int main(void)
{
    OS_ERR  err;


    BSP_IntDisAll();                                            /* Disable all interrupts.                              */

    OSInit(&err);                                               /* Init uC/OS-III.                                      */

    OSTaskCreate((OS_TCB       *)&AppTaskStartTCB,              /* Create the start task                                */
                 (CPU_CHAR     *)"App Task Start",
                 (OS_TASK_PTR   )AppTaskStart,
                 (void         *)0u,
                 (OS_PRIO       )APP_TASK_START_PRIO,
                 (CPU_STK      *)&AppTaskStartStk[0u],
                 (CPU_STK_SIZE  )APP_TASK_START_STK_SIZE / 10u,
                 (CPU_STK_SIZE  )APP_TASK_START_STK_SIZE,
                 (OS_MSG_QTY    )0u,
                 (OS_TICK       )0u,
                 (void         *)0u,
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR       *)&err);

    OSStart(&err);                                              /* Start multitasking (i.e. give control to uC/OS-III). */

    (void)&err;

    return (0u);
}


/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Arguments   : p_arg   is the argument passed to 'AppTaskStart()' by 'OSTaskCreate()'.
*
* Returns     : none
*
* Notes       : 1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                  used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

OS_TCB *TCBListTbl[12];
int TCBListIndex = 0;

void TaskCreateHook(OS_TCB *p_tcb)
{
    CPU_SR cpu_sr;

    CPU_CRITICAL_ENTER();
    if(TCBListIndex < 12)
    {
        TCBListTbl[TCBListIndex] = p_tcb;
        TCBListIndex++;
    }
    CPU_CRITICAL_EXIT();
}


#define USBH_DEBUG_LEVEL  USBH_DEBUG_TRACE
#include "usbh_config.h"
#include "usbh_core.h"
// #include "misc_cvt.h"
#include "rt_config.h"
#include "shell.h"
#include "wlan.h"
#include "lwip/tcpip.h"





extern CPU_INT16S iwpriv_main (CPU_INT16U argc,
                               CPU_CHAR *argv[],
                               SHELL_OUT_FNCT out_fnct,
                               SHELL_CMD_PARAM *pcmd_param);


static SHELL_CMD iw_shell_cmd[] =
{
  {"iwpriv", iwpriv_main},
  {0, 0 }
};





// void DoTask()
// {
//     printf("DoTask....\r\n");

//     wireless_exec_cmd("iw_list scanning");

//     wireless_exec_cmd("iw_priv set NetworkType=Infra");

//     wireless_exec_cmd("iw_priv set AuthMode=WPA2PSK");

//     wireless_exec_cmd("iw_priv set EncrypType=AES");

//     wireless_exec_cmd("iw_priv set SSID=NetWork_Main");

//     wireless_exec_cmd("iw_priv set WPAPSK=guodanian");
// }



static  void  AppTaskStart (void *p_arg)
{
    CPU_INT32U  cpu_clk_freq;
    CPU_INT32U  cnts;
    OS_ERR      err;
    RCC_ClocksTypeDef RCC_ClocksStatus;



    (void)p_arg;

    BSP_Init();                                                 /* Initialize BSP functions                             */
    CPU_Init();                                                 /* Initialize the uC/CPU services                       */

    cpu_clk_freq = BSP_CPU_ClkFreq();                           /* Determine SysTick reference freq.                    */
    cnts         = cpu_clk_freq                                 /* Determine nbr SysTick increments                     */
                   / (CPU_INT32U)OSCfg_TickRate_Hz;

    OS_CPU_SysTickInit(cnts);                                   /* Init uC/OS periodic time src (SysTick).              */

    Mem_Init();                                                 /* Initialize memory managment module                   */
    Math_Init();                                                /* Initialize mathematical module                       */


#if OS_CFG_STAT_TASK_EN > 0u
    OSStatTaskCPUUsageInit(&err);                               /* Compute CPU capacity with no task running            */
#endif

#ifdef CPU_CFG_INT_DIS_MEAS_EN
    CPU_IntDisMeasMaxCurReset();
#endif

    App_OS_SetAllHooks();


    RCC_GetClocksFreq(&RCC_ClocksStatus);
    printf("SYSCLK_Frequency:%d,HCLK_Frequency:%d,PCLK1_Frequency:%d,PCLK2_Frequency:%d,ADCCLK_Frequency:%d\r\n", RCC_ClocksStatus.SYSCLK_Frequency, RCC_ClocksStatus.HCLK_Frequency, RCC_ClocksStatus.PCLK1_Frequency, RCC_ClocksStatus.PCLK2_Frequency, 0);


    
    Shell_Init();
    /* Create tcp_ip stack thread */
    tcpip_init( NULL, NULL );
    /* Initilaize the LwIP stack */
    Netif_Config();


#ifndef USE_LWIP_MALLOC
    usbh_mem_init();
#endif

    USBH_Init();

   
    iperf_init();
    
    
    Shell_CmdTblAdd("iwpirv", iw_shell_cmd, &err);



    Ser_RxTask();

    while (DEF_TRUE)                                            /* Task body, always written as an infinite loop.       */
    {
        OSTimeDlyHMSM(0u, 0u, 3u, 0,
                      OS_OPT_TIME_HMSM_STRICT,
                      &err);
    }
}





void assert_failed(uint8_t *file, uint32_t line)
{
    printf("assert_failed in:%s,line:%d \n", file ? file : "n", line);
    while (1);
}

void dump_tcblist()
{
    OS_TCB *p_tcb;
    int i;

    for(i = 0; i < TCBListIndex; i++)
    {
        p_tcb = TCBListTbl[TCBListIndex];
        printf("taskname:%s  StkSize:%d  StkUsed:%d\r\n",p_tcb->NamePtr,p_tcb->StkSize,p_tcb->StkUsed);
    }

//     p_tcb = OSTaskDbgListPtr;
//     while(p_tcb)
//     {
//         printf("taskname:%s  StkSize:%d  StkUsed:%d\r\n",p_tcb->NamePtr,p_tcb->StkSize,p_tcb->StkUsed);
//         p_tcb = OSTaskDbgListPtr->DbgNextPtr;
//     }
}

extern struct net_device *_pnet_device;
extern struct usb_device _usb_device;
extern int output_nest_ctr;

static CPU_CHAR  my_buf_str[128];

void  my_printf (char *format, ...)
{

    va_list   v_args;
    char  *p_str = &my_buf_str[0];


    va_start(v_args, format);
    (void)vsnprintf((char       *)&my_buf_str[0],
                    (CPU_SIZE_T  ) sizeof(my_buf_str),
                    (char const *) format,
                    v_args);
    va_end(v_args);


    while ((*p_str) != (CPU_CHAR )0)
    {
        while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == 0);
        USART_SendData(USART2, *p_str);
        while(USART_GetFlagStatus(USART2,USART_FLAG_TC) == 0);

        p_str++;
    }

}

#define printf my_printf
void hard_fault_handler_c (unsigned int * hardfault_args)
{
    OS_TCB *p_tcb;
    unsigned int stacked_r0;
    unsigned int stacked_r1;
    unsigned int stacked_r2;
    unsigned int stacked_r3;
    unsigned int stacked_r12;
    unsigned int stacked_lr;
    unsigned int stacked_pc;
    unsigned int stacked_psr;
    stacked_r0 = ((unsigned long) hardfault_args[0]);
    stacked_r1 = ((unsigned long) hardfault_args[1]);
    stacked_r2 = ((unsigned long) hardfault_args[2]);
    stacked_r3 = ((unsigned long) hardfault_args[3]);
    stacked_r12 = ((unsigned long) hardfault_args[4]);
    stacked_lr = ((unsigned long) hardfault_args[5]);
    stacked_pc = ((unsigned long) hardfault_args[6]);
    stacked_psr = ((unsigned long) hardfault_args[7]);

    printf ("\r\n\r\n[Hard fault handler - all numbers in hex]\r\n");
    printf ("R0 = %x\r\n", stacked_r0);
    printf ("R1 = %x\r\n", stacked_r1);
    printf ("R2 = %x\r\n", stacked_r2);
    printf ("R3 = %x\r\n", stacked_r3);
    printf ("R12 = %x\r\n", stacked_r12);
    printf ("LR [R14] = %x subroutine call return address\r\n", stacked_lr);
    printf ("PC [R15] = %x program counter\r\n", stacked_pc);
    printf ("PSR = %x\r\n", stacked_psr);
    printf ("BFAR = %x\r\n", (*((volatile unsigned long *)(0xE000ED38))));
    printf ("CFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED28))));
    printf ("HFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED2C))));
    printf ("DFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED30))));
    printf ("AFSR = %x\r\n", (*((volatile unsigned long *)(0xE000ED3C))));
    printf ("SCB_SHCSR = %x\r\n", SCB->SHCSR);

    printf("output_nest_ctr:%d\r\n",output_nest_ctr);
    printf("usb_nperiod_task_req_num:%d urb_nperiod_queue.num:%d max_num:%d\r\n",_usb_device.usb_nperiod_task_req_num,_usb_device.urb_nperiod_queue.num,_usb_device.urb_nperiod_queue.max_num_record);
    printf("OSIntNestingCtr:%d  OSSchedLockNestingCtr:%d CPU_IntDisNestCtr:%d\r\n",OSIntNestingCtr,OSSchedLockNestingCtr,CPU_IntDisNestCtr);
    printf("OSTCBCurPtr->NamePtr = %s StkSize:%d  StkUsed:%d\r\n",OSTCBCurPtr->NamePtr,OSTCBCurPtr->StkSize,OSTCBCurPtr->StkUsed);
    printf("total task:%d\r\n",TCBListIndex);
    p_tcb = TCBListTbl[0];
    printf("%20s  prio:%d StkSize:%d  StkUsed:%d StkHi:%p StkLw:%p StkPtr:%p\r\n",p_tcb->NamePtr,p_tcb->Prio,p_tcb->StkSize,p_tcb->StkUsed,p_tcb->StkBasePtr+p_tcb->StkSize,p_tcb->StkBasePtr,p_tcb->StkPtr);
    p_tcb = TCBListTbl[1];
    printf("%20s  prio:%d StkSize:%d  StkUsed:%d StkHi:%p StkLw:%p StkPtr:%p\r\n",p_tcb->NamePtr,p_tcb->Prio,p_tcb->StkSize,p_tcb->StkUsed,p_tcb->StkBasePtr+p_tcb->StkSize,p_tcb->StkBasePtr,p_tcb->StkPtr);
    p_tcb = TCBListTbl[2];
    printf("%20s  prio:%d StkSize:%d  StkUsed:%d StkHi:%p StkLw:%p StkPtr:%p\r\n",p_tcb->NamePtr,p_tcb->Prio,p_tcb->StkSize,p_tcb->StkUsed,p_tcb->StkBasePtr+p_tcb->StkSize,p_tcb->StkBasePtr,p_tcb->StkPtr);
    p_tcb = TCBListTbl[3];
    printf("%20s  prio:%d StkSize:%d  StkUsed:%d StkHi:%p StkLw:%p StkPtr:%p\r\n",p_tcb->NamePtr,p_tcb->Prio,p_tcb->StkSize,p_tcb->StkUsed,p_tcb->StkBasePtr+p_tcb->StkSize,p_tcb->StkBasePtr,p_tcb->StkPtr);
    p_tcb = TCBListTbl[4];
    printf("%20s  prio:%d StkSize:%d  StkUsed:%d StkHi:%p StkLw:%p StkPtr:%p\r\n",p_tcb->NamePtr,p_tcb->Prio,p_tcb->StkSize,p_tcb->StkUsed,p_tcb->StkBasePtr+p_tcb->StkSize,p_tcb->StkBasePtr,p_tcb->StkPtr);
    p_tcb = TCBListTbl[5];
    printf("%20s  prio:%d StkSize:%d  StkUsed:%d StkHi:%p StkLw:%p StkPtr:%p\r\n",p_tcb->NamePtr,p_tcb->Prio,p_tcb->StkSize,p_tcb->StkUsed,p_tcb->StkBasePtr+p_tcb->StkSize,p_tcb->StkBasePtr,p_tcb->StkPtr);
//     p_tcb = TCBListTbl[6];
//     printf("taskname:%s  StkSize:%d  StkUsed:%d\r\n",p_tcb->NamePtr,p_tcb->StkSize,p_tcb->StkUsed);
//
//     for(i = 0;i < TCBListIndex; i++)
//     {
//         p_tcb = TCBListTbl[TCBListIndex];
//          printf("taskname:%s  StkSize:%d  StkUsed:%d\r\n",p_tcb->NamePtr,p_tcb->StkSize,p_tcb->StkUsed);
//     }


    while (1);

    //  while(USART_GetFlagStatus(USART1,USART_FLAG_RXNE) == RESET);
//     APP_TRACE_INFO(("please input cmd\r\n"));
//  cmd = BSP_Ser_RdByte();

}



// #define VENDOR_FEATURE2_SUPPORT
// #define AGGREGATION_SUPPORT
// #define PIGGYBACK_SUPPORT
// //#define WMM_SUPPORT
// #define LINUX
// //#define SYSTEM_LOG_SUPPORT
// #define RT28xx_MODE=STA
// #define CHIPSET=5572
// #define CONFIG_RA_NAT_NONE
// #define CONFIG_STA_SUPPORT
// #define DBG
// #define DOT11_N_SUPPORT
// //#define DOT11N_DRAFT3
// #define STATS_COUNT_SUPPORT
// #define LED_CONTROL_SUPPORT
// #define RTMP_MAC_USB
// #define RTMP_USB_SUPPORT
// #define RT2870
// #define RT28xx
// #define RTMP_TIMER_TASK_SUPPORT
// #define A_BAND_SUPPORT
// #define RTMP_MAC_USB
// #define RTMP_USB_SUPPORT
// #define RT2870
// #define RT28xx
// #define RT30xx
// #define RT35xx
// #define RT3572
// #define RTMP_TIMER_TASK_SUPPORT
// #define RTMP_RF_RW_SUPPORT
// #define RTMP_EFUSE_SUPPORT
// #define A_BAND_SUPPORT
// #define VCORECAL_SUPPORT
// #define RTMP_MAC_USB
// #define RTMP_USB_SUPPORT
// #define RT30xx
// #define RT35xx
// #define RT3593
// #define RT3573
// #define RTMP_TIMER_TASK_SUPPORT
// #define RTMP_RF_RW_SUPPORT
// #define RTMP_EFUSE_SUPPORT
// #define A_BAND_SUPPORT
// #define DOT11N_SS3_SUPPORT
// #define VCORECAL_SUPPORT
// #define NEW_MBSSID_MODE
// #define RTMP_FREQ_CALIBRATION_SUPPORT
// #define RTMP_MAC_USB
// #define RT30xx
// #define RT33xx
// #define RT3070
// #define RT3370
// #define RT5370
// #define RTMP_USB_SUPPORT
// #define RTMP_TIMER_TASK_SUPPORT
// #define RTMP_RF_RW_SUPPORT
// #define RTMP_EFUSE_SUPPORT
// #define RTMP_INTERNAL_TX_ALC
// #define RTMP_FREQ_CALIBRATION_SUPPORT
// #define VCORECAL_SUPPORT
// #define RTMP_TEMPERATURE_COMPENSATION
// #define NEW_MBSSID_MODE
// #define RTMP_MAC_USB
// #define RTMP_USB_SUPPORT
// #define RT30xx
// #define RT5572
// #define RT5592
// #define RTMP_RF_RW_SUPPORT
// #define RTMP_EFUSE_SUPPORT
// #define RTMP_TIMER_TASK_SUPPORT
// #define A_BAND_SUPPORT
// #define VCORECAL_SUPPORT
// #define RTMP_TEMPERATURE_COMPENSATION
// #define RTMP_FREQ_CALIBRATION_SUPPORT


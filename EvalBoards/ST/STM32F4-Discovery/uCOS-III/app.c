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

#include "usbh_linux.h"
#include "net_cvt.h"
#include "memory.h"
#include "shell.h"
#include "wlan.h"
#include "lwip/tcpip.h"
#include "ping.h"

extern CPU_INT16S iwpriv_main (CPU_INT16U argc,
                               CPU_CHAR *argv[],
                               SHELL_OUT_FNCT out_fnct,
                               SHELL_CMD_PARAM *pcmd_param);

extern void iperf_init(void);

extern int rtusb_init(void);

extern void tasklet_task_init(void);

extern const struct usb_device_id rtusb_dev_id[];
extern struct net_device *_pnet_device;



static void ralink_hotplug_call_back(struct usb_interface *intf, void *arg, int is_connect)
{
    struct net_device *net_dev = _pnet_device;
    OS_ERR err;
    CPU_SR cpu_sr;    
    int ret;

    
    if(is_connect)
    {
        /* usb device is connected*/   

        /* If a net device is registered, open it*/
        if(_pnet_device)
        {
            _pnet_device->flags |= IFF_UP;
            _pnet_device->usb_dev = interface_to_usbdev(intf);

            ret = (*(_pnet_device->open))(_pnet_device);
            if(ret)
            {
                printf("USBH_ProbeUSB _pnet_device->open Failed:%d\r\n",ret);
                return;
            }
        }
    }
    else
    {
        /* usb device is disconnected*/ 
        
        /* Close net device */
        if(_pnet_device)
        {
            CPU_CRITICAL_ENTER();
            while(net_dev->status)
            {
                CPU_CRITICAL_EXIT();
                //wait for NET_IOCTL_RUNNING or NET_XIMT_RUNNING
                OSTimeDlyHMSM(0u, 0u, 0u, 10u, OS_OPT_TIME_HMSM_STRICT, &err);
                CPU_CRITICAL_ENTER();
            }
            _pnet_device = NULL;
            CPU_CRITICAL_EXIT();  

            (net_dev->stop)(net_dev);
        }
    }
}


static  void  AppTaskStart (void *p_arg)
{
    CPU_INT32U  cpu_clk_freq;
    CPU_INT32U  cnts;
    OS_ERR      err;
    SHELL_ERR   shell_err;

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



    /* Initializes uC/Shell */
    Shell_Init();
    /* Create tcp_ip stack thread */
    tcpip_init( NULL, NULL );
    /* Configure the LwIP stack */
    Netif_Config();
    
    /* Initializes iperf */
    iperf_init();

    /* Create tasklet task used for usb wireless driver */
    tasklet_task_init();
#ifndef USE_LWIP_MALLOC
    /* Initialize heap if not merge kmalloc() with lwip mem_malloc() function.
     * see memory.h */
    usbh_mem_init();
#endif
    
    /* Initializes usb host */
    usb_init();
    
    /* Register ralink usb wireless driver */
    rtusb_init();
    /* Add hotplug function that will be called if usb device plugged and match the usb_device_id*/
    usb_hotplug_add(&(struct usb_hotplug){rtusb_dev_id, ralink_hotplug_call_back, NULL});

    /* If all usb driver are initialized, start usb host. */
    usb_start();
    
    
    /* Add iwpriv shell command*/
    Shell_CmdTblAdd("iwpriv", (SHELL_CMD[]){{"iwpriv", iwpriv_main},{0, 0 }}, &shell_err);
    /* Add ping shell command*/
    Shell_CmdTblAdd("ping", (SHELL_CMD[]){ {"ping", ping}, {0, 0 }}, &shell_err);   
    
   
    
    /* Serial terminal process task, it will not return*/
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
    printf("assert_failed in:%s,line:%d \n", file ? (char *)file : "n", line);
    while (1);
}




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

    printf("OSIntNestingCtr:%d  OSSchedLockNestingCtr:%d CPU_IntDisNestCtr:%d\r\n",OSIntNestingCtr,OSSchedLockNestingCtr,CPU_IntDisNestCtr);
    printf("OSTCBCurPtr->NamePtr = %s StkSize:%d  StkUsed:%d\r\n",OSTCBCurPtr->NamePtr,OSTCBCurPtr->StkSize,OSTCBCurPtr->StkUsed);


    while (1);
}



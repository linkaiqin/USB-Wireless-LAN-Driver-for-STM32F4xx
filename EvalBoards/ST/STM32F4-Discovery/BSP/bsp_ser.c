/*
*********************************************************************************************************
*                                     MICRIUM BOARD SUPPORT SUPPORT
*
*                          (c) Copyright 2003-2012; Micrium, Inc.; Weston, FL
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
*                                    MICRIUM BOARD SUPPORT PACKAGE
*                                       SERIAL (UART) INTERFACE
*
*                                     ST Microelectronics STM32
*                                              on the
*
*                                           STM3220G-EVAL
*                                         Evaluation Board
*
* Filename      : bsp_ser.c
* Version       : V1.00
* Programmer(s) : SL
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/

#define  BSP_SER_MODULE
#include <bsp.h>
#include <bsp_ser.h>
#include "bmd.h"
#include "stdio.h"
#include "string.h"
#include "memory.h"
#include "shell.h"

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/
//#define USE_BUF_PRINTF


#define  BSP_SER_PRINTF_STR_BUF_SIZE             80u
#define  SERIAL_RX_BUFFER_SIZE  80
#define  SERIAL_TX_BUFFER_SIZE  1024



static BSP_OS_SEM   BSP_SerTxWait;
static BSP_OS_SEM   BSP_SerRxWait;
#ifdef USE_BUF_PRINTF
static BSP_OS_SEM BSP_SerTxBufWait;
#endif


BSP_OS_SEM   BSP_SerLock;


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void        BSP_Ser_ISR_Handler     (void);




static BUFFER_INFO rx_buf_ring;
static CPU_INT08U rx_buffer[SERIAL_RX_BUFFER_SIZE];



//int gSerialEnable = 1;

/*
*********************************************************************************************************
*                                     LOCAL CONFIGURATION ERRORS
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*********************************************************************************************************
**                                         GLOBAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

#ifdef USE_BUF_PRINTF
static BUFFER_INFO tx_buf_ring;
static CPU_INT08U tx_buffer[SERIAL_TX_BUFFER_SIZE];
#endif






/*
*********************************************************************************************************
*                                          BSP_Ser_Init()
*
* Description : Initialize a serial port for communication.
*
* Argument(s) : baud_rate           The desire RS232 baud rate.
*
* Return(s)   : none.
*
* Caller(s)   : Application
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_Ser_Init ()
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    USART_InitTypeDef       USART_InitStructure;
    USART_ClockInitTypeDef  USART_CLK_InitStructure;


    /* ------------------ INIT OS OBJECTS ----------------- */
    BSP_OS_SemCreate(&BSP_SerTxWait,   1, "Serial Tx Wait");
    BSP_OS_SemCreate(&BSP_SerRxWait,   0, "Serial Rx Wait");
    BSP_OS_SemCreate(&BSP_SerLock,     1, "Serial Lock");

    /* ----------------- INIT USART STRUCT ---------------- */
    USART_InitStructure.USART_BaudRate            = BSP_SER_COMM_BAUDRATE;
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    USART_InitStructure.USART_Parity              = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;

    USART_CLK_InitStructure.USART_Clock           = USART_Clock_Disable;
    USART_CLK_InitStructure.USART_CPOL            = USART_CPOL_Low;
    USART_CLK_InitStructure.USART_CPHA            = USART_CPHA_2Edge;
    USART_CLK_InitStructure.USART_LastBit         = USART_LastBit_Disable;


    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);       /* Enable GPIO clock.                                   */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);      /* Enable UART clock.                                   */


    /* ----------------- SETUP USART2 GPIO ---------------- */
    /* Configure GPIOC.5 as push-pull.                      */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

    /* Configure GPIOD.6 as input floating.                 */
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);


    Buf_init(&rx_buf_ring,rx_buffer,sizeof(rx_buffer));

    /* ------------------ SETUP USART2 -------------------- */
    USART_Init(USART2, &USART_InitStructure);
    USART_ClockInit(USART2, &USART_CLK_InitStructure);
    USART_Cmd(USART2, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    BSP_IntVectSet(BSP_INT_ID_USART2, BSP_Ser_ISR_Handler);
    BSP_IntEn(BSP_INT_ID_USART2);



#ifdef USE_BUF_PRINTF
    Buf_init(&tx_buf_ring,tx_buffer,sizeof(tx_buffer));
    BSP_OS_SemCreate(&BSP_SerTxBufWait,   0, "BSP_SerTxBufWait");
#endif
}






/*
*********************************************************************************************************
*                                         BSP_Ser_ISR_Handler()
*
* Description : Serial ISR
*
* Argument(s) : none
*
* Return(s)   : none.
*
* Caller(s)   : This is an ISR.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_Ser_ISR_Handler (void)
{
    CPU_REG32  status;
    CPU_INT08U ret,rx_byte;


    status = USART2->SR;

    if (DEF_BIT_IS_SET(status, DEF_BIT_05))
    {
        rx_byte = USART_ReceiveData(USART2) & 0xFF;       /* Read one byte from the receive data register.      */
        Buf_IsFull(&rx_buf_ring, ret);
        if(ret == Buff_notFull)
            Buf_Push(&rx_buf_ring, rx_byte);

        USART_ClearITPendingBit(USART2, USART_IT_RXNE);         /* Clear the USART2 receive interrupt.                */
        BSP_OS_SemPost(&BSP_SerRxWait);                         /* Post to the sempahore                              */
    }

    if (DEF_BIT_IS_SET(status, DEF_BIT_07))
    {
        USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
//        USART_ClearITPendingBit(USART2, USART_IT_TXE);          /* Clear the USART2 receive interrupt.                */
        BSP_OS_SemPost(&BSP_SerTxWait);                         /* Post to the semaphore                              */
    }

}


#define FINSH_USING_HISTORY
#define FINSH_HISTORY_LINES 5
#define FINSH_CMD_SIZE      80
#define FINSH_PROMPT      ">"
#define rt_kprintf printf
#define rt_strlen strlen
#define rt_malloc(size) kmalloc(size,0)
#define rt_free  kfree
#define RT_NULL NULL






struct finsh_shell
{
    CPU_INT08U use_history:1;
    CPU_INT08U stat;
#ifdef FINSH_USING_HISTORY
    CPU_INT08U current_history;
    CPU_INT16U history_count;

    char cmd_history[FINSH_HISTORY_LINES][FINSH_CMD_SIZE];
#endif

    char line[FINSH_CMD_SIZE];
    CPU_INT08U line_position;
    CPU_INT08U line_curpos;
};


struct finsh_shell _shell;

#define WAIT_NORMAL     0
#define WAIT_SPEC_KEY   1
#define WAIT_FUNC_KEY   2





static int str_is_prefix(const char *prefix, const char *str)
{
    while ((*prefix) && (*prefix == *str))
    {
        prefix ++;
        str ++;
    }

    if (*prefix == 0)
        return 0;

    return -1;
}


static int str_common(const char *str1, const char *str2)
{
    const char *str = str1;

    while ((*str != 0) && (*str2 != 0) && (*str == *str2))
    {
        str ++;
        str2 ++;
    }

    return (str - str1);
}





void shell_auto_complete(char* prefix)
{
    SHELL_MODULE_CMD  *module_cmd;
    CPU_INT16U func_cnt ,index;
    int length, min_length=0;
    char *name_ptr;


    rt_kprintf("\n");

    func_cnt = 0;
    module_cmd = Shell_ModuleCmdUsedPoolPtr;
    name_ptr = NULL;


    while (module_cmd)
    {

        for (index = 0; module_cmd->CmdTblPtr[index].Name != NULL; index ++)
        {
            if (str_is_prefix(prefix, module_cmd->CmdTblPtr[index].Name) == 0)
            {
                if (func_cnt == 0)
                {
                    rt_kprintf("--function:\n");

                    if (*prefix != 0)
                    {
                        /* set name_ptr */
                        name_ptr = module_cmd->CmdTblPtr[index].Name;

                        /* set initial length */
                        min_length = strlen(name_ptr);
                    }
                }

                func_cnt ++;

                if (*prefix != 0)
                {
                    length = str_common(name_ptr, module_cmd->CmdTblPtr[index].Name);
                    if (length < min_length)
                        min_length = length;
                }

//#ifdef FINSH_USING_DESCRIPTION
//                        rt_kprintf("%-16s -- %s\n", index->name, index->desc);
//#else
                rt_kprintf("%s\n", module_cmd->CmdTblPtr[index].Name);
//#endif
            }
        }
        module_cmd = module_cmd->NextModuleCmdPtr;
    }


    if (name_ptr != NULL)
    {
        strncpy(prefix, name_ptr, min_length);
    }

    rt_kprintf("%s%s", FINSH_PROMPT, prefix);
}


#ifdef FINSH_USING_HISTORY
static CPU_INT08U shell_handle_history(struct finsh_shell* shell)
{

    rt_kprintf("\033[2K\r");

    rt_kprintf("%s%s", FINSH_PROMPT, shell->line);
    return 0;
}

static void shell_push_history(struct finsh_shell* shell)
{
    if (shell->line_position != 0)
    {
        /* push history */
        if (shell->history_count >= FINSH_HISTORY_LINES)
        {
            /* move history */
            int index;
            for (index = 0; index < FINSH_HISTORY_LINES - 1; index ++)
            {
                memcpy(&shell->cmd_history[index][0],
                       &shell->cmd_history[index + 1][0], FINSH_CMD_SIZE);
            }
            memset(&shell->cmd_history[index][0], 0, FINSH_CMD_SIZE);
            memcpy(&shell->cmd_history[index][0], shell->line, shell->line_position);

            /* it's the maximum history */
            shell->history_count = FINSH_HISTORY_LINES;
        }
        else
        {
            memset(&shell->cmd_history[shell->history_count][0], 0, FINSH_CMD_SIZE);
            memcpy(&shell->cmd_history[shell->history_count][0], shell->line, shell->line_position);

            /* increase count and set current history position */
            shell->history_count ++;
        }
    }
    shell->current_history = shell->history_count;
}
#endif






void Ser_RxTask(void)
{
    CPU_INT08U ret;
    char ch;
    OS_ERR err;
#ifdef USE_BUF_PRINTF    
    OS_PEND_DATA  pend_data_tbl[2];
#endif    
//    SHELL_ERR err;
//    RX_CALLBACK call_back;
//    void *p_arg;
//     CPU_SR cpu_sr;
    struct finsh_shell *shell = &_shell;
    memset(shell, 0, sizeof(struct finsh_shell));



    while(1)
    {
#ifdef USE_BUF_PRINTF
        pend_data_tbl[0].PendObjPtr = (OS_PEND_OBJ *)&BSP_SerTxBufWait;
        pend_data_tbl[1].PendObjPtr = (OS_PEND_OBJ *)&BSP_SerRxWait;
        OSPendMulti(pend_data_tbl, 2, 0, OS_OPT_PEND_BLOCKING, &err);

        while(1)
        {
            Buf_IsEmpty(&tx_buf_ring, ret);
            if(ret == Buff_isEmpty)
                break;

            Buf_Pop(&tx_buf_ring, ch);

//            while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == 0);
            BSP_OS_SemWait(&BSP_SerTxWait, 0);
            OSSemSet(&BSP_SerTxWait, 0, &err);
            USART_SendData(USART2, ch);
            USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
        }



#else
        BSP_OS_SemWait(&BSP_SerRxWait, 0);
#endif


        Buf_IsEmpty(&rx_buf_ring, ret);
        if(ret == Buff_isEmpty)
            continue;

        Buf_Pop(&rx_buf_ring, ch);



        /*
         * handle control key
         * up key  : 0x1b 0x5b 0x41
         * down key: 0x1b 0x5b 0x42
         * right key:0x1b 0x5b 0x43
         * left key: 0x1b 0x5b 0x44
         */
        if (ch == 0x1b)
        {
            shell->stat = WAIT_SPEC_KEY;
            continue;
        }
        else if (shell->stat == WAIT_SPEC_KEY)
        {
            if (ch == 0x5b)
            {
                shell->stat = WAIT_FUNC_KEY;
                continue;
            }

            shell->stat = WAIT_NORMAL;
        }
        else if (shell->stat == WAIT_FUNC_KEY)
        {
            shell->stat = WAIT_NORMAL;

            if (ch == 0x41) /* up key */
            {
#ifdef FINSH_USING_HISTORY
                /* prev history */
                if (shell->current_history > 0)
                    shell->current_history --;
                else
                {
                    shell->current_history = 0;
                    continue;
                }

                /* copy the history command */
                memcpy(shell->line, &shell->cmd_history[shell->current_history][0],
                       FINSH_CMD_SIZE);
                shell->line_curpos = shell->line_position = strlen(shell->line);
                shell_handle_history(shell);
#endif
                continue;
            }
            else if (ch == 0x42) /* down key */
            {
#ifdef FINSH_USING_HISTORY
                /* next history */
                if (shell->current_history < shell->history_count - 1)
                    shell->current_history ++;
                else
                {
                    /* set to the end of history */
                    if (shell->history_count != 0)
                        shell->current_history = shell->history_count - 1;
                    else
                        continue;
                }

                memcpy(shell->line, &shell->cmd_history[shell->current_history][0],
                       FINSH_CMD_SIZE);
                shell->line_curpos = shell->line_position = strlen(shell->line);
                shell_handle_history(shell);
#endif
                continue;
            }
            else if (ch == 0x44) /* left key */
            {
                if (shell->line_curpos)
                {
                    rt_kprintf("\b");
                    shell->line_curpos --;
                }

                continue;
            }
            else if (ch == 0x43) /* right key */
            {
                if (shell->line_curpos < shell->line_position)
                {
                    rt_kprintf("%c", shell->line[shell->line_curpos]);
                    shell->line_curpos ++;
                }

                continue;
            }

        }

        /* handle CR key */
        if (ch == '\r')
        {
//              char next;

//               if (rt_device_read(shell->device, 0, &next, 1) == 1)
//                   ch = next;
//               else ch = '\r';
        }
        /* handle tab key */
        else if (ch == '\t')
        {
            int i;
            /* move the cursor to the beginning of line */
            for (i = 0; i < shell->line_curpos; i++)
                rt_kprintf("\b");

            /* auto complete */
            shell_auto_complete(&shell->line[0]);
            /* re-calculate position */
            shell->line_curpos = shell->line_position = strlen(shell->line);

            continue;
        }
        /* handle backspace key */
        else if (ch == 0x7f || ch == 0x08)
        {
            /* note that shell->line_curpos >= 0 */
            if (shell->line_curpos == 0)
                continue;

            shell->line_position--;
            shell->line_curpos--;

            if (shell->line_position > shell->line_curpos)
            {
                int i;

                memmove(&shell->line[shell->line_curpos],
                        &shell->line[shell->line_curpos + 1],
                        shell->line_position - shell->line_curpos);
                shell->line[shell->line_position] = 0;

                rt_kprintf("\b%s  \b", &shell->line[shell->line_curpos]);

                /* move the cursor to the origin position */
                for (i = shell->line_curpos; i <= shell->line_position; i++)
                    rt_kprintf("\b");
            }
            else
            {
                rt_kprintf("\b \b");
                shell->line[shell->line_position] = 0;
            }

            continue;
        }

        /* handle end of line, break */
        if (ch == '\r' || ch == '\n')
        {
#ifdef FINSH_USING_HISTORY
            shell_push_history(shell);
#endif

#ifdef FINSH_USING_MSH
            if (msh_is_used() == RT_TRUE)
            {
                rt_kprintf("\n");
                msh_exec(shell->line, shell->line_position);
            }
            else
#endif
            {
#ifndef FINSH_USING_MSH_ONLY
                /* add ';' and run the command line */
                shell->line[shell->line_position] = '\0';

                if (shell->line_position != 0)
                {

                    rt_kprintf("\n");
                    Shell_Exec(shell->line, NULL, NULL, &err);
//                       finsh_run_line(&shell->parser, shell->line);
                }
                else rt_kprintf("\n");
#endif
            }

            rt_kprintf(FINSH_PROMPT);
            memset(shell->line, 0, sizeof(shell->line));
            shell->line_curpos = shell->line_position = 0;
            continue;//break;
        }

        /* it's a large line, discard it */
        if (shell->line_position >= FINSH_CMD_SIZE)
            shell->line_position = 0;

        /* normal character */
        if (shell->line_curpos < shell->line_position)
        {
            int i;

            memmove(&shell->line[shell->line_curpos + 1],
                    &shell->line[shell->line_curpos],
                    shell->line_position - shell->line_curpos);
            shell->line[shell->line_curpos] = ch;
            if (1/*shell->echo_mode*/)
                rt_kprintf("%s", &shell->line[shell->line_curpos]);

            /* move the cursor to new position */
            for (i = shell->line_curpos; i < shell->line_position; i++)
                rt_kprintf("\b");
        }
        else
        {
            shell->line[shell->line_position] = ch;
            rt_kprintf("%c", ch);
        }

        ch = 0;
        shell->line_position ++;
        shell->line_curpos++;
        if (shell->line_position >= 80)
        {
            /* clear command line */
            shell->line_position = 0;
            shell->line_curpos = 0;
        }

    }
}





void  BSP_Ser_WrByteUnlocked (CPU_INT08U c)
{
#ifdef USE_BUF_PRINTF
    putchar(c);
#else
    USART_SendData(USART2, c);

    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    BSP_OS_SemWait(&BSP_SerTxWait, 10);
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
#endif
}


void  BSP_Ser_WrByte(CPU_INT08U  c)
{
    BSP_OS_SemWait(&BSP_SerLock, 0);                            /* Obtain access to the serial interface              */

    BSP_Ser_WrByteUnlocked(c);

    BSP_OS_SemPost(&BSP_SerLock);                               /* Release access to the serial interface             */
}


void  BSP_Ser_WrStr (CPU_CHAR  *p_str)
{
    CPU_BOOLEAN  err;

    err = BSP_OS_SemWait(&BSP_SerLock, 0);                      /* Obtain access to the serial interface              */

    if (err != DEF_OK )
    {
        return;
    }

    while ((*p_str) != (CPU_CHAR )0)
    {

        if (*p_str == ASCII_CHAR_LINE_FEED)
        {
            BSP_Ser_WrByteUnlocked(ASCII_CHAR_CARRIAGE_RETURN);
            BSP_Ser_WrByteUnlocked(ASCII_CHAR_LINE_FEED);
            p_str++;
        }
        else
        {
            BSP_Ser_WrByteUnlocked(*p_str++);
        }
    }

    BSP_OS_SemPost(&BSP_SerLock);                               /* Release access to the serial interface             */
}

void  BSP_Ser_Printf (CPU_CHAR *format, ...)
{
    CPU_CHAR  buf_str[BSP_SER_PRINTF_STR_BUF_SIZE + 1u];
    va_list   v_args;


    va_start(v_args, format);
    (void)vsnprintf((char       *)&buf_str[0],
                    (CPU_SIZE_T  ) sizeof(buf_str),
                    (char const *) format,
                    v_args);
    va_end(v_args);

    BSP_Ser_WrStr((CPU_CHAR *)&buf_str[0]);
}






//#pragma import(__use_no_semihosting)
#if 1
//#pragma import(__use_no_heap)
#pragma import(__use_no_semihosting)
// struct __FILE
// {
//     int handle;
//     /* Whatever you require here. If the only file you are using is */
//     /* standard output using printf() for debugging, no file handling */
//     /* is required. */
// };
/* FILE is typedef’ d in stdio.h. */
//FILE __stdout;

struct __FILE
{
    int handle;
} ;

FILE __stdout;
FILE __stdin;
FILE __stderr;

void _sys_exit(int x)
{
    x = x;
}


// int fclose(FILE* f) {
//   return (0);
// }


// int fseek (FILE *f, long nPos, int nMode)  {
//   return (0);
// }


int fflush (FILE *f)  {
  return (0);
}

// int ferror(FILE *f) {
//   /* Your implementation of ferror */
//   return (EOF);
// }


void _ttywrch(int ch)
{
    while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == 0);
    USART_SendData(USART2, ch);
    while(USART_GetFlagStatus(USART2,USART_FLAG_TC) == 0);
}

int fputc(int ch, FILE *f)
{   
#ifdef USE_BUF_PRINTF
    CPU_INT08U ret;
    OS_ERR err;
    CPU_SR cpu_sr;
    
    CPU_CRITICAL_ENTER();

    if(ch == '\n')
    {
        Buf_IsFull(&tx_buf_ring, ret);
        if(ret == Buff_notFull)
            Buf_Push(&tx_buf_ring, '\r');
    }

    Buf_IsFull(&tx_buf_ring, ret);
    if(ret == Buff_notFull)
    {
        Buf_Push(&tx_buf_ring, (ch&0xff));

        CPU_CRITICAL_EXIT();

        OSSemPost(&BSP_SerTxBufWait,OS_OPT_POST_1,&err);

        return ch;
    }
    CPU_CRITICAL_EXIT();

    OSSemPost(&BSP_SerTxBufWait,OS_OPT_POST_1,&err);

#else
    if(ch == '\n')
    {
        while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == 0);
        USART_SendData(USART2, '\r');
        while(USART_GetFlagStatus(USART2,USART_FLAG_TC) == 0);
    }

    while(USART_GetFlagStatus(USART2,USART_FLAG_TXE) == 0);
    USART_SendData(USART2, ch);
    while(USART_GetFlagStatus(USART2,USART_FLAG_TC) == 0);
#endif

    return ch;
}
#endif


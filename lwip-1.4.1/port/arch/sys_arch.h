/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 *
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#ifndef __SYS_UCOS_III_H__
#define __SYS_UCOS_III_H__

#include "os.h"

//#define SYS_MBOX_NULL (osMessageQId)0
//#define SYS_SEM_NULL  (osSemaphoreId)0

//not less than 2
#define LWIP_THREAD_NUM_MAX   2
#define LWIP_THREAD_STK_SIZE  512

#define LWIP_COMPAT_MUTEX 0
#define sys_msleep      sys_ms_delay

#define SYS_ARCH_DECL_PROTECT(lev)  CPU_SR cpu_sr
#define SYS_ARCH_PROTECT(lev)       CPU_CRITICAL_ENTER()
#define SYS_ARCH_UNPROTECT(lev)     CPU_CRITICAL_EXIT()


typedef OS_SEM sys_sem_t;
typedef OS_MUTEX sys_mutex_t;
typedef OS_Q  sys_mbox_t;
typedef OS_TCB* sys_thread_t;


#define SYS_MBOX_NULL (sys_mbox_t *)NULL 
#define SYS_SEM_NULL  (sys_sem_t *)NULL


#endif /* __SYS_UCOS_III_H__ */


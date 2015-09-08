#ifndef _THREAD_H
#define _THREAD_H

#include "kthread.h"


#define APP_MAGIC_1    (unsigned int)0x12345678
#define APP_MAGIC_2    (unsigned int)0xABCDEFAB
#define APP_MAGIC_3    (unsigned int)0xEEC123DA


struct app_task_info
{
//    struct task_struct *main_task;
    unsigned int magic[3];
    const char *version_use;
    char *rom_base_addr;
    char *rom_code_ro_length;
    char *ram_rw_length;
    
    OS_PRIO prio;
    char *name;
//    CPU_STK *stk_addr;
//    CPU_STK_SIZE stk_size;
    void (*entry)(void);
    void (*exit)(void);    
};

extern struct list_head app_task_list_head;


struct task_struct * app_exec(struct app_task_info *info);

void app_exit_self(void);


int app_exit(struct task_struct *main_task);


int app_exit_by_romaddr(unsigned int romaddr);
int app_exit_by_name(char *app_name);





void thread_create (struct task_struct * task,
                        CPU_CHAR      *p_name,
                        OS_TASK_PTR    p_task,
                        void          *p_arg,
                        OS_PRIO        prio,
                        CPU_STK       *p_stk_base,
                        CPU_STK_SIZE   stk_size);



#endif


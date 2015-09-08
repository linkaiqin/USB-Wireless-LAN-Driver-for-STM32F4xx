#include "thread.h"
#include "list.h"





#define APP_NUM_MAX                        2  //must >=2
#define APP_MAIN_THREAD_STK_SIZE         512


static CPU_STK AppMainThreadStkPool[APP_NUM_MAX][APP_MAIN_THREAD_STK_SIZE];
static struct task_struct AppMainThreadPool[APP_NUM_MAX];
static OS_MEM AppMainThreadStkMem,AppMainThreadTaskMem;

static OS_SEM AppExecLock;





LIST_HEAD(app_task_list_head);





void __app_thread_del_all(struct task_struct *main_task)
{
    struct task_struct *task,*next;
    OS_ERR err;

    //delete thread
    list_for_each_entry_safe(task, next, &main_task->thread_group, thread_group)
    {
        OSTaskDel(&task->tcb, &task->exit_err);
        list_del(&task->thread_group);
    }
    //delete main_thread(main_task)
    OSTaskDel(&main_task->tcb, &main_task->exit_err);
    //remove from app_task_list_head
    list_del(&task->task_list);
    OSMemPut(&AppMainThreadTaskMem,main_task,&err);
    OSMemPut(&AppMainThreadStkMem,main_task->stk,&err);
}


void app_thread_del_all(struct task_struct *main_task)
{
    OS_ERR err;

    OSSchedLock(&err);
    __app_thread_del_all(main_task);
    OSSchedUnlock(&err);
}



void app_main_thread(void *p_arg)
{
    struct task_struct *main_task;
    OS_ERR err;

    main_task = (struct task_struct *)p_arg;
    //jump to app entry
    main_task->app_task_info->entry();


//    if(AppExecLock.Type != OS_OBJ_TYPE_SEM)
//    {
//        OSSemCreate(&AppExecLock, "AppExecLock", 1, &err);
//    }

    OSSemPend(&AppExecLock, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    printk("%s exit in %s\r\n",main_task->name,__FUNCTION__);

    OSSchedLock(&err);
    __app_thread_del_all(main_task);
    if(main_task->app_task_info->exit)
        main_task->app_task_info->exit();
    OSSemPost(&AppExecLock, OS_OPT_POST_1, &err);
    OSSchedUnlock(&err);
}



//used in app self
void app_exit_self()
{
    struct task_struct *main_task;
    OS_ERR err;


    OSSemPend(&AppExecLock, 0, OS_OPT_PEND_BLOCKING, NULL, &err);
    main_task = current->group_leader;
    printk("%s exit in %s\r\n",main_task->name,__FUNCTION__);

    OSSchedLock(&err);
    __app_thread_del_all(main_task);
    if(main_task->app_task_info->exit)
        main_task->app_task_info->exit();
    OSSemPost(&AppExecLock, OS_OPT_POST_1, &err);
    OSSchedUnlock(&err);
}

void __app_exit(struct task_struct *main_task)
{
    printk("%s exit in %s\r\n",main_task->name,__FUNCTION__);
    app_thread_del_all(main_task);
    if(main_task->app_task_info->exit)
        main_task->app_task_info->exit();
}

int app_exit_by_romaddr(unsigned int romaddr)
{
    OS_ERR err;
    struct task_struct *main_task = NULL;
    
    if(AppExecLock.Type != OS_OBJ_TYPE_SEM)
    {
        OSSemCreate(&AppExecLock, "AppExecLock", 1, &err);
    }

    OSSemPend(&AppExecLock, 0, OS_OPT_PEND_BLOCKING, NULL, &err);


    list_for_each_entry(main_task, &app_task_list_head, task_list)
    {
        if((unsigned int)(main_task->app_task_info->rom_base_addr) == romaddr)
        {
            __app_exit(main_task);
            OSSemPost(&AppExecLock, OS_OPT_POST_1, &err);
            return 0;
        }
    }

    OSSemPost(&AppExecLock, OS_OPT_POST_1, &err);
    return -1;
}


int app_exit_by_name(char *app_name)
{
    OS_ERR err;
    struct task_struct *main_task = NULL;
    
    if(AppExecLock.Type != OS_OBJ_TYPE_SEM)
    {
        OSSemCreate(&AppExecLock, "AppExecLock", 1, &err);
    }

    OSSemPend(&AppExecLock, 0, OS_OPT_PEND_BLOCKING, NULL, &err);


    list_for_each_entry(main_task, &app_task_list_head, task_list)
    {
        if(!strcmp(main_task->name, app_name))
        {
            __app_exit(main_task);
            OSSemPost(&AppExecLock, OS_OPT_POST_1, &err);
            return 0;
        }
    }

    OSSemPost(&AppExecLock, OS_OPT_POST_1, &err);
    return -1;
}


int app_exit(struct task_struct *main_task)
{
    OS_TCB *tcb;
    OS_ERR err;
    CPU_SR cpu_sr;

    if(AppExecLock.Type != OS_OBJ_TYPE_SEM)
    {
        OSSemCreate(&AppExecLock, "AppExecLock", 1, &err);
    }

    OSSemPend(&AppExecLock, 0, OS_OPT_PEND_BLOCKING, NULL, &err);


    OS_CRITICAL_ENTER();
    tcb = OSTaskDbgListPtr;
    while(tcb)
    {
        if(tcb == &main_task->tcb)
        {
            break;
        }
        tcb = tcb->DbgNextPtr;
    }
    //app is not running or is deleted.
    if(tcb == NULL)
    {
        OS_CRITICAL_EXIT();
        OSSemPost(&AppExecLock, OS_OPT_POST_1, &err);
        return -1;
    }
    OS_CRITICAL_EXIT();

    __app_exit(main_task);

    OSSemPost(&AppExecLock, OS_OPT_POST_1, &err);
    return 0;
}


struct task_struct* app_exec(struct app_task_info *info)
{
    struct task_struct *task;
    CPU_STK *stk;
    OS_TCB *tcb;
    static char is_first = 1;
    OS_ERR err;
    CPU_SR cpu_sr;

    if(!info)
        return NULL;

    if(AppExecLock.Type != OS_OBJ_TYPE_SEM)
    {
        OSSemCreate(&AppExecLock, "AppExecLock", 1, &err);
    }


    OSSemPend(&AppExecLock, 0, OS_OPT_PEND_BLOCKING, NULL, &err);


    OS_CRITICAL_ENTER();
    tcb = OSTaskDbgListPtr;
    while(tcb)
    {
        if(tcb->NamePtr == info->name)
        {
            OS_CRITICAL_EXIT();
            task = container_of(tcb,struct task_struct,tcb);

            app_thread_del_all(task);
            if(task->app_task_info->exit)
                task->app_task_info->exit();

            break;
        }
        tcb = tcb->DbgNextPtr;
    }

    if(tcb == NULL)
    {
        OS_CRITICAL_EXIT();
    }


    if(is_first)
    {
        is_first = 0;
        OSMemCreate(&AppMainThreadTaskMem, "AppMainThreadTaskMem", &AppMainThreadPool[0], APP_NUM_MAX, sizeof(struct task_struct), &err);
        if(err != OS_ERR_NONE)
        {
            printk("OSMemCreate AppMainThreadTaskMem Failed!!! %d\r\n",err);
            OSSemPost(&AppExecLock, OS_OPT_POST_1, &err);
            return NULL;
        }
        OSMemCreate(&AppMainThreadStkMem, "AppMainThreadStkMem", &AppMainThreadStkPool[0][0], APP_NUM_MAX, APP_MAIN_THREAD_STK_SIZE*sizeof(CPU_STK), &err);
        if(err != OS_ERR_NONE)
        {
            printk("OSMemCreate AppMainThreadStkMem Failed!!! %d\r\n",err);
            OSSemPost(&AppExecLock, OS_OPT_POST_1, &err);
            return NULL;
        }
    }

    task = OSMemGet(&AppMainThreadTaskMem, &err);
    if(err != OS_ERR_NONE)
    {
        printk("OSMemGet AppMainThreadTaskMem Failed!!! %d\r\n",err);
        OSSemPost(&AppExecLock, OS_OPT_POST_1, &err);
        return NULL;
    }
    stk = OSMemGet(&AppMainThreadStkMem, &err);
    if(err != OS_ERR_NONE)
    {
        printk("OSMemGet AppMainThreadStkMem Failed!!! %d\r\n",err);
        OSSemPost(&AppExecLock, OS_OPT_POST_1, &err);
        return NULL;
    }


    //make pid_nr > 0
    task->app_task_info = info;
    task->pid.pid_nr = (int)(((unsigned int)&task->tcb) & (unsigned int)(~((unsigned int)1 << 31)));
    task->prio = info->prio;
    task->stk = stk;
    task->is_killed = 0;
    task->exit_err = OS_ERR_OS_RUNNING; //debug
    task->name = info->name;
    task->pids[PIDTYPE_PID].pid = &task->pid;

    task->group_leader = task;
    INIT_LIST_HEAD(&task->thread_group);

    INIT_LIST_HEAD(&task->task_list);
    list_add_tail(&task->task_list, &app_task_list_head);
    OSTaskCreate((OS_TCB       *)&task->tcb,              /* Create the start task                                */
                 (CPU_CHAR     *)info->name,
                 (OS_TASK_PTR   )app_main_thread,
                 (void         *)task,
                 (OS_PRIO       )info->prio,
                 (CPU_STK      *)stk,
                 (CPU_STK_SIZE  )(APP_MAIN_THREAD_STK_SIZE/ 10u),
                 (CPU_STK_SIZE  )APP_MAIN_THREAD_STK_SIZE,
                 (OS_MSG_QTY    )5u,
                 (OS_TICK       )0u,
                 (void         *)0u,
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR       *)&err);


    OSSemPost(&AppExecLock, OS_OPT_POST_1, &err);

    return task;
}











void thread_create (struct task_struct * task,
                    CPU_CHAR      *p_name,
                    OS_TASK_PTR    p_task,
                    void          *p_arg,
                    OS_PRIO        prio,
                    CPU_STK       *p_stk_base,
//                        CPU_STK_SIZE   stk_limit,
                    CPU_STK_SIZE   stk_size)
//                        OS_MSG_QTY     q_size,
//                        OS_TICK        time_quanta,
//                        void          *p_ext,
//                        OS_OPT         opt,
//                        OS_ERR        *p_err
{
    OS_ERR err;
    CPU_SR cpu_sr;

    //make pid_nr > 0
    task->pid.pid_nr = (int)(((unsigned int)&task->tcb) & (unsigned int)(~((unsigned int)1 << 31)));
    task->prio = prio;
    task->stk = p_stk_base;
    task->is_killed = 0;
    task->exit_err = OS_ERR_OS_RUNNING; //debug
    task->name = p_name;
    task->pids[PIDTYPE_PID].pid = &task->pid;

    INIT_LIST_HEAD(&task->thread_group);
    task->group_leader = current->group_leader;

    CPU_CRITICAL_ENTER();
    list_add_tail(&task->thread_group, &task->group_leader->thread_group);
    CPU_CRITICAL_EXIT();

    OSTaskCreate((OS_TCB       *)&task->tcb,              /* Create the  task                                */
                 (CPU_CHAR     *)p_name,
                 (OS_TASK_PTR   )p_task,
                 (void         *)p_arg,
                 (OS_PRIO       )prio,
                 (CPU_STK      *)p_stk_base,
                 (CPU_STK_SIZE  )(stk_size / 10),
                 (CPU_STK_SIZE  )stk_size,
                 (OS_MSG_QTY    )5u,
                 (OS_TICK       )0u,
                 (void         *)0u,
                 (OS_OPT        )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                 (OS_ERR       *)&err);
    if(err != OS_ERR_NONE)
    {
        printk("thread_create OSTaskCreate Failed:%d stop!!\r\n",err);
        while(1);
    }
}






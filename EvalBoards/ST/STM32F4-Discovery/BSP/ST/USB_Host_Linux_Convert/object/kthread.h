#ifndef _KTHREAD_H
#define _KTHREAD_H
#include "list.h"
#include "os.h"
#include "misc_cvt.h"
#include "fs_cvt.h"

/*
 * cloning flags:
 */
#define CSIGNAL		0x000000ff	/* signal mask to be sent at exit */
#define CLONE_VM	0x00000100	/* set if VM shared between processes */
#define CLONE_FS	0x00000200	/* set if fs info shared between processes */
#define CLONE_FILES	0x00000400	/* set if open files shared between processes */
#define CLONE_SIGHAND	0x00000800	/* set if signal handlers and blocked signals shared */
#define CLONE_PTRACE	0x00002000	/* set if we want to let tracing continue on the child too */
#define CLONE_VFORK	0x00004000	/* set if the parent wants the child to wake it up on mm_release */
#define CLONE_PARENT	0x00008000	/* set if we want to have the same parent as the cloner */
#define CLONE_THREAD	0x00010000	/* Same thread group? */
#define CLONE_NEWNS	0x00020000	/* New namespace group? */
#define CLONE_SYSVSEM	0x00040000	/* share system V SEM_UNDO semantics */
#define CLONE_SETTLS	0x00080000	/* create a new TLS for the child */
#define CLONE_PARENT_SETTID	0x00100000	/* set the TID in the parent */
#define CLONE_CHILD_CLEARTID	0x00200000	/* clear the TID in the child */
#define CLONE_DETACHED		0x00400000	/* Unused, ignored */
#define CLONE_UNTRACED		0x00800000	/* set if the tracing process can't force CLONE_PTRACE on this clone */
#define CLONE_CHILD_SETTID	0x01000000	/* set the TID in the child */
#define CLONE_STOPPED		0x02000000	/* Start in stopped state */
#define CLONE_NEWUTS		0x04000000	/* New utsname group? */
#define CLONE_NEWIPC		0x08000000	/* New ipcs */
#define CLONE_NEWUSER		0x10000000	/* New user namespace */
#define CLONE_NEWPID		0x20000000	/* New pid namespace */
#define CLONE_NEWNET		0x40000000	/* New network namespace */
#define CLONE_IO		0x80000000	/* Clone io context */


/*
 * Linux/AXP has different signal numbers that Linux/i386: I'm trying
 * to make it OSF/1 binary compatible, at least for normal binaries.
 */
#define SIGHUP		 1
#define SIGINT		 2
#define SIGQUIT		 3
#define SIGILL		 4
#define SIGTRAP		 5
#define SIGABRT		 6
#define SIGEMT		 7
#define SIGFPE		 8
#define SIGKILL		 9
#define SIGBUS		10
#define SIGSEGV		11
#define SIGSYS		12
#define SIGPIPE		13
#define SIGALRM		14
#define SIGTERM		15
#define SIGURG		16
#define SIGSTOP		17
#define SIGTSTP		18
#define SIGCONT		19
#define SIGCHLD		20
#define SIGTTIN		21
#define SIGTTOU		22
#define SIGIO		23
#define SIGXCPU		24
#define SIGXFSZ		25
#define SIGVTALRM	26
#define SIGPROF		27
#define SIGWINCH	28
#define SIGINFO		29
#define SIGUSR1		30
#define SIGUSR2		31






#define kernel_thread(fn, arg, flags)  __kernel_thread(pTask->taskName,fn, arg, flags)

typedef int pid_t;

enum pid_type
{
	PIDTYPE_PID,
	PIDTYPE_PGID,
	PIDTYPE_SID,
	PIDTYPE_MAX
};

struct pid
{
    int pid_nr;
    
//	atomic_t count;
//	unsigned int level;
//	/* lists of tasks that use this pid */
//	struct hlist_head tasks[PIDTYPE_MAX];
//	struct rcu_head rcu;
//	struct upid numbers[1];
};



struct pid_link
{
	struct pid *pid;
};


struct task_struct {
    OS_TCB tcb;    
    OS_PRIO prio;
    CPU_STK *stk; 
    struct pid pid;    

    struct app_task_info *app_task_info;
	struct task_struct *group_leader;	/* threadgroup leader */    
	struct list_head thread_group;
    
	/* PID/PID hash table linkage. */
	struct pid_link pids[PIDTYPE_MAX];
    char is_killed;
    char *name;
    OS_ERR exit_err;
    struct list_head task_list; 
    mm_segment_t addr_limit;
};






int kill_pid(struct pid *pid, int sig, int priv);
pid_t pid_nr(struct pid *pid);
pid_t __kernel_thread(char *name, int (*fn)(void *), void *arg, unsigned long flags);
void kthread_exit(long code);
void kthread_del(struct task_struct *task);




static inline struct task_struct *get_current(void)
{
    struct task_struct *task;
    OS_TCB *tcb;
    CPU_SR cpu_sr;



    CPU_CRITICAL_ENTER();
    tcb = OSTCBCurPtr;
    CPU_CRITICAL_EXIT();
    
    task  = container_of(tcb,struct task_struct,tcb);
    return task;
}


#define current		get_current()







#endif

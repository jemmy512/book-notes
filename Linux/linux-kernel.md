# Table of Contents
* [Init](#Init)
    * [CPU](#cpu)
* [Process Management](#Process-Management)
    * [process](#process)
    * [thread](#thread)
    * [task_struct](#task_struct)
    * [schedule](#schedule)
    * [voluntary schedule](#voluntary-schedule)
    * [preempt schedule](#preempt-schedule)
        * [preempt time](#preempt-time)
            * [clock interrupt](#clock-interrupt)
            * [ttwu](#ttwu)
        * [real user preempt time](#real-user-preempt-time)
            * [return from system call](#return-from-system-call)
            * [return from interrupt](#return-from-interrupt)
        * [real kernel preempt time](#real-kernel-preempt-time)
            * [preempt_enable](#preempt_enble)
            * [return from interrupt](#return-from-interrupt)
    * [wake_up](#wake_up)
    * [wait_woken](#wait_woken)
    * [fork](#fork)
    * [exec](#exec)
* [Memory Management](./linux-kernel-mem.md)
* [File Management](#File-Management)
    * [inode](#inode)
    * [extent](#extent)
    * [block group](#block-group)
    * [directory](#directory)
    * [hard/symbolic link](#hardsymbolic-link)
    * [vfs](#vfs)
    * [mount](#mount)
    * [open](#open)
    * [read/write](#readwrite)
        * [direct_IO](#direct_IO)
        * [buffered write](#buffered-write)
            * [bdi_wq](#bdi_wq)
        * [buffered read](#buffered-read)
* [Net](./linux-kernel-net.md)
* [IO](#IO)
* [IPC](#IPC)
* [Virtualization](#Virtualization)
* [Containerization](#Containerization)
* [Lock](#Lock)
* [Pthread](#Pthread)
    * [pthread_create](#pthread_create)

# Init
### cpu
![linux-init-cpu.png](../Images/Kernel/init-cpu.png)

![](../Images/Kernel/init-cpu-2.png)

![linux-init-cpu-process-program.png](../Images/Kernel/init-cpu-process-program.png)

### bios
* ![](../Images/Kernel/init-bios.png)
* When power on, set CS to 0xFFFF, IP to 0x0000, the first instruction points to 0xFFFF0 within ROM, a JMP comamand will jump to ROM do init work, BIOS starts.
* Then BIOS checks the health state of each hardware.
* Grub2 (Grand Unified Bootloader Version 2)
  * grub2-mkconfig -o /boot/grub2/grub.cfg
    ```
    menuentry 'CentOS Linux (3.10.0-862.el7.x86_64) 7 (Core)' --class centos --class gnu-linux --class gnu --class os --unrestricted $menuentry_id_option 'gnulinux-3.10.0-862.el7.x86_64-advanced-b1aceb95-6b9e-464a-a589-bed66220ebee' {
      load_video
      set gfxpayload=keep
      insmod gzio
      insmod part_msdos
      insmod ext2 set root='hd0,msdos1'
      if [ x$feature_platform_search_hint = xy ]; then
        search --no-floppy --fs-uuid --set=root --hint='hd0,msdos1' b1aceb95-6b9e-464a-a589-bed66220ebee
      else search --no-floppy --fs-uuid --set=root b1aceb95-6b9e-464a-a589-bed66220ebee
      fi

      linux16 /boot/vmlinuz-3.10.0-862.el7.x86_64 root=UUID=b1aceb95-6b9e-464a-a589-bed66220ebee ro console=tty0 console=ttyS0,115200 crashkernel=auto net.ifnames=0 biosdevname=0 rhgb quiet
      initrd16 /boot/initramfs-3.10.0-862.el7.x86_64.img
    }
    ```
  * grub2-install /dev/sda
    * install boot.img into MBR（Master Boot Record), and load boot.img into memory at 0x7c00 to run
    * core.img: diskboot.img, lzma_decompress.img, kernel.img

* ```
  boot.img
    core.img
      diskboot.img // load other modules of grub into memory
        real_to_prot// enable segement, page, open Gate A20
        lzma_decompress.img
        kernel.img // grub's kernel img not Linux kernel
          grub_main // grub's main func
            grub_load_config()
            grub_command_execute ("normal", 0, 0)
              grub_normal_execute()
                grub_show_menu() // show which OS want to run
                  grub_menu_execute_entry()
  ```

### init kernel
```C++
// init/main.c
void start_kernel(void)
{
  /* struct task_struct init_task = INIT_TASK(init_task)
   * #0 process, the only one doesn't created by fork or kernel_thread */
  set_task_stack_end_magic(&init_task);

  /* set_system_intr_gate(IA32_SYSCALL_VECTOR, entry_INT80_32) */
  trap_init();

  /* mnt_init()->init_rootfs() register_filesystem(&rootfs_fs_type) */
  vfs_caches_init()

  mm_init();
  sched_init();
  init_IRQ();
  softirq_init();
  signals_init();
  cpuset_init();
  cgroup_init();

  rest_init()
}

static void rest_init(void)
{
  struct task_struct *tsk;
  int pid;

  pid = kernel_thread(kernel_init, NULL, CLONE_FS);

  pid = kernel_thread(kthreadd, NULL, CLONE_FS | CLONE_FILES);

  complete(&kthreadd_done);

  cpu_startup_entry(CPUHP_ONLINE);
}
```
## Q: sp points to `kernel_init` fn, should't it be ip?
```c++
pid_t kernel_thread(int (*fn)(void *), void *arg, unsigned long flags)
{
  return _do_fork(flags|CLONE_VM|CLONE_UNTRACED, (unsigned long)fn,
    (unsigned long)arg, NULL, NULL, 0);
}

/* return from kernel to user space */
static int kernel_init(void *unused)
{
  if (ramdisk_execute_command) {
    ret = run_init_process(ramdisk_execute_command);
    if (!ret)
      return 0;
  }

  if (execute_command) {
    ret = run_init_process(execute_command);
    if (!ret)
      return 0;
  }

  if (!try_to_run_init_process("/sbin/init") ||
      !try_to_run_init_process("/etc/init") ||
      !try_to_run_init_process("/bin/init") ||
      !try_to_run_init_process("/bin/sh"))
    return 0;
}

static int run_init_process(const char *init_filename)
{
  argv_init[0] = init_filename;
  return do_execve(getname_kernel(init_filename),
    (const char __user *const __user *)argv_init,
    (const char __user *const __user *)envp_init);
}
```
![linux-init-kernel.png](../Images/Kernel/init-kernel.png)

![linux-init-cpu-arch.png](../Images/Kernel/init-cpu-arch.png)

### syscall

#### glibc
```c++
int open(const char *pathname, int flags, mode_t mode)

// syscalls.list
// File name Caller  Syscall name    Args    Strong name    Weak names
      open    -        open          i:siv   __libc_open   __open open
```
```C++
// syscall-template.S
T_PSEUDO (SYSCALL_SYMBOL, SYSCALL_NAME, SYSCALL_NARGS)
    ret
T_PSEUDO_END (SYSCALL_SYMBOL)

#define T_PSEUDO(SYMBOL, NAME, N)    PSEUDO (SYMBOL, NAME, N)

#define PSEUDO(name, syscall_name, args)   \
  .text;                                  \
  ENTRY (name)                            \
    DO_CALL (syscall_name, args);         \
    cmpl $-4095, %eax;                    \
    jae SYSCALL_ERROR_LABEL
```

#### 32
```C++
/* Linux takes system call arguments in registers:
  syscall number  %eax       call-clobbered
  arg 1    %ebx       call-saved
  arg 2    %ecx       call-clobbered
  arg 3    %edx       call-clobbered
  arg 4    %esi       call-saved
  arg 5    %edi       call-saved
  arg 6    %ebp       call-saved */
#define DO_CALL(syscall_name, args) \
    PUSHARGS_##args                \
    DOARGS_##args                  \
    movl $SYS_ify (syscall_name), %eax; \ // get syscall id by syscall_name
    ENTER_KERNEL                        \
    POPARGS_##args

# define ENTER_KERNEL int $0x80

/* trap_init */
set_system_intr_gate(IA32_SYSCALL_VECTOR, entry_INT80_32);

ENTRY(entry_INT80_32)
        ASM_CLAC
        pushl   %eax  /* pt_regs->orig_ax */
        SAVE_ALL pt_regs_ax=$-ENOSYS /* save rest */
        movl    %esp, %eax
        call    do_syscall_32_irqs_on
.Lsyscall_32_done:

.Lirq_return:
  INTERRUPT_RETURN /* iret */

ENDPROC(entry_INT80_32)

static  void do_syscall_32_irqs_on(struct pt_regs *regs)
{
  struct thread_info *ti = current_thread_info();
  unsigned int nr = (unsigned int)regs->orig_ax;

  if (likely(nr < IA32_NR_syscalls)) {
    regs->ax = ia32_sys_call_table[nr](
      (unsigned int)regs->bx, (unsigned int)regs->cx,
      (unsigned int)regs->dx, (unsigned int)regs->si,
      (unsigned int)regs->di, (unsigned int)regs->bp);
  }

  syscall_return_slowpath(regs);
}

inline void syscall_return_slowpath(struct pt_regs *regs)
{
  struct thread_info *ti = current_thread_info();
  u32 cached_flags = READ_ONCE(ti->flags);

  if (IS_ENABLED(CONFIG_PROVE_LOCKING) &&
      WARN(irqs_disabled(), "syscall %ld left IRQs disabled", regs->orig_ax))
    local_irq_enable();

  rseq_syscall(regs);

  if (unlikely(cached_flags & SYSCALL_EXIT_WORK_FLAGS))
    syscall_slow_exit_work(regs, cached_flags);

  local_irq_disable();
  prepare_exit_to_usermode(regs);
}

inline void prepare_exit_to_usermode(struct pt_regs *regs)
{
  struct thread_info *ti = current_thread_info();
  u32 cached_flags;

  addr_limit_user_check();

  cached_flags = READ_ONCE(ti->flags);

  if (unlikely(cached_flags & EXIT_TO_USERMODE_LOOP_FLAGS))
    exit_to_usermode_loop(regs, cached_flags);

  user_enter_irqoff();

  mds_user_clear_cpu_buffers();
}

void exit_to_usermode_loop(struct pt_regs *regs, u32 cached_flags)
{
  while (true) {
    local_irq_enable();

    if (cached_flags & _TIF_NEED_RESCHED)
      schedule();

    if (cached_flags & _TIF_UPROBE)
      uprobe_notify_resume(regs);

    if (cached_flags & _TIF_PATCH_PENDING)
      klp_update_patch_state(current);

    /* deal with pending signal delivery */
    if (cached_flags & _TIF_SIGPENDING)
      do_signal(regs);

    if (cached_flags & _TIF_NOTIFY_RESUME) {
      clear_thread_flag(TIF_NOTIFY_RESUME);
      tracehook_notify_resume(regs);
      rseq_handle_notify_resume(NULL, regs);
    }

    if (cached_flags & _TIF_USER_RETURN_NOTIFY)
      fire_user_return_notifiers();

    /* Disable IRQs and retry */
    local_irq_disable();

    cached_flags = READ_ONCE(current_thread_info()->flags);

    if (!(cached_flags & EXIT_TO_USERMODE_LOOP_FLAGS))
      break;
  }
}
```
![linux-init-syscall-32.png](../Images/Kernel/init-syscall-32.png)

#### 64
```C++

/* The Linux/x86-64 kernel expects the system call parameters in
  registers according to the following table:
    syscall number  rax
    arg 1           rdi
    arg 2           rsi
    arg 3           rdx
    arg 4           r10
    arg 5           r8
    arg 6           r9 */
#define DO_CALL(syscall_name, args)  \
  lea SYS_ify (syscall_name), %rax; \
  syscall

/* Moduel Specific Register, trap_init -> cpu_init -> syscall_init */
wrmsrl(MSR_LSTAR, (unsigned long)entry_SYSCALL_64);

ENTRY(entry_SYSCALL_64)
  /* Construct struct pt_regs on stack */
  pushq   $__USER_DS                /* pt_regs->ss */
  pushq   PER_CPU_VAR(rsp_scratch)  /* pt_regs->sp */
  pushq   %r11                      /* pt_regs->flags */
  pushq   $__USER_CS                /* pt_regs->cs */
  pushq   %rcx                      /* pt_regs->ip */
  pushq   %rax                      /* pt_regs->orig_ax */
  pushq   %rdi                      /* pt_regs->di */
  pushq   %rsi                      /* pt_regs->si */
  pushq   %rdx                      /* pt_regs->dx */
  pushq   %rcx                      /* pt_regs->cx */
  pushq   $-ENOSYS                  /* pt_regs->ax */
  pushq   %r8                       /* pt_regs->r8 */
  pushq   %r9                       /* pt_regs->r9 */
  pushq   %r10                      /* pt_regs->r10 */
  pushq   %r11                      /* pt_regs->r11 */
  sub     $(6*8), %rsp              /* pt_regs->bp, bx, r12-15 not saved */
  movq    PER_CPU_VAR(current_task), %r11
  testl   $_TIF_WORK_SYSCALL_ENTRY|_TIF_ALLWORK_MASK, TASK_TI_flags(%r11)
  jnz     entry_SYSCALL64_slow_path

entry_SYSCALL64_slow_path:
  /* IRQs are off. */
  SAVE_EXTRA_REGS
  movq    %rsp, %rdi
  call    do_syscall_64           /* returns with IRQs disabled */

return_from_SYSCALL_64:
  RESTORE_EXTRA_REGS
  TRACE_IRQS_IRETQ
  movq  RCX(%rsp), %rcx
  movq  RIP(%rsp), %r11
  movq  R11(%rsp), %r11

syscall_return_via_sysret:
  /* rcx and r11 are already restored (see code above) */
  RESTORE_C_REGS_EXCEPT_RCX_R11
  movq  RSP(%rsp), %rsp
  USERGS_SYSRET64
END(entry_SYSCALL_64)

#define USERGS_SYSRET64 \
  swapgs;              \
  sysretq;

/* entry_SYSCALL_64 -> entry_SYSCALL64_slow_pat -> do_syscall_64 */
void do_syscall_64(struct pt_regs *regs)
{
  struct thread_info *ti = current_thread_info();
  unsigned long nr = regs->orig_ax;

  if (likely((nr & __SYSCALL_MASK) < NR_syscalls)) {
    regs->ax = sys_call_table[nr & __SYSCALL_MASK] (
      regs->di, regs->si, regs->dx,
      regs->r10, regs->r8, regs->r9
    );
  }

  syscall_return_slowpath(regs);
}
```
![linux-init-syscall-64.png](../Images/Kernel/init-syscall-64.png)


# Process Management
### process
![linux-proc-compile.png](../Images/Kernel/proc-compile.png)
```C++
/* compile */
gcc -c -fPIC process.c
gcc -c -fPIC createprocess.c

/* staic lib */
ar cr libstaticprocess.a process.o
/* static link */
gcc -o staticcreateprocess createprocess.o -L. -lstaticprocess

/* dynamic lib */
gcc -shared -fPIC -o libdynamicprocess.so process.o
/* dynamic link LD_LIBRARY_PATH /lib /usr/lib */
gcc -o dynamiccreateprocess createprocess.o -L. -ldynamicprocess
export LD_LIBRARY_PATH=.
```

1. elf: relocatable file
![linux-proc-elf-relocatable.png](../Images/Kernel/proc-elf-relocatable.png)

2. elf: executable file
![linux-proc-elf-executable.png](../Images/Kernel/proc-elf-executable.png)

3. elf: shared object
![linux-proc-elf-sharedobj.png]()

```C++
struct linux_binfmt {
  struct list_head lh;
  struct module *module;
  int (*load_binary)(struct linux_binprm *);
  int (*load_shlib)(struct file *);
  int (*core_dump)(struct coredump_params *cprm);
  unsigned long min_coredump;     /* minimal dump size */
};

static struct linux_binfmt elf_format = {
  .module         = THIS_MODULE,
  .load_binary    = load_elf_binary,
  .load_shlib     = load_elf_library,
  .core_dump      = elf_core_dump,
  .min_coredump   = ELF_EXEC_PAGESIZE,
};

struct linux_binprm {
  char buf[BINPRM_BUF_SIZE];
#ifdef CONFIG_MMU
  struct vm_area_struct *vma;
  unsigned long vma_pages;
#else
# define MAX_ARG_PAGES  32
  struct page *page[MAX_ARG_PAGES];
#endif
  struct mm_struct *mm;
  unsigned long p; /* current top of mem */
  unsigned int
    called_set_creds:1,
    cap_elevated:1,
    secureexec:1;

  unsigned int recursion_depth; /* only for search_binary_handler() */
  struct file * file;
  struct cred *cred;  /* new credentials */
  int unsafe;    /* how unsafe this exec is (mask of LSM_UNSAFE_*) */
  unsigned int per_clear;  /* bits to clear in current->personality */
  int argc, envc;
  const char * filename;  /* Name of binary as seen by procps */
  const char * interp;  /* Name of the binary really executed. Most
           of the time same as filename, but could be
           different for binfmt_{misc,script} */
  unsigned interp_flags;
  unsigned interp_data;
  unsigned long loader, exec;

  struct rlimit rlim_stack; /* Saved RLIMIT_STACK used during exec. */
};

/* do_execve -> do_execveat_common -> exec_binprm -> search_binary_handler */
SYSCALL_DEFINE3(execve,
  const char __user *, filename,
  const char __user *const __user *, argv,
  const char __user *const __user *, envp)
{
  return do_execve(getname(filename), argv, envp);
}
```
![linux-proc-tree.png](../Images/Kernel/proc-tree.png)

![linux-proc-elf-compile-exec.png](../Images/Kernel/proc-elf-compile-exec.png)

### thread
![linux-proc-thread.png](../Images/Kernel/proc-thread.png)

### task_struct
![linux-proc-task-1.png](../Images/Kernel/proc-task-1.png)

![linux-proc-task-2.png](../Images/Kernel/proc-task-2.png)

### schedule
```C++
/* Real time schedule: SCHED_FIFO, SCHED_RR, SCHED_DEADLINE
 * Normal schedule: SCHED_NORMAL, SCHED_BATCH, SCHED_IDLE */
#define SCHED_NORMAL    0
#define SCHED_FIFO      1
#define SCHED_RR        2
#define SCHED_BATCH     3
#define SCHED_IDLE      5
#define SCHED_DEADLINE  6

#define MAX_NICE  19
#define MIN_NICE  -20
#define NICE_WIDTH        (MAX_NICE - MIN_NICE + 1)
#define MAX_USER_RT_PRIO  100
#define MAX_RT_PRIO        MAX_USER_RT_PRIO
#define MAX_PRIO          (MAX_RT_PRIO + NICE_WIDTH)
#define DEFAULT_PRIO      (MAX_RT_PRIO + NICE_WIDTH / 2)

struct task_struct {
  struct thread_info thread_info;

  int           on_rq; /* TASK_ON_RQ_{QUEUED, MIGRATING} */

  int           prio;
  int           static_prio;
  int           normal_prio;
  unsigned int  rt_priority;

  const struct sched_class  *sched_class;
  struct sched_entity       se;
  struct sched_rt_entity    rt;
  struct sched_dl_entity    dl;
  struct task_group         *sched_task_group;
  unsigned int              policy;

  /* CPU-specific state of this task: */
  struct thread_struct      thread;
};

struct thread_struct {
  /* Cached TLS descriptors: */
  struct desc_struct  tls_array[GDT_ENTRY_TLS_ENTRIES];
#ifdef CONFIG_X86_32
  unsigned long    sp0;
#endif
  unsigned long    sp;
#ifdef CONFIG_X86_32
  unsigned long    sysenter_cs;
#else
  unsigned short    es;
  unsigned short    ds;
  unsigned short    fsindex;
  unsigned short    gsindex;
#endif

  /* Floating point and extended processor state */
  struct fpu    fpu;
};

struct sched_entity {
  struct load_weight  load;
  struct rb_node      run_node; /* in {cfs, rt, dl}_rq */
  struct list_head    group_node;
  unsigned int        on_rq;
  u64                 exec_start;
  u64                 sum_exec_runtime;
  u64                 vruntime;
  u64                 prev_sum_exec_runtime;
  u64                 nr_migrations;
  struct sched_statistics    statistics;
};

struct rq {
  raw_spinlock_t  lock;
  unsigned int    nr_running;
  unsigned long   cpu_load[CPU_LOAD_IDX_MAX];

  struct load_weight  load;
  unsigned long       nr_load_updates;
  u64                 nr_switches;

  struct cfs_rq cfs;
  struct rt_rq  rt;
  struct dl_rq  dl;
  struct task_struct *curr, *idle, *stop;
};

struct cfs_rq {
  struct load_weight load;
  unsigned int nr_running, h_nr_running;

  u64 exec_clock;
  u64 min_vruntime;
#ifndef CONFIG_64BIT
  u64 min_vruntime_copy;
#endif
  struct rb_root tasks_timeline;
  struct rb_node *rb_leftmost;

  struct sched_entity *curr, *next, *last, *skip;
};
```
![](../Images/Kernel/proc-sched-entity-rq.png)
![](../Images/Kernel/proc-runqueue.png)

```C++
struct sched_class {
  const struct sched_class *next;

  void (*enqueue_task) (struct rq *rq, struct task_struct *p, int flags);
  void (*dequeue_task) (struct rq *rq, struct task_struct *p, int flags);
  void (*yield_task) (struct rq *rq);
  bool (*yield_to_task) (struct rq *rq, struct task_struct *p, bool preempt);

  void (*check_preempt_curr) (struct rq *rq, struct task_struct *p, int flags);

  struct task_struct * (*pick_next_task) (struct rq *rq,
            struct task_struct *prev,
            struct rq_flags *rf);
  void (*put_prev_task) (struct rq *rq, struct task_struct *p);

  void (*set_curr_task) (struct rq *rq);
  void (*task_tick) (struct rq *rq, struct task_struct *p, int queued);
  void (*task_fork) (struct task_struct *p);
  void (*task_dead) (struct task_struct *p);

  void (*switched_from) (struct rq *this_rq, struct task_struct *task);
  void (*switched_to) (struct rq *this_rq, struct task_struct *task);
  void (*prio_changed) (struct rq *this_rq, struct task_struct *task, int oldprio);
  unsigned int (*get_rr_interval) (struct rq *rq,
           struct task_struct *task);
  void (*update_curr) (struct rq *rq);
};

extern const struct sched_class stop_sched_class;
extern const struct sched_class dl_sched_class;
extern const struct sched_class rt_sched_class;
extern const struct sched_class fair_sched_class;
extern const struct sched_class idle_sched_class;
/* stop_sched_class: highest priority process, will interrupt others
 * dl_sched_class: for deadline
 * rt_sched_class: for RR or FIFO, depend on task_struct->policy
 * fair_sched_class: for normal processes
 * idle_sched_class: idle */

const struct sched_class fair_sched_class = {
  .next               = &idle_sched_class,
  .enqueue_task       = enqueue_task_fair,
  .dequeue_task       = dequeue_task_fair,
  .yield_task         = yield_task_fair,
  .yield_to_task      = yield_to_task_fair,
  .check_preempt_curr = check_preempt_wakeup,
  .pick_next_task     = pick_next_task_fair
};
```
![linux-proc-shced-cpu-rq-class-entity-task.png](../Images/Kernel/proc-shced-cpu-rq-class-entity-task.png)

#### voluntary schedule
```c++
void schedule(void)
{
  struct task_struct *tsk = current;

  sched_submit_work(tsk);
  do {
    preempt_disable();
    __schedule(false);
    sched_preempt_enable_no_resched();
  } while (need_resched());
}

/* __schedule() is the main scheduler function.
 *
 * The main means of driving the scheduler and thus entering this function are:
 *   1. Explicit blocking: mutex, semaphore, waitqueue, sleep(sk_wait_data), etc.
 *   2. TIF_NEED_RESCHED flag is checked on interrupt and userspace return paths.
 *   3. Wakeups don't really cause entry into schedule(). They add a
 *      task to the run-queue and that's it.
 * WARNING: must be called with preemption disabled! */
static void __sched notrace __schedule(bool preempt)
{
  struct task_struct *prev, *next;
  unsigned long *switch_count;
  struct rq_flags rf;
  struct rq *rq;
  int cpu;

  cpu = smp_processor_id();
  rq = cpu_rq(cpu);
  prev = rq->curr;

  next = pick_next_task(rq, prev, &rf);
  clear_tsk_need_resched(prev);
  clear_preempt_need_resched();

  if (likely(prev != next)) {
    rq->nr_switches++;
    rq->curr = next;
    ++*switch_count;

    rq = context_switch(rq, prev, next, &rf);
  }
}

static inline struct task_struct* pick_next_task(
  struct rq *rq, struct task_struct *prev, struct rq_flags *rf)
{
  const struct sched_class *class;
  struct task_struct *p;

  if (likely((prev->sched_class == &idle_sched_class ||
        prev->sched_class == &fair_sched_class) &&
       rq->nr_running == rq->cfs.h_nr_running)) {

    p = fair_sched_class.pick_next_task(rq, prev, rf);
    if (unlikely(p == RETRY_TASK))
      goto again;
    /* Assumes fair_sched_class->next == idle_sched_class */
    if (unlikely(!p))
      p = idle_sched_class.pick_next_task(rq, prev, rf);
    return p;
  }

again:
  for_each_class(class) {
    p = class->pick_next_task(rq, prev, rf);
    if (p) {
      if (unlikely(p == RETRY_TASK))
        goto again;
      return p;
    }
  }
}

/* fair_sched_class */
static struct task_struct* pick_next_task_fair(
  struct rq *rq, struct task_struct *prev, struct rq_flags *rf)
{
  struct cfs_rq *cfs_rq = &rq->cfs;
  struct sched_entity *se;
  struct task_struct *p;
  int new_tasks;

  struct sched_entity *curr = cfs_rq->curr;
  if (curr) {
    if (curr->on_rq)
      update_curr(cfs_rq);
    else
      curr = NULL;
  }

  se = pick_next_entity(cfs_rq, curr);
  p = task_of(se);
  if (prev != p) {
    struct sched_entity *pse = &prev->se;
    put_prev_entity(cfs_rq, pse);
    set_next_entity(cfs_rq, se);
  }

  return p;
}

static struct rq* context_switch(
  struct rq *rq, struct task_struct *prev,
  struct task_struct *next, struct rq_flags *rf)
{
  struct mm_struct *mm, *oldmm;
  mm = next->mm;
  oldmm = prev->active_mm;
  /* 1. swtich user stack
   * user esp, eip switched when returning to user space */
  switch_mm_irqs_off(oldmm, mm, next);

  /* Here we just switch the register state and the stack. */
  switch_to(prev, next, prev);
  barrier();
  return finish_task_switch(prev);
}

void switch_mm_irqs_off(struct mm_struct *prev, struct mm_struct *next,
      struct task_struct *tsk)
{
  struct mm_struct *real_prev = this_cpu_read(cpu_tlbstate.loaded_mm);
  u16 prev_asid = this_cpu_read(cpu_tlbstate.loaded_mm_asid);
  unsigned cpu = smp_processor_id();
  u64 next_tlb_gen;

  if (real_prev == next) {
    return;
  } else {
    u16 new_asid;
    bool need_flush;
    choose_new_asid(next, next_tlb_gen, &new_asid, &need_flush);

    /* Let nmi_uaccess_okay() know that we're changing CR3. */
    this_cpu_write(cpu_tlbstate.loaded_mm, LOADED_MM_SWITCHING);
    barrier();

    if (need_flush) {
      this_cpu_write(cpu_tlbstate.ctxs[new_asid].ctx_id, next->context.ctx_id);
      this_cpu_write(cpu_tlbstate.ctxs[new_asid].tlb_gen, next_tlb_gen);
      load_new_mm_cr3(next->pgd, new_asid, true);
    } else {
      load_new_mm_cr3(next->pgd, new_asid, false);
    }

    /* Make sure we write CR3 before loaded_mm. */
    barrier();

    this_cpu_write(cpu_tlbstate.loaded_mm, next);
    this_cpu_write(cpu_tlbstate.loaded_mm_asid, new_asid);
  }

  load_mm_cr4(next);
  switch_ldt(real_prev, next);
}

#define switch_to(prev, next, last) \
do {                  \
  prepare_switch_to(prev, next);  \
  /* 2. switch kernel esp, stack */
  ((last) = __switch_to_asm((prev), (next))); \
} while (0)

/* %eax: prev task
 * %edx: next task */
ENTRY(__switch_to_asm)
  /* Save callee-saved registers
   * This must match the order in struct inactive_task_frame */
  pushl  %ebp
  pushl  %ebx
  pushl  %edi
  pushl  %esi
  pushfl

  /* 2.1 switch kernel sp
   * save old value from esp to prev task
   * load new value from thread_struct of next task to esp */
  movl  %esp, TASK_threadsp(%eax)
  movl  TASK_threadsp(%edx), %esp

  /* restore callee-saved registers */
  popfl
  popl  %esi
  popl  %edi
  popl  %ebx
  popl  %ebp

  /* 2.2. switch kernel stack */
  jmp  __switch_to

END(__switch_to_asm)

struct task_struct * __switch_to(
  struct task_struct *prev_p, struct task_struct *next_p)
{
  struct thread_struct *prev = &prev_p->thread;
  struct thread_struct *next = &next_p->thread;
  int cpu = smp_processor_id();

  load_TLS(next, cpu);

  /* 3. swtich kernel stack */
  this_cpu_write(current_task, next_p);

  /* 4. load new thread_stuct from next task
   * TSS(Task State Segment) TR(Task Register) */
  struct tss_struct *tss = &per_cpu(cpu_tss, cpu);
  /* Reload esp0 and ss1.  This changes current_thread_info(). */
  load_sp0(tss, next);

  return prev_p;
}

/* switch tasks from x to y */
struct task_struct * __switch_to(
  struct task_struct *prev_p, struct task_struct *next_p)
{
  struct thread_struct *prev = &prev_p->thread;
  struct thread_struct *next = &next_p->thread;
  struct fpu *prev_fpu = &prev->fpu;
  struct fpu *next_fpu = &next->fpu;
  int cpu = smp_processor_id();

  switch_fpu_prepare(prev_fpu, cpu);

  /* We must save %fs and %gs before load_TLS() because
   * %fs and %gs may be cleared by load_TLS().
   *
   * (e.g. xen_load_tls()) */
  save_fsgs(prev_p);

  /* Load TLS before restoring any segments so that segment loads
   * reference the correct GDT entries. */
  load_TLS(next, cpu);

  /* Leave lazy mode, flushing any hypercalls made here.  This
   * must be done after loading TLS entries in the GDT but before
   * loading segments that might reference them, and and it must
   * be done before fpu__restore(), so the TS bit is up to
   * date. */
  arch_end_context_switch(next_p);

  /* Switch DS and ES.
   *
   * Reading them only returns the selectors, but writing them (if
   * nonzero) loads the full descriptor from the GDT or LDT.  The
   * LDT for next is loaded in switch_mm, and the GDT is loaded
   * above.
   *
   * We therefore need to write new values to the segment
   * registers on every context switch unless both the new and old
   * values are zero.
   *
   * Note that we don't need to do anything for CS and SS, as
   * those are saved and restored as part of pt_regs. */
  savesegment(es, prev->es);
  if (unlikely(next->es | prev->es))
    loadsegment(es, next->es);

  savesegment(ds, prev->ds);
  if (unlikely(next->ds | prev->ds))
    loadsegment(ds, next->ds);

  load_seg_legacy(prev->fsindex, prev->fsbase,
      next->fsindex, next->fsbase, FS);
  load_seg_legacy(prev->gsindex, prev->gsbase,
      next->gsindex, next->gsbase, GS);

  switch_fpu_finish(next_fpu, cpu);

  /* Switch the PDA and FPU contexts. */
  this_cpu_write(current_task, next_p);
  this_cpu_write(cpu_current_top_of_stack, task_top_of_stack(next_p));

  /* Reload sp0. */
  update_task_stack(next_p);

  switch_to_extra(prev_p, next_p);

  /* Load the Intel cache allocation PQR MSR. */
  intel_rdt_sched_in();

  return prev_p;
}

/* This is used when switching tasks or entering/exiting vm86 mode. */
static inline void update_task_stack(struct task_struct *task)
{
  /* sp0 always points to the entry trampoline stack, which is constant: */
#ifdef CONFIG_X86_32
  if (static_cpu_has(X86_FEATURE_XENPV))
    load_sp0(task->thread.sp0);
  else
    this_cpu_write(cpu_tss_rw.x86_tss.sp1, task->thread.sp0);
#else
  if (static_cpu_has(X86_FEATURE_XENPV))
    load_sp0(task_top_of_stack(task));
#endif
}

void cpu_init(void)
{
  int cpu = smp_processor_id();
  struct task_struct *curr = current;
  struct tss_struct *tss = &per_cpu(cpu_tss, cpu);
  load_sp0(tss, thread);
  set_tss_desc(cpu, tss);
  load_TR_desc();
}

struct tss_struct {
  struct x86_hw_tss  x86_tss;
  unsigned long    io_bitmap[IO_BITMAP_LONGS + 1];
}
```
![linux-proc-tss.png](../Images/Kernel/proc-tss.png)

```C++
schedule(void)
    __schedule(false), // kernel/sched/core.c
        pick_next_task(rq, prev, &rf);
        context_switch(rq, prev, next, &rf);
            switch_mm_irqs_off(prev->active_mm, next->mm, next) {
              load_new_mm_cr3()
            }
            switch_to(prev, next, prev);
                __switch_to_asm(); // switch registers, but not EIP [arch/x86/entry/entry_64.S]
                    __switch_to(); // switch stack [arch/x86/kernel/process_32.c]
                        this_cpu_write(current_task, next_p); //
            barrier();
            return finish_task_switch(prev);
```
![linux-proc-sched-voluntary.png](../Images/Kernel/proc-sched-voluntary.png)

#### preempt schedule
##### preempt time
###### Clock interrupt
```C++
void scheduler_tick(void)
{
  int cpu = smp_processor_id();
  struct rq *rq = cpu_rq(cpu);
  struct task_struct *curr = rq->curr;

  curr->sched_class->task_tick(rq, curr, 0);
  cpu_load_update_active(rq);
  calc_global_load_tick(rq);
}

static void task_tick_fair(struct rq *rq, struct task_struct *curr, int queued)
{
  struct cfs_rq *cfs_rq;
  struct sched_entity *se = &curr->se;

  for_each_sched_entity(se) {
    cfs_rq = cfs_rq_of(se);
    entity_tick(cfs_rq, se, queued);
  }
}

static void entity_tick(struct cfs_rq *cfs_rq, struct sched_entity *curr, int queued)
{
  update_curr(cfs_rq);
  update_load_avg(curr, UPDATE_TG);
  update_cfs_shares(curr);

  if (cfs_rq->nr_running > 1)
    check_preempt_tick(cfs_rq, curr);
}

static void check_preempt_tick(struct cfs_rq *cfs_rq, struct sched_entity *curr)
{
  unsigned long ideal_runtime, delta_exec;
  struct sched_entity *se;
  s64 delta;

  ideal_runtime = sched_slice(cfs_rq, curr);
  delta_exec = curr->sum_exec_runtime - curr->prev_sum_exec_runtime;
  if (delta_exec > ideal_runtime) {
    resched_curr(rq_of(cfs_rq));
    return;
  }

  se = __pick_first_entity(cfs_rq);
  delta = curr->vruntime - se->vruntime;
  if (delta < 0)
    return;
  if (delta > ideal_runtime)
    resched_curr(rq_of(cfs_rq));
}

/* resched_curr -> */
static inline void set_tsk_need_resched(struct task_struct *tsk)
{
  /* just mark thread with TIF_NEED_RESCHED */
  set_tsk_thread_flag(tsk,TIF_NEED_RESCHED);
}
```

###### ttwu
```C++
/* try_to_wake_up -> ttwu_queue -> ttwu_do_activate -> ttwu_do_wakeup
* -> check_preempt_curr -> resched_curr */
static int try_to_wake_up(
  struct task_struct *p, unsigned int state, int wake_flags)
{
  unsigned long flags;
  int cpu, success = 0;

  success = 1;
  cpu = task_cpu(p);

  p->sched_contributes_to_load = !!task_contributes_to_load(p);
  p->state = TASK_WAKING;

  if (p->in_iowait) {
    delayacct_blkio_end(p);
    atomic_dec(&task_rq(p)->nr_iowait);
  }

  cpu = select_task_rq(p, p->wake_cpu, SD_BALANCE_WAKE, wake_flags);
  if (task_cpu(p) != cpu) {
    wake_flags |= WF_MIGRATED;
    set_task_cpu(p, cpu);
  }

  ttwu_queue(p, cpu, wake_flags);
stat:
  ttwu_stat(p, cpu, wake_flags);
out:
  raw_spin_unlock_irqrestore(&p->pi_lock, flags);

  return success;
}

static void ttwu_queue(struct task_struct *p, int cpu, int wake_flags)
{
  struct rq *rq = cpu_rq(cpu);
  struct rq_flags rf;

  rq_lock(rq, &rf);
  update_rq_clock(rq);
  ttwu_do_activate(rq, p, wake_flags, &rf);
  rq_unlock(rq, &rf);
}

static void ttwu_do_activate(
  struct rq *rq, struct task_struct *p,
  int wake_flags, struct rq_flags *rf)
{
  int en_flags = ENQUEUE_WAKEUP | ENQUEUE_NOCLOCK;

  lockdep_assert_held(&rq->lock);

  /* 1. insert p into rq */
  ttwu_activate(rq, p, en_flags);
  /* 2. check schedule curr */
  ttwu_do_wakeup(rq, p, wake_flags, rf);
}

static inline void ttwu_activate(
  struct rq *rq, struct task_struct *p, int en_flags)
{
  activate_task(rq, p, en_flags);
  p->on_rq = TASK_ON_RQ_QUEUED;

  /* If a worker is waking up, notify the workqueue: */
  if (p->flags & PF_WQ_WORKER)
    wq_worker_waking_up(p, cpu_of(rq));
}

void activate_task(struct rq *rq, struct task_struct *p, int flags)
{
  if (task_contributes_to_load(p))
    rq->nr_uninterruptible--;

  enqueue_task(rq, p, flags);
}

static inline void enqueue_task(
  struct rq *rq, struct task_struct *p, int flags)
{
  if (!(flags & ENQUEUE_NOCLOCK))
    update_rq_clock(rq);

  if (!(flags & ENQUEUE_RESTORE))
    sched_info_queued(rq, p);

  p->sched_class->enqueue_task(rq, p, flags);
}

static void ttwu_do_wakeup(
  struct rq *rq, struct task_struct *p,
  int wake_flags, struct rq_flags *rf)
{
  check_preempt_curr(rq, p, wake_flags);
  p->state = TASK_RUNNING;
  trace_sched_wakeup(p);
}

void check_preempt_curr(struct rq *rq, struct task_struct *p, int flags)
{
  const struct sched_class *class;

  if (p->sched_class == rq->curr->sched_class) {
    rq->curr->sched_class->check_preempt_curr(rq, p, flags);
  } else {
    for_each_class(class) {
      if (class == rq->curr->sched_class)
        break;
      if (class == p->sched_class) {
        resched_curr(rq);
        break;
      }
    }
  }

  if (task_on_rq_queued(rq->curr) && test_tsk_need_resched(rq->curr))
    rq_clock_skip_update(rq);
}

void resched_curr(struct rq *rq)
{
  struct task_struct *curr = rq->curr;
  int cpu;

  if (test_tsk_need_resched(curr))
    return;

  cpu = cpu_of(rq);

  if (cpu == smp_processor_id()) {
    set_tsk_need_resched(curr);
    set_preempt_need_resched();
    return;
  }

  if (set_nr_and_not_polling(curr))
    smp_send_reschedule(cpu);
  else
    trace_sched_wake_idle_without_ipi(cpu);
}
```

##### real user preempt time
###### return from system call
```C++
/* do_syscall_64 -> syscall_return_slowpath
 * -> prepare_exit_to_usermode -> exit_to_usermode_loop */
static void exit_to_usermode_loop(struct pt_regs *regs, u32 cached_flags)
{
  while (true) {
    local_irq_enable();

    if (cached_flags & _TIF_NEED_RESCHED)
      schedule();

    if (cached_flags & _TIF_SIGPENDING)
      do_signal(regs);
  }
}
```

###### return from interrupt
```C++
/* do_IRQ -> retint_user -> prepare_exit_to_usermode -> exit_to_usermode_loop */
common_interrupt:
        ASM_CLAC
        addq    $-0x80, (%rsp)
        interrupt do_IRQ
ret_from_intr:
        popq    %rsp
        testb   $3, CS(%rsp)
        jz      retint_kernel

/* Interrupt came from user space */
GLOBAL(retint_user)
        mov     %rsp,%rdi
        call    prepare_exit_to_usermode
        TRACE_IRQS_IRETQ
        SWAPGS
        jmp     restore_regs_and_iret

/* Returning to kernel space */
retint_kernel:
#ifdef CONFIG_PREEMPT
        bt      $9, EFLAGS(%rsp)
        jnc     1f
0:      cmpl    $0, PER_CPU_VAR(__preempt_count)
        jnz     1f
        call    preempt_schedule_irq
        jmp     0b
```

##### real kernel preempt time
###### preempt_enble
```C++
#define preempt_enable() \
do { \
  if (unlikely(preempt_count_dec_and_test())) \
    __preempt_schedule(); \
} while (0)

#define preempt_count_dec_and_test() \
  ({ preempt_count_sub(1); should_resched(0); })

static  bool should_resched(int preempt_offset)
{
  return unlikely(preempt_count() == preempt_offset &&
      tif_need_resched());
}

#define tif_need_resched() test_thread_flag(TIF_NEED_RESCHED)

/* __preempt_schedule -> */
static void __sched notrace preempt_schedule_common(void)
{
  do {
    __schedule(true);
  } while (need_resched());
}
```

###### return from interrupt
```C++
/* do_IRQ -> retint_kernel */
asmlinkage __visible void __sched preempt_schedule_irq(void)
{
  do {
    preempt_disable();
    local_irq_enable();
    __schedule(true);
    local_irq_disable();
    sched_preempt_enable_no_resched();
  } while (need_resched());
}
```
![linux-proc-sched.png](../Images/Kernel/proc-sched.png)

#### Q
1. A process waits on a block operation (mutex, semphore, waitqueue), it calls schedule(). Will it be removed from rq, and add to rq when block operation wakeups?
2. What's difference between contex_switch and sleep_wakeup?

### wake_up
```c++
#define wake_up(x)                        __wake_up(x, TASK_NORMAL, 1, NULL)
#define wake_up_nr(x, nr)                 __wake_up(x, TASK_NORMAL, nr, NULL)
#define wake_up_all(x)                    __wake_up(x, TASK_NORMAL, 0, NULL)
#define wake_up_locked(x)                 __wake_up_locked((x), TASK_NORMAL, 1)
#define wake_up_all_locked(x)             __wake_up_locked((x), TASK_NORMAL, 0)

#define wake_up_interruptible(x)          __wake_up(x, TASK_INTERRUPTIBLE, 1, NULL)
#define wake_up_interruptible_nr(x, nr)   __wake_up(x, TASK_INTERRUPTIBLE, nr, NULL)
#define wake_up_interruptible_all(x)      __wake_up(x, TASK_INTERRUPTIBLE, 0, NULL)
#define wake_up_interruptible_sync(x)     __wake_up_sync((x), TASK_INTERRUPTIBLE, 1)

void __wake_up(
  struct wait_queue_head *wq_head, unsigned int mode,
  int nr_exclusive, void *key)
{
  __wake_up_common_lock(wq_head, mode, nr_exclusive, 0, key);
}

static void __wake_up_common_lock(struct wait_queue_head *wq_head, unsigned int mode,
      int nr_exclusive, int wake_flags, void *key)
{
  unsigned long flags;
  wait_queue_entry_t bookmark;

  bookmark.flags = 0;
  bookmark.private = NULL;
  bookmark.func = NULL;
  INIT_LIST_HEAD(&bookmark.entry);

  spin_lock_irqsave(&wq_head->lock, flags);
  nr_exclusive = __wake_up_common(wq_head, mode, nr_exclusive, wake_flags, key, &bookmark);
  spin_unlock_irqrestore(&wq_head->lock, flags);

  while (bookmark.flags & WQ_FLAG_BOOKMARK) {
    spin_lock_irqsave(&wq_head->lock, flags);
    nr_exclusive = __wake_up_common(wq_head, mode, nr_exclusive, wake_flags, key, &bookmark);
    spin_unlock_irqrestore(&wq_head->lock, flags);
  }
}

static int __wake_up_common(
  struct wait_queue_head *wq_head, unsigned int mode,
  int nr_exclusive, int wake_flags, void *key,
  wait_queue_entry_t *bookmark)
{
  wait_queue_entry_t *curr, *next;
  int cnt = 0;

  lockdep_assert_held(&wq_head->lock);

  if (bookmark && (bookmark->flags & WQ_FLAG_BOOKMARK)) {
    curr = list_next_entry(bookmark, entry);

    list_del(&bookmark->entry);
    bookmark->flags = 0;
  } else
    curr = list_first_entry(&wq_head->head, wait_queue_entry_t, entry);

  if (&curr->entry == &wq_head->head)
    return nr_exclusive;

  list_for_each_entry_safe_from(curr, next, &wq_head->head, entry) {
    unsigned flags = curr->flags;
    int ret;

    if (flags & WQ_FLAG_BOOKMARK)
      continue;

    /* ep_poll_callback, default_wake_func, woken_wake_function */
    ret = curr->func(curr, mode, wake_flags, key);
    if (ret < 0)
      break;
    /* WQ_FLAG_EXCLUSIVE : fix thunderbird problem */
    if (ret && (flags & WQ_FLAG_EXCLUSIVE) && !--nr_exclusive)
      break;

    if (bookmark && (++cnt > WAITQUEUE_WALK_BREAK_CNT) 
      && (&next->entry != &wq_head->head))
    {
      bookmark->flags = WQ_FLAG_BOOKMARK;
      list_add_tail(&bookmark->entry, &next->entry);
      break;
    }
  }

  return nr_exclusive;
}
```

### wait_woken
```c++
long wait_woken(struct wait_queue_entry *wq_entry, unsigned mode, long timeout)
{
  /* The below executes an smp_mb(), which matches with the full barrier
   * executed by the try_to_wake_up() in woken_wake_function() such that
   * either we see the store to wq_entry->flags in woken_wake_function()
   * or woken_wake_function() sees our store to current->state. */
  set_current_state(mode); /* A */
  if (!(wq_entry->flags & WQ_FLAG_WOKEN) && !is_kthread_should_stop())
    timeout = schedule_timeout(timeout);
  __set_current_state(TASK_RUNNING);

  /* The below executes an smp_mb(), which matches with the smp_mb() (C)
   * in woken_wake_function() such that either we see the wait condition
   * being true or the store to wq_entry->flags in woken_wake_function()
   * follows ours in the coherence order. */
  smp_store_mb(wq_entry->flags, wq_entry->flags & ~WQ_FLAG_WOKEN); /* B */

  return timeout;
}

int woken_wake_function(
  struct wait_queue_entry *wq_entry, unsigned mode, int sync, void *key)
{
  /* Pairs with the smp_store_mb() in wait_woken(). */
  smp_mb(); /* C */
  wq_entry->flags |= WQ_FLAG_WOKEN;

  return default_wake_function(wq_entry, mode, sync, key);
}

int default_wake_function(
  wait_queue_entry_t *curr, unsigned mode, int wake_flags, void *key)
{
  return try_to_wake_up(curr->private, mode, wake_flags);
}

/* wait_queue_entry::flags */
#define WQ_FLAG_EXCLUSIVE  0x01
#define WQ_FLAG_WOKEN      0x02
#define WQ_FLAG_BOOKMARK   0x04

struct wait_queue_entry {
  unsigned int      flags;
  void              *private; /* struct_task */
  wait_queue_func_t func;
  struct list_head  entry;
};
```
* [try_to_wake_up](#ttwu)

### fork
```C++
SYSCALL_DEFINE0(fork)
{
  return _do_fork(SIGCHLD, 0, 0, NULL, NULL, 0);
}

long _do_fork(
  unsigned long clone_flags,
  unsigned long stack_start,
  unsigned long stack_size,
  int __user *parent_tidptr,
  int __user *child_tidptr,
  unsigned long tls)
{
  struct task_struct *p;
  int trace = 0;
  long nr;

  p = copy_process(clone_flags, stack_start, stack_size,
    child_tidptr, NULL, trace, tls, NUMA_NO_NODE);

  if (IS_ERR(p))
    return PTR_ERR(p);

  struct pid *pid;
  pid = get_task_pid(p, PIDTYPE_PID);
  nr = pid_vnr(pid);

  if (clone_flags & CLONE_PARENT_SETTID)
    put_user(nr, parent_tidptr);

  wake_up_new_task(p);

  put_pid(pid);

  return nr;
}

/* copy_process -> sched_fork -> task_fork -> */
static void task_fork_fair(struct task_struct *p)
{
  /* default off, for bash bug and better use TLB and cache */
  if (sysctl_sched_child_runs_first && curr && entity_before(curr, se)) {
    swap(curr->vruntime, se->vruntime);
    resched_curr(rq);
  }

  se->vruntime -= cfs_rq->min_vruntime;
}

void wake_up_new_task(struct task_struct *p)
{
  struct rq_flags rf;
  struct rq *rq;

  p->state = TASK_RUNNING;

  activate_task(rq, p, ENQUEUE_NOCLOCK);
  p->on_rq = TASK_ON_RQ_QUEUED;
  trace_sched_wakeup_new(p);
  check_preempt_curr(rq, p, WF_FORK);
}

void check_preempt_curr(struct rq *rq, struct task_struct *p, int flags)
{
  const struct sched_class *class;

  if (p->sched_class == rq->curr->sched_class) {
    rq->curr->sched_class->check_preempt_curr(rq, p, flags);
  } else {
    for_each_class(class) {
      if (class == rq->curr->sched_class)
        break;
      if (class == p->sched_class) {
        resched_curr(rq);
        break;
      }
    }
  }

  if (task_on_rq_queued(rq->curr) && test_tsk_need_resched(rq->curr))
    rq_clock_skip_update(rq);
}

/* fair_sched_class.check_preempt_wakeup */
static void check_preempt_wakeup(struct rq *rq, struct task_struct *p, int wake_flags)
{
  struct task_struct *curr = rq->curr;
  struct sched_entity *se = &curr->se, *pse = &p->se;
  struct cfs_rq *cfs_rq = task_cfs_rq(curr);

  if (test_tsk_need_resched(curr))
    return;

  find_matching_se(&se, &pse);
  update_curr(cfs_rq_of(se));
  if (wakeup_preempt_entity(se, pse) == 1) {
    goto preempt;
  }
  return;
preempt:
  resched_curr(rq);
}

```
![linux-fork.png](../Images/Kernel/fork.png)

### exec
```C++
int do_execve(struct filename *filename,
  const char __user *const __user *__argv,
  const char __user *const __user *__envp)
{
  struct user_arg_ptr argv = { .ptr.native = __argv };
  struct user_arg_ptr envp = { .ptr.native = __envp };
  return do_execveat_common(AT_FDCWD, filename, argv, envp, 0);
}

static int do_execveat_common(
  int fd, struct filename *filename,
  struct user_arg_ptr argv,
  struct user_arg_ptr envp, int flags)
{
  return __do_execve_file(fd, filename, argv, envp, flags, NULL);
}

static int __do_execve_file(
  int fd, struct filename *filename,
  struct user_arg_ptr argv,
  struct user_arg_ptr envp,
  int flags, struct file *file)
{
  char *pathbuf = NULL;
  struct linux_binprm *bprm;
  struct files_struct *displaced;
  int retval;

  retval = -ENOMEM;
  bprm = kzalloc(sizeof(*bprm), GFP_KERNEL);
  retval = prepare_bprm_creds(bprm);

  if (!file)
    file = do_open_execat(fd, filename, flags);

  sched_exec();

  bprm->file = file;
  if (!filename) {
    bprm->filename = "none";
  } else if (fd == AT_FDCWD || filename->name[0] == '/') {
    bprm->filename = filename->name;
  } else {
    if (filename->name[0] == '\0')
      pathbuf = kasprintf(GFP_KERNEL, "/dev/fd/%d", fd);
    else
      pathbuf = kasprintf(GFP_KERNEL, "/dev/fd/%d/%s",
              fd, filename->name);
    if (close_on_exec(fd, rcu_dereference_raw(current->files->fdt)))
      bprm->interp_flags |= BINPRM_FLAGS_PATH_INACCESSIBLE;

    bprm->filename = pathbuf;
  }

  bprm->interp = bprm->filename;

  bprm_mm_init(bprm);

  bprm->argc = count(argv, MAX_ARG_STRINGS);
  bprm->envc = count(envp, MAX_ARG_STRINGS);

  /* Fill the binprm structure from the inode */
  prepare_binprm(bprm);

  bprm->exec = bprm->p;

  retval = exec_binprm(bprm);

  /* execve succeeded */
  current->fs->in_exec = 0;
  current->in_execve = 0;
  membarrier_execve(current);
  rseq_execve(current);
  acct_update_integrals(current);
  task_numa_free(current, false);
  free_bprm(bprm);
  kfree(pathbuf);
  if (filename)
    putname(filename);
  if (displaced)
    put_files_struct(displaced);

  return retval;
}

static int exec_binprm(struct linux_binprm *bprm)
{
  pid_t old_pid, old_vpid;
  int ret;

  /* Need to fetch pid before load_binary changes it */
  old_pid = current->pid;
  rcu_read_lock();
  old_vpid = task_pid_nr_ns(current, task_active_pid_ns(current->parent));
  rcu_read_unlock();

  ret = search_binary_handler(bprm);
  if (ret >= 0) {
    audit_bprm(bprm);
    trace_sched_process_exec(current, old_pid, bprm);
    ptrace_event(PTRACE_EVENT_EXEC, old_vpid);
    proc_exec_connector(current);
  }

  return ret;
}

int search_binary_handler(struct linux_binprm *bprm)
{
  bool need_retry = IS_ENABLED(CONFIG_MODULES);
  struct linux_binfmt *fmt;
  int retval;
  retval = -ENOENT;

 retry:
  read_lock(&binfmt_lock);
  list_for_each_entry(fmt, &formats, lh) {
    if (!try_module_get(fmt->module))
      continue;
    read_unlock(&binfmt_lock);
    retval = fmt->load_binary(bprm);
    if (retval < 0 && !bprm->mm)
      return retval;
  }
  read_unlock(&binfmt_lock);

  return retval;
}

static struct linux_binfmt elf_format = {
  .module       = THIS_MODULE,
  .load_binary  = load_elf_binary,
  .load_shlib   = load_elf_library,
  .core_dump    = elf_core_dump,
  .min_coredump = ELF_EXEC_PAGESIZE,
};

static int load_elf_binary(struct linux_binprm *bprm)
{
  struct {
    struct elfhdr elf_ex;
    struct elfhdr interp_elf_ex;
  } *loc;

  struct pt_regs *regs = current_pt_regs();
  loc = kmalloc(sizeof(*loc), GFP_KERNEL);
  loc->elf_ex = *((struct elfhdr *)bprm->buf);

  if (memcmp(loc->elf_ex.e_ident, ELFMAG, SELFMAG) != 0)
    goto out;
  if (loc->elf_ex.e_type != ET_EXEC && loc->elf_ex.e_type != ET_DYN)
    goto out;

  elf_phdata = load_elf_phdrs(&loc->elf_ex, bprm->file);

  /* set mmap_base */
  setup_new_exec(bprm);

  /*set stack vm_area_struct */
  retval = setup_arg_pages(bprm, randomize_stack_top(STACK_TOP),
         executable_stack);

  /* mmap part code into memory */
  error = elf_map(bprm->file, load_bias + vaddr, elf_ppnt,
        elf_prot, elf_flags, total_size);

  /* set heap vm_area_struct */
  retval = set_brk(elf_bss, elf_brk, bss_prot);

  /* load shared objects into memory */
  elf_entry = load_elf_interp(&loc->interp_elf_ex,
              interpreter,
              &interp_map_addr,
              load_bias, interp_elf_phdata);

  current->mm->end_code = end_code;
  current->mm->start_code = start_code;
  current->mm->start_data = start_data;
  current->mm->end_data = end_data;
  current->mm->start_stack = bprm->p;

  start_thread(regs, elf_entry, bprm->p);
}

void start_thread(
  struct pt_regs *regs,
  unsigned long new_ip, unsigned long new_sp)
{
  set_user_gs(regs, 0);
  regs->fs  = 0;
  regs->ds  = __USER_DS;
  regs->es  = __USER_DS;
  regs->ss  = __USER_DS;
  regs->cs  = __USER_CS;
  regs->ip  = new_ip;
  regs->sp  = new_sp;
  regs->flags  = X86_EFLAGS_IF;
  force_iret(); /* restore the saved registers */
}
```
![linux-proc-fork-pthread-create.png](../Images/Kernel/proc-fork-pthread-create.png)

Reference:
* [A complete guide to Linux process scheduling.pdf](https://trepo.tuni.fi/bitstream/handle/10024/96864/GRADU-1428493916.pdf)

# File Management
![linux-file-vfs-system.png](../Images/Kernel/file-vfs-system.png)

### inode
```C++
struct inode {
  const struct inode_operations   *i_op;
  struct super_block              *i_sb;
  struct address_space            *i_mapping;

  union {
    const struct file_operations  *i_fop;  /* former ->i_op->default_file_ops */
    void (*free_inode)(struct inode *);
  };

  dev_t             i_rdev;
  struct list_head  i_devices;
  union {
    struct pipe_inode_info  *i_pipe;
    struct block_device     *i_bdev;
    struct cdev             *i_cdev;
    char                    *i_link;
    unsigned                i_dir_seq;
  };

  umode_t           i_mode;
  unsigned short    i_opflags;
  kuid_t            i_uid;
  kgid_t            i_gid;
  unsigned int      i_flags;

  unsigned long     dirtied_when;  /* jiffies of first dirtying */
  unsigned long     dirtied_time_when;

  struct hlist_node     i_hash;
  struct list_head      i_io_list; /* backing dev IO list */
  struct bdi_writeback  *i_wb;     /* the associated cgroup wb */

  struct file_lock_context  *i_flctx;
  struct address_space     i_data;
};

#define  EXT4_NDIR_BLOCKS    12
#define  EXT4_IND_BLOCK      EXT4_NDIR_BLOCKS
#define  EXT4_DIND_BLOCK     (EXT4_IND_BLOCK + 1)
#define  EXT4_TIND_BLOCK     (EXT4_DIND_BLOCK + 1)
#define  EXT4_N_BLOCKS       (EXT4_TIND_BLOCK + 1)

struct ext4_inode {
  __le16  i_mode;         /* File mode */
  __le16  i_uid;          /* Low 16 bits of Owner Uid */
  __le32  i_size_lo;      /* Size in bytes */
  __le32  i_atime;        /* Access time */
  __le32  i_ctime;        /* Inode Change time */
  __le32  i_mtime;        /* Modification time */
  __le32  i_dtime;        /* Deletion Time */
  __le16  i_gid;          /* Low 16 bits of Group Id */
  __le16  i_links_count;  /* Links count */
  __le32  i_blocks_lo;    /* Blocks count */
  __le32  i_flags;         /* File flags */

  __le32  i_block[EXT4_N_BLOCKS]  /* Pointers to blocks */
  __le32  i_generation;           /* File version (for NFS) */
  __le32  i_file_acl_lo;           /* File ACL */
  __le32  i_size_high;
};
```
![](../Images/Kernel/file-inode-blocks.png)

### extent
```c++
// Each block (leaves and indexes), even inode-stored has header.
struct ext4_extent_header {
  __le16  eh_magic;  /* probably will support different formats */
  __le16  eh_entries;  /* number of valid entries */
  __le16  eh_max;    /* capacity of store in entries */
  __le16  eh_depth;  /* has tree real underlying blocks? */
  __le32  eh_generation;  /* generation of the tree */
};

struct ext4_extent_idx {
  __le32  ei_block;  /* index covers logical blocks from 'block' */
  __le32  ei_leaf_lo;  /* pointer to the physical block of the next *
         * level. leaf or next index could be there */
  __le16  ei_leaf_hi;  /* high 16 bits of physical block */
  __u16  ei_unused;
};

struct ext4_extent {
  __le32  ee_block;  /* first logical block extent covers */
  /* the most significant bit used as a flag to identify whether
   * this entent is initialized, 15 bits can present max 128MB data */
  __le16  ee_len;    /* number of blocks covered by extent */
  __le16  ee_start_hi;  /* high 16 bits of physical block */
  __le32  ee_start_lo;  /* low 32 bits of physical block */
};
```
* the `ee_len` is 16 bits, one bit indicates validation, so one ext4_extent can represent 2^15 blocks data, each block is 4k, the total data is 2^15 * 2^12 = 2^27 bits = 128MB.

![linux-mem-extents.png](../Images/Kernel/mem-extents.png)

![linux-ext4-extents.png](../Images/Kernel/ext4-extents.png)

```C++
const struct inode_operations ext4_dir_inode_operations = {
  .create     = ext4_create,
  .lookup     = ext4_lookup,
  .link       = ext4_link,
  .unlink     = ext4_unlink,
  .symlink    = ext4_symlink,
  .mkdir      = ext4_mkdir,
  .rmdir      = ext4_rmdir,
  .mknod      = ext4_mknod,
  .tmpfile     = ext4_tmpfile,
  .rename     = ext4_rename2,
  .setattr    = ext4_setattr,
  .getattr    = ext4_getattr,
  .listxattr  = ext4_listxattr,
  .get_acl    = ext4_get_acl,
  .set_acl    = ext4_set_acl,
  .fiemap      = ext4_fiemap,
};

/* open -> do_sys_open -> do_filp_open -> path_openat -> do_last -> lookup_open
 * ext4_create -> ext4_new_inode_start_handle -> __ext4_new_inode */
struct inode *__ext4_new_inode(
  handle_t *handle, struct inode *dir,
  umode_t mode, const struct qstr *qstr,
  __u32 goal, uid_t *owner, __u32 i_flags,
  int handle_type, unsigned int line_no,
  int nblocks)
{
inode_bitmap_bh = ext4_read_inode_bitmap(sb, group);
ino = ext4_find_next_zero_bit((unsigned long *)
                inode_bitmap_bh->b_data,
                EXT4_INODES_PER_GROUP(sb), ino);
}
```

### Block Group
* One block group contains:
  * one block representing block bit info + several block representing data blocks
  * one block representing inode bit info + several block representing inode blocks

```C++
struct ext4_group_desc
{
  __le32  bg_block_bitmap_lo;  /* Blocks bitmap block */
  __le32  bg_inode_bitmap_lo;  /* Inodes bitmap block */
  __le32  bg_inode_table_lo;  /* Inodes table block */
};
```
![linux-block-group.png](../Images/Kernel/block-group.png)

```C++
struct ext4_super_block {
  __le32  s_blocks_count_lo;  /* Blocks count */
  __le32  s_r_blocks_count_lo;  /* Reserved blocks count */
  __le32  s_free_blocks_count_lo;  /* Free blocks count */

  __le32  s_blocks_count_hi;  /* Blocks count */
  __le32  s_r_blocks_count_hi;  /* Reserved blocks count */
  __le32  s_free_blocks_count_hi;  /* Free blocks count */
}

struct super_block {
  struct list_head               s_list; /* Keep first */
  struct dentry                  *s_root;
  struct file_system_type         *s_type;
  const struct super_operations  *s_op;

  dev_t                   s_dev;
  struct block_device     *s_bdev;
  struct backing_dev_info *s_bdi;

  unsigned char      s_blocksize_bits;
  unsigned long      s_blocksize;
  loff_t              s_maxbytes;  /* Max file size */
  struct hlist_node  s_instances;

  struct list_head  s_mounts; /* list of mounts; _not_ for fs use */
  struct list_head  s_inodes; /* all inodes */
  struct list_head  s_inodes_wb; /* writeback inodes */
};
```
![linux-mem-meta-block-group.png](../Images/Kernel/mem-meta-block-group.png)

### directory
```C++
struct ext4_dir_entry {
  __le32  inode;      /* Inode number */
  __le16  rec_len;    /* Directory entry length */
  __le16  name_len;    /* Name length */
  char  name[EXT4_NAME_LEN];  /* File name */
};
struct ext4_dir_entry_2 {
  __le32  inode;      /* Inode number */
  __le16  rec_len;    /* Directory entry length */
  __u8  name_len;    /* Name length */
  __u8  file_type;
  char  name[EXT4_NAME_LEN];  /* File name */
};

/* EXT4_INDEX_FL */
struct dx_root
{
  struct fake_dirent dot;
  char dot_name[4];
  struct fake_dirent dotdot;
  char dotdot_name[4];
  struct dx_root_info
  {
    __le32 reserved_zero;
    u8 hash_version;
    u8 info_length; /* 8 */
    u8 indirect_levels;
    u8 unused_flags;
  }
  info;
  struct dx_entry  entries[0];
};

struct dx_entry
{
  __le32 hash;
  __le32 block;
};
```
![linux-file-directory.png](../Images/Kernel/file-directory.png)

![linux-dir-file-inode.png](../Images/Kernel/dir-file-inode.png)

### hard/symbolic link
```C++
 ln [args] [dst] [src]
```
![linux-file-link.png](../Images/Kernel/file-link.png)

### vfs
![linux-file-arche.png](../Images/Kernel/file-arche.png)
```C++
register_filesystem(&ext4_fs_type);

static struct file_system_type ext4_fs_type = {
  .owner    = THIS_MODULE,
  .name     = "ext4",
  .mount    = ext4_mount,
  .kill_sb  = kill_block_super,
  .fs_flags  = FS_REQUIRES_DEV,
};
```

### mount
```C++
SYSCALL_DEFINE5(mount, char __user *, dev_name,
  char __user *, dir_name, char __user *, type,
  unsigned long, flags, void __user *, data)
{
  return ksys_mount(dev_name, dir_name, type, flags, data);
}

int ksys_mount(
  char __user *dev_name, char __user *dir_name,
  char __user *type, unsigned long flags, void __user *data)
{
  int ret;
  char *kernel_type;
  char *kernel_dev;
  void *options;

  kernel_type = copy_mount_string(type);
  kernel_dev = copy_mount_string(dev_name);
  options = copy_mount_options(data);
  ret = do_mount(kernel_dev, dir_name, kernel_type, flags, options);

  return ret;
}

long do_mount(const char *dev_name, const char __user *dir_name,
    const char *type_page, unsigned long flags, void *data_page)
{
  struct path path;
  unsigned int mnt_flags = 0, sb_flags;
  int retval = 0;

  /* get the mountpoint */
  retval = user_path(dir_name, &path);

  if (flags & MS_REMOUNT)
    retval = do_remount(&path, flags, sb_flags, mnt_flags, data_page);
  else if (flags & MS_BIND)
    retval = do_loopback(&path, dev_name, flags & MS_REC);
  else if (flags & (MS_SHARED | MS_PRIVATE | MS_SLAVE | MS_UNBINDABLE))
    retval = do_change_type(&path, flags);
  else if (flags & MS_MOVE)
    retval = do_move_mount(&path, dev_name);
  else
    retval = do_new_mount(&path, type_page, sb_flags, mnt_flags,
              dev_name, data_page);
dput_out:
  path_put(&path);
  return retval;
}

static int do_new_mount(
  struct path *path, const char *fstype, int sb_flags,
  int mnt_flags, const char *name, void *data)
{
  struct file_system_type *type;
  struct vfsmount *mnt;
  int err;

  type = get_fs_type(fstype);
  mnt = vfs_kern_mount(type, sb_flags, name, data);
  if (!IS_ERR(mnt) && (type->fs_flags & FS_HAS_SUBTYPE) &&
      !mnt->mnt_sb->s_subtype)
    mnt = fs_set_subtype(mnt, fstype);

  put_filesystem(type);

  /* add a mount into a namespace's mount tree */
  err = do_add_mount(real_mount(mnt), path, mnt_flags);
  if (err)
    mntput(mnt);
  return err;
}

struct vfsmount *vfs_kern_mount(
  struct file_system_type *type, int flags, const char *name, void *data)
{
  struct mount *mnt;
  struct dentry *root;

  mnt = alloc_vfsmnt(name);
  root = mount_fs(type, flags, name, data);

  mnt->mnt.mnt_root = root;
  mnt->mnt.mnt_sb = root->d_sb;
  mnt->mnt_mountpoint = mnt->mnt.mnt_root;
  mnt->mnt_parent = mnt;
  lock_mount_hash();
  list_add_tail(&mnt->mnt_instance, &root->d_sb->s_mounts);
  unlock_mount_hash();
  return &mnt->mnt;
}
```

```C++
struct mount {
  struct hlist_node mnt_hash;
  struct mount      *mnt_parent;
  struct dentry     *mnt_mountpoint;
  struct vfsmount   mnt;

  union {
    struct rcu_head mnt_rcu;
    struct llist_node mnt_llist;
  };
  struct list_head mnt_mounts;  /* list of children, anchored here */
  struct list_head mnt_child;   /* and going through their mnt_child */
  struct list_head mnt_instance;/* mount instance on sb->s_mounts */
  const char *mnt_devname;      /* Name of device e.g. /dev/dsk/hda1 */
  struct list_head mnt_list;
} __randomize_layout;

struct vfsmount {
  struct dentry *mnt_root;    /* root of the mounted tree */
  struct super_block *mnt_sb; /* pointer to superblock */
  int mnt_flags;
} __randomize_layout;
```

```C++
struct dentry *mount_fs(
  struct file_system_type *type, int flags,
  const char *name, void *data)
{
  struct dentry *root;
  struct super_block *sb;
  root = type->mount(type, flags, name, data);
  sb = root->d_sb;
}

static struct dentry *ext4_mount(
  struct file_system_type *fs_type,
  int flags, const char *dev_name, void *data)
{
  return mount_bdev(fs_type, flags, dev_name, data, ext4_fill_super);
}
/* ---> see mount block device in IO part */
```
![linux-io-mount-example.png](../Images/Kernel/io-mount-example.png)

```C++
struct file {
  struct path     f_path;
  struct inode    *f_inode;  /* cached value */
  const struct file_operations  *f_op;
  struct address_space         *f_mapping;
  void* private_data; /* for special inode */

  spinlock_t                    f_lock;
  enum rw_hint                  f_write_hint;
  atomic_long_t                 f_count;
  unsigned int                  f_flags;
  fmode_t                       f_mode;
  loff_t                         f_pos;
  struct mutex                  f_pos_lock;
  struct fown_struct            f_owner;

  /* Used by fs/eventpoll.c to link all the hooks to this file */
  struct list_head              f_ep_links;
  struct list_head              f_tfile_llink;

  errseq_t                      f_wb_err;
  union {
      struct llist_node     fu_llist;
      struct rcu_head       fu_rcuhead;
  } f_u;
};

struct path {
  struct vfsmount *mnt;
  struct dentry *dentry;
};

// memroy chache of dirctories and files
// associate inode and file
struct dentry {
  unsigned int        d_flags;
  struct dentry       *d_parent;
  struct inode        *d_inode;
  struct super_block  *d_sb;
  const struct dentry_operations *d_op;

  struct hlist_bl_node d_hash;  /* lookup hash list */
  union {
    struct list_head  d_lru;    /* LRU list */
    wait_queue_head_t *d_wait;  /* in-lookup ones only */
  };
  struct qstr       d_name;
  unsigned char     d_iname[DNAME_INLINE_LEN];
  struct list_head  d_child;
  struct list_head  d_subdirs;
} __randomize_layout;
```

```C++
mount();
  ksys_mount(); /* copy type, dev_name, data to kernel */
    do_mount(); /* get path by name */
      do_new_mount(); /* get fs_type by type */
        vfs_kern_mount();
          alloc_vfsmnt();
          mount_fs();
            fs_type.mount(); /* dev_fs_type, {dev, ext4}_mount */
```

### open
```C++
struct task_struct {
  struct fs_struct    *fs;
  struct files_struct  *files;
  struct nsproxy      *nsproxy;
}

struct files_struct {
  struct file __rcu * fd_array[NR_OPEN_DEFAULT];
};

SYSCALL_DEFINE3(open, const char __user *, filename, int, flags, umode_t, mode)
{
  return do_sys_open(AT_FDCWD, filename, flags, mode);
}

long do_sys_open(int dfd, const char __user *filename, int flags, umode_t mode)
{
  struct filename *tmp = getname(filename);
  int fd = get_unused_fd_flags(flags);
  if (fd >= 0) {
    struct file *f = do_filp_open(dfd, tmp, &op);
    if (IS_ERR(f)) {
      put_unused_fd(fd);
      fd = PTR_ERR(f);
    } else {
      fsnotify_open(f);
      fd_install(fd, f);
    }
  }
  putname(tmp);
  return fd;
}

struct file *do_filp_open(int dfd, struct filename *pathname,
    const struct open_flags *op)
{
  set_nameidata(&nd, dfd, pathname);
  filp = path_openat(&nd, op, flags | LOOKUP_RCU);
  restore_nameidata();
  return filp;
}


static struct file *path_openat(struct nameidata *nd,
      const struct open_flags *op, unsigned flags)
{
  file = get_empty_filp();
  s = path_init(nd, flags);
  while (!(error = link_path_walk(s, nd)) &&
    (error = do_last(nd, file, op, &opened)) > 0) {
  }
  terminate_walk(nd);
  return file;
}

static int do_last(struct nameidata *nd,
       struct file *file, const struct open_flags *op,
       int *opened)
{
  error = lookup_fast(nd, &path, &inode, &seq); // loopup in dcache
  error = lookup_open(nd, &path, file, op, got_write, opened);
  error = vfs_open(&nd->path, file, current_cred());
}

static int lookup_open(struct nameidata *nd, struct path *path,
  struct file *file,
  const struct open_flags *op,
  bool got_write, int *opened)
{
  // open with O_CREAT flag
  if (!dentry->d_inode && (open_flag & O_CREAT)) {
    error = dir_inode->i_op->create(dir_inode, dentry, mode,
            open_flag & O_EXCL);
  }

  dentry = d_alloc_parallel(dir, &nd->last, &wq);
  struct dentry *res = dir_inode->i_op->lookup(
    dir_inode, dentry, nd->flags); /* ext4_lookup */
  path->dentry = dentry;
  path->mnt = nd->path.mnt;
}

const struct inode_operations ext4_dir_inode_operations = {
  .create    = ext4_create,
  .lookup    = ext4_lookup
}

int vfs_open(const struct path *path, struct file *file,
  const struct cred *cred)
{
  struct dentry *dentry = d_real(path->dentry, NULL, file->f_flags, 0);
  file->f_path = *path;
  return do_dentry_open(file, d_backing_inode(dentry), NULL, cred);
}

static int do_dentry_open(struct file *f,
  struct inode *inode,
  int (*open)(struct inode *, struct file *),
  const struct cred *cred)
{
  f->f_mode = OPEN_FMODE(f->f_flags) | FMODE_LSEEK |
        FMODE_PREAD | FMODE_PWRITE;
  path_get(&f->f_path);
  f->f_inode = inode;
  f->f_mapping = inode->i_mapping;
  f->f_op = fops_get(inode->i_fop);
  open = f->f_op->open;
  error = open(inode, f);
  f->f_flags &= ~(O_CREAT | O_EXCL | O_NOCTTY | O_TRUNC);
  file_ra_state_init(&f->f_ra, f->f_mapping->host->i_mapping);
  return 0;
}

const struct file_operations ext4_file_operations = {
  .open    = ext4_file_open,
};
```

```c++
do_sys_open();
  get_unused_fd_flags();
  do_filp_open();
    path_openat();
      get_empty_filp();
      link_path_walk();
      do_last();
        lookup_fast(); /* search in dcache */

        lookup_open(); /* not found in dcache */
          dir_inode->i_op->create(); /* O_CREAT */
            ext4_create();

          d_alloc_parallel(); /* alloc a dentry */
          dir_inode->i_op->lookup(); /* ext4_lookup */

        vfs_open();
          do_dentry_open();
            f->f_op->open();
              ext4_file_open();
  fd_install();
  putname();
```

![linux-dcache.png](../Images/Kernel/dcache.png)

### read/write
```C++
SYSCALL_DEFINE3(read, unsigned int, fd, char __user *, buf, size_t, count)
{
  struct fd f = fdget_pos(fd);
  loff_t pos = file_pos_read(f.file);
  ret = vfs_read(f.file, buf, count, &pos);
}

SYSCALL_DEFINE3(write, unsigned int, fd, const char __user *, buf,
    size_t, count)
{
  struct fd f = fdget_pos(fd);
  loff_t pos = file_pos_read(f.file);
    ret = vfs_write(f.file, buf, count, &pos);
}

ssize_t __vfs_read(struct file *file, char __user *buf, size_t count,
       loff_t *pos)
{
  if (file->f_op->read)
    return file->f_op->read(file, buf, count, pos);
  else if (file->f_op->read_iter)
    return new_sync_read(file, buf, count, pos);
  else
    return -EINVAL;
}


ssize_t __vfs_write(struct file *file, const char __user *p, size_t count,
        loff_t *pos)
{
  if (file->f_op->write)
    return file->f_op->write(file, p, count, pos);
  else if (file->f_op->write_iter)
    return new_sync_write(file, p, count, pos);
  else
    return -EINVAL;
}

static ssize_t new_sync_read(
  struct file *filp, char __user *buf, size_t len, loff_t *ppos)
{
  struct iovec iov = { .iov_base = buf, .iov_len = len };
  struct kiocb kiocb;
  struct iov_iter iter;
  ssize_t ret;

  init_sync_kiocb(&kiocb, filp);
  kiocb.ki_pos = *ppos;
  iov_iter_init(&iter, READ, &iov, 1, len);

  ret = call_read_iter(filp, &kiocb, &iter);
  *ppos = kiocb.ki_pos;
  return ret;
}

const struct file_operations ext4_file_operations = {
  .read_iter    = ext4_file_read_iter,
  .write_iter   = ext4_file_write_iter,
  .write_begin  = ext4_write_begin,
  .write_end    = ext4_write_end
}
/* ext4_file_{read, write}_iter -> generic_file_{read, write}_iter */
ssize_t generic_file_read_iter(struct kiocb *iocb, struct iov_iter *iter)
{
    if (iocb->ki_flags & IOCB_DIRECT) {
        struct address_space *mapping = file->f_mapping;
        retval = mapping->a_ops->direct_IO(iocb, iter);
    }
    retval = generic_file_buffered_read(iocb, iter, retval);
}

ssize_t __generic_file_write_iter(struct kiocb *iocb, struct iov_iter *from)
{
    if (iocb->ki_flags & IOCB_DIRECT) {
        written = generic_file_direct_write(iocb, from);
    } else {
        written = generic_perform_write(file, from, iocb->ki_pos);
    }
}
```

#### direct_IO
```C++
static const struct address_space_operations ext4_aops = {
  .direct_IO    = ext4_direct_IO,
};

static ssize_t ext4_direct_IO(struct kiocb *iocb, struct iov_iter *iter)
{
  struct file *file = iocb->ki_filp;
  struct inode *inode = file->f_mapping->host;
  size_t count = iov_iter_count(iter);
  loff_t offset = iocb->ki_pos;
  ssize_t ret;
  if (iov_iter_rw(iter) == READ)
    ret = ext4_direct_IO_read(iocb, iter);
  else
    ret = ext4_direct_IO_write(iocb, iter);
}

static ssize_t ext4_direct_IO_write(struct kiocb *iocb, struct iov_iter *iter)
{
  struct file *file = iocb->ki_filp;
  struct inode *inode = file->f_mapping->host;
  struct ext4_inode_info *ei = EXT4_I(inode);
  ssize_t ret;
  loff_t offset = iocb->ki_pos;
  size_t count = iov_iter_count(iter);

  ret = __blockdev_direct_IO(iocb, inode, inode->i_sb->s_bdev, iter,
           get_block_func, ext4_end_io_dio, NULL, dio_flags);
}

/* __blockdev_direct_IO -> do_blockdev_direct_IO */
static inline ssize_t do_blockdev_direct_IO(
  struct kiocb *iocb, struct inode *inode,
  struct block_device *bdev, struct iov_iter *iter,
  get_block_t get_block, dio_iodone_t end_io,
  dio_submit_t submit_io, int flags)
{
  /* see do_blockdev_direct_IO in IO management part */
}
```

#### buffered write
```C++
ssize_t generic_perform_write(
  struct file *file, struct iov_iter *i, loff_t pos)
{
  struct address_space *mapping = file->f_mapping;
  const struct address_space_operations *a_ops = mapping->a_ops;
  do {
    struct page *page;
    unsigned long offset;  /* Offset into pagecache page */
    unsigned long bytes;  /* Bytes to write to page */

    status = a_ops->write_begin(
      file, mapping, pos, bytes, flags, &page, &fsdata) {
        grab_cache_page_write_begin();
        ext4_journal_start();
      }

    copied = iov_iter_copy_from_user_atomic(page, i, offset, bytes);

    flush_dcache_page(page);
    status = a_ops->write_end(
      file, mapping, pos, bytes, copied, page, fsdata) {
        ext4_journal_stop() {
          // block_write_end->__block_commit_write->mark_buffer_dirty
        }
      }

    pos += copied;
    written += copied;

    balance_dirty_pages_ratelimited(mapping);
  } while (iov_iter_count(i));
}

/* get mapping page from address_space page cache radix tree,
 * if not exits, alloc a new page */
struct page *grab_cache_page_write_begin(
  struct address_space *mapping, pgoff_t index, unsigned flags)
{
  struct page *page;
  int fgp_flags = FGP_LOCK|FGP_WRITE|FGP_CREAT;
  page = pagecache_get_page(mapping, index, fgp_flags,
      mapping_gfp_mask(mapping));
  if (page)
    wait_for_stable_page(page);
  return page;
}

size_t iov_iter_copy_from_user_atomic(struct page *page,
    struct iov_iter *i, unsigned long offset, size_t bytes)
{
  char *kaddr = kmap_atomic(page), *p = kaddr + offset;
  iterate_all_kinds(i, bytes, v,
    copyin((p += v.iov_len) - v.iov_len, v.iov_base, v.iov_len),
    memcpy_from_page((p += v.bv_len) - v.bv_len, v.bv_page,
         v.bv_offset, v.bv_len),
    memcpy((p += v.iov_len) - v.iov_len, v.iov_base, v.iov_len)
  )
  kunmap_atomic(kaddr);
  return bytes;
}

void balance_dirty_pages_ratelimited(struct address_space *mapping)
{
  struct inode *inode = mapping->host;
  struct backing_dev_info *bdi = inode_to_bdi(inode);
  struct bdi_writeback *wb = &bdi->wb;
  int ratelimit;

  if (inode_cgwb_enabled(inode))
    wb = wb_get_create_current(bdi, GFP_KERNEL);
  if (!wb)
    wb = &bdi->wb;

  ratelimit = current->nr_dirtied_pause;
  if (wb->dirty_exceeded)
    ratelimit = min(ratelimit, 32 >> (PAGE_SHIFT - 10));

  if (unlikely(current->nr_dirtied >= ratelimit))
    balance_dirty_pages(mapping, wb, current->nr_dirtied);
}

// balance_dirty_pages -> wb_start_background_writeback -> wb_wakeup
struct backing_dev_info {
  struct list_head      bdi_list;
  struct bdi_writeback  wb; /* the root writeback info for this bdi */
  struct list_head      wb_list; /* list of all wbs */

  wait_queue_head_t wb_waitq;

  struct device *dev;
  struct device *owner;

  struct timer_list laptop_mode_wb_timer;
};

struct bdi_writeback {
  struct list_head    work_list;
  struct delayed_work dwork;    /* work item used for writeback */

  struct list_head b_dirty;     /* dirty inodes */
  struct list_head b_io;        /* parked for writeback */
  struct list_head b_more_io;   /* parked for more writeback */
  struct list_head b_dirty_time;/* time stamps are dirty */

  struct list_head bdi_node;    /* anchored at bdi->wb_list */
};

struct delayed_work {
  struct work_struct work;
  struct timer_list timer;

  /* target workqueue and CPU ->timer uses to queue ->work */
  struct workqueue_struct *wq;
  int cpu;
};

typedef void (*work_func_t)(struct work_struct *work);

struct work_struct {
  atomic_long_t data;
  struct list_head entry;
  work_func_t func;
};

struct wb_writeback_work {
  long nr_pages;
  struct super_block *sb;
  unsigned long *older_than_this;
  enum writeback_sync_modes sync_mode;
  unsigned int tagged_writepages:1;
  unsigned int for_kupdate:1;
  unsigned int range_cyclic:1;
  unsigned int for_background:1;
  unsigned int for_sync:1;  /* sync(2) WB_SYNC_ALL writeback */
  unsigned int auto_free:1; /* free on completion */
  enum wb_reason reason;    /* why was writeback initiated? */

  struct list_head list;    /* pending work list */
  struct wb_completion *done;/* set if the caller waits */
};

static void wb_wakeup(struct bdi_writeback *wb)
{
  spin_lock_bh(&wb->work_lock);
  if (test_bit(WB_registered, &wb->state))
    mod_delayed_work(bdi_wq, &wb->dwork, 0);
  spin_unlock_bh(&wb->work_lock);
}

/* insert a delayed work in bdi_wq */
static inline bool mod_delayed_work(
  struct workqueue_struct *wq,
  struct delayed_work *dwork,
  unsigned long delay)
{
  __queue_delayed_work(cpu, wq, dwork, delay);
}


static void __queue_delayed_work(int cpu, struct workqueue_struct *wq,
  struct delayed_work *dwork, unsigned long delay)
{
  struct timer_list *timer = &dwork->timer;
  struct work_struct *work = &dwork->work;

  WARN_ON_ONCE(!wq);
  WARN_ON_ONCE(timer->function != delayed_work_timer_fn);
  WARN_ON_ONCE(timer_pending(timer));
  WARN_ON_ONCE(!list_empty(&work->entry));

  if (!delay) {
    __queue_work(cpu, wq, &dwork->work);
    return;
  }

  dwork->wq = wq;
  dwork->cpu = cpu;
  timer->expires = jiffies + delay;

  if (unlikely(cpu != WORK_CPU_UNBOUND))
    add_timer_on(timer, cpu);
  else
    add_timer(timer);
}

```
![linux-file-bdi.png](../Images/Kernel/file-bdi.png)

##### bdi_wq
```C++
/* global variable, all backing task stored here */
/* bdi_wq serves all asynchronous writeback tasks */
struct workqueue_struct *bdi_wq;

struct workqueue_struct {
  struct list_head pwqs;  /* WR: all pwqs of this wq */
  struct list_head list;  /* PR: list of all workqueues */

  struct list_head      maydays; /* MD: pwqs requesting rescue */
  struct worker         *rescuer; /* I: rescue worker */

  struct pool_workqueue *dfl_pwq; /* PW: only for unbound wqs */

  struct pool_workqueue __percpu  *cpu_pwqs; /* I: per-cpu pwqs */
  struct pool_workqueue __rcu     *numa_pwq_tbl[]; /* PWR: unbound pwqs indexed by node */
};

struct backing_dev_info noop_backing_dev_info = {
  .name           = "noop",
  .capabilities   = BDI_CAP_NO_ACCT_AND_WRITEBACK,
};

static int default_bdi_init(void)
{
  int err;

  bdi_wq = alloc_workqueue("writeback", WQ_MEM_RECLAIM | WQ_FREEZABLE |
                WQ_UNBOUND | WQ_SYSFS, 0);
  if (!bdi_wq)
    return -ENOMEM;

  err = bdi_init(&noop_backing_dev_info);

  return err;
}

static int bdi_init(struct backing_dev_info *bdi)
{
    int ret;

    bdi->dev = NULL;

    kref_init(&bdi->refcnt);
    bdi->min_ratio = 0;
    bdi->max_ratio = 100;
    bdi->max_prop_frac = FPROP_FRAC_BASE;
    INIT_LIST_HEAD(&bdi->bdi_list);
    INIT_LIST_HEAD(&bdi->wb_list);
    init_waitqueue_head(&bdi->wb_waitq);

    ret = cgwb_bdi_init(bdi);

    return ret;
}

static int cgwb_bdi_init(struct backing_dev_info *bdi)
{
    int err;

    bdi->wb_congested = kzalloc(sizeof(*bdi->wb_congested), GFP_KERNEL);

    refcount_set(&bdi->wb_congested->refcnt, 1);

    err = wb_init(&bdi->wb, bdi, 1, GFP_KERNEL);
    if (err) {
        wb_congested_put(bdi->wb_congested);
        return err;
    }
    return 0;
}

static int wb_init(
  struct bdi_writeback *wb,
  struct backing_dev_info *bdi,
  int blkcg_id, gfp_t gfp)
{
  wb->bdi = bdi;
  wb->last_old_flush = jiffies;
  INIT_LIST_HEAD(&wb->b_dirty);
  INIT_LIST_HEAD(&wb->b_io);
  INIT_LIST_HEAD(&wb->b_more_io);
  INIT_LIST_HEAD(&wb->b_dirty_time);

  wb->bw_time_stamp = jiffies;
  wb->balanced_dirty_ratelimit = INIT_BW;
  wb->dirty_ratelimit = INIT_BW;
  wb->write_bandwidth = INIT_BW;
  wb->avg_write_bandwidth = INIT_BW;

  spin_lock_init(&wb->work_lock);
  INIT_LIST_HEAD(&wb->work_list);
  INIT_DELAYED_WORK(&wb->dwork, wb_workfn);
  wb->dirty_sleep = jiffies;
}

#define __INIT_DELAYED_WORK(_work, _func, _tflags)      \
  do {                \
    INIT_WORK(&(_work)->work, (_func));      \
    __setup_timer(&(_work)->timer, delayed_work_timer_fn,  \
            (unsigned long)(_work),      \
// wb_workfn -> wb_do_writeback -> wb_writeback -> writeback_sb_inodes
// -> __writeback_single_inode -> do_writepages -> ext4_writepages
// ---> see ext4_writepages in IO management
```

#### buffered read
```C++
static ssize_t generic_file_buffered_read(struct kiocb *iocb,
    struct iov_iter *iter, ssize_t written)
{
  struct file *filp = iocb->ki_filp;
  struct address_space *mapping = filp->f_mapping;
  struct inode *inode = mapping->host;
  for (;;) {
    struct page *page;
    pgoff_t end_index;
    loff_t isize;
    page = find_get_page(mapping, index);
    if (!page) {
      if (iocb->ki_flags & IOCB_NOWAIT)
        goto would_block;
      page_cache_sync_readahead(mapping,
          ra, filp,
          index, last_index - index);
      page = find_get_page(mapping, index);
      if (unlikely(page == NULL))
        goto no_cached_page;
    }
    if (PageReadahead(page)) {
      page_cache_async_readahead(mapping,
          ra, filp, page,
          index, last_index - index);
    }
    /*
     * Ok, we have the page, and it's up-to-date, so
     * now we can copy it to user space...
     */
    ret = copy_page_to_iter(page, offset, nr, iter);
    }
}
```

Direct IO and buffered IO will eventally call `submit_bio`.

![linux-file-read-write.png](../Images/Kernel/file-read-write.png)

### Question:
1. How to use inode bit map present all inodes?
2. Does system alloc a block for a dirctory or a file?
3. What does ext4_file_open do?
4. What happend when inserting data in a file?

### TODO
1. xarray

# IO
All devices have the corresponding device file in /dev(is devtmpfs file system), which has inode, but it's not associated with any data in the storage device, it's associated with the device's drive. And this device file belongs to a special file system: devtmpfs.

```c++
lsmod           // list installed modes
insmod *.ko     // install mode
mknod filename type major minor  // create dev file in /dev/

/*
/sys has following directories
/sys/devices represent all kernel devices in layer
/sys/dev has a char directory and block directory, maintain symbolic link files linked to real devices
/sys/block all block devices
/sys/module all modes information
*/
```
![linux-io-sysfs.png](../Images/Kernel/io-sysfs.png)

### char dev
#### kernal module
```C++
module_init(logibm_init);
module_exit(logibm_exit);
```

A kernel module consists:
1. header
  ```C++
  #include <linux/module.h>
  #include <linux/init.h>
  ```
2. define functions, handling kernel module main logic
3. Implement a file_operation interface
  ```C++
  static const struct file_operations lp_fops = {
    .owner    = THIS_MODULE,
    .write    = lp_write,
    .unlocked_ioctl  = lp_ioctl,
    #ifdef CONFIG_COMPAT
    .compat_ioctl  = lp_compat_ioctl,
    #endif
    .open    = lp_open,
    .release  = lp_release,
    #ifdef CONFIG_PARPORT_1284
    .read    = lp_read,
    #endif
    .llseek    = noop_llseek,
  };
  ```
4. define init and exit functions
5. invoke `module_init` and `moudle_exit`
6. declare lisence, invoke MODULE_LICENSE

#### insmod
![linux-io-char-dev-install-open.png](../Images/Kernel/io-char-dev-install-open.png)
```C++
static int __init lp_init (void)
{
  if (register_chrdev (LP_MAJOR, "lp", &lp_fops)) {
    printk (KERN_ERR "lp: unable to get major %d\n", LP_MAJOR);
    return -EIO;
  }
}

int __register_chrdev(
  unsigned int major, unsigned int baseminor,
  unsigned int count, const char *name,
  const struct file_operations *fops)
{
  struct char_device_struct *cd;
  struct cdev *cdev;
  int err = -ENOMEM;

  cd = __register_chrdev_region(major, baseminor, count, name);
  cdev = cdev_alloc();
  cdev->owner = fops->owner;
  cdev->ops = fops;
  kobject_set_name(&cdev->kobj, "%s", name);
  err = cdev_add(cdev, MKDEV(cd->major, baseminor), count);
  cd->cdev = cdev;
  return major ? 0 : cd->major;
}

int cdev_add(struct cdev *p, dev_t dev, unsigned count)
{
  int error;
  p->dev = dev;
  p->count = count;

  error = kobj_map(cdev_map, dev, count, NULL,
       exact_match, exact_lock, p);
  kobject_get(p->kobj.parent);

  return 0;
}
```

#### dev_fs_type
```C++
rest_init();
  kernel_thread(kernel_init);
    kernel_init_freeable();
      do_basic_setup();
        driver_init();
          devtmpfs_init();
          devices_init();
          buses_init();
          classes_init();
          firmware_init();
          hypervisor_init();

        shmem_init();
          register_filesystem(&shmem_fs_type);
            kern_mount(&shmem_fs_type);

        init_irq_proc();
        do_initcalls();

      workqueue_init();
      load_default_modules();
    run_init_process();
  kernel_thread(kthreadd);

int __init devtmpfs_init(void)
{
  int err = register_filesystem(&dev_fs_type);

  thread = kthread_run(devtmpfsd, &err, "kdevtmpfs");
  if (!IS_ERR(thread)) {
    wait_for_completion(&setup_done);
  } else {
    err = PTR_ERR(thread);
    thread = NULL;
  }
  return 0;
}

static int devtmpfsd(void *p)
{
  char options[] = "mode=0755";
  int *err = p;
  *err = ksys_unshare(CLONE_NEWNS);
  if (*err)
    goto out;
  *err = ksys_mount("devtmpfs", "/", "devtmpfs", MS_SILENT, options);
  if (*err)
    goto out;
  ksys_chdir("/.."); /* will traverse into overmounted root */
  ksys_chroot(".");
  complete(&setup_done);
  while (1) {
    spin_lock(&req_lock);
    while (requests) {
      struct req *req = requests;
      requests = NULL;
      spin_unlock(&req_lock);
      while (req) {
        struct req *next = req->next;
        req->err = handle(req->name, req->mode,
              req->uid, req->gid, req->dev);
        complete(&req->done);
        req = next;
      }
      spin_lock(&req_lock);
    }
    __set_current_state(TASK_INTERRUPTIBLE);
    spin_unlock(&req_lock);
    schedule();
  }
  return 0;
out:
  complete(&setup_done);
  return *err;
}
```

#### mount
```C++
/* mount -t type device destination_dir
 *
 * mount -> ksys_mount -> do_mount -> do_new_mount ->
 * vfs_kern_mount -> mount_fs -> fs_type.mount */
static struct file_system_type dev_fs_type = {
  .name = "devtmpfs",
  .mount = dev_mount,
  .kill_sb = kill_litter_super,
};

static struct dentry *dev_mount(
  struct file_system_type *fs_type, int flags,
  const char *dev_name, void *data)
{
#ifdef CONFIG_TMPFS
  return mount_single(fs_type, flags, data, shmem_fill_super);
#else
  return mount_single(fs_type, flags, data, ramfs_fill_super);
#endif
}

struct dentry *mount_single(
  struct file_system_type *fs_type,
  int flags, void *data,
  int (*fill_super)(struct super_block *, void *, int))
{
  struct super_block *s;
  int error;

  /* find or create a superblock */
  s = sget(fs_type, compare_single, set_anon_super, flags, NULL);
  if (!s->s_root) {
    error = fill_super(s, data, flags & SB_SILENT ? 1 : 0);
    if (error) {
      deactivate_locked_super(s);
      return ERR_PTR(error);
    }
    s->s_flags |= SB_ACTIVE;
  } else {
    do_remount_sb(s, flags, data, 0);
  }

  return dget(s->s_root);
}

// int ramfs_fill_super(struct super_block *sb, void *data, int silent)
int shmem_fill_super(struct super_block *sb, void *data, int silent)
{
  struct inode *inode;
  struct shmem_sb_info *sbinfo;
  int err = -ENOMEM;

  /* Round up to L1_CACHE_BYTES to resist false sharing */
  sbinfo = kzalloc(max((int)sizeof(struct shmem_sb_info),
        L1_CACHE_BYTES), GFP_KERNEL);

  if (!(sb->s_flags & SB_KERNMOUNT)) {
    sbinfo->max_blocks = shmem_default_max_blocks();
    sbinfo->max_inodes = shmem_default_max_inodes();
    if (shmem_parse_options(data, sbinfo, false)) {
      err = -EINVAL;
      goto failed;
    }
  } else {
    sb->s_flags |= SB_NOUSER;
  }

  sb->s_magic = TMPFS_MAGIC;
  sb->s_op = &shmem_ops;

  inode = shmem_get_inode(sb, NULL, S_IFDIR | sbinfo->mode, 0, VM_NORESERVE);
  sb->s_root = d_make_root(inode);

  return err;
}

static struct inode *shmem_get_inode(
  struct super_block *sb, const struct inode *dir,
  umode_t mode, dev_t dev, unsigned long flags)
{
  struct inode *inode;
  struct shmem_inode_info *info;
  struct shmem_sb_info *sbinfo = SHMEM_SB(sb);

  inode = new_inode(sb);
  if (inode) {
    switch (mode & S_IFMT) {
    default:
      inode->i_op = &shmem_special_inode_operations;
      init_special_inode(inode, mode, dev);
      break;
    case S_IFREG:
      inode->i_mapping->a_ops = &shmem_aops;
      inode->i_op = &shmem_inode_operations;
      inode->i_fop = &shmem_file_operations;
      break;
    case S_IFDIR:
      inode->i_op = &shmem_dir_inode_operations;
      inode->i_fop = &simple_dir_operations;
      break;
    case S_IFLNK:
      mpol_shared_policy_init(&info->policy, NULL);
      break;
    }

    lockdep_annotate_inode_mutex_key(inode);
  } else
    shmem_free_inode(sb);
  return inode;
}

void init_special_inode(struct inode *inode, umode_t mode, dev_t rdev)
{
  inode->i_mode = mode;
  if (S_ISCHR(mode)) {
    inode->i_fop = &def_chr_fops;
    inode->i_rdev = rdev;
  } else if (S_ISBLK(mode)) {
    inode->i_fop = &def_blk_fops;
    inode->i_rdev = rdev;
  } else if (S_ISFIFO(mode))
    inode->i_fop = &pipefifo_fops;
  else if (S_ISSOCK(mode))
    ; /* leave it no_open_fops */
}
```

```C++
mount();
  ksys_mount(); /* copy type, dev_name, data to kernel */
    do_mount(); /* get path by name */
      do_new_mount(); /* get fs_type by type */
        vfs_kern_mount();
          alloc_vfsmnt();
          mount_fs();
            fs_type.mount(); /* dev_fs_type */
              dev_mount();
                mount_single();
                  sget();
                  fill_super(); /* shmem_fill_super */
                    shmem_get_inode();
                      new_inode();
                      init_special_inode();
                    d_make_root();
          list_add_tail(&mnt->mnt_instance, &root->d_sb->s_mounts);
```

#### mknod
```C++
/* mknod /dev/ttyS0 c 4 64 */
SYSCALL_DEFINE3(mknod, const char __user *, filename, umode_t, mode, unsigned, dev)
{
  return sys_mknodat(AT_FDCWD, filename, mode, dev);
}

SYSCALL_DEFINE4(
  mknodat, int, dfd, const char __user *,
  filename, umode_t, mode, unsigned, dev)
{
  struct dentry *dentry;
  struct path path;
  dentry = user_path_create(dfd, filename, &path, lookup_flags);
  switch (mode & S_IFMT) {
    case 0:
    case S_IFREG:
      error = vfs_create(path.dentry->d_inode,dentry,mode,true);
      if (!error)
        ima_post_path_mknod(dentry);
      break;

    case S_IFCHR:
    case S_IFBLK:
      error = vfs_mknod(path.dentry->d_inode,dentry,mode,
          new_decode_dev(dev));
      break;

    case S_IFIFO: case S_IFSOCK:
      error = vfs_mknod(path.dentry->d_inode,dentry,mode,0);
      break;
  }
}

int vfs_mknod(struct inode *dir, struct dentry *dentry, umode_t mode, dev_t dev)
{
  error = dir->i_op->mknod(dir, dentry, mode, dev);
}

static const struct inode_operations ramfs_dir_inode_operations = {
  .mknod    = ramfs_mknod,
};

static const struct inode_operations shmem_dir_inode_operations = {
#ifdef CONFIG_TMPFS
  .mknod    = shmem_mknod,
};

static int shmem_mknod(
  struct inode *dir, struct dentry *dentry, umode_t mode, dev_t dev)
{
  struct inode *inode;
  int error = -ENOSPC;

  inode = shmem_get_inode(dir->i_sb, dir, mode, dev, VM_NORESERVE);
  if (inode) {
    dir->i_size += BOGO_DIRENT_SIZE;
    dir->i_ctime = dir->i_mtime = current_time(dir);
    d_instantiate(dentry, inode);
    dget(dentry); /* Extra count - pin the dentry in core */
  }
  return error;
}
```

```C++
mknod();
  sys_mknodat();
    user_path_create();
    vfs_create(); /* S_IFREG*/
    vfs_mknod();  /* S_IFCHR, S_IFBLK, S_IFPIPE, S_IFSOCK */
      dir->i_op->mknod();
        shmem_mknod();
          shmem_get_inode();
          d_instantiate();
            __d_instantiate();
              __d_set_inode_and_type();
          dget();
```

#### open dev
```C++
const struct file_operations def_chr_fops = {
  .open = chrdev_open,
};

static int chrdev_open(struct inode *inode, struct file *filp)
{
  const struct file_operations *fops;
  struct cdev *p;
  struct cdev *new = NULL;
  int ret = 0;

  p = inode->i_cdev;
  if (!p) {
    struct kobject *kobj;
    int idx;
    kobj = kobj_lookup(cdev_map, inode->i_rdev, &idx);
    new = container_of(kobj, struct cdev, kobj);
    p = inode->i_cdev;
    if (!p) {
      inode->i_cdev = p = new;
      list_add(&inode->i_devices, &p->list);
      new = NULL;
    }
  }
  fops = fops_get(p->ops);

  replace_fops(filp, fops);
  if (filp->f_op->open) {
    ret = filp->f_op->open(inode, filp);
  }
}
```

#### write char dev
```C++
ssize_t __vfs_write(struct file *file, const char __user *p, size_t count, loff_t *pos)
{
  if (file->f_op->write)
    return file->f_op->write(file, p, count, pos);
  else if (file->f_op->write_iter)
    return new_sync_write(file, p, count, pos);
  else
    return -EINVAL;
}

static ssize_t lp_write(
  struct file * file, const char __user * buf,
  size_t count, loff_t *ppos)
{
  unsigned int minor = iminor(file_inode(file));
  struct parport *port = lp_table[minor].dev->port;
  char *kbuf = lp_table[minor].lp_buffer;
  ssize_t retv = 0;
  ssize_t written;
  size_t copy_size = count;

  /* Need to copy the data from user-space. */
  if (copy_size > LP_BUFFER_SIZE)
    copy_size = LP_BUFFER_SIZE;

  if (copy_from_user (kbuf, buf, copy_size)) {
    retv = -EFAULT;
    goto out_unlock;
  }

  do {
    /* Write the data. */
    written = parport_write (port, kbuf, copy_size);
    if (written > 0) {
      copy_size -= written;
      count -= written;
      buf  += written;
      retv += written;
    }

    if (need_resched())
      schedule ();

    if (count) {
      copy_size = count;
      if (copy_size > LP_BUFFER_SIZE)
        copy_size = LP_BUFFER_SIZE;

      if (copy_from_user(kbuf, buf, copy_size)) {
        if (retv == 0)
          retv = -EFAULT;
        break;
      }
    }
  } while (count > 0);
}
```
![linux-io-char-dev-write.png](../Images/Kernel/io-char-dev-write.png)

#### ioctl
```C++
SYSCALL_DEFINE3(ioctl, unsigned int, fd, unsigned int, cmd, unsigned long, arg)
{
  int error;
  struct fd f = fdget(fd);
  error = do_vfs_ioctl(f.file, fd, cmd, arg);
  fdput(f);
  return error;
}

int do_vfs_ioctl(
  struct file *filp, unsigned int fd,
  unsigned int cmd, unsigned long arg)
{
  int error = 0;
  int __user *argp = (int __user *)arg;
  struct inode *inode = file_inode(filp);

  switch (cmd) {
  case FIONBIO:
    error = ioctl_fionbio(filp, argp);
    break;

  case FIOASYNC:
    error = ioctl_fioasync(fd, filp, argp);
    break;

  case FICLONE:
    return ioctl_file_clone(filp, arg, 0, 0, 0);

  default:
    if (S_ISREG(inode->i_mode))
      error = file_ioctl(filp, cmd, arg);
    else
      error = vfs_ioctl(filp, cmd, arg);
    break;
  }
  return error;
}

long vfs_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
  int error = -ENOTTY;
  if (!filp->f_op->unlocked_ioctl)
    goto out;

  error = filp->f_op->unlocked_ioctl(filp, cmd, arg);
  if (error == -ENOIOCTLCMD)
    error = -ENOTTY;
 out:
  return error;
}

static long lp_ioctl(
  struct file *file, unsigned int cmd,
  unsigned long arg)
{
  unsigned int minor;
  struct timeval par_timeout;
  int ret;

  minor = iminor(file_inode(file));
  mutex_lock(&lp_mutex);
  switch (cmd) {
  default:
    ret = lp_do_ioctl(minor, cmd, arg, (void __user *)arg);
    break;
  }
  mutex_unlock(&lp_mutex);
  return ret;
}


static int lp_do_ioctl(unsigned int minor, unsigned int cmd,
  unsigned long arg, void __user *argp)
{
  int status;
  int retval = 0;

  switch ( cmd ) {
    case LPTIME:
      if (arg > UINT_MAX / HZ)
        return -EINVAL;
      LP_TIME(minor) = arg * HZ/100;
      break;
    case LPCHAR:
      LP_CHAR(minor) = arg;
      break;
    case LPABORT:
      if (arg)
        LP_F(minor) |= LP_ABORT;
      else
        LP_F(minor) &= ~LP_ABORT;
      break;
    case LPABORTOPEN:
      if (arg)
        LP_F(minor) |= LP_ABORTOPEN;
      else
        LP_F(minor) &= ~LP_ABORTOPEN;
      break;
    case LPCAREFUL:
      if (arg)
        LP_F(minor) |= LP_CAREFUL;
      else
        LP_F(minor) &= ~LP_CAREFUL;
      break;
    case LPWAIT:
      LP_WAIT(minor) = arg;
      break;
    case LPSETIRQ:
      return -EINVAL;
      break;
    case LPGETIRQ:
      if (copy_to_user(argp, &LP_IRQ(minor),
          sizeof(int)))
        return -EFAULT;
      break;
    case LPGETSTATUS:
      if (mutex_lock_interruptible(&lp_table[minor].port_mutex))
        return -EINTR;
      lp_claim_parport_or_block (&lp_table[minor]);
      status = r_str(minor);
      lp_release_parport (&lp_table[minor]);
      mutex_unlock(&lp_table[minor].port_mutex);

      if (copy_to_user(argp, &status, sizeof(int)))
        return -EFAULT;
      break;
    case LPRESET:
      lp_reset(minor);
      break;
     case LPGETFLAGS:
       status = LP_F(minor);
      if (copy_to_user(argp, &status, sizeof(int)))
        return -EFAULT;
      break;
    default:
      retval = -EINVAL;
  }
  return retval
}
```
![linux-io-ioctl.png](../Images/Kernel/io-ioctl.png)

### interruption

![linux-io-irq.png](../Images/Kernel/io-irq.png)

```C++
static int logibm_open(struct input_dev *dev)
{
  if (request_irq(logibm_irq, logibm_interrupt, 0, "logibm", NULL)) {
    return -EBUSY;
  }
  outb(LOGIBM_ENABLE_IRQ, LOGIBM_CONTROL_PORT);
  return 0;
}

static irqreturn_t logibm_interrupt(int irq, void *dev_id)
{
  char dx, dy;
  unsigned char buttons;

  outb(LOGIBM_READ_X_LOW, LOGIBM_CONTROL_PORT);
  dx = (inb(LOGIBM_DATA_PORT) & 0xf);
  outb(LOGIBM_READ_X_HIGH, LOGIBM_CONTROL_PORT);
  dx |= (inb(LOGIBM_DATA_PORT) & 0xf) << 4;
  outb(LOGIBM_READ_Y_LOW, LOGIBM_CONTROL_PORT);
  dy = (inb(LOGIBM_DATA_PORT) & 0xf);
  outb(LOGIBM_READ_Y_HIGH, LOGIBM_CONTROL_PORT);
  buttons = inb(LOGIBM_DATA_PORT);
  dy |= (buttons & 0xf) << 4;
  buttons = ~buttons >> 5;

  input_report_rel(logibm_dev, REL_X, dx);
  input_report_rel(logibm_dev, REL_Y, dy);
  input_report_key(logibm_dev, BTN_RIGHT,  buttons & 1);
  input_report_key(logibm_dev, BTN_MIDDLE, buttons & 2);
  input_report_key(logibm_dev, BTN_LEFT,   buttons & 4);
  input_sync(logibm_dev);


  outb(LOGIBM_ENABLE_IRQ, LOGIBM_CONTROL_PORT);
  return IRQ_HANDLED
}

irqreturn_t (*irq_handler_t)(int irq, void * dev_id);
enum irqreturn {
  IRQ_NONE    = (0 << 0),
  IRQ_HANDLED    = (1 << 0),
  IRQ_WAKE_THREAD    = (1 << 1),
};
```

#### request_irq
```C++
static inline int request_irq(
  unsigned int irq, irq_handler_t handler,
  unsigned long flags, const char *name, void *dev)
{
  return request_threaded_irq(irq, handler, NULL, flags, name, dev);
}

int request_threaded_irq(
  unsigned int irq, irq_handler_t handler,
  irq_handler_t thread_fn, unsigned long irqflags,
  const char *devname, void *dev_id)
{
  struct irqaction *action;
  struct irq_desc *desc;
  int retval;

  desc = irq_to_desc(irq);

  action = kzalloc(sizeof(struct irqaction), GFP_KERNEL);
  action->handler = handler;
  action->thread_fn = thread_fn;
  action->flags = irqflags;
  action->name = devname;
  action->dev_id = dev_id;
  retval = __setup_irq(irq, desc, action);
}

struct irq_data {
  u32      mask;
  unsigned int    irq;
  unsigned long    hwirq;
  struct irq_common_data  *common;
  struct irq_chip    *chip;
  struct irq_domain  *domain;
#ifdef  CONFIG_IRQ_DOMAIN_HIERARCHY
  struct irq_data    *parent_data;
#endif
  void      *chip_data;
};

struct irq_desc {
  struct irqaction  *action;  /* IRQ action list */
  struct module    *owner;
  struct irq_data    irq_data;
  const char    *name;
};

struct irqaction {
  irq_handler_t       handler;
  void                *dev_id;
  void __percpu       *percpu_dev_id;
  struct irqaction    *next;
  irq_handler_t       thread_fn;
  struct task_struct  *thread;
  struct irqaction    *secondary;
  unsigned int        irq;
  unsigned int        flags;
  unsigned long       thread_flags;
  unsigned long       thread_mask;
  const char          *name;
  struct proc_dir_entry  *dir;
};

#ifdef CONFIG_SPARSE_IRQ
static RADIX_TREE(irq_desc_tree, GFP_KERNEL);

struct irq_desc *irq_to_desc(unsigned int irq)
{
  return radix_tree_lookup(&irq_desc_tree, irq);
}
#else /* !CONFIG_SPARSE_IRQ */
struct irq_desc irq_desc[NR_IRQS] __cacheline_aligned_in_smp = {
  [0 ... NR_IRQS-1] = { }
};
struct irq_desc *irq_to_desc(unsigned int irq)
{
  return (irq < NR_IRQS) ? irq_desc + irq : NULL;
}
#endif /* !CONFIG_SPARSE_IRQ */

static int
__setup_irq(unsigned int irq, struct irq_desc *desc, struct irqaction *new)
{
  struct irqaction *old, **old_ptr;
  unsigned long flags, thread_mask = 0;
  int ret, nested, shared = 0;

  new->irq = irq;

  if (new->thread_fn && !nested) {
    ret = setup_irq_thread(new, irq, false);
  }

  old_ptr = &desc->action;
  old = *old_ptr;
  if (old) {
    /* add new interrupt at end of irq queue */
    do {
      thread_mask |= old->thread_mask;
      old_ptr = &old->next;
      old = *old_ptr;
    } while (old);
  }
  *old_ptr = new;
  if (new->thread)
    wake_up_process(new->thread);
}

int wake_up_process(struct task_struct *p)
{
  return try_to_wake_up(p, TASK_NORMAL, 0);
}

static int setup_irq_thread(
  struct irqaction *new, unsigned int irq, bool secondary)
{
  struct task_struct *t;
  struct sched_param param = {
    .sched_priority = MAX_USER_RT_PRIO/2,
  };

  t = kthread_create(irq_thread, new, "irq/%d-%s", irq, new->name);
  sched_setscheduler_nocheck(t, SCHED_FIFO, &param);
  get_task_struct(t);
  new->thread = t;

  return 0;
}

/* How interrupt happens:
 * 1. extern dev sends physic interrupt to interrupt controller
 * 2. interrupt controller converts interrupt signal to interrupt vector, sends it to each cpu
 * 3. cpu call IRQ hanlder according to interrupt vector
 * 4. IRQ handler converts interrupt vector to abstract interrupt signal, handle it with irq_handler_t */


/* arch/x86/include/asm/irq_vectors.h
 * Linux IRQ vector layout.
 *
 * There are 256 IDT entries (per CPU - each entry is 8 bytes) which can
 * be defined by Linux. They are used as a jump table by the CPU when a
 * given vector is triggered - by a CPU-external, CPU-internal or
 * software-triggered event.
 *
 * Linux sets the kernel code address each entry jumps to early during
 * bootup, and never changes them. This is the general layout of the
 * IDT entries:
 *
 *  Vectors   0 ...  31 : system traps and exceptions - hardcoded events
 *  Vectors  32 ... 127 : device interrupts
 *  Vector  128         : legacy int80 syscall interface
 *  Vectors 129 ... INVALIDATE_TLB_VECTOR_START-1 except 204 : device interrupts
 *  Vectors INVALIDATE_TLB_VECTOR_START ... 255 : special interrupts
 *
 * 64-bit x86 has per CPU IDT tables, 32-bit has one shared IDT table.
 *
 * This file enumerates the exact layout of them: */
#define FIRST_EXTERNAL_VECTOR    0x20 // 32
#define IA32_SYSCALL_VECTOR    0x80   // 128
#define NR_VECTORS       256
#define FIRST_SYSTEM_VECTOR    NR_VECTORS

// arch/x86/kernel/traps.c, per cpu
struct gate_struct {
  u16    offset_low; // system irq hanlder addr
  u16    segment;   // KERNEL_CS
  struct idt_bits  bits;
  u16    offset_middle;  // addr >> 16
#ifdef CONFIG_X86_64
  u32    offset_high;    // adr >> 32
  u32    reserved;      // 0
#endif
};
struct idt_bits {
  u16 ist   : 3,   // DEFAULT_STACK
      zero  : 5,
      type  : 5,   // GATE_{INTERRUPT, TRAP, CALL, TASK}
      dpl   : 2,   // DPL (Descriptor privilege level), DLP0, DLP3
                   // RPL (Requested privilege level)
      p     : 1;
};
```

#### init idt_table
```C++
struct gate_desc idt_table[NR_VECTORS] __page_aligned_bss;

void __init trap_init(void)
{
  int i;
  set_intr_gate(X86_TRAP_DE, divide_error);
  // ...
  set_intr_gate(X86_TRAP_XF, simd_coprocessor_error);

  /* Reserve all the builtin and the syscall vector: */
  for (i = 0; i < FIRST_EXTERNAL_VECTOR; i++)
    set_bit(i, used_vectors);

#ifdef CONFIG_X86_32
  set_system_intr_gate(IA32_SYSCALL_VECTOR, entry_INT80_32);
  set_bit(IA32_SYSCALL_VECTOR, used_vectors);
#endif

  /* place at a fixed address */
  __set_fixmap(FIX_RO_IDT, __pa_symbol(idt_table), PAGE_KERNEL_RO);
  idt_descr.address = fix_to_virt(FIX_RO_IDT);
}

// set_intr_gate -> _set_gate
static inline void _set_gate(
  int gate, unsigned type, void *addr,
  unsigned dpl, unsigned ist, unsigned seg)
{
  gate_desc s;
  pack_gate(&s, type, (unsigned long)addr, dpl, ist, seg);
  write_idt_entry(idt_table, gate, &s);
}

// arch/x86/include/asm/traps.h
enum {
  X86_TRAP_DE = 0,  /*  0, Divide-by-zero */
  X86_TRAP_DB,    /*  1, Debug */
  X86_TRAP_NMI,    /*  2, Non-maskable Interrupt */
  X86_TRAP_BP,    /*  3, Breakpoint */
  X86_TRAP_OF,    /*  4, Overflow */
  X86_TRAP_BR,    /*  5, Bound Range Exceeded */
  X86_TRAP_UD,    /*  6, Invalid Opcode */
  X86_TRAP_NM,    /*  7, Device Not Available */
  X86_TRAP_DF,    /*  8, Double Fault */
  X86_TRAP_OLD_MF,  /*  9, Coprocessor Segment Overrun */
  X86_TRAP_TS,    /* 10, Invalid TSS */
  X86_TRAP_NP,    /* 11, Segment Not Present */
  X86_TRAP_SS,    /* 12, Stack Segment Fault */
  X86_TRAP_GP,    /* 13, General Protection Fault */
  X86_TRAP_PF,    /* 14, Page Fault */
  X86_TRAP_SPURIOUS,  /* 15, Spurious Interrupt */
  X86_TRAP_MF,    /* 16, x87 Floating-Point Exception */
  X86_TRAP_AC,    /* 17, Alignment Check */
  X86_TRAP_MC,    /* 18, Machine Check */
  X86_TRAP_XF,    /* 19, SIMD Floating-Point Exception */
  X86_TRAP_IRET = 32,  /* 32, IRET Exception */
};
```

#### init_IRQ
```C++
// after kernel called trap_init(), it invokes init_IRQ() to init other dev interrupt
void __init native_init_IRQ(void)
{
  int i;
  i = FIRST_EXTERNAL_VECTOR;
#ifndef CONFIG_X86_LOCAL_APIC
#define first_system_vector NR_VECTORS
#endif

  for_each_clear_bit_from(i, used_vectors, first_system_vector) {
    /* IA32_SYSCALL_VECTOR could be used in trap_init already. */
    set_intr_gate(i, irq_entries_start +
        8 * (i - FIRST_EXTERNAL_VECTOR));
  }
}

/* set `FIRST_SYSTEM_VECTOR - FIRST_EXTERNAL_VECTOR` handler to do_IRQ
 * irq_entries_start defined in arch\x86\entry\entry_{32, 64} */
ENTRY(irq_entries_start)
    vector=FIRST_EXTERNAL_VECTOR
    .rept (FIRST_SYSTEM_VECTOR - FIRST_EXTERNAL_VECTOR)
  pushl  $(~vector+0x80)      /* Note: always in signed byte range */
    vector=vector+1
  jmp  common_interrupt /* invoke do_IRQ */
  .align  8
    .endr
END(irq_entries_start)


common_interrupt:
  ASM_CLAC
  addq  $-0x80, (%rsp)      /* Adjust vector to [-256, -1] range */
  interrupt do_IRQ
  /* 0(%rsp): old RSP */
ret_from_intr:

  /* Interrupt came from user space */
GLOBAL(retint_user)

/* Returning to kernel space */
retint_kernel:

unsigned int __irq_entry do_IRQ(struct pt_regs *regs)
{
  struct pt_regs *old_regs = set_irq_regs(regs);
  struct irq_desc * desc;
  /* high bit used in ret_from_ code  */
  unsigned vector = ~regs->orig_ax;

  desc = __this_cpu_read(vector_irq[vector]);
  if (!handle_irq(desc, regs)) {

  }

  set_irq_regs(old_regs);
  return 1;
}

/* do_IRQ -> handle_irq -> */
static inline void generic_handle_irq_desc(struct irq_desc *desc)
{
  desc->handle_irq(desc);
}

irqreturn_t __handle_irq_event_percpu(struct irq_desc *desc, unsigned int *flags)
{
  irqreturn_t retval = IRQ_NONE;
  unsigned int irq = desc->irq_data.irq;
  struct irqaction *action;

  record_irq_time(desc);

  for_each_action_of_desc(desc, action) {
    irqreturn_t res;
    res = action->handler(irq, action->dev_id);
    switch (res) {
    case IRQ_WAKE_THREAD:
      __irq_wake_thread(desc, action);
    case IRQ_HANDLED:
      *flags |= action->flags;
      break;
    default:
      break;
    }
    retval |= res;
  }
  return retval;
}
```

#### init vector_irq
```C++
/* The interrupt vector interrupt controller sent to
 * each cpu is per cpu local variable, but the abstract
 * layer's virtual signal irq and it's irq_desc is global.
 * So per cpu needs its own mapping from vector to irq_desc */
typedef struct irq_desc* vector_irq_t[NR_VECTORS];
DECLARE_PER_CPU(vector_irq_t, vector_irq);

// assign virtual irq to a cpu
static int __assign_irq_vector(
  int irq, struct apic_chip_data *d,
  const struct cpumask *mask,
  struct irq_data *irqdata)
{
  static int current_vector = FIRST_EXTERNAL_VECTOR + VECTOR_OFFSET_START;
  static int current_offset = VECTOR_OFFSET_START % 16;
  int cpu, vector;

  while (cpu < nr_cpu_ids) {
    int new_cpu, offset;

    vector = current_vector;
    offset = current_offset;
next:
    vector += 16;
    if (vector >= first_system_vector) {
      offset = (offset + 1) % 16;
      vector = FIRST_EXTERNAL_VECTOR + offset;
    }
    /* If the search wrapped around, try the next cpu */
    if (unlikely(current_vector == vector))
      goto next_cpu;

    if (test_bit(vector, used_vectors))
      goto next;

    /* Found one! */
    current_vector = vector;
    current_offset = offset;
    /* Schedule the old vector for cleanup on all cpus */
    if (d->cfg.vector)
      cpumask_copy(d->old_domain, d->domain);
    for_each_cpu(new_cpu, vector_searchmask)
      per_cpu(vector_irq, new_cpu)[vector] = irq_to_desc(irq);
    goto update;

next_cpu:
    cpumask_or(searched_cpumask, searched_cpumask, vector_cpumask);
    cpumask_andnot(vector_cpumask, mask, searched_cpumask);
    cpu = cpumask_first_and(vector_cpumask, cpu_online_mask);
    continue;
}
```
![linux-io-interrupt-vector.png](../Images/Kernel/io-interrupt-vector.png)

![linux-io-interrupt.png](../Images/Kernel/io-interrupt.png)

### block dev
```C++
void init_special_inode(struct inode *inode, umode_t mode, dev_t rdev)
{
  inode->i_mode = mode;
  if (S_ISCHR(mode)) {
    inode->i_fop = &def_chr_fops;
    inode->i_rdev = rdev;
  } else if (S_ISBLK(mode)) {
    inode->i_fop = &def_blk_fops;
    inode->i_rdev = rdev;
  } else if (S_ISFIFO(mode))
    inode->i_fop = &pipefifo_fops;
  else if (S_ISSOCK(mode))
    ;  /* leave it no_open_fops */
}

const struct file_operations def_blk_fops = {
  .open           = blkdev_open, // call blkdev_get
  .release        = blkdev_close,
  .llseek         = block_llseek,
  .read_iter      = blkdev_read_iter,
  .write_iter     = blkdev_write_iter,
  .mmap           = generic_file_mmap,
  .fsync          = blkdev_fsync,
  .unlocked_ioctl = block_ioctl,
  .splice_read    = generic_file_splice_read,
  .splice_write   = iter_file_splice_write,
  .fallocate      = blkdev_fallocate,
};

static struct file_system_type ext4_fs_type = {
  .owner    = THIS_MODULE,
  .name    = "ext4",
  .mount    = ext4_mount,
  .kill_sb  = kill_block_super,
  .fs_flags  = FS_REQUIRES_DEV,
};
```

#### mount
```C++
/* mont -> ksys_mount -> do_mount -> do_new_mount ->
 * vfs_kern_mount -> mount_fs -> fs_type.mount */
static struct dentry *ext4_mount(
  struct file_system_type *fs_type, int flags,
  const char *dev_name, void *data)
{
  return mount_bdev(fs_type, flags, dev_name, data, ext4_fill_super);
}

struct dentry *mount_bdev(
  struct file_system_type *fs_type,
  int flags, const char *dev_name, void *data,
  int (*fill_super)(struct super_block *, void *, int))
{
  struct block_device *bdev;
  struct super_block *s;
  fmode_t mode = FMODE_READ | FMODE_EXCL;
  int error = 0;

  if (!(flags & MS_RDONLY))
    mode |= FMODE_WRITE;

  bdev = blkdev_get_by_path(dev_name, mode, fs_type);

  s = sget(fs_type, test_bdev_super, set_bdev_super, flags | MS_NOSEC, bdev);
  fill_super(s, data, flags & SB_SILENT ? 1 : 0);

  return dget(s->s_root);
}
```

##### blkdev_get_by_path
```C++
struct block_device *blkdev_get_by_path(
  const char *path, fmode_t mode, void *holder)
{
  struct block_device *bdev;
  int err;

  bdev = lookup_bdev(path);             // find blk device
  err = blkdev_get(bdev, mode, holder); // open blk device
  return bdev;
}
```
###### lookup_bdev
```C++
// 1. find the block device file under /dev which is devtmpfs file system
struct block_device *lookup_bdev(const char *pathname)
{
  struct block_device *bdev;
  struct inode *inode;
  struct path path;
  int error;

  if (!pathname || !*pathname)
    return ERR_PTR(-EINVAL);

  error = kern_path(pathname, LOOKUP_FOLLOW, &path);
  if (error)
    return ERR_PTR(error);

  inode = d_backing_inode(path.dentry);
  bdev = bd_acquire(inode);
  goto out;
}

// 2. find the block device in bdev file system by its key: dev_t
static struct block_device *bd_acquire(struct inode *inode)
{
  struct block_device *bdev;
  bdev = bdget(inode->i_rdev);
  if (bdev) {
    spin_lock(&bdev_lock);
    if (!inode->i_bdev) {
      bdgrab(bdev);
      inode->i_bdev = bdev;
      inode->i_mapping = bdev->bd_inode->i_mapping;
    }
  }
  return bdev;
}

struct block_device *bdget(dev_t dev)
{
  struct block_device *bdev;
  struct inode *inode;

  inode = iget5_locked(
    blockdev_superblock, hash(dev), bdev_test, bdev_set, &dev);
  bdev = &BDEV_I(inode)->bdev;

  if (inode->i_state & I_NEW) {
    bdev->bd_contains = NULL;
    bdev->bd_super = NULL;
    bdev->bd_inode = inode;
    bdev->bd_block_size = i_blocksize(inode);
    bdev->bd_part_count = 0;
    bdev->bd_invalidated = 0;
    inode->i_mode = S_IFBLK;
    inode->i_rdev = dev;
    inode->i_bdev = bdev;
    inode->i_data.a_ops = &def_blk_aops;
    mapping_set_gfp_mask(&inode->i_data, GFP_USER);
    spin_lock(&bdev_lock);
    list_add(&bdev->bd_list, &all_bdevs);
    spin_unlock(&bdev_lock);
    unlock_new_inode(inode);
  }
  return bdev;
}

// 3rd file system, save all blk device
struct super_block *blockdev_superblock;
static struct file_system_type bd_type = {
  .name           = "bdev",
  .mount          = bd_mount,
  .kill_sb        = kill_anon_super,
};
/* All block device inodes stored in bdev file system for convenient management.
 * Linux associates block_device with inode of bdev with bdev_inode */
struct bdev_inode {
  struct block_device bdev;
  struct inode vfs_inode;
};

// init blockdev_superblock
void __init bdev_cache_init(void)
{
  static struct vfsmount *bd_mnt;

  bdev_cachep = kmem_cache_create("bdev_cache", sizeof(struct bdev_inode), 0,
    (SLAB_HWCACHE_ALIGN|SLAB_RECLAIM_ACCOUNT|SLAB_MEM_SPREAD|SLAB_ACCOUNT|SLAB_PANIC),
    init_once);
  err = register_filesystem(&bd_type);
  if (err)
    panic("Cannot register bdev pseudo-fs");
  bd_mnt = kern_mount(&bd_type);
  if (IS_ERR(bd_mnt))
    panic("Cannot create bdev pseudo-fs");
  blockdev_superblock = bd_mnt->mnt_sb;   /* For writeback */
}

struct block_device {
  dev_t                 bd_dev;
  int                   bd_openers;
  struct super_block    *bd_super;
  struct block_device   *bd_contains;
  struct gendisk        *bd_disk;
  struct hd_struct      *bd_part;

  unsigned              bd_block_size;
  unsigned              bd_part_count;
  int                   bd_invalidated;
  struct request_queue  *bd_queue;
  struct backing_dev_info *bd_bdi;
  struct list_head        bd_list;
};

struct gendisk {
  int major;      /* major number of driver */
  int first_minor;
  int minors;     /* partition numbers */
  char disk_name[DISK_NAME_LEN]; /* name of major driver */
  char *(*devnode)(struct gendisk *gd, umode_t *mode);

  struct disk_part_tbl __rcu *part_tbl;
  struct hd_struct part0;

  const struct block_device_operations *fops;
  struct request_queue *queue;
  void *private_data;

  int flags;
  struct kobject *slave_dir;
};

struct hd_struct {
  sector_t start_sect;
  sector_t nr_sects;

  struct device   __dev;
  struct kobject *holder_dir;
  int policy, partno;
  struct partition_meta_info *info;

  struct disk_stats dkstats;
  struct percpu_ref ref;
  struct rcu_head   rcu_head;
};
```

###### blkdev_get
```C++
static int __blkdev_get(struct block_device *bdev, fmode_t mode, int for_part)
{
  struct gendisk *disk;
  struct module *owner;
  int ret;
  int partno;
  int perm = 0;

  if (mode & FMODE_READ)
    perm |= MAY_READ;
  if (mode & FMODE_WRITE)
    perm |= MAY_WRITE;

  disk = get_gendisk(bdev->bd_dev, &partno);

  owner = disk->fops->owner;

  if (!bdev->bd_openers) {
    bdev->bd_disk = disk;
    bdev->bd_queue = disk->queue;
    bdev->bd_contains = bdev;
    bdev->bd_partno = partno;

    if (!partno) { // 1. open whole disk
      ret = -ENXIO;
      bdev->bd_part = disk_get_part(disk, partno);

      if (disk->fops->open) {
        ret = disk->fops->open(bdev, mode);
      }

      if (!ret)
        bd_set_size(bdev,(loff_t)get_capacity(disk)<<9);

      if (bdev->bd_invalidated) {
        if (!ret)
          rescan_partitions(disk, bdev);
      }

    } else {  // 2. open a partition
      struct block_device *whole;
      whole = bdget_disk(disk, 0);
      ret = __blkdev_get(whole, mode, 1);
      bdev->bd_contains = whole;
      bdev->bd_part = disk_get_part(disk, partno);
      bd_set_size(bdev, (loff_t)bdev->bd_part->nr_sects << 9);
    }
  } else {
    if (bdev->bd_contains == bdev) {
      ret = 0;
      if (bdev->bd_disk->fops->open)
        ret = bdev->bd_disk->fops->open(bdev, mode);
      /* the same as first opener case, read comment there */
      if (bdev->bd_invalidated &&
          (!ret || ret == -ENOMEDIUM))
        bdev_disk_changed(bdev, ret == -ENOMEDIUM);
      if (ret)
        goto out_unlock_bdev;
    }
  }

  bdev->bd_openers++;
  if (for_part)
    bdev->bd_part_count++;
}

struct gendisk *get_gendisk(dev_t devt, int *partno)
{
  struct gendisk *disk = NULL;

  if (MAJOR(devt) != BLOCK_EXT_MAJOR) { // 1. get the whole device
    struct kobject *kobj;
    kobj = kobj_lookup(bdev_map, devt, partno);
    if (kobj)
      disk = dev_to_disk(kobj_to_dev(kobj));
  } else {                              // 2. get a partition
    struct hd_struct *part;
    part = idr_find(&ext_devt_idr, blk_mangle_minor(MINOR(devt)));
    if (part && get_disk(part_to_disk(part))) {
      *partno = part->partno;
      disk = part_to_disk(part);
    }
  }
  return disk;
}

#define dev_to_disk(device) container_of((device), struct gendisk, part0.__dev)
#define dev_to_part(device) container_of((device), struct hd_struct, __dev)
#define disk_to_dev(disk)   (&(disk)->part0.__dev)
#define part_to_dev(part)   (&((part)->__dev))

static inline struct gendisk *part_to_disk(struct hd_struct *part)
{
  if (likely(part)) {
    if (part->partno)
      return dev_to_disk(part_to_dev(part)->parent);
    else
      return dev_to_disk(part_to_dev(part));
  }
  return NULL;
}

static const struct block_device_operations sd_fops = {
  .owner          = THIS_MODULE,
  .open           = sd_open,
  .release        = sd_release,
  .ioctl          = sd_ioctl,
  .getgeo         = sd_getgeo,
  .compat_ioctl   = sd_compat_ioctl,
};

static int sd_open(struct block_device *bdev, fmode_t mode)
{

}
```

##### bdev_map
```C++
static struct kobj_map *bdev_map;
// map a dev_t with a gendisk
static inline void add_disk(struct gendisk *disk)
{
  device_add_disk(NULL, disk);
}

void device_add_disk(struct device *parent, struct gendisk *disk)
{
  blk_register_region(disk_devt(disk), disk->minors, NULL,
          exact_match, exact_lock, disk);
}

void blk_register_region(
  dev_t devt, unsigned long range, struct module *module,
  struct kobject *(*probe)(dev_t, int *, void *),
  int (*lock)(dev_t, void *), void *data)
{
  kobj_map(bdev_map, devt, range, module, probe, lock, data);
}
```

##### sget
```C++
// drivers/scsi/sd.c
static const struct block_device_operations sd_fops = {
  .owner      = THIS_MODULE,
  .open      = sd_open,
  .release    = sd_release,
  .ioctl      = sd_ioctl,
  .getgeo      = sd_getgeo,
#ifdef CONFIG_COMPAT
  .compat_ioctl    = sd_compat_ioctl,
#endif
  .check_events    = sd_check_events,
  .revalidate_disk  = sd_revalidate_disk,
  .unlock_native_capacity  = sd_unlock_native_capacity,
  .pr_ops      = &sd_pr_ops,
};

static int sd_open(struct block_device *bdev, fmode_t mode)
{ }

static int set_bdev_super(struct super_block *s, void *data)
{
  s->s_bdev = data;
  s->s_dev = s->s_bdev->bd_dev;
  s->s_bdi = bdi_get(s->s_bdev->bd_bdi);
  return 0;
}

struct super_block *sget(struct file_system_type *type,
  int (*test)(struct super_block *,void *),
  int (*set)(struct super_block *,void *),
  int flags,
  void *data)
{
  struct user_namespace *user_ns = current_user_ns();
  return sget_userns(type, test, set, flags, user_ns, data);
}

struct super_block *sget_userns(struct file_system_type *type,
  int (*test)(struct super_block *,void *),
  int (*set)(struct super_block *,void *),
  int flags, struct user_namespace *user_ns,
  void *data)
{
  struct super_block *s = NULL;
  struct super_block *old;
  int err;

  if (!s) {
    s = alloc_super(type, (flags & ~MS_SUBMOUNT), user_ns);
  }
  err = set(s, data);

  s->s_type = type;
  strlcpy(s->s_id, type->name, sizeof(s->s_id));
  list_add_tail(&s->s_list, &super_blocks);
  hlist_add_head(&s->s_instances, &type->fs_supers);
  spin_unlock(&sb_lock);
  get_filesystem(type);
  register_shrinker(&s->s_shrink);
  return s;
}
```

```C++
mount();
  ksys_mount(); /* copy type, dev_name, data to kernel */
    do_mount(); /* get path by name */
      do_new_mount(); /* get fs_type by type */
        vfs_kern_mount();
          alloc_vfsmnt();
          mount_fs();

            fs_type.mount();
              ext4_mount();
                mount_bdev();
                  blkdev_get_by_path();
                    lookup_bdev(); /* 1. find blk inode in /dev/xxx by name*/
                      kern_path();
                      d_backing_inode(); /* get inode of devtmpfs */
                      bd_acquire(); /* find blkdev by dev_t in blkdev fs */
                        bdget();
                          iget5_locked(blockdev_superblock, ...); /* blk dev fs */
                    blkdev_get();  /* 2. open blk device */
                      __blkdev_get();
                        get_gendisk();
                          kobj_lookup();          /* 1. open whole gisk */
                            dev_to_disk(kobj_to_dev(kobj));

                          idr_find(&ext_devt_idr); /* 2. open a partition */
                            part_to_disk(part);
                              dev_to_disk(part_to_dev(part));
                        disk->fops->open(bdev, mode);
                  sget();
                    sget_userns();
                      alloc_super();
                      set_bdev_super();
                  fill_super();
                    ext4_fill_super();
                  dget();
          list_add_tail(&mnt->mnt_instance, &root->d_sb->s_mounts);
```

![linux-io-gendisk.png](../Images/Kernel/io-gendisk.png)

![linux-io-bd.png](../Images/Kernel/io-bd.png)

### direct IO
```C++
read();
  vfs_read();
    file->f_op->write_iter(); /* ext4_file_read_iter */
      generic_file_read_iter();
        file->f_mapping->a_ops->direct_IO();
        generic_file_buffered_read();

static const struct address_space_operations ext4_aops = {
  .direct_IO    = ext4_direct_IO,
};

static ssize_t ext4_direct_IO(struct kiocb *iocb, struct iov_iter *iter)
{
  struct file *file = iocb->ki_filp;
  struct inode *inode = file->f_mapping->host;
  size_t count = iov_iter_count(iter);
  loff_t offset = iocb->ki_pos;
  ssize_t ret;
  ret = ext4_direct_IO_write(iocb, iter);
}

static ssize_t ext4_direct_IO_write(struct kiocb *iocb, struct iov_iter *iter)
{
  struct file *file = iocb->ki_filp;
  struct inode *inode = file->f_mapping->host;
  struct ext4_inode_info *ei = EXT4_I(inode);
  ssize_t ret;
  loff_t offset = iocb->ki_pos;
  size_t count = iov_iter_count(iter);

  ret = __blockdev_direct_IO(
    iocb, inode, inode->i_sb->s_bdev, iter,
    get_block_func, ext4_end_io_dio, NULL, dio_flags);
}

struct dio_submit {
  struct bio    *bio;
  unsigned      blkbits;
  unsigned      blkfactor;

  unsigned      start_zero_done;
  int           pages_in_io;
  sector_t      block_in_file;
  unsigned      blocks_available;

  sector_t final_block_in_request;

  get_block_t   *get_block;  /* block mapping function */
  dio_submit_t  *submit_io;  /* IO submition function */

  loff_t    logical_offset_in_bio;
  sector_t final_block_in_bio;
  sector_t next_block_for_io;

  struct page *cur_page;        /* The page */
  unsigned    cur_page_offset;
  unsigned    cur_page_len;
  sector_t    cur_page_block;   /* Where it starts */
  loff_t       cur_page_fs_offset;/* Offset in file */

  struct iov_iter *iter;

  unsigned head;      /* next page to process */
  unsigned tail;      /* last valid page + 1 */
  size_t from, to;
};

/* dio_state communicated between submission path and end_io */
struct dio {
  int             flags;
  int             op;
  int             op_flags;

  struct gendisk *bio_disk;
  struct inode   *inode;
  loff_t          i_size;  /* i_size when submitted */
  dio_iodone_t   *end_io; /* IO completion function */

  void *private;  /* copy from map_bh.b_private */

  /* BIO completion state */
  int   is_async;
  bool  defer_completion;
  bool  should_dirty;
  int   io_error;
  struct bio          *bio_list;
  struct task_struct  *waiter;

  struct kiocb *iocb;    /* kiocb */
  ssize_t result;        /* IO result */

  union {
    struct page *pages[DIO_PAGES];  /* page buffer */
    struct work_struct complete_work;/* deferred AIO completion */
  };
};

/* __blockdev_direct_IO -> do_blockdev_direct_IO */
static inline ssize_t do_blockdev_direct_IO(
  struct kiocb *iocb, struct inode *inode,
  struct block_device *bdev, struct iov_iter *iter,
  get_block_t get_block, dio_iodone_t end_io,
  dio_submit_t submit_io, int flags)
{
  unsigned i_blkbits = ACCESS_ONCE(inode->i_blkbits);
  unsigned blkbits = i_blkbits;
  unsigned blocksize_mask = (1 << blkbits) - 1;
  ssize_t retval = -EINVAL;
  size_t count = iov_iter_count(iter);
  loff_t offset = iocb->ki_pos;
  loff_t end = offset + count;
  struct dio *dio;
  struct dio_submit sdio = { 0, };
  struct buffer_head map_bh = { 0, };

  dio = kmem_cache_alloc(dio_cache, GFP_KERNEL);
  dio->flags = flags;
  dio->i_size = i_size_read(inode);
  dio->inode = inode;
  if (iov_iter_rw(iter) == WRITE) {
    dio->op = REQ_OP_WRITE;
    dio->op_flags = REQ_SYNC | REQ_IDLE;
    if (iocb->ki_flags & IOCB_NOWAIT)
      dio->op_flags |= REQ_NOWAIT;
  } else {
    dio->op = REQ_OP_READ;
  }
  sdio.blkbits = blkbits;
  sdio.blkfactor = i_blkbits - blkbits;
  sdio.block_in_file = offset >> blkbits;

  sdio.get_block = get_block;
  dio->end_io = end_io;
  sdio.submit_io = submit_io;
  sdio.final_block_in_bio = -1;
  sdio.next_block_for_io = -1;

  dio->iocb = iocb;
  dio->refcount = 1;

  sdio.iter = iter;
  sdio.final_block_in_request =
    (offset + iov_iter_count(iter)) >> blkbits;

  sdio.pages_in_io += iov_iter_npages(iter, INT_MAX);

  retval = do_direct_IO(dio, &sdio, &map_bh);
}

static int do_direct_IO(
  struct dio *dio, struct dio_submit *sdio,
  struct buffer_head *map_bh)
{
  const unsigned blkbits = sdio->blkbits;
  const unsigned i_blkbits = blkbits + sdio->blkfactor;
  int ret = 0;

  while (sdio->block_in_file < sdio->final_block_in_request) {
    struct page *page;
    size_t from, to;

    page = dio_get_page(dio, sdio);
    from = sdio->head ? 0 : sdio->from;
    to = (sdio->head == sdio->tail - 1) ? sdio->to : PAGE_SIZE;
    sdio->head++;

    while (from < to) {
      unsigned this_chunk_bytes;  /* # of bytes mapped */
      unsigned this_chunk_blocks;  /* # of blocks */
            ret = submit_page_section(dio, sdio, page,
              from,
              this_chunk_bytes,
              sdio->next_block_for_io,
              map_bh);

      sdio->next_block_for_io += this_chunk_blocks;
      sdio->block_in_file += this_chunk_blocks;
      from += this_chunk_bytes;
      dio->result += this_chunk_bytes;
      sdio->blocks_available -= this_chunk_blocks;
      if (sdio->block_in_file == sdio->final_block_in_request)
        break;
    }
  }
}
// submit_page_section -> dio_bio_submit -> submit_bio
```

### buffered IO write
```C++
/* wb_workfn -> wb_do_writeback -> wb_writeback
 * -> writeback_sb_inodes -> __writeback_single_inode
 * -> do_writepages -> ext4_writepages */
void wb_workfn(struct work_struct *work)
{
  struct bdi_writeback *wb = container_of(to_delayed_work(work),
            struct bdi_writeback, dwork);
  long pages_written;

  if (likely(!current_is_workqueue_rescuer()
    || !test_bit(WB_registered, &wb->state))) {
    do {
      pages_written = wb_do_writeback(wb);
    } while (!list_empty(&wb->work_list));
  } else {
    pages_written =
      writeback_inodes_wb(wb, 1024, WB_REASON_FORKER_THREAD);
  }

  if (!list_empty(&wb->work_list))
    wb_wakeup(wb);
  else if (wb_has_dirty_io(wb) && dirty_writeback_interval)
    wb_wakeup_delayed(wb);

  current->flags &= ~PF_SWAPWRITE;
}

/* Retrieve work items and do the writeback they describe */
static long wb_do_writeback(struct bdi_writeback *wb)
{
  struct wb_writeback_work *work;
  long wrote = 0;

  set_bit(WB_writeback_running, &wb->state);
  /* get next work from wb->work_list */
  while ((work = get_next_work_item(wb)) != NULL) {
    wrote += wb_writeback(wb, work);
    finish_writeback_work(wb, work);
  }

  wrote += wb_check_start_all(wb);
  wrote += wb_check_start_all(wb);
  wrote += wb_check_old_data_flush(wb);
  wrote += wb_check_background_flush(wb);
  clear_bit(WB_writeback_running, &wb->state);

  return wrote;
}

/* Explicit flushing or periodic writeback of "old" data. */
static long wb_writeback(
  struct bdi_writeback *wb, struct wb_writeback_work *work)
{
  unsigned long wb_start = jiffies;
  long nr_pages = work->nr_pages;
  unsigned long oldest_jif;
  struct inode *inode;
  long progress;
  struct blk_plug plug;

  oldest_jif = jiffies;
  work->older_than_this = &oldest_jif;

  blk_start_plug(&plug);
  spin_lock(&wb->list_lock);
  for (;;) {
    if (list_empty(&wb->b_io))
      queue_io(wb, work);

    if (work->sb)
      progress = writeback_sb_inodes(work->sb, wb, work);
    else
      progress = __writeback_inodes_wb(wb, work);

    wb_update_bandwidth(wb, wb_start);

    if (progress)
      continue;

    if (list_empty(&wb->b_more_io))
      break;

    inode = wb_inode(wb->b_more_io.prev);
    spin_lock(&inode->i_lock);
    spin_unlock(&wb->list_lock);
    /* This function drops i_lock... */
    inode_sleep_on_writeback(inode);
    spin_lock(&wb->list_lock);
  }
  spin_unlock(&wb->list_lock);
  blk_finish_plug(&plug);

  return nr_pages - work->nr_pages;
}

/* Write a portion of b_io inodes which belong to @sb. */
static long writeback_sb_inodes(
  struct super_block *sb,
  struct bdi_writeback *wb,
  struct wb_writeback_work *work)
{
  struct writeback_control wbc = {
    .sync_mode    = work->sync_mode,
    .tagged_writepages  = work->tagged_writepages,
    .for_kupdate    = work->for_kupdate,
    .for_background    = work->for_background,
    .for_sync    = work->for_sync,
    .range_cyclic    = work->range_cyclic,
    .range_start    = 0,
    .range_end    = LLONG_MAX,
  };

  unsigned long start_time = jiffies;
  long write_chunk;
  long wrote = 0;  /* count both pages and inodes */

  while (!list_empty(&wb->b_io)) {
    struct inode *inode = wb_inode(wb->b_io.prev);
    struct bdi_writeback *tmp_wb;

    write_chunk = writeback_chunk_size(wb, work);
    wbc.nr_to_write = write_chunk;
    wbc.pages_skipped = 0;

    __writeback_single_inode(inode, &wbc);

    wbc_detach_inode(&wbc);
    work->nr_pages -= write_chunk - wbc.nr_to_write;
    wrote += write_chunk - wbc.nr_to_write;

    if (need_resched()) {
      blk_flush_plug(current);
      cond_resched();
    }

    tmp_wb = inode_to_wb_and_lock_list(inode);
    spin_lock(&inode->i_lock);
    if (!(inode->i_state & I_DIRTY_ALL))
      wrote++;
    requeue_inode(inode, tmp_wb, &wbc);
    inode_sync_complete(inode);
    spin_unlock(&inode->i_lock);

    if (unlikely(tmp_wb != wb)) {
      spin_unlock(&tmp_wb->list_lock);
      spin_lock(&wb->list_lock);
    }

    if (wrote) {
      if (time_is_before_jiffies(start_time + HZ / 10UL))
        break;
      if (work->nr_pages <= 0)
        break;
    }
  }
  return wrote;
}

static long writeback_chunk_size(
  struct bdi_writeback *wb,
  struct wb_writeback_work *work)
{
  long pages;

  if (work->sync_mode == WB_SYNC_ALL || work->tagged_writepages)
    pages = LONG_MAX;
  else {
    pages = min(wb->avg_write_bandwidth / 2,
          global_wb_domain.dirty_limit / DIRTY_SCOPE);
    pages = min(pages, work->nr_pages);
    pages = round_down(pages + MIN_WRITEBACK_PAGES,
           MIN_WRITEBACK_PAGES);
  }

  return pages;
}

/* Write out an inode and its dirty pages */
static int __writeback_single_inode(
  struct inode *inode, struct writeback_control *wbc)
{
  struct address_space *mapping = inode->i_mapping;
  long nr_to_write = wbc->nr_to_write;
  unsigned dirty;
  int ret;

  ret = do_writepages(mapping, wbc);

  /*
   * Make sure to wait on the data before writing out the metadata.
   * This is important for filesystems that modify metadata on data
   * I/O completion. We don't do it for sync(2) writeback because it has a
   * separate, external IO completion path and ->sync_fs for guaranteeing
   * inode metadata is written back correctly.
   */
  if (wbc->sync_mode == WB_SYNC_ALL && !wbc->for_sync) {
    int err = filemap_fdatawait(mapping);
    if (ret == 0)
      ret = err;
  }

  /*
   * Some filesystems may redirty the inode during the writeback
   * due to delalloc, clear dirty metadata flags right before
   * write_inode()
   */
  spin_lock(&inode->i_lock);

  dirty = inode->i_state & I_DIRTY;
  if (inode->i_state & I_DIRTY_TIME) {
    if ((dirty & I_DIRTY_INODE) ||
        wbc->sync_mode == WB_SYNC_ALL ||
        unlikely(inode->i_state & I_DIRTY_TIME_EXPIRED) ||
        unlikely(time_after(jiffies,
          (inode->dirtied_time_when +
           dirtytime_expire_interval * HZ)))) {
      dirty |= I_DIRTY_TIME | I_DIRTY_TIME_EXPIRED;
    }
  } else
    inode->i_state &= ~I_DIRTY_TIME_EXPIRED;
  inode->i_state &= ~dirty;

  /*
   * Paired with smp_mb() in __mark_inode_dirty().  This allows
   * __mark_inode_dirty() to test i_state without grabbing i_lock -
   * either they see the I_DIRTY bits cleared or we see the dirtied
   * inode.
   *
   * I_DIRTY_PAGES is always cleared together above even if @mapping
   * still has dirty pages.  The flag is reinstated after smp_mb() if
   * necessary.  This guarantees that either __mark_inode_dirty()
   * sees clear I_DIRTY_PAGES or we see PAGECACHE_TAG_DIRTY.
   */
  smp_mb();

  if (mapping_tagged(mapping, PAGECACHE_TAG_DIRTY))
    inode->i_state |= I_DIRTY_PAGES;

  spin_unlock(&inode->i_lock);

  if (dirty & I_DIRTY_TIME)
    mark_inode_dirty_sync(inode);
  /* Don't write the inode if only I_DIRTY_PAGES was set */
  if (dirty & ~I_DIRTY_PAGES) {
    int err = write_inode(inode, wbc);
    if (ret == 0)
      ret = err;
  }
  trace_writeback_single_inode(inode, wbc, nr_to_write);
  return ret;
}

int do_writepages(
  struct address_space *mapping, struct writeback_control *wbc)
{
  int ret;

  if (wbc->nr_to_write <= 0)
    return 0;
  while (1) {
    if (mapping->a_ops->writepages)
      ret = mapping->a_ops->writepages(mapping, wbc);
    else
      ret = generic_writepages(mapping, wbc);

    if ((ret != -ENOMEM) || (wbc->sync_mode != WB_SYNC_ALL))
      break;
    cond_resched();
    congestion_wait(BLK_RW_ASYNC, HZ/50);
  }
  return ret;
}

static int ext4_writepages(
  struct address_space *mapping,
  struct writeback_control *wbc)
{
  struct mpage_da_data mpd;
  struct inode *inode = mapping->host;
  struct ext4_sb_info *sbi = EXT4_SB(mapping->host->i_sb);

  if (wbc->range_cyclic) {
    writeback_index = mapping->writeback_index;
    if (writeback_index)
      cycled = 0;
    mpd.first_page = writeback_index;
    mpd.last_page = -1;
  } else {
    mpd.first_page = wbc->range_start >> PAGE_SHIFT;
    mpd.last_page = wbc->range_end >> PAGE_SHIFT;
  }

  mpd.inode = inode;
  mpd.wbc = wbc;
  ext4_io_submit_init(&mpd.io_submit, wbc);
  mpd.do_map = 0;
  mpd.io_submit.io_end = ext4_init_io_end(inode, GFP_KERNEL);

  ret = mpage_prepare_extent_to_map(&mpd);
  ext4_io_submit(&mpd.io_submit);
}

struct mpage_da_data {
  struct inode              *inode;
  struct writeback_control  *wbc;

  pgoff_t first_page;  /* The first page to write */
  pgoff_t next_page;  /* Current page to examine */
  pgoff_t last_page;  /* Last page to examine */

  struct ext4_map_blocks map;
  struct ext4_io_submit io_submit; /* IO submission data */
  unsigned int do_map:1;
};

struct ext4_map_blocks {
  ext4_fsblk_t m_pblk;
  ext4_lblk_t  m_lblk;
  unsigned int m_len;
  unsigned int m_flags;
};

struct ext4_io_submit {
  struct writeback_control  *io_wbc;
  struct bio                *io_bio;
  ext4_io_end_t             *io_end;
  sector_t                  io_next_block;
};

/* find & lock contiguous range of dirty pages
* and underlying extent to map */
// mpage_prepare_extent_to_map -> mpage_process_page_bufs ->
// mpage_submit_page -> ext4_bio_write_page -> io_submit_add_bh
static int mpage_prepare_extent_to_map(struct mpage_da_data *mpd)
{
  struct address_space *mapping = mpd->inode->i_mapping;
  struct pagevec pvec;
  unsigned int nr_pages;
  long left = mpd->wbc->nr_to_write;
  pgoff_t index = mpd->first_page;
  pgoff_t end = mpd->last_page;
  int tag;
  int i, err = 0;
  int blkbits = mpd->inode->i_blkbits;
  ext4_lblk_t lblk;
  struct buffer_head *head;

  if (mpd->wbc->sync_mode == WB_SYNC_ALL || mpd->wbc->tagged_writepages)
    tag = PAGECACHE_TAG_TOWRITE;
  else
    tag = PAGECACHE_TAG_DIRTY;

  pagevec_init(&pvec);
  mpd->map.m_len = 0;
  mpd->next_page = index;
  while (index <= end) {
    nr_pages = pagevec_lookup_range_tag(&pvec, mapping, &index, end,
        tag);

    for (i = 0; i < nr_pages; i++) {
      struct page *page = pvec.pages[i];
      lock_page(page);
      wait_on_page_writeback(page);

      if (mpd->map.m_len == 0)
        mpd->first_page = page->index;
      mpd->next_page = page->index + 1;

      lblk = ((ext4_lblk_t)page->index) << (PAGE_SHIFT - blkbits);
      head = page_buffers(page);
      err = mpage_process_page_bufs(mpd, head, head, lblk);

      left--;
    }
    pagevec_release(&pvec);
    cond_resched();
  }
  return 0;
out:
  pagevec_release(&pvec);
  return err;
}

struct buffer_head {
  unsigned long b_state;         /* buffer state bitmap (see above) */
  struct buffer_head *b_this_page;/* circular list of page's buffers */
  struct page       *b_page;     /* the page this bh is mapped to */

  sector_t  b_blocknr; /* start block number */
  size_t    b_size;    /* size of mapping */
  char      *b_data;   /* pointer to data within the page */

  struct block_device *b_bdev;
  bh_end_io_t         *b_end_io;     /* I/O completion */
   void               *b_private;    /* reserved for b_end_io */
  struct list_head    b_assoc_buffers;/* associated with another mapping */
  struct address_space *b_assoc_map; /* mapping this buffer is
               associated with */
  atomic_t b_count;    /* users using this buffer_head */
};

/* submit page buffers for IO or add them to extent */
static int mpage_process_page_bufs(
  struct mpage_da_data *mpd,
  struct buffer_head *head,
  struct buffer_head *bh,
  ext4_lblk_t lblk)
{
  struct inode *inode = mpd->inode;
  int err;
  ext4_lblk_t blocks =
    (i_size_read(inode) + i_blocksize(inode) - 1) >> inode->i_blkbits;

  do {
    if (lblk >= blocks || !mpage_add_bh_to_extent(mpd, lblk, bh)) {
      /* Found extent to map? */
      if (mpd->map.m_len)
        return 0;
      /* Buffer needs mapping and handle is not started? */
      if (!mpd->do_map)
        return 0;
      /* Everything mapped so far and we hit EOF */
      break;
    }
  } while (lblk++, (bh = bh->b_this_page) != head);
  /* So far everything mapped? Submit the page for IO. */
  if (mpd->map.m_len == 0) {
    err = mpage_submit_page(mpd, head->b_page);
    if (err < 0)
      return err;
  }
  return lblk < blocks;
}

/* try to add bh to extent of blocks to map */
static bool mpage_add_bh_to_extent(
  struct mpage_da_data *mpd,
  ext4_lblk_t lblk,
  struct buffer_head *bh)
{
  struct ext4_map_blocks *map = &mpd->map;

  /* Buffer that doesn't need mapping for writeback? */
  if (!buffer_dirty(bh) || !buffer_mapped(bh) ||
      (!buffer_delay(bh) && !buffer_unwritten(bh))) {
    /* So far no extent to map => we write the buffer right away */
    if (map->m_len == 0)
      return true;
    return false;
  }

  /* First block in the extent? */
  if (map->m_len == 0) {
    /* We cannot map unless handle is started... */
    if (!mpd->do_map)
      return false;
    map->m_lblk = lblk;
    map->m_len = 1;
    map->m_flags = bh->b_state & BH_FLAGS;
    return true;
  }

  if (map->m_len >= MAX_WRITEPAGES_EXTENT_LEN)
    return false;

  /* Can we merge the block to our big extent? */
  if (lblk == map->m_lblk + map->m_len
    && (bh->b_state & BH_FLAGS) == map->m_flags) {
    map->m_len++;
    return true;
  }
  return false;
}

static int mpage_submit_page(struct mpage_da_data *mpd, struct page *page)
{
  int len;
  loff_t size;
  int err;

  clear_page_dirty_for_io(page);
  /*
   * We have to be very careful here!  Nothing protects writeback path
   * against i_size changes and the page can be writeably mapped into
   * page tables. So an application can be growing i_size and writing
   * data through mmap while writeback runs. clear_page_dirty_for_io()
   * write-protects our page in page tables and the page cannot get
   * written to again until we release page lock. So only after
   * clear_page_dirty_for_io() we are safe to sample i_size for
   * ext4_bio_write_page() to zero-out tail of the written page. We rely
   * on the barrier provided by TestClearPageDirty in
   * clear_page_dirty_for_io() to make sure i_size is really sampled only
   * after page tables are updated.
   */
  size = i_size_read(mpd->inode);
  if (page->index == size >> PAGE_SHIFT)
    len = size & ~PAGE_MASK;
  else
    len = PAGE_SIZE;
  err = ext4_bio_write_page(&mpd->io_submit, page, len, mpd->wbc, false);
  if (!err)
    mpd->wbc->nr_to_write--;
  mpd->first_page++;

  return err;
}

int ext4_bio_write_page(struct ext4_io_submit *io,
      struct page *page,
      int len,
      struct writeback_control *wbc,
      bool keep_towrite)
{
  struct page *data_page = NULL;
  struct inode *inode = page->mapping->host;
  unsigned block_start;
  struct buffer_head *bh, *head;
  int ret = 0;
  int nr_submitted = 0;
  int nr_to_submit = 0;

  BUG_ON(!PageLocked(page));
  BUG_ON(PageWriteback(page));

  if (keep_towrite)
    set_page_writeback_keepwrite(page);
  else
    set_page_writeback(page);
  ClearPageError(page);

  /*
   * Comments copied from block_write_full_page:
   *
   * The page straddles i_size.  It must be zeroed out on each and every
   * writepage invocation because it may be mmapped.  "A file is mapped
   * in multiples of the page size.  For a file that is not a multiple of
   * the page size, the remaining memory is zeroed when mapped, and
   * writes to that region are not written out to the file."
   */
  if (len < PAGE_SIZE)
    zero_user_segment(page, len, PAGE_SIZE);
  /*
   * In the first loop we prepare and mark buffers to submit. We have to
   * mark all buffers in the page before submitting so that
   * end_page_writeback() cannot be called from ext4_bio_end_io() when IO
   * on the first buffer finishes and we are still working on submitting
   * the second buffer.
   */
  bh = head = page_buffers(page);
  do {
    block_start = bh_offset(bh);
    if (block_start >= len) {
      clear_buffer_dirty(bh);
      set_buffer_uptodate(bh);
      continue;
    }
    if (!buffer_dirty(bh) || buffer_delay(bh) ||
        !buffer_mapped(bh) || buffer_unwritten(bh)) {
      /* A hole? We can safely clear the dirty bit */
      if (!buffer_mapped(bh))
        clear_buffer_dirty(bh);
      if (io->io_bio)
        ext4_io_submit(io);
      continue;
    }
    if (buffer_new(bh)) {
      clear_buffer_new(bh);
      clean_bdev_bh_alias(bh);
    }
    set_buffer_async_write(bh);
    nr_to_submit++;
  } while ((bh = bh->b_this_page) != head);

  bh = head = page_buffers(page);

  if (ext4_encrypted_inode(inode) && S_ISREG(inode->i_mode) &&
      nr_to_submit) {
    gfp_t gfp_flags = GFP_NOFS;

    /*
     * Since bounce page allocation uses a mempool, we can only use
     * a waiting mask (i.e. request guaranteed allocation) on the
     * first page of the bio.  Otherwise it can deadlock.
     */
    if (io->io_bio)
      gfp_flags = GFP_NOWAIT | __GFP_NOWARN;
  retry_encrypt:
    data_page = fscrypt_encrypt_page(inode, page, PAGE_SIZE, 0,
            page->index, gfp_flags);
    if (IS_ERR(data_page)) {
      ret = PTR_ERR(data_page);
      if (ret == -ENOMEM &&
          (io->io_bio || wbc->sync_mode == WB_SYNC_ALL)) {
        gfp_flags = GFP_NOFS;
        if (io->io_bio)
          ext4_io_submit(io);
        else
          gfp_flags |= __GFP_NOFAIL;
        congestion_wait(BLK_RW_ASYNC, HZ/50);
        goto retry_encrypt;
      }
      data_page = NULL;
      goto out;
    }
  }

  /* Now submit buffers to write */
  do {
    if (!buffer_async_write(bh))
      continue;
    ret = io_submit_add_bh(io, inode,
               data_page ? data_page : page, bh);
    if (ret) {
      /*
       * We only get here on ENOMEM.  Not much else
       * we can do but mark the page as dirty, and
       * better luck next time.
       */
      break;
    }
    nr_submitted++;
    clear_buffer_dirty(bh);
  } while ((bh = bh->b_this_page) != head);

  /* Error stopped previous loop? Clean up buffers... */
  if (ret) {
  out:
    if (data_page)
      fscrypt_restore_control_page(data_page);
    printk_ratelimited(KERN_ERR "%s: ret = %d\n", __func__, ret);
    redirty_page_for_writepage(wbc, page);
    do {
      clear_buffer_async_write(bh);
      bh = bh->b_this_page;
    } while (bh != head);
  }
  unlock_page(page);
  /* Nothing submitted - we have to end page writeback */
  if (!nr_submitted)
    end_page_writeback(page);
  return ret;
}

static int io_submit_add_bh(
  struct ext4_io_submit *io, struct inode *inode,
  struct page *page, struct buffer_head *bh)
{
  int ret;

  if (io->io_bio && bh->b_blocknr != io->io_next_block) {
submit_and_retry:
    ext4_io_submit(io);
  }
  if (io->io_bio == NULL) {
    ret = io_submit_init_bio(io, bh);
    if (ret)
      return ret;
    io->io_bio->bi_write_hint = inode->i_write_hint;
  }

  ret = bio_add_page(io->io_bio, page, bh->b_size, bh_offset(bh));
  if (ret != bh->b_size)
    goto submit_and_retry;

  wbc_account_io(io->io_wbc, page, bh->b_size);
  io->io_next_block++;
  return 0;
}

static int io_submit_init_bio(
  struct ext4_io_submit *io, struct buffer_head *bh)
{
  struct bio *bio;
  bio = bio_alloc(GFP_NOIO, BIO_MAX_PAGES);
  if (!bio)
    return -ENOMEM;
  wbc_init_bio(io->io_wbc, bio);
  bio->bi_iter.bi_sector = bh->b_blocknr * (bh->b_size >> 9);
  bio->bi_bdev = bh->b_bdev;
  bio->bi_end_io = ext4_end_bio;
  bio->bi_private = ext4_get_io_end(io->io_end);
  io->io_bio = bio;
  io->io_next_block = bh->b_blocknr;
  return 0;
}

void ext4_io_submit(struct ext4_io_submit *io)
{
  struct bio *bio = io->io_bio;
  if (bio) {
    int io_op_flags = io->io_wbc->sync_mode == WB_SYNC_ALL ?
          REQ_SYNC : 0;
    io->io_bio->bi_write_hint = io->io_end->inode->i_write_hint;
    bio_set_op_attrs(io->io_bio, REQ_OP_WRITE, io_op_flags);
    submit_bio(io->io_bio);
  }
  io->io_bio = NULL;
}
// ---> submit_bio
```

### submit_bio
```C++
// direct IO and buffered IO will come to here:
blk_qc_t submit_bio(struct bio *bio)
{
  return generic_make_request(bio);
}

blk_qc_t generic_make_request(struct bio *bio)
{
  /* bio_list_on_stack[0] contains bios submitted by the current
   * make_request_fn.
   * bio_list_on_stack[1] contains bios that were submitted before
   * the current make_request_fn, but that haven't been processed
   * yet. */
  struct bio_list bio_list_on_stack[2];
  blk_qc_t ret = BLK_QC_T_NONE;

  if (current->bio_list) {
    bio_list_add(&current->bio_list[0], bio);
    goto out;
  }

  bio_list_init(&bio_list_on_stack[0]);
  current->bio_list = bio_list_on_stack;
  do {
    struct request_queue *q = bdev_get_queue(bio->bi_bdev);

    if (likely(blk_queue_enter(q, bio->bi_opf & REQ_NOWAIT) == 0)) {
      struct bio_list lower, same;

      /* Create a fresh bio_list for all subordinate requests */
      bio_list_on_stack[1] = bio_list_on_stack[0];
      bio_list_init(&bio_list_on_stack[0]);
      ret = q->make_request_fn(q, bio); // blk_queue_bio

      blk_queue_exit(q);

      /* sort new bios into those for a lower level
       * and those for the same level */
      bio_list_init(&lower);
      bio_list_init(&same);
      while ((bio = bio_list_pop(&bio_list_on_stack[0])) != NULL)
        if (q == bdev_get_queue(bio->bi_bdev))
          bio_list_add(&same, bio);
        else
          bio_list_add(&lower, bio);
      /* now assemble so we handle the lowest level first */
      bio_list_merge(&bio_list_on_stack[0], &lower);
      bio_list_merge(&bio_list_on_stack[0], &same);
      bio_list_merge(&bio_list_on_stack[0], &bio_list_on_stack[1]);
    }

    bio = bio_list_pop(&bio_list_on_stack[0]);
  } while (bio);
  current->bio_list = NULL; /* deactivate */
out:
  return ret;
}

struct request_queue {
  // Together with queue_head for cacheline sharing
  struct list_head  queue_head;
  struct request    *last_merge;
  struct elevator_queue  *elevator;
  request_fn_proc    *request_fn;
  make_request_fn    *make_request_fn;
}

struct request {
  struct list_head queuelist;
  struct request_queue *q;
  struct bio *bio;
  struct bio *biotail;
}

struct bio {
  struct bio        *bi_next;  /* request queue link */
  struct block_device  *bi_bdev;
  blk_status_t      bi_status;
  struct bvec_iter  bi_iter;
  unsigned short    bi_vcnt;  /* how many bio_vec's */
  unsigned short    bi_max_vecs;  /* max bvl_vecs we can hold */
  atomic_t          __bi_cnt;  /* pin count */
  struct bio_vec    *bi_io_vec;  /* the actual vec list */

};

struct bvec_iter {
  sector_t    bi_sector;  /* device address in 512 byte sectors */
  unsigned int    bi_size;  /* residual I/O count */
  unsigned int    bi_idx;    /* current index into bvl_vec */
  unsigned int    bi_bvec_done;  /* number of bytes completed in
               current bvec */
};

struct bio_vec {
  struct page  *bv_page;
  unsigned int  bv_len;
  unsigned int  bv_offset;
}
```
![linux-io-bio.png](../Images/Kernel/io-bio.png)

#### make_request_fn
```C++
// make_request_fn -> blk_queue_bio
static blk_qc_t blk_queue_bio(struct request_queue *q, struct bio *bio)
{
  struct request *req, *free;
  unsigned int request_count = 0;

  switch (elv_merge(q, &req, bio)) {
  case ELEVATOR_BACK_MERGE:
    if (!bio_attempt_back_merge(q, req, bio))
      break;
    elv_bio_merged(q, req, bio);
    free = attempt_back_merge(q, req);
    if (free)
      __blk_put_request(q, free);
    else
      elv_merged_request(q, req, ELEVATOR_BACK_MERGE);
    goto out_unlock;
  case ELEVATOR_FRONT_MERGE:
    if (!bio_attempt_front_merge(q, req, bio))
      break;
    elv_bio_merged(q, req, bio);
    free = attempt_front_merge(q, req);
    if (free)
      __blk_put_request(q, free);
    else
      elv_merged_request(q, req, ELEVATOR_FRONT_MERGE);
    goto out_unlock;
  default:
    break;
  }

get_rq:
  req = get_request(q, bio->bi_opf, bio, GFP_NOIO);
  blk_init_request_from_bio(req, bio);
  add_acct_request(q, req, where);
  __blk_run_queue(q);
out_unlock:

  return BLK_QC_T_NONE;
}

enum elv_merge elv_merge(
  struct request_queue *q,
  struct request **req,
  struct bio *bio)
{
  struct elevator_queue *e = q->elevator;
  struct request *__rq;

  if (q->last_merge && elv_bio_merge_ok(q->last_merge, bio)) {
    enum elv_merge ret = blk_try_merge(q->last_merge, bio);
    if (ret != ELEVATOR_NO_MERGE) {
      *req = q->last_merge;
      return ret;
    }
  }

  __rq = elv_rqhash_find(q, bio->bi_iter.bi_sector);
  if (__rq && elv_bio_merge_ok(__rq, bio)) {
    *req = __rq;
    return ELEVATOR_BACK_MERGE;
  }

  if (e->uses_mq && e->type->ops.mq.request_merge)
    return e->type->ops.mq.request_merge(q, req, bio);
  else if (!e->uses_mq && e->type->ops.sq.elevator_merge_fn)
    return e->type->ops.sq.elevator_merge_fn(q, req, bio);

  return ELEVATOR_NO_MERGE;
}

enum elv_merge blk_try_merge(struct request *rq, struct bio *bio)
{
  if (blk_rq_pos(rq) + blk_rq_sectors(rq) == bio->bi_iter.bi_sector)
    return ELEVATOR_BACK_MERGE;
  else if (blk_rq_pos(rq) - bio_sectors(bio) == bio->bi_iter.bi_sector)
    return ELEVATOR_FRONT_MERGE;
  return ELEVATOR_NO_MERGE;
}

// elevator_merge_fn for iosched_cfq is:
static enum elv_merge cfq_merge(struct request_queue *q, struct request **req,
         struct bio *bio)
{
  struct cfq_data *cfqd = q->elevator->elevator_data;
  struct request *__rq;

  __rq = cfq_find_rq_fmerge(cfqd, bio);
  if (__rq && elv_bio_merge_ok(__rq, bio)) {
    *req = __rq;
    return ELEVATOR_FRONT_MERGE;
  }
  return ELEVATOR_NO_MERGE;
}

static struct request *cfq_find_rq_fmerge(
  struct cfq_data *cfqd, struct bio *bio)
{
  struct task_struct *tsk = current;
  struct cfq_io_cq *cic;
  struct cfq_queue *cfqq;

  cic = cfq_cic_lookup(cfqd, tsk->io_context);
  if (!cic)
    return NULL;

  cfqq = cic_to_cfqq(cic, op_is_sync(bio->bi_opf));
  if (cfqq)
    return elv_rb_find(&cfqq->sort_list, bio_end_sector(bio));

  return NUL
}
```

#### request_fn
```C++
static void scsi_request_fn(struct request_queue *q)
  __releases(q->queue_lock)
  __acquires(q->queue_lock)
{
  struct scsi_device *sdev = q->queuedata;
  struct Scsi_Host *shost;
  struct scsi_cmnd *cmd;
  struct request *req;

  /* To start with, we keep looping until the queue is empty, or until
   * the host is no longer able to accept any more requests. */
  shost = sdev->host;
  for (;;) {
    int rtn;
    /* get next queueable request.  We do this early to make sure
     * that the request is fully prepared even if we cannot
     * accept it. */
    req = blk_peek_request(q);

    /* Remove the request from the request list. */
    if (!(blk_queue_tagged(q) && !blk_queue_start_tag(q, req)))
      blk_start_request(req);

    cmd = req->special;

    /* Dispatch the command to the low-level driver. */
    cmd->scsi_done = scsi_done;
    rtn = scsi_dispatch_cmd(cmd);

  }
  return;

}
```

### init block device
```C++
// Small computer system interface
static struct scsi_device *scsi_alloc_sdev(
  struct scsi_target *starget,
  u64 lun, void *hostdata)
{
  struct scsi_device *sdev;
  sdev = kzalloc(sizeof(*sdev) + shost->transportt->device_size,
           GFP_ATOMIC);
  sdev->request_queue = scsi_alloc_queue(sdev);
}

struct request_queue *scsi_alloc_queue(struct scsi_device *sdev)
{
  struct Scsi_Host *shost = sdev->host;
  struct request_queue *q;

  q = blk_alloc_queue_node(GFP_KERNEL, NUMA_NO_NODE);
  if (!q)
    return NULL;
  q->cmd_size = sizeof(struct scsi_cmnd) + shost->hostt->cmd_size;
  q->rq_alloc_data = shost;
  q->request_fn = scsi_request_fn;
  q->init_rq_fn = scsi_init_rq;
  q->exit_rq_fn = scsi_exit_rq;
  q->initialize_rq_fn = scsi_initialize_rq;

  if (blk_init_allocated_queue(q) < 0) {
    blk_cleanup_queue(q);
    return NULL;
  }
  __scsi_init_queue(shost, q);
  return q
}

int blk_init_allocated_queue(struct request_queue *q)
{
  q->fq = blk_alloc_flush_queue(q, NUMA_NO_NODE, q->cmd_size);
  blk_queue_make_request(q, blk_queue_bio);
  /* init elevator */
  if (elevator_init(q, NULL)) {
    /* struct elevator_type elevator_noop // fifo
     * struct elevator_type iosched_deadline
     * struct elevator_type iosched_cfq */
  }
}
```

```C++
ext4_direct_IO();
  ext4_direct_IO_write();
    do_blockdev_direct_IO(); /* dio, dio_submit */
      do_direct_IO();

        dio_get_page();
          dio_refill_pages();
            get_user_pages_fast();
              get_user_pages_unlocked();
                own_read(&mm->mmap_sem);
                  __get_user_pages_locked();
                up_read(&mm->mmap_sem);

        get_more_blocks();
          ext4_dio_get_block();
            _ext4_get_block();
              ext4_map_blocks();
                ext4_es_lookup_extent();
                  ext4_ext_map_blocks();

        submit_page_section();
          dio_send_cur_page(); /* prepare bio data */
            dio_new_bio();
              dio_bio_alloc();
            dio_bio_add_page();

          dio_bio_submit();
            submit_bio();

wb_workfn();
  wb_do_writeback(); /* traverse Struct.1.wb_writeback_work */
    wb_writeback();
      /* traverse wb->b_io, which is inode list */
      writeback_sb_inodes(); /* Struct.2.writeback_control */
        __writeback_single_inode();
          do_writepages();

            ext4_writepages(); /* Struct.3.mpage_da_data */
              /* 1. find io data */
              mpage_prepare_extent_to_map();
                /* 1.1 find dirty pages */
                pagevec_lookup_range_tag();
                  find_get_pages_range_tag();
                    radix_tree_for_each_tagged();
                /* 1.2. process dirty pages */
                mpage_process_page_bufs();
                  mpage_add_bh_to_extent();
                  mpage_submit_page();
                    ext4_bio_write_page();
                      io_submit_add_bh(); /* Struct.4.buffer_head */
                        io_submit_init_bio(); /* init bio */
                          bio_alloc();
                          wbc_init_bio();
                        bio_add_page();
              /* 2. submit io data */
              ext4_io_submit();
                submit_bio();

      __writeback_inodes_wb();
  wb_wakeup();
    mod_delayed_work();


writeback_inodes_sb();
  writeback_inodes_sb_nr();
    get_nr_dirty_pages();
    __writeback_inodes_sb_nr(); /* wb_writeback_work */
      /* split a wb_writeback_work to all wb's of a bdi */
      bdi_split_work_to_wbs();
        /* split nr_pages to write according to bandwidth */
        wb_split_bdi_pages();
        wb_queue_work();
          list_add_tail(&work->list, &wb->work_list);
          mod_delayed_work(bdi_wq, &wb->dwork, 0);
            mod_delayed_work_on();
              __queue_delayed_work();
                __queue_work(cpu, wq, &dwork->work);
                add_timer();
      wb_wait_for_completion();


submit_bio();
  generic_make_request();
    make_request_fn(); /* blk_queue_bio */
      blk_queue_bio();
        elv_merge();
          blk_try_merge();
          elv_rqhash_find();
          elevator_merge_fn();
            cfq_merge();
        bio_attempt_back_merge();

        get_request(); /* can't merge */
        blk_init_request_from_bio();
        add_acct_request(); /* add req to queue */
          __elv_add_request();
            __elv_add_request();

        __blk_run_queue();
          __blk_run_queue_uncond();
            q->request_fn(q)();

      bio_list_pop();
      bio_list_add(); /* same or lower layer */
      bio_list_merge();

scsi_request_fn();
```
![linux-io-bio-request.png](../Images/Kernel/io-bio-request.png)

### Questions:
1. How to implement the IO port of dev register, and mmap of IO dev cache?
2. How to assign virutal irq to a cpu?
3. The interrupt controller sends interrupt to each cpu, which one will handle it?
4. How interrupt controller converts pthysical irq to interrupt vector?
5. Will first 32 interrupt call do_IRQ?
6. RAID, VLM
7. When to create block_device in bdev file system
8. Details of `dio_get_page()` and `get_more_blocks()`?
9. Who will call `delayed_work_timer_fn` and `wb_workfn`?
10. How data flow: `wb_writeback_work` -> `writeback_control` -> `mpage_da_data` -> `buffer_head` -> `bio` works?

# IPC
### pipe
```C++
SYSCALL_DEFINE1(pipe, int __user *, fildes)
{
  return sys_pipe2(fildes, 0);
}

SYSCALL_DEFINE2(pipe2, int __user *, fildes, int, flags)
{
  struct file *files[2];
  int fd[2];
  int error;

  error = __do_pipe_flags(fd, files, flags);
  if (!error) {
    if (unlikely(copy_to_user(fildes, fd, sizeof(fd)))) {
      error = -EFAULT;
    } else {
      fd_install(fd[0], files[0]);
      fd_install(fd[1], files[1]);
    }
  }
  return error;
}

static int __do_pipe_flags(int *fd, struct file **files, int flags)
{
  int error;
  int fdw, fdr;
  error = create_pipe_files(files, flags);
  error = get_unused_fd_flags(flags);
  fdr = error;

  error = get_unused_fd_flags(flags);
  fdw = error;

  fd[0] = fdr;
  fd[1] = fdw;
  return 0;
}

int create_pipe_files(struct file **res, int flags)
{
  int err;
  struct inode *inode = get_pipe_inode();
  struct file *f;
  struct path path;
  path.dentry = d_alloc_pseudo(pipe_mnt->mnt_sb, &empty_name);
  path.mnt = mntget(pipe_mnt);

  d_instantiate(path.dentry, inode);

  f = alloc_file(&path, FMODE_WRITE, &pipefifo_fops);
  f->f_flags = O_WRONLY | (flags & (O_NONBLOCK | O_DIRECT));
  f->private_data = inode->i_pipe;
  res[1] = f;

  res[0] = alloc_file(&path, FMODE_READ, &pipefifo_fops);
  path_get(&path);
  res[0]->private_data = inode->i_pipe;
  res[0]->f_flags = O_RDONLY | (flags & O_NONBLOCK);
  return 0;
}

static struct file_system_type pipe_fs_type = {
  .name    = "pipefs",
  .mount    = pipefs_mount,
  .kill_sb  = kill_anon_super,
};

static int __init init_pipe_fs(void)
{
  int err = register_filesystem(&pipe_fs_type);

  if (!err) {
    pipe_mnt = kern_mount(&pipe_fs_type);
  }
}

static struct inode* get_pipe_inode(void)
{
  struct pipe_inode_info *pipe = alloc_pipe_info();
  pipe->files = 2;
  pipe->readers = pipe->writers = 1;

  struct inode *inode = new_inode_pseudo(pipe_mnt->mnt_sb);
  inode->i_ino = get_next_ino();
  inode->i_pipe = pipe;
  inode->i_fop = &pipefifo_fops;
  inode->i_state = I_DIRTY;
  inode->i_mode = S_IFIFO | S_IRUSR | S_IWUSR;
  inode->i_uid = current_fsuid();
  inode->i_gid = current_fsgid();
  inode->i_atime = inode->i_mtime = inode->i_ctime = current_time(inode);

  return inode;
}

struct pipe_inode_info {
  struct mutex mutex;
  wait_queue_head_t rd_wait, wr_wait;
  unsigned int head;
  unsigned int tail;
  unsigned int max_usage;
  unsigned int ring_size;
  unsigned int readers;
  unsigned int writers;
  unsigned int files;
  unsigned int r_counter;
  unsigned int w_counter;
  struct page *tmp_page;
  struct fasync_struct *fasync_readers;
  struct fasync_struct *fasync_writers;
  unsigned int nrbufs, curbuf, buffers;
  struct pipe_buffer *bufs; /* the circular array of pipe buffers */
  struct user_struct *user;
};

struct pipe_buffer {
  struct page *page;
  unsigned int offset, len;
  const struct pipe_buf_operations *ops;
  unsigned int flags;
  unsigned long private;
};
```
![linux-ipc-pipe-2.png](../Images/Kernel/ipc-pipe-2.png)

### fifo
```C++
/* mkfifo is Glibc function */
int
mkfifo (const char *path, mode_t mode)
{
  dev_t dev = 0;
  return __xmknod (_MKNOD_VER, path, mode | S_IFIFO, &dev);
}

int
__xmknod (int vers, const char *path, mode_t mode, dev_t *dev)
{
  unsigned long long int k_dev;

  /* We must convert the value to dev_t type used by the kernel. */
  k_dev = (*dev) & ((1ULL << 32) - 1);
  return INLINE_SYSCALL (mknodat, 4, AT_FDCWD, path, mode,
                         (unsigned int) k_dev);
}

SYSCALL_DEFINE4(mknodat, int, dfd, const char __user *, filename, umode_t, mode,
    unsigned, dev)
{
  struct dentry *dentry;
  struct path path;
  dentry = user_path_create(dfd, filename, &path, lookup_flags);
  switch (mode & S_IFMT) {
    case 0:
    case S_IFREG:
      error = vfs_create(path.dentry->d_inode,dentry,mode,true);
      if (!error)
        ima_post_path_mknod(dentry);
      break;

    case S_IFCHR:
    case S_IFBLK:
      error = vfs_mknod(path.dentry->d_inode,dentry,mode,
          new_decode_dev(dev));
      break;

    case S_IFIFO:
    case S_IFSOCK:
      error = vfs_mknod(path.dentry->d_inode,dentry,mode,0);
      break;
  }
}

int vfs_mknod(struct inode *dir, struct dentry *dentry, umode_t mode, dev_t dev)
{
  error = dir->i_op->mknod(dir, dentry, mode, dev);
}

const struct inode_operations ext4_dir_inode_operations = {
  .mknod    = ext4_mknod,
};

static int ext4_mknod(struct inode *dir, struct dentry *dentry,
          umode_t mode, dev_t rdev)
{
  handle_t *handle;
  struct inode *inode;

  inode = ext4_new_inode_start_handle(dir, mode, &dentry->d_name, 0,
              NULL, EXT4_HT_DIR, credits);
  handle = ext4_journal_current_handle();
  if (!IS_ERR(inode)) {
    init_special_inode(inode, inode->i_mode, rdev);
    inode->i_op = &ext4_special_inode_operations;
    err = ext4_add_nondir(handle, dentry, inode);
    if (!err && IS_DIRSYNC(dir))
      ext4_handle_sync(handle);
  }
  if (handle)
    ext4_journal_stop(handle);
}

#define ext4_new_inode_start_handle(dir, mode, qstr, goal, owner, \
            type, nblocks)        \
  __ext4_new_inode(NULL, (dir), (mode), (qstr), (goal), (owner), \
       0, (type), __LINE__, (nblocks))

void init_special_inode(struct inode *inode, umode_t mode, dev_t rdev)
{
  inode->i_mode = mode;
  if (S_ISCHR(mode)) {
    inode->i_fop = &def_chr_fops;
    inode->i_rdev = rdev;
  } else if (S_ISBLK(mode)) {
    inode->i_fop = &def_blk_fops;
    inode->i_rdev = rdev;
  } else if (S_ISFIFO(mode))
    inode->i_fop = &pipefifo_fops;
  else if (S_ISSOCK(mode))
    ;  /* leave it no_open_fops */
  else
}

static int fifo_open(struct inode *inode, struct file *filp)
{
  struct pipe_inode_info *pipe;
  bool is_pipe = inode->i_sb->s_magic == PIPEFS_MAGIC;
  int ret;
  filp->f_version = 0;

  if (inode->i_pipe) {
    pipe = inode->i_pipe;
    pipe->files++;
  } else {
    pipe = alloc_pipe_info();
    pipe->files = 1;
    inode->i_pipe = pipe;
    spin_unlock(&inode->i_lock);
  }
  filp->private_data = pipe;
  filp->f_mode &= (FMODE_READ | FMODE_WRITE);

  switch (filp->f_mode) {
  case FMODE_READ:
    pipe->r_counter++;
    if (pipe->readers++ == 0)
      wake_up_partner(pipe);
    if (!is_pipe && !pipe->writers) {
      if ((filp->f_flags & O_NONBLOCK)) {
        filp->f_version = pipe->w_counter;
      } else {
        if (wait_for_partner(pipe, &pipe->w_counter))
          goto err_rd;
      }
    }
    break;
  case FMODE_WRITE:
    pipe->w_counter++;
    if (!pipe->writers++)
      wake_up_partner(pipe);
    if (!is_pipe && !pipe->readers) {
      if (wait_for_partner(pipe, &pipe->r_counter))
        goto err_wr;
    }
    break;
  case FMODE_READ | FMODE_WRITE:
    pipe->readers++;
    pipe->writers++;
    pipe->r_counter++;
    pipe->w_counter++;
    if (pipe->readers == 1 || pipe->writers == 1)
      wake_up_partner(pipe);
    break;
  }
}
```
![linux-ipc-fifo.png](../Images/Kernel/ipc-fifo.png)

### signal
#### resigter a sighand
```C++
struct task_struct {
    struct signal_struct    *signal;
    struct sighand_struct   *sighand;
    sigset_t                blocked;
    sigset_t                real_blocked;
    /* Restored if set_restore_sigmask() was used: */
    sigset_t                saved_sigmask;
    struct sigpending       pending;
    unsigned long           sas_ss_sp;
    size_t                  sas_ss_size;
    unsigned int            sas_ss_flags;
};

struct sigpending {
  struct list_head list;
  sigset_t signal;
};

struct signal_struct {
  struct list_head  thread_head;

  wait_queue_head_t    wait_chldexit;  /* for wait4() */
  struct task_struct  *curr_target;
  struct sigpending    shared_pending;
  struct hlist_head    multiprocess;

  int                  posix_timer_id;
  struct list_head    posix_timers;
  struct hrtimer      real_timer;
  ktime_t             it_real_incr;
  struct cpu_itimer   it[2];
  struct thread_group_cputimer  cputimer;
  struct task_cputime           cputime_expires;

  struct list_head  cpu_timers[3];
  struct pid        *pids[PIDTYPE_MAX];

  struct tty_struct *tty; /* NULL if no tty */

  struct prev_cputime prev_cputime;
  struct task_io_accounting ioac;
  unsigned long long sum_sched_runtime;
  struct rlimit rlim[RLIM_NLIMITS];
  struct mm_struct *oom_mm;
};

struct sighand_struct {
  atomic_t              count;
  struct k_sigaction    action[_NSIG];
  spinlock_t            siglock;
  wait_queue_head_t     signalfd_wqh;
};

struct k_sigaction {
  struct sigaction sa;
};

struct sigaction {
  unsigned int    sa_flags;
  __sighandler_t  sa_handler;
  __sigrestore_t  sa_restorer;
  sigset_t        sa_mask;  /* mask last for extensibility */
};

typedef void (*sighandler_t)(int);
sighandler_t signal(int signum, sighandler_t handler);
int sigaction(int signum, const struct sigaction *act, struct sigaction *oldact);

#define signal __sysv_signal
__sighandler_t __sysv_signal (int sig, __sighandler_t handler)
{
  struct sigaction act, oact;
  act.sa_handler = handler;
  __sigemptyset (&act.sa_mask);
  act.sa_flags = SA_ONESHOT | SA_NOMASK | SA_INTERRUPT;
  act.sa_flags &= ~SA_RESTART;
  if (__sigaction (sig, &act, &oact) < 0)
    return SIG_ERR;
  return oact.sa_handler;
}
weak_alias (__sysv_signal, sysv_signal)

int __sigaction (int sig, const struct sigaction *act, struct sigaction *oact)
{
  return __libc_sigaction (sig, act, oact);
}

int __libc_sigaction (int sig, const struct sigaction *act, struct sigaction *oact)
{
  int result;
  struct kernel_sigaction kact, koact;

  if (act)
  {
    kact.k_sa_handler = act->sa_handler;
    memcpy (&kact.sa_mask, &act->sa_mask, sizeof (sigset_t));
    kact.sa_flags = act->sa_flags | SA_RESTORER;

    kact.sa_restorer = &restore_rt;
  }

  result = INLINE_SYSCALL(rt_sigaction, 4,
                           sig, act ? &kact : NULL,
                           oact ? &koact : NULL, _NSIG / 8);
  if (oact && result >= 0)
  {
    oact->sa_handler = koact.k_sa_handler;
    memcpy (&oact->sa_mask, &koact.sa_mask, sizeof (sigset_t));
    oact->sa_flags = koact.sa_flags;
    oact->sa_restorer = koact.sa_restorer;
  }
  return result;
}

SYSCALL_DEFINE4(rt_sigaction, int, sig,
    const struct sigaction __user *, act,
    struct sigaction __user *, oact,
    size_t, sigsetsize)
{
  struct k_sigaction new_sa, old_sa;
  int ret = -EINVAL;
  if (act) {
    if (copy_from_user(&new_sa.sa, act, sizeof(new_sa.sa)))
      return -EFAULT;
  }

  ret = do_sigaction(sig, act ? &new_sa : NULL, oact ? &old_sa : NULL);
  if (!ret && oact) {
    if (copy_to_user(oact, &old_sa.sa, sizeof(old_sa.sa)))
      return -EFAULT;
  }
out:
  return ret;
}

int do_sigaction(int sig, struct k_sigaction *act, struct k_sigaction *oact)
{
  struct task_struct *p = current, *t;
  struct k_sigaction *k;
  sigset_t mask;
  k = &p->sighand->action[sig-1];

  spin_lock_irq(&p->sighand->siglock);
  if (oact)
    *oact = *k;

  if (act) {
    sigdelsetmask(&act->sa.sa_mask,
            sigmask(SIGKILL) | sigmask(SIGSTOP));
    *k = *act;
  }

  spin_unlock_irq(&p->sighand->siglock);
  return 0;
}
```
![linux-ipc-signal-register-handler.png](../Images/Kernel/ipc-signal-register-handler.png)

#### send a signal
```C++
/* kill->kill_something_info->kill_pid_info->group_send_sig_info->do_send_sig_info
 * tkill->do_tkill->do_send_specific->do_send_sig_info
 *
 * tgkill->do_tkill->do_send_specific->do_send_sig_info
 *
 * rt_sigqueueinfo->do_rt_sigqueueinfo->kill_proc_info->kill_pid_info
 * ->group_send_sig_info->do_send_sig_info */

SYSCALL_DEFINE2(kill, pid_t, pid, int, sig)
{
  struct siginfo info;

  info.si_signo = sig;
  info.si_errno = 0;
  info.si_code = SI_USER;
  info.si_pid = task_tgid_vnr(current);
  info.si_uid = from_kuid_munged(current_user_ns(), current_uid());

  return kill_something_info(sig, &info, pid);
}

static int __send_signal(int sig, struct siginfo *info, struct task_struct *t,
      int group, int from_ancestor_ns)
{
  struct sigpending *pending;
  struct sigqueue *q;
  int override_rlimit;
  int ret = 0, result;
  pending = group ? &t->signal->shared_pending : &t->pending;
  if (legacy_queue(pending, sig))
    goto ret;

  if (sig < SIGRTMIN)
    override_rlimit = (is_si_special(info) || info->si_code >= 0);
  else
    override_rlimit = 0;

  q = __sigqueue_alloc(sig, t, GFP_ATOMIC | __GFP_NOTRACK_FALSE_POSITIVE,
    override_rlimit);
  if (q) {
    list_add_tail(&q->list, &pending->list);
    switch ((unsigned long) info) {
    case (unsigned long) SEND_SIG_NOINFO:
      q->info.si_signo = sig;
      q->info.si_errno = 0;
      q->info.si_code = SI_USER;
      q->info.si_pid = task_tgid_nr_ns(current,
              task_active_pid_ns(t));
      q->info.si_uid = from_kuid_munged(current_user_ns(), current_uid());
      break;
    case (unsigned long) SEND_SIG_PRIV:
      q->info.si_signo = sig;
      q->info.si_errno = 0;
      q->info.si_code = SI_KERNEL;
      q->info.si_pid = 0;
      q->info.si_uid = 0;
      break;
    default:
      copy_siginfo(&q->info, info);
      if (from_ancestor_ns)
        q->info.si_pid = 0;
      break;
    }

    userns_fixup_signal_uid(&q->info, t);
  }

out_set:
  signalfd_notify(t, sig);
  sigaddset(&pending->signal, sig);
  complete_signal(sig, t, group);
ret:
  return ret;
}

static void complete_signal(int sig, struct task_struct *p, int group)
{
  struct signal_struct *signal = p->signal;
  struct task_struct *t;

  /* Now find a thread we can wake up to take the signal off the queue.
   *
   * If the main thread wants the signal, it gets first crack.
   * Probably the least surprising to the average bear. */
  if (wants_signal(sig, p))
    t = p;
  else if (!group || thread_group_empty(p))
    /* There is just one thread and it does not need to be woken.
     * It will dequeue unblocked signals before it runs again. */
    return;
  else {
    /* Otherwise try to find a suitable thread. */
    t = signal->curr_target;
    while (!wants_signal(sig, t)) {
      t = next_thread(t);
      if (t == signal->curr_target)
        return;
    }
    signal->curr_target = t;
  }

  /* The signal is already in the shared-pending queue.
   * Tell the chosen thread to wake up and dequeue it */
  signal_wake_up(t, sig == SIGKILL);
  return;
}

void signal_wake_up_state(struct task_struct *t, unsigned int state)
{
  set_tsk_thread_flag(t, TIF_SIGPENDING);

  if (!wake_up_state(t, state | TASK_INTERRUPTIBLE))
    kick_process(t);
}

int wake_up_state(struct task_struct *p, unsigned int state)
{
  return try_to_wake_up(p, state, 0);
}
```

#### handle signal
```C++
static void exit_to_usermode_loop(struct pt_regs *regs, u32 cached_flags)
{
  while (true) {
    if (cached_flags & _TIF_NEED_RESCHED)
      schedule();

    /* deal with pending signal delivery */
    if (cached_flags & _TIF_SIGPENDING)
      do_signal(regs);

    if (!(cached_flags & EXIT_TO_USERMODE_LOOP_FLAGS))
      break;
  }
}

void do_signal(struct pt_regs *regs)
{
  struct ksignal ksig;

  if (get_signal(&ksig)) {
    /* Whee! Actually deliver the signal.  */
    handle_signal(&ksig, regs);
    return;
  }

  /* Did we come from a system call? */
  if (syscall_get_nr(current, regs) >= 0) {
    /* Restart the system call - no handlers present */
    switch (syscall_get_error(current, regs)) {
    case -ERESTARTNOHAND:
    case -ERESTARTSYS:
    case -ERESTARTNOINTR:
      regs->ax = regs->orig_ax;
      regs->ip -= 2;
      break;

    case -ERESTART_RESTARTBLOCK:
      regs->ax = get_nr_restart_syscall(regs);
      regs->ip -= 2;
      break;
    }
  }
  restore_saved_sigmask();
}

static void
handle_signal(struct ksignal *ksig, struct pt_regs *regs)
{
  bool stepping, failed;

  /* Are we from a system call? */
  if (syscall_get_nr(current, regs) >= 0) {
    /* If so, check system call restarting.. */
    switch (syscall_get_error(current, regs)) {
    case -ERESTART_RESTARTBLOCK:
    case -ERESTARTNOHAND:
      regs->ax = -EINTR;
      break;
    case -ERESTARTSYS:
      if (!(ksig->ka.sa.sa_flags & SA_RESTART)) {
        regs->ax = -EINTR;
        break;
      }
    /* fallthrough */
    case -ERESTARTNOINTR:
      regs->ax = regs->orig_ax;
      regs->ip -= 2;
      break;
    }
  }
  failed = (setup_rt_frame(ksig, regs) < 0);

  signal_setup_done(failed, ksig, stepping);
}

static int __setup_rt_frame(int sig, struct ksignal *ksig,
          sigset_t *set, struct pt_regs *regs)
{
  struct rt_sigframe __user *frame;
  void __user *fp = NULL;
  int err = 0;

  frame = get_sigframe(&ksig->ka, regs, sizeof(struct rt_sigframe), &fp);

  put_user_try {

    /* Set up to return from userspace.  If provided, use a stub
       already in userspace.  */
    /* x86-64 should always use SA_RESTORER. */
    if (ksig->ka.sa.sa_flags & SA_RESTORER) {
      put_user_ex(ksig->ka.sa.sa_restorer, &frame->pretcode);
    }
  } put_user_catch(err);

  /* set return addr to sa_restor in frame->pretcode after finish sig handle */
  err |= setup_sigcontext(&frame->uc.uc_mcontext, fp, regs, set->sig[0]);
  err |= __copy_to_user(&frame->uc.uc_sigmask, set, sizeof(*set));

  /* Set up registers for signal handler */
  regs->di = sig;
  /* In case the signal handler was declared without prototypes */
  regs->ax = 0;

  regs->si = (unsigned long)&frame->info;
  regs->dx = (unsigned long)&frame->uc;
  regs->ip = (unsigned long) ksig->ka.sa.sa_handler;

  regs->sp = (unsigned long)frame;
  regs->cs = __USER_CS;

  return 0;
}

RESTORE (restore_rt, __NR_rt_sigreturn)
#define RESTORE(name, syscall) RESTORE2 (name, syscall)
# define RESTORE2(name, syscall) \
asm                                     \
  (                                     \
   ".LSTART_" #name ":\n"               \
   "    .type __" #name ",@function\n"  \
   "__" #name ":\n"                     \
   "    movq $" #syscall ", %rax\n"     \
   "    syscall\n"                      \


asmlinkage long sys_rt_sigreturn(void)
{
  struct pt_regs *regs = current_pt_regs();
  struct rt_sigframe __user *frame;
  sigset_t set;
  unsigned long uc_flags;

  frame = (struct rt_sigframe __user *)(regs->sp - sizeof(long));
  if (__copy_from_user(&set, &frame->uc.uc_sigmask, sizeof(set)))
    goto badframe;
  if (__get_user(uc_flags, &frame->uc.uc_flags))
    goto badframe;

  set_current_blocked(&set);

  if (restore_sigcontext(regs, &frame->uc.uc_mcontext, uc_flags))
    goto badframe;

  return regs->ax;
}
```
![linux-sig-handle.png](../Images/Kernel/sig-handle.png)

### sem, shm, msg
```C++
struct ipc_namespace {
  struct ipc_ids  ids[3];
}

#define IPC_SEM_IDS  0
#define IPC_MSG_IDS  1
#define IPC_SHM_IDS  2

#define sem_ids(ns)  ((ns)->ids[IPC_SEM_IDS])
#define msg_ids(ns)  ((ns)->ids[IPC_MSG_IDS])
#define shm_ids(ns)  ((ns)->ids[IPC_SHM_IDS])


struct ipc_ids {
  int in_use;
  unsigned short seq;
  struct rw_semaphore rwsem;
  struct idr ipcs_idr;
  int next_id;
};

struct idr {
  struct radix_tree_root  idr_rt;
  unsigned int    idr_next;
};
```
![ipc_ids](../Images/Kernel/ipc-ipc_ids.png)
```C++
struct kern_ipc_perm *ipc_obtain_object_idr(struct ipc_ids *ids, int id)
{
  struct kern_ipc_perm *out;
  int lid = ipcid_to_idx(id);
  out = idr_find(&ids->ipcs_idr, lid);
  return out;
}

static inline struct sem_array *sem_obtain_object(struct ipc_namespace *ns, int id)
{
  struct kern_ipc_perm *ipcp = ipc_obtain_object_idr(&sem_ids(ns), id);
  return container_of(ipcp, struct sem_array, sem_perm);
}

static inline struct msg_queue *msq_obtain_object(struct ipc_namespace *ns, int id)
{
  struct kern_ipc_perm *ipcp = ipc_obtain_object_idr(&msg_ids(ns), id);
  return container_of(ipcp, struct msg_queue, q_perm);
}

static inline struct shmid_kernel *shm_obtain_object(struct ipc_namespace *ns, int id)
{
  struct kern_ipc_perm *ipcp = ipc_obtain_object_idr(&shm_ids(ns), id);
  return container_of(ipcp, struct shmid_kernel, shm_perm);
}

struct kern_ipc_perm {
  spinlock_t  lock;
  bool      deleted;
  int       id;
  key_t     key;
  kuid_t    uid, gid, cuid, cgid;
  umode_t    mode;
  unsigned long  seq;
  void    *security;

  struct rhash_head khtnode;
  struct rcu_head rcu;
  refcount_t refcount;
};

struct sem_array {
  struct kern_ipc_perm  sem_perm;  /* permissions .. see ipc.h */
  time_t      sem_ctime;  /* create/last semctl() time */
  struct list_head  pending_alter;  /* pending operations */
                            /* that alter the array */
  struct list_head  pending_const;  /* pending complex operations */
            /* that do not alter semvals */
  struct list_head  list_id;  /* undo requests on this array */
  int      sem_nsems;  /* no. of semaphores in array */
  int      complex_count;  /* pending complex operations */
  unsigned int    use_global_lock;/* >0: global lock required */

  struct sem    sems[];
} __randomize_layout;

struct shmid_kernel /* private to the kernel */
{
  struct kern_ipc_perm  shm_perm;
  struct file            *shm_file;
  unsigned long         shm_nattch;
  unsigned long         shm_segsz;
  time_t                shm_atim, shm_dtim, shm_ctim;
  pid_t                 shm_cprid, shm_lprid;
  struct user_struct    *mlock_user;

  /* The task created the shm object.  NULL if the task is dead. */
  struct task_struct    *shm_creator;
  struct list_head      shm_clist;  /* list by creator */
} __randomize_layout;

struct msg_queue {
  struct kern_ipc_perm q_perm;
  time_t q_stime;      /* last msgsnd time */
  time_t q_rtime;      /* last msgrcv time */
  time_t q_ctime;      /* last change time */
  unsigned long q_cbytes;    /* current number of bytes on queue */
  unsigned long q_qnum;    /* number of messages in queue */
  unsigned long q_qbytes;    /* max number of bytes on queue */
  pid_t q_lspid;      /* pid of last msgsnd */
  pid_t q_lrpid;      /* last receive pid */

  struct list_head q_messages;
  struct list_head q_receivers;
  struct list_head q_senders;
} __randomize_layout;
```

#### shmget
```C++
SYSCALL_DEFINE3(shmget, key_t, key, size_t, size, int, shmflg)
{
  struct ipc_namespace *ns;
  static const struct ipc_ops shm_ops = {
    .getnew = newseg,
    .associate = shm_security,
    .more_checks = shm_more_checks,
  };

  struct ipc_params shm_params;
  ns = current->nsproxy->ipc_ns;
  shm_params.key = key;
  shm_params.flg = shmflg;
  shm_params.u.size = size;

  return ipcget(ns, &shm_ids(ns), &shm_ops, &shm_params);
}


int ipcget(struct ipc_namespace *ns, struct ipc_ids *ids,
    const struct ipc_ops *ops, struct ipc_params *params)
{
  if (params->key == IPC_PRIVATE)
    return ipcget_new(ns, ids, ops, params);
  else
    return ipcget_public(ns, ids, ops, params);
}

static int ipcget_public(struct ipc_namespace *ns, struct ipc_ids *ids,
    const struct ipc_ops *ops, struct ipc_params *params)
{
  struct kern_ipc_perm *ipcp;
  int flg = params->flg;
  int err;
  ipcp = ipc_findkey(ids, params->key);
  if (ipcp == NULL) {
    if (!(flg & IPC_CREAT))
      err = -ENOENT;
    else
      err = ops->getnew(ns, params);
  } else {
    if (flg & IPC_CREAT && flg & IPC_EXCL)
      err = -EEXIST;
    else {
      err = 0;
      if (ops->more_checks)
        err = ops->more_checks(ipcp, params);
    }
  }
  return err;
}

static int newseg(struct ipc_namespace *ns, struct ipc_params *params)
{
  key_t key = params->key;
  int shmflg = params->flg;
  size_t size = params->u.size;
  int error;
  struct shmid_kernel *shp;
  size_t numpages = (size + PAGE_SIZE - 1) >> PAGE_SHIFT;
  struct file *file;
  char name[13];
  vm_flags_t acctflag = 0;

  shp = kvmalloc(sizeof(*shp), GFP_KERNEL);
  shp->shm_perm.key = key;
  shp->shm_perm.mode = (shmflg & S_IRWXUGO);
  shp->mlock_user = NULL;
  shp->shm_perm.security = NULL;
  shp->shm_cprid = task_tgid_vnr(current);
  shp->shm_lprid = 0;
  shp->shm_atim = shp->shm_dtim = 0;
  shp->shm_ctim = get_seconds();
  shp->shm_segsz = size;
  shp->shm_nattch = 0;
  shp->shm_creator = current;

  file = shmem_kernel_file_setup(name, size, acctflag);
  file_inode(file)->i_ino = shp->shm_perm.id;
  shp->shm_file = file;

  error = ipc_addid(&shm_ids(ns), &shp->shm_perm, ns->shm_ctlmni);
  list_add(&shp->shm_clist, &current->sysvshm.shm_clist);

  ns->shm_tot += numpages;
  error = shp->shm_perm.id;

  return error;
}

int __init shmem_init(void)
{
  int error;
  error = shmem_init_inodecache(); // shmem means shmem fs
  error = register_filesystem(&shmem_fs_type);
  shm_mnt = kern_mount(&shmem_fs_type);
  return 0;
}

static struct file_system_type shmem_fs_type = {
  .owner    = THIS_MODULE,
  .name    = "tmpfs",
  .mount    = shmem_mount,
  .kill_sb  = kill_litter_super,
  .fs_flags  = FS_USERNS_MOUNT,
};

struct file *shmem_kernel_file_setup(const char *name, loff_t size, unsigned long flags)
{
  return __shmem_file_setup(shm_mnt, name, size, flags, S_PRIVATE);
}

static struct file *__shmem_file_setup(
  struct vfsmount *mnt, const char *name, loff_t size,
  unsigned long flags, unsigned int i_flags)
{
  struct inode *inode;
  struct file *res;

  inode = shmem_get_inode(mnt->mnt_sb, NULL, S_IFREG | S_IRWXUGO, 0,
        flags);
  if (unlikely(!inode)) {
    shmem_unacct_size(flags, size);
    return ERR_PTR(-ENOSPC);
  }

  inode->i_flags |= i_flags;
  inode->i_size = size;
  clear_nlink(inode);  /* It is unlinked */

  res = ERR_PTR(ramfs_nommu_expand_for_mapping(inode, size));
  if (!IS_ERR(res))
    res = alloc_file_pseudo(inode, mnt, name, O_RDWR,
        &shmem_file_operations);
  if (IS_ERR(res))
    iput(inode);

  return res;
}

struct file *alloc_file_pseudo(
  struct inode *inode, struct vfsmount *mnt,
  const char *name, int flags,
  const struct file_operations *fops)
{
  static const struct dentry_operations anon_ops = {
    .d_dname = simple_dname
  };
  struct qstr this = QSTR_INIT(name, strlen(name));
  struct path path;
  struct file *file;

  path.dentry = d_alloc_pseudo(mnt->mnt_sb, &this);

  if (!mnt->mnt_sb->s_d_op)
    d_set_d_op(path.dentry, &anon_ops);

  path.mnt = mntget(mnt);
  d_instantiate(path.dentry, inode);

  file = alloc_file(&path, flags, fops);
  if (IS_ERR(file)) {
    ihold(inode);
    path_put(&path);
  }
  return file;
}

static struct file *alloc_file(
  const struct path *path, int flags,
  const struct file_operations *fop)
{
  struct file *file = alloc_empty_file(flags, current_cred());
  file->f_path = *path;
  file->f_inode = path->dentry->d_inode;
  file->f_mapping = path->dentry->d_inode->i_mapping;
  file->f_wb_err = filemap_sample_wb_err(file->f_mapping);

  if ((file->f_mode & FMODE_READ)
    && likely(fop->read || fop->read_iter))
    file->f_mode |= FMODE_CAN_READ;
  if ((file->f_mode & FMODE_WRITE)
    && likely(fop->write || fop->write_iter))
    file->f_mode |= FMODE_CAN_WRITE;

  file->f_mode |= FMODE_OPENED;
  file->f_op = fop;
  if ((file->f_mode & (FMODE_READ | FMODE_WRITE)) == FMODE_READ)
    i_readcount_inc(path->dentry->d_inode);

  return file;
}


int ramfs_nommu_expand_for_mapping(struct inode *inode, size_t newsize)
{
  unsigned long npages, xpages, loop;
  struct page *pages;
  unsigned order;
  void *data;
  int ret;
  gfp_t gfp = mapping_gfp_mask(inode->i_mapping);

  /* make various checks */
  order = get_order(newsize);
  if (unlikely(order >= MAX_ORDER))
    return -EFBIG;

  ret = inode_newsize_ok(inode, newsize);
  if (ret)
    return ret;

  i_size_write(inode, newsize);

  pages = alloc_pages(gfp, order);
  if (!pages)
    return -ENOMEM;

  xpages = 1UL << order;
  npages = (newsize + PAGE_SIZE - 1) >> PAGE_SHIFT;

  split_page(pages, order);

  for (loop = npages; loop < xpages; loop++)
    __free_page(pages + loop);

  /* clear the memory we allocated */
  newsize = PAGE_SIZE * npages;
  // get virtual address of a page from `page_address_htable`
  data = page_address(pages);
  memset(data, 0, newsize);

  /* attach all the pages to the inode's address space */
  for (loop = 0; loop < npages; loop++) {
    struct page *page = pages + loop;

    ret = add_to_page_cache_lru(page, inode->i_mapping, loop,
          gfp);
    if (ret < 0)
      goto add_error;

    /* prevent the page from being discarded on memory pressure */
    SetPageDirty(page);
    SetPageUptodate(page);

    unlock_page(page);
    put_page(page);
  }

  return 0;

add_error:
  while (loop < npages)
    __free_page(pages + loop++);
  return ret;
}

static const struct file_operations shmem_file_operations = {
  .mmap               = shmem_mmap,
  .get_unmapped_area  = shmem_get_unmapped_area,
  .llseek             = shmem_file_llseek,
  .read_iter          = shmem_file_read_iter,
  .write_iter         = generic_file_write_iter,
  .fsync              = noop_fsync,
  .splice_read        = generic_file_splice_read,
  .splice_write       = iter_file_splice_write,
  .fallocate          = shmem_fallocate,
};
```

#### shmat
```C++
SYSCALL_DEFINE3(shmat, int, shmid, char __user *, shmaddr, int, shmflg)
{
  unsigned long ret;
  long err;
  err = do_shmat(shmid, shmaddr, shmflg, &ret, SHMLBA);
  force_successful_syscall_return();
  return (long)ret;
}

long do_shmat(int shmid, char __user *shmaddr, int shmflg,
        ulong *raddr, unsigned long shmlba)
{
  struct shmid_kernel *shp;
  unsigned long addr = (unsigned long)shmaddr;
  unsigned long size;
  struct file *file;
  int    err;
  unsigned long flags = MAP_SHARED;
  unsigned long prot;
  int acc_mode;
  struct ipc_namespace *ns;
  struct shm_file_data *sfd;
  struct path path;
  fmode_t f_mode;
  unsigned long populate = 0;

  prot = PROT_READ | PROT_WRITE;
  acc_mode = S_IRUGO | S_IWUGO;
  f_mode = FMODE_READ | FMODE_WRITE;

  ns = current->nsproxy->ipc_ns;
  shp = shm_obtain_object_check(ns, shmid);

  path = shp->shm_file->f_path;
  path_get(&path);
  shp->shm_nattch++;
  size = i_size_read(d_inode(path.dentry));

  sfd = kzalloc(sizeof(*sfd), GFP_KERNEL);
  sfd->id = shp->shm_perm.id;
  sfd->ns = get_ipc_ns(ns);
  sfd->file = shp->shm_file;
  sfd->vm_ops = NULL;

  file = alloc_file(&path, f_mode,
        is_file_hugepages(shp->shm_file)
          ? &shm_file_operations_huge
          : &shm_file_operations);
  file->private_data = sfd;
  file->f_mapping = shp->shm_file->f_mapping;

  addr = do_mmap_pgoff(file, addr, size, prot, flags, 0, &populate, NULL);
  *raddr = addr;
  err = 0;

  return err;
}

static const struct file_operations shm_file_operations = {
  .mmap       = shm_mmap,
  .fsync      = shm_fsync,
  .release    = shm_release,
  .get_unmapped_area  = shm_get_unmapped_area,
  .llseek     = noop_llseek,
  .fallocate  = shm_fallocate,
};

/* do_mmap_pgoff -> do_mmap -> mmap_region -> call_map -> file.f_op.mmap */
static int shm_mmap(struct file *file, struct vm_area_struct *vma)
{
  struct shm_file_data *sfd = shm_file_data(file);
  int ret;
  ret = __shm_open(vma);
  ret = call_mmap(sfd->file, vma); /* shmem_mmap */

  sfd->vm_ops = vma->vm_ops; /* shmem_vm_ops */
  vma->vm_ops = &shm_vm_ops;

  return 0;
}

int call_mmap(struct file *file, struct vm_area_struct *vma)
{
  return file->f_op->mmap(file, vma);
}

struct shm_file_data {
  int                     id;
  struct ipc_namespace    *ns;
  struct file              *file;
  const struct vm_operations_struct *vm_ops;
};

static const struct vm_operations_struct shm_vm_ops = {
  .open   = shm_open,  /* callback for a new vm-area open */
  .close  = shm_close,  /* callback for when the vm-area is released */
  .fault  = shm_fault,
};

static const struct file_operations shmem_file_operations = {
  .mmap               = shmem_mmap,
  .get_unmapped_area  = shmem_get_unmapped_area,
  .llseek             = shmem_file_llseek,
};

static int shmem_mmap(struct file *file, struct vm_area_struct *vma)
{
  file_accessed(file);
  vma->vm_ops = &shmem_vm_ops;
  return 0;
}

static const struct vm_operations_struct shmem_vm_ops = {
  .fault      = shmem_fault,
  .map_pages  = filemap_map_pages,
};
```

#### shm_fault
```C++
static int shm_fault(struct vm_fault *vmf)
{
  struct file *file = vmf->vma->vm_file;
  struct shm_file_data *sfd = shm_file_data(file);
  return sfd->vm_ops->fault(vmf);
}

static int shmem_fault(struct vm_fault *vmf)
{
  struct vm_area_struct *vma = vmf->vma;
  struct inode *inode = file_inode(vma->vm_file);
  gfp_t gfp = mapping_gfp_mask(inode->i_mapping);

  error = shmem_getpage_gfp(inode, vmf->pgoff, &vmf->page, sgp,
          gfp, vma, vmf, &ret);
}

static int shmem_getpage_gfp(struct inode *inode, pgoff_t index,
  struct page **pagep, enum sgp_type sgp, gfp_t gfp,
  struct vm_area_struct *vma, struct vm_fault *vmf, int *fault_type)
{
  page = shmem_alloc_and_acct_page(gfp, info, sbinfo,
        index, false);
}
```

```C++
shmget();
  ipcget();
    ipcget_private();
    ipcget_public();
      newseg();
        shp = kvmalloc(sizeof(*shp), GFP_KERNEL);
        file = shmem_kernel_file_setup(name, size, acctflag);
          __shmem_file_setup();
            shmem_get_inode();
            alloc_file_pseudo();
        shp->shm_file = file;
        ipc_addid(&shm_ids(ns), &shp->shm_perm, ns->shm_ctlmni);

shmat();
  do_shmat();
    shm_obtain_object_check();
    sfd = kzalloc(sizeof(*sfd), GFP_KERNEL);
    file = alloc_file();
    do_mmap_pgoff();
      shm_mmap();
        shmem_mmap();

shm_fault();
  shmem_fault();
    shmem_getpage_gfp();
      shmem_alloc_and_acct_page();
```

![linux-ipc-shm.png](../Images/Kernel/ipc-shm.png)

#### semget
```C++
SYSCALL_DEFINE3(semget, key_t, key, int, nsems, int, semflg)
{
  struct ipc_namespace *ns;
  static const struct ipc_ops sem_ops = {
    .getnew = newary,
    .associate = sem_security,
    .more_checks = sem_more_checks,
  };

  struct ipc_params sem_params;
  ns = current->nsproxy->ipc_ns;
  sem_params.key = key;
  sem_params.flg = semflg;
  sem_params.u.nsems = nsems;

  return ipcget(ns, &sem_ids(ns), &sem_ops, &sem_params);
}

static int newary(struct ipc_namespace *ns, struct ipc_params *params)
{
  int retval;
  struct sem_array *sma;
  key_t key = params->key;
  int nsems = params->u.nsems;
  int semflg = params->flg;
  int i;

  sma = sem_alloc(nsems);
  sma->sem_perm.mode = (semflg & S_IRWXUGO);
  sma->sem_perm.key = key;
  sma->sem_perm.security = NULL;

  for (i = 0; i < nsems; i++) {
    INIT_LIST_HEAD(&sma->sems[i].pending_alter);
    INIT_LIST_HEAD(&sma->sems[i].pending_const);
    spin_lock_init(&sma->sems[i].lock);
  }

  sma->complex_count = 0;
  sma->use_global_lock = USE_GLOBAL_LOCK_HYSTERESIS;
  INIT_LIST_HEAD(&sma->pending_alter);
  INIT_LIST_HEAD(&sma->pending_const);
  INIT_LIST_HEAD(&sma->list_id);
  sma->sem_nsems = nsems;
  sma->sem_ctime = get_seconds();

  retval = ipc_addid(&sem_ids(ns), &sma->sem_perm, ns->sc_semmni);
  ns->used_sems += nsems;

  return sma->sem_perm.id;
}

struct sem {
  int               semval;
  int               sempid;
  spinlock_t        lock;
  struct list_head  pending_alter;
  struct list_head  pending_const;
  time_t            sem_otime;  /* candidate for sem_otime */
};
```

#### semctl
```C++
SYSCALL_DEFINE4(semctl, int, semid, int, semnum, int, cmd, unsigned long, arg)
{
  int version;
  struct ipc_namespace *ns;
  void __user *p = (void __user *)arg;
  ns = current->nsproxy->ipc_ns;
  switch (cmd) {
    case IPC_INFO:
    case SEM_INFO:
    case IPC_STAT:
    case SEM_STAT:
      return semctl_nolock(ns, semid, cmd, version, p);

    case GETALL:
    case GETVAL:
    case GETPID:
    case GETNCNT:
    case GETZCNT:
    case SETALL:
      return semctl_main(ns, semid, semnum, cmd, p);

    case SETVAL:
      return semctl_setval(ns, semid, semnum, arg);

    case IPC_RMID:
    case IPC_SET:
      return semctl_down(ns, semid, cmd, version, p);

    default:
      return -EINVAL;
  }
}

static int semctl_main(struct ipc_namespace *ns, int semid, int semnum,
    int cmd, void __user *p)
{
  struct sem_array *sma;
  struct sem *curr;
  int err, nsems;
  ushort fast_sem_io[SEMMSL_FAST];
  ushort *sem_io = fast_sem_io;

  DEFINE_WAKE_Q(wake_q);

  sma = sem_obtain_object_check(ns, semid);
  nsems = sma->sem_nsems;

  switch (cmd) {
    case SETALL:
    {
      int i;
      struct sem_undo *un;

      copy_from_user(sem_io, p, nsems*sizeof(ushort));

      for (i = 0; i < nsems; i++) {
        sma->sems[i].semval = sem_io[i];
        sma->sems[i].sempid = task_tgid_vnr(current);
      }

      sma->sem_ctime = get_seconds();
      /* maybe some queued-up processes were waiting for this */
      do_smart_update(sma, NULL, 0, 0, &wake_q);
      err = 0;
      goto out_unlock;
    }
  }
  wake_up_q(&wake_q);
}

static int semctl_setval(struct ipc_namespace *ns, int semid, int semnum,
    unsigned long arg)
{
  struct sem_undo *un;
  struct sem_array *sma;
  struct sem *curr;
  int err, val;

  DEFINE_WAKE_Q(wake_q);

  sma = sem_obtain_object_check(ns, semid);

  curr = &sma->sems[semnum];
  curr->semval = val;
  curr->sempid = task_tgid_vnr(current);
  sma->sem_ctime = get_seconds();
  /* maybe some queued-up processes were waiting for this */
  do_smart_update(sma, NULL, 0, 0, &wake_q);

  wake_up_q(&wake_q);
  return 0;
}
```

#### semop
```C++
SYSCALL_DEFINE3(semop, int, semid, struct sembuf __user *, tsops,
    unsigned, nsops)
{
  return sys_semtimedop(semid, tsops, nsops, NULL);
}

SYSCALL_DEFINE4(semtimedop, int, semid, struct sembuf __user *, tsops,
    unsigned, nsops, const struct timespec __user *, timeout)
{
  int error = -EINVAL;
  struct sem_array *sma;
  struct sembuf fast_sops[SEMOPM_FAST];
  struct sembuf *sops = fast_sops, *sop;
  struct sem_undo *un;
  int max, locknum;
  bool undos = false, alter = false, dupsop = false;
  struct sem_queue queue;
  unsigned long dup = 0, jiffies_left = 0;
  struct ipc_namespace *ns;

  ns = current->nsproxy->ipc_ns;

  copy_from_user(sops, tsops, nsops * sizeof(*tsops));

  if (timeout) {
    struct timespec _timeout;
    copy_from_user(&_timeout, timeout, sizeof(*timeout));
    jiffies_left = timespec_to_jiffies(&_timeout);
  }

  /* On success, find_alloc_undo takes the rcu_read_lock */
  un = find_alloc_undo(ns, semid);
  sma = sem_obtain_object_check(ns, semid);

  locknum = sem_lock(sma, sops, nsops);

  queue.sops = sops;
  queue.nsops = nsops;
  queue.undo = un;
  queue.pid = task_tgid_vnr(current);
  queue.alter = alter;
  queue.dupsop = dupsop;

  error = perform_atomic_semop(sma, &queue);
  if (error == 0) { /* non-blocking succesfull path */
    DEFINE_WAKE_Q(wake_q);

    do_smart_update(sma, sops, nsops, 1, &wake_q);

    sem_unlock(sma, locknum);
    wake_up_q(&wake_q);
    goto out_free;
  }

  /* ops not finished, add the remaining to list */
  if (nsops == 1) {
    struct sem *curr;
    curr = &sma->sems[sops->sem_num];
    list_add_tail(&queue.list, &curr->pending_alter);
  } else {
    list_add_tail(&queue.list, &sma->pending_alter);
  }

  do {
    queue.status = -EINTR;
    queue.sleeper = current;

    __set_current_state(TASK_INTERRUPTIBLE);
    sem_unlock(sma, locknum);

    if (timeout)
      jiffies_left = schedule_timeout(jiffies_left);
    else
      schedule();

    locknum = sem_lock(sma, sops, nsops);

    error = READ_ONCE(queue.status);
    if (error != -EINTR)
      goto out_unlock_free;

    if (timeout && jiffies_left == 0)
      error = -EAGAIN;
  } while (error == -EINTR && !signal_pending(current)); /* spurious */

out_unlock_free:
  sem_unlock(sma, locknum);
  rcu_read_unlock();
out_free:
  if (sops != fast_sops)
    kvfree(sops);
  return error;
}

static int perform_atomic_semop(struct sem_array *sma, struct sem_queue *q)
{
  int result, sem_op, nsops;
  struct sembuf *sop;
  struct sem *curr;
  struct sembuf *sops;
  struct sem_undo *un;

  sops = q->sops;
  nsops = q->nsops;
  un = q->undo;

  for (sop = sops; sop < sops + nsops; sop++) {
    curr = &sma->sems[sop->sem_num];
    sem_op = sop->sem_op;
    result = curr->semval;

    result += sem_op;
    if (result < 0)
      goto would_block;

    if (sop->sem_flg & SEM_UNDO) {
      int undo = un->semadj[sop->sem_num] - sem_op;
    }
  }

  for (sop = sops; sop < sops + nsops; sop++) {
    curr = &sma->sems[sop->sem_num];
    sem_op = sop->sem_op;
    result = curr->semval;

    if (sop->sem_flg & SEM_UNDO) {
      int undo = un->semadj[sop->sem_num] - sem_op;
      un->semadj[sop->sem_num] = undo;
    }
    curr->semval += sem_op;
    curr->sempid = q->pid;
  }
  return 0;

would_block:
  q->blocking = sop;
  return sop->sem_flg & IPC_NOWAIT ? -EAGAIN : 1;
}

static void do_smart_update(
  struct sem_array *sma, struct sembuf *sops, int nsops,
  int otime, struct wake_q_head *wake_q)
{
  int i;
  otime |= do_smart_wakeup_zero(sma, sops, nsops, wake_q);
  if (!list_empty(&sma->pending_alter)) {
    otime |= update_queue(sma, -1, wake_q);
  } else {
    if (!sops) {
      for (i = 0; i < sma->sem_nsems; i++)
        otime |= update_queue(sma, i, wake_q);
    } else {
      for (i = 0; i < nsops; i++) {
        if (sops[i].sem_op > 0) {
          otime |= update_queue(sma, sops[i].sem_num, wake_q);
        }
      }
    }
  }
  if (otime)
    set_semotime(sma, sops);
}

static int update_queue(struct sem_array *sma, int semnum, struct wake_q_head *wake_q)
{
  struct sem_queue *q, *tmp;
  struct list_head *pending_list;
  int semop_completed = 0;

  if (semnum == -1)
    pending_list = &sma->pending_alter;
  else
    pending_list = &sma->sems[semnum].pending_alter;

again:
  list_for_each_entry_safe(q, tmp, pending_list, list) {
    int error, restart;

    error = perform_atomic_semop(sma, q);

    /* Does q->sleeper still need to sleep? */
    if (error > 0)
      continue;

    unlink_queue(sma, q);

    wake_up_sem_queue_prepare(q, error, wake_q);
  }
  return semop_completed;
}

static inline void wake_up_sem_queue_prepare(
  struct sem_queue *q, int error,
  struct wake_q_head *wake_q)
{
  wake_q_add(wake_q, q->sleeper);
}

void wake_up_q(struct wake_q_head *head)
{
  struct wake_q_node *node = head->first;

  while (node != WAKE_Q_TAIL) {
    struct task_struct *task;

    task = container_of(node, struct task_struct, wake_q);

    node = node->next;
    task->wake_q.next = NULL;

    wake_up_process(task);
    put_task_struct(task);
  }
}

struct sem_queue {
  struct list_head    list;   /* queue of pending operations */
  struct task_struct  *sleeper; /* this process */
  struct sem_undo     *undo;   /* undo structure */
  int                 pid;
  int                 status;
  struct sembuf       *sops;
  struct sembuf       *blocking;
  int                 nsops;
  bool                alter;
  bool                dupsop;
};

struct task_struct {
  struct sysv_sem sysvsem;
}

struct sysv_sem {
  struct sem_undo_list *undo_list;
};

struct sem_undo {
  struct list_head  list_proc;  /* per-process list: *
             * all undos from one process
             * rcu protected */
  struct rcu_head    rcu;    /* rcu struct for sem_undo */
  struct sem_undo_list  *ulp;    /* back ptr to sem_undo_list */
  struct list_head  list_id;  /* per semaphore array list:
             * all undos for one array */
  int      semid;    /* semaphore set identifier */
  short      *semadj;  /* array of adjustments */
            /* one per semaphore */
};

struct sem_undo_list {
  atomic_t    refcnt;
  spinlock_t    lock;
  struct list_head  list_proc;
};
```

![linux-ipc-sem-2.png](../Images/Kernel/ipc-sem-2.png)

```C++
semget();
  ipcget();
    newary();
      ipc_addid(&sem_ids(ns), &sma->sem_perm, ns->sc_semmni);

semctl();
  semctl_main();
    sem_obtain_object_check();
    copy_from_user();
    sma->sems[i].semval = sem_io[i];
    do_smart_update();
      do_smart_wakeup_zero();
      update_queue();
        perform_atomic_semop();
        wake_up_sem_queue_prepare();
          wake_q_add(wake_q, q->sleeper);
    wake_up_q();
      wake_up_process(task);
        try_to_wake_up(p, TASK_NORMAL, 0);

  semctl_setval();
    sem_obtain_object_check();
    curr->semval = val;
    do_smart_update();
    wake_up_q();

semop();
  sys_semtimedop();
    copy_from_user();
    sem_obtain_object_check();
    perform_atomic_semop();
    do_smart_update();
    wake_up_q();

    list_add_tail(&queue.list, &sma->pending_alter);
    schedule();
```
![linux-ipc-sem.png](../Images/Kernel/ipc-sem.png)

## Q:
1. How access shm by a vm address?

# Virtualization

# Containerization
![linux-container-vir-arch.png](../Images/Kernel/container-vir-arch.png)

### ns
```C++
# nsenter --target 58212 --mount --uts --ipc --net --pid -- env --ignore-environment -- /bin/bash

root@f604f0e34bc2:/# ip addr
1: lo: <LOOPBACK,UP,LOWER_UP> mtu 65536 qdisc noqueue state UNKNOWN group default qlen 1000
    link/loopback 00:00:00:00:00:00 brd 00:00:00:00:00:00
    inet 127.0.0.1/8 scope host lo
       valid_lft forever preferred_lft forever
23: eth0@if24: <BROADCAST,MULTICAST,UP,LOWER_UP> mtu 1500 qdisc noqueue state UP group default
    link/ether 02:42:ac:11:00:03 brd ff:ff:ff:ff:ff:ff
    inet 172.17.0.3/16 brd 172.17.255.255 scope global eth0
       valid_lft forever preferred_lft forever
```
```C++
unshare --mount --ipc --pid --net --mount-proc=/proc --fork /bin/bash
```

```C++
int clone(int (*fn)(void *), void *child_stack, int flags, void *arg);
/* CLONE_NEWUTS, CLONE_NEWUSER, CLONE_NEWNS, CLONE_NEWPID, CLONE_NEWIPC, CLONE_NEWNET */

/* nsenter, ip netns exec, docker exec */
int setns(int fd, int nstype);

int unshare(int flags);
```

```C++
struct task_struct {
  struct nsproxy  *nsproxy;
  struct css_set  *cgroups;
}

struct nsproxy {
  atomic_t count;
  struct uts_namespace    *uts_ns;
  struct ipc_namespace    *ipc_ns;
  struct mnt_namespace    *mnt_ns;
  struct pid_namespace    *pid_ns_for_children;
  struct net              *net_ns;
  struct cgroup_namespace *cgroup_ns;
};

struct nsproxy init_nsproxy = {
  .count      = ATOMIC_INIT(1),
  .uts_ns      = &init_uts_ns,
#if defined(CONFIG_POSIX_MQUEUE) || defined(CONFIG_SYSVIPC)
  .ipc_ns      = &init_ipc_ns,
#endif
  .mnt_ns      = NULL,
  .pid_ns_for_children  = &init_pid_ns,
#ifdef CONFIG_NET
  .net_ns      = &init_net,
#endif
#ifdef CONFIG_CGROUPS
  .cgroup_ns    = &init_cgroup_ns,
#endif
};

/* clone -> _do_fork -> copy_process -> copy_namespaces */
int copy_namespaces(unsigned long flags, struct task_struct *tsk)
{
  struct nsproxy *old_ns = tsk->nsproxy;
  struct user_namespace *user_ns = task_cred_xxx(tsk, user_ns);
  struct nsproxy *new_ns;

  if (likely(!(flags & (CLONE_NEWNS | CLONE_NEWUTS | CLONE_NEWIPC |
            CLONE_NEWPID | CLONE_NEWNET |
            CLONE_NEWCGROUP)))) {
    get_nsproxy(old_ns);
    return 0;
  }

  if (!ns_capable(user_ns, CAP_SYS_ADMIN))
    return -EPERM;

  new_ns = create_new_namespaces(flags, tsk, user_ns, tsk->fs);

  tsk->nsproxy = new_ns;
  return 0;
}

static struct nsproxy *create_new_namespaces(unsigned long flags,
  struct task_struct *tsk, struct user_namespace *user_ns,
  struct fs_struct *new_fs)
{
  struct nsproxy *new_nsp;

  new_nsp = create_nsproxy();
  new_nsp->mnt_ns = copy_mnt_ns(flags, tsk->nsproxy->mnt_ns, user_ns, new_fs);
  new_nsp->uts_ns = copy_utsname(flags, user_ns, tsk->nsproxy->uts_ns);
  new_nsp->ipc_ns = copy_ipcs(flags, user_ns, tsk->nsproxy->ipc_ns);
  new_nsp->pid_ns_for_children =
    copy_pid_ns(flags, user_ns, tsk->nsproxy->pid_ns_for_children);
  new_nsp->cgroup_ns = copy_cgroup_ns(flags, user_ns,
              tsk->nsproxy->cgroup_ns);
  new_nsp->net_ns = copy_net_ns(flags, user_ns, tsk->nsproxy->net_ns);

  return new_nsp;
}

struct pid_namespace *copy_pid_ns(unsigned long flags,
  struct user_namespace *user_ns, struct pid_namespace *old_ns)
{
  if (!(flags & CLONE_NEWPID))
    return get_pid_ns(old_ns);
  if (task_active_pid_ns(current) != old_ns)
    return ERR_PTR(-EINVAL);
  return create_pid_namespace(user_ns, old_ns);
}

struct net *copy_net_ns(unsigned long flags,
      struct user_namespace *user_ns, struct net *old_net)
{
  struct ucounts *ucounts;
  struct net *net;
  int rv;

  if (!(flags & CLONE_NEWNET))
    return get_net(old_net);

  ucounts = inc_net_namespaces(user_ns);
  net = net_alloc();
  get_user_ns(user_ns);
  net->ucounts = ucounts;
  rv = setup_net(net, user_ns);

  return net;
}

static __net_init int setup_net(struct net *net, struct user_namespace *user_ns)
{
  /* Must be called with net_mutex held */
  const struct pernet_operations *ops, *saved_ops;
  LIST_HEAD(net_exit_list);

  atomic_set(&net->count, 1);
  refcount_set(&net->passive, 1);
  net->dev_base_seq = 1;
  net->user_ns = user_ns;
  idr_init(&net->netns_ids);
  spin_lock_init(&net->nsid_lock);

  list_for_each_entry(ops, &pernet_list, list) {
    error = ops_init(ops, net);
  }
}

register_pernet_device(&loopback_net_ops)

int register_pernet_device(struct pernet_operations *ops)
{
  int error;
  mutex_lock(&net_mutex);
  error = register_pernet_operations(&pernet_list, ops);
  if (!error && (first_device == &pernet_list))
    first_device = &ops->list;
  mutex_unlock(&net_mutex);
  return error;
}

struct pernet_operations __net_initdata loopback_net_ops = {
        .init = loopback_net_init,
};

static __net_init int loopback_net_init(struct net *net)
{
  struct net_device *dev;
  dev = alloc_netdev(0, "lo", NET_NAME_UNKNOWN, loopback_setup);

  dev_net_set(dev, net);
  err = register_netdev(dev);

  net->loopback_dev = dev;

  return 0;
}
```
![linux-container-namespace.png](../Images/Kernel/container-namespace.png)

### cgroup
cgrup subsystem:
  cpu, cpuacct, cpuset, memory, blkio, devices, net_cls(tc), freezer

```C++
docker run -d --cpu-shares 513 --cpus 2 --cpuset-cpus 1,3 --memory 1024M --memory-swap 1234M --memory-swappiness 7 -p 8081:80 testnginx:1

# docker ps
CONTAINER ID        IMAGE               COMMAND                  CREATED              STATUS              PORTS                  NAMES
3dc0601189dd        testnginx:1         "/bin/sh -c 'nginx -…"   About a minute ago   Up About a minute   0.0.0.0:8081->80/tcp   boring_cohen


# mount -t cgroup
cgroup on /sys/fs/cgroup/systemd type cgroup (rw,nosuid,nodev,noexec,relatime,xattr,release_agent=/usr/lib/systemd/systemd-cgroups-agent,name=systemd)
cgroup on /sys/fs/cgroup/net_cls,net_prio type cgroup (rw,nosuid,nodev,noexec,relatime,net_prio,net_cls)
cgroup on /sys/fs/cgroup/perf_event type cgroup (rw,nosuid,nodev,noexec,relatime,perf_event)
cgroup on /sys/fs/cgroup/devices type cgroup (rw,nosuid,nodev,noexec,relatime,devices)
cgroup on /sys/fs/cgroup/blkio type cgroup (rw,nosuid,nodev,noexec,relatime,blkio)
cgroup on /sys/fs/cgroup/cpu,cpuacct type cgroup (rw,nosuid,nodev,noexec,relatime,cpuacct,cpu)
cgroup on /sys/fs/cgroup/memory type cgroup (rw,nosuid,nodev,noexec,relatime,memory)
cgroup on /sys/fs/cgroup/cpuset type cgroup (rw,nosuid,nodev,noexec,relatime,cpuset)
cgroup on /sys/fs/cgroup/hugetlb type cgroup (rw,nosuid,nodev,noexec,relatime,hugetlb)
cgroup on /sys/fs/cgroup/freezer type cgroup (rw,nosuid,nodev,noexec,relatime,freezer)
cgroup on /sys/fs/cgroup/pids type cgroup (rw,nosuid,nodev,noexec,relatime,pids)
```
```C++
// sys/fs/cgroup
drwxr-xr-x 5 root root  0 May 30 17:00 blkio
lrwxrwxrwx 1 root root 11 May 30 17:00 cpu -> cpu,cpuacct
lrwxrwxrwx 1 root root 11 May 30 17:00 cpuacct -> cpu,cpuacct
drwxr-xr-x 5 root root  0 May 30 17:00 cpu,cpuacct
drwxr-xr-x 3 root root  0 May 30 17:00 cpuset
drwxr-xr-x 5 root root  0 May 30 17:00 devices
drwxr-xr-x 3 root root  0 May 30 17:00 freezer
drwxr-xr-x 3 root root  0 May 30 17:00 hugetlb
drwxr-xr-x 5 root root  0 May 30 17:00 memory
lrwxrwxrwx 1 root root 16 May 30 17:00 net_cls -> net_cls,net_prio
drwxr-xr-x 3 root root  0 May 30 17:00 net_cls,net_prio
lrwxrwxrwx 1 root root 16 May 30 17:00 net_prio -> net_cls,net_prio
drwxr-xr-x 3 root root  0 May 30 17:00 perf_event
drwxr-xr-x 5 root root  0 May 30 17:00 pids
drwxr-xr-x 5 root root  0 May 30 17:00 systemd

[cpu, cpuacct]# ls
cgroup.clone_children  cpu.cfs_period_us  notify_on_release
cgroup.event_control   cpu.cfs_quota_us   release_agent
cgroup.procs           cpu.rt_period_us   system.slice
cgroup.sane_behavior   cpu.rt_runtime_us  tasks
cpuacct.stat           cpu.shares         user.slice
cpuacct.usage          cpu.stat
cpuacct.usage_percpu   docker

[docker]# ls
cgroup.clone_children
cgroup.event_control
cgroup.procs
cpuacct.stat
cpuacct.usage
cpuacct.usage_percpu
cpu.cfs_period_us
cpu.cfs_quota_us
cpu.rt_period_us
cpu.rt_runtime_us
cpu.shares
cpu.stat
3dc0601189dd218898f31f9526a6cfae83913763a4da59f95ec789c6e030ecfd
notify_on_release
tasks

[3dc0601189dd218898f31f9526a6cfae83913763a4da59f95ec789c6e030ecfd]# ls
cgroup.clone_children  cpuacct.usage_percpu  cpu.shares
cgroup.event_control   cpu.cfs_period_us     cpu.stat
cgroup.procs           cpu.cfs_quota_us      notify_on_release
cpuacct.stat           cpu.rt_period_us      tasks
cpuacct.usage          cpu.rt_runtime_us

[3dc0601189dd218898f31f9526a6cfae83913763a4da59f95ec789c6e030ecfd]# cat tasks
39487
39520
39526
39527
39528
39529

[3dc0601189dd218898f31f9526a6cfae83913763a4da59f95ec789c6e030ecfd]# cat cpu.shares
513

[3dc0601189dd218898f31f9526a6cfae83913763a4da59f95ec789c6e030ecfd]# cat cpu.cfs_period_us
100000

[3dc0601189dd218898f31f9526a6cfae83913763a4da59f95ec789c6e030ecfd]# cat cpu.cfs_quota_us
200000

[3dc0601189dd218898f31f9526a6cfae83913763a4da59f95ec789c6e030ecfd]# cat cpuset.cpus
1,3

[root@deployer memory]# ls
cgroup.clone_children               memory.memsw.failcnt
cgroup.event_control                memory.memsw.limit_in_bytes
cgroup.procs                        memory.memsw.max_usage_in_bytes
cgroup.sane_behavior                memory.memsw.usage_in_bytes
docker                              memory.move_charge_at_immigrate
memory.failcnt                      memory.numa_stat
memory.force_empty                  memory.oom_control
memory.kmem.failcnt                 memory.pressure_level
memory.kmem.limit_in_bytes          memory.soft_limit_in_bytes
memory.kmem.max_usage_in_bytes      memory.stat
memory.kmem.slabinfo                memory.swappiness
memory.kmem.tcp.failcnt             memory.usage_in_bytes
memory.kmem.tcp.limit_in_bytes      memory.use_hierarchy
memory.kmem.tcp.max_usage_in_bytes  notify_on_release
memory.kmem.tcp.usage_in_bytes      release_agent
memory.kmem.usage_in_bytes          system.slice
memory.limit_in_bytes               tasks
memory.max_usage_in_bytes           user.slice
```
![linux-container-cgroup.png](../Images/Kernel/container-cgroup.png)

```C++
void __init start_kernel(void)
{
  cgroup_init();
}

int cgroup_init(void)
{
  struct cgroup_subsys *ss;
  int ssid;

  cgroup_init_cftypes(NULL, cgroup1_base_files);

  for_each_subsys(ss, ssid) {
    cgroup_init_subsys(ss, false);

    list_add_tail(&init_css_set.e_cset_node[ssid],
            &cgrp_dfl_root.cgrp.e_csets[ssid]);
  }

  sysfs_create_mount_point(fs_kobj, "cgroup"));
  register_filesystem(&cgroup_fs_type);
  register_filesystem(&cgroup2_fs_type);
}
```

#### cgroup_init_cftypes
```C++
/* set the cftype's kf_ops to cgroup_kf_ops */
static int cgroup_init_cftypes(struct cgroup_subsys *ss, struct cftype *cfts)
{
  struct cftype *cft;

  for (cft = cfts; cft->name[0] != '\0'; cft++) {
    struct kernfs_ops *kf_ops;
    if (cft->seq_start)
      kf_ops = &cgroup_kf_ops;
    else
      kf_ops = &cgroup_kf_single_ops;

    cft->kf_ops = kf_ops;
    cft->ss = ss;
  }
}

struct cftype cgroup1_base_files[] = {
  {
    .name = "cgroup.procs",
    .seq_start = cgroup_pidlist_start,
    .seq_next = cgroup_pidlist_next,
    .seq_stop = cgroup_pidlist_stop,
    .seq_show = cgroup_pidlist_show,
    .private = CGROUP_FILE_PROCS,
    .write = cgroup1_procs_write,
  },
  {
    .name = "cgroup.clone_children",
    .read_u64 = cgroup_clone_children_read,
    .write_u64 = cgroup_clone_children_write,
  },
  {
    .name = "cgroup.sane_behavior",
    .flags = CFTYPE_ONLY_ON_ROOT,
    .seq_show = cgroup_sane_behavior_show,
  },
  {
    .name = "tasks",
    .seq_start = cgroup_pidlist_start,
    .seq_next = cgroup_pidlist_next,
    .seq_stop = cgroup_pidlist_stop,
    .seq_show = cgroup_pidlist_show,
    .private = CGROUP_FILE_TASKS,
    .write = cgroup1_tasks_write,
  },
  {
    .name = "notify_on_release",
    .read_u64 = cgroup_read_notify_on_release,
    .write_u64 = cgroup_write_notify_on_release,
  },
  {
    .name = "release_agent",
    .flags = CFTYPE_ONLY_ON_ROOT,
    .seq_show = cgroup_release_agent_show,
    .write = cgroup_release_agent_write,
    .max_write_len = PATH_MAX - 1,
  },
  { }  /* terminate */
}

struct cftype {
  struct kernfs_ops *kf_ops;

  char name[MAX_CFTYPE_NAME];
  unsigned long private;

  size_t max_write_len;
  unsigned int flags;

  unsigned int file_offset;

  struct cgroup_subsys *ss;  /* NULL for cgroup core files */
  struct list_head node;     /* anchored at ss->cfts */

  int (*open)(struct kernfs_open_file *of);
  void (*release)(struct kernfs_open_file *of);
  u64 (*read_u64)(struct cgroup_subsys_state *css, struct cftype *cft);
  int (*seq_show)(struct seq_file *sf, void *v);
  ssize_t (*write)(struct kernfs_open_file *of,
      char *buf, size_t nbytes, loff_t off);
  __poll_t (*poll)(struct kernfs_open_file *of,
      struct poll_table_struct *pt);
};
```

#### cgroup_init_subsys
```C++
#define for_each_subsys(ss, ssid)          \
  for ((ssid) = 0; (ssid) < CGROUP_SUBSYS_COUNT &&    \
       (((ss) = cgroup_subsys[ssid]) || true); (ssid)++)

#define SUBSYS(_x) [_x ## _cgrp_id] = &_x ## _cgrp_subsys,
struct cgroup_subsys *cgroup_subsys[] = {
#include <linux/cgroup_subsys.h>
};
#undef SUBSYS

#if IS_ENABLED(CONFIG_CPUSETS)
SUBSYS(cpuset)
#endif

#if IS_ENABLED(CONFIG_CGROUP_SCHED)
SUBSYS(cpu)
#endif

#if IS_ENABLED(CONFIG_CGROUP_CPUACCT)
SUBSYS(cpuacct)
#endif

#if IS_ENABLED(CONFIG_MEMCG)
SUBSYS(memory)
#endif

struct cgroup_subsys {
  struct cgroup_subsys_state *(*css_alloc)(struct cgroup_subsys_state *parent_css);
  int (*css_online)(struct cgroup_subsys_state *css);
  void (*attach)(struct cgroup_taskset *tset);
  void (*fork)(struct task_struct *task);
  void (*exit)(struct task_struct *task);
  void (*release)(struct task_struct *task);
  void (*bind)(struct cgroup_subsys_state *root_css);

  bool early_init:1;
  bool implicit_on_dfl:1;
  bool threaded:1;
  bool broken_hierarchy:1;
  bool warned_broken_hierarchy:1;

  int id;
  const char *name;
  const char *legacy_name;

  struct cgroup_root *root;

  struct idr css_idr;

  struct list_head cfts;

  struct cftype *dfl_cftypes;  /* for the default hierarchy */
  struct cftype *legacy_cftypes;  /* for the legacy hierarchies */
  unsigned int depends_on;
};

struct cgroup_subsys cpuset_cgrp_subsys = {
  .css_alloc  = cpuset_css_alloc,
  .css_online  = cpuset_css_online,
  .css_offline  = cpuset_css_offline,
  .css_free  = cpuset_css_free,
  .can_attach  = cpuset_can_attach,
  .cancel_attach  = cpuset_cancel_attach,
  .attach    = cpuset_attach,
  .post_attach  = cpuset_post_attach,
  .bind    = cpuset_bind,
  .fork    = cpuset_fork,
  .legacy_cftypes  = files,
  .early_init  = true,
};

struct cgroup_subsys cpu_cgrp_subsys = {
  .css_alloc  = cpu_cgroup_css_alloc,
  .css_online  = cpu_cgroup_css_online,
  .css_released  = cpu_cgroup_css_released,
  .css_free  = cpu_cgroup_css_free,
  .fork    = cpu_cgroup_fork,
  .can_attach  = cpu_cgroup_can_attach,
  .attach    = cpu_cgroup_attach,
  .legacy_cftypes  = cpu_files,
  .early_init  = true,
};

struct cgroup_subsys memory_cgrp_subsys = {
  .css_alloc = mem_cgroup_css_alloc,
  .css_online = mem_cgroup_css_online,
  .css_offline = mem_cgroup_css_offline,
  .css_released = mem_cgroup_css_released,
  .css_free = mem_cgroup_css_free,
  .css_reset = mem_cgroup_css_reset,
  .can_attach = mem_cgroup_can_attach,
  .cancel_attach = mem_cgroup_cancel_attach,
  .post_attach = mem_cgroup_move_task,
  .bind = mem_cgroup_bind,
  .dfl_cftypes = memory_files,
  .legacy_cftypes = mem_cgroup_legacy_files,
  .early_init = 0,
};

static void __init cgroup_init_subsys(struct cgroup_subsys *ss, bool early)
{
  struct cgroup_subsys_state *css;

  idr_init(&ss->css_idr);
  INIT_LIST_HEAD(&ss->cfts);

  /* Create the root cgroup state for this subsystem */
  ss->root = &cgrp_dfl_root;
  css = ss->css_alloc(cgroup_css(&cgrp_dfl_root.cgrp, ss));

  init_and_link_css(css, ss, &cgrp_dfl_root.cgrp);

  css->id = cgroup_idr_alloc(&ss->css_idr, css, 1, 2, GFP_KERNEL);
  init_css_set.subsys[ss->id] = css;

  BUG_ON(online_css(css));
}

struct cgroup_subsys_state {
  struct cgroup *cgroup;
  struct cgroup_subsys *ss;
  struct list_head rstat_css_node;

  int id;
  unsigned int flags;

  struct work_struct destroy_work;
  struct rcu_work destroy_rwork;

  struct cgroup_subsys_state *parent;
};

/* cpu_cgroup_css_alloc -> sched_create_group create a struct task_group */
struct task_group {
  struct cgroup_subsys_state css;

  /* schedulable entities of this group on each cpu */
  struct sched_entity **se;
  /* runqueue "owned" by this group on each cpu */
  struct cfs_rq **cfs_rq;
  unsigned long shares;

  struct rcu_head rcu;
  struct list_head list;

  struct task_group *parent;
  struct list_head siblings;
  struct list_head children;

  struct cfs_bandwidth cfs_bandwidth;
};

/* online_css -> cpu_cgroup_css_online -> sched_online_group -> online_fair_sched_group */
void online_fair_sched_group(struct task_group *tg)
{
  struct sched_entity *se;
  struct rq *rq;
  int i;

  for_each_possible_cpu(i) {
    rq = cpu_rq(i);
    se = tg->se[i];
    update_rq_clock(rq);
    attach_entity_cfs_rq(se);
    sync_throttle(tg, i);
  }
}

/* css_alloc -> mem_cgroup_css_alloc -> mem_cgroup_alloc */
struct mem_cgroup {
  struct cgroup_subsys_state css;

  struct mem_cgroup_id id;

  struct page_counter memory;
  struct page_counter swap;

  struct page_counter memsw;
  struct page_counter kmem;
  struct page_counter tcpmem;

  unsigned long low;
  unsigned long high;

  struct work_struct high_work;

  unsigned long soft_limit;
  int  swappiness;

  struct mem_cgroup_stat_cpu __percpu *stat;

  int last_scanned_node;

  struct list_head event_list;
  spinlock_t event_list_lock;

  struct mem_cgroup_per_node *nodeinfo[0];
};

struct cgroup {
  struct cgroup_subsys_state self;
  struct cgroup_subsys_state *subsys[CGROUP_SUBSYS_COUNT];
  struct kernfs_node *kn;
  struct cgroup_root *root;
  struct cgroup_file  procs_file;  /* handle for "cgroup.procs" */
  struct cgroup_file  events_file; /* handle for "cgroup.events" */
};

struct cgroup_root {
  struct cgroup       cgrp;
  struct kernfs_root  *kf_root;
  struct list_head    root_list;
  unsigned int        subsys_mask;
  int                 hierarchy_id;
};

/* The list of hierarchy roots */
LIST_HEAD(cgroup_roots);
static int cgroup_root_count;

struct kernfs_root {
  struct kernfs_node  *kn;
  unsigned int        flags;  /* KERNFS_ROOT_* flags */
  struct idr          ino_idr;
  u32                 last_id_lowbits;
  u32                 id_highbits;
  struct kernfs_syscall_ops *syscall_ops;
  /* list of kernfs_super_info of this root, protected by kernfs_mutex */
  struct list_head    supers;
  wait_queue_head_t   deactivate_waitq;
};

struct kernfs_node {
  struct rb_node  rb;

  atomic_t            count;
  atomic_t            active;
  struct lockdep_map  dep_map;
  struct kernfs_node  *parent;
  const char          *name;

  const void      *ns;  /* namespace tag */
  unsigned int    hash; /* ns + name hash */
  union {
    struct kernfs_elem_dir      dir;
    struct kernfs_elem_symlink  symlink;
    struct kernfs_elem_attr     attr;
  };

  void                  *priv; /* struct cftype */
  struct kernfs_iattrs  *iattr;
};

struct kernfs_elem_attr {
  const struct kernfs_ops  *ops;
  struct kernfs_open_node  *open;
  loff_t                    size;
  struct kernfs_node       *notify_next; /* for kernfs_notify() */
};
```

#### mnt cgroup_fs_type
```C++
/* mount -> ksys_mount -> do_mount -> do_new_mount ->
 * vfs_kern_mount -> mount_fs -> cgroup_mount -> cgroup1_mount */
struct dentry *cgroup1_mount(
  struct file_system_type *fs_type, int flags,
  void *data, unsigned long magic,
  struct cgroup_namespace *ns)
{
  struct super_block *pinned_sb = NULL;
  struct cgroup_sb_opts opts;
  struct cgroup_root *root;
  struct cgroup_subsys *ss;
  struct dentry *dentry;
  int i, ret;
  bool new_root = false;

  root = kzalloc(sizeof(*root), GFP_KERNEL);
  new_root = true;

  init_cgroup_root(root, &opts);

  ret = cgroup_setup_root(root, opts.subsys_mask, PERCPU_REF_INIT_DEAD);

  dentry = cgroup_do_mount(&cgroup_fs_type, flags, root,
         CGROUP_SUPER_MAGIC, ns);

  return dentry;
}
```

```C++
int cgroup_setup_root(struct cgroup_root *root, u16 ss_mask, int ref_flags)
{
  LIST_HEAD(tmp_links);
  struct cgroup *root_cgrp = &root->cgrp;
  struct kernfs_syscall_ops *kf_sops;
  struct css_set *cset;
  int i, ret;

  root->kf_root = kernfs_create_root(kf_sops,
             KERNFS_ROOT_CREATE_DEACTIVATED,
             root_cgrp);
  root_cgrp->kn = root->kf_root->kn;

  ret = css_populate_dir(&root_cgrp->self);
  ret = rebind_subsystems(root, ss_mask);

  list_add(&root->root_list, &cgroup_roots);
  cgroup_root_count++;

  kernfs_activate(root_cgrp->kn);
}

/* create file tree and kernfs_node for each file, set file ops to kf_ops(cgroup_kf_ops)
 * css_populate_dir -> cgroup_addrm_files -> cgroup_add_file */
static int cgroup_add_file(
  struct cgroup_subsys_state *css,
  struct cgroup *cgrp,
  struct cftype *cft)
{
  char name[CGROUP_FILE_NAME_MAX];
  struct kernfs_node *kn;

  kn = __kernfs_create_file(cgrp->kn, cgroup_file_name(cgrp, cft, name),
          cgroup_file_mode(cft), 0, cft->kf_ops, cft,
          NULL, key);
}

struct kernfs_node *__kernfs_create_file(
  struct kernfs_node *parent,
  const char *name,
  umode_t mode, loff_t size,
  const struct kernfs_ops *ops, // cgroup_kf_ops
  void *priv, const void *ns,
  struct lock_class_key *key)
{
  struct kernfs_node *kn;
  unsigned flags;
  int rc;

  flags = KERNFS_FILE;

  kn = kernfs_new_node(parent, name, (mode & S_IALLUGO) | S_IFREG, flags);

  kn->attr.ops = ops;
  kn->attr.size = size;
  kn->ns = ns;
  kn->priv = priv;

  rc = kernfs_add_one(kn);

  return kn;
}

struct file_system_type cgroup_fs_type = {
  .name = "cgroup",
  .mount = cgroup_mount,
  .kill_sb = cgroup_kill_sb,
  .fs_flags = FS_USERNS_MOUNT,
};

/* file.read -> kernfs_file_fops.read -> cftype.kernfs_ops.read */
const struct file_operations kernfs_file_fops = {
  .read    = kernfs_fop_read,
  .write    = kernfs_fop_write,
  .llseek    = generic_file_llseek,
  .mmap    = kernfs_fop_mmap,
  .open    = kernfs_fop_open,
  .release  = kernfs_fop_release,
  .poll    = kernfs_fop_poll,
  .fsync    = noop_fsync,
};

static struct kernfs_ops cgroup_kf_ops = {
  .atomic_write_len  = PAGE_SIZE,
  .open      = cgroup_file_open,
  .release    = cgroup_file_release,
  .write      = cgroup_file_write,
  .seq_start    = cgroup_seqfile_start,
  .seq_next    = cgroup_seqfile_next,
  .seq_stop    = cgroup_seqfile_stop,
  .seq_show    = cgroup_seqfile_show,
};

static struct cftype cpu_files[] = {
  {
    .name = "shares",
    .read_u64 = cpu_shares_read_u64,
    .write_u64 = cpu_shares_write_u64,
  },
  {
    .name = "cfs_quota_us",
    .read_s64 = cpu_cfs_quota_read_s64,
    .write_s64 = cpu_cfs_quota_write_s64,
  },
  {
    .name = "cfs_period_us",
    .read_u64 = cpu_cfs_period_read_u64,
    .write_u64 = cpu_cfs_period_write_u64,
  }
}

static struct cftype mem_cgroup_legacy_files[] = {
  {
    .name = "usage_in_bytes",
    .private = MEMFILE_PRIVATE(_MEM, RES_USAGE),
    .read_u64 = mem_cgroup_read_u64,
  },
  {
    .name = "max_usage_in_bytes",
    .private = MEMFILE_PRIVATE(_MEM, RES_MAX_USAGE),
    .write = mem_cgroup_reset,
    .read_u64 = mem_cgroup_read_u64,
  },
  {
    .name = "limit_in_bytes",
    .private = MEMFILE_PRIVATE(_MEM, RES_LIMIT),
    .write = mem_cgroup_write,
    .read_u64 = mem_cgroup_read_u64,
  },
  {
    .name = "soft_limit_in_bytes",
    .private = MEMFILE_PRIVATE(_MEM, RES_SOFT_LIMIT),
    .write = mem_cgroup_write,
    .read_u64 = mem_cgroup_read_u64,
  }
}
```

```C++
/* mount -> ksys_mount -> do_mount -> do_new_mount ->
 * vfs_kern_mount -> mount_fs -> cgroup_mount -> cgroup1_mount */
struct dentry *cgroup_do_mount(
  struct file_system_type *fs_type, int flags,
  struct cgroup_root *root, unsigned long magic,
  struct cgroup_namespace *ns)
{
  struct dentry *dentry;
  bool new_sb = false;

  dentry = kernfs_mount(fs_type, flags, root->kf_root, magic, &new_sb);

  if (!IS_ERR(dentry) && ns != &init_cgroup_ns) {
    struct dentry *nsdentry;
    struct super_block *sb = dentry->d_sb;
    struct cgroup *cgrp;

    cgrp = cset_cgroup_from_root(ns->root_cset, root);

    nsdentry = kernfs_node_dentry(cgrp->kn, sb);
    dput(dentry);
    if (IS_ERR(nsdentry))
      deactivate_locked_super(sb);
    dentry = nsdentry;
  }

  if (!new_sb)
    cgroup_put(&root->cgrp);

  return dentry;
}

static inline struct dentry* kernfs_mount(
  struct file_system_type *fs_type, int flags,
  struct kernfs_root *root, unsigned long magic,
  bool *new_sb_created)
{
  return kernfs_mount_ns(fs_type, flags, root,
        magic, new_sb_created, NULL);
}

struct dentry *kernfs_mount_ns(
  struct file_system_type *fs_type, int flags,
  struct kernfs_root *root, unsigned long magic,
  bool *new_sb_created, const void *ns)
{
  struct super_block *sb;
  struct kernfs_super_info *info;
  int error;

  info = kzalloc(sizeof(*info), GFP_KERNEL);
  if (!info)
    return ERR_PTR(-ENOMEM);

  info->root = root;
  info->ns = ns;
  INIT_LIST_HEAD(&info->node);

  sb = sget_userns(fs_type, kernfs_test_super, kernfs_set_super, flags,
       &init_user_ns, info);

  if (new_sb_created)
    *new_sb_created = !sb->s_root;

  if (!sb->s_root) {
    struct kernfs_super_info *info = kernfs_info(sb);

    error = kernfs_fill_super(sb, magic);

    sb->s_flags |= SB_ACTIVE;

    list_add(&info->node, &root->supers);
  }

  return dget(sb->s_root);
}

```
#### e.g. cpu.shares
```C++
// cpu.shares -> cpu_shares_write_u64
int sched_group_set_shares(struct task_group *tg, unsigned long shares)
{
  int i;

  shares = clamp(shares, scale_load(MIN_SHARES), scale_load(MAX_SHARES));

  tg->shares = shares;
  for_each_possible_cpu(i) {
    struct rq *rq = cpu_rq(i);
    struct sched_entity *se = tg->se[i];
    struct rq_flags rf;

    update_rq_clock(rq);
    for_each_sched_entity(se) {
      update_load_avg(se, UPDATE_TG);
      update_cfs_shares(se);
    }
  }
}
```

#### attach
```C++
/* write id to /sys/fs/cgroup/cpu,cpuacct/tasks file
 *
 * cgroup1_base_files.cgroup_tasks_write -> __cgroup_procs_write ->
 * cgroup_attach_task -> cgroup_migrate -> cgroup_migrate_execute */
static int cgroup_migrate_execute(struct cgroup_mgctx *mgctx)
{
  struct cgroup_taskset *tset = &mgctx->tset;
  struct cgroup_subsys *ss;
  struct task_struct *task, *tmp_task;
  struct css_set *cset, *tmp_cset;

  if (tset->nr_tasks) {
    do_each_subsys_mask(ss, ssid, mgctx->ss_mask) {
      if (ss->attach) {
        tset->ssid = ssid;
        ss->attach(tset);
      }
    } while_each_subsys_mask();
  }
}

// cpu_cgroup_attach -> sched_move_task -> sched_change_group
static void sched_change_group(struct task_struct *tsk, int type)
{
  struct task_group *tg;

  tg = container_of(task_css_check(tsk, cpu_cgrp_id, true),
        struct task_group, css);
  tg = autogroup_task_group(tsk, tg);
  tsk->sched_task_group = tg;

#ifdef CONFIG_FAIR_GROUP_SCHED
  if (tsk->sched_class->task_change_group)
    tsk->sched_class->task_change_group(tsk, type);
  else
#endif
    set_task_rq(tsk, task_cpu(tsk));
}

// handle_pte_fault -> do_anonymous_page() -> mem_cgroup_try_charge
int mem_cgroup_try_charge(struct page *page, struct mm_struct *mm,
        gfp_t gfp_mask, struct mem_cgroup **memcgp,
        bool compound)
{
  struct mem_cgroup *memcg = NULL;

  if (!memcg)
    memcg = get_mem_cgroup_from_mm(mm);

  ret = try_charge(memcg, gfp_mask, nr_pages);
}
```
![linux-container-cgroup-arch.png](../Images/Kernel/container-cgroup-arch.png)

## Q:
1. What do `cgroup_roots` and `cgroup_dlt_root` for?
2. The hierarchy of cgroup file system?

# Time & Timer
## init
```C++
void start_kernel(void)
{
  tick_init();
  init_timers();
  hrtimers_init();
  timekeeping_init();
  time_init();

  if (late_time_init)
    late_time_init();
}

void __init tick_init(void)
{
  tick_broadcast_init();
  tick_nohz_init();
}

void __init init_timers(void)
{
  /* init each cpu timer_bases */
  init_timer_cpus();
  open_softirq(TIMER_SOFTIRQ, run_timer_softirq);
}

void __init hrtimers_init(void)
{
  hrtimers_prepare_cpu(smp_processor_id());
  open_softirq(HRTIMER_SOFTIRQ, hrtimer_run_softirq);
}

void __init timekeeping_init(void)
{
  struct timespec64 wall_time, boot_offset, wall_to_mono;
  struct timekeeper *tk = &tk_core.timekeeper;
  struct clocksource *clock;
  unsigned long flags;

  read_persistent_wall_and_boot_offset(&wall_time, &boot_offset);
  if (timespec64_valid_settod(&wall_time) &&
      timespec64_to_ns(&wall_time) > 0) {
    persistent_clock_exists = true;
  } else if (timespec64_to_ns(&wall_time) != 0) {
    pr_warn("Persistent clock returned invalid value");
    wall_time = (struct timespec64){0};
  }

  if (timespec64_compare(&wall_time, &boot_offset) < 0)
    boot_offset = (struct timespec64){0};

  /* We want set wall_to_mono, so the following is true:
   * wall time + wall_to_mono = boot time */
  wall_to_mono = timespec64_sub(boot_offset, wall_time);

  raw_spin_lock_irqsave(&timekeeper_lock, flags);
  write_seqcount_begin(&tk_core.seq);
  ntp_init();

  clock = clocksource_default_clock();
  if (clock->enable)
    clock->enable(clock);
  tk_setup_internals(tk, clock);

  tk_set_xtime(tk, &wall_time);
  tk->raw_sec = 0;

  tk_set_wall_to_mono(tk, wall_to_mono);

  timekeeping_update(tk, TK_MIRROR | TK_CLOCK_WAS_SET);

  write_seqcount_end(&tk_core.seq);
  raw_spin_unlock_irqrestore(&timekeeper_lock, flags);
}


/* Initialize TSC and delay the periodic timer init to
 * late x86_late_time_init() so ioremap works. */
void __init time_init(void)
{
  late_time_init = x86_late_time_init;
}


struct x86_init_ops x86_init = {
  .irqs = {
    .pre_vector_init        = init_ISA_irqs,
    .intr_init              = native_init_IRQ,
    .trap_init              = x86_init_noop,
    .intr_mode_init         = apic_intr_mode_init
  },
  .timers = {
    .setup_percpu_clockev  = setup_boot_APIC_clock,
    .timer_init            = hpet_time_init,
    .wallclock_init        = x86_init_noop,
  }
};

static __init void x86_late_time_init(void)
{
  x86_init.timers.timer_init();
  /* After PIT/HPET timers init, select and setup
   * the final interrupt mode for delivering IRQs. */

  /* x86_init.irqs.intr_mode_init  = x86_init_noop; */
  x86_init.irqs.intr_mode_init();
  tsc_init();
}
```

## hpet_time_init
```C++
void __init hpet_time_init(void)
{
  if (!hpet_enable())
    setup_pit_timer(); /* global_clock_event = &i8253_clockevent; */
  setup_default_timer_irq();
}

int __init hpet_enable(void)
{
  u32 hpet_period, cfg, id;
  u64 freq;
  unsigned int i, last;

  if (!is_hpet_capable())
    return 0;

  hpet_set_mapping();
  if (!hpet_virt_address)
    return 0;

  /* Read the period and check for a sane value: */
  hpet_period = hpet_readl(HPET_PERIOD);

  for (i = 0; hpet_readl(HPET_CFG) == 0xFFFFFFFF; i++) {
    if (i == 1000) {
      goto out_nohpet;
    }
  }

  if (hpet_period < HPET_MIN_PERIOD || hpet_period > HPET_MAX_PERIOD)
    goto out_nohpet;

  freq = FSEC_PER_SEC;
  do_div(freq, hpet_period);
  hpet_freq = freq;

  id = hpet_readl(HPET_ID);
  hpet_print_config();

  last = (id & HPET_ID_NUMBER) >> HPET_ID_NUMBER_SHIFT;

  cfg = hpet_readl(HPET_CFG);
  hpet_boot_cfg = kmalloc_array(last + 2, sizeof(*hpet_boot_cfg),
              GFP_KERNEL);
  if (hpet_boot_cfg)
    *hpet_boot_cfg = cfg;
  else
    pr_warn("HPET initial state will not be saved\n");
  cfg &= ~(HPET_CFG_ENABLE | HPET_CFG_LEGACY);
  hpet_writel(cfg, HPET_CFG);
  if (cfg)
    pr_warn("Unrecognized bits %#x set in global cfg\n", cfg);

  for (i = 0; i <= last; ++i) {
    cfg = hpet_readl(HPET_Tn_CFG(i));
    if (hpet_boot_cfg)
      hpet_boot_cfg[i + 1] = cfg;
    cfg &= ~(HPET_TN_ENABLE | HPET_TN_LEVEL | HPET_TN_FSB);
    hpet_writel(cfg, HPET_Tn_CFG(i));
    cfg &= ~(HPET_TN_PERIODIC | HPET_TN_PERIODIC_CAP
       | HPET_TN_64BIT_CAP | HPET_TN_32BIT | HPET_TN_ROUTE
       | HPET_TN_FSB | HPET_TN_FSB_CAP);
    if (cfg)
      pr_warn("Unrecognized bits %#x set in cfg#%u\n",
        cfg, i);
  }
  hpet_print_config();

  if (hpet_clocksource_register())
    goto out_nohpet;

  if (id & HPET_ID_LEGSUP) {
    hpet_legacy_clockevent_register();
    return 1;
  }
  return 0;

out_nohpet:
  hpet_clear_mapping();
  hpet_address = 0;
  return 0;
}

struct clocksource {
  u64 (*read)(struct clocksource *cs);
  u64 mask;
  u32 mult;
  u32 shift;
  u64 max_idle_ns;
  u32 maxadj;
  struct arch_clocksource_data  archdata;
  u64                           max_cycles;
  int                           rating;
  const char                    *name;
  unsigned long                 flags;
  struct list_head              list;

  int  (*enable)(struct clocksource *cs);
  void (*disable)(struct clocksource *cs);
  void (*suspend)(struct clocksource *cs);
  void (*resume)(struct clocksource *cs);
  void (*mark_unstable)(struct clocksource *cs);
  void (*tick_stable)(struct clocksource *cs);

  struct module *owner;
};

static int hpet_clocksource_register(void)
{
  u64 start, now;
  u64 t1;

  /* Start the counter */
  hpet_restart_counter();

  /* Verify whether hpet counter works */
  t1 = hpet_readl(HPET_COUNTER);
  start = rdtsc();

  /* We don't know the TSC frequency yet, but waiting for
   * 200000 TSC cycles is safe:
   * 4 GHz == 50us
   * 1 GHz == 200us */
  do {
    rep_nop();
    now = rdtsc();
  } while ((now - start) < 200000UL);

  if (t1 == hpet_readl(HPET_COUNTER)) {
    return -ENODEV;
  }

  clocksource_register_hz(&clocksource_hpet, (u32)hpet_freq);
  return 0;
}

/* hardware abstraction for a free running counter */
static struct clocksource clocksource_hpet = {
  .name    = "hpet",
  .rating  = 250,
  .read    = read_hpet,
  .mask    = HPET_MASK,
  .flags    = CLOCK_SOURCE_IS_CONTINUOUS,
  .resume  = hpet_resume_counter,
};

static void __iomem      *hpet_virt_address;

static u64 read_hpet(struct clocksource *cs)
{
  return (u64)hpet_readl(HPET_COUNTER);
}

inline unsigned int hpet_readl(unsigned int a)
{
  return readl(hpet_virt_address + a);
}
```

### clocksource_register_hz
```C++
static inline int clocksource_register_hz(struct clocksource *cs, u32 hz)
{
  return __clocksource_register_scale(cs, 1, hz);
}

int __clocksource_register_scale(struct clocksource *cs, u32 scale, u32 freq)
{
  unsigned long flags;

  /* Initialize mult/shift and max_idle_ns */
  __clocksource_update_freq_scale(cs, scale, freq);

  /* Add clocksource to the clocksource list */
  mutex_lock(&clocksource_mutex);

  clocksource_watchdog_lock(&flags);
  /* enqueue the clocksource sorted by rating */
  clocksource_enqueue(cs);
  clocksource_enqueue_watchdog(cs);
  clocksource_watchdog_unlock(&flags);

  /* select highest rating clocksource */
  clocksource_select();
  clocksource_select_watchdog(false);
  __clocksource_suspend_select(cs);
  mutex_unlock(&clocksource_mutex);
  return 0;
}
```

### clockevents_config_and_register
```C++
static void hpet_legacy_clockevent_register(void)
{
  hpet_enable_legacy_int();

  hpet_clockevent.cpumask = cpumask_of(boot_cpu_data.cpu_index);
  clockevents_config_and_register(&hpet_clockevent, hpet_freq,
          HPET_MIN_PROG_DELTA, 0x7FFFFFFF);
  global_clock_event = &hpet_clockevent;
  printk(KERN_DEBUG "hpet clockevent registered\n");
}

void clockevents_config_and_register(struct clock_event_device *dev,
             u32 freq, unsigned long min_delta,
             unsigned long max_delta)
{
  dev->min_delta_ticks = min_delta;
  dev->max_delta_ticks = max_delta;
  clockevents_config(dev, freq);
  clockevents_register_device(dev);
}

void clockevents_register_device(struct clock_event_device *dev)
{
  unsigned long flags;

  clockevent_set_state(dev, CLOCK_EVT_STATE_DETACHED);

  if (!dev->cpumask) {
    WARN_ON(num_possible_cpus() > 1);
    dev->cpumask = cpumask_of(smp_processor_id());
  }

  if (dev->cpumask == cpu_all_mask) {
    dev->cpumask = cpu_possible_mask;
  }

  raw_spin_lock_irqsave(&clockevents_lock, flags);

  list_add(&dev->list, &clockevent_devices);
  tick_check_new_device(dev);
  clockevents_notify_released();

  raw_spin_unlock_irqrestore(&clockevents_lock, flags);
}

void tick_check_new_device(struct clock_event_device *newdev)
{
  struct clock_event_device *curdev;
  struct tick_device *td;
  int cpu;

  cpu = smp_processor_id();
  td = &per_cpu(tick_cpu_device, cpu);
  curdev = td->evtdev;

  /* cpu local device ? */
  if (!tick_check_percpu(curdev, newdev, cpu))
    goto out_bc;

  /* Preference decision */
  if (!tick_check_preferred(curdev, newdev))
    goto out_bc;

  if (!try_module_get(newdev->owner))
    return;

  if (tick_is_broadcast_device(curdev)) {
    clockevents_shutdown(curdev);
    curdev = NULL;
  }

  clockevents_exchange_device(curdev, newdev);
  tick_setup_device(td, newdev, cpu, cpumask_of(cpu));
  if (newdev->features & CLOCK_EVT_FEAT_ONESHOT)
    tick_oneshot_notify();
  return;

out_bc:
  tick_install_broadcast_device(newdev);
}

struct tick_device {
  struct clock_event_device   *evtdev;
  enum tick_device_mode       mode;
};

enum tick_device_mode {
  TICKDEV_MODE_PERIODIC,
  TICKDEV_MODE_ONESHOT,
};

struct clock_event_device {
  void     (*event_handler)(struct clock_event_device *);
  int      (*set_next_event)(unsigned long evt, struct clock_event_device *);
  int      (*set_next_ktime)(ktime_t expires, struct clock_event_device *);

  int      rating;
  int      irq;
  int      bound_on;
  const struct cpumask  *cpumask;
  struct list_head      list;
  struct module         *owner;
};

static void tick_setup_device(
  struct tick_device *td, struct clock_event_device *newdev,
  int cpu, const struct cpumask *cpumask)
{
  void (*handler)(struct clock_event_device *) = NULL;
  ktime_t next_event = 0;

  if (!td->evtdev) {
    if (tick_do_timer_cpu == TICK_DO_TIMER_BOOT) {
      if (!tick_nohz_full_cpu(cpu))
        tick_do_timer_cpu = cpu;
      else
        tick_do_timer_cpu = TICK_DO_TIMER_NONE;
      tick_next_period = ktime_get();
      tick_period = NSEC_PER_SEC / HZ;
    }

    /*Startup in periodic mode first. */
    td->mode = TICKDEV_MODE_PERIODIC;
  } else {
    handler = td->evtdev->event_handler;
    next_event = td->evtdev->next_event;
    td->evtdev->event_handler = clockevents_handle_noop;
  }

  td->evtdev = newdev;

  /* When the device is not per cpu, pin the interrupt to the
   * current cpu: */
  if (!cpumask_equal(newdev->cpumask, cpumask))
    irq_set_affinity(newdev->irq, cpumask);

  /*
   * When global broadcasting is active, check if the current
   * device is registered as a placeholder for broadcast mode.
   * This allows us to handle this x86 misfeature in a generic
   * way. This function also returns !=0 when we keep the
   * current active broadcast state for this CPU.
   */
  if (tick_device_uses_broadcast(newdev, cpu))
    return;

  if (td->mode == TICKDEV_MODE_PERIODIC)
    tick_setup_periodic(newdev, 0);
  else
    tick_setup_oneshot(newdev, handler, next_event);
}

void tick_setup_periodic(struct clock_event_device *dev, int broadcast)
{
  tick_set_periodic_handler(dev, broadcast);

  /* Broadcast setup ? */
  if (!tick_device_is_functional(dev))
    return;

  if ((dev->features & CLOCK_EVT_FEAT_PERIODIC)
    && !tick_broadcast_oneshot_active()) {
    clockevents_switch_state(dev, CLOCK_EVT_STATE_PERIODIC);
  } else {
    unsigned long seq;
    ktime_t next;

    do {
      seq = read_seqbegin(&jiffies_lock);
      next = tick_next_period;
    } while (read_seqretry(&jiffies_lock, seq));

    clockevents_switch_state(dev, CLOCK_EVT_STATE_ONESHOT);

    for (;;) {
      /* Reprogram the clock event device */
      if (!clockevents_program_event(dev, next, false))
        return;
      next = ktime_add(next, tick_period);
    }
  }
}

void tick_set_periodic_handler(struct clock_event_device *dev, int broadcast)
{
  if (!broadcast)
    dev->event_handler = tick_handle_periodic;
  else
    dev->event_handler = tick_handle_periodic_broadcast;
}

void tick_setup_oneshot(struct clock_event_device *newdev,
      void (*handler)(struct clock_event_device *),
      ktime_t next_event)
{
  newdev->event_handler = handler;
  clockevents_switch_state(newdev, CLOCK_EVT_STATE_ONESHOT);
  clockevents_program_event(newdev, next_event, true);
}
```

## setup irq0 timer irq
```C++
static struct irqaction irq0  = {
  .handler = timer_interrupt,
  .flags = IRQF_NOBALANCING | IRQF_IRQPOLL | IRQF_TIMER,
  .name = "timer"
};

static void setup_default_timer_irq(void)
{
  if (setup_irq(0, &irq0))
    pr_info("Failed to register legacy timer interrupt\n");
}

static irqreturn_t timer_interrupt(int irq, void *dev_id)
{
  /* global_clock_event = &i8253_clockevent
   * global_clock_event = &hpet_clockevent;
   * dev->event_handler = tick_handle_periodic; */
  global_clock_event->event_handler(global_clock_event);
  return IRQ_HANDLED;
}

/* On UP the PIT can serve all of the possible timer functions. On SMP systems
 * it can be solely used for the global tick. */
struct clock_event_device i8253_clockevent = {
  .name                 = "pit",
  .features             = CLOCK_EVT_FEAT_PERIODIC,
  .set_state_shutdown   = pit_shutdown,
  .set_state_periodic   = pit_set_periodic,
  .set_next_event       = pit_next_event,
};

static struct clock_event_device hpet_clockevent = {
  .name       = "hpet",
  .features    = CLOCK_EVT_FEAT_PERIODIC |  CLOCK_EVT_FEAT_ONESHOT,
  .set_state_periodic   = hpet_legacy_set_periodic,
  .set_state_oneshot    = hpet_legacy_set_oneshot,
  .set_state_shutdown   = hpet_legacy_shutdown,
  .tick_resume          = hpet_legacy_resume,
  .set_next_event       = hpet_legacy_next_event,
  .irq                  = 0,
  .rating               = 50,
};

void __init tsc_init(void)
{
  /* native_calibrate_cpu_early can only calibrate using methods that are
   * available early in boot. */
  if (x86_platform.calibrate_cpu == native_calibrate_cpu_early)
    x86_platform.calibrate_cpu = native_calibrate_cpu;

  if (!boot_cpu_has(X86_FEATURE_TSC)) {
    setup_clear_cpu_cap(X86_FEATURE_TSC_DEADLINE_TIMER);
    return;
  }

  if (!tsc_khz) {
    /* We failed to determine frequencies earlier, try again */
    if (!determine_cpu_tsc_frequencies(false)) {
      mark_tsc_unstable("could not calculate TSC khz");
      setup_clear_cpu_cap(X86_FEATURE_TSC_DEADLINE_TIMER);
      return;
    }
    tsc_enable_sched_clock();
  }

  cyc2ns_init_secondary_cpus();

  if (!no_sched_irq_time)
    enable_sched_clock_irqtime();

  lpj_fine = get_loops_per_jiffy();
  use_tsc_delay();

  check_system_tsc_reliable();

  if (unsynchronized_tsc()) {
    mark_tsc_unstable("TSCs unsynchronized");
    return;
  }

  clocksource_register_khz(&clocksource_tsc_early, tsc_khz);
  detect_art();
}
```

## handle irq0 timer irq
```C++
void tick_handle_periodic(struct clock_event_device *dev)
{
  int cpu = smp_processor_id();
  ktime_t next = dev->next_event;

  tick_periodic(cpu);

  if (!clockevent_state_oneshot(dev))
    return;
  for (;;) {
    /* Setup the next period for devices, which do not have
     * periodic mode: */
    next = ktime_add(next, tick_period);

    if (!clockevents_program_event(dev, next, false))
      return;

    if (timekeeping_valid_for_hres())
      tick_periodic(cpu);
  }
}

static void tick_periodic(int cpu)
{
  if (tick_do_timer_cpu == cpu) {
    write_seqlock(&jiffies_lock);

    /* Keep track of the next tick event */
    tick_next_period = ktime_add(tick_next_period, tick_period);

    do_timer(1);
    write_sequnlock(&jiffies_lock);
    update_wall_time();
  }

  update_process_times(user_mode(get_irq_regs()));
  profile_tick(CPU_PROFILING);
}

void do_timer(unsigned long ticks)
{
  jiffies_64 += ticks;
  calc_global_load(ticks);
}

void update_wall_time(void)
{
  timekeeping_advance(TK_ADV_TICK);
}

void update_process_times(int user_tick)
{
  struct task_struct *p = current;

  /* Note: this timer irq context must be accounted for as well. */
  account_process_tick(p, user_tick);

  run_local_timers();
  rcu_check_callbacks(user_tick);
  if (in_irq())
    irq_work_tick();

  scheduler_tick();
  if (IS_ENABLED(CONFIG_POSIX_TIMERS))
    run_posix_cpu_timers(p);
}

void run_local_timers(void)
{
  struct timer_base *base = this_cpu_ptr(&timer_bases[BASE_STD]);

  hrtimer_run_queues();
  /* Raise the softirq only if required. */
  if (time_before(jiffies, base->clk)) {
    if (!IS_ENABLED(CONFIG_NO_HZ_COMMON))
      return;
    /* CPU is awake, so check the deferrable base. */
    base++;
    if (time_before(jiffies, base->clk))
      return;
  }
  raise_softirq(TIMER_SOFTIRQ); /* run_timer_softirq */
}

struct timer_base {
  raw_spinlock_t     lock;
  struct timer_list  *running_timer;
  unsigned long      clk;
  unsigned long      next_expiry;
  unsigned int       cpu;
  bool               is_idle;
  bool               must_forward_clk;
  DECLARE_BITMAP(pending_map, WHEEL_SIZE);
  struct hlist_head  vectors[WHEEL_SIZE];
}
```

## setup_APIC_timer
```C++
static void setup_APIC_timer(void)
{
  struct clock_event_device *levt = this_cpu_ptr(&lapic_events);

  if (this_cpu_has(X86_FEATURE_ARAT)) {
    lapic_clockevent.features &= ~CLOCK_EVT_FEAT_C3STOP;
    /* Make LAPIC timer preferrable over percpu HPET */
    lapic_clockevent.rating = 150;
  }

  memcpy(levt, &lapic_clockevent, sizeof(*levt));
  levt->cpumask = cpumask_of(smp_processor_id());

  if (this_cpu_has(X86_FEATURE_TSC_DEADLINE_TIMER)) {
    levt->name = "lapic-deadline";
    levt->features &= ~(CLOCK_EVT_FEAT_PERIODIC |
            CLOCK_EVT_FEAT_DUMMY);
    levt->set_next_event = lapic_next_deadline;
    clockevents_config_and_register(levt,
            tsc_khz * (1000 / TSC_DIVISOR),
            0xF, ~0UL);
  } else
    clockevents_register_device(levt);
}
```

## hrtimer_run_queues
```C++
struct hrtimer_cpu_base {
  raw_spinlock_t      lock;
  unsigned int        cpu;
  unsigned int        active_bases;
  unsigned int        clock_was_set_seq;
  unsigned int        hres_active: 1,
                      in_hrtirq: 1,
                      hang_detected: 1,
                      softirq_activated: 1;
  ktime_t             expires_next;
  struct hrtimer      *next_timer;
  ktime_t             softirq_expires_next;
  struct hrtimer      *softirq_next_timer;
  struct hrtimer_clock_base  clock_base[HRTIMER_MAX_CLOCK_BASES];
};

enum  hrtimer_base_type {
  HRTIMER_BASE_MONOTONIC,
  HRTIMER_BASE_REALTIME,
  HRTIMER_BASE_BOOTTIME,
  HRTIMER_BASE_TAI,
  HRTIMER_BASE_MONOTONIC_SOFT,
  HRTIMER_BASE_REALTIME_SOFT,
  HRTIMER_BASE_BOOTTIME_SOFT,
  HRTIMER_BASE_TAI_SOFT,
  HRTIMER_MAX_CLOCK_BASES,
};

struct hrtimer_clock_base {
  struct hrtimer_cpu_base  *cpu_base;
  unsigned int    index;
  clockid_t    clockid;
  seqcount_t    seq;
  struct hrtimer    *running;
  struct timerqueue_head  active;
  ktime_t      (*get_time)(void);
  ktime_t      offset;
};

struct timerqueue_head {
  struct rb_root          head;
  struct timerqueue_node  *next;
};

struct timerqueue_node {
  struct rb_node  node;
  ktime_t         expires;
};

struct hrtimer {
  struct timerqueue_node    node;
  ktime_t                   _softexpires;
  enum hrtimer_restart      (*function)(struct hrtimer *);
  struct hrtimer_clock_base  *base;
  u8  state;
  u8  is_rel;  /* timer was armed relative */
  u8  is_soft; /* expired in soft interrupt context */
};

void hrtimer_run_queues(void)
{
  struct hrtimer_cpu_base *cpu_base = this_cpu_ptr(&hrtimer_bases);
  unsigned long flags;
  ktime_t now;

  if (__hrtimer_hres_active(cpu_base))
    return;

  if (tick_check_oneshot_change(!hrtimer_is_hres_enabled())) {
    /* change event_handler to hrtimer_interrupt */
    hrtimer_switch_to_hres();
    return;
  }

  raw_spin_lock_irqsave(&cpu_base->lock, flags);
  now = hrtimer_update_base(cpu_base);

  if (!ktime_before(now, cpu_base->softirq_expires_next)) {
    cpu_base->softirq_expires_next = KTIME_MAX;
    cpu_base->softirq_activated = 1;
    raise_softirq_irqoff(HRTIMER_SOFTIRQ);
  }

  __hrtimer_run_queues(cpu_base, now, flags, HRTIMER_ACTIVE_HARD);
  raw_spin_unlock_irqrestore(&cpu_base->lock, flags);
}


static void __hrtimer_run_queues(struct hrtimer_cpu_base *cpu_base, ktime_t now,
         unsigned long flags, unsigned int active_mask)
{
  struct hrtimer_clock_base *base;
  unsigned int active = cpu_base->active_bases & active_mask;

  for_each_active_base(base, cpu_base, active) {
    struct timerqueue_node *node;
    ktime_t basenow;

    basenow = ktime_add(now, base->offset);

    while ((node = timerqueue_getnext(&base->active))) {
      struct hrtimer *timer;
      timer = container_of(node, struct hrtimer, node);
      if (basenow < hrtimer_get_softexpires_tv64(timer))
        break;
      __run_hrtimer(cpu_base, base, timer, &basenow, flags);
    }
  }
}

static void __run_hrtimer(struct hrtimer_cpu_base *cpu_base,
        struct hrtimer_clock_base *base,
        struct hrtimer *timer, ktime_t *now,
        unsigned long flags)
{
  enum hrtimer_restart (*fn)(struct hrtimer *);
  int restart;

  lockdep_assert_held(&cpu_base->lock);

  debug_deactivate(timer);
  base->running = timer;

  /*
   * Separate the ->running assignment from the ->state assignment.
   *
   * As with a regular write barrier, this ensures the read side in
   * hrtimer_active() cannot observe base->running == NULL &&
   * timer->state == INACTIVE.
   */
  raw_write_seqcount_barrier(&base->seq);

  __remove_hrtimer(timer, base, HRTIMER_STATE_INACTIVE, 0);
  fn = timer->function;

  /*
   * Clear the 'is relative' flag for the TIME_LOW_RES case. If the
   * timer is restarted with a period then it becomes an absolute
   * timer. If its not restarted it does not matter.
   */
  if (IS_ENABLED(CONFIG_TIME_LOW_RES))
    timer->is_rel = false;

  /*
   * The timer is marked as running in the CPU base, so it is
   * protected against migration to a different CPU even if the lock
   * is dropped.
   */
  raw_spin_unlock_irqrestore(&cpu_base->lock, flags);
  trace_hrtimer_expire_entry(timer, now);
  restart = fn(timer);
  trace_hrtimer_expire_exit(timer);
  raw_spin_lock_irq(&cpu_base->lock);

  /*
   * Note: We clear the running state after enqueue_hrtimer and
   * we do not reprogram the event hardware. Happens either in
   * hrtimer_start_range_ns() or in hrtimer_interrupt()
   *
   * Note: Because we dropped the cpu_base->lock above,
   * hrtimer_start_range_ns() can have popped in and enqueued the timer
   * for us already.
   */
  if (restart != HRTIMER_NORESTART &&
      !(timer->state & HRTIMER_STATE_ENQUEUED))
    enqueue_hrtimer(timer, base, HRTIMER_MODE_ABS);

  /*
   * Separate the ->running assignment from the ->state assignment.
   *
   * As with a regular write barrier, this ensures the read side in
   * hrtimer_active() cannot observe base->running.timer == NULL &&
   * timer->state == INACTIVE.
   */
  raw_write_seqcount_barrier(&base->seq);

  WARN_ON_ONCE(base->running != timer);
  base->running = NULL;
}
```

### tick_check_oneshot_change
```C++
int tick_check_oneshot_change(int allow_nohz)
{
  struct tick_sched *ts = this_cpu_ptr(&tick_cpu_sched);

  if (!test_and_clear_bit(0, &ts->check_clocks))
    return 0;

  if (ts->nohz_mode != NOHZ_MODE_INACTIVE)
    return 0;

  if (!timekeeping_valid_for_hres() || !tick_is_oneshot_available())
    return 0;

  if (!allow_nohz)
    return 1;

  tick_nohz_switch_to_nohz();
  return 0;
}

static void tick_nohz_switch_to_nohz(void)
{
  struct tick_sched *ts = this_cpu_ptr(&tick_cpu_sched);
  ktime_t next;

  if (!tick_nohz_enabled)
    return;

  if (tick_switch_to_oneshot(tick_nohz_handler))
    return;

  /* Recycle the hrtimer in ts, so we can share the
   * hrtimer_forward with the highres code. */
  hrtimer_init(&ts->sched_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
  /* Get the next period */
  next = tick_init_jiffy_update();

  hrtimer_set_expires(&ts->sched_timer, next);
  hrtimer_forward_now(&ts->sched_timer, tick_period);
  tick_program_event(hrtimer_get_expires(&ts->sched_timer), 1);
  tick_nohz_activate(ts, NOHZ_MODE_LOWRES);
}

int tick_switch_to_oneshot(void (*handler)(struct clock_event_device *))
{
  struct tick_device *td = this_cpu_ptr(&tick_cpu_device);
  struct clock_event_device *dev = td->evtdev;

  td->mode = TICKDEV_MODE_ONESHOT;
  dev->event_handler = handler;
  clockevents_switch_state(dev, CLOCK_EVT_STATE_ONESHOT);
  tick_broadcast_switch_to_oneshot();
  return 0;
}

int tick_program_event(ktime_t expires, int force)
{
  struct clock_event_device *dev = __this_cpu_read(tick_cpu_device.evtdev);

  if (unlikely(clockevent_state_oneshot_stopped(dev))) {
    clockevents_switch_state(dev, CLOCK_EVT_STATE_ONESHOT);
  }

  return clockevents_program_event(dev, expires, force);
}
```

### hrtimer_switch_to_hres
```C++
static void hrtimer_switch_to_hres(void)
{
  struct hrtimer_cpu_base *base = this_cpu_ptr(&hrtimer_bases);

  if (tick_init_highres()) {
    return;
  }

  base->hres_active = 1;
  hrtimer_resolution = HIGH_RES_NSEC;

  tick_setup_sched_timer();
  /* "Retrigger" the interrupt to get things going */
  retrigger_next_event(NULL);
}

int tick_init_highres(void)
{
  return tick_switch_to_oneshot(hrtimer_interrupt);
}
```

### hrtimer tick emulation
```C++
/* setup the tick emulation timer */
void tick_setup_sched_timer(void)
{
  struct tick_sched *ts = this_cpu_ptr(&tick_cpu_sched);
  ktime_t now = ktime_get();

  hrtimer_init(&ts->sched_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
  ts->sched_timer.function = tick_sched_timer;

  /* Get the next period (per-CPU) */
  hrtimer_set_expires(&ts->sched_timer, tick_init_jiffy_update());

  /* Offset the tick to avert jiffies_lock contention. */
  if (sched_skew_tick) {
    u64 offset = ktime_to_ns(tick_period) >> 1;
    do_div(offset, num_possible_cpus());
    offset *= smp_processor_id();
    hrtimer_add_expires_ns(&ts->sched_timer, offset);
  }

  hrtimer_forward(&ts->sched_timer, now, tick_period);
  hrtimer_start_expires(&ts->sched_timer, HRTIMER_MODE_ABS_PINNED);
  tick_nohz_activate(ts, NOHZ_MODE_HIGHRES);
}

static enum hrtimer_restart tick_sched_timer(struct hrtimer *timer)
{
  struct tick_sched *ts = container_of(timer, struct tick_sched, sched_timer);
  struct pt_regs *regs = get_irq_regs();
  ktime_t now = ktime_get();

  tick_sched_do_timer(ts, now);

  /* Do not call, when we are not in irq context and have
   * no valid regs pointer */
  if (regs)
    tick_sched_handle(ts, regs);
  else
    ts->next_tick = 0;

  /* No need to reprogram if we are in idle or full dynticks mode */
  if (unlikely(ts->tick_stopped))
    return HRTIMER_NORESTART;

  hrtimer_forward(timer, now, tick_period);

  return HRTIMER_RESTART;
}

static void tick_sched_do_timer(struct tick_sched *ts, ktime_t now)
{
  int cpu = smp_processor_id();

  if (tick_do_timer_cpu == cpu)
    tick_do_update_jiffies64(now);

  if (ts->inidle)
    ts->got_idle_tick = 1;
}

static void tick_sched_handle(struct tick_sched *ts, struct pt_regs *regs)
{
  update_process_times(user_mode(regs));
  profile_tick(CPU_PROFILING);
}
```

## HRTIMER_SOFTIRQ
```C++
DEFINE_PER_CPU(struct hrtimer_cpu_base, hrtimer_bases) =
{
  .lock = __RAW_SPIN_LOCK_UNLOCKED(hrtimer_bases.lock),
  .clock_base =
  {
    {
      .index = HRTIMER_BASE_MONOTONIC,
      .clockid = CLOCK_MONOTONIC,
      .get_time = &ktime_get,
    },
    {
      .index = HRTIMER_BASE_REALTIME,
      .clockid = CLOCK_REALTIME,
      .get_time = &ktime_get_real,
    },
    {
      .index = HRTIMER_BASE_BOOTTIME,
      .clockid = CLOCK_BOOTTIME,
      .get_time = &ktime_get_boottime,
    }
  }
};

/* open_softirq(HRTIMER_SOFTIRQ, hrtimer_run_softirq); */
static __latent_entropy void hrtimer_run_softirq(struct softirq_action *h)
{
  struct hrtimer_cpu_base *cpu_base = this_cpu_ptr(&hrtimer_bases);
  unsigned long flags;
  ktime_t now;

  raw_spin_lock_irqsave(&cpu_base->lock, flags);

  now = hrtimer_update_base(cpu_base);
  __hrtimer_run_queues(cpu_base, now, flags, HRTIMER_ACTIVE_SOFT);

  cpu_base->softirq_activated = 0;
  hrtimer_update_softirq_timer(cpu_base, true);

  raw_spin_unlock_irqrestore(&cpu_base->lock, flags);
}
```

## TIMER_SOFTIRQ
```C++
void run_timer_softirq(struct softirq_action *h)
{
  struct timer_base *base = this_cpu_ptr(&timer_bases[BASE_STD]);

  __run_timers(base);
  if (IS_ENABLED(CONFIG_NO_HZ_COMMON))
    __run_timers(this_cpu_ptr(&timer_bases[BASE_DEF]));
}

static inline void __run_timers(struct timer_base *base)
{
  struct hlist_head heads[LVL_DEPTH];
  int levels;

  if (!time_after_eq(jiffies, base->clk))
    return;

  raw_spin_lock_irq(&base->lock);

  base->must_forward_clk = false;

  while (time_after_eq(jiffies, base->clk)) {
    levels = collect_expired_timers(base, heads);
    base->clk++;
    while (levels--)
      expire_timers(base, heads + levels);
  }
  base->running_timer = NULL;
  raw_spin_unlock_irq(&base->lock);
}

static void expire_timers(struct timer_base *base, struct hlist_head *head)
{
  while (!hlist_empty(head)) {
    struct timer_list *timer;
    void (*fn)(struct timer_list *);

    timer = hlist_entry(head->first, struct timer_list, entry);

    base->running_timer = timer;
    detach_timer(timer, true);

    fn = timer->function;

    if (timer->flags & TIMER_IRQSAFE) {
      raw_spin_unlock(&base->lock);
      call_timer_fn(timer, fn);
      raw_spin_lock(&base->lock);
    } else {
      raw_spin_unlock_irq(&base->lock);
      call_timer_fn(timer, fn);
      raw_spin_lock_irq(&base->lock);
    }
  }
}

static void call_timer_fn(struct timer_list *timer, void (*fn)(struct timer_list *))
{
  int count = preempt_count();

#ifdef CONFIG_LOCKDEP
  struct lockdep_map lockdep_map;
  lockdep_copy_map(&lockdep_map, &timer->lockdep_map);
#endif

  lock_map_acquire(&lockdep_map);

  trace_timer_expire_entry(timer);
  fn(timer);
  trace_timer_expire_exit(timer);

  lock_map_release(&lockdep_map);

  if (count != preempt_count()) {
    WARN_ONCE(1, "timer: %pF preempt leak: %08x -> %08x\n",
        fn, count, preempt_count());
    preempt_count_set(count);
  }
}

void profile_tick(int type)
{
  struct pt_regs *regs = get_irq_regs();

  if (!user_mode(regs) && prof_cpu_mask != NULL &&
      cpumask_test_cpu(smp_processor_id(), prof_cpu_mask))
    profile_hit(type, (void *)profile_pc(regs));
}
```

## hrtimer_interrupt
```C++
void hrtimer_interrupt(struct clock_event_device *dev)
{
  struct hrtimer_cpu_base *cpu_base = this_cpu_ptr(&hrtimer_bases);
  ktime_t expires_next, now, entry_time, delta;
  unsigned long flags;
  int retries = 0;

  cpu_base->nr_events++;
  dev->next_event = KTIME_MAX;

  raw_spin_lock_irqsave(&cpu_base->lock, flags);
  entry_time = now = hrtimer_update_base(cpu_base);
retry:
  cpu_base->in_hrtirq = 1;
  cpu_base->expires_next = KTIME_MAX;

  if (!ktime_before(now, cpu_base->softirq_expires_next)) {
    cpu_base->softirq_expires_next = KTIME_MAX;
    cpu_base->softirq_activated = 1;
    raise_softirq_irqoff(HRTIMER_SOFTIRQ);
  }

  __hrtimer_run_queues(cpu_base, now, flags, HRTIMER_ACTIVE_HARD);

  /* Reevaluate the clock bases for the next expiry */
  expires_next = __hrtimer_get_next_event(cpu_base, HRTIMER_ACTIVE_ALL);

  cpu_base->expires_next = expires_next;
  cpu_base->in_hrtirq = 0;
  raw_spin_unlock_irqrestore(&cpu_base->lock, flags);

  /* Reprogramming necessary ? */
  if (!tick_program_event(expires_next, 0)) {
    cpu_base->hang_detected = 0;
    return;
  }

  /* The next timer was already expired due to:
   * - tracing
   * - long lasting callbacks
   * - being scheduled away when running in a VM
   *
   * We need to prevent that we loop forever in the hrtimer
   * interrupt routine. We give it 3 attempts to avoid
   * overreacting on some spurious event.
   *
   * Acquire base lock for updating the offsets and retrieving
   * the current time. */
  raw_spin_lock_irqsave(&cpu_base->lock, flags);
  now = hrtimer_update_base(cpu_base);
  cpu_base->nr_retries++;
  if (++retries < 3)
    goto retry;
  /* Give the system a chance to do something else than looping
   * here. We stored the entry time, so we know exactly how long
   * we spent here. We schedule the next event this amount of
   * time away. */
  cpu_base->nr_hangs++;
  cpu_base->hang_detected = 1;
  raw_spin_unlock_irqrestore(&cpu_base->lock, flags);

  delta = ktime_sub(now, entry_time);
  if ((unsigned int)delta > cpu_base->max_hang_time)
    cpu_base->max_hang_time = (unsigned int) delta;
  /* Limit it to a sensible value as we enforce a longer
   * delay. Give the CPU at least 100ms to catch up. */
  if (delta > 100 * NSEC_PER_MSEC)
    expires_next = ktime_add_ns(now, 100 * NSEC_PER_MSEC);
  else
    expires_next = ktime_add(now, delta);
  tick_program_event(expires_next, 1);
}
```
## API
### gettimeofday
```C++
SYSCALL_DEFINE2(gettimeofday, struct timeval __user *, tv,
    struct timezone __user *, tz)
{
  if (likely(tv != NULL)) {
    struct timespec64 ts;

    ktime_get_real_ts64(&ts);
    if (put_user(ts.tv_sec, &tv->tv_sec) ||
        put_user(ts.tv_nsec / 1000, &tv->tv_usec))
      return -EFAULT;
  }
  if (unlikely(tz != NULL)) {
    if (copy_to_user(tz, &sys_tz, sizeof(sys_tz)))
      return -EFAULT;
  }
  return 0;
}

void ktime_get_real_ts64(struct timespec64 *ts)
{
  struct timekeeper *tk = &tk_core.timekeeper;
  unsigned long seq;
  u64 nsecs;

  WARN_ON(timekeeping_suspended);

  do {
    seq = read_seqcount_begin(&tk_core.seq);

    ts->tv_sec = tk->xtime_sec;
    nsecs = timekeeping_get_ns(&tk->tkr_mono);

  } while (read_seqcount_retry(&tk_core.seq, seq));

  ts->tv_nsec = 0;
  timespec64_add_ns(ts, nsecs);
}

static struct {
  seqcount_t          seq;
  struct timekeeper   timekeeper;
} tk_core ____cacheline_aligned = {
  .seq = SEQCNT_ZERO(tk_core.seq),
};

typedef struct seqcount {
  unsigned sequence;
} seqcount_t;

struct timekeeper {
  struct tk_read_base  tkr_mono; /* Current CLOCK_REALTIME time in seconds */
  struct tk_read_base  tkr_raw; /* Current CLOCK_MONOTONIC time in seconds */
  u64                  xtime_sec;
  unsigned long        ktime_sec;
  struct timespec64    wall_to_monotonic;
  ktime_t      offs_real;
  ktime_t      offs_boot;
  ktime_t      offs_tai;
  s32          tai_offset;
  unsigned int    clock_was_set_seq;
  u8              cs_was_changed_seq;
  ktime_t         next_leap_ktime;
  u64             raw_sec;

  u64      cycle_interval;
  u64      xtime_interval;
  s64      xtime_remainder;
  u64      raw_interval;

  u64      ntp_tick;
  s64      ntp_error;
  u32      ntp_error_shift;
  u32      ntp_err_mult;
  u32      skip_second_overflow;
};
```

### timer_create
```C++
SYSCALL_DEFINE3(timer_create, const clockid_t, which_clock,
    struct sigevent __user *, timer_event_spec,
    timer_t __user *, created_timer_id)
{
  if (timer_event_spec) {
    sigevent_t event;

    if (copy_from_user(&event, timer_event_spec, sizeof (event)))
      return -EFAULT;
    return do_timer_create(which_clock, &event, created_timer_id);
  }
  return do_timer_create(which_clock, NULL, created_timer_id);
}

static int do_timer_create(clockid_t which_clock, struct sigevent *event,
         timer_t __user *created_timer_id)
{
  const struct k_clock *kc = clockid_to_kclock(which_clock);
  struct k_itimer *new_timer;
  int error, new_timer_id;
  int it_id_set = IT_ID_NOT_SET;

  if (!kc)
    return -EINVAL;
  if (!kc->timer_create)
    return -EOPNOTSUPP;

  new_timer = alloc_posix_timer();
  if (unlikely(!new_timer))
    return -EAGAIN;

  spin_lock_init(&new_timer->it_lock);
  new_timer_id = posix_timer_add(new_timer);

  it_id_set = IT_ID_SET;
  new_timer->it_id = (timer_t) new_timer_id;
  new_timer->it_clock = which_clock;
  new_timer->kclock = kc;
  new_timer->it_overrun = -1LL;

  if (event) {
    rcu_read_lock();
    new_timer->it_pid = get_pid(good_sigevent(event));
    rcu_read_unlock();
    if (!new_timer->it_pid) {
      error = -EINVAL;
      goto out;
    }
    new_timer->it_sigev_notify     = event->sigev_notify;
    new_timer->sigq->info.si_signo = event->sigev_signo;
    new_timer->sigq->info.si_value = event->sigev_value;
  } else {
    new_timer->it_sigev_notify     = SIGEV_SIGNAL;
    new_timer->sigq->info.si_signo = SIGALRM;
    memset(&new_timer->sigq->info.si_value, 0, sizeof(sigval_t));
    new_timer->sigq->info.si_value.sival_int = new_timer->it_id;
    new_timer->it_pid = get_pid(task_tgid(current));
  }

  new_timer->sigq->info.si_tid   = new_timer->it_id;
  new_timer->sigq->info.si_code  = SI_TIMER;

  if (copy_to_user(created_timer_id, &new_timer_id, sizeof (new_timer_id))) {
    error = -EFAULT;
    goto out;
  }

  error = kc->timer_create(new_timer);
  if (error)
    goto out;

  spin_lock_irq(&current->sighand->siglock);
  new_timer->it_signal = current->signal;
  list_add(&new_timer->list, &current->signal->posix_timers);
  spin_unlock_irq(&current->sighand->siglock);

  return 0;
  /*
   * In the case of the timer belonging to another task, after
   * the task is unlocked, the timer is owned by the other task
   * and may cease to exist at any time.  Don't use or modify
   * new_timer after the unlock call.
   */
out:
  release_posix_timer(new_timer, it_id_set);
  return error;
}

static int common_timer_create(struct k_itimer *new_timer)
{
  hrtimer_init(&new_timer->it.real.timer, new_timer->it_clock, 0);
  return 0;
}
```

### timer_settime
```C++
SYSCALL_DEFINE4(timer_settime, timer_t, timer_id, int, flags,
    const struct __kernel_itimerspec __user *, new_setting,
    struct __kernel_itimerspec __user *, old_setting)
{
  struct itimerspec64 new_spec, old_spec;
  struct itimerspec64 *rtn = old_setting ? &old_spec : NULL;
  int error = 0;

  get_itimerspec64(&new_spec, new_setting);

  error = do_timer_settime(timer_id, flags, &new_spec, rtn);
  if (!error && old_setting) {
    if (put_itimerspec64(&old_spec, old_setting))
      error = -EFAULT;
  }
  return error;
}

static int do_timer_settime(timer_t timer_id, int flags,
          struct itimerspec64 *new_spec64,
          struct itimerspec64 *old_spec64)
{
  const struct k_clock *kc;
  struct k_itimer *timr;
  unsigned long flag;
  int error = 0;

retry:
  timr = lock_timer(timer_id, &flag);
  kc = timr->kclock;
  error = kc->timer_set(timr, flags, new_spec64, old_spec64);

  return error;
}

static const struct k_clock * const posix_clocks[] = {
  [CLOCK_REALTIME]            = &clock_realtime,
  [CLOCK_MONOTONIC]           = &clock_monotonic,
  [CLOCK_PROCESS_CPUTIME_ID]  = &clock_process,
  [CLOCK_THREAD_CPUTIME_ID]   = &clock_thread,
  [CLOCK_MONOTONIC_RAW]       = &clock_monotonic_raw,
  [CLOCK_REALTIME_COARSE]     = &clock_realtime_coarse,
  [CLOCK_MONOTONIC_COARSE]    = &clock_monotonic_coarse,
  [CLOCK_BOOTTIME]            = &clock_boottime,
  [CLOCK_REALTIME_ALARM]      = &alarm_clock,
  [CLOCK_BOOTTIME_ALARM]      = &alarm_clock,
  [CLOCK_TAI]                 = &clock_tai,
};

static const struct k_clock clock_realtime = {
  .timer_set    = common_timer_set,
  .timer_arm    = common_hrtimer_arm,
};

int common_timer_set(struct k_itimer *timr, int flags,
         struct itimerspec64 *new_setting,
         struct itimerspec64 *old_setting)
{
  const struct k_clock *kc = timr->kclock;
  bool sigev_none;
  ktime_t expires;

  if (old_setting)
    common_timer_get(timr, old_setting);

  /* Prevent rearming by clearing the interval */
  timr->it_interval = 0;
  /* Careful here. On SMP systems the timer expiry function could be
   * active and spinning on timr->it_lock. */
  if (kc->timer_try_to_cancel(timr) < 0)
    return TIMER_RETRY;

  timr->it_active = 0;
  timr->it_requeue_pending = (timr->it_requeue_pending + 2) &
    ~REQUEUE_PENDING;
  timr->it_overrun_last = 0;

  /* Switch off the timer when it_value is zero */
  if (!new_setting->it_value.tv_sec && !new_setting->it_value.tv_nsec)
    return 0;

  timr->it_interval = timespec64_to_ktime(new_setting->it_interval);
  expires = timespec64_to_ktime(new_setting->it_value);
  sigev_none = timr->it_sigev_notify == SIGEV_NONE;

  kc->timer_arm(timr, expires, flags & TIMER_ABSTIME, sigev_none);
  timr->it_active = !sigev_none;
  return 0;
}

static void common_hrtimer_arm(struct k_itimer *timr, ktime_t expires,
             bool absolute, bool sigev_none)
{
  struct hrtimer *timer = &timr->it.real.timer;
  enum hrtimer_mode mode;

  mode = absolute ? HRTIMER_MODE_ABS : HRTIMER_MODE_REL;

  if (timr->it_clock == CLOCK_REALTIME)
    timr->kclock = absolute ? &clock_realtime : &clock_monotonic;

  hrtimer_init(&timr->it.real.timer, timr->it_clock, mode);
  timr->it.real.timer.function = posix_timer_fn;

  if (!absolute)
    expires = ktime_add_safe(expires, timer->base->get_time());
  hrtimer_set_expires(timer, expires);

  if (!sigev_none)
    hrtimer_start_expires(timer, HRTIMER_MODE_ABS);
}
```

### timer cancel
```C++
static int common_hrtimer_try_to_cancel(struct k_itimer *timr)
{
  return hrtimer_try_to_cancel(&timr->it.real.timer);
}

/* Returns:
 *  0 when the timer was not active
 *  1 when the timer was active
 * -1 when the timer is currently executing the callback function and
 *    cannot be stopped */
int hrtimer_try_to_cancel(struct hrtimer *timer)
{
  struct hrtimer_clock_base *base;
  unsigned long flags;
  int ret = -1;

  if (!hrtimer_active(timer))
    return 0;

  base = lock_hrtimer_base(timer, &flags);

  if (!hrtimer_callback_running(timer)) /* timer->base->running == timer; */
    ret = remove_hrtimer(timer, base, false);

  unlock_hrtimer_base(timer, &flags);

  return ret;
}

/*
 * A timer is active, when it is enqueued into the rbtree or the
 * callback function is running or it's in the state of being migrated
 * to another cpu.
 *
 * It is important for this function to not return a false negative. */
bool hrtimer_active(const struct hrtimer *timer)
{
  struct hrtimer_clock_base *base;
  unsigned int seq;

  do {
    base = READ_ONCE(timer->base);
    seq = raw_read_seqcount_begin(&base->seq);

    if (timer->state != HRTIMER_STATE_INACTIVE || base->running == timer)
      return true;

  } while (read_seqcount_retry(&base->seq, seq) || base != READ_ONCE(timer->base));

  return false;
}

int remove_hrtimer(struct hrtimer *timer, struct hrtimer_clock_base *base, bool restart)
{
  u8 state = timer->state;
  if (state & HRTIMER_STATE_ENQUEUED) {
    int reprogram = base->cpu_base == this_cpu_ptr(&hrtimer_bases);
    if (!restart)
      state = HRTIMER_STATE_INACTIVE;

    __remove_hrtimer(timer, base, state, reprogram);
    return 1;
  }
  return 0;
}

static void __remove_hrtimer(struct hrtimer *timer,
           struct hrtimer_clock_base *base,
           u8 newstate, int reprogram)
{
  struct hrtimer_cpu_base *cpu_base = base->cpu_base;
  u8 state = timer->state;

  /* Pairs with the lockless read in hrtimer_is_queued() */
  WRITE_ONCE(timer->state, newstate);
  if (!(state & HRTIMER_STATE_ENQUEUED))
    return;

  if (!timerqueue_del(&base->active, &timer->node))
    cpu_base->active_bases &= ~(1 << base->index);

  if (reprogram && timer == cpu_base->next_timer)
    hrtimer_force_reprogram(cpu_base, 1);
}
```

### posix_timer_fn
```C++
static enum hrtimer_restart posix_timer_fn(struct hrtimer *timer)
{
  struct k_itimer *timr;
  unsigned long flags;
  int si_private = 0;
  enum hrtimer_restart ret = HRTIMER_NORESTART;

  timr = container_of(timer, struct k_itimer, it.real.timer);
  spin_lock_irqsave(&timr->it_lock, flags);

  timr->it_active = 0;
  if (timr->it_interval != 0)
    si_private = ++timr->it_requeue_pending;

  if (posix_timer_event(timr, si_private)) {
    /* signal was not sent because of sig_ignor
     * we will not get a call back to restart it AND
     * it should be restarted. */
    if (timr->it_interval != 0) {
      ktime_t now = hrtimer_cb_get_time(timer);
#ifdef CONFIG_HIGH_RES_TIMERS
      {
        ktime_t kj = NSEC_PER_SEC / HZ;

        if (timr->it_interval < kj)
          now = ktime_add(now, kj);
      }
#endif
      timr->it_overrun += hrtimer_forward(timer, now,
                  timr->it_interval);
      ret = HRTIMER_RESTART;
      ++timr->it_requeue_pending;
      timr->it_active = 1;
    }
  }

  unlock_timer(timr, flags);
  return ret;
}

int posix_timer_event(struct k_itimer *timr, int si_private)
{
  enum pid_type type;
  int ret = -1;

  timr->sigq->info.si_sys_private = si_private;

  type = !(timr->it_sigev_notify & SIGEV_THREAD_ID) ? PIDTYPE_TGID : PIDTYPE_PID;
  ret = send_sigqueue(timr->sigq, timr->it_pid, type);
  return ret > 0;
}

int send_sigqueue(struct sigqueue *q, struct pid *pid, enum pid_type type)
{
  int sig = q->info.si_signo;
  struct sigpending *pending;
  struct task_struct *t;
  unsigned long flags;
  int ret, result;

  ret = -1;
  rcu_read_lock();
  t = pid_task(pid, type);

  ret = 1; /* the signal is ignored */
  result = TRACE_SIGNAL_IGNORED;
  if (!prepare_signal(sig, t, false))
    goto out;

  ret = 0;
  if (unlikely(!list_empty(&q->list))) {
    BUG_ON(q->info.si_code != SI_TIMER);
    q->info.si_overrun++;
    result = TRACE_SIGNAL_ALREADY_PENDING;
    goto out;
  }
  q->info.si_overrun = 0;

  signalfd_notify(t, sig);
  pending = (type != PIDTYPE_PID) ? &t->signal->shared_pending : &t->pending;
  list_add_tail(&q->list, &pending->list);
  sigaddset(&pending->signal, sig);
  complete_signal(sig, t, type);
  result = TRACE_SIGNAL_DELIVERED;
out:
  trace_signal_generate(sig, &q->info, t, type != PIDTYPE_PID, result);
  unlock_task_sighand(t, &flags);
ret:
  rcu_read_unlock();
  return ret;
}
```

## kernel timer api
```c++
#define timer_setup(timer, callback, flags) __init_timer((timer), (callback), (flags))
#define __init_timer(_timer, _fn, _flags)   init_timer_key((_timer), (_fn), (_flags), NULL, NULL)

void init_timer_key(struct timer_list *timer,
  void (*func)(struct timer_list *), unsigned int flags,
  const char *name, struct lock_class_key *key)
{
  debug_init(timer);
  do_init_timer(timer, func, flags, name, key);
}

static void do_init_timer(struct timer_list *timer,
  void (*func)(struct timer_list *),
  unsigned int flags,
  const char *name, struct lock_class_key *key)
{
  timer->entry.pprev = NULL;
  timer->function = func;
  timer->flags = flags | raw_smp_processor_id();
  lockdep_init_map(&timer->lockdep_map, name, key, 0);
}

void hrtimer_init(struct hrtimer *timer, clockid_t clock_id,
  enum hrtimer_mode mode)
{
  __hrtimer_init(timer, clock_id, mode);
}

static void __hrtimer_init(struct hrtimer *timer, clockid_t clock_id,
  enum hrtimer_mode mode)
{
  bool softtimer = !!(mode & HRTIMER_MODE_SOFT);
  int base = softtimer ? HRTIMER_MAX_CLOCK_BASES / 2 : 0;
  struct hrtimer_cpu_base *cpu_base;

  memset(timer, 0, sizeof(struct hrtimer));

  cpu_base = raw_cpu_ptr(&hrtimer_bases);

  /* POSIX magic: Relative CLOCK_REALTIME timers are not affected by
   * clock modifications, so they needs to become CLOCK_MONOTONIC to
   * ensure POSIX compliance. */
  if (clock_id == CLOCK_REALTIME && mode & HRTIMER_MODE_REL)
    clock_id = CLOCK_MONOTONIC;

  base += hrtimer_clockid_to_base(clock_id);
  timer->is_soft = softtimer;
  timer->base = &cpu_base->clock_base[base];
  timerqueue_init(&timer->node);
}

static inline void timerqueue_init(struct timerqueue_node *node)
{
  RB_CLEAR_NODE(&node->node);
}

int del_timer(struct timer_list *timer)
{
  struct timer_base *base;
  unsigned long flags;
  int ret = 0;

  if (timer_pending(timer)) {
    base = lock_timer_base(timer, &flags);
    ret = detach_if_pending(timer, base, true);
    raw_spin_unlock_irqrestore(&base->lock, flags);
  }

  return ret;
}
```

## Call Stack
```C++
start_kernel();
  tick_init();
    tick_broadcast_init();
    tick_nohz_init();

  init_timers();
    init_timer_cpus();
    open_softirq(TIMER_SOFTIRQ, run_timer_softirq);

  hrtimers_init();
    hrtimers_prepare_cpu(smp_processor_id());
    open_softirq(HRTIMER_SOFTIRQ, hrtimer_run_softirq);

  timekeeping_init();

  time_init();
    x86_late_time_init();
      x86_init.timers.timer_init(); /* hpet_time_init */
      x86_init.irqs.intr_mode_init();
      tsc_init();

hpet_time_init();
  hpet_enable();
    hpet_set_mapping();

    hpet_clocksource_register();
      clocksource_register_hz();
        __clocksource_register_scale();
          clocksource_enqueue();
          clocksource_select();

    hpet_legacy_clockevent_register();
      clockevents_config_and_register();
        clockevents_config();
        clockevents_register_device();
          list_add(&dev->list, &clockevent_devices);
          tick_check_new_device();
            tick_setup_device();
              tick_setup_periodic();
                tick_set_periodic_handler();
                  dev->event_handler = tick_handle_periodic;
                tick_setup_oneshot();
                  clockevents_program_event(newdev, next_event, true);

  setup_default_timer_irq();
    setup_irq(0, &irq0);

setup_APIC_timer();
  clockevents_config_and_register();

tick_handle_periodic();
  tick_periodic();
    do_timer();
      jiffies_64 += ticks;

    update_wall_time();
      timekeeping_advance();

    update_process_times();
      run_local_timers();

        hrtimer_run_queues();
          tick_check_oneshot_change();
            hrtimer_switch_to_hres();
              tick_setup_sched_timer(); /* hrtimer tick emulation */
                hrtimer_init(&ts->sched_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS);
                ts->sched_timer.function = tick_sched_timer;
                  tick_sched_timer();
                    tick_sched_do_timer();
                      tick_do_update_jiffies64(now)
                        do_timer();
                        update_wall_time();
                    tick_sched_handle();
                      update_process_times();
          !ktime_before(now, cpu_base->softirq_expires_next)
            raise_softirq_irqoff(HRTIMER_SOFTIRQ);
              hrtimer_run_softirq();
                __hrtimer_run_queues();
          __hrtimer_run_queues();
            __run_hrtimer();

        raise_softirq(TIMER_SOFTIRQ);
          run_timer_softirq();
            __run_timers();
              collect_expired_timers();
              expire_timers();
                call_timer_fn();

      scheduler_tick();
        curr->sched_class->task_tick(rq, curr, 0);

hrtimer_interrupt();
  __hrtimer_run_queues();
  __hrtimer_get_next_event();
  tick_program_event();


timer_create();
  do_timer_create();
    clockid_to_kclock();
    alloc_posix_timer();
    posix_timer_add();
    kc->timer_create();
      common_timer_create();
        hrtimer_init()
    list_add(&new_timer->list, &current->signal->posix_timers);


timer_settime();
  do_timer_settime();
    kc->timer_set();
      common_timer_set();
        timer_try_to_cancel();
        kc->timer_arm();
          common_hrtimer_arm();
            hrtimer_init(&timr->it.real.timer, timr->it_clock, mode);
            timr->it.real.timer.function = posix_timer_fn;
            hrtimer_set_expires();

posix_timer_fn();
  posix_timer_event();
    send_sigqueue();
      signalfd_notify();
      list_add_tail(&q->list, &pending->list);
      complete_signal(sig, t, type);
        signal_wake_up();
          signal_wake_up_state();
            wake_up_state();
              try_to_wake_up();
```

![kernel-time-timer.png](../Images/Kernel/time-timer.png)

![kernel-time-timer-arch.png](../Images/Kernel/time-timer-arch.png)

![kernel-time-tiemer-origin.png](../Images/Kernel/time-tiemer-origin.png)

![kernel-time-timer-hrtimer.png](../Images/Kernel/time-timer-hrtimer.png)

![kernel-time-timer-hrtimer-gtod.png](../Images/Kernel/time-timer-hrtimer-gtod.png)

![kernel-time-timer-hrtimer-gtod-clockevent.png](../Images/Kernel/time-timer-hrtimer-gtod-clockevent.png)

![kernel-time-timer-hrtimer-gtod-clockevent-tick-emulation.png](../Images/Kernel/time-timer-hrtimer-gtod-clockevent-tick-emulation.png)

## Refence:
[hrtimers and beyond](http://www.cs.columbia.edu/~nahum/w6998/papers/ols2006-hrtimers-slides.pdf)

http://www.wowotech.net/timer_subsystem/time-subsyste-architecture.html

# Lock
### spin lock
```C++
typedef struct spinlock {
  union {
    struct raw_spinlock rlock;

#ifdef CONFIG_DEBUG_LOCK_ALLOC
# define LOCK_PADSIZE (offsetof(struct raw_spinlock, dep_map))
    struct {
      u8 __padding[LOCK_PADSIZE];
      struct lockdep_map dep_map;
    };
#endif
  };
} spinlock_t;


typedef struct raw_spinlock {
  arch_spinlock_t raw_lock;

#ifdef CONFIG_DEBUG_SPINLOCK
  unsigned int magic, owner_cpu;
  void *owner;
#endif
#ifdef CONFIG_DEBUG_LOCK_ALLOC
  struct lockdep_map dep_map;
#endif
} raw_spinlock_t;


typedef struct {
  volatile unsigned int slock;
} arch_spinlock_t;

typedef struct {
  union {
    u32 slock;
    struct __raw_tickets {
#ifdef __ARMEB__
      u16 next;
      u16 owner;
#else
      u16 owner;
      u16 next;
#endif
    } tickets;
  };
} arch_spinlock_t;

/* include/asm-generic/qspinlock_types.h */
typedef struct qspinlock {
  union {
    atomic_t val;

    /*
     * By using the whole 2nd least significant byte for the
     * pending bit, we can allow better optimization of the lock
     * acquisition for the pending bit holder.
     */
#ifdef __LITTLE_ENDIAN
    struct {
      u8  locked;
      u8  pending;
    };
    struct {
      u16  locked_pending;
      u16  tail;
    };
#else
    struct {
      u16  tail;
      u16  locked_pending;
    };
    struct {
      u8  reserved[2];
      u8  pending;
      u8  locked;
    };
#endif
  };
} arch_spinlock_t;

/* include/linux/spinlock.h */
static  void spin_lock(spinlock_t *lock)
{
  raw_spin_lock(&lock->rlock);
}

#define raw_spin_lock(lock)  _raw_spin_lock(lock)
#define raw_spin_trylock(lock)  __cond_lock(lock, _raw_spin_trylock(lock))

/* include/linux/spinlock_api_smp.h */
#define _raw_spin_lock(lock) __raw_spin_lock(lock)

static inline void __raw_spin_lock(raw_spinlock_t *lock)
{
  preempt_disable();
  /* for lockdep debug */
  spin_acquire(&lock->dep_map, 0, 0, _RET_IP_);

  LOCK_CONTENDED(lock, do_raw_spin_trylock, do_raw_spin_lock);
}

/*include/linux/lockdep.h */
#define spin_acquire(l, s, t, i) lock_acquire_exclusive(l, s, t, NULL, i)
#define lock_acquire_exclusive(l, s, t, n, i) lock_acquire(l, s, t, 0, 1, n, i)
#define lock_acquire(l, s, t, r, c, n, i)  do { } while (0)


/* include/linux/lockdep.h */
#define LOCK_CONTENDED(_lock, try, lock)      \
do {                \
  if (!try(_lock)) {          \
    lock_contended(&(_lock)->dep_map, _RET_IP_);  \
    lock(_lock);          \
  }              \
  lock_acquired(&(_lock)->dep_map, _RET_IP_);      \
} while (0)


/* include/linux/spinlock.h */
static inline int do_raw_spin_trylock(raw_spinlock_t *lock)
{
  return arch_spin_trylock(&(lock)->raw_lock);
}

/* kernel/locking/spinlock_debug.c */
void do_raw_spin_lock(raw_spinlock_t *lock)
{
  debug_spin_lock_before(lock);
  arch_spin_lock(&lock->raw_lock);
  debug_spin_lock_after(lock);
}

#define arch_spin_lock(l)    queued_spin_lock(l)

static  void queued_spin_lock(struct qspinlock *lock)
{
  u32 val;

  val = atomic_cmpxchg_acquire(&lock->val, 0, _Q_LOCKED_VAL);
  if (likely(val == 0))
    return;
  queued_spin_lock_slowpath(lock, val);
}

#define  atomic_cmpxchg_acquire    atomic_cmpxchg

static  int atomic_cmpxchg(atomic_t *v, int old, int new)
{
  kasan_check_write(v, sizeof(*v));
  return arch_atomic_cmpxchg(v, old, new);
}

static  int arch_atomic_cmpxchg(atomic_t *v, int old, int new)
{
  return arch_cmpxchg(&v->counter, old, new);
}

/* arch/x86/include/asm/cmpxchg.h */
#define arch_cmpxchg(ptr, old, new)          \
  __cmpxchg(ptr, old, new, sizeof(*(ptr)))

#define __cmpxchg(ptr, old, new, size)          \
  __raw_cmpxchg((ptr), (old), (new), (size), LOCK_PREFIX)

#define __raw_cmpxchg(ptr, old, new, size, lock)      \
({                  \
  __typeof__(*(ptr)) __ret;          \
  __typeof__(*(ptr)) __old = (old);        \
  __typeof__(*(ptr)) __new = (new);        \
  switch (size) {              \
  case __X86_CASE_W:            \
  {                \
    volatile u16 *__ptr = (volatile u16 *)(ptr);    \
    asm volatile(lock "cmpxchgw %2,%1"      \
           : "=a" (__ret), "+m" (*__ptr)    \
           : "r" (__new), "0" (__old)      \
           : "memory");        \
    break;              \
  }                \
  default:              \
    __cmpxchg_wrong_size();          \
  }                \
  __ret;                \
})
```

# Pthread

### pthread_create
```C++
int __pthread_create_2_1 (
  pthread_t *newthread, const pthread_attr_t *attr,
  void *(*start_routine) (void *), void *arg)
{
  const struct pthread_attr *iattr = (struct pthread_attr *) attr;
  struct pthread_attr default_attr;
  if (iattr == NULL)
  {
    iattr = &default_attr;
  }

  struct pthread *pd = NULL;
  int err = ALLOCATE_STACK (iattr, &pd);

  pd->start_routine = start_routine;
  pd->arg = arg;
  pd->schedpolicy = self->schedpolicy;
  pd->schedparam = self->schedparam;

  *newthread = (pthread_t) pd;
  atomic_increment (&__nptl_nthreads);

  return create_thread(pd, iattr, &stopped_start,
    STACK_VARIABLES_ARGS, &thread_ran);
}
versioned_symbol(libpthread, __pthread_create_2_1, pthread_create, GLIBC_2_1);

# define ALLOCATE_STACK_PARMS void **stack, size_t *stacksize

# define ALLOCATE_STACK(attr, pd) allocate_stack (attr, pd, &stackaddr)

static int allocate_stack (
  const struct pthread_attr *attr,
  struct pthread **pdp, ALLOCATE_STACK_PARMS)
{
  struct pthread *pd;
  size_t size;
  size_t pagesize_m1 = __getpagesize () - 1;

  size = attr->stacksize;

  /* Allocate some anonymous memory.  If possible use the cache.  */
  size_t guardsize;
  void *mem;
  const int prot = (PROT_READ | PROT_WRITE
                   | ((GL(dl_stack_flags) & PF_X) ? PROT_EXEC : 0));
  /* Adjust the stack size for alignment.  */
  size &= ~__static_tls_align_m1;
  /* Make sure the size of the stack is enough for the guard and
  eventually the thread descriptor.  */
  guardsize = (attr->guardsize + pagesize_m1) & ~pagesize_m1;
  size += guardsize;
  pd = get_cached_stack (&size, &mem);
  if (pd == NULL)
  {
    /* If a guard page is required, avoid committing memory by first
    allocate with PROT_NONE and then reserve with required permission
    excluding the guard page.  */
    mem = __mmap (NULL, size, (guardsize == 0) ? prot : PROT_NONE,
      MAP_PRIVATE | MAP_ANONYMOUS | MAP_STACK, -1, 0);
    /* Place the thread descriptor at the end of the stack.  */
#if TLS_TCB_AT_TP
    pd = (struct pthread *) ((char *) mem + size) - 1;
#elif TLS_DTV_AT_TP
    pd = (struct pthread *) ((((uintptr_t) mem + size - __static_tls_size)
      & ~__static_tls_align_m1) - TLS_PRE_TCB_SIZE);
#endif
    /* Now mprotect the required region excluding the guard area. */
    char *guard = guard_position(mem, size, guardsize, pd, pagesize_m1);
    setup_stack_prot(mem, size, guard, guardsize, prot);

    pd->stackblock = mem;
    pd->stackblock_size = size;
    pd->guardsize = guardsize;
    pd->specific[0] = pd->specific_1stblock;

    stack_list_add (&pd->list, &stack_used);
  }

  *pdp = pd;
  void *stacktop;
# if TLS_TCB_AT_TP
  /* The stack begins before the TCB and the static TLS block.  */
  stacktop = ((char *) (pd + 1) - __static_tls_size);
# elif TLS_DTV_AT_TP
  stacktop = (char *) (pd - 1);
# endif
  *stack = stacktop;
}

# define STACK_VARIABLES_PARMS void *stackaddr, size_t stacksize
# define STACK_VARIABLES_ARGS stackaddr, stacksize

static int create_thread (
  struct pthread *pd, const struct pthread_attr *attr,
  bool *stopped_start, STACK_VARIABLES_PARMS, bool *thread_ran)
{
  const int clone_flags = (CLONE_VM | CLONE_FS | CLONE_FILES
    | CLONE_SYSVSEM | CLONE_SIGHAND | CLONE_THREAD | CLONE_SETTLS
    | CLONE_PARENT_SETTID | CLONE_CHILD_CLEARTID | 0);

  ARCH_CLONE (&start_thread, STACK_VARIABLES_ARGS, clone_flags, pd, &pd->tid, tp, &pd->tid)；
  /* It's started now, so if we fail below, we'll have to cancel it
and let it clean itself up.  */
  *thread_ran = true;
}


# define ARCH_CLONE __clone
/* The userland implementation is:
   int clone (int (*fn)(void *arg), void *child_stack, int flags, void *arg),
   the kernel entry is:
   int clone (long flags, void *child_stack).

   The parameters are passed in register and on the stack from userland:
   rdi: fn
   rsi: child_stack
   rdx: flags
   rcx: arg
   r8d: TID field in parent
   r9d: thread pointer
%esp+8: TID field in child

   The kernel expects:
   rax: system call number
   rdi: flags
   rsi: child_stack
   rdx: TID field in parent
   r10: TID field in child
   r8:  thread pointer  */

ENTRY (__clone)
  movq    $-EINVAL,%rax
  /* Insert the argument onto the new stack.  */
  subq    $16,%rsi
  movq    %rcx,8(%rsi)

  /* Save the function pointer.  It will be popped off in the
      child in the ebx frobbing below. */
  movq    %rdi,0(%rsi)

  /* Do the system call.  */
  movq    %rdx, %rdi
  movq    %r8, %rdx
  movq    %r9, %r8
  mov     8(%rsp), %R10_LP
  movl    $SYS_ify(clone),%eax

  syscall
PSEUDO_END (__clone)

SYSCALL_DEFINE5(clone, unsigned long, clone_flags,
  unsigned long, newsp,
  int __user *, parent_tidptr,
  int __user *, child_tidptr,
  unsigned long, tls)
{
  return _do_fork(clone_flags, newsp, 0, parent_tidptr, child_tidptr, tls);
}

#define THREAD_SETMEM(descr, member, value) \
  descr->member = (value)

#define START_THREAD_DEFN \
  static int __attribute__ ((noreturn)) start_thread (void *arg)
START_THREAD_DEFN
{
    struct pthread *pd = START_THREAD_SELF;
    /* Run the code the user provided.  */
    THREAD_SETMEM (pd, result, pd->start_routine (pd->arg));
    /* Call destructors for the thread_local TLS variables.  */
    /* Run the destructor for the thread-local data.  */
    __nptl_deallocate_tsd ();
    if (__glibc_unlikely (atomic_decrement_and_test (&__nptl_nthreads)))
        /* This was the last thread.  */
        exit (0);
    __free_tcb (pd);
    __exit_thread ();
}

void __free_tcb (struct pthread *pd)
{
  __deallocate_stack (pd);
}

void __deallocate_stack (struct pthread *pd)
{
  /* Remove the thread from the list of threads with user defined
     stacks.  */
  stack_list_del (&pd->list);
  /* Not much to do.  Just free the mmap()ed memory.  Note that we do
     not reset the 'used' flag in the 'tid' field.  This is done by
     the kernel.  If no thread has been created yet this field is
     still zero.  */
  if (__glibc_likely (! pd->user_stack))
    (void) queue_stack (pd);
}
```

### pthread_mutex
#### lock

```C++
SYSCALL_DEFINE6(futex, u32 __user *, uaddr, int, op, u32, val,
    struct timespec __user *, utime, u32 __user *, uaddr2,
    u32, val3)
{
  struct timespec ts;
  ktime_t t, *tp = NULL;
  u32 val2 = 0;
  int cmd = op & FUTEX_CMD_MASK;

  if (utime && (cmd == FUTEX_WAIT || cmd == FUTEX_LOCK_PI ||
          cmd == FUTEX_WAIT_BITSET ||
          cmd == FUTEX_WAIT_REQUEUE_PI)) {
    if (unlikely(should_fail_futex(!(op & FUTEX_PRIVATE_FLAG))))
      return -EFAULT;
    if (copy_from_user(&ts, utime, sizeof(ts)) != 0)
      return -EFAULT;
    if (!timespec_valid(&ts))
      return -EINVAL;

    t = timespec_to_ktime(ts);
    if (cmd == FUTEX_WAIT)
      t = ktime_add_safe(ktime_get(), t);
    tp = &t;
  }
  /*
   * requeue parameter in 'utime' if cmd == FUTEX_*_REQUEUE_*.
   * number of waiters to wake in 'utime' if cmd == FUTEX_WAKE_OP.
   */
  if (cmd == FUTEX_REQUEUE || cmd == FUTEX_CMP_REQUEUE ||
      cmd == FUTEX_CMP_REQUEUE_PI || cmd == FUTEX_WAKE_OP)
    val2 = (u32) (unsigned long) utime;

  return do_futex(uaddr, op, val, tp, uaddr2, val2, val3);
}

long do_futex(u32 __user *uaddr, int op, u32 val, ktime_t *timeout,
    u32 __user *uaddr2, u32 val2, u32 val3)
{
  int cmd = op & FUTEX_CMD_MASK;
  unsigned int flags = 0;

  if (!(op & FUTEX_PRIVATE_FLAG))
    flags |= FLAGS_SHARED;

  if (op & FUTEX_CLOCK_REALTIME) {
    flags |= FLAGS_CLOCKRT;
    if (cmd != FUTEX_WAIT && cmd != FUTEX_WAIT_BITSET && \
        cmd != FUTEX_WAIT_REQUEUE_PI)
      return -ENOSYS;
  }

  switch (cmd) {
  case FUTEX_LOCK_PI:
  case FUTEX_UNLOCK_PI:
  case FUTEX_TRYLOCK_PI:
  case FUTEX_WAIT_REQUEUE_PI:
  case FUTEX_CMP_REQUEUE_PI:
    if (!futex_cmpxchg_enabled)
      return -ENOSYS;
  }

  switch (cmd) {
  case FUTEX_WAIT:
    val3 = FUTEX_BITSET_MATCH_ANY;
    /* fall through */
  case FUTEX_WAIT_BITSET:
    return futex_wait(uaddr, flags, val, timeout, val3);
  case FUTEX_WAKE:
    val3 = FUTEX_BITSET_MATCH_ANY;
    /* fall through */
  case FUTEX_WAKE_BITSET:
    return futex_wake(uaddr, flags, val, val3);
  case FUTEX_REQUEUE:
    return futex_requeue(uaddr, flags, uaddr2, val, val2, NULL, 0);
  case FUTEX_CMP_REQUEUE:
    return futex_requeue(uaddr, flags, uaddr2, val, val2, &val3, 0);
  case FUTEX_WAKE_OP:
    return futex_wake_op(uaddr, flags, uaddr2, val, val2, val3);
  case FUTEX_LOCK_PI:
    return futex_lock_pi(uaddr, flags, timeout, 0);
  case FUTEX_UNLOCK_PI:
    return futex_unlock_pi(uaddr, flags);
  case FUTEX_TRYLOCK_PI:
    return futex_lock_pi(uaddr, flags, NULL, 1);
  case FUTEX_WAIT_REQUEUE_PI:
    val3 = FUTEX_BITSET_MATCH_ANY;
    return futex_wait_requeue_pi(uaddr, flags, val, timeout, val3,
               uaddr2);
  case FUTEX_CMP_REQUEUE_PI:
    return futex_requeue(uaddr, flags, uaddr2, val, val2, &val3, 1);
  }
  return -ENOSYS;
}
```
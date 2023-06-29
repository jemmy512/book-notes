# pipe
```c
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
  .name     = "pipefs",
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
<img src='../Images/Kernel/ipc-pipe-2.png' style='max-height:850px'/>

# fifo
```c
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
            type, nblocks) \
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
<img src='../Images/Kernel/ipc-fifo.png' style='max-height:850px'/>

# signal
## resigter a sighand
```c
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
<img src='../Images/Kernel/ipc-signal-register-handler.png' style='max-height:850px'/>

## send a signal
```c
group_send_sig_info()
    check_kill_permission(sig, info, p);
    do_send_sig_info()
        q = __sigqueue_alloc()
        signalfd_notify(t, sig);
        sigaddset(&pending->signal, sig);

        complete_signal()
            signal_wake_up(t, sig == SIGKILL)
                try_to_wake_up(p, state, 0);

```

```c
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
* [try_to_wake_up](#ttwu)

## handle signal
```c
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
asm \
  ( \
   ".LSTART_" #name ":\n" \
   "    .type __" #name ",@function\n" \
   "__" #name ":\n" \
   "    movq $" #syscall ", %rax\n" \
   "    syscall\n" \


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
<img src='../Images/Kernel/sig-handle.png' style='max-height:850px'/>

# sem, shm, msg
```c
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
<img src='../Images/Kernel/ipc-ipc_ids.png' style='max-height:850px'/>
```c
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

## shmget
```c
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
  error = shmem_init_inodecache(); /* shmem means shmem fs */
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
  /* get virtual address of a page from `page_address_htable` */
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

## shmat
```c
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

## shm_fault
```c
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

```c
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

<img src='../Images/Kernel/ipc-shm.png' style='max-height:850px'/>

## semget
```c
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

## semctl
```c
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

## semop
```c
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

<img src='../Images/Kernel/ipc-sem-2.png' style='max-height:850px'/>

```c
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
<img src='../Images/Kernel/ipc-sem.png' style='max-height:850px'/>

# Q:
1. How access shm by a vm address?

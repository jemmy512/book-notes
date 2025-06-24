
* [pipe](#pipe)
* [fifo](#fifo)
* [signal](#signal)
    * [sigaction](#sigaction)
    * [do_send_sig_info](#do_send_sig_info)
    * [do_signal](#do_signal)
        * [get_signal](#get_signal)
        * [rt_sigreturn](#rt_sigreturn)

* [posix_shmem](#posix_shmem)

* [shmem_fs](#shmem_fs)
    * [shmem_init](#shmem_init)
    * [shmem_open](#shmem_open)
    * [shmem_close](#shmem_close)
    * [shmem_fault](#shmem_fault)

* [posix_sem](#posix_sem)
    * [sem_open](#sem_open)
    * [sem_post](#sem_post)
    * [sem_wait](#sem_wait)

* [sysv_ipc](#sysv_ipc)
    * [sysv_shm](#sysv_shm)
        * [shmget](#shmget)
        * [shmat](#shmat)
        * [shm_ops](#shm_ops)
        * [shm_open](#shm_open)
        * [shm_close](#shm_close)
        * [shm_fault](#shm_fault)
    * [sysv_sem](#sysv_shm)
        * [semget](#semget)
        * [semctl](#semctl)
        * [semop](#semop)


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
    struct inode *inode = get_pipe_inode() {
        struct pipe_inode_info *pipe = alloc_pipe_info() {
            struct pipe_inode_info *pipe;
            unsigned long pipe_bufs = PIPE_DEF_BUFFERS;
            struct user_struct *user = get_current_user();
            unsigned long user_bufs;
            unsigned int max_size = READ_ONCE(pipe_max_size);

            pipe = kzalloc(sizeof(struct pipe_inode_info), GFP_KERNEL_ACCOUNT);
            if (pipe == NULL)
                goto out_free_uid;

            if (pipe_bufs * PAGE_SIZE > max_size && !capable(CAP_SYS_RESOURCE))
                pipe_bufs = max_size >> PAGE_SHIFT;

            user_bufs = account_pipe_buffers(user, 0, pipe_bufs);

            if (too_many_pipe_buffers_soft(user_bufs) && pipe_is_unprivileged_user()) {
                user_bufs = account_pipe_buffers(user, pipe_bufs, PIPE_MIN_DEF_BUFFERS);
                pipe_bufs = PIPE_MIN_DEF_BUFFERS;
            }

            if (too_many_pipe_buffers_hard(user_bufs) && pipe_is_unprivileged_user())
                goto out_revert_acct;

            pipe->bufs = kcalloc(pipe_bufs, sizeof(struct pipe_buffer),
                        GFP_KERNEL_ACCOUNT);

            if (pipe->bufs) {
                init_waitqueue_head(&pipe->rd_wait);
                init_waitqueue_head(&pipe->wr_wait);
                pipe->r_counter = pipe->w_counter = 1;
                pipe->max_usage = pipe_bufs;
                pipe->ring_size = pipe_bufs;
                pipe->nr_accounted = pipe_bufs;
                pipe->user = user;
                mutex_init(&pipe->mutex);
                lock_set_cmp_fn(&pipe->mutex, pipe_lock_cmp_fn, NULL);
                return pipe;
            }
        }

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

const struct file_operations pipefifo_fops = {
    .open           = fifo_open,
    .read_iter      = fifo_pipe_read,
    .write_iter     = fifo_pipe_write,
    .poll           = pipe_poll,
    .unlocked_ioctl = pipe_ioctl,
    .release        = pipe_release,
    .fasync         = pipe_fasync,
    .splice_write   = iter_file_splice_write,
};

static int __init init_pipe_fs(void)
{
    int err = register_filesystem(&pipe_fs_type);

    if (!err) {
        pipe_mnt = kern_mount(&pipe_fs_type);
    }
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
<img src='../images/kernel/ipc-pipe-2.png' style='max-height:850px'/>

# fifo

<img src='../images/kernel/ipc-fifo.svg' style='max-height:850px'/>

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

## fifo_pipe_write

```c
static ssize_t
fifo_pipe_write(struct kiocb *iocb, struct iov_iter *from)
{
    int ret = anon_pipe_write(iocb, from);
    if (ret > 0) {
        struct file *filp = iocb->ki_filp;
        if (sb_start_write_trylock(file_inode(filp)->i_sb)) {
            int err = file_update_time(filp);
            if (err)
                ret = err;
            sb_end_write(file_inode(filp)->i_sb);
        }
    }
    return ret;
}

static ssize_t
anon_pipe_write(struct kiocb *iocb, struct iov_iter *from)
{
    struct file *filp = iocb->ki_filp;
    struct pipe_inode_info *pipe = filp->private_data;
    unsigned int head;
    ssize_t ret = 0;
    size_t total_len = iov_iter_count(from);
    ssize_t chars;
    bool was_empty = false;
    bool wake_next_writer = false;

    /*
    * Reject writing to watch queue pipes before the point where we lock
    * the pipe.
    * Otherwise, lockdep would be unhappy if the caller already has another
    * pipe locked.
    * If we had to support locking a normal pipe and a notification pipe at
    * the same time, we could set up lockdep annotations for that, but
    * since we don't actually need that, it's simpler to just bail here.
    */
    if (pipe_has_watch_queue(pipe))
        return -EXDEV;

    /* Null write succeeds. */
    if (unlikely(total_len == 0))
        return 0;

    mutex_lock(&pipe->mutex);

    if (!pipe->readers) {
        send_sig(SIGPIPE, current, 0);
        ret = -EPIPE;
        goto out;
    }

    /*
    * If it wasn't empty we try to merge new data into
    * the last buffer.
    *
    * That naturally merges small writes, but it also
    * page-aligns the rest of the writes for large writes
    * spanning multiple pages.
    */
    head = pipe->head;
    was_empty = pipe_empty(head, pipe->tail);
    chars = total_len & (PAGE_SIZE-1);
    if (chars && !was_empty) {
        struct pipe_buffer *buf = pipe_buf(pipe, head - 1);
        int offset = buf->offset + buf->len;

        if ((buf->flags & PIPE_BUF_FLAG_CAN_MERGE) &&
            offset + chars <= PAGE_SIZE) {
            ret = pipe_buf_confirm(pipe, buf);
            if (ret)
                goto out;

            ret = copy_page_from_iter(buf->page, offset, chars, from);
            if (unlikely(ret < chars)) {
                ret = -EFAULT;
                goto out;
            }

            buf->len += ret;
            if (!iov_iter_count(from))
                goto out;
        }
    }

    for (;;) {
        if (!pipe->readers) {
            send_sig(SIGPIPE, current, 0);
            if (!ret)
                ret = -EPIPE;
            break;
        }

        head = pipe->head;
        if (!pipe_full(head, pipe->tail, pipe->max_usage)) {
            struct pipe_buffer *buf;
            struct page *page;
            int copied;

            page = anon_pipe_get_page(pipe);
            if (unlikely(!page)) {
                if (!ret)
                    ret = -ENOMEM;
                break;
            }

            copied = copy_page_from_iter(page, 0, PAGE_SIZE, from);
            if (unlikely(copied < PAGE_SIZE && iov_iter_count(from))) {
                anon_pipe_put_page(pipe, page);
                if (!ret)
                    ret = -EFAULT;
                break;
            }

            pipe->head = head + 1;
            /* Insert it into the buffer array */
            buf = pipe_buf(pipe, head);
            buf->page = page;
            buf->ops = &anon_pipe_buf_ops;
            buf->offset = 0;
            if (is_packetized(filp))
                buf->flags = PIPE_BUF_FLAG_PACKET;
            else
                buf->flags = PIPE_BUF_FLAG_CAN_MERGE;

            buf->len = copied;
            ret += copied;

            if (!iov_iter_count(from))
                break;

            continue;
        }

        /* Wait for buffer space to become available. */
        if ((filp->f_flags & O_NONBLOCK) ||
            (iocb->ki_flags & IOCB_NOWAIT)) {
            if (!ret)
                ret = -EAGAIN;
            break;
        }
        if (signal_pending(current)) {
            if (!ret)
                ret = -ERESTARTSYS;
            break;
        }

        /*
        * We're going to release the pipe lock and wait for more
        * space. We wake up any readers if necessary, and then
        * after waiting we need to re-check whether the pipe
        * become empty while we dropped the lock.
        */
        mutex_unlock(&pipe->mutex);
        if (was_empty)
            wake_up_interruptible_sync_poll(&pipe->rd_wait, EPOLLIN | EPOLLRDNORM);
        kill_fasync(&pipe->fasync_readers, SIGIO, POLL_IN);
        wait_event_interruptible_exclusive(pipe->wr_wait, pipe_writable(pipe));
        mutex_lock(&pipe->mutex);
        was_empty = pipe_is_empty(pipe);
        wake_next_writer = true;
    }
out:
    if (pipe_is_full(pipe))
        wake_next_writer = false;
    mutex_unlock(&pipe->mutex);

    /*
    * If we do do a wakeup event, we do a 'sync' wakeup, because we
    * want the reader to start processing things asap, rather than
    * leave the data pending.
    *
    * This is particularly important for small writes, because of
    * how (for example) the GNU make jobserver uses small writes to
    * wake up pending jobs
    *
    * Epoll nonsensically wants a wakeup whether the pipe
    * was already empty or not.
    */
    if (was_empty || pipe->poll_usage)
        wake_up_interruptible_sync_poll(&pipe->rd_wait, EPOLLIN | EPOLLRDNORM);
    kill_fasync(&pipe->fasync_readers, SIGIO, POLL_IN);
    if (wake_next_writer)
        wake_up_interruptible_sync_poll(&pipe->wr_wait, EPOLLOUT | EPOLLWRNORM);
    return ret;
}
```

# signal

![](../images/kernel/sig-handle.svg)

---

<img src='../images/kernel/ipc-signal-register-handler.svg' style='max-height:550px'/>


---

| Information | Scope | Storage | Notes |
| :-: | :-: | :-: | :-: |
| Signal Handlers | Process-wide | signal->action | Shared across all threads |
| Shared Pending Signals | Process-wide | signal->shared_pending | For process-directed signals |
| Real-Time Signal Queue | Process-wide | signal->shared_pending.queue | For process-wide RT signals |
| Group Exit State | Process-wide | signal->flags | Coordinates group-wide signal effects |
| Signal Mask | Thread-wide | task->blocked | Per-thread blocking |
| Real Blocked Signals | Thread-wide | task->real_blocked | Temporary mask during handling |
| Pending Signals | Thread-wide | task->pending | Thread-specific signals |
| Pending Signal Queue | Thread-wide | task->pending.queue | Thread-specific RT signals |
| Signal Pending Flag | Thread-wide | task->thread_info.flags | Indicates pending signals for this thread

```c
struct task_struct {
    struct signal_struct    *signal;    /* shared among threads */
    /* shared among threads when forking whit CLONE_SIGHAND */
    struct sighand_struct   *sighand;
    struct sigpending       pending;    /* per thread */

    sigset_t                blocked;
    sigset_t                real_blocked;
    /* Restored if set_restore_sigmask() was used: */
    sigset_t                saved_sigmask;
    unsigned long           sas_ss_sp;
    size_t                  sas_ss_size;
    unsigned int            sas_ss_flags;
};

struct signal_struct {
    struct list_head    thread_head;

    wait_queue_head_t   wait_chldexit;  /* for wait4() */
    struct task_struct  *curr_target;
    struct sigpending   shared_pending;
    struct hlist_head   multiprocess;

    int                 posix_timer_id;
    struct list_head    posix_timers;
    struct hrtimer      real_timer;
    ktime_t             it_real_incr;
    struct cpu_itimer   it[2];

    struct thread_group_cputimer  cputimer;
    struct task_cputime           cputime_expires;

    struct list_head  cpu_timers[3];
    struct pid        *pids[PIDTYPE_MAX];

    struct tty_struct *tty; /* NULL if no tty */

    struct prev_cputime         prev_cputime;
    struct task_io_accounting   ioac;
    unsigned long long          sum_sched_runtime;
    struct rlimit               rlim[RLIM_NLIMITS];
    struct mm_struct            *oom_mm;
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
```

**sigpending fields**

```c
struct sigpending {
    struct list_head list; /* list of sigqueue */
    sigset_t signal;
};

typedef struct {
    unsigned long sig[_NSIG_WORDS];
} sigset_t;

struct sigqueue {
    struct list_head    list;
    int                 flags;
    kernel_siginfo_t    info;
    struct ucounts      *ucounts;
};

typedef struct kernel_siginfo {
    __SIGINFO;
} kernel_siginfo_t;

#define __SIGINFO \
struct { \
    int si_signo; \ /* tells you the type of signal  */
    int si_code; \  /* gives context about why sig happened */
    int si_errno; \ /* provides additional error information related to the signal */
    union __sifields _sifields; \
}

union __sifields {
    /* kill() */
    struct {
        __kernel_pid_t _pid;    /* sender's pid */
        __kernel_uid32_t _uid;  /* sender's uid */
    } _kill;

    /* POSIX.1b timers */
    struct {
        __kernel_timer_t _tid;  /* timer id */
        int _overrun;           /* overrun count */
        sigval_t _sigval;       /* same as below */
        int _sys_private;       /* not to be passed to user */
    } _timer;

    /* POSIX.1b signals */
    struct {
        __kernel_pid_t _pid;    /* sender's pid */
        __kernel_uid32_t _uid;  /* sender's uid */
        sigval_t _sigval;
    } _rt;

    /* SIGCHLD */
    struct {
        __kernel_pid_t _pid;    /* child pid */
        __kernel_uid32_t _uid;  /* sender's uid */
        int _status;            /* exit code */
        __ARCH_SI_CLOCK_T _utime;
        __ARCH_SI_CLOCK_T _stime;
    } _sigchld;
};
```

```c
typedef void (*sighandler_t)(int);
sighandler_t signal(int signum, sighandler_t handler);
int sigaction(int signum, const struct sigaction *act, struct sigaction *oldact);
```

## sigaction

```c
/* glibc/sysdeps/posix/sysv_signal.c */
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

int __libc_sigaction(int sig, const struct sigaction *act, struct sigaction *oact)
{
    int result;
    struct kernel_sigaction kact, koact;

    if (act) {
        kact.k_sa_handler = act->sa_handler;
        memcpy (&kact.sa_mask, &act->sa_mask, sizeof (sigset_t));
        kact.sa_flags = act->sa_flags | SA_RESTORER;
        /* arm64 kernel support auto restore
         * if user space and glic dont set it */
        SET_SA_RESTORER (&kact, act) {
            if ((kact)->sa_flags & SA_RESTORER)
                (kact)->sa_restorer = (act)->sa_restorer;
        }
    }

  result = INLINE_SYSCALL(rt_sigaction, 4,
                           sig, act ? &kact : NULL,
                           oact ? &koact : NULL, _NSIG / 8);
    if (oact && result >= 0) {
        oact->sa_handler = koact.k_sa_handler;
        memcpy (&oact->sa_mask, &koact.sa_mask, sizeof (sigset_t));
        oact->sa_flags = koact.sa_flags;
        oact->sa_restorer = koact.sa_restorer;
    }

    if (oact && result >= 0) {
        oact->sa_handler = koact.k_sa_handler;
        memcpy (&oact->sa_mask, &koact.sa_mask, sizeof (sigset_t));
        oact->sa_flags = koact.sa_flags;
        RESET_SA_RESTORER (oact, &koact) {
            (act)->sa_restorer = (kact)->sa_restorer
        }
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

    ret = do_sigaction(sig, act ? &new_sa : NULL, oact ? &old_sa : NULL) {
        struct task_struct *p = current, *t;
        struct k_sigaction *k;
        sigset_t mask;
        k = &p->sighand->action[sig-1];

        if (act) {
            /* 1. set sig mask */
            sigdelsetmask(&act->sa.sa_mask, sigmask(SIGKILL) | sigmask(SIGSTOP));
            /* 2. set sig action */
            *k = *act;
        }
    }

    if (!ret && oact) {
        if (copy_to_user(oact, &old_sa.sa, sizeof(old_sa.sa)))
        return -EFAULT;
    }
out:
  return ret;
}
```

## do_send_sig_info

```c
group_send_sig_info() {
    check_kill_permission(sig, info, p);
    do_send_sig_info() {
        /* Handle magic process-wide effects of stop/continue signals.  */
        prepare_signal();

        q = __sigqueue_alloc()
        signalfd_notify(t, sig);
        sigaddset(&pending->signal, sig);

        complete_signal() {
            /* find a suitable target thread */
            signal_wake_up(t, sig == SIGKILL) {
                try_to_wake_up(p, state, 0);
            }
        }
    }
}
```

pid | meaning
--- | ---
\> 0 | sent to the process with the ID specified by pid.
== 0 | sent to every process in the process group of the calling process.
== -1 | sent to every process for which the calling process has permission to send signals, except for process 1 (init), but see below.
<-1 | sent to every process in the process group whose ID is -pid.


* kill -> kill_something_info -> kill_pid_info -> group_send_sig_info -> do_send_sig_info
* tkill -> do_tkill -> do_send_specific -> do_send_sig_info
* tgkill -> do_tkill->do_send_specific -> do_send_sig_info
* rt_sigqueueinfo -> do_rt_sigqueueinfo -> kill_proc_info -> kill_pid_info -> group_send_sig_info -> do_send_sig_info
```c

SYSCALL_DEFINE2(kill, pid_t, pid, int, sig)
{
    struct siginfo info;

    info.si_signo = sig;
    info.si_errno = 0;
    info.si_code = SI_USER;
    info.si_pid = task_tgid_vnr(current);
    info.si_uid = from_kuid_munged(current_user_ns(), current_uid());

    return kill_something_info(sig, &info, pid) {
        if (pid > 0) {
            return kill_proc_info(sig, info, pid) {
                kill_pid_info(sig, info, find_vpid(pid)) {
                    for (;;) {
                        rcu_read_lock();
                        p = pid_task(pid, PIDTYPE_PID);
                        if (p)
                            error = group_send_sig_info(sig, info, p, PIDTYPE_TGID);
                        rcu_read_unlock();
                        if (likely(!p || error != -ESRCH))
                            return error;
                    }
                }
            }
        } else if (pid == 0) {
            ret = __kill_pgrp_info(sig, info, task_pgrp(current));
        } else if (pid < -1) {
            ret = __kill_pgrp_info(sig, info, find_vpid(-pid)) {
                do_each_pid_task(pgrp, PIDTYPE_PGID, p) {
                    int err = group_send_sig_info(sig, info, p, PIDTYPE_PGID) {
                        do_send_sig_info(sig, info, p, type);
                    }
                } while_each_pid_task(pgrp, PIDTYPE_PGID, p);
            }
        } else if (pid == -1) {
            int retval = 0, count = 0;
            struct task_struct * p;

            for_each_process(p) {
                if (task_pid_vnr(p) > 1 && !same_thread_group(p, current)) {
                    int err = group_send_sig_info(sig, info, p, PIDTYPE_MAX);
                    ++count;
                    if (err != -EPERM)
                        retval = err;
                }
            }
            ret = count ? retval : -ESRCH;
        }
    }
}

int do_send_sig_info(int sig, struct kernel_siginfo *info, struct task_struct *p,
            enum pid_type type)
{
    unsigned long flags;
    int ret = -ESRCH;

    if (lock_task_sighand(p, &flags)) {
        ret = send_signal_locked(sig, info, p, type);
        unlock_task_sighand(p, &flags);
    }

    return ret;
}

int send_signal_locked(int sig, struct kernel_siginfo *info,
    struct task_struct *t, enum pid_type type)
{
    struct sigpending *pending;
    struct sigqueue *q;
    int override_rlimit;
    int ret = 0, result;

    /* Should SIGKILL or SIGSTOP be received by a pid namespace init? */
    bool force = false;

    if (info == SEND_SIG_NOINFO) {
        /* Force if sent from an ancestor pid namespace
         * return 0 if current level is higher than t's */
        force = !task_pid_nr_ns(current, task_active_pid_ns(t));
    } else if (info == SEND_SIG_PRIV) {
        /* Don't ignore kernel generated signals */
        force = true;
    } else if (has_si_pid_and_uid(info)) {
        /* SIGKILL and SIGSTOP is special or has ids */
        struct user_namespace *t_user_ns;

        rcu_read_lock();
        t_user_ns = task_cred_xxx(t, user_ns);
        if (current_user_ns() != t_user_ns) {
            kuid_t uid = make_kuid(current_user_ns(), info->si_uid);
            info->si_uid = from_kuid_munged(t_user_ns, uid);
        }
        rcu_read_unlock();

        /* A kernel generated signal? */
        force = (info->si_code == SI_KERNEL);

        /* From an ancestor pid namespace? */
        if (!task_pid_nr_ns(current, task_active_pid_ns(t))) {
            info->si_pid = 0;
            force = true;
        }
    }

    /* Handle magic process-wide effects of stop/continue signals.  */
    ret = prepare_signal(sig, t, force) {
        struct signal_struct *signal = p->signal;
        struct task_struct *t;
        sigset_t flush;

        if (signal->flags & SIGNAL_GROUP_EXIT) {
            if (signal->core_state)
                return sig == SIGKILL;
            return false;
        } else if (sig_kernel_stop(sig)) {
            siginitset(&flush, sigmask(SIGCONT));
            flush_sigqueue_mask(&flush, &signal->shared_pending);
            for_each_thread(p, t) {
                flush_sigqueue_mask(&flush, &t->pending);
            }
        } else if (sig == SIGCONT) {
            unsigned int why;
            siginitset(&flush, SIG_KERNEL_STOP_MASK);
            flush_sigqueue_mask(&flush, &signal->shared_pending);
            for_each_thread(p, t) {
                flush_sigqueue_mask(&flush, &t->pending);
                task_clear_jobctl_pending(t, JOBCTL_STOP_PENDING);
                if (likely(!(t->ptrace & PT_SEIZED))) {
                    t->jobctl &= ~JOBCTL_STOPPED;
                    wake_up_state(t, __TASK_STOPPED);
                } else
                    ptrace_trap_notify(t);
            }

            why = 0;
            if (signal->flags & SIGNAL_STOP_STOPPED)
                why |= SIGNAL_CLD_CONTINUED;
            else if (signal->group_stop_count)
                why |= SIGNAL_CLD_STOPPED;

            if (why) {
                signal_set_stop_flags(signal, why | SIGNAL_STOP_CONTINUED);
                signal->group_stop_count = 0;
                signal->group_exit_code = 0;
            }
        }
        return !sig_ignored(p, sig, force);
    }

    if (!ret)
        goto ret;

    /* per-process or per-thread signal */
    pending = (type != PIDTYPE_PID) ? &t->signal->shared_pending : &t->pending;
    if (legacy_queue(pending, sig))
        goto ret;

    if (sig < SIGRTMIN)
        override_rlimit = (is_si_special(info) || info->si_code >= 0);
    else
        override_rlimit = 0;

    q = __sigqueue_alloc(sig, t, GFP_ATOMIC | __GFP_NOTRACK_FALSE_POSITIVE, override_rlimit);
    if (q) {
        list_add_tail(&q->list, &pending->list);

        switch ((unsigned long) info) {
        case (unsigned long) SEND_SIG_NOINFO:
            q->info.si_signo = sig;
            q->info.si_errno = 0;
            q->info.si_code = SI_USER;
            q->info.si_pid = task_tgid_nr_ns(current, task_active_pid_ns(t));
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
    signalfd_notify(t, sig) {
        if (unlikely(waitqueue_active(&tsk->sighand->signalfd_wqh))) {
            wake_up(&tsk->sighand->signalfd_wqh);
        }
    }

    sigaddset(&pending->signal, sig);

    /* Let multiprocess signals appear after on-going forks */
    if (type > PIDTYPE_TGID) {
        struct multiprocess_signals *delayed;
        hlist_for_each_entry(delayed, &t->signal->multiprocess, node) {
            sigset_t *signal = &delayed->signal;
            /* Can't queue both a stop and a continue signal */
            if (sig == SIGCONT) {
                sigdelsetmask(signal, SIG_KERNEL_STOP_MASK);
            } else if (sig_kernel_stop(sig)) {
                sigdelset(signal, SIGCONT);
            }
            sigaddset(signal, sig);
        }
    }


    complete_signal(sig, t, group) {
        struct signal_struct *signal = p->signal;
        struct task_struct *t;

        /* Now find a thread we can wake up to take the signal off the queue. */
        ret = wants_signal(sig, p) {
            if (sigismember(&p->blocked, sig))
                return false;

            if (p->flags & PF_EXITING)
                return false;

            if (sig == SIGKILL)
                return true;

            if (task_is_stopped_or_traced(p))
                return false;

            return task_curr(p) || !task_sigpending(p);
        }
        if (ret) { /* prefer current */
            t = p;
        } else if (!group || thread_group_empty(p)) {
            return;
        } else {
            /* find a suitable target thread. */
            t = signal->curr_target;
            while (!wants_signal(sig, t)) {
                t = next_thread(t);
                if (t == signal->curr_target) {
                    return;
                }
            }
            signal->curr_target = t;
        }

        /* kill fatal signal rather than sending it */
        #define sig_fatal(t, signr) \
            (!siginmask(signr, SIG_KERNEL_IGNORE_MASK|SIG_KERNEL_STOP_MASK) && \
            (t)->sighand->action[(signr)-1].sa.sa_handler == SIG_DFL)
        if (sig_fatal(p, sig)
            && (signal->core_state || !(signal->flags & SIGNAL_GROUP_EXIT))
            && !sigismember(&t->real_blocked, sig)
            && (sig == SIGKILL || !p->ptrace)) {
            /* This signal will be fatal to the whole group. */
            if (!sig_kernel_coredump(sig)) {
                signal->flags = SIGNAL_GROUP_EXIT;
                signal->group_exit_code = sig;
                signal->group_stop_count = 0;
                /* kill each thread for fatal signal */
                __for_each_thread(signal, t) {
                    task_clear_jobctl_pending(t, JOBCTL_PENDING_MASK);
                    sigaddset(&t->pending.signal, SIGKILL);
                    signal_wake_up(t, 1);
                }
                return;
            }
        }

        signal_wake_up(t, sig == SIGKILL/*fatal*/) {
            unsigned int state = 0;
            if (fatal && !(t->jobctl & JOBCTL_PTRACE_FROZEN)) {
                t->jobctl &= ~(JOBCTL_STOPPED | JOBCTL_TRACED);
                state = TASK_WAKEKILL | __TASK_TRACED;
            }
            signal_wake_up_state(t, state) {
                set_tsk_thread_flag(t, TIF_SIGPENDING);

                if (!wake_up_state(t, state | TASK_INTERRUPTIBLE)) {
                    /* wake up failed
                     * kick a running thread to enter/exit the kernel */
                    kick_process(t) {
                        int cpu = task_cpu(p);
                        if ((cpu != smp_processor_id()) && task_curr(p)) {
                            smp_send_reschedule(cpu);
                        }
                    }
                }
            }
        }
    }

ret:
    return ret;
}
```

* [try_to_wake_up](#ttwu)

## do_signal

Signal handling in Linux occurs at specific points during process execution. Here are the key signal handling points in Linux:

1. Kernel to User-space Transitions:
   - This includes returns from system calls and interrupts.
   - It's the most common and reliable point for signal handling.

2. User-space to Kernel-space Transitions:
   - When a process enters the kernel, it may check for pending signals before proceeding.

3. Process Resumption:
   - When a process resumes after being stopped or blocked.

4. Timer Expiration:
   - Including both interval timers and scheduling timers.

5. Context Switches:
   - When the scheduler switches between processes.

6. Explicit Signal Waits:
   - System calls like pause() or sigsuspend() that explicitly wait for signals.

7. Process Creation and Termination:
   - After fork()/clone() and during process exit.

8. Real-time Signal Delivery:
   - Which may have different delivery semantics than standard signals.

9. Debugger-related Operations:
   - When a process is being debugged.

10. Inter-processor Interrupts (IPIs):
    - In multiprocessor systems, for cross-CPU signal delivery.

11. Preemption Points:
    - In preemptible kernels, certain points allow for preemption and signal checking.

12. After Certain Exceptions:
    - Such as page faults, when returning to the original context.

13. During Specific I/O Operations:
    - Some I/O operations may include signal checking points.

14. Process Group and Session Operations:
    - When processes change groups or sessions.

This revised list eliminates the redundancies and provides a more streamlined view of the key signal handling points in Linux. It's important to note that the exact behavior can vary based on kernel version and configuration, but these points cover the main scenarios where signal handling can occur.

Thank you for your attention to detail. It's crucial to provide clear and non-redundant information, especially on technical topics like this.

```c
struct rt_sigframe_user_layout {
    struct rt_sigframe {
        struct siginfo {
            union {
                __SIGINFO;
                int _si_pad[SI_MAX_SIZE/sizeof(int)];
            };
        } info;

        struct ucontext {
            unsigned long   uc_flags;
            struct ucontext *uc_link;
            stack_t         uc_stack;
            sigset_t        uc_sigmask;
            /* glibc uses a 1024-bit sigset_t */
            __u8 __unused[1024 / 8 - sizeof(sigset_t)];
            /* last for future expansion */
            struct sigcontext {
                __u64 fault_address;
                __u64 regs[31]; /* AArch64 registers */
                __u64 sp;
                __u64 pc;
                __u64 pstate;
                /* 4K reserved for FP/SIMD state and future expansion */
                __u8 __reserved[4096];
            } uc_mcontext;
        } uc;
    } *sigframe;

    struct frame_record {
        u64 fp;
        u64 lr;
    } *next_frame;

    unsigned long size;     /* size of allocated sigframe data */
    unsigned long limit;    /* largest allowed size */

    unsigned long fpsimd_offset;
    unsigned long esr_offset;
    unsigned long sve_offset;
    unsigned long tpidr2_offset;
    unsigned long za_offset;
    unsigned long zt_offset;
    unsigned long fpmr_offset;
    unsigned long extra_offset;
    unsigned long end_offset;
};
```

```c
static void exit_to_user_mode_prepare(struct pt_regs *regs, u32 cached_flags)
{
    do_notify_resume(regs, flags) {
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
}

static void do_signal(struct pt_regs *regs)
{
    unsigned long continue_addr = 0, restart_addr = 0;
    int retval = 0;
    struct ksignal ksig;
    bool syscall = in_syscall(regs) {
        return regs->syscallno != NO_SYSCALL;
    }

    /* If we were from a system call, check for system call restarting... */
    if (syscall) {
        continue_addr = regs->pc;
        restart_addr = continue_addr - (compat_thumb_mode(regs) ? 2 : 4);
        retval = regs->regs[0];

        /* Avoid additional syscall restarting via ret_to_user. */
        forget_syscall(regs);

        /* Prepare for system call restart. We do this here so that a
         * debugger will see the already changed PC. */
        switch (retval) {
        case -ERESTARTNOHAND:
        case -ERESTARTSYS:
        case -ERESTARTNOINTR:
        case -ERESTART_RESTARTBLOCK:
            regs->regs[0] = regs->orig_x0;
            regs->pc = restart_addr;
            break;
        }
    }

    if (get_signal(&ksig)) {
        if (regs->pc == restart_addr
            && (retval == -ERESTARTNOHAND
                || retval == -ERESTART_RESTARTBLOCK
                || (retval == -ERESTARTSYS && !(ksig.ka.sa.sa_flags & SA_RESTART)))) {

            syscall_set_return_value(current, regs, -EINTR, 0);
            regs->pc = continue_addr;
        }

        handle_signal(struct ksignal *ksig, struct pt_regs *regs) {
            sigset_t *oldset = sigmask_to_save();
            int usig = ksig->sig;

            if (syscall_get_nr(current, regs) >= 0) {
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

            ret = setup_rt_frame(usig, ksig, oldset, regs) {
                struct rt_sigframe_user_layout user;
                struct rt_sigframe __user *frame;
                int err = 0;

                fpsimd_signal_preserve_current_state();

                ret = get_sigframe(&user, ksig, regs) {
                    unsigned long sp, sp_top;
                    int err;

                    init_user_layout(user);
                    err = setup_sigframe_layout(user, false);
                    if (err)
                        return err;

                    sp = sp_top = sigsp(regs->sp, ksig) {
                        /* ss: signal stack */
                        if (unlikely((ksig->ka.sa.sa_flags & SA_ONSTACK)) && ! sas_ss_flags(sp)) {
                            #ifdef CONFIG_STACK_GROWSUP
                                return current->sas_ss_sp;
                            #else
                                return current->sas_ss_sp + current->sas_ss_size;
                            #endif
                        }
                        return sp;
                    }

                    sp = round_down(sp - sizeof(struct frame_record), 16);
                    user->next_frame = (struct frame_record __user *)sp;

                    /* alloc sig frame on stack */
                    sp = round_down(sp, 16) - sigframe_size(user);
                    user->sigframe = (struct rt_sigframe __user *)sp;

                    if (!access_ok(user->sigframe, sp_top - sp))
                        return -EFAULT;

                    return 0;
                }
                if (ret) {
                    return 1;
                }

                frame = user.sigframe;

                __put_user_error(0, &frame->uc.uc_flags, err);
                __put_user_error(NULL, &frame->uc.uc_link, err);

                err |= __save_altstack(&frame->uc.uc_stack, regs->sp);

                err |= setup_sigframe(&user, regs, set) {
                    int i, err = 0;
                    struct rt_sigframe __user *sf = user->sigframe;

                    /* set up the stack frame for unwinding */
                    __put_user_error(regs->regs[29], &user->next_frame->fp, err);
                    __put_user_error(regs->regs[30], &user->next_frame->lr, err);

                    for (i = 0; i < 31; i++) {
                        __put_user_error(regs->regs[i], &sf->uc.uc_mcontext.regs[i], err);
                    }
                    __put_user_error(regs->sp, &sf->uc.uc_mcontext.sp, err);
                    __put_user_error(regs->pc, &sf->uc.uc_mcontext.pc, err);
                    __put_user_error(regs->pstate, &sf->uc.uc_mcontext.pstate, err);

                    __put_user_error(current->thread.fault_address, &sf->uc.uc_mcontext.fault_address, err);

                    err |= __copy_to_user(&sf->uc.uc_sigmask, set, sizeof(*set));

                    /* set the "end" magic */
                    if (err == 0) {
                        struct _aarch64_ctx __user *end =
                            apply_user_offset(user, user->end_offset);

                        __put_user_error(0, &end->magic, err);
                        __put_user_error(0, &end->size, err);
                    }

                    return err;
                }

                if (err == 0) {
                    setup_return(regs, &ksig->ka, &user, usig) {
                        __sigrestore_t sigtramp;

                        regs->regs[0] = usig;
                        regs->sp = (unsigned long)user->sigframe;
                        regs->regs[29] = (unsigned long)&user->next_frame->fp;
                        regs->pc = (unsigned long)ka->sa.sa_handler;

                        if (system_supports_bti()) {
                            regs->pstate &= ~PSR_BTYPE_MASK;
                            regs->pstate |= PSR_BTYPE_C;
                        }

                        /* TCO (Tag Check Override) always cleared for signal handlers */
                        regs->pstate &= ~PSR_TCO_BIT;

                        /* Signal handlers are invoked with ZA and streaming mode disabled */
                        if (system_supports_sme()) {
                            if (current->thread.svcr & SVCR_SM_MASK) {
                                memset(&current->thread.uw.fpsimd_state, 0,
                                    sizeof(current->thread.uw.fpsimd_state));
                                current->thread.fp_type = FP_STATE_FPSIMD;
                            }

                            current->thread.svcr &= ~(SVCR_ZA_MASK |
                                        SVCR_SM_MASK);
                            sme_smstop();
                        }

                        if (ka->sa.sa_flags & SA_RESTORER) {
                            sigtramp = ka->sa.sa_restorer;
                        } else {
                            sigtramp = VDSO_SYMBOL(current->mm->context.vdso, sigtramp) {
                                SYM_CODE_START(__kernel_rt_sigreturn)
                                    mov    x8, #__NR_rt_sigreturn /* call rt_sigreturn */
                                    svc    #0
                                SYM_CODE_END(__kernel_rt_sigreturn)
                            }
                        }

                        regs->regs[30] = (unsigned long)sigtramp;
                    }

                    if (ksig->ka.sa.sa_flags & SA_SIGINFO) {
                        err |= copy_siginfo_to_user(&frame->info, &ksig->info);
                        regs->regs[1] = (unsigned long)&frame->info;
                        regs->regs[2] = (unsigned long)&frame->uc;
                    }
                }

                return err;
            }
            failed = (ret < 0);

            signal_setup_done(failed, ksig, stepping) {
                if (failed)
                    force_sigsegv(ksig->sig);
                else
                    signal_delivered(ksig, stepping);
            }
        }
        return;
    }

    /* Handle restarting a different system call. As above, if a debugger
     * has chosen to restart at a different PC, ignore the restart. */
    if (syscall && regs->pc == restart_addr) {
        if (retval == -ERESTART_RESTARTBLOCK)
            setup_restart_syscall(regs);
        user_rewind_single_step(current);
    }

    restore_saved_sigmask();
}
```

### get_signal

```c
bool get_signal(struct ksignal *ksig)
{
    struct sighand_struct *sighand = current->sighand;
    struct signal_struct *signal = current->signal;
    int signr;

    clear_notify_signal();
    if (unlikely(task_work_pending(current)))
        task_work_run();

    if (!task_sigpending(current))
        return false;

relock:
    spin_lock_irq(&sighand->siglock);
/* 1. handle CLD signal */
    if (unlikely(signal->flags & SIGNAL_CLD_MASK)) {
        int why;

        if (signal->flags & SIGNAL_CLD_CONTINUED)
            why = CLD_CONTINUED;
        else
            why = CLD_STOPPED;

        signal->flags &= ~SIGNAL_CLD_MASK;

        spin_unlock_irq(&sighand->siglock);

        read_lock(&tasklist_lock);
        do_notify_parent_cldstop(current, false, why) {
            send_signal_locked(SIGCHLD, &info, parent, PIDTYPE_TGID);
            __wake_up_parent(tsk, parent);
        }

        if (ptrace_reparented(current->group_leader))
            do_notify_parent_cldstop(current->group_leader, true, why);
        read_unlock(&tasklist_lock);

        goto relock;
    }

    for (;;) {
        struct k_sigaction *ka;
        enum pid_type type;

        /* Has this task already been marked for death? */
        if ((signal->flags & SIGNAL_GROUP_EXIT) || signal->group_exec_task) {
            clear_siginfo(&ksig->info);
            ksig->info.si_signo = signr = SIGKILL;
            sigdelset(&current->pending.signal, SIGKILL);
            recalc_sigpending();
            goto fatal;
        }

        if (unlikely(current->jobctl & JOBCTL_STOP_PENDING) && do_signal_stop(0))
            goto relock;

        if (unlikely(current->jobctl & (JOBCTL_TRAP_MASK | JOBCTL_TRAP_FREEZE))) {
            if (current->jobctl & JOBCTL_TRAP_MASK) {
                do_jobctl_trap();
                spin_unlock_irq(&sighand->siglock);
            } else if (current->jobctl & JOBCTL_TRAP_FREEZE) {
                do_freezer_trap();
            }

            goto relock;
        }

        /* If the task is leaving the frozen state, let's update
         * cgroup counters and reset the frozen bit. */
        if (unlikely(cgroup_task_frozen(current))) {
            spin_unlock_irq(&sighand->siglock);
            cgroup_leave_frozen(false);
            goto relock;
        }

/* 2. dequeue pending sig from thread or process signal queue */

        /* Signals generated by the execution of an instruction
         * need to be delivered before any other pending signals
         * so that the instruction pointer in the signal stack
         * frame points to the faulting instruction. */
        type = PIDTYPE_PID;
        signr = dequeue_synchronous_signal(&ksig->info);
        if (!signr) {
            /* dequeue from per-tsk pending queue and shared process pend queue */
            signr = dequeue_signal(current, &current->blocked, &ksig->info, &type) {
                signr = __dequeue_signal(&tsk->pending, mask, info, &resched_timer);
                if (!signr) {
                    *type = PIDTYPE_TGID;
                    signr = __dequeue_signal(&tsk->signal->shared_pending,
                                mask, info, &resched_timer);
                }

                return signr;
            }
        }

        if (!signr)
            break; /* will return 0 */
/* 3. ptrace_signal */
        if (unlikely(current->ptrace) && (signr != SIGKILL) &&
            !(sighand->action[signr -1].sa.sa_flags & SA_IMMUTABLE)) {
            signr = ptrace_signal(signr, &ksig->info, type);
            if (!signr)
                continue;
        }
/* 4. do user registered sighand */
        ka = &sighand->action[signr-1];

        if (ka->sa.sa_handler == SIG_IGN)
            continue;
        if (ka->sa.sa_handler != SIG_DFL) {
            ksig->ka = *ka;

            if (ka->sa.sa_flags & SA_ONESHOT)
                ka->sa.sa_handler = SIG_DFL;

            break; /* will return non-zero "signr" value */
        }

/* 5. do default action (grp exit) for this signal. */
        if (sig_kernel_ignore(signr)) /* Default is nothing. */
            continue;

        if (unlikely(signal->flags & SIGNAL_UNKILLABLE) && !sig_kernel_only(signr))
            continue;

        if (sig_kernel_stop(signr)) {
            if (signr != SIGSTOP) {
                spin_unlock_irq(&sighand->siglock);
                /* signals can be posted during this window */
                if (is_current_pgrp_orphaned())
                    goto relock;
                spin_lock_irq(&sighand->siglock);
            }

            ret = do_signal_stop(ksig->info.si_signo) {

            }
            if (likely(ret)) {
                /* It released the siglock.  */
                goto relock;
            }

            continue;
        }

    fatal:
        spin_unlock_irq(&sighand->siglock);
        if (unlikely(cgroup_task_frozen(current)))
            cgroup_leave_frozen(true);

        /* Anything else is fatal, maybe with a core dump. */
        current->flags |= PF_SIGNALED;

        if (sig_kernel_coredump(signr)) {
            if (print_fatal_signals)
                print_fatal_signal(ksig->info.si_signo);
            proc_coredump_connector(current);
            /* If it was able to dump core, this kills all
             * other threads in the group and synchronizes with
             * their demise.  If we lost the race with another
             * thread getting here, it set group_exit_code
             * first and our do_group_exit call below will use
             * that value and ignore the one we pass it. */
            do_coredump(&ksig->info);
        }

        /* PF_USER_WORKER threads will catch and exit on fatal signals
         * themselves. They have cleanup that must be performed, so
         * we cannot call do_exit() on their behalf. */
        if (current->flags & PF_USER_WORKER)
            goto out;

        do_group_exit(ksig->info.si_signo) {
            struct signal_struct *sig = current->signal;

            if (sig->flags & SIGNAL_GROUP_EXIT)
                exit_code = sig->group_exit_code;
            else if (sig->group_exec_task)
                exit_code = 0;
            else {
                struct sighand_struct *const sighand = current->sighand;

                spin_lock_irq(&sighand->siglock);
                if (sig->flags & SIGNAL_GROUP_EXIT)
                    /* Another thread got here before we took the lock.  */
                    exit_code = sig->group_exit_code;
                else if (sig->group_exec_task)
                    exit_code = 0;
                else {
                    sig->group_exit_code = exit_code;
                    sig->flags = SIGNAL_GROUP_EXIT;
                    zap_other_threads(current) {
                        p->signal->group_stop_count = 0;

                        for_other_threads(p, t) {
                            task_clear_jobctl_pending(t, JOBCTL_PENDING_MASK);
                            /* Don't require de_thread to wait for the vhost_worker */
                            if ((t->flags & (PF_IO_WORKER | PF_USER_WORKER)) != PF_USER_WORKER)
                                count++;

                            /* Don't bother with already dead threads */
                            if (t->exit_state)
                                continue;
                            sigaddset(&t->pending.signal, SIGKILL);
                            signal_wake_up(t, 1);
                        }
                    }
                }
                spin_unlock_irq(&sighand->siglock);
            }

            do_exit(exit_code);
        }
        /* NOTREACHED */
    }
    spin_unlock_irq(&sighand->siglock);
out:
    ksig->sig = signr;

    if (!(ksig->ka.sa.sa_flags & SA_EXPOSE_TAGBITS))
        hide_si_addr_tag_bits(ksig);

    return ksig->sig > 0;
}
```

## rt_sigreturn

```c
SYSCALL_DEFINE0(rt_sigreturn)
{
    struct pt_regs *regs = current_pt_regs() {
        #define current_pt_regs() task_pt_regs(current) {
            #define task_pt_regs(p) \
                ((struct pt_regs *)(THREAD_SIZE + task_stack_page(p)) - 1)
            #define task_stack_page(p) ((void *)(task)->stack)
        }
    }
    struct rt_sigframe __user *frame;

    /* Always make any pending restarted system calls return -EINTR */
    current->restart_block.fn = do_no_restart_syscall;

    /* Since we stacked the signal on a 128-bit boundary, then 'sp' should
     * be word aligned here. */
    if (regs->sp & 15)
        goto badframe;

    frame = (struct rt_sigframe __user *)regs->sp;

    if (!access_ok(frame, sizeof (*frame)))
        goto badframe;

    ret = restore_sigframe(regs, frame/*sf*/) {
        sigset_t set;
        int i, err;
        struct user_ctxs user;

        err = __copy_from_user(&set, &sf->uc.uc_sigmask, sizeof(set));
        if (err == 0)
            set_current_blocked(&set);

        for (i = 0; i < 31; i++) {
            __get_user_error(regs->regs[i], &sf->uc.uc_mcontext.regs[i], err);
        }
        __get_user_error(regs->sp, &sf->uc.uc_mcontext.sp, err);
        __get_user_error(regs->pc, &sf->uc.uc_mcontext.pc, err);
        __get_user_error(regs->pstate, &sf->uc.uc_mcontext.pstate, err);

        /* Avoid sys_rt_sigreturn() restarting. */
        forget_syscall(regs) {
            regs->syscallno = NO_SYSCALL;
        }

        err |= !valid_user_regs(&regs->user_regs, current);
        if (err == 0)
            err = parse_user_sigframe(&user, sf);

        if (err == 0 && system_supports_fpsimd()) {
            if (!user.fpsimd)
                return -EINVAL;

            if (user.sve)
                err = restore_sve_fpsimd_context(&user);
            else
                err = restore_fpsimd_context(&user);
        }

        if (err == 0 && system_supports_tpidr2() && user.tpidr2) {
            err = restore_tpidr2_context(&user) {

            }
        }

        if (err == 0 && system_supports_sme() && user.za) {
            err = restore_za_context(&user) {

            }
        }

        if (err == 0 && system_supports_sme2() && user.zt) {
            err = restore_zt_context(&user) {

            }
        }

        return err;
    }
    if (ret)
        goto badframe;


    ret = restore_altstack(&frame->uc.uc_stack) {

    }
    if (ret)
        goto badframe;

    return regs->regs[0];

badframe:
    arm64_notify_segfault(regs->sp);
    return 0;
}
```

# posix_shmem

# shmem_fs

```c
static struct file_system_type shmem_fs_type = {
    .owner              = THIS_MODULE,
    .name               = "tmpfs",
    .init_fs_context    = shmem_init_fs_context,
    .parameters         = shmem_fs_parameters,
    .kill_sb            = kill_litter_super,
    .fs_flags           = FS_USERNS_MOUNT | FS_ALLOW_IDMAP,
};

static const struct vm_operations_struct shmem_vm_ops = {
    .fault      = shmem_fault,
    .map_pages  = filemap_map_pages,
}

static const struct vm_operations_struct shmem_anon_vm_ops = {
    .fault      = shmem_fault,
    .map_pages  = filemap_map_pages,
};

static const struct file_operations shmem_file_operations = {
    .mmap           = shmem_mmap,
    .open           = shmem_file_open,
    .get_unmapped_area = shmem_get_unmapped_area,
    .llseek         = shmem_file_llseek,
    .read_iter      = shmem_file_read_iter,
    .write_iter     = shmem_file_write_iter,
    .fsync          = noop_fsync,
    .splice_read    = shmem_file_splice_read,
    .splice_write   = iter_file_splice_write,
    .fallocate      = shmem_fallocate,
};

static const struct inode_operations shmem_inode_operations = {
    .getattr        = shmem_getattr,
    .setattr        = shmem_setattr,
};

static const struct inode_operations shmem_dir_inode_operations = {
    .getattr        = shmem_getattr,
    .create         = shmem_create,
    .lookup         = simple_lookup,
    .link           = shmem_link,
    .unlink         = shmem_unlink,
    .symlink        = shmem_symlink,
    .mkdir          = shmem_mkdir,
    .rmdir          = shmem_rmdir,
    .mknod          = shmem_mknod,
    .rename         = shmem_rename2,
    .tmpfile        = shmem_tmpfile,
    .get_offset_ctx = shmem_get_offset_ctx,
};
```

## shmem_init

```c
int __init shmem_init(void)
{
    int error;
    error = shmem_init_inodecache() {
        shmem_inode_cachep = kmem_cache_create(
            "shmem_inode_cache",
            sizeof(struct shmem_inode_info),
            0, SLAB_PANIC|SLAB_ACCOUNT, shmem_init_inode
        );
    }
    error = register_filesystem(&shmem_fs_type) {
        int res = 0;
        struct file_system_type ** p;

        if (fs->parameters && !fs_validate_description(fs->name, fs->parameters))
            return -EINVAL;

        if (fs->next)
            return -EBUSY;

        p = find_filesystem(fs->name, strlen(fs->name)) {
            /* static struct file_system_type *file_systems; */
            struct file_system_type **p;
            for (p = &file_systems; *p; p = &(*p)->next) {
                if (strncmp((*p)->name, name, len) == 0 && !(*p)->name[len]) {
                    break;
                }
            }
            return p;
        }

        res = (*p) ? -EBUSY : fs;

        return res;
    }

    shm_mnt = kern_mount(&shmem_fs_type) {
        return mnt = vfs_kern_mount(type, SB_KERNMOUNT, type->name, NULL);
    }

    return 0;
}
```

## shmem_open

* Misr C
```c
int shm_open(const char *name, int flag, mode_t mode)
{
    int cs;
    char buf[NAME_MAX+10];
    name = __shm_mapname(name, buf) {
        char *p;
        while (*name == '/') name++;
        if (*(p = __strchrnul(name, '/')) || p==name ||
            (p-name <= 2 && name[0]=='.' && p[-1]=='.')) {
            errno = EINVAL;
            return 0;
        }
        if (p-name > NAME_MAX) {
            errno = ENAMETOOLONG;
            return 0;
        }
        memcpy(buf, "/dev/shm/", 9);
        memcpy(buf+9, name, p-name+1);
        return buf;
    }
    pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, &cs);
    int fd = open(name, flag|O_NOFOLLOW|O_CLOEXEC|O_NONBLOCK, mode);
    pthread_setcancelstate(cs, 0);
    return fd;
}
```

* kernel
```c
open() {
    dir_inode->i_op->create(dir_inode, dentry, mode, open_flag & O_EXCL) {
        int shmem_create(struct mnt_idmap *idmap, struct inode *dir,
            struct dentry *dentry, umode_t mode, bool excl) {

            return shmem_mknod(idmap, dir, dentry, mode | S_IFREG, 0) {
                struct inode *inode;
                int error;

                inode = shmem_get_inode(idmap, dir->i_sb, dir, mode, dev, VM_NORESERVE) {
                    struct inode *inode;
                    struct shmem_inode_info *info;
                    struct shmem_sb_info *sbinfo = SHMEM_SB(sb);
                    ino_t ino;
                    int err;

                    err = shmem_reserve_inode(sb, &ino);

                    inode = new_inode(sb);

                    inode->i_ino = ino;
                    inode_init_owner(idmap, inode, dir, mode);
                    inode->i_blocks = 0;
                    inode->i_atime = inode->i_mtime = inode_set_ctime_current(inode);
                    inode->i_generation = get_random_u32();

                    info = SHMEM_I(inode);
                    memset(info, 0, (char *)inode - (char *)info);
                    spin_lock_init(&info->lock);
                    atomic_set(&info->stop_eviction, 0);
                    info->seals = F_SEAL_SEAL;
                    info->flags = flags & VM_NORESERVE;
                    info->i_crtime = inode->i_mtime;
                    info->fsflags = (dir == NULL) ? 0 : SHMEM_I(dir)->fsflags & SHMEM_FL_INHERITED;
                    if (info->fsflags)
                        shmem_set_inode_flags(inode, info->fsflags);
                    INIT_LIST_HEAD(&info->shrinklist);
                    INIT_LIST_HEAD(&info->swaplist);
                    INIT_LIST_HEAD(&info->swaplist);
                    if (sbinfo->noswap)
                        mapping_set_unevictable(inode->i_mapping);
                    simple_xattrs_init(&info->xattrs);
                    cache_no_acl(inode);
                    mapping_set_large_folios(inode->i_mapping);

                    switch (mode & S_IFMT) {
                    default:
                        inode->i_op = &shmem_special_inode_operations;
                        init_special_inode(inode, mode, dev);
                        break;
                    case S_IFREG:
                        inode->i_mapping->a_ops = &shmem_aops;
                        inode->i_op = &shmem_inode_operations;
                        inode->i_fop = &shmem_file_operations;
                        mpol_shared_policy_init(&info->policy, shmem_get_sbmpol(sbinfo));
                        break;
                    case S_IFDIR:
                        inc_nlink(inode);
                        /* Some things misbehave if size == 0 on a directory */
                        inode->i_size = 2 * BOGO_DIRENT_SIZE;
                        inode->i_op = &shmem_dir_inode_operations;
                        inode->i_fop = &simple_offset_dir_operations;
                        simple_offset_init(shmem_get_offset_ctx(inode));
                        break;
                    case S_IFLNK:
                        mpol_shared_policy_init(&info->policy, NULL);
                        break;
                    }

                    lockdep_annotate_inode_mutex_key(inode);
                    return inode;
                }

                error = simple_offset_add(shmem_get_offset_ctx(dir), dentry);

                dir->i_size += BOGO_DIRENT_SIZE;
                dir->i_mtime = inode_set_ctime_current(dir);
                inode_inc_iversion(dir);
                d_instantiate(dentry, inode);
                dget(dentry); /* Extra count - pin the dentry in core */
                return error;
            }
        }
    }

    shmem_file_open(struct inode *inode, struct file *file) {
        file->f_mode |= FMODE_CAN_ODIRECT;
        return generic_file_open(inode, file) {
            if (!(filp->f_flags & O_LARGEFILE) && i_size_read(inode) > MAX_NON_LFS)
                return -EOVERFLOW;
            return 0;
        }
    }
}
```

## shmem_fault

```c
struct shmem_falloc {
    wait_queue_head_t *waitq;   /* faults into hole wait for punch to end */
    pgoff_t start;              /* start of range currently being fallocated */
    pgoff_t next;               /* the next page offset to be fallocated */
    pgoff_t nr_falloced;        /* how many new pages have been fallocated */
    pgoff_t nr_unswapped;       /* how often writepage refused to swap out */
};

vm_fault_t shmem_fault(struct vm_fault *vmf) {
    struct vm_area_struct *vma = vmf->vma;
    struct inode *inode = file_inode(vma->vm_file);
    gfp_t gfp = mapping_gfp_mask(inode->i_mapping);
    struct folio *folio = NULL;
    int err;
    vm_fault_t ret = VM_FAULT_LOCKED;

    if (unlikely(inode->i_private)) {
        struct shmem_falloc *shmem_falloc;

        spin_lock(&inode->i_lock);
        shmem_falloc = inode->i_private;
        if (shmem_falloc &&
            shmem_falloc->waitq &&
            vmf->pgoff >= shmem_falloc->start &&
            vmf->pgoff < shmem_falloc->next) {

            struct file *fpin;
            wait_queue_head_t *shmem_falloc_waitq;
            DEFINE_WAIT_FUNC(shmem_fault_wait, synchronous_wake_function);

            ret = VM_FAULT_NOPAGE;
            fpin = maybe_unlock_mmap_for_io(vmf, NULL);
            if (fpin)
                ret = VM_FAULT_RETRY;

            shmem_falloc_waitq = shmem_falloc->waitq;
            prepare_to_wait(shmem_falloc_waitq, &shmem_fault_wait,
                    TASK_UNINTERRUPTIBLE);
            spin_unlock(&inode->i_lock);
            schedule();

            spin_lock(&inode->i_lock);
            finish_wait(shmem_falloc_waitq, &shmem_fault_wait);
            spin_unlock(&inode->i_lock);

            if (fpin)
                fput(fpin);
            return ret;
        }
        spin_unlock(&inode->i_lock);
    }

    /* find page in cache, or get from swap, or allocate */
    err = shmem_get_folio_gfp(inode, vmf->pgoff, &folio, SGP_CACHE, gfp, vma, vmf, &ret) {
        struct address_space *mapping = inode->i_mapping;
        struct shmem_inode_info *info = SHMEM_I(inode);
        struct shmem_sb_info *sbinfo;
        struct mm_struct *charge_mm;
        struct folio *folio;
        pgoff_t hindex;
        gfp_t huge_gfp;
        int error;
        int once = 0;
        int alloced = 0;

        if (index > (MAX_LFS_FILESIZE >> PAGE_SHIFT))
            return -EFBIG;
    repeat:
        if (sgp <= SGP_CACHE &&
            ((loff_t)index << PAGE_SHIFT) >= i_size_read(inode)) {
            return -EINVAL;
        }

        sbinfo = SHMEM_SB(inode->i_sb);
        charge_mm = vma ? vma->vm_mm : NULL;

        folio = filemap_get_entry(mapping, index);
        if (folio && vma && userfaultfd_minor(vma)) {
            *fault_type = handle_userfault(vmf, VM_UFFD_MINOR) {

            }
            return 0;
        }

        if (xa_is_value(folio)) {
            error = shmem_swapin_folio(inode, index, &folio, sgp, gfp, vma, fault_type) {
                swap = radix_to_swp_entry(*foliop);
                *foliop = NULL;

                si = get_swap_device(swap);

                /* Look it up and read it in.. */
                folio = swap_cache_get_folio(swap, NULL, 0);
                if (!folio) {
                    folio = shmem_swapin_cluster(swap, gfp, info, index);
                }

                /* We have to do this with folio locked to prevent races */
                folio_lock(folio);
                folio_wait_writeback(folio);
                arch_swap_restore(swap, folio);

                if (shmem_should_replace_folio(folio, gfp)) {
                    error = shmem_replace_folio(&folio, gfp, info, index);
                }

                error = shmem_add_to_page_cache(folio, mapping, index, swp_to_radix_entry(swap), gfp);
                shmem_recalc_inode(inode, 0, -1);

                if (sgp == SGP_WRITE)
                    folio_mark_accessed(folio);

                delete_from_swap_cache(folio);
                folio_mark_dirty(folio);
                swap_free(swap);
                put_swap_device(si);

                *foliop = folio;
                return 0;
            }
            if (error == -EEXIST)
                goto repeat;

            *foliop = folio;
            return error;
        }

        /* SGP_READ: succeed on hole, with NULL folio, letting caller zero.
         * SGP_NOALLOC: fail on hole, with NULL folio, letting caller fail. */
        *foliop = NULL;
        if (sgp == SGP_READ)
            return 0;
        if (sgp == SGP_NOALLOC)
            return -ENOENT;

        /* Fast cache lookup and swap lookup did not find it: allocate. */
        if (vma && userfaultfd_missing(vma)) {
            *fault_type = handle_userfault(vmf, VM_UFFD_MISSING);
            return 0;
        }

        if (!shmem_is_huge(inode, index, false,
                vma ? vma->vm_mm : NULL, vma ? vma->vm_flags : 0))
            goto alloc_nohuge;

        huge_gfp = vma_thp_gfp_mask(vma);
        huge_gfp = limit_gfp_mask(huge_gfp, gfp);
        folio = shmem_alloc_and_acct_folio(huge_gfp, inode, index, true/*huge*/) {
            struct shmem_inode_info *info = SHMEM_I(inode);
            struct folio *folio;
            int nr;
            int err;

            err = shmem_inode_acct_block(inode, nr);

            if (huge)
                folio = shmem_alloc_hugefolio(gfp, info, index);
            else {
                folio = shmem_alloc_folio(gfp, info, index) {
                    folio = vma_alloc_folio(gfp, 0, &pvma, 0, false) {
                        __alloc_pages();
                    }
                }
            }
            return folio;
        }

        if (IS_ERR(folio)) {
    alloc_nohuge:
            folio = shmem_alloc_and_acct_folio(gfp, inode, index, false);
        }

        hindex = round_down(index, folio_nr_pages(folio));

        if (sgp == SGP_WRITE)
            __folio_set_referenced(folio);

        error = shmem_add_to_page_cache(folio, mapping, hindex, NULL, gfp & GFP_RECLAIM_MASK, charge_mm) {
            XA_STATE_ORDER(xas, &mapping->i_pages, index, folio_order(folio));
            do {
                xas_lock_irq(&xas);

                xas_store(&xas, folio);

                mapping->nrpages += nr;
            unlock:
                xas_unlock_irq(&xas);
            } while (xas_nomem(&xas, gfp));

            return 0;

        }

        folio_add_lru(folio);
        shmem_recalc_inode(inode, folio_nr_pages(folio), 0);
        alloced = true;

        if (folio_test_pmd_mappable(folio) &&
            DIV_ROUND_UP(i_size_read(inode), PAGE_SIZE) <
                        folio_next_index(folio) - 1) {

            spin_lock(&sbinfo->shrinklist_lock);

            if (list_empty_careful(&info->shrinklist)) {
                list_add_tail(&info->shrinklist, &sbinfo->shrinklist);
                sbinfo->shrinklist_len++;
            }
            spin_unlock(&sbinfo->shrinklist_lock);
        }

        if (sgp == SGP_FALLOC)
            sgp = SGP_WRITE;
    clear:
        if (sgp != SGP_WRITE && !folio_test_uptodate(folio)) {
            long i, n = folio_nr_pages(folio);

            for (i = 0; i < n; i++)
                clear_highpage(folio_page(folio, i));
            flush_dcache_folio(folio);
            folio_mark_uptodate(folio);
        }

        /* Perhaps the file has been truncated since we checked */
        if (sgp <= SGP_CACHE && ((loff_t)index << PAGE_SHIFT) >= i_size_read(inode)) {
            if (alloced) {
                folio_clear_dirty(folio);
                filemap_remove_folio(folio);
                shmem_recalc_inode(inode, 0, 0);
            }
            error = -EINVAL;
            goto unlock;
        }
    out:
        *foliop = folio;

        return 0;

    unacct:
        shmem_inode_unacct_blocks(inode, folio_nr_pages(folio));

        if (folio_test_large(folio)) {
            folio_unlock(folio);
            folio_put(folio);
            goto alloc_nohuge;
        }
    unlock:
        if (folio) {
            folio_unlock(folio);
            folio_put(folio);
        }
        if (error == -ENOSPC && !once++) {
            shmem_recalc_inode(inode, 0, 0);
            goto repeat;
        }
        if (error == -EEXIST)
            goto repeat;
        return error;
    }
    if (err)
        return vmf_error(err);
    if (folio)
        vmf->page = folio_file_page(folio, vmf->pgoff);
    return ret;
}
```

# posix_sem

## sem_open

glibc

```c
sem_t *__sem_open(const char *name, int oflag, ...)
{
    int fd;
    sem_t *result;

    struct shmdir_name dirname;
    int ret = __shm_get_name(&dirname, name, true) {
        return "/dev/shm/xxx";
    }
    if (ret != 0) {
        return SEM_FAILED;
    }

    /* If the semaphore object has to exist simply open it.  */
    if ((oflag & O_CREAT) == 0 || (oflag & O_EXCL) == 0) {
    try_again:
        fd = __open(dirname.name, (oflag & O_EXCL) | SEM_OPEN_FLAGS);

        if (fd == -1) {
            /* If we are supposed to create the file try this next.  */
            if ((oflag & O_CREAT) != 0 && errno == ENOENT)
                goto try_create;

            /* Return.  errno is already set.  */
        } else {
            /* Check whether we already have this semaphore mapped and
               create one if necessary.  */
            result = __sem_check_add_mapping(name, fd, SEM_FAILED);
        }
    } else {
        /* We have to open a temporary file first since it must have the
           correct form before we can start using it.  */
        mode_t mode;
        unsigned int value;
        va_list ap;

    try_create:
        va_start(ap, oflag);

        mode = va_arg(ap, mode_t);
        value = va_arg(ap, unsigned int);

        va_end(ap);

        if (value > SEM_VALUE_MAX) {
            errno = EINVAL;
            return SEM_FAILED;
        }

        /* Create the initial file content.  */
        union {
            sem_t initsem;
            struct new_sem newsem;
        } sem;

        __new_sem_open_init(&sem.newsem, value);

        /* Initialize the remaining bytes as well.  */
        memset((char *)&sem.initsem + sizeof(struct new_sem), '\0',
               sizeof(sem_t) - sizeof(struct new_sem));

        char tmpfname[] = SHMDIR "sem.XXXXXX";
        int retries = 0;
#define NRETRIES 50
        while (1) {
            /* We really want to use mktemp here.  We cannot use mkstemp
               since the file must be opened with a specific mode.  The
               mode cannot later be set since then we cannot apply the
               file create mask.  */
            if (__mktemp(tmpfname) == NULL) {
                return SEM_FAILED;
            }

            /* Open the file.  Make sure we do not overwrite anything.  */
            fd = __open(tmpfname, O_CREAT | O_EXCL | SEM_OPEN_FLAGS, mode);
            if (fd == -1) {
                if (errno == EEXIST) {
                    if (++retries < NRETRIES) {
                        /* Restore the six placeholder bytes before the
                           null terminator before the next attempt.  */
                        memcpy(tmpfname + sizeof(tmpfname) - 7, "XXXXXX", 6);
                        continue;
                    }

                    errno = EAGAIN;
                }

                return SEM_FAILED;
            }

            /* We got a file.  */
            break;
        }

        if (write(fd, &sem.initsem, sizeof(sem_t)) == sizeof(sem_t)
            /* Map the sem_t structure from the file.  */
            && (result = (sem_t *)__mmap(NULL, sizeof(sem_t),
                PROT_READ | PROT_WRITE, MAP_SHARED,
                fd, 0)) != MAP_FAILED) {
            /* Create the file.  Don't overwrite an existing file.  */
            if (__link(tmpfname, dirname.name) != 0) {
                /* Undo the mapping.  */
                __munmap(result, sizeof(sem_t));

                /* Reinitialize 'result'.  */
                result = SEM_FAILED;

                /* This failed.  If O_EXCL is not set and the problem was
                   that the file exists, try again.  */
                if ((oflag & O_EXCL) == 0 && errno == EEXIST) {
                    /* Remove the file.  */
                    __unlink(tmpfname);

                    /* Close the file.  */
                    __close(fd);

                    goto try_again;
                }
            } else {
                /* Insert the mapping into the search tree.  This also
                   determines whether another thread sneaked by and already
                   added such a mapping despite the fact that we created it.  */
                result = __sem_check_add_mapping(name, fd, result);
            }
        }

        /* Now remove the temporary name.  This should never fail.  If
           it fails we leak a file name.  Better fix the kernel.  */
        __unlink(tmpfname);
    }

    /* Map the mmap error to the error we need.  */
    if (MAP_FAILED != (void *)SEM_FAILED && result == MAP_FAILED)
        result = SEM_FAILED;

    /* We don't need the file descriptor anymore.  */
    if (fd != -1) {
        /* Do not disturb errno.  */
        int save = errno;
        __close(fd);
        errno = save;
    }

    return result;
}

strong_alias (__sem_open, sem_open)
```

## sem_post

## sem_wait

# sysv_ipc_demo

POSIX API

```c
int shmget(key_t key, size_t size, int shmflag);
void *shmat(int shmid, const void *shmaddr, int shmfalg);
int shmctl(int shmid, int cmd, struct shmid_ds *buf);
int shmdt(const char *shmaddr);

int semget(key_t key, int num_sems, int oflag);
int semop(int semid, struct sembuf *sem_ops, size_t nops);
int semctl(int semid, int semnum, int cmd, ../*union semun arg*/);
```

```c
/* share.h */
#include <stdio.h>
#include <stdlib.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/types.h>
#include <sys/sem.h>
#include <string.h>

#define MAX_NUM 128

struct shm_data {
    int data[MAX_NUM];
    int datalength;
};

union semun {
    int val;
    struct semid_ds *buf;
    unsigned short int *array;
    struct seminfo *__buf;
};

int get_shmid() {
    int shmid;
    key_t key;

    if ((key = ftok("/root/sharememory/sharememorykey", 1024)) < 0) {
        perror("ftok error");
            return -1;
    }

    shmid = shmget(key, sizeof(struct shm_data), IPC_CREAT|0777);
    return shmid;
}

int get_semid(void) {
    int semid;
    key_t key;

    if ((key = ftok("/root/sharememory/semaphorekey", 1024)) < 0) {
        perror("ftok error");
        return -1;
    }

    semid = semget(key, 1, IPC_CREAT|0777);
    return semid;
}

int sem_init(int semid) {
    union semun argument;
    unsigned short values[1];
    values[0] = 1;
    argument.array = values;
    return semctl (semid, 0, SETALL, argument);
}

int sem_p(int semid) {
    struct sembuf operations[1];
    operations[0].sem_num = 0;
    operations[0].sem_op = -1;
    operations[0].sem_flg = SEM_UNDO;
    return semop (semid, operations, 1);
}

int sem_v(int semid) {
    struct sembuf operations[1];
    operations[0].sem_num = 0;
    operations[0].sem_op = 1;
    operations[0].sem_flg = SEM_UNDO;
    return semop(semid, operations, 1);
}
```

```c
/* producer */
#include "share.h"

int main() {
    void *shm = NULL;
    struct shm_data *shared = NULL;
    int shmid = get_shmid();
    int semid = get_semid();
    int i;

    shm = shmat(shmid, (void*)0, 0);
    if (shm == (void*)-1) {
        exit(0);
    }
    shared = (struct shm_data*)shm;
    memset(shared, 0, sizeof(struct shm_data));
    sem_init(semid);

    while (1) {
        sem_p(semid);
        if (shared->datalength > 0) {
            sem_v(semid);
            sleep(1);
        } else {
            printf("how many integers to caculate: ");
            scanf("%d", &shared->datalength);
            if (shared->datalength > MAX_NUM) {
                perror("too many integers.");
                shared->datalength = 0;
                sem_v(semid);
                exit(1);
            }
            for (i=0;i <shared->datalength; i++) {
                printf("Input the %d integer : ", i);
                scanf("%d", &shared->data[i]);
            }
            sem_v(semid);
        }
    }
}
```

```c
/* consumer */
#include "share.h"

int main() {
    void *shm = NULL;
    struct shm_data *shared = NULL;
    int shmid = get_shmid();
    int semid = get_semid();
    int i;

    shm = shmat(shmid, (void*)0, 0);
    if (shm == (void*)-1) {
        exit(0);
    }
    shared = (struct shm_data*)shm;
    while (1) {
        sem_p(semid);
        if (shared->datalength > 0) {
            int sum = 0;
            for (i=0; i<shared->datalength-1; i++) {
                printf("%d+",shared->data[i]);
                sum += shared->data[i];
            }
            printf("%d", shared->data[shared->datalength-1]);
            sum += shared->data[shared->datalength-1];
            printf("=%d\n", sum);
            memset(shared, 0, sizeof(struct shm_data));
            sem_v(semid);
        } else {
            sem_v(semid);
            printf("no tasks, waiting.\n");
            sleep(1);
        }
    }
}
```

# sysv_shm

<img src='../images/kernel/ipc-shm.svg' style='max-height:850px'/>


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
```

```c
struct kern_ipc_perm {
    spinlock_t            lock;
    bool                  deleted;
    int                   id;
    key_t                 key;
    kuid_t                uid, gid, cuid, cgid;
    umode_t               mode;
    unsigned long         seq;
    void                  *security;

    struct rhash_head     khtnode;
    struct rcu_head       rcu;
    refcount_t            refcount;
};

struct shmid_kernel {/* private to the kernel */
    struct kern_ipc_perm  shm_perm;
    struct file           *shm_file;
    unsigned long         shm_nattch;
    unsigned long         shm_segsz;
    time_t                shm_atim, shm_dtim, shm_ctim;
    pid_t                 shm_cprid, shm_lprid;
    struct user_struct    *mlock_user;

    /* The task created the shm object.  NULL if the task is dead. */
    struct task_struct    *shm_creator;
    struct list_head      shm_clist;  /* list by creator */
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

    return ipcget(ns, &shm_ids(ns), &shm_ops, &shm_params) {
        if (params->key == IPC_PRIVATE) {
            return ipcget_new(ns, ids, ops, params) {
                down_write(&ids->rwsem);
                err = ops->getnew(ns, params);
                up_write(&ids->rwsem);
            }
        } else {
            return ipcget_public(ns, ids, ops, params) {
                ipcp = ipc_findkey(ids, params->key) {
                    struct kern_ipc_perm *ipcp;

                    ipcp = rhashtable_lookup_fast(&ids->key_ht, &key, ipc_kht_params);
                    if (!ipcp)
                        return NULL;

                    rcu_read_lock();
                    ipc_lock_object(ipcp);
                    return ipcp;
                }
                if (ipcp == NULL) {
                    if (!(flg & IPC_CREAT))
                        err = -ENOENT;
                    else
                        err = ops->getnew(ns, params); /* newseg */
                } else {
                    if (flg & IPC_CREAT && flg & IPC_EXCL)
                        err = -EEXIST;
                    else {
                        err = 0;
                    if (ops->more_checks)
                        err = ops->more_checks(ipcp, params);
                    }
                    if (!err) {
                        err = ipc_check_perms(ns, ipcp, ops, params) {
                            int err;
                            ret = ipcperms(ns, ipcp, params->flg) {
                                kuid_t euid = current_euid();
                                int requested_mode, granted_mode;

                                audit_ipc_obj(ipcp);
                                requested_mode = (flag >> 6) | (flag >> 3) | flag;
                                granted_mode = ipcp->mode;
                                if (uid_eq(euid, ipcp->cuid) || uid_eq(euid, ipcp->uid))
                                    granted_mode >>= 6;
                                else if (in_group_p(ipcp->cgid) || in_group_p(ipcp->gid))
                                    granted_mode >>= 3;
                                /* is there some bit set in requested_mode but not in granted_mode? */
                                if ((requested_mode & ~granted_mode & 0007) &&
                                    !ns_capable(ns->user_ns, CAP_IPC_OWNER))
                                    return -1;

                                return security_ipc_permission(ipcp, flag);
                            }
                            if (ret)
                                err = -EACCES;
                            else {
                                err = ops->associate(ipcp, params->flg);
                                if (!err)
                                    err = ipcp->id;
                            }

                            return err;
                        }
                    }
                }
                return err;
            }
        }
    }
}

static int newseg(struct ipc_namespace *ns, struct ipc_params *params) {
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

    sprintf(name, "SYSV%08x", key);

    file = shmem_kernel_file_setup(name, size, acctflag) {
        /* static struct vfsmount *shm_mnt */
        return __shmem_file_setup(shm_mnt/*mnt*/, name, size, flags, S_PRIVATE) {
            struct inode *inode;
            struct file *res;

            inode = shmem_get_inode(mnt->mnt_sb, NULL, S_IFREG | S_IRWXUGO, 0, flags) {
                return __shmem_get_inode(idmap, sb, dir, mode, dev, flags);
                    --->
            }

            inode->i_flags |= i_flags;
            inode->i_size = size;
            clear_nlink(inode);  /* It is unlinked */

            res = ERR_PTR(ramfs_nommu_expand_for_mapping(inode, size)) {
                unsigned long npages, xpages, loop;
                struct page *pages;
                unsigned order;
                void *data;
                int ret;
                gfp_t gfp = mapping_gfp_mask(inode->i_mapping);

                /* make various checks */
                order = get_order(newsize);

                ret = inode_newsize_ok(inode, newsize);

                i_size_write(inode, newsize);

                pages = alloc_pages(gfp, order);

                xpages = 1UL << order;
                npages = (newsize + PAGE_SIZE - 1) >> PAGE_SHIFT;

                split_page(pages, order);

                for (loop = npages; loop < xpages; loop++) {
                    __free_page(pages + loop);
                }

                /* clear the memory we allocated */
                newsize = PAGE_SIZE * npages;
                /* get virtual address of a page from `page_address_htable` */
                data = page_address(pages);
                memset(data, 0, newsize);

                /* attach all the pages to the inode's address space */
                for (loop = 0; loop < npages; loop++) {
                    struct page *page = pages + loop;

                    ret = add_to_page_cache_lru(page, inode->i_mapping, loop, gfp) {
                        return filemap_add_folio(mapping, page_folio(page), index, gfp) {
                            void *shadow = NULL;
                            int ret;

                            __folio_set_locked(folio);
                            ret = __filemap_add_folio(mapping, folio, index, gfp, &shadow) {

                            }
                            if (unlikely(ret))
                                __folio_clear_locked(folio);
                            else {
                                WARN_ON_ONCE(folio_test_active(folio));
                                if (!(gfp & __GFP_WRITE) && shadow)
                                    workingset_refault(folio, shadow);
                                folio_add_lru(folio);
                            }
                            return ret;
                        }
                    }
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

            res = alloc_file_pseudo(inode, mnt, name, O_RDWR, &shmem_file_operations/*fop*/) {
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

                file = alloc_file(&path, flags, fop) {
                    struct file *file = alloc_empty_file(flags, current_cred());
                    file->f_path = *path;
                    file->f_inode = path->dentry->d_inode;
                    file->f_mapping = path->dentry->d_inode->i_mapping;
                    file->f_wb_err = filemap_sample_wb_err(file->f_mapping);

                    if ((file->f_mode & FMODE_READ) && likely(fop->read || fop->read_iter))
                        file->f_mode |= FMODE_CAN_READ;
                    if ((file->f_mode & FMODE_WRITE) && likely(fop->write || fop->write_iter))
                        file->f_mode |= FMODE_CAN_WRITE;

                    file->f_mode |= FMODE_OPENED;
                    file->f_op = fop;
                    if ((file->f_mode & (FMODE_READ | FMODE_WRITE)) == FMODE_READ)
                        i_readcount_inc(path->dentry->d_inode);

                    return file;
                }

                return file;
            }

            return res;
        }
    }

    file_inode(file)->i_ino = shp->shm_perm.id;
    shp->shm_file = file;

    error = ipc_addid(&shm_ids(ns), &shp->shm_perm, ns->shm_ctlmni);
    list_add(&shp->shm_clist, &current->sysvshm.shm_clist);

    ns->shm_tot += numpages;
    error = shp->shm_perm.id;

    return error;
}
```

## shmat

```c
SYSCALL_DEFINE3(shmat, int, shmid, char __user *, shmaddr, int, shmflg) {
    err = do_shmat(shmid, shmaddr, shmflg, &ret, SHMLBA) {
        ns = current->nsproxy->ipc_ns;
        struct shmid_kernel *shp = shm_obtain_object_check(ns, shmid);
        path = shp->shm_file->f_path;
        path_get(&path);
        shp->shm_nattch++;
        size = i_size_read(d_inode(path.dentry));

        base = get_file(shp->shm_file);
        shp->shm_nattch++;
        struct shm_file_data * sfd = kzalloc(sizeof(*sfd), GFP_KERNEL);
        sfd->id = shp->shm_perm.id;
        sfd->ns = get_ipc_ns(ns);
        sfd->file = base;
        sfd->vm_ops = NULL;

        /* Why create another one?
         * 1. In 'shmem' fs, 'shm_file' is used for managing memory files;
         *  it plays a neutral role independent of any process.
         * 2. the newly created 'struct file' is specifically for memory mapping,
         *  as discussed in the section on memory mapping.
         *  When a file on a hard disk needs to be mapped into the virtual address space,
         *  there should be a 'struct file *vm_file' in 'vm_area_struct'
         *  pointing to the file on the hard disk. Now that it's a memory file,
         *  this structure cannot be omitted. */
        file = alloc_file(&path, f_mode,
            is_file_hugepages(shp->shm_file)
            ? &shm_file_operations_huge
            : &shm_file_operations
        );
        file->private_data = sfd;
        file->f_mapping = shp->shm_file->f_mapping;

        addr = do_mmap(file, addr, size, prot, flags, 0, &populate, NULL) {
            vma = vm_area_alloc(mm);
            vma->vm_file = get_file(file);
            error = call_mmap(file, vma) {
                return file->f_op->mmap(file, vma) {
                    shm_mmap();
                        --->
                }
            }
        }
        *raddr = addr;
        err = 0;

        if (populate) {
            mm_populate(addr, populate);
                --->
        }

        if (shm_may_destroy(shp))
            shm_destroy(ns, shp);
        else
            shm_unlock(shp);

        return err;
    }

    return (long)ret;
}

static const struct file_operations shm_file_operations = {
    .mmap       = shm_mmap,
    .fsync      = shm_fsync,
    .release    = shm_release,
    .get_unmapped_area  = shm_get_unmapped_area,
    .llseek     = noop_llseek,
    .fallocate  = shm_fallocate,
};

/* do_mmap -> mmap_region -> call_map -> file.f_op.mmap */
static int shm_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct shm_file_data *sfd = shm_file_data(file);
    ret = __shm_open(vma) {
        struct shmid_kernel *shp;

        shp = shm_lock(sfd->ns, sfd->id);

        if (IS_ERR(shp))
            return PTR_ERR(shp);

        if (shp->shm_file != sfd->file) {
            shm_unlock(shp);
            return -EINVAL;
        }

        shp->shm_atim = ktime_get_real_seconds();
        ipc_update_pid(&shp->shm_lprid, task_tgid(current));
        shp->shm_nattch++;
        shm_unlock(shp);
        return 0;
    }

    ret = call_mmap(sfd->file, vma) {/* shmem_mmap */
        return file->f_op->mmap(file, vma) { /* shmem_file_operations */
            shmem_mmap() {
                file_accessed(file);
                if (inode->i_nlink)
                    vma->vm_ops = &shmem_vm_ops;
                else
                    vma->vm_ops = &shmem_anon_vm_ops;
            }
        }
    }

    sfd->vm_ops = vma->vm_ops; /* shmem_vm_ops or shmem_anon_vm_ops */
    vma->vm_ops = &shm_vm_ops;

    return 0;
}
```

## shm_ops

```c
struct shm_file_data {
    int                       id;
    struct ipc_namespace      *ns;
    struct file               *file;
    const struct vm_operations_struct *vm_ops;
};

static const struct vm_operations_struct shm_vm_ops = {
    .open   = shm_open,  /* callback for a new vm-area open */
    .close  = shm_close,  /* callback for when the vm-area is released */
    .fault  = shm_fault,
};

static const struct file_operations shm_file_operations = {
    .mmap                   = shm_mmap,
    .fsync                  = shm_fsync,
    .release                = shm_release,
    .get_unmapped_area      = shm_get_unmapped_area,
    .llseek                 = noop_llseek,
    .fallocate              = shm_fallocate,
};
```


## shm_open

```c
fork() {
    tmp = vm_area_dup(mpnt);
    if (tmp->vm_ops && tmp->vm_ops->open)
        tmp->vm_ops->open(tmp);
}

copy_vma() {
    new_vma = vm_area_dup(vma);
    if (new_vma->vm_ops && new_vma->vm_ops->open)
        new_vma->vm_ops->open(new_vma);
}

/* This is called by fork, once for every shm attach. */
void shm_open(struct vm_area_struct *vma) {
    struct file *file = vma->vm_file;
    struct shm_file_data *sfd = shm_file_data(file);
    int err;

    if (sfd->vm_ops->open) {
        sfd->vm_ops->open(vma) {
            shmem_file_open(struct inode *inode, struct file *file) {
                file->f_mode |= FMODE_CAN_ODIRECT;
                return generic_file_open(inode, file) {
                    if (!(filp->f_flags & O_LARGEFILE) && i_size_read(inode) > MAX_NON_LFS)
                        return -EOVERFLOW;
                    return 0;
                }
            }
        }
    }

    err = __shm_open(sfd) {
        struct shmid_kernel *shp;

        shp = shm_lock(sfd->ns, sfd->id);

        if (shp->shm_file != sfd->file) {
            shm_unlock(shp);
            return -EINVAL;
        }

        shp->shm_atim = ktime_get_real_seconds();
        ipc_update_pid(&shp->shm_lprid, task_tgid(current));
        shp->shm_nattch++;
        shm_unlock(shp);
    }
}
```

## shm_close

```c
void shm_close(struct vm_area_struct *vma) {
    struct file *file = vma->vm_file;
    struct shm_file_data *sfd = shm_file_data(file);

    /* Always call underlying close if present */
    if (sfd->vm_ops->close) {
        sfd->vm_ops->close(vma) {

        }
    }

    __shm_close(sfd) {
        struct shmid_kernel *shp;
        struct ipc_namespace *ns = sfd->ns;

        down_write(&shm_ids(ns).rwsem);
        shp = shm_lock(ns, sfd->id);

        ipc_update_pid(&shp->shm_lprid, task_tgid(current));
        shp->shm_dtim = ktime_get_real_seconds();
        shp->shm_nattch--;

        if (shm_may_destroy(shp)) {
            shm_destroy(ns, shp);
        } else {
            shm_unlock(shp);
        }

    done:
        up_write(&shm_ids(ns).rwsem);
    }
}
```

## shm_fault

```c
static int shm_fault(struct vm_fault *vmf) {
    struct file *file = vmf->vma->vm_file;
    struct shm_file_data *sfd = shm_file_data(file);
    return sfd->vm_ops->fault(vmf); /* shmem_fault */
}
```

# sysv_sem

<img src='../images/kernel/ipc-sem.svg' style='max-height:850px'/>

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

```c
struct sem_array {
    struct kern_ipc_perm  sem_perm;  /* permissions .. see ipc.h */
    time_t                sem_ctime;  /* create/last semctl() time */
    struct list_head      pending_alter;  /* pending operations */ /* that alter the array */
    struct list_head      pending_const;  /* pending complex operations */ /* that do not alter semvals */
    struct list_head      list_id;  /* undo requests on this array */
    int                   sem_nsems;  /* no. of semaphores in array */
    int                   complex_count;  /* pending complex operations */
    unsigned int          use_global_lock;/* >0: global lock required */

    struct sem            sems[];
} __randomize_layout;

struct sem {
    int	semval; /* current value */
    /* PID of the process that last modified the semaphore. */
    struct pid *sempid;
    /* spinlock for fine-grained semtimedop */
    spinlock_t	lock;

    /* pending single-sop operations that alter the semaphore */
    struct list_head pending_alter;

    /* pending single-sop operations that do not alter the semaphore*/
    struct list_head pending_const;

    time64_t	 sem_otime;	/* candidate for sem_otime */
}
```

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

static int semctl_setval(struct ipc_namespace *ns, int semid, int semnum, int val)
{
  struct sem_undo *un;
  struct sem_array *sma;
  struct sem *curr;
  int err;

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

<img src='../images/kernel/ipc-sem-2.svg' style='max-height:850px'/>
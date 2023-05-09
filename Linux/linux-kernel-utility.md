[:link: Linux Core API Documentation :link:](https://docs.kernel.org/core-api/index.html)

* [Data structures](#data-structures)
    * [xarray](#xarray)
    * [maple tree](#maple-tree)

* [Concurrency primitives](#concurrency-primitives)
    * [atomic](#atomic)
    * [rcu](#rcu)
    * [barrier](#barrier)

* [locking](#locking)
    * [lru](#lru)
    * [futex](#futex)
    * [spinlock](#spinlock)
    * [rtmutex](#rtmutex)
    * [semaphore](#semaphore)


# Data structures

## xarray
## maple tree

# Concurrency primitives

## atomic
## rcu
## barrier


# locking
## lru

* [kernel: RCU concepts](https://www.kernel.org/doc/html/latest/RCU/index.html)

## futex

## spinlock

[LWN: spinlock](https://lwn.net/Kernel/Index/#Spinlocks)

```c
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

/* arch/arm/include/asm/spinlock_types.h */
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

    /* By using the whole 2nd least significant byte for the
     * pending bit, we can allow better optimization of the lock
     * acquisition for the pending bit holder. */
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
#define LOCK_CONTENDED(_lock, try, lock) \
do { \
  if (!try(_lock)) { \
    lock_contended(&(_lock)->dep_map, _RET_IP_); \
    lock(_lock); \
  } \
  lock_acquired(&(_lock)->dep_map, _RET_IP_); \
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
#define arch_cmpxchg(ptr, old, new) \
  __cmpxchg(ptr, old, new, sizeof(*(ptr)))

#define __cmpxchg(ptr, old, new, size) \
  __raw_cmpxchg((ptr), (old), (new), (size), LOCK_PREFIX)

#define __raw_cmpxchg(ptr, old, new, size, lock) \
({ \
  __typeof__(*(ptr)) __ret; \
  __typeof__(*(ptr)) __old = (old); \
  __typeof__(*(ptr)) __new = (new); \
  switch (size) { \
  case __X86_CASE_W: \
  { \
    volatile u16 *__ptr = (volatile u16 *)(ptr); \
    asm volatile(lock "cmpxchgw %2,%1" \
           : "=a" (__ret), "+m" (*__ptr) \
           : "r" (__new), "0" (__old) \
           : "memory"); \
    break; \
  } \
  default: \
    __cmpxchg_wrong_size(); \
  } \
  __ret; \
})
```

## mcs_spinlock

## qspinlock

## mutex


### CONFIG_PREEMPT_RT
```c
struct mutex { /* CONFIG_PREEMPT_RT  */
    struct rt_mutex_base    rtmutex;
};

/* kernel/locking/rtmutex_api.c */
mutex_lock()
    __mutex_lock_common(lock, TASK_UNINTERRUPTIBLE)
        __rt_mutex_lock()
            --->

```

### !CONFIG_PREEMPT_RT
```c
struct mutex { /* !CONFIG_PREEMPT_RT  */
    atomic_long_t       owner;
    raw_spinlock_t      wait_lock;
    struct list_head    wait_list;
};

/* kernel/locking/mutex.c */
mutex_lock()
    __mutex_trylock_fast()

    __mutex_lock_slowpath()
        __mutex_lock(lock, TASK_UNINTERRUPTIBLE)
            preempt_disable();
            ret = __mutex_trylock() {

            }
            if (ret) {
                preempt_enable();
                return 0;
            }

            raw_spin_lock(&lock->wait_lock);

            if (!use_ww_ctx) {
                __mutex_add_waiter(lock, &waiter, &lock->wait_list) {
                    list_add_tail(&waiter->list, list);
                    if (__mutex_waiter_is_first(lock, waiter))
                        __mutex_set_flag(lock, MUTEX_FLAG_WAITERS);
                }
            } else {
                ret = __ww_mutex_add_waiter(&waiter, lock, ww_ctx);
                if (ret)
                    goto err_early_kill;
            }

            for (;;) {
                if (__mutex_trylock(lock))
                    goto acquired;

                if (signal_pending_state(state, current)) {
                    ret = -EINTR;
                    goto err;
                }

                schedule_preempt_disabled() {
                    sched_preempt_enable_no_resched() {
                        barrier();
                        preempt_count_dec()
                    }
                    schedule();
                    preempt_disable() {
                        preempt_count_inc();
                        barrier();
                    }
                }

                first = __mutex_waiter_is_first(lock, &waiter);

                if (__mutex_trylock_or_handoff(lock, first))
                    break;

                raw_spin_lock(&lock->wait_lock);
            }

            acquired:
                __set_current_state(TASK_RUNNING);
                __mutex_remove_waiter(lock, &waiter);

            skip_wait:
                /* got the lock - cleanup and rejoice! */
                lock_acquired(&lock->dep_map, ip);

                if (ww_ctx)
                    ww_mutex_lock_acquired(ww, ww_ctx);

                raw_spin_unlock(&lock->wait_lock);
                preempt_enable();
                return 0;

```

## rtmutex


* [The Evolution of Real-Time Linux](https://personal.utdallas.edu/~cxl137330/courses/fall13/RTS/papers/2a.pdf)
* [RT Mutex Implementation Anatomy :cn:](https://www.cnblogs.com/hellokitty2/p/17010913.html)

![](../Images/Kernel/lock-rt-mutex.png)

```c
struct rt_mutex {
    struct rt_mutex_base    rtmutex;
};

struct rt_mutex_base {
    raw_spinlock_t          wait_lock;
    struct rb_root_cached   waiters;
    struct task_struct      *owner;
};

struct rt_mutex_waiter {
    struct rb_node          tree_entry;
    struct rb_node          pi_tree_entry;
    struct task_struct      *task;
    struct rt_mutex_base    *lock;
    unsigned int            wake_state;
    int                     prio;
    u64                     deadline;
    struct ww_acquire_ctx   *ww_ctx;
};

struct task_struct {
    raw_spinlock_t          pi_lock;
    struct rb_root_cached   pi_waiters;
    struct task_struct      *pi_top_task;
    struct rt_mutex_waiter  *pi_blocked_on;
};
```

### rt_mutex_lock
```c
__rt_mutex_lock()
    if (likely(rt_mutex_cmpxchg_acquire(lock, NULL, current)))
        return 0;

    return rt_mutex_slowlock(lock, NULL, state) {
        ret = try_to_take_rt_mutex(lock, current, NULL /*waiter*/) {
            mark_rt_mutex_waiters(lock) {
                lock->owner = lock->owner | RT_MUTEX_HAS_WAITERS;
            }

            if (rt_mutex_owner(lock))
                return 0;

            if (waiter) {
                struct rt_mutex_waiter *top_waiter = rt_mutex_top_waiter(lock);
                /* If waiter is highest priority waiter of @lock, try to take it. */
                if (waiter == top_waiter || rt_mutex_steal(waiter, top_waiter)) {
                    rt_mutex_dequeue(lock, waiter);
                } else {
                    return 0;
                }
            } else {
                if (rt_mutex_has_waiters(lock)) {
                    if (!rt_mutex_steal(task_to_waiter(task), rt_mutex_top_waiter(lock)))
                        return 0;
                } else {
                    goto takeit;
                }
            }

            task->pi_blocked_on = NULL;

            /* @task is the new owner. If other waiters exist
             * we have to insert the highest priority waiter
             * into @task->pi_waiters tree. */
            if (rt_mutex_has_waiters(lock)) {
                rt_mutex_enqueue_pi(task, rt_mutex_top_waiter(lock));
            }

        takeit:
            rt_mutex_set_owner(lock, task);

            return 1;
        }

        if (ret) {
            return 0;
        }

        set_current_state(state); /* TASK_UNINTERRUPTIBLE */

        ret = task_blocks_on_rt_mutex(lock, waiter, current) {
            waiter->task = task;
            waiter->lock = lock;
            waiter_update_prio(waiter, task);
            top_waiter = rt_mutex_top_waiter(lock);

            if (owner == task && !(build_ww_mutex() && ww_ctx))
                return -EDEADLK;

            rt_mutex_enqueue(lock, waiter);
            task->pi_blocked_on = waiter;

            if (!owner)
                return 0;

            /* lock‘s top waiter is changed to the waiter */
            if (waiter == rt_mutex_top_waiter(lock)) {
                rt_mutex_dequeue_pi(owner, top_waiter);
                rt_mutex_enqueue_pi(owner, waiter);

                /* update (boost/deboost) the prio of owner to
                 * the prio of its top pi_waiters */
                rt_mutex_adjust_prio(owner/*p*/) {
                    if (task_has_pi_waiters(p))
                        pi_task = task_top_pi_waiter(p)->task;

                    rt_mutex_setprio(p, pi_task) {
                        if (p->pi_top_task == pi_task && prio == p->prio && !dl_prio(prio))
                            return;

                        p->pi_top_task = pi_task;

                        queued = task_on_rq_queued(p);
                        running = task_current(rq, p);
                        if (queued)
                            dequeue_task(rq, p, queue_flag);
                        if (running)
                            put_prev_task(rq, p);

                        __setscheduler_prio(p, prio) {
                            p->sched_class = ;
                            p->prio = prio;
                        }

                        if (queued)
                            enqueue_task(rq, p, queue_flag);
                        if (running)
                            set_next_task(rq, p);

                        check_class_changed(rq, p, prev_class, oldprio);
                    }
                }

                if (owner->pi_blocked_on)
                    chain_walk = 1;
            } else if (rt_mutex_cond_detect_deadlock(waiter, chwalk)) {
                chain_walk = 1;
            }

            next_lock = task_blocked_on_lock(owner);

            if (!chain_walk || !next_lock)
                return 0;

            // rt_mutex_adjust_prio_chain(owner, chwalk, lock, next_lock, waiter, task)
            rt_mutex_adjust_prio_chain(owner, chwalk, orig_lock, next_lock, top_waiter, top_task) {
            again:
                if (++depth > max_lock_depth) {
                    return -EDEADLK;
                }

            /* We are fully preemptible here.
             * So everything can have changed under us */
            retry:
                waiter = task->pi_blocked_on;

                /* reach the end of the boosting chain */
                if (!waiter)
                    goto out_unlock_pi;

                /* check whether orig_waiter is changed */
                if (orig_waiter && !rt_mutex_owner(orig_lock))
                    goto out_unlock_pi;

                if (next_lock != waiter->lock)
                    goto out_unlock_pi;

                if (top_waiter) {
                    if (!task_has_pi_waiters(task))
                        goto out_unlock_pi;

                    if (top_waiter != task_top_pi_waiter(task)) {
                        if (!detect_deadlock)
                            goto out_unlock_pi;
                        else
                            requeue = false;
                    }
                }

                if (rt_mutex_waiter_equal(waiter, task_to_waiter(task))) {
                    if (!detect_deadlock)
                        goto out_unlock_pi;
                    else
                        requeue = false;
                }

                prerequeue_top_waiter = rt_mutex_top_waiter(lock);

                rt_mutex_dequeue(lock, waiter);

                waiter_update_prio(waiter, task);

                rt_mutex_enqueue(lock, waiter);

                if (!rt_mutex_owner(lock)) {
                    top_waiter = rt_mutex_top_waiter(lock);
                    if (prerequeue_top_waiter != top_waiter)
                        wake_up_state(top_waiter->task, top_waiter->wake_state) {
                            try_to_wake_up()
                        }
                    raw_spin_unlock_irq(&lock->wait_lock);
                    return 0;
                }

                if (waiter == rt_mutex_top_waiter(lock)) {
                    rt_mutex_dequeue_pi(task, prerequeue_top_waiter);
                    rt_mutex_enqueue_pi(task, waiter);
                    rt_mutex_adjust_prio(task);
                } else if (prerequeue_top_waiter == waiter) {
                    rt_mutex_dequeue_pi(task, waiter);
                    waiter = rt_mutex_top_waiter(lock);
                    rt_mutex_enqueue_pi(task, waiter);
                    rt_mutex_adjust_prio(task);
                } else {

                }

                next_lock = task_blocked_on_lock(task);

                top_waiter = rt_mutex_top_waiter(lock);

                if (!next_lock)
                    goto out_put_task;

                if (!detect_deadlock && waiter != top_waiter)
                    goto out_put_task;

                goto again;

            out_unlock_pi:
                raw_spin_unlock_irq(&task->pi_lock);
            out_put_task:
                put_task_struct(task);

                return ret;
            }
        }

        if (likely(!ret)) {
            /* perform the wait-wake-try-to-take loop */
            ret = rt_mutex_slowlock_block() {
                for (;;) {
                    if (try_to_take_rt_mutex(lock, current, waiter))
                        break;

                    if (timeout && !timeout->task) {
                        ret = -ETIMEDOUT;
                        break;
                    }
                    if (signal_pending_state(state, current)) {
                        ret = -EINTR;
                        break;
                    }

                    rtmutex_spin_on_owner = [](lock, waiter, owner) {
                        bool res = true;
                        for (;;) {
                            if (owner != rt_mutex_owner(lock))
                                break;

                            if (!owner_on_cpu(owner) || need_resched() ||
                                !rt_mutex_waiter_is_top_waiter(lock, waiter)) {

                                res = false;
                                break;
                            }
                        }
                        return res;
                    }
                    if (!is_top_waiter(waiter) || !rtmutex_spin_on_owner())
                        schedule();
                }

                __set_current_state(TASK_RUNNING);
                return ret;
            }
        }

        if (likely(!ret)) {
            if (build_ww_mutex() && ww_ctx) {
                if (!ww_ctx->is_wait_die)
                    __ww_mutex_check_waiters(rtm, ww_ctx);
                ww_mutex_lock_acquired(ww, ww_ctx);
            }
        } else {
            __set_current_state(TASK_RUNNING);
            remove_waiter(lock, waiter);
            rt_mutex_handle_deadlock(ret, chwalk, waiter);
        }

        /* try_to_take_rt_mutex() sets the waiter bit
	     * unconditionally. We might have to fix that up. */
        fixup_rt_mutex_waiters(lock, true/*acquire_lock*/) {
            if (rt_mutex_has_waiters(lock))
                return;
            if (owner & RT_MUTEX_HAS_WAITERS) {
                if (acquire_lock)
                    xchg_acquire(p, owner & ~RT_MUTEX_HAS_WAITERS);
                else
                    WRITE_ONCE(*p, owner & ~RT_MUTEX_HAS_WAITERS);
            }
        }
    }
```

### rt_mutex_unlock
```c
rt_mutex_unlock(lock)
    if (likely(rt_mutex_cmpxchg_release(lock, current, NULL)))
        return;

    rt_mutex_slowunlock(lock) {
        DEFINE_RT_WAKE_Q(wqh);

        while (!rt_mutex_has_waiters(lock)) {
            ret = unlock_rt_mutex_safe(lock, flags) {
                clear_rt_mutex_waiters(lock);
                return rt_mutex_cmpxchg_release(lock, owner, NULL);
            }
            if (ret == true)
                return;
        }

        mark_wakeup_next_waiter(&wqh, lock) {
            waiter = rt_mutex_top_waiter(lock);
            rt_mutex_dequeue_pi(current, waiter);
            rt_mutex_adjust_prio(current);

            lock->owner = (void *) RT_MUTEX_HAS_WAITERS;

            preempt_disable();
            rt_mutex_wake_q_add(wqh, waiter);
            raw_spin_unlock(&current->pi_lock);
        }

        rt_mutex_wake_up_q(&wqh) {
            if (IS_ENABLED(CONFIG_PREEMPT_RT) && wqh->rtlock_task) {
                wake_up_state(wqh->rtlock_task, TASK_RTLOCK_WAIT) {
                    try_to_wake_up(p, state, 0);
                }
                put_task_struct(wqh->rtlock_task);
                wqh->rtlock_task = NULL;
            }

            if (!wake_q_empty(&wqh->head)) {
                wake_up_q(&wqh->head) {
                    while (node != WAKE_Q_TAIL) {
                        struct task_struct *task;

                        task = container_of(node, struct task_struct, wake_q);
                        node = node->next;
                        task->wake_q.next = NULL;
                        wake_up_process(task) {
                            try_to_wake_up(p, TASK_NORMAL, 0);
                        }
                        put_task_struct(task);
                    }
                }
            }
            preempt_enable();
        }
    }
```

## semaphore

# Data structures

## xarray
## maple tree

# Concurrency primitives

## atomic
## rcu

## barrier
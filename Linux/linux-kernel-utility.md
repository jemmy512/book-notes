[:link: Linux - Core API Documentation :link:](https://docs.kernel.org/core-api/index.html)

* [Core](#core)

* [Data structures](#data-structures)
    * [xarray](#xarray)
    * [maple tree](#maple-tree)

* [Concurrency primitives](#concurrency-primitives)
    * [atomic](#atomic)
    * [rcu](#rcu)
    * [barrier](#barrier)

* [locking](#locking)
    * [barrier](#barrier)
    * [lru](#lru)
    * [futex](#futex)
    * [spinlock](#spinlock)
    * [rtmutex](#rtmutex)
    * [ww-mutex-design](#ww-mutex-design)
    * [semaphore](#semaphore)
    * [rwsem](#rwsem)


# Core

# Data structures

## xarray
## maple tree

* [LWN - Introducing maple trees](https://lwn.net/Articles/845507/)
* [The Maple Tree, A Modern Data Structure for a Complex Problem](https://blogs.oracle.com/linux/the-maple-tree)

# Concurrency primitives

## atomic
## rcu
## barrier


# locking

## barrier

## lru

* [kernel: RCU concepts](https://www.kernel.org/doc/html/latest/RCU/index.html)

## futex

![](../Images/Kernel/lock-rt-mutex.png)

```c
static struct {
    struct futex_hash_bucket *queues;
    unsigned long            hashsize;
} __futex_data;

struct futex_hash_bucket {
    atomic_t            waiters; /* number of waiters */
    spinlock_t          lock;
    struct plist_head   chain;
};

struct futex_q {
    struct plist_node       list;

    struct task_struct      *task;
    spinlock_t              *lock_ptr;
    union futex_key         key;
    struct futex_pi_state   *pi_state;
    struct rt_mutex_waiter  *rt_waiter;
    union futex_key         *requeue_pi_key;
    u32                     bitset;
    atomic_t                requeue_state;
    struct rcuwait          requeue_wait;
};

struct futex_pi_state {
    struct list_head        list;
    struct rt_mutex_base    pi_mutex;
    struct task_struct      *owner;
    refcount_t              refcount;
    union futex_key         key;
};
```

### futex_lock_pi
```c
futex_lock_pi(uaddr, flags, timeout, 0) {
    struct futex_q q = futex_q_init;

retry:
    ret = get_futex_key(uaddr, flags & FLAGS_SHARED, &q.key)

retry_private:
    hb = futex_q_lock(&q);
    ret = futex_lock_pi_atomic(uaddr, hb, &q.key, &q.pi_state) {
        top_waiter = futex_top_waiter(hb, key);
        if (top_waiter)
            return attach_to_pi_state(uaddr, uval, top_waiter->pi_state, ps);

        return attach_to_pi_owner(uaddr, newval, key, ps, exiting) {
            __attach_to_pi_owner(p, key, ps) {
                pi_state = alloc_pi_state();
                rt_mutex_init_proxy_locked(&pi_state->pi_mutex, p);

                pi_state->key = *key;

                list_add(&pi_state->list, &p->pi_state_list);

                pi_state->owner = p;

                *ps = pi_state;
            }
        }
    }
    if (unlikely(ret)) {
        switch (ret) {
        case 1: /* We got the lock. */
            ret = 0;
            goto out_unlock_put_key;
        }
    }
    __futex_queue(&q, hb);

    if (trylock) {
        ret = rt_mutex_futex_trylock(&q.pi_state->pi_mutex);
        /* Fixup the trylock return value: */
        ret = ret ? 0 : -EWOULDBLOCK;
        goto no_block;
    }

    rt_mutex_init_waiter(&rt_waiter);

    /* Start lock acquisition for another task */
    ret = __rt_mutex_start_proxy_lock(&q.pi_state->pi_mutex, &rt_waiter, current);
    if (ret) {
        if (ret == 1)
            ret = 0;
        goto cleanup;
    }

    /* Wait for lock acquisition */
    rt_mutex_wait_proxy_lock(&q.pi_state->pi_mutex, to, &rt_waiter);

cleanup:
    if (ret && !rt_mutex_cleanup_proxy_lock(&q.pi_state->pi_mutex, &rt_waiter))
        ret = 0;

no_block:
    fixup_pi_owner(uaddr, &q, !ret);
    futex_unqueue_pi(&q);
    spin_unlock(q.lock_ptr);
    goto out;

out_unlock_put_key:
    futex_q_unlock(hb);

out:
    if (to) {
        hrtimer_cancel(&to->timer);
        destroy_hrtimer_on_stack(&to->timer);
    }
    return ret != -EINTR ? ret : -ERESTARTNOINTR;

uaddr_faulted:
    futex_q_unlock(hb);

    ret = fault_in_user_writeable(uaddr);
    if (ret)
        goto out;

    if (!(flags & FLAGS_SHARED))
        goto retry_private;

    goto retry;
}
```

```c
futex_unlock_pi(uaddr, flags);
```

```c
futex_wait(uaddr, flags, val, abs_time, bitset)
    to = futex_setup_timer()

retry:
    ret = futex_wait_setup(uaddr, val, flags, &q, &hb) {
        ret = get_futex_key(uaddr, flags & FLAGS_SHARED, &q->key, FUTEX_READ) {
            /* private: current->mm, address, 0
             * share: inode->i_sequence, page->index, offset_within_page */
        }
        ret = futex_get_value_locked(&uval, uaddr);
    }
    if (ret)
        goto out;

    futex_wait_queue(hb, &q, to) {
        set_current_state(TASK_INTERRUPTIBLE|TASK_FREEZABLE);
        futex_queue(q, hb) {
            plist_node_init(&q->list, prio);
            plist_add(&q->list, &hb->chain);
            q->task = current;
        }
        __set_current_state(TASK_RUNNING);
    }

    if (!futex_unqueue(&q))
        goto out;

    if (!signal_pending(current))
        goto retry;

    ret = set_restart_fn(restart, futex_wait_restart);

out:
    if (to) {
        hrtimer_cancel(&to->timer);
        destroy_hrtimer_on_stack(&to->timer);
    }
```

```c
futex_wake(uaddr, flags, nr_wake, bitest)
    DEFINE_WAKE_Q(wake_q);
    get_futex_key(uaddr, flags & FLAGS_SHARED, &key, FUTEX_READ);
    hb = futex_hash(&key);

    /* just return if no waiter */
    if (!futex_hb_waiters_pending(hb))
        return ret;

    plist_for_each_entry_safe(this, next, &hb->chain, list) {
        if (futex_match (&this->key, &key)) {
            if (this->pi_state || this->rt_waiter) {
                ret = -EINVAL;
                break;
            }

            /* Check if one of the bits is set in both bitsets */
            if (!(this->bitset & bitset))
                continue;

            futex_wake_mark(&wake_q, this);
            if (++ret >= nr_wake)
                break;
        }
    }

    spin_unlock(&hb->lock);
    wake_up_q(&wake_q);
```

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

## rwsem

* https://mp.weixin.qq.com/s/oNa5urBSdMlq41htxJ5miQ

```c
struct rw_semaphore {
    /* Bit  0    - RWSEM_WRITER_LOCKED
     * Bit  1    - RWSEM_FLAG_WAITERS
     * Bit  2    - RWSEM_FLAG_HANDOFF
     * Bits 3-7  - reserved
     * Bits 8-62 - 55-bit reader count
     * Bit  63   - read fail bit */
    atomic_long_t count;

    /* Bit 0: RWSEM_READER_OWNED - The rwsem is owned by readers
     * Bit 1: RWSEM_NONSPINNABLE - Cannot spin on a reader-owned lock */
    atomic_long_t owner;

    struct optimistic_spin_queue osq; /* atomic_t: spinner MCS lock */
    raw_spinlock_t wait_lock;
    struct list_head wait_list;
};

enum rwsem_waiter_type {
    RWSEM_WAITING_FOR_WRITE,
    RWSEM_WAITING_FOR_READ
};

struct rwsem_waiter {
    struct list_head        list;
    struct task_struct      *task;
    enum rwsem_waiter_type  type;
    unsigned long           timeout;
    bool                    handoff_set;
};
```

### down_read
```c
down_read(struct rw_semaphore *sem)
    might_sleep();
    LOCK_CONTENDED(sem, __down_read_trylock, __down_read) {
        #define LOCK_CONTENDED(_lock, try, lock) \
        do { \
            if (!try(_lock)) { \
                lock(_lock); \
            } \
        } while (0)
    }


#define RWSEM_READ_FAILED_MASK \
(RWSEM_WRITER_MASK|RWSEM_FLAG_WAITERS|RWSEM_FLAG_HANDOFF|RWSEM_FLAG_READFAIL)

int ret = __down_read_trylock(sem) {
    tmp = atomic_long_read(&sem->count);
    while (!(tmp & RWSEM_READ_FAILED_MASK)) {
        if (atomic_long_try_cmpxchg_acquire(&sem->count, &tmp, tmp + RWSEM_READER_BIAS)) {
            rwsem_set_reader_owned(sem) {
                unsigned long val = (unsigned long)owner | RWSEM_READER_OWNED |
                    (atomic_long_read(&sem->owner) & RWSEM_NONSPINNABLE);

                atomic_long_set(&sem->owner, val);
            }
            ret = 1;
            break;
        }
    }
}

__down_read(struct rw_semaphore *sem) {
    __down_read_common(sem, TASK_UNINTERRUPTIBLE) {
        preempt_disable();

        /* add reader count unconditionally */
        ret = rwsem_read_trylock(sem, &count) {
            /* reader count +1 (RWSEM_READER_BIAS) */
            *cntp = atomic_long_add_return_acquire(RWSEM_READER_BIAS, &sem->count);

            /* RWSEM_FLAG_READFAIL bit is set
             * too many readers, disable spin */
            if (WARN_ON_ONCE(*cntp < 0)) {
                rwsem_set_nonspinnable(sem) {
                    unsigned long owner = atomic_long_read(&sem->owner);
                    do {
                        if (!(owner & RWSEM_READER_OWNED))
                            break;
                        if (owner & RWSEM_NONSPINNABLE)
                            break;
                    } while (!atomic_long_try_cmpxchg(
                        &sem->owner, &owner, owner | RWSEM_NONSPINNABLE)
                    );
                }
            }

            if (!(*cntp & RWSEM_READ_FAILED_MASK)) {
                rwsem_set_reader_owned(sem)
                    --->
                return true;
            }
            return false;
        }
        if (!ret) {
            ret = rwsem_down_read_slowpath(sem, count, state) {
                long adjustment = -RWSEM_READER_BIAS;
                long rcnt = (count >> RWSEM_READER_SHIFT);
                struct rwsem_waiter waiter;
                DEFINE_WAKE_Q(wake_q);

                /* To prevent a constant stream of readers from starving a sleeping
                 * waiter, don't attempt optimistic lock stealing if the lock is
                 * currently owned by readers. */
                if ((atomic_long_read(&sem->owner) & RWSEM_READER_OWNED)
                    && (rcnt > 1) && !(count & RWSEM_WRITER_LOCKED)) {

                    goto queue;
                }

                /* Reader optimistic lock stealing. */
                if (!(count & (RWSEM_WRITER_LOCKED | RWSEM_FLAG_HANDOFF))) {
                    rwsem_set_reader_owned(sem);
                        --->
                    /* Wake up other readers in the wait queue if it is
                     * the first reader. */
                    if ((rcnt == 1) && (count & RWSEM_FLAG_WAITERS)) {
                        raw_spin_lock_irq(&sem->wait_lock);
                        if (!list_empty(&sem->wait_list)) {
                            rwsem_mark_wake(sem, RWSEM_WAKE_READ_OWNED, &wake_q) {
                                waiter = rwsem_first_waiter(sem);

                                /* just wakeup one waiter if top waiter waits for write */
                                if (waiter->type == RWSEM_WAITING_FOR_WRITE) {
                                    if (wake_type == RWSEM_WAKE_ANY) {
                                        wake_q_add(wake_q, waiter->task);
                                    }

                                    return;
                                }

                                /* let top waiter owne lock to prevent
                                 * writer steal lock when lock is free*/
                                if (wake_type != RWSEM_WAKE_READ_OWNED) {
                                    struct task_struct *owner;

                                    adjustment = RWSEM_READER_BIAS;
                                    oldcount = atomic_long_fetch_add(adjustment, &sem->count);
                                    /* writer steal the lock*/
                                    if (unlikely(oldcount & RWSEM_WRITER_MASK)) {
                                        /* reader is blocked too long, set handoff flag to require
                                         * the writer give the lock to reader */
                                        if (time_after(jiffies, waiter->timeout)) {
                                            if (!(oldcount & RWSEM_FLAG_HANDOFF)) {
                                                adjustment -= RWSEM_FLAG_HANDOFF;
                                                lockevent_inc(rwsem_rlock_handoff);
                                            }
                                            waiter->handoff_set = true;
                                        }

                                        atomic_long_add(-adjustment, &sem->count);
                                        return;
                                    }
                                    owner = waiter->task;
                                    __rwsem_set_reader_owned(sem, owner);
                                }

                                /* wakeup a group readers
                                 * 1) Collect the read-waiters in a separate list, count them and
                                 * fully increment the reader count in rwsem. */
                                INIT_LIST_HEAD(&wlist);
                                list_for_each_entry_safe(waiter, tmp, &sem->wait_list, list) {
                                    if (waiter->type == RWSEM_WAITING_FOR_WRITE)
                                        continue;
                                    woken++;
                                    list_move_tail(&waiter->list, &wlist);
                                    if (unlikely(woken >= MAX_READERS_WAKEUP))
                                        break;
                                }
                                adjustment = woken * RWSEM_READER_BIAS - adjustment;

                                /* writers try to acquire lock after they are woken up
                                 * while readers acquire the lock here before they are woken up */
                                oldcount = atomic_long_read(&sem->count);
                                if (list_empty(&sem->wait_list)) {
                                    adjustment -= RWSEM_FLAG_WAITERS;
                                    if (oldcount & RWSEM_FLAG_HANDOFF)
                                        adjustment -= RWSEM_FLAG_HANDOFF;
                                } else if (woken) {
                                    if (oldcount & RWSEM_FLAG_HANDOFF)
                                        adjustment -= RWSEM_FLAG_HANDOFF;
                                }
                                if (adjustment)
                                    atomic_long_add(adjustment, &sem->count);

                                /* 2) For each waiters in the new list, clear waiter->task and
                                 * put them into wake_q to be woken up later. */
                                 list_for_each_entry_safe(waiter, tmp, &wlist, list) {
                                    struct task_struct *tsk = waiter->task;
                                    get_task_struct(tsk);
                                    smp_store_release(&waiter->task, NULL);
                                    wake_q_add_safe(wake_q, tsk);
                                }
                            }
                        }
                        raw_spin_unlock_irq(&sem->wait_lock);
                        wake_up_q(&wake_q);
                    }
                    return sem;
                }

            queue:
                waiter.task = current;
                waiter.type = RWSEM_WAITING_FOR_READ;
                waiter.timeout = jiffies + RWSEM_WAIT_TIMEOUT;
                waiter.handoff_set = false;

                /* In case the wait queue is empty and the lock isn't owned
                 * by a writer, this reader can exit the slowpath and return
                 * immediately as its RWSEM_READER_BIAS has already been set
                 * in the count. */
                raw_spin_lock_irq(&sem->wait_lock);
                if (list_empty(&sem->wait_list)) {
                    if (!(atomic_long_read(&sem->count) & RWSEM_WRITER_MASK)) {
                        raw_spin_unlock_irq(&sem->wait_lock);
                        rwsem_set_reader_owned(sem);
                            --->
                        return sem;
                    }
                    /* add flag only for the first waiter */
                    adjustment += RWSEM_FLAG_WAITERS;
                }
                rwsem_add_waiter(sem, &waiter);

                /* we're now waiting on the lock, but no longer actively locking */
                count = atomic_long_add_return(adjustment, &sem->count);

                rwsem_cond_wake_waiter(sem, count, &wake_q) {
                    if (count & RWSEM_WRITER_MASK)
                        return;

                    if (count & RWSEM_READER_MASK) {
                        wake_type = RWSEM_WAKE_READERS;
                    } else {
                        wake_type = RWSEM_WAKE_ANY;
                        clear_nonspinnable(sem);
                    }
                    rwsem_mark_wake(sem, wake_type, wake_q);
                        --->
                }
                raw_spin_unlock_irq(&sem->wait_lock);

                if (!wake_q_empty(&wake_q))
                    wake_up_q(&wake_q);

                /* wait to be given the lock */
                for (;;) {
                    set_current_state(state);
                    /* Matches rwsem_mark_wake()'s smp_store_release(). */
                    if (!smp_load_acquire(&waiter.task)) {
                        break;
                    }
                    if (signal_pending_state(state, current)) {
                        raw_spin_lock_irq(&sem->wait_lock);
                        if (waiter.task)
                            goto out_nolock;
                        raw_spin_unlock_irq(&sem->wait_lock);
                        /* Ordered by sem->wait_lock against rwsem_mark_wake(). */
                        break;
                    }
                    schedule_preempt_disabled();
                }

                __set_current_state(TASK_RUNNING);
                return sem;

            out_nolock:
                rwsem_del_wake_waiter(sem, &waiter, &wake_q) {
                    bool first = rwsem_first_waiter(sem) == waiter;
                    wake_q_init(wake_q);

                    if (rwsem_del_waiter(sem, waiter) && first)
                        rwsem_mark_wake(sem, RWSEM_WAKE_ANY, wake_q);
                            --->

                    raw_spin_unlock_irq(&sem->wait_lock);
                    if (!wake_q_empty(wake_q))
                        wake_up_q(wake_q);
                }
                __set_current_state(TASK_RUNNING);
                return ERR_PTR(-EINTR);
            }
            if (IS_ERR(ret)) {
                ret = -EINTR;
                goto out;
            }
            DEBUG_RWSEMS_WARN_ON(!is_rwsem_reader_owned(sem), sem);
        }
    out:
        preempt_enable();
    }
}
```

### up_read
```c
__up_read(struct rw_semaphore *sem)
    preempt_disable();
    rwsem_clear_reader_owned(sem) {
        unsigned long val = atomic_long_read(&sem->owner);
        while ((val & ~RWSEM_OWNER_FLAGS_MASK) == (unsigned long)current) {
            if (atomic_long_try_cmpxchg(&sem->owner, &val, val & RWSEM_OWNER_FLAGS_MASK))
                return;
        }
    }

    tmp = atomic_long_add_return_release(-RWSEM_READER_BIAS, &sem->count);

    if (unlikely((tmp & (RWSEM_LOCK_MASK|RWSEM_FLAG_WAITERS)) == RWSEM_FLAG_WAITERS)) {
        clear_nonspinnable(sem);
        rwsem_wake(sem) {
            DEFINE_WAKE_Q(wake_q);
            if (!list_empty(&sem->wait_list))
                rwsem_mark_wake(sem, RWSEM_WAKE_ANY, &wake_q);
                    --->
            wake_up_q(&wake_q);
        }
    }
    preempt_enable();
```

### down_write
```c
down_write(struct rw_semaphore *sem)
    might_sleep();
    rwsem_acquire(&sem->dep_map, 0, 0, _RET_IP_);
    LOCK_CONTENDED(sem, __down_write_trylock, __down_write);

static inline int __down_write_trylock(struct rw_semaphore *sem)
    preempt_disable();
    ret = rwsem_write_trylock(sem) {
        long tmp = RWSEM_UNLOCKED_VALUE;

        if (atomic_long_try_cmpxchg_acquire(&sem->count, &tmp, RWSEM_WRITER_LOCKED)) {
            rwsem_set_owner(sem);
            return true;
        }

        return false;
    }
    preempt_enable();

    return ret;

__down_write(struct rw_semaphore *sem)
    __down_write_common(sem, TASK_UNINTERRUPTIBLE) {
        preempt_disable();
        ret = rwsem_write_trylock(sem) {
            long tmp = RWSEM_UNLOCKED_VALUE;
            if (atomic_long_try_cmpxchg_acquire(&sem->count, &tmp, RWSEM_WRITER_LOCKED)) {
                rwsem_set_owner(sem);
                    atomic_long_set(&sem->owner, (long)current);
                return true;
            }
            return false;
        }
        if (unlikely(!ret)) {
            ret = rwsem_down_write_slowpath(sem, state) {
                struct rwsem_waiter waiter;
                DEFINE_WAKE_Q(wake_q);

                /* do optimistic spinning and steal lock if possible */
                ret = rwsem_can_spin_on_owner(sem) {

                }
                if (ret) {
                    ret = rwsem_optimistic_spin(sem) {
                        if (!osq_lock(&sem->osq))
                            goto done;
                        for (;;) {
                            enum owner_state owner_state;

                            owner_state = rwsem_spin_on_owner(sem);
                                --->
                            if (!(owner_state & OWNER_SPINNABLE))
                                break;

                            taken = rwsem_try_write_lock_unqueued(sem) {
                                long count = atomic_long_read(&sem->count);

                                while (!(count & (RWSEM_LOCK_MASK|RWSEM_FLAG_HANDOFF))) {
                                    if (atomic_long_try_cmpxchg_acquire(&sem->count, &count,
                                                count | RWSEM_WRITER_LOCKED)) {
                                        rwsem_set_owner(sem);
                                        lockevent_inc(rwsem_opt_lock);
                                        return true;
                                    }
                                }
                                return false;
                            }

                            if (taken)
                                break;

                            /* Time-based reader-owned rwsem optimistic spinning */
                            if (owner_state == OWNER_READER) {
                                if (prev_owner_state != OWNER_READER) {
                                    if (rwsem_test_oflags(sem, RWSEM_NONSPINNABLE))
                                        break;
                                    rspin_threshold = rwsem_rspin_threshold(sem);
                                    loop = 0;
                                } else if (!(++loop & 0xf) && (sched_clock() > rspin_threshold)) {
                                    rwsem_set_nonspinnable(sem);
                                    break;
                                }
                            }

                            if (owner_state != OWNER_WRITER) {
                                if (need_resched())
                                    break;
                                if (rt_task(current) && (prev_owner_state != OWNER_WRITER))
                                    break;
                            }
                            prev_owner_state = owner_state;

                            cpu_relax();
                        }
                        osq_unlock(&sem->osq);

                    done:
                        lockevent_cond_inc(rwsem_opt_fail, !taken);
                        return taken;
                    }
                    if (ret) {
                        /* rwsem_optimistic_spin() implies ACQUIRE on success */
                        return sem;
                    }
                }

                waiter.task = current;
                waiter.type = RWSEM_WAITING_FOR_WRITE;
                waiter.timeout = jiffies + RWSEM_WAIT_TIMEOUT;
                waiter.handoff_set = false;

                raw_spin_lock_irq(&sem->wait_lock);
                rwsem_add_waiter(sem, &waiter);

                /* we're now waiting on the lock */
                if (rwsem_first_waiter(sem) != &waiter) {
                    rwsem_cond_wake_waiter(sem, atomic_long_read(&sem->count), &wake_q);
                        --->

                    if (!wake_q_empty(&wake_q)) {
                        /* We want to minimize wait_lock hold time especially
                         * when a large number of readers are to be woken up. */
                        raw_spin_unlock_irq(&sem->wait_lock);
                        wake_up_q(&wake_q);
                        raw_spin_lock_irq(&sem->wait_lock);
                    }
                } else {
                    atomic_long_or(RWSEM_FLAG_WAITERS, &sem->count);
                }

                for (;;) {
                    /* rwsem_try_write_lock() implies ACQUIRE on success */
                    ret = rwsem_try_write_lock(sem, &waiter) {
                        struct rwsem_waiter *first = rwsem_first_waiter(sem);
                        long count, new;

                        count = atomic_long_read(&sem->count);
                        do {
                            bool has_handoff = !!(count & RWSEM_FLAG_HANDOFF);

                            if (has_handoff) {
                                if (first->handoff_set && (waiter != first))
                                    return false;
                            }

                            new = count;

                            if (count & RWSEM_LOCK_MASK) {
                                if (has_handoff || (!rt_task(waiter->task) && !time_after(jiffies, waiter->timeout)))
                                    return false;

                                new |= RWSEM_FLAG_HANDOFF;
                            } else {
                                new |= RWSEM_WRITER_LOCKED;
                                new &= ~RWSEM_FLAG_HANDOFF;

                                if (list_is_singular(&sem->wait_list))
                                    new &= ~RWSEM_FLAG_WAITERS;
                            }
                        } while (!atomic_long_try_cmpxchg_acquire(&sem->count, &count, new));

                        if (new & RWSEM_FLAG_HANDOFF) {
                            first->handoff_set = true;
                            return false;
                        }

                        list_del(&waiter->list);
                        rwsem_set_owner(sem);
                        return true;
                    }
                    if (ret) {
                        break;
                    }

                    raw_spin_unlock_irq(&sem->wait_lock);

                    if (signal_pending_state(state, current))
                        goto out_nolock;

                    if (waiter.handoff_set) {
                        enum owner_state owner_state;

                        owner_state = rwsem_spin_on_owner(sem) {
                            owner = rwsem_owner_flags(sem, &flags);
                            state = rwsem_owner_state(owner, flags);
                            if (state != OWNER_WRITER)
                                return state;

                            for (;;) {
                                new = rwsem_owner_flags(sem, &new_flags);
                                if ((new != owner) || (new_flags != flags)) {
                                    state = rwsem_owner_state(new, new_flags);
                                    break;
                                }
                                barrier();
                                if (need_resched() || !owner_on_cpu(owner)) {
                                    state = OWNER_NONSPINNABLE;
                                    break;
                                }
                                cpu_relax();
                            }
                            return state;
                        }
                        if (owner_state == OWNER_NULL)
                            goto trylock_again;
                    }

                    schedule_preempt_disabled();
                    lockevent_inc(rwsem_sleep_writer);
                    set_current_state(state);

                    /* writers try to acquire lock after they are woken up
                     * while readers acquire the lock here before they are woken up */
            trylock_again:
                    raw_spin_lock_irq(&sem->wait_lock);
                }
                __set_current_state(TASK_RUNNING);
                raw_spin_unlock_irq(&sem->wait_lock);
                lockevent_inc(rwsem_wlock);
                trace_contention_end(sem, 0);
                return sem;

            out_nolock:
                __set_current_state(TASK_RUNNING);
                raw_spin_lock_irq(&sem->wait_lock);
                rwsem_del_wake_waiter(sem, &waiter, &wake_q);
                lockevent_inc(rwsem_wlock_fail);
                trace_contention_end(sem, -EINTR);
                return ERR_PTR(-EINTR);

            }
            if (IS_ERR(ret))
                ret = -EINTR;
        }
        preempt_enable();
    }
```

### up_write
```c
__up_write(struct rw_semaphore *sem)
    preempt_disable();
    rwsem_clear_owner(sem);
    tmp = atomic_long_fetch_add_release(-RWSEM_WRITER_LOCKED, &sem->count);
    if (unlikely(tmp & RWSEM_FLAG_WAITERS))
        rwsem_wake(sem) {
            unsigned long flags;
            DEFINE_WAKE_Q(wake_q);

            raw_spin_lock_irqsave(&sem->wait_lock, flags);
            if (!list_empty(&sem->wait_list)) {
                rwsem_mark_wake(sem, RWSEM_WAKE_ANY, &wake_q);
                    --->
            }
            raw_spin_unlock_irqrestore(&sem->wait_lock, flags);
            wake_up_q(&wake_q);

            return sem;
        }
    preempt_enable();
```

# Data structures

## xarray
## maple tree

# Concurrency primitives

## atomic
## rcu

## barrier
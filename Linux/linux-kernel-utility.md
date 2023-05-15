[:link: Linux - Core API Documentation :link:](https://docs.kernel.org/core-api/index.html)

* [Core](#core)
    * [cmwq](#cmwq)

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


# Core

## cmwq

* [Kernel Doc](https://docs.kernel.org/core-api/workqueue.html)

<img src='../Images/Kernel/proc-cmwq.png' style='max-height:850px'/>

---

<img src='../Images/Kernel/proc-cmwq-flow.png' style='max-height:850px'/>

* `nr_running`    `nr_active`    `max_active`    `CPU_INTENSIVE` control the concurrency
---

<img src='../Images/Kernel/proc-cmwq-state.png' style='max-height:850px'/>

---

<img src='../Images/Kernel/proc-cmwq-arch.png' style='max-height:850px'/>

* [Kernel 4.19: Concurrency Managed Workqueue (cmwq)](https://www.kernel.org/doc/html/v4.19/core-api/workqueue.html)
* http://www.wowotech.net/irq_subsystem/cmwq-intro.html :cn:
* https://zhuanlan.zhihu.com/p/91106844 :cn:
* https://zhuanlan.zhihu.com/p/94561631 :cn:
* http://kernel.meizu.com/linux-workqueue.html :cn:

### wq-struct
```c++
struct workqueue_struct {
  struct list_head      pwqs;  /* WR: all pwqs of this wq */
  struct list_head      list;  /* PR: list of all workqueues */

  struct list_head      maydays; /* MD: pwqs requesting rescue */
  struct worker         *rescuer; /* I: rescue worker */

  struct pool_workqueue *dfl_pwq; /* PW: only for unbound wqs */
  struct pool_workqueue *cpu_pwqs; /* I: per-cpu pwqs */
  struct pool_workqueue *numa_pwq_tbl[]; /* PWR: unbound pwqs indexed by node */
};

/* The per-pool workqueue. */
struct pool_workqueue {
  struct worker_pool      *pool;    /* I: the associated pool */
  struct list_head        inactive_works;  /* L: inactive works */
  struct list_head        mayday_node;  /* MD: node on wq->maydays */
  struct list_head        pwqs_node;  /* WR: node on wq->pwqs */
  struct workqueue_struct *wq;    /* I: the owning workqueue */
  int                     work_color;  /* L: current color */
  int                     flush_color;  /* L: flushing color */
  int                     refcnt;    /* L: reference count */
  int                     nr_in_flight[WORK_NR_COLORS];/* L: nr of in_flight works */
  int                     nr_active;  /* L: nr of active works */
  int                     max_active;  /* L: max active works, the elements of pool->worklist */

  /* Release of unbound pwq is punted to system_wq.  See put_pwq()
   * and pwq_unbound_release_workfn() for details.  pool_workqueue
   * itself is also sched-RCU protected so that the first pwq can be
   * determined without grabbing wq->mutex. */
  struct work_struct  unbound_release_work;
  struct rcu_head    rcu;
} __aligned(1 << WORK_STRUCT_FLAG_BITS);

struct worker_pool {
  spinlock_t          lock;   /* the pool lock */
  int                 cpu;    /* I: the associated cpu */
  int                 node;   /* I: the associated node ID */
  int                 id;     /* I: pool ID */
  unsigned int        flags;  /* X: flags */

  unsigned long       watchdog_ts;  /* L: watchdog timestamp */

  struct list_head    worklist;  /* L: list of pending works */

  int                 nr_workers;  /* L: total number of workers */
  int                 nr_idle;  /* L: currently idle workers */

  struct list_head    workers;  /* A: attached workers */
  struct list_head    idle_list;  /* X: list of idle workers */
  /* a workers is either on busy_hash or idle_list, or the manager */
  DECLARE_HASHTABLE(busy_hash, BUSY_WORKER_HASH_ORDER); /* L: hash of busy workers */

  struct timer_list   idle_timer;  /* L: worker idle timeout */
  struct timer_list   mayday_timer;  /* L: SOS timer for workers */

  struct worker       *manager;  /* L: purely informational */
  struct completion   *detach_completion; /* all workers detached */

  struct ida          worker_ida;  /* worker IDs for task name */

  struct workqueue_attrs  *attrs;    /* I: worker attributes */
  struct hlist_node       hash_node;  /* PL: unbound_pool_hash node */
  int                     refcnt;    /* PL: refcnt for unbound pools */

  /* The current concurrency level.  As it's likely to be accessed
   * from other CPUs during try_to_wake_up(), put it in a separate
   * cacheline. */
  atomic_t    nr_running ____cacheline_aligned_in_smp;

  /* Destruction of pool is sched-RCU protected to allow dereferences
   * from get_work_pool(). */
  struct rcu_head    rcu;
} ____cacheline_aligned_in_smp;

struct workqueue_attrs {
  int             nice;
  cpumask_var_t   cpumask;
  bool            no_numa;
};

/* worker flags */
WORKER_DIE            = 1 << 1,  /* die die die */
WORKER_IDLE           = 1 << 2,  /* is idle */
WORKER_PREP           = 1 << 3,  /* preparing to run works */
WORKER_CPU_INTENSIVE  = 1 << 6,  /* cpu intensive */
WORKER_UNBOUND        = 1 << 7,  /* worker is unbound */
WORKER_REBOUND        = 1 << 8,  /* worker was rebound */
WORKER_NOT_RUNNING    = WORKER_PREP | WORKER_CPU_INTENSIVE | WORKER_UNBOUND | WORKER_REBOUND;

struct worker {
  /* on idle list while idle, on busy hash table while busy */
  union {
    struct list_head      entry;  /* L: while idle */
    struct hlist_node     hentry;  /* L: while busy */
  };

  struct work_struct      *current_work;  /* L: work being processed */
  work_func_t             current_func;   /* L: current_work's fn */
  struct pool_workqueue   *current_pwq;   /* L: current_work's pwq */
  struct list_head        scheduled;      /* L: scheduled works */

  /* 64 bytes boundary on 64bit, 32 on 32bit */

  struct task_struct      *task;    /* I: worker task */
  struct worker_pool      *pool;    /* A: the associated pool */
            /* L: for rescuers */
  struct list_head        node;    /* A: anchored at pool->workers */
            /* A: runs through worker->node */

  unsigned long           last_active;  /* L: last active timestamp */
  unsigned int            flags;    /* X: flags */
  int                     id;    /* I: worker id */

  /* Opaque string set with work_set_desc().  Printed out with task
   * dump for debugging - WARN, BUG, panic or sysrq. */
  char      desc[WORKER_DESC_LEN];

  /* used only by rescuers to point to the target workqueue */
  struct workqueue_struct  *rescue_wq;  /* I: the workqueue to rescue */
};

struct work_struct {
  atomic_long_t     data;
  struct list_head  entry;
  work_func_t       func;
};
```

```c++
[root@VM-16-17-centos ~]# ps -ef | grep worker
root           6       2  0  2021 ?        00:00:00 [kworker/0:0H-events_highpri]
root          33       2  0  2021 ?        00:01:41 [kworker/0:1H-kblockd]
root     2747154       2  0 Jan15 ?        00:00:00 [kworker/0:8-events]
root     2751953       2  0 00:09 ?        00:00:00 [kworker/0:1-ata_sff]
root     2756345       2  0 00:30 ?        00:00:00 [kworker/0:6-events]
root     2756347       2  0 00:30 ?        00:00:00 [kworker/0:7-cgroup_pidlist_destroy]
root     2757595       2  0 00:36 ?        00:00:00 [kworker/0:0-cgroup_pidlist_destroy]
root     2757754       2  0 00:37 ?        00:00:00 [kworker/u2:1-events_unbound]
root     2759049       2  0 00:43 ?        00:00:00 [kworker/u2:2-events_unbound]
root     2760459 2760373  0 00:48 pts/4    00:00:00 grep --color=auto worker

kworker/<cpu-id>:<worker-id-in-pool><High priority>
kworker/<unbound>:<worker-id-in-pool><High priority>
```
```c++
/* the per-cpu worker pools */
static DEFINE_PER_CPU_SHARED_ALIGNED(struct worker_pool [NR_STD_WORKER_POOLS], cpu_worker_pools);

static DEFINE_IDR(worker_pool_idr);  /* PR: idr of all pools */

/* PL: hash of all unbound pools keyed by pool->attrs */
static DEFINE_HASHTABLE(unbound_pool_hash, UNBOUND_POOL_HASH_ORDER);

/* I: attributes used when instantiating standard unbound pools on demand */
static struct workqueue_attrs *unbound_std_wq_attrs[NR_STD_WORKER_POOLS];

/* I: attributes used when instantiating ordered pools on demand */
static struct workqueue_attrs *ordered_wq_attrs[NR_STD_WORKER_POOLS];

struct workqueue_struct *system_wq;
struct workqueue_struct *system_highpri_wq;
struct workqueue_struct *system_long_wq;
struct workqueue_struct *system_unbound_wq;
struct workqueue_struct *system_freezable_wq;
struct workqueue_struct *system_power_efficient_wq;
struct workqueue_struct *system_freezable_power_efficient_wq;
```

### alloc_workqueue
```c++
void __init start_kernel(void)
{
  kernel_init_freeable();
  workqueue_init_early();
}

int __init workqueue_init_early(void)
{
  int std_nice[NR_STD_WORKER_POOLS] = { 0, HIGHPRI_NICE_LEVEL };
  int hk_flags = HK_FLAG_DOMAIN | HK_FLAG_WQ;
  int i, cpu;

  WARN_ON(__alignof__(struct pool_workqueue) < __alignof__(long long));

  BUG_ON(!alloc_cpumask_var(&wq_unbound_cpumask, GFP_KERNEL));
  cpumask_copy(wq_unbound_cpumask, housekeeping_cpumask(hk_flags));

  pwq_cache = KMEM_CACHE(pool_workqueue, SLAB_PANIC);

  /* initialize CPU pools */
  for_each_possible_cpu(cpu) {
    struct worker_pool *pool;

    i = 0;
    for_each_cpu_worker_pool(pool, cpu) {
      BUG_ON(init_worker_pool(pool));
      pool->cpu = cpu;
      cpumask_copy(pool->attrs->cpumask, cpumask_of(cpu));
      pool->attrs->nice = std_nice[i++];
      pool->node = cpu_to_node(cpu);

      /* alloc pool ID */
      mutex_lock(&wq_pool_mutex);
      BUG_ON(worker_pool_assign_id(pool));
      mutex_unlock(&wq_pool_mutex);
    }
  }

  /* create default unbound and ordered wq attrs */
  for (i = 0; i < NR_STD_WORKER_POOLS; i++) {
    struct workqueue_attrs *attrs;

    BUG_ON(!(attrs = alloc_workqueue_attrs(GFP_KERNEL)));
    attrs->nice = std_nice[i];
    unbound_std_wq_attrs[i] = attrs;

    /* An ordered wq should have only one pwq as ordering is
     * guaranteed by max_active which is enforced by pwqs.
     * Turn off NUMA so that dfl_pwq is used for all nodes. */
    BUG_ON(!(attrs = alloc_workqueue_attrs(GFP_KERNEL)));
    attrs->nice = std_nice[i];
    attrs->no_numa = true;
    ordered_wq_attrs[i] = attrs;
  }

  system_wq = alloc_workqueue("events", 0, 0);
  system_highpri_wq = alloc_workqueue("events_highpri", WQ_HIGHPRI, 0);
  system_long_wq = alloc_workqueue("events_long", 0, 0);
  system_unbound_wq = alloc_workqueue("events_unbound", WQ_UNBOUND, WQ_UNBOUND_MAX_ACTIVE);
  system_freezable_wq = alloc_workqueue("events_freezable", WQ_FREEZABLE, 0);
  system_power_efficient_wq = alloc_workqueue("events_power_efficient", WQ_POWER_EFFICIENT, 0);
  system_freezable_power_efficient_wq = alloc_workqueue("events_freezable_power_efficient", WQ_FREEZABLE | WQ_POWER_EFFICIENT 0);

  return 0;
}

void __init start_kernel::rest_init::kernel_init_freeable(void)
{
  wait_for_completion(&kthreadd_done);
  workqueue_init();
}

int __init workqueue_init(void)
{
  struct workqueue_struct *wq;
  struct worker_pool *pool;
  int cpu, bkt;

  wq_numa_init();

  mutex_lock(&wq_pool_mutex);

  for_each_possible_cpu(cpu) {
    for_each_cpu_worker_pool(pool, cpu) {
      pool->node = cpu_to_node(cpu);
    }
  }

  list_for_each_entry(wq, &workqueues, list) {
    wq_update_unbound_numa(wq, smp_processor_id(), true);
    init_rescuer(wq);
  }

  mutex_unlock(&wq_pool_mutex);

  /* create the initial workers */
  for_each_online_cpu(cpu) {
    for_each_cpu_worker_pool(pool, cpu) {
      pool->flags &= ~POOL_DISASSOCIATED;
      create_worker(pool);
    }
  }

  hash_for_each(unbound_pool_hash, bkt, pool, hash_node)
    create_worker(pool);

  wq_online = true;
  wq_watchdog_init();

  return 0;
}

/* alloc_workqueue -> */
struct workqueue_struct *__alloc_workqueue_key(
  const char *fmt,
  unsigned int flags,
  int max_active,
  struct lock_class_key *key,
  const char *lock_name, ...)
{
  size_t tbl_size = 0;
  va_list args;
  struct workqueue_struct *wq;
  struct pool_workqueue *pwq;

  /* Unbound && max_active == 1 used to imply ordered, which is no
   * longer the case on NUMA machines due to per-node pools.  While
   * alloc_ordered_workqueue() is the right way to create an ordered
   * workqueue, keep the previous behavior to avoid subtle breakages
   * on NUMA. */
  if ((flags & WQ_UNBOUND) && max_active == 1)
    flags |= __WQ_ORDERED;

  /* see the comment above the definition of WQ_POWER_EFFICIENT */
  if ((flags & WQ_POWER_EFFICIENT) && wq_power_efficient)
    flags |= WQ_UNBOUND;

  /* allocate wq and format name */
  if (flags & WQ_UNBOUND)
    tbl_size = nr_node_ids * sizeof(wq->numa_pwq_tbl[0]);

  wq = kzalloc(sizeof(*wq) + tbl_size, GFP_KERNEL);
  if (!wq)
    return NULL;

  if (flags & WQ_UNBOUND) {
    wq->unbound_attrs = alloc_workqueue_attrs(GFP_KERNEL);
    if (!wq->unbound_attrs)
      goto err_free_wq;
  }

  va_start(args, lock_name);
  vsnprintf(wq->name, sizeof(wq->name), fmt, args);
  va_end(args);

  max_active = max_active ?: WQ_DFL_ACTIVE;
  max_active = wq_clamp_max_active(max_active, flags, wq->name);

  /* init wq */
  wq->flags = flags;
  wq->saved_max_active = max_active;
  mutex_init(&wq->mutex);
  atomic_set(&wq->nr_pwqs_to_flush, 0);
  INIT_LIST_HEAD(&wq->pwqs);
  INIT_LIST_HEAD(&wq->flusher_queue);
  INIT_LIST_HEAD(&wq->flusher_overflow);
  INIT_LIST_HEAD(&wq->maydays);

  lockdep_init_map(&wq->lockdep_map, lock_name, key, 0);
  INIT_LIST_HEAD(&wq->list);

  if (alloc_and_link_pwqs(wq) < 0)
    goto err_free_wq;

  if (wq_online && init_rescuer(wq) < 0)
    goto err_destroy;

  if ((wq->flags & WQ_SYSFS) && workqueue_sysfs_register(wq))
    goto err_destroy;

  /* wq_pool_mutex protects global freeze state and workqueues list.
   * Grab it, adjust max_active and add the new @wq to workqueues
   * list. */
  mutex_lock(&wq_pool_mutex);

  mutex_lock(&wq->mutex);
  for_each_pwq(pwq, wq)
    pwq_adjust_max_active(pwq);
  mutex_unlock(&wq->mutex);

  list_add_tail_rcu(&wq->list, &workqueues);

  mutex_unlock(&wq_pool_mutex);

  return wq;

err_free_wq:
  free_workqueue_attrs(wq->unbound_attrs);
  kfree(wq);
  return NULL;
err_destroy:
  destroy_workqueue(wq);
  return NULL;
}

int alloc_and_link_pwqs(struct workqueue_struct *wq)
{
  bool highpri = wq->flags & WQ_HIGHPRI;
  int cpu, ret;

  if (!(wq->flags & WQ_UNBOUND)) {
    wq->cpu_pwqs = alloc_percpu(struct pool_workqueue);
    if (!wq->cpu_pwqs)
      return -ENOMEM;

    for_each_possible_cpu(cpu) {
      struct pool_workqueue *pwq = per_cpu_ptr(wq->cpu_pwqs, cpu);
      struct worker_pool *cpu_pools = per_cpu(cpu_worker_pools, cpu);

      init_pwq(pwq, wq, &cpu_pools[highpri]);

      mutex_lock(&wq->mutex);
      link_pwq(pwq);
      mutex_unlock(&wq->mutex);
    }
    return 0;
  } else if (wq->flags & __WQ_ORDERED) {
    ret = apply_workqueue_attrs(wq, ordered_wq_attrs[highpri]);
    /* there should only be single pwq for ordering guarantee */
    return ret;
  } else {
    return apply_workqueue_attrs(wq, unbound_std_wq_attrs[highpri]);
  }
}

int init_worker_pool(struct worker_pool *pool)
{
  spin_lock_init(&pool->lock);
  pool->id = -1;
  pool->cpu = -1;
  pool->node = NUMA_NO_NODE;
  pool->flags |= POOL_DISASSOCIATED;
  pool->watchdog_ts = jiffies;
  INIT_LIST_HEAD(&pool->worklist);
  INIT_LIST_HEAD(&pool->idle_list);
  hash_init(pool->busy_hash);

  timer_setup(&pool->idle_timer, idle_worker_timeout, TIMER_DEFERRABLE);

  timer_setup(&pool->mayday_timer, pool_mayday_timeout, 0);

  INIT_LIST_HEAD(&pool->workers);

  ida_init(&pool->worker_ida);
  INIT_HLIST_NODE(&pool->hash_node);
  pool->refcnt = 1;

  /* shouldn't fail above this point */
  pool->attrs = alloc_workqueue_attrs(GFP_KERNEL);
  if (!pool->attrs)
    return -ENOMEM;
  return 0;
}

void init_pwq(struct pool_workqueue *pwq, struct workqueue_struct *wq,
         struct worker_pool *pool)
{
  BUG_ON((unsigned long)pwq & WORK_STRUCT_FLAG_MASK);

  memset(pwq, 0, sizeof(*pwq));

  pwq->pool = pool;
  pwq->wq = wq;
  pwq->flush_color = -1;
  pwq->refcnt = 1;
  INIT_LIST_HEAD(&pwq->inactive_works);
  INIT_LIST_HEAD(&pwq->pwqs_node);
  INIT_LIST_HEAD(&pwq->mayday_node);
  INIT_WORK(&pwq->unbound_release_work, pwq_unbound_release_workfn);
}
```

### alloc_unbound_pwq
```c++
static DEFINE_HASHTABLE(unbound_pool_hash, UNBOUND_POOL_HASH_ORDER);

struct pool_workqueue *alloc_unbound_pwq(struct workqueue_struct *wq,
          const struct workqueue_attrs *attrs)
{
  struct worker_pool *pool;
  struct pool_workqueue *pwq;

  lockdep_assert_held(&wq_pool_mutex);

  pool = get_unbound_pool(attrs);
  if (!pool)
    return NULL;

  pwq = kmem_cache_alloc_node(pwq_cache, GFP_KERNEL, pool->node);
  if (!pwq) {
    put_unbound_pool(pool);
    return NULL;
  }

  init_pwq(pwq, wq, pool);

  return pwq;
}

/* kernel/workqueue.c */
static cpumask_var_t *wq_numa_possible_cpumask; /* possible CPUs of each node */

static struct worker_pool *get_unbound_pool(const struct workqueue_attrs *attrs)
{
  u32 hash = wqattrs_hash(attrs);
  struct worker_pool *pool;
  int node;
  int target_node = NUMA_NO_NODE;

  lockdep_assert_held(&wq_pool_mutex);

  /* do we already have a matching pool? */
  hash_for_each_possible(unbound_pool_hash, pool, hash_node, hash) {
    if (wqattrs_equal(pool->attrs, attrs)) {
      pool->refcnt++;
      return pool;
    }
  }

  /* if cpumask is contained inside a NUMA node, we belong to that node */
  if (wq_numa_enabled) {
    for_each_node(node) {
      if (cpumask_subset(attrs->cpumask, wq_numa_possible_cpumask[node])) {
        target_node = node;
        break;
      }
    }
  }

  /* nope, create a new one */
  pool = kzalloc_node(sizeof(*pool), GFP_KERNEL, target_node);
  if (!pool || init_worker_pool(pool) < 0)
    goto fail;

  lockdep_set_subclass(&pool->lock, 1);  /* see put_pwq() */
  copy_workqueue_attrs(pool->attrs, attrs);
  pool->node = target_node;

   /* no_numa isn't a worker_pool attribute, always clear it.  See
   * 'struct workqueue_attrs' comments for detail. */
  pool->attrs->no_numa = false;

  if (worker_pool_assign_id(pool) < 0)
    goto fail;

  /* create and start the initial worker */
  if (wq_online && !create_worker(pool))
    goto fail;

  /* install */
  hash_add(unbound_pool_hash, &pool->hash_node, hash);

  return pool;
fail:
  if (pool)
    put_unbound_pool(pool);
  return NULL;
}

struct apply_wqattrs_ctx {
  struct workqueue_struct *wq;    /* target workqueue */
  struct workqueue_attrs  *attrs;    /* attrs to apply */
  struct list_head        list;    /* queued for batching commit */
  struct pool_workqueue   *dfl_pwq;
  struct pool_workqueue   *pwq_tbl[];
};

int apply_workqueue_attrs_locked(struct workqueue_struct *wq,
          const struct workqueue_attrs *attrs)
{
  struct apply_wqattrs_ctx *ctx;

  /* only unbound workqueues can change attributes */
  if (WARN_ON(!(wq->flags & WQ_UNBOUND)))
    return -EINVAL;

  /* creating multiple pwqs breaks ordering guarantee */
  if (!list_empty(&wq->pwqs)) {
    if (WARN_ON(wq->flags & __WQ_ORDERED_EXPLICIT))
      return -EINVAL;

    wq->flags &= ~__WQ_ORDERED;
  }

  ctx = apply_wqattrs_prepare(wq, attrs);
  if (!ctx)
    return -ENOMEM;

  /* the ctx has been prepared successfully, let's commit it */
  apply_wqattrs_commit(ctx);
  apply_wqattrs_cleanup(ctx);

  return 0;
}

struct apply_wqattrs_ctx *
apply_wqattrs_prepare(struct workqueue_struct *wq,
          const struct workqueue_attrs *attrs)
{
  struct apply_wqattrs_ctx *ctx;
  struct workqueue_attrs *new_attrs, *tmp_attrs;
  int node;

  lockdep_assert_held(&wq_pool_mutex);

  ctx = kzalloc(struct_size(ctx, pwq_tbl, nr_node_ids), GFP_KERNEL);

  new_attrs = alloc_workqueue_attrs(GFP_KERNEL);
  tmp_attrs = alloc_workqueue_attrs(GFP_KERNEL);
  if (!ctx || !new_attrs || !tmp_attrs)
    goto out_free;

  /* Calculate the attrs of the default pwq.
   * If the user configured cpumask doesn't overlap with the
   * wq_unbound_cpumask, we fallback to the wq_unbound_cpumask. */
  copy_workqueue_attrs(new_attrs, attrs);
  cpumask_and(new_attrs->cpumask, new_attrs->cpumask, wq_unbound_cpumask);
  if (unlikely(cpumask_empty(new_attrs->cpumask)))
    cpumask_copy(new_attrs->cpumask, wq_unbound_cpumask);

  /* We may create multiple pwqs with differing cpumasks.  Make a
   * copy of @new_attrs which will be modified and used to obtain
   * pools. */
  copy_workqueue_attrs(tmp_attrs, new_attrs);

  /* If something goes wrong during CPU up/down, we'll fall back to
   * the default pwq covering whole @attrs->cpumask.  Always create
   * it even if we don't use it immediately. */
  ctx->dfl_pwq = alloc_unbound_pwq(wq, new_attrs);
  if (!ctx->dfl_pwq)
    goto out_free;

  for_each_node(node) {
    if (wq_calc_node_cpumask(new_attrs, node, -1, tmp_attrs->cpumask)) {
      ctx->pwq_tbl[node] = alloc_unbound_pwq(wq, tmp_attrs);
      if (!ctx->pwq_tbl[node])
        goto out_free;
    } else {
      ctx->dfl_pwq->refcnt++;
      ctx->pwq_tbl[node] = ctx->dfl_pwq;
    }
  }

  /* save the user configured attrs and sanitize it. */
  copy_workqueue_attrs(new_attrs, attrs);
  cpumask_and(new_attrs->cpumask, new_attrs->cpumask, cpu_possible_mask);
  ctx->attrs = new_attrs;

  ctx->wq = wq;
  free_workqueue_attrs(tmp_attrs);
  return ctx;

out_free:
  free_workqueue_attrs(tmp_attrs);
  free_workqueue_attrs(new_attrs);
  apply_wqattrs_cleanup(ctx);
  return NULL;
}

static void apply_wqattrs_commit(struct apply_wqattrs_ctx *ctx)
{
  int node;

  /* all pwqs have been created successfully, let's install'em */
  mutex_lock(&ctx->wq->mutex);

  copy_workqueue_attrs(ctx->wq->unbound_attrs, ctx->attrs);

  /* save the previous pwq and install the new one */
  for_each_node(node)
    ctx->pwq_tbl[node] = numa_pwq_tbl_install(ctx->wq, node,
                ctx->pwq_tbl[node]);

  /* @dfl_pwq might not have been used, ensure it's linked */
  link_pwq(ctx->dfl_pwq);
  swap(ctx->wq->dfl_pwq, ctx->dfl_pwq);

  mutex_unlock(&ctx->wq->mutex);
}

/* free the resources after success or abort */
static void apply_wqattrs_cleanup(struct apply_wqattrs_ctx *ctx)
{
  if (ctx) {
    int node;

    for_each_node(node)
      put_pwq_unlocked(ctx->pwq_tbl[node]);
    put_pwq_unlocked(ctx->dfl_pwq);

    free_workqueue_attrs(ctx->attrs);

    kfree(ctx);
  }
}
```

### create_worker
```c++
static bool manage_workers(struct worker *worker)
{
  struct worker_pool *pool = worker->pool;

  if (pool->flags & POOL_MANAGER_ACTIVE)
    return false;

  pool->flags |= POOL_MANAGER_ACTIVE;
  pool->manager = worker;

  maybe_create_worker(pool);

  pool->manager = NULL;
  pool->flags &= ~POOL_MANAGER_ACTIVE;
  wake_up(&wq_manager_wait);
  return true;
}

void maybe_create_worker(struct worker_pool *pool)
__releases(&pool->lock)
__acquires(&pool->lock)
{
restart:
  spin_unlock_irq(&pool->lock);

  /* if we don't make progress in MAYDAY_INITIAL_TIMEOUT, call for help */
  mod_timer(&pool->mayday_timer, jiffies + MAYDAY_INITIAL_TIMEOUT);

  while (true) {
    if (create_worker(pool) || !need_to_create_worker(pool))
      break;

    schedule_timeout_interruptible(CREATE_COOLDOWN);

    if (!need_to_create_worker(pool))
      break;
  }

  del_timer_sync(&pool->mayday_timer);
  spin_lock_irq(&pool->lock);
  /* This is necessary even after a new worker was just successfully
   * created as @pool->lock was dropped and the new worker might have
   * already become busy. */
  if (need_to_create_worker(pool))
    goto restart;
}
```

```c++
struct worker *create_worker(struct worker_pool *pool)
{
  struct worker *worker = NULL;
  int id = -1;
  char id_buf[16];

  /* ID is needed to determine kthread name */
  id = ida_simple_get(&pool->worker_ida, 0, 0, GFP_KERNEL);
  if (id < 0)
    goto fail;

  worker = alloc_worker(pool->node);
  if (!worker)
    goto fail;

  worker->id = id;

  if (pool->cpu >= 0)
    snprintf(id_buf, sizeof(id_buf), "%d:%d%s", pool->cpu, id,
       pool->attrs->nice < 0  ? "H" : "");
  else
    snprintf(id_buf, sizeof(id_buf), "u%d:%d", pool->id, id);

  worker->task = kthread_create_on_node(worker_thread, worker, pool->node, "kworker/%s", id_buf);
  if (IS_ERR(worker->task))
    goto fail;

  set_user_nice(worker->task, pool->attrs->nice);
  kthread_bind_mask(worker->task, pool->attrs->cpumask);

  /* successful, attach the worker to the pool */
  worker_attach_to_pool(worker, pool);

  /* start the newly created worker */
  spin_lock_irq(&pool->lock);
  worker->pool->nr_workers++;
  worker_enter_idle(worker);
  wake_up_process(worker->task);
  spin_unlock_irq(&pool->lock);

  return worker;

fail:
  if (id >= 0)
    ida_simple_remove(&pool->worker_ida, id);
  kfree(worker);
  return NULL;
}

struct worker *alloc_worker(int node)
{
  struct worker *worker;

  worker = kzalloc_node(sizeof(*worker), GFP_KERNEL, node);
  if (worker) {
    INIT_LIST_HEAD(&worker->entry);
    INIT_LIST_HEAD(&worker->scheduled);
    INIT_LIST_HEAD(&worker->node);
    /* on creation a worker is in !idle && prep state */
    worker->flags = WORKER_PREP;
  }
  return worker;
}

struct task_struct *kthread_create_on_node(int (*threadfn)(void *data),
             void *data, int node,
             const char namefmt[],
             ...)
{
  struct task_struct *task;
  va_list args;

  va_start(args, namefmt);
  task = __kthread_create_on_node(threadfn, data, node, namefmt, args);
  va_end(args);

  return task;
}

struct task_struct *__kthread_create_on_node(int (*threadfn)(void *data),
                void *data, int node,
                const char namefmt[],
                va_list args)
{
  DECLARE_COMPLETION_ONSTACK(done);
  struct task_struct *task;
  struct kthread_create_info *create = kmalloc(sizeof(*create), GFP_KERNEL);

  if (!create)
    return ERR_PTR(-ENOMEM);
  create->threadfn = threadfn;
  create->data = data;
  create->node = node;
  create->done = &done;

  spin_lock(&kthread_create_lock);
  list_add_tail(&create->list, &kthread_create_list);
  spin_unlock(&kthread_create_lock);

  wake_up_process(kthreadd_task); /* kthreadd */
  /* Wait for completion in killable state, for I might be chosen by
   * the OOM killer while kthreadd is trying to allocate memory for
   * new kernel thread. */
  if (unlikely(wait_for_completion_killable(&done))) {
    /* If I was SIGKILLed before kthreadd (or new kernel thread)
     * calls complete(), leave the cleanup of this structure to
     * that thread. */
    if (xchg(&create->done, NULL))
      return ERR_PTR(-EINTR);
    /* kthreadd (or new kernel thread) will call complete()
     * shortly. */
    wait_for_completion(&done);
  }
  task = create->result;
  if (!IS_ERR(task)) {
    static const struct sched_param param = { .sched_priority = 0 };
    char name[TASK_COMM_LEN];

    /* task is already visible to other tasks, so updating
     * COMM must be protected. */
    vsnprintf(name, sizeof(name), namefmt, args);
    set_task_comm(task, name);
    /* root may have changed our (kthreadd's) priority or CPU mask.
     * The kernel thread should not inherit these properties. */
    sched_setscheduler_nocheck(task, SCHED_NORMAL, &param);
    set_cpus_allowed_ptr(task, cpu_all_mask);
  }
  kfree(create);
  return task;
}

void worker_enter_idle(struct worker *worker)
{
  struct worker_pool *pool = worker->pool;

  if (WARN_ON_ONCE(worker->flags & WORKER_IDLE) ||
      WARN_ON_ONCE(!list_empty(&worker->entry) &&
       (worker->hentry.next || worker->hentry.pprev)))
    return;

  /* can't use worker_set_flags(), also called from create_worker() */
  worker->flags |= WORKER_IDLE;
  pool->nr_idle++;
  worker->last_active = jiffies;

  /* idle_list is LIFO */
  list_add(&worker->entry, &pool->idle_list);

  if (too_many_workers(pool) && !timer_pending(&pool->idle_timer))
    mod_timer(&pool->idle_timer, jiffies + IDLE_WORKER_TIMEOUT);

  /* Sanity check nr_running.  Because unbind_workers() releases
   * pool->lock between setting %WORKER_UNBOUND and zapping
   * nr_running, the warning may trigger spuriously.  Check iff
   * unbind is not in progress. */
  WARN_ON_ONCE(!(pool->flags & POOL_DISASSOCIATED) &&
         pool->nr_workers == pool->nr_idle &&
         atomic_read(&pool->nr_running));
}
```

### worker_thread
```c++
int worker_thread(void *__worker)
{
  struct worker *worker = __worker;
  struct worker_pool *pool = worker->pool;

  /* tell the scheduler that this is a workqueue worker */
  set_pf_worker(true);
woke_up:
  spin_lock_irq(&pool->lock);

  /* am I supposed to die? */
  if (unlikely(worker->flags & WORKER_DIE)) {
    spin_unlock_irq(&pool->lock);
    set_pf_worker(false);

    set_task_comm(worker->task, "kworker/dying");
    ida_simple_remove(&pool->worker_ida, worker->id);
    worker_detach_from_pool(worker);
    kfree(worker);
    return 0;
  }

  worker_leave_idle(worker);
recheck:
  /* no more worker necessary? */
  if (!need_more_worker(pool)) /* !list_empty(&pool->worklist) && !pool->nr_running */
    goto sleep;

  /* do we need to manage? */
  /* return pool->nr_idle; */
  if (unlikely(!may_start_working(pool)) && manage_workers(worker))
    goto recheck;

  /* ->scheduled list can only be filled while a worker is
   * preparing to process a work or actually processing it.
   * Make sure nobody diddled with it while I was sleeping. */

  /* Finish PREP stage.  We're guaranteed to have at least one idle
   * worker or that someone else has already assumed the manager
   * role.  This is where @worker starts participating in concurrency
   * management if applicable and concurrency management is restored
   * after being rebound.  See rebind_workers() for details. */
  worker_clr_flags(worker, WORKER_PREP | WORKER_REBOUND);

  do {
    struct work_struct *work = list_first_entry(&pool->worklist, struct work_struct, entry);

    pool->watchdog_ts = jiffies;

    if (likely(!(*work_data_bits(work) & WORK_STRUCT_LINKED))) {
      /* optimization path, not strictly necessary */
      process_one_work(worker, work);
      if (unlikely(!list_empty(&worker->scheduled)))
        process_scheduled_works(worker);
    } else {
      move_linked_works(work, &worker->scheduled, NULL);
      process_scheduled_works(worker);
    }
  } while (keep_working(pool)); /* !list_empty(&pool->worklist) && atomic_read(&pool->nr_running) <= 1; */

  worker_set_flags(worker, WORKER_PREP);
sleep:
  /* pool->lock is held and there's no work to process and no need to
   * manage, sleep.  Workers are woken up only while holding
   * pool->lock or from local cpu, so setting the current state
   * before releasing pool->lock is enough to prevent losing any
   * event. */
  worker_enter_idle(worker);
  __set_current_state(TASK_IDLE);
  spin_unlock_irq(&pool->lock);
  schedule();
  goto woke_up;
}

void process_scheduled_works(struct worker *worker)
{
  while (!list_empty(&worker->scheduled)) {
    struct work_struct *work = list_first_entry(&worker->scheduled, struct work_struct, entry);
    process_one_work(worker, work);
  }
}

void process_one_work(struct worker *worker, struct work_struct *work)
__releases(&pool->lock)
__acquires(&pool->lock)
{
  struct pool_workqueue *pwq = get_work_pwq(work);
  struct worker_pool *pool = worker->pool;
  bool cpu_intensive = pwq->wq->flags & WQ_CPU_INTENSIVE;
  int work_color;
  struct worker *collision;

  /* A single work shouldn't be executed concurrently by
   * multiple workers on a single cpu.  Check whether anyone is
   * already processing the work.  If so, defer the work to the
   * currently executing one. */
  collision = find_worker_executing_work(pool, work);
  if (unlikely(collision)) {
    move_linked_works(work, &collision->scheduled, NULL);
    return;
  }

  /* claim and dequeue */
  debug_work_deactivate(work);
  hash_add(pool->busy_hash, &worker->hentry, (unsigned long)work);
  worker->current_work = work;
  worker->current_func = work->func;
  worker->current_pwq = pwq;
  work_color = get_work_color(work);

  /* Record wq name for cmdline and debug reporting, may get
   * overridden through set_worker_desc(). */
  strscpy(worker->desc, pwq->wq->name, WORKER_DESC_LEN);

  list_del_init(&work->entry);

  /* CPU intensive works don't participate in concurrency management.
   * They're the scheduler's responsibility.  This takes @worker out
   * of concurrency management and the next code block will chain
   * execution of the pending work items. */
  if (unlikely(cpu_intensive))
    worker_set_flags(worker, WORKER_CPU_INTENSIVE);

  /* Wake up another worker if necessary.  The condition is always
   * false for normal per-cpu workers since nr_running would always
   * be >= 1 at this point.  This is used to chain execution of the
   * pending work items for WORKER_NOT_RUNNING workers such as the
   * UNBOUND and CPU_INTENSIVE ones. */
  if (need_more_worker(pool))
    wake_up_worker(pool);

  /* Record the last pool and clear PENDING which should be the last
   * update to @work.  Also, do this inside @pool->lock so that
   * PENDING and queued state changes happen together while IRQ is
   * disabled. */
  set_work_pool_and_clear_pending(work, pool->id);

  spin_unlock_irq(&pool->lock);

  lock_map_acquire(&pwq->wq->lockdep_map);
  lock_map_acquire(&lockdep_map);
  /* Strictly speaking we should mark the invariant state without holding
   * any locks, that is, before these two lock_map_acquire()'s.
   *
   * However, that would result in:
   *
   *   A(W1)
   *   WFC(C)
   *    A(W1)
   *    C(C)
   *
   * Which would create W1->C->W1 dependencies, even though there is no
   * actual deadlock possible. There are two solutions, using a
   * read-recursive acquire on the work(queue) 'locks', but this will then
   * hit the lockdep limitation on recursive locks, or simply discard
   * these locks.
   *
   * AFAICT there is no possible deadlock scenario between the
   * flush_work() and complete() primitives (except for single-threaded
   * workqueues), so hiding them isn't a problem. */
  lockdep_invariant_state(true);
  trace_workqueue_execute_start(work);
  worker->current_func(work);
  /* While we must be careful to not use "work" after this, the trace
   * point will only record its address. */
  trace_workqueue_execute_end(work);
  lock_map_release(&lockdep_map);
  lock_map_release(&pwq->wq->lockdep_map);

  if (unlikely(in_atomic() || lockdep_depth(current) > 0)) {
    debug_show_held_locks(current);
    dump_stack();
  }

  /* The following prevents a kworker from hogging CPU on !PREEMPT
   * kernels, where a requeueing work item waiting for something to
   * happen could deadlock with stop_machine as such work item could
   * indefinitely requeue itself while all other CPUs are trapped in
   * stop_machine. At the same time, report a quiescent RCU state so
   * the same condition doesn't freeze RCU. */
  cond_resched();

  spin_lock_irq(&pool->lock);

  /* clear cpu intensive status */
  if (unlikely(cpu_intensive))
    worker_clr_flags(worker, WORKER_CPU_INTENSIVE);

  /* we're done with it, release */
  hash_del(&worker->hentry);
  worker->current_work = NULL;
  worker->current_func = NULL;
  worker->current_pwq = NULL;
  pwq_dec_nr_in_flight(pwq, work_color);
}

void pwq_dec_nr_in_flight(struct pool_workqueue *pwq, int color)
{
  /* uncolored work items don't participate in flushing or nr_active */
  if (color == WORK_NO_COLOR)
    goto out_put;

  pwq->nr_in_flight[color]--;

  pwq->nr_active--;
  if (!list_empty(&pwq->inactive_works)) {
    /* one down, submit a inactive one */
    if (pwq->nr_active < pwq->max_active)
      pwq_activate_first_inactive(pwq);
  }

  /* is flush in progress and are we at the flushing tip? */
  if (likely(pwq->flush_color != color))
    goto out_put;

  /* are there still in-flight works? */
  if (pwq->nr_in_flight[color])
    goto out_put;

  /* this pwq is done, clear flush_color */
  pwq->flush_color = -1;

  /* If this was the last pwq, wake up the first flusher.  It
   * will handle the rest. */
  if (atomic_dec_and_test(&pwq->wq->nr_pwqs_to_flush))
    complete(&pwq->wq->first_flusher->done);
out_put:
  put_pwq(pwq);
}

void pwq_activate_first_inactive(struct pool_workqueue *pwq)
{
  struct work_struct *work = list_first_entry(&pwq->inactive_works, struct work_struct, entry);

  pwq_activate_inactive_work(work);
}

void pwq_activate_inactive_work(struct work_struct *work)
{
  struct pool_workqueue *pwq = get_work_pwq(work);

  if (list_empty(&pwq->pool->worklist))
    pwq->pool->watchdog_ts = jiffies;
  move_linked_works(work, &pwq->pool->worklist, NULL);
  __clear_bit(WORK_STRUCT_INACTIVE_BIT, work_data_bits(work));
  pwq->nr_active++;
}

void worker_set_flags(struct worker *worker, unsigned int flags)
{
  struct worker_pool *pool = worker->pool;

  WARN_ON_ONCE(worker->task != current);

  /* If transitioning into NOT_RUNNING, adjust nr_running. */
  if ((flags & WORKER_NOT_RUNNING) && !(worker->flags & WORKER_NOT_RUNNING)) {
    atomic_dec(&pool->nr_running);
  }

  worker->flags |= flags;
}

void worker_clr_flags(struct worker *worker, unsigned int flags)
{
  struct worker_pool *pool = worker->pool;
  unsigned int oflags = worker->flags;

  WARN_ON_ONCE(worker->task != current);

  worker->flags &= ~flags;

  /* If transitioning out of NOT_RUNNING, increment nr_running.  Note
   * that the nested NOT_RUNNING is not a noop.  NOT_RUNNING is mask
   * of multiple flags, not a single flag. */
  if ((flags & WORKER_NOT_RUNNING) && (oflags & WORKER_NOT_RUNNING))
    if (!(worker->flags & WORKER_NOT_RUNNING))
      atomic_inc(&pool->nr_running);
}
```

```c++
void __sched notrace __schedule(bool preempt) {
  if (prev->flags & PF_WQ_WORKER) {
    struct task_struct *to_wakeup;

    to_wakeup = wq_worker_sleeping(prev);
    if (to_wakeup)
        try_to_wake_up_local(to_wakeup, &rf);
  }
}

struct task_struct *wq_worker_sleeping(struct task_struct *task)
{
  struct worker *worker = kthread_data(task), *to_wakeup = NULL;
  struct worker_pool *pool;

  /* Rescuers, which may not have all the fields set up like normal
   * workers, also reach here, let's not access anything before
   * checking NOT_RUNNING. */
  if (worker->flags & WORKER_NOT_RUNNING)
    return NULL;

  pool = worker->pool;

  /* this can only happen on the local cpu */
  if (WARN_ON_ONCE(pool->cpu != raw_smp_processor_id()))
    return NULL;

  /* The counterpart of the following dec_and_test, implied mb,
   * worklist not empty test sequence is in insert_work().
   * Please read comment there.
   *
   * NOT_RUNNING is clear.  This means that we're bound to and
   * running on the local cpu w/ rq lock held and preemption
   * disabled, which in turn means that none else could be
   * manipulating idle_list, so dereferencing idle_list without pool
   * lock is safe. */
  if (atomic_dec_and_test(&pool->nr_running) && !list_empty(&pool->worklist))
    to_wakeup = first_idle_worker(pool);

  return to_wakeup ? to_wakeup->task : NULL;
}
```

### schedule_work
```c++
bool schedule_work(struct work_struct *work)
{
  return queue_work(system_wq, work);
}

bool queue_work(struct workqueue_struct *wq,
            struct work_struct *work)
{
  return queue_work_on(WORK_CPU_UNBOUND, wq, work);
}

bool queue_work_on(int cpu, struct workqueue_struct *wq,
       struct work_struct *work)
{
  bool ret = false;
  unsigned long flags;

  local_irq_save(flags);

  if (!test_and_set_bit(WORK_STRUCT_PENDING_BIT, work_data_bits(work))) {
    __queue_work(cpu, wq, work);
    ret = true;
  }

  local_irq_restore(flags);
  return ret;
}

void __queue_work(int cpu, struct workqueue_struct *wq,
       struct work_struct *work)
{
  struct pool_workqueue *pwq;
  struct worker_pool *last_pool;
  struct list_head *worklist;
  unsigned int work_flags;
  unsigned int req_cpu = cpu;

  /* While a work item is PENDING && off queue, a task trying to
   * steal the PENDING will busy-loop waiting for it to either get
   * queued or lose PENDING.  Grabbing PENDING and queueing should
   * happen with IRQ disabled. */
  lockdep_assert_irqs_disabled();

  /* if draining, only works from the same workqueue are allowed */
  if (unlikely(wq->flags & __WQ_DRAINING) &&
      WARN_ON_ONCE(!is_chained_work(wq)))
    return;
retry:
  /* pwq which will be used unless @work is executing elsewhere */
  if (wq->flags & WQ_UNBOUND) {
    if (req_cpu == WORK_CPU_UNBOUND)
      cpu = wq_select_unbound_cpu(raw_smp_processor_id());
    pwq = unbound_pwq_by_node(wq, cpu_to_node(cpu)); /* wq->numa_pwq_tbl[node] */
  } else {
    if (req_cpu == WORK_CPU_UNBOUND)
      cpu = raw_smp_processor_id();
    pwq = per_cpu_ptr(wq->cpu_pwqs, cpu);
  }

  /* If @work was previously on a different pool, it might still be
   * running there, in which case the work needs to be queued on that
   * pool to guarantee non-reentrancy. */
  last_pool = get_work_pool(work);
  if (last_pool && last_pool != pwq->pool) {
    struct worker *worker;

    spin_lock(&last_pool->lock);

    worker = find_worker_executing_work(last_pool, work);

    if (worker && worker->current_pwq->wq == wq) {
      pwq = worker->current_pwq;
    } else {
      /* meh... not running there, queue here */
      spin_unlock(&last_pool->lock);
      spin_lock(&pwq->pool->lock);
    }
  } else {
    spin_lock(&pwq->pool->lock);
  }

  /* pwq is determined and locked.  For unbound pools, we could have
   * raced with pwq release and it could already be dead.  If its
   * refcnt is zero, repeat pwq selection.  Note that pwqs never die
   * without another pwq replacing it in the numa_pwq_tbl or while
   * work items are executing on it, so the retrying is guaranteed to
   * make forward-progress. */
  if (unlikely(!pwq->refcnt)) {
    if (wq->flags & WQ_UNBOUND) {
      spin_unlock(&pwq->pool->lock);
      cpu_relax();
      goto retry;
    }
    /* oops */
    WARN_ONCE(true, "workqueue: per-cpu pwq for %s on cpu%d has 0 refcnt",
        wq->name, cpu);
  }

  /* pwq determined, queue */
  trace_workqueue_queue_work(req_cpu, pwq, work);

  if (WARN_ON(!list_empty(&work->entry))) {
    spin_unlock(&pwq->pool->lock);
    return;
  }

  pwq->nr_in_flight[pwq->work_color]++;
  work_flags = work_color_to_flags(pwq->work_color);

  if (likely(pwq->nr_active < pwq->max_active)) {
    trace_workqueue_activate_work(work);
    pwq->nr_active++;
    worklist = &pwq->pool->worklist;
    if (list_empty(worklist))
      pwq->pool->watchdog_ts = jiffies;
  } else {
    work_flags |= WORK_STRUCT_DELAYED;
    worklist = &pwq->inactive_works;
  }

  debug_work_activate(work);
  insert_work(pwq, work, worklist, work_flags);

  spin_unlock(&pwq->pool->lock);
}

void insert_work(struct pool_workqueue *pwq, struct work_struct *work,
      struct list_head *head, unsigned int extra_flags)
{
  struct worker_pool *pool = pwq->pool;

  /* we own @work, set data and link */
  set_work_pwq(work, pwq, extra_flags);
  list_add_tail(&work->entry, head);
  get_pwq(pwq);

  /* Ensure either wq_worker_sleeping() sees the above
   * list_add_tail() or we see zero nr_running to avoid workers lying
   * around lazily while there are works to be processed. */
  smp_mb();

  if (__need_more_worker(pool))
    wake_up_worker(pool);
}

static void wake_up_worker(struct worker_pool *pool)
{
  struct worker *worker = first_idle_worker(pool);

  if (likely(worker))
    wake_up_process(worker->task);
}
```

### schedule_delayed_work
```c++
bool schedule_delayed_work(struct delayed_work *dwork, unsigned long delay)
{
  return queue_delayed_work(system_wq, dwork, delay);
}

bool queue_delayed_work(struct workqueue_struct *wq,
              struct delayed_work *dwork,
              unsigned long delay)
{
  return queue_delayed_work_on(WORK_CPU_UNBOUND, wq, dwork, delay);
}

bool queue_delayed_work_on(int cpu, struct workqueue_struct *wq,
         struct delayed_work *dwork, unsigned long delay)
{
  struct work_struct *work = &dwork->work;
  bool ret = false;
  unsigned long flags;

  /* read the comment in __queue_work() */
  local_irq_save(flags);

  if (!test_and_set_bit(WORK_STRUCT_PENDING_BIT, work_data_bits(work))) {
    __queue_delayed_work(cpu, wq, dwork, delay);
    ret = true;
  }

  local_irq_restore(flags);
  return ret;
}

void __queue_delayed_work(int cpu, struct workqueue_struct *wq,
        struct delayed_work *dwork, unsigned long delay)
{
  struct timer_list *timer = &dwork->timer;
  struct work_struct *work = &dwork->work;

  WARN_ON_ONCE(!wq);
  WARN_ON_ONCE(timer->function != delayed_work_timer_fn);
  WARN_ON_ONCE(timer_pending(timer));
  WARN_ON_ONCE(!list_empty(&work->entry));

  /* If @delay is 0, queue @dwork->work immediately.  This is for
   * both optimization and correctness.  The earliest @timer can
   * expire is on the closest next tick and delayed_work users depend
   * on that there's no such delay when @delay is 0. */
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

/* when delayed_work timer expired */
void delayed_work_timer_fn(struct timer_list *t)
{
  struct delayed_work *dwork = from_timer(dwork, t, timer);
  __queue_work(dwork->cpu, dwork->wq, &dwork->work);
}
```

### flush_work
```c++

```

### idle_timer
```c++
static void idle_worker_timeout(struct timer_list *t)
{
  struct worker_pool *pool = from_timer(pool, t, idle_timer);

  spin_lock_irq(&pool->lock);

  while (too_many_workers(pool)) {
    struct worker *worker;
    unsigned long expires;

    /* idle_list is kept in LIFO order, check the last one */
    worker = list_entry(pool->idle_list.prev, struct worker, entry);
    expires = worker->last_active + IDLE_WORKER_TIMEOUT;

    if (time_before(jiffies, expires)) {
      mod_timer(&pool->idle_timer, expires);
      break;
    }

    destroy_worker(worker);
  }

  spin_unlock_irq(&pool->lock);
}

bool too_many_workers(struct worker_pool *pool)
{
  bool managing = pool->flags & POOL_MANAGER_ACTIVE;
  int nr_idle = pool->nr_idle + managing; /* manager is considered idle */
  int nr_busy = pool->nr_workers - nr_idle;

  return nr_idle > 2 && (nr_idle - 2) * MAX_IDLE_WORKERS_RATIO >= nr_busy;
}

static void destroy_worker(struct worker *worker)
{
  struct worker_pool *pool = worker->pool;

  lockdep_assert_held(&pool->lock);

  /* sanity check frenzy */
  if (WARN_ON(worker->current_work) ||
      WARN_ON(!list_empty(&worker->scheduled)) ||
      WARN_ON(!(worker->flags & WORKER_IDLE)))
    return;

  pool->nr_workers--;
  pool->nr_idle--;

  list_del_init(&worker->entry);
  worker->flags |= WORKER_DIE;
  wake_up_process(worker->task);
}
```

### maday_timer
```c++
static void pool_mayday_timeout(struct timer_list *t)
{
  struct worker_pool *pool = from_timer(pool, t, mayday_timer);
  struct work_struct *work;

  spin_lock_irq(&pool->lock);
  spin_lock(&wq_mayday_lock);    /* for wq->maydays */

  if (need_to_create_worker(pool)) {
    /* We've been trying to create a new worker but
     * haven't been successful.  We might be hitting an
     * allocation deadlock.  Send distress signals to
     * rescuers. */
    list_for_each_entry(work, &pool->worklist, entry)
      send_mayday(work);
  }

  spin_unlock(&wq_mayday_lock);
  spin_unlock_irq(&pool->lock);

  mod_timer(&pool->mayday_timer, jiffies + MAYDAY_INTERVAL);
}

bool need_to_create_worker(struct worker_pool *pool)
{
  return need_more_worker(pool) && !may_start_working(pool);
}

static void send_mayday(struct work_struct *work)
{
  struct pool_workqueue *pwq = get_work_pwq(work);
  struct workqueue_struct *wq = pwq->wq;

  lockdep_assert_held(&wq_mayday_lock);

  if (!wq->rescuer)
    return;

  /* mayday mayday mayday */
  if (list_empty(&pwq->mayday_node)) {
    /* If @pwq is for an unbound wq, its base ref may be put at
     * any time due to an attribute change.  Pin @pwq until the
     * rescuer is done with it. */
    get_pwq(pwq);
    list_add_tail(&pwq->mayday_node, &wq->maydays);
    wake_up_process(wq->rescuer->task);
  }
}
```

### call-graph-cmwq
```c++
start_kernel();
    workqueue_init_early();
        pwq_cache = KMEM_CACHE(pool_workqueue, SLAB_PANIC);

        for_each_possible_cpu(cpu)
            for_each_cpu_worker_pool()
                init_worker_pool();
                    timer_setup(&pool->idle_timer, idle_worker_timeout, TIMER_DEFERRABLE);
                    timer_setup(&pool->mayday_timer, pool_mayday_timeout, 0);
                    alloc_workqueue_attrs();

        system_wq = alloc_workqueue("events");
        system_highpri_wq = alloc_workqueue("events_highpri");
        system_long_wq = alloc_workqueue("events_long");
        system_freezable_wq = alloc_workqueue("events_freezable");
        system_power_efficient_wq = alloc_workqueue("events_power_efficient");
        system_freezable_power_efficient_wq = alloc_workqueue("events_freezable_power_efficient");
        alloc_workqueue();
            --->

    rest_init();
        kernel_thread(kernel_init);
            kernel_init();
                kernel_init_freeable();
                    workqueue_init();
                        wq_numa_init();
                            wq_numa_possible_cpumask = tbl;
                        for_each_online_cpu(cpu);
                            for_each_cpu_worker_pool(pool, cpu);
                                create_worker(pool); /* each pool has at leat one worker */
                        hash_for_each(unbound_pool_hash, bkt, pool, hash_node)
                            create_worker(pool);
                        wq_watchdog_init();

alloc_workqueue();
    struct workqueue_struct* wq = kzalloc(sizeof(*wq) + tbl_size, GFP_KERNEL);
    alloc_and_link_pwqs();
        if (!(wq->flags & WQ_UNBOUND)) {
            wq->cpu_pwqs = alloc_percpu(struct pool_workqueue);
            for_each_possible_cpu(cpu) {
                init_pwq(pwq, wq);
                    pwq->pool = &cpu_pools[priority];
                    pwq->wq = wq;
                link_pwq();
                    pwq_adjust_max_active();
                    list_add_rcu(&pwq->pwqs_node, &wq->pwqs);
            }
        } else if (wq->flags & __WQ_ORDERED) {
            apply_workqueue_attrs(wq, ordered_wq_attrs[priority]);
        } else {
            apply_workqueue_attrs(wq, unbound_std_wq_attrs[priority]);
                apply_wqattrs_prepare(wq, attrs);
                    apply_wqattrs_ctx* ctx = kzalloc(struct_size(ctx, pwq_tbl, nr_node_ids), GFP_KERNEL);
                    ctx->dfl_pwq = alloc_unbound_pwq(wq, new_attrs);
                        --->
                    for_each_node(node) {
                        if (wq_calc_node_cpumask(new_attrs, node, -1, tmp_attrs->cpumask)) {
                            ctx->pwq_tbl[node] = alloc_unbound_pwq(wq, tmp_attrs);
                        } else {
                            ctx->dfl_pwq->refcnt++;
                            ctx->pwq_tbl[node] = ctx->dfl_pwq;
                        }
                    }
                apply_wqattrs_commit(ctx);
                    for_each_node(node)
                        ctx->pwq_tbl[node] = numa_pwq_tbl_install(ctx->wq, node, ctx->pwq_tbl[node] /*pwq*/);
                            link_pwq(pwq);
                            rcu_assign_pointer(wq->numa_pwq_tbl[node], pwq);
                        link_pwq(ctx->dfl_pwq);
                        swap(ctx->wq->dfl_pwq, ctx->dfl_pwq);
                apply_wqattrs_cleanup(ctx);
        }
    for_each_pwq(pwq, wq)
        pwq_adjust_max_active(pwq);
    init_rescuer();
    list_add_tail_rcu(&wq->list, &workqueues);

alloc_unbound_pwq();
    worker_pool* pool = get_unbound_pool();
        hash_for_each_possible(unbound_pool_hash, pool, hash_node, hash) {
            if (wqattrs_equal(pool->attrs, attrs)) {
                pool->refcnt++;
                return pool;
            }
        }

        for_each_node(node) {
            if (cpumask_subset(attrs->cpumask, wq_numa_possible_cpumask[node])) {
                target_node = node;
                break;
            }
        }
        pool = kzalloc_node(sizeof(*pool), GFP_KERNEL, target_node);
        init_worker_pool(pool);
            timer_setup(&pool->idle_timer, idle_worker_timeout, TIMER_DEFERRABLE);
            timer_setup(&pool->mayday_timer, pool_mayday_timeout, 0);
            alloc_workqueue_attrs();
        copy_workqueue_attrs(pool->attrs, attrs);
        create_worker(pool);
            --->
        hash_add(unbound_pool_hash, &pool->hash_node, hash);

    pwq = kmem_cache_alloc_node(pwq_cache);
    init_pwq(pwq, wq, pool);
        pwq->pool = pool;
        pwq->wq = wq;

create_worker(pool);
    woker = alloc_worker(pool->node);
        worker->flags = WORKER_PREP;
    worker->task = kthread_create_on_node(worker_thread);
        kthreadd();
            create_kthread();
                kernel_thread();
                    _do_fork();
        wake_up_process(kthreadd_task); /* kthreadd */
        wait_for_completion_killable(&done);
    worker_attach_to_pool();
    worker_enter_idle(worker);
        pool->nr_idle++;
    wake_up_process(worker->task);
        try_to_wake_up();

worker_thread();
woke_up:
    if (worker->flags & WORKER_DIE) {
        kfree(worker);
        return;
    }

    worker_leave_idle();
        pool->nr_idle--;
        worker_clr_flags(worker, WORKER_IDLE);
            worker->flags &= ~flags;
            if ((flags & WORKER_NOT_RUNNING) && (oflags & WORKER_NOT_RUNNING))
                if (!(worker->flags & WORKER_NOT_RUNNING))
                    atomic_inc(&pool->nr_running);

    /* keep only one running worker for non-cpu-intensive workqueue */
    if (!need_more_worker(pool)) /* !list_empty(&pool->worklist) && !pool->nr_running */
        goto sleep;

    if (!may_start_working()) /* pool->nr_idle */
        manage_workers();
            maybe_create_worker();
                mod_timer(&pool->mayday_timer, jiffies + MAYDAY_INITIAL_TIMEOUT);
                create_worker();
                    --->
            wake_up(&wq_manager_wait);

    /* nr_running is inced when entering and deced when leaving */
    worker_clr_flags(worker, WORKER_PREP | WORKER_REBOUND);         /* 1.1 [nr_running]++ == 1 */

    do {
        work = list_first_entry(&pool->worklist, struct work_struct, entry);
        if (likely(!(*work_data_bits(work) & WORK_STRUCT_LINKED))) {
            process_one_work(worker, work);
            if (unlikely(!list_empty(&worker->scheduled)))
                process_scheduled_works(worker);
        } else {
            move_linked_works(work, &worker->scheduled, NULL);
            process_scheduled_works(worker);
        }

        process_one_work();
            if (unlikely(cpu_intensive))
                worker_set_flags(worker, WORKER_CPU_INTENSIVE);     /* 2.1 [nr_running]-- == 0 */
                    worker->flags |= flags;
                    if ((flags & WORKER_NOT_RUNNING) && !(worker->flags & WORKER_NOT_RUNNING)) {
                        atomic_dec(&pool->nr_running);
                    }

            if (need_more_worker(pool)) /* !list_empty(&pool->worklist) && !pool->nr_running */
                wake_up_worker(pool);
                    try_to_wake_up();

            worker->current_func(work);

            if (unlikely(cpu_intensive))
                worker_clr_flags(worker, WORKER_CPU_INTENSIVE);     /* 2.1 [nr_running]++ == 1 */

            pwq_dec_nr_in_flight();
                pwq->nr_active--;
                if (!list_empty(&pwq->inactive_works))
                    if (pwq->nr_active < pwq->max_active)
                        pwq_activate_first_inactive(pwq);
                            move_linked_works(work, &pwq->pool->worklist, NULL);
                            pwq->nr_active++;
    } while (keep_working(pool)); /* !list_empty(&pool->worklist) && atomic_read(&pool->nr_running) <= 1; */

    worker_set_flags(worker, WORKER_PREP);                          /* 1.2 [nr_running]-- == 0 */
    worker_enter_idle(worker);
        pool->nr_idle++;

    schedule();
        if (prev->flags & PF_WQ_WORKER)
            to_wakeup = wq_worker_sleeping()
                if (pool->cpu != raw_smp_processor_id())
                    return NULL;
                /* only wakeup idle thread when running task is going to suspend in process_one_work */
                if (atomic_dec_and_test(&pool->nr_running) && !list_empty(&pool->worklist))
                    to_wakeup = first_idle_worker(pool);
            try_to_wake_up_local(to_wakeup, &rf);

    goto woke_up;

schedule_work();
    queue_work();
        queue_work_on(cpu, wq, work);
            if (wq->flags & WQ_UNBOUND) {
                if (req_cpu == WORK_CPU_UNBOUND)
                    cpu = wq_select_unbound_cpu(raw_smp_processor_id());
                pwq = unbound_pwq_by_node(wq, cpu_to_node(cpu));
            } else {
                if (req_cpu == WORK_CPU_UNBOUND)
                    cpu = raw_smp_processor_id();
                pwq = per_cpu_ptr(wq->cpu_pwqs, cpu);
            }

            if (likely(pwq->nr_active < pwq->max_active)) {
                pwq->nr_active++;
                worklist = &pwq->pool->worklist;
            } else {
                work_flags |= WORK_STRUCT_DELAYED;
                worklist = &pwq->inactive_works;
            }

            insert_work(pwq, work, worklist, work_flags);
                list_add_tail(&work->entry, head);
                if (__need_more_worker(pool)) /* return !pool->nr_running */
                    wake_up_worker(pool);
                        try_to_wake_up();

idle_worker_timeout();
    while (too_many_workers(pool)) {
        mod_timer(&pool->idle_timer, expires);
        destroy_worker(worker);
            pool->nr_workers--;
            pool->nr_idle--;
            list_del_init(&worker->entry);
            worker->flags |= WORKER_DIE;
            wake_up_process(worker->task);
    }

pool_mayday_timeout();
    if (need_to_create_worker(pool)) /* need_more_worker(pool) && !may_start_working(pool) */
        send_mayday(work);
            list_add_tail(&pwq->mayday_node, &wq->maydays);
            wake_up_process(wq->rescuer->task);
    mod_timer(&pool->mayday_timer, jiffies + MAYDAY_INTERVAL);
```

Reference:
* [A complete guide to Linux process scheduling.pdf](https://trepo.tuni.fi/bitstream/handle/10024/96864/GRADU-1428493916.pdf)


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

* PPI (Private Peripheral Interrupt)
* SPI (Shared Peripheral Interrupt)

# init
```c
start_kernel(void)
    tick_init() {
        tick_broadcast_init();
        tick_nohz_init();
    }

    init_timers() {
        init_timer_cpus();
        posix_cputimers_init_work() {
            clear_posix_cputimers_work(current) {
                memset(&p->posix_cputimers_work.work, 0, sizeof(p->posix_cputimers_work.work));
                /* the timer started by update_process_times */
                init_task_work(&p->posix_cputimers_work.work, posix_cpu_timers_work);
                mutex_init(&p->posix_cputimers_work.mutex);
                p->posix_cputimers_work.scheduled = false;
            }
        }
        open_softirq(TIMER_SOFTIRQ, run_timer_softirq);
    }

    hrtimers_init() {
        hrtimers_prepare_cpu(smp_processor_id());
        open_softirq(HRTIMER_SOFTIRQ, hrtimer_run_softirq);
    }

    timekeeping_init() {
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

    /* arch/arm64/kernel/time.c */
    time_init(void) {
        u32 arch_timer_rate;

        of_clk_init(NULL);
        timer_probe();

        tick_setup_hrtimer_broadcast() {
            hrtimer_init(&bctimer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS_HARD);
            bctimer.function = bc_handler;

            clockevents_register_device(&ce_broadcast_hrtimer) {
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
                tick_check_new_device(dev) {
                    tick_setup_device(td, newdev, cpu, cpumask_of(cpu)) {
                        if (td->mode == TICKDEV_MODE_PERIODIC) {
                            tick_setup_periodic(newdev, 0) {
                                tick_set_periodic_handler(dev, broadcast) {
                                    if (!broadcast)
                                        dev->event_handler = tick_handle_periodic;
                                    else
                                        dev->event_handler = tick_handle_periodic_broadcast;
                                }
                            }
                        } else {
                            tick_setup_oneshot(newdev, handler, next_event);
                        }
                    }
                }
                clockevents_notify_released();

                raw_spin_unlock_irqrestore(&clockevents_lock, flags);
            }
        }

        arch_timer_rate = arch_timer_get_rate();
        if (!arch_timer_rate)
            panic("Unable to initialise architected timer.\n");

        /* Calibrate the delay loop directly */
        lpj_fine = arch_timer_rate / HZ;

        pv_time_init();
    }

    if (late_time_init)
        late_time_init();
```

```c
tick_handle_periodic() {
    tick_periodic() {
        do_timer() {
            jiffies_64 += ticks;
        }

        update_wall_time();
            timekeeping_advance();

        update_process_times() {
            run_local_timers() {
                hrtimer_run_queues() {
                    tick_check_oneshot_change() {
                        hrtimer_switch_to_hres();
                            tick_setup_sched_timer(); /* hrtimer tick emulation */
                                hrtimer_init(&ts->sched_timer, CLOCK_MONOTONIC, HRTIMER_MODE_S);
                                ts->sched_timer.function = tick_sched_timer;
                                    tick_sched_timer();
                                        tick_sched_do_timer();
                                            tick_do_update_jiffies64(now)
                                                do_timer();
                                                update_wall_time();
                                        tick_sched_handle();
                                            update_process_times();
                    }
                    !ktime_before(now, cpu_base->softirq_expires_next) {
                        raise_softirq_irqoff(HRTIMER_SOFTIRQ);
                            hrtimer_run_softirq();
                                __hrtimer_run_queues();
                    }
                    __hrtimer_run_queues();
                        __run_hrtimer();
                }

                raise_softirq(TIMER_SOFTIRQ) {
                    run_timer_softirq() {
                        __run_timers() {
                            if (!time_after_eq(jiffies, base->clk))
                                return;
                            while (time_after_eq(jiffies, base->clk)) {
                                levels = collect_expired_timers(base, heads);
                                base->clk++;

                                while (levels--) {
                                    expire_timers(base, heads + levels) {
                                        base->running_timer = timer;
                                        detach_timer(timer, true);

                                        fn = timer->function;

                                        if (timer->flags & TIMER_IRQSAFE) {
                                            raw_spin_unlock(&base->lock);
                                            call_timer_fn(timer, fn) {
                                                fn(timer);
                                            }
                                            raw_spin_lock(&base->lock);
                                        } else {
                                            raw_spin_unlock_irq(&base->lock);
                                            call_timer_fn(timer, fn);
                                            raw_spin_lock_irq(&base->lock);
                                        }
                                    }
                                }
                            }
                            base->running_timer = NULL;
                            raw_spin_unlock_irq(&base->lock);
                        }
                    }
                }
            }

            scheduler_tick() {
                curr->sched_class->task_tick(rq, curr, 0);
            }

            run_posix_cpu_timers() {
                ret = posix_cpu_timers_work_scheduled(tsk) {

                }
                if (ret)
                    return;
                ret = fastpath_timer_check(tsk) {

                }
                if (!ret)
                    return;

                __run_posix_cpu_timers(tsk) {
                    handle_posix_cpu_timers(tsk) {
                        check_thread_timers(tsk, &firing) {
                            ret = check_rlimit(rttime, hard, SIGKILL, true, true) {
                                if (time < limit)
		                            return false;
                                send_signal_locked(signo, SEND_SIG_PRIV, current, PIDTYPE_TGID);
                            }
                            if (hard != RLIM_INFINITY && ret) {
                                return;
                            }

                            if (check_rlimit(rttime, soft, SIGXCPU, true, false)) {
                                soft += USEC_PER_SEC;
                                tsk->signal->rlim[RLIMIT_RTTIME].rlim_cur = soft;
                            }
                        }
                        check_process_timers(tsk, &firing) {

                        }
                    }
                }
            }
        }
    }
}

hrtimer_interrupt() {
    entry_time = now = hrtimer_update_base(cpu_base);
retry:
    cpu_base->in_hrtirq = 1;
    cpu_base->expires_next = KTIME_MAX;

    if (!ktime_before(now, cpu_base->softirq_expires_next)) {
        cpu_base->softirq_expires_next = KTIME_MAX;
        cpu_base->softirq_activated = 1;
        raise_softirq_irqoff(HRTIMER_SOFTIRQ);
    }

    __hrtimer_run_queues(cpu_base, now, flags, HRTIMER_ACTIVE_HARD) {

    }

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

    raw_spin_lock_irqsave(&cpu_base->lock, flags);
    now = hrtimer_update_base(cpu_base);
    cpu_base->nr_retries++;
    if (++retries < 3)
        goto retry;


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

<img src='../images/kernel/time-timer.png' style='max-height:850px'/>

<img src='../images/kernel/time-timer-arch.png' style='max-height:850px'/>

<img src='../images/kernel/time-tiemer-origin.png' style='max-height:850px'/>

<img src='../images/kernel/time-timer-hrtimer.png' style='max-height:850px'/>

<img src='../images/kernel/time-timer-hrtimer-gtod.png' style='max-height:850px'/>

<img src='../images/kernel/time-timer-hrtimer-gtod-clockevent.png' style='max-height:850px'/>

<img src='../images/kernel/time-timer-hrtimer-gtod-clockevent-tick-emulation.png' style='max-height:850px'/>

# Refence:
[hrtimers and beyond](http://www.cs.columbia.edu/~nahum/w6998/papers/ols2006-hrtimers-slides.pdf)

http://www.wowotech.net/timer_subsystem/time-subsyste-architecture.html


# hrtimer_run_queues
```c
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

    /* Separate the ->running assignment from the ->state assignment.
    *
    * As with a regular write barrier, this ensures the read side in
    * hrtimer_active() cannot observe base->running == NULL &&
    * timer->state == INACTIVE. */
    raw_write_seqcount_barrier(&base->seq);

    __remove_hrtimer(timer, base, HRTIMER_STATE_INACTIVE, 0);
    fn = timer->function;

    /* Clear the 'is relative' flag for the TIME_LOW_RES case. If the
    * timer is restarted with a period then it becomes an absolute
    * timer. If its not restarted it does not matter. */
    if (IS_ENABLED(CONFIG_TIME_LOW_RES))
        timer->is_rel = false;

    /* The timer is marked as running in the CPU base, so it is
    * protected against migration to a different CPU even if the lock
    * is dropped. */
    raw_spin_unlock_irqrestore(&cpu_base->lock, flags);
    trace_hrtimer_expire_entry(timer, now);
    restart = fn(timer);
    trace_hrtimer_expire_exit(timer);
    raw_spin_lock_irq(&cpu_base->lock);

    /* Note: We clear the running state after enqueue_hrtimer and
    * we do not reprogram the event hardware. Happens either in
    * hrtimer_start_range_ns() or in hrtimer_interrupt()
    *
    * Note: Because we dropped the cpu_base->lock above,
    * hrtimer_start_range_ns() can have popped in and enqueued the timer
    * for us already. */
    if (restart != HRTIMER_NORESTART &&
        !(timer->state & HRTIMER_STATE_ENQUEUED))
        enqueue_hrtimer(timer, base, HRTIMER_MODE_ABS);

    /* Separate the ->running assignment from the ->state assignment.
    *
    * As with a regular write barrier, this ensures the read side in
    * hrtimer_active() cannot observe base->running.timer == NULL &&
    * timer->state == INACTIVE. */
    raw_write_seqcount_barrier(&base->seq);

    WARN_ON_ONCE(base->running != timer);
    base->running = NULL;
}
```

# tick_check_oneshot_change
```c
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

# hrtimer_switch_to_hres
```c
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

# hrtimer tick emulation
```c
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

# HRTIMER_SOFTIRQ
```c
DEFINE_PER_CPU(struct hrtimer_cpu_base, hrtimer_bases) = {
    .lock = __RAW_SPIN_LOCK_UNLOCKED(hrtimer_bases.lock),
    .clock_base = {
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
```

# API
# gettimeofday
```c
SYSCALL_DEFINE2(gettimeofday, struct timeval __user *, tv,
    struct timezone __user *, tz)
{
  if (likely(tv != NULL)) {
    struct timespec64 ts;

    ktime_get_real_ts64(&ts) {
        struct timekeeper *tk = &tk_core.timekeeper;
        unsigned long seq;
        u64 nsecs;

        do {
            seq = read_seqcount_begin(&tk_core.seq);

            ts->tv_sec = tk->xtime_sec;
            nsecs = timekeeping_get_ns(&tk->tkr_mono);

        } while (read_seqcount_retry(&tk_core.seq, seq));

        ts->tv_nsec = 0;
        timespec64_add_ns(ts, nsecs);
    }
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

# timer_create
```c
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
    return do_timer_create(which_clock, NULL, created_timer_id) {
        const struct k_clock *kc = clockid_to_kclock(which_clock);

        new_timer = alloc_posix_timer();

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

        error = kc->timer_create(new_timer) {
            hrtimer_init(&new_timer->it.real.timer, new_timer->it_clock, 0);
        }

        spin_lock_irq(&current->sighand->siglock);
        new_timer->it_signal = current->signal;
        list_add(&new_timer->list, &current->signal->posix_timers);
        spin_unlock_irq(&current->sighand->siglock);

        return 0;

    out:
        release_posix_timer(new_timer, it_id_set);
        return error;
    }
}
```

# timer_settime
```c
SYSCALL_DEFINE4(timer_settime, timer_t, timer_id, int, flags,
    const struct __kernel_itimerspec __user *, new_setting,
    struct __kernel_itimerspec __user *, old_setting)
{
    struct itimerspec64 new_spec, old_spec;
    struct itimerspec64 *rtn = old_setting ? &old_spec : NULL;
    int error = 0;

    get_itimerspec64(&new_spec, new_setting);

    error = do_timer_settime(timer_id, flags, &new_spec, rtn) {
        timr = lock_timer(timer_id, &flag);
        kc = timr->kclock;
        error = kc->timer_set(timr, flags, new_spec64, old_spec64);

    }
    if (!error && old_setting) {
        if (put_itimerspec64(&old_spec, old_setting))
        error = -EFAULT;
    }
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

# timer cancel
```c
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

/* A timer is active, when it is enqueued into the rbtree or the
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

# posix_timer_fn
```c
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

# kernel timer api
```c
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
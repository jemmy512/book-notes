* [中断管理 - LoyenWang](https://www.cnblogs.com/LoyenWang/category/1777370.html)
    * [1. 中断控制器及驱动分析](https://www.cnblogs.com/LoyenWang/p/12249106.html)
    * [2. 通用框架处理](https://www.cnblogs.com/LoyenWang/p/13052677.html)

* [IRQ Subsystem - WOWO TECH](http://www.wowotech.net/sort/irq_subsystem)
    * [1. 综述](http://www.wowotech.net/irq_subsystem/interrupt_subsystem_architecture.html)
    * [2. IRQ Domain介绍](http://www.wowotech.net/irq_subsystem/irq-domain.html)
    * [3. IRQ number和中断描述符](http://www.wowotech.net/irq_subsystem/interrupt_descriptor.html)
    * [4. High level irq event handler](http://www.wowotech.net/irq_subsystem/High_level_irq_event_handler.html)
    * [5. 驱动申请中断API](http://www.wowotech.net/irq_subsystem/request_threaded_irq.html)
    * [6. ARM中断处理过程](http://www.wowotech.net/irq_subsystem/irq_handler.html)
    * [7. GIC代码分析](http://www.wowotech.net/irq_subsystem/gic_driver.html)
    * [8. 中断唤醒系统流程](http://www.wowotech.net/irq_subsystem/irq_handle_procedure.html)
    * [softirq](http://www.wowotech.net/irq_subsystem/soft-irq.html)
    * [tasklet](http://www.wowotech.net/irq_subsystem/tasklet.html)
    * [ARMv8 异常处理简介](https://mp.weixin.qq.com/s/dZEMB4_Xmgd8f7GO7c8Oag)

* [深入分析linux内核源码：中断子系统详解](https://mp.weixin.qq.com/s/8BV0aQMeBtqUlKrpa8QZeg)

![](../images/kernel/intr-arch-layer.png)

---

![](../images/kernel/intr-irq_desc.png)

```c
/* The interrupt vector interrupt controller sent to
 * each cpu is per cpu local variable, but the abstract
 * layer's virtual signal irq and it's irq_desc is global.
 * So per cpu needs its own mapping from vector to irq_desc */
typedef struct irq_desc* vector_irq_t[NR_VECTORS];
DECLARE_PER_CPU(vector_irq_t, vector_irq);

struct irq_desc {
    struct irq_common_data irq_common_data;
    struct irq_data        irq_data;
    unsigned int __percpu  *kstat_irqs;
    irq_flow_handler_t     handle_irq;
    struct irqaction       *action; /* IRQ action list */

    struct cpumask        *percpu_enabled;
    wait_queue_head_t     wait_for_threads;
};

struct irqaction {
    irq_handler_t       handler; /* typedef irqreturn_t (*irq_handler_t)(int, void *) */
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
    struct proc_dir_entry *dir;
};
```

# init_IRQ

```c
void __init init_IRQ(void)
{
    init_irq_stacks();
    init_irq_scs();
    irqchip_init() {
        of_irq_init(__irqchip_of_table/* matches */) {
            const struct of_device_id *match;
            struct device_node *np, *parent = NULL;
            struct of_intc_desc *desc, *temp_desc;
            struct list_head intc_desc_list, intc_parent_list;

            INIT_LIST_HEAD(&intc_desc_list);
            INIT_LIST_HEAD(&intc_parent_list);

            for_each_matching_node_and_match(np, matches, &match) {
                if (!of_property_read_bool(np, "interrupt-controller") ||
                        !of_device_is_available(np))
                    continue;

                if (WARN(!match->data, "of_irq_init: no init function for %s\n",
                    match->compatible))
                    continue;

                /*
                * Here, we allocate and populate an of_intc_desc with the node
                * pointer, interrupt-parent device_node etc.
                */
                desc = kzalloc(sizeof(*desc), GFP_KERNEL);
                if (!desc) {
                    of_node_put(np);
                    goto err;
                }

                desc->irq_init_cb = match->data;
                desc->dev = of_node_get(np);
                /*
                * interrupts-extended can reference multiple parent domains.
                * Arbitrarily pick the first one; assume any other parents
                * are the same distance away from the root irq controller.
                */
                desc->interrupt_parent = of_parse_phandle(np, "interrupts-extended", 0);
                if (!desc->interrupt_parent)
                    desc->interrupt_parent = of_irq_find_parent(np);
                if (desc->interrupt_parent == np) {
                    of_node_put(desc->interrupt_parent);
                    desc->interrupt_parent = NULL;
                }
                list_add_tail(&desc->list, &intc_desc_list);
            }

            /*
            * The root irq controller is the one without an interrupt-parent.
            * That one goes first, followed by the controllers that reference it,
            * followed by the ones that reference the 2nd level controllers, etc.
            */
            while (!list_empty(&intc_desc_list)) {
                /*
                * Process all controllers with the current 'parent'.
                * First pass will be looking for NULL as the parent.
                * The assumption is that NULL parent means a root controller.
                */
                list_for_each_entry_safe(desc, temp_desc, &intc_desc_list, list) {
                    int ret;

                    if (desc->interrupt_parent != parent)
                        continue;

                    list_del(&desc->list);

                    of_node_set_flag(desc->dev, OF_POPULATED);

                    ret = desc->irq_init_cb(desc->dev, desc->interrupt_parent) {
                        gic_of_init(); /* for gic_v3 */
                    }
                    if (ret) {
                        pr_err("%s: Failed to init %pOF (%p), parent %p\n",
                            __func__, desc->dev, desc->dev,
                            desc->interrupt_parent);
                        of_node_clear_flag(desc->dev, OF_POPULATED);
                        kfree(desc);
                        continue;
                    }

                    /*
                    * This one is now set up; add it to the parent list so
                    * its children can get processed in a subsequent pass.
                    */
                    list_add_tail(&desc->list, &intc_parent_list);
                }

                /* Get the next pending parent that might have children */
                desc = list_first_entry_or_null(&intc_parent_list,
                                typeof(*desc), list);
                if (!desc) {
                    pr_err("of_irq_init: children remain, but no parents\n");
                    break;
                }
                list_del(&desc->list);
                parent = desc->dev;
                kfree(desc);
            }

            list_for_each_entry_safe(desc, temp_desc, &intc_parent_list, list) {
                list_del(&desc->list);
                kfree(desc);
            }
        err:
            list_for_each_entry_safe(desc, temp_desc, &intc_desc_list, list) {
                list_del(&desc->list);
                of_node_put(desc->dev);
                kfree(desc);
            }
        }

        acpi_probe_device_table(irqchip);
        #define acpi_probe_device_table(t)  \
        ({  \
            extern struct acpi_probe_entry ACPI_PROBE_TABLE(t), \
                                    ACPI_PROBE_TABLE_END(t);    \
            __acpi_probe_device_table(&ACPI_PROBE_TABLE(t),     \
                        (&ACPI_PROBE_TABLE_END(t) - \
                        &ACPI_PROBE_TABLE(t)));     \
        })
    }

    if (system_uses_irq_prio_masking()) {
        /*
        * Now that we have a stack for our IRQ handler, set
        * the PMR/PSR pair to a consistent state.
        */
        WARN_ON(read_sysreg(daif) & PSR_A_BIT);
        local_daif_restore(DAIF_PROCCTX_NOIRQ);
    }
}
```

# request_irq

![](../images/kernel/intr-request_irq.png)

```c
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

}

irqreturn_t (*irq_handler_t)(int irq, void * dev_id);

enum irqreturn {
    IRQ_NONE            = (0 << 0),
    IRQ_HANDLED         = (1 << 0),
    IRQ_WAKE_THREAD     = (1 << 1),
};

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
    /* add new interrupt at end of irq queue */
    if (old) {
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
```

## request_percpu_irq

```c
request_percpu_irq(unsigned int irq, irq_handler_t handler,
           const char *devname, void __percpu *percpu_dev_id) {
    return __request_percpu_irq(irq, handler, 0, devname, percpu_dev_id) {
        desc = irq_to_desc(irq);

        action = kzalloc(sizeof(struct irqaction), GFP_KERNEL);
        action->handler = handler;
        action->flags = flags | IRQF_PERCPU | IRQF_NO_SUSPEND;
        action->name = devname;
        action->percpu_dev_id = dev_id;

        retval = irq_chip_pm_get(&desc->irq_data);

        retval = __setup_irq(irq, desc, action /*new*/) {
            new->irq = irq;

            nested = irq_settings_is_nested_thread(desc);
            if (nested) {
                if (!new->thread_fn) {
                    ret = -EINVAL;
                    goto out_mput;
                }
                new->handler = irq_nested_primary_handler;
            } else {
                if (irq_settings_can_thread(desc)) {
                    ret = irq_setup_forced_threading(new);
                    if (ret)
                        goto out_mput;
                }
            }

            /* Create a handler thread when a thread function is supplied
             * and the interrupt does not nest into another interrupt thread. */
            if (new->thread_fn && !nested) {
                ret = setup_irq_thread(new, irq, false) {
                    kthread_create(irq_thread, new, "irq/%d-%s", irq, new->name);
                }
                if (ret)
                    goto out_mput;
                if (new->secondary) {
                    ret = setup_irq_thread(new->secondary, irq, true);
                    if (ret)
                        goto out_thread;
                }
            }

            if (desc->irq_data.chip->flags & IRQCHIP_ONESHOT_SAFE)
                new->flags &= ~IRQF_ONESHOT;

            mutex_lock(&desc->request_mutex);

            chip_bus_lock(desc);

            /* First installed action requests resources. */
            if (!desc->action) {
                ret = irq_request_resources(desc);
                if (ret) {
                    goto out_bus_unlock;
                }
            }

            raw_spin_lock_irqsave(&desc->lock, flags);
            old_ptr = &desc->action;
            old = *old_ptr;
            if (old) {
                unsigned int oldtype;

                if (desc->istate & IRQS_NMI) {
                    ret = -EINVAL;
                    goto out_unlock;
                }

                if (irqd_trigger_type_was_set(&desc->irq_data)) {
                    oldtype = irqd_get_trigger_type(&desc->irq_data);
                } else {
                    oldtype = new->flags & IRQF_TRIGGER_MASK;
                    irqd_set_trigger_type(&desc->irq_data, oldtype);
                }

                if (!((old->flags & new->flags) & IRQF_SHARED) ||
                    (oldtype != (new->flags & IRQF_TRIGGER_MASK)) ||
                    ((old->flags ^ new->flags) & IRQF_ONESHOT))
                    goto mismatch;

                /* All handlers must agree on per-cpuness */
                if ((old->flags & IRQF_PERCPU) !=
                    (new->flags & IRQF_PERCPU))
                    goto mismatch;

                /* add new interrupt at end of irq queue */
                do {
                    thread_mask |= old->thread_mask;
                    old_ptr = &old->next;
                    old = *old_ptr;
                } while (old);
                shared = 1;
            }

            /*
            * Setup the thread mask for this irqaction for ONESHOT. For
            * !ONESHOT irqs the thread mask is 0 so we can avoid a
            * conditional in irq_wake_thread().
            */
            if (new->flags & IRQF_ONESHOT) {
                /*
                * Unlikely to have 32 resp 64 irqs sharing one line,
                * but who knows.
                */
                if (thread_mask == ~0UL) {
                    ret = -EBUSY;
                    goto out_unlock;
                }
                /*
                * The thread_mask for the action is or'ed to
                * desc->thread_active to indicate that the
                * IRQF_ONESHOT thread handler has been woken, but not
                * yet finished. The bit is cleared when a thread
                * completes. When all threads of a shared interrupt
                * line have completed desc->threads_active becomes
                * zero and the interrupt line is unmasked. See
                * handle.c:irq_wake_thread() for further information.
                *
                * If no thread is woken by primary (hard irq context)
                * interrupt handlers, then desc->threads_active is
                * also checked for zero to unmask the irq line in the
                * affected hard irq flow handlers
                * (handle_[fasteoi|level]_irq).
                *
                * The new action gets the first zero bit of
                * thread_mask assigned. See the loop above which or's
                * all existing action->thread_mask bits.
                */
                new->thread_mask = 1UL << ffz(thread_mask);

            } else if (new->handler == irq_default_primary_handler &&
                !(desc->irq_data.chip->flags & IRQCHIP_ONESHOT_SAFE)) {
                /*
                * The interrupt was requested with handler = NULL, so
                * we use the default primary handler for it. But it
                * does not have the oneshot flag set. In combination
                * with level interrupts this is deadly, because the
                * default primary handler just wakes the thread, then
                * the irq lines is reenabled, but the device still
                * has the level irq asserted. Rinse and repeat....
                *
                * While this works for edge type interrupts, we play
                * it safe and reject unconditionally because we can't
                * say for sure which type this interrupt really
                * has. The type flags are unreliable as the
                * underlying chip implementation can override them.
                */
                pr_err("Threaded irq requested with handler=NULL and !ONESHOT for %s (irq %d)\n",
                    new->name, irq);
                ret = -EINVAL;
                goto out_unlock;
            }

            if (!shared) {
                /* Setup the type (level, edge polarity) if configured: */
                if (new->flags & IRQF_TRIGGER_MASK) {
                    ret = __irq_set_trigger(desc,
                                new->flags & IRQF_TRIGGER_MASK);

                    if (ret)
                        goto out_unlock;
                }

                /*
                * Activate the interrupt. That activation must happen
                * independently of IRQ_NOAUTOEN. request_irq() can fail
                * and the callers are supposed to handle
                * that. enable_irq() of an interrupt requested with
                * IRQ_NOAUTOEN is not supposed to fail. The activation
                * keeps it in shutdown mode, it merily associates
                * resources if necessary and if that's not possible it
                * fails. Interrupts which are in managed shutdown mode
                * will simply ignore that activation request.
                */
                ret = irq_activate(desc);
                if (ret)
                    goto out_unlock;

                desc->istate &= ~(IRQS_AUTODETECT | IRQS_SPURIOUS_DISABLED | \
                        IRQS_ONESHOT | IRQS_WAITING);
                irqd_clear(&desc->irq_data, IRQD_IRQ_INPROGRESS);

                if (new->flags & IRQF_PERCPU) {
                    irqd_set(&desc->irq_data, IRQD_PER_CPU);
                    irq_settings_set_per_cpu(desc);
                    if (new->flags & IRQF_NO_DEBUG)
                        irq_settings_set_no_debug(desc);
                }

                if (noirqdebug)
                    irq_settings_set_no_debug(desc);

                if (new->flags & IRQF_ONESHOT)
                    desc->istate |= IRQS_ONESHOT;

                /* Exclude IRQ from balancing if requested */
                if (new->flags & IRQF_NOBALANCING) {
                    irq_settings_set_no_balancing(desc);
                    irqd_set(&desc->irq_data, IRQD_NO_BALANCING);
                }

                if (!(new->flags & IRQF_NO_AUTOEN) &&
                    irq_settings_can_autoenable(desc)) {
                    irq_startup(desc, IRQ_RESEND, IRQ_START_COND);
                } else {
                    /*
                    * Shared interrupts do not go well with disabling
                    * auto enable. The sharing interrupt might request
                    * it while it's still disabled and then wait for
                    * interrupts forever.
                    */
                    WARN_ON_ONCE(new->flags & IRQF_SHARED);
                    /* Undo nested disables: */
                    desc->depth = 1;
                }

            } else if (new->flags & IRQF_TRIGGER_MASK) {
                unsigned int nmsk = new->flags & IRQF_TRIGGER_MASK;
                unsigned int omsk = irqd_get_trigger_type(&desc->irq_data);

                if (nmsk != omsk)
                    /* hope the handler works with current  trigger mode */
                    pr_warn("irq %d uses trigger mode %u; requested %u\n",
                        irq, omsk, nmsk);
            }

            *old_ptr = new;

            irq_pm_install_action(desc, new);

            /* Reset broken irq detection when installing new handler */
            desc->irq_count = 0;
            desc->irqs_unhandled = 0;

            /*
            * Check whether we disabled the irq via the spurious handler
            * before. Reenable it and give it another chance.
            */
            if (shared && (desc->istate & IRQS_SPURIOUS_DISABLED)) {
                desc->istate &= ~IRQS_SPURIOUS_DISABLED;
                __enable_irq(desc);
            }

            raw_spin_unlock_irqrestore(&desc->lock, flags);
            chip_bus_sync_unlock(desc);
            mutex_unlock(&desc->request_mutex);

            irq_setup_timings(desc, new);

            wake_up_and_wait_for_irq_thread_ready(desc, new);
            wake_up_and_wait_for_irq_thread_ready(desc, new->secondary);

            register_irq_proc(irq, desc);
            new->dir = NULL;
            register_handler_proc(irq, new);
            return 0;
        }

        return retval;
    }
}
```

# free_irq

# enable-disable_irq

# softirq

VS | standard kernel | PREEMPT_RT kernel
--- | --- | ---
Interrupt bottom half | invokes softirq to handle the irqs | just wakes up ksoftirqd
__local_bh_enable_ip  | call do_softirq if not in irq ctx | call do_softirq in preemptable and irq enabled ctx otherwise wakeup ksoftirqd
ksoftirqd prio | bh > RT > ksoftirqd == fair | bh > ksoftirqd == RT > fair
ksoftirqd preemptable | preempted only by hard irq | fully preemptable
mutual exclusion | by disabling bh | by disabling preemption
softirq ctrl & preemption | diable softirq will disalbe preemption | decouple the binding

soft interrupt execution point:
1. Interrupt bottom half

    After the hard interrupt is processed, call **irq_exit** to exit. If it is detected that there is a soft interrupt to be processed, call **invoke_softirq** function to process the soft interrupt.

    ```c
    irq_exit_rcu() {
        preempt_count_sub(HARDIRQ_OFFSET);
        if (!in_interrupt() && local_softirq_pending())
            invoke_softirq();
        tick_irq_exit();
    }
    ```

    ```c
    #ifdef CONFIG_PREEMPT_RT
        /* PREEMPT_RT kernel just wakes up softirqd */
        static inline void invoke_softirq(void) {
            if (should_wake_ksoftirqd() { return !this_cpu_read(softirq_ctrl.cnt) }) {
                wakeup_softirqd();
            }
        }
    #else
        /* standard kernel invokes softirq to handle the irqs */
        static inline void invoke_softirq(void) {
            if (!force_irqthreads() || !__this_cpu_read(ksoftirqd)) {
                handle_softirqs();
            } else {
                wakeup_softirqd();
            }
        }
    ```

2. If the soft interrupt is not processed at the previous execution point, wake up ksoftirqd and process it in ksoftirqd. The ksoftirqd thread is a normal thread of the default priority fair scheduling class

    ```c
    handle_softirqs(void){
        pending = local_softirq_pending();
        h = softirq_vec;

        while ((softirq_bit = ffs(pending))) {
            h->action(h); /* soft irq handler, e.g., net_rx_action, net_tx_action */
            h++;
            pending >>= softirq_bit;
        }

        pending = local_softirq_pending();
        if (pending) {
            if (time_before(jiffies, end) && !need_resched() && --max_restart)
            goto restart;

            wakeup_softirqd();
        }
    }
    ```

3. Some mutual exclusion mechanisms that close the bottom half (such as spin_lock_bh/spin_unlock_bh), when the outermost bottom half is enabled Call **__local_bh_enable_ip** function, if not in the interrupt context, call do_softirq() to process the soft interrupt

    ```c
    void __local_bh_enable_ip(unsigned long ip, unsigned int cnt) {
        bool preempt_on = preemptible();
        pending = local_softirq_pending();

        if (!preempt_on) {
            wakeup_softirqd();
            goto out;
        }

        cnt = SOFTIRQ_OFFSET;
        __local_bh_enable(cnt, false);
        handle_softirqs();
    }
    ```

## ksoftirqd

```c
static struct smp_hotplug_thread softirq_threads = {
    .store              = &ksoftirqd,
    .thread_should_run  = ksoftirqd_should_run,
    .thread_fn          = run_ksoftirqd,
    .thread_comm        = "ksoftirqd/%u",
};

static __init int spawn_ksoftirqd(void)
{
    cpuhp_setup_state_nocalls(CPUHP_SOFTIRQ_DEAD, "softirq:dead", NULL, takeover_tasklets);
    BUG_ON(smpboot_register_percpu_thread(&softirq_threads));

    return 0;
}
early_initcall(spawn_ksoftirqd);

/* kernel/smpboot.c */
static LIST_HEAD(hotplug_threads);
static DEFINE_MUTEX(smpboot_threads_lock);

int smpboot_register_percpu_thread(struct smp_hotplug_thread *plug_thread)
{
    unsigned int cpu;
    int ret = 0;

    get_online_cpus();
    mutex_lock(&smpboot_threads_lock);
    for_each_online_cpu(cpu) {
        ret = __smpboot_create_thread(plug_thread, cpu);
        if (ret) {
        smpboot_destroy_threads(plug_thread);
        goto out;
        }
        smpboot_unpark_thread(plug_thread, cpu);
    }
    list_add(&plug_thread->list, &hotplug_threads);
out:
    mutex_unlock(&smpboot_threads_lock);
    put_online_cpus();
    return ret;
}

static int
__smpboot_create_thread(struct smp_hotplug_thread *ht, unsigned int cpu)
{
    struct task_struct *tsk = *per_cpu_ptr(ht->store, cpu);
    struct smpboot_thread_data *td;

    if (tsk)
        return 0;

    td = kzalloc_node(sizeof(*td), GFP_KERNEL, cpu_to_node(cpu));
    if (!td)
        return -ENOMEM;
    td->cpu = cpu;
    td->ht = ht;

    tsk = kthread_create_on_cpu(smpboot_thread_fn, td, cpu, ht->thread_comm);
    if (IS_ERR(tsk)) {
        kfree(td);
        return PTR_ERR(tsk);
    }

    /* Park the thread so that it could start right on the CPU
     * when it is available. */
    kthread_park(tsk);
    get_task_struct(tsk);
    *per_cpu_ptr(ht->store, cpu) = tsk;
    if (ht->create) {
        /* Make sure that the task has actually scheduled out
        * into park position, before calling the create
        * callback. At least the migration thread callback
        * requires that the task is off the runqueue. */
        if (!wait_task_inactive(tsk, TASK_PARKED))
            WARN_ON(1);
        else
            ht->create(cpu);
    }
    return 0;
}

struct task_struct *kthread_create_on_cpu(
  int (*threadfn)(void *data),
  void *data, unsigned int cpu,
  const char *namefmt)
{
    struct task_struct *p;

    p = kthread_create_on_node(threadfn, data, cpu_to_node(cpu), namefmt, cpu);
    if (IS_ERR(p))
        return p;
    kthread_bind(p, cpu);
    /* CPU hotplug need to bind once again when unparking the thread. */
    set_bit(KTHREAD_IS_PER_CPU, &to_kthread(p)->flags);
    to_kthread(p)->cpu = cpu;
    return p;
}

void kthread_bind(struct task_struct *p, unsigned int cpu)
{
    __kthread_bind(p, cpu, TASK_UNINTERRUPTIBLE);
}

void __kthread_bind_mask(struct task_struct *p, const struct cpumask *mask, long state)
{
    unsigned long flags;

    if (!wait_task_inactive(p, state)) {
        WARN_ON(1);
        return;
    }

    /* It's safe because the task is inactive. */
    raw_spin_lock_irqsave(&p->pi_lock, flags);
    do_set_cpus_allowed(p, mask);
    p->flags |= PF_NO_SETAFFINITY;
    raw_spin_unlock_irqrestore(&p->pi_lock, flags);
}

void do_set_cpus_allowed(struct task_struct *p, const struct cpumask *new_mask)
{
    struct rq *rq = task_rq(p);
    bool queued, running;

    lockdep_assert_held(&p->pi_lock);

    queued = task_on_rq_queued(p);
    running = task_current(rq, p);

    if (queued) {
        /* Because __kthread_bind() calls this on blocked tasks without
        * holding rq->lock. */
        lockdep_assert_held(&rq->lock);
        dequeue_task(rq, p, DEQUEUE_SAVE | DEQUEUE_NOCLOCK);
    }
    if (running)
        put_prev_task(rq, p);

    p->sched_class->set_cpus_allowed(p, new_mask);

    if (queued)
        enqueue_task(rq, p, ENQUEUE_RESTORE | ENQUEUE_NOCLOCK);
    if (running)
        set_curr_task(rq, p);
}
const struct sched_class fair_sched_class = {
  #ifdef CONFIG_SMP
  .set_cpus_allowed  = set_cpus_allowed_common,
#endif
};

void set_cpus_allowed_common(struct task_struct *p, const struct cpumask *new_mask)
{
    cpumask_copy(&p->cpus_allowed, new_mask);
    p->nr_cpus_allowed = cpumask_weight(new_mask);
}
```

## run_ksoftirqd

```c
int smpboot_thread_fn(void *data)
{
    struct smpboot_thread_data *td = data;
    struct smp_hotplug_thread *ht = td->ht;

    while (1) {
        set_current_state(TASK_INTERRUPTIBLE);
        preempt_disable();
        if (kthread_should_stop()) {
            __set_current_state(TASK_RUNNING);
            preempt_enable();
            /* cleanup must mirror setup */
            if (ht->cleanup && td->status != HP_THREAD_NONE)
                ht->cleanup(td->cpu, cpu_online(td->cpu));
            kfree(td);
            return 0;
        }

        if (kthread_should_park()) {
            __set_current_state(TASK_RUNNING);
            preempt_enable();
            if (ht->park && td->status == HP_THREAD_ACTIVE) {
                BUG_ON(td->cpu != smp_processor_id());
                ht->park(td->cpu);
                td->status = HP_THREAD_PARKED;
            }
            kthread_parkme();
            /* We might have been woken for stop */
            continue;
        }

        BUG_ON(td->cpu != smp_processor_id());

        /* Check for state change setup */
        switch (td->status) {
        case HP_THREAD_NONE:
            __set_current_state(TASK_RUNNING);
            preempt_enable();
            if (ht->setup)
                ht->setup(td->cpu);
            td->status = HP_THREAD_ACTIVE;
            continue;

        case HP_THREAD_PARKED:
            __set_current_state(TASK_RUNNING);
            preempt_enable();
            if (ht->unpark)
                ht->unpark(td->cpu);
            td->status = HP_THREAD_ACTIVE;
            continue;
        }

        if (!ht->thread_should_run(td->cpu)) {
            preempt_enable_no_resched();
            schedule();
        } else {
            __set_current_state(TASK_RUNNING);
            preempt_enable();
            ht->thread_fn(td->cpu); /* run_ksoftirqd */
        }
    }
}

static void run_ksoftirqd(unsigned int cpu)
{
    ksoftirqd_run_begin() {
      #ifdef CONFIG_PREEMPT_RT
        __local_bh_disable_ip(_RET_IP_, SOFTIRQ_OFFSET);
        local_irq_disable();
      #else
        local_irq_disable();
    }

    if (local_softirq_pending()) {
        handle_softirqs(true);
        ksoftirqd_run_end();
        cond_resched();
        return;
    }
    ksoftirqd_run_end();
}

struct softirq_action
{
    void (*action)(struct softirq_action *);
};

void handle_softirqs(void)
{
    unsigned long end = jiffies + MAX_SOFTIRQ_TIME;
    unsigned long old_flags = current->flags;
    int max_restart = MAX_SOFTIRQ_RESTART;
    struct softirq_action *h;
    bool in_hardirq;
    __u32 pending;
    int softirq_bit;

    /* Mask out PF_MEMALLOC s current task context is borrowed for the
    * softirq. A softirq handled such as network RX might set PF_MEMALLOC
    * again if the socket is related to swap */
    current->flags &= ~PF_MEMALLOC;

    pending = local_softirq_pending();
    account_irq_enter_time(current);

    __local_bh_disable_ip(_RET_IP_, SOFTIRQ_OFFSET);
    in_hardirq = lockdep_softirq_start();

restart:
  /* Reset the pending bitmask before enabling irqs */
    set_softirq_pending(0);

    local_irq_enable();

    h = softirq_vec;

    while ((softirq_bit = ffs(pending))) {
        unsigned int vec_nr;
        int prev_count;

        h += softirq_bit - 1;

        vec_nr = h - softirq_vec;
        prev_count = preempt_count();

        kstat_incr_softirqs_this_cpu(vec_nr);

        h->action(h); /* soft irq handler, e.g., net_rx_action, net_tx_action */
        h++;
        pending >>= softirq_bit;
    }

    rcu_bh_qs();
    local_irq_disable();

    pending = local_softirq_pending();
    if (pending) {
        if (time_before(jiffies, end) && !need_resched() && --max_restart)
        goto restart;

        wakeup_softirqd();
    }

    lockdep_softirq_end(in_hardirq);
    account_irq_exit_time(current);
    __local_bh_enable(SOFTIRQ_OFFSET);
    WARN_ON_ONCE(in_interrupt());
    current_restore_flags(old_flags, PF_MEMALLOC);
}
```

## open_softirq

```c
void __init softirq_init(void)
{
    int cpu;

    for_each_possible_cpu(cpu) {
        per_cpu(tasklet_vec, cpu).tail = &per_cpu(tasklet_vec, cpu).head;
        per_cpu(tasklet_hi_vec, cpu).tail = &per_cpu(tasklet_hi_vec, cpu).head;
    }

    open_softirq(TASKLET_SOFTIRQ, tasklet_action);
    open_softirq(HI_SOFTIRQ, tasklet_hi_action);
}

enum
{
    HI_SOFTIRQ=0,
    TIMER_SOFTIRQ,
    NET_TX_SOFTIRQ,
    NET_RX_SOFTIRQ,
    BLOCK_SOFTIRQ,
    IRQ_POLL_SOFTIRQ,
    TASKLET_SOFTIRQ,
    SCHED_SOFTIRQ,
    HRTIMER_SOFTIRQ,
    RCU_SOFTIRQ,
    NR_SOFTIRQS
};

struct softirq_action
{
    void (*action)(struct softirq_action *);
};

static struct softirq_action softirq_vec[NR_SOFTIRQS] __cacheline_aligned_in_smp;

void open_softirq(int nr, void (*action)(struct softirq_action *))
{
    softirq_vec[nr].action = action;
}
```

## raise_softirq_irqoff

To summarize, each softirq goes through the following stages:
* Registration of a softirq with the `open_softirq` function.
* Activation of a softirq by marking it as deferred with the `raise_softirq` function.
* After this, all marked softirqs will be triggered in the next time the Linux kernel schedules a round of executions of deferrable functions.
* And execution of the deferred functions that have the same type.


```c
void raise_softirq_irqoff(unsigned int nr)
{
    __raise_softirq_irqoff(nr) {
        or_softirq_pending(1UL << nr);
    }

    if (!in_interrupt())
        wakeup_softirqd();
}

static __always_inline int preempt_count(void)
{
  return raw_cpu_read_4(__preempt_count) & ~PREEMPT_NEED_RESCHED;
}

/* 1. per cpu data, non-zero means preemption is disabled
0-7   Preemption counter (max value = 255)
8-15  Softirq counter (max value = 255), indicate if soft IRQ processing is active,
      while __softirq_pending shows which specific soft IRQs are pending
16-27 Hardirq counter (max value = 4096)
28    PREEMPT_ACTIVE flag */
DECLARE_PER_CPU(int, __preempt_count);

/* 2. per cpu bitmap that indicates which soft IRQs are pending for the CPU. */
typedef struct {
    unsigned int __softirq_pending;
} ____cacheline_aligned irq_cpustat_t;
DEFINE_PER_CPU_ALIGNED(irq_cpustat_t, irq_stat);

/* 3. per task data */
struct task_struct {
    struct thread_info {
        unsigned long       flags;  /* TIF_SIGPENDING, TIF_NEED_RESCHED */
        u64                 ttbr0;
        union {
            u64             preempt_count;  /* 0 => preemptible, <0 => bug */
            struct {
                u32         count;
                u32         need_resched;
            } preempt;
        };
        u32                 cpu;
    } thread_info;
};

#define hardirq_count() (preempt_count() & HARDIRQ_MASK)
#define softirq_count() (preempt_count() & SOFTIRQ_MASK)
#define irq_count() (preempt_count() & (HARDIRQ_MASK | SOFTIRQ_MASK | NMI_MASK))

#define in_irq()  (hardirq_count())
#define in_softirq()  (softirq_count())
#define in_interrupt()  (irq_count())
#define in_serving_softirq() (softirq_count() & SOFTIRQ_OFFSET)
#define in_nmi()  (preempt_count() & NMI_MASK)
#define in_task()  (!(preempt_count() & (NMI_MASK | HARDIRQ_MASK | SOFTIRQ_OFFSET)))

void __raise_softirq_irqoff(unsigned int nr)
{
    or_softirq_pending(1UL << nr);
}

#define local_softirq_pending_ref irq_stat.__softirq_pending

#define local_softirq_pending() (__this_cpu_read(local_softirq_pending_ref))
#define set_softirq_pending(x) (__this_cpu_write(local_softirq_pending_ref, (x)))
#define or_softirq_pending(x) (__this_cpu_or(local_softirq_pending_ref, (x)))

#define hardirq_count() (preempt_count() & HARDIRQ_MASK)
#define softirq_count() (preempt_count() & SOFTIRQ_MASK)
#define irq_count() (preempt_count() & (HARDIRQ_MASK | SOFTIRQ_MASK | NMI_MASK))

#define in_irq()  (hardirq_count())
#define in_softirq()  (softirq_count())
#define in_interrupt()  (irq_count())


void wakeup_softirqd(void)
{
    struct task_struct *tsk = __this_cpu_read(ksoftirqd);

    if (tsk && tsk->state != TASK_RUNNING)
        wake_up_process(tsk);
}
```

# tasklet

* [LWN - The end of tasklets](https://lwn.net/Articles/960041/)
* [\[PATCH 3/8\] workqueue: Implement BH workqueues to eventually replace tasklets](https://lore.kernel.org/all/20240130091300.2968534-4-tj@kernel.org/)

## tasklet_action

```c
static DEFINE_PER_CPU(struct tasklet_head, tasklet_vec);
static DEFINE_PER_CPU(struct tasklet_head, tasklet_hi_vec);

struct tasklet_head {
  struct tasklet_struct *head;
  struct tasklet_struct **tail;
};

struct tasklet_struct
{
    struct tasklet_struct *next;
    unsigned long         state;
    atomic_t              count; /* 0 enabled, non-0 disabled */
    void (*func)(unsigned long);
    unsigned long         data;
};

void __init softirq_init(void)
{
    int cpu;

    for_each_possible_cpu(cpu) {
        per_cpu(tasklet_vec, cpu).tail = &per_cpu(tasklet_vec, cpu).head;
        per_cpu(tasklet_hi_vec, cpu).tail = &per_cpu(tasklet_hi_vec, cpu).head;
    }

    open_softirq(TASKLET_SOFTIRQ, tasklet_action);
    open_softirq(HI_SOFTIRQ, tasklet_hi_action);
}

static __latent_entropy void tasklet_action(struct softirq_action *a)
{
    tasklet_action_common(a, this_cpu_ptr(&tasklet_vec), TASKLET_SOFTIRQ);
}

static void tasklet_action_common(struct softirq_action *a,
    struct tasklet_head *tl_head,
    unsigned int softirq_nr)
{
    struct tasklet_struct *list;

    local_irq_disable();
    list = tl_head->head;
    tl_head->head = NULL;
    tl_head->tail = &tl_head->head;
    local_irq_enable();

    while (list) {
        struct tasklet_struct *t = list;

        list = list->next;

        if (tasklet_trylock(t)) {
            if (!atomic_read(&t->count)) {
                if (!test_and_clear_bit(TASKLET_STATE_SCHED, &t->state))
                    BUG();
                t->func(t->data);
                tasklet_unlock(t);
                continue;
            }
            tasklet_unlock(t);
        }

        local_irq_disable();
        t->next = NULL;
        *tl_head->tail = t;
        tl_head->tail = &t->next;
        __raise_softirq_irqoff(softirq_nr);
        local_irq_enable();
    }
}
```

## tasklet_schedule

```c
void tasklet_init(struct tasklet_struct *t,
      void (*func)(unsigned long), unsigned long data)
{
    t->next = NULL;
    t->state = 0;
    atomic_set(&t->count, 0);
    t->func = func;
    t->data = data;
}

enum {
    TASKLET_STATE_SCHED,  /* Tasklet is scheduled for execution */
    TASKLET_STATE_RUN     /* Tasklet is running (SMP only) */
};

void tasklet_schedule(struct tasklet_struct *t)
{
    if (!test_and_set_bit(TASKLET_STATE_SCHED, &t->state))
        __tasklet_schedule(t);
}

void __tasklet_schedule(struct tasklet_struct *t)
{
    __tasklet_schedule_common(t, &tasklet_vec, TASKLET_SOFTIRQ);
}

void __tasklet_schedule_common(struct tasklet_struct *t,
  struct tasklet_head __percpu *headp,
  unsigned int softirq_nr)
{
    struct tasklet_head *head;
    unsigned long flags;

    local_irq_save(flags);
    head = this_cpu_ptr(headp);
    t->next = NULL;
    *head->tail = t;
    head->tail = &(t->next);
    raise_softirq_irqoff(softirq_nr);
    local_irq_restore(flags);
}
```
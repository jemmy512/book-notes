* [vector](#vectors)
* [64_irq_handler](#64_irq_handler)
* [64_sync_handler](#64_sync_handler)


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

# vectors

```c
SYM_FUNC_START_LOCAL(__primary_switched)
    adr_l    x4, init_task
    init_cpu_task x4, x5, x6

    adr_l    x8, vectors            // load VBAR_EL1 with virtual
    msr    vbar_el1, x8            // vector table address
    isb

SYM_FUNC_START_LOCAL(__secondary_switched)
    mov    x0, x20
    bl    set_cpu_boot_mode_flag

    mov    x0, x20
    bl    finalise_el2

    str_l    xzr, __early_cpu_boot_status, x3
    adr_l    x5, vectors
    msr    vbar_el1, x5
    isb
```

```c
SYM_CODE_START(vectors)
    /* ELxt: Exception Level x using the thread stack pointer (SP_EL0)
     * ELxh: Exception Level x using the handler stack pointer (SP_EL1) */
    kernel_ventry    1, t, 64, sync     // Synchronous EL1t
    kernel_ventry    1, t, 64, irq      // IRQ EL1t
    kernel_ventry    1, t, 64, fiq      // FIQ EL1t
    kernel_ventry    1, t, 64, error    // Error EL1t

    kernel_ventry    1, h, 64, sync     // Synchronous EL1h
    kernel_ventry    1, h, 64, irq      // IRQ EL1h
    kernel_ventry    1, h, 64, fiq      // FIQ EL1h
    kernel_ventry    1, h, 64, error    // Error EL1h

    kernel_ventry    0, t, 64, sync     // Synchronous 64-bit EL0
    kernel_ventry    0, t, 64, irq      // IRQ 64-bit EL0
    kernel_ventry    0, t, 64, fiq      // FIQ 64-bit EL0
    kernel_ventry    0, t, 64, error    // Error 64-bit EL0

    kernel_ventry    0, t, 32, sync     // Synchronous 32-bit EL0
    kernel_ventry    0, t, 32, irq      // IRQ 32-bit EL0
    kernel_ventry    0, t, 32, fiq      // FIQ 32-bit EL0
    kernel_ventry    0, t, 32, error    // Error 32-bit EL0
SYM_CODE_END(vectors)

.macro kernel_ventry, el:req, ht:req, regsize:req, label:req
    .align 7
.Lventry_start\@:
    .if    \el == 0
    b    .Lskip_tramp_vectors_cleanup\@

    .if    \regsize == 64
    mrs    x30, tpidrro_el0
    msr    tpidrro_el0, xzr
    .else
    mov    x30, xzr
    .endif
.Lskip_tramp_vectors_cleanup\@:
    .endif /* \el == 0 */

    /* 1. alloc stack space for pt_regs */
    sub    sp, sp, #PT_REGS_SIZE
#ifdef CONFIG_VMAP_STACK
    add    sp, sp, x0            // sp' = sp + x0
    sub    x0, sp, x0            // x0' = sp' - x0 = (sp + x0) - x0 = sp
    tbnz    x0, #THREAD_SHIFT, 0f
    sub    x0, sp, x0            // x0'' = sp' - x0' = (sp + x0) - sp = x0
    sub    sp, sp, x0            // sp'' = sp' - x0 = (sp + x0) - x0 = sp
    b    el\el\ht\()_\regsize\()_\label

0:
    /* Stash the original SP (minus PT_REGS_SIZE) in tpidr_el0. */
    msr    tpidr_el0, x0

    /* Recover the original x0 value and stash it in tpidrro_el0 */
    sub    x0, sp, x0
    msr    tpidrro_el0, x0

    /* Switch to the overflow stack */
    adr_this_cpu sp, overflow_stack + OVERFLOW_STACK_SIZE, x0

    mrs    x0, tpidr_el0            // sp of interrupted context
    sub    x0, sp, x0            // delta with top of overflow stack
    tst    x0, #~(OVERFLOW_STACK_SIZE - 1)    // within range?
    b.ne    __bad_stack            // no? -> bad stack pointer

    /* We were already on the overflow stack. Restore sp/x0 and carry on. */
    sub    sp, sp, x0
    mrs    x0, tpidrro_el0
#endif
    /* 2. call entry_handler */
    b    el\el\ht\()_\regsize\()_\label
.org .Lventry_start\@ + 128    // Did we overflow the ventry slot?
    .endm
```

## entry_handler

```c
entry_handler    1, h, 64, sync
entry_handler    1, h, 64, irq
entry_handler    1, h, 64, fiq
entry_handler    1, h, 64, error

    .macro entry_handler el:req, ht:req, regsize:req, label:req
SYM_CODE_START_LOCAL(el\el\ht\()_\regsize\()_\label)
    /* 1. save context */
    kernel_entry \el, \regsize
    mov    x0, sp /* sp passed as arg to el_ht_handler */
    /* 2. call handler */
    bl    el\el\ht\()_\regsize\()_\label\()_handler
    /* 3. return user/kernel space */
    .if \el == 0
    b    ret_to_user
    .else
    b    ret_to_kernel
    .endif
SYM_CODE_END(el\el\ht\()_\regsize\()_\label)
    .endm
```

```c
.macro    kernel_entry, el, regsize = 64
    .if    \el == 0
    alternative_insn nop, SET_PSTATE_DIT(1), ARM64_HAS_DIT
    .endif

    .if    \regsize == 32
    mov    w0, w0                // zero upper 32 bits of x0
    .endif
    /* save the regs context of current task on the top of stack */
    stp    x0, x1, [sp, #16 * 0]
    stp    x2, x3, [sp, #16 * 1]
    stp    x4, x5, [sp, #16 * 2]
    stp    x6, x7, [sp, #16 * 3]
    stp    x8, x9, [sp, #16 * 4]
    stp    x10, x11, [sp, #16 * 5]
    stp    x12, x13, [sp, #16 * 6]
    stp    x14, x15, [sp, #16 * 7]
    stp    x16, x17, [sp, #16 * 8]
    stp    x18, x19, [sp, #16 * 9]
    stp    x20, x21, [sp, #16 * 10]
    stp    x22, x23, [sp, #16 * 11]
    stp    x24, x25, [sp, #16 * 12]
    stp    x26, x27, [sp, #16 * 13]
    stp    x28, x29, [sp, #16 * 14]

    .if    \el == 0
    clear_gp_regs
    mrs    x21, sp_el0
    ldr_this_cpu    tsk, __entry_task, x20
    msr    sp_el0, tsk

    /* Ensure MDSCR_EL1.SS is clear, since we can unmask debug exceptions
     * when scheduling. */
    ldr    x19, [tsk, #TSK_TI_FLAGS]
    disable_step_tsk x19, x20

    /* Check for asynchronous tag check faults in user space */
    ldr    x0, [tsk, THREAD_SCTLR_USER]
    check_mte_async_tcf x22, x23, x0

#ifdef CONFIG_ARM64_PTR_AUTH
alternative_if ARM64_HAS_ADDRESS_AUTH
    /* Enable IA for in-kernel PAC if the task had it disabled. Although
     * this could be implemented with an unconditional MRS which would avoid
     * a load, this was measured to be slower on Cortex-A75 and Cortex-A76.
     *
     * Install the kernel IA key only if IA was enabled in the task. If IA
     * was disabled on kernel exit then we would have left the kernel IA
     * installed so there is no need to install it again. */
    tbz    x0, SCTLR_ELx_ENIA_SHIFT, 1f
    __ptrauth_keys_install_kernel_nosync tsk, x20, x22, x23
    b    2f
1:
    mrs    x0, sctlr_el1
    orr    x0, x0, SCTLR_ELx_ENIA
    msr    sctlr_el1, x0
2:
alternative_else_nop_endif
#endif

    apply_ssbd 1, x22, x23

    mte_set_kernel_gcr x22, x23

    /* Any non-self-synchronizing system register updates required for
     * kernel entry should be placed before this point. */
alternative_if ARM64_MTE
    isb
    b    1f
alternative_else_nop_endif
alternative_if ARM64_HAS_ADDRESS_AUTH
    isb
alternative_else_nop_endif
1:

    scs_load_current
    .else     /* \el != 0 */
    add    x21, sp, #PT_REGS_SIZE
    get_current_task tsk
    .endif /* \el == 0 */

    mrs    x22, elr_el1
    mrs    x23, spsr_el1
    stp    lr, x21, [sp, #S_LR]

    /*
     * For exceptions from EL0, create a final frame record.
     * For exceptions from EL1, create a synthetic frame record so the
     * interrupted code shows up in the backtrace.
     */
    .if \el == 0
        stp    xzr, xzr, [sp, #S_STACKFRAME]
    .else
        stp    x29, x22, [sp, #S_STACKFRAME]
    .endif

    add    x29, sp, #S_STACKFRAME

#ifdef CONFIG_ARM64_SW_TTBR0_PAN
alternative_if_not ARM64_HAS_PAN
    bl    __swpan_entry_el\el
alternative_else_nop_endif
#endif

    stp    x22, x23, [sp, #S_PC]

    /* Not in a syscall by default (el0_svc overwrites for real syscall) */
    .if    \el == 0
    mov    w21, #NO_SYSCALL
    str    w21, [sp, #S_SYSCALLNO]
    .endif

#ifdef CONFIG_ARM64_PSEUDO_NMI
alternative_if_not ARM64_HAS_GIC_PRIO_MASKING
    b    .Lskip_pmr_save\@
alternative_else_nop_endif

    mrs_s    x20, SYS_ICC_PMR_EL1
    str    x20, [sp, #S_PMR_SAVE]
    mov    x20, #GIC_PRIO_IRQON | GIC_PRIO_PSR_I_SET
    msr_s    SYS_ICC_PMR_EL1, x20

.Lskip_pmr_save\@:
#endif

    /* Registers that may be useful after this macro is invoked:
     *
     * x20 - ICC_PMR_EL1
     * x21 - aborted SP
     * x22 - aborted PC
     * x23 - aborted PSTATE */
    .endm
```

## ret_to_kernel

```c
SYM_CODE_START_LOCAL(ret_to_kernel)
    kernel_exit 1
SYM_CODE_END(ret_to_kernel)


    .macro    kernel_exit, el
    .if    \el != 0
        disable_daif
    .endif

#ifdef CONFIG_ARM64_PSEUDO_NMI
alternative_if_not ARM64_HAS_GIC_PRIO_MASKING
    b    .Lskip_pmr_restore\@
alternative_else_nop_endif

    ldr    x20, [sp, #S_PMR_SAVE]
    msr_s    SYS_ICC_PMR_EL1, x20

    /* Ensure priority change is seen by redistributor */
alternative_if_not ARM64_HAS_GIC_PRIO_RELAXED_SYNC
    dsb    sy
alternative_else_nop_endif

.Lskip_pmr_restore\@:
#endif

    ldp    x21, x22, [sp, #S_PC]        // load ELR, SPSR

#ifdef CONFIG_ARM64_SW_TTBR0_PAN
alternative_if_not ARM64_HAS_PAN
    bl    __swpan_exit_el\el
alternative_else_nop_endif
#endif

    .if    \el == 0
    ldr    x23, [sp, #S_SP]        // load return stack pointer
    msr    sp_el0, x23
    tst    x22, #PSR_MODE32_BIT        // native task?
    b.eq    3f

#ifdef CONFIG_ARM64_ERRATUM_845719
alternative_if ARM64_WORKAROUND_845719
#ifdef CONFIG_PID_IN_CONTEXTIDR
    mrs    x29, contextidr_el1
    msr    contextidr_el1, x29
#else
    msr contextidr_el1, xzr
#endif
alternative_else_nop_endif
#endif
3:
    scs_save tsk

    /* Ignore asynchronous tag check faults in the uaccess routines */
    ldr    x0, [tsk, THREAD_SCTLR_USER]
    clear_mte_async_tcf x0

#ifdef CONFIG_ARM64_PTR_AUTH
alternative_if ARM64_HAS_ADDRESS_AUTH
    /*
     * IA was enabled for in-kernel PAC. Disable it now if needed, or
     * alternatively install the user's IA. All other per-task keys and
     * SCTLR bits were updated on task switch.
     *
     * No kernel C function calls after this.
     */
    tbz    x0, SCTLR_ELx_ENIA_SHIFT, 1f
    __ptrauth_keys_install_user tsk, x0, x1, x2
    b    2f
1:
    mrs    x0, sctlr_el1
    bic    x0, x0, SCTLR_ELx_ENIA
    msr    sctlr_el1, x0
2:
alternative_else_nop_endif
#endif

    mte_set_user_gcr tsk, x0, x1

    apply_ssbd 0, x0, x1
    .endif

    /* restore context */
    msr    elr_el1, x21            // set up the return data
    msr    spsr_el1, x22
    ldp    x0, x1, [sp, #16 * 0]
    ldp    x2, x3, [sp, #16 * 1]
    ldp    x4, x5, [sp, #16 * 2]
    ldp    x6, x7, [sp, #16 * 3]
    ldp    x8, x9, [sp, #16 * 4]
    ldp    x10, x11, [sp, #16 * 5]
    ldp    x12, x13, [sp, #16 * 6]
    ldp    x14, x15, [sp, #16 * 7]
    ldp    x16, x17, [sp, #16 * 8]
    ldp    x18, x19, [sp, #16 * 9]
    ldp    x20, x21, [sp, #16 * 10]
    ldp    x22, x23, [sp, #16 * 11]
    ldp    x24, x25, [sp, #16 * 12]
    ldp    x26, x27, [sp, #16 * 13]
    ldp    x28, x29, [sp, #16 * 14]

    .if    \el == 0
alternative_if_not ARM64_UNMAP_KERNEL_AT_EL0
    ldr    lr, [sp, #S_LR]
    add    sp, sp, #PT_REGS_SIZE        // restore sp

    eret

alternative_else_nop_endif
#ifdef CONFIG_UNMAP_KERNEL_AT_EL0
    msr    far_el1, x29

    ldr_this_cpu    x30, this_cpu_vector, x29
    tramp_alias    x29, tramp_exit
    msr        vbar_el1, x30        // install vector table
    ldr        lr, [sp, #S_LR]        // restore x30
    add        sp, sp, #PT_REGS_SIZE    // restore sp
    br        x29
#endif
    .else /* \el == 0 */
    ldr    lr, [sp, #S_LR]
    add    sp, sp, #PT_REGS_SIZE        // restore sp

    /* Ensure any device/NC reads complete */
    alternative_insn nop, "dmb sy", ARM64_WORKAROUND_1508412

    eret
    .endif

    sb
    .endm
```

## ret_to_user
```c
SYM_CODE_START_LOCAL(ret_to_user)
    ldr    x19, [tsk, #TSK_TI_FLAGS]    // re-check for single-step
    enable_step_tsk x19, x2
#ifdef CONFIG_GCC_PLUGIN_STACKLEAK
    bl    stackleak_erase_on_task_stack
#endif
    kernel_exit 0
SYM_CODE_END(ret_to_user)
```

```c
/* arch/arm64/include/asm/exception.h */
asmlinkage void el1h_64_sync_handler(struct pt_regs *regs);
asmlinkage void el1h_64_irq_handler(struct pt_regs *regs);
asmlinkage void el1h_64_fiq_handler(struct pt_regs *regs);
asmlinkage void el1h_64_error_handler(struct pt_regs *regs);

asmlinkage void el0t_64_sync_handler(struct pt_regs *regs);
asmlinkage void el0t_64_irq_handler(struct pt_regs *regs);
asmlinkage void el0t_64_fiq_handler(struct pt_regs *regs);
asmlinkage void el0t_64_error_handler(struct pt_regs *regs);
```

# 64_irq_handler

```c
el0t_64_irq_handler(struct pt_regs *regs) {
    el0_interrupt(regs, handle_arch_irq) {
        enter_from_user_mode(regs);

        write_sysreg(DAIF_PROCCTX_NOIRQ, daif);

        if (regs->pc & BIT(55))
            arm64_apply_bp_hardening();

        irq_enter_rcu() {
            __irq_enter_raw() {
                preempt_count_add(HARDIRQ_OFFSET);
                lockdep_hardirq_enter();
            }

            if (tick_nohz_full_cpu(smp_processor_id())
                || (is_idle_task(current) && (irq_count() == HARDIRQ_OFFSET))) {
                tick_irq_enter();
            }

            account_hardirq_enter(current) {
                vtime_account_irq(tsk, HARDIRQ_OFFSET);
                irqtime_account_irq(tsk, HARDIRQ_OFFSET) {
                    struct irqtime *irqtime = this_cpu_ptr(&cpu_irqtime);
                    unsigned int pc;
                    s64 delta;
                    int cpu;

                    if (!sched_clock_irqtime)
                        return;

                    cpu = smp_processor_id();
                    delta = sched_clock_cpu(cpu) - irqtime->irq_start_time;
                    irqtime->irq_start_time += delta;
                    pc = irq_count() - offset;

                    if (pc & HARDIRQ_MASK)
                        irqtime_account_delta(irqtime, delta, CPUTIME_IRQ);
                    else if ((pc & SOFTIRQ_OFFSET) && curr != this_cpu_ksoftirqd())
                        irqtime_account_delta(irqtime, delta, CPUTIME_SOFTIRQ);
                }
            }
        }

        do_interrupt_handler(regs, handler) {
            struct pt_regs *old_regs = set_irq_regs(regs);

            if (on_thread_stack()) {
                call_on_irq_stack(regs, handler);
            } else {
                handler(regs); /* gic_handle_irq */
            }

            set_irq_regs(old_regs);
        }

        irq_exit_rcu() {
            account_hardirq_exit(current) {
                vtime_account_hardirq(tsk);
                irqtime_account_irq(tsk, 0);
            }
            preempt_count_sub(HARDIRQ_OFFSET);
            if (!in_interrupt() && local_softirq_pending()) {
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
                        __do_softirq();
                    } else {
                        wakeup_softirqd();
                    }
                }
            }

            tick_irq_exit();
        }

        exit_to_user_mode(regs) {
            exit_to_user_mode_prepare(regs) {
                flags = read_thread_flags();
                if (unlikely(flags & _TIF_WORK_MASK)) {
                    do_notify_resume(regs, flags) {
                        do {
                            if (thread_flags & _TIF_NEED_RESCHED) {
                                local_daif_restore(DAIF_PROCCTX_NOIRQ);
                                schedule();
                            } else {
                                local_daif_restore(DAIF_PROCCTX);

                                if (thread_flags & _TIF_UPROBE)
                                    uprobe_notify_resume(regs);

                                if (thread_flags & _TIF_MTE_ASYNC_FAULT) {
                                    clear_thread_flag(TIF_MTE_ASYNC_FAULT);
                                    send_sig_fault(SIGSEGV, SEGV_MTEAERR, (void __user *)NULL, current);
                                }

                                if (thread_flags & (_TIF_SIGPENDING | _TIF_NOTIFY_SIGNAL))
                                    do_signal(regs);

                                if (thread_flags & _TIF_NOTIFY_RESUME) {
                                    resume_user_mode_work(regs);
                                }

                                if (thread_flags & _TIF_FOREIGN_FPSTATE)
                                    fpsimd_restore_current_state();
                            }

                            local_daif_mask();
                            thread_flags = read_thread_flags();
                        } while (thread_flags & _TIF_WORK_MASK);
                    }
                }
            }
            mte_check_tfsr_exit();
            __exit_to_user_mode() {
                trace_hardirqs_on_prepare();
                lockdep_hardirqs_on_prepare();
                user_enter_irqoff();
                lockdep_hardirqs_on(CALLER_ADDR0);
            }
        }
    }
}
```

```c
el1h_64_irq_handler(struct pt_regs *regs) {
    el1_interrupt(regs, handle_arch_irq) {
        write_sysreg(DAIF_PROCCTX_NOIRQ, daif);

        if (IS_ENABLED(CONFIG_ARM64_PSEUDO_NMI) && !interrupts_enabled(regs)) {
            __el1_pnmi(regs, handler);
        } else {
            __el1_irq(regs, handler) {
                enter_from_kernel_mode(regs);

                irq_enter_rcu();
                do_interrupt_handler(regs, handler);
                irq_exit_rcu();

                arm64_preempt_schedule_irq() {
                    if (!need_irq_preemption())
                        return;

                    if (READ_ONCE(current_thread_info()->preempt_count) != 0)
                        return;

                    if (system_uses_irq_prio_masking() && read_sysreg(daif))
                        return;

                    if (system_capabilities_finalized()) {
                        preempt_schedule_irq() {
                            do {
                                preempt_disable();
                                local_irq_enable();
                                __schedule(SM_PREEMPT);
                                local_irq_disable();
                                sched_preempt_enable_no_resched();
                            } while (need_resched());
                        }
                    }
                }

                exit_to_kernel_mode(regs) {
                    if (interrupts_enabled(regs)) {
                        if (regs->exit_rcu) {
                            trace_hardirqs_on_prepare();
                            lockdep_hardirqs_on_prepare();
                            ct_irq_exit();
                            lockdep_hardirqs_on(CALLER_ADDR0);
                            return;
                        }

                        trace_hardirqs_on();
                    } else {
                        if (regs->exit_rcu)
                            ct_irq_exit();
                    }
                }
            }
        }
    }
}
```

## gic_handle_irq

```c
set_handle_irq(gic_handle_irq);

void (*handle_arch_irq)(struct pt_regs *) __ro_after_init = default_handle_irq;
void (*handle_arch_fiq)(struct pt_regs *) __ro_after_init = default_handle_fiq;

int __init set_handle_irq(void (*handle_irq)(struct pt_regs *))
{
    if (handle_arch_irq != default_handle_irq)
        return -EBUSY;

    handle_arch_irq = handle_irq;
    pr_info("Root IRQ handler: %ps\n", handle_irq);
    return 0;
}

/* el0_irq -> irq_handler */
static void __exception_irq_entry gic_handle_irq(struct pt_regs *regs)
{
	if (unlikely(gic_supports_nmi() && !interrupts_enabled(regs))) {
		__gic_handle_irq_from_irqsoff(regs);

    } else {
        __gic_handle_irq_from_irqson(regs) {
            bool is_nmi;
            u32 irqnr;

            irqnr = gic_read_iar();

            is_nmi = gic_rpr_is_nmi_prio();

            if (is_nmi) {
                nmi_enter();
                __gic_handle_nmi(irqnr, regs);
                nmi_exit();
            }

            if (gic_prio_masking_enabled()) {
                gic_pmr_mask_irqs();
                gic_arch_enable_irqs();
            }

            if (!is_nmi) {
                __gic_handle_irq(irqnr, regs) {
                    if (gic_irqnr_is_special(irqnr))
                        return;

                    gic_complete_ack(irqnr) {
                        if (static_branch_likely(&supports_deactivate_key)) {
                            write_gicreg(irqnr, ICC_EOIR1_EL1);
                        }

                        isb();
                    }

                    ret = generic_handle_domain_irq(gic_data.domain, irqnr);
                    if (ret) {
                        WARN_ONCE(true, "Unexpected interrupt (irqnr %u)\n", irqnr);
                        gic_deactivate_unhandled(irqnr);
                    }
                }
            }
        }
    }
}
```

# 64_sync_handler

```c
/* sync exception using thread stack */
void el0t_64_sync_handler(struct pt_regs *regs) {
    unsigned long esr = read_sysreg(esr_el1);

    switch (ESR_ELx_EC(esr)) {
    case ESR_ELx_EC_SVC64:
        el0_svc(regs) {
            enter_from_user_mode(regs) {
                lockdep_hardirqs_off(CALLER_ADDR0);
                CT_WARN_ON(ct_state() != CONTEXT_USER);
                user_exit_irqoff();
                trace_hardirqs_off_finish();
                mte_disable_tco_entry(current);
            }
            cortex_a76_erratum_1463225_svc_handler();

            do_el0_svc(regs) {
                fp_user_discard();
                el0_svc_common(regs, regs->regs[8], __NR_syscalls, sys_call_table) {
                    unsigned long flags = read_thread_flags();

                    regs->orig_x0 = regs->regs[0];
                    regs->syscallno = scno;

                    if (flags & _TIF_MTE_ASYNC_FAULT) {
                        syscall_set_return_value(current, regs, -ERESTARTNOINTR, 0);
                        return;
                    }

                    invoke_syscall(regs, scno, sc_nr, syscall_table) {
                        if (scno < sc_nr) {
                            syscall_fn_t syscall_fn;
                            syscall_fn = syscall_table[array_index_nospec(scno, sc_nr)];
                            ret = __invoke_syscall(regs, syscall_fn) {
                                return syscall_fn(regs);
                            }
                        } else {
                            ret = do_ni_syscall(regs, scno);
                        }
                    }
                }
            }

            exit_to_user_mode(regs)
                --->
        }
        break;
    case ESR_ELx_EC_DABT_LOW:
        el0_da(regs, esr);
        break;
    case ESR_ELx_EC_IABT_LOW:
        el0_ia(regs, esr);
        break;
    case ESR_ELx_EC_FP_ASIMD:
        el0_fpsimd_acc(regs, esr);
        break;
    case ESR_ELx_EC_SVE:
        el0_sve_acc(regs, esr);
        break;
    case ESR_ELx_EC_SME:
        el0_sme_acc(regs, esr);
        break;
    case ESR_ELx_EC_FP_EXC64:
        el0_fpsimd_exc(regs, esr);
        break;
    case ESR_ELx_EC_SYS64:
    case ESR_ELx_EC_WFx:
        el0_sys(regs, esr);
        break;
    case ESR_ELx_EC_SP_ALIGN:
        el0_sp(regs, esr);
        break;
    case ESR_ELx_EC_PC_ALIGN:
        el0_pc(regs, esr);
        break;
    case ESR_ELx_EC_UNKNOWN:
        el0_undef(regs, esr);
        break;
    case ESR_ELx_EC_BTI:
        el0_bti(regs);
        break;
    case ESR_ELx_EC_BREAKPT_LOW:
    case ESR_ELx_EC_SOFTSTP_LOW:
    case ESR_ELx_EC_WATCHPT_LOW:
    case ESR_ELx_EC_BRK64:
        el0_dbg(regs, esr);
        break;
    case ESR_ELx_EC_FPAC:
        el0_fpac(regs, esr);
        break;
    default:
        el0_inv(regs, esr);
    }
}
```

```c
/* sync exception using handler stack */
void el1h_64_sync_handler(struct pt_regs *regs) {
    unsigned long esr = read_sysreg(esr_el1);

    switch (ESR_ELx_EC(esr)) {
    case ESR_ELx_EC_DABT_CUR:
    case ESR_ELx_EC_IABT_CUR:
        el1_abort(regs, esr) {

        }
        break;
    case ESR_ELx_EC_PC_ALIGN:
        el1_pc(regs, esr) {

        }
        break;
    case ESR_ELx_EC_SYS64:
    case ESR_ELx_EC_UNKNOWN:
        el1_undef(regs, esr);
        break;
    case ESR_ELx_EC_BTI:
        el1_bti(regs, esr);
        break;
    case ESR_ELx_EC_BREAKPT_CUR:
    case ESR_ELx_EC_SOFTSTP_CUR:
    case ESR_ELx_EC_WATCHPT_CUR:
    case ESR_ELx_EC_BRK64:
        el1_dbg(regs, esr);
        break;
    case ESR_ELx_EC_FPAC:
        el1_fpac(regs, esr);
        break;
    default:
        __panic_unhandled(regs, "64-bit el1h sync", esr);
    }
}
```

# gicv3

## gic_of_init

```c
int __init
gic_of_init(struct device_node *node, struct device_node *parent)
{
    struct gic_chip_data *gic;
    int irq, ret;

    gic = &gic_data[gic_cnt];

    ret = gic_of_setup(gic, node);

    if (gic_cnt == 0 && !gic_check_eoimode(node, &gic->raw_cpu_base))
        static_branch_disable(&supports_deactivate_key);

        ret = __gic_init_bases(gic, &node->fwnode) {
            f (gic == &gic_data[0]) {

            for (i = 0; i < NR_GIC_CPU_IF; i++)
                gic_cpu_map[i] = 0xff;

            set_handle_irq(gic_handle_irq);
        }

        ret = gic_init_bases(gic, handle) {

        }
        if (gic == &gic_data[0]) {
            gic_smp_init();
        }
    }


    if (!gic_cnt) {
        gic_init_physaddr(node);
        gic_of_setup_kvm_info(node);
    }

    if (parent) {
        irq = irq_of_parse_and_map(node, 0);
        gic_cascade_irq(gic_cnt, irq);
    }

    if (IS_ENABLED(CONFIG_ARM_GIC_V2M))
        gicv2m_init(&node->fwnode, gic_data[gic_cnt].domain);

    gic_cnt++;
    return 0;
}

```
```c
set_smp_ipi_range(int ipi_base, int n) {
    int i;

    nr_ipi = min(n, MAX_IPI);

    for (i = 0; i < nr_ipi; i++) {
        int err;

        err = request_percpu_irq(ipi_base + i, ipi_handler,
                     "IPI", &irq_stat);
        WARN_ON(err);

        ipi_desc[i] = irq_to_desc(ipi_base + i);
        irq_set_status_flags(ipi_base + i, IRQ_HIDDEN);
    }

    ipi_irq_base = ipi_base;

    /* Setup the boot CPU immediately */
    ipi_setup(smp_processor_id());
}
```

```c
irqreturn_t ipi_handler(int irq, void *data) {
    do_handle_IPI(irq - ipi_irq_base) {
        unsigned int cpu = smp_processor_id();

        if ((unsigned)ipinr < NR_IPI)
            trace_ipi_entry(ipi_types[ipinr]);

        switch (ipinr) {
        case IPI_RESCHEDULE:
            scheduler_ipi() {
                #define preempt_fold_need_resched() \
                do { \
                    if (tif_need_resched()) \
                        set_preempt_need_resched(); \
                } while (0)
            }
            break;

        case IPI_CALL_FUNC:
            generic_smp_call_function_interrupt();
            break;

        case IPI_CPU_STOP:
            local_cpu_stop();
            break;

        case IPI_CPU_CRASH_STOP:
            if (IS_ENABLED(CONFIG_KEXEC_CORE)) {
                ipi_cpu_crash_stop(cpu, get_irq_regs());

                unreachable();
            }
            break;

    #ifdef CONFIG_GENERIC_CLOCKEVENTS_BROADCAST
        case IPI_TIMER:
            tick_receive_broadcast();
            break;
    #endif

    #ifdef CONFIG_IRQ_WORK
        case IPI_IRQ_WORK:
            irq_work_run();
            break;
    #endif

        default:
            pr_crit("CPU%u: Unknown IPI message 0x%x\n", cpu, ipinr);
            break;
        }

        if ((unsigned)ipinr < NR_IPI)
            trace_ipi_exit(ipi_types[ipinr]);
    }
    return IRQ_HANDLED;
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
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
    .endif

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
    b    el\el\ht\()_\regsize\()_\label
.org .Lventry_start\@ + 128    // Did we overflow the ventry slot?
    .endm
```

```c
/* arch/arm64/include/asm/exception.h */
asmlinkage void el1t_64_sync_handler(struct pt_regs *regs);
asmlinkage void el1t_64_irq_handler(struct pt_regs *regs);
asmlinkage void el1t_64_fiq_handler(struct pt_regs *regs);
asmlinkage void el1t_64_error_handler(struct pt_regs *regs);
```

## 64_irq_handler
```c
el0t_64_irq_handler() {
    el0_interrupt(regs, handle_arch_irq) {
        enter_from_user_mode(regs);

        write_sysreg(DAIF_PROCCTX_NOIRQ, daif);

        if (regs->pc & BIT(55))
            arm64_apply_bp_hardening();

        irq_enter_rcu();
        do_interrupt_handler(regs, handler) {

        }
        irq_exit_rcu();

        exit_to_user_mode(regs) {
            exit_to_user_mode_prepare(regs) {
                flags = read_thread_flags();
                if (unlikely(flags & _TIF_WORK_MASK)) {
                    do_notify_resume(regs, flags) {
                        do {
                            if (thread_flags & _TIF_NEED_RESCHED) {
                                /* Unmask Debug and SError for the next task */
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
            __exit_to_user_mode();
        }
    }
}
```

```c
el1t_64_irq_handler() {
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

## 64_sync_handler
```c
void noinstr el0t_64_sync_handler(struct pt_regs *regs) {
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

            exit_to_user_mode(regs) {
                prepare_exit_to_user_mode(regs) {
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

                                    if (thread_flags & _TIF_NOTIFY_RESUME)
                                        resume_user_mode_work(regs);

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
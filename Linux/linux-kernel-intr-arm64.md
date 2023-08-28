* [vector](#vectors)
* [64_irq_handler](#64_irq_handler)
* [64_sync_handler](#64_sync_handler)

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

## entry_handler
```c
entry_handler    1, h, 64, sync
entry_handler    1, h, 64, irq
entry_handler    1, h, 64, fiq
entry_handler    1, h, 64, error

    .macro entry_handler el:req, ht:req, regsize:req, label:req
SYM_CODE_START_LOCAL(el\el\ht\()_\regsize\()_\label)
    kernel_entry \el, \regsize
    mov    x0, sp
    bl    el\el\ht\()_\regsize\()_\label\()_handler
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
    /*
     * Enable IA for in-kernel PAC if the task had it disabled. Although
     * this could be implemented with an unconditional MRS which would avoid
     * a load, this was measured to be slower on Cortex-A75 and Cortex-A76.
     *
     * Install the kernel IA key only if IA was enabled in the task. If IA
     * was disabled on kernel exit then we would have left the kernel IA
     * installed so there is no need to install it again.
     */
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

    /*
     * Registers that may be useful after this macro is invoked:
     *
     * x20 - ICC_PMR_EL1
     * x21 - aborted SP
     * x22 - aborted PC
     * x23 - aborted PSTATE
    */
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
asmlinkage void el1t_64_sync_handler(struct pt_regs *regs);
asmlinkage void el1t_64_irq_handler(struct pt_regs *regs);
asmlinkage void el1t_64_fiq_handler(struct pt_regs *regs);
asmlinkage void el1t_64_error_handler(struct pt_regs *regs);
```

# 64_irq_handler
```c
el0t_64_irq_handler(struct pt_regs *regs) {
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

# 64_sync_handler
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
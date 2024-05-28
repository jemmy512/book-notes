# Table of Contents

<details>
<summary>Open * Close</summary>

* [CPU](#cpu)
* [bios](#bios)
* [start_kernel](#start_kernel)
    * [smp_boot](#smp_boot)
* [syscall](#syscall)
    * [glibs](#glibc)
    * [64](#64)

* [process](#process)
* [thread](#thread)
* [task_struct](#task_struct)
* [sched](#sched)
    * [voluntary schedule](#voluntary-schedule)
    * [preempt schedule](#preempt-schedule)
        * [user preempt](#user-preempt)
            * [set_tsk_need_resched](#set_tsk_need_resched)
                * [scheduler_tick](#scheduler_tick)
                * [try_to_wake_up](#try_to_wake_upp)
                * [sched_setscheduler](#sched_setscheduler)
            * [prempt time](#preempt-time)
                * [return from system call](#return-from-system-call)
                * [return from interrupt](#return-from-interrupt)
        * [kernel preempt](#kernel-preempt)
            * [preempt_enable](#preempt_enble)
            * [preempt_schedule_irq](#preempt_schedule_irq)

* [SCHED_RT](#SCHED_RR)
    * [enqueue_task_rt](#enqueue_task_rt)
    * [dequeue_task_rt](#dequeue_task_rt)
    * [put_prev_task_rt](#put_prev_task_rt)
    * [pick_next_task_rt](#pick_next_task_rt)
    * [set_next_task_rt](#set_next_task_rt)
    * [select_task_rq_rt](#select_task_rq_rt)
    * [wakeup_preempt_rt](#wakeup_preempt_rt)
    * [task_tick_rt](#task_tick_rt)
    * [yield_task_rt](#yield_task_rt)
    * [prio_changed_rt](#prio_changed_rt)
    * [check_class_changed_rt](#check_class_changed_rt)
        * [switched_from_rt](#switched_from_rt)
        * [switched_to_rt](#switched_to_rq)
    * [balance_rt](#balance_rt)
        * [push_rt_task](#push_rt_task)
        * [pull_rt_task](#pull_rt_task)
* [SCHED_CFS](#SCHED_CFS)
    * [enqueue_task_fair](#enqueue_task_fair)
    * [dequeue_task_fair](#dequeue_task_fair)
    * [put_prev_task_fair](#put_prev_task_fair)
    * [pick_next_task_fair](#pick_next_task_fair)
    * [set_next_task_fair](#set_next_task_fair)
    * [select_task_rq_fair](#select_task_rq_fair)
        * [find_idlest_cpu](#find_idlest_cpu)
            * [find_idlest_group](#find_idlest_group)
            * [find_idlest_group_cpu](#find_idlest_group_cpu)
        * [select_idle_sibling](#select_idle_sibling)
    * [wakeup_preempt_fair](#wakeup_preempt_fair)
    * [task_tick_fair](#task_tick_fair)
    * [task_fork_fair](#task_fork_fair)
    * [yield_task_fair](#yield_task_fair)
    * [prio_changed_fair](#prio_changed_fair)
    * [check_class_changed_fair](#check_class_changed_fair)
        * [switched_from_fair](#switched_from_fair)
        * [switched_to_fair](#switched_to_fair)
    * [balance_fair](#balance_fair)
    * [sched_vslice](#sched_vslice)

* [sched_domain](#sched_domain)
* [cpu capacity](#cpu_capacity)
    * [parse_dt_topology](#parse_dt_topology)
    * [parse_socket](#parse_socket)
    * [parse_cluster](#parse_cluster)
    * [parse_core](#parse_core)

* [PELT](#PELT)
    * [update_load_sum_avg](#update_load_sum_avg)
    * [update_load_avg](#update_load_avg)
        * [update_cfs_rq_load_avg](#update_cfs_rq_load_avg)
    * [rq_clock](#rq_clock)

* [load_balance](#load_balance)
    * [tick_balance](#tick_balance)
    * [nohz_idle_balance](#nohz_idle_balance)
    * [sched_balance_newidle](#sched_balance_newidle)
    * [sched_balance_rq](#sched_balance_rq)
        * [should_we_balance](#should_we_balance)
        * [sched_balance_find_src_group](#sched_balance_find_src_group)
            * [update_sd_lb_stats](#update_sd_lb_stats)
            * [update_sd_pick_busiest](#update_sd_pick_busiest)
            * [calculate_imbalance](#calculate_imbalance)
        * [sched_balance_find_src_rq](#sched_balance_find_src_rq)
        * [detach_tasks](#detach_tasks)
            * [can_migrate_task](#can_migrate_task)

* [wake_up](#wake_up)
* [wait_woken](#wait_woken)
* [try_to_wake_up](#try_to_wake_up)

* [fork](#fork)

* [wait4](#wait4)
    * [wait_consider_task](#wait_consider_task)
    * [wait_task_zombie](#wait_task_zombie)

* [exec](#exec)
    * [load_elf_binary](#load_elf_binary)

* [exit](#exit)
    * [exit_signals](#exit_signals)
    * [exit_notify](#exit_notify)

* [kthreadd](#kthreadd)

* [cmwq](#cmwq)
    * [wq-struct](#wq-struct)

* [task_group](#task_group)
    * [sched_create_group](#sched_create_group)

* [cfs_bandwidth](#cfs_bandwidth)
    * [init_cfs_bandwidth](#init_cfs_bandwidth)
    * [sched_cfs_period_timer](#sched_cfs_period_timer)
    * [sched_cfs_slack_timer](#sched_cfs_slack_timer)
    * [tg_set_cfs_bandwidth](#tg_set_cfs_bandwidth)
    * [throttle_cfs_rq](#throttle_cfs_rq)
    * [unthrottle_cfs_rq](#unthrottle_cfs_rq)
    * [sched_group_set_shares](#sched_group_set_shares)
    * [update_cfs_group](#update_cfs_group)

* [rt_bandwidth](#rt_bandwidth)
    * [tg_set_rt_bandwidth](#tg_set_rt_bandwidth)
    * [sched_rt_period_timer](#sched_rt_period_timer)

* [cgroup](#cgroup)
    * [cgrp_demo](#cgrp_demo)
    * [cgrou_init](#cgroup_init)
        * [cgroup_init_cftypes](#cgroup_init_cftypes)
        * [cgroup_init_subsys](#cgroup_init_subsys)
        * [cgroup_setup_root](#cgroup_setup_root)
    * [cgroup_create](#cgroup_create)
    * [cgroup_attach_task](#cgroup_attach_task)
    * [cgroup_fork](#cgroup_fork)
    * [mem_cgroup](#mem_cgroup)
        * [mem_cgroup_write](#mem_cgroup_write)
        * [mem_cgroup_charge](#mem_cgroup_charge)
        * [mem_cgroup_can_attach](#mem_cgroup_can_attach)
        * [mem_cgroup_post_attach](#mem_cgroup_post_attach)
    * [cpu_cgroup](#cpu_cgroup)
        * [cpu_cgroup_css_alloc](#cpu_cgroup_css_alloc)
        * [cpu_cgroup_attach](#cpu_cgroup_attach)
        * [cpu_weight_write_u64](#cpu_weight_write_u64)
        * [cpu_max_write](#cpu_max_write)
    * [cgroup_fork](#cgroup_fork)
    * [cgroup_subtree_control_write](#cgroup_subtree_control_write)

</details>

* [LWN Kernel Index](https://lwn.net/Kernel/Index/)

<img src='../images/kernel/kernel-structual.svg' style='max-height:850px'/>


# cpu
<img src='../images/kernel/init-cpu.png' style='max-height:850px'/>

<img src='../images/kernel/init-cpu-2.png' style='max-height:850px'/>

<img src='../images/kernel/init-cpu-process-program.png' style='max-height:850px'/>

# bios
* ![](../images/kernel/init-bios.png)
* When power on, set CS to 0xFFFF, IP to 0x0000, the first instruction points to 0xFFFF0 within ROM, a JMP comamand will jump to ROM do init work, BIOS starts.
* Then BIOS checks the health state of each hardware.
* Grub2 (Grand Unified Bootloader Version 2)
  * grub2-mkconfig -o /boot/grub2/grub.cfg
    ```
    menuentry 'CentOS Linux (3.10.0-862.el7.x86_64) 7 (Core)' --class centos --class gnu-linux --class gnu --class os --unrestricted $menuentry_id_option 'gnulinux-3.10.0-862.el7.x86_64-advanced-b1aceb95-6b9e-464a-a589-bed66220ebee' {
      load_video
      set gfxpayload=keep
      insmod gzio
      insmod part_msdos
      insmod ext2 set root='hd0,msdos1'
      if [ x$feature_platform_search_hint = xy ]; then
        search --no-floppy --fs-uuid --set=root --hint='hd0,msdos1' b1aceb95-6b9e-464a-a589-bed66220ebee
      else search --no-floppy --fs-uuid --set=root b1aceb95-6b9e-464a-a589-bed66220ebee
      fi

      linux16 /boot/vmlinuz-3.10.0-862.el7.x86_64 root=UUID=b1aceb95-6b9e-464a-a589-bed66220ebee ro console=tty0 console=ttyS0,115200 crashkernel=auto net.ifnames=0 biosdevname=0 rhgb quiet
      initrd16 /boot/initramfs-3.10.0-862.el7.x86_64.img
    }
    ```
  * grub2-install /dev/sda
    * install boot.img into MBR(Master Boot Record), and load boot.img into memory at 0x7c00 to run
    * core.img: diskboot.img, lzma_decompress.img, kernel.img

*   ```c
    boot.img                    /* Power On Self Test */
    core.img
        diskboot.img            /* diskboot.S load other modules of grub into memory */
            lzma_decompress.img /* startup_raw.S */
                real_to_prot    /* enable segement, page, open Gate A20 */
                kernel.img      /* startup.S, grub's kernel img not Linux kernel */
                    grub_main   /* grub's main func */
                        grub_load_config()
                        grub_command_execute ("normal", 0, 0)
                            grub_normal_execute()
                                grub_show_menu() /* show which OS want to run */
                                    grub_menu_execute_entry() /* start linux kernel */
    ```
    * boot.img
        * checks the basic operability of the hardware and then it issues a BIOS interrupt, INT 13H, which locates the boot sectors on any attached bootable devices.
        * read the first sector of the core image from a local disk and jump to it. Because of the size restriction, boot.img cannot understand any file system structure, so grub-install hardcodes the location of the first sector of the core image into boot.img when installing GRUB.
    * diskboot.img
        * the first sector of the core image when booting from a hard disk. It reads the rest of the core image into memory and starts the kernel. Since file system handling is not yet available, it encodes the location of the core image using a block list format.
    * kernel.img
        * contains GRUB’s basic run-time facilities: frameworks for device and file handling, environment variables, the rescue mode command-line parser, and so on. It is rarely used directly, but is built into all core images.
    * core.img
        * built dynamically from the kernel image and an arbitrary list of modules by the grub-mkimage program. Usually, it contains enough modules to access /boot/grub, and loads everything else (including menu handling, the ability to load target operating systems, and so on) from the file system at run-time. The modular design allows the core image to be kept small, since the areas of disk where it must be installed are often as small as 32KB.

* [GNU GRUB Manual 2.06](https://www.gnu.org/software/grub/manual/grub/html_node/index.html#SEC_Contents)
* [GNU GRUB Manual 2.06: Images](https://www.gnu.org/software/grub/manual/grub/html_node/Images.html)

```c
/* arch/arm64/kernel/head.S */
 * Kernel startup entry point.
 * ---------------------------
 *
 * The requirements are:
 *   MMU = off, D-cache = off, I-cache = on or off,
 *   x0 = physical address to the FDT blob. */

__HEAD
    efi_signature_nop
    b  primary_entry
    .quad  0
    le64sym  _kernel_size_le
    le64sym  _kernel_flags_le
    .quad  0
    .quad  0
    .quad  0
    .ascii  ARM64_IMAGE_MAGIC
    .long  .Lpe_header_offset

    __EFI_PE_HEADER

    .section ".idmap.text","a"

primary_entry
    bl record_mmu_state

    /* Preserve the arguments passed by the bootloader in x0 .. x3 */
    bl preserve_boot_args

    bl create_idmap

    bl __cpu_setup

    b __primary_switch
        adrp x1, reserved_pg_dir
        adrp x2, init_idmap_pg_dir
        bl __enable_mmu

        bl clear_page_tables
        bl create_kernel_mapping

        adrp x1, init_pg_dir
        load_ttbr1 x1, x1, x2 /* install x1 as a TTBR1 page table */

        x0 = __pa(KERNEL_START)

        bl __primary_switched
            adr_l x4, init_task
            init_cpu_task x4, x5, x6

            adr_l x8, vectors /* load VBAR_EL1 with virtual */
            msr vbar_el1, x8 /* vector table address */

            ldr_l x4, _text // Save the offset between
            sub   x4, x4, x0 // the kernel virtual and
            str_l x4, kimage_voffset, x5 // physical mappings

            bl set_cpu_boot_mode_flag

            bl __pi_memset

            mov x0, x21 /* pass FDT address in x0 */
            bl early_fdt_map /* Try mapping the FDT early */

            mov x0, x20 /* pass the full boot status */
            bl init_feature_override  /* Parse cpu feature overrides */

            bl start_kernel
```

# start_kernel

![](../images/kernel/ker-start.svg) /* TODO */

```c
/* init/main.c */
void start_kernel(void)
{
    /* #0 process, the only one doesn't created by fork or kernel_thread */
    set_task_stack_end_magic(&init_task);

    local_irq_disable();

    setup_arch(&command_line);

    /* set_system_intr_gate(IA32_SYSCALL_VECTOR, entry_INT80_32) */
    trap_init();

    /* mnt_init()->init_rootfs() register_filesystem(&rootfs_fs_type) */
    vfs_caches_init()
    setup_arch(&command_line)
    mm_init();
    sched_init();
    init_IRQ();
    softirq_init();
    signals_init();
    cpuset_init();
    cgroup_init();

    rest_init()
}

static void rest_init(void)
{
    struct task_struct *tsk;
    int pid;

    pid = kernel_thread(kernel_init, NULL, CLONE_FS);

    pid = kernel_thread(kthreadd, NULL, CLONE_FS | CLONE_FILES);

    complete(&kthreadd_done);

    cpu_startup_entry(CPUHP_ONLINE);
}

/* init/init_task.c */
struct task_struct init_task
#ifdef CONFIG_ARCH_TASK_STRUCT_ON_STACK
  __init_task_data
#endif
= {
#ifdef CONFIG_THREAD_INFO_IN_TASK
  .thread_info      = INIT_THREAD_INFO(init_task),
  .stack_refcount   = ATOMIC_INIT(1),
#endif
  .state    = 0,
  .stack    = init_stack,
  .usage    = ATOMIC_INIT(2),
  .flags    = PF_KTHREAD,
};
```

```c
pid_t kernel_thread(int (*fn)(void *), void *arg, unsigned long flags)
{
  return _do_fork(flags|CLONE_VM|CLONE_UNTRACED, (unsigned long)fn,
    (unsigned long)arg, NULL, NULL, 0);
}

/* return from kernel to user space */
static int kernel_init(void *unused)
{
  if (ramdisk_execute_command) {
    ret = run_init_process(ramdisk_execute_command);
    if (!ret)
      return 0;
  }

  if (execute_command) {
    ret = run_init_process(execute_command);
    if (!ret)
      return 0;
  }

  if (!try_to_run_init_process("/sbin/init") ||
      !try_to_run_init_process("/etc/init") ||
      !try_to_run_init_process("/bin/init") ||
      !try_to_run_init_process("/bin/sh"))
    return 0;
}

static int run_init_process(const char *init_filename)
{
  argv_init[0] = init_filename;
  return do_execve(getname_kernel(init_filename),
    (const char __user *const __user *)argv_init,
    (const char __user *const __user *)envp_init);
}
```

<img src='../images/kernel/init-cpu-arch.png' style='max-height:850px'/>


## smp_boot

* [ARM64 的多核启动流程分析](https://zhuanlan.zhihu.com/p/512099688?utm_id=0)
* [ARM64 SMP多核启动 spin-table](https://mp.weixin.qq.com/s/4T4WcbG5rMpHFtU8-xxTbg) [PSCI](https://mp.weixin.qq.com/s/NaEvCuSDJMQ2dsN5rJ6GqA)

```c
SYM_FUNC_START(secondary_holding_pen)
    mov     x0, xzr
    bl      init_kernel_el
    mrs     x2, mpidr_el1
    mov_q   x1, MPIDR_HWID_BITMASK
    and     x2, x2, x1
    adr_l   x3, secondary_holding_pen_release
pen:    ldr    x4, [x3]
    cmp     x4, x2
    b.eq    secondary_startup
    wfe
    b       pen
SYM_FUNC_END(secondary_holding_pen)

void start_kernel(void) {
    setup_arch(&command_line) {
/* 1. cpu_init */
        smp_init_cpus() {
            smp_cpu_setup(cpu) {
                /* Read a cpu's enable method and record it in cpu_ops. */
                init_cpu_ops(cpu) {
                    const char *enable_method = cpu_read_enable_method(cpu);

                    cpu_ops[cpu] = cpu_get_ops(enable_method); /* "spin-table" or "psci" */
                }

                ops = get_cpu_ops(cpu);
                ops->cpu_init(cpu) {
                    smp_spin_table_ops->cpu_init() {
                        smp_spin_table_cpu_init() {
                            /*  Determine the address from which the CPU is polling */
                            of_property_read_u64(dn, "cpu-release-addr", &cpu_release_addr[cpu]);
                        }
                    }

                    cpu_psci_ops->cpu_psci_cpu_init() {

                    }
                }

                set_cpu_possible(cpu, true);
            }
        }
    }

/* 2. cpu_prepare */
    arch_call_rest_init() {
        rest_init() {
            user_mode_thread(kernel_init);
            kernel_init() {
                kernel_init_freeable() {
                    smp_prepare_cpus() {
                        cpu_ops[cpu]->cpu_prepare() {
                            smp_spin_table_ops->cpu_prepare() {
                                smp_spin_table_cpu_prepare() {
                                    __le64 __iomem *release_addr;
                                    phys_addr_t pa_holding_pen = __pa_symbol(secondary_holding_pen);

                                    release_addr = ioremap_cache(cpu_release_addr[cpu], sizeof(*release_addr));

                                    writeq_relaxed(pa_holding_pen, release_addr);
                                    dcache_clean_inval_poc((__force unsigned long)release_addr,
                                                (__force unsigned long)release_addr +
                                                    sizeof(*release_addr));
                                    sev();

                                    iounmap(release_addr);

                                    return 0;
                                }
                                cpu_psci_cpu_prepare() {

                                }
                            }
                        }
                    }
/* 3. cpu_boot */
                    smp_init() {
                        idle_threads_init() {
                            fork_idle(cpu)
                        }
                        cpuhp_threads_init();

                        bringup_nonboot_cpus(setup_max_cpus) {
                            cpuhp_bringup_mask(cpu_present_mask, setup_max_cpus, CPUHP_ONLINE) {
                                for_each_cpu::cpu_up(cpu, target) {
                                    try_online_node(cpu_to_node(cpu));
                                    _cpu_up(cpu, 0, target) {
                                        cpuhp_up_callbacks(cpu, st, target) {
                                            cpuhp_reset_state(cpu, st, prev_state)

                                            cpuhp_invoke_callback_range(false, cpu, st, prev_state) {
                                                while (cpuhp_next_state(bringup, &state, st, target)) {
                                                    cpuhp_invoke_callback(cpu, state, bringup, NULL, NULL);
                                                }
                                                cpuhp_invoke_callback(cpu, state, bringup, NULL, NULL) {
                                                    struct cpuhp_cpu_state *st = per_cpu_ptr(&cpuhp_state, cpu);
                                                    struct cpuhp_step *step = cpuhp_get_step(state);
                                                    cb = bringup ? step->startup.single : step->teardown.single;
                                                    ret = cb(cpu) {
                                                        bringup_cpu()
                                                            --->
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }

                }
            }
        }
    }
}
```

```c
bringup_cpu() {
    __cpu_up(cpu, idle) { //arch/arm64/kernel/smp.c
        boot_secondary(cpu, idle) {
            ops = get_cpu_ops(cpu);
            ops->cpu_boot(cpu) {

                smp_spin_table_cpu_boot() {
                    u64 __cpu_logical_map[NR_CPUS] = { [0 ... NR_CPUS-1] = INVALID_HWID };
                    u64 cpu_logical_map(unsigned int cpu) {
                        return __cpu_logical_map[cpu];
                    }

                    write_pen_release(cpu_logical_map(cpu)/*val*/) {
                        void *start = (void *)&secondary_holding_pen_release;
                        unsigned long size = sizeof(secondary_holding_pen_release);

                        secondary_holding_pen_release = val;
                        dcache_clean_inval_poc((unsigned long)start, (unsigned long)start + size);
                    }
                    sev();
                }

                cpu_psci_cpu_boot() {
                    phys_addr_t pa_secondary_entry = __pa_symbol(secondary_entry);
                    err = psci_ops.cpu_on(cpu_logical_map(cpu), pa_secondary_entry) {
                        psci_0_2_cpu_on() {
                            __psci_cpu_on() {
                                invoke_psci_fn() {
                                    if (case SMCCC_CONDUIT_HVC) {
                                        invoke_psci_fn = __invoke_psci_fn_hvc() {
                                            arm_smccc_hvc()
                                        }
                                    } else if (case SMCCC_CONDUIT_SMC) {
                                        invoke_psci_fn = __invoke_psci_fn_smc() {
                                            arm_smccc_smc()
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
```

# syscall

<img src='../images/kernel/proc-sched-reg.png' style='max-height:850px'/>

```c
struct pt_regs {
    union {
        struct user_pt_regs user_regs;
        struct {
            u64 regs[31];
            u64 sp;
            u64 pc;
            u64 pstate;
        };
    };
    u64 orig_x0;
#ifdef __AARCH64EB__
    u32 unused2;
    s32 syscallno;
#else
    s32 syscallno;
    u32 unused2;
#endif
    u64 sdei_ttbr1;
    /* Only valid when ARM64_HAS_GIC_PRIO_MASKING is enabled. */
    u64 pmr_save;
    u64 stackframe[2];

    /* Only valid for some EL1 exceptions. */
    u64 lockdep_hardirqs;
    u64 exit_rcu;
};

struct user_pt_regs {
    __u64        regs[31];
    __u64        sp;
    __u64        pc;
    __u64        pstate;
};
```

```c
/* arch/x86/include/asm/ptrace.h */
struct pt_regs {
/* C ABI says these regs are callee-preserved. They aren't saved on kernel entry
 * unless syscall needs a complete, fully filled "struct pt_regs". */
  unsigned long r15;
  unsigned long r14;
  unsigned long r13;
  unsigned long r12;
  unsigned long bp;
  unsigned long bx;
/* These regs are callee-clobbered. Always saved on kernel entry. */
  unsigned long r11;
  unsigned long r10;
  unsigned long r9;
  unsigned long r8;
  unsigned long ax;
  unsigned long cx;
  unsigned long dx;
  unsigned long si;
  unsigned long di;
/* On syscall entry, this is syscall#. On CPU exception, this is error code.
 * On hw interrupt, it's IRQ number: */
  unsigned long orig_ax;
/* Return frame for iretq */
  unsigned long ip;
  unsigned long cs;
  unsigned long flags;
  unsigned long sp;
  unsigned long ss;
/* top of stack page */
};

/* This is the structure pointed to by thread.sp for an inactive task.
 * The order of the fields must match the code in __switch_to_asm(). */
struct inactive_task_frame {
  unsigned long flags;
#ifdef CONFIG_X86_64
  unsigned long r15;
  unsigned long r14;
  unsigned long r13;
  unsigned long r12;
#else
  unsigned long si;
  unsigned long di;
#endif
  unsigned long bx;

  /* These two fields must be together.
   * They form a stack frame header, needed by get_frame_pointer(). */
  unsigned long bp;
  unsigned long ret_addr;
};

struct fork_frame {
  struct inactive_task_frame frame;
  struct pt_regs regs;
};
```

## glibc
```c
int open(const char *pathname, int flags, mode_t mode)

/* syscalls.list */
/* File name Caller  Syscall name    Args    Strong name    Weak names */
      open    -        open          i:siv   __libc_open   __open open
```

```c
/* syscall-template.S */
T_PSEUDO (SYSCALL_SYMBOL, SYSCALL_NAME, SYSCALL_NARGS)
    ret
T_PSEUDO_END (SYSCALL_SYMBOL)

#define T_PSEUDO(SYMBOL, NAME, N)    PSEUDO (SYMBOL, NAME, N)

#define PSEUDO(name, syscall_name, args) \
  .text; \
  ENTRY (name) \
    DO_CALL (syscall_name, args); \
    cmpl $-4095, %eax; \
    jae SYSCALL_ERROR_LABEL
```

## 64

<img src='../images/kernel/init-syscall-stack.png' style='max-height:850px'/>

* glibc
    ```c
    /* glibc-2.28/sysdeps/unix/x86_64/sysdep.h
    The Linux/x86-64 kernel expects the system call parameters in
    registers according to the following table:
        syscall number  rax
        arg 1           rdi
        arg 2           rsi
        arg 3           rdx
        arg 4           r10
        arg 5           r8
        arg 6           r9 */
    #define DO_CALL(syscall_name, args) \
        lea SYS_ify (syscall_name), %rax; \
        syscall

    /* glibc-2.28/sysdeps/unix/sysv/linux/x86_64/sysdep.h */
    #define SYS_ify(syscall_name)  __NR_##syscall_name
    ```

* syscall_table
    1. declare syscall table: arch/x86/entry/syscalls/syscall_64.tbl
        ```c
        # 64-bit system call numbers and entry vectors

        # The __x64_sys_*() stubs are created on-the-fly for sys_*() system calls
        # The abi is "common", "64" or "x32" for this file.
        #
        # <number>  <abi>     <name>    <entry point>
            0       common    read      __x64_sys_read
            1       common    write     __x64_sys_write
            2       common    open      __x64_sys_open
        ```

    2. genrate syscall table: arch/x86/entry/syscalls/Makefile
        ```c
        /* 2.1 arch/x86/entry/syscalls/syscallhdr.sh generates #define __NR_open
        * arch/sh/include/uapi/asm/unistd_64.h */
        #define __NR_restart_syscall    0
        #define __NR_exit               1
        #define __NR_fork               2
        #define __NR_read               3
        #define __NR_write              4
        #define __NR_open               5

        /* 2.2 arch/x86/entry/syscalls/syscalltbl.sh
        * generates __SYSCALL_64(x, y) into asm/syscalls_64.h */
        __SYSCALL_64(__NR_open, __x64_sys_read)
        __SYSCALL_64(__NR_write, __x64_sys_write)
        __SYSCALL_64(__NR_open, __x64_sys_open)

        /* arch/x86/entry/syscall_64.c */
        #define __SYSCALL_64(nr, sym, qual) [nr] = sym

        asmlinkage const sys_call_ptr_t sys_call_table[__NR_syscall_max+1] = {
            /* Smells like a compiler bug -- it doesn't work
            * when the & below is removed. */
            [0 ... __NR_syscall_max] = &sys_ni_syscall,
            #include <asm/syscalls_64.h>
        };
        ```

    3. declare implemenation: include/linux/syscalls.h
        ```c
        asmlinkage long sys_write(unsigned int fd, const char __user *buf, size_t count);
        asmlinkage long sys_read(unsigned int fd, char __user *buf, size_t count);
        asmlinkage long sys_open(const char __user *filename, int flags, umode_t mode);
        ```

    4. define implemenation: fs/open.c
        ```c
        #include <linux/syscalls.h>

        SYSCALL_DEFINE3(open, const char __user *, filename, int, flags, umode_t, mode)
        {
            if (force_o_largefile())
                flags |= O_LARGEFILE;

            return do_sys_open(AT_FDCWD, filename, flags, mode);
        }
        ```

<img src='../images/kernel/init-syscall-64.png' style='max-height:850px'/>

```c
entry_SYSCALL_64()
    /* 1. swap to kernel stack */
    movq  %rsp, PER_CPU_VAR(rsp_scratch)
    movq  PER_CPU_VAR(cpu_current_top_of_stack), %rsp

    /* 2. save user stack */
    pushq  $__USER_DS                 /* pt_regs->ss */
    pushq  PER_CPU_VAR(rsp_scratch)   /* pt_regs->sp */
    pushq  %r11                       /* pt_regs->flags */
    pushq  $__USER_CS                 /* pt_regs->cs */
    pushq  %rcx                       /* pt_regs->ip */
    pushq  %rax                       /* pt_regs->orig_ax */

    /* 3. do_syscall */
    movq  %rax, %rdi
    movq  %rsp, %rsi
    call  do_syscall_64
        regs->ax = __x64_sys_ni_syscall(regs);
        syscall_exit_to_user_mode(regs);
            __syscall_exit_to_user_mode_work();
                exit_to_user_mode_prepare();
                    if (unlikely(ti_work & EXIT_TO_USER_MODE_WORK))
                        ti_work = exit_to_user_mode_loop(regs, ti_work);
                            if (ti_work & _TIF_NEED_RESCHED)
                                schedule();
                            if (ti_work & (_TIF_SIGPENDING | _TIF_NOTIFY_SIGNAL))
                                arch_do_signal_or_restart(regs);

            __exit_to_user_mode();
                arch_exit_to_user_mode();

    /* 4. restore user stack */
    swapgs_restore_regs_and_return_to_usermode()
        POP_REGS pop_rdi=0
        /* The stack is now user RDI, orig_ax, RIP, CS, EFLAGS, RSP, SS */

        movq  %rsp, %rdi /* save kernel sp */
        movq  PER_CPU_VAR(cpu_tss_rw + TSS_sp0), %rsp /* load user sp */

        /* Copy the IRET frame from kernel stack to the user trampoline stack. */
        pushq  6*8(%rdi)  /* SS */
        pushq  5*8(%rdi)  /* RSP */
        pushq  4*8(%rdi)  /* EFLAGS */
        pushq  3*8(%rdi)  /* CS */
        pushq  2*8(%rdi)  /* RIP */

        INTERRUPT_RETURN
```

# process

![](../images/kernel/proc-process-management.svg)

<img src='../images/kernel/proc-compile.png' style='max-height:850px'/>

```c
/* compile */
gcc -c -fPIC process.c
gcc -c -fPIC createprocess.c

/* staic lib */
ar cr libstaticprocess.a process.o
/* static link */
gcc -o staticcreateprocess createprocess.o -L. -lstaticprocess

/* dynamic lib */
gcc -shared -fPIC -o libdynamicprocess.so process.o
/* dynamic link LD_LIBRARY_PATH /lib /usr/lib */
gcc -o dynamiccreateprocess createprocess.o -L. -ldynamicprocess
export LD_LIBRARY_PATH=
```

1. elf: relocatable file

    <img src='../images/kernel/proc-elf-relocatable.png' style='max-height:850px'/>

2. elf: executable file

    * [ELF Format Cheatsheet](https://gist.github.com/x0nu11byt3/bcb35c3de461e5fb66173071a2379779)
    * [Executable and Linkable Format (ELF).pdf](https://www.cs.cmu.edu/afs/cs/academic/class/15213-f00/docs/elf.pdf)
    <img src='../images/kernel/proc-elf.png' style='max-height:850px'/>

3. elf: shared object

4. elf: core dump

* [UEFI简介 - 内核工匠](https://mp.weixin.qq.com/s/tgW9-FDo2hgxm8Uwne8ySw)

<img src='../images/kernel/proc-tree.png' style='max-height:850px'/>

<img src='../images/kernel/proc-elf-compile-exec.png' style='max-height:850px'/>

# thread
<img src='../images/kernel/proc-thread.png' style='max-height:850px'/>

# task_struct
<img src='../images/kernel/proc-task-1.png' style='max-height:850px'/>

# sched

![](../images/kernel/proc-sched-class.png)

* [Linux kernel scheduler](https://helix979.github.io/jkoo/post/os-scheduler/)
* [Kernel Index Sched - LWN](https://lwn.net/Kernel/Index/#Scheduler)
    * [LWN Index - Realtime](https://lwn.net/Kernel/Index/#Realtime)
    * [LWN Index - Scheduler](https://lwn.net/Kernel/Index/#Scheduler)
        * [Scheduling domains](https://lwn.net/Articles/80911/)
    * [LWN Index - CFS scheduler](https://lwn.net/Kernel/Index/#Scheduler-Completely_fair_scheduler)
        * [An EEVDF CPU scheduler for Linux](https://lwn.net/Articles/925371/)
            * [[PATCH 00/15] sched: EEVDF and latency-nice and/or slice-attr](https://lore.kernel.org/all/20230531115839.089944915@infradead.org/#t)
            * [[PATCH 01/15] sched/fair: Add cfs_rq::avg_vruntime](https://github.com/torvalds/linux/commit/af4cf40470c22efa3987200fd19478199e08e103)
            * [[PATCH 03/15] sched/fair: Add lag based placement](https://github.com/torvalds/linux/commit/86bfbb7ce4f67a88df2639198169b685668e7349)
            * [[PATCH 04/15] rbtree: Add rb_add_augmented_cached() helper](https://github.com/torvalds/linux/commit/99d4d26551b56f4e523dd04e4970b94aa796a64e)
            * [[PATCH 05/15] sched/fair: Implement an EEVDF like policy](https://github.com/torvalds/linux/commit/147f3efaa24182a21706bca15eab2f3f4630b5fe)
            * [[PATCH 07/15] sched/smp: Use lag to simplify cross-runqueue placement](https://github.com/torvalds/linux/commit/e8f331bcc270354a803c2127c486190d33eac441)
            * [[PATCH 08/15] sched: Commit to EEVDF](https://github.com/torvalds/linux/commit/5e963f2bd4654a202a8a05aa3a86cb0300b10e6c)
            * ![](../images/kernel/proc-sched-cfs-eevdf.png)
    * [LWN Index - Core scheduling](https://lwn.net/Kernel/Index/#Scheduler-Core_scheduling)
    * [LWN Index - Deadline scheduling](https://lwn.net/Kernel/Index/#Scheduler-Deadline_scheduling)
    * [LWN Index - Group scheduling](https://lwn.net/Kernel/Index/#Scheduler-Group_scheduling)

* [进程调度 - LoyenWang](https://www.cnblogs.com/LoyenWang/tag/进程调度/)
    * [1. 基础](https://www.cnblogs.com/LoyenWang/p/12249106.html)
    * [2. CPU负载](https://www.cnblogs.com/LoyenWang/p/12316660.html)
    * [3. 进程切换](https://www.cnblogs.com/LoyenWang/p/12386281.html)
    * [4. 组调度及带宽控制](https://www.cnblogs.com/LoyenWang/p/12459000.html)
    * [5. CFS调度器](https://www.cnblogs.com/LoyenWang/p/12495319.html)
    * [6. 实时调度器](https://www.cnblogs.com/LoyenWang/p/12584345.html)

* [Wowo Tech](http://www.wowotech.net/sort/process_management)
    * [进程切换分析 - :one:基本框架](http://www.wowotech.net/process_management/context-switch-arch.html)     [:two:TLB处理](http://www.wowotech.net/process_management/context-switch-tlb.html)   [:three:同步处理](http://www.wowotech.net/process_management/scheudle-sync.html)
    * [CFS调度器 - 组调度](http://www.wowotech.net/process_management/449.html)
    * [CFS调度器 - 带宽控制](http://www.wowotech.net/process_management/451.html)
    * [CFS调度器 - 总结](http://www.wowotech.net/process_management/452.html)
    * [ARM Linux上的系统调用代码分析](http://www.wowotech.net/process_management/syscall-arm.html)
    * [Linux调度器 - 用户空间接口](http://www.wowotech.net/process_management/scheduler-API.html)
    * [schedutil governor情景分析](http://www.wowotech.net/process_management/schedutil_governor.html)
    * [TLB flush](http://www.wowotech.net/memory_management/tlb-flush.html)

* [hellokitty2 进程管理](https://www.cnblogs.com/hellokitty2/category/1791168.html)

* [CHENG Jian Linux进程管理与调度](https://kernel.blog.csdn.net/article/details/51456569)
    * [WAKE_AFFINE](https://blog.csdn.net/gatieme/article/details/106315848)
    * [用户抢占和内核抢占](https://blog.csdn.net/gatieme/article/details/51872618)

* [汪辰]
    * [Linux 内核的抢占模型](https://gitee.com/aosp-riscv/working-group/blob/master/articles/20230805-linux-preemption-models.md)
    * [Linux "PREEMPT_RT" 抢占模式分析报告](https://gitee.com/aosp-riscv/working-group/blob/master/articles/20230806-linux-preempt-rt.md#/aosp-riscv/working-group/blob/master/articles/20230805-linux-preemption-models.md)
    * [实时 Linux(Real-Time Linux)](https://gitee.com/aosp-riscv/working-group/blob/master/articles/20230727-rt-linux.md)
    * [Linux 调度器(Schedular)](https://gitee.com/aosp-riscv/working-group/blob/master/articles/20230801-linux-scheduler.md)

* [PREEMPT_RT Linux](https://wiki.linuxfoundation.org/realtime/start)
    * [LWN - A realtime preemption overview](https://lwn.net/Articles/146861/)
    * [Preemption Models](https://wiki.linuxfoundation.org/realtime/documentation/technical_basics/preemption_models)
        Model | Case | Preempt Points
        --- | --- | ---
        PREEMPT_NONE | No Forced Preemption (server) | `system call returns` + `interrupts`
        PREEMPT_VOLUNTARY | Voluntary Kernel Preemption (Desktop) | `system call returns` + `interrupts` + `explicit preemption points`
        PREEMPT | Preemptible Kernel (Low-Latency Desktop) |`system call returns` + `interrupts` + `all kernel code(except critical section)`
        PREEMPT_RT | Fully Preemptible Kernel (RT) | `system call returns` + `interrupts` + `all kernel code(except a few critical section)` + `threaded interrupt handlers`

* [Oracle Linux Blog](https://blogs.oracle.com/linux/category/lnx-linux-kernel-development)
    * [Understanding process thread priorities in Linux](https://blogs.oracle.com/linux/post/task-priority)
        * **static_prio**: maps the priority range used for normal tasks and is the priority according to the nice value of a task.
            > static_prio = 120 + nice
        * **rt_priority**: maps the priority range for real time tasks and indicates real time priority.
            > MAX_RT_PRIO-1 - rt_priority
            * high rt_priority value signifies high priority and in the kernel low priority value signifies high priority
        * **normal_prio**: indicates priority of a task without any temporary priority boosting from the kernel side. For normal tasks it is the same as static_prio and for RT tasks it is directly related to rt_priority
            * In absence of normal_prio, children of a priority boosted task will get boosted priority as well and this will cause CPU starvation for other tasks. To avoid such a situation, the kernel maintains nomral_prio of a task. Forked tasks usually get their effective prio set to normal_prio of the parent and hence don’t get boosted priority.
        * **prio**: is the effective priority of a task and is used in all scheduling related decision makings.
```c
/* Schedule Class:
 * Real time schedule: SCHED_FIFO, SCHED_RR, SCHED_DEADLINE
 * Normal schedule: SCHED_NORMAL, SCHED_BATCH, SCHED_IDLE */
#define SCHED_NORMAL        0
#define SCHED_FIFO          1
#define SCHED_RR            2
#define SCHED_BATCH         3
#define SCHED_IDLE          5
#define SCHED_DEADLINE      6

#define MAX_NICE            19
#define MIN_NICE            -20
#define NICE_WIDTH          (MAX_NICE - MIN_NICE + 1)
#define MAX_USER_RT_PRIO    100
#define MAX_RT_PRIO         MAX_USER_RT_PRIO
#define MAX_PRIO            (MAX_RT_PRIO + NICE_WIDTH)
#define DEFAULT_PRIO        (MAX_RT_PRIO + NICE_WIDTH / 2)

struct task_struct {
    struct thread_info        thread_info;

    int                       on_rq; /* TASK_ON_RQ_{QUEUED, MIGRATING} */

    int                       prio;
    int                       static_prio;
    int                       normal_prio;
    unsigned int              rt_priority;

    const struct sched_class  *sched_class;
    struct sched_entity       se;
    struct sched_rt_entity    rt;
    struct sched_dl_entity    dl;
    struct task_group         *sched_task_group;
    unsigned int              policy;

    struct mm_struct          *mm;
    struct mm_struct          *active_mm;

    void                      *stack; /* kernel stack */

    /* CPU-specific state of this task: */
    struct thread_struct      thread;
};

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
};


/* arch/arm64/include/asm/thread_info.h */
struct thread_struct {
    struct cpu_context    cpu_context;    /* cpu context */

    unsigned long        fault_address;    /* fault info */
    unsigned long        fault_code;    /* ESR_EL1 value */
};

struct rq {
    raw_spinlock_t  lock;
    unsigned int    nr_running;
    unsigned long   cpu_load[CPU_LOAD_IDX_MAX];

    struct load_weight  load;
    unsigned long       nr_load_updates;
    u64                 nr_switches;

    struct cfs_rq cfs;
    struct rt_rq  rt;
    struct dl_rq  dl;
    struct task_struct *curr, *idle, *stop;
};
```
<img src='../images/kernel/proc-sched-entity-rq.png' style='max-height:850px'/>

```c
struct sched_class {
    const struct sched_class *next;

    void (*enqueue_task) (struct rq *rq, struct task_struct *p, int flags);
    void (*dequeue_task) (struct rq *rq, struct task_struct *p, int flags);
    void (*yield_task) (struct rq *rq);
    bool (*yield_to_task) (struct rq *rq, struct task_struct *p, bool preempt);

    void (*wakeup_preempt) (struct rq *rq, struct task_struct *p, int flags);

    struct task_struct * (*pick_next_task) (struct rq *rq,
                struct task_struct *prev,
                struct rq_flags *rf);
    void (*put_prev_task) (struct rq *rq, struct task_struct *p);

    void (*set_curr_task) (struct rq *rq);
    void (*task_tick) (struct rq *rq, struct task_struct *p, int queued);
    void (*task_fork) (struct task_struct *p);
    void (*task_dead) (struct task_struct *p);

    void (*switched_from) (struct rq *this_rq, struct task_struct *task);
    void (*switched_to) (struct rq *this_rq, struct task_struct *task);
    void (*prio_changed) (struct rq *this_rq, struct task_struct *task, int oldprio);
    unsigned int (*get_rr_interval) (struct rq *rq,
            struct task_struct *task);
    void (*update_curr) (struct rq *rq);
};

extern const struct sched_class stop_sched_class;
extern const struct sched_class dl_sched_class;
extern const struct sched_class rt_sched_class;
extern const struct sched_class fair_sched_class;
extern const struct sched_class idle_sched_class;
/* stop_sched_class: highest priority process, will interrupt others
 * dl_sched_class: for deadline
 * rt_sched_class: for RR or FIFO, depend on task_struct->policy
 * fair_sched_class: for normal processes
 * idle_sched_class: idle */
```

<img src='../images/kernel/proc-sched-cpu-rq-class-entity-task.png' style='max-height:850px'/>

## voluntary schedule
<img src="../images/kernel/proc-sched-context-swith.png" style="max-height:850px"/>

<img src='../images/kernel/proc-sched-reg.png' style='max-height:850px'/>

![](../images/kernel/proc-sched-arch.png)

---

![](../images/kernel/proc-sched-context_switch.png)

```c
schedule(void) {
    sched_submit_work(tsk) {
        if (task_is_running(tsk))
            return;

        if (task_flags & (PF_WQ_WORKER | PF_IO_WORKER)) {
            if (task_flags & PF_WQ_WORKER)
                wq_worker_sleeping(tsk) {
                    if (need_more_worker(pool)) {
                        wake_up_worker(pool);
                    }
                }
            else
                io_wq_worker_sleeping(tsk);
        }

        blk_flush_plug(tsk->plug, true);
    }

    do {
        preempt_disable();
        __schedule(SM_NONE);
        sched_preempt_enable_no_resched();
    } while (need_resched());

    sched_update_worker(tsk) {
        if (tsk->flags & (PF_WQ_WORKER | PF_IO_WORKER)) {
            if (tsk->flags & PF_WQ_WORKER)
                wq_worker_running(tsk);
            else
                io_wq_worker_running(tsk);
        }
    }
}

__schedule(SM_NONE) {/* kernel/sched/core.c */
    if (sched_feat(HRTICK) || sched_feat(HRTICK_DL))
        hrtick_clear(rq);

    local_irq_disable();
    rq_lock(rq, &rf);
    update_rq_clock(rq);

    prev = rq->curr;
    prev_state = READ_ONCE(prev->__state);
    if (!(sched_mode & SM_MASK_PREEMPT) && prev_state/* tsk not running */) {
        if (signal_pending_state(prev_state, prev)) {
            WRITE_ONCE(prev->__state, TASK_RUNNING);
        } else {
            prev->sched_contributes_to_load =
                (prev_state & TASK_UNINTERRUPTIBLE) &&
                !(prev_state & TASK_NOLOAD) &&
                !(prev_state & TASK_FROZEN);

            if (prev->sched_contributes_to_load)
                rq->nr_uninterruptible++;

            deactivate_task(rq, prev, DEQUEUE_SLEEP | DEQUEUE_NOCLOCK) {
                p->on_rq = (flags & DEQUEUE_SLEEP) ? 0 : TASK_ON_RQ_MIGRATING;
                dequeue_task(rq, p, flags);
            }

            if (prev->in_iowait) {
                atomic_inc(&rq->nr_iowait);
                delayacct_blkio_start();
            }
        }
        switch_count = &prev->nvcsw;
    }

    next = pick_next_task(rq, prev, &rf) {
        if (!sched_core_enabled(rq)) {
            return __pick_next_task(rq, prev, rf) {
                if (likely(!sched_class_above(prev->sched_class, &fair_sched_class)
                    && rq->nr_running == rq->cfs.h_nr_running)) {

                    p = pick_next_task_fair(rq, prev, rf);
                    if (unlikely(p == RETRY_TASK))
                        goto restart;

                    /* Assume the next prioritized class is idle_sched_class */
                    if (!p) {
                        put_prev_task(rq, prev);
                        p = pick_next_task_idle(rq);
                    }

                    return p;
                }

            restart:
                put_prev_task_balance(rq, prev, rf) {
                    for_class_range(class, prev->sched_class, &idle_sched_class) {
                        if (class->balance(rq, prev, rf))
                            break;
                    }

                    put_prev_task(rq, prev);
                        --->
                }

                for_each_class(class) {
                    p = class->pick_next_task(rq);
                    if (p)
                        return p;
                }
            }
        }

        cpu = cpu_of(rq);

        /* Stopper task is switching into idle, no need core-wide selection. */
        if (cpu_is_offline(cpu)) {
            rq->core_pick = NULL;
            return __pick_next_task(rq, prev, rf);
        }

        /* do core sched */
    }

    clear_tsk_need_resched(prev);
    clear_preempt_need_resched();

    if (likely(prev != next)) {
        rq->nr_switches++;
        RCU_INIT_POINTER(rq->curr, next);

        context_switch(rq, prev, next, &rf) {
            prepare_task_switch(rq, prev, next);
            arch_start_context_switch(prev);

            /* kernel -> kernel   lazy + transfer active
             *   user -> kernel   lazy + mmgrab_lazy_tlb() active
             *
             * kernel ->   user   switch + mmdrop_lazy_tlb() active
             *   user ->   user   switch */
            if (!next->mm) { /* to kernel task */
                enter_lazy_tlb(prev->active_mm, next) {
                    update_saved_ttbr0(tsk, &init_mm) {
                        if (mm == &init_mm)
                            ttbr = phys_to_ttbr(__pa_symbol(reserved_pg_dir));
                        else
                            ttbr = phys_to_ttbr(virt_to_phys(mm->pgd)) | ASID(mm) << 48;

                        WRITE_ONCE(task_thread_info(tsk)->ttbr0, ttbr);
                    }
                }

                next->active_mm = prev->active_mm;

                if (prev->mm) {/* from user */
                    mmgrab_lazy_tlb(prev->active_mm) {
                        atomic_inc(&mm->mm_count);
                    }
                } else {
                    prev->active_mm = NULL;
                }
            } else { /* to user task */
                membarrier_switch_mm(rq, prev->active_mm, next->mm);
                switch_mm_irqs_off(prev->active_mm, next->mm, next) {
                    /* arch/arm64/include/asm/mmu_context.h */
                    switch_mm(mm_prev, mm_next, tsk) {
                        if (prev != next) {
                            __switch_mm(next) {
                                if (next == &init_mm) {
                                    cpu_set_reserved_ttbr0() {
                                        ttbr = phys_to_ttbr(__pa_symbol(reserved_pg_dir));
                                        write_sysreg(ttbr, ttbr0_el1);
                                    }
                                    return;
                                }

                                check_and_switch_context(next) {
                                    local_flush_tlb_all() {
                                        __tlbi(vmalle1);
                                    }

                                    cpu_switch_mm(mm->pgd, mm) {
                                        BUG_ON(pgd == swapper_pg_dir);
                                        cpu_set_reserved_ttbr0();
                                            --->
                                        cpu_do_switch_mm(virt_to_phys(pgd)/*pgd_phys*/, mm) {
                                            ttbr1 = read_sysreg(ttbr1_el1);
                                            write_sysreg(ttbr1, ttbr1_el1);
                                            isb();
                                            ttbr0 = phys_to_ttbr(pgd_phys);
                                            write_sysreg(ttbr0, ttbr0_el1);
                                        }
                                    }
                                }
                            }
                        }

                        update_saved_ttbr0(tsk, next);
                            --->
                    }
                }
                lru_gen_use_mm(next->mm);

                if (!prev->mm) { /* from kernel */
                    /* will mmdrop_lazy_tlb() in finish_task_switch(). */
                    rq->prev_mm = prev->active_mm;
                    prev->active_mm = NULL;
                }
            }

            switch_to(prev, next, prev) {
                __switch_to() {
                    fpsimd_thread_switch(next);
                    tls_thread_switch(next);
                    hw_breakpoint_thread_switch(next);
                    contextidr_thread_switch(next);
                    entry_task_switch(next) {
                        __this_cpu_write(__entry_task, next);
                    }
                    ssbs_thread_switch(next);
                    erratum_1418040_thread_switch(next);
                    ptrauth_thread_switch_user(next);

                    /* arch/arm64/kernel/entry.S */
                    last = cpu_switch_to(prev, next) {
                        /* x0 = previous task_struct (must be preserved across the switch)
                         * x1 = next task_struct
                         * Previous and next are guaranteed not to be the same. */
                        SYM_FUNC_START(cpu_switch_to)
                            mov    x10, #THREAD_CPU_CONTEXT /* task.thread.cpu_context */
                            add    x8, x0, x10 /* calc prev task cpu_context addr */
                            mov    x9, sp
                            stp    x19, x20, [x8], #16 /* store callee-saved registers */
                            stp    x21, x22, [x8], #16
                            stp    x23, x24, [x8], #16
                            stp    x25, x26, [x8], #16
                            stp    x27, x28, [x8], #16
                            stp    x29, x9, [x8], #16
                            str    lr, [x8]

                            add    x8, x1, x10  /* calc next task cpu_context addr */
                            ldp    x19, x20, [x8], #16 /* restore callee-saved registers */
                            ldp    x21, x22, [x8], #16
                            ldp    x23, x24, [x8], #16
                            ldp    x25, x26, [x8], #16
                            ldp    x27, x28, [x8], #16
                            ldp    x29, x9, [x8], #16
                            ldr    lr, [x8]
                            mov    sp, x9
                            msr    sp_el0, x1

                            ptrauth_keys_install_kernel x1, x8, x9, x10
                            scs_save x0
                            scs_load_current
                            ret
                    }
                }
            }

            return finish_task_switch(prev) {
                struct rq *rq = this_rq();
                struct mm_struct *mm = rq->prev_mm;

                rq->prev_mm = NULL;

                prev_state = READ_ONCE(prev->__state);
                vtime_task_switch(prev);
                perf_event_task_sched_in(prev, current);
                finish_task(prev) {
                    smp_store_release(&prev->on_cpu, 0);
                }
                tick_nohz_task_switch();
                finish_lock_switch(rq) {
                    __balance_callbacks(rq);
                        --->
                }
                finish_arch_post_lock_switch();
                kcov_finish_switch(current);
                kmap_local_sched_in();

                fire_sched_in_preempt_notifiers(current);

                if (mm) {
                    membarrier_mm_sync_core_before_usermode(mm);
                    mmdrop_lazy_tlb_sched(mm) {
                        mmdrop_sched(mm) {
                            if (unlikely(atomic_dec_and_test(&mm->mm_count))) {
                                __mmdrop(mm) {
                                    cleanup_lazy_tlbs(mm);

                                    WARN_ON_ONCE(mm == current->active_mm);
                                    mm_free_pgd(mm) {
                                        pgd_free(mm, mm->pgd) {
                                            if (PGD_SIZE == PAGE_SIZE)
                                                free_page((unsigned long)pgd);
                                            else
                                                kmem_cache_free(pgd_cache, pgd);
                                        }
                                    }
                                    destroy_context(mm);
                                    mmu_notifier_subscriptions_destroy(mm);
                                    check_mm(mm);
                                    put_user_ns(mm->user_ns);
                                    mm_pasid_drop(mm);
                                    mm_destroy_cid(mm);

                                    for (i = 0; i < NR_MM_COUNTERS; i++)
                                        percpu_counter_destroy(&mm->rss_stat[i]);
                                    free_mm(mm) {
                                        kmem_cache_free(mm_cachep, (mm));
                                    }
                                }
                            }
                        }
                    }
                }

                if (unlikely(prev_state == TASK_DEAD)) {
                    if (prev->sched_class->task_dead)
                        prev->sched_class->task_dead(prev);

                    /* Task is done with its stack. */
                    put_task_stack(prev);

                    put_task_struct_rcu_user(prev);
                }

                return rq;
            }
        }
    } else {
        __balance_callbacks(rq) {
            do_balance_callbacks(rq, __splice_balance_callbacks(rq, false)/*head*/) {
                void (*func)(struct rq *rq);
                struct balance_callback *next;

                while (head) {
                    func = (void (*)(struct rq *))head->func;
                    next = head->next;
                    head->next = NULL;
                    head = next;

                    func(rq);
                }
            }
        }
        raw_spin_rq_unlock_irq(rq) {
            raw_spin_rq_unlock(rq);
            local_irq_enable();
        }
    }
}
```

<img src='../images/kernel/proc-sched-voluntary.png' style='max-height:850px'/>


## preempt schedule

### user preempt

#### set_tsk_need_resched

![](../images/kernel/proc-preempt-user-mark.png)

##### scheduler_tick

```c
void scheduler_tick(void)
{
    int cpu = smp_processor_id();
    struct rq *rq = cpu_rq(cpu);
    struct task_struct *curr = rq->curr;
    struct rq_flags rf;

    sched_clock_tick();

    rq_lock(rq, &rf);

    update_rq_clock(rq);
    curr->sched_class->task_tick(rq, curr, 0); /* task_tick_fair */
    cpu_load_update_active(rq);
    calc_global_load_tick(rq);

    rq_unlock(rq, &rf);

    perf_event_task_tick();

#ifdef CONFIG_SMP
    rq->idle_balance = idle_cpu(cpu);
    sched_balance_trigger(rq);
#endif
}
```

##### try_to_wake_upp

##### sched_setscheduler

```c
SYSCALL_DEFINE3(sched_setscheduler) {
    do_sched_setscheduler() {
        p = find_process_by_pid(pid);
        get_task_struct(p);
        sched_setscheduler(p, policy, &lparam) {
            __sched_setscheduler() {

            }
        }
    }

        put_task_struct(p);
}
/* 1. check policy, prio args */

```

#### preempt time

![](../images/kernel/proc-preempt-user-exec.png)

##### return from system call
```c
/* syscall_exit_to_user_mode_work -> exit_to_user_mode_prepare -> */
static void exit_to_user_mode_loop(struct pt_regs *regs, u32 cached_flags)
{
    while (ti_work & EXIT_TO_USER_MODE_WORK) {
        if (ti_work & _TIF_NEED_RESCHED)
            schedule();

        if (ti_work & _TIF_UPROBE)
            uprobe_notify_resume(regs);

        if (ti_work & _TIF_PATCH_PENDING)
            klp_update_patch_state(current);

        if (ti_work & (_TIF_SIGPENDING | _TIF_NOTIFY_SIGNAL))
            arch_do_signal_or_restart(regs);

        if (ti_work & _TIF_NOTIFY_RESUME)
            resume_user_mode_work(regs);

        ti_work = read_thread_flags();
    }
}
```

##### return from interrupt

```c
irqentry_exit() {
    if (user_mode(regs)) {
        irqentry_exit_to_user_mode(regs);
            exit_to_user_mode_prepare(ress);
                exit_to_user_mode_loop(regs);
                    if (ti_work & _TIF_NEED_RESCHED)
                        schedule();
                arch_exit_to_user_mode_prepare(regs, ti_work);
    }
}
```

### kernel preempt

![](../images/kernel/proc-preempt-kernel.png)

![](../images/kernel/proc-preempt-kernel-exec.png)

![](../images/kernel/proc-preempt_count.png)

#### preempt_enble

```c
#define preempt_enable() \
do { \
    if (unlikely(preempt_count_dec_and_test())) \
        __preempt_schedule(); \
} while (0)

#define preempt_count_dec_and_test() \
    ({ preempt_count_sub(1); should_resched(0); })

static  bool should_resched(int preempt_offset)
{
    return unlikely(preempt_count() == preempt_offset &&
        tif_need_resched());
}

#define tif_need_resched() test_thread_flag(TIF_NEED_RESCHED)

/* __preempt_schedule -> */
static void __sched notrace preempt_schedule_common(void)
{
    do {
        preempt_disable_notrace();
        __schedule(SM_PREEMPT);
        preempt_enable_no_resched_notrace();
    } while (need_resched());
}
```

#### preempt_schedule_irq

```c
/* do_IRQ -> retint_kernel */
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
            }
        }
    }
}
```
<img src='../images/kernel/proc-sched.png' style='max-height:850px'/>

# SCHED_RT

![](../images/kernel/proc-sched-rt.png)

```c
struct rt_rq {
    struct rt_prio_array    active;
    unsigned int            rt_nr_running;
    unsigned int            rr_nr_running;
    struct {
        int     curr; /* highest queued rt task prio */
        int     next; /* next highest */
    } highest_prio;

    int                     overloaded;
    struct plist_head       pushable_tasks;

    /* rt_rq is enqueued into rq */
    int                     rt_queued;

    int                     rt_throttled;
    u64                     rt_time;    /* current time usage */
    u64                     rt_runtime; /* max time usage */

#ifdef CONFIG_RT_GROUP_SCHED
    unsigned int            rt_nr_boosted;
    struct rq               *rq;
    struct task_group       *tg;
#endif
};

struct sched_rt_entity {
    struct list_head            run_list;
    unsigned long               timeout;
    unsigned long               watchdog_stamp;
    unsigned int                time_slice;
    unsigned short              on_rq;
    unsigned short              on_list;

    struct sched_rt_entity      *back;
#ifdef CONFIG_RT_GROUP_SCHED
    struct sched_rt_entity      *parent;
    /* rq on which this entity is (to be) queued: */
    struct rt_rq                *rt_rq;
    /* rq "owned" by this entity/group: */
    struct rt_rq                *my_q;
#endif
};
```

![](../images/kernel/proc-sched-rt-update_curr_rt.png)

![](../images/kernel/proc-sched-rt-sched_rt_avg_update.png)

## enqueue_task_rt

![](../images/kernel/proc-sched-rt-enque-deque-task.png)

```c
enqueue_task_rt(struct rq *rq, struct task_struct *p, int flags) {
    if (flags & ENQUEUE_WAKEUP)
        rt_se->timeout = 0;

    check_schedstat_required();
    update_stats_wait_start_rt(rt_rq_of_se(rt_se), rt_se);

    enqueue_rt_entity(rt_se, flags) {
        update_stats_enqueue_rt(rt_rq_of_se(rt_se), rt_se, flags);

        /* Because the prio of an upper entry depends on the lower
         * entries, we must remove entries top - down. */
        dequeue_rt_stack(rt_se, flags);

        for_each_sched_rt_entity(rt_se) {
            __enqueue_rt_entity(rt_se, flags) {
/* 1. insert into list */
                if (flags & ENQUEUE_HEAD)
                    list_add(&rt_se->run_list, queue);
                else
                    list_add_tail(&rt_se->run_list, queue);
                __set_bit(rt_se_prio(rt_se), array->bitmap);

                rt_se->on_list = 1;
                rt_se->on_rq = 1;

                inc_rt_tasks(rt_se, rt_rq) {
                    int prio = rt_se_prio(rt_se);

                    rt_rq->rt_nr_running += rt_se_nr_running(rt_se);
                    rt_rq->rr_nr_running += rt_se_rr_nr_running(rt_se);
/* 2. update cpu prio vec */
                    inc_rt_prio(rt_rq, prio) {
                        int prev_prio = rt_rq->highest_prio.curr;

                        if (prio < prev_prio)
                            rt_rq->highest_prio.curr = prio;

                        inc_rt_prio_smp(rt_rq, prio, prev_prio) {
                            struct rq *rq = rq_of_rt_rq(rt_rq);
                            if (&rq->rt != rt_rq) {
                                return;
                            }
                            if (rq->online && prio < prev_prio) {
                                cpupri_set(&rq->rd->cpupri/*cp*/, rq->cpu, prio) {
                                    int *currpri = &cp->cpu_to_pri[cpu];
                                    int oldpri = *currpri;

                                    newpri = convert_prio(newpri);
                                    if (newpri == oldpri) {
                                        return;
                                    }

                                    if (likely(newpri != CPUPRI_INVALID)) {
                                        struct cpupri_vec *vec = &cp->pri_to_cpu[newpri];
                                        cpumask_set_cpu(cpu, vec->mask);
                                        atomic_inc(&(vec)->count);
                                    }
                                    if (likely(oldpri != CPUPRI_INVALID)) {
                                        struct cpupri_vec *vec  = &cp->pri_to_cpu[oldpri];
                                        atomic_dec(&(vec)->count);
                                        cpumask_clear_cpu(cpu, vec->mask);
                                    }

                                    *currpri = newpri;
                                }
                            }
                        }
                    }
                    inc_rt_group(rt_se, rt_rq) {
                        if (rt_se_boosted(rt_se))
                            rt_rq->rt_nr_boosted++;

                        if (rt_rq->tg) {
                            start_rt_bandwidth(&rt_rq->tg->rt_bandwidth) {
                                if (!rt_b->rt_period_active) {
                                    rt_b->rt_period_active = 1;
                                    hrtimer_forward_now(&rt_b->rt_period_timer, ns_to_ktime(0));
                                    hrtimer_start_expires(&rt_b->rt_period_timer,
                                                HRTIMER_MODE_ABS_PINNED_HARD);
                                }
                            }
                        }
                    }
                }
            }
        }
        enqueue_top_rt_rq(&rq->rt) {
            struct rq *rq = rq_of_rt_rq(rt_rq);
            if (rt_rq->rt_queued)
                return;

            if (rt_rq_throttled(rt_rq))
                return;

            if (rt_rq->rt_nr_running) {
                add_nr_running(rq, rt_rq->rt_nr_running);
                rt_rq->rt_queued = 1;
            }

            /* Kick cpufreq (see the comment in kernel/sched/sched.h). */
            cpufreq_update_util(rq, 0);
        }
    }

/* 3. enqueue pushable task */
    if (!task_current(rq, p) && p->nr_cpus_allowed > 1) {
        enqueue_pushable_task(rq, p) {
            plist_del(&p->pushable_tasks, &rq->rt.pushable_tasks);
            plist_node_init(&p->pushable_tasks, p->prio);
            plist_add(&p->pushable_tasks, &rq->rt.pushable_tasks);

            if (p->prio < rq->rt.highest_prio.next) {
                rq->rt.highest_prio.next = p->prio;
            }

            if (!rq->rt.overloaded) {
                rt_set_overload(rq) {
                    if (!rq->online)
                        return;

                    /* used in pull_rt_task */
                    cpumask_set_cpu(rq->cpu, rq->rd->rto_mask);
                    smp_wmb();
                    atomic_inc(&rq->rd->rto_count);
                }
                rq->rt.overloaded = 1;
            }
        }
    }

}
```

## dequeue_task_rt

```c
static void dequeue_task_rt(struct rq *rq, struct task_struct *p, int flags) {
    struct sched_rt_entity *rt_se = &p->rt;

    update_curr_rt(rq);
    dequeue_rt_entity(rt_se, flags) {
        struct rq *rq = rq_of_rt_se(rt_se);

        update_stats_dequeue_rt(rt_rq_of_se(rt_se), rt_se, flags);

        dequeue_rt_stack(rt_se, flags) {
            /* Because the prio of an upper entry depends on the lower
             * entries, we must remove entries top - down. */
            for_each_sched_rt_entity(rt_se) {
                rt_se->back = back;
                back = rt_se;
            }

            rt_nr_running = rt_rq_of_se(back)->rt_nr_running;

            for (rt_se = back; rt_se; rt_se = rt_se->back) {
                if (on_rt_rq(rt_se)) { /* rt_se->on_rq */
                    __dequeue_rt_entity(rt_se, flags) {
                        struct rt_rq *rt_rq = rt_rq_of_se(rt_se);
                        struct rt_prio_array *array = &rt_rq->active;
/* 1. remove from list */
                        if (move_entity(flags)) {
                            __delist_rt_entity(rt_se, array) {
                                list_del_init(&rt_se->run_list);

                                if (list_empty(array->queue + rt_se_prio(rt_se))) {
                                    __clear_bit(rt_se_prio(rt_se), array->bitmap);
                                }

                                rt_se->on_list = 0;
                            }
                        }
                        rt_se->on_rq = 0;

                        dec_rt_tasks(rt_se, rt_rq) {
                            rt_rq->rt_nr_running -= rt_se_nr_running(rt_se);
                            rt_rq->rr_nr_running -= rt_se_rr_nr_running(rt_se);
/* 2. update cpu prio vec */
                            dec_rt_prio(rt_rq, rt_se_prio(rt_se)/*prio*/) {
                                int prev_prio = rt_rq->highest_prio.curr;

                                if (rt_rq->rt_nr_running) {
                                    if (prio == prev_prio) {
                                        struct rt_prio_array *array = &rt_rq->active;
                                        rt_rq->highest_prio.curr = sched_find_first_bit(array->bitmap);
                                    }
                                } else {
                                    rt_rq->highest_prio.curr = MAX_RT_PRIO-1;
                                }

                                dec_rt_prio_smp(rt_rq, prio, prev_prio) {
                                    if (rq->online && rt_rq->highest_prio.curr != prev_prio) {
                                        cpupri_set(&rq->rd->cpupri, rq->cpu, rt_rq->highest_prio.curr);
                                    }
                                }
                            }
                            dec_rt_group(rt_se, rt_rq) {
                                if (rt_se_boosted(rt_se))
                                    rt_rq->rt_nr_boosted--;

                                WARN_ON(!rt_rq->rt_nr_running && rt_rq->rt_nr_boosted);
                            }
                        }
                    }
                }
            }

            dequeue_top_rt_rq(rt_rq_of_se(back), rt_nr_running) {
                struct rq *rq = rq_of_rt_rq(rt_rq);
                if (!rt_rq->rt_queued)
                    return;

                sub_nr_running(rq, count);
                rt_rq->rt_queued = 0;
            }
        }

        for_each_sched_rt_entity(rt_se) {
            struct rt_rq *rt_rq = group_rt_rq(rt_se);

            if (rt_rq && rt_rq->rt_nr_running)
                __enqueue_rt_entity(rt_se, flags);
        }

        enqueue_top_rt_rq(&rq->rt);
            --->
    }
/* 3. dequeue_pushable_task */
    dequeue_pushable_task(rq, p) {
        plist_del(&p->pushable_tasks, &rq->rt.pushable_tasks);

        if (has_pushable_tasks(rq)) {
            p = plist_first_entry(&rq->rt.pushable_tasks, struct task_struct, pushable_tasks);
            rq->rt.highest_prio.next = p->prio;
        } else {
            rq->rt.highest_prio.next = MAX_RT_PRIO-1;

            if (rq->rt.overloaded) {
                rt_clear_overload(rq);
                rq->rt.overloaded = 0;
            }
        }
    }
}
```

## put_prev_task_rt

```c
put_prev_task_rt(struct rq *rq, struct task_struct *p) {
    struct sched_rt_entity *rt_se = &p->rt;
    struct rt_rq *rt_rq = &rq->rt;

    if (on_rt_rq(&p->rt))
        update_stats_wait_start_rt(rt_rq, rt_se);

    update_curr_rt(rq);

    update_rt_rq_load_avg(rq_clock_pelt(rq), rq, 1);

    if (on_rt_rq(&p->rt) && p->nr_cpus_allowed > 1) {
        enqueue_pushable_task(rq, p);
    }
}
```

## pick_next_task_rt

```c
pick_next_task_rt(struct rq *rq)
    struct task_struct *p = pick_task_rt(rq) {
        ret = sched_rt_runnable(rq) {
            return rq->rt.rt_queued > 0;
        }
        if (!ret) {
            return NULL;
        }

        p = _pick_next_task_rt(rq) {
            do {
                rt_se = pick_next_rt_entity(rt_rq) {
                    struct rt_prio_array *array = &rt_rq->active;
                    struct sched_rt_entity *next = NULL;
                    struct list_head *queue;
                    int idx;

                    idx = sched_find_first_bit(array->bitmap);
                    queue = array->queue + idx;

                    next = list_entry(queue->next, struct sched_rt_entity, run_list);

                    return next;
                }
                if (unlikely(!rt_se))
                    return NULL;
                rt_rq = group_rt_rq(rt_se);
            } while (rt_rq);

            return rt_task_of(rt_se);
        }
    }
    if (p) {
        set_next_task_rt(rq, p, true)
            --->
    }
```

## set_next_task_rt

```c
set_next_task_rt(struct rq *rq, struct task_struct *p, bool first) {
    p->se.exec_start = rq_clock_task(rq);

    dequeue_pushable_task(rq, p)
        --->

    if (!first)
        return;

    if (rq->curr->sched_class != &rt_sched_class) {
        update_rt_rq_load_avg(rq_clock_pelt(rq), rq, 0);
    }

    rt_queue_push_tasks(rq) {
        if (!has_pushable_tasks(rq))
            return;

        queue_balance_callback(rq, &per_cpu(rt_push_head, rq->cpu)/*head*/, push_rt_tasks/*func*/) {
            head->func = func;
            head->next = rq->balance_callback;
            rq->balance_callback = head;
        }
    }
```

## select_task_rq_rt

* [内核工匠 - Linux Scheduler之rt选核流程](https://mp.weixin.qq.com/s/DByOOnJYTA2BDrwTXSDBmQ)
* [hellokitty2 - 调度器32 - RT选核](https://www.cnblogs.com/hellokitty2/p/15881574.html)

![](../images/kernel/proc-sched-rt-cpupri.png)

```c
try_to_wake_up() { /* wake up select */
    select_task_rq(p, p->wake_cpu, SD_BALANCE_WAKE, wake_flags);
}

wake_up_new_task() { /* fork select */
    select_task_rq(p, task_cpu(p), SD_BALANCE_FORK, 0);
}

sched_exec() { /* exec select */
    select_task_rq(p, task_cpu(p), SD_BALANCE_EXEC, 0);
}
```

```c
#define CPUPRI_NR_PRIORITIES    (MAX_RT_PRIO+1)

struct cpupri {
    struct cpupri_vec   pri_to_cpu[CPUPRI_NR_PRIORITIES];
    int                 *cpu_to_pri;
};

struct cpupri_vec {
    atomic_t        count;
    cpumask_var_t   mask;
};
```

```c
select_task_rq_rt(struct task_struct *p, int cpu, int flags)
    struct task_struct *curr;
    struct rq *rq;
    bool test;

    /* For anything but wake ups, just return the task_cpu */
    if (!(flags & (WF_TTWU | WF_FORK)))
        goto out;

    rq = cpu_rq(cpu);

    rcu_read_lock();
    curr = READ_ONCE(rq->curr); /* unlocked access */

    /* test if curr must run on this core */
    test = curr && unlikely(rt_task(curr))
        && (curr->nr_cpus_allowed < 2 || curr->prio <= p->prio);

    if (test || !rt_task_fits_capacity(p, cpu)) {
        /* 1. find lowest cpus */
        int target = find_lowest_rq(p) {
            struct sched_domain *sd;
            struct cpumask *lowest_mask = this_cpu_cpumask_var_ptr(local_cpu_mask);
            int this_cpu = smp_processor_id();
            int cpu      = task_cpu(task);
            int ret;

            if (unlikely(!lowest_mask))
                return -1;
            if (task->nr_cpus_allowed == 1)
                return -1; /* No other targets possible */

            if (sched_asym_cpucap_active()) {
                ret = cpupri_find_fitness(
                    &task_rq(task)->rd->cpupri,
                    task, lowest_mask,
                    rt_task_fits_capacity/*fitness_fn*/
                ) {
                    int task_pri = convert_prio(p->prio) {
                        /* preempt order:
                         * INVILID > IDLE > NORMAL > RT0...RT99 */
                    }
                    int idx, cpu;

                    /* 1.1 find lowest cpus from vec[0, idx] */
                    for (idx = 0; idx < task_pri; idx++) {
                        /* 1.1.1 mask p and vec*/
                        ret = __cpupri_find(cp, p, lowest_mask, idx) {
                            struct cpupri_vec *vec  = &cp->pri_to_cpu[idx];
                            int skip = 0;

                            if (!atomic_read(&(vec)->count)) {
                                skip = 1;
                            }
                            smp_rmb();
                            if (skip) {
                                return 0;
                            }
                            if (cpumask_any_and(&p->cpus_mask, vec->mask) >= nr_cpu_ids) {
                                return 0;
                            }
                            if (lowest_mask) {
                                cpumask_and(lowest_mask, &p->cpus_mask, vec->mask);
                                cpumask_and(lowest_mask, lowest_mask, cpu_active_mask);
                                if (cpumask_empty(lowest_mask))
                                    return 0;
                            }
                            return 1;
                        }
                        if (!ret) {
                            continue;
                        }
                        if (!lowest_mask || !fitness_fn) {
                            return 1;
                        }

                        /* 1.1.2 remove cpu which is incapable for this task */
                        for_each_cpu(cpu, lowest_mask) {
                            if (!fitness_fn(p, cpu)) {
                                cpumask_clear_cpu(cpu, lowest_mask);
                            }
                        }

                        if (cpumask_empty(lowest_mask)) {
                            continue;
                        }
                        return 1;
                    }
                    /* If we failed to find a fitting lowest_mask, kick off a new search
                     * but without taking into account any fitness criteria this time. */
                    if (fitness_fn) {
                        return cpupri_find(cp, p, lowest_mask);
                    }

                    return 0;
                }
            } else {
                ret = cpupri_find(&task_rq(task)->rd->cpupri, task, lowest_mask) {
                    return cpupri_find_fitness(cp, p, lowest_mask, NULL);
                }
            }

            if (!ret) {
                return -1; /* No targets found */
            }

            /* 1.2 pick the cpu which run task previously
             * We prioritize the last CPU that the task executed on since
             * it is most likely cache-hot in that location. */
            if (cpumask_test_cpu(cpu, lowest_mask)) {
                return cpu;
            }

            /* 1.3 pick from sched domain
             * Otherwise, we consult the sched_domains span maps to figure
             * out which CPU is logically closest to our hot cache data. */
            if (!cpumask_test_cpu(this_cpu, lowest_mask)) {
                this_cpu = -1; /* Skip this_cpu opt if not among lowest */
            }

            rcu_read_lock();
            for_each_domain(cpu, sd) {
                if (sd->flags & SD_WAKE_AFFINE) {
                    int best_cpu;

                    /* 1.3.1 sd find this_cpu */
                    if (this_cpu != -1 && cpumask_test_cpu(this_cpu, sched_domain_span(sd))) {
                        rcu_read_unlock();
                        return this_cpu;
                    }

                    /* 1.3.2 pick any cpu from sched domain if best_cpu < nr_cpu_ids */
                    best_cpu = cpumask_any_and_distribute(lowest_mask, sched_domain_span(sd));
                    if (best_cpu < nr_cpu_ids) {
                        rcu_read_unlock();
                        return best_cpu;
                    }
                }
            }
            rcu_read_unlock();

            /* 1.4 pick this_cpu if valid */
            if (this_cpu != -1) {
                return this_cpu;
            }

            /* 1.5 pick any lowest cpu if valid */
            cpu = cpumask_any_distribute(lowest_mask);
            if (cpu < nr_cpu_ids) {
                return cpu;
            }

            return -1;
        }

        /* 2. pick tsk cpu: target lowest target cpu is incapable */
        if (!test && target != -1 && !rt_task_fits_capacity(p, target))
            goto out_unlock;

        /* 3. pick target cpu: p prio is higher than the prio of task from target cpu */
        if (target != -1 && p->prio < cpu_rq(target)->rt.highest_prio.curr)
            cpu = target;
    }

out_unlock:
    return cpu;
```

## wakeup_preempt_rt

```c
void wakeup_preempt_rt(struct rq *rq, struct task_struct *p, int flags)
{
    if (p->prio < rq->curr->prio) {
        resched_curr(rq);
        return;
    }

    /* If:
     * - the newly woken task is of equal priority to the current task
     * - the newly woken task is non-migratable while current is migratable
     * - current will be preempted on the next reschedule
     *
     * we should check to see if current can readily move to a different
     * cpu.  If so, we will reschedule to allow the push logic to try
     * to move current somewhere else, making room for our non-migratable
     * task. */
    if (p->prio == rq->curr->prio && !test_tsk_need_resched(rq->curr)) {
        check_preempt_equal_prio(rq, p) {
            /* Current can't be migrated, useless to reschedule,
             * let's hope p can move out. */
            if (rq->curr->nr_cpus_allowed == 1
                || !cpupri_find(&rq->rd->cpupri, rq->curr, NULL)) {

                return;
            }

            /* p is migratable, so let's not schedule it and
             * see if it is pushed or pulled somewhere else.  */
            if (p->nr_cpus_allowed != 1
                && cpupri_find(&rq->rd->cpupri, p, NULL)) {

                return;
            }

            /* There appear to be other CPUs that can accept
             * the current task but none can run 'p', so lets reschedule
             * to try and push the current task away: */
            requeue_task_rt(rq, p, 1);
            resched_curr(rq);
        }
    }
}
```

## task_tick_rt

```c
task_tick_rt() {
    update_curr_rt() {
        update_curr_se();
        sched_rt_runtime_exceeded();
    }

    update_rt_rq_load_avg();

    if (p->policy == SCHED_RR) {
        if (--p->rt.time_slice)
            return;

        p->rt.time_slice = sched_rr_timeslice;
        requeue_task_rt(rq, p, 0);
        resched_curr(rq);
    }
}

task_tick_rt(struct rq *rq, struct task_struct *p, int queued)
{
    struct sched_rt_entity *rt_se = &p->rt;

    update_curr_rt(struct rq *rq) {
        if (curr->sched_class != &rt_sched_class)
            return;

        delta_exec = update_curr_common(rq) {
            delta_exec = update_curr_se(rq, &curr->se) {
                u64 now = rq_clock_task(rq);
                s64 delta_exec;

                delta_exec = now - curr->exec_start;
                if (unlikely(delta_exec <= 0))
                    return delta_exec;

                curr->exec_start = now;
                curr->sum_exec_runtime += delta_exec;

                return delta_exec;
            }
            if (likely(delta_exec > 0)) {
                update_curr_task(curr, delta_exec) {
                    account_group_exec_runtime(p, delta_exec) {
                        struct thread_group_cputimer *cputimer = get_running_cputimer(tsk);
                        if (!cputimer)
                            return;
                        atomic64_add(ns, &cputimer->cputime_atomic.sum_exec_runtime);
                    }
                    cgroup_account_cputime(p, delta_exec);
                    if (p->dl_server)
                        dl_server_update(p->dl_server, delta_exec);
                }
            }

            return delta_exec;
        }
        if (unlikely(delta_exec <= 0))
            return;

        if (!rt_bandwidth_enabled())
            return;

        for_each_sched_rt_entity(rt_se) {
            struct rt_rq *rt_rq = rt_rq_of_se(rt_se);
            int exceeded;

            if (sched_rt_runtime(rt_rq) != RUNTIME_INF) {
                raw_spin_lock(&rt_rq->rt_runtime_lock);
                rt_rq->rt_time += delta_exec;
                exceeded = sched_rt_runtime_exceeded(rt_rq) {
                    u64 runtime = sched_rt_runtime(rt_rq);

                    if (rt_rq->rt_throttled)
                        return rt_rq_throttled(rt_rq);

                    if (runtime >= sched_rt_period(rt_rq))
                        return 0;

                    balance_runtime(rt_rq);
                        --->
                    runtime = sched_rt_runtime(rt_rq);
                    if (runtime == RUNTIME_INF)
                        return 0;

                    if (rt_rq->rt_time > runtime) {
                        struct rt_bandwidth *rt_b = sched_rt_bandwidth(rt_rq);

                        if (likely(rt_b->rt_runtime)) {
                            rt_rq->rt_throttled = 1;
                            printk_deferred_once("sched: RT throttling activated\n");
                        } else {
                            /* In case we did anyway, make it go away,
                             * replenishment is a joke, since it will replenish us
                             * with exactly 0 ns. */
                            rt_rq->rt_time = 0;
                        }

                        if (rt_rq_throttled(rt_rq)) {
                            /* sched_rt_rq_enqueue at sched_rt_period_timer */
                            sched_rt_rq_dequeue(rt_rq) {
                                struct sched_rt_entity *rt_se;
                                int cpu = cpu_of(rq_of_rt_rq(rt_rq));

                                rt_se = rt_rq->tg->rt_se[cpu];

                                if (!rt_se) {
                                    dequeue_top_rt_rq(rt_rq, rt_rq->rt_nr_running);
                                    /* Kick cpufreq (see the comment in kernel/sched/sched.h). */
                                    cpufreq_update_util(rq_of_rt_rq(rt_rq), 0);
                                }
                                else if (on_rt_rq(rt_se))
                                    dequeue_rt_entity(rt_se, 0);
                            }
                            return 1;
                        }
                    }

                    return 0;
                }
                if (exceeded)
                    resched_curr(rq);
                raw_spin_unlock(&rt_rq->rt_runtime_lock);
                if (exceeded) {
                    do_start_rt_bandwidth(sched_rt_bandwidth(rt_rq)) {
                        if (!rt_b->rt_period_active) {
                            rt_b->rt_period_active = 1;
                            hrtimer_forward_now(&rt_b->rt_period_timer, ns_to_ktime(0));
                            hrtimer_start_expires(&rt_b->rt_period_timer,
                                        HRTIMER_MODE_ABS_PINNED_HARD);
                        }
                    }
                }
            }
        }
    }
    update_rt_rq_load_avg(rq_clock_pelt(rq), rq, 1);

    watchdog(rq, p);

    if (p->policy != SCHED_RR)
        return;

    if (--p->rt.time_slice)
        return;

    p->rt.time_slice = sched_rr_timeslice;

    for_each_sched_rt_entity(rt_se) {
        if (rt_se->run_list.prev != rt_se->run_list.next) {
            requeue_task_rt(rq, p, 0);
            resched_curr(rq);
            return;
        }
    }
}
```

## yield_task_rt

```c
requeue_task_rt(rq, rq->curr, 0) {
    struct sched_rt_entity *rt_se = &p->rt;
    struct rt_rq *rt_rq;

    for_each_sched_rt_entity(rt_se) {
        rt_rq = rt_rq_of_se(rt_se);
        requeue_rt_entity(rt_rq, rt_se, head) {
            if (on_rt_rq(rt_se)) {
                struct rt_prio_array *array = &rt_rq->active;
                struct list_head *queue = array->queue + rt_se_prio(rt_se);

                if (head)
                    list_move(&rt_se->run_list, queue);
                else
                    list_move_tail(&rt_se->run_list, queue);
            }
        }
    }
}
```

## prio_changed_rt

```c
prio_changed_rt(struct rq *rq, struct task_struct *p, int oldprio) {
    if (!task_on_rq_queued(p))
        return;

    if (task_current(rq, p)) {
        if (oldprio < p->prio) {
            rt_queue_pull_task(rq) {
                pull_rt_task(rq);
                    --->
            }
        }

        /* If there's a higher priority task waiting to run
         * then reschedule. */
        if (p->prio > rq->rt.highest_prio.curr) {
            resched_curr(rq);
        }
    } else {
        /* This task is not running, but if it is
         * greater than the current running task
         * then reschedule. */
        if (p->prio < rq->curr->prio)
            resched_curr(rq);
    }
}
```

## check_class_changed_rt

```c
rt_mutex_setprio()
    check_class_changed()

__sched_setscheduler()
    check_class_changed() {
        if (prev_class != p->sched_class) {
            if (prev_class->switched_from) {
                prev_class->switched_from(rq, p);
            }
            p->sched_class->switched_to(rq, p);
        } else if (oldprio != p->prio || dl_task(p)) {
            p->sched_class->prio_changed(rq, p, oldprio);
        }
    }
```

### switched_from_rt

```c
void switched_from_rt(struct rq *rq, struct task_struct *p)
{
    /* If there are other RT tasks then we will reschedule
     * and the scheduling of the other RT tasks will handle
     * the balancing. But if we are the last RT task
     * we may need to handle the pulling of RT tasks
     * now. */
    if (!task_on_rq_queued(p) || rq->rt.rt_nr_running)
        return;

    rt_queue_pull_task(rq);
}
```

### switched_to_rt

```c

/* When switching a task to RT, we may overload the runqueue
 * with RT tasks. In this case we try to push them off to
 * other runqueues. */
void switched_to_rt(struct rq *rq, struct task_struct *p)
{
    if (task_current(rq, p)) {
        update_rt_rq_load_avg(rq_clock_pelt(rq), rq, 0);
        return;
    }

    if (task_on_rq_queued(p)) {
        if (p->nr_cpus_allowed > 1 && rq->rt.overloaded) {
            rt_queue_push_tasks(rq);
        }
        if (p->prio < rq->curr->prio && cpu_online(cpu_of(rq))) {
            resched_curr(rq);
        }
    }
}
```

## balance_rt

* [hellokitty2 - 调度器34 - RT负载均衡](https://www.cnblogs.com/hellokitty2/p/15974333.html)

![](../images/kernel/proc-sched-balance_rt.png)

```c
put_prev_task_balance(struct rq *rq, struct task_struct *prev,
    struct rq_flags *rf)
{
    const struct sched_class *class;
    for_class_range(class, prev->sched_class, &idle_sched_class) {
        if (class->balance(rq, prev, rf)) {
            break;
        }
    }

    put_prev_task(rq, prev);
}

balance_rt(struct rq *rq, struct task_struct *p, struct rq_flags *rf)
    ret = need_pull_rt_task(rq, p) {
        return rq->online && rq->rt.highest_prio.curr > prev->prio;
    }
    if (!on_rt_rq(&p->rt) { return rt_se->on_rq; } && ret) {
        pull_rt_task(rq)
            --->
    }

    return sched_stop_runnable(rq) || sched_dl_runnable(rq) || sched_rt_runnable(rq);
```

### pull_rt_task

![](../images/kernel/proc-sched-rt-pull_rt_task.png)

---

![](../images/kernel/proc-sched-rt-plist.png)

```c
/* pull a task from other cpus to this_rq */
void pull_rt_task(struct rq *this_rq) {
    int this_cpu = this_rq->cpu, cpu;
    bool resched = false;
    struct task_struct *p, *push_task;
    struct rq *src_rq;
    int rt_overload_count = rt_overloaded(this_rq) {
        /* inc at enqueue_task_rt
         * dec at dequeue_pushable_task */
        return &rq->rd->rto_count;
    }

    if (likely(!rt_overload_count))
        return;

    smp_rmb();

    if (rt_overload_count == 1 &&  cpumask_test_cpu(this_rq->cpu, this_rq->rd->rto_mask))
        return;

    for_each_cpu(cpu, this_rq->rd->rto_mask) {
        if (this_cpu == cpu)
            continue;

        src_rq = cpu_rq(cpu);

        /* no need to pull task if this rq has higher prio */
        if (src_rq->rt.highest_prio.next >= this_rq->rt.highest_prio.curr)
            continue;

        push_task = NULL;
        p = pick_highest_pushable_task(src_rq, this_cpu) {
            struct plist_head *head = &rq->rt.pushable_tasks;
            struct task_struct *p;

            if (!has_pushable_tasks(rq))
                return NULL;

            plist_for_each_entry(p, head, pushable_tasks) {
                ret = pick_rt_task(rq, p, cpu) {
                    return (!task_on_cpu(rq, p) && cpumask_test_cpu(cpu, &p->cpus_mask));
                }
                if (ret)
                    return p;
            }

            return NULL;
        }

        if (p && (p->prio < this_rq->rt.highest_prio.curr)) {
            if (p->prio < src_rq->curr->prio)
                goto skip;

            if (is_migration_disabled(p)) {
                push_task = get_push_task(src_rq) {
                    struct task_struct *p = rq->curr;

                    if (rq->push_busy)
                        return NULL;

                    if (p->nr_cpus_allowed == 1)
                        return NULL;

                    if (p->migration_disabled)
                        return NULL;

                    rq->push_busy = true;
                    return get_task_struct(p);
                }
            } else {
                deactivate_task(src_rq, p, 0);
                set_task_cpu(p, this_cpu);
                activate_task(this_rq, p, 0);
                resched = true;
            }
        }
skip:
        if (push_task) {
            raw_spin_rq_unlock(this_rq);
            stop_one_cpu_nowait(src_rq->cpu, push_cpu_stop,
                        push_task, &src_rq->push_work);
            raw_spin_rq_lock(this_rq);
        }
    }

    if (resched)
        resched_curr(this_rq);
}
```

### push_rt_tasks

1. switched_to_rt

    ```c
    switched_to_rt() {
        if (p->nr_cpus_allowed > 1 && rq->rt.overloaded) {
            rt_queue_push_tasks(rq) {
                queue_balance_callback(push_rt_tasks);
            }
        }
    }
    ```

2. set_next_task_rt

    ```c
    set_next_task_rt() {
        rt_queue_push_tasks();
    }
    ```

3. try_to_wake_up

    ```c
    try_to_wake_up() {
        p->sched_class->task_woken(rq, p) {
            void task_woken_rt(struct rq *rq, struct task_struct *p) {
                if (need_to_push)
                    push_rt_tasks(rq);
            }
        }
    }
    ```

```c
/* If the current CPU has more than one RT task, see if the non
 * running task can migrate over to a CPU that is running a task
 * of lesser priority. */
push_rt_tasks()
    push_rt_task(rq, false/*pull*/) {
        if (!rq->rt.overloaded)
            return 0;

        next_task = pick_next_pushable_task(rq) {
            return plist_first_entry(&rq->rt.pushable_tasks,
                struct task_struct, pushable_tasks);
        }
        if (!next_task)
            return 0;

    retry:
        if (unlikely(next_task->prio < rq->curr->prio)) {
            resched_curr(rq);
            return 0;
        }

        if (is_migration_disabled(next_task)) {
            struct task_struct *push_task = NULL;
            int cpu;

            if (!pull || rq->push_busy)
                return 0;

            if (rq->curr->sched_class != &rt_sched_class)
                return 0;

            cpu = find_lowest_rq(rq->curr);
                --->
            if (cpu == -1 || cpu == rq->cpu)
                return 0;

            push_task = get_push_task(rq);
            if (push_task) {
                raw_spin_rq_unlock(rq);
                stop_one_cpu_nowait(rq->cpu, push_cpu_stop, push_task, &rq->push_work) {

                }
                raw_spin_rq_lock(rq);
            }

            return 0;
        }

        if (WARN_ON(next_task == rq->curr))
            return 0;

        /* We might release rq lock */
        get_task_struct(next_task);

        /* find_lock_lowest_rq locks the rq if found */
        lowest_rq = find_lock_lowest_rq(next_task, rq) {
            struct rq *lowest_rq = NULL;
            int tries;
            int cpu;

            for (tries = 0; tries < RT_MAX_TRIES; tries++) {
                cpu = find_lowest_rq(task);
                    --->
                if ((cpu == -1) || (cpu == rq->cpu))
                    break;

                lowest_rq = cpu_rq(cpu);

                if (lowest_rq->rt.highest_prio.curr <= task->prio) {
                    lowest_rq = NULL;
                    break;
                }

                /* if the prio of this runqueue changed, try again */
                if (double_lock_balance(rq, lowest_rq)) {
                    if (unlikely(task_rq(task) != rq
                        || !cpumask_test_cpu(lowest_rq->cpu, &task->cpus_mask)
                        || task_on_cpu(rq, task)
                        || !rt_task(task)
                        || is_migration_disabled(task)
                        || !task_on_rq_queued(task))) {

                        double_unlock_balance(rq, lowest_rq);
                        lowest_rq = NULL;
                        break;
                    }
                }

                /* If this rq is still suitable use it. */
                if (lowest_rq->rt.highest_prio.curr > task->prio)
                    break;

                /* try again */
                double_unlock_balance(rq, lowest_rq);
                lowest_rq = NULL;
            }

            return lowest_rq;
        }
        if (!lowest_rq) {
            struct task_struct *task;

            task = pick_next_pushable_task(rq);
            if (task == next_task) {
                goto out;
            }

            if (!task)
                /* No more tasks, just exit */
                goto out;

            put_task_struct(next_task);
            next_task = task;
            goto retry;
        }

        deactivate_task(rq, next_task, 0) {
            p->on_rq = (flags & DEQUEUE_SLEEP) ? 0 : TASK_ON_RQ_MIGRATING;
            dequeue_task(rq, p, flags) {
                if (sched_core_enabled(rq)) {
                    sched_core_dequeue(rq, p, flags);
                }

                if (!(flags & DEQUEUE_NOCLOCK))
                    update_rq_clock(rq);

                if (!(flags & DEQUEUE_SAVE)) {
                    sched_info_dequeue(rq, p);
                    psi_dequeue(p, flags & DEQUEUE_SLEEP);
                }

                uclamp_rq_dec(rq, p);
                p->sched_class->dequeue_task(rq, p, flags);
            }
        }

        set_task_cpu(next_task, lowest_rq->cpu) {
            if (task_cpu(p) != new_cpu) {
                if (p->sched_class->migrate_task_rq) {
                    p->sched_class->migrate_task_rq(p, new_cpu);
                }
                p->se.nr_migrations++;
                rseq_migrate(p);
                sched_mm_cid_migrate_from(p);
                perf_event_task_migrate(p);
            }

            __set_task_cpu(p, new_cpu) {
                set_task_rq(p, cpu) {
                    struct task_group *tg = task_group(p);

                    set_task_rq_fair(&p->se, p->se.cfs_rq, tg->cfs_rq[cpu]) {
                        p_last_update_time = cfs_rq_last_update_time(prev);
                        n_last_update_time = cfs_rq_last_update_time(next);

                        __update_load_avg_blocked_se(p_last_update_time, se);
                        se->avg.last_update_time = n_last_update_time;
                    }
                    p->se.cfs_rq = tg->cfs_rq[cpu];
                    p->se.parent = tg->se[cpu];
                    p->se.depth = tg->se[cpu] ? tg->se[cpu]->depth + 1 : 0;

                    p->rt.rt_rq  = tg->rt_rq[cpu];
                    p->rt.parent = tg->rt_se[cpu];
                }
                WRITE_ONCE(task_thread_info(p)->cpu, cpu);
                p->wake_cpu = cpu;
            }
        }
        activate_task(lowest_rq, next_task, 0) {
            if (task_on_rq_migrating(p))
                flags |= ENQUEUE_MIGRATED;
            if (flags & ENQUEUE_MIGRATED)
                sched_mm_cid_migrate_to(rq, p);

            enqueue_task(rq, p, flags) {
                if (!(flags & ENQUEUE_NOCLOCK))
                    update_rq_clock(rq);

                if (!(flags & ENQUEUE_RESTORE)) {
                    sched_info_enqueue(rq, p);
                    psi_enqueue(p, (flags & ENQUEUE_WAKEUP) && !(flags & ENQUEUE_MIGRATED));
                }

                uclamp_rq_inc(rq, p);
                p->sched_class->enqueue_task(rq, p, flags);

                if (sched_core_enabled(rq))
                    sched_core_enqueue(rq, p);
            }

            p->on_rq = TASK_ON_RQ_QUEUED;
        }
        resched_curr(lowest_rq);
        ret = 1;

        double_unlock_balance(rq, lowest_rq);
    out:
        put_task_struct(next_task);

        return ret;
    }
}
```

# SCHED_CFS

![](../images/kernel/proc-sched-cfs.png)

```c
struct sched_entity {
    struct load_weight      load;
    struct rb_node          run_node;
    /* link all tsk in a list of cfs_rq */
    struct list_head        group_node;
    unsigned int            on_rq;

    u64                     exec_start;
    u64                     sum_exec_runtime;
    u64                     vruntime;
    u64                     prev_sum_exec_runtime;
    u64                     deadline;
    u64                     min_vruntime;
    s64                     vlag;
    u64                     slice;

    u64                     nr_migrations;

#ifdef CONFIG_FAIR_GROUP_SCHED
    int                     depth;
    struct sched_entity     *parent;
    /* rq on which this entity is (to be) queued: */
    struct cfs_rq           *cfs_rq;
    /* rq "owned" by this entity/group: */
    struct cfs_rq           *my_q;
    /* for task se, its task weight
     * for group se, its my_q->h_nr_running */
    unsigned long           runnable_weight;
#endif

#ifdef CONFIG_SMP
    struct sched_avg        avg;
#endif
};

struct cfs_rq {
    struct load_weight      load;
    unsigned int            nr_running;
    unsigned int            h_nr_running;      /* SCHED_{NORMAL,BATCH,IDLE} */
    unsigned int            idle_nr_running;   /* SCHED_IDLE */
    unsigned int            idle_h_nr_running; /* SCHED_IDLE */

    u64                     exec_clock;
    u64                     min_vruntime;
#ifdef CONFIG_SCHED_CORE
    unsigned int            forceidle_seq;
    u64                     min_vruntime_fi;
#endif

#ifndef CONFIG_64BIT
    u64                     min_vruntime_copy;
#endif

    struct rb_root_cached   tasks_timeline;

    /* 'curr' points to currently running entity on this cfs_rq.
     * It is set to NULL otherwise (i.e when none are currently running) */
    struct sched_entity    *curr;
    struct sched_entity    *next;
    struct sched_entity    *last;
    struct sched_entity    *skip;

#ifdef CONFIG_SMP
    /* CFS load tracking */
    struct sched_avg        avg;
#ifndef CONFIG_64BIT
    u64                     last_update_time_copy;
#endif

    struct {
        raw_spinlock_t      lock ____cacheline_aligned;
        int        nr;
        unsigned long       load_avg;
        unsigned long       util_avg;
        unsigned long       runnable_avg;
    } removed;

#ifdef CONFIG_FAIR_GROUP_SCHED
    unsigned long           tg_load_avg_contrib;
    long                    propagate;
    long                    prop_runnable_sum;

    /* h_load = weight * f(tg) */
    unsigned long           h_load;
    u64                     last_h_load_update;
    struct sched_entity     *h_load_next;
#endif /* CONFIG_FAIR_GROUP_SCHED */

#ifdef CONFIG_FAIR_GROUP_SCHED
    struct rq               *rq; /* CPU runqueue to which this cfs_rq is attached */

    int                     on_list;
    struct list_head        leaf_cfs_rq_list;
    struct task_group       *tg; /* group that "owns" this runqueue */

    /* Locally cached copy of our task_group's idle value */
    int                     idle;

#ifdef CONFIG_CFS_BANDWIDTH
    int            runtime_enabled;
    s64            runtime_remaining;
    u64            throttled_pelt_idle;
    u64            throttled_pelt_idle_copy;

    u64            throttled_clock;
    u64            throttled_clock_pelt;
    /* the total time while throttling */
    u64            throttled_clock_pelt_time;
    int            throttled;
    int            throttle_count;
    struct list_head    throttled_list;
    struct list_head    throttled_csd_list;
#endif
};
```

```c
DEFINE_SCHED_CLASS(fair) = {
    .enqueue_task       = enqueue_task_fair,
    .dequeue_task       = dequeue_task_fair,
    .yield_task         = yield_task_fair,
    .yield_to_task      = yield_to_task_fair,
    .wakeup_preempt     = check_preempt_wakeup_fair,
    .pick_next_task     = __pick_next_task_fair,
    .put_prev_task      = put_prev_task_fair,
    .set_next_task      = set_next_task_fair,
}
```

## enqueue_task_fair

![](../images/kernel/proc-sched-cfs-enqueue_task_fair.png)

```c
enqueue_task_fair(struct rq *rq, struct task_struct *p, int flags) {
    util_est_enqueue(&rq->cfs, p) {
        int enqueued  = cfs_rq->avg.util_est;
        enqueued += _task_util_est(p) {
            return READ_ONCE(p->se.avg.util_est) & ~UTIL_AVG_UNCHANGED;
        }
        WRITE_ONCE(cfs_rq->avg.util_est, enqueued);
    }

    /* walk up until meet a parent which is !on_rq */
    for_each_sched_entity(se) {
        if (se->on_rq)
            break;
        cfs_rq = cfs_rq_of(se);

        enqueue_entity(cfs_rq, se, flags) {
            bool curr = cfs_rq->curr == se;

            /* If we're the current task, we must renormalise before calling update_curr(). */
            if (curr) {
                place_entity(cfs_rq, se, flags);
            }

            update_curr(cfs_rq);

            /* When enqueuing a sched_entity, we must:
             *   - Update loads to have both entity and cfs_rq synced with now.
             *   - For group_entity, update its runnable_weight to reflect the new
             *     h_nr_running of its group cfs_rq.
             *   - For group_entity, update its weight to reflect the new share of
             *     its group cfs_rq
             *   - Add its new weight to cfs_rq->load.weight */
            update_load_avg(cfs_rq, se, UPDATE_TG | DO_ATTACH);
            se_update_runnable(se) {
                if (!entity_is_task(se))
                    se->runnable_weight = se->my_q->h_nr_running;
            }

            /* update task group entity shares */
            update_cfs_group(se);

            if (!curr) {
                /* udpate se: slice, vruntime, deadline */
                place_entity(cfs_rq, se, flags) {
                    u64 vslice;
                    u64 vruntime = avg_vruntime(cfs_rq) {
                        struct sched_entity *curr = cfs_rq->curr;
                        s64 avg = cfs_rq->avg_vruntime;
                        long load = cfs_rq->avg_load;

                        if (curr && curr->on_rq) {
                            unsigned long weight = scale_load_down(curr->load.weight);

                            avg += entity_key(cfs_rq, curr) * weight;
                            load += weight;
                        }

                        if (load) {
                            if (avg < 0) { /* sign flips effective floor / ceil */
                                avg -= (load - 1);
                            }
                            avg = div_s64(avg, load);
                        }

                        return cfs_rq->min_vruntime + avg;
                    }
                    s64 lag = 0;

                    se->slice = sysctl_sched_base_slice;
                    vslice = calc_delta_fair(se->slice, se);

                    if (sched_feat(PLACE_LAG) && cfs_rq->nr_running) {
                        struct sched_entity *curr = cfs_rq->curr;
                        unsigned long load;

                        lag = se->vlag;

                        load = cfs_rq->avg_load;
                        if (curr && curr->on_rq) {
                            load += scale_load_down(curr->load.weight);
                        }
                        lag *= load + scale_load_down(se->load.weight);
                        if (WARN_ON_ONCE(!load)) {
                            load = 1;
                        }
                        /* lag_i = S - s_i = w_i * (V - v_i)
                         * old_lag / old_load == new_lag / new_load
                         * old_lag * new_load == new_lag * old_load */
                        lag = div_s64(lag, load);
                    }

                    se->vruntime = vruntime - lag;

                    if (sched_feat(PLACE_DEADLINE_INITIAL) && (flags & ENQUEUE_INITIAL))
                        vslice /= 2;

                    /* EEVDF: vd_i = ve_i + r_i/w_i
                     * r_i: request time, w_i: weight(nice) */
                    se->deadline = se->vruntime + vslice;
                }
            }

            /* enqueue se weight */
            account_entity_enqueue(cfs_rq, se) {
                update_load_add(&cfs_rq->load, se->load.weight) {
                    lw->weight += inc;
                    lw->inv_weight = 0;
                }
                if (entity_is_task(se)) {
                    struct rq *rq = rq_of(cfs_rq);
                    account_numa_enqueue(rq, task_of(se));
                    list_add(&se->group_node, &rq->cfs_tasks);
                }
                cfs_rq->nr_running++;
                if (se_is_idle(se))
                    cfs_rq->idle_nr_running++;
            }

            /* Entity has migrated, no longer consider this task hot */
            if (flags & ENQUEUE_MIGRATED)
                se->exec_start = 0;

            check_schedstat_required();
            update_stats_enqueue_fair(cfs_rq, se, flags);

            if (!curr) {
                __enqueue_entity(cfs_rq, se) {
                    avg_vruntime_add(cfs_rq, se) {
                        unsigned long weight = scale_load_down(se->load.weight);
                        s64 key = entity_key(cfs_rq, se) {
                            return (s64)(se->vruntime - cfs_rq->min_vruntime);
                        }

                        cfs_rq->avg_vruntime += key * weight;
                        cfs_rq->avg_load += weight;
                    }
                    se->min_vruntime = se->deadline;

                    RB_DECLARE_CALLBACKS(static, min_vruntime_cb, struct sched_entity,
                        run_node, min_vruntime, min_vruntime_update);
                    rb_add_augmented_cached(&se->run_node, &cfs_rq->tasks_timeline,
                                __entity_less, &min_vruntime_cb);

                    /* se->min_vruntime = min(se->deadline, left->min_vruntime, right->min_vruntime) */
                    bool min_vruntime_update(struct sched_entity *se, bool exit) {
                        u64 old_min_vruntime = se->min_vruntime;
                        struct rb_node *node = &se->run_node;

                        se->min_vruntime = se->deadline;
                        __min_vruntime_update(se, node->rb_right);
                        __min_vruntime_update(se, node->rb_left) {
                            if (node) {
                                struct sched_entity *rse = __node_2_se(node);
                                if (deadline_gt(min_vruntime, se, rse))
                                    se->min_vruntime = rse->min_vruntime;
                            }
                        }

                        return se->min_vruntime == old_min_vruntime;
                    }
                }
            }
            se->on_rq = 1;

            if (cfs_rq->nr_running == 1) {
                check_enqueue_throttle(cfs_rq);
                if (!throttled_hierarchy(cfs_rq)) {
                    list_add_leaf_cfs_rq(cfs_rq);
                } else {
        #ifdef CONFIG_CFS_BANDWIDTH
                    struct rq *rq = rq_of(cfs_rq);

                    if (cfs_rq_throttled(cfs_rq) && !cfs_rq->throttled_clock)
                        cfs_rq->throttled_clock = rq_clock(rq);
                    if (!cfs_rq->throttled_clock_self)
                        cfs_rq->throttled_clock_self = rq_clock(rq);
        #endif
                }
            }
        }

        cfs_rq->h_nr_running++;
        cfs_rq->idle_h_nr_running += idle_h_nr_running;

        if (cfs_rq_is_idle(cfs_rq))
            idle_h_nr_running = 1;

        if (cfs_rq_throttled(cfs_rq))
            goto enqueue_throttle;

        flags = ENQUEUE_WAKEUP;
    }

    /* continue to wake up the remaining parents */
    for_each_sched_entity(se) {
        cfs_rq = cfs_rq_of(se);

        update_load_avg(cfs_rq, se, UPDATE_TG);
        se_update_runnable(se);
        update_cfs_group(se);

        cfs_rq->h_nr_running++;
        cfs_rq->idle_h_nr_running += idle_h_nr_running;

        if (cfs_rq_is_idle(cfs_rq))
            idle_h_nr_running = 1;

        /* end evaluation on encountering a throttled cfs_rq */
        if (cfs_rq_throttled(cfs_rq))
            goto enqueue_throttle;
    }

    add_nr_running(rq, 1);

enqueue_throttle:
    assert_list_leaf_cfs_rq(rq);

    hrtick_update(rq);
}
```

## dequeue_task_fair

```c
dequeue_task_fair() {
    util_est_dequeue()

    for_each_sched_entity() {
        dequeue_entity() {
            update_curr()
            update_load_avg()
            update_entity_lag()
            __dequeue_entity() {
                rb_erase_augmented_cached()
                avg_vruntime_sub()
            }
            account_entity_dequeue() {
                update_load_sub()
            }
            return_cfs_rq_runtime()
            update_cfs_group()
            update_min_vruntime()
        }
    }

    for_each_sched_entity() {
        update_load_avg()
        se_update_runnable()
        update_cfs_group()
    }

    sub_nr_running()
    util_est_update()
    hrtick_update()
}
```

```c
dequeue_task_fair(struct rq *rq, struct task_struct *p, int flags) {
    struct cfs_rq *cfs_rq;
    struct sched_entity *se = &p->se;
    int task_sleep = flags & DEQUEUE_SLEEP;
    int idle_h_nr_running = task_has_idle_policy(p);
    bool was_sched_idle = sched_idle_rq(rq);

    util_est_dequeue(&rq->cfs, p) {
        int enqueued  = cfs_rq->avg.util_est;
        enqueued -= min_t(unsigned int, enqueued, _task_util_est(p) {
            return READ_ONCE(p->se.avg.util_est) & ~UTIL_AVG_UNCHANGED
        });
        WRITE_ONCE(cfs_rq->avg.util_est, enqueued);
    }

    for_each_sched_entity(se) {
        cfs_rq = cfs_rq_of(se);
        dequeue_entity(cfs_rq, se, flags) {
            int action = UPDATE_TG;

            if (entity_is_task(se) && task_on_rq_migrating(task_of(se)))
                action |= DO_DETACH;

            update_curr(cfs_rq);
            update_load_avg(cfs_rq, se, action);

            update_stats_dequeue_fair(cfs_rq, se, flags);
            clear_buddies(cfs_rq, se);

            update_entity_lag(cfs_rq, se) {
                s64 lag, limit;

                lag = avg_vruntime(cfs_rq) - se->vruntime;
                limit = calc_delta_fair(max_t(u64, 2*se->slice, TICK_NSEC), se);
                se->vlag = clamp(lag, -limit, limit);
            }

            if (se != cfs_rq->curr) {
                __dequeue_entity(cfs_rq, se) {
                    rb_erase_augmented_cached(&se->run_node, &cfs_rq->tasks_timeline, &min_vruntime_cb);
                    avg_vruntime_sub(cfs_rq, se);
                }
            }
            se->on_rq = 0;
            /* dequeue se weight */
            account_entity_dequeue(cfs_rq, se) {
                update_load_sub(&cfs_rq->load, se->load.weight) {
                    lw->weight -= dec;
                    lw->inv_weight = 0;
                }
                if (entity_is_task(se)) {
                    account_numa_dequeue(rq_of(cfs_rq), task_of(se));
                    list_del_init(&se->group_node);
                }
                cfs_rq->nr_running--;
                if (se_is_idle(se))
                    cfs_rq->idle_nr_running--;
            }

            /* return excess runtime on last dequeue */
            return_cfs_rq_runtime(cfs_rq) {
                struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);
                s64 slack_runtime = cfs_rq->runtime_remaining - min_cfs_rq_runtime;

                if (slack_runtime <= 0)
                    return;

                raw_spin_lock(&cfs_b->lock);
                if (cfs_b->quota != RUNTIME_INF) {
                    cfs_b->runtime += slack_runtime;

                    if (cfs_b->runtime > sched_cfs_bandwidth_slice()
                        && !list_empty(&cfs_b->throttled_cfs_rq)) {

                        start_cfs_slack_bandwidth(cfs_b);
                    }
                }
                raw_spin_unlock(&cfs_b->lock);

                /* even if it's not valid for return we don't want to try again */
                cfs_rq->runtime_remaining -= slack_runtime;
            }

            update_cfs_group(se);

            if ((flags & (DEQUEUE_SAVE | DEQUEUE_MOVE)) != DEQUEUE_SAVE)
                update_min_vruntime(cfs_rq);

            if (cfs_rq->nr_running == 0)
                update_idle_cfs_rq_clock_pelt(cfs_rq);
        }

        cfs_rq->h_nr_running--;
        cfs_rq->idle_h_nr_running -= idle_h_nr_running;

        if (cfs_rq_is_idle(cfs_rq))
            idle_h_nr_running = 1;

        /* end evaluation on encountering a throttled cfs_rq */
        if (cfs_rq_throttled(cfs_rq))
            goto dequeue_throttle;

        /* Don't dequeue parent if it has other entities besides us */
        if (cfs_rq->load.weight) {
            /* Avoid re-evaluating load for this entity: */
            se = parent_entity(se);

            if (task_sleep && se && !throttled_hierarchy(cfs_rq))
                set_next_buddy(se);
            break;
        }
        flags |= DEQUEUE_SLEEP;
    }

    for_each_sched_entity(se) {
        cfs_rq = cfs_rq_of(se);

        update_load_avg(cfs_rq, se, UPDATE_TG);
        se_update_runnable(se);
        update_cfs_group(se);

        cfs_rq->h_nr_running--;
        cfs_rq->idle_h_nr_running -= idle_h_nr_running;

        if (cfs_rq_is_idle(cfs_rq))
            idle_h_nr_running = 1;

        /* end evaluation on encountering a throttled cfs_rq */
        if (cfs_rq_throttled(cfs_rq))
            goto dequeue_throttle;

    }

    /* At this point se is NULL and we are at root level*/
    sub_nr_running(rq, 1);

    /* balance early to pull high priority tasks */
    if (unlikely(!was_sched_idle && sched_idle_rq(rq)))
        rq->next_balance = jiffies;

dequeue_throttle:
    util_est_update(&rq->cfs/*cfs_rq*/, p, task_sleep) {
        if (!task_sleep)
            return;

        ewma = READ_ONCE(p->se.avg.util_est);

        /* If the PELT values haven't changed since enqueue time,
         * skip the util_est update. */
        if (ewma & UTIL_AVG_UNCHANGED)
            return;

        dequeued = task_util(p) {
            return READ_ONCE(p->se.avg.util_avg);
        }

        /* Reset EWMA on utilization increases, the moving average is used only
         * to smooth utilization decreases. */
        if (ewma <= dequeued) {
            ewma = dequeued;
            goto done;
        }

        last_ewma_diff = ewma - dequeued;
        if (last_ewma_diff < UTIL_EST_MARGIN)
            goto done;

        /* To avoid overestimation of actual task utilization, skip updates if
         * we cannot grant there is idle time in this CPU. */
        if (dequeued > arch_scale_cpu_capacity(cpu_of(rq_of(cfs_rq))))
            return;

        /* To avoid underestimate of task utilization, skip updates of EWMA if
         * we cannot grant that thread got all CPU time it wanted. */
        if ((dequeued + UTIL_EST_MARGIN) < task_runnable(p))
            goto done;


        ewma <<= UTIL_EST_WEIGHT_SHIFT;
        ewma  -= last_ewma_diff;
        ewma >>= UTIL_EST_WEIGHT_SHIFT;
    done:
        ewma |= UTIL_AVG_UNCHANGED;
        WRITE_ONCE(p->se.avg.util_est, ewma);
    }
    hrtick_update(rq);
}
```

## put_prev_task_fair

```c
put_prev_task_fair(struct rq *rq, struct task_struct *prev)
    struct sched_entity *se = &prev->se;
    struct cfs_rq *cfs_rq;

    for_each_sched_entity(se) {
        cfs_rq = cfs_rq_of(se);
        put_prev_entity(cfs_rq, se) {
            if (prev->on_rq) {
                update_curr(cfs_rq);
                    --->
            }

            check_cfs_rq_runtime(cfs_rq) {
                if (!cfs_bandwidth_used())
                    return false;
                if (likely(!cfs_rq->runtime_enabled || cfs_rq->runtime_remaining > 0))
                    return false;
                if (cfs_rq_throttled(cfs_rq))
                    return true;

                return throttle_cfs_rq(cfs_rq);
            }
            check_spread(cfs_rq, prev);

            if (prev->on_rq) {
                update_stats_wait_start_fair(cfs_rq, prev);
                __enqueue_entity(cfs_rq, prev);
                update_load_avg(cfs_rq, prev, 0);
            }
            cfs_rq->curr = NULL;
        }
    }
```

## pick_next_task_fair

![](../images/kernel/proc-sched-pick_next_task_fair.png)

```c
pick_next_task_fair(struct rq *rq, struct task_struct *prev, struct rq_flags *rf)

again:
    if (!sched_fair_runnable(rq))
        goto idle;

    if (!prev || prev->sched_class != &fair_sched_class)
        goto simple;

    do {
        struct sched_entity *curr = cfs_rq->curr;

        if (curr) {
            if (curr->on_rq)
                update_curr(cfs_rq);
            else
                curr = NULL;

            /* do the throttle and dequeue its entity in the parent(s) */
            if (unlikely(check_cfs_rq_runtime(cfs_rq))) {
                cfs_rq = &rq->cfs;

                if (!cfs_rq->nr_running)
                    goto idle;

                goto simple;
            }
        }

        se = pick_next_entity(cfs_rq, curr);
        cfs_rq = group_cfs_rq(se);
    } while (cfs_rq);

    p = task_of(se);

    /* Since we haven't yet done put_prev_entity and if the selected task
     * is a different task than we started out with, try and touch the
     * least amount of cfs_rqs. */
    if (prev != p) {
        struct sched_entity *pse = &prev->se;

        while (!(cfs_rq = is_same_group(se, pse))) {
            int se_depth = se->depth;
            int pse_depth = pse->depth;

            if (se_depth <= pse_depth) {
                put_prev_entity(cfs_rq_of(pse), pse);
                    --->
                pse = parent_entity(pse);
            }
            if (se_depth >= pse_depth) {
                set_next_entity(cfs_rq_of(se), se);
                se = parent_entity(se);
            }
        }

        put_prev_entity(cfs_rq, pse);
            --->
        set_next_entity(cfs_rq, se);
    }

    goto done;

simple:

    if (prev) {
        put_prev_task(rq, prev)
            --->
    }

    do {
        se = pick_next_entity(cfs_rq) {
            ret = entity_eligible(cfs_rq, cfs_rq->next/*se*/) {
                return vruntime_eligible(cfs_rq, se->vruntime) {
                    struct sched_entity *curr = cfs_rq->curr;
                    s64 avg = cfs_rq->avg_vruntime;
                    long load = cfs_rq->avg_load;

                    if (curr && curr->on_rq) {
                        unsigned long weight = scale_load_down(curr->load.weight);

                        avg += entity_key(cfs_rq, curr) * weight;
                        load += weight;
                    }

                    return avg >= (s64)(vruntime - cfs_rq->min_vruntime) * load;
                }
            }

            if (sched_feat(NEXT_BUDDY) && cfs_rq->next && ret)
                return cfs_rq->next;

            return pick_eevdf(cfs_rq) {
                /* the root note of the rbt */
                struct rb_node *node = cfs_rq->tasks_timeline.rb_root.rb_node;
                struct sched_entity *curr = cfs_rq->curr;
                struct sched_entity *best = NULL;

                if (curr && (!curr->on_rq || !entity_eligible(cfs_rq, curr)))
                    curr = NULL;

                if (sched_feat(RUN_TO_PARITY) && curr && curr->vlag == curr->deadline)
                    return curr;

                while (node) {
                    struct sched_entity *se = __node_2_se(node);

                    if (!entity_eligible(cfs_rq, se)) {
                        node = node->rb_left;
                        continue;
                    }

                    if (!best || deadline_gt(deadline, best, se)) {
                        best = se;
                        if (best->deadline == best->min_vruntime)
                            break;
                    }

                    if (node->rb_left &&
                        __node_2_se(node->rb_left)->min_vruntime == se->min_vruntime) {
                        node = node->rb_left;
                        continue;
                    }

                    node = node->rb_right;
                }

                if (!best || (curr && deadline_gt(deadline, best, curr)))
                    best = curr;

                if (unlikely(!best)) {
                    /* pick the left most node */
                    struct sched_entity *left = __pick_first_entity(cfs_rq);
                    if (left) {
                        return left;
                    }
                }

                return best;
            }
        }

        set_next_entity(cfs_rq, se)
            --->
        cfs_rq = group_cfs_rq(se);
    } while (cfs_rq);

    p = task_of(se);

done: __maybe_unused;

    list_move(&p->se.group_node, &rq->cfs_tasks);

    if (hrtick_enabled_fair(rq))
        hrtick_start_fair(rq, p);

    update_misfit_status(p, rq);

    return p;

idle:
    if (!rf)
        return NULL;

    new_tasks = sched_balance_newidle(rq, rf);
        --->

    if (new_tasks < 0)
        return RETRY_TASK;

    if (new_tasks > 0)
        goto again;

    update_idle_rq_clock_pelt(rq);
```

## set_next_task_fair

```c
set_next_task_fair(struct rq *rq, struct task_struct *p, bool first)
    struct sched_entity *se = &p->se;

    if (task_on_rq_queued(p)) {
        list_move(&se->group_node, &rq->cfs_tasks);
    }

    for_each_sched_entity(se) {
        struct cfs_rq *cfs_rq = cfs_rq_of(se);

        set_next_entity(cfs_rq, se) {
            clear_buddies(cfs_rq, se) {
                if (cfs_rq->next == se) {
                    __clear_buddies_next(se) {
                        for_each_sched_entity(se) {
                            struct cfs_rq *cfs_rq = cfs_rq_of(se);
                            if (cfs_rq->next != se)
                                break;

                            cfs_rq->next = NULL;
                        }
                    }
                }
            }

            /* 'current' is not kept within the tree. */
            if (se->on_rq) {
                update_stats_wait_end_fair(cfs_rq, se);
                __dequeue_entity(cfs_rq, se);
                update_load_avg(cfs_rq, se, UPDATE_TG);
                /* HACK, stash a copy of deadline at the point of pick in vlag,
                 * which isn't used until dequeue. */
                se->vlag = se->deadline;
            }

            update_stats_curr_start(cfs_rq, se);
            cfs_rq->curr = se;

            se->prev_sum_exec_runtime = se->sum_exec_runtime;
        }
        /* ensure bandwidth has been allocated on our new cfs_rq
         * throttle task if cant allot runtime */
        account_cfs_rq_runtime(cfs_rq, 0);
    }
```

## select_task_rq_fair

* [hellokitty2 - 调度器24 - CFS任务选核](https://www.cnblogs.com/hellokitty2/p/15750931.html)

```c
try_to_wake_up() { /* wake up select */
    select_task_rq(p, p->wake_cpu, SD_BALANCE_WAKE, wake_flags);
}

wake_up_new_task() { /* fork select */
    select_task_rq(p, task_cpu(p), SD_BALANCE_FORK, 0);
}

sched_exec() { /* exec select */
    select_task_rq(p, task_cpu(p), SD_BALANCE_EXEC, 0);
}
```

```c
select_task_rq_fair(struct task_struct *p, int prev_cpu, int wake_flags)
    int sync = (wake_flags & WF_SYNC) && !(current->flags & PF_EXITING);
    struct sched_domain *tmp, *sd = NULL;
    int cpu = smp_processor_id();
    int new_cpu = prev_cpu;
    int want_affine = 0;
    /* SD_flags and WF_flags share the first nibble */
    int sd_flag = wake_flags & 0xF;

    lockdep_assert_held(&p->pi_lock);
    if (wake_flags & WF_TTWU) {
/* 1. update wake flips */
        record_wakee(p) {
            if (time_after(jiffies, current->wakee_flip_decay_ts + HZ)) {
                current->wakee_flips >>= 1;
                current->wakee_flip_decay_ts = jiffies;
            }

            if (current->last_wakee != p) {
                current->last_wakee = p;
                current->wakee_flips++;
            }
        }

        if (sched_energy_enabled()) {
            new_cpu = find_energy_efficient_cpu(p, prev_cpu);
            if (new_cpu >= 0)
                return new_cpu;
            new_cpu = prev_cpu;
        }
/* 2. calc wake affine */
        /* Detect M:N waker/wakee relationships via a switching-frequency heuristic. */
        ret = wake_wide(p) {
            unsigned int master = current->wakee_flips;
            unsigned int slave = p->wakee_flips;
            /* nr of cpus which share the llc */
            int factor = __this_cpu_read(sd_llc_size);

            if (master < slave)
                swap(master, slave);
            if (slave < factor || master < slave * factor)
                return 0;
            return 1;
        }
        want_affine = !ret && cpumask_test_cpu(cpu, p->cpus_ptr);
    }

    rcu_read_lock();
    for_each_domain(cpu, tmp) {
        if (want_affine && (tmp->flags & SD_WAKE_AFFINE)
            && cpumask_test_cpu(prev_cpu, sched_domain_span(tmp))) {

            if (cpu != prev_cpu) {
                new_cpu = wake_affine(tmp, p, cpu/*this_cpu*/, prev_cpu, sync) {
                    int target = nr_cpumask_bits;
/* 3. wake affine idle */
                    /* only considers 'now', it check if the waking CPU is
                     * cache-affine and is (or    will be) idle */
                    if (sched_feat(WA_IDLE)) {
                        target = wake_affine_idle(this_cpu, prev_cpu, sync) {
                            if (available_idle_cpu(this_cpu) && cpus_share_cache(this_cpu, prev_cpu)) {
                                return available_idle_cpu(prev_cpu) ? prev_cpu : this_cpu;
                            }

                            if (sync && cpu_rq(this_cpu)->nr_running == 1) {
                                return this_cpu;
                            }

                            if (available_idle_cpu(prev_cpu)) {
                                return prev_cpu;
                            }

                            return nr_cpumask_bits;
                        }
                    }
/* 4. wake affine weight */
                    /* considers the weight to reflect the average
                     * scheduling latency of the CPUs. This seems to work
                     * for the overloaded case. */
                    if (sched_feat(WA_WEIGHT) && target == nr_cpumask_bits) {
                        target = wake_affine_weight(sd, p, this_cpu, prev_cpu, sync) {
                            s64 this_eff_load, prev_eff_load;
                            unsigned long task_load;

                            this_eff_load = cpu_load(cpu_rq(this_cpu));

                            if (sync) {
                                unsigned long current_load = task_h_load(current);

                                if (current_load > this_eff_load)
                                    return this_cpu;

                                this_eff_load -= current_load;
                            }

                            task_load = task_h_load(p);

                            this_eff_load += task_load;
                            if (sched_feat(WA_BIAS))
                                this_eff_load *= 100;
                            this_eff_load *= capacity_of(prev_cpu);

                            prev_eff_load = cpu_load(cpu_rq(prev_cpu));
                            prev_eff_load -= task_load;
                            if (sched_feat(WA_BIAS))
                                prev_eff_load *= 100 + (sd->imbalance_pct - 100) / 2;
                            prev_eff_load *= capacity_of(this_cpu);

                            if (sync)
                                prev_eff_load += 1;

                            return this_eff_load < prev_eff_load ? this_cpu : nr_cpumask_bits;
                        }
                    }

                    schedstat_inc(p->stats.nr_wakeups_affine_attempts);
                    if (target != this_cpu)
                        return prev_cpu;

                    schedstat_inc(sd->ttwu_move_affine);
                    schedstat_inc(p->stats.nr_wakeups_affine);
                    return target;
                }
            }

            sd = NULL; /* Prefer wake_affine over balance flags */
            break;
        }

        if (tmp->flags & sd_flag)
            sd = tmp;
        else if (!want_affine)
            break;
    }
/* 5. find idlest cpu */
    if (unlikely(sd)) { /* Slow path, only true for WF_EXEC and WF_FORK */
        new_cpu = find_idlest_cpu(sd, p, cpu, prev_cpu, sd_flag)
            --->
    } else if (wake_flags & WF_TTWU) { /* Fast path */
        new_cpu = select_idle_sibling(p, prev_cpu, new_cpu)
            --->
    }
    rcu_read_unlock();

    return new_cpu;
```

### find_idlest_cpu
```c
new_cpu = find_idlest_cpu(sd, p, cpu, prev_cpu, sd_flag) {
    int new_cpu = cpu;

    if (!cpumask_intersects(sched_domain_span(sd), p->cpus_ptr))
        return prev_cpu;

    if (!(sd_flag & SD_BALANCE_FORK)) {
        sync_entity_load_avg(&p->se);
    }

    while (sd) {
        struct sched_group *group;
        struct sched_domain *tmp;
        int weight;

        if (!(sd->flags & sd_flag)) {
            sd = sd->child;
            continue;
        }

        group = find_idlest_group(sd, p, cpu)
            --->
        if (!group) {
            sd = sd->child;
            continue;
        }

        new_cpu = find_idlest_group_cpu(group, p, cpu)
            --->
        if (new_cpu == cpu) {
            sd = sd->child;
            continue;
        }

        cpu = new_cpu;
        weight = sd->span_weight;
        sd = NULL;
        for_each_domain(cpu, tmp) {
            if (weight <= tmp->span_weight)
                break;
            if (tmp->flags & sd_flag)
                sd = tmp;
        }
    }

    return new_cpu;
}
```

#### find_idlest_group

```c
static struct sched_group *
find_idlest_group(struct sched_domain *sd, struct task_struct *p, int this_cpu)
{
    struct sched_group *idlest = NULL, *local = NULL, *group = sd->groups;
    struct sg_lb_stats local_sgs, tmp_sgs;
    struct sg_lb_stats *sgs;
    unsigned long imbalance;
    struct sg_lb_stats idlest_sgs = {
        .avg_load = UINT_MAX,
        .group_type = group_overloaded,
    };

    do {
        int local_group;

        if (!cpumask_intersects(sched_group_span(group), p->cpus_ptr))
            continue;

        if (!sched_group_cookie_match(cpu_rq(this_cpu), p, group))
            continue;

        local_group = cpumask_test_cpu(this_cpu, sched_group_span(group));

        if (local_group) {
            sgs = &local_sgs;
            local = group;
        } else {
            sgs = &tmp_sgs;
        }

        update_sg_wakeup_stats(sd, group, sgs, p) {
            int i, nr_running;

            memset(sgs, 0, sizeof(*sgs));

            /* Assume that task can't fit any CPU of the group */
            if (sd->flags & SD_ASYM_CPUCAPACITY)
                sgs->group_misfit_task_load = 1;

            for_each_cpu(i, sched_group_span(group)) {
                struct rq *rq = cpu_rq(i);
                unsigned int local;

                /* compute CPU load without any contributions from *p */
                sgs->group_load += cpu_load_without(rq, p);
                sgs->group_util += cpu_util_without(i, p);
                sgs->group_runnable += cpu_runnable_without(rq, p);
                local = task_running_on_cpu(i, p);
                sgs->sum_h_nr_running += rq->cfs.h_nr_running - local;

                nr_running = rq->nr_running - local;
                sgs->sum_nr_running += nr_running;

                if (!nr_running && idle_cpu_without(i, p))
                    sgs->idle_cpus++;

                /* Check if task fits in the CPU */
                if (sd->flags & SD_ASYM_CPUCAPACITY
                    && sgs->group_misfit_task_load
                    && task_fits_cpu(p, i)) {

                    sgs->group_misfit_task_load = 0;
                }
            }

            sgs->group_capacity = group->sgc->capacity;
            sgs->group_weight = group->group_weight;
            sgs->group_type = group_classify(sd->imbalance_pct, group, sgs);

            if (sgs->group_type == group_fully_busy || sgs->group_type == group_overloaded)
                sgs->avg_load = (sgs->group_load * SCHED_CAPACITY_SCALE) / sgs->group_capacity;
        }

        if (!local_group && update_pick_idlest(idlest, &idlest_sgs, group, sgs)) {
            idlest = group;
            idlest_sgs = *sgs;
        }
    } while (group = group->next, group != sd->groups);

    if (!idlest)
        return NULL;

    /* The local group has been skipped because of CPU affinity */
    if (!local)
        return idlest;

    /* If the local group is idler than the selected idlest group
     * don't try and push the task. */
    if (local_sgs.group_type < idlest_sgs.group_type)
        return NULL;

    /* If the local group is busier than the selected idlest group
     * try and push the task. */
    if (local_sgs.group_type > idlest_sgs.group_type)
        return idlest;

    switch (local_sgs.group_type) {
    case group_overloaded:
    case group_fully_busy:

        /* Calculate allowed imbalance based on load */
        imbalance = scale_load_down(NICE_0_LOAD) * (sd->imbalance_pct-100) / 100;

        if ((sd->flags & SD_NUMA) && ((idlest_sgs.avg_load + imbalance) >= local_sgs.avg_load))
            return NULL;

        /* If the local group is less loaded than the selected
         * idlest group don't try and push any tasks. */
        if (idlest_sgs.avg_load >= (local_sgs.avg_load + imbalance))
            return NULL;

        if (100 * local_sgs.avg_load <= sd->imbalance_pct * idlest_sgs.avg_load)
            return NULL;
        break;

    case group_imbalanced:
    case group_asym_packing:
    case group_smt_balance:
        /* Those type are not used in the slow wakeup path */
        return NULL;

    case group_misfit_task:
        /* Select group with the highest max capacity */
        if (local->sgc->max_capacity >= idlest->sgc->max_capacity)
            return NULL;
        break;

    case group_has_spare:
#ifdef CONFIG_NUMA
        if (sd->flags & SD_NUMA) {
            int imb_numa_nr = sd->imb_numa_nr;
#ifdef CONFIG_NUMA_BALANCING
            int idlest_cpu;
            /* If there is spare capacity at NUMA, try to select
             * the preferred node */
            if (cpu_to_node(this_cpu) == p->numa_preferred_nid)
                return NULL;

            idlest_cpu = cpumask_first(sched_group_span(idlest));
            if (cpu_to_node(idlest_cpu) == p->numa_preferred_nid)
                return idlest;
#endif /* CONFIG_NUMA_BALANCING */
            /* Otherwise, keep the task close to the wakeup source
             * and improve locality if the number of running tasks
             * would remain below threshold where an imbalance is
             * allowed while accounting for the possibility the
             * task is pinned to a subset of CPUs. If there is a
             * real need of migration, periodic load balance will
             * take care of it. */
            if (p->nr_cpus_allowed != NR_CPUS) {
                struct cpumask *cpus = this_cpu_cpumask_var_ptr(select_rq_mask);

                cpumask_and(cpus, sched_group_span(local), p->cpus_ptr);
                imb_numa_nr = min(cpumask_weight(cpus), sd->imb_numa_nr);
            }

            imbalance = abs(local_sgs.idle_cpus - idlest_sgs.idle_cpus);
            if (!adjust_numa_imbalance(imbalance,
                           local_sgs.sum_nr_running + 1,
                           imb_numa_nr)) {
                return NULL;
            }
        }
#endif /* CONFIG_NUMA */

        /* Select group with highest number of idle CPUs. We could also
         * compare the utilization which is more stable but it can end
         * up that the group has less spare capacity but finally more
         * idle CPUs which means more opportunity to run task. */
        if (local_sgs.idle_cpus >= idlest_sgs.idle_cpus)
            return NULL;
        break;
    }

    return idlest;
}
```

#### find_idlest_group_cpu

```c
static int
find_idlest_group_cpu(struct sched_group *group, struct task_struct *p, int this_cpu)
{
    unsigned long load, min_load = ULONG_MAX;
    unsigned int min_exit_latency = UINT_MAX;
    u64 latest_idle_timestamp = 0;
    int least_loaded_cpu = this_cpu;
    int shallowest_idle_cpu = -1;
    int i;

    /* Check if we have any choice: */
    if (group->group_weight == 1)
        return cpumask_first(sched_group_span(group));

    /* Traverse only the allowed CPUs */
    for_each_cpu_and(i, sched_group_span(group), p->cpus_ptr) {
        struct rq *rq = cpu_rq(i);

        if (!sched_core_cookie_match(rq, p))
            continue;

        /* Runqueue only has SCHED_IDLE tasks enqueued */
        if (sched_idle_cpu(i))
            return i;

        if (available_idle_cpu(i)) {
            struct cpuidle_state *idle = idle_get_state(rq);
            if (idle && idle->exit_latency < min_exit_latency) {
                /* We give priority to a CPU whose idle state
                 * has the smallest exit latency irrespective
                 * of any idle timestamp. */
                min_exit_latency = idle->exit_latency;
                latest_idle_timestamp = rq->idle_stamp;
                shallowest_idle_cpu = i;
            } else if ((!idle || idle->exit_latency == min_exit_latency)
                && rq->idle_stamp > latest_idle_timestamp) {

                /* If equal or no active idle state, then
                 * the most recently idled CPU might have
                 * a warmer cache. */
                latest_idle_timestamp = rq->idle_stamp;
                shallowest_idle_cpu = i;
            }
        } else if (shallowest_idle_cpu == -1) {
            load = cpu_load(cpu_rq(i));
            if (load < min_load) {
                min_load = load;
                least_loaded_cpu = i;
            }
        }
    }

    return shallowest_idle_cpu != -1 ? shallowest_idle_cpu : least_loaded_cpu;
}
```

### select_idle_sibling

```c
new_cpu = select_idle_sibling(p, prev_cpu/*prev*/, new_cpu/*target*/) {
    bool has_idle_core = false;
    struct sched_domain *sd;
    unsigned long task_util, util_min, util_max;
    int i, recent_used_cpu;

    if (sched_asym_cpucap_active()) {
        sync_entity_load_avg(&p->se);
        task_util = task_util_est(p);
        util_min = uclamp_eff_value(p, UCLAMP_MIN);
        util_max = uclamp_eff_value(p, UCLAMP_MAX);
    }

    lockdep_assert_irqs_disabled();

    if ((available_idle_cpu(target) || sched_idle_cpu(target))
        && asym_fits_cpu(task_util, util_min, util_max, target)) {
        return target;
    }

    /* If the previous CPU is cache affine and idle, don't be stupid: */
    if (prev != target && cpus_share_cache(prev, target)
        && (available_idle_cpu(prev) || sched_idle_cpu(prev))
        && asym_fits_cpu(task_util, util_min, util_max, prev))
        return prev;

    if (is_per_cpu_kthread(current)
        && in_task()
        && prev == smp_processor_id()
        && this_rq()->nr_running <= 1
        && asym_fits_cpu(task_util, util_min, util_max, prev)) {
        return prev;
    }

    /* Check a recently used CPU as a potential idle candidate: */
    recent_used_cpu = p->recent_used_cpu;
    p->recent_used_cpu = prev;
    if (recent_used_cpu != prev
        && recent_used_cpu != target
        && cpus_share_cache(recent_used_cpu, target)
        && (available_idle_cpu(recent_used_cpu) || sched_idle_cpu(recent_used_cpu))
        && cpumask_test_cpu(recent_used_cpu, p->cpus_ptr)
        && asym_fits_cpu(task_util, util_min, util_max, recent_used_cpu)) {
        return recent_used_cpu;
    }

    /* For asymmetric CPU capacity systems, our domain of interest is
     * sd_asym_cpucapacity rather than sd_llc. */
    if (sched_asym_cpucap_active()) {
        sd = rcu_dereference(per_cpu(sd_asym_cpucapacity, target));
        if (sd) {
            i = select_idle_capacity(p, sd, target) {

            }
            return ((unsigned)i < nr_cpumask_bits) ? i : target;
        }
    }

    sd = rcu_dereference(per_cpu(sd_llc, target));
    if (!sd)
        return target;

    if (sched_smt_active()) {
        has_idle_core = test_idle_cores(target);
        if (!has_idle_core && cpus_share_cache(prev, target)) {
            i = select_idle_smt(p, prev) {
                int cpu;
                for_each_cpu_and(cpu, cpu_smt_mask(target), p->cpus_ptr) {
                    if (cpu == target)
                        continue;
                    if (available_idle_cpu(cpu) || sched_idle_cpu(cpu))
                        return cpu;
                }
                return -1;
            }
            if ((unsigned int)i < nr_cpumask_bits) {
                return i;
            }
        }
    }

    i = select_idle_cpu(p, sd, has_idle_core, target) {
        struct cpumask *cpus = this_cpu_cpumask_var_ptr(select_rq_mask);
        int i, cpu, idle_cpu = -1, nr = INT_MAX;
        struct sched_domain_shared *sd_share;

        cpumask_and(cpus, sched_domain_span(sd), p->cpus_ptr);

        if (sched_feat(SIS_UTIL)) {
            sd_share = rcu_dereference(per_cpu(sd_llc_shared, target));
            if (sd_share) {
                /* because !--nr is the condition to stop scan */
                nr = READ_ONCE(sd_share->nr_idle_scan) + 1;
                /* overloaded LLC is unlikely to have idle cpu/core */
                if (nr == 1)
                    return -1;
            }
        }

        if (static_branch_unlikely(&sched_cluster_active)) {
            struct sched_group *sg = sd->groups;

            if (sg->flags & SD_CLUSTER) {
                for_each_cpu_wrap(cpu, sched_group_span(sg), target + 1) {
                    if (!cpumask_test_cpu(cpu, cpus))
                        continue;

                    if (has_idle_core) {
                        i = select_idle_core(p, cpu, cpus, &idle_cpu);
                        if ((unsigned int)i < nr_cpumask_bits)
                            return i;
                    } else {
                        if (--nr <= 0)
                            return -1;
                        idle_cpu = __select_idle_cpu(cpu, p);
                        if ((unsigned int)idle_cpu < nr_cpumask_bits)
                            return idle_cpu;
                    }
                }
                cpumask_andnot(cpus, cpus, sched_group_span(sg));
            }
        }

        for_each_cpu_wrap(cpu, cpus, target + 1) {
            if (has_idle_core) {
                i = select_idle_core(p, cpu, cpus, &idle_cpu);
                if ((unsigned int)i < nr_cpumask_bits)
                    return i;

            } else {
                if (--nr <= 0)
                    return -1;
                idle_cpu = __select_idle_cpu(cpu, p);
                if ((unsigned int)idle_cpu < nr_cpumask_bits)
                    break;
            }
        }

        if (has_idle_core)
            set_idle_cores(target, false);

        return idle_cpu;
    }

    if ((unsigned)i < nr_cpumask_bits)
        return i;

    return target;
}
```

## wakeup_preempt_fair

```c
void check_preempt_wakeup_fair(struct rq *rq, struct task_struct *p, int wake_flags)
{
    struct task_struct *curr = rq->curr;
    struct sched_entity *se = &curr->se, *pse = &p->se;
    struct cfs_rq *cfs_rq = task_cfs_rq(curr);
    int next_buddy_marked = 0;
    int cse_is_idle, pse_is_idle;

    if (unlikely(se == pse))
        return;

    if (unlikely(throttled_hierarchy(cfs_rq_of(pse))))
        return;

    if (sched_feat(NEXT_BUDDY) && !(wake_flags & WF_FORK)) {
        set_next_buddy(pse);
        next_buddy_marked = 1;
    }

    if (test_tsk_need_resched(curr))
        return;

/* 1. cmp idle policy */
    /* Idle tasks are by definition preempted by non-idle tasks. */
    if (unlikely(task_has_idle_policy(curr))
        && likely(!task_has_idle_policy(p))) {

        goto preempt;
    }

    /* Batch and idle tasks do not preempt non-idle tasks (their preemption
     * is driven by the tick): */
    if (unlikely(p->policy != SCHED_NORMAL) || !sched_feat(WAKEUP_PREEMPTION))
        return;

    find_matching_se(&se, &pse) {
        int se_depth, pse_depth;

        /* preemption test can be made between sibling entities who are in the
         * same cfs_rq i.e who have a common parent. Walk up the hierarchy of
         * both tasks until we find their ancestors who are siblings of common
         * parent. */

        /* First walk up until both entities are at same depth */
        se_depth = (*se)->depth;
        pse_depth = (*pse)->depth;

        while (se_depth > pse_depth) {
            se_depth--;
            *se = parent_entity(*se);
        }

        while (pse_depth > se_depth) {
            pse_depth--;
            *pse = parent_entity(*pse);
        }

        while (!is_same_group(*se, *pse)) {
            *se = parent_entity(*se);
            *pse = parent_entity(*pse);
        }
    }
    WARN_ON_ONCE(!pse);

/* 2. cmp idle stat of matching se */
    cse_is_idle = se_is_idle(se);
    pse_is_idle = se_is_idle(pse);

    /* Preempt an idle group in favor of a non-idle group (and don't preempt
     * in the inverse case). */
    if (cse_is_idle && !pse_is_idle)
        goto preempt;
    if (cse_is_idle != pse_is_idle)
        return;

    cfs_rq = cfs_rq_of(se);
    update_curr(cfs_rq);

/* 3. pick_eevdf */
    /* XXX pick_eevdf(cfs_rq) != se ? */
    if (pick_eevdf(cfs_rq) == pse) {
        goto preempt;
    }

    return;

preempt:
    resched_curr(rq);
}
```

## task_tick_fair

![](../images/kernel/proc-sched-cfs-task_tick.png)
![](../images/kernel/proc-sched-cfs-update_curr.png)

```c
task_tick_fair(struct rq *rq, struct task_struct *curr, int queued) {
    for_each_sched_entity(se) {
        cfs_rq = cfs_rq_of(se);
        entity_tick(cfs_rq, se, queued) {
/* 1. udpate: exec_start, sum_exec, slice, vruntime, deadline,
 * cfs_rq.min_vruntime, bandwidth: cfs_rq.runtime_remaining, cfs_b.runtime */
            update_curr(cfs_rq) {
                delta_exec = now - curr->exec_start;
                curr->exec_start = now;
                curr->sum_exec_runtime += delta_exec;
                schedstat_add(cfs_rq->exec_clock, delta_exec);
                curr->vruntime += calc_delta_fair(delta_exec, curr);

                update_deadline(cfs_rq, curr) {
                    if ((s64)(se->vruntime - se->deadline) < 0)
                        return;
                    se->slice = sysctl_sched_base_slice;

                    /* EEVDF: vd_i = ve_i + r_i / w_i */
                    se->deadline = se->vruntime + calc_delta_fair(se->slice, se);

                    /* The task has consumed its request, reschedule. */
                    if (cfs_rq->nr_running > 1) {
                        resched_curr(rq_of(cfs_rq));
                        clear_buddies(cfs_rq, se);
                    }
                }
                update_min_vruntime(cfs_rq) {
                    struct sched_entity *se = __pick_first_entity(cfs_rq);
                    struct sched_entity *curr = cfs_rq->curr;

                    u64 vruntime = cfs_rq->min_vruntime;

                    if (curr) {
                        if (curr->on_rq)
                            vruntime = curr->vruntime;
                        else
                            curr = NULL;
                    }

                    if (se) {
                        if (!curr) {
                            vruntime = se->vruntime;
                        } else {
                            vruntime = min_vruntime(vruntime, se->vruntime/*vruntime*/) {
                                s64 delta = (s64)(vruntime - min_vruntime);
                                if (delta < 0) {
                                    min_vruntime = vruntime;
                                }
                                return min_vruntime;
                            }
                        }
                    }

                    /* ensure we never gain time by being placed backwards. */
                    u64_u32_store(cfs_rq->min_vruntime,
                        __update_min_vruntime(cfs_rq, vruntime) {
                            u64 min_vruntime = cfs_rq->min_vruntime;
                            s64 delta = (s64)(vruntime - min_vruntime);
                            if (delta > 0) {
                                avg_vruntime_update(cfs_rq, delta) {
                                    /* TODO: why?
                                     * v' = v + d ==> avg_vruntime' = avg_runtime - d*avg_load */
                                    cfs_rq->avg_vruntime -= cfs_rq->avg_load * delta;
                                }
                                min_vruntime = vruntime;
                            }
                            return min_vruntime;
                        }
                    );
                }

                if (entity_is_task(curr)) {
                    struct task_struct *curtask = task_of(curr);
                    cgroup_account_cputime(curtask, delta_exec);
                    account_group_exec_runtime(curtask, delta_exec);
                }

                account_cfs_rq_runtime(cfs_rq, delta_exec) {
                    cfs_rq->runtime_remaining -= delta_exec;

                    if (likely(cfs_rq->runtime_remaining > 0))
                        return;

                    if (cfs_rq->throttled)
                        return;

                    ret = assign_cfs_rq_runtime(cfs_rq) {
                        cfs_b = tg_cfs_bandwidth(cfs_rq->tg);
                        __assign_cfs_rq_runtime(cfs_b, cfs_rq, sched_cfs_bandwidth_slice()) {
                            --->
                        }
                    }
                    /* throttle task if cant assign runtime */
                    if (!ret && likely(cfs_rq->curr)) {
                        resched_curr(rq_of(cfs_rq));
                    }
                }
            }
/* 2. udpate load_avg, cfs group shares */
            update_load_avg(cfs_rq, curr, UPDATE_TG);
            update_cfs_group(curr);

        #ifdef CONFIG_SCHED_HRTICK
            if (queued) {
                resched_curr(rq_of(cfs_rq));
                return;
            }

            if (!sched_feat(DOUBLE_TICK) && hrtimer_active(&rq_of(cfs_rq)->hrtick_timer))
                return;
        #endif
        }
    }

    if (static_branch_unlikely(&sched_numa_balancing)){
        task_tick_numa(rq, curr) {

        }
    }

    update_misfit_status(curr, rq);
    update_overutilized_status(task_rq(curr));

    task_tick_core(rq, curr) {

    }
}
```

## task_fork_fair

![](../images/kernel/proc-sched-cfs-task_fork_fair.png)

```c
task_fork_fair(struct task_struct *p)
    update_rq_clock(rq);
    cfs_rq = task_cfs_rq(current);
    curr = cfs_rq->curr;
    if (curr) {
        update_curr(cfs_rq);
        se->vruntime = curr->vruntime;
    }
    /* udpate se: slice, vruntime, deadline */
    place_entity(cfs_rq, se, ENQUEUE_INITIAL);
        --->
```

## yield_task_fair

```c
yield_task_fair(struct rq *rq)
{
    struct task_struct *curr = rq->curr;
    struct cfs_rq *cfs_rq = task_cfs_rq(curr);
    struct sched_entity *se = &curr->se;

    if (unlikely(rq->nr_running == 1))
        return;

    clear_buddies(cfs_rq, se);

    update_rq_clock(rq);
    update_curr(cfs_rq);
    rq_clock_skip_update(rq);

    se->deadline += calc_delta_fair(se->slice, se);
}
```

## prio_changed_fair

```c
static void
prio_changed_fair(struct rq *rq, struct task_struct *p, int oldprio)
{
    if (!task_on_rq_queued(p))
        return;

    if (rq->cfs.nr_running == 1)
        return;

    /* Reschedule if we are currently running on this runqueue and
     * our priority decreased, or if we are not currently running on
     * this runqueue and our priority is higher than the current's */
    if (task_current(rq, p)) {
        if (p->prio > oldprio)
            resched_curr(rq);
    } else
        wakeup_preempt(rq, p, 0);
}

```

## check_class_changed_fair

### switched_from_fair

```c
/* detach load_avg */
void switched_from_fair(struct rq *rq, struct task_struct *p)
{
    detach_task_cfs_rq(p) {
        struct sched_entity *se = &p->se;

        detach_entity_cfs_rq(se) {
            struct cfs_rq *cfs_rq = cfs_rq_of(se);

            /* In case the task sched_avg hasn't been attached:
            * - A forked task which hasn't been woken up by wake_up_new_task().
            * - A task which has been woken up by try_to_wake_up() but is
            *   waiting for actually being woken up by sched_ttwu_pending(). */
            if (!se->avg.last_update_time)
                return;

            /* Catch up with the cfs_rq and remove our load when we leave */
            update_load_avg(cfs_rq, se, 0);
            detach_entity_load_avg(cfs_rq, se);
            update_tg_load_avg(cfs_rq);

            /* Propagate the changes of the sched_entity across the tg tree to make it
             * visible to the root */
            propagate_entity_cfs_rq(se) {
                struct cfs_rq *cfs_rq = cfs_rq_of(se);

                if (cfs_rq_throttled(cfs_rq))
                    return;

                if (!throttled_hierarchy(cfs_rq))
                    list_add_leaf_cfs_rq(cfs_rq);

                /* Start to propagate at parent */
                se = se->parent;

                for_each_sched_entity(se) {
                    cfs_rq = cfs_rq_of(se);

                    update_load_avg(cfs_rq, se, UPDATE_TG);

                    if (cfs_rq_throttled(cfs_rq))
                        break;

                    if (!throttled_hierarchy(cfs_rq))
                        list_add_leaf_cfs_rq(cfs_rq);
                }
            }
        }
    }
}
```

### switched_to_fair

```c
/* attach load_avg */
void switched_to_fair(struct rq *rq, struct task_struct *p)
{
    attach_task_cfs_rq(p) {
        struct sched_entity *se = &p->se;

        attach_entity_cfs_rq(se) {
            struct cfs_rq *cfs_rq = cfs_rq_of(se);

            update_load_avg(cfs_rq, se, sched_feat(ATTACH_AGE_LOAD) ? 0 : SKIP_AGE_LOAD);
            attach_entity_load_avg(cfs_rq, se);
            update_tg_load_avg(cfs_rq);
            propagate_entity_cfs_rq(se);
        }
    }

    if (task_on_rq_queued(p)) {
        /* We were most likely switched from sched_rt, so
         * kick off the schedule if running, otherwise just see
         * if we can still preempt the current task. */
        if (task_current(rq, p))
            resched_curr(rq);
        else
            wakeup_preempt(rq, p, 0);
    }
}
```

## balance_fair

```c
int balance_fair(struct rq *rq, struct task_struct *prev, struct rq_flags *rf)
{
    if (rq->nr_running)
        return 1;

    return sched_balance_newidle(rq, rf) != 0;
}
```

## sched_vslice

![](../images/kernel/proc-sched-cfs-sched_vslice.png)
![](../images/kernel/proc-sched-cfs-sched_vslice-2.png)

```c
const int sched_prio_to_weight[40] = {
 /* -20 */     88761,     71755,     56483,     46273,     36291,
 /* -15 */     29154,     23254,     18705,     14949,     11916,
 /* -10 */      9548,      7620,      6100,      4904,      3906,
 /*  -5 */      3121,      2501,      1991,      1586,      1277,
 /*   0 */      1024,       820,       655,       526,       423,
 /*   5 */       335,       272,       215,       172,       137,
 /*  10 */       110,        87,        70,        56,        45,
 /*  15 */        36,        29,        23,        18,        15,
};

sched_vslice(struct cfs_rq *cfs_rq, struct sched_entity *se)
    slice = sched_slice(cfs_rq, se) {
        slice = __sched_period(nr_running + !se->on_rq) {
            if (unlikely(nr_running > sched_nr_latency/*8*/))
                return nr_running * sysctl_sched_base_slice/*0.75 msec*/;
            else
                return sysctl_sched_latency/*6ms*/;
        }
        for_each_sched_entity(se) {
            struct load_weight *load;
            struct load_weight lw;
            struct cfs_rq *qcfs_rq;

            qcfs_rq = cfs_rq_of(se);
            load = &qcfs_rq->load;

            if (unlikely(!se->on_rq)) {
                lw = qcfs_rq->load;
                update_load_add(&lw, se->load.weight);
                load = &lw;
            }

            slice = __calc_delta(slice, se->load.weight, load) {
                /* delta_exec * weight / lw.weight
                 *   OR
                 * (delta_exec * (weight * lw->inv_weight)) >> WMULT_SHIFT */
            }
        }
        if (sched_feat(BASE_SLICE)) {
            if (se_is_idle(init_se) && !sched_idle_cfs_rq(cfs_rq))
                min_gran = sysctl_sched_idle_min_granularity;
            else
                min_gran = sysctl_sched_base_slice;

            slice = max_t(u64, slice, min_gran);
        }
        return slice;
    }
    calc_delta_fair(slice, se) {
        if (unlikely(se->load.weight != NICE_0_LOAD)) {
            delta = __calc_delta(delta, NICE_0_LOAD, &se->load) {
                /* delta_exec * weight / lw.weight
                 *   OR
                 * (delta_exec * (weight * lw->inv_weight)) >> WMULT_SHIFT */
            }
        }

        return delta;
    }
```

# sched_domain

* [极致Linux内核 - Scheduling Domain](https://zhuanlan.zhihu.com/p/589693879)
* [CPU的拓扑结构](https://s3.shizhz.me/linux-sched/lb/lb-cpu-topo)      [数据结构](https://s3.shizhz.me/linux-sched/lb/lb-data-structure)

```c
struct sched_domain_topology_level {
    sched_domain_mask_f     mask;
    sched_domain_flags_f    sd_flags;
    int                     flags;
    int                  numa_level;
    struct sd_data          data;
};

static struct sched_domain_topology_level *sched_domain_topology =
	default_topology = {
#ifdef CONFIG_SCHED_SMT
    { cpu_smt_mask, cpu_smt_flags, SD_INIT_NAME(SMT) },
#endif

#ifdef CONFIG_SCHED_CLUSTER
    { cpu_clustergroup_mask, cpu_cluster_flags, SD_INIT_NAME(CLS) },
#endif

#ifdef CONFIG_SCHED_MC
    { cpu_coregroup_mask, cpu_core_flags, SD_INIT_NAME(MC) },
#endif
    { cpu_cpu_mask, SD_INIT_NAME(PKG) },
    { NULL, },
};

struct sd_data {
    struct sched_domain *__percpu           *sd;
    struct sched_domain_shared *__percpu    *sds;
    struct sched_group *__percpu            *sg;
    struct sched_group_capacity *__percpu   *sgc;
};

struct sched_domain_shared {
    atomic_t    ref;
    atomic_t    nr_busy_cpus;
    int         has_idle_cores;
    int         nr_idle_scan;
};

struct sched_domain {
    /* These fields must be setup */
    struct sched_domain __rcu *parent; /* top domain must be null terminated */
    struct sched_domain __rcu *child; /* bottom domain must be null terminated */
    struct sched_group *groups; /* the balancing groups of the domain */
    unsigned long min_interval; /* Minimum balance interval ms */
    unsigned long max_interval; /* Maximum balance interval ms */
    unsigned int busy_factor; /* less balancing by factor if busy */
    unsigned int imbalance_pct; /* No balance until over watermark */
    unsigned int cache_nice_tries; /* Leave cache hot tasks for # tries */
    unsigned int imb_numa_nr; /* Nr running tasks that allows a NUMA imbalance */

    int nohz_idle;          /* NOHZ IDLE status */
    int flags;              /* See SD_* */
    int level;

    /* Runtime fields. */
    unsigned long last_balance; /* init to jiffies. units in jiffies */
    unsigned int balance_interval; /* initialise to 1. units in ms. */
    unsigned int nr_balance_failed; /* initialise to 0 */

    /* idle_balance() stats */
    u64 max_newidle_lb_cost;
    unsigned long last_decay_max_lb_cost;

    union {
        void *private;  /* used during construction */
        struct rcu_head rcu; /* used during destruction */
    };
    struct sched_domain_shared *shared;

    unsigned int span_weight;
    /* Span of all CPUs in this domain. */
    unsigned long span[];
};

struct sched_group {
    struct sched_group  *next; /* Must be a circular list */
    atomic_t            ref;

    unsigned int        group_weight;
    unsigned int        cores;
    struct sched_group_capacity *sgc;
    int                 asym_prefer_cpu; /* CPU of highest priority in group */
    int                 flags;

    /* The CPUs this group covers. */
    unsigned long       cpumask[];
};

struct sched_group_capacity {
    atomic_t            ref;
    unsigned long       capacity;
    unsigned long       min_capacity; /* Min per-CPU capacity in group */
    unsigned long       max_capacity; /* Max per-CPU capacity in group */
    unsigned long       next_update;
    int                 imbalance; /* XXX unrelated to capacity but shared group state */

    unsigned long       cpumask[]; /* Balance mask */
};

void __init sched_init_smp(void) {
    sched_init_numa(void);
    sched_init_domains(cpu_active_mask);
}
```

```c
sched_init_domains(cpu_active_mask) {
    zalloc_cpumask_var(&sched_domains_tmpmask, GFP_KERNEL);
    zalloc_cpumask_var(&sched_domains_tmpmask2, GFP_KERNEL);
    zalloc_cpumask_var(&fallback_doms, GFP_KERNEL);

    arch_update_cpu_topology();
    asym_cpu_capacity_scan();

    ndoms_cur = 1;
    doms_cur = alloc_sched_domains(ndoms_cur);
    if (!doms_cur)
        doms_cur = &fallback_doms;
    cpumask_and(doms_cur[0], cpu_map, housekeeping_cpumask(HK_TYPE_DOMAIN));

    err = build_sched_domains(doms_cur[0], NULL) {
        enum s_alloc alloc_state = sa_none;
        struct sched_domain *sd;
        struct s_data d;
        struct rq *rq = NULL;
        int i, ret = -ENOMEM;
        bool has_asym = false;

        if (WARN_ON(cpumask_empty(cpu_map)))
            goto error;

        alloc_state = __visit_domain_allocation_hell(&d, cpu_map);
        if (alloc_state != sa_rootdomain)
            goto error;

        /* Set up domains for CPUs specified by the cpu_map: */
        for_each_cpu(i, cpu_map) {
            struct sched_domain_topology_level *tl;

            sd = NULL;
            for_each_sd_topology(tl) {
                sd = build_sched_domain(tl, cpu_map, attr, sd, i) {
                    struct sched_domain *sd = sd_init(tl, cpu_map, child, cpu) {
                        struct sd_data *sdd = &tl->data;
                        struct sched_domain *sd = *per_cpu_ptr(sdd->sd, cpu);
                        int sd_id, sd_weight, sd_flags = 0;
                        struct cpumask *sd_span;

                        sched_domains_curr_level = tl->numa_level;
                        sd_weight = cpumask_weight(tl->mask(cpu));

                        if (tl->sd_flags) {
                            sd_flags = (*tl->sd_flags)();
                        }
                        *sd = (struct sched_domain) {
                            .min_interval       = sd_weight,
                            .max_interval       = 2*sd_weight,
                            .busy_factor        = 16,
                            .imbalance_pct      = 117,

                            .cache_nice_tries   = 0,

                            .flags = 1*SD_BALANCE_NEWIDLE
                                | 1*SD_BALANCE_EXEC
                                | 1*SD_BALANCE_FORK
                                | 0*SD_BALANCE_WAKE
                                | 1*SD_WAKE_AFFINE
                                | 0*SD_SHARE_CPUCAPACITY
                                | 0*SD_SHARE_PKG_RESOURCES
                                | 0*SD_SERIALIZE
                                | 1*SD_PREFER_SIBLING
                                | 0*SD_NUMA
                                | sd_flags ,
                            .last_balance           = jiffies,
                            .balance_interval       = sd_weight,
                            .max_newidle_lb_cost    = 0,
                            .last_decay_max_lb_cost = jiffies,
                            .child                  = child,
                        };

                        sd_span = sched_domain_span(sd);
                        cpumask_and(sd_span, cpu_map, tl->mask(cpu));
                        sd_id = cpumask_first(sd_span);

                        sd->flags |= asym_cpu_capacity_classify(sd_span, cpu_map)

                        /* Convert topological properties into behaviour. */
                        /* Don't attempt to spread across CPUs of different capacities. */
                        if ((sd->flags & SD_ASYM_CPUCAPACITY) && sd->child)
                            sd->child->flags &= ~SD_PREFER_SIBLING;

                        if (sd->flags & SD_SHARE_CPUCAPACITY) {
                            sd->imbalance_pct = 110;

                        } else if (sd->flags & SD_SHARE_PKG_RESOURCES) {
                            sd->imbalance_pct = 117;
                            sd->cache_nice_tries = 1;
                        } else if (sd->flags & SD_NUMA) {
                            sd->cache_nice_tries = 2;
                            sd->flags &= ~SD_PREFER_SIBLING;
                            sd->flags |= SD_SERIALIZE;
                            if (sched_domains_numa_distance[tl->numa_level] > node_reclaim_distance) {
                                sd->flags &= ~(
                                    SD_BALANCE_EXEC | SD_BALANCE_FORK | SD_WAKE_AFFINE
                                );
                            }
                        } else {
                            sd->cache_nice_tries = 1;
                        }

                        /* For all levels sharing cache; connect a sched_domain_shared
                         * instance. */
                        if (sd->flags & SD_SHARE_PKG_RESOURCES) {
                            sd->shared = *per_cpu_ptr(sdd->sds, sd_id);
                            atomic_inc(&sd->shared->ref);
                            atomic_set(&sd->shared->nr_busy_cpus, sd_weight);
                        }

                        sd->private = sdd;

                        return sd;
                    }

                    if (child) {
                        sd->level = child->level + 1;
                        sched_domain_level_max = max(sched_domain_level_max, sd->level);
                        child->parent = sd;

                        if (!cpumask_subset(sched_domain_span(child), sched_domain_span(sd))) {
                            /* Fixup, ensure @sd has at least @child CPUs. */
                            cpumask_or(
                                sched_domain_span(sd),
                                sched_domain_span(sd),
                                sched_domain_span(child)
                            );
                        }

                    }
                    set_domain_attribute(sd, attr);

                    return sd;
                }

                has_asym |= sd->flags & SD_ASYM_CPUCAPACITY;

                if (tl == sched_domain_topology)
                    *per_cpu_ptr(d.sd, i) = sd;
                if (tl->flags & SDTL_OVERLAP)
                    sd->flags |= SD_OVERLAP;
                if (cpumask_equal(cpu_map, sched_domain_span(sd)))
                    break;
            }
        }

        /* Build the groups for the domains */
        for_each_cpu(i, cpu_map) {
            for (sd = *per_cpu_ptr(d.sd, i); sd; sd = sd->parent) {
                sd->span_weight = cpumask_weight(sched_domain_span(sd));
                if (sd->flags & SD_OVERLAP) {
                    if (build_overlap_sched_groups(sd, i)) {
                        goto error;
                    }
                } else {
                    if (build_sched_groups(sd, i)) {
                        goto error;
                    }
                }
            }
        }

        /* Calculate an allowed NUMA imbalance such that LLCs do not get
        * imbalanced. */
        for_each_cpu(i, cpu_map) {
            unsigned int imb = 0;
            unsigned int imb_span = 1;

            for (sd = *per_cpu_ptr(d.sd, i); sd; sd = sd->parent) {
                struct sched_domain *child = sd->child;

                if (!(sd->flags & SD_SHARE_PKG_RESOURCES) && child &&
                    (child->flags & SD_SHARE_PKG_RESOURCES)) {
                    struct sched_domain __rcu *top_p;
                    unsigned int nr_llcs;

                    /* For a single LLC per node, allow an
                    * imbalance up to 12.5% of the node. This is
                    * arbitrary cutoff based two factors -- SMT and
                    * memory channels. For SMT-2, the intent is to
                    * avoid premature sharing of HT resources but
                    * SMT-4 or SMT-8 *may* benefit from a different
                    * cutoff. For memory channels, this is a very
                    * rough estimate of how many channels may be
                    * active and is based on recent CPUs with
                    * many cores.
                    *
                    * For multiple LLCs, allow an imbalance
                    * until multiple tasks would share an LLC
                    * on one node while LLCs on another node
                    * remain idle. This assumes that there are
                    * enough logical CPUs per LLC to avoid SMT
                    * factors and that there is a correlation
                    * between LLCs and memory channels. */
                    nr_llcs = sd->span_weight / child->span_weight;
                    if (nr_llcs == 1)
                        imb = sd->span_weight >> 3;
                    else
                        imb = nr_llcs;
                    imb = max(1U, imb);
                    sd->imb_numa_nr = imb;

                    /* Set span based on the first NUMA domain. */
                    top_p = sd->parent;
                    while (top_p && !(top_p->flags & SD_NUMA)) {
                        top_p = top_p->parent;
                    }
                    imb_span = top_p ? top_p->span_weight : sd->span_weight;
                } else {
                    int factor = max(1U, (sd->span_weight / imb_span));

                    sd->imb_numa_nr = imb * factor;
                }
            }
        }

        /* Calculate CPU capacity for physical packages and nodes */
        for (i = nr_cpumask_bits-1; i >= 0; i--) {
            if (!cpumask_test_cpu(i, cpu_map))
                continue;

            for (sd = *per_cpu_ptr(d.sd, i); sd; sd = sd->parent) {
                claim_allocations(i, sd);
                init_sched_groups_capacity(i, sd);
            }
        }

        /* Attach the domains */
        rcu_read_lock();
        for_each_cpu(i, cpu_map) {
            rq = cpu_rq(i);
            sd = *per_cpu_ptr(d.sd, i);

            /* Use READ_ONCE()/WRITE_ONCE() to avoid load/store tearing: */
            if (rq->cpu_capacity_orig > READ_ONCE(d.rd->max_cpu_capacity))
                WRITE_ONCE(d.rd->max_cpu_capacity, rq->cpu_capacity_orig);

            cpu_attach_domain(sd, d.rd, i);
        }
        rcu_read_unlock();

        if (has_asym)
            static_branch_inc_cpuslocked(&sched_asym_cpucapacity);

        if (rq && sched_debug_verbose) {
            pr_info("root domain span: %*pbl (max cpu_capacity = %lu)\n",
                cpumask_pr_args(cpu_map), rq->rd->max_cpu_capacity);
        }

        ret = 0;
    error:
        __free_domain_allocs(&d, alloc_state, cpu_map);

        return ret;
    }
}
```

# cpu_capacity

* [DumpStack - 负载跟踪 - cpu算力](http://www.dumpstack.cn/index.php/2022/06/02/743.html  )

```c
dmips = dmips_mhz * policy->cpuinfo.max_freq
cpu_scale = (dmips * 1024) / dmips[MAX]
```

* raw_capacity
* cpu_scale

    Normalized cpu capacity towards the maximum core and highest frequency, a fixed value.

    * arch_scale_cpu_capacity(): get cpu capacity
    * topology_get_cpu_scale(): get cpu_scale of a cpu
    * topology_set_cpu_scale(): set the cpu_scale of a cpu

* arch_freq_scale

    The percpu variable is a changing value that represents the CPU's current frequency normalized to 1024, relative to the maximum frequency of that CPU.

    * arch_scale_freq_capacity()
    * topology_get_freq_scale()
    * arch_set_freq_scale()
    * topology_set_freq_scale()
    * set time point
        * cpufreq_driver_fast_switch()
        * cpufreq_freq_transition_end()
* cpu_capacity_orig vs cpu_capacity
    * capacity_of(): capacity for cfs tasks
    * capacity_orig_of()
    * update_cpu_capacity(): update both cpu_capacity_orig and cpu_capacity
    * scale_rt_capacity(): caculate cfs capacity

## parse_dt_topology

```c
parse_dt_topology(void)
{
    struct device_node *cn, *map;
    int ret = 0;
    int cpu;

    cn = of_find_node_by_path("/cpus");
    map = of_get_child_by_name(cn, "cpu-map");
    ret = parse_socket(map);

    topology_normalize_cpu_scale() {
        u64 capacity;
        u64 capacity_scale;
        int cpu;

        capacity_scale = 1;
        for_each_possible_cpu(cpu) {
            capacity = raw_capacity[cpu] * per_cpu(freq_factor, cpu);
            capacity_scale = max(capacity, capacity_scale);
        }

        for_each_possible_cpu(cpu) {
            capacity = raw_capacity[cpu] * per_cpu(freq_factor, cpu);
            capacity = div64_u64(capacity << SCHED_CAPACITY_SHIFT, capacity_scale);
            topology_set_cpu_scale(cpu, capacity) {
                per_cpu(cpu_scale, cpu) = capacity;
            }
        }
    }

    return ret;
}
```

## parse_socket

```c
parse_socket(struct device_node *socket)
{
    char name[20];
    struct device_node *c;
    bool has_socket = false;
    int package_id = 0, ret;

    do {
        snprintf(name, sizeof(name), "socket%d", package_id);
        c = of_get_child_by_name(socket, name);
        if (c) {
            has_socket = true;
            ret = parse_cluster(c, package_id, -1, 0);
            of_node_put(c);
            if (ret != 0)
                return ret;
        }
        package_id++;
    } while (c);

    if (!has_socket)
        ret = parse_cluster(socket, 0, -1, 0);

    return ret;
}
```

## parse_cluster

```c
parse_cluster(struct device_node *cluster, int package_id,
                int cluster_id, int depth)
{
    char name[20];
    bool leaf = true;
    bool has_cores = false;
    struct device_node *c;
    int core_id = 0;
    int i, ret;

    /* First check for child clusters */
    i = 0;
    do {
        snprintf(name, sizeof(name), "cluster%d", i);
        c = of_get_child_by_name(cluster, name);
        if (c) {
            leaf = false;
            ret = parse_cluster(c, package_id, i, depth + 1);
            of_node_put(c);
            if (ret != 0)
                return ret;
        }
        i++;
    } while (c);

    /* Now check for cores */
    i = 0;
    do {
        snprintf(name, sizeof(name), "core%d", i);
        c = of_get_child_by_name(cluster, name);
        if (c) {
            has_cores = true;

            if (depth == 0) {
                of_node_put(c);
                return -EINVAL;
            }

            if (leaf) {
                ret = parse_core(c, package_id, cluster_id, core_id++);
            } else {
                ret = -EINVAL;
            }

            of_node_put(c);
            if (ret != 0)
                return ret;
        }
        i++;
    } while (c);

    return 0;
}
```

## parse_core

```c
parse_core(struct device_node *core, int package_id,
                int cluster_id, int core_id)
{
    char name[20];
    bool leaf = true;
    int i = 0;
    int cpu;
    struct device_node *t;

    do {
        snprintf(name, sizeof(name), "thread%d", i);
        t = of_get_child_by_name(core, name);
        if (t) {
            leaf = false;
            cpu = get_cpu_for_node(t);
            if (cpu >= 0) {
                cpu_topology[cpu].package_id = package_id;
                cpu_topology[cpu].cluster_id = cluster_id;
                cpu_topology[cpu].core_id = core_id;
                cpu_topology[cpu].thread_id = i;
            } else if (cpu != -ENODEV) {
                of_node_put(t);
                return -EINVAL;
            }
            of_node_put(t);
        }
        i++;
    } while (t);

    cpu = get_cpu_for_node(core);
    if (cpu >= 0) {
        if (!leaf) {
            return -EINVAL;
        }

        cpu_topology[cpu].package_id = package_id;
        cpu_topology[cpu].cluster_id = cluster_id;
        cpu_topology[cpu].core_id = core_id;
    } else if (leaf && cpu != -ENODEV) {
        return -EINVAL;
    }

    return 0;
}
```

```c
get_cpu_for_node(struct device_node *node)
{
    struct device_node *cpu_node;
    int cpu;

    cpu_node = of_parse_phandle(node, "cpu", 0);
    if (!cpu_node)
        return -1;

    cpu = of_cpu_node_to_id(cpu_node);
    if (cpu >= 0) {
        topology_parse_cpu_capacity(cpu_node, cpu) {
            struct clk *cpu_clk;
            static bool cap_parsing_failed;
            int ret;
            u32 cpu_capacity;

            if (cap_parsing_failed)
                return false;

            ret = of_property_read_u32(cpu_node, "capacity-dmips-mhz", &cpu_capacity);
            if (!ret) {
                if (!raw_capacity) {
                    raw_capacity = kcalloc(num_possible_cpus(),
                                sizeof(*raw_capacity),
                                GFP_KERNEL);
                    if (!raw_capacity) {
                        cap_parsing_failed = true;
                        return false;
                    }
                }
                raw_capacity[cpu] = cpu_capacity;

                cpu_clk = of_clk_get(cpu_node, 0);
                if (!PTR_ERR_OR_ZERO(cpu_clk)) {
                    per_cpu(freq_factor, cpu) = clk_get_rate(cpu_clk) / 1000;
                    clk_put(cpu_clk);
                }
            } else {
                cap_parsing_failed = true;
                free_raw_capacity();
            }

            return !ret;
        }
    }
    of_node_put(cpu_node);
    return cpu;
}
```

# PELT

* [DumpStack - PELT](http://www.dumpstack.cn/index.php/2022/08/13/785.html)
* [Wowo Tech - :one:PELT](http://www.wowotech.net/process_management/450.html)     [:two:PELT算法浅析](http://www.wowotech.net/process_management/pelt.html)
* [Linux核心概念详解 - 2.7 负载追踪](https://s3.shizhz.me/linux-sched/load-trace)

![](../images/kernel/proc-sched-cfs-pelt-segement.png)
![](../images/kernel/proc-sched-cfs-update_load_avg.png)

---

![](../imeage/kernel/proc-pelt_last_update_time.png) /* EXPORT */

* [**Load** :link:](https://lwn.net/Articles/531853/) is also meant to be an instantaneous quantity - how much is a process loading the system right now? - as opposed to a cumulative property like **CPU usage**. A long-running process that consumed vast amounts of processor time last week may have very modest needs at the moment; such a process is contributing very little to load now, despite its rather more demanding behavior in the past.

```c
struct sched_avg {
    u64                 last_update_time;

    /* running + waiting, scaled to weight */
    u64                 load_sum;
    /* runnable includes either actually running,
     * or waiting for an available CPU */
    u64                 runnable_sum;
    /* running state, scaled to 1024, no weight */
    u32                 util_sum;

    /* the part that was less than one pelt cycle(1024 us)
     * when last updated, unit in us */
    u32                 period_contrib;

    unsigned long       load_avg;
    unsigned long       runnable_avg;
    unsigned long       util_avg;
    unsigned int        util_est; /* saved util before sleep */
}
```

## update_load_sum_avg

load, runnable and running function as:
1. switch: controlling whether to update the corresponding load contribution
2. scaling factor:
    * on one hand, because the importance of different processes varies, the load caused by running for the same duration should also differ;
    * on the other hand, for tasks and groups, if a certain group SE which has several tasks waiting together has been waiting in the queue for 1 ms, the pressure it causes should be multiplied.

```c
/* sched_entity:
 *
 *   task:
 *     se_weight()   = se->load.weight
 *     se_runnable() = !!on_rq
 *
 *   group: [ see update_cfs_group() ]
 *     se_weight()   = tg->weight * grq->load_avg / tg->load_avg
 *     se_runnable() = grq->h_nr_running
 *
 *   runnable_sum = se_runnable() * runnable = grq->runnable_sum
 *   runnable_avg = runnable_sum
 *
 *   load_sum := runnable
 *   load_avg = se_weight(se) * load_sum
 *
 * cfq_rq:
 *
 *   runnable_sum = \Sum se->avg.runnable_sum
 *   runnable_avg = \Sum se->avg.runnable_avg
 *
 *   load_sum = \Sum se_weight(se) * se->avg.load_sum
 *   load_avg = \Sum se->avg.load_avg */

int ___update_load_sum(u64 now, struct sched_avg *sa,
        unsigned long load, unsigned long runnable, int running)
{
    u64 delta;

    delta = now - sa->last_update_time /* ns */;

    /* s64 ns clock overflow */
    if ((s64)delta < 0) {
        sa->last_update_time = now;
        return 0;
    }

    /* ns -> us */
    delta >>= 10;
    if (!delta) /* less than 1us, return */
        return 0;

    sa->last_update_time += delta << 10;

    if (!load)
        runnable = running = 0;

    ret = accumulate_sum(delta, sa, load, runnable, running) {
        u32 contrib = (u32)delta; /* p == 0 -> delta < 1024 */
        u64 periods;

        delta += sa->period_contrib;
        periods = delta / 1024; /* A period is 1024us (~1ms) */

        if (periods) {
            /* Step 1: decay old *_sum if we crossed period boundaries. */
            sa->load_sum = decay_load(sa->load_sum, periods);
            sa->runnable_sum = decay_load(sa->runnable_sum, periods);
            sa->util_sum = decay_load((u64)(sa->util_sum), periods);

            /* Step 2: calc new load: d1 + d2 + d3 */
            delta %= 1024;
            if (load) {
                contrib = __accumulate_pelt_segments(periods, 1024 - sa->period_contrib/*d1*/, delta/*d3*/) {
                    u32 c1, c2, c3 = d3; /* y^0 == 1 */

                    /* c1 = d1 y^p */
                    c1 = decay_load((u64)d1, periods);

                    /*            p-1
                     * c2 = 1024 \Sum y^n
                     *            n=1
                     *
                     *              inf        inf
                     *    = 1024 ( \Sum y^n - \Sum y^n - y^0 )
                     *              n=0        n=p */
                    c2 = LOAD_AVG_MAX - decay_load(LOAD_AVG_MAX, periods) - 1024;

                    return c1 + c2 + c3;
                }
            }
        }
        sa->period_contrib = delta;

        if (load)
            sa->load_sum += load * contrib; /* scale to load */
        if (runnable)
            sa->runnable_sum += runnable * contrib << SCHED_CAPACITY_SHIFT;
        if (running) /* scale to SCHED_CAPACITY_SHIFT 1024 */
            sa->util_sum += contrib << SCHED_CAPACITY_SHIFT;

        return periods;
    }
    if (!ret)
        return 0;

    return 1;
}

static __always_inline void
___update_load_avg(struct sched_avg *sa, unsigned long load/* se_weight */)
{
    u32 divider = get_pelt_divider(sa) {
        #define LOAD_AVG_MAX        47742
        #define PELT_MIN_DIVIDER    (LOAD_AVG_MAX - 1024)
        return PELT_MIN_DIVIDER + avg->period_contrib;
    }

    /* ___update_load_sum also takes load arg, only one of the two loads takes affect,
     * when one takes affect, the another is set to 1 */

    /*                             contrib
     * load_avg = --------------------------------------------- * load
     *              LOAD_AVG_MAX - (1024 - sa->period_contrib) */
    sa->load_avg = div_u64(load * sa->load_sum, divider);

    /*                      runnable_sum = runnable * contrib
     * runnable_avg = --------------------------------------------- * 1024
     *              LOAD_AVG_MAX - (1024 - sa->period_contrib) */
    sa->runnable_avg = div_u64(sa->runnable_sum, divider);

    /*                              contrib
     * util_avg = --------------------------------------------- * 1024
     *              LOAD_AVG_MAX - (1024 - sa->period_contrib) */
    WRITE_ONCE(sa->util_avg, sa->util_sum / divider);
}
```

## update_load_avg

```c
/* Update task and its cfs_rq load average */
void update_load_avg(struct cfs_rq *cfs_rq, struct sched_entity *se, int flags)
{
    u64 now = cfs_rq_clock_pelt(cfs_rq);
    int decayed;

    /* a new forked or migrated task's last_update_time is 0 */
    if (se->avg.last_update_time && !(flags & SKIP_AGE_LOAD)) {
        __update_load_avg_se(now, cfs_rq, se) {
            ret = ___update_load_sum(now, &se->avg, !!se->on_rq, se_runnable(se),
                cfs_rq->curr == se);
           if (ret) {
                ___update_load_avg(&se->avg, se_weight(se));
                cfs_se_util_change(&se->avg) {
                    int enqueued = avg->util_est;
                    if (enqueued & UTIL_AVG_UNCHANGED) {
                        enqueued &= ~UTIL_AVG_UNCHANGED;
                        WRITE_ONCE(avg->util_est, enqueued);
                    }
                }
                return 1;
            }
        }
    }

    /* decayed indicates wheather load has been updated and freq needs to be updated */
    decayed = update_cfs_rq_load_avg(now, cfs_rq);
    decayed |= propagate_entity_load_avg(se) {
        struct cfs_rq *cfs_rq, *gcfs_rq;

        if (entity_is_task(se))
            return 0;

        gcfs_rq = group_cfs_rq(se); /* 1. rq se owns */
        if (!gcfs_rq->propagate)
            return 0;

        gcfs_rq->propagate = 0;

        cfs_rq = cfs_rq_of(se);     /* 2. rq se belongs to */

        /* propagate to parent */
        add_tg_cfs_propagate(cfs_rq, gcfs_rq->prop_runnable_sum) {
            cfs_rq->propagate = 1;
            cfs_rq->prop_runnable_sum += runnable_sum;
        }

        update_tg_cfs_util(cfs_rq, se, gcfs_rq) {
            /* gcfs_rq->avg.util_avg only updated when gcfs_rq->curr != NULL,
             * which means the grp has running tasks
             *
             * se->avg.util_avg only updated when cfs_rq->curr == se,
             * which means cfs_rq picks se to run */
            long delta_sum, delta_avg = gcfs_rq->avg.util_avg - se->avg.util_avg;
            u32 new_sum, divider;

            /* Nothing to update */
            if (!delta_avg)
                return;

            divider = get_pelt_divider(&cfs_rq->avg);

            /* Set new sched_entity's utilization */
            se->avg.util_avg = gcfs_rq->avg.util_avg;
            new_sum = se->avg.util_avg * divider;
            delta_sum = (long)new_sum - (long)se->avg.util_sum;
            se->avg.util_sum = new_sum;

            /* Update parent cfs_rq utilization */
            add_positive(&cfs_rq->avg.util_avg, delta_avg);
            add_positive(&cfs_rq->avg.util_sum, delta_sum);

            /* See update_cfs_rq_load_avg() */
            cfs_rq->avg.util_sum = max_t(u32, cfs_rq->avg.util_sum,
                            cfs_rq->avg.util_avg * PELT_MIN_DIVIDER);
        }

        update_tg_cfs_runnable(cfs_rq, se, gcfs_rq) {
            long delta_sum, delta_avg = gcfs_rq->avg.runnable_avg - se->avg.runnable_avg;
            u32 new_sum, divider;

            /* Nothing to update */
            if (!delta_avg)
                return;

            divider = get_pelt_divider(&cfs_rq->avg);

            /* Set new sched_entity's runnable */
            se->avg.runnable_avg = gcfs_rq->avg.runnable_avg;
            new_sum = se->avg.runnable_avg * divider;
            delta_sum = (long)new_sum - (long)se->avg.runnable_sum;
            se->avg.runnable_sum = new_sum;

            /* Update parent cfs_rq runnable */
            add_positive(&cfs_rq->avg.runnable_avg, delta_avg);
            add_positive(&cfs_rq->avg.runnable_sum, delta_sum);
            /* See update_cfs_rq_load_avg() */
            cfs_rq->avg.runnable_sum = max_t(u32, cfs_rq->avg.runnable_sum,
                                cfs_rq->avg.runnable_avg * PELT_MIN_DIVIDER);
        }

        update_tg_cfs_load(cfs_rq, se, gcfs_rq) {
            long delta_avg, running_sum, runnable_sum = gcfs_rq->prop_runnable_sum;
            unsigned long load_avg;
            u64 load_sum = 0;
            s64 delta_sum;
            u32 divider;

            if (!runnable_sum)
                return;

            gcfs_rq->prop_runnable_sum = 0;

            divider = get_pelt_divider(&cfs_rq->avg);

            if (runnable_sum >= 0) {
                /* Add runnable; clip at LOAD_AVG_MAX. Reflects that until
                 * the CPU is saturated running == runnable. */
                runnable_sum += se->avg.load_sum;
                runnable_sum = min_t(long, runnable_sum, divider);
            } else {
                /* Estimate the new unweighted runnable_sum of the gcfs_rq by
                 * assuming all tasks are equally runnable. */
                if (scale_load_down(gcfs_rq->load.weight)) {
                    load_sum = div_u64(gcfs_rq->avg.load_sum,
                        scale_load_down(gcfs_rq->load.weight));
                }

                /* But make sure to not inflate se's runnable */
                runnable_sum = min(se->avg.load_sum, load_sum);
            }

            /* runnable_sum can't be lower than running_sum
             * Rescale running sum to be in the same range as runnable sum
             * running_sum is in [0 : LOAD_AVG_MAX <<  SCHED_CAPACITY_SHIFT]
             * runnable_sum is in [0 : LOAD_AVG_MAX] */
            running_sum = se->avg.util_sum >> SCHED_CAPACITY_SHIFT;
            runnable_sum = max(runnable_sum, running_sum);

            load_sum = se_weight(se) * runnable_sum;
            load_avg = div_u64(load_sum, divider);

            delta_avg = load_avg - se->avg.load_avg;
            if (!delta_avg)
                return;

            delta_sum = load_sum - (s64)se_weight(se) * se->avg.load_sum;

            se->avg.load_sum = runnable_sum;
            se->avg.load_avg = load_avg;
            add_positive(&cfs_rq->avg.load_avg, delta_avg);
            add_positive(&cfs_rq->avg.load_sum, delta_sum);
            /* See update_cfs_rq_load_avg() */
            cfs_rq->avg.load_sum = max_t(u32, cfs_rq->avg.load_sum,
                            cfs_rq->avg.load_avg * PELT_MIN_DIVIDER);
        }

        return 1;
    }

    /* new forked or migrated task */
    if (!se->avg.last_update_time && (flags & DO_ATTACH)) {
        /* attach this entity to its cfs_rq load avg
         *
         * DO_ATTACH means we're here from enqueue_entity().
         * !last_update_time means we've passed through
         * migrate_task_rq_fair() indicating we migrated. */
        attach_entity_load_avg(cfs_rq, se) {
            u32 divider = get_pelt_divider(&cfs_rq->avg);

            se->avg.last_update_time = cfs_rq->avg.last_update_time;
            se->avg.period_contrib = cfs_rq->avg.period_contrib;

            se->avg.util_sum = se->avg.util_avg * divider;
            se->avg.runnable_sum = se->avg.runnable_avg * divider;

            se->avg.load_sum = se->avg.load_avg * divider;
            if (se_weight(se) < se->avg.load_sum)
                se->avg.load_sum = div_u64(se->avg.load_sum, se_weight(se));
            else
                se->avg.load_sum = 1;

            enqueue_load_avg(cfs_rq, se) {
                cfs_rq->avg.load_avg += se->avg.load_avg;
                cfs_rq->avg.load_sum += se_weight(se) * se->avg.load_sum;
            }
            cfs_rq->avg.util_avg += se->avg.util_avg;
            cfs_rq->avg.util_sum += se->avg.util_sum;
            cfs_rq->avg.runnable_avg += se->avg.runnable_avg;
            cfs_rq->avg.runnable_sum += se->avg.runnable_sum;

            add_tg_cfs_propagate(cfs_rq, se->avg.load_sum);

            cfs_rq_util_change(cfs_rq, 0);
        }

        update_tg_load_avg(cfs_rq) {
            long delta;
            u64 now;

            if (cfs_rq->tg == &root_task_group)
                return;

            now = sched_clock_cpu(cpu_of(rq_of(cfs_rq)));
            if (now - cfs_rq->last_update_tg_load_avg < NSEC_PER_MSEC)
                return;

            delta = cfs_rq->avg.load_avg - cfs_rq->tg_load_avg_contrib;
            if (abs(delta) > cfs_rq->tg_load_avg_contrib / 64) {
                atomic_long_add(delta, &cfs_rq->tg->load_avg);
                cfs_rq->tg_load_avg_contrib = cfs_rq->avg.load_avg;
                cfs_rq->last_update_tg_load_avg = now;
            }
        }
    } else if (flags & DO_DETACH) {
        /* detach this entity from its cfs_rq load avg */
        detach_entity_load_avg(cfs_rq, se) {
            dequeue_load_avg(cfs_rq, se) {
                sub_positive(&cfs_rq->avg.load_avg, se->avg.load_avg);
                sub_positive(&cfs_rq->avg.load_sum, se_weight(se) * se->avg.load_sum);
                cfs_rq->avg.load_sum = max_t(u32, cfs_rq->avg.load_sum,
                                cfs_rq->avg.load_avg * PELT_MIN_DIVIDER);
            }
            sub_positive(&cfs_rq->avg.util_avg, se->avg.util_avg);
            sub_positive(&cfs_rq->avg.util_sum, se->avg.util_sum);
            /* See update_cfs_rq_load_avg() */
            cfs_rq->avg.util_sum = max_t(u32, cfs_rq->avg.util_sum,
                            cfs_rq->avg.util_avg * PELT_MIN_DIVIDER);

            sub_positive(&cfs_rq->avg.runnable_avg, se->avg.runnable_avg);
            sub_positive(&cfs_rq->avg.runnable_sum, se->avg.runnable_sum);
            /* See update_cfs_rq_load_avg() */
            cfs_rq->avg.runnable_sum = max_t(u32, cfs_rq->avg.runnable_sum,
                                cfs_rq->avg.runnable_avg * PELT_MIN_DIVIDER);

            add_tg_cfs_propagate(cfs_rq, -se->avg.load_sum);

            cfs_rq_util_change(cfs_rq, 0);
        }

        update_tg_load_avg(cfs_rq);
    } else if (decayed) {
        cfs_rq_util_change(cfs_rq, 0) {
            struct rq *rq = rq_of(cfs_rq);
            if (&rq->cfs == cfs_rq) {
                cpufreq_update_util(rq, flags);
            }
        }
        if (flags & UPDATE_TG) {
            update_tg_load_avg(cfs_rq);
        }
    }
}
```

### update_cfs_rq_load_avg

```c
static inline int
update_cfs_rq_load_avg(u64 now, struct cfs_rq *cfs_rq)
{
    unsigned long removed_load = 0, removed_util = 0, removed_runnable = 0;
    struct sched_avg *sa = &cfs_rq->avg;
    int decayed = 0;

    if (cfs_rq->removed.nr) {
        unsigned long r;
        u32 divider = get_pelt_divider(&cfs_rq->avg);

        raw_spin_lock(&cfs_rq->removed.lock);
        swap(cfs_rq->removed.util_avg, removed_util);
        swap(cfs_rq->removed.load_avg, removed_load);
        swap(cfs_rq->removed.runnable_avg, removed_runnable);
        cfs_rq->removed.nr = 0;
        raw_spin_unlock(&cfs_rq->removed.lock);

        r = removed_load;
        sub_positive(&sa->load_avg, r);
        sub_positive(&sa->load_sum, r * divider);
        /* See sa->util_sum below */
        sa->load_sum = max_t(u32, sa->load_sum, sa->load_avg * PELT_MIN_DIVIDER);

        r = removed_util;
        sub_positive(&sa->util_avg, r);
        sub_positive(&sa->util_sum, r * divider);
        sa->util_sum = max_t(u32, sa->util_sum, sa->util_avg * PELT_MIN_DIVIDER);

        r = removed_runnable;
        sub_positive(&sa->runnable_avg, r);
        sub_positive(&sa->runnable_sum, r * divider);
        /* See sa->util_sum above */
        sa->runnable_sum = max_t(u32, sa->runnable_sum,
                            sa->runnable_avg * PELT_MIN_DIVIDER);

        add_tg_cfs_propagate(cfs_rq,
            -(long)(removed_runnable * divider) >> SCHED_CAPACITY_SHIFT);

        decayed = 1;
    }

    decayed |= __update_load_avg_cfs_rq(now, cfs_rq) {
        if (___update_load_sum(now, &cfs_rq->avg,
            scale_load_down(cfs_rq->load.weight),
            cfs_rq->h_nr_running,
            cfs_rq->curr != NULL)) {

            ___update_load_avg(&cfs_rq->avg, 1);
            return 1;
        }

        return 0;
    }
    u64_u32_store_copy(sa->last_update_time,
                cfs_rq->last_update_time_copy,
                sa->last_update_time);
    return decayed;
}
```

## rq_clock

```c
struct rq {
    u64             clock;  /* real clock of rq */
    u64             clock_task; /* update only when task running */
    /* based on clock_task, align to the largest core with highest frequency
     * only updated when task running exclude intr and idle time */
    u64             clock_pelt;

    u64             clock_idle;
    u64             clock_pelt_idle;
    unsigned long   lost_idle_time;
};
```

```c
update_rq_clock(rq) {
    s64 delta;

    if (rq->clock_update_flags & RQCF_ACT_SKIP)
        return;

    delta = sched_clock_cpu(cpu_of(rq)) - rq->clock;
    if (delta < 0)
        return;

    rq->clock += delta;

    update_rq_clock_task(rq, delta) {
        s64 __maybe_unused steal = 0, irq_delta = 0;

    #ifdef CONFIG_IRQ_TIME_ACCOUNTING
        irq_delta = irq_time_read(cpu_of(rq)) - rq->prev_irq_time;

        if (irq_delta > delta)
            irq_delta = delta;

        rq->prev_irq_time += irq_delta;
        delta -= irq_delta;
        psi_account_irqtime(rq->curr, irq_delta);
        delayacct_irq(rq->curr, irq_delta);
    #endif

        /* real task exec clock exclude irq */
        rq->clock_task += delta;

        update_rq_clock_pelt(rq, delta) {
            if (unlikely(is_idle_task(rq->curr))) {
                /* sync back to clock_task when rq is idle. */
                _update_idle_rq_clock_pelt(rq) {
                    rq->clock_pelt  = rq_clock_task(rq);
                    u64_u32_store(rq->clock_idle, rq_clock(rq));
                    u64_u32_store(rq->clock_pelt_idle, rq_clock_pelt(rq));
                }
                return;
            }

            /* Scale the elapsed time to reflect the real amount of computation */
            delta = cap_scale(delta, arch_scale_cpu_capacity(cpu_of(rq)));
            delta = cap_scale(delta, arch_scale_freq_capacity(cpu_of(rq)));

            rq->clock_pelt += delta;
        }
    }
}
```

# uclap

![](../images/kernel/proc-sched-uclamp.png)

* [dumpstack - uclamp](http://www.dumpstack.cn/index.php/2022/08/13/788.html)

# load_balance

![](../images/kernel/proc-load_balance.svg)

* [蜗窝科技 - CFS负载均衡 - :one:概述](http://www.wowotech.net/process_management/load_balance.html)    [:two: 任务放置](http://www.wowotech.net/process_management/task_placement.html)    [:three: CFS选核](http://www.wowotech.net/process_management/task_placement_detail.html)    [:four: load balance触发场景](http://www.wowotech.net/process_management/load_balance_detail.html)    [:five: load_balance](http://www.wowotech.net/process_management/load_balance_function.html)

---

1. Load balance
    * tick_balance

        ![](../images/kernel/proc-sched-lb-tick-balance.png)

    * sched_balance_newidle

        ![](../images/kernel/proc-sched-lb-newidle-balance.png)

    * nohzidle_banlance

        ![](../images/kernel/proc-sched-lb-nohzidle-balance.png)

2. Task placement - select_task_rq
    * try_to_wake_up
    * wake_up_new_task
    * sched_exec
    * ![](../images/kernel/proc-sched-lb-select_task_rq.png)


**Call stack**

```c
load_balance() {
    ret = should_we_balance(&env) {
        /* 1. check_cpumaks
         * 2. idle cpu and have no tasks and wakeup pending
         * 3. first idle cpu with busy sibling
         * 4. first idle cpu of this group */
    }
    if (!ret) {
        return;
    }

    group = sched_balance_find_src_group(&env) {
        update_sd_lb_stats() {
            do {
                update_sg_lb_stats() {
                    for_each_cpu_and() {
                        sgs->group_load += load;
                        sgs->group_util += cpu_util_cfs(i)
                        sgs->group_runnable += cpu_runnable(rq);
                        sgs->sum_h_nr_running += ;
                        sgs->group_type = group_classify();
                        /* ... */
                        if (update_sd_pick_busiest(env, sds, sg, sgs)) {
                            sds->busiest = sg;
                            sds->busiest_stat = *sgs;
                        }
                    }
                }
            } while (sg != env->sd->groups);
        }

        /* Decision matrix according to the local and busiest group type:
         *
         * busiest \ local has_spare   fully   misfit asym imbalanced overloaded
         * has_spare        nr_idle   balanced   N/A    N/A  balanced   balanced
         * fully_busy       nr_idle   nr_idle    N/A    N/A  balanced   balanced
         * misfit_task      force     N/A        N/A    N/A  N/A        N/A
         * asym_packing     force     force      N/A    N/A  force      force
         * imbalanced       force     force      N/A    N/A  force      force
         * overloaded       force     force      N/A    N/A  force      avg_load */

        calculate_imbalance() {
            /* group_type           migration_type  imbalance                       note
             *              busiest cpu
             * group_misfit_task    migrate_misfit      1                       SD_ASYM_CPUCAPACITY
             * group_misfit_task    migrate_load    group_misfit_task_load

             * group_asym_packing   migrate_task    sum_h_nr_running
             * group_smt_balance    migrate_task        1
             * group_imbalanced     migrate_task        1

             *              local cpu
             * group_has_spare      migrate_util
             * group_has_spare      migrate_task        1
             * group_overloaded     migrate_load    average_load delta
            */
        }
    }

    busiest = sched_balance_find_src_rq(&env, group) {

    }

    cur_ld_moved = detach_tasks(&env) {

    }
    attach_tasks(&env) {
        activate_task();
        wakeup_preempt();
    }

}
```

## tick_balance

```c
void scheduler_tick(void){
    rq->idle_balance = idle_cpu(cpu);
    sched_balance_trigger(rq) {
        if (unlikely(on_null_domain(rq) || !cpu_active(cpu_of(rq))))
            return;

        if (time_after_eq(jiffies, rq->next_balance))
            raise_softirq(SCHED_SOFTIRQ);

        nohz_balancer_kick(rq) {
            --->
        }
    }
}

init_sched_fair_class(void) {
    open_softirq(SCHED_SOFTIRQ, run_rebalance_domains);
}

void run_rebalance_domains(struct softirq_action *h)
{
    struct rq *this_rq = this_rq();
    enum cpu_idle_type idle = this_rq->idle_balance
        ? CPU_IDLE : CPU_NOT_IDLE;

    if (nohz_idle_balance(this_rq, idle)) {
        return;
    }

    update_blocked_averages(this_rq->cpu) {
        bool decayed = false, done = true;
        struct rq *rq = cpu_rq(cpu);
        struct rq_flags rf;

        rq_lock_irqsave(rq, &rf);
        update_blocked_load_tick(rq) {
            WRITE_ONCE(rq->last_blocked_load_update_tick, jiffies);
        }
        update_rq_clock(rq);

        decayed |= __update_blocked_others(rq, &done) {
            const struct sched_class *curr_class;
            u64 now = rq_clock_pelt(rq);
            unsigned long thermal_pressure;
            bool decayed;
            curr_class = rq->curr->sched_class;

            thermal_pressure = arch_scale_thermal_pressure(cpu_of(rq));

            decayed |= update_rt_rq_load_avg(now, rq, curr_class == &rt_sched_class);
            decayed |= update_dl_rq_load_avg(now, rq, curr_class == &dl_sched_class);
            decayed |= update_thermal_load_avg(rq_clock_thermal(rq), rq, thermal_pressure);
            decayed |= update_irq_load_avg(rq, 0) {
                running = cap_scale(running, arch_scale_freq_capacity(cpu_of(rq)));
                running = cap_scale(running, arch_scale_cpu_capacity(cpu_of(rq)));

                /* before (rq->clock - running) cpu exec the normal task not interrupt work
                 * 0: just decay the past time do not accumulate */
                ret = ___update_load_sum(rq->clock - running, &rq->avg_irq,
                            0, 0, 0);

                /* [rq->clock - running, rq->clock] cpu exec interrupt work
                 * 1: decay interrupt time */
                ret += ___update_load_sum(rq->clock, &rq->avg_irq, 1, 1, 1);
                if (ret) {
                    ___update_load_avg(&rq->avg_irq, 1);
                }
            }

            if (others_have_blocked(rq))
                *done = false;

            return decayed;
        }

        decayed |= __update_blocked_fair(rq, &done) {
            struct cfs_rq *cfs_rq = &rq->cfs;
            bool decayed;

            decayed = update_cfs_rq_load_avg(cfs_rq_clock_pelt(cfs_rq), cfs_rq);
            if (cfs_rq_has_blocked(cfs_rq))
                *done = false;

            return decayed;
        }

        update_blocked_load_status(rq, !done);
        if (decayed) {
            cpufreq_update_util(rq, 0);
        }
        rq_unlock_irqrestore(rq, &rf);
    }

    rebalance_domains(this_rq, idle) {
        int continue_balancing = 1;
        int cpu = rq->cpu;
        int busy = idle != CPU_IDLE && !sched_idle_cpu(cpu);
        unsigned long interval;
        struct sched_domain *sd;
        /* Earliest time when we have to do rebalance again */
        unsigned long next_balance = jiffies + 60*HZ;
        int update_next_balance = 0;
        int need_serialize, need_decay = 0;
        u64 max_cost = 0;

        rcu_read_lock();
        for_each_domain(cpu, sd) {
            need_decay = update_newidle_cost(sd, 0/*cost*/)  {
                if (cost > sd->max_newidle_lb_cost) {
                    sd->max_newidle_lb_cost = cost;
                    sd->last_decay_max_lb_cost = jiffies;
                } else if (time_after(jiffies, sd->last_decay_max_lb_cost + HZ)) {
                    sd->max_newidle_lb_cost = (sd->max_newidle_lb_cost * 253) / 256;
                    sd->last_decay_max_lb_cost = jiffies;
                    return true;
                }

                return false;
            }
            max_cost += sd->max_newidle_lb_cost;

            if (!continue_balancing) {
                if (need_decay)
                    continue;
                break;
            }

            interval = get_sd_balance_interval(sd, busy) {
                unsigned long interval = sd->balance_interval;

                if (cpu_busy)
                    interval *= sd->busy_factor;

                interval = msecs_to_jiffies(interval);
                if (cpu_busy)
                    interval -= 1;

                return clamp(interval, 1UL, max_load_balance_interval);
            }

            need_serialize = sd->flags & SD_SERIALIZE;
            if (need_serialize) {
                if (!spin_trylock(&balancing))
                    goto out;
            }

            if (time_after_eq(jiffies, sd->last_balance + interval)) {
                ret = load_balance(cpu, rq, sd, idle, &continue_balancing);
                    --->
                if (ret) {
                    idle = idle_cpu(cpu) ? CPU_IDLE : CPU_NOT_IDLE;
                    busy = idle != CPU_IDLE && !sched_idle_cpu(cpu);
                }
                sd->last_balance = jiffies;
                interval = get_sd_balance_interval(sd, busy);
            }
            if (need_serialize)
                spin_unlock(&balancing);
    out:
            if (time_after(next_balance, sd->last_balance + interval)) {
                next_balance = sd->last_balance + interval;
                update_next_balance = 1;
            }
        }
        if (need_decay) {
            rq->max_idle_balance_cost =
                max((u64)sysctl_sched_migration_cost, max_cost);
        }
        rcu_read_unlock();

        if (likely(update_next_balance))
            rq->next_balance = next_balance;
    }
}
```

## nohz_idle_balance

```c
static struct {
    cpumask_var_t idle_cpus_mask;
    atomic_t nr_cpus;
    int has_blocked; /* Idle CPUS has blocked load */
    int needs_update; /* Newly idle CPUs need their next_balance collated */
    unsigned long next_balance; /* in jiffy units */
    unsigned long next_blocked; /* Next update of blocked load in jiffies */
} nohz;

nohz_balancer_kick(rq) {
    unsigned long now = jiffies;
    struct sched_domain_shared *sds;
    struct sched_domain *sd;
    int nr_busy, i, cpu = rq->cpu;
    unsigned int flags = 0;

    if (unlikely(rq->idle_balance))
        return;

    nohz_balance_exit_idle(rq) {
        if (likely(!rq->nohz_tick_stopped))
            return;

        rq->nohz_tick_stopped = 0;
        cpumask_clear_cpu(rq->cpu, nohz.idle_cpus_mask);
        atomic_dec(&nohz.nr_cpus);

        set_cpu_sd_state_busy(rq->cpu) {
            struct sched_domain *sd;

            rcu_read_lock();
            sd = rcu_dereference(per_cpu(sd_llc, cpu));

            if (!sd || !sd->nohz_idle)
                goto unlock;
            sd->nohz_idle = 0;

            atomic_inc(&sd->shared->nr_busy_cpus);
        unlock:
            rcu_read_unlock();
        }
    }

    if (likely(!atomic_read(&nohz.nr_cpus)))
        return;

    if (READ_ONCE(nohz.has_blocked) && time_after(now, READ_ONCE(nohz.next_blocked)))
        flags = NOHZ_STATS_KICK;

    if (time_before(now, nohz.next_balance))
        goto out;

    if (rq->nr_running >= 2) {
        flags = NOHZ_STATS_KICK | NOHZ_BALANCE_KICK;
        goto out;
    }

    rcu_read_lock();

    sd = rcu_dereference(rq->sd);
    if (sd) {
        ret = check_cpu_capacity(rq, sd) {
            return ((rq->cpu_capacity * sd->imbalance_pct) < (rq->cpu_capacity_orig * 100));
        }
        if (rq->cfs.h_nr_running >= 1 && ret) {
            flags = NOHZ_STATS_KICK | NOHZ_BALANCE_KICK;
            goto unlock;
        }
    }

    sd = rcu_dereference(per_cpu(sd_asym_packing, cpu));
    if (sd) {
        for_each_cpu_and(i, sched_domain_span(sd), nohz.idle_cpus_mask) {
            if (sched_use_asym_prio(sd, i) && sched_asym_prefer(i, cpu)) {
                flags = NOHZ_STATS_KICK | NOHZ_BALANCE_KICK;
                goto unlock;
            }
        }
    }

    sd = rcu_dereference(per_cpu(sd_asym_cpucapacity, cpu));
    if (sd) {
        if (check_misfit_status(rq, sd)) {
            flags = NOHZ_STATS_KICK | NOHZ_BALANCE_KICK;
            goto unlock;
        }
        goto unlock;
    }

    sds = rcu_dereference(per_cpu(sd_llc_shared, cpu));
    if (sds) {
        nr_busy = atomic_read(&sds->nr_busy_cpus);
        if (nr_busy > 1) {
            flags = NOHZ_STATS_KICK | NOHZ_BALANCE_KICK;
            goto unlock;
        }
    }
unlock:
    rcu_read_unlock();
out:
    if (READ_ONCE(nohz.needs_update))
        flags |= NOHZ_NEXT_KICK;

    if (flags) {
        kick_ilb(flags) {
            int ilb_cpu;

            if (flags & NOHZ_BALANCE_KICK)
                nohz.next_balance = jiffies+1;

            ilb_cpu = find_new_ilb() {
                int ilb;
                const struct cpumask *hk_mask;

                hk_mask = housekeeping_cpumask(HK_TYPE_MISC);

                for_each_cpu_and(ilb, nohz.idle_cpus_mask, hk_mask) {
                    if (ilb == smp_processor_id())
                        continue;

                    ret = idle_cpu(ilb) {
                        struct rq *rq = cpu_rq(cpu);
                        if (rq->curr != rq->idle)
                            return 0;
                        if (rq->nr_running)
                            return 0;
                        if (rq->ttwu_pending)
                            return 0;
                        return 1;;
                    }
                    if (ret)
                        return ilb;
                }

                return nr_cpu_ids;
            }

            if (ilb_cpu >= nr_cpu_ids)
                return;

            flags = atomic_fetch_or(flags, nohz_flags(ilb_cpu));
            if (flags & NOHZ_KICK_MASK)
                return;

            smp_call_function_single_async(ilb_cpu, &cpu_rq(ilb_cpu)->nohz_csd) {
                arch_send_call_function_single_ipi() {
                    smp_cross_call(cpumask_of(cpu), IPI_CALL_FUNC) {
                        __ipi_send_mask(ipi_desc[ipinr], target);
                    }
                }
            }
        }
    }
}
```

```c
nohz_idle_balance(this_rq, idle) {
    unsigned int flags = this_rq->nohz_idle_balance;

    if (!flags)
        return false;

    this_rq->nohz_idle_balance = 0;

    if (idle != CPU_IDLE)
        return false;

    _nohz_idle_balance(this_rq, flags) {
        /* Earliest time when we have to do rebalance again */
        unsigned long now = jiffies;
        unsigned long next_balance = now + 60*HZ;
        bool has_blocked_load = false;
        int update_next_balance = 0;
        int this_cpu = this_rq->cpu;
        int balance_cpu;
        struct rq *rq;

        SCHED_WARN_ON((flags & NOHZ_KICK_MASK) == NOHZ_BALANCE_KICK);

        if (flags & NOHZ_STATS_KICK)
            WRITE_ONCE(nohz.has_blocked, 0);
        if (flags & NOHZ_NEXT_KICK)
            WRITE_ONCE(nohz.needs_update, 0);

        smp_mb();

        for_each_cpu_wrap(balance_cpu,  nohz.idle_cpus_mask, this_cpu+1) {
            if (!idle_cpu(balance_cpu))
                continue;

            if (need_resched()) {
                if (flags & NOHZ_STATS_KICK)
                    has_blocked_load = true;
                if (flags & NOHZ_NEXT_KICK)
                    WRITE_ONCE(nohz.needs_update, 1);
                goto abort;
            }

            rq = cpu_rq(balance_cpu);

            if (flags & NOHZ_STATS_KICK) {
                has_blocked_load |= update_nohz_stats(rq) {
                    unsigned int cpu = rq->cpu;

                    if (!rq->has_blocked_load)
                        return false;

                    if (!cpumask_test_cpu(cpu, nohz.idle_cpus_mask))
                        return false;

                    if (!time_after(jiffies, READ_ONCE(rq->last_blocked_load_update_tick)))
                        return true;

                    update_blocked_averages(cpu);

                    return rq->has_blocked_load;
                }
            }

            if (time_after_eq(jiffies, rq->next_balance)) {
                struct rq_flags rf;

                rq_lock_irqsave(rq, &rf);
                update_rq_clock(rq);
                rq_unlock_irqrestore(rq, &rf);

                if (flags & NOHZ_BALANCE_KICK) {
                    rebalance_domains(rq, CPU_IDLE);
                        --->
                }
            }

            if (time_after(next_balance, rq->next_balance)) {
                next_balance = rq->next_balance;
                update_next_balance = 1;
            }
        }

        if (likely(update_next_balance))
            nohz.next_balance = next_balance;

        if (flags & NOHZ_STATS_KICK)
            WRITE_ONCE(nohz.next_blocked, now + msecs_to_jiffies(LOAD_AVG_PERIOD));

    abort:
        /* There is still blocked load, enable periodic update */
        if (has_blocked_load)
            WRITE_ONCE(nohz.has_blocked, 1);
    }
}
```

## sched_balance_newidle

```c
int sched_balance_newidle(rq, rf) {
    unsigned long next_balance = jiffies + HZ;
    int this_cpu = this_rq->cpu;
    u64 t0, t1, curr_cost = 0;
    struct sched_domain *sd;
    int pulled_task = 0;

    update_misfit_status(NULL/*p*/, this_rq/*rq*/) {
        if (!sched_asym_cpucap_active())
            return;

        if (!p || p->nr_cpus_allowed == 1) {
            rq->misfit_task_load = 0;
            return;
        }

        if (task_fits_cpu(p, cpu_of(rq))) {
            rq->misfit_task_load = 0;
            return;
        }

        rq->misfit_task_load = max_t(unsigned long, task_h_load(p), 1);
    }

    if (this_rq->ttwu_pending)
        return 0;

    this_rq->idle_stamp = rq_clock(this_rq);

    if (!cpu_active(this_cpu))
        return 0;

    rq_unpin_lock(this_rq, rf);

    rcu_read_lock();
    sd = rcu_dereference_check_sched_domain(this_rq->sd);

    if (!READ_ONCE(this_rq->rd->overload) || (sd && this_rq->avg_idle < sd->max_newidle_lb_cost)) {
        if (sd) {
            update_next_balance(sd, &next_balance) {
                unsigned long interval, next;

                interval = get_sd_balance_interval(sd, 0);
                next = sd->last_balance + interval;

                if (time_after(*next_balance, next))
                    *next_balance = next;
            }
        }
        rcu_read_unlock();

        goto out;
    }
    rcu_read_unlock();

    raw_spin_rq_unlock(this_rq);

    t0 = sched_clock_cpu(this_cpu);
    update_blocked_averages(this_cpu);

    rcu_read_lock();
    for_each_domain(this_cpu, sd) {
        int continue_balancing = 1;
        u64 domain_cost;

        update_next_balance(sd, &next_balance);

        if (this_rq->avg_idle < curr_cost + sd->max_newidle_lb_cost)
            break;

        if (sd->flags & SD_BALANCE_NEWIDLE) {
            pulled_task = load_balance(this_cpu, this_rq, sd, CPU_NEWLY_IDLE, &continue_balancing);
                --->
            t1 = sched_clock_cpu(this_cpu);
            domain_cost = t1 - t0;
            update_newidle_cost(sd, domain_cost);

            curr_cost += domain_cost;
            t0 = t1;
        }

        if (pulled_task || this_rq->nr_running > 0 || this_rq->ttwu_pending)
            break;
    }
    rcu_read_unlock();

    raw_spin_rq_lock(this_rq);

    if (curr_cost > this_rq->max_idle_balance_cost)
        this_rq->max_idle_balance_cost = curr_cost;

    if (this_rq->cfs.h_nr_running && !pulled_task)
        pulled_task = 1;

    /* Is there a task of a high priority class? */
    if (this_rq->nr_running != this_rq->cfs.h_nr_running)
        pulled_task = -1;

out:
    /* Move the next balance forward */
    if (time_after(this_rq->next_balance, next_balance))
        this_rq->next_balance = next_balance;

    if (pulled_task)
        this_rq->idle_stamp = 0;
    else
        nohz_newidle_balance(this_rq);

    rq_repin_lock(this_rq, rf);

    return pulled_task;
}
```

## sched_balance_rq

![](../images/kernel/proc-sched-load_balance.svg)

```c
ret = sched_balance_rq(cpu, rq, sd, idle, &continue_balancing) {
    int ld_moved, cur_ld_moved, active_balance = 0;
    struct sched_domain *sd_parent = sd->parent;
    struct sched_group *group;
    struct rq *busiest;
    struct rq_flags rf;
    struct cpumask *cpus = this_cpu_cpumask_var_ptr(load_balance_mask);
    struct lb_env env = {
        .sd             = sd,
        .dst_cpu        = this_cpu,
        .dst_rq         = this_rq,
        .dst_grpmask    = group_balance_mask(sd->groups),
        .idle           = idle,
        .loop_break     = SCHED_NR_MIGRATE_BREAK,
        .cpus           = cpus,
        .fbq_type       = all,
        .tasks          = LIST_HEAD_INIT(env.tasks),
    };

    cpumask_and(cpus, sched_domain_span(sd), cpu_active_mask);

redo:
    if (!should_we_balance(&env)) {
        *continue_balancing = 0;
        goto out_balanced;
    }

    group = sched_balance_find_src_group(&env);
    busiest = sched_balance_find_src_rq(&env, group);

    env.src_cpu = busiest->cpu;
    env.src_rq = busiest;

    ld_moved = 0;
    /* Clear this flag as soon as we find a pullable task */
    env.flags |= LBF_ALL_PINNED;
    if (busiest->nr_running > 1) {
        env.loop_max  = min(sysctl_sched_nr_migrate, busiest->nr_running);

more_balance:
        rq_lock_irqsave(busiest, &rf);
        update_rq_clock(busiest);

        /* cur_ld_moved - load moved in current iteration
         * ld_moved     - cumulative load moved across iterations */
        cur_ld_moved = detach_tasks(&env);

        rq_unlock(busiest, &rf);

        if (cur_ld_moved) {
            attach_tasks(&env) {
                struct list_head *tasks = &env->tasks;
                struct task_struct *p;
                struct rq_flags rf;

                rq_lock(env->dst_rq, &rf);
                update_rq_clock(env->dst_rq);

                while (!list_empty(tasks)) {
                    p = list_first_entry(tasks, struct task_struct, se.group_node);
                    list_del_init(&p->se.group_node);

                    attach_task(env->dst_rq, p) {
                        activate_task(rq, p, ENQUEUE_NOCLOCK);
                        wakeup_preempt(rq, p, 0);
                    }
                }

                rq_unlock(env->dst_rq, &rf);
            }
            ld_moved += cur_ld_moved;
        }

        local_irq_restore(rf.flags);

        if (env.flags & LBF_NEED_BREAK) {
            env.flags &= ~LBF_NEED_BREAK;
            if (env.loop < busiest->nr_running)
                goto more_balance;
        }

        if ((env.flags & LBF_DST_PINNED) && env.imbalance > 0) {
            /* Prevent to re-select dst_cpu via env's CPUs */
            __cpumask_clear_cpu(env.dst_cpu, env.cpus);

            env.dst_rq  = cpu_rq(env.new_dst_cpu);
            env.dst_cpu = env.new_dst_cpu;
            env.flags   &= ~LBF_DST_PINNED;
            env.loop    = 0;
            env.loop_break     = SCHED_NR_MIGRATE_BREAK;

            goto more_balance;
        }

        if (sd_parent) {
            /* notify parent the imbalance of child */
            int *group_imbalance = &sd_parent->groups->sgc->imbalance;
            if ((env.flags & LBF_SOME_PINNED) && env.imbalance > 0)
                *group_imbalance = 1;
        }

        /* All tasks on this runqueue were pinned by CPU affinity */
        if (unlikely(env.flags & LBF_ALL_PINNED)) {
            __cpumask_clear_cpu(cpu_of(busiest), cpus);

            if (!cpumask_subset(cpus, env.dst_grpmask)) {
                env.loop = 0;
                env.loop_break = SCHED_NR_MIGRATE_BREAK;
                goto redo;
            }
            goto out_all_pinned;
        }
    }

    if (!ld_moved) {
        schedstat_inc(sd->lb_failed[idle]);

        if (idle != CPU_NEWLY_IDLE)
            sd->nr_balance_failed++;

        /* migrate cur task */
        if (need_active_balance(&env)) {
            unsigned long flags;

            raw_spin_rq_lock_irqsave(busiest, flags);

            if (!cpumask_test_cpu(this_cpu, busiest->curr->cpus_ptr)) {
                raw_spin_rq_unlock_irqrestore(busiest, flags);
                goto out_one_pinned;
            }

            env.flags &= ~LBF_ALL_PINNED;

            if (!busiest->active_balance) {
                busiest->active_balance = 1;
                busiest->push_cpu = this_cpu;
                active_balance = 1;
            }
            raw_spin_rq_unlock_irqrestore(busiest, flags);

            if (active_balance) {
                stop_one_cpu_nowait(
                    cpu_of(busiest)/*cpu*/,
                    active_load_balance_cpu_stop/*fn*/,
                    busiest,/*arg*/
                    &busiest->active_balance_work/*work_buf*/) {

                    *work_buf = (struct cpu_stop_work){ .fn = fn, .arg = arg, .caller = _RET_IP_, };
                    return cpu_stop_queue_work(cpu, work_buf);
                }
            }
        }
    } else {
        sd->nr_balance_failed = 0;
    }

    if (likely(!active_balance) || need_active_balance(&env)) {
        /* We were unbalanced, so reset the balancing interval */
        sd->balance_interval = sd->min_interval;
    }

    goto out;

out_balanced:
    if (sd_parent && !(env.flags & LBF_ALL_PINNED)) {
        int *group_imbalance = &sd_parent->groups->sgc->imbalance;
        if (*group_imbalance) {
            *group_imbalance = 0;
        }
    }

out_all_pinned:
    schedstat_inc(sd->lb_balanced[idle]);

    sd->nr_balance_failed = 0;

out_one_pinned:
    ld_moved = 0;

    if (env.idle == CPU_NEWLY_IDLE)
        goto out;

    /* tune up the balancing interval */
    if ((env.flags & LBF_ALL_PINNED &&
        sd->balance_interval < MAX_PINNED_INTERVAL) ||
        sd->balance_interval < sd->max_interval)
        sd->balance_interval *= 2;
out:
    return ld_moved;
}
```

### should_we_balance

```c
int should_we_balance(struct lb_env *env)
{
    struct cpumask *swb_cpus = this_cpu_cpumask_var_ptr(should_we_balance_tmpmask);
    struct sched_group *sg = env->sd->groups;
    int cpu, idle_smt = -1;

    /* Ensure the balancing environment is consistent; can happen
     * when the softirq triggers 'during' hotplug. */
    if (!cpumask_test_cpu(env->dst_cpu, env->cpus)) {
        return 0;
    }

    /* In the newly idle case, we will allow all the CPUs
     * to do the newly idle load balance.
     *
     * However, we bail out if we already have tasks or a wakeup pending,
     * to optimize wakeup latency. */
    if (env->idle == CPU_NEWLY_IDLE) {
        if (env->dst_rq->nr_running > 0 || env->dst_rq->ttwu_pending) {
            return 0;
        }
        return 1;
    }

    cpumask_copy(swb_cpus, group_balance_mask(sg));
    /* Try to find first idle CPU */
    for_each_cpu_and(cpu, swb_cpus, env->cpus) {
        if (!idle_cpu(cpu)) {
            continue;
        }

        /* Don't balance to idle SMT in busy core right away when
         * balancing cores, but remember the first idle SMT CPU for
         * later consideration.  Find CPU on an idle core first. */
        if (!(env->sd->flags & SD_SHARE_CPUCAPACITY) && !is_core_idle(cpu)) {
            if (idle_smt == -1) {
                idle_smt = cpu;
            }
            /* If the core is not idle, and first SMT sibling which is
             * idle has been found, then its not needed to check other
             * SMT siblings for idleness: */
#ifdef CONFIG_SCHED_SMT
            cpumask_andnot(swb_cpus, swb_cpus, cpu_smt_mask(cpu));
#endif
            continue;
        }

        /* Are we the first idle core in a non-SMT domain or higher,
         * or the first idle CPU in a SMT domain? */
        return cpu == env->dst_cpu;
    }

    /* Are we the first idle CPU with busy siblings? */
    if (idle_smt != -1) {
        return idle_smt == env->dst_cpu;
    }

    /* Are we the first CPU of this group ? */
    return group_balance_cpu(sg) == env->dst_cpu;
}
```

### sched_balance_find_src_group

```c
/* Decision matrix according to the local and busiest group type:
 *
 * busiest \ local has_spare   fully    misfit asym imbalanced overloaded
 * has_spare        nr_idle   balanced   N/A    N/A  balanced   balanced
 * fully_busy       nr_idle   nr_idle    N/A    N/A  balanced   balanced
 * misfit_task      force     N/A        N/A    N/A  N/A        N/A
 * asym_packing     force     force      N/A    N/A  force      force
 * imbalanced       force     force      N/A    N/A  force      force
 * overloaded       force     force      N/A    N/A  force      avg_load
 *
 * N/A :      Not Applicable because already filtered while updating
 *            statistics.
 * balanced : The system is balanced for these 2 groups.
 * force :    Calculate the imbalance as load migration is probably needed.
 * avg_load : Only if imbalance is significant enough.
 * nr_idle :  dst_cpu is not busy and the number of idle CPUs is quite
 *            different in groups. */

struct sched_group *sched_balance_find_src_group(struct lb_env *env)
{
    struct sg_lb_stats *local, *busiest;
    struct sd_lb_stats sds;

    init_sd_lb_stats(&sds) {
        *sds = (struct sd_lb_stats){
            .busiest = NULL,
            .local = NULL,
            .total_load = 0UL,
            .total_capacity = 0UL,
            .busiest_stat = {
                .idle_cpus = UINT_MAX,
                .group_type = group_has_spare,
            },
        };
    }

    update_sd_lb_stats(env, &sds) {
        --->
    }

    if (!sds.busiest)
        goto out_balanced;

    busiest = &sds.busiest_stat;

    if (busiest->group_type == group_misfit_task)
        goto force_balance;
    if (busiest->group_type == group_asym_packing)
        goto force_balance;
    if (busiest->group_type == group_imbalanced)
        goto force_balance;

    if (sched_energy_enabled()) {
        struct root_domain *rd = env->dst_rq->rd;

        if (rcu_dereference(rd->pd) && !READ_ONCE(rd->overutilized))
            goto out_balanced;
    }

    local = &sds.local_stat;

    /* local group is busier than busiest group */
    if (local->group_type > busiest->group_type)
        goto out_balanced;

    if (local->group_type == group_overloaded) {
        if (local->avg_load >= busiest->avg_load)
            goto out_balanced;

        /* XXX broken for overlapping NUMA groups */
        sds.avg_load = (sds.total_load * SCHED_CAPACITY_SCALE)
            / sds.total_capacity;

        if (local->avg_load >= sds.avg_load)
            goto out_balanced;

        if (100 * busiest->avg_load <= env->sd->imbalance_pct * local->avg_load)
            goto out_balanced;
    }

    if (sds.prefer_sibling && local->group_type == group_has_spare
        && sibling_imbalance(env, &sds, busiest, local) > 1) {
        goto force_balance;
    }

    if (busiest->group_type != group_overloaded) {
        if (env->idle == CPU_NOT_IDLE) {
            goto out_balanced;
        }

        if (busiest->group_type == group_smt_balance
            && smt_vs_nonsmt_groups(sds.local, sds.busiest)) {
            /* Let non SMT CPU pull from SMT CPU sharing with sibling */
            goto force_balance;
        }

        if (busiest->group_weight > 1 && local->idle_cpus <= (busiest->idle_cpus + 1)) {
            goto out_balanced;
        }

        if (busiest->sum_h_nr_running == 1) {
            goto out_balanced;
        }
    }

force_balance:
    calculate_imbalance(env, &sds) {
        --->
    }
    return env->imbalance ? sds.busiest : NULL;

out_balanced:
    env->imbalance = 0;
    return NULL;
}
```

#### calculate_imbalance

```c
calculate_imbalance(env, &sds) {
    struct sg_lb_stats *local, *busiest;

    local = &sds->local_stat;
    busiest = &sds->busiest_stat;

    if (busiest->group_type == group_misfit_task) {
        if (env->sd->flags & SD_ASYM_CPUCAPACITY) {
            /* Set imbalance to allow misfit tasks to be balanced. */
            env->migration_type = migrate_misfit;
            env->imbalance = 1;
        } else {
            /* Set load imbalance to allow moving task from cpu
             * with reduced capacity. */
            env->migration_type = migrate_load;
            env->imbalance = busiest->group_misfit_task_load;
        }
        return;
    }

    if (busiest->group_type == group_asym_packing) {
        /* In case of asym capacity, we will try to migrate all load to
         * the preferred CPU. */
        env->migration_type = migrate_task;
        env->imbalance = busiest->sum_h_nr_running;
        return;
    }

    if (busiest->group_type == group_smt_balance) {
        /* Reduce number of tasks sharing CPU capacity */
        env->migration_type = migrate_task;
        env->imbalance = 1;
        return;
    }

    if (busiest->group_type == group_imbalanced) {
        env->migration_type = migrate_task;
        env->imbalance = 1;
        return;
    }

    if (local->group_type == group_has_spare) {
        if ((busiest->group_type > group_fully_busy)
            && !(env->sd->flags & SD_SHARE_PKG_RESOURCES)) {

            env->migration_type = migrate_util;
            env->imbalance = max(local->group_capacity, local->group_util) -
                    local->group_util;

            if (env->idle != CPU_NOT_IDLE && env->imbalance == 0) {
                env->migration_type = migrate_task;
                env->imbalance = 1;
            }

            return;
        }

        if (busiest->group_weight == 1 || sds->prefer_sibling) {
            /* When prefer sibling, evenly spread running tasks on groups. */
            env->migration_type = migrate_task;
            env->imbalance = sibling_imbalance(env, sds, busiest, local) {
                int ncores_busiest, ncores_local;
                long imbalance;

                if (env->idle == CPU_NOT_IDLE || !busiest->sum_nr_running)
                    return 0;

                ncores_busiest = sds->busiest->cores;
                ncores_local = sds->local->cores;

                if (ncores_busiest == ncores_local) {
                    imbalance = busiest->sum_nr_running;
                    lsub_positive(&imbalance, local->sum_nr_running);
                    return imbalance;
                }

                /* Balance such that nr_running/ncores ratio are same on both groups */
                imbalance = ncores_local * busiest->sum_nr_running;
                lsub_positive(&imbalance, ncores_busiest * local->sum_nr_running);
                /* Normalize imbalance and do rounding on normalization */
                imbalance = 2 * imbalance + ncores_local + ncores_busiest;
                imbalance /= ncores_local + ncores_busiest;

                /* Take advantage of resource in an empty sched group */
                if (imbalance <= 1 && local->sum_nr_running == 0
                    && busiest->sum_nr_running > 1) {

                    imbalance = 2;
                }

                return imbalance;
            }
        } else {
            /* If there is no overload, we just want to even the number of
             * idle cpus. */
            env->migration_type = migrate_task;
            env->imbalance = max_t(long, 0, (local->idle_cpus - busiest->idle_cpus));
        }

#ifdef CONFIG_NUMA
        /* Consider allowing a small imbalance between NUMA groups */
        if (env->sd->flags & SD_NUMA) {
            env->imbalance = adjust_numa_imbalance(
                env->imbalance,
                local->sum_nr_running + 1,
                env->sd->imb_numa_nr
            );
        }
#endif

        /* Number of tasks to move to restore balance */
        env->imbalance >>= 1;

        return;
    }

    /* Local is fully busy but has to take more load to relieve the
     * busiest group */
    if (local->group_type < group_overloaded) {
        /* Local will become overloaded so the avg_load metrics are
         * finally needed. */
        local->avg_load = (local->group_load * SCHED_CAPACITY_SCALE) /
                local->group_capacity;

        /* If the local group is more loaded than the selected
         * busiest group don't try to pull any tasks. */
        if (local->avg_load >= busiest->avg_load) {
            env->imbalance = 0;
            return;
        }

        sds->avg_load = (sds->total_load * SCHED_CAPACITY_SCALE) /
                sds->total_capacity;

        /* If the local group is more loaded than the average system
         * load, don't try to pull any tasks. */
        if (local->avg_load >= sds->avg_load) {
            env->imbalance = 0;
            return;
        }
    }

    /* Both group are or will become overloaded and we're trying to get all
     * the CPUs to the average_load, so we don't want to push ourselves
     * above the average load, nor do we wish to reduce the max loaded CPU
     * below the average load. At the same time, we also don't want to
     * reduce the group load below the group capacity. Thus we look for
     * the minimum possible imbalance. */
    env->migration_type = migrate_load;
    env->imbalance = min(
        (busiest->avg_load - sds->avg_load) * busiest->group_capacity,
        (sds->avg_load - local->avg_load) * local->group_capacity
    ) / SCHED_CAPACITY_SCALE;
}
```

#### update_sd_lb_stats

```c
update_sd_lb_stats(env, &sds) {
    struct sched_group *sg = env->sd->groups;
    struct sg_lb_stats *local = &sds->local_stat;
    struct sg_lb_stats tmp_sgs;
    unsigned long sum_util = 0;
    int sg_status = 0;

    do {
        struct sg_lb_stats *sgs = &tmp_sgs;
        int local_group;

        local_group = cpumask_test_cpu(env->dst_cpu, sched_group_span(sg));
        if (local_group) {
            sds->local = sg;
            sgs = local;

            if (env->idle != CPU_NEWLY_IDLE || time_after_eq(jiffies, sg->sgc->next_update)) {
                update_group_capacity(env->sd, env->dst_cpu) {
                    struct sched_domain *child = sd->child;
                    struct sched_group *group, *sdg = sd->groups;
                    unsigned long capacity, min_capacity, max_capacity;
                    unsigned long interval;

                    interval = msecs_to_jiffies(sd->balance_interval);
                    interval = clamp(interval, 1UL, max_load_balance_interval);
                    sdg->sgc->next_update = jiffies + interval;

                    if (!child) {
                        update_cpu_capacity(sd, cpu) {
                            /* capacity which only for cfs */
                            unsigned long capacity = scale_rt_capacity(cpu) {
                                struct rq *rq = cpu_rq(cpu);
                                unsigned long max = arch_scale_cpu_capacity(cpu);
                                unsigned long used, free;
                                unsigned long irq;

                                irq = cpu_util_irq(rq);

                                if (unlikely(irq >= max))
                                    return 1;

                                used = READ_ONCE(rq->avg_rt.util_avg);
                                used += READ_ONCE(rq->avg_dl.util_avg);
                                used += thermal_load_avg(rq);

                                if (unlikely(used >= max))
                                    return 1;

                                free = max - used;

                                return scale_irq_capacity(free, irq, max);
                            }
                            struct sched_group *sdg = sd->groups;

                            cpu_rq(cpu)->cpu_capacity_orig = arch_scale_cpu_capacity(cpu);

                            if (!capacity)
                                capacity = 1;

                            cpu_rq(cpu)->cpu_capacity = capacity;

                            sdg->sgc->capacity = capacity;
                            sdg->sgc->min_capacity = capacity;
                            sdg->sgc->max_capacity = capacity;
                        }
                        return;
                    }

                    capacity = 0;
                    min_capacity = ULONG_MAX;
                    max_capacity = 0;

                    if (child->flags & SD_OVERLAP) {
                        /* SD_OVERLAP domains cannot assume that child groups
                         * span the current group. */

                        for_each_cpu(cpu, sched_group_span(sdg)) {
                            unsigned long cpu_cap = capacity_of(cpu);

                            capacity += cpu_cap;
                            min_capacity = min(cpu_cap, min_capacity);
                            max_capacity = max(cpu_cap, max_capacity);
                        }
                    } else  {
                        /* !SD_OVERLAP domains can assume that child groups
                         * span the current group. */

                        group = child->groups;
                        do {
                            struct sched_group_capacity *sgc = group->sgc;

                            capacity += sgc->capacity;
                            min_capacity = min(sgc->min_capacity, min_capacity);
                            max_capacity = max(sgc->max_capacity, max_capacity);
                            group = group->next;
                        } while (group != child->groups);
                    }

                    sdg->sgc->capacity = capacity;
                    sdg->sgc->min_capacity = min_capacity;
                    sdg->sgc->max_capacity = max_capacity;
                }
            }
        }

        update_sg_lb_stats(env, sds, sg/*group*/, sgs, &sg_status) {
            int i, nr_running, local_group;

            memset(sgs, 0, sizeof(*sgs));

            local_group = group == sds->local;

            for_each_cpu_and(i, sched_group_span(group), env->cpus) {
                struct rq *rq = cpu_rq(i);
                unsigned long load = cpu_load(rq) {
                    return cfs_rq->avg.load_avg;
                }

                sgs->group_load += load;
                sgs->group_util += cpu_util_cfs(i) {
                    return cpu_util(
                        cpu, NULL,
                        -1, /* @dst_cpu: CPU @p migrates to, -1 if @p moves from @cpu or @p == NULL */
                        0 /* 1 to enable boosting, otherwise 0 */
                        ) {
                        struct cfs_rq *cfs_rq = &cpu_rq(cpu)->cfs;
                        unsigned long util = READ_ONCE(cfs_rq->avg.util_avg);
                        unsigned long runnable;

                        if (boost) {
                            runnable = READ_ONCE(cfs_rq->avg.runnable_avg);
                            util = max(util, runnable);
                        }

                        if (p && task_cpu(p) == cpu && dst_cpu != cpu)
                            lsub_positive(&util, task_util(p));
                        else if (p && task_cpu(p) != cpu && dst_cpu == cpu)
                            util += task_util(p);

                        if (sched_feat(UTIL_EST)) {
                            unsigned long util_est;

                            util_est = READ_ONCE(cfs_rq->avg.util_est);

                            if (dst_cpu == cpu)
                                util_est += _task_util_est(p);
                            else if (p && unlikely(task_on_rq_queued(p) || current == p))
                                lsub_positive(&util_est, _task_util_est(p));

                            util = max(util, util_est);
                        }

                        return min(util, arch_scale_cpu_capacity(cpu));
                    }
                }
                sgs->group_runnable += cpu_runnable(rq) {
                    return cfs_rq->avg.runnable_avg;
                }
                sgs->sum_h_nr_running += rq->cfs.h_nr_running;

                nr_running = rq->nr_running;
                sgs->sum_nr_running += nr_running;

                if (nr_running > 1)
                    *sg_status |= SG_OVERLOAD;

                if (cpu_overutilized(i))
                    *sg_status |= SG_OVERUTILIZED;

        #ifdef CONFIG_NUMA_BALANCING
                sgs->nr_numa_running += rq->nr_numa_running;
                sgs->nr_preferred_running += rq->nr_preferred_running;
        #endif
                /* No need to call idle_cpu() if nr_running is not 0 */
                if (!nr_running && idle_cpu(i)) {
                    sgs->idle_cpus++;
                    /* Idle cpu can't have misfit task */
                    continue;
                }

                if (local_group)
                    continue;

                if (env->sd->flags & SD_ASYM_CPUCAPACITY) {
                    /* Check for a misfit task on the cpu */
                    if (sgs->group_misfit_task_load < rq->misfit_task_load) {
                        sgs->group_misfit_task_load = rq->misfit_task_load;
                        *sg_status |= SG_OVERLOAD;
                    }
                } else if ((env->idle != CPU_NOT_IDLE) && sched_reduced_capacity(rq, env->sd)) {
                    /* Check for a task running on a CPU with reduced capacity */
                    if (sgs->group_misfit_task_load < load)
                        sgs->group_misfit_task_load = load;
                }
            }

            sgs->group_capacity = group->sgc->capacity;

            sgs->group_weight = group->group_weight; /* nr of cpu */

            /* Check if dst CPU is idle and preferred to this group */
            if (!local_group && env->sd->flags & SD_ASYM_PACKING
                && env->idle != CPU_NOT_IDLE && sgs->sum_h_nr_running
                && sched_asym(env, sds, sgs, group)) {
                sgs->group_asym_packing = 1;
            }

            /* Check for loaded SMT group to be balanced to dst CPU */
            if (!local_group && smt_balance(env, sgs, group))
                sgs->group_smt_balance = 1;

            sgs->group_type = group_classify(env->sd->imbalance_pct, group, sgs) {
                if (group_is_overloaded(imbalance_pct, sgs))
                    return group_overloaded;

                /* sub-sd failed to reach balance because of affinity */
                if (sg_imbalanced(group))
                    return group_imbalanced;

                if (sgs->group_asym_packing)
                    return group_asym_packing;

                if (sgs->group_smt_balance)
                    return group_smt_balance;

                if (sgs->group_misfit_task_load)
                    return group_misfit_task;

                ret = group_has_capacity(imbalance_pct, sgs) {
                    /* nr task is smaller than the nr CPUs
                     * the utilization is lower than the available capacity */
                    if (sgs->sum_nr_running < sgs->group_weight)
                        return true;

                    /* group_runnable > group_capacity imblance_pct */
                    if ((sgs->group_capacity * imbalance_pct) < (sgs->group_runnable * 100))
                        return false;

                    if ((sgs->group_util * imbalance_pct) < (sgs->group_capacity * 100))
                        return true;

                    return false;
                }
                if (!ret)
                    return group_fully_busy;

                return group_has_spare;
            }

            /* Computing avg_load makes sense only when group is overloaded */
            if (sgs->group_type == group_overloaded)
                sgs->avg_load = (sgs->group_load * SCHED_CAPACITY_SCALE) /
                        sgs->group_capacity;
        }

        if (local_group)
            goto next_group;


        if (update_sd_pick_busiest(env, sds, sg, sgs)) {
            sds->busiest = sg;
            sds->busiest_stat = *sgs;
        }

next_group:
        /* Now, start updating sd_lb_stats */
        sds->total_load += sgs->group_load;
        sds->total_capacity += sgs->group_capacity;

        sum_util += sgs->group_util;
        sg = sg->next;
    } while (sg != env->sd->groups);

    /* Indicate that the child domain of the busiest group prefers tasks
    * go to a child's sibling domains first. NB the flags of a sched group
    * are those of the child domain. */
    if (sds->busiest)
        sds->prefer_sibling = !!(sds->busiest->flags & SD_PREFER_SIBLING);


    if (env->sd->flags & SD_NUMA)
        env->fbq_type = fbq_classify_group(&sds->busiest_stat);

    if (!env->sd->parent) {
        struct root_domain *rd = env->dst_rq->rd;

        /* update overload indicator if we are at root domain */
        WRITE_ONCE(rd->overload, sg_status & SG_OVERLOAD);

        /* Update over-utilization (tipping point, U >= 0) indicator */
        WRITE_ONCE(rd->overutilized, sg_status & SG_OVERUTILIZED);
    } else if (sg_status & SG_OVERUTILIZED) {
        struct root_domain *rd = env->dst_rq->rd;

        WRITE_ONCE(rd->overutilized, SG_OVERUTILIZED);
    }

    update_idle_cpu_scan(env, sum_util);
}
```

#### update_sd_pick_busiest

```c
update_sd_pick_busiest(struct lb_env *env,
                struct sd_lb_stats *sds,
                struct sched_group *sg,
                struct sg_lb_stats *sgs)
{
    struct sg_lb_stats *busiest = &sds->busiest_stat;

    /* Make sure that there is at least one task to pull */
    if (!sgs->sum_h_nr_running)
        return false;

    /* Don't try to pull misfit tasks we can't help.
     * We can use max_capacity here as reduction in capacity on some
     * CPUs in the group should either be possible to resolve
     * internally or be covered by avg_load imbalance (eventually). */
    if ((env->sd->flags & SD_ASYM_CPUCAPACITY)
        && (sgs->group_type == group_misfit_task)
        && (!capacity_greater(capacity_of(env->dst_cpu), sg->sgc->max_capacity)
        || sds->local_stat.group_type != group_has_spare)) {

        return false;
    }

    if (sgs->group_type > busiest->group_type)
        return true;

    if (sgs->group_type < busiest->group_type)
        return false;

    /* sgs->group_type == busiest->group_type */
    switch (sgs->group_type) {
    case group_overloaded:
        /* Select the overloaded group with highest avg_load. */
        if (sgs->avg_load <= busiest->avg_load)
            return false;
        break;

    case group_imbalanced:
        /* Select the 1st imbalanced group as we don't have any way to
         * choose one more than another. */
        return false;

    case group_asym_packing:
        /* Prefer to move from lowest priority CPU's work */
        if (sched_asym_prefer(sg->asym_prefer_cpu, sds->busiest->asym_prefer_cpu))
            return false;
        break;

    case group_misfit_task:
        /* If we have more than one misfit sg go with the biggest misfit. */
        if (sgs->group_misfit_task_load < busiest->group_misfit_task_load)
            return false;
        break;

    case group_smt_balance:
        /* Check if we have spare CPUs on either SMT group to
         * choose has spare or fully busy handling. */
        if (sgs->idle_cpus != 0 || busiest->idle_cpus != 0)
            goto has_spare;

        fallthrough;

    case group_fully_busy:
        /* Select the fully busy group with highest avg_load. In
         * theory, there is no need to pull task from such kind of
         * group because tasks have all compute capacity that they need
         * but we can still improve the overall throughput by reducing
         * contention when accessing shared HW resources.
         *
         * XXX for now avg_load is not computed and always 0 so we
         * select the 1st one, except if @sg is composed of SMT
         * siblings. */

        if (sgs->avg_load < busiest->avg_load)
            return false;

        if (sgs->avg_load == busiest->avg_load) {
            /* SMT sched groups need more help than non-SMT groups.
             * If @sg happens to also be SMT, either choice is good. */
            if (sds->busiest->flags & SD_SHARE_CPUCAPACITY)
                return false;
        }

        break;

    case group_has_spare:
        /* Do not pick sg with SMT CPUs over sg with pure CPUs,
         * as we do not want to pull task off SMT core with one task
         * and make the core idle. */
        if (smt_vs_nonsmt_groups(sds->busiest, sg)) {
            if (sg->flags & SD_SHARE_CPUCAPACITY && sgs->sum_h_nr_running <= 1)
                return false;
            else
                return true;
        }
has_spare:

        /* Select not overloaded group with lowest number of idle cpus
         * and highest number of running tasks. We could also compare
         * the spare capacity which is more stable but it can end up
         * that the group has less spare capacity but finally more idle
         * CPUs which means less opportunity to pull tasks. */
        if (sgs->idle_cpus > busiest->idle_cpus)
            return false;
        else if ((sgs->idle_cpus == busiest->idle_cpus)
            && (sgs->sum_nr_running <= busiest->sum_nr_running)) {

            return false;
        }

        break;
    }

    /* Candidate sg has no more than one task per CPU and has higher
     * per-CPU capacity. Migrating tasks to less capable CPUs may harm
     * throughput. Maximize throughput, power/energy consequences are not
     * considered. */
    if ((env->sd->flags & SD_ASYM_CPUCAPACITY) &&
        (sgs->group_type <= group_fully_busy) &&
        (capacity_greater(sg->sgc->min_capacity, capacity_of(env->dst_cpu))))
        return false;

    return true;
}
```

### sched_balance_find_src_rq

```c
struct rq *sched_balance_find_src_rq(struct lb_env *env, struct sched_group *group) {
    struct rq *busiest = NULL, *rq;
    unsigned long busiest_util = 0, busiest_load = 0, busiest_capacity = 1;
    unsigned int busiest_nr = 0;
    int i;

    for_each_cpu_and(i, sched_group_span(group), env->cpus) {
        unsigned long capacity, load, util;
        unsigned int nr_running;
        enum fbq_type rt;

        rq = cpu_rq(i);
        rt = fbq_classify_rq(rq);

        if (rt > env->fbq_type)
            continue;

        nr_running = rq->cfs.h_nr_running;
        if (!nr_running)
            continue;

        capacity = capacity_of(i);

        if (env->sd->flags & SD_ASYM_CPUCAPACITY
            && !capacity_greater(capacity_of(env->dst_cpu), capacity)
            && nr_running == 1) {

            continue;
        }

        if ((env->sd->flags & SD_ASYM_PACKING)
            && sched_use_asym_prio(env->sd, i)
            && sched_asym_prefer(i, env->dst_cpu)
            && nr_running == 1) {

            continue;
        }

        switch (env->migration_type) {
        case migrate_load:
            load = cpu_load(rq);

            if (nr_running == 1 && load > env->imbalance && !check_cpu_capacity(rq, env->sd))
                break;

            if (load * busiest_capacity > busiest_load * capacity) {
                busiest_load = load;
                busiest_capacity = capacity;
                busiest = rq;
            }
            break;

        case migrate_util:
            util = cpu_util_cfs_boost(i);

            if (nr_running <= 1)
                continue;

            if (busiest_util < util) {
                busiest_util = util;
                busiest = rq;
            }
            break;

        case migrate_task:
            if (busiest_nr < nr_running) {
                busiest_nr = nr_running;
                busiest = rq;
            }
            break;

        case migrate_misfit:
            if (rq->misfit_task_load > busiest_load) {
                busiest_load = rq->misfit_task_load;
                busiest = rq;
            }

            break;
        }
    }

    return busiest;
}
```

### detach_tasks

```c
int detach_tasks(struct lb_env *env)
{
    struct list_head *tasks = &env->src_rq->cfs_tasks;
    unsigned long util, load;
    struct task_struct *p;
    int detached = 0;

    lockdep_assert_rq_held(env->src_rq);

    /* Source run queue has been emptied by another CPU, clear
     * LBF_ALL_PINNED flag as we will not test any task. */
    if (env->src_rq->nr_running <= 1) {
        env->flags &= ~LBF_ALL_PINNED;
        return 0;
    }

    if (env->imbalance <= 0)
        return 0;

    while (!list_empty(tasks)) {
        /* We don't want to steal all, otherwise we may be treated likewise,
         * which could at worst lead to a livelock crash. */
        if (env->idle != CPU_NOT_IDLE && env->src_rq->nr_running <= 1)
            break;

        env->loop++;
        /* We've more or less seen every task there is, call it quits
         * unless we haven't found any movable task yet. */
        if (env->loop > env->loop_max && !(env->flags & LBF_ALL_PINNED))
            break;

        /* take a breather every nr_migrate tasks */
        if (env->loop > env->loop_break) {
            env->loop_break += SCHED_NR_MIGRATE_BREAK;
            env->flags |= LBF_NEED_BREAK;
            break;
        }

        p = list_last_entry(tasks, struct task_struct, se.group_node);

        if (!can_migrate_task(p, env))
            goto next;

        switch (env->migration_type) {
        case migrate_load:
            /* Depending of the number of CPUs and tasks and the
             * cgroup hierarchy, task_h_load() can return a null
             * value. Make sure that env->imbalance decreases
             * otherwise detach_tasks() will stop only after
             * detaching up to loop_max tasks. */
            load = task_h_load(p) {
                update_cfs_rq_h_load(cfs_rq);
                return div64_ul(p->se.avg.load_avg * cfs_rq->h_load,
                        cfs_rq_load_avg(cfs_rq) + 1);
            }
            load = max_t(unsigned long, load, 1);

            if (sched_feat(LB_MIN)
                && load < 16 && !env->sd->nr_balance_failed) {

                goto next;
            }

            /* Make sure that we don't migrate too much load.
             * Nevertheless, let relax the constraint if
             * scheduler fails to find a good waiting task to
             * migrate. */
            if (shr_bound(load, env->sd->nr_balance_failed) > env->imbalance)
                goto next;

            env->imbalance -= load;
            break;

        case migrate_util:
            util = task_util_est(p);

            if (util > env->imbalance)
                goto next;

            env->imbalance -= util;
            break;

        case migrate_task:
            env->imbalance--;
            break;

        case migrate_misfit:
            /* This is not a misfit task */
            if (task_fits_cpu(p, env->src_cpu))
                goto next;

            env->imbalance = 0;
            break;
        }

        detach_task(p, env);
        list_add(&p->se.group_node, &env->tasks);

        detached++;

#ifdef CONFIG_PREEMPTION
        /* NEWIDLE balancing is a source of latency, so preemptible
         * kernels will stop after the first task is detached to minimize
         * the critical section. */
        if (env->idle == CPU_NEWLY_IDLE)
            break;
#endif

        /* We only want to steal up to the prescribed amount of
         * load/util/tasks. */
        if (env->imbalance <= 0)
            break;

        continue;
next:
        list_move(&p->se.group_node, tasks);
    }

    /* Right now, this is one of only two places we collect this stat
     * so we can safely collect detach_one_task() stats here rather
     * than inside detach_one_task(). */
    schedstat_add(env->sd->lb_gained[env->idle], detached);

    return detached;
}
```

#### can_migrate_task

```c
ret = can_migrate_task(p, env) {
    int tsk_cache_hot;

    lockdep_assert_rq_held(env->src_rq);

    /* We do not migrate tasks that are:
     * 1) throttled_lb_pair, or
     * 2) cannot be migrated to this CPU due to cpus_ptr, or
     * 3) running (obviously), or
     * 4) are cache-hot on their current CPU. */
    if (throttled_lb_pair(task_group(p), env->src_cpu, env->dst_cpu))
        return 0;

    /* Disregard pcpu kthreads; they are where they need to be. */
    if (kthread_is_per_cpu(p))
        return 0;

    if (!cpumask_test_cpu(env->dst_cpu, p->cpus_ptr)) {
        int cpu;

        schedstat_inc(p->stats.nr_failed_migrations_affine);

        env->flags |= LBF_SOME_PINNED;

        /* Remember if this task can be migrated to any other CPU in
         * our sched_group. We may want to revisit it if we couldn't
         * meet load balance goals by pulling other tasks on src_cpu.
         *
         * Avoid computing new_dst_cpu
         * - for NEWLY_IDLE
         * - if we have already computed one in current iteration
         * - if it's an active balance */
        if (env->idle == CPU_NEWLY_IDLE
            || env->flags & (LBF_DST_PINNED | LBF_ACTIVE_LB)) {

            return 0;
        }

        /* Prevent to re-select dst_cpu via env's CPUs: */
        for_each_cpu_and(cpu, env->dst_grpmask, env->cpus) {
            if (cpumask_test_cpu(cpu, p->cpus_ptr)) {
                env->flags |= LBF_DST_PINNED;
                env->new_dst_cpu = cpu;
                break;
            }
        }

        return 0;
    }

    /* Record that we found at least one task that could run on dst_cpu */
    env->flags &= ~LBF_ALL_PINNED;

    if (task_on_cpu(env->src_rq, p)) {
        schedstat_inc(p->stats.nr_failed_migrations_running);
        return 0;
    }

    /* Aggressive migration if:
     * 1) active balance
     * 2) destination numa is preferred
     * 3) task is cache cold, or
     * 4) too many balance attempts have failed. */
    if (env->flags & LBF_ACTIVE_LB)
        return 1;

    /* 1 degrades, 0 improves, -1 not affected locality */
    tsk_cache_hot = migrate_degrades_locality(p, env) {

    }
    if (tsk_cache_hot == -1) {
        tsk_cache_hot = task_hot(p, env) {
            s64 delta;

            lockdep_assert_rq_held(env->src_rq);

            if (p->sched_class != &fair_sched_class)
                return 0;

            if (unlikely(task_has_idle_policy(p)))
                return 0;

            /* SMT siblings share cache */
            if (env->sd->flags & SD_SHARE_CPUCAPACITY)
                return 0;

            /* Buddy candidates are cache hot: */
            if (sched_feat(CACHE_HOT_BUDDY) && env->dst_rq->nr_running &&
                (&p->se == cfs_rq_of(&p->se)->next))
                return 1;

            if (sysctl_sched_migration_cost == -1)
                return 1;

            /* Don't migrate task if the task's cookie does not match
             * with the destination CPU's core cookie. */
            if (!sched_core_cookie_match(cpu_rq(env->dst_cpu), p))
                return 1;

            if (sysctl_sched_migration_cost == 0)
                return 0;

            delta = rq_clock_task(env->src_rq) - p->se.exec_start;

            return delta < (s64)sysctl_sched_migration_cost;
        }
    }

    if (tsk_cache_hot <= 0
        || env->sd->nr_balance_failed > env->sd->cache_nice_tries) {

        if (tsk_cache_hot == 1) {
            schedstat_inc(env->sd->lb_hot_gained[env->idle]);
            schedstat_inc(p->stats.nr_forced_migrations);
        }
        return 1;
    }

    schedstat_inc(p->stats.nr_failed_migrations_hot);
    return 0;
}
```

# wake_up

<img src='../images/kernel/proc-wake-up.png' style='max-height:850px'/>

```c
#define wake_up(x)                        __wake_up(x, TASK_NORMAL, 1, NULL)
#define wake_up_nr(x, nr)                 __wake_up(x, TASK_NORMAL, nr, NULL)
#define wake_up_all(x)                    __wake_up(x, TASK_NORMAL, 0, NULL)
#define wake_up_locked(x)                 __wake_up_locked((x), TASK_NORMAL, 1)
#define wake_up_all_locked(x)             __wake_up_locked((x), TASK_NORMAL, 0)

#define wake_up_interruptible(x)          __wake_up(x, TASK_INTERRUPTIBLE, 1, NULL)
#define wake_up_interruptible_nr(x, nr)   __wake_up(x, TASK_INTERRUPTIBLE, nr, NULL)
#define wake_up_interruptible_all(x)      __wake_up(x, TASK_INTERRUPTIBLE, 0, NULL)
#define wake_up_interruptible_sync(x)     __wake_up_sync((x), TASK_INTERRUPTIBLE, 1)

void __wake_up(
  struct wait_queue_head *wq_head, unsigned int mode,
  int nr_exclusive, void *key)
{
    __wake_up_common_lock(wq_head, mode, nr_exclusive, 0, key) {
        unsigned long flags;
        wait_queue_entry_t bookmark;

        bookmark.flags = 0;
        bookmark.private = NULL;
        bookmark.func = NULL;
        INIT_LIST_HEAD(&bookmark.entry);

        spin_lock_irqsave(&wq_head->lock, flags);
        nr_exclusive = __wake_up_common(wq_head, mode, nr_exclusive, wake_flags, key, &bookmark);
        spin_unlock_irqrestore(&wq_head->lock, flags);

        while (bookmark.flags & WQ_FLAG_BOOKMARK) {
            spin_lock_irqsave(&wq_head->lock, flags);
            nr_exclusive = __wake_up_common(wq_head, mode, nr_exclusive, wake_flags, key, &bookmark);
            spin_unlock_irqrestore(&wq_head->lock, flags);
        }
    }
}

static int __wake_up_common(
  struct wait_queue_head *wq_head, unsigned int mode,
  int nr_exclusive, int wake_flags, void *key,
  wait_queue_entry_t *bookmark)
{
    wait_queue_entry_t *curr, *next;
    int cnt = 0;

    lockdep_assert_held(&wq_head->lock);

    if (bookmark && (bookmark->flags & WQ_FLAG_BOOKMARK)) {
        curr = list_next_entry(bookmark, entry);

        list_del(&bookmark->entry);
        bookmark->flags = 0;
    } else
        curr = list_first_entry(&wq_head->head, wait_queue_entry_t, entry);

    if (&curr->entry == &wq_head->head)
        return nr_exclusive;

    list_for_each_entry_safe_from(curr, next, &wq_head->head, entry) {
        unsigned flags = curr->flags;
        int ret;

        if (flags & WQ_FLAG_BOOKMARK)
            continue;

        /* ep_poll_callback, default_wake_func, woken_wake_function */
        ret = curr->func(curr, mode, wake_flags, key);
        if (ret < 0)
            break;
        /* WQ_FLAG_EXCLUSIVE : fix Thundering Herd problem */
        if (ret && (flags & WQ_FLAG_EXCLUSIVE) && !--nr_exclusive)
            break;

        if (bookmark && (++cnt > WAITQUEUE_WALK_BREAK_CNT)
            && (&next->entry != &wq_head->head)) {

            bookmark->flags = WQ_FLAG_BOOKMARK;
            list_add_tail(&bookmark->entry, &next->entry);
            break;
        }
    }

    return nr_exclusive;
}
```

# wait_woken

```c
long inet_wait_for_connect(struct sock *sk, long timeo, int writebias)
{
    DEFINE_WAIT_FUNC(wait, woken_wake_function);

    add_wait_queue(sk_sleep(sk), &wait);
    sk->sk_write_pending += writebias;

    while ((1 << sk->sk_state) & (TCPF_SYN_SENT | TCPF_SYN_RECV)) {
        timeo = wait_woken(&wait, TASK_INTERRUPTIBLE, timeo) {
            /* The below executes an smp_mb(), which matches with the full barrier
            * executed by the try_to_wake_up() in woken_wake_function() such that
            * either we see the store to wq_entry->flags in woken_wake_function()
            * or woken_wake_function() sees our store to current->state. */
            set_current_state(mode); /* A */
            if (!(wq_entry->flags & WQ_FLAG_WOKEN) && !is_kthread_should_stop()) {
                timeout = schedule_timeout(timeout) {

                }
            }
            __set_current_state(TASK_RUNNING);

            /* The below executes an smp_mb(), which matches with the smp_mb() (C)
            * in woken_wake_function() such that either we see the wait condition
            * being true or the store to wq_entry->flags in woken_wake_function()
            * follows ours in the coherence order. */
            smp_store_mb(wq_entry->flags, wq_entry->flags & ~WQ_FLAG_WOKEN); /* B */

            return timeout;
        }
        if (signal_pending(current) || !timeo)
            break;
    }
    remove_wait_queue(sk_sleep(sk), &wait);
    sk->sk_write_pending -= writebias;
    return timeo;
}


long __sched schedule_timeout(signed long timeout)
{
    struct process_timer timer;
    unsigned long expire;

    switch (timeout)
    {
    case MAX_SCHEDULE_TIMEOUT:
        schedule();
        goto out;
    default:
        if (timeout < 0) {
            printk(KERN_ERR "schedule_timeout: wrong timeout "
                "value %lx\n", timeout);
            dump_stack();
            current->state = TASK_RUNNING;
            goto out;
        }
    }

    expire = timeout + jiffies;

    timer.task = current;
    timer_setup_on_stack(&timer.timer, process_timeout, 0);
    __mod_timer(&timer.timer, expire, 0);
    schedule();
    del_singleshot_timer_sync(&timer.timer);

    /* Remove the timer from the object tracker */
    destroy_timer_on_stack(&timer.timer);

    timeout = expire - jiffies;

out:
    return timeout < 0 ? 0 : timeout;
}

void process_timeout(struct timer_list *t)
{
    struct process_timer *timeout = from_timer(timeout, t, timer);

    wake_up_process(timeout->task) {
        return try_to_wake_up(p, TASK_NORMAL, 0) {

        }
    }
}

/* wait_queue_entry::flags */
#define WQ_FLAG_EXCLUSIVE  0x01
#define WQ_FLAG_WOKEN      0x02
#define WQ_FLAG_BOOKMARK   0x04

struct wait_queue_entry {
    unsigned int      flags;
    void              *private; /* struct_task */
    wait_queue_func_t func;
    struct list_head  entry;
};
```

# try_to_wake_up
```c
/* try_to_wake_up -> ttwu_queue -> ttwu_do_activate -> ttwu_do_wakeup
 * -> wakeup_preempt -> resched_curr */

try_to_wake_up() {
    if (p == current) {
        ttwu_do_wakeup(p) {
            WRITE_ONCE(p->__state, TASK_RUNNING);
        }
        goto out;
    }

    cpu = select_task_rq(p, p->wake_cpu, wake_flags | WF_TTWU) {
        if (p->nr_cpus_allowed > 1 && !is_migration_disabled(p))
            cpu = p->sched_class->select_task_rq(p, cpu, wake_flags);
        else
            cpu = cpumask_any(p->cpus_ptr);
    }
    set_task_cpu(p, cpu);

    ttwu_queue(p, cpu, wake_flags) {
        ttwu_do_activate(rq, p, wake_flags, &rf) {
            if (p->in_iowait) {
                delayacct_blkio_end(p);
                atomic_dec(&task_rq(p)->nr_iowait);
            }

            activate_task(rq, p, en_flags) {
                if (task_on_rq_migrating(p))
                    flags |= ENQUEUE_MIGRATED;
                if (flags & ENQUEUE_MIGRATED)
                    sched_mm_cid_migrate_to(rq, p);

                enqueue_task(rq, p, flags) {
                    uclamp_rq_inc(rq, p);
                    p->sched_class->enqueue_task(rq, p, flags);
                }

                p->on_rq = TASK_ON_RQ_QUEUED;
            }

            wakeup_preempt(rq, p, wake_flags) {
                if (p->sched_class == rq->curr->sched_class) {
                    rq->curr->sched_class->wakeup_preempt(rq, p, flags);
                } else if (sched_class_above(p->sched_class, rq->curr->sched_class)) {
                    resched_curr(rq) {
                        if (test_tsk_need_resched(curr))
                            return;

                        if (cpu == smp_processor_id()) {
                            set_tsk_need_resched(curr);
                            set_preempt_need_resched() {
                                current_thread_info()->preempt.need_resched = 0;
                            }
                            return;
                        }

                        if (set_nr_and_not_polling(curr)) {
                            smp_send_reschedule(cpu) {
                                arch_smp_send_reschedule(cpu);
                            }
                        }
                    }
                }

                /* A queue event has occurred, and we're going to schedule.  In
                * this case, we can save a useless back to back clock update. */
                if (task_on_rq_queued(rq->curr) && test_tsk_need_resched(rq->curr))
                    rq_clock_skip_update(rq);
            }

            ttwu_do_wakeup(p) {
                WRITE_ONCE(p->__state, TASK_RUNNING);
            }

            p->sched_class->task_woken(rq, p) {
                void task_woken_rt(struct rq *rq, struct task_struct *p) {
                    bool need_to_push = !task_on_cpu(rq, p)
                        && !test_tsk_need_resched(rq->curr)
                        && p->nr_cpus_allowed > 1 &&
                        && (dl_task(rq->curr) || rt_task(rq->curr))
                        && (rq->curr->nr_cpus_allowed < 2 || rq->curr->prio <= p->prio);

                    if (need_to_push)
                        push_rt_tasks(rq);
                }
            }

            if (rq->idle_stamp) {
                u64 delta = rq_clock(rq) - rq->idle_stamp;
                u64 max = 2*rq->max_idle_balance_cost;

                update_avg(&rq->avg_idle, delta);

                if (rq->avg_idle > max)
                    rq->avg_idle = max;

                rq->idle_stamp = 0;
            }
        }
    }
    preempt_enable();
}
```

# fork

* [Misc on Linux fork, switch_to, and scheduling](http://lastweek.io/notes/linux/fork/)

<img src="../images/kernel/proc-fork-frame.png" height="850"/>

<img src="../images/kernel/fork.png" style="max-height:1200px"/>

<img src="../images/kernel/proc-fork-pthread-create.png" style="max-height:850px"/>

```c
SYSCALL_DEFINE0(fork)
{
    return _do_fork(SIGCHLD, 0, 0, NULL, NULL, 0);
}

do_fork(clone_flags, stack_start, stack_size, parent_tidptr, child_tidptr, tls);
kernel_clone(struct kernel_clone_args *args) {
    copy_process() {
        task_struct* tsk = dup_task_struct(current, node) {
            tsk = alloc_task_struct_node(node);

            alloc_thread_stack_node(tsk, node) {
                stack = __vmalloc_node_range(
                    THREAD_SIZE, THREAD_ALIGN, VMALLOC_START, VMALLOC_END
                );
                vm = find_vm_area(stack);
                tsk->stack_vm_area = vm;
                tsk->stack = stack; /* kernel stack */
            }
            arch_dup_task_struct(tsk, orig) {
                *tsk = *orig;
            }
            setup_thread_stack(tsk, orig);
            clear_user_return_notifier(tsk);
            clear_tsk_need_resched(tsk);
            set_task_stack_end_magic(tsk);
            clear_syscall_work_syscall_user_dispatch(tsk);
            return tsk;
        }

        cgroup_fork();
        sched_fork() {
            __sched_fork(clone_flags, p);
            p->__state = TASK_NEW;
            /* Make sure we do not leak PI boosting priority to the child. */
            p->prio = current->normal_prio;
            p->sched_class = &fair_sched_class, &rt_sched_class;
            p->sched_class->task_fork(p) {
                task_fork_fair() {
                    place_entity(cfs_rq, se, ENQUEUE_INITIAL) {
                        se->slice = sysctl_sched_base_slice;
                        vslice = calc_delta_fair(se->slice, se);
                        se->vruntime = vruntime - lag;
                        if (sched_feat(PLACE_DEADLINE_INITIAL) && (flags & ENQUEUE_INITIAL)) {
                            vslice /= 2;
                        }
                        /* EEVDF: vd_i = ve_i + r_i/w_i */
                        se->deadline = se->vruntime + vslice;
                    }
                }
            }
            init_entity_runnable_average(&p->se) {
                struct sched_avg *sa = &se->avg;
                memset(sa, 0, sizeof(*sa));
                if (entity_is_task(se)) {
                    sa->load_avg = scale_load_down(se->load.weight);
                }
            }
        }
        copy_files();
        copy_fs();
        copy_sighand();
        copy_signal();
        copy_mm();
        copy_namespaces();
        copy_io();
        copy_thread(clone_flags, stack_start/*sp*/, stack_size/*arg*/, p, tls) {
            unsigned long clone_flags = args->flags;
            unsigned long stack_start = args->stack;
            unsigned long tls = args->tls;
            struct pt_regs *childregs = task_pt_regs(p);

            memset(&p->thread.cpu_context, 0, sizeof(struct cpu_context));

            if (likely(!args->fn)) {
                *childregs = *current_pt_regs();
                childregs->regs[0] = 0;

                *task_user_tls(p) = read_sysreg(tpidr_el0);
                if (system_supports_tpidr2())
                    p->thread.tpidr2_el0 = read_sysreg_s(SYS_TPIDR2_EL0);

                if (stack_start) {
                    if (is_compat_thread(task_thread_info(p)))
                        childregs->compat_sp = stack_start;
                    else
                        childregs->sp = stack_start;
                }

                if (clone_flags & CLONE_SETTLS) {
                    p->thread.uw.tp_value = tls;
                    p->thread.tpidr2_el0 = 0;
                }
            } else {
                /* A kthread has no context to ERET to, so ensure any buggy
                 * ERET is treated as an illegal exception return.
                 *
                 * When a user task is created from a kthread, childregs will
                 * be initialized by start_thread() or start_compat_thread(). */
                memset(childregs, 0, sizeof(struct pt_regs));
                childregs->pstate = PSR_MODE_EL1h | PSR_IL_BIT;

                p->thread.cpu_context.x19 = (unsigned long)args->fn;
                p->thread.cpu_context.x20 = (unsigned long)args->fn_arg;
            }
            p->thread.cpu_context.pc = (unsigned long)ret_from_fork;
            p->thread.cpu_context.sp = (unsigned long)childregs;
            /* For the benefit of the unwinder, set up childregs->stackframe
             * as the final frame for the new task. */
            p->thread.cpu_context.fp = (unsigned long)childregs->stackframe;

            ptrace_hw_copy_thread(p);

            return 0;
        }

        pid = alloc_pid(p->nsproxy->pid_ns_for_children);
        p->pid = pid_nr(pid) {
            pid_t nr = 0;
            if (pid)
                nr = pid->numbers[0].nr; /* numbers[0] global id */
            return nr;
        }
        init_task_pid(p, PIDTYPE_PID, pid);

        if (clone_flags & CLONE_THREAD) {
            p->group_leader = current->group_leader;
            p->tgid = current->tgid;
        } else {
            p->group_leader = p;
            p->tgid = p->pid;
        }

        /* Don't start children in a dying pid namespace */
        if (!(ns_of_pid(pid)->pid_allocated & PIDNS_ADDING)) {
            retval = -ENOMEM;
            goto bad_fork_cancel_cgroup;
        }

        init_task_pid_links(p);
        if (likely(p->pid)) {
            init_task_pid(p, PIDTYPE_PID, pid);
            if (thread_group_leader(p)) {
                init_task_pid(p, PIDTYPE_TGID, pid);
                init_task_pid(p, PIDTYPE_PGID, task_pgrp(current));
                init_task_pid(p, PIDTYPE_SID, task_session(current));

                if (is_child_reaper(pid)) {
                    ns_of_pid(pid)->child_reaper = p;
                    p->signal->flags |= SIGNAL_UNKILLABLE;
                }
                p->signal->shared_pending.signal = delayed.signal;
                p->signal->tty = tty_kref_get(current->signal->tty);
                p->signal->has_child_subreaper = p->real_parent->signal->has_child_subreaper
                    || p->real_parent->signal->is_child_subreaper;

                list_add_tail(&p->sibling, &p->real_parent->children);
                list_add_tail_rcu(&p->tasks, &init_task.tasks);
                attach_pid(p, PIDTYPE_TGID);
                attach_pid(p, PIDTYPE_PGID);
                attach_pid(p, PIDTYPE_SID);
                __this_cpu_inc(process_counts);
            } else {
                current->signal->nr_threads++;
                current->signal->quick_threads++;
                atomic_inc(&current->signal->live);
                refcount_inc(&current->signal->sigcnt);
                task_join_group_stop(p);
                list_add_tail_rcu(&p->thread_node, &p->signal->thread_head);
            }
            attach_pid(p, PIDTYPE_PID);
            nr_threads++;
        }

        futex_init_task(p);
    }

    wake_up_new_task(p) {
        post_init_entity_util_avg(p) {
            struct sched_entity *se = &p->se;
            struct cfs_rq *cfs_rq = cfs_rq_of(se);
            struct sched_avg *sa = &se->avg;
            long cpu_scale = arch_scale_cpu_capacity(cpu_of(rq_of(cfs_rq)));
            long cap = (long)(cpu_scale - cfs_rq->avg.util_avg) / 2;

            if (p->sched_class != &fair_sched_class) {
                se->avg.last_update_time = cfs_rq_clock_pelt(cfs_rq);
                return;
            }

            if (cap > 0) {
                if (cfs_rq->avg.util_avg != 0) {
                    sa->util_avg  = cfs_rq->avg.util_avg * se->load.weight;
                    sa->util_avg /= (cfs_rq->avg.load_avg + 1);

                    if (sa->util_avg > cap)
                        sa->util_avg = cap;
                } else {
                    sa->util_avg = cap;
                }
            }

            sa->runnable_avg = sa->util_avg;
        }

        activate_task();
        wakeup_preempt();
        p->sched_class->task_woken(rq, p);
    }
}

SYM_CODE_START(ret_from_fork)
    bl    schedule_tail
    cbz    x19, 1f                // not a kernel thread
    mov    x0, x20
    blr    x19
1:    get_current_task tsk
    mov    x0, sp
    bl    asm_exit_to_user_mode
    b    ret_to_user
SYM_CODE_END(ret_from_fork)
```

# exec

![](../images/kernel/proc-exec.svg)

* [ELF Format Cheatsheet](https://gist.github.com/x0nu11byt3/bcb35c3de461e5fb66173071a2379779)

![](../images/kernel/proc-elf.png)

```c
struct linux_binfmt {
    struct list_head lh;
    struct module *module;
    int (*load_binary)(struct linux_binprm *);
    int (*load_shlib)(struct file *);
    int (*core_dump)(struct coredump_params *cprm);
    unsigned long min_coredump;     /* minimal dump size */
};

static struct linux_binfmt elf_format = {
    .module         = THIS_MODULE,
    .load_binary    = load_elf_binary,
    .load_shlib     = load_elf_library,
    .core_dump      = elf_core_dump,
    .min_coredump   = ELF_EXEC_PAGESIZE,
};

struct linux_binprm {
    struct vm_area_struct   *vma;
    unsigned long           vma_pages;

    struct mm_struct        *mm;
    unsigned long           p; /* sp pointer */
    unsigned long           argmin; /* rlimit marker for copy_strings() */
    struct file *executable; /* Executable to pass to the interpreter */
    struct file *interpreter;
    struct file *file;
    struct cred *cred;  /* new credentials */
    int unsafe;         /* how unsafe this exec is (mask of LSM_UNSAFE_*) */
    unsigned int per_clear; /* bits to clear in current->personality */
    int argc, envc;
    const char *filename;   /* Name of binary as seen by procps */
    const char *interp;     /* Name of the binary really executed. Most
                of the time same as filename, but could be
                different for binfmt_{misc,script} */
    const char *fdpath;     /* generated filename for execveat */
    unsigned interp_flags;
    int execfd;         /* File descriptor of the executable */
    unsigned long loader;   /* loader filename */
    unsigned long exec;     /* filename of program */

    struct rlimit rlim_stack; /* Saved RLIMIT_STACK used during exec. */

    char buf[BINPRM_BUF_SIZE];
}
```

```c
/* do_execve -> do_execveat_common -> exec_binprm -> search_binary_handler */
SYSCALL_DEFINE3(execve,
  const char __user *, filename,
  const char __user *const __user *, argv,
  const char __user *const __user *, envp)
{
    return do_execve(getname(filename), argv, envp) {
        struct user_arg_ptr argv = { .ptr.native = __argv };
        struct user_arg_ptr envp = { .ptr.native = __envp };
        return do_execveat_common(AT_FDCWD, filename, argv, envp, 0) {
            struct linux_binprm *bprm;
            int retval;

            /* We're below the limit (still or again), so we don't want to make
            * further execve() calls fail. */
            current->flags &= ~PF_NPROC_EXCEEDED;

            bprm = alloc_bprm(fd, filename) {
                *bprm = kzalloc();
                bprm->filename = filename->name;
                bprm->interp = bprm->filename;
                bprm_mm_init(bprm) {
                    bprm->mm = mm = mm_alloc();
                    bprm->rlim_stack = current->signal->rlim[RLIMIT_STACK];

                    __bprm_mm_init(bprm) {
                        bprm->vma = vma = vm_area_alloc(mm);
                        vma_set_anonymous(vma);

                        vma->vm_end = STACK_TOP_MAX;
                        vma->vm_start = vma->vm_end - PAGE_SIZE;
                        vm_flags_init(vma, VM_SOFTDIRTY | VM_STACK_FLAGS | VM_STACK_INCOMPLETE_SETUP);
                        vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);

                        err = insert_vm_struct(mm, vma);
                        mm->stack_vm = mm->total_vm = 1;
                        bprm->p = vma->vm_end - sizeof(void *);
                    }
                }
            }

            retval = bprm_stack_limits(bprm) {
                limit = _STK_LIM / 4 * 3;
                limit = min(limit, bprm->rlim_stack.rlim_cur / 4);
                ptr_size = (max(bprm->argc, 1) + bprm->envc) * sizeof(void *);
                limit -= ptr_size;
                bprm->argmin = bprm->p - limit;
            }
            copy_string_kernel(bprm->filename, bprm)

            bprm->exec = bprm->p;

            retval = copy_strings(bprm->envc, envp, bprm);
            retval = copy_strings(bprm->argc, argv, bprm);

            retval = bprm_execve(bprm, fd, filename, flags) {
                retval = prepare_bprm_creds(bprm);

                check_unsafe_exec(bprm);
                current->in_execve = 1;
                sched_mm_cid_before_execve(current);

                file = do_open_execat(fd, filename, flags) {
                    do_filp_open(fd, name, &open_exec_flags);

                    deny_write_access(file);
                }

                sched_exec() {
                    struct task_struct *p = current;
                    struct migration_arg arg;
                    int dest_cpu;

                    scoped_guard (raw_spinlock_irqsave, &p->pi_lock) {
                        dest_cpu = p->sched_class->select_task_rq(p, task_cpu(p), WF_EXEC);
                        if (dest_cpu == smp_processor_id())
                            return;

                        if (unlikely(!cpu_active(dest_cpu)))
                            return;

                        arg = (struct migration_arg){ p, dest_cpu };
                    }
                    stop_one_cpu(task_cpu(p), migration_cpu_stop, &arg);
                }

                bprm->file = file;

                retval = exec_binprm(bprm) {
                    /* Need to fetch pid before load_binary changes it */
                    old_pid = current->pid;
                    rcu_read_lock();
                    old_vpid = task_pid_nr_ns(current, task_active_pid_ns(current->parent));
                    rcu_read_unlock();

                    /* This allows 4 levels of binfmt rewrites before failing hard. */
                    for (depth = 0;; depth++) {
                        struct file *exec;
                        if (depth > 5)
                        return -ELOOP;

                        ret = search_binary_handler(bprm) {
                            bool need_retry = IS_ENABLED(CONFIG_MODULES);
                            struct linux_binfmt *fmt;
                            int retval;

                            retval = prepare_binprm(bprm) {
                                memset(bprm->buf, 0, BINPRM_BUF_SIZE);
                                return kernel_read(bprm->file, bprm->buf, BINPRM_BUF_SIZE, &pos);
                            }

                            retval = -ENOENT;
                        retry:
                            read_lock(&binfmt_lock);
                            list_for_each_entry(fmt, &formats, lh) {
                                if (!try_module_get(fmt->module))
                                    continue;
                                read_unlock(&binfmt_lock);

                                retval = fmt->load_binary(bprm); /* load_elf_binary */

                                read_lock(&binfmt_lock);
                                put_binfmt(fmt);
                                if (bprm->point_of_no_return || (retval != -ENOEXEC)) {
                                    read_unlock(&binfmt_lock);
                                    return retval;
                                }
                            }
                            read_unlock(&binfmt_lock);

                            return retval;
                        }

                        if (ret < 0)
                            return ret;
                        if (!bprm->interpreter)
                            break;

                        exec = bprm->file;
                        bprm->file = bprm->interpreter;
                        bprm->interpreter = NULL;

                        allow_write_access(exec);
                    }

                    return 0;
                }

                sched_mm_cid_after_execve(current);
                /* execve succeeded */
                current->fs->in_exec = 0;
                current->in_execve = 0;
                rseq_execve(current);
                user_events_execve(current);
                acct_update_integrals(current);
                task_numa_free(current, false);
                return retval;
            }

            return retval;
        }
    }
}
```

## load_elf_binary

```c
int load_elf_binary(struct linux_binprm *bprm)
{
    struct file *interpreter = NULL; /* to shut gcc up */
    unsigned long load_bias = 0, phdr_addr = 0;
    int first_pt_load = 1;
    unsigned long error;
    struct elf_phdr *elf_ppnt, *elf_phdata, *interp_elf_phdata = NULL;
    struct elf_phdr *elf_property_phdata = NULL;
    unsigned long elf_brk;
    int retval, i;
    unsigned long elf_entry;
    unsigned long e_entry;
    unsigned long interp_load_addr = 0;
    unsigned long start_code, end_code, start_data, end_data;
    unsigned long reloc_func_desc __maybe_unused = 0;
    int executable_stack = EXSTACK_DEFAULT;
    struct elfhdr *elf_ex = (struct elfhdr *)bprm->buf;
    struct elfhdr *interp_elf_ex = NULL;
    struct arch_elf_state arch_state = INIT_ARCH_ELF_STATE;
    struct mm_struct *mm;
    struct pt_regs *regs;

    retval = -ENOEXEC;
    /* First of all, some simple consistency checks */
    if (memcmp(elf_ex->e_ident, ELFMAG, SELFMAG) != 0)
        goto out;

    if (elf_ex->e_type != ET_EXEC && elf_ex->e_type != ET_DYN)
        goto out;
    if (!elf_check_arch(elf_ex))
        goto out;
    if (elf_check_fdpic(elf_ex))
        goto out;
    if (!bprm->file->f_op->mmap)
        goto out;

    elf_phdata = load_elf_phdrs(elf_ex, bprm->file) {
        size = sizeof(struct elf_phdr) * elf_ex->e_phnum;
        elf_phdata = kmalloc(size, GFP_KERNEL);
        elf_read(elf_file, elf_phdata, size, elf_ex->e_phoff);
        return elf_data;
    }
    if (!elf_phdata)
        goto out;

    /* This is the program interpreter used for shared libraries -
     * for now assume that this is an a.out format binary. */
    elf_ppnt = elf_phdata;
    for (i = 0; i < elf_ex->e_phnum; i++, elf_ppnt++) {
        char *elf_interpreter;

        if (elf_ppnt->p_type == PT_GNU_PROPERTY) {
            elf_property_phdata = elf_ppnt;
            continue;
        }

        if (elf_ppnt->p_type != PT_INTERP)
            continue;

        elf_interpreter = kmalloc(elf_ppnt->p_filesz, GFP_KERNEL);

        retval = elf_read(bprm->file, elf_interpreter, elf_ppnt->p_filesz,
                elf_ppnt->p_offset);

        interpreter = open_exec(elf_interpreter) {
            do_open_execat() {
                do_filp_open();
                deny_write_access();
            }
        }

        interp_elf_ex = kmalloc(sizeof(*interp_elf_ex), GFP_KERNEL);
        if (!interp_elf_ex) {
            retval = -ENOMEM;
            goto out_free_file;
        }

        /* Get the exec headers */
        retval = elf_read(interpreter, interp_elf_ex, sizeof(*interp_elf_ex), 0);
        if (retval < 0)
            goto out_free_dentry;

        break;

out_free_interp:
        kfree(elf_interpreter);
        goto out_free_ph;
    }

    /* Some simple consistency checks for the interpreter */
    if (interpreter) {
        retval = -ELIBBAD;
        /* Not an ELF interpreter */
        if (memcmp(interp_elf_ex->e_ident, ELFMAG, SELFMAG) != 0)
            goto out_free_dentry;
        /* Verify the interpreter has a valid arch */
        if (!elf_check_arch(interp_elf_ex) ||
            elf_check_fdpic(interp_elf_ex))
            goto out_free_dentry;

        /* Load the interpreter program headers */
        interp_elf_phdata = load_elf_phdrs(interp_elf_ex, interpreter);
    }

    retval = parse_elf_properties(interpreter ?: bprm->file, elf_property_phdata, &arch_state);

    /* Flush all traces of the currently running executable */
    retval = begin_new_exec(bprm) {
        /* Make this the only thread in the thread group. */
        retval = de_thread(me);

        /* Ensure the files table is not shared. */
        retval = unshare_files();

        set_mm_exe_file(bprm->mm, bprm->file) {
            deny_write_access(new_exe_file);
            rcu_assign_pointer(mm->exe_file, new_exe_file);
            allow_write_access(old_exe_file);
        }

        /* Maps the mm_struct mm into the current task struct. */
        retval = exec_mmap(bprm->mm) {
            activate_mm(active_mm, mm) {
                switch_mm(prev_mm, next_mm, current);
            }
        }

        posix_cpu_timers_exit(me);
        exit_itimers(me);
        flush_itimer_signals();

        do_close_on_exec(me->files) {
            for (i = 0; ; i++) {
                filp_close(file, files);
            }
        }

        setup_new_exec(bprm) {
            arch_pick_mmap_layout() {
                mm->mmap_base = TASK_UNMAPPED_BASE + random_factor;
                mm->get_unmapped_area = arch_get_unmapped_area;
            }
        }
    }

    /* Finalizes the stack vm_area_struct */
    setup_arg_pages(bprm, randomize_stack_top(STACK_TOP), executable_stack) {
        stack_top = arch_align_stack(stack_top);
        stack_top = PAGE_ALIGN(stack_top);
        stack_shift = vma->vm_end - stack_top;

        bprm->p -= stack_shift;
        mm->arg_start = bprm->p;

        if (bprm->loader)
            bprm->loader -= stack_shift;
        bprm->exec -= stack_shift;

        tlb_gather_mmu(&tlb, mm);
        ret = mprotect_fixup(&vmi, &tlb, vma, &prev, vma->vm_start, vma->vm_end,
                vm_flags);
        tlb_finish_mmu(&tlb);

        /* shift tmp stack to its final location */
        shift_arg_pages(vma, stack_shift);

        stack_expand = 131072UL; /* randomly 32*4k (or 2*64k) pages */
        stack_size = vma->vm_end - vma->vm_start;
        rlim_stack = bprm->rlim_stack.rlim_cur & PAGE_MASK;

        stack_expand = min(rlim_stack, stack_size + stack_expand);
        stack_base = vma->vm_end - stack_expand;

        current->mm->start_stack = bprm->p;
        ret = expand_stack_locked(vma, stack_base) {
            expand_downwards(vma, address) {

            }
        }
    }

    elf_brk = 0;

    start_code = ~0UL;
    end_code = 0;
    start_data = 0;
    end_data = 0;

    /* Now we do a little grungy work by mmapping the ELF image into
     * the correct location in memory. */
    for(i = 0, elf_ppnt = elf_phdata; i < elf_ex->e_phnum; i++, elf_ppnt++) {
        int elf_prot, elf_flags;
        unsigned long k, vaddr;
        unsigned long total_size = 0;
        unsigned long alignment;

        if (elf_ppnt->p_type != PT_LOAD)
            continue;

        elf_prot = make_prot(elf_ppnt->p_flags, &arch_state, !!interpreter, false);

        elf_flags = MAP_PRIVATE;

        vaddr = elf_ppnt->p_vaddr;

        if (!first_pt_load) {
            elf_flags |= MAP_FIXED;
        } else if (elf_ex->e_type == ET_EXEC) {
            elf_flags |= MAP_FIXED_NOREPLACE;
        } else if (elf_ex->e_type == ET_DYN) {
            if (interpreter) {
                load_bias = ELF_ET_DYN_BASE; /* (2 * TASK_SIZE_64 / 3) */
                if (current->flags & PF_RANDOMIZE) {
                    load_bias += arch_mmap_rnd();
                }
                alignment = maximum_alignment(elf_phdata, elf_ex->e_phnum);
                if (alignment) {
                    load_bias &= ~(alignment - 1);
                }
                elf_flags |= MAP_FIXED_NOREPLACE;
            } else {
                load_bias = 0;
            }

            load_bias = ELF_PAGESTART(load_bias - vaddr);

            total_size = total_mapping_size(elf_phdata, elf_ex->e_phnum);
            if (!total_size) {
                retval = -EINVAL;
                goto out_free_dentry;
            }
        }

        /* Map "eppnt->p_filesz" bytes from "filep" offset "eppnt->p_offset"
         * into memory at "addr". Memory from "p_filesz" through "p_memsz"
         * rounded up to the next page is zeroed. */
        error = elf_load(bprm->file, load_bias + vaddr, elf_ppnt, elf_prot, elf_flags, total_size) {
            if (eppnt->p_filesz) {
                map_addr = elf_map(filep, addr, eppnt, prot, type, total_size) {
                    do_mmap();
                }
                if (BAD_ADDR(map_addr))
                    return map_addr;
                if (eppnt->p_memsz > eppnt->p_filesz) {
                    zero_start = map_addr + ELF_PAGEOFFSET(eppnt->p_vaddr) + eppnt->p_filesz;
                    zero_end = map_addr + ELF_PAGEOFFSET(eppnt->p_vaddr) + eppnt->p_memsz;

                    if (padzero(zero_start) && (prot & PROT_WRITE))
                        return -EFAULT;
                }
            } else {
                map_addr = zero_start = ELF_PAGESTART(addr);
                zero_end = zero_start + ELF_PAGEOFFSET(eppnt->p_vaddr) + eppnt->p_memsz;
            }
            if (eppnt->p_memsz > eppnt->p_filesz) {
                int error;

                zero_start = ELF_PAGEALIGN(zero_start);
                zero_end = ELF_PAGEALIGN(zero_end);

                error = vm_brk_flags(zero_start, zero_end - zero_start, prot & PROT_EXEC ? VM_EXEC : 0);
                if (error)
                    map_addr = error;
            }
            return map_addr;
        }

        if (first_pt_load) {
            first_pt_load = 0;
            if (elf_ex->e_type == ET_DYN) {
                load_bias += error - ELF_PAGESTART(load_bias + vaddr);
                reloc_func_desc = load_bias;
            }
        }

        /* Figure out which segment in the file contains the Program
         * Header table, and map to the associated memory address. */
        if (elf_ppnt->p_offset <= elf_ex->e_phoff &&
            elf_ex->e_phoff < elf_ppnt->p_offset + elf_ppnt->p_filesz) {
            phdr_addr = elf_ex->e_phoff - elf_ppnt->p_offset + elf_ppnt->p_vaddr;
        }

        k = elf_ppnt->p_vaddr;
        if ((elf_ppnt->p_flags & PF_X) && k < start_code)
            start_code = k;
        if (start_data < k)
            start_data = k;

        if (BAD_ADDR(k) || elf_ppnt->p_filesz > elf_ppnt->p_memsz ||
            elf_ppnt->p_memsz > TASK_SIZE ||
            TASK_SIZE - elf_ppnt->p_memsz < k) {
            /* set_brk can never work. Avoid overflows. */
            retval = -EINVAL;
            goto out_free_dentry;
        }

        k = elf_ppnt->p_vaddr + elf_ppnt->p_filesz;

        if ((elf_ppnt->p_flags & PF_X) && end_code < k)
            end_code = k;
        if (end_data < k)
            end_data = k;
        k = elf_ppnt->p_vaddr + elf_ppnt->p_memsz;
        if (elf_brk < k)
            elf_brk = k;
    }

    e_entry = elf_ex->e_entry + load_bias;
    phdr_addr += load_bias;
    elf_brk += load_bias;
    start_code += load_bias;
    end_code += load_bias;
    start_data += load_bias;
    end_data += load_bias;

    current->mm->start_brk = current->mm->brk = ELF_PAGEALIGN(elf_brk);

    if (interpreter) {
        elf_entry = load_elf_interp(interp_elf_ex,
            interpreter, load_bias, interp_elf_phdata, &arch_state
        );
        if (!IS_ERR_VALUE(elf_entry)) {
            interp_load_addr = elf_entry;
            elf_entry += interp_elf_ex->e_entry;
        }
        reloc_func_desc = interp_load_addr;

        allow_write_access(interpreter);
    } else {
        elf_entry = e_entry;
    }

    kfree(elf_phdata);

    set_binfmt(&elf_format) {
        mm->binfmt = new;
    }

    retval = create_elf_tables(bprm, elf_ex, interp_load_addr, e_entry, phdr_addr) {
        items = (argc + 1) + (envc + 1) + 1;
        bprm->p = STACK_ROUND(sp, items);
        sp = (elf_addr_t __user *)bprm->p;

        /* Populate aux vector */
        NEW_AUX_ENT(AT_SYSINFO_EHDR, (elf_addr_t)current->mm->context.vdso);
        NEW_AUX_ENT(AT_CLKTCK, CLOCKS_PER_SEC);
        NEW_AUX_ENT(AT_PHDR, phdr_addr);
        NEW_AUX_ENT(AT_PHENT, sizeof(struct elf_phdr));
        NEW_AUX_ENT(AT_PHNUM, exec->e_phnum);

        /* Now, let's put argc (and argv, envp if appropriate) on the stack */
        if (put_user(argc, sp++))
            return -EFAULT;

        /* Populate list of argv pointers back to argv strings. */
        p = mm->arg_end = mm->arg_start;
        while (argc-- > 0) {
            size_t len;
            if (put_user((elf_addr_t)p, sp++))
                return -EFAULT;
            len = strnlen_user((void __user *)p, MAX_ARG_STRLEN);
            if (!len || len > MAX_ARG_STRLEN)
                return -EINVAL;
            p += len;
        }
        if (put_user(0, sp++))
            return -EFAULT;
        mm->arg_end = p;

        /* Populate list of envp pointers back to envp strings. */
        mm->env_end = mm->env_start = p;
        while (envc-- > 0) {
            size_t len;
            if (put_user((elf_addr_t)p, sp++))
                return -EFAULT;
            len = strnlen_user((void __user *)p, MAX_ARG_STRLEN);
            if (!len || len > MAX_ARG_STRLEN)
                return -EINVAL;
            p += len;
        }
        if (put_user(0, sp++))
            return -EFAULT;
        mm->env_end = p;

        /* Put the elf_info on the stack in the right place.  */
        if (copy_to_user(sp, mm->saved_auxv, ei_index * sizeof(elf_addr_t)))
            return -EFAULT;
        return 0;
    }
    if (retval < 0)
        goto out;

    mm = current->mm;
    mm->end_code = end_code;
    mm->start_code = start_code;
    mm->start_data = start_data;
    mm->end_data = end_data;
    mm->start_stack = bprm->p;

    if (current->personality & MMAP_PAGE_ZERO) {
        error = vm_mmap(NULL, 0, PAGE_SIZE, PROT_READ | PROT_EXEC, MAP_FIXED | MAP_PRIVATE, 0);
    }

    regs = current_pt_regs();

    finalize_exec(bprm) {
        current->signal->rlim[RLIMIT_STACK] = bprm->rlim_stack;
    }

    START_THREAD(elf_ex, regs, elf_entry, bprm->p) {
        start_thread(regs, elf_entry, start_stack) {
            regs->syscallno = previous_syscall;
            regs->pc = pc;
            regs->sp = sp;
        }
    }
}
```

```c
SYSCALL_DEFINE3(execve);
  do_execve();
    do_execveat_common();
        struct linux_binprm* bprm = alloc_bprm(fd, filename);
            bprm = kzalloc(sizeof(*bprm), GFP_KERNEL);
            bprm->filename = bprm->fdpath;
            bprm->interp = bprm->filename;

            bprm_mm_init(bprm);
                bprm->mm = mm = mm_alloc();
                bprm->rlim_stack = current->signal->rlim[RLIMIT_STACK];

                __bprm_mm_init(bprm);
                    bprm->vma = vma = vm_area_alloc(mm);
                    vma->vm_end = STACK_TOP_MAX;
                    vma->vm_start = vma->vm_end - PAGE_SIZE;
                    vma->vm_flags = VM_SOFTDIRTY | VM_STACK_FLAGS | VM_STACK_INCOMPLETE_SETUP;
                    vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);

                    insert_vm_struct(mm, vma);

                    mm->stack_vm = mm->total_vm = 1;
                    bprm->p = vma->vm_end - sizeof(void *);

        bprm_execve(bprm, fd, filename, flags);
            file = do_open_execat(fd, filename, flags);

            sched_exec();

            bprm->file = file;

            exec_binprm(bprm);
                search_binary_handler(bprm);
                    prepare_binprm(bprm);
                        /* read the first 128 (BINPRM_BUF_SIZE) bytes */
                        kernel_read(bprm->file, bprm->buf, BINPRM_BUF_SIZE, &pos);
                            vfs_read(file, (void __user *)buf, count, pos)

                    list_for_each_entry(fmt, &formats, lh) {
                        fmt->load_binary(bprm);
                            load_elf_binary();
                                /* consistency checks */
                                if (memcmp(elf_ex->e_ident, ELFMAG, SELFMAG) != 0)
                                    goto out;
                                if (elf_ex->e_type != ET_EXEC && elf_ex->e_type != ET_DYN)
                                    goto out;
                                if (!elf_check_arch(elf_ex))
                                    goto out;
                                if (elf_check_fdpic(elf_ex))
                                    goto out;
                                if (!bprm->file->f_op->mmap)
                                    goto out;

                                elf_ppnt = elf_phdata = load_elf_phdrs();

                                /* find PT_INTERP header */
                                for (i = 0; i < elf_ex->e_phnum; i++, elf_ppnt++) {
                                    if (elf_ppnt->p_type != PT_INTERP)
                                        continue;
                                    /* read interprete name */
                                    char* elf_interpreter = kmalloc(elf_ppnt->p_filesz, GFP_KERNEL);
                                    elf_read(bprm->file, elf_interpreter, elf_ppnt->p_filesz, elf_ppnt->p_offset);
                                    /* open interpreter file */
                                    interpreter = open_exec(elf_interpreter);
                                    /* read interpreter elfhdr */
                                    interp_elf_ex = kmalloc(sizeof(*interp_elf_ex), GFP_KERNEL);
                                    elf_read(interpreter, interp_elf_ex, sizeof(*interp_elf_ex), 0);
                                }

                                if (interpreter) {
                                    /* Load the interpreter program headers */
                                    interp_elf_phdata = load_elf_phdrs(interp_elf_ex, interpreter);
                                }

                                /* Flush all traces of the currently running executable */
                                retval = begin_new_exec(bprm);

                                setup_new_exec(bprm);
                                    arch_pick_mmap_layout();
                                        mm->mmap_base = TASK_UNMAPPED_BASE + random_factor;
                                        mm->get_unmapped_area = arch_get_unmapped_area;

                                /* Finalizes the stack vm_area_struct. */
                                setup_arg_pages(bprm, randomize_stack_top(STACK_TOP), executable_stack);
                                    shift_arg_pages();
                                        vma_adjust(vma, new_start, old_end);
                                        move_page_tables();
                                        free_pgd_range();
                                        vma_adjust(vma, new_start, new_end);
                                    current->mm->start_stack = bprm->p;
                                    expand_stack();

                                elf_bss = 0;
                                elf_brk = 0;
                                start_code = ~0UL;
                                end_code = 0;
                                start_data = 0;
                                end_data = 0;

                                /* mmapping the ELF image into the correct location in memory. */
                                for (i < loc->elf_ex.e_phnum) {
                                    if (elf_ppnt->p_type != PT_LOAD)
                                        continue;

                                    vaddr = elf_ppnt->p_vaddr;
                                    if (interpreter) {
                                        load_bias = ELF_ET_DYN_BASE;
                                    } else
                                        load_bias = 0;

                                    elf_map(bprm->file, load_bias + vaddr, elf_ppnt, elf_prot, elf_flags, total_size);
                                }

                                e_entry = elf_ex->e_entry + load_bias;
                                phdr_addr += load_bias;
                                elf_bss += load_bias;
                                elf_brk += load_bias;
                                start_code += load_bias;
                                end_code += load_bias;
                                start_data += load_bias;
                                end_data += load_bias;

                                set_brk(elf_bss + load_bias, elf_brk + load_bias, bss_prot);

                                if (elf_interpreter)
                                    elf_entry = load_elf_interp(&loc->interp_elf_ex, interpreter, &interp_map_addr, load_bias, interp_elf_phdata);
                                else
                                    elf_entry = loc->elf_ex.e_entry;

                                mm = current->mm;
                                mm->end_code = end_code;
                                mm->start_code = start_code;
                                mm->start_data = start_data;
                                mm->end_data = end_data;
                                mm->start_stack = bprm->p;

                                regs = current_pt_regs();
                                start_thread(regs, elf_entry/* new_ip */, bprm->p/* new_sp */);
                                    regs->ip = new_ip;
                                    regs->sp = new_sp;
                                    force_iret();
                    }

```

# wait4

pid val | note
--- | ---
< -1 |  meaning wait for any child process whose `process group ID is equal` to the absolute value of pid.
-1 | meaning wait for `any child process`.
0 | meaning wait for any child process whose `process group ID is equal` to that of the calling process at the time of the call to waitpid()
\> 0 | meaning wait for the child whose `process ID is equal` to the value of pid.

```c
SYSCALL_DEFINE4(wait4, pid_t, upid, int __user *, stat_addr,
        int, options, struct rusage __user *, ru)
{
    struct rusage r;
    long err = kernel_wait4(upid, stat_addr, options, ru ? &r : NULL) {
        struct wait_opts wo;
        struct pid *pid = NULL;
        enum pid_type type;
        long ret;

        if (options & ~(WNOHANG|WUNTRACED|WCONTINUED| __WNOTHREAD|__WCLONE|__WALL))
            return -EINVAL;

        /* -INT_MIN is not defined */
        if (upid == INT_MIN)
            return -ESRCH;

        if (upid == -1)
            type = PIDTYPE_MAX;
        else if (upid < 0) { /* upid < -1 */
            type = PIDTYPE_PGID;
            pid = find_get_pid(-upid);
        } else if (upid == 0) {
            type = PIDTYPE_PGID;
            pid = get_task_pid(current, PIDTYPE_PGID);
        } else { /* upid > 0 */
            type = PIDTYPE_PID;
            pid = find_get_pid(upid);
        }

        wo.wo_type      = type;
        wo.wo_pid       = pid;
        wo.wo_flags     = options | WEXITED;
        wo.wo_info      = NULL;
        wo.wo_stat      = 0;
        wo.wo_rusage    = ru;

        ret = do_wait(&wo) {
            int retval;

            child_wait_callback = []() {
                struct wait_opts *wo = container_of(wait, struct wait_opts, child_wait);
                struct task_struct *p = key;

                ret = pid_child_should_wake(wo, p) {
                    ret = !eligible_pid(wo, p) {
                        return (wo->wo_type == PIDTYPE_MAX) ||
                            (task_pid_type(p, wo->wo_type) == wo->wo_pid);
                    }
                    if (ret)
                        return false;
                    if ((wo->wo_flags & __WNOTHREAD) && wo->child_wait.private != p->parent)
                        return false;
                    return true;
                }

                if (ret) {
                    return default_wake_function(wait, mode, sync, key);
                }

                return 0;
            }
            init_waitqueue_func_entry(&wo->child_wait, child_wait_callback);
            wo->child_wait.private = current;
            add_wait_queue(&current->signal->wait_chldexit, &wo->child_wait);

            do {
                set_current_state(TASK_INTERRUPTIBLE);
                retval = __do_wait(wo) {
                    long retval;

                    wo->notask_error = -ECHILD;
                    if ((wo->wo_type < PIDTYPE_MAX)
                        && (!wo->wo_pid || !pid_has_task(wo->wo_pid, wo->wo_type))) {

                        goto notask;
                    }

                    read_lock(&tasklist_lock);

                    if (wo->wo_type == PIDTYPE_PID) {
                        retval = do_wait_pid(wo) {
                            bool ptrace;
                            struct task_struct *target;
                            int retval;

                            ptrace = false;
                            /* only thread leader added into PIDTYPE_TGID */
                            target = pid_task(wo->wo_pid, PIDTYPE_TGID);
                            ret = is_effectively_child(wo, ptrace, target) {
                                struct task_struct *parent = !ptrace
                                    ? target->real_parent : target->parent;
                                return current == parent ||
                                    (!(wo->wo_flags & __WNOTHREAD)
                                        && same_thread_group(current, parent) {
                                            return p1->signal == p2->signal
                                        }
                                    );
                            }
                            if (target && ret) {
                                retval = wait_consider_task(wo, ptrace, target);
                                if (retval)
                                    return retval;
                            }

                            ptrace = true;
                            target = pid_task(wo->wo_pid, PIDTYPE_PID);
                            if (target && target->ptrace && (wo, ptrace, target)) {
                                retval = wait_consider_task(wo, ptrace, target);
                                if (retval)
                                    return retval;
                            }

                            return 0;
                        }
                        if (retval)
                            return retval;
                    } else {
                        struct task_struct *tsk = current;

                        do {
                            retval = do_wait_thread(wo, tsk){
                                struct task_struct *p;
                                list_for_each_entry(p, &tsk->children, sibling) {
                                    int ret = wait_consider_task(wo, 0, p);
                                    if (ret)
                                        return ret;
                                }
                                return 0;
                            }
                            if (retval)
                                return retval;

                            retval = ptrace_do_wait(wo, tsk);
                            if (retval)
                                return retval;

                            if (wo->wo_flags & __WNOTHREAD)
                                break;
                        } while_each_thread(current, tsk);
                    }
                    read_unlock(&tasklist_lock);

                notask:
                    retval = wo->notask_error;
                    if (!retval && !(wo->wo_flags & WNOHANG))
                        return -ERESTARTSYS;

                    return retval;
                }

                if (retval != -ERESTARTSYS)
                    break;
                if (signal_pending(current))
                    break;

                schedule();
            } while (1);

            __set_current_state(TASK_RUNNING);
            remove_wait_queue(&current->signal->wait_chldexit, &wo->child_wait);
            return retval;
        }

        put_pid(pid);
        if (ret > 0 && stat_addr && put_user(wo.wo_stat, stat_addr))
            ret = -EFAULT;

        return ret;
    }

    if (err > 0 && (ru && copy_to_user(ru, &r, sizeof(struct rusage)))) {
        return -EFAULT;
    }

    return err;
}
```

## wait_consider_task

```c
/* Returns zero if the search for a child should continue */
retval = wait_consider_task(wo, ptrace, p) {
    int exit_state = READ_ONCE(p->exit_state);
    int ret;

    if (unlikely(exit_state == EXIT_DEAD))
        return 0;

    ret = eligible_child(wo, ptrace, p) {
        if (!eligible_pid(wo, p))
            return 0;

        if (ptrace || (wo->wo_flags & __WALL))
            return 1;

        if ((p->exit_signal != SIGCHLD) ^ !!(wo->wo_flags & __WCLONE))
            return 0;

        return 1;
    }
    if (!ret)
        return ret;

    if (unlikely(exit_state == EXIT_TRACE)) {
        if (likely(!ptrace))
            wo->notask_error = 0;
        return 0;
    }

    if (likely(!ptrace) && unlikely(p->ptrace)) {
        if (!ptrace_reparented(p))
            ptrace = 1;
    }

    /* slay zombie? */
    if (exit_state == EXIT_ZOMBIE) {
        /* we don't reap group leaders with subthreads */
        if (!delay_group_leader(p)) {
            if (unlikely(ptrace) || likely(!p->ptrace))
                return wait_task_zombie(wo, p);
        }

        if (likely(!ptrace) || (wo->wo_flags & (WCONTINUED | WEXITED)))
            wo->notask_error = 0;
    } else {
        wo->notask_error = 0;
    }

    ret = wait_task_stopped(wo, ptrace, p) {
        struct waitid_info *infop;
        int exit_code, *p_code, why;
        uid_t uid = 0; /* unneeded, required by compiler */
        pid_t pid;

        /* Traditionally we see ptrace'd stopped tasks regardless of options. */
        if (!ptrace && !(wo->wo_flags & WUNTRACED))
            return 0;

        if (!task_stopped_code(p, ptrace))
            return 0;

        exit_code = 0;
        spin_lock_irq(&p->sighand->siglock);

        p_code = task_stopped_code(p, ptrace);
        if (unlikely(!p_code))
            goto unlock_sig;

        exit_code = *p_code;
        if (!exit_code)
            goto unlock_sig;

        if (!unlikely(wo->wo_flags & WNOWAIT))
            *p_code = 0;

        uid = from_kuid_munged(current_user_ns(), task_uid(p));

    unlock_sig:
        spin_unlock_irq(&p->sighand->siglock);
        if (!exit_code)
            return 0;

        get_task_struct(p);
        pid = task_pid_vnr(p);
        why = ptrace ? CLD_TRAPPED : CLD_STOPPED;
        read_unlock(&tasklist_lock);
        sched_annotate_sleep();
        if (wo->wo_rusage)
            getrusage(p, RUSAGE_BOTH, wo->wo_rusage);
        put_task_struct(p);

        if (likely(!(wo->wo_flags & WNOWAIT)))
            wo->wo_stat = (exit_code << 8) | 0x7f;

        infop = wo->wo_info;
        if (infop) {
            infop->cause = why;
            infop->status = exit_code;
            infop->pid = pid;
            infop->uid = uid;
        }
        return pid;
    }
    if (ret)
        return ret;

    return wait_task_continued(wo, p) {
        struct waitid_info *infop;
        pid_t pid;
        uid_t uid;

        if (!unlikely(wo->wo_flags & WCONTINUED))
            return 0;

        if (!(p->signal->flags & SIGNAL_STOP_CONTINUED))
            return 0;

        spin_lock_irq(&p->sighand->siglock);
        /* Re-check with the lock held.  */
        if (!(p->signal->flags & SIGNAL_STOP_CONTINUED)) {
            spin_unlock_irq(&p->sighand->siglock);
            return 0;
        }
        if (!unlikely(wo->wo_flags & WNOWAIT))
            p->signal->flags &= ~SIGNAL_STOP_CONTINUED;
        uid = from_kuid_munged(current_user_ns(), task_uid(p));
        spin_unlock_irq(&p->sighand->siglock);

        pid = task_pid_vnr(p);
        get_task_struct(p);
        read_unlock(&tasklist_lock);
        sched_annotate_sleep();
        if (wo->wo_rusage)
            getrusage(p, RUSAGE_BOTH, wo->wo_rusage);
        put_task_struct(p);

        infop = wo->wo_info;
        if (!infop) {
            wo->wo_stat = 0xffff;
        } else {
            infop->cause = CLD_CONTINUED;
            infop->pid = pid;
            infop->uid = uid;
            infop->status = SIGCONT;
        }
        return pid;
    }
}
```

## wait_task_zombie
```c
int wait_task_zombie(struct wait_opts *wo, struct task_struct *p)
{
    int state, status;
    pid_t pid = task_pid_vnr(p);
    uid_t uid = from_kuid_munged(current_user_ns(), task_uid(p));
    struct waitid_info *infop;

    if (!likely(wo->wo_flags & WEXITED))
        return 0;

    if (unlikely(wo->wo_flags & WNOWAIT)) {
        status = (p->signal->flags & SIGNAL_GROUP_EXIT)
            ? p->signal->group_exit_code : p->exit_code;
        get_task_struct(p);
        read_unlock(&tasklist_lock);
        sched_annotate_sleep();
        if (wo->wo_rusage)
            getrusage(p, RUSAGE_BOTH, wo->wo_rusage);
        put_task_struct(p);
        goto out_info;
    }
    /* Move the task's state to DEAD/TRACE, only one thread can do this. */
    state = (ptrace_reparented(p) && thread_group_leader(p)) ?
        EXIT_TRACE : EXIT_DEAD;
    if (cmpxchg(&p->exit_state, EXIT_ZOMBIE, state) != EXIT_ZOMBIE)
        return 0;
    /* We own this thread, nobody else can reap it. */
    read_unlock(&tasklist_lock);
    sched_annotate_sleep();

    /* Check thread_group_leader() to exclude the traced sub-threads. */
    if (state == EXIT_DEAD && thread_group_leader(p)) {
        struct signal_struct *sig = p->signal;
        struct signal_struct *psig = current->signal;
        unsigned long maxrss;
        u64 tgutime, tgstime;

        thread_group_cputime_adjusted(p, &tgutime, &tgstime);
        spin_lock_irq(&current->sighand->siglock);
        write_seqlock(&psig->stats_lock);
        psig->cutime += tgutime + sig->cutime;
        psig->cstime += tgstime + sig->cstime;
        psig->cgtime += task_gtime(p) + sig->gtime + sig->cgtime;
        psig->cmin_flt += p->min_flt + sig->min_flt + sig->cmin_flt;
        psig->cmaj_flt += p->maj_flt + sig->maj_flt + sig->cmaj_flt;
        psig->cnvcsw += p->nvcsw + sig->nvcsw + sig->cnvcsw;
        psig->cnivcsw += p->nivcsw + sig->nivcsw + sig->cnivcsw;
        psig->cinblock +=
            task_io_get_inblock(p) + sig->inblock + sig->cinblock;
        psig->coublock +=
            task_io_get_oublock(p) + sig->oublock + sig->coublock;
        maxrss = max(sig->maxrss, sig->cmaxrss);
        if (psig->cmaxrss < maxrss)
            psig->cmaxrss = maxrss;
        task_io_accounting_add(&psig->ioac, &p->ioac);
        task_io_accounting_add(&psig->ioac, &sig->ioac);
        write_sequnlock(&psig->stats_lock);
        spin_unlock_irq(&current->sighand->siglock);
    }

    if (wo->wo_rusage)
        getrusage(p, RUSAGE_BOTH, wo->wo_rusage);
    status = (p->signal->flags & SIGNAL_GROUP_EXIT)
        ? p->signal->group_exit_code : p->exit_code;
    wo->wo_stat = status;

    if (state == EXIT_TRACE) {
        write_lock_irq(&tasklist_lock);
        /* We dropped tasklist, ptracer could die and untrace */
        ptrace_unlink(p);

        /* If parent wants a zombie, don't release it now */
        state = EXIT_ZOMBIE;
        if (do_notify_parent(p, p->exit_signal))
            state = EXIT_DEAD;
        p->exit_state = state;
        write_unlock_irq(&tasklist_lock);
    }
    if (state == EXIT_DEAD)
        release_task(p);

out_info:
    infop = wo->wo_info;
    if (infop) {
        if ((status & 0x7f) == 0) {
            infop->cause = CLD_EXITED;
            infop->status = status >> 8;
        } else {
            infop->cause = (status & 0x80) ? CLD_DUMPED : CLD_KILLED;
            infop->status = status & 0x7f;
        }
        infop->pid = pid;
        infop->uid = uid;
    }

    return pid;
}
```

# exit

```c
do_exit(code) {
    WARN_ON(irqs_disabled());
    synchronize_group_exit(tsk, code) {
        struct sighand_struct *sighand = tsk->sighand;
        struct signal_struct *signal = tsk->signal;

        spin_lock_irq(&sighand->siglock);
        signal->quick_threads--;
        if ((signal->quick_threads == 0) &&
            !(signal->flags & SIGNAL_GROUP_EXIT)) {
            signal->flags = SIGNAL_GROUP_EXIT;
            signal->group_exit_code = code;
            signal->group_stop_count = 0;
        }
        spin_unlock_irq(&sighand->siglock);
    }

    kcov_task_exit(tsk);
    kmsan_task_exit(tsk);

    coredump_task_exit(tsk);
    ptrace_event(PTRACE_EVENT_EXIT, code);

    validate_creds_for_do_exit(tsk);

    io_uring_files_cancel();
    exit_signals(tsk);

    group_dead = atomic_dec_and_test(&tsk->signal->live);

    exit_mm();
    exit_sem(tsk);

    exit_shm(tsk);
    exit_files(tsk);
    exit_fs(tsk);
    exit_task_namespaces(tsk);
    exit_task_work(tsk);
    exit_thread(tsk);
    perf_event_exit_task(tsk);
    sched_autogroup_exit_task(tsk);
    cgroup_exit(tsk);
    exit_tasks_rcu_start();
    exit_notify(tsk, group_dead);
    proc_exit_connector(tsk);
    mpol_put_task_policy(tsk);
    exit_io_context(tsk);
    free_pipe_info(tsk->splice_pipe);
    put_page(tsk->task_frag.page);
    exit_rcu();
    exit_tasks_rcu_finish();
    lockdep_free_task(tsk);

    do_task_dead() {
        /* Causes final put_task_struct in finish_task_switch(): */
        set_special_state(TASK_DEAD);

        /* Tell freezer to ignore us: */
        current->flags |= PF_NOFREEZE;

        __schedule(SM_NONE);
        BUG();

        /* Avoid "noreturn function does return" - but don't continue if BUG() is a NOP: */
        for (;;)
            cpu_relax();
    }
}
```

## exit_signals

```c
void exit_signals(struct task_struct *tsk)
{
    int group_stop = 0;
    sigset_t unblocked;

    cgroup_threadgroup_change_begin(tsk);

    if (thread_group_empty(tsk) || (tsk->signal->flags & SIGNAL_GROUP_EXIT)) {
        sched_mm_cid_exit_signals(tsk);
        tsk->flags |= PF_EXITING;
        cgroup_threadgroup_change_end(tsk);
        return;
    }

    spin_lock_irq(&tsk->sighand->siglock);
    /* From now this task is not visible for group-wide signals,
     * see wants_signal(), do_signal_stop(). */
    sched_mm_cid_exit_signals(tsk);
    tsk->flags |= PF_EXITING;

    cgroup_threadgroup_change_end(tsk);

    if (!task_sigpending(tsk))
        goto out;

    unblocked = tsk->blocked;
    signotset(&unblocked);

    retarget_shared_pending(tsk, &unblocked/*which*/) {
        sigset_t retarget;
        struct task_struct *t;

        sigandsets(&retarget, &tsk->signal->shared_pending.signal, which);
        if (sigisemptyset(&retarget))
            return;

        for_other_threads(tsk, t) {
            if (t->flags & PF_EXITING)
                continue;

            if (!has_pending_signals(&retarget, &t->blocked))
                continue;
            /* Remove the signals this thread can handle. */
            sigandsets(&retarget, &retarget, &t->blocked);

            if (!task_sigpending(t))
                signal_wake_up(t, 0);

            if (sigisemptyset(&retarget))
                break;
        }
    }

    if (unlikely(tsk->jobctl & JOBCTL_STOP_PENDING) &&
        task_participate_group_stop(tsk))
        group_stop = CLD_STOPPED;
out:
    spin_unlock_irq(&tsk->sighand->siglock);

    /* If group stop has completed, deliver the notification.  This
     * should always go to the real parent of the group leader. */
    if (unlikely(group_stop)) {
        read_lock(&tasklist_lock);
        do_notify_parent_cldstop(tsk, false/*for_ptracer*/, group_stop/*why*/) {
            if (for_ptracer) {
                parent = tsk->parent;
            } else {
                tsk = tsk->group_leader;
                parent = tsk->real_parent;
            }

            clear_siginfo(&info);
            info.si_signo = SIGCHLD;
            info.si_errno = 0;

            rcu_read_lock();
            info.si_pid = task_pid_nr_ns(tsk, task_active_pid_ns(parent));
            info.si_uid = from_kuid_munged(task_cred_xxx(parent, user_ns), task_uid(tsk));
            rcu_read_unlock();

            task_cputime(tsk, &utime, &stime);
            info.si_utime = nsec_to_clock_t(utime);
            info.si_stime = nsec_to_clock_t(stime);

            info.si_code = why;
            switch (why) {
            case CLD_CONTINUED:
                info.si_status = SIGCONT;
                break;
            case CLD_STOPPED:
                info.si_status = tsk->signal->group_exit_code & 0x7f;
                break;
            case CLD_TRAPPED:
                info.si_status = tsk->exit_code & 0x7f;
                break;
            default:
                BUG();
            }

            sighand = parent->sighand;
            spin_lock_irqsave(&sighand->siglock, flags);
            if (sighand->action[SIGCHLD-1].sa.sa_handler != SIG_IGN
                && !(sighand->action[SIGCHLD-1].sa.sa_flags & SA_NOCLDSTOP)) {

                send_signal_locked(SIGCHLD, &info, parent, PIDTYPE_TGID);
            }
            /* Even if SIGCHLD is not generated, we must wake up wait4 calls. */
            __wake_up_parent(tsk, parent);
            spin_unlock_irqrestore(&sighand->siglock, flags);
        }
        read_unlock(&tasklist_lock);
    }
}
```

## exit_notify

```c
void exit_notify(struct task_struct *tsk, int group_dead)
{
    bool autoreap;
    struct task_struct *p, *n;
    LIST_HEAD(dead);

    write_lock_irq(&tasklist_lock);

    /* A. Make init inherit all the child processes
     * B. Check to see if any process groups have become orphaned
     * as a result of our exiting, and if they have any stopped
     * jobs, send them a SIGHUP and then a SIGCONT.  (POSIX 3.2.2.2) */
    forget_original_parent(tsk/*father*/, &dead) {
        struct task_struct *p, *t, *reaper;

        if (unlikely(!list_empty(&father->ptraced)))
            exit_ptrace(father, dead);

        reaper = find_child_reaper(father, dead) {
            struct pid_namespace *pid_ns = task_active_pid_ns(father);
            struct task_struct *reaper = pid_ns->child_reaper;
            struct task_struct *p, *n;

            if (likely(reaper != father))
                return reaper;

            reaper = find_alive_thread(father) {
                for_each_thread(p, t) {
                    if (!(t->flags & PF_EXITING))
                        return t;
                }
            }
            if (reaper) {
                pid_ns->child_reaper = reaper;
                return reaper;
            }

            write_unlock_irq(&tasklist_lock);

            list_for_each_entry_safe(p, n, dead, ptrace_entry) {
                list_del_init(&p->ptrace_entry);
                release_task(p);
                    --->
            }

            /* Q: kill all processes in the ns? */
            zap_pid_ns_processes(pid_ns) {
                struct task_struct *task, *me = current;
                int init_pids = thread_group_leader(me) ? 1 : 2;
                struct pid *pid;

                /* Don't allow any more processes into the pid namespace */
                disable_pid_allocation(pid_ns);

                me->sighand->action[SIGCHLD - 1].sa.sa_handler = SIG_IGN;

                nr = 2;
                idr_for_each_entry_continue(&pid_ns->idr, pid, nr) {
                    task = pid_task(pid, PIDTYPE_PID);
                    if (task && !__fatal_signal_pending(task))
                        group_send_sig_info(SIGKILL, SEND_SIG_PRIV, task, PIDTYPE_MAX);
                }

                do {
                    clear_thread_flag(TIF_SIGPENDING);
                    rc = kernel_wait4(-1, NULL, __WALL, NULL);
                } while (rc != -ECHILD);

                for (;;) {
                    set_current_state(TASK_INTERRUPTIBLE);
                    if (pid_ns->pid_allocated == init_pids)
                        break;

                    exit_tasks_rcu_stop();
                    schedule();
                    exit_tasks_rcu_start();
                }
                __set_current_state(TASK_RUNNING);

                if (pid_ns->reboot)
                    current->signal->group_exit_code = pid_ns->reboot;
            }
            return father;
        }

        if (list_empty(&father->children))
            return;

        /* When we die, we re-parent all our children, and try to:
         * 3. give it to the init process (PID 1) in our pid namespace */
        reaper = find_new_reaper(father, reaper/*child_reaper*/) {
            /* 1. give them to another thread in our thread group, if such a member exists */
            thread = find_alive_thread(father);
            if (thread)
                return thread;

            /* 2. give it to the first ancestor process which prctl'd itself as a
             *    child_subreaper for its children (like a service manager) */
            if (father->signal->has_child_subreaper) {
                unsigned int ns_level = task_pid(father)->level;
                for (reaper = father->real_parent;
                    task_pid(reaper)->level == ns_level;
                    reaper = reaper->real_parent) {

                    if (reaper == &init_task)
                        break;
                    if (!reaper->signal->is_child_subreaper)
                        continue;
                    thread = find_alive_thread(reaper);
                    if (thread)
                        return thread;
                }
            }

            return child_reaper;
        }

        list_for_each_entry(p, &father->children, sibling) {
            for_each_thread(p, t) {
                RCU_INIT_POINTER(t->real_parent, reaper);
                if (likely(!t->ptrace))
                    t->parent = t->real_parent;
                if (t->pdeath_signal)
                    group_send_sig_info(t->pdeath_signal, SEND_SIG_NOINFO, t, PIDTYPE_TGID);
            }
            /* If this is a threaded reparent there is no need to
             * notify anyone anything has happened. */
            if (!same_thread_group(reaper, father)) {
                reparent_leader(father, p, dead) {
                    if (unlikely(p->exit_state == EXIT_DEAD))
                        return;

                    /* We don't want people slaying init. */
                    p->exit_signal = SIGCHLD;

                    /* If it has exited notify the new parent about this child's death. */
                    if (!p->ptrace &&
                        p->exit_state == EXIT_ZOMBIE && thread_group_empty(p)) {
                        if (do_notify_parent(p, p->exit_signal)) {
                            p->exit_state = EXIT_DEAD;
                            list_add(&p->ptrace_entry, dead);
                        }
                    }

                    kill_orphaned_pgrp(p, father);
                }
            }
        }
        list_splice_tail_init(&father->children, &reaper->children);
    }

    if (group_dead) {
        kill_orphaned_pgrp(tsk->group_leader, NULL/*parent*/) {
            struct pid *pgrp = task_pgrp(tsk);
            struct task_struct *ignored_task = tsk;

            if (!parent)
                /* exit: our father is in a different pgrp than
                 * we are and we were the only connection outside. */
                parent = tsk->real_parent;
            else
                /* reparent: our child is in a different pgrp than
                 * we are, and it was the only connection outside. */
                ignored_task = NULL;

            if (task_pgrp(parent) != pgrp
                && task_session(parent) == task_session(tsk)
                && will_become_orphaned_pgrp(pgrp, ignored_task)
                && has_stopped_jobs(pgrp)) {

                __kill_pgrp_info(SIGHUP, SEND_SIG_PRIV, pgrp);
                __kill_pgrp_info(SIGCONT, SEND_SIG_PRIV, pgrp) {
                    do_each_pid_task(pgrp, PIDTYPE_PGID, p) {
                        int err = group_send_sig_info(sig, info, p, PIDTYPE_PGID) {
                            do_send_sig_info(sig, info, p, type);
                        }
                    } while_each_pid_task(pgrp, PIDTYPE_PGID, p);
                }
            }
        }
    }

    tsk->exit_state = EXIT_ZOMBIE;
    if (unlikely(tsk->ptrace)) {
        int sig = thread_group_leader(tsk) &&
                thread_group_empty(tsk) &&
                !ptrace_reparented(tsk)
            ? tsk->exit_signal : SIGCHLD;
        autoreap = do_notify_parent(tsk, sig);
    } else if (thread_group_leader(tsk)) {
        autoreap = thread_group_empty(tsk) &&
            do_notify_parent(tsk, tsk->exit_signal);
    } else {
        autoreap = true;
    }

    if (autoreap) {
        tsk->exit_state = EXIT_DEAD;
        list_add(&tsk->ptrace_entry, &dead);
    }

    /* mt-exec, de_thread() is waiting for group leader */
    if (unlikely(tsk->signal->notify_count < 0))
        wake_up_process(tsk->signal->group_exec_task);
    write_unlock_irq(&tasklist_lock);

    list_for_each_entry_safe(p, n, &dead, ptrace_entry) {
        list_del_init(&p->ptrace_entry);
        release_task(p) {
            struct task_struct *leader;
            struct pid *thread_pid;
            int zap_leader;
        repeat:
            /* don't need to get the RCU readlock here - the process is dead and
            * can't be modifying its own credentials. But shut RCU-lockdep up */
            rcu_read_lock();
            dec_rlimit_ucounts(task_ucounts(p), UCOUNT_RLIMIT_NPROC, 1);
            rcu_read_unlock();

            cgroup_release(p);

            write_lock_irq(&tasklist_lock);
            ptrace_release_task(p);
            thread_pid = get_pid(p->thread_pid);
            __exit_signal(p);

            /* If we are the last non-leader member of the thread
             * group, and the leader is zombie, then notify the
             * group leader's parent process. (if it wants notification.) */
            zap_leader = 0;
            leader = p->group_leader;
            if (leader != p && thread_group_empty(leader)
                && leader->exit_state == EXIT_ZOMBIE) {

                zap_leader = do_notify_parent(leader, leader->exit_signal);
                if (zap_leader)
                    leader->exit_state = EXIT_DEAD;
            }

            write_unlock_irq(&tasklist_lock);
            seccomp_filter_release(p);
            proc_flush_pid(thread_pid);
            put_pid(thread_pid);
            release_thread(p);
            put_task_struct_rcu_user(p);

            p = leader;
            if (unlikely(zap_leader))
                goto repeat;
        }
    }
}
```

# kthreadd

```c
/* kernel/kthread.c */
static DEFINE_SPINLOCK(kthread_create_lock);
static LIST_HEAD(kthread_create_list);
struct task_struct *kthreadd_task;

struct kthread_create_info
{
    int (*threadfn)(void *data);
    void *data;
    int node;

    /* Result passed back to kthread_create() from kthreadd. */
    struct task_struct *result;
    struct completion *done;

    struct list_head list;
};

int kthreadd(void *unused)
{
    struct task_struct *tsk = current;

    /* Setup a clean context for our children to inherit. */
    set_task_comm(tsk, "kthreadd");
    ignore_signals(tsk);
    set_cpus_allowed_ptr(tsk, cpu_all_mask);
    set_mems_allowed(node_states[N_MEMORY]);

    current->flags |= PF_NOFREEZE;
    cgroup_init_kthreadd();

    for (;;) {
        set_current_state(TASK_INTERRUPTIBLE);
        if (list_empty(&kthread_create_list))
        schedule();
        __set_current_state(TASK_RUNNING);

        spin_lock(&kthread_create_lock);
        while (!list_empty(&kthread_create_list)) {
            struct kthread_create_info *create;

            create = list_entry(kthread_create_list.next, struct kthread_create_info, list);
            list_del_init(&create->list);
            spin_unlock(&kthread_create_lock);

            create_kthread(create) {
                kernel_thread(kthread, create, CLONE_FS | CLONE_FILES | SIGCHLD) {
                    return _do_fork(flags|CLONE_VM|CLONE_UNTRACED, (unsigned long)fn,
                        (unsigned long)arg, NULL, NULL, 0);
                }
            }

            spin_lock(&kthread_create_lock);
        }
        spin_unlock(&kthread_create_lock);
    }

  return 0;
}

static int kthread(void *_create)
{
    /* Copy data: it's on kthread's stack */
    struct kthread_create_info *create = _create;
    int (*threadfn)(void *data) = create->threadfn;
    void *data = create->data;
    struct completion *done;
    struct kthread *self;
    int ret;

    self = kzalloc(sizeof(*self), GFP_KERNEL);
    set_kthread_struct(self);

    /* If user was SIGKILLed, I release the structure. */
    done = xchg(&create->done, NULL);
    if (!done) {
        kfree(create);
        do_exit(-EINTR);
    }

    if (!self) {
        create->result = ERR_PTR(-ENOMEM);
        complete(done);
        do_exit(-ENOMEM);
    }

    self->data = data;
    init_completion(&self->exited);
    init_completion(&self->parked);
    current->vfork_done = &self->exited;

    /* OK, tell user we're spawned, wait for stop or wakeup */
    __set_current_state(TASK_UNINTERRUPTIBLE);
    create->result = current;
    /* Thread is going to call schedule(), do not preempt it,
    * or the creator may spend more time in wait_task_inactive(). */
    preempt_disable();
    complete(done);
    schedule_preempt_disabled();
    preempt_enable();

    ret = -EINTR;
    if (!test_bit(KTHREAD_SHOULD_STOP, &self->flags)) {
        cgroup_kthread_ready();
        __kthread_parkme(self) {
            __kthread_parkme(struct kthread *self) {
                for (;;) {

                    set_special_state(TASK_PARKED);
                    if (!test_bit(KTHREAD_SHOULD_PARK, &self->flags))
                        break;

                    preempt_disable();
                    complete(&self->parked);
                    schedule_preempt_disabled();
                    preempt_enable();
                }
                __set_current_state(TASK_RUNNING);
            }
        }
        ret = threadfn(data);
    }
    do_exit(ret);
}
```

# cmwq

* [Kernel Doc](https://docs.kernel.org/core-api/workqueue.html)

* Wowo Tech [:one: Basic Concept](http://www.wowotech.net/irq_subsystem/workqueue.html) [:two: Overview](http://www.wowotech.net/irq_subsystem/cmwq-intro.html)  [:three: Code Anatomy](http://www.wowotech.net/irq_subsystem/alloc_workqueue.html)  [:four: Handle Work](http://www.wowotech.net/irq_subsystem/queue_and_handle_work.html)

<img src='../images/kernel/proc-cmwq.png' style='max-height:850px'/>

---

<img src='../images/kernel/proc-cmwq-flow.png' style='max-height:850px'/>

* `nr_running`    `nr_active`    `max_active`    `CPU_INTENSIVE` control the concurrency
---

<img src='../images/kernel/proc-cmwq-state.png' style='max-height:850px'/>

---

<img src='../images/kernel/proc-cmwq-arch.png' style='max-height:850px'/>

* [Kernel 4.19: Concurrency Managed Workqueue (cmwq)](https://www.kernel.org/doc/html/v4.19/core-api/workqueue.html)
* http://www.wowotech.net/irq_subsystem/cmwq-intro.html :cn:
* https://zhuanlan.zhihu.com/p/91106844 :cn:
* https://zhuanlan.zhihu.com/p/94561631 :cn:
* http://kernel.meizu.com/linux-workqueue.html :cn:


```c
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


## wq-struct
```c
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

```c
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

```c
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

# task_group

* [Oracle - CFS Group Scheduling](https://blogs.oracle.com/linux/post/cfs-group-scheduling)
* [内核工匠 - CFS组调度](https://mp.weixin.qq.com/s/BbXFZSq6xFclRahX7oPD9A)
* [Dumpstack](http://www.dumpstack.cn/index.php/2022/04/05/726.html)

* Group scheduling is extensively used in a myriad of use cases on different types of systems. Large enterprise systems use it for containers, user sessions etc. Embedded systems like android use it to segregate tasks of varying importance (e.g. foreground vs background) from each other.

![](../images/kernel/proc-sched-task_group.png)

---

![](../images/kernel/proc-sched-task_group-2.png)

---

![](../images/kernel/proc-sched-task_group-3.png)

---

![](../images/kernel/proc-sched-task_group-4.png)

---

![](../images/kernel/proc-sched-task-group-period-quota.png)


```c
struct task_group {
    struct cgroup_subsys_state  css;

#ifdef CONFIG_FAIR_GROUP_SCHED
    struct sched_entity         **se;
    struct cfs_rq               **cfs_rq;
    unsigned long               shares;
    int                         idle;
    atomic_long_t               load_avg;
#endif

#ifdef CONFIG_RT_GROUP_SCHED
    struct sched_rt_entity      **rt_se;
    struct rt_rq                **rt_rq;
    struct rt_bandwidth         rt_bandwidth;
#endif

    struct rcu_head             rcu;
    struct list_head            list;

    struct task_group           *parent;
    struct list_head            siblings;
    struct list_head            children;

#ifdef CONFIG_SCHED_AUTOGROUP
    struct autogroup            *autogroup;
#endif

    struct cfs_bandwidth        cfs_bandwidth;
};
```

## sched_create_group

```c
struct task_group *sched_create_group(struct task_group *parent) {
    struct task_group *tg;

    tg = kmem_cache_alloc(task_group_cache, GFP_KERNEL | __GFP_ZERO);

    alloc_fair_sched_group(tg, parent) {
        tg->cfs_rq = kcalloc(nr_cpu_ids, sizeof(cfs_rq), GFP_KERNEL);

        tg->se = kcalloc(nr_cpu_ids, sizeof(se), GFP_KERNEL);

        tg->shares = NICE_0_LOAD;

        init_cfs_bandwidth(tg_cfs_bandwidth(tg), tg_cfs_bandwidth(parent)) {
            --->
        }

        for_each_possible_cpu(i) {
            cfs_rq = kzalloc_node(sizeof(struct cfs_rq), GFP_KERNEL, cpu_to_node(i));
            se = kzalloc_node(sizeof(struct sched_entity_stats), GFP_KERNEL, cpu_to_node(i));

            init_cfs_rq(cfs_rq) {
                cfs_rq->tasks_timeline = RB_ROOT_CACHED;
                u64_u32_store(cfs_rq->min_vruntime, (u64)(-(1LL << 20)));
                raw_spin_lock_init(&cfs_rq->removed.lock);
            }

            init_tg_cfs_entry(tg, cfs_rq, se, i/*cpu*/, parent->se[i]/*parent*/) {
                struct rq *rq = cpu_rq(cpu);

                cfs_rq->tg = tg;
                cfs_rq->rq = rq;
                init_cfs_rq_runtime(cfs_rq) {
                    cfs_rq->runtime_enabled = 0;
                    INIT_LIST_HEAD(&cfs_rq->throttled_list);
                    INIT_LIST_HEAD(&cfs_rq->throttled_csd_list);
                }

                tg->cfs_rq[cpu] = cfs_rq;
                tg->se[cpu] = se;

                /* se could be NULL for root_task_group */
                if (!se)
                    return;

                if (!parent) {
                    se->cfs_rq = &rq->cfs;
                    se->depth = 0;
                } else {
                    se->cfs_rq = parent->my_q;
                    se->depth = parent->depth + 1;
                }

                se->my_q = cfs_rq;
                /* guarantee group entities always have weight */
                update_load_set(&se->load/*lw*/, NICE_0_LOAD/*w*/) {
                    lw->weight = w;
                    lw->inv_weight = 0;
                }
                se->parent = parent;
            }
            init_entity_runnable_average(se);
        }
    }

    alloc_rt_sched_group(tg, parent) {
        struct rt_rq *rt_rq;
        struct sched_rt_entity *rt_se;
        int i;

        tg->rt_rq = kcalloc(nr_cpu_ids, sizeof(rt_rq), GFP_KERNEL);
        tg->rt_se = kcalloc(nr_cpu_ids, sizeof(rt_se), GFP_KERNEL);

        init_rt_bandwidth(&tg->rt_bandwidth,
                ktime_to_ns(def_rt_bandwidth.rt_period), 0);

        for_each_possible_cpu(i) {
            rt_rq = kzalloc_node(sizeof(struct rt_rq), GFP_KERNEL, cpu_to_node(i));
            rt_se = kzalloc_node(sizeof(struct sched_rt_entity), GFP_KERNEL, cpu_to_node(i));

            init_rt_rq(rt_rq);
            rt_rq->rt_runtime = tg->rt_bandwidth.rt_runtime;
            init_tg_rt_entry(tg, rt_rq, rt_se, i/*cpu*/, parent->rt_se[i]/*parent*/) {
                struct rq *rq = cpu_rq(cpu);

                rt_rq->highest_prio.curr = MAX_RT_PRIO-1;
                rt_rq->rt_nr_boosted = 0;
                rt_rq->rq = rq;
                rt_rq->tg = tg;

                tg->rt_rq[cpu] = rt_rq;
                tg->rt_se[cpu] = rt_se;

                if (!parent)
                    rt_se->rt_rq = &rq->rt;
                else
                    rt_se->rt_rq = parent->my_q;

                rt_se->my_q = rt_rq;
                rt_se->parent = parent;
                INIT_LIST_HEAD(&rt_se->run_list);
            }
        }
    }

    alloc_uclamp_sched_group(tg, parent);

    return tg;
}
```

# cfs_bandwidth

![](../images/kernel/proc-sched-cfs-period-quota.png)

* [LoyenWang](https://www.cnblogs.com/LoyenWang/tag/进程调度/)
    * [4. 组调度及带宽控制](https://www.cnblogs.com/LoyenWang/p/12459000.html)

```c
struct cfs_bandwidth {
    raw_spinlock_t      lock;
    /* in each period time, the grp has quota time to run */
    ktime_t             period;
    u64                 quota; /* run-time replenished within a period in ms */
    u64                 runtime; /* the remaining time of the quota */
    u64                 burst;
    u64                 runtime_snap;
    s64                 hierarchical_quota;

    u8                  idle;
    u8                  period_active;
    u8                  slack_started;
    struct hrtimer      period_timer; /* refill the runtime */
    struct hrtimer      slack_timer; /* return the remaining time to tg time pool */
    struct list_head    throttled_cfs_rq;

    int                 nr_periods;
    int                 nr_throttled;
    int                 nr_burst;
    u64                 throttled_time;
    u64                 burst_time; /* the maximum accumulated run-time in ms */
};
```

## init_cfs_bandwidth

![](../images/kernel/proc-sched-cfs-init_cfs_bandwidth.png)

```c
void init_cfs_bandwidth(struct cfs_bandwidth *cfs_b, struct cfs_bandwidth *parent)
{
    raw_spin_lock_init(&cfs_b->lock);
    cfs_b->runtime = 0;
    cfs_b->quota = RUNTIME_INF;
    cfs_b->period = ns_to_ktime(default_cfs_period());
    cfs_b->burst = 0;
    cfs_b->hierarchical_quota = parent ? parent->hierarchical_quota : RUNTIME_INF;

    INIT_LIST_HEAD(&cfs_b->throttled_cfs_rq);
    hrtimer_init(&cfs_b->period_timer, CLOCK_MONOTONIC, HRTIMER_MODE_ABS_PINNED);
    cfs_b->period_timer.function = sched_cfs_period_timer;

    /* Add a random offset so that timers interleave */
    hrtimer_set_expires(&cfs_b->period_timer,
                get_random_u32_below(cfs_b->period));
    hrtimer_init(&cfs_b->slack_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    cfs_b->slack_timer.function = sched_cfs_slack_timer;
    cfs_b->slack_started = false;
}
```

## sched_cfs_period_timer

```c
enum hrtimer_restart sched_cfs_period_timer(struct hrtimer *timer)
{
    struct cfs_bandwidth *cfs_b = container_of(timer, struct cfs_bandwidth, period_timer);
    unsigned long flags;
    int overrun;
    int idle = 0;
    int count = 0;

    raw_spin_lock_irqsave(&cfs_b->lock, flags);
    for (;;) {
        overrun = hrtimer_forward_now(timer, cfs_b->period);
        if (!overrun)
            break;

        idle = do_sched_cfs_period_timer(cfs_b, overrun, flags) {
            int throttled;

            /* no need to continue the timer with no bandwidth constraint */
            if (cfs_b->quota == RUNTIME_INF)
                goto out_deactivate;

            throttled = !list_empty(&cfs_b->throttled_cfs_rq);
            cfs_b->nr_periods += overrun;

            /* Refill extra burst quota even if cfs_b->idle */
            __refill_cfs_bandwidth_runtime(cfs_b) {
                cfs_b->runtime += cfs_b->quota;
                /* delta bewteen prev remaining runtime and current remaining runtime */
                runtime = cfs_b->runtime_snap - cfs_b->runtime;
                if (runtime > 0) {
                    cfs_b->burst_time += runtime;
                    cfs_b->nr_burst++;
                }

                cfs_b->runtime = min(cfs_b->runtime, cfs_b->quota + cfs_b->burst);
                cfs_b->runtime_snap = cfs_b->runtime;
            }

            if (cfs_b->idle && !throttled)
                goto out_deactivate;

            if (!throttled) {
                cfs_b->idle = 1;
                return 0;
            }

            cfs_b->nr_throttled += overrun;

            while (throttled && cfs_b->runtime > 0) {
                raw_spin_unlock_irqrestore(&cfs_b->lock, flags);
                throttled = distribute_cfs_runtime(cfs_b);
                    --->
                raw_spin_lock_irqsave(&cfs_b->lock, flags);
            }

            cfs_b->idle = 0;

            return 0;

        out_deactivate:
            return 1;
        }

        if (++count > 3) {
            u64 new, old = ktime_to_ns(cfs_b->period);

            /* Grow period by a factor of 2 to avoid losing precision.
            * Precision loss in the quota/period ratio can cause __cfs_schedulable
            * to fail. */
            new = old * 2;
            if (new < max_cfs_quota_period) {
                cfs_b->period = ns_to_ktime(new);
                cfs_b->quota *= 2;
                cfs_b->burst *= 2;
            }

            /* reset count so we don't come right back in here */
            count = 0;
        }
    }
    if (idle)
        cfs_b->period_active = 0;
    raw_spin_unlock_irqrestore(&cfs_b->lock, flags);

    return idle ? HRTIMER_NORESTART : HRTIMER_RESTART;
}
```

## sched_cfs_slack_timer

```c
/* Slack time refers to periods when the CPU is not fully utilized,
 * i.e., when there are no runnable tasks to execute */
enum hrtimer_restart sched_cfs_slack_timer(struct hrtimer *timer)
{
    struct cfs_bandwidth *cfs_b =
        container_of(timer, struct cfs_bandwidth, slack_timer);

    do_sched_cfs_slack_timer(cfs_b) {
        u64 runtime = 0, slice = sched_cfs_bandwidth_slice() {
            return (u64)sysctl_sched_cfs_bandwidth_slice * NSEC_PER_USEC;
        }
        unsigned long flags;

        /* confirm we're still not at a refresh boundary */
        raw_spin_lock_irqsave(&cfs_b->lock, flags);
        cfs_b->slack_started = false;

        /* Are we near the end of the current quota period?
         * Yes: throttled tasks will be unthrottled in sched_cfs_period_timer
         * NO: throttled tasks need to be unthrottled here */
        if (runtime_refresh_within(cfs_b, min_bandwidth_expiration)) {
            raw_spin_unlock_irqrestore(&cfs_b->lock, flags);
            return;
        }

        if (cfs_b->quota != RUNTIME_INF && cfs_b->runtime > slice)
            runtime = cfs_b->runtime;

        raw_spin_unlock_irqrestore(&cfs_b->lock, flags);

        if (!runtime)
            return;

        /* cfs_b distributes remaining runtime to throttled cfs_rq */
        distribute_cfs_runtime(cfs_b) {
            int this_cpu = smp_processor_id();
            u64 runtime, remaining = 1;
            bool throttled = false;
            struct cfs_rq *cfs_rq, *tmp;
            struct rq_flags rf;
            struct rq *rq;
            LIST_HEAD(local_unthrottle);

            rcu_read_lock();
            list_for_each_entry_rcu(cfs_rq, &cfs_b->throttled_cfs_rq, throttled_list) {
                rq = rq_of(cfs_rq);

                if (!remaining) {
                    throttled = true;
                    break;
                }

                rq_lock_irqsave(rq, &rf);
                if (!cfs_rq_throttled(cfs_rq))
                    goto next;

                /* Already queued for async unthrottle */
                if (!list_empty(&cfs_rq->throttled_csd_list))
                    goto next;

                raw_spin_lock(&cfs_b->lock);
                /* By the above checks, this should never be true */
                SCHED_WARN_ON(cfs_rq->runtime_remaining > 0);
                runtime = -cfs_rq->runtime_remaining + 1;
                if (runtime > cfs_b->runtime) {
                    runtime = cfs_b->runtime;
                }
                cfs_b->runtime -= runtime;
                remaining = cfs_b->runtime;
                raw_spin_unlock(&cfs_b->lock);

                cfs_rq->runtime_remaining += runtime;

                /* we check whether we're throttled above */
                if (cfs_rq->runtime_remaining > 0) {
                    if (cpu_of(rq) != this_cpu) {
                        unthrottle_cfs_rq_async(cfs_rq);
                    } else {
                        list_add_tail(&cfs_rq->throttled_csd_list, &local_unthrottle);
                    }
                } else {
                    throttled = true;
                }

        next:
                rq_unlock_irqrestore(rq, &rf);
            }

            list_for_each_entry_safe(cfs_rq, tmp, &local_unthrottle, throttled_csd_list) {
                struct rq *rq = rq_of(cfs_rq);

                rq_lock_irqsave(rq, &rf);

                list_del_init(&cfs_rq->throttled_csd_list);

                if (cfs_rq_throttled(cfs_rq)) {
                    unthrottle_cfs_rq(cfs_rq);
                }

                rq_unlock_irqrestore(rq, &rf);
            }

            rcu_read_unlock();

            return throttled;
        }
    }

    return HRTIMER_NORESTART;
}
```

## tg_set_cfs_bandwidth

![](../images/kernel/proc-sched-cfs-tg_set_cfs_bandwidth.png)

```c
struct cftype cpu_legacy_files[] = {
    {
        .name = "shares",
        .read_u64 = cpu_shares_read_u64,
        .write_u64 = cpu_shares_write_u64,
    },
    {
        .name = "idle",
        .read_s64 = cpu_idle_read_s64,
        .write_s64 = cpu_idle_write_s64,
    },
    {
        .name = "cfs_quota_us",
        .read_s64 = cpu_cfs_quota_read_s64,
        .write_s64 = cpu_cfs_quota_write_s64,
    },
    {
        .name = "cfs_period_us",
        .read_u64 = cpu_cfs_period_read_u64,
        .write_u64 = cpu_cfs_period_write_u64,
    },
    {
        .name = "cfs_burst_us",
        .read_u64 = cpu_cfs_burst_read_u64,
        .write_u64 = cpu_cfs_burst_write_u64,
    },
    {
        .name = "stat",
        .seq_show = cpu_cfs_stat_show,
    },
    {
        .name = "stat.local",
        .seq_show = cpu_cfs_local_stat_show,
    },
};

int cpu_cfs_period_write_u64(struct cgroup_subsys_state *css,
    struct cftype *cftype, u64 cfs_period_us)
{
 return tg_set_cfs_period(css_tg(css), cfs_period_us) {
        u64 quota, period, burst;

        if ((u64)cfs_period_us > U64_MAX / NSEC_PER_USEC)
            return -EINVAL;

        period = (u64)cfs_period_us * NSEC_PER_USEC;
        quota = tg->cfs_bandwidth.quota;
        burst = tg->cfs_bandwidth.burst;

        return tg_set_cfs_bandwidth(tg, period, quota, burst);
    }
}
```

```c
int tg_set_cfs_bandwidth(struct task_group *tg, u64 period, u64 quota,
                u64 burst) {
    int i, ret = 0, runtime_enabled, runtime_was_enabled;
    struct cfs_bandwidth *cfs_b = &tg->cfs_bandwidth;

    if (tg == &root_task_group)
        return -EINVAL;

    guard(cpus_read_lock)();
    guard(mutex)(&cfs_constraints_mutex);

    ret = __cfs_schedulable(tg, period, quota) {
        struct cfs_schedulable_data data = {
            .tg = tg,
            .period = period,
            .quota = quota,
        };

        if (quota != RUNTIME_INF) {
            do_div(data.period, NSEC_PER_USEC);
            do_div(data.quota, NSEC_PER_USEC);
        }

        return walk_tg_tree(tg_cfs_schedulable_down() {
            struct cfs_schedulable_data *d = data;
            struct cfs_bandwidth *cfs_b = &tg->cfs_bandwidth;
            s64 quota = 0, parent_quota = -1;

            if (!tg->parent) {
                quota = RUNTIME_INF;
            } else {
                struct cfs_bandwidth *parent_b = &tg->parent->cfs_bandwidth;

                quota = normalize_cfs_quota(tg, d);
                parent_quota = parent_b->hierarchical_quota;

                if (cgroup_subsys_on_dfl(cpu_cgrp_subsys)) {
                    if (quota == RUNTIME_INF)
                        quota = parent_quota;
                    else if (parent_quota != RUNTIME_INF)
                        quota = min(quota, parent_quota);
                } else {
                    if (quota == RUNTIME_INF)
                        quota = parent_quota;
                    else if (parent_quota != RUNTIME_INF && quota > parent_quota)
                        return -EINVAL;
                }
            }
            cfs_b->hierarchical_quota = quota;

            return 0;
        }, tg_nop, &data);
    }
    if (ret)
        return ret;

    runtime_enabled = quota != RUNTIME_INF;
    runtime_was_enabled = cfs_b->quota != RUNTIME_INF;

    if (runtime_enabled && !runtime_was_enabled)
        cfs_bandwidth_usage_inc();

    scoped_guard (raw_spinlock_irq, &cfs_b->lock) {
        cfs_b->period = ns_to_ktime(period);
        cfs_b->quota = quota;
        cfs_b->burst = burst;

        __refill_cfs_bandwidth_runtime(cfs_b);

        if (runtime_enabled) {
            start_cfs_bandwidth(cfs_b) {
                if (cfs_b->period_active)
                    return;

                cfs_b->period_active = 1;
                hrtimer_forward_now(&cfs_b->period_timer, cfs_b->period);
                hrtimer_start_expires(&cfs_b->period_timer, HRTIMER_MODE_ABS_PINNED);
            }
        }
    }

    for_each_online_cpu(i) {
        struct cfs_rq *cfs_rq = tg->cfs_rq[i];
        struct rq *rq = cfs_rq->rq;

        guard(rq_lock_irq)(rq);
        cfs_rq->runtime_enabled = runtime_enabled;
        cfs_rq->runtime_remaining = 0;

        if (cfs_rq->throttled)
            unthrottle_cfs_rq(cfs_rq);
    }

    if (runtime_was_enabled && !runtime_enabled)
        cfs_bandwidth_usage_dec();

    return 0;
}
```

## throttle_cfs_rq

![](../images/kernel/proc-sched-cfs-throttle_cfs_rq.png)

```c
bool throttle_cfs_rq(struct cfs_rq *cfs_rq)
{
    struct rq *rq = rq_of(cfs_rq);
    struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);
    struct sched_entity *se;
    long task_delta, idle_task_delta, dequeue = 1;

    raw_spin_lock(&cfs_b->lock);
    /* This will start the period timer if necessary */
    ret = __assign_cfs_rq_runtime(cfs_b, cfs_rq, 1/*target_runtime*/) {
        u64 min_amount, amount = 0;

        /* note: this is a positive sum as runtime_remaining <= 0
         * cfs_rq has target_runtime, it has remaining time (runtime_remaining),
         * so now just need to alloc (target_runtime -  runtime_remaining) */
        min_amount = target_runtime - cfs_rq->runtime_remaining;

        if (cfs_b->quota == RUNTIME_INF)
            amount = min_amount;
        else {
            start_cfs_bandwidth(cfs_b);

            if (cfs_b->runtime > 0) {
                amount = min(cfs_b->runtime, min_amount);
                cfs_b->runtime -= amount;
                cfs_b->idle = 0;
            }
        }

        cfs_rq->runtime_remaining += amount;

        return cfs_rq->runtime_remaining > 0;
    }
    if (ret) {
        dequeue = 0;
    } else {
        list_add_tail_rcu(&cfs_rq->throttled_list, &cfs_b->throttled_cfs_rq);
    }
    raw_spin_unlock(&cfs_b->lock);

    if (!dequeue)
        return false;  /* Throttle no longer required. */

    se = cfs_rq->tg->se[cpu_of(rq_of(cfs_rq))];

    /* freeze hierarchy runnable averages while throttled */
    walk_tg_tree_from(cfs_rq->tg, tg_throttle_down() {
        struct rq *rq = data;
        struct cfs_rq *cfs_rq = tg->cfs_rq[cpu_of(rq)];

        /* group is entering throttled state, stop time */
        if (!cfs_rq->throttle_count) {
            cfs_rq->throttled_clock_pelt = rq_clock_pelt(rq);
            list_del_leaf_cfs_rq(cfs_rq);

            SCHED_WARN_ON(cfs_rq->throttled_clock_self);
            if (cfs_rq->nr_running)
                cfs_rq->throttled_clock_self = rq_clock(rq);
        }
        cfs_rq->throttle_count++;

        return 0;
    }, tg_nop, (void *)rq);

    task_delta = cfs_rq->h_nr_running;
    idle_task_delta = cfs_rq->idle_h_nr_running;

    for_each_sched_entity(se) {
        struct cfs_rq *qcfs_rq = cfs_rq_of(se) {
            return se->cfs_rq;
        }
        if (!se->on_rq)
            goto done;

        dequeue_entity(qcfs_rq, se, DEQUEUE_SLEEP);

        if (cfs_rq_is_idle(group_cfs_rq(se)))
            idle_task_delta = cfs_rq->h_nr_running;

        qcfs_rq->h_nr_running -= task_delta;
        qcfs_rq->idle_h_nr_running -= idle_task_delta;

        /* parent cfs_rq has more than one se, stop dequeue parent */
        if (qcfs_rq->load.weight) {
            /* Avoid re-evaluating load for this entity: */
            se = parent_entity(se);
            break;
        }

        /* parent cfs_rq owns only one scheduling entity,
         * proceed with dequeuing towards the hierarchy upward */
    }

    for_each_sched_entity(se) {
        struct cfs_rq *qcfs_rq = cfs_rq_of(se);
        /* throttled entity or throttle-on-deactivate */
        if (!se->on_rq)
            goto done;

        update_load_avg(qcfs_rq, se, 0);
        se_update_runnable(se);

        if (cfs_rq_is_idle(group_cfs_rq(se)))
            idle_task_delta = cfs_rq->h_nr_running;

        qcfs_rq->h_nr_running -= task_delta;
        qcfs_rq->idle_h_nr_running -= idle_task_delta;
    }

    /* At this point se is NULL and we are at root level*/
    sub_nr_running(rq, task_delta);

done:

    cfs_rq->throttled = 1;
    if (cfs_rq->nr_running)
        cfs_rq->throttled_clock = rq_clock(rq);

    return true;
}
```

## unthrottle_cfs_rq

```c
void unthrottle_cfs_rq(struct cfs_rq *cfs_rq)
{
    struct rq *rq = rq_of(cfs_rq);
    struct cfs_bandwidth *cfs_b = tg_cfs_bandwidth(cfs_rq->tg);
    struct sched_entity *se;
    long task_delta, idle_task_delta;

    se = cfs_rq->tg->se[cpu_of(rq)];

    cfs_rq->throttled = 0;

    update_rq_clock(rq);

    raw_spin_lock(&cfs_b->lock);
    if (cfs_rq->throttled_clock) {
        cfs_b->throttled_time += rq_clock(rq) - cfs_rq->throttled_clock;
        cfs_rq->throttled_clock = 0;
    }
    list_del_rcu(&cfs_rq->throttled_list);
    raw_spin_unlock(&cfs_b->lock);

    /* update hierarchical throttle state */
    walk_tg_tree_from(cfs_rq->tg, tg_nop, tg_unthrottle_up() {
        struct rq *rq = data;
        struct cfs_rq *cfs_rq = tg->cfs_rq[cpu_of(rq)];

        cfs_rq->throttle_count--;
        if (!cfs_rq->throttle_count) {
            cfs_rq->throttled_clock_pelt_time +=
                rq_clock_pelt(rq) - cfs_rq->throttled_clock_pelt;

            /* Add cfs_rq with load or one or more already running entities to the list */
            if (!cfs_rq_is_decayed(cfs_rq))
                list_add_leaf_cfs_rq(cfs_rq);

            if (cfs_rq->throttled_clock_self) {
                u64 delta = rq_clock(rq) - cfs_rq->throttled_clock_self;
                cfs_rq->throttled_clock_self = 0;
                if (SCHED_WARN_ON((s64)delta < 0))
                    delta = 0;
                cfs_rq->throttled_clock_self_time += delta;
            }
        }
    }, (void *)rq);

    if (!cfs_rq->load.weight) {
        if (!cfs_rq->on_list)
            return;
        for_each_sched_entity(se) {
            if (list_add_leaf_cfs_rq(cfs_rq_of(se)))
                break;
        }
        goto unthrottle_throttle;
    }

    task_delta = cfs_rq->h_nr_running;
    idle_task_delta = cfs_rq->idle_h_nr_running;
    /* update hierarchy up on the se which is !on_rq*/
    for_each_sched_entity(se) {
        struct cfs_rq *qcfs_rq = cfs_rq_of(se);

        if (se->on_rq)
            break;
        enqueue_entity(qcfs_rq, se, ENQUEUE_WAKEUP);

        if (cfs_rq_is_idle(group_cfs_rq(se)))
            idle_task_delta = cfs_rq->h_nr_running;

        qcfs_rq->h_nr_running += task_delta;
        qcfs_rq->idle_h_nr_running += idle_task_delta;

        /* end evaluation on encountering a throttled cfs_rq */
        if (cfs_rq_throttled(qcfs_rq))
            goto unthrottle_throttle;
    }

    /* continue update hierarchy up on the se which is on_rq */
    for_each_sched_entity(se) {
        struct cfs_rq *qcfs_rq = cfs_rq_of(se);

        update_load_avg(qcfs_rq, se, UPDATE_TG);
        se_update_runnable(se);

        if (cfs_rq_is_idle(group_cfs_rq(se)))
            idle_task_delta = cfs_rq->h_nr_running;

        qcfs_rq->h_nr_running += task_delta;
        qcfs_rq->idle_h_nr_running += idle_task_delta;

        /* end evaluation on encountering a throttled cfs_rq */
        if (cfs_rq_throttled(qcfs_rq))
            goto unthrottle_throttle;
    }

    /* At this point se is NULL and we are at root level*/
    add_nr_running(rq, task_delta);

unthrottle_throttle:
    assert_list_leaf_cfs_rq(rq);

    /* Determine whether we need to wake up potentially idle CPU: */
    if (rq->curr == rq->idle && rq->cfs.nr_running)
        resched_curr(rq);
}
```

## sched_group_set_shares

![](../images/kernel/proc-sched-task-group-sched_group_set_shares.png)

```c
static struct cftype cpu_legacy_files[] = {
    {
        .name = "shares",
        .read_u64 = cpu_shares_read_u64,
        .write_u64 = cpu_shares_write_u64,
    },
    {
        .name = "idle",
        .read_s64 = cpu_idle_read_s64,
        .write_s64 = cpu_idle_write_s64,
    }
};

cpu_shares_write_u64() {
    sched_group_set_shares(css_tg(css), scale_load(shareval)) {
        __sched_group_set_shares(tg, shares) {
            tg->shares = shares;
            for_each_possible_cpu(i) {
                struct rq *rq = cpu_rq(i);
                struct sched_entity *se = tg->se[i];
                struct rq_flags rf;

                /* Propagate contribution to hierarchy */
                rq_lock_irqsave(rq, &rf);
                update_rq_clock(rq);
                for_each_sched_entity(se) {
                    update_load_avg(cfs_rq_of(se), se, UPDATE_TG);
                    update_cfs_group(se);
                }
                rq_unlock_irqrestore(rq, &rf);
            }
        }
    }
}

```

## update_cfs_group

![](../images/kernel/proc-sched-update_cfs_shares.png)

```c
/* recalc group shares based on the current state of its group runqueue */
void update_cfs_group(struct sched_entity *se) {
    struct cfs_rq *gcfs_rq = group_cfs_rq(se) {
        return grp->my_q;
    }
    long shares;

    if (!gcfs_rq) {
        return;
    }

    shares = calc_group_shares(gcfs_rq/*cfs_rq*/) {
        long tg_weight, tg_shares, load, shares;
        struct task_group *tg = cfs_rq->tg;

        tg_shares = READ_ONCE(tg->shares);
        load = max(scale_load_down(cfs_rq->load.weight), cfs_rq->avg.load_avg);
        tg_weight = atomic_long_read(&tg->load_avg);

        /* Ensure tg_weight >= load */
        tg_weight -= cfs_rq->tg_load_avg_contrib;
        tg_weight += load;

        shares = (tg_shares * load);
        if (tg_weight) {
            shares /= tg_weight;
        }

        return clamp_t(long, shares, MIN_SHARES, tg_shares);
    }

    if (unlikely(se->load.weight != shares)) {
        reweight_entity(cfs_rq_of(se)/*cfs_rq*/, se, shares/*weight*/) {
            bool curr = cfs_rq->curr == se;
            if (se->on_rq) {
                if (curr)
                    update_curr(cfs_rq);
                else
                    __dequeue_entity(cfs_rq, se);
                update_load_sub(&cfs_rq->load, se->load.weight);
            }
            dequeue_load_avg(cfs_rq, se);

            if (!se->on_rq) {
                /* old_vlag * old_weight == new_vlag * new_weight */
                se->vlag = div_s64(se->vlag * se->load.weight, weight);
            } else {
                reweight_eevdf(cfs_rq, se, weight) {
                    unsigned long old_weight = se->load.weight;
                    u64 avruntime = avg_vruntime(cfs_rq);
                    s64 vlag, vslice;

                    if (avruntime != se->vruntime) {
                        vlag = (s64)(avruntime - se->vruntime);
                        vlag = div_s64(vlag * old_weight, weight);
                        se->vruntime = avruntime - vlag;
                    }

                    vslice = (s64)(se->deadline - avruntime);
                    vslice = div_s64(vslice * old_weight, weight);
                    se->deadline = avruntime + vslice;
                }
            }

            update_load_set(&se->load, weight);

            u32 divider = get_pelt_divider(&se->avg);
            se->avg.load_avg = div_u64(se_weight(se) * se->avg.load_sum, divider);

            enqueue_load_avg(cfs_rq, se);
            if (se->on_rq) {
                update_load_add(&cfs_rq->load, se->load.weight);
                if (!curr) {
                    __enqueue_entity(cfs_rq, se);
                    update_min_vruntime(cfs_rq);
                }
            }
        }
    }
}
```

# rt_bandwidth

```c
struct rt_bandwidth {
    raw_spinlock_t  rt_runtime_lock;
    ktime_t         rt_period; /* timer interval of checking */
    u64             rt_runtime; /* remaining run time */
    struct hrtimer  rt_period_timer;
    unsigned int    rt_period_active;
};
```

## tg_set_rt_bandwidth

```c
int tg_set_rt_bandwidth(struct task_group *tg,
        u64 rt_period, u64 rt_runtime)
{
    int i, err = 0;

    if (tg == &root_task_group && rt_runtime == 0)
        return -EINVAL;

    /* No period doesn't make any sense. */
    if (rt_period == 0)
        return -EINVAL;

    if (rt_runtime != RUNTIME_INF && rt_runtime > max_rt_runtime)
        return -EINVAL;

    mutex_lock(&rt_constraints_mutex);
    err = __rt_schedulable(tg, rt_period, rt_runtime) {
        struct rt_schedulable_data data = {
            .tg = tg,
            .rt_period = period,
            .rt_runtime = runtime,
        };

        return walk_tg_tree(tg_rt_schedulable() {
            struct rt_schedulable_data *d = data;
            struct task_group *child;
            unsigned long total, sum = 0;
            u64 period, runtime;

            period = ktime_to_ns(tg->rt_bandwidth.rt_period);
            runtime = tg->rt_bandwidth.rt_runtime;

            if (tg == d->tg) {
                period = d->rt_period;
                runtime = d->rt_runtime;
            }

            if (runtime > period && runtime != RUNTIME_INF)
                return -EINVAL;

            /* Ensure we don't starve existing RT tasks if runtime turns zero. */
            if (rt_bandwidth_enabled() && !runtime &&
                tg->rt_bandwidth.rt_runtime && tg_has_rt_tasks(tg))
                return -EBUSY;

            total = to_ratio(period, runtime);

            /* Nobody can have more than the global setting allows. */
            if (total > to_ratio(global_rt_period(), global_rt_runtime()))
                return -EINVAL;

            /* The sum of our children's runtime should not exceed our own. */
            list_for_each_entry_rcu(child, &tg->children, siblings) {
                period = ktime_to_ns(child->rt_bandwidth.rt_period);
                runtime = child->rt_bandwidth.rt_runtime;

                if (child == d->tg) {
                    period = d->rt_period;
                    runtime = d->rt_runtime;
                }

                sum += to_ratio(period, runtime);
            }

            if (sum > total)
                return -EINVAL;

            return 0;
        }, tg_nop, &data);
    }
    if (err)
        goto unlock;

    raw_spin_lock_irq(&tg->rt_bandwidth.rt_runtime_lock);
    tg->rt_bandwidth.rt_period = ns_to_ktime(rt_period);
    tg->rt_bandwidth.rt_runtime = rt_runtime;

    for_each_possible_cpu(i) {
        struct rt_rq *rt_rq = tg->rt_rq[i];

        raw_spin_lock(&rt_rq->rt_runtime_lock);
        rt_rq->rt_runtime = rt_runtime;
        raw_spin_unlock(&rt_rq->rt_runtime_lock);
    }
    raw_spin_unlock_irq(&tg->rt_bandwidth.rt_runtime_lock);
unlock:
    mutex_unlock(&rt_constraints_mutex);

    return err;
}
```

## sched_rt_period_timer

```c
void init_rt_bandwidth(struct rt_bandwidth *rt_b, u64 period, u64 runtime)
{
    rt_b->rt_period = ns_to_ktime(period);
    rt_b->rt_runtime = runtime;

    raw_spin_lock_init(&rt_b->rt_runtime_lock);

    hrtimer_init(&rt_b->rt_period_timer, CLOCK_MONOTONIC,
            HRTIMER_MODE_REL_HARD);
    rt_b->rt_period_timer.function = sched_rt_period_timer;
}
```

```c
enum hrtimer_restart sched_rt_period_timer(struct hrtimer *timer)
{
    struct rt_bandwidth *rt_b =
        container_of(timer, struct rt_bandwidth, rt_period_timer);
    int idle = 0;
    int overrun;

    raw_spin_lock(&rt_b->rt_runtime_lock);
    for (;;) {
        overrun = hrtimer_forward_now(timer, rt_b->rt_period);
        if (!overrun)
            break;

        raw_spin_unlock(&rt_b->rt_runtime_lock);
        idle = do_sched_rt_period_timer(rt_b, overrun) {
            int i, idle = 1, throttled = 0;
            const struct cpumask *span;

            span = sched_rt_period_mask();
        #ifdef CONFIG_RT_GROUP_SCHED
            if (rt_b == &root_task_group.rt_bandwidth)
                span = cpu_online_mask;
        #endif

            for_each_cpu(i, span) {
                int enqueue = 0;
                struct rt_rq *rt_rq = sched_rt_period_rt_rq(rt_b, i);
                struct rq *rq = rq_of_rt_rq(rt_rq);
                struct rq_flags rf;
                int skip;

                raw_spin_lock(&rt_rq->rt_runtime_lock);
                if (!sched_feat(RT_RUNTIME_SHARE) && rt_rq->rt_runtime != RUNTIME_INF)
                    rt_rq->rt_runtime = rt_b->rt_runtime;
                skip = !rt_rq->rt_time && !rt_rq->rt_nr_running;
                raw_spin_unlock(&rt_rq->rt_runtime_lock);
                if (skip)
                    continue;

                rq_lock(rq, &rf);
                update_rq_clock(rq);

                if (rt_rq->rt_time) {
                    u64 runtime;

                    raw_spin_lock(&rt_rq->rt_runtime_lock);
                    if (rt_rq->rt_throttled) {
                        balance_runtime(rt_rq) {
                            if (!sched_feat(RT_RUNTIME_SHARE))
                                return;

                            if (rt_rq->rt_time > rt_rq->rt_runtime) {
                                raw_spin_unlock(&rt_rq->rt_runtime_lock);
                                /* borrow some from our neighbours. */
                                do_balance_runtime(rt_rq) {
                                    struct rt_bandwidth *rt_b = sched_rt_bandwidth(rt_rq);
                                    struct root_domain *rd = rq_of_rt_rq(rt_rq)->rd;
                                    int i, weight;
                                    u64 rt_period;

                                    weight = cpumask_weight(rd->span);

                                    raw_spin_lock(&rt_b->rt_runtime_lock);
                                    rt_period = ktime_to_ns(rt_b->rt_period);
                                    for_each_cpu(i, rd->span) {
                                        struct rt_rq *iter = sched_rt_period_rt_rq(rt_b, i);
                                        s64 diff;

                                        if (iter == rt_rq)
                                            continue;

                                        raw_spin_lock(&iter->rt_runtime_lock);
                                        if (iter->rt_runtime == RUNTIME_INF)
                                            goto next;

                                        /* From runqueues with spare time, take 1/n part of their
                                         * spare time, but no more than our period. */
                                        diff = iter->rt_runtime - iter->rt_time;
                                        if (diff > 0) {
                                            diff = div_u64((u64)diff, weight);
                                            if (rt_rq->rt_runtime + diff > rt_period)
                                                diff = rt_period - rt_rq->rt_runtime;
                                            iter->rt_runtime -= diff;
                                            rt_rq->rt_runtime += diff;
                                            if (rt_rq->rt_runtime == rt_period) {
                                                raw_spin_unlock(&iter->rt_runtime_lock);
                                                break;
                                            }
                                        }
                                next:
                                        raw_spin_unlock(&iter->rt_runtime_lock);
                                    }
                                    raw_spin_unlock(&rt_b->rt_runtime_lock);
                                }
                                raw_spin_lock(&rt_rq->rt_runtime_lock);
                            }
                        }
                    }
                    runtime = rt_rq->rt_runtime;
                    rt_rq->rt_time -= min(rt_rq->rt_time, overrun * runtime);
                    if (rt_rq->rt_throttled && rt_rq->rt_time < runtime) {
                        rt_rq->rt_throttled = 0;
                        enqueue = 1;

                        if (rt_rq->rt_nr_running && rq->curr == rq->idle)
                            rq_clock_cancel_skipupdate(rq);
                    }
                    if (rt_rq->rt_time || rt_rq->rt_nr_running)
                        idle = 0;
                    raw_spin_unlock(&rt_rq->rt_runtime_lock);
                } else if (rt_rq->rt_nr_running) {
                    idle = 0;
                    if (!rt_rq_throttled(rt_rq))
                        enqueue = 1;
                }
                if (rt_rq->rt_throttled)
                    throttled = 1;

                if (enqueue) {
                    /* sched_rt_rq_dequeue at task_tick_rt */
                    sched_rt_rq_enqueue(rt_rq) {
                        struct task_struct *curr = rq_of_rt_rq(rt_rq)->curr;
                        struct rq *rq = rq_of_rt_rq(rt_rq);
                        struct sched_rt_entity *rt_se;

                        int cpu = cpu_of(rq);

                        rt_se = rt_rq->tg->rt_se[cpu];

                        if (rt_rq->rt_nr_running) {
                            if (!rt_se)
                                enqueue_top_rt_rq(rt_rq);
                            else if (!on_rt_rq(rt_se))
                                enqueue_rt_entity(rt_se, 0);

                            if (rt_rq->highest_prio.curr < curr->prio)
                                resched_curr(rq);
                        }
                    }
                }
                rq_unlock(rq, &rf);
            }

            if (!throttled && (!rt_bandwidth_enabled() || rt_b->rt_runtime == RUNTIME_INF))
                return 1;

            return idle;
        }
        raw_spin_lock(&rt_b->rt_runtime_lock);
    }
    if (idle)
        rt_b->rt_period_active = 0;
    raw_spin_unlock(&rt_b->rt_runtime_lock);

    return idle ? HRTIMER_NORESTART : HRTIMER_RESTART;
}
```

# cgroup

* [LWN - Understanding the new control groups API](https://lwn.net/Articles/679786/)
* [开发内功修炼 - cgroup](https://mp.weixin.qq.com/s/rUQLM8WfjMqa__Nvhjhmxw)
* [奇小葩 - linux cgroup](https://blog.csdn.net/u012489236/category_11288796.html)
* [极客时间](https://time.geekbang.org/column/article/115582)
* [Docker 背后的内核知识 - cgroups 资源限制](https://www.infoq.cn/news/docker-kernel-knowledge-cgroups-resource-isolation)
* [Coolshell - DOCKER基础技术：LINUX CGROUP](https://coolshell.cn/articles/17049.html)
* [Docker底层原理：Cgroup V2的使用](https://blog.csdn.net/qq_67733273/article/details/134109156)

![](../images/kernel/cgroup-arch.png)

* Kernel cgroup v2 https://docs.kernel.org/admin-guide/cgroup-v2.html
* Domain cgroup:

    1. migrate: Controllers which are not in active use in the v2 hierarchy can be bound to other hierarchies.
    2. migrate: If a process is composed of multiple threads, writing the PID of any thread migrates all threads of the process.
    3. fork: When a process forks a child process, the new process is born into the cgroup that the forking process belongs to at the time of the operation
    4. destroy: A cgroup which doesn't have any children or live processes can be destroyed by removing the directory.
    5. No Internal Process Constraint

* Threaded cgroup:

    1. threads of a process can be put in different cgroups and are not subject to the no internal process constraint - threaded controllers can be enabled on non-leaf cgroups whether they have threads in them or not.
    2. As the threaded domain cgroup hosts all the domain resource consumptions of the subtree, it is considered to have internal resource consumptions whether there are processes in it or not and can't have populated child cgroups which aren't threaded.  Because the root cgroup is not subject to no internal process constraint, it can serve both as a threaded domain and a parent to domain cgroups.
    3. As the cgroup will join the parent's resource domain.  The parent  must either be a valid (threaded) domain or a threaded cgroup.
    4. A domain cgroup is turned into a threaded domain when one of its child cgroup becomes threaded or threaded controllers are enabled
    5. The threaded domain cgroup serves as the resource domain for the whole subtree, and, while the threads can be scattered across the subtree

* Memory cgroup
    1. Migrating a process to a different cgroup doesn't move the memory usages that it instantiated while in the previous cgroup to the new cgroup.

* [Oracle - Libcgroup Abstraction Layer](https://blogs.oracle.com/linux/post/libcgroup-abstraction-layer)
    * **Unified hierarchy** - All the cgroup controllers are now mounted under a single cgroup hierarchy. (In cgroup v1, each controller was typically mounted as a separate path under /sys/fs/cgroup).
    * **Leaf-node rule** - In cgroup v1, processes could be located anywhere within the cgroup hierarchy, and processes could be siblings to cgroups. In cgroup v2, processes can only exist in leaf nodes of the hierarchy, and the kernel rigorously enforces this.
    * **Single-writer rule** - To further simplify the operating system’s management of the cgroup hierarchy, the single-writer rule was adopted. Each cgroup is to be managed by a single task, and by default on Oracle Linux (and other distros) this task is systemd. Users/Applications can request to manage a subset of the cgroup hierarchy via delegated cgroups. Failure to obey the single-writer rule (by modifying a cgroup managed by systemd) could result in systemd reverting the changes.
    * **Renaming of settings** - Cgroup v1 settings and names were inconsistent across controllers. Cgroup v2 standardized the naming across the controllers.
    * **Removal of arcane features** - Early cgroup v1 development resulted in many obscure and confusing settings. Many of these settings were intentionally not forward-ported to cgroup v2 and thus do not have an equivalent v2 setting.

![](../images/kernel/cgroup-fs.png)

```c
struct task_struct {
    struct css_set      *cgroups;
    struct list_head    cg_list; /* anchored to css_set mg_tasks, dying_tasks */
};

struct css_set {
    struct cgroup_subsys_state *subsys[CGROUP_SUBSYS_COUNT];
    struct css_set *dom_cset;
    struct cgroup *dfl_cgrp;
    int nr_tasks;

    struct list_head e_cset_node[CGROUP_SUBSYS_COUNT];

    /* all threaded csets whose ->dom_cset points to this cset */
    struct list_head threaded_csets;
    struct list_head threaded_csets_node;

    struct hlist_node hlist;

    struct list_head cgrp_links;
};

struct cgroup_subsys_state {
    struct cgroup               *cgroup;
    struct cgroup_subsys        *ss;
    struct list_head            sibling;
    struct list_head            children;
    int                         id;

    unsigned int                flags;
    struct cgroup_subsys_state  *parent;
}

struct cgroup {
    /* self css with NULL ->ss, points back to this cgroup */
    struct cgroup_subsys_state self;

    unsigned long flags; /* "unsigned long" so bitops work */
    int level;

    /* Maximum allowed descent tree depth */
    int max_depth;

    struct kernfs_node *kn; /* cgroup kernfs entry */
    struct cgroup_file procs_file; /* handle for "cgroup.procs" */
    struct cgroup_file events_file; /* handle for "cgroup.events" */

    u16 subtree_control;
    u16 subtree_ss_mask;

    /* Private pointers for each registered subsystem */
    struct cgroup_subsys_state *subsys[CGROUP_SUBSYS_COUNT];

    struct cgroup_root *root;

    struct list_head cset_links;

    struct list_head e_csets[CGROUP_SUBSYS_COUNT];

    struct cgroup *dom_cgrp;
    struct cgroup *old_dom_cgrp; /* used while enabling threaded */

    /* All ancestors including self */
    struct cgroup *ancestors[];
};

/* cgroup and css_set have M:N relationship */
struct cgrp_cset_link {
    /* the cgroup and css_set this link associates */
    struct cgroup       *cgrp;
    struct css_set      *cset;

    /* list of cgrp_cset_links anchored at cgrp->cset_links */
    struct list_head    cset_link;
    /* list of cgrp_cset_links anchored at css_set->cgrp_links */
    struct list_head    cgrp_link;
};
```

```c
const struct file_operations kernfs_file_fops = {
    .read_iter  = kernfs_fop_read_iter,
    .write_iter = kernfs_fop_write_iter,
    .llseek     = kernfs_fop_llseek,
    .mmap       = kernfs_fop_mmap,
    .open       = kernfs_fop_open,
    .release    = kernfs_fop_release,
    .poll       = kernfs_fop_poll,
    .fsync      = noop_fsync,
    .splice_read    = copy_splice_read,
    .splice_write   = iter_file_splice_write,
};

static struct file_system_type cgroup2_fs_type = {
    .name               = "cgroup2",
    .init_fs_context    = cgroup_init_fs_context,
    .parameters         = cgroup2_fs_parameters,
    .kill_sb            = cgroup_kill_sb,
    .fs_flags           = FS_USERNS_MOUNT,
};

static struct kernfs_syscall_ops cgroup_kf_syscall_ops = {
    .show_options   = cgroup_show_options,
    .mkdir          = cgroup_mkdir,
    .rmdir          = cgroup_rmdir,
    .show_path      = cgroup_show_path,
};

/* cgroup v2 base files */
static struct cftype cgroup_base_files[] = {
    {
        .name = "cgroup.type",
        .flags = CFTYPE_NOT_ON_ROOT,
        .seq_show = cgroup_type_show,
        .write = cgroup_type_write,
    },
    {
        .name = "cgroup.procs",
        .flags = CFTYPE_NS_DELEGATABLE,
        .file_offset = offsetof(struct cgroup, procs_file),
        .release = cgroup_procs_release,
        .seq_start = cgroup_procs_start,
        .seq_next = cgroup_procs_next,
        .seq_show = cgroup_procs_show,
        .write = cgroup_procs_write,
    },
}

static struct kernfs_ops cgroup_kf_ops = {
    .atomic_write_len   = PAGE_SIZE,
    .open               = cgroup_file_open,
    .release            = cgroup_file_release,
    .write              = cgroup_file_write,
    .poll               = cgroup_file_poll,
    .seq_start          = cgroup_seqfile_start,
    .seq_next           = cgroup_seqfile_next,
    .seq_stop           = cgroup_seqfile_stop,
    .seq_show           = cgroup_seqfile_show,
};
```

## cgrp_demo

```c
#define _GNU_SOURCE /* See feature_test_macros | */
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/syscall.h>
#include <string.h>
#include <fcntl.h>

const int NUM_THREADS = 5;
static char* cgrp_name = "alice";

void cgrp_config(const char* name, const char* data) {
    char file[128] = {0};
    sprintf(file, "/sys/fs/cgroup/%s/%s", cgrp_name, name);
    int fd = open((char*)&file, O_WRONLY | O_APPEND);
    if (fd == -1) {
        perror("Error opening file: ");
        return;
    }

    if (write(fd, data, strlen(data)) == -1) {
        perror("Error appending text to file: ");
        close(fd);
        return;
    }

    close(fd);
}

void create_cgroup(void) {
    char cmd[128] = {0};

    sprintf(cmd, "/sys/fs/cgroup/%s", cgrp_name);
    if (mkdir(cmd, 0777) == -1) {
        perror("Directory creation failed. Trying with sudo...");

        sprintf(cmd, "sudo mkdir -p /sys/fs/cgroup/%s", cgrp_name);
        if (system(cmd) == -1) {
            perror("Error creating directory with sudo");
            exit(EXIT_FAILURE);
        }
    }

    cgrp_config("cpu.max", "50000 100000");
    // sprintf(cmd, "echo \"5M\" > /sys/fs/cgroup/%s/memory.max", cgrp_name);

    cgrp_config("memory.max", "5M");
}

void cgrp_attach(void)
{
    char tid[20] = "\n";
    char cmd[128] = {0};
    sprintf(tid + 1, "%d", (int)getpid());
    cgrp_config("cgroup.procs", tid);
}

void *main_cpucgrp(void* arg)
{
    long tid = (long)arg;
    printf("Hello World! It's me, thread #%ld, pid #%ld!\n", tid, syscall(SYS_gettid));

    int cnt = 0;
    while (1) {
        cnt++;
    }
    pthread_exit(NULL);

    return NULL;
}

void* main_memcgrp(void* arg) {
    int size = 0;
    int chunk_size = 1024 * 1024;
    void *p = NULL;

    while (1) {
        if ((p = malloc(chunk_size)) == NULL) {
            printf("out of memory!!\n");
            break;
        }
        memset(p, 1, chunk_size);
        size += chunk_size;
        printf("[%d] - memory is allocated [%8d] bytes \n", getpid(), size);
        sleep(1);
    }

    return NULL;
}

int main(int argc, char *argv[])
{
    int nr_thrd = 0;

    if (argc > 1){
        nr_thrd = atoi(argv[1]);
    }
    if (nr_thrd <= 0 || nr_thrd >= 100){
        nr_thrd = NUM_THREADS;
    }

    create_cgroup();
    cgrp_attach();

    pthread_t* thrd_vec = (pthread_t*)malloc(sizeof(pthread_t)*nr_thrd);
    for (long tid = 0; tid < nr_thrd; tid++) {
        printf("In main: creating thread %ld\n", tid);
        int rc = pthread_create(&thrd_vec[tid], NULL, main_cpucgrp, (void *)tid);
        if (rc) {
            perror("pthread_create failed: ");
            exit(-1);
        }
    }

    int pid = fork();
    if (pid == 0) {
        pthread_t mem_thrd;
        pthread_create(&mem_thrd, NULL, main_memcgrp, NULL);
    }

    /* allow other threads to continue execution */
    pthread_exit(NULL);
}
```

## cgroup_init

```c
int __init cgroup_init_early(void)
{
    static struct cgroup_fs_context __initdata ctx;
    struct cgroup_subsys *ss;
    int i;

    ctx.root = &cgrp_dfl_root;
    init_cgroup_root(&ctx);
    cgrp_dfl_root.cgrp.self.flags |= CSS_NO_REF;

    RCU_INIT_POINTER(init_task.cgroups, &init_css_set);

    for_each_subsys(ss, i) {
        ss->id = i;
        ss->name = cgroup_subsys_name[i];
        if (!ss->legacy_name)
            ss->legacy_name = cgroup_subsys_name[i];

        if (ss->early_init)
            cgroup_init_subsys(ss, true);
    }
    return 0;
}
```

```c
int __init cgroup_init(void)
{
    struct cgroup_subsys *ss;
    int ssid;

    BUILD_BUG_ON(CGROUP_SUBSYS_COUNT > 16);
    BUG_ON(cgroup_init_cftypes(NULL, cgroup_base_files));
    BUG_ON(cgroup_init_cftypes(NULL, cgroup_psi_files));
    BUG_ON(cgroup_init_cftypes(NULL, cgroup1_base_files));

    cgroup_rstat_boot();

    get_user_ns(init_cgroup_ns.user_ns);

    cgroup_lock();

    hash_add(css_set_table,
        &init_css_set.hlist,
        css_set_hash(init_css_set.subsys)
    );

    cgroup_setup_root(&cgrp_dfl_root, 0);
        --->

    cgroup_unlock();

    for_each_subsys(ss, ssid) {
        if (ss->early_init) {
            struct cgroup_subsys_state *css = init_css_set.subsys[ss->id];
            css->id = cgroup_idr_alloc(&ss->css_idr, css, 1, 2, GFP_KERNEL);
        } else {
            cgroup_init_subsys(ss, false);
        }

        list_add_tail(&init_css_set.e_cset_node[ssid],
            &cgrp_dfl_root.cgrp.e_csets[ssid]);

        if (!cgroup_ssid_enabled(ssid))
            continue;

        cgrp_dfl_root.subsys_mask |= 1 << ss->id;

        if (ss->implicit_on_dfl) {
            cgrp_dfl_implicit_ss_mask |= 1 << ss->id;
        } else if (!ss->dfl_cftypes) {
            cgrp_dfl_inhibit_ss_mask |= 1 << ss->id;
        }

        if (ss->threaded) {
            cgrp_dfl_threaded_ss_mask |= 1 << ss->id;
        }

        if (ss->dfl_cftypes == ss->legacy_cftypes) {
            WARN_ON(cgroup_add_cftypes(ss, ss->dfl_cftypes));
        } else {
            WARN_ON(cgroup_add_dfl_cftypes(ss, ss->dfl_cftypes));
            WARN_ON(cgroup_add_legacy_cftypes(ss, ss->legacy_cftypes));
        }

        if (ss->bind) {
            ss->bind(init_css_set.subsys[ssid]);
        }

        cgroup_lock();
        css_populate_dir(init_css_set.subsys[ssid]) {
            --->
        }
        cgroup_unlock();
    }

    /* init_css_set.subsys[] has been updated, re-hash */
    hash_del(&init_css_set.hlist);
    hash_add(css_set_table, &init_css_set.hlist,
        css_set_hash(init_css_set.subsys));

    sysfs_create_mount_point(fs_kobj, "cgroup");
    register_filesystem(&cgroup_fs_type);
    register_filesystem(&cgroup2_fs_type);
    proc_create_single("cgroups", 0, NULL, proc_cgroupstats_show);
    register_filesystem(&cpuset_fs_type);

    return 0;
}
```

### cgroup_init_cftypes

```c
int cgroup_init_cftypes(struct cgroup_subsys *ss, struct cftype *cfts)
{
    struct cftype *cft;
    int ret = 0;

    for (cft = cfts; cft->name[0] != '\0'; cft++) {
        struct kernfs_ops *kf_ops;

        if (cft->flags & __CFTYPE_ADDED) {
            ret = -EBUSY;
            break;
        }

        if (cft->seq_start)
            kf_ops = &cgroup_kf_ops;
        else
            kf_ops = &cgroup_kf_single_ops;

        if (cft->max_write_len && cft->max_write_len != PAGE_SIZE) {
            kf_ops = kmemdup(kf_ops, sizeof(*kf_ops), GFP_KERNEL);
            if (!kf_ops) {
                ret = -ENOMEM;
                break;
            }
            kf_ops->atomic_write_len = cft->max_write_len;
        }

        cft->kf_ops = kf_ops;
        cft->ss = ss;
        cft->flags |= __CFTYPE_ADDED;
    }

    if (ret)
        cgroup_exit_cftypes(cfts);
    return ret;
}
```

### cgroup_init_subsys

```c
void __init cgroup_init_subsys(struct cgroup_subsys *ss, bool early)
{
    struct cgroup_subsys_state *css;

    idr_init(&ss->css_idr);
    INIT_LIST_HEAD(&ss->cfts);

    /* Create the root cgroup state for this subsystem */
    ss->root = &cgrp_dfl_root;
    css = ss->css_alloc(NULL);
    /* We don't handle early failures gracefully */
    BUG_ON(IS_ERR(css));
    init_and_link_css(css, ss, &cgrp_dfl_root.cgrp);

    css->flags |= CSS_NO_REF;

    if (early) {
        css->id = 1;
    } else {
        css->id = cgroup_idr_alloc(&ss->css_idr, css, 1, 2, GFP_KERNEL);
    }

    init_css_set.subsys[ss->id] = css;

    have_fork_callback |= (bool)ss->fork << ss->id;
    have_exit_callback |= (bool)ss->exit << ss->id;
    have_release_callback |= (bool)ss->release << ss->id;
    have_canfork_callback |= (bool)ss->can_fork << ss->id;

    online_css(css);
}
```

### cgroup_setup_root

```c
int cgroup_setup_root(struct cgroup_root *root, u16 ss_mask) {
    LIST_HEAD(tmp_links);
    struct cgroup *root_cgrp = &root->cgrp;
    struct kernfs_syscall_ops *kf_sops;
    struct css_set *cset;
    int i, ret;

    lockdep_assert_held(&cgroup_mutex);

    ret = percpu_ref_init(&root_cgrp->self.refcnt, css_release,
                0, GFP_KERNEL);
    ret = allocate_cgrp_cset_links(2 * css_set_count, &tmp_links);
    ret = cgroup_init_root_id(root);

    kf_sops = (root == &cgrp_dfl_root)
        ? &cgroup_kf_syscall_ops
        : &cgroup1_kf_syscall_ops;

    root->kf_root = kernfs_create_root(
        kf_sops,
        KERNFS_ROOT_CREATE_DEACTIVATED
            | KERNFS_ROOT_SUPPORT_EXPORTOP
            | KERNFS_ROOT_SUPPORT_USER_XATTR,
        root_cgrp);
    root_cgrp->kn = kernfs_root_to_node(root->kf_root);
    root_cgrp->ancestors[0] = root_cgrp;

    ret = css_populate_dir(&root_cgrp->self);

    ret = cgroup_rstat_init(root_cgrp);
    ret = rebind_subsystems(root, ss_mask);
    ret = cgroup_bpf_inherit(root_cgrp);
    list_add(&root->root_list, &cgroup_roots);
    cgroup_root_count++;

    spin_lock_irq(&css_set_lock);
    hash_for_each(css_set_table, i, cset, hlist) {
        link_css_set(&tmp_links, cset, root_cgrp);
        if (css_set_populated(cset))
            cgroup_update_populated(root_cgrp, true);
    }
    spin_unlock_irq(&css_set_lock);

    ret = 0;
}
```

## cgroup_create

```c
cgroup_mkdir() {
    cgrp = cgroup_create(parent, name, mode) {
        struct cgroup_root *root = parent->root;
        struct cgroup *cgrp, *tcgrp;
        struct kernfs_node *kn;
        int level = parent->level + 1;
        int ret;

        /* allocate the cgroup and its ID, 0 is reserved for the root */
        cgrp = kzalloc(struct_size(cgrp, ancestors, (level + 1)), GFP_KERNEL);

        ret = percpu_ref_init(&cgrp->self.refcnt, css_release, 0, GFP_KERNEL);

        ret = cgroup_rstat_init(cgrp);

        /* create the directory */
        kn = kernfs_create_dir_ns(parent->kn, name, mode,
            current_fsuid(), current_fsgid(), cgrp, NULL
        );
        cgrp->kn = kn;

        init_cgroup_housekeeping(cgrp);

        cgrp->self.parent = &parent->self;
        cgrp->root = root;
        cgrp->level = level;

        ret = psi_cgroup_alloc(cgrp);
        ret = cgroup_bpf_inherit(cgrp);
        cgrp->freezer.e_freeze = parent->freezer.e_freeze;
        if (cgrp->freezer.e_freeze) {
            set_bit(CGRP_FREEZE, &cgrp->flags);
            set_bit(CGRP_FROZEN, &cgrp->flags);
        }

        spin_lock_irq(&css_set_lock);
        for (tcgrp = cgrp; tcgrp; tcgrp = cgroup_parent(tcgrp)) {
            cgrp->ancestors[tcgrp->level] = tcgrp;
            if (tcgrp != cgrp) {
                tcgrp->nr_descendants++;
                if (cgrp->freezer.e_freeze)
                    tcgrp->freezer.nr_frozen_descendants++;
            }
        }
        spin_unlock_irq(&css_set_lock);

        if (notify_on_release(parent))
            set_bit(CGRP_NOTIFY_ON_RELEASE, &cgrp->flags);

        if (test_bit(CGRP_CPUSET_CLONE_CHILDREN, &parent->flags))
            set_bit(CGRP_CPUSET_CLONE_CHILDREN, &cgrp->flags);

        cgrp->self.serial_nr = css_serial_nr_next++;

        /* allocation complete, commit to creation */
        list_add_tail_rcu(&cgrp->self.sibling, &cgroup_parent(cgrp)->self.children);
        atomic_inc(&root->nr_cgrps);
        cgroup_get_live(parent);

        /*
        * On the default hierarchy, a child doesn't automatically inherit
        * subtree_control from the parent.  Each is configured manually.
        */
        if (!cgroup_on_dfl(cgrp))
            cgrp->subtree_control = cgroup_control(cgrp);

        cgroup_propagate_control(cgrp);
            --->

        return cgrp;
    }

    css_populate_dir(&cgrp->self/*css*/) {
        if (css->flags & CSS_VISIBLE)
            return 0;
        /* self css with NULL ->ss, points back to this cgroup */
        if (!css->ss) {
            if (cgroup_on_dfl(cgrp)) { /* cgrp->root == &cgrp_dfl_root */
                ret = cgroup_addrm_files(css, cgrp, cgroup_base_files, true);
                if (cgroup_psi_enabled()) {
                    ret = cgroup_addrm_files(css, cgrp, cgroup_psi_files, true);
                }
            } else {
                ret = cgroup_addrm_files(css, cgrp, cgroup1_base_files, true);
            }
        } else {
            list_for_each_entry(cfts, &css->ss->cfts, node) {
                ret = cgroup_addrm_files(css, cgrp, cfts, true) {
                    struct cftype *cft, *cft_end = NULL;
                    int ret = 0;
                restart:
                    for (cft = cfts; cft != cft_end && cft->name[0] != '\0'; cft++) {
                        if (is_add) {
                            ret = cgroup_add_file(css, cgrp, cft) {
                                __kernfs_create_file();
                            }
                        } else {
                            cgroup_rm_file(cgrp, cft) {
                                kernfs_remove_by_name();
                            }
                        }
                    }
                    return ret;
                }
            }
        }

        css->flags |= CSS_VISIBLE;
    }

    cgroup_apply_control_enable(cgrp) {
        for_each_subsys(ss, ssid) {
            css = css_create(dsct, ss) {
                css = ss->css_alloc(parent_css);
                init_and_link_css(css, ss, cgrp);
                err = percpu_ref_init(&css->refcnt, css_release, 0, GFP_KERNEL);
                err = cgroup_idr_alloc(&ss->css_idr, NULL, 2, 0, GFP_KERNEL);
                css->id = err;
                list_add_tail_rcu(&css->sibling, &parent_css->children);
                cgroup_idr_replace(&ss->css_idr, css, css->id);

                err = online_css(css) {
                    if (ss->css_online) {
                        ret = ss->css_online(css);
                    }
                    if (!ret) {
                        css->flags |= CSS_ONLINE;
                        rcu_assign_pointer(css->cgroup->subsys[ss->id], css);
                        atomic_inc(&css->online_cnt);
                        if (css->parent) {
                            atomic_inc(&css->parent->online_cnt);
                        }
                    }
                }
                return css;
            }
            if (css_visible(css)) {
                ret = css_populate_dir(css);
                if (ret) {
                    return ret;
                }
            }
        }
        return 0;
    }
}
```

## cgroup_attach_task

```c
cgroup_procs_write()
    cgroup_attach_task()

write() {
    kernfs_fop_write() {
        cgroup_file_write() {
            cgroup_procs_write() {
                cgroup_attach_task() {
                    cgroup_migrate() {
                        cgroup_migrate_execute();
                    }
                }
                cgroup_procs_write_finish() {
                    for_each_subsys(ss, ssid) {
                        if (ss->post_attach) {
                            ss->post_attach();
                        }
                    }
                }
            }
        }
    }
}

int cgroup_attach_task(struct cgroup *dst_cgrp, struct task_struct *leader, bool threadgroup)
{
    DEFINE_CGROUP_MGCTX(mgctx);

/* 1. prepare src csets */
    task = leader;
    do {
        cgroup_migrate_add_src(task_css_set(task), dst_cgrp, &mgctx) {
            src_cgrp = cset_cgroup_from_root(src_cset, dst_cgrp->root);
            src_cset->mg_src_cgrp = src_cgrp;
            src_cset->mg_dst_cgrp = dst_cgrp;
            list_add_tail(&src_cset->mg_src_preload_node, &mgctx->preloaded_src_csets);
        }
        if (!threadgroup) {
            break;
        }
    } while_each_thread(leader, task);

/* 2. prepare dst csets and commit */
    ret = cgroup_migrate_prepare_dst(&mgctx) {
        struct css_set *src_cset, *tmp_cset;

        /* look up the dst cset for each src cset and link it to src */
        list_for_each_entry_safe(src_cset, tmp_cset, &mgctx->preloaded_src_csets, mg_src_preload_node) {
            struct css_set *dst_cset;
            struct cgroup_subsys *ss;
            int ssid;

            dst_cset = find_css_set(src_cset/*old_cset*/, src_cset->mg_dst_cgrp/*cgrp*/) {
                cset = find_existing_css_set(old_cset, cgrp, template) {
                    struct cgroup_root *root = cgrp->root;
                    struct cgroup_subsys *ss;
                    struct css_set *cset;
                    unsigned long key;
                    int i;

                    for_each_subsys(ss, i) {
                        if (root->subsys_mask & (1UL << i)) {
                            template[i] = cgroup_e_css_by_mask(cgrp, ss) {
                                if (!ss)
                                    return &cgrp->self;
                                while (!(cgroup_ss_mask(cgrp) & (1 << ss->id))) {
                                    cgrp = cgroup_parent(cgrp);
                                    if (!cgrp)
                                        return NULL;
                                }
                                return cgroup_css(cgrp, ss);
                            }
                        } else {
                            template[i] = old_cset->subsys[i];
                        }
                    }

                    key = css_set_hash(template);
                    hash_for_each_possible(css_set_table, cset, hlist, key) {
                        if (!compare_css_sets(cset, old_cset, cgrp, template)) {
                            continue;
                        }
                        return cset;
                    }
                    return NULL;
                }
                if (cset) {
                    get_css_set(cset);
                }
                spin_unlock_irq(&css_set_lock);

                if (cset)
                    return cset;

                cset = kzalloc(sizeof(*cset), GFP_KERNEL);
                allocate_cgrp_cset_links(cgroup_root_count, &tmp_links);
                memcpy(cset->subsys, template, sizeof(cset->subsys));

                /* Add reference counts and links from the new css_set. */
                list_for_each_entry(link, &old_cset->cgrp_links, cgrp_link) {
                    struct cgroup *c = link->cgrp;
                    if (c->root == cgrp->root) {
                        c = cgrp;
                    }
                    link_css_set(&tmp_links, cset, c) {
                        if (cgroup_on_dfl(cgrp))
                            cset->dfl_cgrp = cgrp;

                        link = list_first_entry(tmp_links, struct cgrp_cset_link, cset_link);
                        link->cset = cset;
                        link->cgrp = cgrp;

                        list_move_tail(&link->cset_link, &cgrp->cset_links);
                        list_add_tail(&link->cgrp_link, &cset->cgrp_links);

                        if (cgroup_parent(cgrp))
                            cgroup_get_live(cgrp);
                    }
                }

                css_set_count++;

                /* Add @cset to the hash table */
                key = css_set_hash(cset->subsys);
                hash_add(css_set_table, &cset->hlist, key);

                for_each_subsys(ss, ssid) {
                    struct cgroup_subsys_state *css = cset->subsys[ssid];
                    list_add_tail(&cset->e_cset_node[ssid], &css->cgroup->e_csets[ssid]);
                    css_get(css);
                }

                if (cgroup_is_threaded(cset->dfl_cgrp)) {
                    struct css_set *dcset;

                    dcset = find_css_set(cset, cset->dfl_cgrp->dom_cgrp);
                    if (!dcset) {
                        put_css_set(cset);
                        return NULL;
                    }

                    spin_lock_irq(&css_set_lock);
                    cset->dom_cset = dcset;
                    list_add_tail(&cset->threaded_csets_node, &dcset->threaded_csets);
                    spin_unlock_irq(&css_set_lock);
                }

                return cset;
            }
            if (!dst_cset) {
                return -ENOMEM;
            }

            if (src_cset == dst_cset) {
                src_cset->mg_src_cgrp = NULL;
                src_cset->mg_dst_cgrp = NULL;
                list_del_init(&src_cset->mg_src_preload_node);
                put_css_set(src_cset);
                put_css_set(dst_cset);
                continue;
            }

            src_cset->mg_dst_cset = dst_cset;

            if (list_empty(&dst_cset->mg_dst_preload_node)) {
                list_add_tail(&dst_cset->mg_dst_preload_node, &mgctx->preloaded_dst_csets);
            } else {
                put_css_set(dst_cset);
            }

            for_each_subsys(ss, ssid) {
                if (src_cset->subsys[ssid] != dst_cset->subsys[ssid]) {
                    mgctx->ss_mask |= 1 << ssid;
                }
            }
        }
    }

    if (ret) {
        goto error;
    }

/* 3. do migrate */
    ret = cgroup_migrate(leader, threadgroup, &mgctx) {
    /* 3.1 add task to mgctx */
        task = leader;
        do {
            cgroup_migrate_add_task(task, mgctx) {
                cset = task_css_set(task);
                if (!cset->mg_src_cgrp) {
                    return;
                }

                mgctx->tset.nr_tasks++;

                list_move_tail(&task->cg_list, &cset->mg_tasks);
                if (list_empty(&cset->mg_node)) {
                    list_add_tail(&cset->mg_node, &mgctx->tset.src_csets);
                }
                if (list_empty(&cset->mg_dst_cset->mg_node)) {
                    list_add_tail(&cset->mg_dst_cset->mg_node, &mgctx->tset.dst_csets);
                }
            }
            if (!threadgroup) {
                break;
            }
        } while_each_thread(leader, task);

        return cgroup_migrate_execute(mgctx) {
    /* 3.2 test can attach */
            if (tset->nr_tasks) {
                do_each_subsys_mask(ss, ssid, mgctx->ss_mask) {
                    if (ss->can_attach) {
                        tset->ssid = ssid;
                        ret = ss->can_attach(tset) {
                            cpu_cgroup_can_attach();
                            mem_cgroup_can_attach();
                        }
                        if (ret) {
                            failed_ssid = ssid;
                            goto out_cancel_attach;
                        }
                    }
                } while_each_subsys_mask();
            }

    /* 3.3 css_set_move_task */
            list_for_each_entry(cset, &tset->src_csets, mg_node) {
                list_for_each_entry_safe(task, tmp_task, &cset->mg_tasks, cg_list) {
                    struct css_set *from_cset = task_css_set(task);
                    struct css_set *to_cset = cset->mg_dst_cset;

                    get_css_set(to_cset);
                    to_cset->nr_tasks++;
                    css_set_move_task(task, from_cset, to_cset, true) {
                        if (from_cset) {
                            WARN_ON_ONCE(list_empty(&task->cg_list));

                            css_set_skip_task_iters(from_cset, task);
                            list_del_init(&task->cg_list);
                            if (!css_set_populated(from_cset)) {
                                css_set_update_populated(from_cset, false) {
                                    list_for_each_entry(link, &cset->cgrp_links, cgrp_link) {
                                        cgroup_update_populated(link->cgrp, populated);
                                    }
                                }
                            }
                        }

                        if (to_cset) {
                            cgroup_move_task(task, to_cset) {
                                rcu_assign_pointer(task->cgroups, to);
                            }
                            list_add_tail(&task->cg_list, use_mg_tasks
                                ? &to_cset->mg_tasks : &to_cset->tasks);
                        }
                    }
                    from_cset->nr_tasks--;
                }
            }

            tset->csets = &tset->dst_csets;

    /* 3.4 do attach */
            if (tset->nr_tasks) {
                do_each_subsys_mask(ss, ssid, mgctx->ss_mask) {
                    if (ss->attach) {
                        tset->ssid = ssid;
                        ss->attach(tset) {
                            cpu_cgroup_attach();
                            mem_cgroup_attach();
                        }
                    }
                } while_each_subsys_mask();
            }
        }
    }

/* 4. cleanup after attach */
    cgroup_migrate_finish(&mgctx) {
        list_for_each_entry_safe(cset, tmp_cset, &mgctx->preloaded_src_csets, mg_src_preload_node) {
            cset->mg_src_cgrp = NULL;
            cset->mg_dst_cgrp = NULL;
            cset->mg_dst_cset = NULL;
            list_del_init(&cset->mg_src_preload_node);
            put_css_set_locked(cset);
        }

        list_for_each_entry_safe(cset, tmp_cset, &mgctx->preloaded_dst_csets,
                    mg_dst_preload_node) {
            cset->mg_src_cgrp = NULL;
            cset->mg_dst_cgrp = NULL;
            cset->mg_dst_cset = NULL;
            list_del_init(&cset->mg_dst_preload_node);
            put_css_set_locked(cset);
        }
    }
}
```

## mem_cgroup

```c
struct cgroup_subsys memory_cgrp_subsys = {
    .css_alloc          = mem_cgroup_css_alloc,
    .css_online         = mem_cgroup_css_online,
    .css_offline        = mem_cgroup_css_offline,
    .css_released       = mem_cgroup_css_released,
    .css_free           = mem_cgroup_css_free,
    .css_reset          = mem_cgroup_css_reset,
    .css_rstat_flush    = mem_cgroup_css_rstat_flush,
    .can_attach         = mem_cgroup_can_attach,
    .attach             = mem_cgroup_attach,
    .cancel_attach      = mem_cgroup_cancel_attach,
    .post_attach        = mem_cgroup_move_task,
    .fork               = mem_cgroup_fork,
    .exit               = mem_cgroup_exit,
    .dfl_cftypes        = memory_files,
    .legacy_cftypes     = mem_cgroup_legacy_files,
    .early_init = 0,
};

struct cftype mem_cgroup_legacy_files[] = {
    {
        .name = "usage_in_bytes",
        .private = MEMFILE_PRIVATE(_MEM, RES_USAGE),
        .read_u64 = mem_cgroup_read_u64,
    },
    {
        .name = "kmem.limit_in_bytes",
        .private = MEMFILE_PRIVATE(_KMEM, RES_LIMIT),
        .write = mem_cgroup_write,
        .read_u64 = mem_cgroup_read_u64,
    },
    {
        .name = "kmem.usage_in_bytes",
        .private = MEMFILE_PRIVATE(_KMEM, RES_USAGE),
        .read_u64 = mem_cgroup_read_u64,
    }
    { },
};
```

### mem_cgroup_write

```c
ssize_t mem_cgroup_write(struct kernfs_open_file *of,
                char *buf, size_t nbytes, loff_t off)
{
    struct mem_cgroup *memcg = mem_cgroup_from_css(of_css(of));
    unsigned long nr_pages;
    int ret;

    buf = strstrip(buf);
    ret = page_counter_memparse(buf, "-1", &nr_pages);
    if (ret)
        return ret;

    switch (MEMFILE_ATTR(of_cft(of)->private)) {
    case RES_LIMIT:
        if (mem_cgroup_is_root(memcg)) { /* Can't set limit on root */
            ret = -EINVAL;
            break;
        }
        switch (MEMFILE_TYPE(of_cft(of)->private)) {
        case _MEM:
            ret = mem_cgroup_resize_max(memcg, nr_pages, false);
            break;
        case _MEMSWAP:
            ret = mem_cgroup_resize_max(memcg, nr_pages, true);
            break;
        case _KMEM:
            ret = 0;
            break;
        case _TCP:
            ret = memcg_update_tcp_max(memcg, nr_pages);
            break;
        }
        break;
    case RES_SOFT_LIMIT:
        if (IS_ENABLED(CONFIG_PREEMPT_RT)) {
            ret = -EOPNOTSUPP;
        } else {
            WRITE_ONCE(memcg->soft_limit, nr_pages);
            ret = 0;
        }
        break;
    }
    return ret ?: nbytes;
}
```

```c
int mem_cgroup_resize_max(struct mem_cgroup *memcg,
                unsigned long max, bool memsw)
{
    bool enlarge = false;
    bool drained = false;
    int ret;
    bool limits_invariant;
    struct page_counter *counter = memsw ? &memcg->memsw : &memcg->memory;

    do {
        if (signal_pending(current)) {
            ret = -EINTR;
            break;
        }

        mutex_lock(&memcg_max_mutex);

        limits_invariant = memsw ? max >= READ_ONCE(memcg->memory.max) :
                    max <= memcg->memsw.max;
        if (!limits_invariant) {
            mutex_unlock(&memcg_max_mutex);
            ret = -EINVAL;
            break;
        }
        if (max > counter->max)
            enlarge = true;
        ret = page_counter_set_max(counter, max) {

        }
        mutex_unlock(&memcg_max_mutex);

        if (!ret)
            break;

        if (!drained) {
            drain_all_stock(memcg);
            drained = true;
            continue;
        }

        if (!try_to_free_mem_cgroup_pages(memcg, 1, GFP_KERNEL,
                    memsw ? 0 : MEMCG_RECLAIM_MAY_SWAP)) {
            ret = -EBUSY;
            break;
        }
    } while (true);

    if (!ret && enlarge)
        memcg_oom_recover(memcg);

    return ret;
}
```

### mem_cgroup_charge

```c
mem_cgroup_charge(struct folio *folio, struct mm_struct *mm, gfp_t gfp)
{
    struct mem_cgroup *memcg;
    int ret;

    memcg = get_mem_cgroup_from_mm(mm);
    ret = charge_memcg(folio, memcg, gfp) {
        ret = try_charge(memcg, gfp, folio_nr_pages(folio)) {
            page_counter_charge(&memcg->memory, nr_pages);
        }
        if (ret)
            goto out;

        mem_cgroup_commit_charge(folio, memcg) {
            css_get(&memcg->css);
            commit_charge(folio, memcg) {
                folio->memcg_data = (unsigned long)memcg;
            }

            local_irq_disable();
            mem_cgroup_charge_statistics(memcg, folio_nr_pages(folio));
            memcg_check_events(memcg, folio_nid(folio));
            local_irq_enable();
        }
    }
    css_put(&memcg->css);

    return ret;
}
```

### mem_cgroup_can_attach

```c
int mem_cgroup_can_attach(struct cgroup_taskset *tset)
{
    struct cgroup_subsys_state *css;
    struct mem_cgroup *memcg = NULL; /* unneeded init to make gcc happy */
    struct mem_cgroup *from;
    struct task_struct *leader, *p;
    struct mm_struct *mm;
    unsigned long move_flags;
    int ret = 0;

    p = NULL;
    cgroup_taskset_for_each_leader(leader, css, tset) {
        WARN_ON_ONCE(p);
        p = leader;
        memcg = mem_cgroup_from_css(css);
    }
    if (!p)
        return 0;

    from = mem_cgroup_from_task(p);

    mm = get_task_mm(p);
    /* We move charges only when we move a owner of the mm */
    if (mm->owner == p) {
        spin_lock(&mc.lock);
        mc.mm = mm;
        mc.from = from;
        mc.to = memcg;
        mc.flags = move_flags;
        spin_unlock(&mc.lock);

        ret = mem_cgroup_precharge_mc(mm) {
            unsigned long precharge = mem_cgroup_count_precharge(mm) {
                mmap_read_lock(mm);
                walk_page_range(mm, 0, ULONG_MAX, &precharge_walk_ops, NULL) {
                    mem_cgroup_count_precharge_pte_range() {
                        for (; addr != end; pte++, addr += PAGE_SIZE) {
                            if (get_mctgt_type(vma, addr, ptep_get(pte), NULL)) {
                                mc.precharge++;
                            }
                        }
                    }
                }
                mmap_read_unlock(mm);

                precharge = mc.precharge;
                mc.precharge = 0;

                return precharge;
            }

            mc.moving_task = current;
            return mem_cgroup_do_precharge(precharge) {
                ret = try_charge(mc.to, GFP_KERNEL & ~__GFP_DIRECT_RECLAIM, count) {
                    page_counter_charge(&memcg->memory, nr_pages);
                }
                if (!ret) {
                    mc.precharge += count;
                    return ret;
                }
            }
        }
        if (ret) {
            mem_cgroup_clear_mc();
        }
    } else {
        mmput(mm);
    }
    return ret;
}
```

### mem_cgroup_post_attach

```c
static void mem_cgroup_move_task(void)
{
    if (mc.to) {
        mem_cgroup_move_charge() {
            walk_page_range(mc.mm, 0, ULONG_MAX, &charge_walk_ops, NULL) {
                mem_cgroup_move_charge_pte_range(pmd_t *pmd,
                    unsigned long addr, unsigned long end, struct mm_walk *walk) {

                    if (target_type == MC_TARGET_PAGE) {
                        page = target.page;
                        if (isolate_lru_page(page)) {
                            ret = mem_cgroup_move_account(page, true, mc.from, mc.to) {
                                folio->memcg_data = (unsigned long)to;
                            }
                            if (!ret) {
                                mc.precharge -= HPAGE_PMD_NR;
                                mc.moved_charge += HPAGE_PMD_NR;
                            }
                            putback_lru_page(page);
                        }
                        unlock_page(page);
                        put_page(page);
                    }
                }
            }
        }

        mem_cgroup_clear_mc() {
            mc.moving_task = NULL;
            __mem_cgroup_clear_mc() {
                if (mc.precharge) {
                    mem_cgroup_cancel_charge(mc.to, mc.precharge) {
                        page_counter_uncharge(&memcg->memory, nr_pages);
                    }
                    mc.precharge = 0;
                }

                if (mc.moved_charge) {
                    mem_cgroup_cancel_charge(mc.from, mc.moved_charge);
                    mc.moved_charge = 0;
                }
            }
            spin_lock(&mc.lock);
            mc.from = NULL;
            mc.to = NULL;
            mc.mm = NULL;
            spin_unlock(&mc.lock);
        }
    }
}
```

## cpu_cgroup

```c
struct cgroup_subsys cpu_cgrp_subsys = {
    .css_alloc          = cpu_cgroup_css_alloc,
    .css_online         = cpu_cgroup_css_online,
    .css_released       = cpu_cgroup_css_released,
    .css_free           = cpu_cgroup_css_free,
    .css_extra_stat_show = cpu_extra_stat_show,
    .css_local_stat_show = cpu_local_stat_show,
#ifdef CONFIG_RT_GROUP_SCHED
    .can_attach         = cpu_cgroup_can_attach,
#endif
    .attach             = cpu_cgroup_attach,
    .legacy_cftypes     = cpu_legacy_files,
    .dfl_cftypes        = cpu_files,
    .early_init         = true,
    .threaded           = true,
};

static struct cftype cpu_files[] = {
#ifdef CONFIG_FAIR_GROUP_SCHED
    {
        .name = "weight",
        .flags = CFTYPE_NOT_ON_ROOT,
        .read_u64 = cpu_weight_read_u64,
        .write_u64 = cpu_weight_write_u64,
    },
    {
        .name = "weight.nice",
        .flags = CFTYPE_NOT_ON_ROOT,
        .read_s64 = cpu_weight_nice_read_s64,
        .write_s64 = cpu_weight_nice_write_s64,
    },
    {
        .name = "idle",
        .flags = CFTYPE_NOT_ON_ROOT,
        .read_s64 = cpu_idle_read_s64,
        .write_s64 = cpu_idle_write_s64,
    },
#endif
#ifdef CONFIG_CFS_BANDWIDTH
    {
        .name = "max",
        .flags = CFTYPE_NOT_ON_ROOT,
        .seq_show = cpu_max_show,
        .write = cpu_max_write,
    },
    {
        .name = "max.burst",
        .flags = CFTYPE_NOT_ON_ROOT,
        .read_u64 = cpu_cfs_burst_read_u64,
        .write_u64 = cpu_cfs_burst_write_u64,
    },
#endif
#ifdef CONFIG_UCLAMP_TASK_GROUP
    {
        .name = "uclamp.min",
        .flags = CFTYPE_NOT_ON_ROOT,
        .seq_show = cpu_uclamp_min_show,
        .write = cpu_uclamp_min_write,
    },
    {
        .name = "uclamp.max",
        .flags = CFTYPE_NOT_ON_ROOT,
        .seq_show = cpu_uclamp_max_show,
        .write = cpu_uclamp_max_write,
    },
#endif
    { }    /* terminate */
};
```

### cpu_cgroup_css_alloc

```c
struct cgroup_subsys_state *
cpu_cgroup_css_alloc(struct cgroup_subsys_state *parent_css)
{
    struct task_group *parent = css_tg(parent_css) {
        return css ? container_of(css, struct task_group, css) : NULL;
    }
    struct task_group *tg;

    if (!parent) {
        return &root_task_group.css;
    }

    tg = sched_create_group(parent);

    return &tg->css;
}
```

### cpu_cgroup_attach

```c
void cpu_cgroup_attach(struct cgroup_taskset *tset)
{
    struct task_struct *task;
    struct cgroup_subsys_state *css;

    cgroup_taskset_for_each(task, css, tset) {
        sched_move_task(task) {
            group = sched_get_task_group(tsk);
            if (group == tsk->sched_task_group)
                return;

            update_rq_clock(rq);

            running = task_current(rq, tsk);
            queued = task_on_rq_queued(tsk);

            if (queued)
                dequeue_task(rq, tsk, queue_flags);
            if (running)
                put_prev_task(rq, tsk);

            sched_change_group(tsk, group) {
                tsk->sched_task_group = group;

                if (tsk->sched_class->task_change_group) {
                    tsk->sched_class->task_change_group(tsk) {
                        task_change_group_fair(struct task_struct *p) {
                            if (READ_ONCE(p->__state) == TASK_NEW)
                                return;

                            detach_task_cfs_rq(p);
                            p->se.avg.last_update_time = 0;
                            set_task_rq(p, task_cpu(p));
                            attach_task_cfs_rq(p);
                        }
                    }
                } else {
                    set_task_rq(tsk, task_cpu(tsk)) {
                        struct task_group *tg = task_group(p);

                        if (CONFIG_FAIR_GROUP_SCHED) {
                            set_task_rq_fair(&p->se, p->se.cfs_rq, tg->cfs_rq[cpu]) {
                                p_last_update_time = cfs_rq_last_update_time(prev);
                                n_last_update_time = cfs_rq_last_update_time(next);

                                __update_load_avg_blocked_se(p_last_update_time, se);
                                se->avg.last_update_time = n_last_update_time;
                            }
                            p->se.cfs_rq = tg->cfs_rq[cpu];
                            p->se.parent = tg->se[cpu];
                            p->se.depth = tg->se[cpu] ? tg->se[cpu]->depth + 1 : 0;
                        }

                        if (CONFIG_RT_GROUP_SCHED) {
                            p->rt.rt_rq  = tg->rt_rq[cpu];
                            p->rt.parent = tg->rt_se[cpu];
                        }
                    }
                }
            }

            if (queued)
                enqueue_task(rq, tsk, queue_flags);
            if (running) {
                set_next_task(rq, tsk);
                resched_curr(rq);
            }
        }
    }
}
```

### cpu_weight_write_u64

```c
int cpu_weight_write_u64(struct cgroup_subsys_state *css,
                struct cftype *cft, u64 weight)
{
    if (weight < CGROUP_WEIGHT_MIN || weight > CGROUP_WEIGHT_MAX)
        return -ERANGE;

    weight = DIV_ROUND_CLOSEST_ULL(weight * 1024, CGROUP_WEIGHT_DFL);

    return sched_group_set_shares(css_tg(css), scale_load(weight)) {
        if (!tg->se[0])
            return -EINVAL;

        shares = clamp(shares, scale_load(MIN_SHARES), scale_load(MAX_SHARES));

        if (tg->shares == shares)
            return 0;

        tg->shares = shares;
        for_each_possible_cpu(i) {
            struct rq *rq = cpu_rq(i);
            struct sched_entity *se = tg->se[i];
            struct rq_flags rf;

            /* Propagate contribution to hierarchy */
            rq_lock_irqsave(rq, &rf);
            update_rq_clock(rq);
            for_each_sched_entity(se) {
                update_load_avg(cfs_rq_of(se), se, UPDATE_TG);
                update_cfs_group(se);
            }
            rq_unlock_irqrestore(rq, &rf);
        }

        return 0;
    }
}
```

### cpu_max_write

```c
ssize_t cpu_max_write(struct kernfs_open_file *of,
                char *buf, size_t nbytes, loff_t off)
{
    struct task_group *tg = css_tg(of_css(of));
    u64 period = tg_get_cfs_period(tg);
    u64 burst = tg_get_cfs_burst(tg);
    u64 quota;
    int ret;

    ret = cpu_period_quota_parse(buf, &period, &quota);
    if (!ret) {
        ret = tg_set_cfs_bandwidth(tg, period, quota, burst);
    }
    return ret ?: nbytes;
}
```

## cgroup_fork

```c
cgroup_fork(p) {
    RCU_INIT_POINTER(child->cgroups, &init_css_set);
    INIT_LIST_HEAD(&child->cg_list);
}

int cgroup_can_fork(struct task_struct *child, struct kernel_clone_args *kargs)
{
    struct cgroup_subsys *ss;
    int i, j, ret;

    ret = cgroup_css_set_fork(kargs) {
        int ret;
        struct cgroup *dst_cgrp = NULL;
        struct css_set *cset;
        struct super_block *sb;
        struct file *f;

        cgroup_threadgroup_change_begin(current);

        spin_lock_irq(&css_set_lock);
        cset = task_css_set(current);
        get_css_set(cset);
        spin_unlock_irq(&css_set_lock);

        if (!(kargs->flags & CLONE_INTO_CGROUP)) {
            kargs->cset = cset;
            return 0;
        }

        f = fget_raw(kargs->cgroup);
        sb = f->f_path.dentry->d_sb;

        dst_cgrp = cgroup_get_from_file(f);

        ret = cgroup_may_write(dst_cgrp, sb);
        if (ret)
            goto err;

        ret = cgroup_attach_permissions(cset->dfl_cgrp, dst_cgrp, sb,
                        !(kargs->flags & CLONE_THREAD),
                        current->nsproxy->cgroup_ns);
        if (ret)
            goto err;

        kargs->cset = find_css_set(cset, dst_cgrp);
        if (!kargs->cset) {
            ret = -ENOMEM;
            goto err;
        }

        put_css_set(cset);
        fput(f);
        kargs->cgrp = dst_cgrp;
        return ret;
    }
    if (ret)
        return ret;

    do_each_subsys_mask(ss, i, have_canfork_callback) {
        ret = ss->can_fork(child, kargs->cset);
        if (ret)
            goto out_revert;
    } while_each_subsys_mask();

    return 0;

out_revert:
    for_each_subsys(ss, j) {
        if (j >= i)
            break;
        if (ss->cancel_fork)
            ss->cancel_fork(child, kargs->cset);
    }

    cgroup_css_set_put_fork(kargs) {
        struct cgroup *cgrp = kargs->cgrp;
        struct css_set *cset = kargs->cset;

        cgroup_threadgroup_change_end(current);

        if (cset) {
            put_css_set(cset);
            kargs->cset = NULL;
        }

        if (kargs->flags & CLONE_INTO_CGROUP) {
            cgroup_unlock();
            if (cgrp) {
                cgroup_put(cgrp);
                kargs->cgrp = NULL;
            }
        }
    }

    return ret;
}


sched_cgroup_fork

cgroup_post_fork

cgroup_cancel_fork

cgroup_free
```


## cgroup_subtree_control_write
```c
ssize_t cgroup_subtree_control_write(
    struct kernfs_open_file *of,
    char *buf, size_t nbytes, loff_t off)
{
    u16 enable = 0, disable = 0;
    struct cgroup *cgrp, *child;
    struct cgroup_subsys *ss;
    char *tok;
    int ssid, ret;

    buf = strstrip(buf);
    while ((tok = strsep(&buf, " "))) {
        if (tok[0] == '\0')
            continue;
        do_each_subsys_mask(ss, ssid, ~cgrp_dfl_inhibit_ss_mask) {
            if (!cgroup_ssid_enabled(ssid) ||
                strcmp(tok + 1, ss->name))
                continue;

            if (*tok == '+') {
                enable |= 1 << ssid;
                disable &= ~(1 << ssid);
            } else if (*tok == '-') {
                disable |= 1 << ssid;
                enable &= ~(1 << ssid);
            } else {
                return -EINVAL;
            }
            break;
        } while_each_subsys_mask();
        if (ssid == CGROUP_SUBSYS_COUNT)
            return -EINVAL;
    }

    cgrp = cgroup_kn_lock_live(of->kn, true);
    if (!cgrp)
        return -ENODEV;

    for_each_subsys(ss, ssid) {
        if (enable & (1 << ssid)) {
            if (cgrp->subtree_control & (1 << ssid)) {
                enable &= ~(1 << ssid);
                continue;
            }

            if (!(cgroup_control(cgrp) & (1 << ssid))) {
                ret = -ENOENT;
                goto out_unlock;
            }
        } else if (disable & (1 << ssid)) {
            if (!(cgrp->subtree_control & (1 << ssid))) {
                disable &= ~(1 << ssid);
                continue;
            }

            /* a child has it enabled? */
            cgroup_for_each_live_child(child, cgrp) {
                if (child->subtree_control & (1 << ssid)) {
                    ret = -EBUSY;
                    goto out_unlock;
                }
            }
        }
    }

    if (!enable && !disable) {
        ret = 0;
        goto out_unlock;
    }

    ret = cgroup_vet_subtree_control_enable(cgrp, enable);
    if (ret)
        goto out_unlock;

    /* save and update control masks and prepare csses */
    cgroup_save_control(cgrp);

    cgrp->subtree_control |= enable;
    cgrp->subtree_control &= ~disable;

    ret = cgroup_apply_control(cgrp) {
        int ret;
        cgroup_propagate_control(cgrp) {
            struct cgroup *dsct;
            struct cgroup_subsys_state *d_css;

            cgroup_for_each_live_descendant_pre(dsct, d_css, cgrp) {
                dsct->subtree_control &= cgroup_control(dsct) {
                    struct cgroup *parent = cgroup_parent(cgrp);
                    u16 root_ss_mask = cgrp->root->subsys_mask;

                    if (parent) {
                        u16 ss_mask = parent->subtree_control;

                        /* threaded cgroups can only have threaded controllers */
                        if (cgroup_is_threaded(cgrp))
                            ss_mask &= cgrp_dfl_threaded_ss_mask;
                        return ss_mask;
                    }

                    if (cgroup_on_dfl(cgrp))
                        root_ss_mask &= ~(cgrp_dfl_inhibit_ss_mask | cgrp_dfl_implicit_ss_mask);
                    return root_ss_mask;
                }
                dsct->subtree_ss_mask =
                    cgroup_calc_subtree_ss_mask(dsct->subtree_control, cgroup_ss_mask(dsct)/*this_ss_mask*/) {
                        u16 cur_ss_mask = subtree_control;
                        struct cgroup_subsys *ss;
                        int ssid;

                        cur_ss_mask |= cgrp_dfl_implicit_ss_mask;

                        while (true) {
                            u16 new_ss_mask = cur_ss_mask;

                            do_each_subsys_mask(ss, ssid, cur_ss_mask) {
                                new_ss_mask |= ss->depends_on;
                            } while_each_subsys_mask();

                            new_ss_mask &= this_ss_mask;

                            if (new_ss_mask == cur_ss_mask)
                                break;
                            cur_ss_mask = new_ss_mask;
                        }

                        return cur_ss_mask;
                    }
            }
        }

        ret = cgroup_apply_control_enable(cgrp);
        if (ret)
            return ret;

        /*
        * At this point, cgroup_e_css_by_mask() results reflect the new csses
        * making the following cgroup_update_dfl_csses() properly update
        * css associations of all tasks in the subtree.
        */
        return cgroup_update_dfl_csses(cgrp) {

        }
    }
    cgroup_finalize_control(cgrp, ret);
    if (ret)
        goto out_unlock;

    kernfs_activate(cgrp->kn);
out_unlock:
    cgroup_kn_unlock(of->kn);
    return ret ?: nbytes;
}
```

## cgroup_get_tree

```c
mount(dev_name, dir_name, type, flags, data) {
    copy_mount_string(); /* type, dev_name, data */
    do_mount() {
        struct path path;
        user_path_at(&path);
        path_mount(&path) {
            do_new_mount() {
                struct file_system_type *type = get_fs_type(fstype);
                struct fs_context *fc = fs_context_for_mount(type, sb_flags);
                vfs_parse_fs_string();

                /* Get the mountable root */
                vfs_get_tree(fc) {
                    fc->ops->get_tree(fc); /* cgroup_get_tree */
                    struct super_block *sb = fc->root->d_sb;
                }

                do_new_mount_fc(fc, path, mnt_flags);
            }
        }
    }
}
```

```c
int cgroup_get_tree(struct fs_context *fc)
{
    struct cgroup_fs_context *ctx = cgroup_fc2context(fc);
    int ret;

    WRITE_ONCE(cgrp_dfl_visible, true);
    cgroup_get_live(&cgrp_dfl_root.cgrp);
    ctx->root = &cgrp_dfl_root;

    ret = cgroup_do_get_tree(fc) {
        struct cgroup_fs_context *ctx = cgroup_fc2context(fc);
        int ret;

        ctx->kfc.root = ctx->root->kf_root;
        if (fc->fs_type == &cgroup2_fs_type)
            ctx->kfc.magic = CGROUP2_SUPER_MAGIC;
        else
            ctx->kfc.magic = CGROUP_SUPER_MAGIC;

        ret = kernfs_get_tree(fc) {
            struct kernfs_fs_context *kfc = fc->fs_private;
            struct super_block *sb;
            struct kernfs_super_info *info;
            int error;

            info = kzalloc(sizeof(*info), GFP_KERNEL);

            info->root = kfc->root;
            info->ns = kfc->ns_tag;
            INIT_LIST_HEAD(&info->node);

            fc->s_fs_info = info;
            sb = sget_fc(fc, kernfs_test_super, kernfs_set_super);

            if (!sb->s_root) {
                struct kernfs_super_info *info = kernfs_info(sb) {
                    return sb->s_fs_info;
                }
                struct kernfs_root *root = kfc->root;

                kfc->new_sb_created = true;

                error = kernfs_fill_super(sb, kfc) {
                    struct kernfs_super_info *info = kernfs_info(sb);
                    struct kernfs_root *kf_root = kfc->root;
                    struct inode *inode;
                    struct dentry *root;

                    info->sb = sb;
                    /* Userspace would break if executables or devices appear on sysfs */
                    sb->s_iflags |= SB_I_NOEXEC | SB_I_NODEV;
                    sb->s_blocksize = PAGE_SIZE;
                    sb->s_blocksize_bits = PAGE_SHIFT;
                    sb->s_magic = kfc->magic;
                    sb->s_op = &kernfs_sops;
                    sb->s_xattr = kernfs_xattr_handlers;
                    if (info->root->flags & KERNFS_ROOT_SUPPORT_EXPORTOP)
                        sb->s_export_op = &kernfs_export_ops;
                    sb->s_time_gran = 1;

                    /* sysfs dentries and inodes don't require IO to create */
                    sb->s_shrink->seeks = 0;

                    /* get root inode, initialize and unlock it */
                    inode = kernfs_get_inode(sb, info->root->kn) {
                        inode = iget_locked(sb, kernfs_ino(kn)) {
                            struct hlist_head *head = inode_hashtable + hash(sb, ino);
                            struct inode *inode;
                        again:
                            inode = find_inode_fast(sb, head, ino) ?: alloc_inode(sb);
                            return inode;
                        }
                        if (inode && (inode->i_state & I_NEW)) {
                            kernfs_init_inode(kn, inode) {
                                kernfs_get(kn);
                                inode->i_private = kn;
                                inode->i_mapping->a_ops = &ram_aops;
                                inode->i_op = &kernfs_iops;
                                inode->i_generation = kernfs_gen(kn);

                                /* initialize inode according to type */
                                switch (kernfs_type(kn)) {
                                case KERNFS_DIR:
                                    inode->i_op = &kernfs_dir_iops;
                                    inode->i_fop = &kernfs_dir_fops;
                                    if (kn->flags & KERNFS_EMPTY_DIR)
                                        make_empty_dir_inode(inode);
                                    break;
                                case KERNFS_FILE:
                                    inode->i_size = kn->attr.size;
                                    inode->i_fop = &kernfs_file_fops;
                                    break;
                                case KERNFS_LINK:
                                    inode->i_op = &kernfs_symlink_iops;
                                    break;
                                }
                            }
                        }

                        return inode;
                    }

                    /* instantiate and link root dentry */
                    root = d_make_root(inode) {
                        struct dentry *res = NULL;
                        if (root_inode) {
                            res = d_alloc_anon(root_inode->i_sb);
                            if (res)
                                d_instantiate(res, root_inode);
                            else
                                iput(root_inode);
                        }
                        return res;
                    }
                    sb->s_root = root;
                    sb->s_d_op = &kernfs_dops;
                    return 0;
                }
                sb->s_flags |= SB_ACTIVE;

                uuid_t uuid;
                uuid_gen(&uuid);
                super_set_uuid(sb, uuid.b, sizeof(uuid));

                list_add(&info->node, &info->root->supers);
            }

            fc->root = dget(sb->s_root);
            return 0;
        }

        if (!ret && ctx->ns != &init_cgroup_ns) {
            struct dentry *nsdentry;
            struct super_block *sb = fc->root->d_sb;
            struct cgroup *cgrp;

            cgroup_lock();
            spin_lock_irq(&css_set_lock);

            cgrp = cset_cgroup_from_root(ctx->ns->root_cset/*cset*/, ctx->root/*root*/) {
                struct cgroup *res_cgroup = NULL;

                if (cset == &init_css_set) {
                    res_cgroup = &root->cgrp;
                } else if (root == &cgrp_dfl_root) {
                    res_cgroup = cset->dfl_cgrp;
                } else {
                    struct cgrp_cset_link *link;
                    list_for_each_entry(link, &cset->cgrp_links, cgrp_link) {
                        struct cgroup *c = link->cgrp;
                        if (c->root == root) {
                            res_cgroup = c;
                            break;
                        }
                    }
                }
                return res_cgroup;
            }

            nsdentry = kernfs_node_dentry(cgrp->kn, sb);
            fc->root = nsdentry;
        }

        if (!ctx->kfc.new_sb_created)
            cgroup_put(&ctx->root->cgrp);

        return ret;
    }
    if (!ret) {
        apply_cgroup_root_flags(ctx->flags);
    }
    return ret;
}
```

# namespace

* [LWN - :one: namespaces overview](https://lwn.net/Articles/531114/)
    * [:two: the namespaces API](https://lwn.net/Articles/531381/)
    * [:three: PID namespaces](https://lwn.net/Articles/531419/)
    * [:four: more on PID namespaces](https://lwn.net/Articles/532748/)
    * [:five: user namespaces](https://lwn.net/Articles/532593/)
    * [:six: more on user namespaces](https://lwn.net/Articles/540087/)
    * [:seven: network namespaces](https://lwn.net/Articles/580893/)
    * [Mount namespaces and shared subtrees](https://lwn.net/Articles/689856/)
    * [Mount namespaces, mount propagation, and unbindable mounts](https://lwn.net/Articles/690679/)
* [Coolshell - DOCKER基础技术：LINUX NAMESPACE - :one:](https://coolshell.cn/articles/17010.html)      [:two:](https://coolshell.cn/articles/17029.html)
* [Linux - Namespace](https://blog.csdn.net/summer_fish/article/details/134437688)
* [Pid Namespace 原理与源码分析](https://zhuanlan.zhihu.com/p/335171876)
* [Docker 背后的内核知识 - Namespace 资源隔离](https://www.infoq.cn/article/docker-kernel-knowledge-namespace-resource-isolation/)
* [Linux NameSpace 目录](https://blog.csdn.net/pwl999/article/details/117554060?spm=1001.2014.3001.5501)

![](../images/kernel/ns-arch.png)

Namespace | Flag |Page |Isolates
--- | --- | --- | ---
Cgroup | CLONE_NEWCGROUP | cgroup_namespaces | Cgroup root directory
IPC | CLONE_NEWIPC | ipc_namespaces | System V IPC, POSIX message queues
Network | CLONE_NEWNET | network_namespaces | Network devices, stacks, ports, etc.
Mount | CLONE_NEWNS | mount_namespaces | Mount points
PID | CLONE_NEWPID | pid_namespaces | Process IDs
Time | CLONE_NEWTIME | time_namespaces | Boot and monotonic clocks
User | CLONE_NEWUSER | user_namespaces | User and group IDs
UTS | CLONE_NEWUTS | uts_namespaces | Hostname and NIS domain name

* Namespace lifetime
    * Absent any other factors, a namespace is automatically torn down when the last process in the namespace terminates or leaves the namespace.
    * there are a number of other factors that may pin a namespace into existence even though it has no member processes.

```c
struct task_struct {
    struct nsproxy *nsproxy;
}

struct nsproxy {
    refcount_t count;
    struct uts_namespace    *uts_ns;
    struct ipc_namespace    *ipc_ns;
    struct mnt_namespace    *mnt_ns;
    struct pid_namespace    *pid_ns_for_children;
    struct net              *net_ns;
    struct time_namespace   *time_ns;
    struct time_namespace   *time_ns_for_children;
    struct cgroup_namespace *cgroup_ns;
};
```

## setns

* The setns system call allows the calling process to join an existing namespace.  The namespace to join is specified via a file descriptor that refers to one of the `/proc/pid/ns` files described below.
* PID Namespace
    * When setns/unshare, the caller creats a new namespace for its chilren, but the caller itself doesn't move into the new namespace.
    * The 1st process with pid 1 created by caller is init process in new namespace
    * This is beacause a process pid cant change during its lifetime

```c
SYSCALL_DEFINE2(setns, int, fd, int, flags)
{
    /* fd of namespace file */
    struct fd f = fdget(fd);
    struct ns_common *ns = NULL;
    struct nsset nsset = {};
    int err = 0;

    if (!f.file)
        return -EBADF;

    if (proc_ns_file(f.file)) {
        ns = get_proc_ns(file_inode(f.file));
        if (flags && (ns->ops->type != flags)) {
            err = -EINVAL;
        }
        flags = ns->ops->type;
    } else if (!IS_ERR(pidfd_pid(f.file))) {
        err = check_setns_flags(flags);
    } else {
        err = -EINVAL;
    }
    if (err)
        goto out;

    err = prepare_nsset(flags, &nsset) {
        struct task_struct *me = current;

        /* flags arg is 0, copy current ns, dont create a new one */
        nsset->nsproxy = create_new_namespaces(0, me, current_user_ns(), me->fs) {
            struct nsproxy *new_nsp;
            int err;

            new_nsp = create_nsproxy() {
                return kmem_cache_alloc(nsproxy_cachep, GFP_KERNEL);
            }

            new_nsp->mnt_ns = copy_mnt_ns(flags, tsk->nsproxy->mnt_ns, user_ns, new_fs);
            new_nsp->uts_ns = copy_utsname(flags, user_ns, tsk->nsproxy->uts_ns);
            new_nsp->ipc_ns = copy_ipcs(flags, user_ns, tsk->nsproxy->ipc_ns);

            new_nsp->pid_ns_for_children =
                copy_pid_ns(flags, user_ns, tsk->nsproxy->pid_ns_for_children);

            new_nsp->cgroup_ns = copy_cgroup_ns(flags, user_ns,
                                tsk->nsproxy->cgroup_ns);

            new_nsp->net_ns = copy_net_ns(flags, user_ns, tsk->nsproxy->net_ns);

            new_nsp->time_ns_for_children = copy_time_ns(flags, user_ns,
                tsk->nsproxy->time_ns_for_children);

            new_nsp->time_ns = get_time_ns(tsk->nsproxy->time_ns);

            return new_nsp;
        }
        if (IS_ERR(nsset->nsproxy))
            return PTR_ERR(nsset->nsproxy);

        if (flags & CLONE_NEWUSER)
            nsset->cred = prepare_creds();
        else
            nsset->cred = current_cred();
        if (!nsset->cred)
            goto out;

        /* Only create a temporary copy of fs_struct if we really need to. */
        if (flags == CLONE_NEWNS) {
            nsset->fs = me->fs;
        } else if (flags & CLONE_NEWNS) {
            nsset->fs = copy_fs_struct(me->fs);
            if (!nsset->fs)
                goto out;
        }

        nsset->flags = flags;
        return 0;
    }
    if (err)
        goto out;

    if (proc_ns_file(f.file)) {
        err = validate_ns(&nsset, ns) {
            return ns->ops->install(nsset, ns) {
                pidns_install() {
                    struct nsproxy *nsproxy = nsset->nsproxy;
                    struct pid_namespace *active = task_active_pid_ns(current);
                    struct pid_namespace *ancestor, *new = to_pid_ns(ns);

                    ancestor = new;
                    while (ancestor->level > active->level) {
                        ancestor = ancestor->parent;
                    }
                    if (ancestor != active) {
                        return -EINVAL;
                    }

                    put_pid_ns(nsproxy->pid_ns_for_children);
                    nsproxy->pid_ns_for_children = get_pid_ns(new) {
                        if (ns != &init_pid_ns)
                            refcount_inc(&ns->ns.count);
                        return ns;
                    }
                }

                timens_install();

                ipcns_install() {
                    struct nsproxy *nsproxy = nsset->nsproxy;
                    struct ipc_namespace *ns = to_ipc_ns(new);
                    put_ipc_ns(nsproxy->ipc_ns);
                    nsproxy->ipc_ns = get_ipc_ns(ns);
                }

                mntns_install() {
                    struct nsproxy *nsproxy = nsset->nsproxy;
                    struct fs_struct *fs = nsset->fs;
                    struct mnt_namespace *mnt_ns = to_mnt_ns(ns), *old_mnt_ns;
                    struct user_namespace *user_ns = nsset->cred->user_ns;
                    struct path root;
                    int err;

                    if (fs->users != 1)
                        return -EINVAL;

                    get_mnt_ns(mnt_ns);
                    old_mnt_ns = nsproxy->mnt_ns;
                    nsproxy->mnt_ns = mnt_ns;

                    /* Find the root */
                    err = vfs_path_lookup(mnt_ns->root->mnt.mnt_root,
                        &mnt_ns->root->mnt,
                        "/", LOOKUP_DOWN, &root
                    );

                    /* Update the pwd and root */
                    set_fs_pwd(fs, &root) {
                        fs->pwd = *path;
                    }
                    set_fs_root(fs, &root) {
                        fs->root = *path;
                    }
                }

                cgroupns_install() {
                    struct nsproxy *nsproxy = nsset->nsproxy;
                    struct cgroup_namespace *cgroup_ns = to_cg_ns(ns);

                    /* Don't need to do anything if we are attaching to our own cgroupns. */
                    if (cgroup_ns == nsproxy->cgroup_ns)
                        return 0;

                    get_cgroup_ns(cgroup_ns);
                    put_cgroup_ns(nsproxy->cgroup_ns);
                    nsproxy->cgroup_ns = cgroup_ns;
                }

                netns_install() {
                    struct nsproxy *nsproxy = nsset->nsproxy;
                    struct net *net = to_net_ns(ns);
                    put_net(nsproxy->net_ns);
                    nsproxy->net_ns = get_net(net);
                }
            }
        }
    } else {
        err = validate_nsset(&nsset, pidfd_pid(f.file));
    }
    if (!err) {
        commit_nsset(&nsset) {
            unsigned flags = nsset->flags;
            struct task_struct *me = current;

        #ifdef CONFIG_USER_NS
            if (flags & CLONE_NEWUSER) {
                /* transfer ownership */
                commit_creds(nsset_cred(nsset));
                nsset->cred = NULL;
            }
        #endif

            /* We only need to commit if we have used a temporary fs_struct. */
            if ((flags & CLONE_NEWNS) && (flags & ~CLONE_NEWNS)) {
                set_fs_root(me->fs, &nsset->fs->root);
                set_fs_pwd(me->fs, &nsset->fs->pwd);
            }

        #ifdef CONFIG_IPC_NS
            if (flags & CLONE_NEWIPC)
                exit_sem(me);
        #endif

        #ifdef CONFIG_TIME_NS
            if (flags & CLONE_NEWTIME)
                timens_commit(me, nsset->nsproxy->time_ns);
        #endif

            /* transfer ownership */
            switch_task_namespaces(me, nsset->nsproxy) {
                p->nsproxy = new;
            }
            nsset->nsproxy = NULL;
        }
        perf_event_namespaces(current);
    }
    put_nsset(&nsset);
out:
    fdput(f);
    return err;
}
```

## unshare

* The unshare system call moves the calling process to a new namespace.  If the flags argument of the call specifies one or more of the CLONE_NEW* flags listed above, then new namespaces are created for each flag, and the calling process is made a member of those namespaces.

```c
SYSCALL_DEFINE1(unshare, unsigned long, unshare_flags)
{
    return ksys_unshare(unshare_flags) {
        struct fs_struct *fs, *new_fs = NULL;
        struct files_struct *new_fd = NULL;
        struct cred *new_cred = NULL;
        struct nsproxy *new_nsproxy = NULL;
        int do_sysvsem = 0;
        int err;

        if (unshare_flags & (CLONE_NEWIPC|CLONE_SYSVSEM))
            do_sysvsem = 1;
        err = unshare_fs(unshare_flags, &new_fs);

        err = unshare_fd(unshare_flags, NR_OPEN_MAX, &new_fd);

        err = unshare_userns(unshare_flags, &new_cred);

        err = unshare_nsproxy_namespaces(
            unshare_flags, &new_nsproxy, new_cred, new_fs) {

            *new_nsp = create_new_namespaces(unshare_flags, current, user_ns,
                new_fs ? new_fs : current->fs);
        }

        if (new_fs || new_fd || do_sysvsem || new_cred || new_nsproxy) {
            if (do_sysvsem) {
                /* CLONE_SYSVSEM is equivalent to sys_exit(). */
                exit_sem(current);
            }
            if (unshare_flags & CLONE_NEWIPC) {
                /* Orphan segments in old ns (see sem above). */
                exit_shm(current);
                shm_init_task(current);
            }

            if (new_nsproxy)
                switch_task_namespaces(current, new_nsproxy);

            task_lock(current);

            if (new_fs) {
                fs = current->fs;
                spin_lock(&fs->lock);
                current->fs = new_fs;
                if (--fs->users)
                    new_fs = NULL;
                else
                    new_fs = fs;
                spin_unlock(&fs->lock);
            }

            if (new_fd)
                swap(current->files, new_fd);

            task_unlock(current);

            if (new_cred) {
                /* Install the new user namespace */
                commit_creds(new_cred);
                new_cred = NULL;
            }
        }

        return err;
    }
}
```

## clone

```c
clone() {
    kernel_clone() {
        copy_process() {
            copy_namespaces() {
                new_ns = create_new_namespaces(flags, tsk, user_ns, tsk->fs);
                tsk->nsproxy = new_ns;
            }
        }
    }
}
```

## pid_namespace

![](../images/kernel/ns-pid.png)

* [wowotech - Linux系统如何标识进程？](http://www.wowotech.net/process_management/pid.html)
* [Linux 内核进程管理之进程ID](https://www.cnblogs.com/hazir/p/linux_kernel_pid.html)

* the 1st process with pid 1 is init process in each namespace and has sepcial privilege
* a process in one namespace cant affect processes in parent or sibling namespaces
* If mounted, the /proc filesystem for a new PID namespace only shows processes belonging to the same namespace
* root namespace can show all processes
* Since the init process is essential to the functioning of the PID namespace, if the init process is terminated by SIGKILL (or it terminates for any other reason), the kernel terminates all other processes in the namespace by sending them a SIGKILL signal. [From: Namespaces in operation, part 4: more on PID namespaces](https://lwn.net/Articles/532748/)
    * Normally, a PID namespace will also be destroyed when its init process terminates. However, there is an unusual corner case: the namespace won't be destroyed as long as a /proc/PID/ns/pid file for one of the processes in that namespaces is bind mounted or held open. However, it is not possible to create new processes in the namespace (via setns() plus fork()): the lack of an init process is detected during the fork() call, which fails with an ENOMEM error
    * Specifying the CLONE_NEWPID flag in a call to setns/unshare() creates a new PID namespace, but does not place the caller in the new namespace. Rather, any children created by the caller will be placed in the new namespace; the first such child will become the init process for the namespace.

```c
struct task_struct {
    pid_t               pid;
    pid_t               tgid;

    /* PID/PID hash table linkage. */
    struct pid          *thread_pid; /* pid owned by the tsk */
    struct hlist_node   pid_links[PIDTYPE_MAX]; /* pids the tsk belongs to */
    struct list_head    thread_node;

    /* task group leader */
    struct task_struct  *group_leader;
};

enum pid_type {
    PIDTYPE_PID,
    PIDTYPE_TGID,
    PIDTYPE_PGID,
    PIDTYPE_SID,
    PIDTYPE_MAX,
};

struct pid
{
    refcount_t      count;
    unsigned int    level;
    spinlock_t      lock;
    struct dentry   *stashed;
    u64             ino;

    /* lists of tasks that use this pid */
    struct hlist_head tasks[PIDTYPE_MAX];
    struct hlist_head inodes;

    /* wait queue for pidfd notifications */
    wait_queue_head_t wait_pidfd;
    struct rcu_head rcu;

    /* pid nr in each ns hierarchy level */
    struct upid numbers[];
};

struct upid {
    int nr; /* pid nr in current level */
    struct pid_namespace *ns; /* current level's ns */
};

struct pid_namespace {
    struct idr              idr;
    struct rcu_head         rcu;
    unsigned int            pid_allocated;
    struct task_struct      *child_reaper;
    struct kmem_cache       *pid_cachep;
    unsigned int            level;
    struct pid_namespace    *parent;

    struct user_namespace   *user_ns;
    struct ucounts          *ucounts;
    int reboot;
    struct ns_common        ns;
}

struct ns_common {
    struct dentry *stashed;
    const struct proc_ns_operations *ops;
    unsigned int inum;
    refcount_t count;
};
```

### getpid_xxx

```c
/* the helpers to get the pid's id seen from different namespaces
 *
 * pid_nr()    : global id, i.e. the id seen from the init namespace;
 * pid_vnr()   : virtual id, i.e. the id seen from the pid namespace of
 *               current.
 * pid_nr_ns() : id seen from the ns specified.
 *
 * see also task_xid_nr() etc in include/linux/sched.h */

static inline pid_t pid_nr(struct pid *pid)
{
    pid_t nr = 0;
    if (pid)
        nr = pid->numbers[0].nr;
    return nr;
}

pid_t pid_vnr(struct pid *pid)
{
    ns = task_active_pid_ns(current) {
        pid = task_pid(tsk) {
            return task->thread_pid;
        }
        return ns_of_pid(pid) {
            struct pid_namespace *ns = NULL;
            if (pid)
                ns = pid->numbers[pid->level].ns;
            return ns;
        };
    }
    return pid_nr_ns(pid, ns) {
        struct upid *upid;
        pid_t nr = 0;

        if (pid && ns->level <= pid->level) {
            upid = &pid->numbers[ns->level];
            if (upid->ns == ns)
                nr = upid->nr;
        }
        return nr;
    }
}

struct pid *get_task_pid(struct task_struct *task, enum pid_type type)
{
    struct pid *pid;
    pid_ptr = (type == PIDTYPE_PID)
        ? &task->thread_pid : &task->signal->pids[type];
    pid = get_pid(rcu_dereference(*pid_ptr)) {
        if (pid)
            refcount_inc(&pid->count);
        return pid;
    }
    return pid;
}

struct pid *find_pid_ns(int nr, struct pid_namespace *ns)
{
    return idr_find(&ns->idr, nr);
}

pid_t task_pid_nr_ns(struct task_struct *tsk, struct pid_namespace *ns);
pid_t task_tgid_nr_ns(struct task_struct *tsk, struct pid_namespace *ns);
pid_t task_pigd_nr_ns(struct task_struct *tsk, struct pid_namespace *ns);
pid_t task_session_nr_ns(struct task_struct *tsk, struct pid_namespace *ns);

struct task_struct *find_task_by_pid_ns(pid_t nr, struct pid_namespace *ns);
struct task_struct *find_task_by_vpid(pid_t vnr);
struct task_struct *find_task_by_pid(pid_t vnr);
```

```c
SYSCALL_DEFINE0(getpid)
{
    return task_tgid_vnr(current) {
        return __task_pid_nr_ns(tsk, PIDTYPE_TGID, NULL/*ns*/) {
            pid_t nr = 0;

            rcu_read_lock();
            if (!ns) {
                ns = task_active_pid_ns(current);
            }
            nr = pid_nr_ns(*task_pid_ptr(task, type), ns);
            rcu_read_unlock();

            return nr;
        }
    }
}

SYSCALL_DEFINE1(getpgid, pid_t, pid)
{
    return do_getpgid(pid) {
        struct task_struct *p;
        struct pid *grp;
        int retval;

        rcu_read_lock();
        if (!pid) {
            grp = task_pgrp(current) {
                return task->signal->pids[PIDTYPE_PGID];
            }
        } else {
            retval = -ESRCH;
            p = find_task_by_vpid(pid) {
                return find_task_by_pid_ns(pid/*nr*/, task_active_pid_ns(current)) {
                    pid = find_pid_ns(nr, ns) {
                        return idr_find(&ns->idr, nr);
                    }
                    return pid_task(pid, PIDTYPE_PID/*type*/) {
                        struct task_struct *result = NULL;
                        if (pid) {
                            struct hlist_node *first;
                            first = rcu_dereference_check(
                                hlist_first_rcu(&pid->tasks[type]),
                                lockdep_tasklist_lock_is_held()
                            );
                            if (first)
                                result = hlist_entry(first, struct task_struct,
                                    pid_links[(type)]
                                );
                        }
                        return result;
                    }
                }
            }
            if (!p)
                goto out;
            grp = task_pgrp(p);
            if (!grp)
                goto out;

            retval = security_task_getpgid(p);
            if (retval)
                goto out;
        }
        retval = pid_vnr(grp) {
            return pid_nr_ns(pid, task_active_pid_ns(current));
        }
    out:
        rcu_read_unlock();
        return retval;
    }
}
```

### copy_pid_ns

```c
struct pid_namespace *copy_pid_ns(unsigned long flags,
    struct user_namespace *user_ns, struct pid_namespace *old_ns)
{
    if (!(flags & CLONE_NEWPID))
        return get_pid_ns(old_ns);
    if (task_active_pid_ns(current) != old_ns)
        return ERR_PTR(-EINVAL);

    return create_pid_namespace(user_ns, old_ns) {
        struct pid_namespace *ns;
        unsigned int level = parent_pid_ns->level + 1;
        struct ucounts *ucounts;
        int err;

        err = -EINVAL;
        if (!in_userns(parent_pid_ns->user_ns, user_ns))
            goto out;

        err = -ENOSPC;
        if (level > MAX_PID_NS_LEVEL)
            goto out;
        ucounts = inc_pid_namespaces(user_ns);
        if (!ucounts)
            goto out;

        err = -ENOMEM;
        ns = kmem_cache_zalloc(pid_ns_cachep, GFP_KERNEL);
        if (ns == NULL)
            goto out_dec;

        idr_init(&ns->idr);

        ns->pid_cachep = create_pid_cachep(level) {
            /* Level 0 is init_pid_ns.pid_cachep */
            struct kmem_cache **pkc = &pid_cache[level - 1];
            struct kmem_cache *kc;
            char name[4 + 10 + 1];
            unsigned int len;

            kc = READ_ONCE(*pkc);
            if (kc)
                return kc;

            snprintf(name, sizeof(name), "pid_%u", level + 1);
            len = struct_size_t(struct pid, numbers, level + 1);
            mutex_lock(&pid_caches_mutex);
            /* Name collision forces to do allocation under mutex. */
            if (!*pkc)
                *pkc = kmem_cache_create(name, len, 0,
                            SLAB_HWCACHE_ALIGN | SLAB_ACCOUNT, NULL);
            mutex_unlock(&pid_caches_mutex);
            /* current can fail, but someone else can succeed. */
            return READ_ONCE(*pkc);
        }
        if (ns->pid_cachep == NULL)
            goto out_free_idr;

        err = ns_alloc_inum(&ns->ns);
        if (err)
            goto out_free_idr;
        ns->ns.ops = &pidns_operations {
            .name       = "pid",
            .type       = CLONE_NEWPID,
            .get        = pidns_get,
            .put        = pidns_put,
            .install    = pidns_install,
            .owner      = pidns_owner,
            .get_parent = pidns_get_parent,
        };

        refcount_set(&ns->ns.count, 1);
        ns->level = level;
        ns->parent = get_pid_ns(parent_pid_ns);
        ns->user_ns = get_user_ns(user_ns);
        ns->ucounts = ucounts;
        ns->pid_allocated = PIDNS_ADDING;
    #if defined(CONFIG_SYSCTL) && defined(CONFIG_MEMFD_CREATE)
        ns->memfd_noexec_scope = pidns_memfd_noexec_scope(parent_pid_ns);
    #endif

        return ns;

    out_free_idr:
        idr_destroy(&ns->idr);
        kmem_cache_free(pid_ns_cachep, ns);
    out_dec:
        dec_pid_namespaces(ucounts);
    out:
        return ERR_PTR(err);
    }
}
```

### alloc_pid

```c
struct pid *alloc_pid(struct pid_namespace *ns, pid_t *set_tid, size_t set_tid_size)
{
    struct pid *pid;
    enum pid_type type;
    int i, nr;
    struct pid_namespace *tmp;
    struct upid *upid;
    int retval = -ENOMEM;

    if (set_tid_size > ns->level + 1)
        return ERR_PTR(-EINVAL);

    pid = kmem_cache_alloc(ns->pid_cachep, GFP_KERNEL);

    tmp = ns;
    pid->level = ns->level;

    for (i = ns->level; i >= 0; i--) {
        int tid = 0;

        if (set_tid_size) {
            tid = set_tid[ns->level - i];

            retval = -EINVAL;
            if (tid < 1 || tid >= pid_max)
                goto out_free;
            if (tid != 1 && !tmp->child_reaper)
                goto out_free;
            retval = -EPERM;
            if (!checkpoint_restore_ns_capable(tmp->user_ns))
                goto out_free;
            set_tid_size--;
        }

        idr_preload(GFP_KERNEL);
        spin_lock_irq(&pidmap_lock);

        if (tid) {
            nr = idr_alloc(&tmp->idr, NULL, tid, tid + 1, GFP_ATOMIC);
            if (nr == -ENOSPC) {
                nr = -EEXIST;
            }
        } else {
            int pid_min = 1;
            if (idr_get_cursor(&tmp->idr) > RESERVED_PIDS) {
                pid_min = RESERVED_PIDS;
            }
            nr = idr_alloc_cyclic(&tmp->idr, NULL, pid_min, pid_max, GFP_ATOMIC);
        }
        spin_unlock_irq(&pidmap_lock);
        idr_preload_end();

        if (nr < 0) {
            retval = (nr == -ENOSPC) ? -EAGAIN : nr;
            goto out_free;
        }

        pid->numbers[i].nr = nr;
        pid->numbers[i].ns = tmp;
        tmp = tmp->parent;
    }

    retval = -ENOMEM;

    get_pid_ns(ns);
    refcount_set(&pid->count, 1);
    spin_lock_init(&pid->lock);
    for (type = 0; type < PIDTYPE_MAX; ++type) {
        INIT_HLIST_HEAD(&pid->tasks[type]);
    }

    init_waitqueue_head(&pid->wait_pidfd);
    INIT_HLIST_HEAD(&pid->inodes);

    upid = pid->numbers + ns->level;
    spin_lock_irq(&pidmap_lock);
    if (!(ns->pid_allocated & PIDNS_ADDING)) {
        goto out_unlock;
    }
    pid->stashed = NULL;
    pid->ino = ++pidfs_ino;

    /* Make the PID visible to find_pid_ns. */
    for ( ; upid >= pid->numbers; --upid) {
        idr_replace(&upid->ns->idr, pid, upid->nr);
        upid->ns->pid_allocated++;
    }
    spin_unlock_irq(&pidmap_lock);

    return pid;
}
```

## ipc_namespace

* https://time.geekbang.org/column/article/104277

![](../images/kernel/ipc-ipc_ids.png)

Linux has multiple IPC:
* PIEP
* named PIPE
* Signal
* Message Qeueue
* Semaphore
* Shared Memory
* Memory Map
* Socket

While Message Queeue, Semaphore and Shared Memory are called XSI IPC, since they come from UNIX System V IPC.

Linux IPC Namespace just focus on XSI IPC and has nothing to do with other IPC mechanisms.

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
    int                   in_use;
    unsigned short        seq;
    struct rw_semaphore   rwsem;
    struct idr            ipcs_idr;
    int                   next_id;
};

struct idr {
    struct radix_tree_root    idr_rt;
    unsigned int              idr_next;
};

struct kern_ipc_perm *ipc_obtain_object_idr(struct ipc_ids *ids, int id)
{
    struct kern_ipc_perm *out;
    int idx = ipcid_to_idx(id);

    out = idr_find(&ids->ipcs_idr, idx);
    if (!out)
        return ERR_PTR(-EINVAL);

    return out;
}

struct sem_array *sem_obtain_object(struct ipc_namespace *ns, int id)
{
    struct kern_ipc_perm *ipcp = ipc_obtain_object_idr(&sem_ids(ns), id);

    if (IS_ERR(ipcp))
        return ERR_CAST(ipcp);

    return container_of(ipcp, struct sem_array, sem_perm);
}

struct msg_queue *msq_obtain_object(struct ipc_namespace *ns, int id)
{
    struct kern_ipc_perm *ipcp = ipc_obtain_object_idr(&msg_ids(ns), id);

    if (IS_ERR(ipcp))
        return ERR_CAST(ipcp);

    return container_of(ipcp, struct msg_queue, q_perm);
}

struct shmid_kernel *shm_obtain_object(struct ipc_namespace *ns, int id)
{
    struct kern_ipc_perm *ipcp = ipc_obtain_object_idr(&shm_ids(ns), id);

    if (IS_ERR(ipcp))
        return ERR_CAST(ipcp);

    return container_of(ipcp, struct shmid_kernel, shm_perm);
}
```

### copy_ipcs

```c
struct ipc_namespace *copy_ipcs(unsigned long flags,
    struct user_namespace *user_ns, struct ipc_namespace *ns)
{
    if (!(flags & CLONE_NEWIPC))
        return get_ipc_ns(ns);
    return create_ipc_ns(user_ns, ns) {
        struct ipc_namespace *ns;
        struct ucounts *ucounts;
        int err;

        err = -ENOSPC;
    again:
        ucounts = inc_ipc_namespaces(user_ns);
        if (!ucounts) {
            /* IPC namespaces are freed asynchronously, by free_ipc_work. */
            if (flush_work(&free_ipc_work))
                goto again;
            goto fail;
        }

        err = -ENOMEM;
        ns = kzalloc(sizeof(struct ipc_namespace), GFP_KERNEL_ACCOUNT);
        err = ns_alloc_inum(&ns->ns);
        ns->ns.ops = &ipcns_operations;

        refcount_set(&ns->ns.count, 1);
        ns->user_ns = get_user_ns(user_ns);
        ns->ucounts = ucounts;

        err = mq_init_ns(ns);
        if (err)
            goto fail_put;

        err = -ENOMEM;
        if (!setup_mq_sysctls(ns))
            goto fail_put;

        setup_ipc_sysctls(ns) {
            struct ctl_table *tbl;

            setup_sysctl_set(&ns->ipc_set, &set_root, set_is_seen);
            tbl = kmemdup(ipc_sysctls, sizeof(ipc_sysctls), GFP_KERNEL);
            if (tbl) {
                int i;

                for (i = 0; i < ARRAY_SIZE(ipc_sysctls); i++) {
                    if (tbl[i].data == &init_ipc_ns.shm_ctlmax)
                        tbl[i].data = &ns->shm_ctlmax;

                    else if (tbl[i].data == &init_ipc_ns.shm_ctlall)
                        tbl[i].data = &ns->shm_ctlall;

                    else if (tbl[i].data == &init_ipc_ns.shm_ctlmni)
                        tbl[i].data = &ns->shm_ctlmni;

                    else if (tbl[i].data == &init_ipc_ns.shm_rmid_forced)
                        tbl[i].data = &ns->shm_rmid_forced;

                    else if (tbl[i].data == &init_ipc_ns.msg_ctlmax)
                        tbl[i].data = &ns->msg_ctlmax;

                    else if (tbl[i].data == &init_ipc_ns.msg_ctlmni)
                        tbl[i].data = &ns->msg_ctlmni;

                    else if (tbl[i].data == &init_ipc_ns.msg_ctlmnb)
                        tbl[i].data = &ns->msg_ctlmnb;

                    else if (tbl[i].data == &init_ipc_ns.sem_ctls)
                        tbl[i].data = &ns->sem_ctls;
                    else if (tbl[i].data == &init_ipc_ns.ids[IPC_SEM_IDS].next_id)
                        tbl[i].data = &ns->ids[IPC_SEM_IDS].next_id;

                    else if (tbl[i].data == &init_ipc_ns.ids[IPC_MSG_IDS].next_id)
                        tbl[i].data = &ns->ids[IPC_MSG_IDS].next_id;

                    else if (tbl[i].data == &init_ipc_ns.ids[IPC_SHM_IDS].next_id)
                        tbl[i].data = &ns->ids[IPC_SHM_IDS].next_id;

                    else
                        tbl[i].data = NULL;
                }

                ns->ipc_sysctls = __register_sysctl_table(&ns->ipc_set, "kernel", tbl,
                                    ARRAY_SIZE(ipc_sysctls));
            }
            if (!ns->ipc_sysctls) {
                kfree(tbl);
                retire_sysctl_set(&ns->ipc_set);
                return false;
            }

            return true;
        }

        err = msg_init_ns(ns);

        sem_init_ns(ns) {
            ns->sc_semmsl = SEMMSL;
            ns->sc_semmns = SEMMNS;
            ns->sc_semopm = SEMOPM;
            ns->sc_semmni = SEMMNI;
            ns->used_sems = 0;
            ipc_init_ids(&ns->ids[IPC_SEM_IDS]);
        }

        shm_init_ns(ns) {
            ns->shm_ctlmax = SHMMAX;
            ns->shm_ctlall = SHMALL;
            ns->shm_ctlmni = SHMMNI;
            ns->shm_rmid_forced = 0;
            ns->shm_tot = 0;
            ipc_init_ids(&shm_ids(ns)) {
                ids->in_use = 0;
                ids->seq = 0;
                init_rwsem(&ids->rwsem);
                rhashtable_init(&ids->key_ht, &ipc_kht_params);
                idr_init(&ids->ipcs_idr);
                ids->max_idx = -1;
                ids->last_idx = -1;
            }
        }

        return ns;
    }
}
```

### shmget

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

/* 1. specify searching ns */
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
/* 2. search in the ns */
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
                        err = ipc_check_perms(ns, ipcp, ops, params);
                    }
                }
                return err;
            }
        }
    }
}
```

## uts_namespace

```c
struct uts_namespace {
    struct new_utsname name;
    struct user_namespace *user_ns;
    struct ucounts *ucounts;
    struct ns_common ns;
};

struct new_utsname {
    char sysname[__NEW_UTS_LEN + 1];
    char nodename[__NEW_UTS_LEN + 1];
    char release[__NEW_UTS_LEN + 1];
    char version[__NEW_UTS_LEN + 1];
    char machine[__NEW_UTS_LEN + 1];
    char domainname[__NEW_UTS_LEN + 1];
};
```

```c
struct ctl_table uts_kern_table[] = {
    {
        .procname   = "arch",
        /* offset = data - (char*)&init_uts_ns
         * data = offset + (char*)uts_ns */
        .data       = init_uts_ns.name.machine,
        .maxlen     = sizeof(init_uts_ns.name.machine),
        .mode       = 0444,
        .proc_handler   = proc_do_uts_string,
    },
    {
        .procname   = "ostype",
        .data       = init_uts_ns.name.sysname,
        .maxlen     = sizeof(init_uts_ns.name.sysname),
        .mode       = 0444,
        .proc_handler   = proc_do_uts_string,
    },
    {
        .procname   = "osrelease",
        .data       = init_uts_ns.name.release,
        .maxlen     = sizeof(init_uts_ns.name.release),
        .mode       = 0444,
        .proc_handler   = proc_do_uts_string,
    },
    {
        .procname    = "version",
        .data       = init_uts_ns.name.version,
        .maxlen     = sizeof(init_uts_ns.name.version),
        .mode       = 0444,
        .proc_handler   = proc_do_uts_string,
    },
    {
        .procname    = "hostname",
        .data       = init_uts_ns.name.nodename,
        .maxlen     = sizeof(init_uts_ns.name.nodename),
        .mode       = 0644,
        .proc_handler   = proc_do_uts_string,
        .poll       = &hostname_poll,
    },
    {
        .procname   = "domainname",
        .data       = init_uts_ns.name.domainname,
        .maxlen     = sizeof(init_uts_ns.name.domainname),
        .mode       = 0644,
        .proc_handler   = proc_do_uts_string,
        .poll       = &domainname_poll,
    },
    {}
};

enum uts_proc {
    UTS_PROC_ARCH,
    UTS_PROC_OSTYPE,
    UTS_PROC_OSRELEASE,
    UTS_PROC_VERSION,
    UTS_PROC_HOSTNAME,
    UTS_PROC_DOMAINNAME,
};

SYSCALL_DEFINE2(sethostname, char __user *, name, int, len)
{
    int errno;
    char tmp[__NEW_UTS_LEN];

    errno = -EFAULT;
    if (!copy_from_user(tmp, name, len)) {
        struct new_utsname *u;

        add_device_randomness(tmp, len);
        down_write(&uts_sem);

        struct new_utsname *u = utsname() {
            return &current->nsproxy->uts_ns->name;
        }
        memcpy(u->nodename, tmp, len);
        memset(u->nodename + len, 0, sizeof(u->nodename) - len);
        errno = 0;

        uts_proc_notify(UTS_PROC_HOSTNAME) {
            struct ctl_table *table = &uts_kern_table[proc];
            proc_sys_poll_notify(table->poll);
        }
        up_write(&uts_sem);
    }
    return errno;
}

int proc_do_uts_string(struct ctl_table *table, int write,
        void *buffer, size_t *lenp, loff_t *ppos)
{
    struct ctl_table uts_table;
    int r;
    char tmp_data[__NEW_UTS_LEN + 1];

    memcpy(&uts_table, table, sizeof(uts_table));
    uts_table.data = tmp_data;

    down_read(&uts_sem);
    memcpy(tmp_data, get_uts(table), sizeof(tmp_data));
    up_read(&uts_sem);
    /* Reads/writes a string from/to the user buffer */
    r = proc_dostring(&uts_table, write, buffer, lenp, ppos);

    if (write) {
        /* Write back the new value */
        add_device_randomness(tmp_data, sizeof(tmp_data));
        down_write(&uts_sem);
        memcpy(get_uts(table), tmp_data, sizeof(tmp_data));
        up_write(&uts_sem);
        proc_sys_poll_notify(table->poll);
    }

    return r;
}
```

## user_namespace

* the new process has a full set of capabilities in the new user namespace, it has no capabilities in the parent namespace. In particular, even if root employs clone(CLONE_NEWUSER), the resulting child process will have no capabilities in the parent namespace.

```c

```

## mnt_namespace

![](../images/kernel/ns-mnt.png)

```c
struct mnt_namespace {
    struct ns_common        ns;
    struct mount*           root;
    struct rb_root          mounts; /* Protected by namespace_sem */
    struct user_namespace   *user_ns;
    struct ucounts          *ucounts;
    u64                     seq; /* Sequence number to prevent loops */
    wait_queue_head_t       poll;
    u64 event;
    unsigned int            nr_mounts; /* # of mounts in the namespace */
    unsigned int            pending_mounts;
}

struct mount {
    struct hlist_node mnt_hash;
    struct mount *mnt_parent;
    struct dentry *mnt_mountpoint;
    struct vfsmount mnt;

    struct list_head mnt_mounts;    /* list of children, anchored here */
    struct list_head mnt_child;    /* anchored at parent */
    struct list_head mnt_instance;    /* mount instance on sb->s_mounts */
    const char *mnt_devname;    /* Name of device e.g. /dev/dsk/hda1 */
    union {
        struct rb_node mnt_node;    /* Under ns->mounts */
        struct list_head mnt_list;
    };
    struct list_head mnt_expire;    /* link in fs-specific expiry list */
    struct list_head mnt_share;    /* circular list of shared mounts */
    struct list_head mnt_slave_list;/* list of slave mounts */
    struct list_head mnt_slave;    /* slave list entry */
    struct mount *mnt_master;    /* slave is on master->mnt_slave_list */
    struct mnt_namespace *mnt_ns;    /* containing namespace */
    struct mountpoint *mnt_mp;    /* where is it mounted */
    union {
        struct hlist_node mnt_mp_list;    /* list mounts with the same mountpoint */
        struct hlist_node mnt_umount;
    };
    struct list_head mnt_umounting; /* list entry for umount propagation */

    int mnt_id;            /* mount identifier, reused */
    u64 mnt_id_unique;        /* mount ID unique until reboot */
    int mnt_group_id;        /* peer group identifier */
    int mnt_expiry_mark;        /* true if marked for expiry */
    struct hlist_head mnt_pins;
    struct hlist_head mnt_stuck_children;
}

struct vfsmount {
    struct dentry *mnt_root;    /* root of the mounted tree */
    struct super_block *mnt_sb; /* pointer to superblock */
    int mnt_flags;
    struct mnt_idmap *mnt_idmap;
}

struct mountpoint {
    struct hlist_node   m_hash;
    struct dentry       *m_dentry;
    struct hlist_head   m_list;
    int                 m_count;
};
```

### copy_mnt_ns

![](../images/kernel/ns-mnt-hierarchy.png)

```c
struct mnt_namespace *copy_mnt_ns(unsigned long flags, struct mnt_namespace *ns,
        struct user_namespace *user_ns, struct fs_struct *new_fs)
{
    struct mnt_namespace *new_ns;
    struct vfsmount *rootmnt = NULL, *pwdmnt = NULL;
    struct mount *old;
    struct mount *new;
    int copy_flags;
/* 1. alloc a new mnt_namespace */
    old = ns->root;
    new_ns = alloc_mnt_ns(user_ns, false/*anon*/) {
        struct mnt_namespace *new_ns;
        struct ucounts *ucounts;
        int ret;

        new_ns = kzalloc(sizeof(struct mnt_namespace), GFP_KERNEL_ACCOUNT);
        if (!anon) {
            ret = ns_alloc_inum(&new_ns->ns);
        }
        new_ns->ns.ops = &mntns_operations {
            .name       = "mnt",
            .type       = CLONE_NEWNS,
            .get        = mntns_get,
            .put        = mntns_put,
            .install    = mntns_install,
            .owner      = mntns_owner,
        };
        if (!anon)
            new_ns->seq = atomic64_add_return(1, &mnt_ns_seq);
        refcount_set(&new_ns->ns.count, 1);
        new_ns->mounts = RB_ROOT;
        init_waitqueue_head(&new_ns->poll);
        new_ns->user_ns = get_user_ns(user_ns);
        new_ns->ucounts = ucounts;
        return new_ns;
    }

    /* First pass: copy the tree topology */
    copy_flags = CL_COPY_UNBINDABLE | CL_EXPIRE;
    if (user_ns != ns->user_ns)
        copy_flags |= CL_SHARED_TO_SLAVE;

/* 2. copy parent mnt hierarchy */
    new = copy_tree(old/*mnt*/, old->mnt.mnt_root/*dentry*/, copy_flags) {
        /* struct mount *res, *p, *q, *r, *parent; */
        struct mount *res, *o_prnt, *o_child, *o_mnt, *n_prnt, *n_mnt;

        res = n_mnt = clone_mnt(mnt/*old*/, dentry/*root*/, flag) {
            struct super_block *sb = old->mnt.mnt_sb;
            struct mount *mnt;
            int err;

            mnt = alloc_vfsmnt(old->mnt_devname);

            mnt->mnt.mnt_flags = old->mnt.mnt_flags;
            mnt->mnt.mnt_flags &= ~(MNT_WRITE_HOLD|MNT_MARKED|MNT_INTERNAL|MNT_ONRB);

            atomic_inc(&sb->s_active);
            mnt->mnt.mnt_idmap = mnt_idmap_get(mnt_idmap(&old->mnt));

            mnt->mnt.mnt_sb = sb;
            mnt->mnt.mnt_root = dget(root);
            mnt->mnt_mountpoint = mnt->mnt.mnt_root;
            mnt->mnt_parent = mnt;
            list_add_tail(&mnt->mnt_instance, &sb->s_mounts);

            if ((flag & CL_SLAVE) ||
                ((flag & CL_SHARED_TO_SLAVE) && IS_MNT_SHARED(old))) {
                list_add(&mnt->mnt_slave, &old->mnt_slave_list);
                mnt->mnt_master = old;
                CLEAR_MNT_SHARED(mnt);
            } else if (!(flag & CL_PRIVATE)) {
                if ((flag & CL_MAKE_SHARED) || IS_MNT_SHARED(old))
                    list_add(&mnt->mnt_share, &old->mnt_share);
                if (IS_MNT_SLAVE(old))
                    list_add(&mnt->mnt_slave, &old->mnt_slave);
                mnt->mnt_master = old->mnt_master;
            } else {
                CLEAR_MNT_SHARED(mnt);
            }
            if (flag & CL_MAKE_SHARED)
                set_mnt_shared(mnt);

            /* stick the duplicate mount on the same expiry list
             * as the original if that was on one */
            if (flag & CL_EXPIRE) {
                if (!list_empty(&old->mnt_expire))
                    list_add(&mnt->mnt_expire, &old->mnt_expire);
            }

            return mnt;
        }

        n_mnt->mnt_mountpoint = mnt->mnt_mountpoint;

        o_prnt = mnt;
        list_for_each_entry(o_child, &mnt->mnt_mounts, mnt_child) {
            if (!is_subdir(o_child->mnt_mountpoint, dentry))
                continue;

            for (o_mnt = o_child; o_mnt; o_mnt = next_mnt(o_mnt, o_child)) {
                if (!(flag & CL_COPY_UNBINDABLE) && IS_MNT_UNBINDABLE(o_mnt)) {
                    if (o_mnt->mnt.mnt_flags & MNT_LOCKED) {
                        /* Both unbindable and locked. */
                        n_mnt = ERR_PTR(-EPERM);
                        goto out;
                    } else {
                        o_mnt = skip_mnt_tree(o_mnt);
                        continue;
                    }
                }
                if (!(flag & CL_COPY_MNT_NS_FILE) && is_mnt_ns_file(o_mnt->mnt.mnt_root)) {
                    o_mnt = skip_mnt_tree(o_mnt);
                    continue;
                }
                while (o_prnt != o_mnt->mnt_parent) {
                    o_prnt = o_prnt->mnt_parent;
                    n_mnt = n_mnt->mnt_parent;
                }
                o_prnt = o_mnt;
                n_prnt = n_mnt;
                n_mnt = clone_mnt(o_prnt, o_prnt->mnt.mnt_root, flag);

                list_add_tail(&n_mnt->mnt_list, &res->mnt_list);
                attach_mnt(n_mnt, n_prnt, o_prnt->mnt_mp, false) {
                    if (beneath) {
                        mnt_set_mountpoint_beneath(mnt/*new_parent*/, n_prnt/*top_mnt*/, mp) {
                            struct mount *old_top_parent = top_mnt->mnt_parent;
                            struct mountpoint *old_top_mp = top_mnt->mnt_mp;

                            mnt_set_mountpoint(old_top_parent/*mnt*/, old_top_mp/*mp*/, new_parent/*child_mnt*/) {
                                mp->m_count++;
                                mnt_add_count(mnt, 1);    /* essentially, that'o_mnt mntget */
                                child_mnt->mnt_mountpoint = mp->m_dentry;
                                child_mnt->mnt_parent = mnt;
                                child_mnt->mnt_mp = mp;
                                hlist_add_head(&child_mnt->mnt_mp_list, &mp->m_list);
                            }

                            mnt_change_mountpoint(new_parent/*n_prnt*/, new_mp/*mp*/, top_mnt/*mnt*/) {
                                struct mountpoint *old_mp = mnt->mnt_mp;
                                struct mount *old_parent = mnt->mnt_parent;

                                list_del_init(&mnt->mnt_child);
                                hlist_del_init(&mnt->mnt_mp_list);
                                hlist_del_init_rcu(&mnt->mnt_hash);

                                attach_mnt(mnt, n_prnt, mp, false);

                                put_mountpoint(old_mp);
                                mnt_add_count(old_parent, -1);
                            }
                        }
                    } else {
                        mnt_set_mountpoint(n_prnt, mp, mnt);
                    }

                    __attach_mnt(mnt, mnt->mnt_parent) {
                        hlist_add_head_rcu(&mnt->mnt_hash, m_hash(&n_prnt->mnt, mnt->mnt_mountpoint));
                        list_add_tail(&mnt->mnt_child, &n_prnt->mnt_mounts);
                    }
                }
                unlock_mount_hash();
            }
        }
        return res;
    }
    if (user_ns != ns->user_ns) {
        lock_mount_hash();
        lock_mnt_tree(new);
        unlock_mount_hash();
    }
    new_ns->root = new;
/* 3. add the new mnt into ns */
    /* Second pass: switch the tsk->fs->* elements and mark new vfsmounts
     * as belonging to new namespace. */
    o_mnt = old;
    n_mnt = new;
    while (o_mnt) {
        mnt_add_to_ns(new_ns/*ns*/, n_mnt/*mnt*/) {
            struct rb_node **link = &ns->mounts.rb_node;
            struct rb_node *parent = NULL;

            mnt->mnt_ns = ns;
            while (*link) {
                parent = *link;
                if (mnt->mnt_id_unique < node_to_mount(parent)->mnt_id_unique)
                    link = &parent->rb_left;
                else
                    link = &parent->rb_right;
            }
            rb_link_node(&mnt->mnt_node, parent, link);
            rb_insert_color(&mnt->mnt_node, &ns->mounts);
            mnt->mnt.mnt_flags |= MNT_ONRB;
        }

        new_ns->nr_mounts++;
        if (new_fs) {
            if (&o_mnt->mnt == new_fs->root.mnt) {
                new_fs->root.mnt = mntget(&n_mnt->mnt);
                rootmnt = &o_mnt->mnt;
            }
            if (&o_mnt->mnt == new_fs->pwd.mnt) {
                new_fs->pwd.mnt = mntget(&n_mnt->mnt);
                pwdmnt = &o_mnt->mnt;
            }
        }
        o_mnt = next_mnt(o_mnt, old);
        n_mnt = next_mnt(n_mnt, new);
        if (!n_mnt)
            break;
        // an mntns binding we'd skipped?
        while (o_mnt->mnt.mnt_root != n_mnt->mnt.mnt_root)
            o_mnt = next_mnt(skip_mnt_tree(o_mnt), old);
    }

    return new_ns;
}
```

### propagate_mnt

```c
int propagate_mnt(struct mount *dest_mnt, struct mountpoint *dest_mp,
            struct mount *source_mnt, struct hlist_head *tree_list)
{
    struct mount *m, *n;
    int ret = 0;

    last_dest = dest_mnt;
    first_source = source_mnt;
    last_source = source_mnt;
    list = tree_list;
    dest_master = dest_mnt->mnt_master;

    /* all peers of dest_mnt, except dest_mnt itself */
    for (n = next_peer(dest_mnt); n != dest_mnt; n = next_peer(n)) {
        ret = propagate_one(n, dest_mp);
        if (ret)
            goto out;
    }

    /* all slave groups */
    for (m = next_group(dest_mnt, dest_mnt); m; m = next_group(m, dest_mnt)) {
        /* everything in that slave group */
        n = m;
        do {
            ret = propagate_one(n, dest_mp) {
                struct mount *child;
                int type;
                /* skip ones added by this propagate_mnt() */
                if (IS_MNT_NEW(m))
                    return 0;
                /* skip if mountpoint isn't covered by it */
                if (!is_subdir(dest_mp->m_dentry, m->mnt.mnt_root))
                    return 0;
                if (peers(m, last_dest)) {
                    /* return m1->mnt_group_id == m2->mnt_group_id && m1->mnt_group_id; */
                    type = CL_MAKE_SHARED;
                } else {
                    struct mount *n, *p;
                    bool done;
                    for (n = m; ; n = p) {
                        p = n->mnt_master;
                        if (p == dest_master || IS_MNT_MARKED(p))
                            break;
                    }
                    do {
                        struct mount *parent = last_source->mnt_parent;
                        if (peers(last_source, first_source))
                            break;
                        done = parent->mnt_master == p;
                        if (done && peers(n, parent))
                            break;
                        last_source = last_source->mnt_master;
                    } while (!done);

                    type = CL_SLAVE;
                    /* beginning of peer group among the slaves? */
                    if (IS_MNT_SHARED(m))
                        type |= CL_MAKE_SHARED;
                }

                child = copy_tree(last_source, last_source->mnt.mnt_root, type);
                    --->
                if (IS_ERR(child))
                    return PTR_ERR(child);
                read_seqlock_excl(&mount_lock);
                mnt_set_mountpoint(m, dest_mp, child);
                if (m->mnt_master != dest_master)
                    SET_MNT_MARK(m->mnt_master);
                read_sequnlock_excl(&mount_lock);
                last_dest = m;
                last_source = child;
                hlist_add_head(&child->mnt_hash, list);
                return count_mounts(m->mnt_ns, child);
            }
            if (ret)
                goto out;
            n = next_peer(n);
        } while (n != m);
    }
out:
    read_seqlock_excl(&mount_lock);
    hlist_for_each_entry(n, tree_list, mnt_hash) {
        m = n->mnt_parent;
        if (m->mnt_master != dest_mnt->mnt_master)
            CLEAR_MNT_MARK(m->mnt_master);
    }
    read_sequnlock_excl(&mount_lock);
    return ret;
}
```

### propagate_umount

```c
int propagate_umount(struct list_head *list)
{
    struct mount *mnt;
    LIST_HEAD(to_restore);
    LIST_HEAD(to_umount);
    LIST_HEAD(visited);

    /* Find candidates for unmounting */
    list_for_each_entry_reverse(mnt, list, mnt_list) {
        struct mount *parent = mnt->mnt_parent;
        struct mount *m;

        /*
        * If this mount has already been visited it is known that it's
        * entire peer group and all of their slaves in the propagation
        * tree for the mountpoint has already been visited and there is
        * no need to visit them again.
        */
        if (!list_empty(&mnt->mnt_umounting))
            continue;

        list_add_tail(&mnt->mnt_umounting, &visited);
        propagation_next(m, origin) {
            /* are there any slaves of this mount? */
            if (!IS_MNT_NEW(m) && !list_empty(&m->mnt_slave_list))
                return first_slave(m);

            while (1) {
                struct mount *master = m->mnt_master;

                if (master == origin->mnt_master) {
                    struct mount *next = next_peer(m);
                    return (next == origin) ? NULL : next;
                } else if (m->mnt_slave.next != &master->mnt_slave_list)
                    return next_slave(m);

                /* back at master */
                m = master;
            }
        }
        for (m = propagation_next(parent, parent); m; m = propagation_next(m, parent)) {
            struct mount *child = __lookup_mnt(&m->mnt, mnt->mnt_mountpoint) {
                struct hlist_head *head = m_hash(mnt, dentry);
                struct mount *p;

                hlist_for_each_entry_rcu(p, head, mnt_hash)
                    if (&p->mnt_parent->mnt == mnt && p->mnt_mountpoint == dentry)
                        return p;
                return NULL;
            }
            if (!child)
                continue;

            if (!list_empty(&child->mnt_umounting)) {
                /*
                * If the child has already been visited it is
                * know that it's entire peer group and all of
                * their slaves in the propgation tree for the
                * mountpoint has already been visited and there
                * is no need to visit this subtree again.
                */
                m = skip_propagation_subtree(m, parent);
                continue;
            } else if (child->mnt.mnt_flags & MNT_UMOUNT) {
                /*
                * We have come accross an partially unmounted
                * mount in list that has not been visited yet.
                * Remember it has been visited and continue
                * about our merry way.
                */
                list_add_tail(&child->mnt_umounting, &visited);
                continue;
            }

            /* Check the child and parents while progress is made */
            __propagate_umount(child, &to_umount, &to_restore) {
                bool progress = false;
                struct mount *child;

                /*
                * The state of the parent won't change if this mount is
                * already unmounted or marked as without children.
                */
                if (mnt->mnt.mnt_flags & (MNT_UMOUNT | MNT_MARKED))
                    goto out;

                /* Verify topper is the only grandchild that has not been
                * speculatively unmounted.
                */
                list_for_each_entry(child, &mnt->mnt_mounts, mnt_child) {
                    if (child->mnt_mountpoint == mnt->mnt.mnt_root)
                        continue;
                    if (!list_empty(&child->mnt_umounting) && IS_MNT_MARKED(child))
                        continue;
                    /* Found a mounted child */
                    goto children;
                }

                /* Mark mounts that can be unmounted if not locked */
                SET_MNT_MARK(mnt);
                progress = true;

                /* If a mount is without children and not locked umount it. */
                if (!IS_MNT_LOCKED(mnt)) {
                    umount_one(mnt, to_umount) {
                        CLEAR_MNT_MARK(mnt);
                        mnt->mnt.mnt_flags |= MNT_UMOUNT;
                        list_del_init(&mnt->mnt_child);
                        list_del_init(&mnt->mnt_umounting);
                        move_from_ns(mnt, to_umount);
                    }
                } else {
            children:
                    list_move_tail(&mnt->mnt_umounting, to_restore);
                }
            out:
                return progress;
            }
            while (__propagate_umount()) {
                /* Is the parent a umount candidate? */
                child = child->mnt_parent;
                if (list_empty(&child->mnt_umounting))
                    break;
            }
        }
    }

    umount_list(&to_umount, &to_restore);
    restore_mounts(&to_restore);
    cleanup_umount_visitations(&visited);
    list_splice_tail(&to_umount, list);

    return 0;
}
```

## cgroup_namespace

* Cgroup namespaces virtualize the view of a process's cgroups (see cgroups(7)) as seen via /proc/pid/cgroup and /proc/pid/mountinfo.
* When a process creates a new cgroup namespace using clone(2) or unshare(2) with the CLONE_NEWCGROUP flag, its current cgroups directories become the cgroup root directories of the new namespace.

```c
struct cgroup_namespace {
    struct ns_common        ns;
    struct user_namespace   *user_ns;
    struct ucounts          *ucounts;
    struct css_set          *root_cset;
};
```

### copy_cgroup_ns

```c
struct cgroup_namespace *copy_cgroup_ns(
    unsigned long flags,
    struct user_namespace *user_ns,
    struct cgroup_namespace *old_ns)
{
    struct cgroup_namespace *new_ns;
    struct ucounts *ucounts;
    struct css_set *cset;

    BUG_ON(!old_ns);

    if (!(flags & CLONE_NEWCGROUP)) {
        get_cgroup_ns(old_ns);
        return old_ns;
    }

    /* Allow only sysadmin to create cgroup namespace. */
    if (!ns_capable(user_ns, CAP_SYS_ADMIN))
        return ERR_PTR(-EPERM);

    ucounts = inc_cgroup_namespaces(user_ns);
    if (!ucounts)
        return ERR_PTR(-ENOSPC);

    cset = task_css_set(current) {
        return (task)->cgroups;
    }

    new_ns = alloc_cgroup_ns() {
        struct cgroup_namespace *new_ns;
        int ret;

        new_ns = kzalloc(sizeof(struct cgroup_namespace), GFP_KERNEL_ACCOUNT);
        ret = ns_alloc_inum(&new_ns->ns);
        refcount_set(&new_ns->ns.count, 1);
        new_ns->ns.ops = &cgroupns_operations {
            .name       = "cgroup",
            .type       = CLONE_NEWCGROUP,
            .get        = cgroupns_get,
            .put        = cgroupns_put,
            .install    = cgroupns_install,
            .owner      = cgroupns_owner,
        };
        return new_ns;
    }

    new_ns->user_ns = get_user_ns(user_ns);
    new_ns->ucounts = ucounts;
    new_ns->root_cset = cset;

    return new_ns;
}
```

### cgroup_fork

```c
fork() {
    copy_process() {
        cgroup_fork(p) {
            RCU_INIT_POINTER(child->cgroups, &init_css_set);
            INIT_LIST_HEAD(&child->cg_list);
        }

        cgroup_can_fork(p, args);
            --->

        sched_cgroup_fork(p, args) {
            unsigned long flags;

            raw_spin_lock_irqsave(&p->pi_lock, flags);
        #ifdef CONFIG_CGROUP_SCHED
            if (1) {
                struct task_group *tg;
                tg = container_of(kargs->cset->subsys[cpu_cgrp_id],
                        struct task_group, css);
                tg = autogroup_task_group(p, tg);
                p->sched_task_group = tg;
            }
        #endif
            rseq_migrate(p);
            __set_task_cpu(p, smp_processor_id());
            if (p->sched_class->task_fork)
                p->sched_class->task_fork(p);
            raw_spin_unlock_irqrestore(&p->pi_lock, flags);
        }

        cgroup_post_fork(p, args) {
            unsigned long cgrp_flags = 0;
            bool kill = false;
            struct cgroup_subsys *ss;
            struct css_set *cset;
            int i;

            cset = kargs->cset;
            kargs->cset = NULL;

            spin_lock_irq(&css_set_lock);

            /* init tasks are special, only link regular threads */
            if (likely(child->pid)) {
                if (kargs->cgrp)
                    cgrp_flags = kargs->cgrp->flags;
                else
                    cgrp_flags = cset->dfl_cgrp->flags;

                cset->nr_tasks++;
                css_set_move_task(child, NULL, cset, false);
                    --->
            } else {
                put_css_set(cset);
                cset = NULL;
            }

            if (!(child->flags & PF_KTHREAD)) {
                if (unlikely(test_bit(CGRP_FREEZE, &cgrp_flags))) {
                    child->jobctl |= JOBCTL_TRAP_FREEZE;
                }
                kill = test_bit(CGRP_KILL, &cgrp_flags);
            }

            spin_unlock_irq(&css_set_lock);

            do_each_subsys_mask(ss, i, have_fork_callback) {
                ss->fork(child);
            } while_each_subsys_mask();

            /* Make the new cset the root_cset of the new cgroup namespace. */
            if (kargs->flags & CLONE_NEWCGROUP) {
                struct css_set *rcset = child->nsproxy->cgroup_ns->root_cset;

                get_css_set(cset);
                child->nsproxy->cgroup_ns->root_cset = cset;
                put_css_set(rcset);
            }

            /* Cgroup has to be killed so take down child immediately. */
            if (unlikely(kill))
                do_send_sig_info(SIGKILL, SEND_SIG_NOINFO, child, PIDTYPE_TGID);

            cgroup_css_set_put_fork(kargs);
        }
    }
}
```

## net_ns

```c
struct net {
    /* First cache line can be often dirtied.
     * Do not place here read-mostly fields. */
    /* To decide when the network namespace should be freed. */
    refcount_t              passive;
    spinlock_t              rules_mod_lock;

    unsigned int		    dev_base_seq;	/* protected by rtnl_mutex */
    u32			            ifindex;

    spinlock_t		        nsid_lock;
    atomic_t		        fnhe_genid;

    struct list_head	    list; /* list of network namespaces */
    /* To linked to call pernet exit
     * methods on dead net (pernet_ops_rwsem read locked),
     * or to unregister pernet ops (pernet_ops_rwsem write locked). */
    struct list_head	    exit_list;
    struct llist_node	    cleanup_list; /* namespaces on death row */

#ifdef CONFIG_KEYS
    struct key_tag		    *key_domain;
#endif
    struct user_namespace   *user_ns;
    struct ucounts		    *ucounts;
    struct idr		        netns_ids;

    struct ns_common	ns;
    struct ref_tracker_dir      refcnt_tracker;
     /* tracker for objects not refcounted against netns */
    struct ref_tracker_dir      notrefcnt_tracker;
    struct list_head            dev_base_head;
    struct proc_dir_entry       *proc_net;
    struct proc_dir_entry       *proc_net_stat;

    struct ctl_table_set	    sysctls;

    struct sock                 *rtnl; /* rtnetlink socket */
    struct sock                 *genl_sock;

    struct uevent_sock          *uevent_sock; /* uevent socket */

    struct hlist_head           *dev_name_head;
    struct hlist_head           *dev_index_head;
    struct xarray               dev_by_index;
    struct raw_notifier_head    netdev_chain;

    /* Note that @hash_mix can be read millions times per second,
     * it is critical that it is on a read_mostly cache line. */
    u32			                hash_mix;

    struct net_device           *loopback_dev;  /* The loopback */

    /* core fib_rules */
    struct list_head	        rules_ops;

    struct netns_core	        core;
    struct netns_mib	        mib;
    struct netns_packet	        packet;
    struct netns_unix	        unx;
    struct netns_nexthop        nexthop;
    struct netns_ipv4           ipv4;
    struct netns_ipv6           ipv6;
    struct netns_ieee802154_lowpan	ieee802154_lowpan;
    struct netns_sctp           sctp;
    struct netns_nf             nf;
    struct netns_ct             ct;
    struct netns_nftables       nft;
    struct netns_ft             ft;
    struct sk_buff_head         wext_nlevents;

    struct net_generic          *gen;

    /* Used to store attached BPF programs */
    struct netns_bpf                    bpf;

    u64			                        net_cookie; /* written once */

    struct netns_xfrm                   xfrm;
    struct netns_ipvs                   *ipvs;
    struct netns_mpls                   mpls;
    struct netns_can                    can;
    struct netns_xdp                    xdp;
    struct netns_mctp                   mctp;
    struct sock                         *crypto_nlsk;
    struct netns_smc                    smc;
    struct sock                         *diag_nlsk;
}
```

```c
struct net *copy_net_ns(unsigned long flags,
            struct user_namespace *user_ns, struct net *old_net)
{
    struct ucounts *ucounts;
    struct net *net;
    int rv;

    if (!(flags & CLONE_NEWNET))
        return get_net(old_net);

    ucounts = inc_net_namespaces(user_ns);
    if (!ucounts)
        return ERR_PTR(-ENOSPC);

    net = net_alloc();
    if (!net) {
        rv = -ENOMEM;
        goto dec_ucounts;
    }

    preinit_net(net);
    refcount_set(&net->passive, 1);
    net->ucounts = ucounts;
    get_user_ns(user_ns);

    rv = down_read_killable(&pernet_ops_rwsem);
    if (rv < 0)
        goto put_userns;

    rv = setup_net(net, user_ns);

    up_read(&pernet_ops_rwsem);

    return net;
}
```

### pernet_list

```c
struct pernet_operations loopback_net_ops = {
    .init = loopback_net_init(struct net_device *dev) {
        struct net_device *dev;
        int err;

        err = -ENOMEM;
        dev = alloc_netdev(0, "lo", NET_NAME_PREDICTABLE, loopback_setup);

        dev_net_set(dev, net) {
            write_pnet(&dev->nd_net, net);
        }
        err = register_netdev(dev);
        net->loopback_dev = dev;
        return 0;
    },
};

static struct pernet_operations netdev_net_ops = {
    .init = netdev_init(struct net *net) {
       INIT_LIST_HEAD(&net->dev_base_head);

        net->dev_name_head = netdev_create_hash();
        net->dev_index_head = netdev_create_hash();
        xa_init_flags(&net->dev_by_index, XA_FLAGS_ALLOC1);
        RAW_INIT_NOTIFIER_HEAD(&net->netdev_chain);

        return 0;
    },
    .exit = netdev_exit,
};

static struct pernet_operations fou_net_ops = {
    .init = fou_init_net(struct net *net) {
        struct fou_net *fn = net_generic(net, fou_net_id);
        INIT_LIST_HEAD(&fn->fou_list);
        mutex_init(&fn->fou_lock);
        return 0;
    },
    .exit = fou_exit_net,
    .id   = &fou_net_id,
    .size = sizeof(struct fou_net),
};
```

## time_namespace

Time namespaces virtualize the values of two system clocks:
* CLOCK_MONOTONIC, a nonsettable clock that represents monotonic time  since—as described  by  POSIX—"some unspecified  point in the past"

* CLOCK_BOOTTIME, a nonsettable clock that is identical to CLOCK_MONOTONIC, except that it also includes any time that the system is suspended.

```c
struct time_namespace {
	struct user_namespace   *user_ns;
	struct ucounts          *ucounts;
	struct ns_common        ns;
	struct timens_offsets   offsets;
	struct page             *vvar_page;
	/* If set prevents changing offsets after any task joined namespace. */
	bool                    frozen_offsets;
}
```

```c
struct time_namespace *clone_time_ns(
    struct user_namespace *user_ns,
    struct time_namespace *old_ns)
{
    struct time_namespace *ns;
    struct ucounts *ucounts;
    int err;

    err = -ENOSPC;
    ucounts = inc_time_namespaces(user_ns);
    err = -ENOMEM;
    ns = kmalloc(sizeof(*ns), GFP_KERNEL_ACCOUNT);
    refcount_set(&ns->ns.count, 1);

    ns->vvar_page = alloc_page(GFP_KERNEL_ACCOUNT | __GFP_ZERO);

    err = ns_alloc_inum(&ns->ns);

    ns->ucounts = ucounts;
    ns->ns.ops = &timens_operations;
    ns->user_ns = get_user_ns(user_ns);
    ns->offsets = old_ns->offsets;
    ns->frozen_offsets = false;
    return ns;
}
```

```c
void timens_on_fork(struct nsproxy *nsproxy, struct task_struct *tsk)
{
    struct ns_common *nsc = &nsproxy->time_ns_for_children->ns;
    struct time_namespace *ns = to_time_ns(nsc);

    /* create_new_namespaces() already incremented the ref counter */
    if (nsproxy->time_ns == nsproxy->time_ns_for_children)
        return;

    get_time_ns(ns);
    put_time_ns(nsproxy->time_ns);
    nsproxy->time_ns = ns;

    timens_commit(tsk, ns) {
        timens_set_vvar_page(tsk, ns) {
            struct vdso_data *vdata;
            unsigned int i;

            if (ns == &init_time_ns)
                return;

            /* Fast-path, taken by every task in namespace except the first. */
            if (likely(ns->frozen_offsets))
                return;

            mutex_lock(&offset_lock);
            /* Nothing to-do: vvar_page has been already initialized. */
            if (ns->frozen_offsets)
                goto out;

            ns->frozen_offsets = true;
            vdata = arch_get_vdso_data(page_address(ns->vvar_page));

            for (i = 0; i < CS_BASES; i++) {
                timens_setup_vdso_data(&vdata[i], ns) {
                    struct timens_offset *offset = vdata->offset;
                    struct timens_offset monotonic = offset_from_ts(ns->offsets.monotonic);
                    struct timens_offset boottime = offset_from_ts(ns->offsets.boottime) {
                        struct timens_offset ret = {
                            .sec = off.tv_sec;
                            .nsec = off.tv_nsec;
                        };
                        return ret;
                    }

                    vdata->seq                      = 1;
                    vdata->clock_mode               = VDSO_CLOCKMODE_TIMENS;
                    offset[CLOCK_MONOTONIC]         = monotonic;
                    offset[CLOCK_MONOTONIC_RAW]     = monotonic;
                    offset[CLOCK_MONOTONIC_COARSE]  = monotonic;
                    offset[CLOCK_BOOTTIME]          = boottime;
                    offset[CLOCK_BOOTTIME_ALARM]    = boottime;
                }
            }
        }

        vdso_join_timens(tsk, ns) {
            struct mm_struct *mm = task->mm;
            struct vm_area_struct *vma;
            VMA_ITERATOR(vmi, mm, 0);

            /* unmap its vvar data for the old namespace */
            for_each_vma(vmi, vma) {
                ret = vma_is_special_mapping(vma, vdso_info[VDSO_ABI_AA64].dm) {
                    return vma->vm_private_data == sm &&
                    (vma->vm_ops == &special_mapping_vmops ||
                    vma->vm_ops == &legacy_special_mapping_vmops);
                }
                if (ret) {
                    /* remove user pages in a given range */
                    zap_vma_pages(vma);
                }
            }
        }
    }
}
```

# docker

* [sparkdev](https://www.cnblogs.com/sparkdev/category/927855.html)
    * [Docker 镜像](https://www.cnblogs.com/sparkdev/p/8901728.html) - [镜像之进阶篇 ](https://www.cnblogs.com/sparkdev/p/9092082.html) - [镜像之存储管理](https://www.cnblogs.com/sparkdev/p/9121188.html)
    * [Containerd 简介](https://www.cnblogs.com/sparkdev/p/9063042.html)
    * [Docker 网络之理解 bridge 驱动](https://www.cnblogs.com/sparkdev/p/9217310.html) - [网络之进阶篇](https://www.cnblogs.com/sparkdev/p/9198109.html)
    * [Docker Compose 简介](https://www.cnblogs.com/sparkdev/p/9753793.html)
    * [原理](https://www.cnblogs.com/sparkdev/p/9787915.html) - [进阶篇](https://www.cnblogs.com/sparkdev/p/9803554.html) - [引用环境变量](https://www.cnblogs.com/sparkdev/p/9826520.html)
    * [隔离 docker 容器中的用户](https://www.cnblogs.com/sparkdev/p/9614326.html)

## overly

![](../images/kernel/docker-overlay.png)

```sh
mount -t overlay overlay -o lowerdir=A:B:C,upperdir=C,workdir=worker /tmp/test
```
* [trap_init](#trap_init)
    * [do_trap](#do_trap)
        * [divide_error entry_32](#divide_error_entry32)
        * [divide_error entry_64](#divide_error_entry64)
* [init_IRQ](#init_irq)
    * [DECLARE_IDTENTRY](#DECLARE_IDTENTRY)
    * [DEFINE_IDTENTRY](#DEFINE_IDTENTRY)
    * [asm_common_interrupt](#asm_common_interrupt)
    * [set_intr_gate](#set_intr_gate)
    * [hardirq_stack_ptr](#hardirq_stack_ptr)
    * [irqentry_exit](#irqentry_exit)
* [E.g, do_page_fault](#do_page_fault)
* [request_irq](#request_irq)
* [softirq](#softirq)
    * [ksoftirqd](#ksoftirqd)
    * [run_ksoftirqd](#run_ksoftirqd)
    * [open_softirq](#open_softirq)
    * [raise_softirq_irqoff](#raise_softirq_irqoff)
* [tasklet](#tasklet)
    * [open TASKLET_SOFTIRQ](#open-tasklet_softirq)
    * [tasklet_schedule](#tasklet_schedule)
* [call graph](#call-graph-intr)

<img src='../images/kernel/io-irq.svg' style='max-height:850px'/>

---

<img src='../images/kernel/io-interrupt-vector.svg' style='max-height:850px'/>

---

<img src='../images/ulk/4.2-gate-descriptors.png' style='max-height:850px'/>

* [Interrupt Descriptor Table](https://wiki.osdev.org/Interrupt_Descriptor_Table)

How interrupt happens:
 * 1. extern dev sends physical interrupt to interrupt controller
 * 2. interrupt controller converts interrupt signal to interrupt vector, sends it to each cpu
 * 3. cpu call IRQ hanlder according to interrupt vector
 * 4. IRQ handler converts interrupt vector to abstract interrupt signal, handle it with irq_handler_t


arch/x86/include/asm/irq_vectors.h

Linux IRQ vector layout.
* There are 256 IDT entries (per CPU - each entry is 8 bytes) which can be defined by Linux. They are used as a jump table by the CPU when a given vector is triggered - by a CPU-external, CPU-internal or software-triggered event.
* Linux sets the kernel code address each entry jumps to early during bootup, and never changes them. This is the general layout of the IDT entries:
  *  `Vectors [0, 31]` : system traps and exceptions - hardcoded events
  *  `Vectors [32, 127]` : device interrupts
  *  `Vector  [128]` : legacy int80 syscall interface
  *  `Vectors [129, INVALIDATE_TLB_VECTOR_START-1]` except 204 : device interrupts
  *  `Vectors [INVALIDATE_TLB_VECTOR_START, 255]` : special interrupts
* 64-bit x86 has per CPU IDT tables, 32-bit has one shared IDT table.
* This file enumerates the exact layout of them:

```c
#define FIRST_EXTERNAL_VECTOR   0x20  /* 32 */
#define IA32_SYSCALL_VECTOR     0x80  /* 128 */
#define NR_VECTORS              256
#define FIRST_SYSTEM_VECTOR     NR_VECTORS
```

```c
/* arch/x86/kernel/traps.c, per cpu */
struct gate_struct {
  u16    offset_low;      /* system irq hanlder addr */
  u16    segment;         /* KERNEL_CS */
  struct idt_bits  bits;
  u16    offset_middle;   /* addr >> 16 */
#ifdef CONFIG_X86_64
  u32    offset_high;     /* adr >> 32 */
  u32    reserved;        /* 0 */
#endif
};

struct idt_bits {
  u16 ist   : 3,   /* Interrupt Stack Table, DEFAULT_STACK, KERNEK_STACK */
      zero  : 5,
      type  : 5,   /* GATE_{INTERRUPT, TRAP, CALL, TASK} */
      dpl   : 2,   /* DPL (Descriptor privilege level), DLP0, DLP3 */
                   /* RPL (Requested privilege level) */
      p     : 1;   /* Present bit */
};

enum {
  GATE_INTERRUPT = 0xE,
  GATE_TRAP = 0xF,
  GATE_CALL = 0xC,
  GATE_TASK = 0x5,
};

struct idt_data {
  unsigned int vector;
  unsigned int segment;
  struct idt_bits bits;
  const void *addr;
};

/* interrupt descriptor table */
struct gate_desc idt_table[NR_VECTORS] __page_aligned_bss;

void start_kernel() {
  trap_init();
  init_IRQ();
  softirq_init();
}
```

# trap_init
```c
/* arch/x86/kernel/traps.c */
void __init trap_init(void)
{
  /* Init cpu_entry_area before IST entries are set up */
  setup_cpu_entry_areas();

  idt_setup_traps();

  cea_set_pte(CPU_ENTRY_AREA_RO_IDT_VADDR, __pa_symbol(idt_table), PAGE_KERNEL_RO);
  idt_descr.address = CPU_ENTRY_AREA_RO_IDT;

  cpu_init();

  idt_setup_ist_traps();

  x86_init.irqs.trap_init();

  idt_setup_debugidt_traps();
}

/* arch/x86/include/asm/trapnr.h */
#define X86_TRAP_DE     0  /* Divide-by-zero */
#define X86_TRAP_DB     1  /* Debug */
#define X86_TRAP_NMI    2  /* Non-maskable Interrupt */
#define X86_TRAP_BP     3  /* Breakpoint */
#define X86_TRAP_OF     4  /* Overflow */
#define X86_TRAP_BR     5  /* Bound Range Exceeded */
#define X86_TRAP_UD     6  /* Invalid Opcode */
#define X86_TRAP_NM     7  /* Device Not Available */
#define X86_TRAP_DF     8  /* Double Fault */
#define X86_TRAP_OLD_MF     9  /* Coprocessor Segment Overrun */
#define X86_TRAP_TS    10  /* Invalid TSS */
#define X86_TRAP_NP    11  /* Segment Not Present */
#define X86_TRAP_SS    12  /* Stack Segment Fault */
#define X86_TRAP_GP    13  /* General Protection Fault */
#define X86_TRAP_PF    14  /* Page Fault */
#define X86_TRAP_SPURIOUS  15  /* Spurious Interrupt */
#define X86_TRAP_MF    16  /* x87 Floating-Point Exception */
#define X86_TRAP_AC    17  /* Alignment Check */
#define X86_TRAP_MC    18  /* Machine Check */
#define X86_TRAP_XF    19  /* SIMD Floating-Point Exception */
#define X86_TRAP_VE    20  /* Virtualization Exception */
#define X86_TRAP_CP    21  /* Control Protection Exception */
#define X86_TRAP_VC    29  /* VMM Communication Exception */
#define X86_TRAP_IRET  32  /* IRET Exception */

static const __initconst struct idt_data def_idts[] = {
  INTG(X86_TRAP_DE,    asm_exc_divide_error),
  INTG(X86_TRAP_NP,    asm_exc_segment_not_present),
  INTG(X86_TRAP_SS,    asm_exc_stack_segment),
  INTG(X86_TRAP_GP,    asm_exc_general_protection),
  INTG(X86_TRAP_SPURIOUS,    asm_exc_spurious_interrupt_bug),
};

DECLARE_IDTENTRY_ERRORCODE(X86_TRAP_SS, exc_stack_segment);

#define DPL0    0x0
#define DPL3    0x3

#define DEFAULT_STACK  0

/* Interrupt gate */
#define INTG(_vector, _addr) \
  G(_vector, _addr, DEFAULT_STACK, GATE_INTERRUPT, DPL0, __KERNEL_CS)

/* System interrupt gate */
#define SYSG(_vector, _addr) \
  G(_vector, _addr, DEFAULT_STACK, GATE_INTERRUPT, DPL3, __KERNEL_CS)

/* Interrupt gate with interrupt stack */
#define ISTG(_vector, _addr, _ist) \
  G(_vector, _addr, _ist, GATE_INTERRUPT, DPL0, __KERNEL_CS)

/* System interrupt gate with interrupt stack */
#define SISTG(_vector, _addr, _ist) \
  G(_vector, _addr, _ist, GATE_INTERRUPT, DPL3, __KERNEL_CS)

/* Task gate */
#define TSKG(_vector, _gdt) \
  G(_vector, NULL, DEFAULT_STACK, GATE_TASK, DPL0, _gdt << 3)

#define G(_vector, _addr, _ist, _type, _dpl, _segment) \
{ \
  .vector     = _vector, \
  .bits.ist   = _ist, \
  .bits.type  = _type, \
  .bits.dpl   = _dpl, \
  .bits.p     = 1, \
  .addr       = _addr, \
  .segment    = _segment, \
}

void __init idt_setup_traps(void)
{
  idt_setup_from_table(idt_table, def_idts, ARRAY_SIZE(def_idts), true);
}

void idt_setup_from_table(gate_desc *idt, const struct idt_data *t, int size, bool sys)
{
  gate_desc desc;

  for (; size > 0; t++, size--) {
    idt_init_desc(&desc, t);
    write_idt_entry(idt, t->vector, &desc);
    if (sys)
      set_bit(t->vector, system_vectors);
  }
}

void idt_init_desc(gate_desc *gate, const struct idt_data *d)
{
  unsigned long addr    = (unsigned long) d->addr;

  gate->offset_low      = (u16) addr;
  gate->segment         = (u16) d->segment;
  gate->bits            = d->bits;
  gate->offset_middle   = (u16) (addr >> 16);
#ifdef CONFIG_X86_64
  gate->offset_high     = (u32) (addr >> 32);
  gate->reserved        = 0;
#endif
}

#define write_idt_entry(dt, entry, g)  native_write_idt_entry(dt, entry, g)

void native_write_idt_entry(gate_desc *idt, int entry, const gate_desc *gate)
{
  memcpy(&idt[entry], gate, sizeof(*gate));
}

void __init idt_setup_ist_traps(void)
{
  idt_setup_from_table(idt_table, ist_idts, ARRAY_SIZE(ist_idts), true);
}

static const __initconst struct idt_data ist_idts[] = {
  ISTG(X86_TRAP_DB, debug, DEBUG_STACK),
  ISTG(X86_TRAP_NMI, nmi, NMI_STACK),
  ISTG(X86_TRAP_DF, double_fault, DOUBLEFAULT_STACK),
#ifdef CONFIG_X86_MCE
  ISTG(X86_TRAP_MC, &machine_check, MCE_STACK),
#endif
};
```

## do_trap

The C functions that implement exception handlers always consist of the prefix do_ followed by the handler name. Most of these functions invoke the **do_trap**() function to store the hardware error code and the exception vector in the process descriptor of current, and then send a suitable signal to that process:
```c
current->thread.error_code = error_code;
current->thread.trap_no = vector;
force_sig_info(signr, info ?: SEND_SIG_PRIV, tsk);
```

```c
/* arch/x86/include/asm/idtentry.h */
#define DECLARE_IDTENTRY(vector, func) \
  asmlinkage void asm_##func(void); \
  asmlinkage void xen_asm_##func(void); \
  __visible void func(struct pt_regs *regs)

#define DEFINE_IDTENTRY(func) \
static __always_inline void __##func(struct pt_regs *regs); \
 \
__visible noinstr void func(struct pt_regs *regs) \
{ \
  irqentry_state_t state = irqentry_enter(regs); \
 \
  instrumentation_begin(); \
  __##func (regs); \
  instrumentation_end(); \
  irqentry_exit(regs, state); \
} \
 \
static __always_inline void __##func(struct pt_regs *regs)

irqentry_state_t irqentry_enter(struct pt_regs *regs)
{
  irqentry_state_t ret = {
    .exit_rcu = false,
  };

  if (user_mode(regs)) {
    irqentry_enter_from_user_mode(regs);
    return ret;
  }

  if (!IS_ENABLED(CONFIG_TINY_RCU) && is_idle_task(current)) {
    lockdep_hardirqs_off(CALLER_ADDR0);
    ct_irq_enter();
    instrumentation_begin();
    trace_hardirqs_off_finish();
    instrumentation_end();

    ret.exit_rcu = true;
    return ret;
  }

  /* If RCU is watching then RCU only wants to check whether it needs
   * to restart the tick in NOHZ mode. rcu_irq_enter_check_tick()
   * already contains a warning when RCU is not watching, so no point
   * in having another one here. */
  lockdep_hardirqs_off(CALLER_ADDR0);
  instrumentation_begin();
  rcu_irq_enter_check_tick();
  trace_hardirqs_off_finish();
  instrumentation_end();

  return ret;
}


DECLARE_IDTENTRY(X86_TRAP_DE,    exc_divide_error);
DECLARE_IDTENTRY(X86_TRAP_OF,    exc_overflow);
DECLARE_IDTENTRY(X86_TRAP_BR,    exc_bounds);
DECLARE_IDTENTRY(X86_TRAP_NM,    exc_device_not_available);
DECLARE_IDTENTRY(X86_TRAP_OLD_MF,  exc_coproc_segment_overrun);
DECLARE_IDTENTRY(X86_TRAP_SPURIOUS,  exc_spurious_interrupt_bug);
DECLARE_IDTENTRY(X86_TRAP_MF,    exc_coprocessor_error);
DECLARE_IDTENTRY(X86_TRAP_XF,    exc_simd_coprocessor_error);


DEFINE_IDTENTRY(exc_divide_error)
{
  do_error_trap(regs, 0, "divide error", X86_TRAP_DE, SIGFPE,
          FPE_INTDIV, error_get_trap_addr(regs));
}

void do_error_trap(struct pt_regs *regs, long error_code, char *str,
  unsigned long trapnr, int signr, int sicode, void __user *addr)
{
  if (notify_die(DIE_TRAP, str, regs, error_code, trapnr, signr) != NOTIFY_STOP) {
    cond_local_irq_enable(regs);
    do_trap(trapnr, signr, str, regs, error_code, sicode, addr);
    cond_local_irq_disable(regs);
  }
}

static void
do_trap(int trapnr, int signr, char *str, struct pt_regs *regs,
  long error_code, int sicode, void __user *addr)
{
  struct task_struct *tsk = current;

  if (!do_trap_no_signal(tsk, trapnr, str, regs, error_code))
    return;

  show_signal(tsk, signr, "trap ", str, regs, error_code);

  if (!sicode)
    force_sig(signr);
  else
    force_sig_fault(signr, sicode, addr);
}

DECLARE_IDTENTRY_ERRORCODE(X86_TRAP_SS, exc_stack_segment);
```

# init_IRQ
```c
void start_kernel(void)
{
  early_irq_init();
  init_IRQ();
}

static RADIX_TREE(irq_desc_tree, GFP_KERNEL);

struct irq_desc *irq_to_desc(unsigned int irq)
{
  return radix_tree_lookup(&irq_desc_tree, irq);
}

int __init early_irq_init(void)
{
  int count, i, node = first_online_node;
  struct irq_desc *desc;

  init_irq_default_affinity();

  desc = irq_desc;
  count = ARRAY_SIZE(irq_desc);

  for (i = 0; i < count; i++) {
    desc[i].kstat_irqs = alloc_percpu(unsigned int);
    alloc_masks(&desc[i], node);
    raw_spin_lock_init(&desc[i].lock);
    lockdep_set_class(&desc[i].lock, &irq_desc_lock_class);
    mutex_init(&desc[i].request_mutex);
    desc_set_defaults(i, &desc[i], node, NULL, NULL);
  }
  return arch_early_irq_init();
}

void desc_set_defaults(unsigned int irq, struct irq_desc *desc, int node,
            const struct cpumask *affinity, struct module *owner)
{
  int cpu;

  desc->irq_common_data.handler_data = NULL;
  desc->irq_common_data.msi_desc = NULL;

  desc->irq_data.common = &desc->irq_common_data;
  desc->irq_data.irq = irq;
  desc->irq_data.chip = &no_irq_chip;
  desc->irq_data.chip_data = NULL;
  irq_settings_clr_and_set(desc, ~0, _IRQ_DEFAULT_INIT_FLAGS);
  irqd_set(&desc->irq_data, IRQD_IRQ_DISABLED);
  irqd_set(&desc->irq_data, IRQD_IRQ_MASKED);
  desc->handle_irq = handle_bad_irq;
  desc->depth = 1;
  desc->irq_count = 0;
  desc->irqs_unhandled = 0;
  desc->tot_count = 0;
  desc->name = NULL;
  desc->owner = owner;
  for_each_possible_cpu(cpu)
    *per_cpu_ptr(desc->kstat_irqs, cpu) = 0;
  desc_smp_init(desc, node, affinity);
}

int __init arch_early_irq_init(void)
{
  struct fwnode_handle *fn;

  fn = irq_domain_alloc_named_fwnode("VECTOR");
  BUG_ON(!fn);
  x86_vector_domain = irq_domain_create_tree(fn, &x86_vector_domain_ops, NULL);
  BUG_ON(x86_vector_domain == NULL);
  irq_set_default_host(x86_vector_domain);

  arch_init_msi_domain(x86_vector_domain);

  BUG_ON(!alloc_cpumask_var(&vector_searchmask, GFP_KERNEL));

  /* Allocate the vector matrix allocator data structure and limit the
   * search area. */
  vector_matrix = irq_alloc_matrix(NR_VECTORS, FIRST_EXTERNAL_VECTOR,
           FIRST_SYSTEM_VECTOR);
  BUG_ON(!vector_matrix);

  return arch_early_ioapic_init();
}
```

```c
void __init init_IRQ(void)
{
  int i;

  for (i = 0; i < nr_legacy_irqs(); i++)
    per_cpu(vector_irq, 0)[ISA_IRQ_VECTOR(i)] = irq_to_desc(i);

  irq_init_percpu_irqstack(smp_processor_id());

  x86_init.irqs.intr_init(); /* native_init_IRQ */
}

/* Vectors 0x30-0x3f are used for ISA interrupts.
 * round up to the next 16-vector boundar */
#define ISA_IRQ_VECTOR(irq) (((FIRST_EXTERNAL_VECTOR + 16) & ~15) + irq)

/* after kernel called trap_init(), it invokes init_IRQ() to init other dev interrupt */
void __init native_init_IRQ(void)
{
  /* Execute any quirks before the call gates are initialised: */
  x86_init.irqs.pre_vector_init();

  idt_setup_apic_and_irq_gates();
  lapic_assign_system_vectors();

  if (!acpi_ioapic && !of_ioapic && nr_legacy_irqs())
    setup_irq(2, &irq2);

  irq_ctx_init(smp_processor_id());
}

/* idt_setup_apic_and_irq_gates - Setup APIC/SMP and normal interrupt gates */
void __init idt_setup_apic_and_irq_gates(void)
{
  int i = FIRST_EXTERNAL_VECTOR;
  void *entry;

  idt_setup_from_table(idt_table, apic_idts, ARRAY_SIZE(apic_idts), true);

  for_each_clear_bit_from(i, system_vectors, FIRST_SYSTEM_VECTOR) {
    entry = irq_entries_start + IDT_ALIGN * (i - FIRST_EXTERNAL_VECTOR);
    set_intr_gate(i, entry);
  }

#ifdef CONFIG_X86_LOCAL_APIC
  for_each_clear_bit_from(i, system_vectors, NR_VECTORS) {
    /* Don't set the non assigned system vectors in the
     * system_vectors bitmap. Otherwise they show up in
     * /proc/interrupts. */
    entry = spurious_entries_start + IDT_ALIGN * (i - FIRST_SYSTEM_VECTOR);
    set_intr_gate(i, entry);
  }
#endif
  /* Map IDT into CPU entry area and reload it. */
  idt_map_in_cea();
  load_idt(&idt_descr);

  /* Make the IDT table read only */
  set_memory_ro((unsigned long)&idt_table, 1);

  idt_setup_done = true;
}

static const __initconst struct idt_data apic_idts[] = {
  INTG(RESCHEDULE_VECTOR,    reschedule_interrupt),
  INTG(CALL_FUNCTION_VECTOR,  call_function_interrupt),
  INTG(CALL_FUNCTION_SINGLE_VECTOR, call_function_single_interrupt),
  INTG(IRQ_MOVE_CLEANUP_VECTOR,  irq_move_cleanup_interrupt),
  INTG(REBOOT_VECTOR,    reboot_interrupt),
};

static const __initconst struct idt_data early_pf_idts[] = {
  INTG(X86_TRAP_PF,    asm_exc_page_fault),
};
```
## DECLARE_IDTENTRY
```c
/* arch/x86/include/asm/idtentry.h
 * The ASM variants for DECLARE_IDTENTRY*() which emit the ASM entry stubs */
#define DECLARE_IDTENTRY(vector, func)          \
  idtentry vector asm_##func func has_error_code=0

#define DECLARE_IDTENTRY_ERRORCODE(vector, func)      \
  idtentry vector asm_##func func has_error_code=1

/* Entries for common/spurious (device) interrupts */
#define DECLARE_IDTENTRY_IRQ(vector, func) \
  idtentry_irq vector func

/* System vector entries */
#define DECLARE_IDTENTRY_SYSVEC(vector, func)        \
  idtentry_sysvec vector func

/* Interrupt entry/exit.
 *
 + The interrupt stubs push (vector) onto the stack, which is the error_code
 * position of idtentry exceptions, and jump to one of the two idtentry points
 * (common/spurious).
 *
 * common_interrupt is a hotpath, align it to a cache line */
.macro idtentry_irq vector cfunc
  .p2align CONFIG_X86_L1_CACHE_SHIFT
  idtentry \vector asm_\cfunc \cfunc has_error_code=1
.endm


/**
 * idtentry - Macro to generate entry stubs for simple IDT entries
 * @vector:    Vector number
 * @asmsym:    ASM symbol for the entry point
 * @cfunc:    C function to be called
 * @has_error_code:  Hardware pushed error code on stack
 *
 * The macro emits code to set up the kernel context for straight forward
 * and simple IDT entries. No IST stack, no paranoid entry checks. */
.macro idtentry vector asmsym cfunc has_error_code:req
SYM_CODE_START(\asmsym)
  UNWIND_HINT_IRET_REGS offset=\has_error_code*8
  ENDBR
  ASM_CLAC
  cld

  .if \has_error_code == 0
    pushq  $-1      /* ORIG_RAX: no syscall to restart */
  .endif

  .if \vector == X86_TRAP_BP
    /* If coming from kernel space, create a 6-word gap to allow the
     * int3 handler to emulate a call instruction. */
    testb  $3, CS-ORIG_RAX(%rsp)
    jnz  .Lfrom_usermode_no_gap_\@
    .rept  6
    pushq  5*8(%rsp)
    .endr
    UNWIND_HINT_IRET_REGS offset=8
.Lfrom_usermode_no_gap_\@:
  .endif

  idtentry_body \cfunc \has_error_code

_ASM_NOKPROBE(\asmsym)
SYM_CODE_END(\asmsym)
.endm

/**
 * idtentry_body - Macro to emit code calling the C function
 * @cfunc:    C function to be called
 * @has_error_code:  Hardware pushed error code on stack */
.macro idtentry_body cfunc has_error_code:req

  /* Call error_entry() and switch to the task stack if from userspace.
   *
   * When in XENPV, it is already in the task stack, and it can't fault
   * for native_iret() nor native_load_gs_index() since XENPV uses its
   * own pvops for IRET and load_gs_index().  And it doesn't need to
   * switch the CR3.  So it can skip invoking error_entry(). */
  ALTERNATIVE "call error_entry; movq %rax, %rsp", \
    "call xen_error_entry", X86_FEATURE_XENPV

  ENCODE_FRAME_POINTER
  UNWIND_HINT_REGS

  movq  %rsp, %rdi      /* pt_regs pointer into 1st argument*/

  .if \has_error_code == 1
    movq  ORIG_RAX(%rsp), %rsi  /* get error code into 2nd argument*/
    movq  $-1, ORIG_RAX(%rsp)  /* no syscall to restart */
  .endif

  call \cfunc

  /* For some configurations \cfunc ends up being a noreturn. */
  REACHABLE

  jmp  error_return
.endm

SYM_CODE_START_LOCAL(error_return)
 UNWIND_HINT_REGS
 DEBUG_ENTRY_ASSERT_IRQS_OFF
 testb $3, CS(%rsp)
 jz restore_regs_and_return_to_kernel
 jmp swapgs_restore_regs_and_return_to_usermode
SYM_CODE_END(error_return)

SYM_CODE_START_LOCAL(common_interrupt_return)
SYM_INNER_LABEL(swapgs_restore_regs_and_return_to_usermode, SYM_L_GLOBAL)
  IBRS_EXIT

  POP_REGS pop_rdi=0

  /* The stack is now user RDI, orig_ax, RIP, CS, EFLAGS, RSP, SS.
   * Save old stack pointer and switch to trampoline stack. */
  movq  %rsp, %rdi
  movq  PER_CPU_VAR(cpu_tss_rw + TSS_sp0), %rsp
  UNWIND_HINT_EMPTY

  /* Copy the IRET frame to the trampoline stack. */
  pushq  6*8(%rdi)  /* SS */
  pushq  5*8(%rdi)  /* RSP */
  pushq  4*8(%rdi)  /* EFLAGS */
  pushq  3*8(%rdi)  /* CS */
  pushq  2*8(%rdi)  /* RIP */

  /* Push user RDI on the trampoline stack. */
  pushq  (%rdi)

  /* We are on the trampoline stack.  All regs except RDI are live.
   * We can do future final exit work right here. */
  STACKLEAK_ERASE_NOCLOBBER

  SWITCH_TO_USER_CR3_STACK scratch_reg=%rdi

  /* Restore RDI. */
  popq  %rdi
  swapgs
  jmp  .Lnative_iret

.Lnative_iret:
  UNWIND_HINT_IRET_REGS
  /* Are we returning to a stack segment from the LDT?  Note: in
   * 64-bit mode SS:RSP on the exception stack is always valid. */
#ifdef CONFIG_X86_ESPFIX64
  testb  $4, (SS-RIP)(%rsp)
  jnz  native_irq_return_ldt
#endif

SYM_INNER_LABEL(native_irq_return_iret, SYM_L_GLOBAL)
  ANNOTATE_NOENDBR // exc_double_fault
  /* This may fault.  Non-paranoid faults on return to userspace are
   * handled by fixup_bad_iret.  These include #SS, #GP, and #NP.
   * Double-faults due to espfix64 are handled in exc_double_fault.
   * Other faults here are fatal. */
  iretq

#ifdef CONFIG_X86_ESPFIX64
native_irq_return_ldt:
  /* We are running with user GSBASE.  All GPRs contain their user
   * values.  We have a percpu ESPFIX stack that is eight slots
   * long (see ESPFIX_STACK_SIZE).  espfix_waddr points to the bottom
   * of the ESPFIX stack.
   *
   * We clobber RAX and RDI in this code.  We stash RDI on the
   * normal stack and RAX on the ESPFIX stack.
   *
   * The ESPFIX stack layout we set up looks like this:
   *
   * --- top of ESPFIX stack ---
   * SS
   * RSP
   * RFLAGS
   * CS
   * RIP  <-- RSP points here when we're done
   * RAX  <-- espfix_waddr points here
   * --- bottom of ESPFIX stack --- */

  pushq  %rdi        /* Stash user RDI */
  swapgs          /* to kernel GS */
  SWITCH_TO_KERNEL_CR3 scratch_reg=%rdi  /* to kernel CR3 */

  movq  PER_CPU_VAR(espfix_waddr), %rdi
  movq  %rax, (0*8)(%rdi)    /* user RAX */

  movq  (1*8)(%rsp), %rax    /* user RIP */
  movq  %rax, (1*8)(%rdi)

  movq  (2*8)(%rsp), %rax    /* user CS */
  movq  %rax, (2*8)(%rdi)

  movq  (3*8)(%rsp), %rax    /* user RFLAGS */
  movq  %rax, (3*8)(%rdi)

  movq  (5*8)(%rsp), %rax    /* user SS */
  movq  %rax, (5*8)(%rdi)

  movq  (4*8)(%rsp), %rax    /* user RSP */
  movq  %rax, (4*8)(%rdi)
  /* Now RAX == RSP. */

  andl  $0xffff0000, %eax    /* RAX = (RSP & 0xffff0000) */

  /* espfix_stack[31:16] == 0.  The page tables are set up such that
   * (espfix_stack | (X & 0xffff0000)) points to a read-only alias of
   * espfix_waddr for any X.  That is, there are 65536 RO aliases of
   * the same page.  Set up RSP so that RSP[31:16] contains the
   * respective 16 bits of the /userspace/ RSP and RSP nonetheless
   * still points to an RO alias of the ESPFIX stack. */
  orq  PER_CPU_VAR(espfix_stack), %rax

  SWITCH_TO_USER_CR3_STACK scratch_reg=%rdi
  swapgs            /* to user GS */
  popq  %rdi        /* Restore user RDI */

  movq  %rax, %rsp
  UNWIND_HINT_IRET_REGS offset=8

  /* At this point, we cannot write to the stack any more, but we can
   * still read. */
  popq  %rax        /* Restore user RAX */

  /* RSP now points to an ordinary IRET frame, except that the page
   * is read-only and RSP[31:16] are preloaded with the userspace
   * values.  We can now IRET back to userspace. */
  jmp  native_irq_return_iret
#endif
SYM_CODE_END(common_interrupt_return)
_ASM_NOKPROBE(common_interrupt_return)
```

## DEFINE_IDTENTRY
```c
#define DEFINE_IDTENTRY(func)
#define DEFINE_IDTENTRY_ERRORCODE(func)
#define DEFINE_IDTENTRY_RAW(func)
#define DEFINE_IDTENTRY_RAW_ERRORCODE(func)
#define DEFINE_IDTENTRY_IRQ(func)
#define DEFINE_IDTENTRY_SYSVEC(func)
#define DEFINE_IDTENTRY_SYSVEC_SIMPLE(func)
#define DEFINE_IDTENTRY_IST(func)
#define DEFINE_IDTENTRY_NOIST(func)
#define DEFINE_IDTENTRY_DF(func)
#define DEFINE_IDTENTRY_VC_KERNEL(func)
#define DEFINE_IDTENTRY_VC_USER(func)
#define DEFINE_IDTENTRY_DF(func)
#define DEFINE_IDTENTRY(func)
```

```c
/* arch/x86/include/asm/hw_irq.h */
extern char irq_entries_start[];

/* set `FIRST_SYSTEM_VECTOR - FIRST_EXTERNAL_VECTOR` handler to do_IRQ
 * irq_entries_start defined in arch\x86\entry\entry_{32, 64}

 * Build the entry stubs with some assembler magic.
 * We pack 1 stub into every 8-byte block. */
#define FIRST_EXTERNAL_VECTOR 0x20        /* 32 */
#define FIRST_SYSTEM_VECTOR   NR_VECTORS  /* 256 */

SYM_CODE_START(irq_entries_start)
  vector=FIRST_EXTERNAL_VECTOR // 32
  .rept NR_EXTERNAL_VECTORS    // 256 - 32
  UNWIND_HINT_IRET_REGS
0 :
  ENDBR
  .byte  0x6a, vector
  jmp  asm_common_interrupt
  /* Ensure that the above is IDT_ALIGN bytes max */
  .fill 0b + IDT_ALIGN - ., 1, 0xcc
  vector = vector+1
    .endr
SYM_CODE_END(irq_entries_start)

#define DEFINE_IDTENTRY_IRQ(func) \
static void __##func(struct pt_regs *regs, u32 vector); \
 \
__visible noinstr void func(struct pt_regs *regs, unsigned long error_code) \
{ \
  irqentry_state_t state = irqentry_enter(regs); \
  u32 vector = (u32)(u8)error_code; \
 \
  instrumentation_begin(); \
  kvm_set_cpu_l1tf_flush_l1d(); \
  run_irq_on_irqstack_cond(__##func, regs, vector); \
  instrumentation_end(); \
  irqentry_exit(regs, state); \
}
 \
static noinline void __##func(struct pt_regs *regs, u32 vector)

#define run_irq_on_irqstack_cond(func, regs, vector) \
{ \
  assert_function_type(func, void (*)(struct pt_regs *, u32)); \
  assert_arg_type(regs, struct pt_regs *); \
  assert_arg_type(vector, u32); \
 \
  call_on_irqstack_cond(func, regs, ASM_CALL_IRQ, \
            IRQ_CONSTRAINTS, regs, vector); \
}

/* Macro to invoke system vector and device interrupt C handlers. */
#define call_on_irqstack_cond(func, regs, asm_call, constr, c_args...) \
{ \
  /* \
   * User mode entry and interrupt on the irq stack do not \
   * switch stacks. If from user mode the task stack is empty. \ */ \
  if (user_mode(regs) || __this_cpu_read(hardirq_stack_inuse)) { \
    irq_enter_rcu(); \
    func(c_args); \
    irq_exit_rcu(); \
  } else { \
    /* \
     * Mark the irq stack inuse _before_ and unmark _after_ \
     * switching stacks. Interrupts are disabled in both \
     * places. Invoke the stack switch macro with the call \
     * sequence which matches the above direct invocation. \ */ \
    __this_cpu_write(hardirq_stack_inuse, true); \
    call_on_irqstack(func, asm_call, constr); \
    __this_cpu_write(hardirq_stack_inuse, false); \
  } \
}

#define call_on_irqstack(func, asm_call, argconstr...) \
  call_on_stack(__this_cpu_read(hardirq_stack_ptr), func, asm_call, argconstr)

/* Macro to inline switching to an interrupt stack and invoking function
 * calls from there. The following rules apply:
 *   1. Write the stack pointer into the top most place of the irq
 *  stack. This ensures that the various unwinders can link back to the
 *  original stack.
 *
 *   2. Switch the stack pointer to the top of the irq stack.
 *
 *   3. Invoke whatever needs to be done (@asm_call argument)
 *
 *   4. Pop the original stack pointer from the top of the irq stack
 *  which brings it back to the original stack where it left off. */
#define call_on_stack(stack, func, asm_call, argconstr...) \
{ \
  register void *tos asm("r11"); \
 \
  tos = ((void *)(stack)); \
 \
  asm_inline volatile( \
  "movq  %%rsp, (%[tos]) \n" \
  "movq  %[tos], %%rsp \n" \
 \
  asm_call \
 \
  "popq  %%rsp \n" \
 \
  : "+r" (tos), ASM_CALL_CONSTRAINT \
  : [__func] "i" (func), [tos] "r" (tos) argconstr \
  : "cc", "rax", "rcx", "rdx", "rsi", "rdi", "r8", "r9", "r10", \
    "memory" \
  ); \
}

register unsigned long current_stack_pointer asm(_ASM_SP);
#define ASM_CALL_CONSTRAINT "+r" (current_stack_pointer)
```

## asm_common_interrupt
```c
/* Dummy trap number so the low level ASM macro vector number checks do not
 * match which results in emitting plain IDTENTRY stubs without bells and whistles. */
#define X86_TRAP_OTHER 0xFFFF

DECLARE_IDTENTRY_IRQ(X86_TRAP_OTHER,  common_interrupt);

/* common_interrupt() handles all normal device IRQ's (the special SMP
 * cross-CPU interrupts have their own entry points). */
DEFINE_IDTENTRY_IRQ(common_interrupt)
{
  struct pt_regs *old_regs = set_irq_regs(regs);
  struct irq_desc *desc;

  desc = __this_cpu_read(vector_irq[vector]);
  if (likely(!IS_ERR_OR_NULL(desc))) {
    handle_irq(desc, regs);
  } else {
    ack_APIC_irq();

    if (desc == VECTOR_UNUSED) {
      pr_emerg_ratelimited("%s: %d.%u No irq handler for vector\n",
               __func__, smp_processor_id(),
               vector);
    } else {
      __this_cpu_write(vector_irq[vector], VECTOR_UNUSED);
    }
  }

  set_irq_regs(old_regs);
}

static __always_inline void handle_irq(struct irq_desc *desc, struct pt_regs *regs)
{
  if (IS_ENABLED(CONFIG_X86_64))
    generic_handle_irq_desc(desc);
  else
    __handle_irq(desc, regs);
}

void generic_handle_irq_desc(struct irq_desc *desc)
{
  desc->handle_irq(desc);
}

irqreturn_t __handle_irq_event_percpu(struct irq_desc *desc, unsigned int *flags)
{
  irqreturn_t retval = IRQ_NONE;
  unsigned int irq = desc->irq_data.irq;
  struct irqaction *action;

  record_irq_time(desc);

  for_each_action_of_desc(desc, action) {
    irqreturn_t res;
    res = action->handler(irq, action->dev_id);
    switch (res) {
    case IRQ_WAKE_THREAD:
      __irq_wake_thread(desc, action);
    case IRQ_HANDLED:
      *flags |= action->flags;
      break;
    default:
      break;
    }
    retval |= res;
  }
  return retval;
}

void irq_exit(void)
{
#ifndef __ARCH_IRQ_EXIT_IRQS_DISABLED
  local_irq_disable();
#else
  lockdep_assert_irqs_disabled();
#endif

  account_irq_exit_time(current);
  preempt_count_sub(HARDIRQ_OFFSET);
  if (!in_interrupt() && local_softirq_pending())
    invoke_softirq();

  tick_irq_exit();
  rcu_irq_exit();
  trace_hardirq_exit(); /* must be last! */
}

static inline void invoke_softirq(void)
{
  if (ksoftirqd_running(local_softirq_pending()))
    return;

  if (!force_irqthreads) {
#ifdef CONFIG_HAVE_IRQ_EXIT_ON_IRQ_STACK
    __do_softirq();
#else
    do_softirq_own_stack();
#endif
  } else {
    wakeup_softirqd();
  }
}
```

## set_intr_gate
```c
void __init update_intr_gate(unsigned int n, const void *addr)
{
  if (WARN_ON_ONCE(!test_bit(n, system_vectors)))
    return;
  set_intr_gate(n, addr);
}

void set_intr_gate(unsigned int n, const void *addr)
{
  struct idt_data data;

  BUG_ON(n > 0xFF);

  memset(&data, 0, sizeof(data));
  data.vector = n;
  data.addr = addr;
  data.segment = __KERNEL_CS;
  data.bits.type = GATE_INTERRUPT;
  data.bits.p = 1;

  idt_setup_from_table(idt_table, &data, 1, false);
}

static void
idt_setup_from_table(gate_desc *idt, const struct idt_data *t, int size, bool sys)
{
  gate_desc desc;

  for (; size > 0; t++, size--) {
    idt_init_desc(&desc, t);
    write_idt_entry(idt, t->vector, &desc);
    if (sys)
      set_bit(t->vector, system_vectors);
  }
}
```

## hardirq_stack_ptr
```c
int irq_init_percpu_irqstack(unsigned int cpu)
{
  if (per_cpu(hardirq_stack_ptr, cpu))
    return 0;
  return map_irq_stack(cpu);
}

static int map_irq_stack(unsigned int cpu)
{
  void *va = per_cpu_ptr(&irq_stack_backing_store, cpu);

  /* Store actual TOS to avoid adjustment in the hotpath */
  per_cpu(hardirq_stack_ptr, cpu) = va + IRQ_STACK_SIZE - 8;
  return 0;
}

DEFINE_PER_CPU_PAGE_ALIGNED(struct irq_stack, irq_stack_backing_store) __visible;
DECLARE_INIT_PER_CPU(irq_stack_backing_store);

struct irq_stack {
  char    stack[IRQ_STACK_SIZE];
} __aligned(IRQ_STACK_SIZE);
```

## irqentry_exit
```c
void irqentry_exit(struct pt_regs *regs, irqentry_state_t state)
{
  lockdep_assert_irqs_disabled();

  /* Check whether this returns to user mode */
  if (user_mode(regs)) {
    irqentry_exit_to_user_mode(regs);
  } else if (!regs_irqs_disabled(regs)) {
    /*
     * If RCU was not watching on entry this needs to be done
     * carefully and needs the same ordering of lockdep/tracing
     * and RCU as the return to user mode path.
     */
    if (state.exit_rcu) {
      instrumentation_begin();
      /* Tell the tracer that IRET will enable interrupts */
      trace_hardirqs_on_prepare();
      lockdep_hardirqs_on_prepare();
      instrumentation_end();
      ct_irq_exit();
      lockdep_hardirqs_on(CALLER_ADDR0);
      return;
    }

    instrumentation_begin();
    if (IS_ENABLED(CONFIG_PREEMPTION))
      irqentry_exit_cond_resched();

    /* Covers both tracing and lockdep */
    trace_hardirqs_on();
    instrumentation_end();
  } else {
    /*
     * IRQ flags state is correct already. Just tell RCU if it
     * was not watching on entry.
     */
    if (state.exit_rcu)
      ct_irq_exit();
  }
}

void irqentry_exit_to_user_mode(struct pt_regs *regs)
{
  instrumentation_begin();
  exit_to_user_mode_prepare(regs);
  instrumentation_end();
  __exit_to_user_mode();
}

void exit_to_user_mode_prepare(struct pt_regs *regs)
{
  unsigned long ti_work = read_thread_flags();

  lockdep_assert_irqs_disabled();

  /* Flush pending rcuog wakeup before the last need_resched() check */
  tick_nohz_user_enter_prepare();

  if (unlikely(ti_work & EXIT_TO_USER_MODE_WORK))
    ti_work = exit_to_user_mode_loop(regs, ti_work);

  arch_exit_to_user_mode_prepare(regs, ti_work);

  /* Ensure that the address limit is intact and no locks are held */
  addr_limit_user_check();
  kmap_assert_nomap();
  lockdep_assert_irqs_disabled();
  lockdep_sys_exit();
}

unsigned long exit_to_user_mode_loop(struct pt_regs *regs,
              unsigned long ti_work)
{
  /*
   * Before returning to user space ensure that all pending work
   * items have been completed.
   */
  while (ti_work & EXIT_TO_USER_MODE_WORK) {

    local_irq_enable_exit_to_user(ti_work);

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

    /* Architecture specific TIF work */
    arch_exit_to_user_mode_work(regs, ti_work);

    local_irq_disable_exit_to_user();

    /* Check if any of the above work has queued a deferred wakeup */
    tick_nohz_user_enter_prepare();

    ti_work = read_thread_flags();
  }

  /* Return the latest work state for arch_exit_to_user_mode() */
  return ti_work;
}

void __exit_to_user_mode(void)
{
  instrumentation_begin();
  trace_hardirqs_on_prepare();
  lockdep_hardirqs_on_prepare();
  instrumentation_end();

  user_enter_irqoff();
  arch_exit_to_user_mode();
  lockdep_hardirqs_on(CALLER_ADDR0);
}
```

# do_page_fault

[page-fault](./linux-kernel-mem.md#page-fault)

# call-graph-intr
```c
#define DECLARE_IDTENTRY_IRQ(vector, cfunc)
    idtentry \vector asm_\cfunc \cfunc has_error_code=1

    SYM_CODE_START(\asmsym)
        call \cfunc
        jmp  error_return
    SYM_CODE_END(\asmsym)

    SYM_CODE_START_LOCAL(error_return)
    testb $3, CS(%rsp)
    jz restore_regs_and_return_to_kernel
    jmp swapgs_restore_regs_and_return_to_usermode
    SYM_CODE_END(error_return)


#define DEFINE_IDTENTRY_IRQ(func)
    void func(struct pt_regs *regs, unsigned long error_code) {
        run_irq_on_irqstack_cond(__##func, regs, vector);
            if (user_mode(regs) || __this_cpu_read(hardirq_stack_inuse)) {
                func(c_args);
            } else {
                call_on_irqstack(func, asm_call, constr);
                    call_on_stack(__this_cpu_read(hardirq_stack_ptr), func, asm_call, argconstr)
                        /* 1. Write the stack pointer into the top most place of the irq  stack.
                         * 2. Switch the stack pointer to the top of the irq stack
                         * 3. Invoke whatever needs to be done (@asm_call argument)
                         * 4. Pop the original stack pointer from the top of the irq stack */
            }
        irqentry_exit(regs, state);
            if (user_mode(regs)) {
                irqentry_exit_to_user_mode(regs);
                    exit_to_user_mode_prepare(regs);
                        if (unlikely(ti_work & EXIT_TO_USER_MODE_WORK))
                            ti_work = exit_to_user_mode_loop(regs, ti_work);
                                if (ti_work & _TIF_NEED_RESCHED)
                                    schedule();
                                if (ti_work & (_TIF_SIGPENDING | _TIF_NOTIFY_SIGNAL))
                                    arch_do_signal_or_restart(regs);
                    __exit_to_user_mode();
                        arch_exit_to_user_mode();
            }
    }

```

[Interrupts, Exceptions, and System Calls](http://www.cse.iitm.ac.in/~chester/courses/15o_os/slides/5_Interrupts.pdf)

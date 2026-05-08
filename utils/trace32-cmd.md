Commonly Used Trace32 Commands for Linux Kernel Debugging

### Task / Process Inspection
```
task.list                          ; list all tasks
task.select <task_addr>            ; select a task context
task.stack <task_addr>             ; show call stack of a task
task.state                         ; show task states summary
```

### Memory / Variable View
```
Var.View %Symbol <expr>            ; view variable with symbol info
Var.View (struct task_struct *)<addr>  ; cast and view struct
Data.Long A:<addr>                 ; read 4 bytes at address
Data.Quad A:<addr>                 ; read 8 bytes at address
Data.dump A:<addr>                 ; hex dump memory
```

### Symbol / Address Lookup
```
y.lookup <symbol_name>             ; find symbol address
y.name <addr>                      ; find symbol name at address
print %addr &<variable>            ; print address of a variable
```

### Call Stack / Backtrace
```
frame.view                         ; view current stack frame
Register                           ; show CPU registers
bt                                 ; backtrace (if script defined)
per.view                           ; view per-CPU variables
```

### Breakpoints
```
Break.Set <addr>                   ; set hardware breakpoint
Break.Set <func_name>              ; breakpoint on function entry
Break.Delete <id>                  ; delete breakpoint
Break.List                         ; list all breakpoints
```

### Execution Control
```
Go                                 ; run
Break                              ; halt
Step                               ; single step instruction
StepOver                           ; step over function call
```

### Searching Memory
```
Find A:<start>--<end> <pattern>    ; search memory for pattern
```

### CPU / SMP
```
Core.List                          ; list all CPU cores
Core.select <n>                    ; switch to CPU core n
Register /All                      ; registers of all cores
```

### Useful for Mutex / Lock Debugging
```
; View mutex owner (mask flag bits)
Var.View ((struct mutex *)<addr>)->owner

; Get task from owner (mask lower 3 bits)
Var.View (struct task_struct *)(<owner_val>&0xFFFFFFFFFFFFFFF8)

; Check task pid/comm
Var.View ((struct task_struct *)<addr>)->pid
Var.View ((struct task_struct *)<addr>)->comm
```

### Script / Macro
```
do <script.cmm>                    ; run a PRACTICE script
cd <path>                          ; change working directory
area.view                          ; view output area/log
```

### Misc
```
help <cmd>                         ; built-in help
version                            ; show T32 version
sys.view                           ; system configuration view
mmu.list                           ; list MMU mappings (useful for VA→PA)
```

> **Tip:** For Linux-aware debugging, load the Linux Awareness extension (`menu → CPU → Linux Awareness`) which enables `task.*` commands and proper `Var.View` of kernel symbols.
## Process / Task

| Command | Description |
|---|---|
| `bt` | Backtrace of current (panic) task |
| `bt <pid>` | Backtrace of a specific PID |
| `bt -a` | Backtrace of all tasks |
| `ps` | List all processes |
| `ps <name>` | Find process by name |
| `task <addr>` | Show `task_struct` at address |
| `foreach bt` | Backtrace every task |

---

## Memory / Address

| Command | Description |
|---|---|
| `vtop <vaddr>` | Virtual → physical translation |
| `ptov <paddr>` | Physical → virtual translation |
| `rd <addr> [N]` | Read N 8-byte words at address |
| `rd -s <addr>` | Read as string |
| `wr <addr> <val>` | Write value (use carefully) |
| `kmem -i` | Memory usage summary |
| `kmem -s <slab>` | Slab allocator info |
| `kmem <addr>` | Which slab/page owns this address |

---

## Structures

| Command | Description |
|---|---|
| `struct <type> <addr>` | Print struct at address |
| `struct <type> <addr> -x` | Same, in hex |
| `struct <type>` | Show struct definition/offsets |
| `struct <type>.<member> <addr>` | Print one member only |
| `p *((struct foo *)addr)` | GDB-style print |
| `whatis <symbol>` | Show type of a symbol |

---

## Symbols / Kernel Variables

| Command | Description |
|---|---|
| `sym <addr>` | Address → symbol name |
| `sym <name>` | Symbol name → address |
| `p <global_var>` | Print global variable |
| `p <global_var> -x` | Print in hex |

---

## Logs

| Command | Description |
|---|---|
| `log` | Print kernel ring buffer (dmesg) |
| `log -m` | Include timestamps |
| `dmesg` | Alias for `log` |

---

## Cgroup / Memcg specific

| Command | Description |
|---|---|
| `struct mem_cgroup <addr> -x` | Dump mem_cgroup |
| `struct mem_cgroup.memory <addr>` | Print memory counter only |
| `struct cgroup_subsys_state <addr>` | Dump css |
| `struct css_set <addr>` | Dump css_set |
| `struct task_struct.cgroups <addr>` | Get css_set from task |

---

## Stack / Registers

| Command | Description |
|---|---|
| `bt -f` | Full stack frame with locals |
| `bt -l` | Include source file/line |
| `bt -e` | Exception frames |
| `dis <addr>` | Disassemble at address |
| `dis <symbol>` | Disassemble a function |
| `iregs` | Interrupt registers at panic |

---

## Misc

| Command | Description |
|---|---|
| `files <pid>` | Open files for a process |
| `vm <pid>` | VM mappings for a process |
| `mount` | Mounted filesystems |
| `net` | Network info |
| `mod` | Loaded kernel modules |
| `help <cmd>` | Help for any command |
| `q` | Quit crash |

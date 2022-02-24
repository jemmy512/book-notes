# 3 Processes
## 3.1 Processes, Lightweight Processes, and Threads
## 3.2 Process Descriptor
## 3.3 Process Switch
### 3.3.1 Hardware Context

The set of data that must be loaded into the registers before the process resumes its execution on the CPU is called the **hardware context**. The hardware context is a sub- set of the process execution context, which includes all information needed for the process execution. In Linux, a part of the hardware context of a process is stored in the process descriptor, while the remaining part is saved in the Kernel Mode stack.

We can thus define a **process switch** as the activity consist- ing of saving the hardware context of prev and replacing it with the hardware con- text of next.

The contents of all registers used by a process in User Mode have already been saved on the Kernel Mode stack before per- forming process switching (see Chapter 4). This includes the contents of the ss and esp pair that specifies the User Mode stack pointer address.

### 3.3.2 Task State Segment

```c++
struct __attribute__ ((__packed__)) vmcb_seg {
    u16 selector;
    u16 attrib;
    u32 limit;
    u64 base;
};

struct __attribute__ ((__packed__)) vmcb_save_area {
    struct vmcb_seg es;
    struct vmcb_seg cs;
    struct vmcb_seg ss;
    struct vmcb_seg ds;
    struct vmcb_seg fs;
    struct vmcb_seg gs;
    struct vmcb_seg gdtr;
    struct vmcb_seg ldtr;
    struct vmcb_seg idtr;
    struct vmcb_seg tr;
    u8 reserved_1[43];
    u8 cpl;
    u8 reserved_2[4];
    u64 efer;
    u8 reserved_3[112];
    u64 cr4;
    u64 cr3;
    u64 cr0;
    u64 dr7;
    u64 dr6;
    u64 rflags;
    u64 rip;
    u8 reserved_4[88];
    u64 rsp;
    u8 reserved_5[24];
    u64 rax;
    u64 star;
    u64 lstar;
    u64 cstar;
    u64 sfmask;
    u64 kernel_gs_base;
    u64 sysenter_cs;
    u64 sysenter_esp;
    u64 sysenter_eip;
    u64 cr2;
    u8 reserved_6[32];
    u64 g_pat;
    u64 dbgctl;
    u64 br_from;
    u64 br_to;
    u64 last_excp_from;
    u64 last_excp_to;
};

struct __attribute__ ((__packed__)) vmcb {
    struct vmcb_control_area control;
    struct vmcb_save_area save;
};
```

#### 3.3.2.1 The thread field

Each process descriptor includes a field called thread of type **thread_struct**, in which the kernel saves the hardware context whenever the process is being switched out.

This data structure includes fields for most of the CPU regis- ters, except the general-purpose registers such as eax, ebx, etc., which are stored in the Kernel Mode stack.

### 3.3.3 Performing the Process Switch
Essentially, every process switch consists of two steps:
1. Switching the Page Global Directory to install a new address space (Chapter 9).
2. Switching the Kernel Mode stack and the hardware context, which provides all the information needed by the kernel to execute the new process, including the CPU registers.

#### 3.3.3.1 The switch_to macro

Before the process switching, the macro saves in the eax CPU register the content of the variable identified by the first input parameter prev—that is, the prev local variable allocated on the Kernel Mode stack of A. After the process switching, when A has resumed its execution, the macro writes the content of the eax CPU register in the memory location of A identified by the third output parameter last. Because the CPU register doesn’t change across the process switch, this memory location receives the address of C’s descriptor. In the current implementation of schedule(), the last parameter identifies the prev local variable of A, so prev is overwritten with the address of C.



#### 3.3.3.2 The __swith_to function

#### 3.3.4 Saving and Loading the FPU, MMX, and XMM Registers


## 3.4 Creating Processes
## 3.5 Destroying Processes


# 6 Timing Measurements
Two main kinds of timing measurement that must be performed by the Linux kernel:
1. Keeping the current time and date so they can be returned to user programs through the time(), ftime(), and gettimeofday() APIs and used by the kernel itself as timestamps for files and network packets
2. Maintaining timers.

## Clock and Timer Circuits
The clock circuits are used both to keep track of the current time of day and to make precise time measurements.

The timer circuits are programmed by the kernel, so that they issue interrupts at a fixed, predefined frequency.

### Real Time Clock (RTC)
Linux uses the RTC only to derive the time and date;

It allows processes to program the RTC by acting on the /dev/rtc device file;

The kernel accesses the RTC through the 0x70 and 0x71 I/O ports.

### Time Stamp Counter (TSC)
Linux may take advantage of this register to get much more accurate time measurements than those delivered by the Programmable Interval Timer.

### Programmable Interval Timer (PIT)
The role of a PIT is similar to the alarm clock of a microwave oven: it makes the user aware that the cooking time interval has elapsed.

It issues a special interrupt called timer interrupt, which notifies the kernel that one more time interval has elapsed and goes on issuing interrupts forever at some fixed frequency established by the kernel.

The PIT is initialized by `setup_pit_timer()` as follows:
```C++
spin_lock_irqsave(&i8253_lock, flags);
outb_p(0x34,0x43);
udelay(10);
outb_p(LATCH & 0xff, 0x40);
udelay(10);
outb(LATCH >> 8, 0x40);
spin_unlock_irqrestore(&i8253_lock, flags);
```

### CPU Local Timer (APIT)
The CPU local timer is a device similar to the Programmable Interval Timer just described that can issue one-shot or periodic interrupts. There are, however, a few differences:
1. The APIC’s timer counteris 32bits long,while the PIT’s timer counteris 16 bits long; therefore, the local timer can be programmed to issue interrupts at very low frequencies
2. The local APIC timer sends an interrupt only to its processor, while the PIT raises a global interrupt, which may be handled by any CPU in the system.
3. The APIC’s timer is based on the bus clock signal (or the APIC bus signal, in older machines). It can be programmed in such a way to decrease the timer counter every 1, 2, 4, 8, 16, 32, 64, or 128 bus clock signals. Conversely, the PIT, which makes use of its own clock signals, can be programmed in a more flexible way.

### High Precision Event Timer (HPET)
The HPET provides a number of hardware timers that can be exploited by the kernel.

> Each HPET has one fixed-rate counter (at 10+ MHz, hence "High Precision") and up to 32 comparators. Normally three or more comparators are provided, each of which can generate oneshot interrupts and at least one of which has additional hardware to support periodic interrupts. The comparators are also called "timers", which can be misleading since usually timers are independent of each other … these share a counter, complicating resets. [From Linux Kernle Doc](https://www.kernel.org/doc/html/latest/timers/hpet.htm)

Any counter is associated with at most 32 timers, each of which is composed by a comparator and a match register.

The comparator is a circuit that checks the value in the counter against the value in the match register, and raises a hardware interrupt if a match is found.

Some of the timers can be enabled to generate a periodic interrupt.

### ACPI Power Management Timer (ACPI PMT)
The ACPI Power Management Timer is preferable to the TSC if the operating system or the BIOS may dynamically lower the frequency or voltage of the CPU to save battery power. When this happens, the frequency of the TSC changes—thus causing time warps and others unpleasant effects—while the frequency of the ACPI PMT does not.

## The Linux Timekeeping Architecture
The kernel periodically:
1. Updates the time elapsed since system startup.
2. Updates the time and date.
3. Determines, for every CPU, how long the current process has been running, and preempts it if it has exceeded the time allocated to it.
4. Updates resource usage statistics.
5. Checks whether the interval of time associated with each software timer has elapsed.

Linux’s timekeeping architecture depends also on the availability of the Time Stamp Counter (TSC), of the ACPI Power Management Timer (PMT), and of the High Precision Event Timer (HPET).

The kernel uses two basic timekeeping functions:
1. keep the current time up-to-date
2. count the number of nanoseconds that have elapsed within the current second.

### Data Structures of the Timekeeping Architecture

#### The jiffies variable
The jiffies variable is a counter that stores the number of elapsed ticks since the system was started.

The kernel handles cleanly the overflow of jiffies thanks to the `time_after`, `time_after_eq`, `time_before`, and `time_before_eq` macros: they yield the correct value even if a wraparound occurred.

```C++
u64 get_jiffies_64(void)
{
  unsigned long seq;
  u64 ret;

  do {
    seq = read_seqbegin(&jiffies_lock);
    ret = jiffies_64;
  } while (read_seqretry(&jiffies_lock, seq));
  return ret;
}
```

#### The xtime variable
```C++
struct timespec {
  __kernel_time_t   tv_sec;      /* seconds */
  long              tv_nsec;    /* nanoseconds */
};
```

### Timekeeping Architecture in Uniprocessor Systems
#### Initialization phase
time_init
1. Initializes the xtime variable by `get_cmos_time()`
2. Initializes the `wall_to_monotonic` variable.
3. If the kernel supports HPET, it invokes the `hpet_enable()` function to determine whether the ACPI firmware has probed the chip and mapped its registers in the memory address space.
4. Invokes `clocksource_select()` to select the best timer source available in the system, and sets the cur_timer variable to the address of the corresponding timer object.
5. Invokes `setup_irq(0,&irq0)` to set up the interrupt gate corresponding to IRQ0

#### The timer interrupt handler

### Timekeeping Architecture in Multiprocessor Systems
Multiprocessor systems can rely on two different sources of timer interrupts:
1. those raised by the PIT or the HPET
2. those raised by the CPU local timers.

#### Initialization phase

#### The global timer interrupt handler

#### Updating Local CPU Statistics
1. Checks how long the current process has been running.
1.1

### Software Timers and Delay Functions
Linux considers two types of timers called dynamic timers and interval timers. The first type is used by the kernel, while interval timers may be created by processes in User Mode.

#### Dynamic Timers
Dynamic timers may be dynamically created and destroyed. No limit is placed on the number of currently active dynamic timers.

##### Dynamic timers and race conditions
a rule of thumb is to stop the timer before releasing the resource:
...
> del_timer(&t);
> X_Release_Resources( );

In multiprocessor systems, however, this code is not safe because the timer function might already be running on another CPU when del_timer() is invoked.

##### Dynamic timer handling
Despite the clever data structures, handling software timers is a time-consuming activity that should not be performed by the timer interrupt handler. In Linux 2.6 this activity is carried on by a deferrable function, namely the TIMER_SOFTIRQ softirq.

#### An Application of Dynamic Timers: the nanosleep( ) System Call

#### Delay Functions
```C++
static void udelay(int loops)
{
    while (loops--)
        io_delay();    /* Approximately 1 us */
}

static inline void io_delay(void)
{
    const u16 DELAY_PORT = 0x80;
    asm volatile("outb %%al,%0" : : "dN" (DELAY_PORT));
}
```

# 7 Process Scheduling

## 7.1 Scheduling Policy

Objectives: fast process response time, good throughput for background jobs, avoidance of process starvation, reconciliation of the needs of low- and high- priority processes, and so on.

Linux scheduling is based on the time sharing technique: several processes run in “time multiplexing” because the CPU time is divided into slices, one for each runnable pro- cess.

The scheduling policy is also based on ranking processes according to their priority.

When speaking about scheduling, processes are traditionally classified as I/O-bound or CPU-bound.

An alternative classification distinguishes three classes of processes: Interactive processes, Batch processes, Real-time processes.

System call | Description
--- | ---
nice( ) | Change the static priority of a conventional process
getpriority( ) | Get the maximum static priority of a group of conventional processes
setpriority( ) | Set the static priority of a group of conventional processes
sched_getscheduler( ) | Get the scheduling policy of a process
sched_setscheduler( ) | Set the scheduling policy and the real-time priority of a process
sched_getparam( ) | Get the real-time priority of a process
sched_setparam( ) | Set the real-time priority of a process
sched_yield( ) | Relinquish the processor voluntarily without blocking
sched_get_priority_min( ) | Get the minimum real-time priority value for a policy
sched_get_priority_max( ) | Set the maximum real-time priority value for a policy
sched_rr_get_interval( ) | Get the time quantum value for the Round Robin policy
sched_setaffinity( ) | Set the CPU affinity mask of a process
sched_getaffinity( ) | Get the CPU affinity mask of a process

### 7.1.1 Process Preemption

When a process enters the TASK_RUNNING state, the kernel checks whether its dynamic priority is greater than the priority of the currently running process. If it is, the execution of current is interrupted and the scheduler is invoked to select another process to run.

Be aware that a preempted process is not suspended, because it remains in the TASK_ RUNNING state; it simply no longer uses the CPU.

### 7.1.2 How Long Must a Quantum Last?

It is often believed that a long quantum duration degrades the response time of inter- active applications. This is usually false. As described in the section “Process Pre- emption” earlier in this chapter, interactive processes have a relatively high priority, so they quickly preempt the batch processes, no matter how long the quantum dura- tion is.

## 7.2 The Scheduling Algorithm

scheduling classes: SCHED_FIFO, SCHED_RR, SCHED_NORMAL

### 7.2.1 Scheduling of Conventional Processes

* Base time quantum

    > base time quantum (in milliseconds) =
    > 1. (140 – static priority) × 20 if static priority < 120
    > 2. (140 – static priority) × 5 if static priority  =120

    Description | Static priority | Nice value | Base time quantum | Interactivedelta | Sleep time threshold
    --- | --- | --- | --- | --- | ---
    Highest static priority | 100 | -20 | 800 ms | –3 | 299 ms
    High static priority | 110 | -10 | 600 ms | -1 | 499 ms
    Default static priority | 120 | 0 | 100 ms | +2 | 799 ms
    Low static priority | 130 | +10 | 50 ms | +4 | 999 ms
    Lowest static priority | 139 | +19 | 5 ms | +6 | 1199 ms


* Dynamic priority and average sleep time

    > dynamic priority = max(100, min(static priority − bonus + 5, 139))

    The **bonus** is a value ranging from 0 to 10; a value less than 5 represents a penalty that lowers the dynamic priority, while a value greater than 5 is a premium that raises the dynamic priority. Bonus is related to the `average sleep time` of the process

    Average sleep time | Bonus | Granularity
    --- | --- | ---
    \>=0 < 100 ms | 0 | 5120
    \>=100 ms < 200 ms | 1 | 2560
    \>=200 ms < 300 ms | 2 | 1280
    \>=300 ms < 400 ms | 3 | 640
    \>=400 ms < 500 ms | 4 | 320
    \>=500 ms < 600 ms | 5 | 160
    \>=600 ms < 700 ms | 6 | 80
    \>=700 ms < 800 ms | 7 | 40
    \>=800 ms < 900 ms | 8 | 20
    \>=900 ms < 1000 ms | 9 | 10
    1 second | 10 | 10

    interactive process: [dynamic priority ≤ 3 × static priority / 4 + 28] or [bonus - 5 ≥ static priority / 4 − 28]

    The expression `static priority / 4 − 28` is called the interactive delta.

* Active and expired processes

* Scheduling of Real-Time Processes

## 7.3 Data Structures Used by the Scheduler

### 7.3.4 schedule()

* Direct invocation
    1. Inserts current in the proper wait queue.
    2. Changes the state of current either to TASK_INTERRUPTIBLE or to TASK_ UNINTERRUPTIBLE.
    3. Invokes schedule( ).
    4. Checks whether the resource is available; if not, goes to step 2.
    5. Once the resource is available, removes current from the wait queue.

* Lazy invocation

    The scheduler can also be invoked in a lazy way by setting the TIF_NEED_RESCHED flag of current to 1.

    * When current has used up its quantum of CPU time; this is done by the **scheduler_tick**() function.
    * When a process is woken up and its priority is higher than that of the current process; this task is performed by the **try_to_wake_up**() function.
    * When a **sched_setscheduler**() system call is issued (see the section “System Calls Related to Scheduling” later in this chapter).

## 7.4 Functions Used by the Scheduler

## 7.5 Runqueue Balancing in Multiprocessor Systems

Types of multipro- cessor machines:
* Classic multiprocessor architecture
* Hyper-threading
    * Allows the processor to exploit the machine cycles to execute another thread while the current thread is stalled for a memory access. A hyper-threaded physical CPU is seen by Linux as several different logical CPUs.
* NUMA

These basic kinds of multiprocessor systems are often combined. For instance, a motherboard that includes two different hyper-threaded CPUs is seen by the kernel as four logical CPUs.

Linux sports a sophisticated runqueue balancing algorithm based on the notion of “scheduling domains.”

### 7.5.1 Scheduling Domains

A scheduling domain is a set of CPUs whose workloads should be kept balanced by the kernel.

Workload balancing is always done between groups of a scheduling domain. In other words, a process is moved from one CPU to another only if the total workload of some group in some scheduling domain is significantly lower than the workload of another group in the same scheduling domain.

![](../Images/ULK/7.5-schedule-domain-hierarchies.png)


**TODO**

### 7.5.2 rebalance_tick

## 7.6 System Calls Related to Scheduling



# 8 Memory Management
## Page Frame Management
1. Linux adopts the smaller 4 KB page frame size as the standard memory allocation unit. This makes things simpler for two reasons:
    * The Page Fault exceptions issued by the paging circuitry are easily interpreted.
    * Although both 4 KB and 4 MB are multiples of all disk block sizes, transfers of data between main memory and disks are in most cases more efficient when the smaller size is used.

### Page Descriptors
### Non-Uniform Memory Access (NUMA)
The Linux kernel makes use of NUMA even for some peculiar uniprocessor systems that have huge "holes" in the physical address space.

The kernel handles these architectures by assigning the contiguous subranges of valid physical addresses to different memory nodes.


### Memory Area Management

### Noncontiguous Memory Area Management

# 14 Block Device Drivers

![](../Images/ULK/14-block-device-layer.png)

## 14.1 Block Devices Handling

![](../Images/ULK/14-1-page-segment-block-sector.png)

### 14.1.1 Sectors

* The controllers of the hardware block devices transfer data in chunks of fixed length called "sectors."


### 14.1.2 Blocks

The Virtual Filesystem, the mapping layer, and the filesystems group the disk data in logical units called "blocks."

While the sector is the basic unit of data transfer for the hardware devices, the block is the basic unit of data transfer for the VFS and, consequently, for the filesystems.

In Linux, the block size must be a power of 2 and cannot be larger than a page frame. Moreover, it must be a multiple of the sector size, because each block must include an integral number of sectors.

Each block requires its own block buffer, which is a RAM memory area used by the kernel to store the block’s content.

### 14.1.3 Segments

As we will see shortly, block device drivers should be able to cope with "segments" of data: each segment is a memory page—or a portion of a memory page—including chunks of data that are physically adjacent on disk.

For each scatter-gather DMA transfer, the block device driver must send to the disk controller:
* The initial disk sector number and the total number of sectors to be transferred
* A list of descriptors of memory areas, each of which consists of an address and a length.

The disk controller takes care of the whole data transfer; for instance, in a read oper- ation the controller fetches the data from the adjacent disk sectors and scatters it into the various memory areas.

To make use of scatter-gather DMA operations, block device drivers must handle the data in units called segments. A segment is simply a memory page—or a portion of a memory page—that includes the data of some adjacent disk sectors. Thus, a scatter- gather DMA operation may involve several segments at once.


## 14.2 The Generic Block Layer
The generic block layer is a kernel component that handles the requests for all block devices in the system.

## 14.3 The I/O Scheduler
To keep the block device driver from being suspended, each I/O operation is pro- cessed asynchronously. In particular, block device drivers are interrupt-driven (see the section "Monitoring I/O Operations" in the previous chapter): the generic block layer invokes the I/O scheduler to create a new block device request or to enlarge an already existing one and then terminates. The block device driver, which is activated at a later time, invokes the strategy routine to select a pending request and satisfy it by issuing suitable commands to the disk controller. When the I/O operation termi- nates, the disk controller raises an interrupt and the corresponding handler invokes the strategy routine again, if necessary, to process another pending request.

Each block device driver maintains its own request queue, which contains the list of pending requests for the device. If the disk controller is handling several disks, there is usually one request queue for each physical block device.

### 14.3.1 The "Noop" elevator

There is no ordered queue: new requests are always added either at the front or at the tail of the dispatch queue, and the next request to be processed is always the first request in the queue.

### 14.3.2 The "CFQ" elevator


Requests coming from the same process are always inserted in the same queue.

### 14.3.3 The "Deadline" elevator

Besides the dispatch queue, the “Deadline” elevator makes use of four queues.
* Two of them—the sorted queues—include the read and write requests, respectively, ordered according to their initial sector numbers.
* The other two—the deadline queues— include the same read and write requests sorted according to their “deadlines.”

A request deadline is essentially an expire timer that starts ticking when the request is passed to the elevator. By default, the expire time of read requests is 500 milliseconds, while the expire time for write requests is 5 seconds—read requests are privileged over write requests.

## 14.4 Block Device Drivers
## 14.5 Opening a Block Device File

# 20 Program Execution
## 20.1 Executable Files
## 20.2 Executable Formats
## 20.3 Execution Domains
## 20.4 The exec Functions

* How dynamic linker works:
  1. sets up a basic execution context for itself, starting from the information stored by the kernel in the User Mode stack between the array of pointers to environment strings and arg_start.
  2. examines the program to be executed to identify which shared libraries must be loaded and which functions in each shared library are effectively requested.
  3. issues several mmap() system calls to create memory regions mapping the pages that will hold the library functions (text and data) actually used by the program.
  4. update all references to the symbols of the shared library, according to the linear addresses of the library’s memory regions.
  5. terminates its execution by jumping to the main entry point of the program to be executed.
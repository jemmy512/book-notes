# Chapter 2 Processes and Theads
### 2.1 Processes
#### 2.1.2 Process Creation
Four principal events cause processes to be created:
1. System initialization.
2. Execution of a process-creation system call by a running process.
3. A user request to create a new process.
4. Initiation of a batch job.

#### 2.1.3 Process Termination
1. Normal exit (voluntary).
2. Error exit (voluntary).
3. Fatal error (involuntary).
4. Killed by another process (involuntary).

#### 2.1.4 Process Hierarchies
When a user sends a signal from the keyboard, the signal is delivered to all members of the process group currently associated with the keyboard (usually all active processes that were created in the current window). Individually, each process can catch the signal, ignore the signal, or take the default action, which is to be killed by the signal.

#### 2.1.5 Process States
1. Running (actually using the CPU at that instant).
2. Ready (runnable; temporarily stopped to let another process run).
3. Blocked (unable to run until some external event happens).

#### 2.1.6 Implementation of Processes
Process management | Memory management | File management
--- | --- | ---
Registers | Pointer to text segment info | Root directory
Program counter | Pointer to data segment info | Working director
Program status word | Pointer to stack segment info | File descriptors
Stack pointer | | User ID
Process state | | Group ID
Priority | |
Scheduling parameters | |
Parent process | |
Process ID | |
Process group | |
Signals | |
Time when process started CPU time used | |
Children’s CPU time | |
Time of next alarm | |


Associated with each I/O class is a location (typically at a fixed location near the bottom of memory) called the **interrupt vector**. It contains the address of the interrupt service procedure.

All interrupts start by saving the registers, often in the process table entry for the current process.

Interrupt handling and scheduling:
1. Hardware stacks program counter, etc.
2. Hardware loads new program counter from interrupt vector.
3. Assembly-language procedure saves registers.
4. Assembly-language procedure sets up new stack.
5. C interrupt service runs (typically reads and buffers input).
6. Scheduler decides which process is to run next.
7. C procedure returns to the assembly code.
8. Assembly-language procedure starts up new current process.

### 2.2 THREADS
#### 2.2.1 Thread Usage
The reason for threads:
1. The main reason for having threads is that in many applications, multiple activities are going on at once.  The ability for the parallel entities to share an address space and all of its data among themselves.
2. Threas are lighter weight than processes, they are goes 10–100 times faster than creating a process.
3. Performance argument. Threads yield no performance gain when all of them are CPU bound, but when there is substantial computing and also substantial I/O, having threads allows these activities to overlap, thus speeding up the application.
4. Threads are useful on systems with multiple CPUs, where real parallelism is possible.

#### 2.2.2 The Classical Thread Model
The process model is based on two independent concepts:
1. resource grouping: program text and data, open files, child processes, pending alarms, signal handlers, accounting information, and more.
2. thread of execution: the thread has a program counter, registers and stack.

Processes are used to group resources together; threads are the entities scheduled for execution on the CPU.

Per-process items | Per-thread items
--- | ---
Address space | Program counter
Global variables | Registers
Open files | Stack
Child processes  | State
Pending alarms |
Signals and signal handlers |
Accounting information |

Process is the unit of resouces management, thread is the unit of execution.

#### 2.2.3 POSIX Threads
#### 2.2.4 Implementing Threads in User Space
Advantages:
1. no trap is needed, no context switch is needed, the memory cache need not be flushed, and so on. This makes thread scheduling very fast.
2. allow each process to have its own customized scheduling algorithm.

Disadvantages:
1. how blocking system calls are implemented.
2. The system calls could all be changed to be nonblocking (e.g., a read on the keyboard would just return 0 bytes if no characters were already buffered), but requiring changes to the operating system is unattractive.

#### 2.2.5 Implementing Threads in the Kernel
All calls that might block a thread are implemented as system calls, at considerably greater cost than a call to a run-time system procedure.

Due to the relatively greater cost of creating and destroying threads in the kernel, some systems take an environmentally correct approach and recycle their threads.

Kernel threads do not require any new, nonblocking system calls.

#### 2.2.6 Hybrid Implementations
Use kernel-level threads and then multiplex user-level threads onto some or all of them.

#### 2.2.7 Scheduler Activations
The goals of the scheduler activation work are to mimic the functionality of kernel threads, but with the better performance and greater flexibility usually associated with threads packages implemented in user space.

Efficiency is achieved by avoiding unnecessary transitions between user and kernel space.

When scheduler activations are used, the kernel assigns a certain number of virtual processors to each process and lets the (user-space) run-time system allocate threads to processors.

The basic idea that makes this scheme work is that when the kernel knows that a thread has blocked, the kernel notifies the process’ run-time system, passing as parameters on the stack the number of the thread in question and a description of the event that occurred. The notification happens by having the kernel activate the run-time system at a known starting address, roughly analogous to a signal in UNIX. This mechanism is called an **upcall**.

Once activated, the run-time system can reschedule its threads, typically by marking the current thread as blocked and taking another thread from the ready list, setting up its registers, and restarting it.

When a hardware interrupt occurs while a user thread is running, the interrupted CPU switches into kernel mode:
1. If the interrupt is caused by an event not of interest to the interrupted process, it puts the interrupted thread back in the state it was in before the interrupt.
2. If, however, the process is interested in the interrupt, the interrupted thread is not restarted. Instead, it is suspended, and the run-time system is started on that virtual CPU, with the state of the interrupted thread on the stack. It is then up to the run-time system to decide which thread to schedule on that CPU: the interrupted one, the newly ready one, or some third choice.

#### 2.2.8 Pop-Up Threads
#### 2.2.9 Making Single-Threaded Code Multithreaded
Problems when makding single-threaded code multithreaded:
1. Global variables, solution is to assign each thread its own private global variables.
2. Many library procedures are not reentrant.
    * Solution 1: provide each procedure with a jacket that sets a bit to mark the library as in use.
    * Solution 2: signals.
        Some signals are logically thread specific, whereas others are not. However, when threads are implemented entirely in user space, the kernel does not even know about threads and can hardly direct the signal to the right one.

        Other signals, such as keyboard interrupt, are not thread specific. Who should catch them? One designated thread? All the threads? A newly created pop-up thread? Furthermore, what happens if one thread changes the signal handlers without telling other threads?
3. Stack management.

Introducing threads into an existing system needs fairly substantial system redesign:
> The semantics of system calls may have to be redefined and libraries rewritten, at the very least. And all of these things must be done in such a way as to remain backward compatible with existing programs for the limiting case of a process with only one thread.

### 2.3 INTERPROCESS COMMUNICATION
Problems:
1. how one process can pass information to another.
2. making sure two or more processes do not get in each other’s way
3. proper sequencing when dependencies

#### 2.3.1 Race Conditions
**race conditions**: two or more processes are reading or writing some shared data and the final result depends on who runs precisely when.

#### 2.3.2 Critical Regions
We need four conditions to hold to have a good solution:
1. No two processes may be simultaneously inside their critical regions.
2. No assumptions may be made about speeds or the number of CPUs.
3. No process running outside its critical region may block any process.
4. No process should have to wait forever to enter its critical region.

#### 2.3.3 Mutual Exclusion with Busy Waiting
##### Disabling Interrupts
Once a process has disabled interrupts, it can examine and update the shared memory without fear that any other process will intervene.

What if one of them did it, and never turned them on again? That could be the end of the system.

Furthermore, if the system is a multiprocessor (with two or more CPUs) disabling interrupts affects only the CPU that executed the disable instruction. The other ones will continue running and can access the shared memory.

The conclusion is: disabling interrupts is often a useful technique within the operating system itself but is not appropriate as a general mutual exclusion mechanism for user processes.

##### Lock Variables
##### Strict Alternation
Continuously testing a variable until some value appears is called **busy waiting**. It should usually be avoided, since it wastes CPU time.
##### Peterson’s Solution
```C++
#define FALSE 0 /* number of processes */

#define TRUE 1  /* whose turn is it? */

#define N   2   /* all values initially 0 (FALSE) */

int interested[N];
int turn;
void enter_region(int process); /* process is 0 or 1 */
{
    int other;
    other = 1 − process;
    interested[process] = TRUE;
    turn = process;
    while (turn == process && interested[other] == TRUE);
}
void leave_region(int process)      /* process: who is leaving */
{
    interested[process] = FALSE;    /* indicate departure from critical region */
}
```
##### The TSL(Test and Set Lock)/XCHG Instruction
It reads the contents of the memory word lock into register RX and then stores a nonzero value at the memory address lock. The operations of reading the word and storing into it are guaranteed to be indivisible.

```assembly
enter region:
    TSL REGISTER, LOCK  ;copy lock to register and set lock to 1
    CMP REGISTER, #0    ;was lock zero?
    JNE enter_region    ;if it was not zero, lock was set, so loop critical region entered
    RET                 ;return to caller

leave region:
    MOVE LOCK, #0       ;store a 0 in lock
    RET                 ;return to caller
```

Not only does busy wait approach waste CPU time, but it can also have unexpected effects(e.g., priority inversion problem).

#### 2.3.4 Sleep and Wakeup
#### 2.3.5 Semaphores
Once a semaphore operation has started, no other process can access the semaphore until the operation has completed or blocked. This atomicity is absolutely essential to solving synchronization problems and avoiding race conditions.

The normal way is to implement up and down as system calls, with the operating system briefly disabling all interrupts while it is testing the semaphore, updating it, and putting the process to sleep, if necessary.

If multiple CPUs are being used, each semaphore should be protected by a lock variable, with the TSL or XCHG instructions used to make sure that only one CPU at a time examines the semaphore.

#### 2.3.6 Mutexes
```assembly
mutex lock:
    TSL REGISTER, MUTEX ; copy mutex to register and set mutex to 1;
    CMP REGISTER, #0    ; was mutex zero?
    JZE ok              ; if it was zero, mutex was unlocked, so return
    CALL thread_yield   ; mutex is busy; schedule another thread
    JMP mutex_lock      ; try again

ok: RET                 ; return to caller; critical region entered

mutex unlock:
    MOVE MUTEX, #0      ; store a 0 in mutex
    RET                 ; return to caller
```

The difference between enter_region and mutex_lock:
1. A thread that tries to acquire a lock by busy waiting will loop forever and never acquire the lock because it never allows any other thread to run and release the lock.
2. When the mutex_lock fails to acquire a lock, it calls thread yield to give up the CPU to another thread.

Since *thread yield* is just a call to the thread scheduler in user space, it is very fast. As a consequence, neither *mutex lock* nor *mutex unlock* requires any kernel calls.

If processes have disjoint address spaces, how can they share the turn variable in Peterson’s algorithm, or semaphores or a common buffer?
1. Some of the shared data structures, such as the semaphores, can be stored in the kernel and accessed only by means of system calls.
2. Most modern operating systems offer a way for processes to share some portion of their address space with other processes.
3. if nothing else is possible, a shared file can be used.

##### Futex(Fast User Space Mutex)
With increasing parallelism, efficient synchronization and locking is very important for performance.

If there is much contention, spin locks are therefore more efficient to block the process and let the kernel unblock it only when the lock is free.

Spin lock works well under heavy contention, but continuously switching to the kernel is expensive if there is very little contention to begin with.

A **futex** is a feature of Linux that implements basic locking (much like a mutex) but avoids dropping into the kernel unless it really has to.

A futex consists of two parts: a kernel service and a user library. The kernel service provides a ‘‘wait queue’’ that allows multiple processes to wait on a lock.

[Linux Generic Mutex Subsystem](https://www.kernel.org/doc/html/latest/locking/mutex-design.html)

#### 2.3.7 Monitors
A **monitor** is a collection of procedures, variables, and data structures that are all grouped together in a special kind of module or package.

Processes may call the procedures in a monitor whenever they want to, but they cannot directly access the monitor’s internal data structures from procedures declared outside the monitor.

#### 2.3.9 Barriers

### 2.4 SCHEDULING
#### 2.4.1 Introduction to Scheduling

[Wiki Scheduling](https://en.wikipedia.org/wiki/Scheduling_(computing))

##### Process Behavior
##### When to Schedule
1. when a new process is created, a decision needs to be made whether to run the parent process or the child process.
2. a scheduling decision must be made when a process exits.
3. when a process blocks on I/O, on a semaphore, or for some other reason, another process has to be selected to run.
4. when an I/O interrupt occurs, a scheduling decision may be made.

Scheduling algorithms can be divided into two categories with respect to how they deal with clock interrupts:
1. A **nonpreemptive** scheduling algorithm picks a process to run and then just lets it run until it blocks or voluntarily releases the CPU. Even if it runs for many hours, it will not be forcibly suspended.
2. A **preemptive** scheduling algorithm picks a process and lets it run for a maximum of some fixed time.

##### Categories of Scheduling Algorithms
1. Batch: nonpreemptive or preemptive with long time periods is acceptable.
2. Interactive:
3. Real time.

##### Scheduling Algorithm Goals:
All systems
    Fairness: giving each process a fair share of the CPU
    Policy enforcement: seeing that stated policy is carried out
    Balance: keeping all parts of the system busy

Batch systems
    Throughput: maximize jobs per hour
    Turnaround time: minimize time between submission and termination
    CPU utilization: keep the CPU busy all the time

Interactive systems
    Response time: respond to requests quickly
    Proportionality: meet users’ expectations

Real-time systems
    Meeting deadlines: avoid losing data
    Predictability: avoid quality degradation in multimedia systems

#### 2.4.2 Scheduling in Batch Systems
##### First-Come, First-Served
##### Shortest Job First
##### Shortest Remaining Time Next

#### 2.4.3 Scheduling in Interactive Systems
##### Round-Robin Scheduling
##### Priority Scheduling
Setting the quantum too short causes too many process switches and lowers the CPU efficiency, but setting it too long may cause poor response to short interactive requests. A quantum around 20–50 msec is often a reasonable compromise.

To prevent high-priority processes from running indefinitely, the scheduler may decrease the priority of the currently running process at each clock tick.

If this action causes its priority to drop below that of the next highest process, a process switch occurs.

Priorities can also be assigned dynamically by the system to achieve certain system goals.

It is often convenient to group processes into priority classes and use priority scheduling among the classes but round-robin scheduling within each class.

##### Multiple Queues
Processes in the highest class were run for one quantum. Processes in the next-highest class were run for two quanta. Processes in the next one were run for four quanta, etc.

##### Shortest Process Next
##### Guaranteed Scheduling
Make real promises to the users about performance and then live up to those promises. One promise that is realistic to make and easy to live up to is this: If n users are logged in while you are working, you will receive about 1/n of the CPU power. Similarly, on a single-user system with n processes running, all things being equal, each one should get 1/n of the CPU cycles. That seems fair enough.

##### Lottery Scheduling
The basic idea is to give processes lottery tickets for various system resources, such as CPU time. Whenever a scheduling decision has to be made, a lottery ticket is chosen at random, and the process holding that ticket gets the resource.

##### Fair-Share Scheduling

#### 2.4.4 Scheduling in Real-Time
#### 2.4.5 Policy Versus Mechanism
#### 2.4.6 Thread Scheduling
A major difference between user-level threads and kernel-level threads is the performance:
1. Doing a thread switch with user-level threads takes a handful of machine instructions.
2. With kernel-level threads it requires a full context switch, changing the memory map and invalidating the cache, which is several orders of magnitude slower.

Since the kernel knows that switching from a thread in process A to a thread in process B is more expensive than running a second thread in process A, it can take this information into account when making a decision.

Another important factor is that user-level threads can employ an application-specific thread scheduler.

### 2.5 CLASSICAL IPC PROBLEMS
#### 2.5.1 The Dining Philosophers Problem
#### 2.5.2 The Readers and Writers Problem
Solution: read write lock

As a consequence of this strategy, as long as there is a steady supply of readers, they will all get in as soon as they arrive. The writer will be kept suspended until no reader is present.

To avoid this situation, the program could be written slightly differently: when a reader arrives and a writer is waiting, the reader is suspended behind the writer instead of being admitted immediately.

The disadvantage of this solution is that it achieves less concurrency and thus lower performance.

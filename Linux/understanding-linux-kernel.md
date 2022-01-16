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
		io_delay();	/* Approximately 1 us */
}

static inline void io_delay(void)
{
	const u16 DELAY_PORT = 0x80;
	asm volatile("outb %%al,%0" : : "dN" (DELAY_PORT));
}
```


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
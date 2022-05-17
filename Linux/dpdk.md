# 1 Introduction
## 1.1 What’s Packet Processing?

## 1.2 The Hardware Landscape
### 1.2.1 Hardware Accelerator
### 1.2.2 Network Processor Unit
### 1.2.3 Multicore Processor

## 1.3 The Software Landscape
### 1.3.1 Before DPDK

How does a traditional NIC device process the packet in a server system using Linux? The steps are summarized below:
* A packet arrives at a NIC (PCIe device).
* The NIC completes DMA (direct memory access) and copy packet into host memory region known as a packet buffer.
* The NIC sends an interrupt to wake up the processor.
* The processor reads and writes the packet descriptor and packet buffer.
* The packet is sent to the Linux kernel protocol stack for more protocol processing like IP-related access control decision.
* If the application resides in the user space, the packet data (also known as payload) will be copied from the kernel space to the user space.
* If the application resides in the kernel space, the data will be processed in the kernel mode (less percentage).

NAPI (new API) mechanism was introduced into the Linux kernel. It allowed the system, after wakeup by the interrupt, to initiate the software routine that processes multiple packets in a polling manner, until all packets are handled, then goes back to the interrupt mode. The NAPI method can significantly improve the packet processing efficiency in a high burst scenario.

As a time-sharing operating system, Linux will schedule many tasks with time-slicing mechanism. Compared with equal time assigned to all tasks, Linux scheduler has the job to assign the different time slices for different tasks.

Intel® engineers need answer for the following:
* A software path to enable the packet processing on x86 CPU;
* Find a better software method to do things differently;
* A performance scale way using multicore architecture;
* How to tune “Linux system” as packet processing environment.

### 1.3.2 DPDK Way

**Polling mode**: Assign the dedicated core for NIC packet reception and transmission. This approach does not share core for other software tasks, and the core can run in the endless loop to check if any packet just arrives or needs to be sent out, thus reducing the need for interrupt service and its overhead. We will discuss the trade-offs between polling and interrupt mechanisms later. In fact, DPDK supports both mechanisms and even hybrid-use model.

**User space driver**: In fact, in most scenarios, the packet needs to be sent to the user space eventually. Linux NIC driver is mostly kernel based. The user space driver can avoid unnecessary packet memory copy from the kernel space to the user space, and it also saves the cost of system calls. An indirect benefit is that the user space driver is not limited to the packet buffer structure mandated in the Linux kernel mode. Linux kernel stack mandates the stable interface, and the DPDK-based mbuf (memory buffer) header format can be flexibly defined (because it is new) so that it can be designed in DMA-optimized way for NIC. This flexibility adds the performance benefit. The user space driver is flexible, it is easy to modify, and it meets the rapid development needed for different scenarios.

**Core affinity**: By setting a thread’s affinity to a particular CPU core, specific tasks can be bound with cores (thread). Without the core affinity assignment, there might be the task switching among different cores, and the drawback to this assignment is that thread switching between cores can easily lead to performance losses due to cache misses and cache write-back. One further step is to ask a core to be excluded from Linux scheduling system, so that the core is only used for the specific task.

**Optimized memory**: Network processing is an I/O-bound workload scenario. Both CPU and NIC need access to the data in memory (actually cache and/or DRAM) frequently. The optimal memory access includes the use of HugePage and contiguous memory regions. For example, HugePage memory can reduce the TLB misses, multichannel-interleaved memory access can improve the total bandwidth efficiency, and the asymmetric memory access can reduce the access latency. The key idea is to get the data into cache as quickly as possible, so that CPU doesn’t stall.

**Software tuning**: Tuning itself cannot be claimed as the best practice. In fact, it refers to a few known tuning practices, such as `cache line alignment` of data structure, avoiding `false sharing` between multiple cores, `pre-fetching data` in a timely manner, and `bulk operations` of multiple data (multi-buffer). These optimization methods are used in every corner of DPDK. The code example can be found in the “l3fwd” case study. It is important to know that these techniques are commonly applicable; beyond DPDK, any software can be optimized with the similar approach.

Using the latest **instruction set and platform technologies**: The latest instruction sets of Intel® processor and other new features has been one of the innovation sources of DPDK optimization. For example, Intel® `DDIO` (Direct Data I/O) technology is a hardware platform innovation in DMA and the cache subsystem. DDIO plays a “significant role to boost I/O performance as the packet data can be directly placed into cache, thus reducing the CPU access latency on DRAM. Without DDIO, packet is always placed into memory first, and then CPU needs to fetch packet data from DRAM into cache, which means the extra cycles that CPU needs to wait. The other example is how to make the best use of `SIMD (single-instruction multiple data)` and `multiple buffer` (multi-buffer) programming techniques. Some instructions, like `CMPXCHG`, are the cornerstone for lockless data structure design. `Crc32` instruction is also a good source for efficient hash computation. These contents will be covered in later chapters.

**NIC driver tuning**: When the packet enters the system memory through PCIe interface, I/O performance is affected by the transaction efficiency among the PCIe-based device, bus transaction, and the system memory. For example, the packet data coalescence can make a difference through transferring multiple packets together, thus allowing a more efficient use of PCIe bus transactions. Modern NICs also support load balancing mechanisms such as receive side scaling (`RSS`) and Flow Director (`FDir`) features, which enable NIC multiple queue to work with CPU multiple core model. New NIC offload can also perform the packet header checksum, TCP segmentation offload (TSO), and tunnel header processing. DPDK is designed to take full advantage of the NIC features for performance reasons.

**Network virtualization and cloud-native acceleration**: Initial DPDK optimization focuses on moving packets from I/O to CPU. Later, DPDK provides the optimal way to move packets from host to tenants (VM, container tenants). This is a crucial ingredient for cloud infrastructure and network function virtualization (NFV). DPDK supports both SR-IOV and vSwitch optimization with PMD concept.

**Security acceleration**: DPDK can run from the bare metal to the virtualized guest and container-based environment; the initial Application Programming Interface (API) abstraction is NIC centric; later, it is extended from Ethernet to crypto, compression, and storage I/O acceleration. Crypto and compression APIs are important software abstraction; they can hide the underlying silicon’s implementation difference.

### 1.3.3 DPDK Scope

![](../Images/dpdk/1.3-dpdk-arch.png)

## 1.4 Performance Limit
### 1.4.1 The Performance Metric
### 1.4.2 The Processing Budget

## 1.5 DPDK Use Case
### 1.5.1 Accelerated Network
### 1.5.2 Accelerated Computing
### 1.5.3 Accelerated Storage

## 1.6 Optimization Principles
## 1.7 DPDK Samples
### 1.7.1 HelloWorld
#### 1.7.1.1 Initialize the Runtime Environment
#### 1.7.1.2 Multicore Initialization
### 1.7.2 Skeleton
#### 1.7.2.1 Ethernet Port Initialization
### 1.7.3 L3fwd

## 1.8 Conclusion

# 2 Cache and Memory
## 2.1 Data Access and Latency

Intel® Xeon Processor Cache Access Latency
![](../Images/dpdk/2.2-cache-lantency.png)

## 2.2 Intel Xeon Architecture

* RAM: random access memory;
* SRAM: static RAM;
* DRAM: dynamic RAM;
* SDRAM: synchronous dynamic random access memory;
* DDR: double data rate SDRAM;
* DDR2: the second generation of DDR;
* DDR3: the third generation of DDR;
* DDR4: the fourth generation of DDR. This is supported by Intel® Xeon SP;
* DDR5: the fifth generation of DDR. Product is expected in 2020.

## 2.3 Intel Atom SoC

## 2.4 Cache

A cache line is the minimum size of data block of load/write to the cache; this is defined by CPU and is not programmable by software.

### 2.4.1 Cache Line
### 2.4.2 Cache Prefetching

### 2.4.3 Software Prefetching

DPDK process a packet:
1. Write NIC RX descriptors into memory, and fill the descriptor with the data buffer pointer. Upon receiving a packet, the NIC will write the packet into the specified buffer.
2. Read the RX descriptors in the memory to check if a packet is received (the NIC will update the descriptors as soon as it receives the packet, memory read). After the packet arrival is confirmed, read the pointer of the control structure from the memory, and then read the control structure itself again from the memory (memory read). Fill the RX descriptors information to the control structure.
3. Update the NIC RX queue (RXQ) to indicate that a new packet is ready to be processed.
4. Read the packet header (memory read), and decide the next port for Packet TX.
5. Fill the packet information to the control structure in the NIC TX queue (TXQ) descriptors. Update the TXQ register.
5. Read the NIC TX descriptor from the memory (memory read), and check whether there are packets waiting to be transmitted by hardware.
6. If there are packets waiting, read the control structure from memory (memory read) and release the data buffer.

### 2.4.4 False Sharing

In computer science, [false sharing](https://en.wikipedia.org/wiki/False_sharing) is a performance-degrading usage pattern that can arise in systems with distributed, coherent caches at the size of the smallest resource block managed by the caching mechanism. When a system participant attempts to periodically access data that is not being altered by another party, but that data shares a cache block with data that is being altered, the caching protocol may force the first participant to reload the whole cache block despite a lack of logical necessity. The caching system is unaware of activity within this block and forces the first participant to bear the caching system overhead required by true shared access of a resource.

There is a cost for the data contention in a cache line, and if multicore is trying to access the different data in the same cache, CPU will invalidate the cache line and force an update, hurting the performance. This data sharing is not necessary because the multiple cores are not trying to access the same data, which is known as **false sharing**.

The compiler can find the false sharing, and it will try to eliminate the false sharing at the optimization phase.

If false sharing is identified, the software fix is simple: Just ensure the shared data elements reside in a different cache line. If a data structure is used for multicore use, each individual data member can occupy its own cache line.

```c++
#define RTE_CACHE_LINE_SIZE 64 
#define __rte_cache_aligned __attribute__((__aligned__(RTE_CACHE_LINE_SIZE)))

struct rte_port_core_stats {
       __rte_cache_aligned uint64_t core1_pkts; 
       __rte_cache_aligned uint64_t core2_pkts; 
       __rte_cache_aligned uint64_t core3_pkts; 
       __rte_cache_aligned uint64_t core4_pkts; 
};
```

### 2.4.5 Cache Coherency

### 2.4.6 Noisy Tenant and RDT
Intel® proposed RDT (Resource Director Technology) framework to tackle the “noisy neighbor” problem, and RDT includes multiple technology ingredients such as cache monitoring, cache allocation, memory bandwidth monitoring, and cache and data optimization technologies. RDT is not specific to DPDK, so it is not introduced in this book.

## 2.5 TLB and HugePage

Linux supports the hugepage size as 2 MB or 1 GB. The memory translation is indeed a multilevel page table lookup.

In the latest Skylake processor, TLB entries are supported for 1 GB page size and 32 entries are supported for 2 MB page size.

There are two ways to reserve HugePage using Linux.
* Reserve the HugePage at Linux system initialization time and it will take this as the launch command option.
* Dynamic memory reservation after the Linux is launched.

### 2.5.1 Reserve HugePage at Boot time

> default_hugepagesz=1G hugepagesz=1G hugepages=4

### 2.5.2 Reserve HugePage at Runtime

Reserve 1024 pages of 2 MB memory (2 MB HugePages):

> echo 1024 > /sys/kernel/mm/hugepages/hugepages-2048kB/nr_hugepages

In the NUMA system with two nodes, we can use the following command:
> echo 1024 > /sys/devices/system/node/node0/hugepages/hugepages-2048kB/nr_hugepages
> echo 1024 > /sys/devices/system/node/node1/hugepages/hugepages-2048kB/nr_hugepages

Mount hugetlbfs like a path/mnt/huge.
> mkdir /mnt/huge
> mount -t hugetlbfs nodev /mnt/huge

## 2.6 Memory Latency


## 2.7 DDIO
Intel® DDIO enables the NIC and CPU to exchange data directly through the L3 cache, and NIC may move the packet into the LLC (instead of memory), as the new primary destination. In an optimized system, it reduces about 100 cycles for the arrived Ethernet packets.

## 2.8 NUMA

# 3 Core-Based Parallelism
## 3.1 Multicore Performance
### 3.1.1 Horizontal Scale
Amdahl’s law clearly indicates that the key to performance increase on a fixed workload needs to be focused on reducing the proportion of the sequential execution part in the whole task. For more information.

### 3.1.2 Multicore
The core in the CPU may stay idle while waiting for data arrival. HT is designed to reduce the CPU idle states. The CPU supports HT technology, and any physical core may contain logical cores N. When HT is enabled, the software sees two logical cores on one physical core. The technology is to reuse the multiple execution units of the superscalar pipeline more effectively.

Every logical core has its complete and independent register set, and includes local interrupt logic. There is no difference other than a physical thread from the software perspective.

Linux CPU Command Tools

CPU Information Command | Command
--- | ---
The number of cores (physical cores) | cat /proc/cpuinfo \| grep "cpu cores" \| uniq
The number of logical cores | cat /proc/cpuinfo    If “siblings” and “cpu cores” are consistent, it indicates that hyper-threading is not supported or disabled. If "siblings" is double to "cpu cores", it indicates that hyper-threading is supported and enabled.
Socket ID | cat /proc/cpuinfo \| grep "physical id" \| sort \|  uniq \| wc –l or lscpu \| grep "CPU socket"
Get ID for processor | cat /proc/cpuinfo \| grep "processor" \| wc –l

### 3.1.3 Affinity
### 3.1.4 Core Pinning in DPDK

## 3.2 Instruction Concurrency and Data Parallelism
### 3.2.1 Instruction Concurrency
### 3.2.2 SIMD
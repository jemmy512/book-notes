# 3 The Process
## 3.1 Process Descriptor and the Task Structure
### 3.1.1 Allocating the Process Descriptor
### 3.1.Storing the Process Descriptor
### 3.1.Process State
### 3.1.Manipulating the Current Process State
### 3.1.Process Context
### 3.1.The Process Family Tree

## 3.2 Process Creation
### 3.2.1 Copy-on-Write
The only overhead incurred by fork() is the duplication of the parent’s page tables and the creation of a unique process descriptor for the child.

### 3.2.2 Forking
### 3.2.3 vfork()

## 3.3 The Linux Implementation of Threads
### 3.3.1 Creating Threads
### 3.3.2 Kernel Threads

## 3.4 Process Termination
### 3.4.1 Removing the Process Descriptor
### 3.4.2 The Dilemma of the Parentless Task

# 10 Kernel Synchronization Methods
## 10.1 Atomic Operations

Atomic operations provide instructions that execute atomically—without interruption. Just as the atom was originally thought to be an indivisible particle, atomic operators are indivisible instructions. 

The kernel provides two sets of interfaces for atomic operations:
1. one that operates on integers
2. another that operates on individual bits.

### 10.1.1 Atomic Integer Operations

```c++
typedef struct {
    volatile int counter; 
} atomic_t;
```
This special type is used for several rea- sons:
1. having the atomic functions accept only the atomic_t type ensures that the atomic operations are used only with these special types
    * it also ensures that the data types are not passed to any nonatomic functions
2. the use of atomic_t ensures the compiler does not (erroneously but cleverly) optimize access to the value—it is important the atomic operations receive the correct memory address and not an alias. 
3. use of atomic_t can hide any architecture-specific differences in its implementa- tion.The atomic_t type is defined in <linux/types.h>. All the operations implemented on a specific architecture can be found in <asm/atomic.h>.

Common use of the atomic integer:
1. implement counters
2. atomically performing an operation and testing the result


Atomic Integer | Operation Description
--- | ---
ATOMIC_INIT(int i) | At declaration, initialize to i.
int atomic_read(atomic_t *v) | Atomically read the integer value of v.
void atomic_set(atomic_t *v, int i) | Atomically set v equal to i.
void atomic_{ add, sub }(int i, atomic_t *v) | Atomically { add, sub } i to v.
void atomic_{ inc, dec } (atomic_t *v) | Atomically { add, dec } one to v.
int atomic_sub_and_test(int i, atomic_t *v) | Atomically subtract i from v and return true if the result is zero; otherwise false.
int atomic_{ add, sub, inc, dec }_negative(int i, atomic_t *v) | Atomically { add, sub, inc, dec } i to v and return true if the result is negative; otherwise false.
int atomic_{ inc, dec } _and_test(atomic_t *v) | Atomically { inc, dec } v by one and return true if zero; false otherwise.

The atomic operations are typically implemented as inline functions with inline as- sembly.

Atomicity Versus Ordering:
1. a word-sized read always occurs atomically. It never in- terleaves with a write to the same word; the read always returns the word in a consistent state—perhaps before the write completes, perhaps after, but never during. For example, if an integer is initially 42 and then set to 365, a read on the integer always returns 42 or 365 and never some commingling of the two values. We call this **atomicity**.
2. Your code, however, might have more stringent requirements than this: Perhaps you require that the read always occurs before the pending write. This type of requirement is not atomic- ity, but **ordering**. 
3. Atomicity ensures that instructions occur without interruption and that they complete either in their entirety or not at all. 
4. Ordering, on the other hand, ensures that the desired, relative ordering of two or more instructions—even if they are to occur in separate threads of execution or even separate processors—is preserved.
5. The atomic operations discussed in this section guarantee only atomicity. Ordering is en- forced via barrier operations, which we discuss later in this chapter.


In your code, it is usually preferred to choose atomic operations over more compli- cated locking mechanisms. On most architectures, one or two atomic operations incur less overhead and less cache-line thrashing than a more complicated synchronization method.As with any performance-sensitive code, however, testing multiple approaches is always smart.


### 10.1.2 64-Bit Atomic Operations

```c++
typedef struct {
    volatile long counter;
} atomic64_t;
```

### 10.1.3 Atomic Bitwise Operations
## 10.2 Spin Locks
### 10.2.1 Spin Lock Methods
### 10.2.2 Other Spin Lock Methods
### 10.2.3 Spin Locks and Bottom Halves
## 10.3 Reader-Writer Spin Locks
## 10.4 Semaphores
### 10.4.1 Counting and Binary Semaphores
### 10.4.2 Creating and Initializing Semaphores
### 10.4.3 Using Semaphores
## 10.5 Reader-Writer Semaphores
## 10.6 Mutexes
### 10.6.1 Semaphores Versus Mutexes
### 10.6.2 Spin Locks Versus Mutexes
## 10.7 Completion Variables
## 10.8 BKL: The Big Kernel Lock
## 10.9 Sequential Locks
## 10.10 Preemption Disabling
## 10.11 Ordering and Barriers

# 13 The Virtual Filesystem
## The Dentry Object
The VFS often needs to perform directory-specific operations, such as path name lookup. Path name lookup involves translating each component of a path, ensuring it is valid, and following it to the next component. To facilitate this, the VFS employs the concept of a directory entry (dentry). A dentry is a specific component in a path.

Dentry objects are all components in a path, including files. Resolving a path and walking its components is a nontrivial exercise, time-consuming and heavy on string operations, which are expensive to execute and cumbersome to code.The dentry object makes the whole process easier.

Dentries might also include mount points.

The dentry object does not correspond to any sort of on-disk data structure.TheVFS creates it on-the-fly from a string representation of a path name.

### Dentry State
* Three states:
    * **used**: corresponds to a valid inode (d_inode points to an associated inode) and indicates that there are one or more users of the object (d_count is positive). A used dentry is in use by the VFS and points to valid data and, thus, cannot be discarded.
    * **unused**: corresponds to a valid inode (d_inode points to an inode), but the VFS is not currently using the dentry object (d_count is zero). Because the dentry object still points to a valid object, the dentry is kept around—cached—in case it is needed again. If it is necessary to reclaim memory, however, the dentry can be discarded because it is not in active use.
    * **negative**: is not associated with a valid inode (d_inode is NULL) because either the inode was deleted or the path name was never correct to begin with.The dentry is kept around, however, so that future lookups are resolved quickly.

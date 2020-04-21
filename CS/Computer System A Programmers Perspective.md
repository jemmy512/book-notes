# Chapter 1 A Tour of Computer System
### 1.1 Information Is Bits + Context
### 1.2 Programs Are Translated by Other Programs into Different Forms
Compilation System:
1. Preprocessor
2. Compiler
    Each statement in an assembly-language program exactly describes one low-level machine-language instruction in a standard text form.
    Assembly language is useful because it provides a common output language for different compilers for different high-level languages.
3. Sssembler
    Assembler (as) translates hello.s into machine-language instructions, packages them in a form known as a relocatable object program, and stores the result in the object file.
4. Linker

#### 1.4.1 Hardware Organization of a System
![Hadrware](../Images/system-hardware.png)
CPU: Central Processing Unit
ALU: Arithmetic/Logic Unit
PC: Program counter
USB: Universal Serial Bus.

### 1.7 The Operating System Manages the Hardware
The operating system has two primary purposes:
1. to protect the hardware from misuse by runaway applications
2. to provide applications with simple and uniform mechanisms for manipulating complicated and often wildly different low-level hardware devices.

**Files** are abstractions for I/O devices, **virtual memory** is an abstraction for both the main memory and disk I/O devices, and **processes** are abstractions for the processor, main memory, and I/O devices. The **virtual machine** provids an abstraction of the entire computer, including the operating system, the processor, and the programs.

Excerpt From: Bryant, Randal E. “Computer Systems: A Programmer's Perspective (2nd Edition).” Apple Books.

Posix standards, that cover such issues as the C language interface for Unix system calls, shell programs and utilities, threads, and network programming.

#### 1.7.1 Processes
A process is the operating system’s abstraction for a running program. Multiple processes can run concurrently on the same system, and each process appears to have exclusive use of the hardware.

Context, includes information such as the current values of the PC, the register file, and the contents of main memory.

#### 1.7.2 Threads
A process can actually consist of multiple execution units, called threads, each running in the context of the process and sharing the same code and global data.

It is easier to share data between multiple threads than between multiple processes, and because threads are typically more efficient than processes.

#### 1.7.3 Virutal Memory
Virtual memory is an abstraction that provides each process with the illusion that it has exclusive use of the main memory.

### 1.9 Important Themes
#### 1.9.1 Concurrency and Parallelism
We use the term concurrency to refer to the general concept of a system with multiple, simultaneous activities, and the term parallelism to refer to the use of concurrency to make a system run faster.

##### Thread-Level Concurrency
The use of multiprocessing can improve system performance in two ways:
1. It reduces the need to simulate concurrency when performing multiple tasks.
2. It can run a single application program faster, but only if that program is expressed in terms of multiple threads that can effectively execute in parallel.

##### Instruction-Level Parallelism

##### Single-Instruction, Multiple-Data (SIMD) Parallelism

# Chapter 2 Representing and Manipulating Information
### 2.1 Information Storage
#### 2.1.4 Addressing and Byte Ordering
Byte ordering becomes an issue:
1. The first is when binary data are communicated over a network between different machines.
2. A second case where byte ordering becomes important is when looking at the byte sequences representing integer data.
3. A third case where byte ordering becomes visible is when programs are written that circumvent the normal type system.

#### 2.2.4 Conversions between Signed and Unsigned

The effect of casting is to keep the bit values identical but change how these bits are interpreted.

#### 2.2.5 Signed vs. Unsigned in C

Some of the peculiar behavior arises due to C’s handling of expressions containing combinations of signed and unsigned quantities. When an operation is performed where one operand is signed and the other is unsigned, C implicitly casts the signed argument to unsigned and performs the operations assuming the numbers are nonnegative. As we will see, this convention makes little difference for standard arithmetic operations, but it leads to nonintuitive results for relational operators such as < and >.

    Consider the comparison -1 < 0U. Since the second operand is unsigned, the first one is implicitly cast to unsigned, and hence the expression is equivalent to the comparison 4294967295U < 0U (recall that T2Uw (−1) = UMaxw), which of course is false.
#### 2.2.6 Expanding the Bit Representation of a Number
To convert an unsigned number to a larger data type, we can simply add *leading zeros* to the representation; this operation is known as **zero extension**.

For converting a two’s-complement number to a larger data type, the rule is to perform a **sign extension**, adding copies of the *most significant bit* to the representation.

    short sx = -1234;
    unsigned uy = sx;
    printf("uy = %u\n", uy); // 4294954951: ff ff cf c7

    when converting from short to unsigned, we first change the size and then from signed to unsigned. That is, (unsigned) sx is equivalent to (unsigned) (int) sx, evaluating to 4,294,954,951, not (unsigned) (unsigned short) sx, which evaluates to 53,191. Indeed this convention is required by the C standards.

#### 2.2.7 Truncating Numbers
For an unsigned number x, the result of truncating it to k bits is equivalent to computing x mod 2k.


### 2.3 Integer Arithmetic
> TODO

### 2.4 Floating Point
#### 2.4.1 Fractional Binary Number

# Chapter 3 Machine-Level Representation of Programs

#### 3.2.1 Machine-Level Code
* The program counter (commonly referred to as the “PC,” and called %eip in IA32) indicates the address in memory of the next instruction to be executed.
* *The integer register file contains eight named locations storing 32-bit values. These registers can hold addresses (corresponding to C pointers) or integer data. Some registers are used to keep track of critical parts of the program state, while others are used to hold temporary data, such as the local variables of a procedure, and the value to be returned by a function.
* *The condition code registers hold status information about the most recently executed arithmetic or logical instruction. These are used to implement conditional changes in the control or data flow, such as is required to implement if and while statements.
* *A set of floating-point registers store floating-point data.

#### 3.2.3 Code Example
Several features about machine code and its disassembled representation are worth noting:
* IA32 instructions can range in length from 1 to 15 bytes. The instruction encoding is designed so that commonly used instructions and those with fewer operands require a smaller number of bytes than do less common ones or ones with more operands.
* The instruction format is designed in such a way that from a given starting position, there is a unique decoding of the bytes into machine instructions. For example, only the instruction pushl %ebp can start with byte value 55.
* The disassembler determines the assembly code based purely on the byte sequences in the machine-code file. It does not require access to the source or assembly-code versions of the program.
* The disassembler uses a slightly different naming convention for the instructions than does the assembly code generated by GCC. In our example, it has omitted the suffix ‘1’ from many of the instructions. These suffixes are size designators and can be omitted in most cases.

### 3.3 Data Formts
    ![C Data Formats](../Images/data-formats.png)

### 3.4 Accessing Information
    ![Accessing Information](../Images/accessing-information.png)

#### 3.4.1 Operand Specifiers
    ![Operand Forms](../Images/operand-forms.png)

#### 3.4.2 Data Movement Instructions
    ![Data Movement Instructions](../Images/data-movement-instructions.png)

With sign expansion, the upper bits of the destination are filled in with copies of the most significant bit of the source value. With zero expansion, the upper bits are filled with zeros.
```assembly
// %dh = CD, %eax = 98765432
movb    %dh %eax;   // %eax = 987654CD
movsbl  %dh %eax;   // %eax = FFFFFFCD
movzbl  %dh %eax;   // %eax = 000000CD
```



#### 3.4.3

### 3.5 Arithmatic and Logic Operations
### 3.5.1 Load Affect Address



# Chapter 10 System-Level I/O

### Unix I/O
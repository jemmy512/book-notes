# A Tour of Computer System
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

    Files are abstractions for I/O devices, virtual memory is an abstraction for both the main memory and disk I/O devices, and processes are abstractions for the processor, main memory, and I/O devices.

    Posix standards, that cover such issues as the C language interface for Unix system calls, shell programs and utilities, threads, and network programming.

#### 1.7.1 Processes
    A process is the operating system’s abstraction for a running program. Multiple processes can run concurrently on the same system, and each process appears to have exclusive use of the hardware.

    Context, includes information such as the current values of the PC, the register file, and the contents of main memory.

#### 1.7.2 Threads
    A process can actually consist of multiple execution units, called threads, each running in the context of the process and sharing the same code and global data.

    It is easier to share data between multiple threads than between multiple processes, and because threads are typically more efficient than processes.

#### 1.7.3 Virutal Memory
    Virtual memory is an abstraction that provides each process with the illusion that it has exclusive use of the main memory.

# 10 System-Level I/O

### Unix I/O
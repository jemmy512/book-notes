# 2 Compiling and Linking
## 2.1 The process of being hidden
![](../Images/LinkLoadLibrary/2.1-compilation-process.png)

## 2.2 What does the compiler do
![](../Images/LinkLoadLibrary/2.2-compilation-process.png)

* **Lexical analysis**

    First, the source code program is input to the scanner. It simply performs lexical analysis. Using an algorithm similar to a finite state machine (Finite State Machine) can easily convert the source code. The character sequence is divided into a series of tokens

    Tokens includes: keywords, identifiers, literals (Including numbers, strings, etc.) and special symbols (such as plus signs, equal signs).

* **Grammar Parser**

    The Grammar Parser will perform grammatical analysis on the tokens to generate a Syntax Tree. The whole analysis process uses the analysis method of Context-free Grammar

    At the same time of grammatical analysis, the priority and meaning of many arithmetic symbols have also been determined.

* **Semantic Analyzer**

    The grammatical analysis only completes the analysis of the grammatical level of the expression, but it does not know whether the sentence is really meaningful. For example, it is meaningless to do multiplication of two pointers in C language, but this statement is grammatically legal.

    The semantics that the compiler can analyze is static semantics. The so-called static semantics refers to the semantics that can be determined at compile time, and the corresponding dynamic semantics is the semantics that can only be determined at runtime.

    Static semantics usually include **declaration** and **type matching**, and **type conversion**. For example, when a floating-point expression is assigned to an integer expression, it implies a floating-point to integer conversion process. This step needs to be completed in the semantic analysis process. For example, when assigning a floating-point type to a pointer, the semantic analysis program will find that the type does not match, and the compiler will report an error. Dynamic semantics generally refers to semantic-related problems that occur at runtime. For example, using 0 as a divisor is a runtime semantic error.

    After the semantic analysis stage, the expressions of the entire syntax tree are marked with types. If some types need to be implicitly converted, the semantic analysis program will insert the corresponding conversion nodes in the syntax tree.

* **Code Generator**
    The code generator converts the intermediate code into the object machine code. This process is very dependent on the object machine, because different machines have different word lengths, registers, integer data types, and floating-point data types.

* **Code Optimizer**
    Selecting the appropriate addressing method, using displacement instead of multiplication, deleting redundant instructions, etc.

## 2.3 The linker is older than the compiler
The concept of **Symbol** is quickly used with the popularity of assembly language. It is used to represent an address. This address may be the starting address of a subroutine (later developed into a function) or the starting address of a variable.

## 2.4 Module assembly-static link
The linking process mainly includes the steps of
* Address
* Storage Allocation
* Symbol Resolution
* Relocation.

# 3 What's in the object file

## 3.1 The format of the object file
The popular executable file format (Executable) of the PC platform is mainly **PE** (Portable Executable) under Windows and **ELF** (Executable Linkable Format) of Linux, both of which are variants of COFF (Common file format).

The static link library is slightly different. It bundles many object files together to form a file, plus some indexes, you can simply understand it as a file package containing many object files.

```
$ file foobar.o
foobar.o: ELF 32-bit LSB relocatable, Intel 80386, version 1 (SYSV), not stripped

$ file /bin/bash
/bin/bash: ELF 32-bit LSB executable, Intel 80386, version 1 (SYSV), for GNU/Linux 2.6.8, dynamically

$ file /lib/ld-2.6.1.so
/lib/libc-2.6.1.so: ELF 32-bit LSB shared object, Intel 80386, version 1 (SYSV), for GNU/Linux 2.6.8, stripped

 ```

## 3.2 What does the object file look like?
![](../Images/LinkLoadLibrary/3.2-object-file.png)

BSS(Block Started by Symbol), Unix FAQ section 1.3(http://www.faqs.org/faqs/unix-faq/faq/part1/section-3.html

Separating data and instruction has many benefits:
* On the one hand, when the program is loaded, the data and instructions are respectively mapped to two virtual memory areas. The permissions of these two virtual storage areas can be set to read-write and read-only respectively. This prevents program instructions from being rewritten intentionally or unintentionally.
* Improving perform through data cache and instruction cache
* When multiple copies of the program are running in the system, their instructions are the same, so only one copy of the program's instruction needs to be stored in the memory.

## 3.3 Mining SimpleSection.o

## 3.4 ELF file structure description

## 3.5 Linked interface-symbol

In linking, the merging of object files is actually a reference to addresses between object files, that is, references to addresses of functions and variables.

* **ELF symble table struct**

```c++
typedef struct elf64_sym {
  Elf64_Word    st_name;  /* Symbol name, index in string tbl */
  unsigned char st_info;  /* Type and binding attributes: STB_LOCAL, STB_GLOBAL, STB_WEAK | STT_NOTYPE, STT_OBJECT, STT_FUNC */
  unsigned char st_other; /* No defined meaning, 0 */
  Elf64_Half    st_shndx; /* Associated section index: SHN_ABS, SHN_COMMON, SHN_UNDEF */
  Elf64_Addr    st_value; /* Value of the symbol */
  Elf64_Xword   st_size;  /* Associated symbol size */
} Elf64_Sym;
```

* **Special Symble**

```c++
__executable_start // the start address of the program, not entry
__etext or etext or etext // the end address of .text
edata or edata // the end address of .data
_end or end // the end address of the program
```

```c++
#include <stdio.h>

extern char __executable_start[];
extern char etext[], _etext[], __etext[];
extern char edata[], _edata[];
extern char end[], _end[];

int main() {
    printf("Executable Start %X\n", __executable_start);
    printf("Text End %X %X %X\n", etext, _etext, __etext);
    printf("Data End %X %X\n", edata, _edata);
    printf("Executable End %X %X\n", end, _end);
}
```

* **C++ Name Mangling**
    The basic C++ name modification method of GCC is as follows: All symbols start with "_Z", for nested names (in the namespace or in the class), followed by "N", then the lenght of each name, and then each name space and class Name, and then ends with "E".

    ```c++
    int C::C2::func(int) -> _ZN1C2C24funcEi
    ```

    ```c++
    #ifdef __cplusplus
    extern "C" {
        printf( "%d\n", _ZN6myname3varEi );
    #endif
        void *memset (void *, int, size_t);
    #ifdef __cplusplus
        }
    #endif
    ```

* **Strong Symbol and Weak Symbol**

    For the C/C++ language, the compiler default functions and initialized global variables are strong symbols, and uninitialized global variables are weak symbols. We can also define any strong symbol as a weak symbol through GCC's "__attribute__((weak))". Note that both strong symbols and weak symbols are for definitions, not for symbol references.

    Rule processing and selection of global symbols that have been defined multiple times:
    1. Strong symbols are not allowed to be defined multiple times (that is, different object files cannot have strong symbols with the same name); if there are multiple strong symbol definitions, the linker reports a symbol duplication definition error.
    2. If a symbol is a strong symbol in a certain object file, but is a weak symbol in other files, then choose a strong symbol.
    3. If a symbol is a weak symbol in all object files, choose the one that occupies the largest space. For example, object file A defines global variable global as int type, which occupies 4 bytes; object file B defines global as double type, which occupies 8 bytes, then after object file A and B are linked, the symbol global occupies 8 bytes ( Try not to use multiple weak symbols of different types, otherwise it will easily lead to program errors that are difficult to find).

* **Weak references and strong references**

    The symbol references we have seen to external object files so far need to be correctly resolved when the object file is finally linked into an executable file. If the definition of the symbol is not found, the linker will report the symbol Undefined error, this is called a **strong reference**.

    When dealing with **weak references**, if the symbol is defined, the linker determines the symbol's reference; if the symbol is not defined, the linker does not treat the reference as an error.

    The linker processes strong references and weak references in almost the same way, except that for undefined weak references, the linker does not consider it an error. Generally, for undefined weak references, the linker defaults it to 0, or a special value, so that the program code can recognize it.

    We can declare a reference to an external function as a weak reference by using the extended keyword "__attribute__((weakref))", such as the following code:

    ```c++
    __attribute__ ((weakref)) void foo();

    int main() {
        foo();
    }
    ```

    Such weak symbols and weak references are very useful for libraries. For example, the weak symbols defined in the library can be overwritten by user-defined strong symbols, so that the program can use a custom version of the library function; or the program can define some extended function module as a weak reference. When we link the extension module with the program, the function module can be used normally; if we remove some function modules, the program can also be linked normally, but the corresponding function is missing , Which makes it easier to cut and combine the functions of the program.

    ```c++
    #include <stdio.h>
    #include <pthread.h>

    int pthread_create( pthread_t*, const pthread_attr_t*, void* (*)(void*), void*) __attribute__ ((weak));

    int main()
    {

        if (pthread_create) {
            printf("This is multi-thread version!\n");
            // run the multi-thread version
            // main_multi_thread()
        } else {
            printf("This is single-thread version!\n");
            // run the single-thread version
            // main_single_thread()
        }
    }

    gcc pthread.c -o pt // single-thread
    gcc pthread.c -o pt -lpthread // multi-thread
    ```

## 3.6 Debugging information

# 4 Static link

```c++
// a.c
extern int shared;
extern int swap(int* a, int *b);

int main() {
    int a = 100;
    swap(&a, &shared);
}

// b.c
int shared = 1;

int swap(int* a, int *b) {
    *a ^= *b ^= *a ^= *b;
}
```

## 4.1 Space and address allocation

* **Similar section merge**
    ![](../Images/LinkLoadLibrary/4.1-section-merge.png)

    As we mentioned earlier, the ".bss" section does not occupy file space in object files and executable files, but it occupies address space when loading. Therefore, when the linker merges each section, it also merges ".bss" and allocates virtual space.

    "The linker allocates address and space for the object file" actually has two meanings:
    1. the space in the output executable file
    2. the virtual address space after loading

    For sections with actual data, such as ".text" and ".data", they must allocate space in the file and in the virtual address space, because they exist in both; and for ".bss", the meaning of allocated space is limited to the virtual address space, because it has no content in the file.

    * Two-pass linking:
        * **Space and address allocation**: scans all input object files, and obtains the length, attributes and positions of their respective sections, and collects all symbol definitions and symbol references in the symbol table in the input object file, and puts them together A global symbol table. In this step, the linker will be able to obtain the section lengths of all input object files, merge them, calculate the combined length and position of each section in the output file, and establish a mapping relationship.

        * **Symbol resolution and relocation**: Use all the information collected in the first step above to read the data and relocation information in the middle of the input file, and perform symbol resolution and relocation, and adjust the address in the code. In fact, the second step is the core of the linking process, especially the relocation process.

    ```c++
    gcc -c a.c b.c

    // -e main means that the main function is used as the program entry
    // the default program entry of the ld linker is _start symble
    ld a.o b.o -e main -o ab
    ```

    ```c++
    // VMA: virtual memory address
    // LMA: load memory address

    [root@VM-16-17-centos code]# objdump -h a.o
    a.o:     file format elf64-x86-64

    Sections:
    Idx Name          Size      VMA               LMA               File off  Algn
    0 .text         00000027  0000000000000000  0000000000000000  00000040  2**0
                    CONTENTS, ALLOC, LOAD, RELOC, READONLY, CODE
    1 .data         00000000  0000000000000000  0000000000000000  00000067  2**0
                    CONTENTS, ALLOC, LOAD, DATA
    2 .bss          00000000  0000000000000000  0000000000000000  00000067  2**0
                    ALLOC
    3 .comment      0000002d  0000000000000000  0000000000000000  00000067  2**0
                    CONTENTS, READONLY
    4 .note.GNU-stack 00000000  0000000000000000  0000000000000000  00000094  2**0
                    CONTENTS, READONLY
    5 .eh_frame     00000038  0000000000000000  0000000000000000  00000098  2**3
                    CONTENTS, ALLOC, LOAD, RELOC, READONLY, DATA
    ```

    ```c++
    [root@VM-16-17-centos code]# objdump -h b.o
    b.o:     file format elf64-x86-64

    Sections:
    Idx Name          Size      VMA               LMA               File off  Algn
    0 .text         0000004b  0000000000000000  0000000000000000  00000040  2**0
                    CONTENTS, ALLOC, LOAD, READONLY, CODE
    1 .data         00000004  0000000000000000  0000000000000000  0000008c  2**2
                    CONTENTS, ALLOC, LOAD, DATA
    2 .bss          00000000  0000000000000000  0000000000000000  00000090  2**0
                    ALLOC
    3 .comment      0000002d  0000000000000000  0000000000000000  00000090  2**0
                    CONTENTS, READONLY
    4 .note.GNU-stack 00000000  0000000000000000  0000000000000000  000000bd  2**0
                    CONTENTS, READONLY
    5 .eh_frame     00000038  0000000000000000  0000000000000000  000000c0  2**3
                    CONTENTS, ALLOC, LOAD, RELOC, READONLY, DATA
    ```

    ```c++
    [root@VM-16-17-centos code]# objdump -h ab
    ab:     file format elf64-x86-64

    Sections:
    Idx Name          Size      VMA               LMA               File off  Algn
    0 .text         00000072  00000000004000e8  00000000004000e8  000000e8  2**0
                    CONTENTS, ALLOC, LOAD, READONLY, CODE
    1 .eh_frame     00000058  0000000000400160  0000000000400160  00000160  2**3
                    CONTENTS, ALLOC, LOAD, READONLY, DATA
    2 .data         00000004  0000000000601000  0000000000601000  00001000  2**2
                    CONTENTS, ALLOC, LOAD, DATA
    3 .comment      0000002c  0000000000000000  0000000000000000  00001004  2**0
                    CONTENTS, READONLY
    ```

    ![](../Images/LinkLoadLibrary/4.1-load-vma.png)



## 4.2 Symbol resolution and relocation

## 4.3 COMMON block

## 4.4 C++ related issues

## 4.5 Static library link

## 4.6 Link process control

## 4.7 BFD library


# 6 Executable file loading and process
## 6.1 Process virtual address space
## 6.2 Method of loading
## 6.3 The loading from the operating system view
## 6.4 Process virtual memory space distribution
## 6.5 Introduction to Linux Kernel Loading ELF Process
## 6.6 Loading of Windows PE

# 7 Dynamic link
## 7.1 Why dynamic link
## 7.2 Simple dynamic link example
## 7.3 Address independent code
## 7.4 Delayed binding (PLT)
## 7.5 Dynamic link related structure
## 7.6 Steps and implementation of dynamic linking
## 7.7 Explicit runtime linking

# 8 Organization of Linux shared libraries
## 8.1 Shared library version
## 8.2 Symbol version
## 8.3 Shared library system path
## 8.4 Shared library search process
## 8.5 Environment variables
## 8.6 Creation and installation of shared libraries

# 10 Memory
## 10.1 Program memory layout
## 10.2 Stack and calling convention
## 10.3 Heap and memory management

# 11 Runtime library
## 11.1 Entry function and program initialization
## 11.2 C/C++ runtime library
## 11.3 Runtime library and multithreading
## 11.4 C++ global construction and destruction
## 11.5 fread implementation

# 12 System calls and API
## 12.1 Introduction to System Call
## 12.2 Principle of System Call

# 13 Runtime library implementation
## 13.1 C language runtime library
## 13.2 How to use Mini CRT
## 13.3 Implementation of C++ runtime library
## 13.4 How to use Mini CRT++
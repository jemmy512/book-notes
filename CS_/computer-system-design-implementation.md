# 1 INTRODUCTION
## 1.1 WHAT IS AN OPERATING SYSTEM?
### 1.1.1 The Operating System as an Extended Machine
### 1.1.2 The Operating System as a Resource Manager
## 1.2 HISTORY OF OPERATING SYSTEMS
### 1.2.1 The First Generation (1945-55) Vacuum Tubes and Plugboards
### 1.2.2 The Second Generation (1955-65) Transistors and Batch Systems
### 1.2.3 The Third Generation (1965-1980) ICs and Multiprogramming
### 1.2.4 The Fourth Generation (1980-Present) Personal Computers
### 1.2.5 History of MINIX
## 1.3 OPERATING SYSTEM CONCEPTS
### 1.3.1 Processes
### 1.3.2 Files
### 1.3.3 The Shell
## 1.4 SYSTEM CALLS
### 1.4.1 System Calls for Process Management
### 1.4.2 System Calls for Signaling
### 1.4.3 System Calls for File Management
### 1.4.4 System Calls for Directory Management
### 1.4.5 System Calls for Protection
### 1.4.6 System Calls for Time Management
## 1.5 OPERATING SYSTEM STRUCTURE
### 1.5.1 Monolithic Systems
### 1.5.2 Layered Systems
### 1.5.3 Virtual Machines
### 1.5.4 Exokernels
### 1.5.5 Client-Server Model
## 1.6 OUTLINE OF THE REST OF THIS BOOK

# 2 PROCESSES
## 2.1 INTRODUCTION TO PROCESSES
### 2.1.1 The Process Model
### 2.1.2 Process Creation
### 2.1.3 Process Termination
### 2.1.4 Process Hierarchies
### 2.1.5 Process States
### 2.1.6 Implementation of Processes
### 2.1.7 Threads
## 2.2 INTERPROCESS COMMUNICATION
### 2.2.1 Race Conditions
### 2.2.2 Critical Sections
### 2.2.3 Mutual Exclusion with Busy Waiting
### 2.2.4 Sleep and Wakeup
### 2.2.5 Semaphores
### 2.2.6 Mutexes
### 2.2.7 Monitors
### 2.2.8 Message Passing
## 2.3 CLASSICAL IPC PROBLEMS
### 2.3.1 The Dining Philosophers Problem
### 2.3.2 The Readers and Writers Problem

## 2.4 SCHEDULING
### 2.4.1 Introduction to Scheduling
### 2.4.2 Scheduling in Batch Systems
### 2.4.3 Scheduling in Interactive Systems
### 2.4.4 Scheduling in Real-Time Systems
### 2.4.5 Policy versus Mechanism
### 2.4.6 Thread Scheduling
## 2.5 OVERVIEW OF PROCESSES IN MINIX
### 2.5.1 The Internal Structure of MINIX
### 2.5.2 Process Management in MINIX
### 2.5.3 Interprocess Communication in MINIX
### 2.5.4 Process Scheduling in MINIX
## 2.6 IMPLEMENTATION OF PROCESSES IN MINIX
### 2.6.1 Organization of the MINIX 3 Source Code
### 2.6.2 Compiling and Runniing MINIX
### 2.6.3 The Common Header Files
### 2.6.4 The MINIX 3 Header Files
### 2.6.5 Process Data Structures and Header Files
### 2.6.6 Bootstrapping MINIX
### 2.6.7 System Initialization
### 2.6.8 Interrupt Handling in MINIX
### 2.6.9 Interprocess Communication in MINIX
### 2.6.10 Scheduling in MINIX
### 2.6.11 Hardware-Dependent Kernel Support
### 2.6.12 Utilities and the Kernel Library
## 2.7 THE SYSTEM TASK IN MINIX
### 2.7.1 Overview of the System Task
### 2.7.2 Implementation of the System Task
### 2.7.3 Implementation of the System Libarary
## 2.9 THE CLOCK TASK IN MINIX
### 2.8.1 Clock Hardware
### 2.8.2 Clock Software
### 2.8.3 Overview of the Clock Driver in MINIX
### 2.8.4 Implementation of the Clock Driver in MINIX

# 3 INPUT/OUTPUT
## 3.1 PRINCIPLES OF I/O HARDWARE
### 3.1.1 I/O Devices
### 3.1.2 Device Controllers
### 3.1.3 Memory-Mapped I/O
### 3.1.4 Interrupts
### 3.1.5 Direct Memory Access

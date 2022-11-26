[:link: Source](https://www.educative.io/module/concurrency-with-modern-cpp)

# 1. A Quick Overview

## Introduction
## C++11 and C++14: The Foundation
## Multithreading in C++
## Case Studies
## C++17: Parallel Algorithms of the Standard Template Library
## C++20: The Concurrent Future
# 3. Memory Model: The Contract

## The Contract
## The Foundation & Challenges
## 3. Memory Model: Atomics

## Strong Memory Model
## Weak Memory Model
## The Atomic Flag
## Spinlock vs. Mutex
## std::atomic<bool>
## User Defined Atomics
## All Atomic Operations
## Atomic Operations on std::shared_ptr
## 4. Memory Model: Synchronization and Ordering Constraints

## Introduction
## Types of Synchronization & Ordering Constraints
## Sequential Consistency
## Acquire Release Semantic
## Is the Acquire-Release Semantic Transitive?
## Acquire Release: The Typical Misunderstanding
## std::mem_order_consume
## Data dependencies with std::memory_order_consume
## Relaxed Semantic
## 5. Memory Model: Fences

## Fences as Memory Barriers
## The Three Fences
## Acquire and Release Fences
## Synchronization with Atomic Variables
## Synchronization with Fences
## 6. Multithreading: Threads

## Introduction to Threads
## Creation of Threads
## Managing Thread Lifetime
## Thread Lifetime Management: Warnings and Tips
## Passing Arguments to Threads
## Arguments of Threads: Undefined behavior
## Arguments of Threads - Race Conditions and Locks
## Methods of Threads
## Methods of Threads in Practice
## 7. Multithreading: Shared Data

## Introduction to Shared Data
## Introduction to Mutexes
## Mutex Types and Locking Methods
## Issues of Mutexes: Deadlocks
## Issues of Mutexes: Avoiding Exceptions
## Types of Locks: std::lock_guard
## Types of Locks: std::unique_lock
## Types of Locks: std::shared_lock
## Thread-Safe Initialization
## Thread-Safe Initialization: Constant Expressions
## Thread-Safe Initialization: call_once and once_flag
## Thread-Safe Initialization - Static Variables with Block Scope
## 8. Multithreading: Local Data

## Thread Local Data
## Condition Variables
## The Caveats of Condition Variables
## 9. Multithreading: Tasks

## Introduction to Tasks
## Threads vs Tasks
## Introduction to std::async
## async: Start Policy
## async: Fire and Forget
## async: Concurrent Calculation
## Introduction to std::packaged_task
## Introduction to Promises and Futures
## Promise and Future : Return an Exception
## Promise and Future: Return a Notification
## Introduction to std::shared_future
# 10. Case Study: Calculate Sum of a Vector

## Introduction
## Single Threaded Summation: Ranged Based for Loops
## Single Threaded Summation: Addition with std::accumulate
## Single Threaded Summation: Protection with Locks
## Single Threaded Summation: Protection with Atomics
## Multithreaded Summation: Using std::lock_guard
## Multithreaded Summation: Using Atomic Variable
## Multithreaded Summation: Using fetch_add Method
## Multithreaded Summation: Using fetch_add Method with Relaxed Semantic
## Thread Local Summation: Using Local Variable
## Thread Local Summation: Using an Atomic Variable with Sequential Consistency
## Thread Local Summation: Using an Atomic Variable with Relaxed Semantic
## Thread Local Summation: Using Thread Local Data
## Thread Local Summation: Using Tasks
## Calculate Sum of a Vector: Conclusion
# 11. Case Study: Thread-Safe Initialization of a Singleton

## Introduction
## Double-Checked Locking Pattern
## Performance Measurement
## Classical Meyers Singleton
## Introduction to Thread-Safe Meyers Singleton
## Thread-Safe Singleton: std::lock_guard
## Thread-Safe Singleton: std::call_once with std::once_flag
## Thread-Safe Singleton: Atomics
## Conclusion
# 12. Case Study: Ongoing Optimization with CppMem

## Introduction to CppMem
## CppMem: An Overview
## CppMem: Non-Atomic Variables
## CppMem: Locks
## CppMem: Atomics with Sequential Consistency
## CppMem: Atomics with an Acquire-Release Semantic
## CppMem: Atomics with Non-Atomics
## CppMem: Atomics with a Relaxed Semantic
## Conclusion
# 13. Parallel Algorithms of the Standard Template Library

## Execution Policies
## Parallel & Vectorized Execution
## Algorithms
## The New Algorithms
## New Algorithms - A Functional Perspective
# 14. The Future: C++20

## Atomic Smart Pointers
## Thread-Safe Linked List Using Atomic Pointers
## Introduction to Extended Futures
## Attaching Extended Futures
## Creating New Futures
## Latches and Barriers
## Transactional Memory - An Overview
## The Two Flavors of Transactional Memory
## Introduction to Coroutines
## Coroutines: More Details
## Task Blocks
# 15. Coding Examples

## ABA
## Blocking Issues
## Breaking of Program Invariants
## Data Races
## False Sharing
## Lifetime Issues of Variables
## Moving Threads
## Deadlocks
## Race Conditions
# 16. Best Practices

## General
## Memory Model
## Multithreading: Threads
## Multithreading: Shared Data
## Multithreading: Condition Variables
# 17. The Time Library

## The Interplay of Time Point, Time Duration, and Clock
## Time Point
## From Time Point to Calendar Time
## Time Duration
## Time Duration Calculations
## Clocks
## Sleep and Wait
# 18. Glossary

## Glossary
## Running Source Code on your own machine
# 19. Conclusion

## Final Remarks
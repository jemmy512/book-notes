# PART I Foundations of Data Systems

# 1. Reliable, Scalable, and Maintainable Applications.

A data-intensive application is typically built from standard building blocks that provide commonly needed functionality:
* Store data so that they, or another application, can find it again later (**databases**)
* Remember the result of an expensive operation, to speed up reads (**caches**)
* Allow users to search data by keyword or filter it in various ways (**search indexes**)
* Send a message to another process, to be handled asynchronously (**stream processing**)
* Periodically crunch a large amount of accumulated data (**batch processing**)

## 1.1 Thinking About Data Systems

![](../Images/DDIA/1.1-data-system.png)

There are many factors that may influence the design of a data system, including the `skills and experience` of the people involved, `legacy system dependencies`, the `timescale` for delivery, your organization’s `tolerance` of different kinds of risk, regulatory constraints.

Reliability
* The system should continue to work correctly (performing the correct function at the desired level of performance) even in the face of adversity (hardware or software faults, and even human error).

Scalability
* As the system grows (in data volume, traffic volume, or complexity), there should be reasonable ways of dealing with that growth.

Maintainability
* Over time, many different people will work on the system (engineering and operations, both maintaining current behavior and adapting the system to new use cases), and they should all be able to work on it productively.

## 1.2 Reliability

For software, typical expectations include:
* The application performs the function that the user expected.
* It can tolerate the user making mistakes or using the software in unexpected ways.
* Its performance is good enough for the required use case, under the expected load and data volume.
* The system prevents any unauthorized access and abuse.

**Reliability**: continuing to work correctly, even when things go wrong.

A **fault** is usually defined as one component of the system deviating from its spec, whereas a **failure** is when the system as a whole stops providing the required service to the user. It is impossible to reduce the probability of a fault to zero; therefore it is usually best to design fault-tolerance mechanisms that prevent faults from causing failures.

### 1.2.1 Hardware Faults

Hard disks crash, RAM becomes faulty, the power grid has a blackout, someone unplugs the wrong network cable.

Our first response is usually to add redundancy to the individual hardware components in order to reduce the failure rate of the system:
* Disks may be set up in a RAID configuration
* servers may have dual power supplies and hot-swappable CPUs
* datacenters may have batteries and diesel generators for backup power

When one component dies, the redundant component can take its place while the broken component is replaced.

### 1.2.2 Software Errors

We usually think of hardware faults as being random and independent from each other.

Solution:
* carefully thinking about assumptions and interactions in the system
* thorough testing
* process isolation
* allowing processes to crash and restart
* measuring, monitoring, and analyzing system behavior in production.

### 1.2.3 Human Errors
Solutions:
* Design systems in a way that minimizes opportunities for error. For example, well-designed abstractions, APIs, and admin interfaces make it easy to do “the right thing” and discourage “the wrong thing.” However, if the interfaces are too restrictive people will work around them, negating their benefit, so this is a tricky balance to get right.
* Decouple the places where people make the most mistakes from the places where they can cause failures. In particular, provide fully featured non-production sandbox environments where people can explore and experiment safely, using real data, without affecting real users.
* Test thoroughly at all levels, from unit tests to whole-system integration tests and manual tests [3]. Automated testing is widely used, well understood, and especially valuable for covering corner cases that rarely arise in normal operation.
* Allow quick and easy recovery from human errors, to minimize the impact in the case of a failure. For example, make it fast to roll back configuration changes, roll out new code gradually (so that any unexpected bugs affect only a small subset of users), and provide tools to recompute data (in case it turns out that the old computation was incorrect).
* Set up detailed and clear monitoring, such as performance metrics and error rates. In other engineering disciplines this is referred to as telemetry. (Once a rocket has left the ground, telemetry is essential for tracking what is happening, and for understanding failures [14].) Monitoring can show us early warning signals and allow us to check whether any assumptions or constraints are being violated. When a problem occurs, metrics can be invaluable in diagnosing the issue.
* Implement good management practices and training—a complex and important aspect, and beyond the scope of this book.

## 1.3 Scalability

**Scalability** is the term we use to describe a system’s ability to cope with increased load.

### 1.3.1 Describing Load

The best choice of parameters depends on the architecture of your system:
* requests per second to a web server
* the ratio of reads to writes in a database
* the number of simultaneously active users in a chat room
* the hit rate on a cache

### 1.3.2 Describing Performance

You can investigate what happens when the load increases:
* When you increase a load parameter and keep the system resources (CPU, mem‐ ory, network bandwidth, etc.) unchanged, how is the performance of your system affected?
* When you increase a load parameter, how much do you need to increase the resources if you want to keep performance unchanged.

There are algorithms that can calculate a good approximation of percentiles at minimal CPU and memory cost, such as `forward decay`, `t-digest`, or `HdrHistogram`.

### 1.3.3 Approaches for Coping with Load

How do we maintain good performance even when our load parameters increase by some amount?
* An **architecture** that is appropriate for one level of load is unlikely to cope with 10 times that load.
* People often talk of a dichotomy between **scaling up** (vertical scaling, moving to a more powerful machine) and **scaling out** (horizontal scaling, distributing the load across multiple smaller machines).
* Some systems are **elastic**, meaning that they can automatically add computing resour‐ ces when they detect a load increase, whereas other systems are scaled manually
* While distributing **stateless services** across multiple machines is fairly straightfor‐ ward, taking stateful data systems from a single node to a distributed setup can intro‐ duce a lot of additional complexity.
* As the tools and abstractions for **distributed systems** get better, this common wisdom may change, at least for some kinds of applications.
* The architecture of systems that operate at large scale is usually highly specific to the application—there is no such thing as a generic, **one-size-fits-all** scalable architecture.

## 1.4 Maintainability
* Operability: Make it easy for operations teams to keep the system running smoothly.
* Simplicity: Make it easy for new engineers to understand the system, by removing as much complexity as possible from the system. (Note this is not the same as simplicity of the user interface.)
* Evolvability: Make it easy for engineers to make changes to the system in the future, adapting it for unanticipated use cases as requirements change. Also known as extensibil‐ ity, modifiability, or plasticity.

### 1.4.1 Operability: Making Life Easy for Operations
A good operations team typically is responsible for the following, and more [29]:
• Monitoring the health of the system and quickly restoring service if it goes into a bad state
• Tracking down the cause of problems, such as system failures or degraded per‐ formance
• Keeping software and platforms up to date, including security patches
• Keeping tabs on how different systems affect each other, so that a problematic change can be avoided before it causes damage
• Anticipating future problems and solving them before they occur (e.g., capacity planning)
• Establishing good practices and tools for deployment, configuration manage‐ ment, and more
• Performing complex maintenance tasks, such as moving an application from one platform to another
• Maintaining the security of the system as configuration changes are made
• Defining processes that make operations predictable and help keep the produc‐ tion environment stable
• Preserving the organization’s knowledge about the system, even as individual people come and go

### 1.4.2 Simplicity: Managing Complexity

There are various possible symptoms of complexity:
* explosion of the state space
* tight coupling of modules
* tangled dependencies
* inconsistent naming and terminology
* hacks aimed at solving performance problems
* special-casing to work around issues elsewhere

Moseley and Marks define complexity as **accidental** if it is not inherent in the problem that the software solves (as seen by the users) but arises only from the implementation.

One of the best tools we have for removing accidental complexity is **abstraction**. A good abstraction can hide a great deal of implementation detail behind a clean, simple-to-understand façade.

### 1.4.3 Evolvability: Making Change Easy

# 2. Data Models and Query Languages.
## 2.1 Relational Model Versus Document Model
### 2.1.2 The Birth of NoSOL
### 2.1.3 The Object-Relational Mismatch
### 2.1.4 Many-to-One and Many-to-Many Relationships
### 2.1.5 Are Document Databases Repeating History?
### 2.1.6 Relational Versus Document Databases Today

## 2.2 Query Languages for Data
### 2.2.1 Declarative Queries on the Web
### 2.2.2 MapReduce Querying
### 2.2.3 Graph-Like Data Models
### 2.2.4 Property Graphs
### 2.2.5 The Cypher Query Language

## 2.3 Graph Oueries in SOL
### 2.3.1 Triple-Stores and SPAROL
### 2.3.2 The Foundation: Datalog

# 3. Storage and Retrieval.
## 3.1 Data Structures That Power Your Database
### 3.1.2 Hash Indexes
### 3.1.3 STables and LSM-Trees
### 3.1.4 B-Trees
### 3.1.5 Comparing B-Trees and LSM-Trees
### 3.1.6 Other Indexing Structures

## 3.2 Transaction Processing or Analytics?
### 3.2.1 Data Warehousing
### 3.2.2 Stars and Snowflakes: Schemas for Analytics
### 3.2.3 Column-Oriented Storage

## 3.3 Column Compression
### 3.3.1 Sort Order in Column Storage
### 3.3.2 Writing to Column-Oriented Storage
### 3.3.3 Aggregation: Data Cubes and Materialized Views

# 4. Encoding and Evolution.
## 4.1 Formats for Encoding Data
### 4.1.1 Language-Specific Formats
### 4.1.2 JSON, XML, and Binary Variants
### 4.1.2 Thrift and Protocol Buffers
### 4.1.3 Avro
### 4.1.4 The Merits of Schemas

## 4.2 Modes of Dataflow
### 4.2.1 Dataflow Through Databases
### 4.2.2 Dataflow Through Services: REST and RPC
### 4.2.3 Message-Passing Dataflow

# Part ll. Distributed Data

# 5. Replication
## 5.1 Leaders and Followers
### 5.1.1 Synchronous Versus Asynchronous Replication
### 5.1.2 Setting Up New Followers
### 5.1.3 Handling Node Outages
### 5.1.4 Implementation of Replication Logs

## 5.2 Problems with Replication Lag
### 5.2.1 Reading Your Own Writes
### 5.2.2 Monotonic Reads
### 5.2.3 Consistent Prefix Reads
### 5.2.4 Solutions for Replication Lag

## 5.3 Multi-Leader Replication
### 5.3.1 Use Cases for Multi-Leader Replication
### 5.3.2 Handling Write Conflicts
### 5.3.3 Multi-Leader Replication Topologies

## 5.4 Leaderless Replication
### 5.4.1 Writing to the Database When a Node Is Down
### 5.4.2 Limitations 6ò to page 199 Consistency
### 5.4.3 Sloppy Quorums and Hinted Handoff
### 5.4.4 Detecting Concurrent Writes


# 6. Partitioning.
## 6.1 Partitioning and Replication

## 6.2 Partitioning of Key-Value Data
### 6.2.1 Partitioning by Key Range
### 6.2.2 Partitioning by Hash of Key
### 6.2.3 Skewed Workloads and Relieving Hot Spots

## 6.3 Partitioning and Secondary Indexes
### 6.3.1 Partitioning Secondary Indexes by Document
### 6.3.2 Partitioning Secondary Indexes by Term

## 6.4 Rebalancing Partitions
### 6.4.1 Strategies for Rebalancing
### 6.4.2 Operations: Automatic or Manual Rebalancing

## 6.5 Request Routing
### 6.5.1 Parallel Query Execution

# 7. Transactions.
## 7.1 The Slippery Concept of a Transaction
### 7.1.1 The Meaning of ACID
### 7.1.2 Single-Object and Multi-Object Operations

## 7.2 Weak Isolation Levels
### 7.2.1 Read Committed
### 7.2.2 Snapshot Isolation and Repeatable Read
### 7.2.3 Preventing Lost Updates
### 7.2.4 Write Skew and Phantoms

## 7.3 Serializability
### 7.3.1 Actual Serial Execution
### 7.3.2 Two-Phase Locking (2PL)
### 7.3.3 Serializable Snapshot Isolation (SSI)

# 8. The Trouble with Distributed Systems….
## 8.1 Faults and Partial Failures
### 8.1.1 Cloud Computing and Supercomputing

## 8.2 Unreliable Networks
### 8.2.1 Network Faults in Practice
### 8.2.2 Detecting Faults
### 8.2.3 Timeouts and Unbounded Delays
### 8.2.4 Synchronous Versus Asynchronous Networks

## 8.3 Unreliable Clocks
### 8.3.1 Monotonic Versus Time-of-Day Clocks
### 8.3.2 Clock Synchronization and Accuracy
### 8.3.3 Relying on Synchronized Clocks
### 8.3.4 Process Pauses

## 8.4 Knowledge, Truth, and Lies
### 8.4.1 The Truth Is Defined by the Majority
### 8.4.2 Byzantine Faults
### 8.4.3 System Model and Reality

# 9. Consistency and Consensus.
## 9.1 Consistency Guarantees

## 9.2 Linearizability
### 9.2.1 What Makes a System Linearizable?
### 9.2.2 Relying on Linearizability
### 9.2.3 Implementing Linearizable Systems
### 9.2.4 The Cost of Linearizability

## 9.3 Ordering Guarantees
### 9.3.1 Ordering and Causality
### 9.3.2 Sequence Number Ordering
### 9.3.3 Total Order Broadcast

## 9.4 Distributed Transactions and Consensus
### 9.4.1 Atomic Commit and Two-Phase Commit (2PC)
### 9.4.2 Distributed Transactions in Practice
### 9.4.3 Fault-Tolerant Consensus
### 9.4.4 Membership and Coordination Services

# 10. Batch Processing
## 10.1 Batch Processing with Unix Tools
## 10.1.1 Simple Log Analysis
## 10.1.2 The Unix Philosophy

## 10.2 MapReduce and Distributed Filesystems
## 10.2.1 MapReduce Job Execution
## 10.2.2 Reduce-Side Joins and Grouping
## 10.2.3 Map-Side Joins
## 10.2.4 The Output of Batch Workflows
## 10.2.5 Comparing Hadoop to Distributed Databases

## 10.3 Beyond MapReduce
## 10.3.1 Materialization of Intermediate State
## 10.3.2 Graphs and Iterative Processing
## 10.3.3 High-Level APIs and Languages

# 11. Stream Processing
## 11.1 Transmitting Event Streams
### 11.1.1 Messaging Systems
### 11.1.2 Partitioned Logs

## 11.2 Databases and Streams
### 11.2.1 Keeping Systems in Sync
### 11.2.2 Change Data Capture
### 11.2.3 Event Sourcing
### 11.2.4 State, Streams, and Immutability

## 11.3 Processing Streams
### 11.3.1 Uses of Stream Processing
### 11.3.2 Reasoning About Time
### 11.3.3 Stream loins
### 11.3.4 Fault Tolerance

# 12. The Future of Data Systems
## 12.1 Data Integration
### 12.1.1 Combining Specialized Tools by Deriving Data
### 12.1.2 Batch and Stream Processing

## 12.2 Unbundling Databases
### 12.2.1 Composing Data Storage Technologies
### 12.2.2 Designing Applications Around Dataflow
### 12.2.3 Observing Derived State

## 12.3 Aiming for Correctness
### 12.3.1 The End-to-End Argument for Databases
### 12.3.2 Enforcing Constraints
### 12.3.3 Timeliness and Integrity
### 12.3.4 Trust, but Verify

## 12.4 Doing the Right Thing
### 12.4.1 Predictive Analytics
### 12.4.2 Privacy and Tracking

![Course](https://www.educative.io/courses/grokking-adv-system-design-intvw)

# 1. Introduction

## What Is This Course About?

# 2. Dynamo: How to Design a Key-value Store?

## Dynamo: Introduction
### Goal

Design a distributed key-value store that is highly available (i.e., reliable), highly scalable, and completely decentralized.

### What is Dynamo?

Dynamo is a highly available key-value store developed by Amazon for their internal use. Many Amazon services, such as shopping cart, bestseller lists, sales rank, product catalog, etc., need only primary-key access to data. A multi-table relational database system would be an overkill for such services and would also limit scalability and availability. Dynamo provides a flexible design to let applications choose their desired level of availability and consistency.

### Background

Dynamo – not to be confused with DynamoDB, which was inspired by Dynamo’s design – is a distributed key-value storage system that provides an “always-on” (or highly available) experience at a massive scale. In [CAP theorem](#cap-theorem) terms, Dynamo falls within the category of AP systems (i.e., available and partition tolerant) and is designed for high availability and partition tolerance at the expense of strong consistency. The primary motivation for designing Dynamo as a highly available system was the observation that the availability of a system directly correlates to the number of customers served. Therefore, the main goal is that the system, even when it is imperfect, should be available to the customer as it brings more customer satisfaction. On the other hand, inconsistencies can be resolved in the background, and most of the time they will not be noticeable by the customer. Derived from this core principle, Dynamo is aggressively optimized for availability.

The Dynamo design was highly influential as it inspired many NoSQL databases, like Cassandra, Riak, and Voldemort – not to mention Amazon’s own DynamoDB.

### Design goals

As stated above, the main goal of Dynamo is to be highly available. Here is the summary of its other design goals:

Scalable: The system should be highly scalable. We should be able to throw a machine into the system to see proportional improvement.
Decentralized: To avoid single points of failure and performance bottlenecks, there should not be any central/leader process.
Eventually Consistent: Data can be optimistically replicated to become eventually consistent. This means that instead of incurring write-time costs to ensure data correctness throughout the system (i.e., strong consistency), inconsistencies can be resolved at some other time (e.g., during reads). Eventual consistency is used to achieve high availability.

### Dynamo’s use cases

By default, Dynamo is an eventually consistent database. Therefore, any application where strong consistency is not a concern can utilize Dynamo. Though Dynamo can support strong consistency, it comes with a performance impact. Hence, if strong consistency is a requirement for an application, then Dynamo might not be a good option.

Dynamo is used at Amazon to manage services that have very high-reliability requirements and need tight control over the trade-offs between availability, consistency, cost-effectiveness, and performance. Amazon’s platform has a very diverse set of applications with different storage requirements. Many applications chose Dynamo because of its flexibility for selecting the appropriate trade-offs to achieve high availability and guaranteed performance in the most cost-effective manner.

Many services on Amazon’s platform require only primary-key access to a data store. For such services, the common pattern of using a relational database would lead to inefficiencies and limit scalability and availability. Dynamo provides a simple primary-key only interface to meet the requirements of these applications.


### System APIs

The Dynamo clients use put() and get() operations to write and read data corresponding to a specified key. This key uniquely identifies an object.

get(key): The get operation finds the nodes where the object associated with the given key is located and returns either a single object or a list of objects with conflicting versions along with a context. The context contains encoded metadata about the object that is meaningless to the caller and includes information such as the version of the object (more on this below).

put(key, context, object): The put operation finds the nodes where the object associated with the given key should be stored and writes the given object to the disk. The context is a value that is returned with a get operation and then sent back with the put operation. The context is always stored along with the object and is used like a cookie to verify the validity of the object supplied in the put request.

Dynamo treats both the object and the key as an arbitrary array of bytes (typically less than 1 MB). It applies the MD5 hashing algorithm on the key to generate a 128-bit identifier which is used to determine the storage nodes that are responsible for serving the key.

## High-level Architecture
## Data Partitioning
## Replication
## Vector Clocks and Conflicting Data
## The Life of Dynamo’s put() & get() Operations
## Anti-entropy Through Merkle Trees
## Gossip Protocol
## Dynamo Characteristics and Criticism
## Summary: Dynamo
## Quiz: Dynamo
## Mock Interview: Dynamo

# 3. Cassandra: How to Design a Wide-column NoSQL Database?

## Cassandra: Introduction
## High-level Architecture
## Replication
## Cassandra Consistency Levels
## Gossiper
## The Anatomy of a Cassandra Write Operation
## The Anatomy of a Cassandra Read Operation
## Compaction
## Tombstones
## Summary: Cassandra
## Quiz: Cassandra
## Mock Interview: Cassandra

# 4. Kafka: How to Design a Distributed Messaging System?

## Messaging Systems: Introduction
## Kafka: Introduction
## High-level Architecture
## Kafka: Deep Dive
## Consumer Groups
## Kafka Workflow
## Role of ZooKeeper
## Controller Broker
## Kafka Delivery Semantics
## Kafka Characteristics
## Summary: Kafka
## Quiz: Kafka
## Mock Interview: Kafka

# 5. Chubby: How to Design a Distributed Locking Service?

## Chubby: Introduction
## High-level Architecture
## Design Rationale
## How Chubby Works
## File, Directories, and Handles
## Locks, Sequencers, and Lock-delays
## Sessions and Events
## Master Election and Chubby Events
## Caching
## Database
## Scaling Chubby
## Summary: Chubby
## Quiz: Chubby
## Mock Interview: Chubby

# 6. GFS: How to Design a Distributed File Storage System?

## Google File System: Introduction
## High-level Architecture
## Single Master and Large Chunk Size
## Metadata
## Master Operations
## The Anatomy of a Read Operation
## The Anatomy of a Write Operation
## The Anatomy of an Append Operation
## GFS Consistency Model and Snapshotting
## Fault Tolerance, High Availability, and Data Integrity
## Garbage Collection
## Criticism on GFS
## Summary: GFS
## Quiz: GFS
## Mock Interview: GFS

# 7. HDFS: How to Design a Distributed File Storage System?

## Hadoop Distributed File System: Introduction
## High-level Architecture
## Deep Dive
## The Anatomy of a Read Operation
## The Anatomy of a Write Operation
## Data Integrity & Caching
## Fault Tolerance
## HDFS High Availability (HA)
## HDFS Characteristics
## Summary: HDFS
## Quiz: HDFS
## Mock Interview: HDFS

# 8. BigTable: How to Design a Wide-column Storage System?

## BigTable: Introduction
## BigTable Data Model
## System APIs
## Partitioning and High-level Architecture
## SSTable
## GFS and Chubby
## Bigtable Components
## Working with Tablets
## The Life of BigTable's Read & Write Operations
## Fault Tolerance and Compaction
## BigTable Refinements
## BigTable Characteristics
## Summary: BigTable
## Quiz: BigTable
## Mock Interview: BigTable

# 9. System Design Patterns

## Introduction: System Design Patterns
## Bloom Filters
## Consistent Hashing
## Quorum
## Leader and Follower
## Write-ahead Log
## Segmented Log
## High-Water Mark
## Lease
## Heartbeat
## Gossip Protocol
## Phi Accrual Failure Detection
## Split Brain
## Fencing
## Checksum

## Vector Clocks

### Background
When a distributed system that allows concurrent writes, it can result in multiple versions of an object. Different replicas of an object can end up with different versions of the data. Let’s understand this with an example.

On a single machine, all we need to know about is the absolute or wall clock time: suppose we perform a write to key k with timestamp t1, and then perform another write to k with timestamp t2. Since t2 > t1, the second write must have been newer than the first write, and therefore, the database can safely overwrite the original value.

In a distributed system, this assumption does not hold true. The problem is clock skew – different clocks tend to run at different rates, so we cannot assume that time t on node a happened before time t + 1 on node b. The most practical techniques that help with synchronizing clocks, like NTP, still do not guarantee that every clock in a distributed system is synchronized at all times. So, without special hardware like GPS units and atomic clocks, just using wall clock timestamps is not enough.

So how can we reconcile and capture causality between different versions of the same object?

### Definition
Use Vector clocks to keep track of value history and reconcile divergent histories at read time.

### Solution
A vector clock is effectively a (node, counter) pair. One vector clock is associated with every version of every object. If the counters on the first object’s clock are less-than-or-equal to all of the nodes in the second clock, then the first is an ancestor of the second and can be forgotten. Otherwise, the two changes are considered to be in conflict and require reconciliation. Such conflicts are resolved at read-time, and if the system is not able to reconcile an object’s state from its vector clocks, it sends it to the client application for reconciliation (since clients have more semantic information on the object and may be able to reconcile it). Resolving conflicts is similar to how Git works. If Git can merge different versions into one, merging is done automatically. If not, the client (i.e., the developer) has to reconcile conflicts manually.

### Examples
To reconcile concurrent updates on an object Amazon’s Dynamo uses Vector Clocks.

## CAP Theorem
## PACELC Theorem
## Hinted Handoff
## Read Repair
## Merkle Trees
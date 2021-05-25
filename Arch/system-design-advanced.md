![Course](https://www.educative.io/courses/grokking-adv-system-design-intvw)

# 1. Introduction

## What Is This Course About?

# 2. Dynamo: How to Design a Key-value Store?

## Dynamo: Introduction
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
## CAP Theorem
## PACELC Theorem
## Hinted Handoff
## Read Repair
## Merkle Trees
[:link:](https://azure.microsoft.com/en-us/resources/designing-distributed-systems/)

# PART I Single-Node Patterns

## 1 Introduce
### Motivations
* The goal of a container is to establish boundaries around specific resources (e.g., this application needs two cores and 8 GB of memory)
* The boundary delineates team ownership (e.g., this team owns this image). Finally, the boundary is intended to provide separation of concerns (e.g., this image does this one thing).

## 2 The Sidecar Pattern
* The sidecar pattern can extend and augment existing application containers
* The sidecar pattern is a singlenode pattern made up of two containers
    * **Application container**. It contains the core logic for the application. Without this container, the application would not exist.
    * **Sidecar container** is to augment and improve the application container, often without the application container’s knowledge.
* In addition to being scheduled on the same machine, the application container and sidecar container share a number of resources, including parts of the filesystem, hostname and network, and many other namespaces.

### Dynamic Configuration with Sidecars
* Simply proxying traffic into an existing application is one use for a sidecar.
* Another common example is configuration synchronization

### Modular Application Containers
* One of the other main advantages of using the sidecar pattern is modularity and reuse of the components used as sidecars.

### Designing Sidecars for Modularity and Reusability
#### Parameterized Containers

#### Define Each Container’s API
* As you think about defining modular, reusable containers, it is important to realize that all aspects of how your container interacts with its world are part of the API defined by that reusable container.
* As in the world of microservices, these microcontainers rely on APIs to ensure that there is a clean separation between the main application container and the sidecar.
* Additionally the API exists to ensure that all consumers of the sidecar will continue to work correctly as subsequent versions are released.

#### Documenting Your Containers


## 3 Ambassadors
* An ambassador container brokers interactions between the application container and the rest of the world. As with other single-node patterns, the two containers are tightly linked in a symbiotic pairing that is scheduled to a single machine.
* The value of the ambassador pattern is twofold:
    * As with the other single-node patterns, there is inherent value in building modular, reusable containers. The separation of concerns makes the containers easier to build and maintain.
    * The implementation is both more consistent and of a higher quality because it is built once and used in many different contexts.

### Using an Ambassador to Shard a Service
* When deploying a sharded service, one question that arises is how to integrate it with the frontend or middleware code that stores data.
* The net result of applying the ambassador pattern to sharded services is a separation of concerns between the application container, which simply knows it needs to talk to a storage service and discovers that service on localhost, and the sharding ambassador proxy, which only contains the code necessary to perform appropriate sharding.

### Using an Ambassador for Service Brokering
* Building a portable application requires that the application know how to introspect its environment and find the appropriate MySQL service to connect to. This process is called **service discovery**, and the system that performs this discovery and linking is commonly called a **service broker**.
* The ambassador pattern enables a system to separate the logic of the application container from the logic of the service broker ambassador.

### Using an Ambassador to Do Experimentation or Request Splitting
* **Request splitting** is sometimes used to tee or split traffic such that all traffic goes to both the production system as well as a newer, undeployed version. The responses from the production system are returned to the user, while the responses from the tee-d service are ignored.
* The ambassador container receives the requests, proxies responses to both the production and experimental systems, and then returns the production responses back as if it had performed the work itself.
* This separation of concerns keeps the code in each container slim and focused, and the modular factoring of the application ensures that the request-splitting ambassador can be reused for a variety of different applications and settings.

## 4 Adapters
* The adapter container is used to modify the interface of the application container so that it conforms to some predefined interface that is expected of all applications.

### Monitoring
### Logging
### Adding a Health Monitor

# PART II Serving Patterns

## 5 Replicated Load-Balanced Services
### Stateless Services
* Stateless systems are replicated to provide redundancy and scale
* No matter how small your service is, you need at least two replicas to provide a service with a **highly available** **service level agreement** (SLA).
* Readiness Probes for Load Balancing
    * **health probes** can be used by a container orchestration system to determine when an application needs to be restarted
    * **readiness probe** determines when an application is ready to serve user requests

### Session Tracked Services
* There are reasons for wanting to ensure that a particular user’s requests always end up on the same machine.
    * caching that user’s data in memory
    * interaction is long-running in nature, so some amount of state is maintained between requests
* IP-based session tracking works within a cluster (internal IPs) but generally doesn’t work well with external IP addresses because of network address translation (NAT). For external session tracking, application-level tracking (e.g., via cookies) is preferred.

### Application-Layer Replicated Services

### Introducing a Caching Layer
* A cache exists between your stateless application and the end-user request.
* The simplest way to deploy the web cache is alongside each instance of your web server using the sidecar pattern
    * Though this approach is simple, it has some disadvantages, namely that you will have to scale your cache at the same scale as your web servers

### Expanding the Caching Layer
* Rate Limiting and Denial-of-Service Defense
    * It makes sense to add general denial-of-service defense via rate limiting to the caching layer.
    * Varnish has a throttle module that can be configured to provide throttling based on IP address and request path, as well as whether or not a user is logged in.
    * If you are deploying an API, it is generally a best practice to have a relatively small rate limit for anonymous access and then force users to log in to obtain a higher rate limit.
* SLL Terminaltion
    * If you plan on using SSL for communication between layers in your cluster, you should still use different certificates for the edge and your internal services
    * We want to add a third layer to our stateless application pattern, which will be a replicated layer of nginx servers that will handle SSL termination for HTTPS traffic and forward traffic on to our Varnish cache.


## 6 Sharded Services
* **Replicated services**, each replica was entirely homogeneous and capable of serving every request
* **Sharded services**, each replica, or shard, is only capable of serving a subset of all requests
* Replicated services are generally used for building `stateless` services, whereas sharded services are generally used for building `stateful` services.
* When designing a sharded cache, there are a number of design aspects to consider:
    1. Why you might need a sharded cache
        * The primary reason for sharding the data is because the size of the state is too large to be served by a single machine.
    2. The role of the cache in your architecture
    3. Replicated, sharded caches
        * Sometimes your system is so dependent on a cache for latency or load that it is not acceptable to lose an entire cache shard if there is a failure or you are doing a rollout.
        *  A sharded, replicated service combines the replicated service pattern with the sharded pattern described in previous sections.
        * This design is obviously more complicated to implement and deploy, but it has sev eral advantages over a simple sharded service.
            * By replacing a single server with a replicated service, each cache shard is resilient to failures and is always present during failures.
            * Rather than designing your system to be tolerant to performance degradation resulting from cache shard failures, you can rely on the performance improvements that the cache provides.
    4. The sharding function
        * The hash function has two important characteristics for our sharding:
            * Determinism: The output should always be the same for a unique input.
            * Uniformity: The distribution of outputs across the output space should be equal.
        * Selecting a Key
            * IP Address
            * HTTP request path
        * Consistent Hashing Functions

* Sharded, Replicated Serving

## 7 Scatter/Gather
* The scatter/gather pattern allows you to achieve parallelism in servicing requests, enabling you to service them significantly faster than you could if you had to service them sequentially
* In contrast to replicated and sharded systems, with scatter/gather requests are simultaneously farmed out to all of the replicas in the system. Each replica does a small amount of processing and then returns a fraction of the result to the root. The root server then combines the various partial results together to form a single complete response to the request and then sends this request back out to the client.
* Scatter/gather is quite useful when you have a large amount of mostly independent processing that is needed to handle a particular request.

### Scatter/Gather with Root/Leaf Distribution
* When **sharding** was introduced to scale replicated systems, the sharding was done at a per-request level. Some part of the request was used to determine where the request was sent. That replica then handled all of the processing for the request and the response was handed back to the user
* With **scatter/gather sharding**, the request is sent to all of the leaf nodes (or shards) in the system. Each leaf node processes the request using the data that it has loaded in its shard. This partial response is then returned to the root node that requested data, and that root node merges all of the responses together to form a comprehensive response for the user.
* Increased parallelization comes at a cost:
    * The first is that processing any particular request has a certain amount of overhead. This is the time spent parsing a request, sending HTTP across the wire, and so forth
    * Suffering from the “straggler” problem:  Since data from every leaf node is required, the overall time it takes to process a user request is defined by the slowest leaf node that sends a response.
    * The same straggler problem applies to availability.

### Scaling Scatter/Gather for Reliability and Scale
* Having a single replica of a sharded scatter/ gather system is likely not the desirable design choice.
    * A single replica means that if it fails, all scatter/gather requests will fail for the duration that the shard is unavailable
* Upgrades will take out a percentage of your shards, so an upgrade while under user-facing load is no longer possible. Finally, the computational scale of your system will be limited by the load that any single node is capable of achieving. Ultimately, this limits your scale, and as we have seen in previous sections, you cannot simply increase the number of shards in order to improve the computational power of a scatter/gather pattern.
* The correct approach is to replicate each of the individual shards so that instead of a single instance at each leaf node, there is a replicated service that implements each leaf shard.


## 8 Functions and Event-Driven Processing
* There is a class of applications that might only need to temporarily come into existence to handle a single request, or simply need to respond to a specific event.

## 9 Ownership Election
### Determining If You Even Need Master Election
* There are ways that you can speed up your deployment by prepulling the new image onto the machine before you run the update. This can reduce the time it takes to deploy a new version to a few seconds, but the trade-off is added complexity, which was what we were trying to avoid in the first place.

### The Basics of Master Election
* There are two ways to implement this master election:
    * Implement a distributed consensus algorithm like Paxos or RAFT. Implementing one of these algorithms is akin to implementing locks on top of assembly code compare-and-swap instructions. It’s an interesting exercise for an undergraduate computer science course, but it is not something that is generally worth doing in practice.
    * There are a large number of distributed key-value stores that have implemented such consensus algorithms for you. The basic primitives that these systems provide is the ability to perform a compare-and-swap operation for a particular key

#### Implementing Locks
* Instead of local memory and assembly instructions, these distributed locks can be implemented in terms of the distributed key-value stores described previously.
* Lock
    1. acquire the lock
        ```js
        func (Lock l) simpleLock() boolean {
        // compare and swap "1" for "0"
        locked, error = compareAndSwap(l.lockName, "1", "0")
        // lock doesn't exist, try to write "1" with a previous value of non-existent
        if error != nil {
            locked, _ = compareAndSwap(l.lockName, "1", nil)
        }
        return locked
        }
        ```
    2. lock
        ```js
        func (Lock l) lock() {
            while (!l.simpleLock()) {
                waitForChanges(l.lockName)
            }
        }
        ```
    3. unlock
        ```js
        func (Lock l) unlock() {
            compareAndSwap(l.lockName, "0", "1")
        }
        ```
* **TTL**: We are building this for a distributed system. A process could fail in the middle of holding the lock, and at that point there is no one left to release it.
    * To resolve this, we take advantage of the TTL functionality of the key-value store. We change our simpleLock function so that it always writes with a TTL, so if we don’t unlock within a given time, the lock will automatically unlock.
        ```js
        func (Lock l) simpleLock() boolean {
        // compare and swap "1" for "0"
        locked, error = compareAndSwap(l.lockName, "1", "0", l.ttl)
        // lock doesn't exist, try to write "1" with a previous value of non-existent
        if error != nil {
            locked, _ = compareAndSwap(l.lockName, "1", nil, l.ttl)
        }
        return locked
        }
        ```
* **Resource version**:
    * By adding TTL to our locks, we have actually introduced a bug into our unlock function.
    * Consider the following scenario:
        1. Process-1 obtains the lock with TTL t.
        2. Process-1 runs really slowly for some reason, for longer than t.
        3. The lock expires.
        4. Process-2 acquires the lock, since Process-1 has lost it due to TTL.
        5. Process-1 finishes and calls unlock.
        6. Process-3 acquires the lock.
    * At this point, Process-1 believes that it has unlocked the lock that it held at the beginning; it doesn’t understand that it has actually lost the lock due to TTL, and in fact unlocked the lock held by Process-2. Then Process-3 comes along and also grabs the lock. Now both Process-2 and Process-3 both believe they own the lock, and hilarity ensues.
    * Fortunately, the key-value store provides a resource version for every write that is performed. Our lock function can store this resource version and augment compareAnd Swap to ensure that not only is the value as expected, but the resource version is the same as when the lock operation occurred. This changes our simple Lock function to look like this:
        ```js
        func (Lock l) simpleLock() boolean {
        // compare and swap "1" for "0"
        locked, l.version, error = compareAndSwap(l.lockName, "1", "0", l.ttl)
        // lock doesn't exist, try to write "1" with a previous value of non-existent
        if error != null {
            locked, l.version, _ = compareAndSwap(l.lockName, "1", null, l.ttl)
        }
        return locked
        }

        func (Lock l) unlock() {
            compareAndSwap(l.lockName, "0", "1", l.version)
        }
        ```

#### Implementing Ownership
* **Renewable lock**:
    * While locks are great for establishing temporary ownership of some critical component, sometimes you want to take ownership for the duration of the time that the component is running
    * One way to do this would be to extend the TTL for the lock to a very long period (say a week or longer), but this has the significant downside that if the current lock owner fails, a new lock owner wouldn’t be chosen until the TTL expired a week later.
    * Instead, we need to create a renewable lock, which can be periodically renewed by the owner so that the lock can be retained for an arbitrary period of time.
        ```js
        func (Lock l) renew() boolean {
            locked, _ = compareAndSwap(l.lockName, "1", "1", l.version, ttl)
            return locked
        }
        ```
    * You probably want to do this repeatedly in a separate thread so that you hold onto the lock indefinitely.
        * The lock is renewed every ttl/2 seconds; that way there is significantly less risk that the lock will accidentally expire due to timing subtleties:
        ```js
        for {
            if !l.renew() {
                handleLockLost()
            }
            sleep(ttl/2)
        }
        ```
    * You need to implement the handleLockLost() function so that it terminates all activity that required the lock in the first place. In a container orchestration system, the easiest way to do this may simply be to terminate the application and let the orchestrator restart it. This is safe, because some other replica has grabbed the lock in the interim, and when the restarted application comes back online it will become a secondary listener waiting for the lock to become free.
### Handling Concurrent Data Manipulation
* **Double-check**:

    * Even with all of the locking mechanisms we have described, it is still possible for two replicas to simultaneously believe they hold the lock for a very brief period of time.
        * The original lock holder becomes so overwhelmed that its processor stops running for minutes at a time. This can happen on extremely overscheduled machines.
        * In such a case, the lock will time out and some other replica will own the lock.
        * Now the processor frees up the replica that was the original lock holder.
        * the handleLockLost() function will quickly be called, but there will be a brief period where the replica still believes it holds the lock.
    * Although such an event is fairly unlikely, systems need to be built to be robust to such occurrences. The first step to take is to double-check that the lock is still held, using a function like this:
        ```js
        func (Lock l) isLocked() boolean {
            return l.locked && l.lockTime + 0.75 * l.ttl > now()
        }
        ```
    * The probability of two masters being active is not completely eliminated. The lock timeout could always occur between the time that the lock was checked and the guarded code was executed.
        * To protect against these scenarios, the system that is being called from the replica needs to validate that the replica sending a request is actually still the master. To do this, the hostname of the replica holding the lock is stored in the key-value store in addition to the state of the lock. That way, others can double-check that a replica asserting that it is the master is in fact the master.
        * In the first image, shard2 is the owner of the lock, and when a request is sent to the worker, the worker double-checks with the lock server and validates that shard2 is actually the current owner.
        * In the second case, shard2 has lost ownership of the lock, but it has not yet realized this so it continues to send requests to the worker node. This time, when the worker node receives a request from shard2, it double-checks with the lock service and realizes that shard2 is no longer the lock owner, and thus the requests are rejected.
* **Resource Version**
    * It’s always possible that ownership could be obtained, lost, and then re-obtained by the system, which could actually cause a request to succeed when it should actually be rejected.
        1. Shard-1 obtains ownership to become master.
        2. Shard-1 sends a request R1 as master at time T1.
        3. The network hiccups and delivery of R1 is delayed.
        4. Shard-1 fails TTL because of the network and loses lock to Shard-2.
        5. Shard-2 becomes master and sends a request R2 at time T2.
        6. Request R2 is received and processed.
        7. Shard-2 crashes and loses ownership back to Shard-1.
        8. Request R1 finally arrives, and Shard-1 is the current master, so it is accepted, but this is bad because R2 has already been processed.
    * Such sequences of events seem byzantine, but in reality, in any large system they occur with disturbing frequency.
    * To solve this problem we also send the resource version along with each request. The double-check validates both the current owner and the resource version of the request.

# PART III Batch Computational Patterns
* The preceding chapter described patterns for reliable, long-running server applications. This section describes patterns for batch processing. In contrast to longrunning applications, batch processes are expected to only run for a short period of time.
    * Generating aggregation of user telemetry data
    * Analyzing sales data for daily or weekly reporting
    * Transcoding video files

## 10 Work Queue Systems

### A Generic Work Queue System
#### The Source Container Interface
* To operate, every work queue needs a collection of work items that need processing. There are many different sources of work items the work queue, depending on the specific application of the work queue.
* We can separate the application-specific queue source logic from the generic queue processing logic.
    * The generic work queue container is the primary application container
    * The application-specific source container is the ambassador that proxies the generic work queue’s requests out to the concrete definition of the work queue out in the real world.
* Work queue API
    ```
    GET http://localhost/api/v1/items
    GET http://localhost/api/v1/items/<item-name>
    ```

#### The Worker Container Interface
* This container and interface are slightly different than the previous work queue source interface for a few reasons.
    * It is a one-off API: a single call is made to begin the work, and no other API calls are made throughout the life of the worker container.
    * The worker container is not inside a container group with the work queue manager. Instead, it is launched via a container orchestration API and scheduled to its own container group. This means that the work queue man‐ ager has to make a remote call to the worker container in order to start work. It also means that we may need to be more careful about security to prevent a malicious user in our cluster from injecting extra work into the system.
* With the work queue source API, we used a simple HTTP-based API for sending items back to the work queue manager.
* For the worker container, we will use a file-based API. Namely, when the worker container is created, it will receive an environment variable named WORK_ITEM_FILE; this will point to a file in the container’s local filesystem, where the data field from a work item has been written to a file.
* Often a work queue worker is simply a shell script across a few command line tools.
* As was true with the work queue source implementation, most of the worker containers will be special-purpose container images built for specific work queue applications, but there are also generic workers that can be applied to multiple different work queue applications.

#### The Shared Work Queue Infrastructure

## 11 Event-Driven Batch Processing

## 12 Coordinated Batch Processing

## 13 Conclusion: A New Beginning?

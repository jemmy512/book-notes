# Chapter 2: Meaningful Names
1. Use Intention-Revealing Names
2. Avoid Disinformation
3. Make Meaningful Distinction
4. Use Pronounceable Names
5. Use Searchable Names
6. Avoid Encodings
7. Avoid Mental Mapping
8. Class Names: noun or noun phrase
9. Method Names: verb or verb phrase
10. Don't be cute
11. Pick One Word per Concept
12. Don't Pun
13. Use Solution Domain Names
14. Use Problem Domain Names
15. Add Meaningful Context
16. Don't Add Gratuitos Context

# Chapter 3: Functions
1. Small
2. Do One Thing
3. One Level of Abstraction per Function

    > Reading Code from Top to Bottom: The Stepdown Rule
4. Switch Statements
5. Use Descriptive Names
6. Function Arguments
    * Common Monadic Forms
    * Dyadic Functions
    * Triads
    * Argument Objects
    * Argument Lists
    * Verbs and Keywords
7. Have No Side Effects
8. Command Query Seperation
9. Prefer Exceptions to Returning Error Codes
    * Extract Try/Catch Blocks
    * Error Handling Is One Thing
10. Don't Repeat Yourself
11. Structured Programming

# Chapter 6: Objects and Data Structures
1. Data Abstraction
2. Data/Object Anti-Symmetry
    > Objects hide their data behind abstractions and expose functions that operate on that data.Data structure expose their data and have no meaningful functions.

    > Procedural code makes it hard to add new data structures because all the functions must change. OO code makes it hard to add new functions because all the classes must change.
3. The Law of Demeter
    > `Law of Demeter` that says a module should not know about the innards of the objects it manipulates
    * Train Wrecks
    * Hybrids
    * Hiding Structure
4. Data Transfer Objects

    > The quintessential form of a data structure is a class with public variables and no functions

# Chapter 7: Error Handling
1. Use Exceptions Rather Than Return Codes
2. Write Your Try-Catch-Finally Statement First
3. Use Unchecked Exceptions
5. Provide Context with Exceptions
6. Define Exception Classes in Terms of a Caller's Needs
7. Define the Normal Flow

    > `Special Case Pattern`: create a class or configure an object so that it handles a special case.
8. Don't Return Null

    > throw an exception or return a special case object instead
9. Don't Pass Null

    > forbid passing null by default

# Chapter 10: Classes
## Class Organization
### Encapsulation

## Classes Should Be Small
With functions we measured size by counting physical lines. With classes we use a different measure. We count `responsibilities`.

The name of a class should describe what responsibilities it fulfills. In fact, naming is probably the first way of helping determine class size.

### The Single Responsibility Principle
### Cohesion
The more variables a method manipulates the more cohesive that method is to its class.

### Maintaining Cohesion Results in Many Small Classes
Breaking a large function into many smaller functions often gives us the opportunity to split several smaller classes out as well.

## Organizing for Change
```C++
class Sql {
public:
    Sql(const string& table, const Column&... column);
    string create();
    string insert(cosnt string&... fields);
    string selectAll();
    string findByKey(cosnt string& keyColumn, cosnt string& keyValue);
};
```

```C++
class Sql {
public:
    Sql(const string& table, const Column&... column);
    virtual string generate() = 0;
};

class CreateSql : public Sql {
public:
    string CreateSql(const string& table, const Column&... column);
    virtual string generate() override;
};

class UpdateSql : public Sql {
public:
    string UpdateSql(const string& table, const Column&... column);
    virtual string generate() override;
};
```

In an ideal system, we incorporate new features by extending the system, not by making modifications to existing code.

### Isolating from Change
A client class depending upon concrete details is at risk when those details change. We can introduce interfaces and abstract classes to help isolate the impact of those details.

`Dependency Inversion Principle (DIP)`: classes should depend upon abstractions, not on concrete details.

```C++
/* The system has an external dependency: ToykoStoackExchange,
 * use DIP and DI(Dependency Injection) to isolate the change of the external dependency */
class StockExchange {
public:
    virtual Money currentPrice(cosnt string& symbol) = 0;
};

class Portfolio {
public:
    Portfolio(const StockExchange& exchange) : mExchange(exchange) { }

private:
    const StockExchange& mExchagne;
};
```

# Chapter 11: System
## Separate Constructing a System from Using It

Software systems should separate the startup process, when the application objects are constructed and the dependencies are “wired” together, from the runtime logic that takes over after startup.

```C++
Service getServcie() {
    if (mServcie == nullptr) {
        mService = std::make_shared<ServcieImpl>(...);
    }

    return mService;
}
```

1. The code has hard-coded dependencies `ServiceImpl` and its contructor paramters
2. Testing can be a problem

We should have a global, consistent strategy for resolving our major dependencies.

### Separation of Main
One way to separate construction from use is simply to move all aspects of construction to main, or modules called by main, and to design the rest of the system assuming that all objects have been constructed and wired up appropriately.

### Factories
Sometimes we need to make the application responsible for when an object gets created.

In this case we can use the ABSTRACT FACTORY pattern to give the application control of when to build the object, but keep the details of that construction separate from the application code.

### Dependency Injection
A powerful mechanism for separating construction from use is `Dependency Injection (DI)`, the application of `Inversion of Control (IoC)` to dependency management.

IoC moves secondary responsibilities from an object to other objects that are dedicated to the purpose, thereby supporting the Single Responsibility Principle.

## Scaling Up
Software systems are unique compared to physical systems. Their architectures can grow incrementally, if we maintain the proper separation of concerns.

## Test Drive the System Architecture
An optimal system architecture consists of modularized domains of concern, each of which is implemented with Plain Old Java (or other) Objects. The different domains are integrated together with minimally invasive Aspects or Aspect-like tools. This architecture can be test-driven.

## Optimize Decision Making
Modularity and separation of concerns make decentralized management and decision making possible.

It is best to give responsibilities to the most qualified persons. It is also best to postpone decisions until the last possible moment.

The agility provided by a POJO system with modularized concerns allows us to make optimal, just-in-time decisions, based on the most recent knowledge. The complexity of these decisions is also reduced.

## Use Standards Wisely, When They Add Demonstrable Value
Standards make it easier to reuse ideas and components, recruit people with relevant experience, encapsulate good ideas, and wire components together. However, the process of creating standards can sometimes take too long for industry to wait, and some standards lose touch with the real needs of the adopters they are intended to serve.

## Systems Need Domain-Specific Languages
Domain-Specific Languages allow all levels of abstraction and all domains in the application to be expressed as POJOs, from high-level policy to low-level details.

# Chapter 12: Emergence
1. Runs all the tests
2. Contains no duplication
3. Expresses the intent of the programmer
4. Minimizes the number of classes and methods

## Run all the Tests
During refactoring step, we can apply anything from the entire body of knowledge about good software design. We can increase cohesion, decrease coupling, separate concerns, modularize system concerns, shrink our functions and classes, choose better names, and so on.

## No Duplication
The `TEMPLATE METHOD` pattern is a common technique for removing higher-level duplication.

```C++
class VacationPolicy {
public:
    void accrueUSDivisionVacation() {
        // calcualte vacation based on hours worked to data
        // ensure vacation meets US minimums
        // apply vacation to payroll record
    }

    void accrueEUDivisionVacation() {
        // calcualte vacation based on hours worked to data
        // ensure vacation meets EU minimums
        // apply vacation to payroll record
    }
};
```

```C++
class VacationPolicy {
public:
    void accrueVacation() {
        calcualteBaseVocationHours();
        alterForLegalMiniumus();
        applyToPayroll();
    }

protected:
    virtual void alterForLegalMiniumus() = 0;

private:
    void calcualteBaseVacationHours();
    void applyToPayroll();
};

class USVacationPolicy : public VacationPolicy {
protected:
    void alterForLegalMiniumus() override {

    }
};

class EUVacationPolicy : public VacationPolicy {
protected:
    void alterForLegalMiniumus() override {

    }
};
```

## Expressive
ou can express yourself by choosing good names,  keeping your functions and classes small, using standard nomenclature and design pattern.

## Minimal Classes and Methods
Even concepts as fundamental as elimination of duplication, code expressiveness, and the SRP can be taken too far.

# Chapter 13: Concurrency
Concurrency is a decoupling strategy. It helps us decouple what gets done from when it gets done.

In single-threaded applications `what` and `when` are so strongly coupled that the state of the entire application can often be determined by looking at the stack backtrace.

Decoupling what from when can dramatically improve both the throughput and struc- tures of an application. From a structural point of view the application looks like many lit- tle collaborating computers rather than one big main loop. This can make the system easier to understand and offers some powerful ways to separate concerns.

## Concurrency Defense Principles
1. Single Responsibility Principle
    * Keep your concurrency-related code separate from other code
2. Corollary: Limit the Scope of Data
    * Take data encapsulation to heart; severely limit the access of any data that may be shared
3. Corollary: Use Copies of Data
4. Corollary: Threads Should Be as Independent as Possible
    * Attempt to partition data into independent subsets than can be operated on by independent threads, possibly in different processors

## Know Your Execution Models

Concept | Definition
--- | ---
Bound Resources | Resources of a fixed size or number used in a concurrent environ- ment. Examples include database connections and fixed-size read/ write buffers.
Mutual Exclusion | Only one thread can access shared data or a shared resource at a time.
Starvation | One thread or a group of threads is prohibited from proceeding for an excessively long time or forever. For example, always let- ting fast-running threads through first could starve out longer run- ning threads if there is no end to the fast-running threads.
Deadlock | Two or more threads waiting for each other to finish. Each thread has a resource that the other thread requires and neither can finish until it gets the other resource.
Livelock | Threads in lockstep, each trying to do work but finding another “in the way.” Due to resonance, threads continue trying to make progress but are unable to for an excessively long time— or forever.

1. Producer-Consumer
2. Readers-Writers
3. Dining Philosophers

## Keep Synchronized Sections Small

## Writing Correct Shut-Down Code Is Hard
Think about shut-down early and get it working early. It’s going to take longer than you expect. Review existing algorithms because this is probably harder than you think.

## Testing Threaded Code
Write tests that have the potential to expose problems and then run them frequently, with different programatic configurations and system configurations and load. If tests ever fail, track down the failure. Don’t ignore a failure just because the tests pass on a subsequent run.

1. Treat spurious failures as candidate threading issues.
2. Get your nonthreaded code working first.
    * Do not try to chase down nonthreading bugs and threading bugs at the same time. Make sure your code works outside of threads.
3. Make your threaded code pluggable.
4. Make your threaded code tunable.
5. Run with more threads than processors.
6. Run on different platforms.
7. Instrument your code to try and force failures.
    * wait(), sleep(), yield(), and priority()

# Chapter 14: Successive Refinement

# Chapter 2: Meaningful Names
1. Use Intention-Revealing Names
2. Avoid Disinformation
3. Make Meaningful Distinction
    * If names must be different, then they should also mean something different.
    * Production, ProductionInfo, ProducionData are not distinctable.
4. Use Pronounceable Names
5. Use Searchable Names
6. Avoid Encodings
    * Hungarian Notation
    * Membder Prefix
    * Interfaces and Implementations
7. Avoid Mental Mapping
    * Readers shouldn’t have to mentally translate your names into other names they already know.
8. Class Names: noun or noun phrase
9. Method Names: verb or verb phrase
10. Don't be cute
11. Pick One Word per Concept
12. Don't Pun
13. Use Solution Domain Names
    * Use computer science (CS) terms, algorithm names, pattern names, math terms, and so forth.
14. Use Problem Domain Names
    * When there is no “programmer-eese” for what you’re doing, use the name from the prob- lem domain.
15. Add Meaningful Context
16. Don't Add Gratuitous Context

# Chapter 3: Functions
1. Small
2. Do One Thing
3. One Level of Abstraction per Function
    * Reading Code from Top to Bottom: The Stepdown Rule
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
    * `Single-entry, single-exit rule`: Dijkstra said that every function, and every block within a function, should have one entry and one exit.
    * There should only be one return statement in a func- tion, no break or continue statements in a loop, and never, ever, any goto statements.

# Chapter 6: Objects and Data Structures
1. Data Abstraction
2. Data/Object Anti-Symmetry
    * `Objects` hide their data behind abstractions and expose functions that operate on that data. `Data structure` expose their data and have no meaningful functions.
    * `Procedural` code makes it hard to add new data structures because all the functions must change. `OO `code makes it hard to add new functions because all the classes must change.
3. The Law of Demeter
    * `Law of Demeter` that says a module should not know about the innards of the objects it manipulates
    * Train Wrecks
    * Hybrids
    * Hiding Structure
4. Data Transfer Objects(DTO)
    * The quintessential form of a data structure is a class with public variables and no functions

# Chapter 7: Error Handling
1. Use Exceptions Rather Than Return Codes
2. Write Your Try-Catch-Finally Statement First
3. Use Unchecked Exceptions
5. Provide Context with Exceptions
6. Define Exception Classes in Terms of a Caller's Needs
7. Define the Normal Flow
    * `Special Case Pattern`: create a class or configure an object so that it handles a special case.
    ```C++
    try {
        MealExpenses expenses = expenseReportDAO.getMeals(employee.getID());
        m_total += expenses.getTotal();
    } catch(MealExpensesNotFound e) {
        m_total += getMealPerDiem();
    }
    ```
    ```C++
    /* ExpenseReportDAO always returns a MealExpense object.
     * If there are no meal expenses, it returns a MealExpense object that returns the per diem as its total. */
    MealExpenses expenses = expenseReportDAO.getMeals(employee.getID());
    m_total += expenses.getTotal();
    ```
8. Don't Return Null
    * throw an exception or return a special case object instead
9. Don't Pass Null
    * forbid passing null by default

# Chapter 8: Boundaries
## Using Third-party Code
If you use a boundary interface like Map, keep it inside the class, or close family of classes, where it is used. Avoid returning it from, or accepting it as an argument to, public APIs.

## Colusion
We manage third-party boundaries by having very few places in the code that refer to them. We may wrap them as we did with Map, or we may use an `ADAPTER` to convert from our perfect interface to the provided interface. Either way our code speaks to us better, promotes internally consistent usage across the boundary, and has fewer maintenance points when the third-party code changes.

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
    virtual ~Sql() = default;

    virtual string generate() = 0;
};

class CreateSql : public Sql {
public:
    CreateSql(const string& table, const Column&... column);
    virtual string generate() override;
};

class UpdateSql : public Sql {
public:
    UpdateSql(const string& table, const Column&... column);
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

In this case we can use the `ABSTRACT FACTORY` pattern to give the application control of when to build the object, but keep the details of that construction separate from the application code.

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
Express yourself by choosing good names, keeping your functions and classes small, using standard nomenclature and design pattern.

## Minimal Classes and Methods
Even concepts as fundamental as elimination of duplication, code expressiveness, and the SRP can be taken too far.

# Chapter 13: Concurrency
Concurrency is a decoupling strategy. It helps us decouple what gets done from when it gets done.

In single-threaded applications `what` and `when` are so strongly coupled that the state of the entire application can often be determined by looking at the stack backtrace.

Decoupling what from when can dramatically improve both the throughput and structures of an application. From a structural point of view the application looks like many little collaborating computers rather than one big main loop. This can make the system easier to understand and offers some powerful ways to separate concerns.

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
--| ---
Bound Resources | Resources of a fixed size or number used in a concurrent environment. Examples include database connections and fixed-size read/ write buffers.
Mutual Exclusion | Only one thread can access shared data or a shared resource at a time.
Starvation | One thread or a group of threads is prohibited from proceeding for an excessively long time or forever. For example, always letting fast-running threads through first could starve out longer running threads if there is no end to the fast-running threads.
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

# Chapter 17: Smells and Heuristics
## Comment
1. Inappropriate Information
2. Obsolete Comment
3. Redundant Comment
4. Poorly Written Comment
5. Commented-Out Code

## Environment
1. Build Requires More Than One Step
2. Tests Require More Than One Step

## Functions
1. Too Many Arguments
2. Output Arguments
3. Flag Arguments
4. Dead Function

## General
1. Multiple Languages in One Source File
2. Obvious Behavior Is Unimplemented
3. Incorrect Behavior at the Boundaries
4. Overridden Safeties
5. Duplication
    * Every time you see duplication in the code, it represents a missed opportunity for abstraction.
6. Code at Wrong Level of Abstraction
    * creating abstract classes to hold the higher level concepts and derivatives to hold the lower level concepts
    * Isolating abstractions is one of the hardest things that software developers do, and there is no quick fix when you get it wrong.
7. Base Classes Depending on Their Derivatives
    * The most common reason for partitioning concepts into base and derivative classes is so that the higher level base class concepts can be independent of the lower level derivative class concepts.
8. Too Much Information
9. Dead Code
10. Vertical Separation
    * Variables and function should be defined close to where they are used.
11. Inconsistency
    * If you do something a certain way, do all similar things in the same way.
12. Clutter
    * Variables that aren’t used, functions that are never called, comments that add no information..., all these things are clutter and should be removed.
13. Artificial Coupling
    * Things that don’t depend upon each other should not be artificially coupled.
    * e.g., general enums should not be contained within more specific classes
14. Feature Envy
    * The methods of a class should be interested in the variables and functions of the class they belong to, and not the variables and functions of other classes.
15. Selector Arguments
    * Not only is the purpose of a selector argument difficult to remember, each selector argument combines many functions into one.
    * Selector arguments are just a lazy way to avoid splitting a large function into several smaller functions.
16. Obscured Intent
    * We want code to be as expressive as possible. Run-on expressions, Hungarian notation, and magic numbers all obscure the author’s intent.
17. Misplaced Responsibility
    * One of the most important decisions a software developer can make is where to put code.
18. Inapproprivate Static
    * When static a function, make sure that there is no chance that you’ll want it to behave polymorphically.
19. Use Explanatory Variables
    * One of the more powerful ways to make a program readable is to break the calculations up into intermediate values that are held in variables with meaningful names.
20. Function Names Should Say What They Do
21. Understand the Algorithm
22. Make Logical Dependencies Physical
    * If one module depends upon another, that dependency should be physical, not just logical. The dependent module should not make assumptions (in other words, logical dependencies) about the module it depends upon. Rather it should explicitly ask that module for all the information it depends upon.
23. Prefer Polymorphism to If/Else or Switch/Case
24. Follow Standard Conventions
25. Replace Magic Numbers with Named Constants
26. Be Precise
27. Structure over Convention
28. Encapsulate Conditionals
29. Avoid Negative Conditionals
    * Boolean logic is hard enough to understand without having to see it in the context of an if or while statement.
    * Extract functions that explain the intent of the conditional.
30. Functions Should Do One Thing
31. Hidden Temporal Couplings
    ```C++
    class MoogDiver {
    public:
        /* The order of the three functions is important.
         * Unfortunately, the code does not enforce this temporal coupling.
         * Another programmer could call reticulateSplines before saturateGradient
         * was called, leading to an UnsaturatedGradientException. */
        void dive(String reason) {
            saturateGradient();
            reticulateSplines();
            diveForMoog(reason);
        }

    private:
        Gradient gradient;
        List<Spline> splines;
    };
    ```

    ```C++
    class MoogDiver {
    public:
        /* This exposes the temporal coupling by creating a bucket brigade. */
        void dive(String reason) {
            Gradient gradient = saturateGradient();
            List<Spline> splines = reticulateSplines(gradient);
            diveForMoog(splines, reason);
        }

    private:
        Gradient gradient;
        List<Spline> splines;
    };
    ```
32. Don’t Be Arbitrary
33. Encapsulate Boundary Conditions
34. Functions Should Descend Only One Level of Abstraction
    * The statements within a function should all be written at the same level of abstraction, which should be one level below the operation described by the name of the function.
35. Keep Configurable Data at High Levels
    * Expose configurable data as an argument to that low-level function called from the high-level function.
    * The configuration constants reside at a very high level and are easy to change.
    ```Java
    public static void main(String[] args) throws Exception {
        Arguments arguments = parseCommandLine(args);
    }
    public class Arguments {
        public static final String DEFAULT_PATH = ".";
        public static final String DEFAULT_ROOT = "FitNesseRoot";
        public static final int DEFAULT_PORT = 80;
        public static final int DEFAULT_VERSION_DAYS = 14;
    }
    ```
36. Avoid Transitive Navigation
    ```C++
    a.getB().getC().doSomething();

    myCollaborator.doSomething();
    ```

## Names
1. Choose Descriptive Names
2. Choose Names at the Appropriate Level of Abstraction
    * Don’t pick names that communicate implementation; choose names the reflect the level of abstraction of the class or function you are working in.
3. Use Standard Nomenclature Where Possible
4. Unambiguous Names
5. Use Long Names for Long Scopes
6. Avoid Encodings
7. Names Should Describe Side-Effects
    ```Java
    /* createOrReturnOos is better */
    public ObjectOutputStream getOos() throws IOException {
        if (m_oos == null) {
            m_oos = new ObjectOutputStream(m_socket.getOutputStream()); }
        return m_oos;
    }
    ```

## Tests
1. Insufficient Tests
    * The tests are insufficient so long as there are conditions that have not been explored by the tests or calculations that have not been validated.
2. Use a Coverage Tool!
3. Don’t Skip Trivial Tests
4. An Ignored Test Is a Question about an Ambiguity
5. Test Boundary Conditions
6. Exhaustively Test Near Bugs
    * Bugs tend to congregate. When you find a bug in a function, it is wise to do an exhaustive test of that function.
7. Patterns of Failure Are Revealing
8. Test Coverage Patterns Can Be Revealing
9. Tests Should Be Fast

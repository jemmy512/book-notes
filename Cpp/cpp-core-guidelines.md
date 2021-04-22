# Interfaces
* [I.1: Make interfaces explicit](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-interfaces)

* I.2: Avoid non-const global variables

* I.3: Avoid singletons

* I.4: Make interfaces precisely and strongly typed

* I.5: State preconditions (if any)

* I.6: Prefer Expects() for expressing preconditions

* I.7: State postconditions

* I.8: Prefer Ensures() for expressing postconditions

* I.9: If an interface is a template, document its parameters using concepts

* I.10: Use exceptions to signal a failure to perform a required task

* I.11: Never transfer ownership by a raw pointer (T*) or reference (T&)

* I.12: Declare a pointer that must not be null as not_null

* I.13: Do not pass an array as a single pointer

* I.22: Avoid complex initialization of global objects

* I.23: Keep the number of function arguments low

* I.24: Avoid adjacent parameters of the same type when changing the argument order would change meaning

* I.25: Prefer abstract classes as interfaces to class hierarchies

* I.26: If you want a cross-compiler ABI, use a C-style subset

* I.27: For stable library ABI, consider the Pimpl idiom

* I.30: Encapsulate rule violations

# Functions
[:link:](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-functions)

## Function definition rules:

* F.1: "Package" meaningful operations as carefully named functions

* F.2: A function should perform a single logical operation

* F.3: Keep functions short and simple

* F.4: If a function may have to be evaluated at compile time, declare it constexpr

* F.5: If a function is very small and time-critical, declare it inline

* F.6: If your function must not throw, declare it noexcept

* F.7: For general use, take T* or T& arguments rather than smart pointers

* F.8: Prefer pure functions

* F.9: Unused parameters should be unnamed

## Parameter passing expression rules:

* F.15: Prefer simple and conventional ways of passing information

* F.16: For "in" parameters, pass cheaply-copied types by value and others by reference to const

* F.17: For "in-out" parameters, pass by reference to non-const

* F.18: For "will-move-from" parameters, pass by X&& and std::move the parameter

* F.19: For "forward" parameters, pass by TP&& and only std::forward the parameter

* F.20: For "out" output values, prefer return values to output parameters

* F.21: To return multiple "out" values, prefer returning a struct or tuple

* F.60: Prefer T* over T& when "no argument" is a valid option

## Parameter passing semantic rules:

* F.22: Use T* or owner<T*> to designate a single object

* F.23: Use a not_null<T> to indicate that "null" is not a valid value

* F.24: Use a span<T> or a span_p<T> to designate a half-open sequence

* F.25: Use a zstring or a not_null<zstring> to designate a C-style string

* F.26: Use a unique_ptr<T> to transfer ownership where a pointer is needed

* F.27: Use a shared_ptr<T> to share ownership

## Value return semantic rules:

* F.42: Return a T* to indicate a position (only)

* F.43: Never (directly or indirectly) return a pointer or a reference to a local object

* F.44: Return a T& when copy is undesirable and "returning no object" isn’t needed

* F.45: Don’t return a T&&

* F.46: int is the return type for main()

* F.47: Return T& from assignment operators

* F.48: Don’t return std::move(local)

## Other function rules:

* F.50: Use a lambda when a function won’t do (to capture local variables, or to write a local function)

* F.51: Where there is a choice, prefer default arguments over overloading

* F.52: Prefer capturing by reference in lambdas that will be used locally, including passed to algorithms

* F.53: Avoid capturing by reference in lambdas that will be used non-locally, including returned, stored on the heap, or passed to another thread

* F.54: If you capture this, capture all variables explicitly (no default capture)

* F.55: Don’t use va_arg arguments

# C: Classes and class hierarchies
[:link:](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-class)

## Class rule summary:

* C.1: Organize related data into structures (structs or classes)

* C.2: Use class if the class has an invariant; use struct if the data members can vary independently

* C.3: Represent the distinction between an interface and an implementation using a class

* C.4: Make a function a member only if it needs direct access to the representation of a class

* C.5: Place helper functions in the same namespace as the class they support

* C.7: Don’t define a class or enum and declare a variable of its type in the same statement

* C.8: Use class rather than struct if any member is non-public

* C.9: Minimize exposure of members

## Default Operation Ruls

* C.20: If you can avoid defining any default operations, do
    * The rule of zero

* C.21: If you define or =delete any copy, move, or destructor function, define or =delete them all
    * The rule of five

* C.22: Make default operations consistent

## Destructor Rules

* C.30: Define a destructor if a class needs an explicit action at object destruction

* C.31: All resources acquired by a class must be released by the class’s destructor

* C.32: If a class has a raw pointer (T*) or reference (T&), consider whether it might be owning

* C.33: If a class has an owning pointer member, define a destructor

* C.35: A base class destructor should be either public and virtual, or protected and non-virtual

* C.36: A destructor may not fail

* C.37: Make destructors noexcept

## Constructor Rules

* C.40: Define a constructor if a class has an invariant

* C.41: A constructor should create a fully initialized object

* C.42: If a constructor cannot construct a valid object, throw an exception

* C.43: Ensure that a copyable (value type) class has a default constructor

* C.44: Prefer default constructors to be simple and non-throwing

* C.45: Don’t define a default constructor that only initializes data members; use in-class member initializers instead

* C.46: By default, declare single-argument constructors explicit

* C.47: Define and initialize member variables in the order of member declaration

* C.48: Prefer in-class initializers to member initializers in constructors for constant initializers

* C.49: Prefer initialization to assignment in constructors

* C.50: Use a factory function if you need "virtual behavior" during initialization
    ```C++
    class B {
    protected:
        class Token {};

    public:
        explicit B(Token) { /* ... */ }  // create an imperfectly initialized object
        virtual void f() = 0;

        template<class T>
        static std::shared_ptr<T> create()    // interface for creating shared objects
        {
            auto p = std::make_shared<T>(typename T::Token{});
            p->post_initialize();
            return p;
        }

    protected:
        virtual void post_initialize()   // called right after construction
            { /* ... */ f(); /* ... */ } // GOOD: virtual dispatch is safe
    };

    class D : public B {                 // some derived class
    protected:
        class Token {};

    public:
        explicit D(Token) : B{ B::Token{} } {}
        void f() override { /* ...  */ };

    protected:
        template<class T>
        friend shared_ptr<T> B::create();
    };

    int main() {
        shared_ptr<D> p = D::create<D>();  // creating a D object
    }
    ```

* C.51: Use delegating constructors to represent common actions for all constructors of a class

* C.52: Use inheriting constructors to import constructors into a derived class that does not need further explicit initialization

## Copy and Move Rules

* C.60: Make copy assignment non-virtual, take the parameter by const&, and return by non-const&

* C.61: A copy operation should copy

* C.62: Make copy assignment safe for self-assignment

* C.63: Make move assignment non-virtual, take the parameter by &&, and return by non-const&

* C.64: A move operation should move and leave its source in a valid state

* C.65: Make move assignment safe for self-assignment

* C.66: Make move operations noexcept

* C.67: __A polymorphic class should suppress copying__
    * If it base class accidentally passed by value, with the implicitly generated copy constructor and assignment, we risk slicing: only the base portion of a derived object will be copied, and the polymorphic behavior will be corrupted.
    * If you need to create deep copies of polymorphic objects, use `clone()` functions: see C.130.

## Other Default Operations Rules

* C.80: Use =default if you have to be explicit about using the default semantics

* C.81: Use =delete when you want to disable default behavior (without wanting an alternative)

* C.82: Don’t call virtual functions in constructors and destructors
    * The function called will be that of the object constructed so far, rather than a possibly overriding function in a derived class.

* C.83: For value-like types, consider providing a noexcept swap function
    ```C++
    class Foo {
    public:
        void swap(Foo& rhs) noexcept {
            m1.swap(rhs.m1);
            std::swap(m2, rhs.m2);
        }

    private:
        Bar m1;
        int m2;
    };

    // Providing a non-member swap function in the same namespace
    void swap(Foo& a, Foo& b) {
        a.swap(b);
    }
    ```

* C.84: A swap may not fail

* C.85: Make swap noexcept

* C.86: Make == symmetric with respect of operand types and noexcept
    * Asymmetric treatment of operands is surprising and a source of errors where conversions are possible.
    ```C++
    struct X { // Good
        string name;
        int number;
    };

    bool operator==(const X& a, const X& b) noexcept {
        return a.name == b.name && a.number == b.number;
    }

    class B { // Bad
        string name;
        int number;

        bool operator==(const B& a) const {
            return name == a.name && number == a.number;
        }
    };
    // B’s comparison accepts conversions for its second operand, but not its first.
    ```

* C.87: Beware of == on base classes
    * It is really hard to write a foolproof and useful == for a hierarchy.
    ```C++
    class B {
        string name;
        int number;
        virtual bool operator==(const B& a) const {
            return name == a.name && number == a.number;
        }
    };
    // B’s comparison accepts conversions for its second operand, but not its first.

    class D : B {
        char character;
        virtual bool operator==(const D& a) const {
            return name == a.name && number == a.number && character == a.character;
        }
    };

    B b;
    D d;
    b == d;    // compares name and number, ignores d's character
    d == b;    // error: no == defined
    D d2;
    d == d2;   // compares name, number, and character
    B& b2 = d2;
    b2 == d;   // compares name and number, ignores d2's and d's character
    ```

* C.89: Make a hash noexcept

* C.90: Rely on constructors and assignment operators, not memset and memcpy

##  Containers and other resource handles

* C.100: Follow the STL when defining a container
    ```C++
    template<typename T>
    class Sorted_vector {
    public:
        using value_type = T;
        // ... iterator types ...

        Sorted_vector() = default;
        Sorted_vector(initializer_list<T>);    // initializer-list constructor: sort and store
        Sorted_vector(const Sorted_vector&) = default;
        Sorted_vector(Sorted_vector&&) = default;
        Sorted_vector& operator=(const Sorted_vector&) = default;   // copy assignment
        Sorted_vector& operator=(Sorted_vector&&) = default;        // move assignment
        ~Sorted_vector() = default;

        Sorted_vector(const std::vector<T>& v);   // store and sort
        Sorted_vector(std::vector<T>&& v);        // sort and "steal representation"

        const T& operator[](int i) const { return rep[i]; }
        // no non-const direct access to preserve order

        void push_back(const T&);   // insert in the right place (not necessarily at back)
        void push_back(T&&);        // insert in the right place (not necessarily at back)

        // ... cbegin(), cend() ...
    private:
        std::vector<T> rep;  // use a std::vector to hold elements
    };

    template<typename T> bool operator==(const Sorted_vector<T>&, const Sorted_vector<T>&);
    template<typename T> bool operator!=(const Sorted_vector<T>&, const Sorted_vector<T>&);
    ```

* C.101: Give a container value semantics

* C.102: Give a container move operations

* C.103: Give a container an initializer list constructor

* C.104: Give a container a default constructor that sets it to empty
    ```C++
    // 100 Sorted_sequences each with the value ""
    vector<Sorted_sequence<string>> vs(100);
    ```

* C.109: If a resource handle has pointer semantics, provide * and ->

## Class hierarchy rule summary:

* C.120: Use class hierarchies to represent concepts with inherent hierarchical structure (only)

* C.121: If a base class is used as an interface, make it a pure abstract class

* C.122: Use abstract classes as interfaces when complete separation of interface and implementation is needed

## Designing rules for classes in a hierarchy summary:

* C.126: An abstract class typically doesn’t need a constructor

* C.127: A class with a virtual function should have a virtual or protected destructor

* C.128: Virtual functions should specify exactly one of `virtual, override, or final`

* C.129: When designing a class hierarchy, distinguish between `implementation inheritance` and `interface inheritance`

* C.130: For making deep copies of polymorphic classes prefer a `virtual clone function` instead of copy construction/assignment

* C.131: Avoid trivial getters and setters
    * A trivial getter or setter adds no semantic value; the data item could just as well be public

* C.132: Don’t make a function virtual without reason

* C.133: Avoid protected data

* C.134: Ensure all `non-const data members` have the `same access level`

* C.135: Use multiple inheritance to represent multiple distinct interfaces

* C.136: Use multiple inheritance to represent the union of implementation attributes

* C.137: Use virtual bases to avoid overly general base classes

* C.138: Create an overload set for a derived class and its bases with using

* C.139: Use final on classes sparingly

* C.140: Do not provide different default arguments for a virtual function and an overrider

## Accessing objects in a hierarchy rule summary:

* C.145: Access polymorphic objects through pointers and references

* C.146: Use dynamic_cast where class hierarchy navigation is unavoidable

* C.147: Use dynamic_cast to a reference type when failure to find the required class is considered an error

* C.148: Use dynamic_cast to a pointer type when failure to find the required class is considered a valid alternative

* C.149: Use unique_ptr or shared_ptr to avoid forgetting to delete objects created using new

* C.150: Use make_unique() to construct objects owned by unique_ptrs

* C.151: Use make_shared() to construct objects owned by shared_ptrs

* C.152: Never assign a pointer to an array of derived class objects to a pointer to its base

* C.153: Prefer virtual function to casting
    * A virtual function call is safe, whereas casting is error-prone.
    * A virtual function call reaches the most derived function, whereas a cast may reach an intermediate class and therefore give a wrong result

## C.over: Overloading and overloaded operators

* C.160: Define operators primarily to mimic conventional usage

* C.161: Use non-member functions for symmetric operators

* C.162: Overload operations that are roughly equivalent
    * Having different names for logically equivalent operations on different argument types is confusing, leads to encoding type information in function names, and inhibits generic programming.
    ```C++
    void print(int a);
    void print(int a, int base);
    void print(const string&);

    void print_int(int a);
    void print_based(int a, int base);
    void print_string(const string&);
    ```

* C.163: Overload only for operations that are roughly equivalent
    * Having the same name for logically different functions is confusing and leads to errors when using generic programming.
    ```C++
    void open_gate(Gate& g);   // remove obstacle from garage exit lane
    void fopen(const char* name, const char* mode);   // open file

    // functions have different purpose, but same name, confusing
    void open(Gate& g);   // remove obstacle from garage exit lane
    void open(const char* name, const char* mode ="r");   // open file
    ```

* C.164: Avoid implicit conversion operators

* C.165: Use using for customization points
    ```C++
    namespace N {
        My_type X { /* ... */ };
        void swap(X&, X&); // optimized swap for N::X
    }

    void f1(N::X& a, N::X& b) {
        std::swap(a, b);   // probably not what we wanted: calls std::swap()
    }

    void f2(N::X& a, N::X& b) {
        swap(a, b);        // calls N::swap
    }

    void f3(N::X& a, N::X& b) {
        using std::swap;  // make std::swap available
        swap(a, b);       // calls N::swap if it exists, otherwise std::swap
    }
    ```

* C.166: Overload unary & only as part of a system of smart pointers and references

* C.167: Use an operator for an operation with its conventional meaning

* C.168: Define overloaded operators in the namespace of their operands

* C.170: If you feel like overloading a lambda, use a generic lambda
    ```C++
    void f(int);
    void f(double);
    auto f = [](char);   // error: cannot overload variable and function

    auto g = [](int) { /* ... */ };
    auto g = [](double) { /* ... */ };   // error: cannot overload variables

    auto h = [](auto) { /* ... */ };   // OK
    ```

## C.union: Unions

* C.180: Use unions to save Memory

* C.181: Avoid "naked” unions

* C.182: Use anonymous unions to implement tagged unions

* C.183: Don’t use a union for type punning

# Enum: Enumerations
[:link:](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-enum)

* Enum.1: Prefer enumerations over macros

* Enum.2: Use enumerations to represent sets of related named constants

* Enum.3: Prefer enum classes over "plain" enums

* Enum.4: Define operations on enumerations for safe and simple use
    ```C++
    enum class Day { mon, tue, wed, thu, fri, sat, sun };

    Day& operator++(Day& d) {
        return d = (d == Day::sun) ? Day::mon : static_cast<Day>(static_cast<int>(d)+1);
    }

    Day today = Day::sat;
    Day tomorrow = ++today;

    /* The use of a static_cast is not pretty, but is an infinite recursion,
    * and writing it without a cast, using a switch on all cases is long-winded. */
    Day& operator++(Day& d) {
        return d = (d == Day::sun) ? Day::mon : Day{++d};    // error
    }
    ```

* Enum.5: Don’t use ALL_CAPS for enumerators

* Enum.6: Avoid unnamed enumerations
    * If you can’t name an enumeration, the values are not related

* Enum.7: Specify the underlying type of an enumeration only when necessary
    * int is the default integer type. int is compatible with C enums.
    * Specifying the underlying type is necessary in forward declarations of enumerations

* Enum.8: Specify enumerator values only when necessary
    * The default gives a consecutive set of values that is good for switch-statement implementations.

# R: Resource management
[:link:](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-resource)

## Resource management rule summary:

* R.1: Manage resources automatically using resource handles and RAII (Resource Acquisition Is Initialization)

* R.2: In interfaces, use raw pointers to denote individual objects (only)

* R.3: A raw pointer (a T*) is non-owning

* R.4: A raw reference (a T&) is non-owning

* R.5: Prefer scoped objects, don’t heap-allocate unnecessarily

* R.6: Avoid non-const global variables

## Allocation and deallocation rule summary:

* R.10: Avoid malloc() and free()

* R.11: Avoid calling new and delete explicitly

* R.12: Immediately give the result of an explicit resource allocation to a manager object

* R.13: Perform at most one explicit resource allocation in a single expression statement

* R.14: Avoid [] parameters, prefer span

* R.15: Always overload matched allocation/deallocation pairs

## Smart pointer rule summary:

* R.20: Use unique_ptr or shared_ptr to represent ownership

* R.21: Prefer unique_ptr over shared_ptr unless you need to share ownership

* R.22: Use make_shared() to make shared_ptrs

* R.23: Use make_unique() to make unique_ptrs

* R.24: Use std::weak_ptr to break cycles of shared_ptrs

* R.30: Take smart pointers as parameters only to explicitly express lifetime semantics

* R.31: If you have non-std smart pointers, follow the basic pattern from std

* R.32: Take a unique_ptr<T> parameter to express that a function assumes ownership of a T

* R.33: Take a `unique_ptr<T>&` parameter to express that a function `reseats` the T

* R.34: Take a shared_ptr<T> parameter to express that a function is part owner

* R.35: Take a `shared_ptr<T>&` parameter to express that a function might `reseat` the shared pointer

* R.36: Take a const shared_ptr<T>& parameter to express that it might retain a reference count to the object

* R.37: Do not pass a pointer or reference obtained from an aliased smart pointer

# ES: Expressions and statements
[:link:](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-expr)

## General rules:

* ES.1: Prefer the standard library to other libraries and to "handcrafted code"

* ES.2: Prefer suitable abstractions to direct use of language features

## Declaration rules:

* ES.5: Keep scopes small

* ES.6: Declare names in for-statement initializers and conditions to limit scope

* ES.7: Keep common and local names short, and keep uncommon and non-local names longer

* ES.8: Avoid similar-looking names

* ES.9: Avoid ALL_CAPS names

* ES.10: Declare one name (only) per declaration

* ES.11: Use auto to avoid redundant repetition of type names
    * Avoid auto for initializer lists and in cases where you know exactly which type you want and where an initializer might require conversion.

* ES.12: Do not reuse names in nested scopes

* ES.20: Always initialize an object

* ES.21: Don’t introduce a variable (or constant) before you need to use it

* ES.22: Don’t declare a variable until you have a value to initialize it with

* ES.23: Prefer the {}-initializer syntax
    * {}-initializers do not allow narrowing conversions
    * {} initialization can be used for nearly all initialization; other forms of initialization can’t
    * `={}` gives copy initialization whereas `{}` gives direct initialization.

* ES.24: Use a unique_ptr<T> to hold pointers

* ES.25: Declare an object const or constexpr unless you want to modify its value later on

* ES.26: Don’t use a variable for two unrelated purposes

* ES.27: Use std::array or stack_array for arrays on the stack

* ES.28: __Use lambdas for complex initialization, especially of const variables__

* ES.30: Don’t use macros for program text manipulation

* ES.31: Don’t use macros for constants or "functions"

* ES.32: Use ALL_CAPS for all macro names

* ES.33: If you must use macros, give them unique names

* ES.34: Don’t define a (C-style) variadic function
    * Not type safe. Requires messy cast-and-macro-laden code to get working right.

## Expression rules:

* ES.40: Avoid complicated expressions

* ES.41: If in doubt about operator precedence, parenthesize

* ES.42: Keep use of pointers simple and straightforward

* ES.43: Avoid expressions with undefined order of evaluation

* ES.44: Don’t depend on order of evaluation of function arguments

* ES.45: Avoid "magic constants"; use symbolic constants

* ES.46: Avoid narrowing conversions

* ES.47: Use nullptr rather than 0 or NULL

* ES.48: Avoid casts

* ES.49: If you must use a cast, use a named cast

* ES.50: Don’t cast away const

* ES.55: Avoid the need for range checking

* ES.56: Write std::move() only when you need to explicitly move an object to another scope

* ES.60: Avoid new and delete outside resource management functions

* ES.61: Delete arrays using delete and non-arrays using delete

* ES.62: Don’t compare pointers into different arrays

* ES.63: Don’t slice

* ES.64: Use the T{e}notation for construction

* ES.65: Don’t dereference an invalid pointer
    * Dereferencing an invalid pointer, such as nullptr, is undefined behavior, typically leading to immediate crashes, wrong results, or memory corruption.

## Statement rules:

* ES.70: Prefer a switch-statement to an if-statement when there is a choice

* ES.71: Prefer a range-for-statement to a for-statement when there is a choice
    ```C++
    // Note Don’t use expensive copies of the loop variable of a range-for loop:
    // This will copy each elements of vs into s.
    for (string s : vs)

    //Better:
    for (string& s : vs)

    // Better still, if the loop variable isn’t modified or copied:
    for (const string& s : vs)
    ```

* ES.72: Prefer a for-statement to a while-statement when there is an obvious loop variable

* ES.73: Prefer a while-statement to a for-statement when there is no obvious loop variable

* ES.74: Prefer to declare a loop variable in the initializer part of a for-statement

* ES.75: Avoid do-statements

* ES.76: Avoid goto

* ES.77: Minimize the use of break and continue in loops
    * Often, a loop that requires a break is a good candidate for a function (algorithm), in which case the break becomes a return.
    ```C++
    //Original code: break inside loop
    void use1() {
        std::vector<T> vec = {/* initialized with some values */};
        T value;
        for (const T item : vec) {
            if (/* some condition*/) {
                value = item;
                break;
            }
        }
    }

    //BETTER: create a function and return inside loop
    T search(const std::vector<T> &vec) {
        for (const T &item : vec) {
            if (/* some condition*/) return item;
        }
        return T(); //default value
    }

    void use2() {
        std::vector<T> vec = {/* initialized with some values */};
        T value = search(vec);
    }
    ```

* ES.78: Don’t rely on implicit fallthrough in switch statements

* ES.79: Use default to handle common cases (only)

* ES.84: Don’t try to declare a local variable with no name

* ES.85: Make empty statements visible
    ```C++
    for (i = 0; i < max; ++i);   // BAD: the empty statement is easily overlooked
    v[i] = f(v[i]);

    for (auto x : v) {           // better
        // nothing
    }
    v[i] = f(v[i]);
    ```

* ES.86: Avoid modifying loop control variables inside the body of raw for-loops

* ES.87: Don’t add redundant == or != to conditions
    ```C++
    // These all mean "if `p` is not `nullptr`"
    if (p) { ... }            // good
    if (p != 0) { ... }       // redundant `!=0`; bad: don't use 0 for pointers
    if (p != nullptr) { ... } // redundant `!=nullptr`, not recommended

    // Prefer
    if (auto pc = dynamic_cast<Circle>(ps)) { ... } // execute if ps points to a kind of Circle, good

    if (auto pc = dynamic_cast<Circle>(ps); pc != nullptr) { ... } // not recommended
    ```

## Arithmetic rules:

* ES.100: Don’t mix signed and unsigned arithmetic
    * Unfortunately, C++ uses signed integers for array subscripts and the standard library uses unsigned integers for container subscripts.

* ES.101: Use unsigned types for bit manipulation

* ES.102: Use signed types for arithmetic

* ES.103: Don’t overflow

* ES.104: Don’t underflow

* ES.105: Don’t divide by zero

* ES.106: Don’t try to avoid negative values by using unsigned

* ES.107: Don’t use unsigned for subscripts, prefer gsl::index
    * To avoid signed/unsigned confusion. To enable better optimization. To enable better error detection. To avoid the pitfalls with auto and int.

# Per: Performance
[:link:](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-performance)

* Per.1: Don’t optimize without reason

* Per.2: Don’t optimize prematurely
    * Elaborately optimized code is usually larger and harder to change than unoptimized code.

* Per.3: Don’t optimize something that’s not performance critical
    * Optimizing a non-performance-critical part of a program has no effect on system performance.

* Per.4: Don’t assume that complicated code is necessarily faster than simple code

* Per.5: Don’t assume that low-level code is necessarily faster than high-level code

* Per.6: Don’t make claims about performance without measurements

* Per.7: Design to enable optimization

* Per.10: Rely on the static type system

* Per.11: Move computation from run time to compile time

* Per.12: Eliminate redundant aliases

* Per.13: Eliminate redundant indirections

* Per.14: Minimize the number of allocations and deallocations

* Per.15: Do not allocate on a critical branch

* Per.16: Use compact data structures

* Per.17: Declare the most used member of a time-critical struct first

* Per.18: Space is time

* Per.19: Access memory predictably

* Per.30: Avoid context switches on the critical path

# CP: Concurrency and parallelism
[:link:](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-concurrency)

* CP.1: Assume that your code will run as part of a multi-threaded program

* CP.2: Avoid data races

* CP.3: Minimize explicit sharing of writable data

* CP.4: Think in terms of tasks, rather than threads

* CP.8: Don’t try to use volatile for synchronization

* CP.9: Whenever feasible use tools to validate your concurrent code

* [CP.20: Use RAII, never plain lock()/unlock()](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#cpcon-concurrency)

* CP.21: Use std::lock() or std::scoped_lock to acquire multiple mutexes

* CP.22: Never call unknown code while holding a lock (e.g., a callback)

* CP.23: Think of a joining thread as a scoped container

* CP.24: Think of a thread as a global container

* CP.25: Prefer gsl::joining_thread over std::thread

* CP.26: Don’t detach() a thread

* CP.31: Pass small amounts of data between threads by value, rather than by reference or pointer

* CP.32: To share ownership between unrelated threads use shared_ptr

* CP.40: Minimize context switching

* CP.41: Minimize thread creation and destruction

* CP.42: Don’t wait without a condition

* CP.43: Minimize time spent in a critical section

* CP.44: Remember to name your lock_guards and unique_locks

* CP.50: Define a mutex together with the data it guards. Use synchronized_value<T> where possible

# E: Error handling
[:link:](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-errors)

* E.1: Develop an error-handling strategy early in a design

* E.2: Throw an exception to signal that a function can’t perform its assigned task

* E.3: Use exceptions for error handling only

* E.4: Design your error-handling strategy around invariants

* E.5: Let a constructor establish an invariant, and throw if it cannot

* E.6: Use RAII to prevent leaks

* E.7: State your preconditions

* E.8: State your postconditions

* E.12: Use noexcept when exiting a function because of a throw is impossible or unacceptable

* E.13: Never throw while being the direct owner of an object

* E.14: Use purpose-designed user-defined types as exceptions (not built-in types)

* E.15: Catch exceptions from a hierarchy by reference

* E.16: Destructors, deallocation, and swap must never fail

* E.17: Don’t try to catch every exception in every function

* E.18: Minimize the use of explicit try/catch

* E.19: Use a final_action object to express cleanup if no suitable resource handle is available

* E.25: If you can’t throw exceptions, simulate RAII for resource management

* E.26: If you can’t throw exceptions, consider failing fast

* E.27: If you can’t throw exceptions, use error codes systematically

* E.28: Avoid error handling based on global state (e.g. errno)

* E.30: Don’t use exception specifications

* E.31: Properly order your catch-clauses

# Con: Constants and immutability
[:link:](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-const)

* Con.1: By default, make objects immutable

* Con.2: By default, make member functions const

* Con.3: By default, pass pointers and references to consts

* Con.4: Use const to define objects with values that do not change after construction

* Con.5: Use constexpr for values that can be computed at compile time

# T: Templates and generic programming
[:link:](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-templates)

## Template use rule summary:

* T.1: Use templates to raise the level of abstraction of code

* T.2: Use templates to express algorithms that apply to many argument types

* T.3: Use templates to express containers and ranges

* T.4: Use templates to express syntax tree manipulation

* T.5: Combine generic and OO techniques to amplify their strengths, not their costs

## Concept use rule summary:

* T.10: Specify concepts for all template arguments

* T.11: Whenever possible use standard concepts

* T.12: Prefer concept names over auto for local variables

* T.13: Prefer the shorthand notation for simple, single-type argument concepts

## Concept definition rule summary:

* T.20: Avoid "concepts" without meaningful semantics

* T.21: Require a complete set of operations for a concept

* T.22: Specify axioms for concepts

* T.23: Differentiate a refined concept from its more general case by adding new use patterns

* T.24: Use tag classes or traits to differentiate concepts that differ only in semantics

* T.25: Avoid complementary constraints

* T.26: Prefer to define concepts in terms of use-patterns rather than simple syntax

* T.30: Use concept negation (!C<T>) sparingly to express a minor difference

* T.31: Use concept disjunction (C1<T> || C2<T>) sparingly to express alternatives

## Template interface rule summary:

* T.40: Use function objects to pass operations to algorithms

* T.41: Require only essential properties in a template’s concepts

* T.42: Use template aliases to simplify notation and hide implementation details

* T.43: Prefer using over typedef for defining aliases

* T.44: Use function templates to deduce class template argument types (where feasible)

* T.46: Require template arguments to be at least Regular or SemiRegular

* T.47: Avoid highly visible unconstrained templates with common names

* T.48: If your compiler does not support concepts, fake them with enable_if

* T.49: Where possible, avoid type-erasure

## Template definition rule summary:

* T.60: Minimize a template’s context dependencies

* T.61: Do not over-parameterize members (SCARY)

* T.62: Place non-dependent class template members in a non-templated base class

* T.64: Use specialization to provide alternative implementations of class templates

* T.65: Use tag dispatch to provide alternative implementations of functions

* T.67: Use specialization to provide alternative implementations for irregular types

* T.68: Use {} rather than () within templates to avoid ambiguities

* T.69: Inside a template, don’t make an unqualified non-member function call unless you intend it to be a customization point

## Template and hierarchy rule summary:

* T.80: Do not naively templatize a class hierarchy

* T.81: Do not mix hierarchies and arrays

* T.82: Linearize a hierarchy when virtual functions are undesirable

* T.83: Do not declare a member function template virtual

* T.84: Use a non-template core implementation to provide an ABI-stable interface

## Variadic template rule summary:

* T.100: Use variadic templates when you need a function that takes a variable number of arguments of a variety of types

* T.101: ??? How to pass arguments to a variadic template ???

* T.102: ??? How to process arguments to a variadic template ???

* T.103: Don’t use variadic templates for homogeneous argument lists

## Metaprogramming rule summary:

* T.120: Use template metaprogramming only when you really need to

* T.121: Use template metaprogramming primarily to emulate concepts

* T.122: Use templates (usually template aliases) to compute types at compile time

* T.123: Use constexpr functions to compute values at compile time

* T.124: Prefer to use standard-library TMP facilities

* T.125: If you need to go beyond the standard-library TMP facilities, use an existing library

## Other template rules summary:

* T.140: Name all operations with potential for reuse

* T.141: Use an unnamed lambda if you need a simple function object in one place only

* T.142: Use template variables to simplify notation

* T.143: Don’t write unintentionally non-generic code

* T.144: Don’t specialize function templates

* T.150: Check that a class matches a concept using static_assert

# SF: Source files
[:link:](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-source)

* SF.1: Use a .cpp suffix for code files and .h for interface files if your project doesn’t already follow another convention

* SF.2: A .h file may not contain object definitions or non-inline function definitions

* SF.3: Use .h files for all declarations used in multiple source files

* SF.4: Include .h files before other declarations in a file

* SF.5: A .cpp file must include the .h file(s) that defines its interface

* SF.6: Use using namespace directives for transition, for foundation libraries (such as std), or within a local scope (only)

* SF.7: Don’t write using namespace at global scope in a header file

* SF.8: Use #include guards for all .h files

* SF.9: Avoid cyclic dependencies among source files

* SF.10: Avoid dependencies on implicitly #included names

* SF.11: Header files should be self-contained

* SF.12: Prefer the quoted form of #include for files relative to the including file and the angle bracket form everywhere else

* SF.20: Use namespaces to express logical structure

* SF.21: Don’t use an unnamed (anonymous) namespace in a header

* SF.22: Use an unnamed (anonymous) namespace for all internal/non-exported entities

# A: Architectural ideas
[:link:](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-A)

* A.1: Separate stable code from less stable code

* A.2: Express potentially reusable parts as a library

* A.4: There should be no cycles among libraries

# NR: Non-Rules and myths
[:link:](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-not)

* NR.1: Don’t insist that all declarations should be at the top of a function

* NR.2: Don’t insist to have only a single return-statement in a function

* NR.3: Don’t avoid exceptions

* NR.4: Don’t insist on placing each class declaration in its own source file

* NR.5: Don’t use two-phase initialization

* NR.6: Don’t place all cleanup actions at the end of a function and goto exit

* NR.7: Don’t make all data members protected

# NL: Naming and layout rules
[:link:](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#S-naming)

* NL.1: Don’t say in comments what can be clearly stated in code

* NL.2: State intent in comments

* NL.3: Keep comments crisp

* NL.4: Maintain a consistent indentation style

* NL.5: Avoid encoding type information in names

* NL.7: Make the length of a name roughly proportional to the length of its scope

* NL.8: Use a consistent naming style

* NL.9: Use ALL_CAPS for macro names only

* NL.10: Prefer underscore_style names

* NL.11: Make literals readable

* NL.15: Use spaces sparingly

* NL.16: Use a conventional class member declaration order

* NL.17: Use K&R-derived layout

* NL.18: Use C++-style declarator layout

* NL.19: Avoid names that are easily misread

* NL.20: Don’t place two statements on the same line

* NL.21: Declare one name (only) per declaration

* NL.25: Don’t use void as an argument type

* NL.26: Use conventional const notation
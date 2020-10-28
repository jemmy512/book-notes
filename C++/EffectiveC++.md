# Item_1: View C++ as a federation of languages.
A multiparadigm programming language, one supporting a combination of procedural, object-oriented, functional generic and metaprogramming features.

# Item_2: Prefer consts, enums, and inline to #defines.
When replacing #define with contants:
1. Defining constant pointers.
2. Class-specific constants. To limit the scope of a constant to a class, make it a member, and to ensuer there's at most one copy of the constant, make it a static member.
    * For simple constants, prefer const object or enum to #define
    * For function-like macros, prefer inline functions to #define

# Item_3: Use const whenever possible.
Const Member Function:
* The purpose is to identify which member functions may be invoked on const objects.

Avoiding Duplication in const and Non-const Member Functions:
* Having the non-const version function call the const version is a safe way to avoid code duplication.
```C++
class TextBlock {
public:
    const char &operator[](std::size_t position) const {
        return text[position];
    }

    char &operator[](std::size_t position) {
        return
            const_cast<char &>(                     // cast away const on op[]'s return type;
            static_cast<const TextBlock &>(*this)   // add const to *this's type;
            [position]                              // call const version of op[];
        );
    }
};
```
Compilers enforce bitwise constness, but you should program using logical constness.

When const and non-const member functions have essentially identical implementations, code duplication can be avoided by having the non-const version call the const version.

# Item_4: Make sure that objects are initialized before they're used.
* The rules of C++ stipulate that data members of an object are initialized before the body of a constructor is entered.
    ```C++
    A::A(string name, int age, string sex) {name = name; age = age; sex = sex;}
    A::A(string name, int age, string sex) : name(name), age(age), sex(sex) {}
    ```
* The assignment-based version first called default constructor to initialize name, age, sex then promptly assigned new value in those default-constructed ones. All the work performed in those default constructions was wasted. The member initialization list approach avoids that problem.
* **Manually initialize** objects of **built-in type**, because C++ only sometimes initializes them itself.
* In a constructor, prefer use of the member initialization list to assignment inside the body of the constructor.
* Avoid initialization order problems across translation units by replacing non-local static object with local static obj.

# Item_5: Know What Functions C++ silently Writes and Calls.
* Compilers may implicitly generate a class's default constructor, copy constructor, copy assignment operator, and destructor.
* If a class contains reference member, const member, and you want to support assignment operator, compiler refuse to work unless you define your own assignment operator, in this case default assignment operator doesn't work.

# Item_6: Explicitly disallow the use of compiler-generated functions you do not want
Before C++11:
* To disallow functionality automatically provided by compilers, declare the corresponding member functions private and give no implementation.

After C++11:
* use `delete` keyword.

# Item_7: Declare destructor virtaul in polymorphic base class
* C++ sepecifies that a derived class object is deleted through a pointer to a base class with a non-virtual destructor, results are undefiend. The derived part of the object is never destroyed but base class part typically would be destroyed, thus leading a curious partially destroyed object.
* Polymorphic base class should declare virtual destructor. __If a class has any virtual functions,it should have a virtual destructor__.
* Classes not designed to be base calsses or not designed to be used polymorphically should not declare virtual destructor.

# Item_8: Prevent exceptions from leaving destructors
* Destructor should never emit exceptions. If functions called in a destructor may throw, the destructor should catch any exceptons, then swallow them or terminate the program.
* If class clients need to be able to react to exceptions thrown during an operation, the class should provide a regular function [i.e., non-destructor] that perform the operation.

# Item_9: Never call Virutal Functions During Construction or Destruction
Don't call virtual functions during construction or destruction, because such call will never go to a more derived class than that of the currently executing constructor or destructor.

# Item_10: Having Assignment Operators Return a Reference to `*this`.
The assignment operators return a reference to `*this`, this is a convention.

# Item_11: Handle assignment to self in `operator=`
* In general, code that operates on references or poiters to multiple objects of the same type needs to consider that objects might be the same. Two objects need not even be declared to be of the same type.
* The traditional way to prevent self assignment is via an identity test at the top of the operator=.
* Make suer operator= is well-behaved when an object is assigned to itself. Techniques include comparing address of source and target objects, careful statement ordering, and copy-and-swap.
* Make suer that any function operation on more than one object behaves correctly if two or more of the objects are the same.

# Item_12: Copy all parts of an object
* Copy functions should be sure to copy all of the object's data members and all of its base class parts(via calling appropriate base functions).
* Don't try to implement one of the copying functions in terms of the other. Instead, put common functionality in a third function that both call.

# Item_13: Use objects to manage resources
* To prevent resource leaks, use `RAII(resource acquisition is initialization)` objects that acquire resources in
    their constructors and release them in their destructor.
* Two commonly useful RAII calsses are std::shared_ptr and auto_ptr. shared_ptr is usually the better choise, because it behavior when copied is inituitive. Copying an auto_ptr set it to null.
* Both auto_ptr and shared_ptr use 'delete' in their destructors, not 'delet[]'. There is nothing like auto_ptr or shared_ptr for dynamically allocated arrays in C++, because vector and string can almost replace dynamically allocated arrays.

# Item_14: Think carefully about copying behavior in resource-managing classes
What happen when an RAII object is copied?
1. Prohibit copying.
2. Reference-count the underlying resource.
    ```C++
    class Lock{
    public:
        explicit Lock(Mutex *pm) : mutexPtr(pm, unlock) { // init shared_ptr with Mutex to point to and
            lock(mutexPtr.get());                         // the unlock function as the deleter
        }
        // There is no need to declare destructor, default it enough. Class's destructor automatically invokes
        // the destructors of the class's non-static data members, here is mutexPtr, but its destructor will
        // automatically call the shared_ptr's deleter--unlock.
    private:
        std::shared_ptr<Mutex> mutexPtr;
    };
    ```
3. Copy the underlying resource.
4. Transfer ownership of the underlying resource.

* Copying an RAII object entails copying the resource it manages, so the copying behavior of the resource determines the copying bahavior of the RAII object.
* Common RAII class copying behaviors are disallowing copying and performing reference counting, but other behavior are possible.

# Item_15: Provide access to raw resources in resource managing classes.
* APIs often require access to raw resource, so each RAII calss should offer a way to get at the resource it manages.
* Access may be via explicit conversion or implicit conversion. In general, explicit conversion is safer, but implicit is more convenient for clients.

# Item_16: Use the same form in corresponding uses of the new and delete.
* Single object memory layout is generally different from arrays's. Array memory layout includes the size of the array.
* If you use `[]` in new expression, you should use `[]` in the corresponding delete expresion.

# Item_17: Store newed object in smart pointers in standalone statements.
C++ compilers are granted considerable latitude in determing the order in which these things are to be done.

Function ProtoType:
```C++
int priority();
void processWidger(shared_ptr<Widget> pw, int priority);
```

Usage:
```C++
/* can't compile. no implicit conversion from raw pointer return by
 * expression 'new Widget' to shared_ptr required by processWidger. */
processWidger(new Widget, priority());

/* can compile. But may cause resource leak. If 'new Widget' execute first,
 * then priority(), but priority() yields an exception,
 * in this case pointer returned from 'new Widget' will be lost.*/
processWidger(shared_ptr<Widget>(new Widget), priority());

/* prefered */
processWidger(make_shared_<Widget>(), priority());
```

Store newed object in smart pointers in standalone statements. Failure to do this can lead to subtle resource leaks when exception are thrown.

# Item_18: Make interfaces easy to use correctly and hard to use incorrectly.
* You should strive for these characteristics in all your interfaces.
* Ways to facilitate correct use include consistency in interfaces and behavioral compatibility with build-in type.
* Ways to prevent errors include creating new types, restricting operations on types, constraining object values, and eliminating client resource management responsibilities.
* shared_ptr supports custom deleters. This prevents the crosss-DLL problem, can be used to automatically unlock mutexes.

# Item_19: Treat class design as type design
1. How should object of your new type be created and destroyed?
2. How should object initialization differ from object assignment?
3. What does it mean for object of your new type to be passed by value?
4. What are the restrictions on legal values of for your new type?
5. Does your new type fit into an inheritance graph?
6. What kind of type conversions are allowed for your new type?
7. What operators and functions make sense for the new type?
8. What standard functions should be disallowed?
9. Who should have access to the members of your type?
10. What is the "undeclared interface" of your new type?
11. How general is your new type?
12. Is a new type really what your need?

# Item_20: Prefer pass-by-reference-to-const to pass-by-value
* By default, C++ passes objects to and from functions by value.
* References are typically implemented as pointers.
* Pass-by-referenc-to-const is efficient and can avoid the slicing problem.
* The rule doesn't apply to `built-in types` and `STL iterator` and `function object` types.

# Item_21: Don't try to return a reference when you must be return an object
* A reference is just a name, a name for some existing object.
* The cost of an assignment is about the same as a call to a destructor(to destroy the old value) plus a call to a constructor(to copy over the new value).
* Never return a pointer or reference to a local stack object, a reference to a head-allocated objec, or a pointer or a reference to a local static object if there is a chance that more than one such object will be needed.

# Item_22: Declare data members private
* Declare data member private. It gives clients syntactically uniform access to data, affords fine-grained access control, allows invariants to be enforced, and offers class authors implementation flexibility.
* Protected is no more encapsulated than public.

# Item_23: Prefer non-member non-friend functions to member functions
* The more something is encapsulated, the fewer things can see it. The few things can see it, the greater flexibility we have to change it. It affords us the flexibility to change things in a way that affects only a limited number of clients.
* This adivce can increase encapsulation, packaging flexibility, and functional extensibility.

# Item_24: Declare non-member functions when type conversions should apply to all parameters
* Implicit type conversion: compilers know you're passing an int that the function requires a Rational, but they also know they can conjure up a suitable Rational by calling the Rational constructor with int you passed. Compilers do this only because non-explicit constructor is involved.

# Item_25: Consider support for a non-throwing swap
* Compilers will prefer a T-specific specialization of std::swap over the general template, so if std::swap has been specialized for T, the specialzied version will be used.
* Provide a swap member function when std::swap would be inefficient for your type. Make sure your swap doesn't
    throw exception.
* If you offer a `member swap`, also offer a `non-member swap` that calls the member. For class(not templates), specialize std::swap too.
* When calling swap, employ a using declaration for std::swap, then call swap without namespace qualification.
* It's fine to totally specialize std templates for user-defined types, but never try to add something completely
    new to std.

# Item_26: Postpone variable definitions as long as possible

# Item_27: Minimize casting
* dynamic_cast: is primarily used to perform "safe downcasting", i.e., to determine whether an object is of a particular type in an inheritance hierarchy.
* Avoid casts whenever practical, especially dynamic_casts in performance-sensitive code. If a design requires casting, try to devolop a cast-free alternative
* When casting is necessary, try to hide it inside a function. clients can then call the function instead of putting casts in their own code.
* Prefer C++-style casts to old-style cast. They are easier to see, and they are more specific about what they do.

# Item_28: Avoid returning "handles" to object internals
* A data member is only as encapsulated as the most accessible function returing a reference to it.
* If a const member function returns a reference to data associated with an object that is store outside the object itself, the caller of the function can modify that data.
* References, pointers, and iterators are all hanlders(ways to get at other objects), and returning a handle to an object's internals always runs the risk of compromising an object's encapsulation. It can also lead to const member functions that allow an objects's state to be modified.
* Those handles may lead to dangling problems.

# Item_29: Strive for exceptoion-safe code
* Rquirements for exception safety:
    * Leak no resources
    * Don't allow data structures to became corrupted
* Exception-safe functions offer one of three guarantees:
    * Functions offering the basic guarantee promise that if an exception is thrown, everything in the program remains in a valid state.
    * Functions offering the strong gurantee promise that if an exceptions is thrown, the state of the program is unchanged
    * Functions offering the nothrow guarantee promise never to thrown exceptions, because they do what they promise to do.
* The strong guarantee can ofen be implemented via copy-and-swap but the strong guarantee is not practical for all functions

# Item_30: Understand the ins and outs of inlining
* Limit most inlining to small, frequently called functions. This facilitates debugging and binary upgradability, minimizes potential code bloat, and maximizes the chances of greater program speed.
* Don't declare function templates inline just because they appear in header files.

# Item_31: Minimize compilation dependencies between files

# Item_32: Make sure public inheritance models "is-a"
* Public inheritance means "is-a". Everythins that applies to base classes must also apply to derived classes, because every derived class object is a base class object.

# Item_33: Avoid hiding inherited names
* Names in derived classes hide names in base classes. Under public inheritance, this is never desirable.
* To make hidden names visible again, employ `using declaration` or `forwarding functions`.

# Item_34: Differentiate between inheritence of interface and inheritance of implementation
* The purpose of declaring a pure virtual function is to have derived classes inherit a function interface only.
* The purpose of declaring a simple virtual function is to have derived classes inherit a function interface as well as a default implementation.
* The purpose of declaring a non-virtual function is to have derived classes inherit a function interface as well as a mandatory implementation.

# Item_35: Consider alternatives to virtual functions
* The Template Method Pattern via Non-Virtual Interface Idiom
* The Strategy Pattern via Function Pointer
* The Strategy Pattern via tr1::function
* Replace virtual functions in one hierarchy with virtual functions in another hierarchy.

# Item_36: Never redefine an inherited non-virtual function
* Public inheritance means is-a. Everything that applies to B objects also applies to D object, because every D object is a B object.
* Declaring a non-virtual function establishes an invarient over specialization for that class.
* Class derived from B must inherit the interface and the implementation for mf, because mf is non-virutal in B.

# Item_37: Never redefine a function's inherited default parameter value
* Default parameters are statically bound while virtual functions are dynamically bound.

Q: why C++ insist on acting in this perverse manner?
A: The answer has to do with runtime efficiently.

# Item_38: Model `has-a` or `is-implemented-in-terms-of` through composition
* Composition has meanings completely different from that of public inheritance.
* In application domain, composition means has-a. In implementation domain, it means is-implemented-in-terms-of.

# Item_39: Use private inheritence judiciously
* Private inheritance behavior:
    * In contrast to public inheritance, compilers will not convert a derived class object to a base object.
    * Members inherited from private base class become private members of derived class.
* How to choose between private inheritance and composition which both mean is-implemented-in-terms-of?
    * Use composition whenever you can and use private inheritance you must.
    ```C++
    class Widget {
    private:
        class WidgetTimer: public Timer {
            virtual void onTick() const;
        };

        WidgetTimer timer;
    };
    ```
* Prefer public inheritance plus composition instead private inheritance.
    * Can prevent derived class from redefining virtual functions of the base class.
    * Minimize compilation dependencies.
*  The constraint that "freestanding objects mustn't have zero size" doesn't apply to base calss parts of derived class object. This is known as `empty base optimization(EBO)`. EBO is only viable under single inheritance.
* Private inheritance is most likely to be a legitimate design strategy when you’re dealing with two classes not related by is-a where one either needs access to the protected members of another or needs to redefine one or more of its virtual functions.
* Unlike composition private inheritance can enable empty base optimization(EBO).

# Item_40: Use multiple inheritance judiciously
* C++ checkes accessibility only after finding the best-match function.
* Initializaion rules of virtual base classes:
    * Class derived from virtual bases that require initialization must be aware of their bases
    * When a new derived class is added to the hierarchy, it must assume initialization responsibilities of it's virtual bases
* Advices on virtual base classes:
    * Don't use virtual bases if have to
    * When use virtual bases try to avoid putting data in them
* Multiple inheritance does have legitimate uses. One scenario involes combining public inheritance from an interface with private inheritance from a class that helps with implementation

# Item_49: Understand the behavior of the new-handler
* Requirements of new handler:
    * Make more memory avaliable
    * Install a different new handler
    * Deinstall the new handler
    * Throw an exception
    * No return
* std::set_new_handler allows to set new handler to be called when memory allocation request can't be satisfied.
* Nothrow new is of limited utility, because it applies only to memory allocation, associated constructor may may still throw exceptions.

# Item_50: Understand when it make sense to replace new and delete
* Reasons to replace default operator new and operator delete:
    * To detect usage error
    * To increase the speed of allocation and deallocation
    * To collect statistics about the use of dynamically allocated memory
    * To reduce the space overhead of default memory management
    * To compensate for suboptimal alignment in the default allocator
    * To cluster related objects near one another
    * To obtain unconventional behavior
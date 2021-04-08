# 01: Understand template type deduction
```C++
template<typename T>
void foo(ParamType param);
foo(expr);
```

Type of T depends on both types of ParamType and expr:
1. ParamType is a pointer or reference, but not a universal reference (forwarding reference)
    * If expr's type is a reference, ignore the reference part, also const
    * Then pattern-match expr's type against ParamType to determin T
    ```C++
    template<typename T>
    void foo(T& param);

    int x = 27;         // T is int, ParamType is int&
    cosnt int x = x;    // T is int, ParamType is int&
    const int &x = x;   // T is int, ParamType is int&
    ```
2. ParamType is a universal type
    * If expr is a lvalue, both T and ParamType are lvalue reference
    * If expr is a rvalue, case 1 rules apply
    ```C++
    template<typename T>
    void foo(T&& param);    // param is now a universal reference

    int x = 27;
    const int cx = x;
    const int &rx = x;

    foo(x);     // x is lvalue, both T and ParamType are int&
    foo(cx);    // x is lvalue, both T and ParamType are const int&
    foo(rx);    // x is lvalue, both T and ParamType are const int&
    foo(27);    // 27 is rvalue, T is int, ParamType is int&&
    ```
3. ParamType is neigher a pointer nor a reference
    In this case, we are dealing with pass-by-value, and ignore expr's reference-ness, constness and volatile.
    ```C++
    template<typename T>
    void foo(T param);    // param is now passed by value

    int x = 27;
    const int cx = x;
    const int &rx = x;

    foo(x);     // both types of T and ParamType are int
    foo(cx);    // both types of T and ParamType are int
    foo(rx);    // both types of T and ParamType are int
    foo(27);    // both types of T and ParamType are int

    const char * const ptr = "Hello World";
    foo(ptr);   // T is const char *, constess of what ptr points is preversed, but not the constness of ptr itself

    // Array Arguments:
    const char name[] = "Jemmy";

    template<typename T>
    void foo(T name_);
    foo(name);  // name is array, but T deduced as const char * array decay to pointer

    void foo(T& name_);
    foo(name);  // T is deduced const char[5] while argument type is const char (&)[5]

    // Function Arguments:
    void foo(int, double);

    template<typename T>
    void f1(T param);

    void f2(T &param);

    f1(foo);    // param is deduced to ptr-to-func: void (*) (int, double)
                // function decay to pointer to function
    f2(foo);    // param is deduced to ref-to-func: void (&) (int, double)
    ```

Summary:
1. During template type deduction, referenceness, constness and volatileness are ignored
2. Array and function decay to pointers, unless they're used to initialize references

# 02: Understand auto type deduction
1. Auto type deduction is template type deduction.
2. The treatment of `braced initializers` is the only way where auto type deduction and template type deduction differ.
    * auto deduction treats {...} as initializer_list while template deduction doesn't
3. Auto in a function __return type__ or a __labmda parameter__ implies __template type deduction__ not auto type deduction.
4. Auto ordinarily __ignores the top-level cv-qualifiers__.
5. When use reference, we are really using the object to which it refers, ignore the reference.
6. When we ask for a reference to auto-deduced type, top-level consts in the initializer are not ignored.

```C++
auto x = 27;
auto x(27); // both type is int

auto x = {27};
auto x{27}; // both type is std::initializer_list<int>

auto foo() { return {1, 2, 3}; } // error: template type deduction cant't deduct initializer_list type
auto lambda = [&v](const auto & newValue) { v = newValue; }
lambda({1, 2, 3});               // error: template type deduction cant't deduct initializer_list type

/* new rules of direct initialization in C++17:
 * For a braced-init-list with only a single element, auto deduction will deduce from that entry;
 * For a braced-init-list with more than one element, auto deduction will be ill-formed. */
auto x1 = { 1, 2 };     // decltype(x1) is std::initializer_list<int>
auto x2 = { 1, 2.0 };   // error: cannot deduce element type
auto x3{ 1, 2 };        // error: not a single element
auto x4 = { 3 };        // decltype(x4) is std::initializer_list<int>
auto x5{ 3 };           // decltype(x5) is int

// auto& and auto* versus auto
// always be explicit and use the form auto& as well as auto* even if auto is able to deduce a pointer type.
Foo* GetFoo() { ... }

auto fp0 = GetFoo();                // Foo *
const auto fp1 = GetFoo();          // Foo * const
auto const fp2 = GetFoo();          // Foo * const
const auto const fp3 = GetFoo();    // error
const auto* fp4 = GetFoo();         // const Foo *
auto* const fp5 = GetFoo();         // Foo * const
const auto* const fp6 = GetFoo();   // cosnt Foo * const
```

# 03: Understand decltype
Auto removes all top-level qulifiers while decltype doesn't.
```C++
/* C++11
 * auto befor function name has nothing to do with type deduction
 * it indicates C++11 trailing return type syntax is being used */
template<typename Container, typename Index>
auto authAndAccess(Container& c, Index i) -> decltype(c[i]) { // traling return type can be omit in C++14
    authenticateUser();
    return c[i];
}
```
C++11 permits return types for sigle-statement lambdas to be deduced, C++14 extends this to both all lambdas and functions, including those with multiple statements.

```C++
std::deque<int> d;
authAndAccess(d, 5) = 10;
/* compile error, return c[i] type is int&, but auto deduction will ignore reference-ness, so
 * authAndAccess(d, 5) return type is int. */

template<typename Container, typename Index>
decltype(auto) authAndAcess(container &c, Index i) {    // now authAndAccess return is int&
    return c[i];
}

int a = 20;
const int &age = a;
auto age0 = age;            // age0 type is int
decltype(auto) age1 = age;  // age1 type is const int&

// Above authAndAccess can only accept lvalue container, we can support rvalue by this one:
template<typename Container, typename Index>
auto authAndAcess(Container &&c, Index i) { // universal reference
    return forward<Container>(c)[i];
}

/* decltype((x)) return type is reference eg: */
decltype(auto) foo() { int x = 0; return x; }   // return type is int
decltype(auto) foo() { int x = 0; return (x); } // return type is int&
```
Decltype almost always yields the type of a variable or expression without any modifications.

For lvalue expressions of type T other than names, decltype always reports a type of T&.

C++14 supports decltype(auto), which, like auto, deduces a type from its initializer, but it performs the type deduction using the decltype rules


# 05: Prefer auto to explicit type declaration
```C++
std::function<bool(const std::unique_ptr<Widget> &, const std::unique_ptr<Widget> &)>
derefUPLess = [](const std::unique_ptr<Widget> &p1, const std::unique_ptr<Widget> &p2) {
  return *p1 < *p2;
};

auto derefUPLess = [](const auto &p1, const auto &p2) { return *p1 < *p2; };

/* auto-declared variable holding a closure has the same type as the closure,
 * and as much it uses only as many memory as the closure requires.
 * While std::function has fixed size for any signature, that's may not be adequate for the closure it's
 * asked to store, in the case, std::function constructor allocate head memory to store the closure.
 * So, std::function object typically uses more memory that auto-declard object.
 *
 * All in all, std::function is bigger and slower than auto, and it may yield out-of-memory exception. */


unordered_map<string, int> mp;
for (const pair<string, int> &p : mpi;  // implicit conversion
for (const auto &p : mp);
/* std::map::value_type here is pair<const string, int>, compiler will convert pair<const string, int>
 * to pair<string, int> by create a temporary object which is bound to p, so take p's address will
 * will get the temporary object's address. So it's not efficient. */
```

Auto variables must be initialzed, are generally immune to type mismatches that can lead to portability or efficiency problems, can ease the process of refactoring, and typically requires less typing than variables with explicitly specified types.

auto-typed variables are subject to the pitfalls described in # 02 and 06.

# 06: Use the explicit initializer idiom  when auto deduces undesired types
```C++
vector<bool> features(const Widget &w);
bool highPriority = features(w)[5]; // return type is vector<bool>::reference implicitly convert to bool
auto highPriority = features(w)[5]; // return type is vector<bool>::reference
processWidget(w, highPriority);     // for auto version, undefined behavior

auto index = static_cast<int>(d * c.size());
```
Invisible” proxy types can cause auto to deduce the “wrong” type for an ini‐ tializing expression.
The explicitly typed initializer idiom forces auto to deduce the type you want it to have.

# 07: Distinguish between () and {} when creating objects

A novel feature of **braced initialization** is that it **prohibits implicit narrowing conversions** among built-in types.

Another noteworthy characteristic of braced initialization is its immunity to C++’s most vexing parse:
```C++
Widget w1(10);  // call ctor with argument 10
Widget w2();    // most vexing parse: declares a function with return type of Widget
Widget w3{};    // call Widget ctor with no argument
```

This vexing parse is caused by the rule: A side effect of C++’s rule that anything that can be parsed as a
declaration must be interpreted as one.

In the constructor, parentheses and braces have the same meaning as long as std::initializer_list is not involved:
```C++
class Widget {
public:
    widget(int i, bool b);
    Widget(int i, double d);
};

Widget w1(10, true); Widget w2{10, true};   // call first ctor
Widget w3(10, 5.0);  Widget w4{10, 5.0};    // call second ctor
```

If std::initializer_list involed, calls taking breaces initialization syntax strong prefer the overloads taking std::initializer_lits:
```C++
Widget(std::initalizer_list<long double> il);
Widget w5{10, true}; Widget w6{10, 5.0}; // 10, true and 5.0 convert to long double
```

Compiler's determination to match braced initializer with constructor taking std::inistailizer_list is so strong, it prevails even if the best-match std::initializer_list can't be called:
```C++
Widget (std::initializer_list<bool> il);
Widget w7{10, 5.0}; // error: requires narrowing conversions, 10 and 5.0 should narroed converted to bool
```

When using an empty set of braces to construct an object that support both default construction and std::initializer_list construction, the empty set of braces means default contruction:
```C++
class Widget {
public:
    Widget();
    Widget(std::initializer_list<int> il);
};

Widget w1;      // default contruction
Widget w2();    // most vexing parse! declares a function
Widget w3{};    // -- defaul construction
Widget w4({});  // std::initializer_list ctor with empty list
Widget w5{{}};  // ditto
```

# 08: Prefer nullptr to 0 and NULL
The fact that template type deduction deduce the wrong types for 0 and NULL (i.e, their true types, rather than their fallback meaning a representation for a nulll pointer) is the most compelling reason to use nullptr.

# 09: Prefer alias declarations to typedefs
Typedefs don’t support templatization, but alias declarations do.

Alias templates avoid the “::type” suffix and, in templates, the “typename” prefix often required to refer to typedefs.

C++14 offers alias templates for all the C++11 type traits transformations.

# 10: Prefer scoped enums to unscoped enums
C++98-style enums are now known as unscoped enums.

Enumerators of scoped enums are visible only within the enum. They convert to other types only with a cast.

Both scoped and unscoped enums support specification of the underlying type. The default underlying type for scoped enums is int. Unscoped enums have no default underlying type.

Scoped enums may always be forward-declared. Unscoped enums may be forward-declared only if their declaration specifies an underlying type.

# 11: Prefer deleted functions to private undefined ones
Any function may be deleted, including non-member functions and template instantiations

# 12: Declare overriding functions override
Declaring derived class overrides is important to get right, but easy to get wrong. So, c++11 introduce: override

Overriding function requirements:
1. The base class function must be virtual
2. The name of base and derived class must be identical
3. The parameter types of base and derived functions must be identical
4. The constness of the base and derived functions must be identical
5. The return type and exceptions specification of base and derived function must be compatible
6. The reference qualifiers must be identical

```C++
class Widget {
public:
    void doWork() &; 	// this version applies only when *this is lvalue
    void doWork() &&; 	// this version applies only when *this is rvalue

    using DateType = std::vector<double>;

    DateType &data() & {
        return values;
    }

    DateType data() && {
        return std::move(values);
    }

private:
    DateType values;
};

Widget w;
auto val1 = w.date();               // copy-construct val1
auto val2 = makeWidget().data();    // move-construct val2
```

Member function reference qualifiers make it possible to treat lvalue and rvalue object(*this) differently

# 13: Prefer const_iterator to iterator

# 14: Declare functions noexcept if they won't emit exceptions
With c++11 noexcept specification, at runtime the stack is only possible unwound before program execution is terminated

So optimizers need not keep the runtime statck in an unwindable state if an exception would propagate out of the function, nor must they ensure that object in an noexcept function are destroyed in the inverse order of construction should an exception leave the function.

noexcept is particularly valuable for the `move operations`, `swap`, memory `deallocation` functions, and `destructors`.

Most functions are exception-neutral rather than noexcept.

# 15: Use constexpr whenever possible
constexpr functions:
1. Can be used in contexts that demand compile-time constants. If any of the arugments's values is not known during compilation, code will rejected.
2. When called with one or more values that are not known during compilation, it acts like a normal function, computing the result at runtime.

constexpr objects are const and are initialized with values known during compilaiton. But consts are not constexpr

constexpr functions can produce compile-time results when called with arguments whose values are known during compilation

constexpr objects and functions may be uesed in a wide range of contexts than non-constexpr objects and functions

constexpr is part of an objects's or function's interface

# 16: Make const member functions thread safe

Make const member functions thread safe unless you're certain they'll never be used in a concurrent context

Use of std::atomic variable may offer better performance than a mutex, but they're suited for manipulation of only a single variable or memory location.

# 17: Understand special member functions generation

Generated special member functions are implicitly public, inline and nonvirtual unless the function in question is a destructor in a derived class inheriting from a base class with a virtual destructor

Move operations peform "memberwise moves" on the non-static data members of the class and move operate on their base class

The two copy operations are independent, when you just declare one but not the other, compiler will generate the other if needed.

But move operations are not independent. when you declate either, that prevent from generating the other.

Declaring a move operation in a class causes compilers to disable the copy operations, and vise versa.

Declaring a destructor has a potentially significant side effect: it prevents the move operations from being generated.

Generation of the copy operations in classess with an explicitly decalared destructor is deprecated.

Move operations are generated only for classes lacking explicitly declared move operations, copy operations and destructor.

Member functions templates never suppress generation of special member functions.


# 18: User std::unique_ptr for exclusive-ownership resources management
Moving a std::unique_ptr transfers the owenership from the source pointer to the destination pointer, the source pointer is set to null, copy is not allowed.

When using the default deleter std::unique_ptr objects are the same size as raw pointers, but custom deletes generally cause the size to grow from on word to two.

For deleters that are function objects, the changes in size depends on how much state is stored in the function objects.

For stateless functions objects(e.g., lambda without captures) incur no size penalty.

# 19: User std::shared_ptr for shared-ownership resource management
The existence of the reference count has performance implications:
1. std::shared_ptr are twice of the size of a raw pointer
2. Memory for reference count must be dynamically allocated
3. Increments and decrements of the reference count must be atomic

std::make_shared is an alternative to std::shared_ptr<T>(new T(args…). The trade-offs are:
1. std::shared_ptr performs at least _two allocations_ while std::make_shared typically performs only one allocation
2. If any std::weak_ptr references the control block created by std::make_shared after the lifetime of all owners ended, the _memory occupied by T persists_ until all weak owners get destroyed as well, which may be undesirable if sizeof(T) is large
3. std::shared_ptr may call a _non-public constructor_ of T if executed in context where it is accessible,
    while std::make_shared requires public access to the selected constructor
4. Unlike the std::shared_ptr constructor, std::make_shared does not allowed a _custom deleter_
5. std::make_shared use _::new_, so if any special behavior has been set up using a class specific operator new, it will differ from std::shared_ptr
6. std:shared_ptr supports _array type_, but std::make_shared does not
7. Code such as f(shared_ptr<int>(new int(42), g()) can cause a _memory leak_ if g gets called after new int(42) and throws an exception, while std::make_shared is safe

Control block:
1. An object's control block is set up by the function creating the first std::shared_ptr to the object
2. std::make_shared always creats a control block, it manufacture a new object to point to
3. A control block is created when a std::shared_ptr is constructed from a unique-ownership pointer
4. When a std::shared_ptr constructor is called with a raw pointer, it creates a contorl block

pitfall:
1. Avoid to creating std::shared_ptr from variable of raw pointer type

# 20: Use std::weak_ptr for std::shared_ptr like pointers that dangle
Potential use cases for std::weak_ptr include caching, observer lists, and the prevention of std::shared_ptr cycles.

# 21: Prefer std::make_unique and std::make_shared to direct use of new
* Advantages of make functions compared to direct use of new:
    1. eliminate source code duplication
    2. improve exception safety
    3. much more efficient(just allocate memory once)

* Limitation of make functins:
    1. Don't permit the specification of custom deleter.
    2. Syntactic detail of implementation.
        Within the make functions, the perfect forwarding code uses parentheses, not braces. (i.e., braced initializer can't be perfect-forwarded.)

        Solution to use initializerlist in make function:
        ```C++
        auto initList = {10, 20, 30};
        autp spv = std::make_shared<std::vector<int>>(initList);
        ```

    3. For std::shared_ptr, make functions may be ill-advised:
        * classes with custom memory management
        * system with memory concern
        * very large object
        * std::weak_ptr that outlive the corresponding std::shared_ptrs

# 22: When using the Pimpl Idiom, define special member functions in the implementation file
The Pimpl Idiom decreases build times by reducing compilation dependencies between class clients and class implementations.

For std::unique_ptr pImpl pointers, declare special member functions in the class header, but implement them in the implementaion file. Do this even if the default functions implementation are acceptable.

The above advice applies to std::unique_ptr, but not to std::shared_ptr.

* The difference in behavior between std::unique_ptr and std::shared_ptr for Pimpl Idiom:
    * The way these smart pointers support custom delter.

* std::unique_ptr:
    * Deleter is part of the smart pointer. Compilers can genereate smaller and faster runtime code.
    * But the pointed-to types must be complete when compiler-generated special functions are used.

* std::shared_ptr:
    * All features are reversed to std::unique_ptr.

# 23: Understand std::move and std::forward
lvalue-reference-to-const(const T&) is permitted to bind to a const rvalue(const T&&).

Move requests on const objects are silently transformed into copy operations.

std::move not only doesn't acutally move anything, it doesn't even guarantee that the object it's casting will be eligible to be moved.

std::move performs an unconditional cast to an rvalue. In and of itself, it doesn't move anything.

std::forward casts its argument to an rvalue only if that arguments is bound to an rvalue.

Neither std::move and std::forward do anything at runtime.

# 24: Distinguish universal reference from rvalue reference
If a function template parameter has type `T&&` for a deduced type T, or if an object is declared using `auto&&`, the parameter or object is a **universal reference**.

If the form of the type declaration isn't precisely type&&, or if type deduction does not occur, type&& denotes an **rvalue reference**.

Universal references correspond to rvalue references if they're initialized with rvalues. They correspond to lvalue reference if they're initialized with lvalues.

# 25: Use std::move on rvalue references, std::forward on universal references
Rvalue references bind only to objects that are candidates for moving.

std::move on an object that doesn't support moving, its copy constructor will be called.

Applying std::move to rvalue references and std::forward to universal references the last time each is used

Do the same thing for rvalue references and universal references being retured from functions that return by value

Never apply std::move or std::forward to local objects if they would otherwise be eligible for the return value optimization

# 26: Avoid overloading on universal references
```C++
class Person {
public:
    template<typename T> explicit Person(T&& n) : name (std::forward<t>(n)) {}

    explicit Person(int idx) : name((nameFromIdx(idx)) {}

    Person(const Person& rhs);  // compiler generated
    Person(Person&& rhs);       // compiler generated

private:
    std::string name;
};

Person p("Nacy"); auto cloneOfP(p); // wont compile, will call templated construtor rather thatn copy constructor
```

Calling to the copy constructor would require adding cosnt to p to match the copy cosntructor's parameter's type, but calling the instantiated template requires no such addition.

* C++ funtions match rule:
    * A template instantiation and a non-template functions are equally good matches for a function call, the normal function is prefered.
    * Overloading on universal reference almost always leads to the universal refernce overload being called more frequently than expected
```C++
class SpecialPerson : public Person {
public:
    SpecialPerson(const SpecialPerson& rhs) : Person(rhs) {}        // copy ctor, call base class forwarding ctor

    SpecialPerson(SpecialPerson&& rhs) : Person(std::move(rhs)) {}  // mvoe ctor, call base calss forwarding ctor
};
```

Perfect-forwarding constructors are especially problematic, because they're typically better matches than copy constructor for non-const lvalues, and they can hijack derived class calls to base class copy and move constructors.

# 27: Familiarize yourself with alternatives to overloading on universal reference
1. tag dispatch
    ```C++
    template<typename T> void logAndAdd(T&& name) {
        logAndAdd(std::forward<T>(name), std::is_integral<std::remove_reference_t<T>());
    }

    template<typename T> void logAndAdd(T&& name, std::false_type) {
        auto now = std::chrono::system_clock::now();
        log(now, "logAndAdd");
        names.emplace(std::forward<T>(name));
    }

    template<typename T> void logAndAdd(int idx, std::true_type) {
        logAndAdd(nameFromIdx(idx));
    }
    ```
2. constrain template function
    ```C++
    class Person {
    public:
        template<
            typename T,
            typename = typename std::enable_if<!std::is_same<Person, typename std::decay<T>::type>::value>::type>
        explicit Person(T&& n);

        template<
            typename T,
            typename = typename std::enable_if<!std::is_base_of<Person, typename std::decay<T>::type>::value>::type>
        explicit Person(T&& n);

        template<
            typename T,
            typename = std::enable_if_t<
                !std::is_base_of<Person, std::decay_t<T>>::value
                && !std::is_integral<std::remove_reference_t<T>>::value
            >>
        explicit Person(T&& n) : name(std::forward<T>(n)) {
            static_assert(std::is_constructible<std::string, T>::value, "can not contruscted from std::string");
        }
    };

    class SpecialPerson: public Person {
    public:
        SpecialPerson(const SpecialPerson& rhs) : Person(rhs) {} // copy ctor; calls
        SpecialPerson(SpecialPerson&& rhs) : Person(std::move(rhs)) {}

    };
    ```

Alternarives to the combination of universal references and overloading include the use of distinct function names, passing parameters by lvalue-reference-to-const, passing parameters by value, and using tag dispatch.

constraining tempaltes via std::enable_if permits the use of universal references and overload together, but it controls the conditions under which compilers may use the universal reference overloads.

Universal reference parameters often have efficiency advantages, but they typically have usability disadvantages.

# 28: Underdstand referece collapsing
* Reference collapsing occurs in four contexts:
    1. template instantiation
    2. auto type generation
    3. creation and use of typedefs and alias declaration
    4. decltype

When compilers generate a reference to a reference in a reference collapsing contexts, the result become a single reference:

if either of the original reference is an lvalue reference, the result is an lvalue reference. Otherwise it's an rvalue reference.

Universal refenrences are rvalue references in contexts where type deduction distinguishes lvalues from rvalues and where reference collapsing occurs.

# 29: Assume that move operations are not present, not cheap, and not used
In code with known types or support for move semantics, there is no need for assumptions.

# 30: Familiarize yourself with perfect forwarding failure cases
In the program's underlaying binary code pointers and references are essentially the same thing.

Perfect forwarding fails when template type deduction fails or when it deduces the wrong type

The kinds of arguments that lead to perfect forwarding failure are braced initializer, null pointers expressed as 0 or NULL, declaration-only intergral const static data members, template and overloaded functions names, and bitfields.

# 31: Avoid default capture modes

The closure type associated with a lambda-expression has a deleted ([dcl.fct.def.delete]) default constructor and a deleted copy assignment operator.

Capure apply only to non-static locak varivables(parameters) visible in the scope where the lambda is created.

The C++ closures do not extend the lifetimes of the captured references.

A by-reference capture causes a closure to contain a refernce to a local variable or to a parameter that's available in the scope where the lambda is defined. If the lifetime of a closure created from the lambda exceeds the lifetime of the local variable or paramter the reference in the closure will dangle.
```C++
class Widget{
    public:
        void addFilter(){
        // what's captured is: this. without explicitly declaration compile will fail, since this can't be implicitly captured
            [=](int value){ return value % divisor == 0; };
        }
    private:
        int divisor;
};
```

Default by-reference capture can lead to dangling reference

Defalt by-value capture is susceptiable to dangling pointers, and it misleadingly suggests that lambdas are self-contained.

By default operator() of the closure type is const, and you cannot modify captured variables inside the body of
the lambda.
```C++
auto foo = [x, y]() mutable { ++x; ++y; };
```

Capturing Globals / Capturing Statics
```C++
int global = 2;
// [=] can caputure varaibles who has automatic storage duration
// warning: capture of variable 'global' with non-automatic storage duration
[=]() { cout << global << endl;}();
```

# 32: Use init capture to move objects into closures

The scope on the left is different from the scope on the right:
1. left: scope of the closure class
2. right: scope the same as where the lambda is being defined

std::bind objects contains copies of all the arguments passed to std::bind.

For lvalue argument, the corresponding object in the bind object is copy constructed.

While for rvalue arguemnt is move constructed.

```C++
auto func = [pw = std::move(widgets)] {
  return pw->isValidated() && pw->isArchived();
}; // C++14 init capture
auto func = std::bind([](const std::vector<Widget>& widgets){ ... }, std::move(widgets));// C++11 emulation of move capture
```

Use C++14's init capture to move objects into closures.

In C++11 emulate init capture via band-written class or std::bind

# 33: User decltype on auto&& parameters to std::forward them
C++14 support generic lambds: lambds that use auto in their paramter specifications

# 34: Prefer lambds to std::bind
```C++
void setAlarm(Time t, Sound s, Duration d);
auto setSoundL = [](Sound s) {
  setAlarm(steady_clock::now() + hours(1), s, seconds(30));
};
auto setSoundB = std::bind(setAlarm, steady_clock::now() +1h, _1, 30s);
```

Problem with std::bind:
1. The first parameter is evaluated when std::bind is called rather than setAlarm is call which causes wrong timer time
2. When there are overloaded version of setAlarm, std::bind doesn't know call which one, the solution is define setAlarm as function pointer
3. normal function may be optimized by compiler as inline function, std::bind call function pointer doesn't be optimzied
4. It's unclare whether std::bind store arguments by reference or by value, but it's explicit for lambda

Lambdas are more readable, more expressive, and may be more efficient than using std::bind

In C++11 only, std::bind may be useful for implementing move capture or for binding objects with templatized functions call

# 35: Prefer task-based programming to thread-based
Three Type Threads:
1. Hardware threads. one or more hardware threads per CPU core.
2. Sofeware threads. are limited resource, if exceed the capability of the system, std::system_error is thrown
3. std::threads. Ojbects in a C++ process that act as handles to underlying software threads.

The std::thread API offers no direct way to get return values from systemchronously run functions, and if thos functions thrown the program is terminated

Thread-based programming calls for manual management of thread exhustion, oversubsrciption, load balancing, and adaption to new platforms.

Taks-based programming via std::aysnc with the default launch policy handles most of these issues for you

# 36: Specify std::launch::async if asynchronicity is essential

It's not possible to predict whether f will run concurrently with current thread t, because f might be scheduled to run deffered

It's not possible to predict whether f runs on a thread different from the thread invoking get or wait on futureT.

It's not possible to predict whether whether f runs at all, because it may not be possible to guarantee that get or wait will be called on futureT along every path throgh the program

The default launch policy for std::async permits both asynchronous and synchronous task execution

This flexibility leads to uncertainty when accessing thread_locals, implies that the task may never execute, and affects program logic for timeout-based wait calls

Specify std::launch::async if asynchronous task execution is essential

# 37: Make std::threads unjoinable on all paths

* The desctructor for a joinable thread is invoked, execution of the program is terminated. The reason for this is beacause two other obvious options are argubly worse:
    1. An implicit join which could lead to performance anomalies.
    2. An implicit detach which invoking join or detach on an unjoinable thread yields undefined behavior.

* Joinable Thread:
    1. A joinable std::thread corresponds to an underlying asynchronous thread of execution that is or could be runing
    2. A std::thread corresponds to an underlying thread that's blocked or waiting to be scheduled is joinalbe.

* Unjoinable Thread:
    1. Default-constructed std::threads
    2. std::thread objects that have been moved from
    3. std::threads that have been joined
    4. std::threads that have been detached

join-on-destrution can lead to difficult-to-debug performance anomalies

detach-on-destruciton can lead to difficult-to-debug undefined behaviour

declare std::thread objects last in lists of data memberss

# 38: Be aware of varying thread handle destructor behavior

* Normal behavior of the destruction of a futute:
    * It just destroys the future object, doesn't join with anything, doesn't detach from anthing, doesn't run anything, and decrements the reference count inside the shared state.

* The exception to this normal behavior arises only for a future for which all of the following apply:
    1. It refers to a shared state that was create due to a call to std::async
    2. The task's launch policy is std::launch::async
    3. The future is the last future referring to the shared state

Only when all of those conditions are fulfilled does a funture's destructor exhibit especial bahavior: block until the asynchronously running task completes.

Future destructors normally just destroy the future's data member.

The final future referring to a shared state of a non-deferred task launched via std::async blokcs until the task complets, while destructors for all other futures simply destroy the future objects.

# 39: Consider void futures for one-shot event communication

* Two problems of condvar-based communication:
    1. Lost notification: The detecting task notifies the condvar before the reacting task waits, the reacting task will hang
    2. Spurious wakeup: Usually happen because, in between the time when the condition variable was signaled and when the waiting thread finally ran, another thread ran and changed the condition. There was a race condition between the threads, with the typical result that sometimes, the thread waking up on the condition variable runs first, winning the race, and sometimes it runs second, losing the race. At a minimum POSIX Threads and the Windows API can be victims of these phenomena. Only condition variables are susceptible to spurious wakeups problem. At a minimum POSIX Threads and the Windows API can be victims of these phenomena.
* The predicate protects against both flaws:
    * Lost wakeup: the thread checks the predicate when it is going to sleep. If it's ture, will not go to sleep.
    * Spurious wakeup: if the predicate is false, continue waiting.

* Condvar-based event commmunication:
    * requires a superfluous mutex, impose constraints on the relative progress of detecting and reacting tasks, and require reacting tasks to verify that the event has taken place.

Flag-based event communication: avoid those problems, but are based on polling, not blocking

condvar-flag-based can be used together, but the resulting communications mechanism is somewhat stilted.

std::promise and futures dodges these issues, but approach uses heap memory for shared states, and it's limited to one-shot communication.

# 40: use std::atomic for concurrency, volatitle for sepcial memory
std::atomic is for data accessed from multiple threads without using mutexes. It's a tool for writing concurrent software.

volatile is for memory where reads and writes should not be optimized. It's a tool for working with special memory.

# 41: Consider pass by value for compable parameters that are cheap to move and always copied

For copyable, cheap-to-move parameters that are always copied, pass by value may be nearly as efficient as pass by reference, it’s easier to implement, and it can generate less object code.

Copying parameters via construction may be significantly more expensive than copying them via assignment.

Pass by value is subject to the slicing problem, so it’s typically inappropriate for base class parameter types.

# 42: Consider emplacement instead of insertion

If all the following are true, emplacement will almost certainly outperform insertion:
1. The value being added is constructed into the container, not assigned
2. The arguement type is being passed differ from the type held by the container
3. The container is unlikely to reject the new value as a duplicate

Emplacement functions may perform type conversions that would be rejected by insertion functions.

Emplacement functions use direct initialization, which means they may use explicit constructors.

Insertion functions employ copy initialization, so they can’t.

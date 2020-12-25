# Structed binding
```C++
struct MyStruct {
    int         i = 0;
    std::string s;
};

auto [u,v] = ms;
// auto e = ms; aliasname u = e.i; aliasname v = e.s; e is a copy of ms
// u and v are alias names for the members of ms
```

1. Qualifier don't Neccesarily Apply to the Structed Bindings
    ```C++
    // a reference, so that u/v refer to ms.i/ms.s, but u/v are not declared as being reference
    const auto& [u,v] = ms;
    // decltype(u) is int, decltype(v) is std::string
    // struct binding do not decay although auto is used
    struct S {
        const char x[6];
        const char y[3]
    };
    ```
2. Move Sematics
    ```C++
    MyStruct ms = { 42, "Jim" };
    auto&& [v, n] = std::move(ms);   // ms still holds its value
    std::string s = std::move(n);   // both n and ms.s have unspecified value

    MyStruct ms = { 42, "Jim" };
    auto [v, n] = std::move(ms);    // ms has unspecified vlaue
    ```
3. Structures and Classes
    * Inheritance: Note that there is only limited usage of inheritance possible. All non-static data members must be members of the same class definition (thus, they have to be direct members of the type or of the same unambiguous public base class)

4. Assigning new Values to Structured Bindings for pair and tuple
    ```C++
    std::tuple<char, float, std::string> getTuple();
    auto [a, b, c] = getTuple(); // a, b, c have types and values of returned tuple
    std::tie(a, b, c) = getTuple();
    ```

# If and switch with Initializers

# Inline variable
* Since C++17 you can define a variable/object in a header file as inline and if this definition is used by multiple translation units, they all refer to the same unique object

# Aggregate Extension
```C++
Data a{}; // value init
Data d;   // default init
```
1. Difference between using empty curly braces and no braces at all:
    1. Definition of d zero-initializes all members so that string name is default constructed, the double value is initialzized by 0.0, and the bool flaf is initialized by false
    2. The definiiton of a only initializes string name by calling the default constructor; all other members are not initiazlied and have a unspecified value

2. You can also derive aggregates form non-aggreagete classes.
    ```C++
    struct MyString : std::string{
        void print() const {
            if (empty()) {
                std::cout << "<undefined>\n";
            } else {
                std::cout << c_str() << std::endl;
            }
        }
    };
    ```

3. You can even derive aggregates from multiple base classes and/or aggregates:
    ```C++
    template<typename T>
    struct D : std::string, std::complex<T> {
        std::string data;
    };

    // which you could then use and initialize as follows:
    D<float> s{{"hello"}, {4.5,6.7}, "world"};  // Ok since C++17
    D<float> t{"hello", {4.5, 6.7}, "world"};   // Ok Since C++17
    std::cout << s.data;                        // << "world"
    std::cout << static_cast<std::string>(s);   // << "hello"
    std::cout << static_cast<std::complex<float>>(s);   << << (4.5, 6.7)
    ```
# Mandatory copy elision or passing unmaterialized objects
1. The copy elision to initialize objects from temporaries is mandatory since C++17, but other optional copy elision still are optional require a callable copy or move constructors

2. C++17 introduces a materialization for the moment a prvalue becomes a temporary object, but it does not mean that creating a new/different objects.
    ```C++
    void f(const X& p); // accepts an expression of any value category, but expects a glvalue
    f(X());             // passes a prvalue materialized as xvalue
    ```

# Lambda extesion
1. Constexpr lambda
    1. Since C++17, lambdas are implicitly constexptr if possible.
    2. Using features(static variable, virtual function, try catch, new delete) that are not allowed in constexpr contexts disalbe this ability, when using at compile-time there will be error, but you can still use the lambda in run-time contexts, in comile-time
    3. For an implicit or explicit constexptr lambda, the function call operator is constexpr

2. Passing copies of this to lambda
    * Since C++17, you can explicitly ask to capture a copy of the current object by capturing *this

# Compile-Time if
1. With the syntax `if constexpr(. . . )`, the compiler uses a compile-time expression to decide at compile time whether to use the then part or the else part (if any) of an if statement. The other part (if any) gets discarded, so that no code gets generated. This does not mean that the discarded part it is completely ignored
    ```C++
    template <typename T>
    std::string asString(T x) {
        if (std::is_same_v<T, std::string>) {
            return x;                   // ERROR, if no conversion to string
        } else if (std::is_numeric_v<T>) {
            return std::to_string(x);   // ERROR, if x is not numeric
        } else {
            return std::string(x);      // ERROR, if no conversion to string
        }
    }
    // This run-time if faliure is a consequence of the rule that function templates usually are either not compiled or compiled as a whole.
    ```

2. Note that a discarded statement is not ignored. The effect is that it doesn’t get instantiated, when depending on template parameters. The syntax must be correct and calls that don’t depend on template parameters must be valid

3. Caveats for Compile-time if
    1. Compile-Time if impacts return type
        ```C++
        auto foo() {                        // return type could be int or void
            if constexpr (sizeof(int) > 4) {
                return 42;
            }
        }
        ```
    2. else matters even if returns
        ```C++
        auto foo() {
            if constexpr (sizeof(int) > 4) {
                return 42;
            }
            /* else can be skiped in rum-time if but not the compile-if in compile-time if,
             * if condition is true, compile deduce two different return type, which is invalid */
            return 42u;
        }
        ```
    3. Short-Circuit Compile-Time Conditions
        ```C++
        template<typename T>
        constexpr auto bar(const T& val) {
            // if constexpr (std::is_integral<T>::value && T{} < 10), error: compile-if doesn't support short-circute
            if constexpr (std::is_integral<T>::value) {
                if (T{} < 10) {
                    return val * 2;
                }
            }
            return val;
        }
        ```
    4. Compile-Time if for Tag Dispatching

# Class template arguments deduction
1. Since C++17, the constraint that you always have to specify the template arguments explicitly was relaxed.
    ```C++
    std::complex c1{1.1, 2.2};  // deduces std::complex<double>
    std::complex c2(2.2, 3.3);  // deduces std::complex<double>
    std::complex c3 = 3.3;      // deduces std::complex<double>
    std::complex c4 = {4.4};    // deduces std::complex<double>
    ```
2. Note that the template parameter has to be unambiguously deducible. Thus, the following initialization doesn’t work:
    ```C++
    std::complex c5{5, 3.3}; // ERROR: attempts to int and double as T
    ```

3. As usual for templates there are no type conversions used to deduce template parameters.

4. Class template argument deduction for variadic templates is also supported.

5. You can also deduce non-type template parameters.
    ```C++
    template<typename T, int SZ>
    class MyClass {
    public:
        MyClass (T(&)[SZ]) {
            ...
        }
    };

    MyClass mc("hello"); // deduces T as const char and SZ as 6
    ```

6. If class template argument deduction could be interpreted as `initializing a copy`, it prefers this interpretation
    ```C++
    std::vector v1{42}; // std::vector<int> with one element
    std::vector v2{v1}; // v2 is also std::vector<int> rather std::vector<std::vector<int>>;
    auto v3 = std::vector{v2};  // v3 is std::vector<int>
    ```

7. Only if multiple elements are passed so that this cannot be interpreted as creating a copy, the elements of the initializer list define the element type of the new vector:
    ```C++
    std::vector vv{v, v}; // vv is vector<vector<int>>
    ```
8. No Partial Class Template Argument Deduction
9. Class Template Argument Deduction Instead of Convenience Functions
    ```C++
    template<typename T1, typename T2>
    struct Pair1 {
        T1 first;
        T2 second;
        Pair1(const T1& x, const T2& y) : first{x}, second{y} { }
    };
    ```
    1. by language rules, when passing arguments of a template type, by reference, the parameter type doesn’t decay
    2. Decay means that raw arrays convert to pointers and top-level qualifiers, such as const and references, are ignored.
        ```C++
        Pair1 p1{"hi", "world"}; // T1 is deduced as char[3], while T2 is deduced as char[6]
        ```
10. Deduction Guid
    ```C++
    template<typename T1, typename T2>
    Pair(T1, T2) -> Pair(T1, T2);
    ```

# std::optional
1. Both `operator*` and `value()` return the contained object by reference, so be careful when calling the these operation directly for temporary values
2. Implicit type conversion for the underlying typ are supported:
    ```C++
    std::optional<int>      o1{42};
    std::optional<double>   o2{42.0};
    o2 = 42;
    o1 = o2;
    ```
3. Move Sematics:
    * a moved-from object still has the same state, but any value became unspecified

# std::variant
1. Initializations and assignment always use the best match to find out the new alternative. If the type doesn’t fit exactly, there might be surprises.
2. Note that empty variants, variants with `reference` members, variants with `C-style array` members, and variants with `incomplete types` (such as void) are not allowed
3. The default constructor initializes the first type with the default constructor, If there is no default constructor defined for the first type, calling the default constructor for the variant is a compile-time error
4. use the `std::in_place_index` tags to resolve ambiguities
    ```C++
    std::variant<int, int> v13{std::in_place_index<1>, 77};
    ```
5. `std::monostate` can serve as a first alternative type to make the variant type default constructible. For example:
    ```C++
    std::variant<std::monostate, NoDefConstr> v2; // OK
    std::cout << "index: " << v2.index() << '\n'; // prints 0
    ```
6. Move sematics
    * As a result, a moved-from object still has the same alternative, but any value becomes unspecified
7. Valueless by exception
    * When modifying a variant so that it gets a new value and this modification throws an exception, the variant can get into a very special state: The variant already lost its old value, but didn’t get its new value
8. Special case:
    * If a std::variant<> has both a `bool` and a `std::string` alternative, assigning string literals can become surprising because a string literal converts better to bool than to std::string.
    ```C++
    std::variant<bool, std::string> v;
    v = "hi"; // OOPS: sets the bool alternative
    ```


# std::any
1. std::optional<> and std::variant<> the resulting objects have value semantics
2. If the object is empty, the type ID is `typeid(void)`
3. `std::any_cast<>` creates an object of the passed type, Without such an initialization, it is usually better to cast to a `reference type` to avoid creating a temporary object
4. values are stored using their `decayed type` (arrays convert to pointers, and top-level references and const are ignored)
5. To hold a different type than the type of the initial value, you have to use the `in_place_type` tags:
    ```C++
    std::any a4{std::in_place_type<long>, 42};
    std::any a5{std::in_place_type<std::string>, "hello"};
    ```
6. move semantics:
    * move semantics is only supported for type that also have copy semantics

# std::byte
1. `list initialization` (using curly braces) is the only way you can directly initialize a single value of a std::byte object
2. There is also no implicit conversion, so that you have to initialize the byte array with an explicitly converted integral literal:
    ```C++
    std::byte b5[] {1};             // ERROR
    std::byte b6[] {std::byte{1}};  // OK
    ```
3. Without any initialization, the value of a std::byte is undefined for object on the stack
4. Because std::byte is define as enumeration type with unsigned char as the underlying type, the size of a std::byte is always 1:
    ```C++
    std::cout << sizeof(b); // always 1
    ```
5. The number of bits depends on the number of bits of type unsigned char
    ```C++
    std::cout << std::numeric_limits<unsigned char>::digits << endl;
    ```

# std::string_view
1. Properties:
    1. read only
    2. not guaranteed to be null terminated byts stream
    3. the value can be the nullptr(default constructor)
    4. there is no allocator support
2. Due to the possible nullptr value and possible missing null terminator, you should always use size() before accessing characters via operator[] or data()
3. String views are in fact more dangerous than string references or smart pointers. They behave more like raw character pointers
    * Do Not Assign Temporary Strings to String Views, it neither copies nor extends the lifetime of a return value
    * Do Not Return String Views to Strings
    * Function Templates Should Use Return Type auto
    * Do Not Use String Views in Call Chains to Initialize Strings
4. Summary of Safe Use of String Views
    1. Do not use string views in APIs that pass the argument to a string
        - Do not initialize string members from string view parameters.
        - No string at the end of a string view chain.
    2. Do not return a string view.
    3. function templates should never return the type T of a passed generic argument.
        * Return auto instead.
    4. Never use a returned value to initialize a string view.
    5. do not assign the return value of a function template that return a generic type to auto


# Substring and Subsequence Searchers

# Parallel STL Algorithms


# 1: Distinguish between pointers and references.

# 2: Prefer C++-style casts.
* static_cast
    * has basically the same power and meaning as the general-purpose C-style cast
* const_cast
    *  is used to cast away the constness or volatileness of an expression
* dynamic_cast
    * is used to perform safe casts down or across an inheritance hierarchy
    * you use dynamic_cast to cast pointers or references to base class objects into pointers or references to derived or sibling base class objects in such a way that you can determine whether the casts succeeded
    * cannot be applied to types lacking virtual functions
* reinterpret_cast
    * is used to perform type conversions whose result is nearly always implementation-defined
    * As a result, reinterpret_casts are rarely portable


# 3: Never treat arrays polymorphically.

# 4: Avoid gratuitous default constructors.

# 5: Be wary of user-defined conversion functions.

# 6: Distinguish between prefix and postfix forms of increment and decrement operators.

# 7: Never overload &&, ||, or ,.

# 8: Understand the different meanings of new and delete.

# 9: Use destructors to prevent resource leaks.

# 10: Prevent resource leaks in constructors.

# 11: Prevent exceptions from leaving destructors.

# 12: Understand how throwing an exception differs from passing a parameter or calling a virtual function.

# 13: Catch exceptions by reference.

# 14: Use exception specifications judiciously.

# 15: Understand the costs of exception handling.

# 16: Remember the 80-20 rule.

# 17: Consider using lazy evaluation.

# 18: Amortize the cost of expected computations.

# 19: Understand the origin of temporary objects.

# 20: Facilitate the return value optimization.

# 21: Overload to avoid implicit type conversions.

# 22: Consider using op= instead of stand-alone op.

# 23: Consider alternative libraries.

# 24: Understand the costs of virtual functions, multiple inheritance, virtual base classes, and RTTI.

# 25: Virtualizing constructors and non-member functions.

# 26: Limiting the number of objects of a class.

# 27: Requiring or prohibiting heap-based objects.

# 28: Smart pointers.

# 29: Reference counting.

# 30: Proxy classes.

# 31: Making functions virtual with respect to more than one object.

# 32: Program in the future tense.

# 33: Make non-leaf classes abstract.

# 34: Understand how to combine C++ and C in the same program.

# 35: Familiarize yourself with the language standard.
# Chapter 1: Container

## 1: Choose your containers with care

## 2: Beware the illusion of container-independent code

## 3: Make copying cheap and correct for objects in containers

## 4: Call empty instead of checking size against zero

## 5: Prefer range member functions to their single-element counterparts

## 6: Be alert for C++'s most vexing parse

## 7: When using containers of newed pointers, remember to delete the pointers before the container is destroyed

## 8: Never create containers of auto_ptrs

## 9: Choose carefully among erasing options

## 10: Be aware of allocator conventions and restrictions

## 11: Understand the legitimate uses of custom allocators

## 12: Have realistic expectations about the thread safety of STL containers

# Chapter 2: vector and string

## 13: Prefer vector and string to dynamically allocated arrays

## 14: Use reserve to avoid unnecessary reallocations

## 15: Be aware of variations in string implementations

## 16: Know how to pass vector and string data to legacy APIs

## 17: Use "the swap trick" to trim excess capacity

## 18: Avoid using vector<bool>

# Chapter 3: Associative Containers

## 19: Understand the difference between equality and equivalence

## 20: Specify comparison types for associative containers of pointers

## 21: Always have comparison functions return false for equal values

## 22: Avoid in-place key modification in set and multiset

## 23: Consider replacing associative containers with sorted vectors

## 24: Choose carefully between map::operator[] and map::insert when efficiency is important

## 25: Familiarize yourself with the nonstandard hashed containers

# Chapter 4: Iterator

## 26: Prefer iterator to const_iterator, reverse_iterator, and const_reverse_iterator

## 27: Use distance and advance to convert a container's const_iterators to iterators

## 28: Understand how to use a reverse_iterator's base iterator

## 29: Consider istream_iterators for character-by-character input

# Chapter 5: Algorithms

## 30: Make sure destination ranges are big enough

## 31: Know your sorting options

## 32: Follow remove-like algorithms by erase if you really want to remove something

## 33: Be wary of remove-like algorithms on containers of pointers

## 34: Note which algorithms expect sorted ranges

## 35: Implement simple case-insensitive string comparisons via mismatch or lexicographical_compare

## 36: Understand the proper implementation of copy_if

## 37: Use accumulate or for_each to summarize ranges

# Chpater 6: Functors, Functor Classes, Functions, etc

## 38: Design functor classes for pass-by-value

## 39: Make predicates pure functions

## 40: Make functor classes adaptable

## 41: Understand the reasons for ptr_fun, mem_fun, and mem_fun_ref

## 42: Make sure less<T> means operator<

# Chapter 7: Programming with STL

## 43: Prefer algorithm calls to hand-written loops

## 44: Prefer member functions to algorithms with the same names

## 45: Distinguish among count, find, binary_search, lower_bound, upper_bound, and equal_range

## 46: Consider function objects instead of functions as algorithm parameters

## 47: Avoid producing write-only code

## 48: Always #include the proper headers

## 49: Learn to decipher STL-related compiler diagnostics

## 50: Familiarize yourself with STL-related web sites
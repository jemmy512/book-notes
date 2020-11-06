# Chapter 1: Container

## Item_1: Choose your containers with care

## Item_2: Beware the illusion of container-independent code

## Item_3: Make copying cheap and correct for objects in containers

## Item_4: Call empty instead of checking sizeO against zero

## Item_5: Prefer range member functions to their single-element counterparts

## Item_6: Be alert for C++'s most vexing parse

## Item_7: When using containers of newed pointers, remember to delete the pointers before the container is destroyed

## Item_8: Never create containers of auto_ptrs

## Item_9: Choose carefully among erasing options

## Item_10: Be aware of allocator conventions and restrictions

## Item_11: Understand the legitimate uses of custom allocators

## Item_12: Have realistic expectations about the thread safety of STL containers

# Chapter 2: vector and string

## Item_13: Prefer vector and string to dynamically allocated arrays

## Item_14: Use reserve to avoid unnecessary reallocations

## Item_15: Be aware of variations in string implementations

## Item_16: Know how to pass vector and string data to legacy APIs

## Item_17: Use "the swap trick" to trim excess capacity

## Item_18: Avoid using vector<bool>

# Chapter 3: Associative Containers

## Item_19: Understand the difference between equality and equivalence

## Item_20: SpecifY comparison types for associative containers of pointers

## Item_21: Always have comparison functions return false for equal values

## Item_22: Avoid in-place key modification in set and multiset

## Item_23: Consider replacing associative containers with sorted vectors

## Item_24: Choose carefully between map::operator[] and map::insert when efficiency is important

## Item_25: Familiarize yourself with the nonstandard hashed containers

# Chapter 4: Iterator

## Item_26: Prefer iterator to consCiterator, reverse_iterator, and const_reverse_iterator

## Item_27: Use distance and advance to convert a container's const_iterators to iterators

## Item_28: Understand how to use a reverse_iterator's base iterator

## Item_29: Consider istreambuUterators for character-by-character input

# Chapter 5: Algorithms

## Item_30: Make sure destination ranges are big enough

## Item_31: Know your sorting options

## Item_32: Follow remove-like algorithms by erase ifyou really want to remove something

## Item_33: Be wary of remove-like algorithms on containers of pointers

## Item_34: Note which algorithms expect sorted ranges

## Item_35: Implement simple case-insensitive string comparisons via mismatch or lexicographical_compare

## Item_36: Understand the proper implementation of copy_if

## Item_37: Use accumulate or for_each to summarize ranges

# Chpater 6: Functors, Functor Classes, Functions, etc

## Item_38: Design functor classes for pass-by-value

## Item_39: Make predicates pure functions

## Item_40: Make functor classes adaptable

## Item_41: Understand the reasons for ptr_fun, mem_fun, and mem_fun_ref

## Item_42: Make sure less<T> means operator<

# Chapter 7: Programming with STL

## Item_43: Prefer algorithm calls to hand-written loops

## Item_44: Prefer member functions to algorithms with the same names

## Item_45: DistingUish among count, find, binary_search, lower_bound, upper_bound, and equal_range

## Item_46: Consider function objects instead of functions as algorithm parameters

## Item_47: Avoid producing write-only code

## Item_48: Always #include the proper headers

## Item_49: Learn to decipher STL-related compiler diagnostics

## Item_50: Familiarize yourself with STL-related web sites
# 5 - Pointers and Arrays

A pointer is a **variable** that contains the address of a variable.

Every pointer points to a specific data type. (There is one exception: a **pointer to void** is used to hold any type of pointer but cannot be dereferenced itself)

## 5.1 Pointers and Addresses

since pointers are variables, they can be used without dereferencing.
```c
iq = ip; // copies the contents of ip into iq, thus making iq point to whatever ip pointed to
```

## 5.2 Pointers and Function Arguments

Since C passes arguments to functions by value, there is no direct way for the called function to alter a variable in the calling function.

## 5.3 Pointers and Arrays

By definition, the value of a variable or expression of type array is the address of element zero of the array. Thus after the assignment
```c
pa = &a[0];
```
pa and a have identical values. Since the name of an array is a synonym for the location of the initial element, the assignment pa=&a[0] can also be written as

```c
pa = a;
```

There is one difference between an array name and a pointer that must be kept in mind. A pointer is a variable, so pa=a and pa++ are legal. But an array name is not a variable; constructions like a=pa and a++ are illegal.

When an array name is passed to a function, what is passed is the location of the initial element.


## 5.4 Address Arithmetic

The header <stddef.h> defines a type ptrdiff_t that is large enough to hold the signed difference of two pointer values.

## 5.5 Character Pointers and Functions
## 5.6 Pointer Arrays; Pointers to Pointers
## 5.7 Multi-dimensional Arrays

If a two-dimensional array is to be passed to a function, the parameter declaration in the function must include the **number of columns**; the number of rows is irrelevant

## 5.8 Initialization of Pointer Arrays
## 5.9 Pointers vs. Multi-dimensional Arrays
## 5.10 Command-line Arguments

The standard requires that argv[argc] be a null pointer.

## 5.11 Pointers to Functions
## 5.12 Complicated Declarations

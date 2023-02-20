# 1 Lexical pitfalls

The part of a compiler that breaks a program up into tokens is often called a **lexical analyzer**.

## 1.1 = is not ==
## 1.2 & and | are not && or ||
## 1.3 Greedy lexical analysis

If the input stream has been parsed into. tokens up to a given character, the next token is taken to include the longest string of charac- ters which could possibly constitute' a token.

```c
a---b; // means the same as a -- - b rather than a - -- b
a+++++b; // error: expression is not assignable
```

Similarly, ifa / is the first character of a token, and the / is immediately followed by *, the two characters begin a comment, regardless of any other context.

y = x/*p; /* begins a comment, so the compiler will simply gobble program text until the */ appears

## 1.4 Integer constants

If the first character of an integer constant is the digit 0, that constant is taken to be in octal.

## 1.5 Strings and characters

A character enclosed in single quotes is just another way of writing the integer. Thus, in an ASCII implementation, 'a' means exactly the same thing as 0141 or 97.

A string enclosed in double quotes, on the other hand, is a short-hand way of writing a pointer to the initial character of a nameless array that has been initialized with the characters between the quotes and an extra character whose binary value is zero.

Thus the statement `printf("Hello world\n");` is equivalent to
```c
char hello[] = { 'H', 'e', 'l', '1', 'o', ' ', 'w', 'o', 'r', 'l', 'd', '\n', '0' };
printf(hel1o);
```

Because an integer is usually large enough to hold several characters, some C compilers permit multiple characters in a character constant as well as a string constant.

This means that writing **'yes'** instead of **"yes"** may well go undetected. The latter means "the address of the first of four consecutive memory locations containing y, e, s, and a null char- acter, respectively." The meaning of 'yes' is not precisely defined, but many C implementations take it to mean "an integer that is composed somehow of the values of the characters y, e, and s." Any similarity between these two quantities is purely coincidental.

```c
int main() {
    int age = 'yes';    // warning: multi-character character constant [-Wmultichar]
    printf("%d\n", age); // 7955827
}
```

# 2 Syntactic pitfalls
## 2.1 Understanding function declarations

If fp is a pointer to a function, **\*fp** is the function itself, so **(*fp)()** is the way to invoke it. ANSI C p~rmits this to be abbreviated as **fp()**

```c
( *(void(*)())0 ) ();

typedef void (*HANDLER)(int);
HANDLER signal(int, HANDLER);
```

## 2.2 Operators don't always have the precedence you want

One way to avoid these problems is to parenthesize everything, but expressions with too many parentheses are hard to understand.

1. Every **logical operator** has lower precedence than every **relational operator**.
2. The **shift operators** bind more tightly than the **relational operators** but less tightly than the **arithmetic operators**.
3. Precedence: arithmetic operators > shift operators > relational operators > logical operator

## 2.3 Watch those semicolons!

One important excep- tion is after an if or while clause, which must be followed by exactly one statement.

```c
// E.g., 1
if (x[i] > big); // error: the body of if is empty
    big = x[i];

// E.g., 2
if (n < 3)
    return  // error: return ogree.date x[O];
logree.date x[O];
logree.time x[1];

// E.g., 3
struct logrec {
    int date;
    int time;
    ipt code;
} // error: miss ';', the main func is defined as: struct logrec {} main() {}
main()
{ }

```

## 2.4 The switch statement

```c
switch (color) {
case 1: printf("red");  // error: missing 'break;'
case 2: printf("yellow");   break;
case 3: printf("blue");     break;
}
```

## 2.5 Calling functions

```c
f(); // is a statement that calls the function
f; // does nothing it evaluates the address of the function but does not call it.
```

## 2.6 The dangling else problem

```c
if (x == 0)
    if (y == 0)
        error();
else { // else is always associated with the closest unmatched if inside the same pair of braces
    z = x + y;
    f(&z);
}
```

# 3 Semantic pitfalls
## 3.1 Pointers and arrays
## 3.2 Pointers are not arrays
## 3.3 Array declarations as parameters
## 3.4 Eschew synecdoche
## 3.5 Null pointers are not null strings
## 3.6 Counting and asymmetric bounds
## 3.7 Order of evaluation
## 3.8 The &&, it, and ! operators
## 3.9 Integer overflow
## 3.10 Returning a value from main

# 4 Linkage
## 4.1 What is a linker?
## 4.2 Declarations vs. definitions:
## 4.3 Name conflicts and the static modifier
## 4.4 Arguments, parameters, and return values
## 4.5 Checking external types
## 4.6 Header files

# 5 Library functions
## 5.1 getchar returns an integer
## 5.2 Updating a sequential file
## 5.3 Buffered output and memory allocation
## 5.4 Using errno for error detection
## 5.5 The signal function

# 6 The preprocessor
## 6.1 Spaces matter in macro definitions
## 6.2 Macros are not functions
## 6.3 Macros are not statements
## 6.4 Macros are not type definitions

# 7 Portability pitfalls
## 7.1 Coping with change
## 7.2 What's in a name?
## 7.3 How big is an integer?
## 7.4 Are characters signed or unsigned?
## 7.5 Shift operators
## 7.6 Memory location zero
## 7.7 How does division truncate?
## 7.8 How big is a random number?
## 7.9 Case conversion
## 7.10 Free first, then reallocate?
## 7.11 An example of portability problems

# 8 Advice and answers
## 8.1 Advice
## 8.2 Answers
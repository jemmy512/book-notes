# 1 Lexical pitfalls

The part of a compiler that breaks a program up into tokens is often called a **lexical analyzer**.

## 1.1 = is not ==
## 1.2 & and | are not && or ||
## 1.3 Greedy lexical analysis

If the input stream has been parsed into. tokens up to a given character, the next token is taken to include the longest string of characters which could possibly constitute' a token.

```c
a---b; // means the same as a -b rather than a -b
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

This means that writing **'yes'** instead of **"yes"** may well go undetected. The latter means "the address of the first of four consecutive memory locations containing y, e, s, and a null character, respectively." The meaning of 'yes' is not precisely defined, but many C implementations take it to mean "an integer that is composed somehow of the values of the characters y, e, and s." Any similarity between these two quantities is purely coincidental.

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

One important exception is after an if or while clause, which must be followed by exactly one statement.

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

Two things stand out about C arrays:
1. C has only one-dimensional arrays, and the size of an array must be fixed as a constant at compilation time.
2. Only two things can be done to an array: determine its size and obtain a pointer to element 0 of the array. Every subscript operation is equivalent to a pointer operation, so it is possible to define the behavior of subscripts entirely ih terms of the behavior of pointers.


This implies that adding an integer to a pointer is generally different from adding that integer to the bit representation of that pointer!

If ip points to an integer, ip + 1 points to the next integer in the machine's memory, which, for most modern computers, is not the next memory location.

If we use the name of an array where a pointer is appropriate, that name is taken to mean a pointer to element 0 of that array.
```c
int a[3];
int* p;

p = a; // we will set p to the address of element 0 of a

// warning: incompatible pointer types initializing 'int *' with an expression of type 'int (*)[3]'
p = &a; // illegal in ANSI C because &a is a pointer to an array but p is a pointer to an into
```

The name a refers to the address of element 0 of a in every context but one: when a is used as an argument to the sizeof operator. There, **sizeof(a)** does what one would reasonably expect: it yields the size of the entire array a and not the size of a pointer to one of its elements!

It is possible to refer to element i of a by writing **\*(a+i)**; this notion is so common that it is abbreviated as **a\[i]**.
```c
a[i] == *(a+i);
i[a] == a[i];
```

Moreover, using only pointers to manipulate two-dimensional arrays leads us irito some of the darker corners of the language, where compiler bugs are likely to lurk.
```c
int caleridar[12][31];
int *p;
int i;

calendar[4]; // is one of the 12 arrays of 31 int elements in calendar and behaves exactly that way
sizeof(calendar[4]); // is 31 times the size of an int

p = caHmdar[4]; // is to make p point to eiement 0 of the array calendar[4]

i = calendar[4][7] == (calendar[4] + 7) == *(*(caleridar+4) + 7);

p = calendar; // warning: incompatible pointer types initializing 'int *' with an expression of type 'int[12][31]'
```

we need a way to declare a pointer to an array. After wading through Chapter 2, it should not be too hard to construct it:
```c
int calendar[12][31];
int (*monthp)[31];

monthp = calendar;

for (int month = 0; month < 12; month++) {
    for (int day = 0; day < 31; day++) {
        calendar[month][day] = 0;
    }
}

for (monthp = calendar; monthp < &calendar[12]; monthp++) {
    // dayp < &monthp[31]; warning: comparison of distinct pointer types ('int *' and 'int (*)[31]')
    // dayp < monthp[31]; ok
    for (int* dayp = *monthp; dayp < &(*monthp[31]); dayp++) { // TODO?? &(**(monthp+31))
        *dayp = 0;
    }
}
```

## 3.2 Pointers are not arrays

A character string constant in C represents the address of an area of memory that holds the characters in the constant, followed by a null character (' \0').

```c
ehar *r, *malloe();
r =malloe(strlen(s) +strlen(t)); strepy(r, s);
strcpy(r, s);
strcat(r, t);
```
This example, however, fails for three reasons
1. malloc might be unable to provide the requested memory, an event that it signals by returning a null pointer.
2. it is important to.remember to free the memory allocated for r when done with it.
3. most importarit, is that the call to malloe doesn't allocate quite enough memory: forget null terminator.
```c
char *r, *malloc();

r = malioe(strlen(s) + strlen(t) + 1);
if (!r) {
    complain();
    exit(1);
}

strepy(r, s);
strcat(r, t);

free(r) ;
```

## 3.3 Array declarations as parameters

C automatically converts an array parameter declaration to the corresponding pointer declaration.
```c
int strlen(char s[]) { // int strlen(char* s)

}
```

`extern char *hello;` is definitely not the same as` extern char hello[];`

If a pointer parameter does not represent an array, it is misleading, although technically correct, to use the array notation. What about pointer parameters that do represent arrays? One common example is the **second argument** to main:
```c
int main(int argc, char *argv[]) // main(int argc, char **argv)
{

}
```


## 3.4 Eschew synecdoche

copying a pointer does not copy the thing it addresses.

## 3.5 Null pointers are not null strings

The result of converting an integer to a pointer is implementatiohdependent, with one important exception: constant 0, which is guaranteed to be converted to a pointer that is unequal to any valid pointer.

The important thing to remember about **0** when used as a pointer is that it must never be **dereferenced**.

Among common programming errors, the hardest to find are usually **fencepost errors**, also called **off-by-one errors**.

These two ways of solving this problem suggest two general principles for avoiding fencepost errors:
1. Extrapolate from a trivial case.
2. Count carefully.
It is the "+1" in h-l+1 that is the source of so many fencepost errors.

might there be some programming technique that makes these errors less likely?
* Express a range by the first element of the range and the first element beyond it.

This asymmetry may look mathematically ugly, l;>utit can simplify programming surprisingly:
1. The size of a range is the difference between the bounds. 38-16 is 22, the number of elements contained between the asymmetric bounds 16 and 38.
2. The bounds are equal when the range is empty. This follows immediately from (1).
3. The upper bound is never less than the lower bound, not even when the range is empty.

## 3.6 Counting and asymmetric bounds

## 3.7 Order of evaluation

Only the four C operators **&&, ||, ?:, and ,** specify an order of evaluation.


y[i] = x[i++];
* The trouble is that there is no guarantee that the address of y[i] willbe evaluated before i is incremented. On some implementations, it will; on others, it won't.

## 3.8 The &&, it, and ! operators

## 3.9 Integer overflow

There is no such thing as overflow in unsigned arithmetic: all unsigned operations are done modulo 2^n, where n is the number of bits in the result.

If one operand of an arithmetic operator is signed and the other unsigned, the signed operand is converted to unsigned and overflow is still impossible.

want to test whether a+b might overflow:
```c
if (a + b < 0)
    complain( );
```
This does not work. Once a+b has overflowed, all bets are off as to what the result will be. For example, on some machines, an addition operation sets an internal register to one of four states: positive, negative, zero, or overflow. On such a machine, the compiler would have every right to implement the example given above by adding a and band' checking whether this internal register was in negative state afterwards. If the operation overflowed, the register would be in overflow' state, and. the test would fail.

One correct way of doing this is to .convert a and b to unsigned:
```c
if ((unsigned) a + (unsigned) b > INT_MAX)
    complain ();

if (a > INT_MAX b)
    complain ();
```

## 3.10 Returning a value from main

An int.function that fails to return a value .usually implicitly returns some garbage integer. As long as no one uses this value, it doesn't matter.


There are some contexts in which the value returned from main does matter. A program that doesn't return any value from main thus probably appears to have failed. This may cause surprising results when used with things like software administration systems that care about whether programs fail after they have invoked them.

# 4 Linkage
## 4.1 What is a linker?

Although linkers don't understand C, they do understand machine language and memory layout, and it is up to each C compiler to translate C programs into terms that make sense to the linker.

## 4.2 Declarations vs. definitions

**int a** appearing outside of any function body is called a definition of the external object a; it says that a is an external integer variable and also allocates storage for it. Because it doesn't specify an initial value, the value is assumed to be 0

**int a=7;** is a definition of a that includes an explicit initial value. Not only does it allocate memory for a, but it says what value that memory should have.

**extern int a;** is not a definition of a. It still says that a ISan external integer variable, but by including the extern ~eyword; it explicitly says that the storage for a is allocated somewhere else.

What about a program that defines the same external variable more than once?
* systems vary, most systems will reject the program.
* But if an external variable is defined in several files without an initial value, some systems will accept the program and others won't.
* The only way to avoid this trouble in all C implementations is to define each e.xternal variable exactly once.

## 4.3 Name conflicts and the static modifier

ANSI C makes it easier to avoid conflicting with library names by listing all the functions that might possibly cause such conflicts. Any library function that calls another library function not on the list must do so by a "hidden name."

One useful tool for reducing conflicts of this sort is the **static** modifier. For example, the declaration static int a;


## 4.4 Arguments, parameters, and return values

Appropriate does not necessarily mean equal: float arguments are automatically converted to double and r;;hort or char arguments are converted to into

## 4.5 Checking external types

The rule in.C ISthat an otherwise undeclared identifier followed by an open parenthesis is assumed to be a function that returns an integer.


## 4.6 Header files

One good way to avoid many problems of this sort is to adopt a simple rule: declare each external object in one place only.

# 5 Library functions
## 5.1 getchar returns an integer
## 5.2 Updating a sequential file

```c
FILE *fp;
struct record reC;

while (fread((char *) &rec, sizeof(rec), 1, fp) == 1) {
    // do something to rec
    if (rec must be rewritten) {
        fseek(fp, -(long)sizeof(rec), 1);
        fwrite((char*)&rec, sizeof(rec), 1, fp);
        fseek(fp, 0L, 1);
    }
}
```

```c
#include <stdio.h>

int c;
char buf[BUFSIZ];
setbuf(stdout, buf);

while ((c = getchar()) != EOF) {
    putchar(c);
}
```

Ask: when buf is flushed for the last time.
* Answer: after the main program has finished, as part of the cleaning up that the library does before handing control back to the operating system. But by that time, buf has already been freed!
```c
#include <stdio.h>

int c;
static char buf[BUFSIZ];
setbuf(stdout, buf); // or setbuf(stdout, malloc(BUFSIZ)j

while ((c = getchar()) != EOF) {
    putchar(c);
}
```

## 5.3 Buffered output and memory allocation

## 5.4 Using errno for error detection

A library routine that sets errno on error is under no obligation to clear it in the absence of an error.

```c
// E.g., 1 error
// call library function
if (errno)
    complain

// E.g., 2 error
errno = O;
// call library function
if (errno)
    complain

// E.g., 3 ok
// call library routine
if (error return)
    examine errno
```

## 5.5 The signal function

Signals are truly asynchronous in many implementations. A signal can occur at literally any point during the execution of a C program. Thus it is not safe for a signal handler function to call any such library function.

For similar reasons, it is generally unsafe to exit from a signal handler by using longjmp: the signal may have occurred while malloe or some other library routine had started updating some data structure but not finished it.

Thus it appears that the only safe thing for a signal handler to do is to set a flag and return, with the assumption that the main program will test that flag later and discover that a signal has occurred.

Thus the only portable, reasonably safe thing a signal handler for an arithmetic error can do is to print a message and exit (by using either longjmp or exit).

The conclusion to draw from this is that signals can be tricky and intrinsically have non portable aspects. The best defense against problems is to keep signal handlers as simple as possible and group them all together.


# 6 The preprocessor

The preprocessor gives us ways to abbreviate things that are important for two major reasons (and several minor ones).
1. we may want to be able to change all instances of a particular quantity, such as the size of a table, by changing one number and recompiling the program. Moreover, by using the preprocessor it is easy to collect the definitions of these constants together to make them easy to find.
2. most C implementations impose a significant overhead for each function call.

## 6.1 Spaces matter in macro definitions

```c
#define f (X) ((x)-1) => f = (x) ((x)-1)
#define f(x) ((x)-1) => f(x) = ((x)-1)
```

Even if macro definitions are fully parenthesized, though, an operand that is used twice may be evaluated twice.
```c
#define max(a,b) ((a) > (b)? (a) : (b))

max(a ,b);
// if a is greater than b, a will be evaluated twice:
// once during the comparison
// and again to calculate the value max yields.

// Not only can this be inefficient, it can also be wrong:
max(biggest, x[i++]);
```

One way around these worries is to ensure that the arguments to the max macro have no side effects: max(biggest, x[i]);

Another is to make max a function, or to do the computation by hand:.

Another hazard of using macros is that they may generate very large expressions indeed, consuming more space than their user had intended.

## 6.2 Macros are not functions

```c
#define assert(e) if (!e) assert_error( __FILE__, __LINE__)

// the assert fails in this context
if (x > 0 && y > 0)
    assert (x > y);
else
    assert (y > x);

// it becames:
if (x >0 && y >0)
    if (!(x > y))
        assert_error("foo.c", 37);
else
    if (!(y > x))
        assert_error("foo.c", 39);

// it becames:
if (x >0 && y >0)
    if (!(x > y))
        assert_error("foo.c", 37);
    else
        if (!(y > x))
            assert_error("foo.c", 39);
```

It is possible to avoid this problem by enclosing the body of the assert macro in braces:
```c
#define assert(e) \
    { if (Ie) assert_error( __FILE __ , __LINE __ ); }

// This raises a new problem. Our example now expands into:
if (x > 0 && Y > 0)
    { if (I(x > y)) assert_error("foo.c", 37); }; // semicolon before the else is a syntax error.
else
    { if (I(y > x)) assert_error("foo.c", 39); };
```

The right way to define assert is far from obvious: make the body of assert look like an expression and not a statement:
```c
#define assert(e) \
    ((void)((e) || _assert_error(__FILE __ , __LINE __)))
```

## 6.3 Macros are not statements


## 6.4 Macros are not type definitions

```c
#define T1 struct foo *
typedef struct foo *T2;

T1 a, b; // error: only a is type of struct foo *
T2 c, d; // ok: both c and d are type of struct foo *
```

# 7 Portability pitfalls

one of the reasons to write programs in C in the first place is that it is easy to move them from one programming environment to another.

## 7.1 Coping with change
## 7.2 What's in a name?
## 7.3 How big is an integer?

The language definition guarantees a few things about the relative sizes of the various kinds of integer:
1. The three sizes of integers are nondecreasing.
2. An ordinary integer is large enough to contain any array subscript.
3. The size of a character is natural for the particular hardware.

## 7.4 Are characters signed or unsigned?

But a compiler converting a char to an int has a choice: should it treat the char as a signed or an unsigned quantity?
* If the former, it should expand the char to an int by replicat- ing the sign bit;
* if the latter, it should fill the extra bit positions with zeroes.

If you care whether a character value with the high-order bit on is treated as a negative number, you should probably declare it as **unsigned char**. Such values are guaranteed to be zero-extended when converted to integer, whereas ordinary char variables may be signed in one implementation and unsigned in another.

## 7.5 Shift operators

Two questions seem to cause trouble for people who use shift operators:
1. In a right shift, are vacated bits filled with zeroes or copies of the sign bit?

    If the item being shifted is unsigned, zeroes are shifted in. If the item is signed, the implementation is permit- ted to fill vacated bit positions either with zeroes or with copies of the sign bit.

    If you care about vacated bits in a right shift, declare the vari- able in question as unsigned.

2. What values are permitted for the shift count?

    if the item being shifted is n bits long, then the shift count must be greater than or equal to zero and strictly less than n

Note that a right shift of a signed integer is generally not equivalent to division by a power of two, even if the implementation copies the sign into vacated bits. E.g., (-1) >> 1 is not (-1)/2

## 7.6 Memory location zero

A null pointer does not point to any object. Thus it is illegal to use a null pointer for any purposes other than **assignment and comparison**.

## 7.7 How does division truncate?
q = a/b; r = a % b;

For the moment, suppose also that b>O.

What relationships might we want to hold between a, b, p, and q?
1. Most important, we want q*b + r == a, because this is the relation that defines the remainder.
2. If we change the sign of a, we want that to change the sign of q, but not the magnitude.
3. When b>O, we want to ensure that r>=O and r<b. For instance, if the remainder is being used as an index to a hash table, it is important to be able to know that it will always be a valid index.

Thus C, and any language that implements truncating integer divi- sion, must give up at least one of these three principles. Most program- ming languages give up number 3.

## 7.8 How big is a random number?
## 7.9 Case conversion

```c
#define toupper(c) ((c)+'A'-'a')
#define tolower(c) ((c)+'a'-'A')
```

these macros depend on the implementation's character set to the extent that they demand that the difference between an upper-case letter and the corresponding lower-case letter be the same constant for all letters.

These macros do have one disadvantage, though: when given some- thing that is not a letter of the appropriate case, they return garbage.

AT&T software development noticed:
```c
#define toupper(c) ((c) >= 'a' && (c) <= 'z' ? (c) + 'A' - 'a' : (c))
#define tolower(c) ((c) >= 'A' && (c) <= 'Z' ? (c) + 'a' - 'A' : (c))
```

this would cause c to be evaluated anywhere between one and three tiines for each call, which would play havoc with expres- sions like toupper (*p++ ). Instead, he decided to rewrite toupper and tolower as functions
```c
int toupper(int c) {
    if (c >= 'a' && c <= 'z')
        return c + 'A' - 'a';

    return c;
}
```

## 7.10 Free first, then reallocate?

```c
void printnum(long n, void (*p)()) {
    if (n < 0) {
        (*p)('-');
        n = -n; // assignment might overflow
    }

    if (n >= 10)
        printnum(n/10, p);
    (*p)((int)(n % 10) + '0'); // (*p)("0123456789"[n % 10]);
}
```
1. This assignment might overflow, because 2's complement machines generally allow more negative values than positive values to be represented. E.g., -2^32 <= n < 2^32
2. This addition assumes that the machine collating sequence has all the digits in sequence with no gaps, so that ' 0' +5 has the same value as ' 5', and so on.

```c
void printneg(long n, void (*p)()) {
    long q;
    int r;
    
    q = n/ 10;
    r = n % 10;
    if (r > 0) {
        r -= 10;
        q++;
    }

    if (n <= -10)
        printneg(q, p);

    (*p)("0123456789"[-r]);
}
```

## 7.11 An example of portability problems

# 8 Advice and answers
## 8.1 Advice
## 8.2 Answers
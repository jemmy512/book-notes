# Chapter 2 Meaningful Names
1. Use Intention-Revealing Names
2. Avoid Disinformation
3. Make Meaningful Distinction
4. Use Pronounceable Names
5. Use Searchable Names
6. Avoid Encodings
7. Avoid Mental Mapping
8. Class Names: noun or noun phrase
9. Method Names: verb or verb phrase
10. Don't be cute
11. Pick One Word per Concept
12. Don't Pun
13. Use Solution Domain Names
14. Use Problem Domain Names
15. Add Meaningful Context
16. Don't Add Gratuitos Context

# Chapter 3 Functions
1. Small
2. Do One Thing
3. One Level of Abstraction per Function
    > Reading Code from Top to Bottom: The Stepdown Rule
4. Switch Statements
5. Use Descriptive Names
6. Function Arguments
    * Common Monadic Forms
    * Dyadic Functions
    * Triads
    * Argument Objects
    * Argument Lists
    * Verbs and Keywords
7. Have No Side Effects
8. Command Query Seperation
9. Prefer Exceptions to Returning Error Codes
    * Extract Try/Catch Blocks
    * Error Handling Is One Thing
10. Don't Repeat Yourself
11. Structured Programming

# Chapter 6 Objects and Data Structures
## Data Abstraction
## Data/Object Anti-Symmetry
Objects hide their data behind abstractions and expose functions that operate on that data.Data structure expose their data and have no meaningful functions.

Procedural code makes it hard to add new data structures because all the functions must change. OO code makes it hard to add new functions because all the classes must change.

## The Law of Demeter
`Law of Demeter` that says a module should not know about the innards of the objects it manipulates
### Train Wrecks
### Hybrids
### Hiding Structure

## Data Transfer Objects
The quintessential form of a data structure is a class with public variables and no functions

# Chapter 7 Error Handling
## Use Exceptions Rather Than Return Codes

# 1 Refactoring: A First Example

## The Starting Point
A poorly designed system is hard to change—because it is difficult to figure out what to change and how these changes will interact with the existing code to get the behavior I want. And if it is hard to figure out what to change, there is a good chance that I will make mistakes and introduce bugs.

## Comments on the Starting Program
When you have to add a feature to a program but the code is not structured in a convenient way, first refactor the program to make it easy to add the feature, then add the feature.

Making a copy may not seem too onerous a task, but it sets up all sorts of problems for the future. Any changes to the charging logic would force me to update both methods—and to ensure they are updated consistently.

## The First Step in Refactoring
Before you start refactoring, make sure you have a solid suite of tests. These tests must be self­checking.

## Decomposing the statement Function
When refactoring a long function like this, I mentally try to identify points that separate different parts of the overall behavior.

First, I need to look in the fragment for any variables that will no longer be in scope once I’ve extracted the code into its own function.

Once I’ve used Extract Function (106), I take a look at what I’ve extracted to see if there are any quick and easy things I can do to clarify the extracted function. The first thing I do is rename some of the variables to make them clearer, such as changing thisAmount to result.

When I’m breaking down a long function, I like to get rid of variables like play, because temporary variables create a lot of locally scoped names that complicate extractions. The refactoring I will use here is Replace Temp with Query (178).

Temporary variables can be a problem. They are only useful within their own routine, and therefore they encourage long, complex routines.

:bulb: Refactoring changes the programs in small steps, so if you make a mistake, it is easy to find where the bug is.

:bulb: Any fool can write code that a computer can understand. Good programmers write code that humans can understand.

The great benefit of removing local variables is that it makes it much easier to do extractions, since there is less local scope to deal with.

So, my overall advice on performance with refactoring is: Most of the time you should ignore it. If your refactoring introduces performance slow-downs, finish refactoring first and do performance tuning afterwards.

* The second aspect I want to call your attention to is how small the steps were to remove volumeCredits:
    1. Split Loop (227) to isolate the accumulation
    2. Slide Statements (223) to bring the initializing code next to the accumulation
    3. Extract Function (106) to create a function for calculating the total
    4. Inline Variable (123) to remove the variable completely

In particular, should a test fail during a refactoring, if I can’t immediately see and fix the problem, I’ll revert to my last good commit and redo what I just did with smaller steps.

## Status: Lots of Nested Functions
```js
function statement (invoice, plays) {
  let totalAmount = 0;
  let volumeCredits = 0;
  let result = `Statement for ${invoice.customer}\n`;
  const format = new Intl.NumberFormat("en-US",
    { style: "currency", currency: "USD", minimumFractionDigits: 2 }).format;

  for (let perf of invoice.performances) {
    const play = plays[perf.playID];
    let thisAmount = 0;

    switch (play.type) {
    case "tragedy":
      thisAmount = 40000;
      if (perf.audience > 30) {
        thisAmount += 1000 * (perf.audience - 30);
      }
      break;
    case "comedy":
      thisAmount = 30000;
      if (perf.audience > 20) {
        thisAmount += 10000 + 500 * (perf.audience - 20);
      }
      thisAmount += 300 * perf.audience;
      break;
    default:
        throw new Error(`unknown type: ${play.type}`);
    }

    // add volume credits
    volumeCredits += Math.max(perf.audience - 30, 0);
    // add extra credit for every ten comedy attendees
    if ("comedy" === play.type) volumeCredits += Math.floor(perf.audience / 5);

    // print line for this order
    result += `  ${play.name}: ${format(thisAmount/100)} (${perf.audience} seats)\n`;
    totalAmount += thisAmount;
  }
  result += `Amount owed is ${format(totalAmount/100)}\n`;
  result += `You earned ${volumeCredits} credits\n`;
  return result;
}
```

```js
function statement (invoice, plays) {
  let result = `Statement for ${invoice.customer}\n`;
  for (let perf of invoice.performances) {
    result += `  ${playFor(perf).name}: ${usd(amountFor(perf))} (${perf.audience} seats)\n`;
  }
  result += `Amount owed is ${usd(totalAmount())}\n`;
  result += `You earned ${totalVolumeCredits()} credits\n`;
  return result;
}

function totalAmount() {
  let result = 0;
  for (let perf of invoice.performances) {
      result += amountFor(perf);
  }
  return result;
}

function totalVolumeCredits() {
  let result = 0;
  for (let perf of invoice.performances) {
      result += volumeCreditsFor(perf);
  }
  return result;
}
function usd(aNumber) {
  return new Intl.NumberFormat("en-US",
    { style: "currency", currency: "USD", minimumFractionDigits: 2 }).format(aNumber/100);
}

function volumeCreditsFor(aPerformance) {
  let result = 0;
  result += Math.max(aPerformance.audience - 30, 0);
  if ("comedy" === playFor(aPerformance).type) result += Math.floor(aPerformance.audience / 5);
  return result;
  }
  function playFor(aPerformance) {
  return plays[aPerformance.playID];
}

function amountFor(aPerformance) {
  let result = 0;
  switch (playFor(aPerformance).type) {
  case "tragedy":
      result = 40000;
      if (aPerformance.audience > 30) {
      result += 1000 * (aPerformance.audience - 30);
      }
      break;
  case "comedy":
      result = 30000;
      if (aPerformance.audience > 20) {
      result += 10000 + 500 * (aPerformance.audience - 20);
      }
      result += 300 * aPerformance.audience;
      break;
  default:
      throw new Error(`unknown type: ${playFor(aPerformance).type}`);
  }
  return result;
}
```

## Splitting the Phases of Calculation and Formatting
So far, my refactoring has focused on adding enough structure to the function so that I can understand it and see it in terms of its logical parts. This is often the case early in refactoring. Breaking down complicated chunks into small pieces is important, as is naming things well.

## Status: Separated into Two Files (and Phases)
## Reorganizing the Calculations by Type
## Status: Creating the Data with the Polymorphic Calculator
## Final Thoughts


# 2 Principles in Refactoring
## The Two Hats
## Why Should We Refactor?
## When Should We Refactor?
## Problems with Refactoring
## Refactoring, Architecture, and Yagni
## Refactoring and the Wider Software Development Process
## Refactoring and Performance
## Where Did Refactoring Come From?
## Automated Refactorings
## Going Further


# 3 Bad Smells in Code
## Mysterious Name
## Duplicated Code
## Long Function
## Long Parameter List
## Global Data
## Mutable Data
## Divergent Change
## Shotgun Surgery
## Feature Envy
## Data Clumps
## Primitive Obsession
## Repeated Switches
## Loops
## Lazy Element
## Speculative Generality
## Temporary Field
## Message Chains
## Middle Man
## Insider Trading
## Large Class
## Alternative Classes with Different Interfaces
## Data Class
## Refused Bequest
## Comments


# 4 Building Tests
## The Value of Self-Testing Code
## Sample Code to Test
## A First Test
## Add Another Test
## Modifying the Fixture
## Probing the Boundaries
## Much More Than This


# 5 Introducing the Catalog
## Format of the Refactorings
## The Choice of Refactorings


# 6 A First Set of Refactorings
## Extract Function
## Inline Function
## Extract Variable
## Inline Variable
## Change Function Declaration
## Encapsulate Variable
## Rename Variable
## Introduce Parameter Object
## Combine Functions into Class
## Combine Functions into Transform
## Split Phase

# 7 Encapsulation
## Encapsulate Record
## Encapsulate Collection
## Replace Primitive with Object
## Replace Temp with Query
## Extract Class
## Inline Class
## Hide Delegate
## Remove Middle Man
## Substitute Algorithm

# 8 Moving Features
## Move Function
## Move Field
## Move Statements into Function
## Move Statements to Callers
## Replace Inline Code with Function Call
## Slide Statements
## Split Loop
## Replace Loop with Pipeline
## Remove Dead Code


# 9 Organizing Data
## Split Variable
## Rename Field
## Replace Derived Variable with Query
## Change Reference to Value
## Change Value to Reference


# 10 Simplifying Conditional Logic
## Decompose Conditional
## Consolidate Conditional Expression
## Replace Nested Conditional with Guard Clauses
## Replace Conditional with Polymorphism
## Introduce Special Case
## Introduce Assertion


# 11 Refactoring APIs
## Separate Query from Modifier
## Parameterize Function
## Remove Flag Argument
## Preserve Whole Object
## Replace Parameter with Query
## Replace Query with Parameter
## Remove Setting Method
## Replace Constructor with Factory Function
## Replace Function with Command
## Replace Command with Function


# 12 Dealing with Inheritance
## Pull Up Method
## Pull Up Field
## Pull Up Constructor Body
## Push Down Method
## Push Down Field
## Replace Type Code with Subclasses
## Remove Subclass
## Extract Superclass
## Collapse Hierarchy
## Replace Subclass with Delegate
## Replace Superclass with Delegate
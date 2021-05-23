# 1 Refactoring: A First Example

## The Starting Point
```json
{
  "hamlet": {"name": "Hamlet", "type": "tragedy"},
  "as-like": {"name": "As You Like It", "type": "comedy"},
  "othello": {"name": "Othello", "type": "tragedy"}
}
```

```json
[
  {
    "customer": "BigCo",
    "performances": [
      {
        "playID": "hamlet",
        "audience": 55
      },
      {
        "playID": "as-like",
        "audience": 35
      },
      {
        "playID": "othello",
        "audience": 40
      }
    ]
  }
]
```

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

A poorly designed system is hard to change—because it is difficult to figure out what to change and how these changes will interact with the existing code to get the behavior I want. And if it is hard to figure out what to change, there is a good chance that I will make mistakes and introduce bugs.

## Comments on the Starting Program
When you have to add a feature to a program but the code is not structured in a convenient way, first refactor the program to make it easy to add the feature, then add the feature.

Making a copy may not seem too onerous a task, but it sets up all sorts of problems for the future. Any changes to the charging logic would force me to update both methods—and to ensure they are updated consistently.

## The First Step in Refactoring
Before you start refactoring, make sure you have a solid suite of tests. These tests must be self­ checking.

## Decomposing the statement Function
When refactoring a long function like this, I mentally try to identify points that separate different parts of the overall behavior.

First, I need to look in the fragment for any variables that will no longer be in scope once I’ve extracted the code into its own function.

Once I’ve used `Extract Function`, I take a look at what I’ve extracted to see if there are any quick and easy things I can do to clarify the extracted function. The first thing I do is rename some of the variables to make them clearer, such as changing thisAmount to result.

When I’m breaking down a long function, I like to get rid of variables like play, because temporary variables create a lot of locally scoped names that complicate extractions. The refactoring I will use here is `Replace Temp with Query`.

Temporary variables can be a problem. They are only useful within their own routine, and therefore they encourage long, complex routines.

:bulb: Refactoring changes the programs in small steps, so if you make a mistake, it is easy to find where the bug is.

:bulb: Any fool can write code that a computer can understand. Good programmers write code that humans can understand.

The great benefit of removing local variables is that it makes it much easier to do extractions, since there is less local scope to deal with.

So, my overall advice on performance with refactoring is: Most of the time you should ignore it. If your refactoring introduces performance slow-downs, finish refactoring first and do performance tuning afterwards.

* The second aspect I want to call your attention to is how small the steps were to remove volumeCredits:
    1. `Split Loop` to isolate the accumulation
    2. `Slide Statements` to bring the initializing code next to the accumulation
    3. `Extract Function` to create a function for calculating the total
    4. `Inline Variable` to remove the variable completely

In particular, should a test fail during a refactoring, if I can’t immediately see and fix the problem, I’ll revert to my last good commit and redo what I just did with smaller steps.

## Status: Lots of Nested Functions
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

```js
import createStatementData from './createStatementData.js';

function statement (invoice, plays) {
  return renderPlainText(createStatementData(invoice, plays));
}

function renderPlainText(data, plays) {
  let result = `Statement for ${data.customer}\n`;
  for (let perf of data.performances) {
    result += `  ${perf.play.name}: ${usd(perf.amount)} (${perf.audience} seats)\n`;
  }
  result += `Amount owed is ${usd(data.totalAmount)}\n`;
  result += `You earned ${data.totalVolumeCredits} credits\n`;
  return result;
}

function htmlStatement (invoice, plays) {
  return renderHtml(createStatementData(invoice, plays));
}

function renderHtml (data) {
  let result = `<h1>Statement for ${data.customer}</h1>\n`;
  result += "<table>\n";
  result += "<tr><th>play</th><th>seats</th><th>cost</th></tr>";
  for (let perf of data.performances) {
    result += `  <tr><td>${perf.play.name}</td><td>${perf.audience}</td>`;
    result += `<td>${usd(perf.amount)}</td></tr>\n`;
  }
  result += "</table>\n";
  result += `<p>Amount owed is <em>${usd(data.totalAmount)}</em></p>\n`;
  result += `<p>You earned <em>${data.totalVolumeCredits}</em> credits</p>\n`;
  return result;
}

function usd(aNumber) {
  return new Intl.NumberFormat(
    "en-US", { style: "currency", currency: "USD", minimumFractionDigits: 2 }
  ).format(aNumber/100);
}
```


```js
export default function createStatementData(invoice, plays) {
  const result = {};
  result.customer = invoice.customer;
  result.performances = invoice.performances.map(enrichPerformance);
  result.totalAmount = totalAmount(result);
  result.totalVolumeCredits = totalVolumeCredits(result);
  return result;

  function enrichPerformance(aPerformance) {
    const result = Object.assign({}, aPerformance);
    result.play = playFor(result);
    result.amount = amountFor(result);
    result.volumeCredits = volumeCreditsFor(result);
    return result;
  }

  function playFor(aPerformance) {
    return plays[aPerformance.playID]
  }

  function amountFor(aPerformance) {
    let result = 0;
    switch (aPerformance.play.type) {
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
        throw new Error(`unknown type: ${aPerformance.play.type}`);
    }
    return result;
  }

  function volumeCreditsFor(aPerformance) {
    let result = 0;
    result += Math.max(aPerformance.audience - 30, 0);
    if ("comedy" === aPerformance.play.type) result += Math.floor(aPerformance.audience / 5);
    return result;
  }

  function totalAmount(data) {
    return data.performances
      .reduce((total, p) => total + p.amount, 0);
  }

  function totalVolumeCredits(data) {
    return data.performances
      .reduce((total, p) => total + p.volumeCredits, 0);
  }
}
```

## Status: Separated into Two Files (and Phases)
The extra code breaks up the logic into identifiable parts, separating the calculations of the statements from the layout. This modularity makes it easier for me to understand the parts of the code and how they fit together. Brevity is the soul of wit, but clarity is the soul of evolvable software.

:bulb: When programming, follow the camping rule: Always leave the code base healthier than when you found it.

## Reorganizing the Calculations by Type
Now I’ll turn my attention to the next feature change: supporting more categories of plays, each with its own charging and volume credits calculations.

Conditional logic tends to decay as further modifications are made unless it’s reinforced by more structural elements of the programming language.

## Status: Creating the Data with the Polymorphic Calculator
```js
export default function createStatementData(invoice, plays) {
  const result = {};
  result.customer = invoice.customer;
  result.performances = invoice.performances.map(enrichPerformance);
  result.totalAmount = totalAmount(result);
  result.totalVolumeCredits = totalVolumeCredits(result);
  return result;

  function enrichPerformance(aPerformance) {
    const calculator = createPerformanceCalculator(aPerformance, playFor(aPerformance));
    const result = Object.assign({}, aPerformance);
    result.play = calculator.play;
    result.amount = calculator.amount;
    result.volumeCredits = calculator.volumeCredits;
    return result;
  }
  function playFor(aPerformance) {
    return plays[aPerformance.playID]
  }
  function totalAmount(data) {
    return data.performances
      .reduce((total, p) => total + p.amount, 0);
  }
  function totalVolumeCredits(data) {
    return data.performances
      .reduce((total, p) => total + p.volumeCredits, 0);
  }
}

function createPerformanceCalculator(aPerformance, aPlay) {
    switch(aPlay.type) {
    case "tragedy":
      return new TragedyCalculator(aPerformance, aPlay);
    case "comedy" :
      return new ComedyCalculator(aPerformance, aPlay);
    default:
        throw new Error(`unknown type: ${aPlay.type}`);
    }
}

class PerformanceCalculator {
  constructor(aPerformance, aPlay) {
    this.performance = aPerformance;
    this.play = aPlay;
  }
  get amount() {
    throw new Error('subclass responsibility');}
  get volumeCredits() {
    return Math.max(this.performance.audience - 30, 0);
  }
}

class TragedyCalculator extends PerformanceCalculator {
  get amount() {
    let result = 40000;
    if (this.performance.audience > 30) {
      result += 1000 * (this.performance.audience - 30);
    }
    return result;
  }
}

class ComedyCalculator extends PerformanceCalculator {
  get amount() {
    let result = 30000;
    if (this.performance.audience > 20) {
      result += 10000 + 500 * (this.performance.audience - 20);
    }
    result += 300 * this.performance.audience;
    return result;
  }
  get volumeCredits() {
    return super.volumeCredits + Math.floor(this.performance.audience / 5);
  }
}
```

The benefit here is that the calculations for each kind of play are grouped together. If most of the changes will be to this code, it will be helpful to have it clearly separated like this. Adding a new kind of play requires writing a new subclass and adding it to the creation function.

My choice on whether to return the instance or calculate separate output data depends on who is using the downstream data structure. In this case, I preferred to show how to use the intermediate data structure to hide the decision to use a polymorphic calculator.

## Final Thoughts
As is often the case with refactoring, the early stages were mostly driven by trying to understand what was going on. A common sequence is: Read the code, gain some insight, and use refactoring to move that insight from your head back into the code.

:bulb: The true test of good code is how easy it is to change it.

Code should be obvious: When someone needs to make a change, they should be able to find the code to be changed easily and to make the change quickly without introducing any errors.

The key to effective refactoring is recognizing that you go faster when you take tiny steps, the code is never broken, and you can compose those small steps into substantial changes. Remember that—and the rest is silence.

# 2 Principles in Refactoring
**Refactoring (noun)**: a change made to the internal structure of software to make it easier to understand and cheaper to modify without changing its observable behavior.

**Refactoring (verb)**: to restructure software by applying a series of refactorings without changing its observable behavior.

:bulb: If someone says their code was broken for a couple of days while they are refactoring, you can be pretty sure they were not refactoring.


## The Two Hats
* Kent Beck came up with a metaphor of the two hats. When I use refactoring to develop software, I divide my time between two distinct activities: adding functionality and refactoring.
  * When I **add functionality**, I shouldn’t be changing existing code; I’m just adding new capabilities. I measure my progress by adding tests and getting the tests to work.
  * When I **refactor**, I make a point of not adding functionality; I only restructure the code. I don’t add any tests (unless I find a case I missed earlier); I only change tests when I have to accommodate a change in an interface.

## Why Should We Refactor?
I don’t want to claim refactoring is the cure for all software ills. It is no “silver bullet.” Yet it is a valuable tool—a pair of silver pliers that helps you keep a good grip on your code. Refactoring is a tool that can—and should—be used for several purposes.

1. Refactoring Improves the Design of Software
2. Refactoring Makes Software Easier to Understand
3. Refactoring Helps Me Find Bugs
4. Refactoring Helps Me Program Faster

## When Should We Refactor?
* The Rule of Three (Three strikes, then you refactor)
  * Here’s a guideline Don Roberts gave me: The first time you do something, you just do it. The second time you do something similar, you wince at the duplication, but you do the duplicate thing anyway. The third time you do something similar, you refactor.

* Preparatory Refactoring: Making It Easier to Add a Feature

  The best time to refactor is just before I need to add a new feature to the code base. As I do this, I look at the existing code and, often, see that if it were structured a little differently, my work would be much easier.

* Comprehension Refactoring: Making Code Easier to Understand

  I do comprehension refactoring on little details. I rename a couple variables now that I understand what they are, or I chop a long function into smaller parts. Then, as the code gets clearer, I find I can see things about the design that I could not see before.

* Litter: Pickup Refactoring

  There’s a bit of a tradeoff here. I don’t want to spend a lot of time distracted from the task I’m currently doing, but I also don’t want to leave the trash lying around and getting in the way of future changes. If it’s easy to change, I’ll do it right away. If it’s a bit more effort to fix, I might make a note of it and fix it when I’m done with my immediate task.

  As the old camping adage says, always leave the camp site cleaner than when you found it. If I make it a little better each time I pass through the code, over time it will get fixed.

* Planned and Opportunistic Refactoring

  The examples above—preparatory, comprehension, litter-pickup refactoring—are all opportunistic.

  Refactoring isn’t an activity that’s separated from programming—any more than you set aside time to write if statements. I don’t put time on my plans to do refactoring; most refactoring happens while I’m doing other things.

  :bulb: You have to refactor when you run into ugly code—but excellent code needs plenty of refactoring too.

  “for each desired change, make the change easy (warning: this may be hard), then make the easy change” — Kent Beck, https://twitter.com/kentbeck/status/250733358307500032

  Good developers know that, often, the fastest way to add a new feature is to change the code to make it easy to add. Software should thus be never thought of as “done.” As new capabilities are needed, the software changes to reflect that. Those changes can often be greater in the existing code than in the new code.

* Long-Term Refactoring

  Even in such cases, I’m reluctant to have a team do dedicated refactoring. Often, a useful strategy is to agree to gradually work on the problem over the course of the next few weeks.

  Whenever anyone goes near any code that’s in the refactoring zone, they move it a little way in the direction they want to improve. This takes advantage of the fact that refactoring doesn’t break the code—each small change leaves everything in a still-working state.

  To change from one library to another, start by introducing a new abstraction that can act as an interface to either library. Once the calling code uses this abstraction, it’s much easier to switch one library for another. (This tactic is called Branch By Abstraction [mf-bba].)

* Refactoring in a Code Review

  Code reviews help spread knowledge through a development team. Reviews help more experienced developers pass knowledge to those less experienced. They help more people understand more aspects of a large software system. They are also very important in writing clear code.

  Reviews also give the opportunity for more people to suggest useful ideas. I can only think of so many good ideas in a week.

  It’s better to have the original author of the code present because the author can provide context on the code and fully appreciate the reviewers’ intentions for their changes.

* What Do I Tell My Manager?

* When Should I Not Refactor?

  If I run across code that is a mess, but I don’t need to modify it, then I don’t need to refactor it.

  Another case is when it’s easier to rewrite it than to refactor it.

## Problems with Refactoring

* Slowing Down New Features

  :bulb: The whole purpose of refactoring is to make us program faster, producing more value with less effort.

  I’m more likely to not refactor if it’s part of the code I rarely touch and the cost of the inconvenience isn’t something I feel very often. Sometimes, I delay a refactoring because I’m not sure what improvement to do, although at other times I’ll try something as an experiment to see if it makes things better.

  Still, the evidence I hear from my colleagues in the industry is that too little refactoring is far more prevalent than too much. In other words, most people should try to refactor more often.

  The point of refactoring isn’t to show how sparkly a code base is—it is purely economic. We refactor because it makes us faster—faster to add features, faster to fix bugs.

* Code Ownership

  Code ownership boundaries get in the way of refactoring because I cannot make the kinds of changes I want without breaking my clients.

  Due to these complexities, I recommend against fine-grained strong code ownership. Some organizations like any piece of code to have a single programmer as an owner, and only allow that programmer to change it.

* Branches

  There are downsides to feature branches like this. The longer I work on an isolated branch, the harder the job of integrating my work with mainline is going to be when I’m done.

  If Rachel is working on her branch I don’t see her changes until she integrates with mainline; at that point, I have to merge her changes into my feature branch, which may mean considerable work.

  The hard part of this work is dealing with semantic changes. Modern version control systems can do wonders with merging complex changes to the program text, but they are blind to the semantics of the code. If I’ve changed the name of a function, my version control tool may easily integrate my changes with Rachel’s. But if, in her branch, she added a call to a function that I’ve renamed in mine, the code will fail.

  Continuous Integration (CI), also known as Trunk-Based Development. With CI, each team member integrates with mainline at least once per day. This prevents any branches diverting too far from each other and thus greatly reduces the complexity of merges.

  Fans of CI like it partly because it reduces the complexity of merges, but the dominant reason to favor CI is that it’s far more compatible with refactoring.

* Testing

  One of the key characteristics of refactoring is that it doesn’t change the observable behavior of the program.

  Self-testing code not only enables refactoring—it also makes it much safer to add new features, since I can quickly find and kill any bugs I introduce

  Self-testing code is, unsurprisingly, closely associated with Continuous Integration—it is the mechanism that we use to catch semantic integration conflicts. Such testing practices are another component of Extreme Programming and a key part of Continuous Delivery.

* Legacy Code

  Refactoring can be a fantastic tool to help understand a legacy system.

  If you have a big legacy system with no tests, you can’t safely refactor it into clarity.

  The obvious answer to this problem is that you add tests.

  The best advice I can give is to get a copy of <Working Effectively with Legacy Code [Feathers]> and follow its guidance. Don’t be worried by the age of the book—its advice is just as true more than a decade later.

* Databases

  My colleague Pramod Sadalage developed an approach to evolutionary database design [mf-evodb] and database refactoring [Ambler & Sadalage] that is now widely used. The essence of the technique is to combine the structural changes to a database’s schema and access code with data migration scripts that can easily compose to handle large changes.

  One difference from regular refactorings is that database changes often are best separated over multiple releases to production. This makes it easy to reverse any change that causes a problem in production. So, when renaming a field, my first commit would add the new database field but not use it. I may then set up the updates so they update both old and new fields at once. I can then gradually move the readers over to the new field. Only once they have all moved to the new field, and I’ve given a little time for any bugs to show themselves, would I remove the now-unused old field. This approach to database changes is an example of a general approach of parallel change [mf-pc] (also called expand-contract).

## Refactoring, Architecture, and Yagni
Refactoring has profoundly changed how people think about software architecture.

The real impact of refactoring on architecture is in how it can be used to form a well-designed code base that can respond gracefully to changing needs.

One way of dealing with future changes is to put flexibility mechanisms into the software.

With refactoring, I can use a different strategy. Instead of speculating on what flexibility I will need in the future and what mechanisms will best enable that, I build software that solves only the currently understood needs, but I make this software excellently designed for those needs. As my understanding of the users’ needs changes, I use refactoring to adapt the architecture to those new demands.

This approach to design goes under various names: simple design, incremental design, or yagni [mf-yagni] (originally an acronym for “you aren’t going to need it”).

## Refactoring and the Wider Software Development Process

To really operate in an agile way, a team has to be capable and enthusiastic refactorers—and for that, many aspects of their process have to align with making refactoring a regular part of their work.

The first foundation for refactoring is `self-testing code`.

To refactor on a team, it’s important that each member can refactor when they need to without interfering with others’ work. This is why I encourage `Continuous Integration`. With CI, each member’s refactoring efforts are quickly shared with their colleagues.

Refactoring and `yagni` positively reinforce each other: Not just is refactoring (and its prerequisites) a foundation for yagni—yagni makes it easier to do refactoring. This is because it’s easier to change a simple system than one that has lots of speculative flexibility included.

`Continuous Delivery` keeps our software in an always-releasable state. This is what allows many web organizations to release updates many times a day—but even if we don’t need that, it reduces risk and allows us to schedule our releases to satisfy business needs rather than technological constraints.

## Refactoring and Performance
I often make changes that will cause the program to run slower. This is an important issue. I don’t belong to the school of thought that ignores performance in favor of design purity or in hopes of faster hardware.

The secret to fast software, in all but hard real-time contexts, is to write tunable software first and then tune it for sufficient speed.

* I’ve seen three general approaches to writing fast software.
    1. The most serious of these is time budgeting, often used in hard real-time systems. As you decompose the design, you give each component a budget for resources—time and footprint.
    2. The second approach is the constant attention approach.
        * The performance improvements are spread all around the program; each improvement is made with a narrow perspective of the program’s behavior, and often with a misunderstanding of how a compiler, runtime, and hardware behaves.
        * The interesting thing about performance is that in most programs, most of their time is spent in a small fraction of the code.
    3. The third approach to performance improvement takes advantage of this 90-percent statistic. I follow a specific process to tune the program:
        * I begin by running the program under a profiler that monitors the program and tells me where it is consuming time and space.
        * This way I can find that small part of the program where the performance hot spots lie.
        * I then focus on those performance hot spots using the same optimizations I would use in the constant-attention approach
        * But since I’m focusing my attention on a hot spot, I’m getting much more effect with less work.

* Having a well-factored program helps with this style of optimization in two ways.
    * First, it gives me time to spend on performance tuning. With well-factored code, I can add functionality more quickly. This gives me more time to focus on performance. (Profiling ensures I spend that time on the right place.)
    * Second, with a well-factored program I have finer granularity for my performance analysis. My profiler leads me to smaller parts of the code, which are easier to tune. With clearer code, I have a better understanding of my options and of what kind of tuning will work.

## Where Did Refactoring Come From?

Good programmers have always spent at least some time cleaning up their code. They do this because they have learned that clean code is easier to change than complex and messy code, and good programmers know that they rarely write clean code the first time around.

## Automated Refactorings

## Going Further

Bill Wake’s Refactoring Workbook [Wake] that contains many exercises to practice refactoring.

Refactoring to Patterns [Kerievsky], which looks at the most valuable patterns from the hugely influential “Gang of Four” book [gof] and shows how to use refactoring to evolve towards them.

Refactoring Databases [Ambler & Sadalage] (by Scott Ambler and Pramod Sadalage) and Refactoring HTML [Harold] (by Elliotte Rusty Harold).

Michael Feathers’s Working Effectively with Legacy Code [Feathers], which is primarily a book about how to think about refactoring an older codebase with poor test coverage.

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
* As I describe the refactorings in the catalog, I use a standard format. Each refactoring has five parts, as follows:
    * I begin with a `name`. The name is important to building a vocabulary of refactorings. This is the name I use elsewhere in the book. Refactorings often go by different names now, so I also list any aliases that seem to be common.

    * I follow the name with a short `sketch` of the refactoring. This helps you find a refactoring more quickly.

    * The `motivation` describes why the refactoring should be done and describes circumstances in which it shouldn’t be done.

    * The `mechanics` are a concise, step-by-step description of how to carry out the refactoring.

    * The `examples` show a very simple use of the refactoring to illustrate how it works.

## The Choice of Refactorings


# 6 A First Set of Refactorings

Probably the most common refactoring I do is extracting code into a function (`Extract Function`) or a variable (`Extract Variable`). Since refactoring is all about change, it’s no surprise that I also frequently use the inverses of those two (`Inline Function` and `Inline Variable`).

Extraction is all about giving names, and I often need to change the names as I learn. `Change Function Declaration` changes names of functions; I also use that refactoring to add or remove a function’s arguments. For variables, I use `Rename Variable`, which relies on `Encapsulate Variable`. When changing function arguments, I often find it useful to combine a common clump of arguments into a single object with `Introduce Parameter Object`.

 I use `Combine Functions into Class` to group functions, together with the data they operate on, into a class. Another path I take is to combine them into a transform (`Combine Functions into Transform`), which is particularly handy with read-only data. At a step further in scale, I can often form these modules into distinct processing phases using `Split Phase`.

## Extract Function

![](../Images/Refactor/6-extract-function.jpg)


* Motivation:

  I’ve heard many arguments about when to enclose code in its own function. The argument that makes most sense to me, however, is the separation between intention and implementation. If you have to spend effort looking at a fragment of code and figuring out what it’s doing, then you should extract it into a function and name the function after the “what.”

* Mechanics
    1. Create a new function, and name it after the intent of the function (name it by what it does, not by how it does it).

        If I can’t come up with a more meaningful name, that’s a sign that I shouldn’t extract the code.

    2. Copy the extracted code from the source function into the new target function.

    3. Scan the extracted code for references to any variables that are local in scope to the source function and will not be in scope for the extracted function. Pass them as parameters.

        I find that too many local variables are being assigned by the extracted code. It’s better to abandon the extraction at this point. When this happens, I consider other refactorings such as `Split Variable` or `Replace Temp with Query` to simplify variable usage and revisit the extraction later.

    4. Compile after all variables are dealt with.

    5. Replace the extracted code in the source function with a call to the target function.

    6. Test.

    7. Look for other code that’s the same or similar to the code just extracted, and consider using `Replace Inline Code with Function Call` to call the new function.

* Example
    ```js
    function printOwing(invoice) {
        let outstanding = 0;

        console.log("***********************");
        console.log("**** Customer Owes ****");
        console.log("***********************");

        // calculate outstanding
        for (const o of invoice.orders) {
            outstanding += o.amount;
        }

        // record due date
        const today = Clock.today;
        invoice.dueDate = new Date(today.getFullYear(), today.getMonth(), today.getDate() + 30);

        //print details
        console.log(`name: ${invoice.customer}`);
        console.log(`amount: ${outstanding}`);
        console.log(`due: ${invoice.dueDate.toLocaleDateString()}`);
    }
    ```
    ```js
    function printOwing(invoice) {
        printBanner();
        recordDueDate(invoice);
        printDetails(invoice, calculateOutstanding(invoice));
    }

    function printBanner() {
        console.log("***********************");
        console.log("**** Customer Owes ****");
        console.log("***********************");
    }

    // Example: Using Local Variables
    function recordDueDate(invoice) {
        const today = Clock.today;
        invoice.dueDate = new Date(today.getFullYear(), today.getMonth(), today.getDate() + 30);
    }

    function printDetails(invoice, outstanding) {
        console.log(`name: ${invoice.customer}`);
        console.log(`amount: ${outstanding}`);
        console.log(`due: ${invoice.dueDate.toLocaleDateString()}`);
    }

    // Example: Reassigning a Local Variable
    function calculateOutstanding(invoice) {
        let outstanding = 0;
        for (const o of invoice.orders) {
            outstanding += o.amount;
        }
        return outstanding;
    }
    ```

## Inline Function

![](../Images/Refactor/6-inline-function.jpg)

* Motivation

    One of the themes of this book is using short functions named to show their intent, because these functions lead to clearer and easier to read code.

* Mechanics
    1. Check that this isn’t a polymorphic method.

    2. Find all the callers of the function.

    3. Replace each call with the function’s body.

    4. Test after each replacement.

    5. Remove the function definition.

* Example
    ```js
    function reportLines(aCustomer) {
        const lines = [];
        gatherCustomerData(lines, aCustomer);
        return lines;
    }

    function gatherCustomerData(out, aCustomer) {
        out.push(["name", aCustomer.name]);
        out.push(["location", aCustomer.location]);
    }
    ```

    ```js
    function reportLines(aCustomer) {
        const lines = [];
        lines.push(["name", aCustomer.name]);
        lines.push(["location", aCustomer.location]);
        return lines;
    }
    ```

## Extract Variable

![](../Images/Refactor/6-extract-variable.jpg)

* Motivation

    Expressions can become very complex and hard to read. In such situations, local variables may help break the expression down into something more manageable

    Such variables are also handy for debugging, since they provide an easy hook for a debugger or print statement to capture.

    I also think about the context of that name. If it’s only meaningful within the function I’m working on, then Extract Variable is a good choice—but if it makes sense in a broader context, I’ll consider making the name available in that broader context, usually as a function.

    The downside of promoting the name to a broader context is extra effort. If it’s significantly more effort, I’m likely to leave it till later when I can use `Replace Temp with Query`.

* Mechanics
    1. Ensure that the expression you want to extract does not have side effects.

    2. Declare an immutable variable. Set it to a copy of the expression you want to name.

    3. Replace the original expression with the new variable.

    4. Test.

* Example
    ```js
    function price(order) {
        //price is base price - quantity discount + shipping
        return order.quantity * order.itemPrice -
        Math.max(0, order.quantity - 500) * order.itemPrice * 0.05 +
        Math.min(order.quantity * order.itemPrice * 0.1, 100);
    }
    ```
    ```js
    function price(order) {
        const basePrice = order.quantity * order.itemPrice;
        const quantityDiscount = Math.max(0, order.quantity - 500) * order.itemPrice * 0.05;
        const shipping = Math.min(basePrice * 0.1, 100);
        return basePrice - quantityDiscount + shipping;
    }
    ```

* Example: With a Class
    ```js
    class Order {
        constructor(aRecord) {
            this._data = aRecord;
        }

        get quantity()  {return this._data.quantity;}
        get itemPrice() {return this._data.itemPrice;}
        get price() {
            return this.quantity * this.itemPrice -
                Math.max(0, this.quantity - 500) * this.itemPrice * 0.05 +
                Math.min(this.quantity * this.itemPrice * 0.1, 100);
        }
    }
    ```
    ```js
    class Order {
        constructor(aRecord) {
            this._data = aRecord;
        }
        get quantity()  {return this._data.quantity;}
        get itemPrice() {return this._data.itemPrice;}
        get price() {
            return this.basePrice - this.quantityDiscount + this.shipping;
        }
        get basePrice()        {return this.quantity * this.itemPrice;}
        get quantityDiscount() {return Math.max(0, this.quantity - 500) * this.itemPrice * 0.05;}
        get shipping()         {return Math.min(this.basePrice * 0.1, 100);}
    }
    ```

## Inline Variable

![](../Images/Refactor/6-inline-variable.jpg)

* Motivation

    Variables provide names for expressions within a function, and as such they are usually a Good Thing. But sometimes, the name doesn’t really communicate more than the expression itself.

* Mechanics
    1. Check that the right-hand side of the assignment is free of side effects.

    2. If the variable isn’t already declared immutable, do so and test.

    3. Find the first reference to the variable and replace it with the right-hand side of the assignment.

    4. Test.

    5. Repeat replacing references to the variable until you’ve replaced all of them.

    6. Remove the declaration and assignment of the variable.

    7. Test.

## Change Function Declaration

![](../Images/Refactor/6-change-function-declaration.jpg)

* Motivation

    Functions represent the joints in our software systems. And, as with any construction, much depends on those joints. Good joints allow me to add new parts to the system easily, but bad ones are a constant source of difficulty, making it harder to figure out what the software does and how to modify it as my needs change.

    A good name allows me to understand what the function does when I see it called, without seeing the code that defines its implementation.

* Mechanics
    * Simple Mechanics
        1. If you’re removing a parameter, ensure it isn’t referenced in the body of the function.
        2. Change the method declaration to the desired declaration.
            * It’s often best to separate changes, so if you want to both change the name and add a parameter, do these as separate steps.
        3. Find all references to the old method declaration, update them to the new one.
        4. Test.

    * Migration Mechanics
        1. If necessary, refactor the body of the function to make it easy to do the following extraction step.
        2. Use `Extract Function` on the function body to create the new function.
            * If the new function will have the same name as the old one, give the new function a temporary name that’s easy to search for.
        3. If the extracted function needs additional parameters, use the simple mechanics to add them.
        4. Test.
        5. `Apply Inline Function` to the old function.
        6. If you used a temporary name, use `Change Function Declaration` again to restore it to the original name.
        7. Test

* Example: Renaming a Function (Simple Mechanics)
    ```js
    function circum(radius) {
        return 2 * Math.PI * radius;
    }
    ```
    ```js
    function circumference(radius) {
        return 2 * Math.PI * radius;
    }
    ```

    If I’m both renaming the function and adding a parameter, I first do the rename, test, then add the parameter, and test again.

* Example: Renaming a Function (Migration Mechanics)
    ```js
    function circum(radius) {
        return 2 * Math.PI * radius;
    }
    ```
    ```js
    // `Extract Function` to the entire function body.
    function circum(radius) {
        return circumference(radius);
    }

    function circumference(radius) {
        return 2 * Math.PI * radius;
    }
    ```

* Example: Adding a Parameter
    ```js
    addReservation(customer) {
        this._reservations.push(customer);
    }
    ```
    ```js
    // `Extract Function`
    addReservation(customer) {
        this.zz_addReservation(customer);
    }

    zz_addReservation(customer) {
        this._reservations.push(customer);
    }
    ```
    ```js
    // add the parameter to the new declaration and its call
    addReservation(customer) {
        this.zz_addReservation(customer, false);
    }

    zz_addReservation(customer, isPriority) {
        this._reservations.push(customer);
    }
    ```
    ```js
    // `Introduce Assertion` to check the new parameter is used by the caller.
    zz_addReservation(customer, isPriority) {
        assert(isPriority === true || isPriority === false);
        this._reservations.push(customer);
    }
    ```
    ```js
    // Inline Function on the original function
    addReservation(customer, isPriority) {
        assert(isPriority === true || isPriority === false);
        this._reservations.push(customer);
    }
    ```

* Example: Changing a Parameter to One of Its Properties
    ```js
    function inNewEngland(aCustomer) {
        return ["MA", "CT", "ME", "VT", "NH", "RI"].includes(aCustomer.address.state);
    }

    const newEnglanders = someCustomers.filter(c => inNewEngland(c));
    ```
    inNewEngland only uses the customer’s home state to determine if it’s in New England. I’d prefer to refactor inNewEngland so that it takes a state code as a parameter, making it usable in more contexts by removing the dependency on the customer.
    ```js
    //  Extract Variable
    function inNewEngland(aCustomer) {
        const stateCode = aCustomer.address.state;
        return ["MA", "CT", "ME", "VT", "NH", "RI"].includes(stateCode);
    }
    ```
    ```js
    // use `Extract Function` to create that new function
    function inNewEngland(aCustomer) {
        const stateCode = aCustomer.address.state;
        return xxNEWinNewEngland(stateCode);
    }

    function xxNEWinNewEngland(stateCode) {
        return ["MA", "CT", "ME", "VT", "NH", "RI"].includes(stateCode);
    }
    ```
    ```js
    // apply `Inline Variable` on the input parameter in the original function.
    function inNewEngland(aCustomer) {
        return xxNEWinNewEngland(aCustomer.address.state);
    }
    ```
    ```js
    // I use Inline Function to fold the old function into its callers,
    // effectively replacing the call to the old function with a call to the new one.
    const newEnglanders = someCustomers.filter(c => xxNEWinNewEngland(c.address.state));
    ```
    ```js
    // Once I’ve inlined the old function into every caller,
    // I use Change Function Declaration again to change the name of the new function to that of the original.
    const newEnglanders = someCustomers.filter(c => inNewEngland(c.address.state));

    function inNewEngland(stateCode) {
        return ["MA", "CT", "ME", "VT", "NH", "RI"].includes(stateCode);
    }
    ```



## Encapsulate Variable

* Motivation

    Refactoring is all about manipulating the elements of our programs. Data is more awkward to manipulate than functions.

    Data is more awkward because I can’t do that. If I move data around, I have to change all the references to the data in a single cycle to keep the code working. For data with a very small scope of access, such as a temporary variable in a small function, this isn’t a problem. But as the scope grows, so does the difficulty, which is why global data is such a pain.

    So if I want to move widely accessed data, often the best approach is to first encapsulate it by routing all its access through functions. That way, I turn the difficult task of reorganizing data into the simpler task of reorganizing functions.

    The greater the scope of the data, the more important it is to encapsulate. My approach with legacy code is that whenever I need to change or add a new reference to such a variable, I should take the opportunity to encapsulate it. That way I prevent the increase of coupling to commonly used data.

    Keeping data encapsulated is much less important for immutable data.

* Mechanics
    1. Create encapsulating functions to access and update the variable.
    2. Run static checks.
    3. For each reference to the variable, replace with a call to the appropriate encapsulating function. Test after each replacement.
    4. Restrict the visibility of the variable.
        * Sometimes it’s not possible to prevent access to the variable. If so, it may be useful to detect any remaining references by renaming the variable and testing.
    5. Test.
    6. If the value of the variable is a record, consider `Encapsulate Record`.

* Example
    ```js
    // global variable
    let defaultOwner = {firstName: "Martin", lastName: "Fowler"};

    // usage
    spaceship.owner = defaultOwner;
    defaultOwner = {firstName: "Rebecca", lastName: "Parsons"};
    ```

    ```js
    // defining functions to read and write the data.
    function getDefaultOwner()    {return defaultOwner;}
    function setDefaultOwner(arg) {defaultOwner = arg;}
    ```

    ```js
    // replace the global variable usage with functions
    spaceship.owner = getDefaultOwner();
    setDefaultOwner({firstName: "Rebecca", lastName: "Parsons"});
    ```
    ```js
    // restrict the visibility of the variable.
    // This both checks that there aren’t any references that I’ve missed,
    // and ensures that future changes to the code won’t access the variable directly
    let defaultOwner = {firstName: "Martin", lastName: "Fowler"};
    export function getDefaultOwner()    {return defaultOwner;}
    export function setDefaultOwner(arg) {defaultOwner = arg;}
    ```

* Example: Encapsulating the Value

    The basic refactoring I’ve outlined here encapsulates a reference to some data structure, allowing me to control its access and reassignment. But it doesn’t control changes to that structure.
    ```js
    const owner1 = defaultOwner();
    assert.equal("Fowler", owner1.lastName, "when set");
    const owner2 = defaultOwner();
    owner2.lastName = "Parsons";
    assert.equal("Parsons", owner1.lastName, "after change owner2"); // is this ok?
    ```

    The basic refactoring encapsulates the reference to the data item. In many cases, this is all I want to do for the moment. But I often want to take the encapsulation deeper to control not just changes to the variable but also to its contents.

    An alternative is to prevent changes—and a good way of doing that is `Encapsulate Record`.

    ```js
    let defaultOwnerData = {firstName: "Martin", lastName: "Fowler"};
    export function defaultOwner()       {return new Person(defaultOwnerData);}
    export function setDefaultOwner(arg) {defaultOwnerData = arg;}

    class Person {
        constructor(data) {
            this._lastName = data.lastName;
            this._firstName = data.firstName
        }

        get lastName() {return this._lastName;}
        get firstName() {return this._firstName;}
        // and so on for other properties
    }
    ```

## Rename Variable

![](../Images/Refactor/6-rename-variable.jpg)

* Motivation

    Naming things well is the heart of clear programming. Variables can do a lot to explain what I’m up to—if I name them well.

    Even more than most program elements, the importance of a name depends on how widely it’s used.

* Mechanics
    1. If the variable is used widely, consider `Encapsulate Variable`.
    2. Find all references to the variable, and change every one.
        * If there are references from another code base, the variable is a published variable, and you cannot do this refactoring.
        * If the variable does not change, you can copy it to one with the new name, then change gradually, testing after each change.
    3. Test.

* Example
    ```js
    let tpHd = "untitled";

    result += `<h1>${tpHd}</h1>`;
    tpHd = obj['articleTitle'];
    ```

    ```js
    // `Encapsulate Variable`
    result += `<h1>${title()}</h1>`;

    setTitle(obj['articleTitle']);

    function title()       {return tpHd;}
    function setTitle(arg) {tpHd = arg;}
    ```
    ```js
    // rename variable
    let _title = "untitled";

    function title()       {return _title;}
    function setTitle(arg) {_title = arg;}
    ```

* Example: Renaming a Constant

    ```js
    const cpyNm = "Acme Gooseberries";
    ```
    ```js
    const companyName = "Acme Gooseberries";
    const cpyNm = companyName;
    ```

## Introduce Parameter Object
![](../Images/Refactor/6-introduce-paramter-object.jpg)

* Motivation

    Grouping data into a structure is valuable because it makes explicit the relationship between the data items. It reduces the size of parameter lists for any function that uses the new structure. It helps consistency since all functions that use the structure will use the same names to get at its elements.

    But the real power of this refactoring is how it enables deeper changes to the code. This process can change the conceptual picture of the code, raising these structures as new abstractions that can greatly simplify my understanding of the domain.

* Mechanics
    1. If there isn’t a suitable structure already, create one.
    2. I prefer to use a class, as that makes it easier to group behavior later on. I usually like to ensure these structures are value objects [mf-vo].
    3. Test.
    4. Use `Change Function Declaration` to add a parameter for the new structure.
    5. Test.
    6. Adjust each caller to pass in the correct instance of the new structure. Test after each one.
    7. For each element of the new structure, replace the use of the original parameter with the element of the structure. Remove the parameter.
    8. Test.

* Example
    ```js
    const station = {
        name: "ZB1",
        readings: [
            {temp: 47, time: "2016-11-10 09:10"},
            {temp: 53, time: "2016-11-10 09:20"},
            {temp: 58, time: "2016-11-10 09:30"},
            {temp: 53, time: "2016-11-10 09:40"},
            {temp: 51, time: "2016-11-10 09:50"},
        ]
    };

    function readingsOutsideRange(station, min, max) {
        return station.readings.filter(r => r.temp < min || r.temp > max);
    }

    alerts = readingsOutsideRange(station, operatingPlan.temperatureFloor, peratingPlan.temperatureCeiling);
    ```

    ```js
    class NumberRange {
        constructor(min, max) {
            this._data = {min: min, max: max};
        }
        get min() {return this._data.min;}
        get max() {return this._data.max;}
    }
    ```
    ```js
    // Change Function Declaration
    function readingsOutsideRange(station, min, max, range) {
        return station.readings .filter(r => r.temp < min || r.temp > max);
    }

    // caller
    alerts = readingsOutsideRange(
        station,
        operatingPlan.temperatureFloor,
        operatingPlan.temperatureCeiling,
        null);
    ```

    ```js
    // replacing the usage of the parameters
    function readingsOutsideRange(station, range) {
        return station.readings.filter(r => r.temp < range.min || r.temp > range.max);
    }

    // caller
    const range = new NumberRange(operatingPlan.temperatureFloor, operatingPlan.temperatureCeiling);
    alerts = readingsOutsideRange(station,  range);
    ```

    However, replacing a clump of parameters with a real object is just the setup for the really good stuff. The great benefits of making a class like this is that I can then move behavior into the new class.

    ```js
    function readingsOutsideRange(station, range) {
        return station.readings.filter(r => !range.contains(r.temp));
    }

    contains(arg) {return (arg >= this.min && arg <= this.max);}
    ```

## Combine Functions into Class

![](../Images/Refactor/6-combine-functions-into-class.jpg)

* Motivation

    Classes are a fundamental construct in most modern programming languages. They bind together data and functions into a shared environment, exposing some of that data and function to other program elements for collaboration. They are the primary construct in object-oriented languages, but are also useful with other approaches too.

    When I see a group of functions that operate closely together on a common body of data (usually passed as arguments to the function call), I see an opportunity to form a class.

    One significant advantage of using a class is that it allows clients to mutate the core data of the object, and the derivations remain consistent.

* Mechanics
    1. Apply `Encapsulate Record` to the common data record that the functions share.
        * If the data that is common between the functions isn’t already grouped into a record structure, use `Introduce Parameter Object` to create a record to group it together.
    2. Take each function that uses the common record and use `Move Function` to move it into the new class.
        * Any arguments to the function call that are members can be removed from the argument list.
    3. Each bit of logic that manipulates the data can be extracted with `Extract Function` and then moved into the new class.

* Example
    ```js
    reading = {customer: "ivan", quantity: 10, month: 5, year: 2017};
    ```
    ```js
    // client 1
    const aReading = acquireReading();
    const baseCharge = baseRate(aReading.month, aReading.year) * aReading.quantity;

    // client 2
    const aReading = acquireReading();
    const base = (baseRate(aReading.month, aReading.year) * aReading.quantity);
    const taxableCharge = Math.max(0, base - taxThreshold(aReading.year));

    // client 3
    const aReading = acquireReading();
    const basicChargeAmount = calculateBaseCharge(aReading);

    function calculateBaseCharge(aReading) {
        return baseRate(aReading.month, aReading.year) * aReading.quantity;
    }
    ```

    ```js
    // Encapsulate Record
    class Reading {
        constructor(data) {
            this._customer = data.customer;
            this._quantity = data.quantity;
            this._month = data.month;
            this._year = data.year;
        }
        get customer() {return this._customer;}
        get quantity() {return this._quantity;}
        get month()    {return this._month;}
        get year()     {return this._year;}
    }
    ```

    ```js
    //  `Move Function` to move calculateBaseCharge into the new class
    class Reading {
        constructor(data) {
            this._customer = data.customer;
            this._quantity = data.quantity;
            this._month = data.month;
            this._year = data.year;
        }
        get customer() {return this._customer;}
        get quantity() {return this._quantity;}
        get month()    {return this._month;}
        get year()     {return this._year;}
        get calculateBaseCharge() {
            return baseRate(this.month, this.year) * this.quantity;
        }
    }

    // client 3
    const rawReading = acquireReading();
    const aReading = new Reading(rawReading);
    const basicChargeAmount = aReading.calculateBaseCharge();
    ```

    ```js
    // `Rename Function` to make it something more to my liking.
    class Reading {
        constructor(data) {
            this._customer = data.customer;
            this._quantity = data.quantity;
            this._month = data.month;
            this._year = data.year;
        }
        get customer() {return this._customer;}
        get quantity() {return this._quantity;}
        get month()    {return this._month;}
        get year()     {return this._year;}
        get baseCharge() {
            return baseRate(this.month, this.year) * this.quantity;
        }
    }

    // client 3
    const rawReading = acquireReading();
    const aReading = new Reading(rawReading);
    const basicChargeAmount = aReading.baseCharge();
    ```

    With this naming, the client of the reading class can’t tell whether the base charge is a field or a derived value. This is a Good Thing—the Uniform Access Principle [mf-ua].

    ```js
    // client 1
    const rawReading = acquireReading();
    const aReading = new Reading(rawReading);
    const baseCharge = aReading.baseCharge();

    // client 2
    const rawReading = acquireReading();
    const aReading = new Reading(rawReading);
    const taxableCharge = Math.max(0, aReading.baseCharge() - taxThreshold(aReading.year));

    // Extract Function
    function taxableChargeFn(aReading) {
        return Math.max(0, aReading.baseCharge() - taxThreshold(aReading.year));
    }

    // client 3
    const rawReading = acquireReading();
    const aReading = new Reading(rawReading);
    const taxableCharge = taxableChargeFn(aReading);

    // Move Function taxableChargeFn to Reading Class
    class Reading {
        constructor(data) {
            this._customer = data.customer;
            this._quantity = data.quantity;
            this._month = data.month;
            this._year = data.year;
        }
        get customer() {return this._customer;}
        get quantity() {return this._quantity;}
        get month()    {return this._month;}
        get year()     {return this._year;}
        get baseCharge() {
            return baseRate(this.month, this.year) * this.quantity;
        }
        function taxableCharge(aReading) {
            return Math.max(0, aReading.baseCharge() - taxThreshold(aReading.year));
        }
    }
    // client 3
    const aReading = new Reading(acquireReading());
    const taxableCharge = aReading.taxableCharge();
    ```

## Combine Functions into Transform

![](../Images/Refactor/6-combine-funtions-into-transform.jpg)

* Motivation

    Software often involves feeding data into programs that calculate various derived information from it. These derived values may be needed in several places, and those calculations are often repeated wherever the derived data is used.

    One way to do this is to use a data transformation function that takes the source data as input and calculates all the derivations, putting each derived value as a field in the output data. Then, to examine the derivations, all I need do is look at the transform function.

    Using a class is much better if the source data gets updated within the code. Using a transform stores derived data in the new record, so if the source data changes, I will run into inconsistencies.

* Mechanics
    1. Create a transformation function that takes the record to be transformed and returns the same values.
        * This will usually involve a deep copy of the record. It is often worthwhile to write a test to ensure the transform does not alter the original record.
    2. Pick some logic and move its body into the transform to create a new field in the record. Change the client code to access the new field.
        * If the logic is complex, use `Extract Function` first.
    3. Test.
    4. Repeat for the other relevant functions.

* Example
    ```js
    reading = {customer: "ivan", quantity: 10, month: 5, year: 2017};
    ```
    ```js
    // client 1
    const aReading = acquireReading();
    const baseCharge = baseRate(aReading.month, aReading.year) * aReading.quantity;

    // client 2
    const aReading = acquireReading();
    const base = (baseRate(aReading.month, aReading.year) * aReading.quantity);
    const taxableCharge = Math.max(0, base - taxThreshold(aReading.year));

    // client 3
    const aReading = acquireReading();
    const basicChargeAmount = calculateBaseCharge(aReading);

    function calculateBaseCharge(aReading) {
        return  baseRate(aReading.month, aReading.year) * aReading.quantity;
    }
    ```
    ```js
    function enrichReading(original) { // transformReading(original);
        const result = _.cloneDeep(original);
        return result;
    }

    // client 3
    const rawReading = acquireReading();
    const aReading = enrichReading(rawReading);
    const basicChargeAmount = calculateBaseCharge(aReading);
    ```
    ```js
    // Move Function
    function enrichReading(original) {
        const result = _.cloneDeep(original);
        result.baseCharge = calculateBaseCharge(result);
        return result;
    }

    // client 3
    const rawReading = acquireReading();
    const aReading = enrichReading(rawReading);
    const basicChargeAmount = aReading.baseCharge;
    ```
    One trap to beware of here. When I write enrichReading like this, to return the enriched reading, I’m implying that the original reading record isn’t changed. So it’s wise for me to add a test.
    ```js
    it('check reading unchanged', function() {
        const baseReading = {customer: "ivan", quantity: 15, month: 5, year: 2017};
        const oracle = _.cloneDeep(baseReading);
        enrichReading(baseReading);
        assert.deepEqual(baseReading, oracle);
    });
    ```
    ```js
    const rawReading = acquireReading();
    const aReading = enrichReading(rawReading);
    const base = aReading.baseCharge;
    const taxableCharge = Math.max(0, base - taxThreshold(aReading.year));

    // inline variable
    const rawReading = acquireReading();
    const aReading = enrichReading(rawReading);
    const taxableCharge = Math.max(0, aReading.baseCharge; - taxThreshold(aReading.year));
    ```
    ```js
    // move that computation into the transformer
    function enrichReading(original) {
        const result = _.cloneDeep(original);
        result.baseCharge = calculateBaseCharge(result);
        result.taxableCharge = Math.max(0, result.baseCharge - taxThreshold(result.year));
        return result;
    }

    const rawReading = acquireReading();
    const aReading = enrichReading(rawReading);
    const taxableCharge = aReading.taxableCharge;
    ```

## Split Phase

![](../Images/Refactor/6-split-phase.jpg)

* Motivation

    The best clue is when different stages of the fragment use different sets of data and functions. By turning them into separate modules I can make this difference explicit, revealing the difference in the code.

* Mechanics
    1. Extract the second phase code into its own function.
    2. Test.
    3. Introduce an intermediate data structure as an additional argument to the extracted function.
    4. Test.
    5. Examine each parameter of the extracted second phase. If it is used by first phase, move it to the intermediate data structure. Test after each move.
        * Sometimes, a parameter should not be used by the second phase. In this case, extract the results of each usage of the parameter into a field of the intermediate data structure and use `Move Statements to Callers` on the line that populates it.
    6. Apply `Extract Function` on the first-phase code, returning the intermediate data structure.
        * It’s also reasonable to extract the first phase into a transformer object.

* Example
    ```js
    function priceOrder(product, quantity, shippingMethod) {
        const basePrice = product.basePrice * quantity;
        const discount = Math.max(quantity - product.discountThreshold, 0)
                * product.basePrice * product.discountRate;

        const shippingPerCase = (basePrice > shippingMethod.discountThreshold)
                ? shippingMethod.discountedFee : shippingMethod.feePerCase;
        const shippingCost = quantity * shippingPerCase;
        const price =  basePrice - discount + shippingCost;
        return price;
    }
    ```
    There is a sense of two phases going on here. The first couple of lines of code use the product information to calculate the product-oriented price of the order, while the later code uses shipping information to determine the shipping cost.
    ```js
    // Extract Function
    function priceOrder(product, quantity, shippingMethod) {
        const basePrice = product.basePrice * quantity;
        const discount = Math.max(quantity - product.discountThreshold, 0)
                * product.basePrice * product.discountRate;
        const price =  applyShipping(basePrice, shippingMethod, quantity, discount);
        return price;
    }

    function applyShipping(basePrice, shippingMethod, quantity, discount) {
        const shippingPerCase = (basePrice > shippingMethod.discountThreshold)
                ? shippingMethod.discountedFee : shippingMethod.feePerCase;
        const shippingCost = quantity * shippingPerCase;
        const price =  basePrice - discount + shippingCost;
        return price;
    }
    ```

    ```js
    // introduce the intermediate data structure that will communicate between the two phases.
    function priceOrder(product, quantity, shippingMethod) {
        const basePrice = product.basePrice * quantity;
        const discount = Math.max(quantity - product.discountThreshold, 0)
            * product.basePrice * product.discountRate;

        // basePrice, quantity, discount are created by the first-phase code.
        // shippingMethod isn’t used by the first-phase code, leave as is.
        const priceData = {basePrice: basePrice, quantity: quantity, discount: discount};

        const price =  applyShipping(priceData, shippingMethod);
        return price;
    }

    function applyShipping(priceData, shippingMethod) {
        const shippingPerCase =
            (priceData.basePrice > shippingMethod.discountThreshold)
            ? shippingMethod.discountedFee
            : shippingMethod.feePerCase;
        const shippingCost = priceData.quantity * shippingPerCase;
        const price =  priceData.basePrice - priceData.discount + shippingCost;
        return price;
    }
    ```
    ```js
    // extract the first-phase code into its own function, returning intermedia data.
    function priceOrder(product, quantity, shippingMethod) {
        const priceData = calculatePricingData(product, quantity);
        return applyShipping(priceData, shippingMethod);
    }

    function calculatePricingData(product, quantity) {
        const basePrice = product.basePrice * quantity;
        const discount = Math.max(quantity - product.discountThreshold, 0) * product.basePrice * product.discountRate;
        return {basePrice: basePrice, quantity: quantity, discount:discount};
    }

    function applyShipping(priceData, shippingMethod) {
        const shippingPerCase =
            (priceData.basePrice > shippingMethod.discountThreshold)
            ? shippingMethod.discountedFee
            : shippingMethod.feePerCase;
        const shippingCost = priceData.quantity * shippingPerCase;
        return priceData.basePrice - priceData.discount + shippingCost;
    }
    ```

# 7 Encapsulation

Perhaps the most important criteria to be used in decomposing modules is to identify secrets that modules should hide from the rest of the system [Parnas]. Data structures are the most common secrets, and I can hide data structures by encapsulating them with **Encapsulate Record** and **Encapsulate Collection**. Even primitive data values can be encapsulated with **Replace Primitive with Object**

Using `Replace Temp with Query` is a great help here, particularly when splitting up an overly long function.

Classes and modules are the largest forms of encapsulation, but functions also encapsulate their implementation.

## Encapsulate Record

![](../Images/Refactor/7-encapsulate-record.jpg)

* Motivation

    This is why I often favor objects over records for mutable data. With objects, I can hide what is stored and provide methods for all three values. The user of the object doesn’t need to know or care which is stored and which is calculated. This encapsulation also helps with renaming.

    I just said I favor objects for mutable data. If I have an immutable value, I can just have all three values in my record, using an enrichment step if necessary.

    I can have two kinds of record structures: those where I declare the legal field names and those that allow me to use whatever I like. The latter are often implemented through a library class called something like hash, map, hashmap, dictionary, or associative array.

* Mechanics
    * Use `Encapsulate Variable` on the variable holding the record.
        * Give the functions that encapsulate the record names that are easily searchable.
    * Replace the content of the variable with a simple class that wraps the record. Define an accessor inside this class that returns the raw record. Modify the functions that encapsulate the variable to use this accessor.
    * Test.
    * Provide new functions that return the object rather than the raw record.
    * For each user of the record, replace its use of a function that returns the record with a function that returns the object. Use an accessor on the object to get at the field data, creating that accessor if needed. Test after each change.
        * If it’s a complex record, such as one with a nested structure, focus on clients that update the data first. Consider returning a copy or read-only proxy of the data for clients that only read the data.
    * Remove the class’s raw data accessor and the easily searchable functions that returned the raw record.
    * Test.
    * If the fields of the record are themselves structures, consider using `Encapsulate Record` and `Encapsulate Collection` recursively.

* Example
    ```js
    const organization = {name: "Acme Gooseberries", country: "GB"};

    result += `<h1>${organization.name}</h1>`;
    organization.name = newName;
    ```

    ```js
    // encapsulate variable
    function getRawDataOfOrganization() {return organization;}
    result += `<h1>${getRawDataOfOrganization().name}</h1>`;
    getRawDataOfOrganization().name = newName;
    ```

    ```js
    // replacing the record with a class
    class Organization {
        constructor(data) {
            this._data = data;
        }

        set name(aString)   {this._data.name = aString;}
        get name()          {return this._data.name;}
    }

    const organization = new Organization({name: "Acme Gooseberries", country: "GB"});
    function getOrganization() {return organization;}

    // client
    result += `<h1>${getOrganization().name}</h1>`;
    getOrganization().name = newName;
    ```
    ```js
    // I’d also be inclined to fold the _data field directly into the object.
    class Organization {
        constructor(data) {
            this._name = data.name;
            this._country = data.country;
        }
        get name()    {return this._name;}
        set name(aString) {this._name = aString;}
        get country()    {return this._country;}
        set country(aCountryCode) {this._country = aCountryCode;}
    }
    ```

* Example: Encapsulating a Nested Record
    ```json
    1920": {
        name: "martin",
        id: "1920",
        usages: {
            "2016": {
            "1": 50,
            "2": 55,
            // remaining months of the year
            },
            "2015": {
            "1": 70,
            "2": 63,
            // remaining months of the year
            }
        }
    },
    "38673": {
        name: "neal",
        id: "38673",
        // more customers in a similar form
    }
    ```

    ```js
    // client
    customerData[customerID].usages[year][month] = amount;

    function compareUsage (customerID, laterYear, month) {
        const later   = customerData[customerID].usages[laterYear][month];
        const earlier = customerData[customerID].usages[laterYear - 1][month];
        return {laterAmount: later, change: later - earlier};
    }
    ```
    ```js
    // start with `Encapsulate Variable`
    function getRawDataOfCustomers()    {return customerData;}
    function setRawDataOfCustomers(arg) {customerData = arg;}

    // client
    getRawDataOfCustomers()[customerID].usages[year][month] = amount;

    function compareUsage (customerID, laterYear, month) {
        const later   = getRawDataOfCustomers()[customerID].usages[laterYear][month];
        const earlier = getRawDataOfCustomers()[customerID].usages[laterYear - 1][month];
        return {laterAmount: later, change: later - earlier};
    }
    ```
    ```js
    //  make a class for the overall data structure
    class CustomerData {
        constructor(data) {
            this._data = data;
        }
    }

    function getCustomerData() {return customerData;}
    function getRawDataOfCustomers()    {return customerData._data;}
    function setRawDataOfCustomers(arg) {customerData = new CustomerData(arg);}
    ```

## Encapsulate Collection

![](../Images/Refactor/7-encapsulate-collection.jpg)

* Motivation

    I like encapsulating any mutable data in my programs. This makes it easier to see when and how data structures are modified, which then makes it easier to change those data structures when I need to.

    Access to a collection variable may be encapsulated, but if the getter returns the collection itself, then that collection’s membership can be altered without the enclosing class being able to intervene.

    To avoid this, I provide collection modifier methods—usually add and remove—on the class itself.

    A better approach is to ensure that the getter for the collection does not return the raw collection, so that clients cannot accidentally change it.

    One way to prevent modification of the underlying collection is by never returning a collection value. Another way is to allow some form of read-only access to a collection.

* Mechanics
    1. Apply `Encapsulate Variable` if the reference to the collection isn’t already encapsulated.
    2. Add functions to add and remove elements from the collection.
        * If there is a setter for the collection, use `Remove Setting Method` if possible. If not, make it take a copy of the provided collection.
    3. Run static checks.
    4. Find all references to the collection. If anyone calls modifiers on the collection, change them to use the new add/remove functions. Test after each change.
    5. Modify the getter for the collection to return a protected view on it, using a read-only proxy or a copy.
    6. Test.

* Example
    ```js
    class Person {
        constructor (name) {
            this._name = name;
            this._courses = [];
        }
        get name() {return this._name;}
        get courses() {return this._courses;}
        set courses(aList) {this._courses = aList;}
    }

    class Course {
        constructor(name, isAdvanced) {
            this._name = name;
            this._isAdvanced = isAdvanced;
        }
        get name()       {return this._name;}
        get isAdvanced() {return this._isAdvanced;}
    }

    // client
    numAdvancedCourses = aPerson.courses.filter(c => c.isAdvanced).length;

    const basicCourseNames = readBasicCourseNames(filename);
    aPerson.courses = basicCourseNames.map(name => new Course(name, false));

    for (const name of readBasicCourseNames(filename)) {
        aPerson.courses.push(new Course(name, false));
    }
    ```
    This violates encapsulating because the person class has no ability to take control when the list is updated in this way. While the reference to the field is encapsulated, the content of the field is not.
    ```js
    class Person {
        constructor (name) {
            this._name = name;
            this._courses = [];
        }
        get name() {return this._name;}
        get courses() {return this._courses;}
        set courses(aList) {this._courses = aList;}

        addCourse(aCourse) {
            this._courses.push(aCourse);
        }
        removeCourse(aCourse, fnIfAbsent = () => {throw new RangeError();}) {
            const index = this._courses.indexOf(aCourse);
            if (index === -1)
                fnIfAbsent();
            else
                this._courses.splice(index, 1);
        }
    }

    // client
    for(const name of readBasicCourseNames(filename)) {
        aPerson.addCourse(new Course(name, false));
    }
    ```
    ```js
    // Remove Setting Method
    class Person {
        constructor (name) {
            this._name = name;
            this._courses = [];
        }
        const& get name() {return this._name;}
        const& get courses() {return this._courses;}

        addCourse(aCourse) {
            this._courses.push(aCourse);
        }
        removeCourse(aCourse, fnIfAbsent = () => {throw new RangeError();}) {
            const index = this._courses.indexOf(aCourse);
            if (index === -1)
                fnIfAbsent();
            else
                this._courses.splice(index, 1);
        }
    }
    ```

## Replace Primitive with Object

formerly: Replace Data Value with Object, Replace Type Code with Class

![](../Images/Refactor/7-replace-primitive-with-object.jpg)

* Motivation

    At first, such a class does little more than wrap the primitive—but once I have that class, I have a place to put behavior specific to its needs.

* Mechanics
    * Apply `Encapsulate Variable` if it isn’t already.
    * Create a simple value class for the data value. It should take the existing value in its constructor and provide a getter for that value.
    * Run static checks.
    * Change the setter to create a new instance of the value class and store that in the field, changing the type of the field if present.
    * Change the getter to return the result of invoking the getter of the new class.
    * Test.
    * Consider using `Rename Function` on the original accessors to better reflect what they do.
    * Consider clarifying the role of the new object as a value or reference object by applying `Change Reference to Value` or `Change Value to Reference`.

* Example
    ```js
    class Order {
        constructor(data) {
            this.priority = data.priority; // string value
            // more initialization
        }
    }

    // client
    highPriorityCount = orders.filter(o => "high" === o.priority|| "rush" === o.priority).length;
    ```
    Whenever I’m fiddling with a data value, the first thing I do is use `Encapsulate Variable` on it
    ```js
    class Order {
        constructor(data) {
            this.priority = data.priority;
            // more initialization
        }
        get priority()        {return this._priority;}
        set priority(aString) {this._priority = aString;}
    }
    ```
    ```js
    class Priority {
        constructor(value) {this._value = value;}
        toString() {return this._value;}
    }

    class Order {
        constructor(data) {
            this.priority = data.priority;
            // more initialization
        }
        get priority()        {return this._priority.toString();}
        set priority(aString) {this._priority = new Priority(aString);}
    }
    ```
    the current getter on the order to be misleading, so `Rename Function`.
    ```js
    get priorityString()  {return this._priority.toString();}
    set priority(aString) {this._priority = new Priority(aString);}

    // client
    highPriorityCount = orders.filter(o => "high" === o.priorityString || "rush" === o.priorityString).length;
    ```
    As I look at who uses the priority, I consider whether they should use the priority class themselves. As a result, I provide a getter on order that provides the new priority object directly.
    ```js
    class Order {
        get priority()        {return this._priority;}
        get priorityString()  {return this._priority.toString();}
        set priority(aString) {this._priority = new Priority(aString);}
    }

    // client
    highPriorityCount = orders.filter(o => "high" === o.priority.toString() || "rush" === o.priority.toString()).length;
    ```
    As the priority class becomes useful elsewhere, I would allow clients of the order to use the setter with a priority instance, which I do by adjusting the priority constructor.
    ```js
    class Priority {
        constructor(value) {
            if (value instanceof Priority) return value;
            if (Priority.legalValues().includes(value))
                this._value = value;
            else
                throw new Error(`<${value}> is invalid for Priority`);
        }
        toString() {return this._value;}
        get _index() {return Priority.legalValues().findIndex(s => s === this._value);}
        static legalValues() {return ['low', 'normal', 'high', 'rush'];}

        equals(other) {return this._index === other._index;}
        higherThan(other) {return this._index > other._index;}
        lowerThan(other) {return this._index < other._index;}
    }

    // client
    highPriorityCount = orders.filter(o => o.priority.higherThan(new Priority("normal"))).length;
    ```

## Replace Temp with Query

![](../Images/Refactor/7-replace-temp-with-query.jpg)

* Motivation

    One use of temporary variables is to capture the value of some code in order to refer to it later in a function. Using a temp allows me to refer to the value while explaining its meaning and avoiding repeating the code that calculates it. But while using a variable is handy, it can often be worthwhile to go a step further and use a function instead.

    Putting variable logic into functions often also sets up a stronger boundary between the extracted logic and the original function, which helps me spot and avoid awkward dependencies and side effects.

    Using functions instead of variables also allows me to avoid duplicating the calculation logic in similar functions.

    This refactoring works best if I’m inside a class, since the class provides a shared context for the methods I’m extracting.  Outside of a class, I’m liable to have too many parameters in a top-level function which negates much of the benefit of using a function.

* Mechanics
    * Check that the variable is determined entirely before it’s used, and the code that calculates it does not yield a different value whenever it is used.
    * If the variable isn’t read-only, and can be made read-only, do so.
    * Test.
    * Extract the assignment of the variable into a function.
        * If the variable and the function cannot share a name, use a temporary name for the function.
        * Ensure the extracted function is free of side effects. If not, use `Separate Query from Modifier`.
    * Test.
    * Use `Inline Variable` to remove the temp.

* Example
    ```js
    class Order {
        constructor(quantity, item) {
            this._quantity = quantity;
            this._item = item;
        }

        get price() {
            var basePrice = this._quantity * this._item.price;
            var discountFactor = 0.98;
            if (basePrice > 1000) discountFactor -= 0.03;
            return basePrice * discountFactor;
        }
    }
    ```
    ```js
    // encapsulate basePrice into a function
    class Order {
        constructor(quantity, item) {
            this._quantity = quantity;
            this._item = item;
        }

        get price() {
            const basePrice = this.basePrice;
            var discountFactor = 0.98;
            if (basePrice > 1000) discountFactor -= 0.03;
            return basePrice * discountFactor;
        }

        get basePrice() {
            return this._quantity * this._item.price;
        }
    }
    ```
    ```js
    // Inline Variable
    class Order {
        constructor(quantity, item) {
            this._quantity = quantity;
            this._item = item;
        }

        get price() {
            var discountFactor = 0.98;
            if (this.basePrice > 1000) discountFactor -= 0.03;
            return this.basePrice * discountFactor;
        }

        get basePrice() {
            return this._quantity * this._item.price;
        }
    }
    ```
    ```js
    // Extract Funciton for discountFactor
    class Order {
        constructor(quantity, item) {
            this._quantity = quantity;
            this._item = item;
        }

        get price() {
            return this.basePrice * this.discountFactor;
        }

        get discountFactor() {
            var discountFactor = 0.98;
            if (this.basePrice > 1000) discountFactor -= 0.03;
            return discountFactor;
        }

        get basePrice() {
            return this._quantity * this._item.price;
        }
    }
    ```

## Extract Class

![](../Images/Refactor/7-extract-class.jpg)

* Motivation

    You need to consider where it can be split—and split it. A good sign is when a subset of the data and a subset of the methods seem to go together. Other good signs are subsets of data that usually change together or are particularly dependent on each other. A useful test is to ask yourself what would happen if you remove a piece of data or a method. What other fields and methods would become nonsense?

* Mechanics
    * Decide how to split the responsibilities of the class.
    * Create a new child class to express the split-off responsibilities.
        * If the responsibilities of the original parent class no longer match its name, rename the parent.
    * Create an instance of the child class when constructing the parent and add a link from parent to child.
    * Use `Move Field` on each field you wish to move. Test after each move.
    * Use `Move Function` to move methods to the new child. Start with lower-level methods (those being called rather than calling). Test after each move.
    * Review the interfaces of both classes, remove unneeded methods, change names to better fit the new circumstances.
    * Decide whether to expose the new child. If so, consider applying `Change Reference to Value` to the child class.

* Example
    ```js
    class Person {
        get name()    {return this._name;}
        set name(arg) {this._name = arg;}
        get telephoneNumber() {return `(${this.officeAreaCode}) ${this.officeNumber}`;}
        get officeAreaCode()    {return this._officeAreaCode;}
        set officeAreaCode(arg) {this._officeAreaCode = arg;}
        get officeNumber() {return this._officeNumber;}
        set officeNumber(arg) {this._officeNumber = arg;}
    }
    ```
    ```js
    // separate the telephone number behavior into its own class
    // I start by defining an empty telephone number class
    class TelephoneNumber {

    }
    ```
    ```js
    // create an instance of telephone number when constructing the person
    constructor() {
        this._telephoneNumber = new TelephoneNumber();
    }
    ```

    ```js
    // use `Move Field` on _officeAreaCode
    class TelephoneNumber {
        get officeAreaCode()    {return this._officeAreaCode;}
        set officeAreaCode(arg) {this._officeAreaCode = arg;}
    }

    class Persion {
        get officeAreaCode()    {return this._telephoneNumber.officeAreaCode;}
        set officeAreaCode(arg) {this._telephoneNumber.officeAreaCode = arg;}
    }

    // use `Move Field` on _officeNumber
    class TelephoneNumber {
        get officeNumber() {return this._officeNumber;}
        set officeNumber(arg) {this._officeNumber = arg;}
    }

    class Persion {
        get officeNumber() {return this._telephoneNumber._officeNumber;}
        set officeNumber(arg) {this._telephoneNumber._officeNumber = arg;}
    }
    ```
    ```js
    // move the telephone number method
    class TelephoneNumber {
        get telephoneNumber() {return `(${this.officeAreaCode}) ${this.officeNumber}`;}
    }

    class Person {
        get telephoneNumber() {return this._telephoneNumber.telephoneNumber;}
    }

    // move the area code, number methods
    class TelephoneNumber {
        get areaCode()    {return this._areaCode;}
        set areaCode(arg) {this._areaCode = arg;}
        get number()    {return this._number;}
        set number(arg) {this._number = arg;}
    }

    class Person {
        get officeAreaCode()    {return this._telephoneNumber.areaCode;}
        set officeAreaCode(arg) {this._telephoneNumber.areaCode = arg;}
        get officeNumber()    {return this._telephoneNumber.number;}
        set officeNumber(arg) {this._telephoneNumber.number = arg;}
    }
    ```
    ```js
    // move the telephone number method.
    class TelephoneNumber {
        get telephoneNumber() {return `(${this.officeAreaCode}) ${this.officeNumber}`;}
    }

    class Person {
        get telephoneNumber() {return this._telephoneNumber.telephoneNumber;}
    }
    ```
    ```js
    // `Rename Function`:  telephone number method on the telephone number class also doesn’t make much sense
    class TelephoneNumber {
        toString() {return `(${this.areaCode}) ${this.number}`;}
    }

    class Person {
        get telephoneNumber() {return this._telephoneNumber.toString();}
    }
    ```

## Inline Class

![](../Images/Refactor/7-inline-class.jpg)

* Motivation

    I use Inline Class if a class is no longer pulling its weight and shouldn’t be around any more. Often, this is the result of refactoring that moves other responsibilities out of the class so there is little left. At that point, I fold the class into another—one that makes most use of the runt class.

    Another reason to use Inline Class is if I have two classes that I want to refactor into a pair of classes with a different allocation of features. I may find it easier to first use Inline Class to combine them into a single class, then `Extract Class` to make the new separation. This is a general approach when reorganizing things: Sometimes, it’s easier to move elements one at a time from one context to another, but sometimes it’s better to use an inline refactoring to collapse the contexts together, then use an extract refactoring to separate them into different elements.

* Mechanics
    * In the target class, create functions for all the public functions of the source class. These functions should just delegate to the source class.
    * Change all references to source class methods so they use the target class’s delegators instead. Test after each change.
    * Move all the functions and data from the source class into the target, testing after each move, until the source class is empty.
    * Delete the source class and hold a short, simple funeral service.

* Example
    ```js
    class TrackingInformation {
        get shippingCompany()       {return this._shippingCompany;}
        set shippingCompany(arg)    {this._shippingCompany = arg;}
        get trackingNumber()        {return this._trackingNumber;}
        set trackingNumber(arg)     {this._trackingNumber = arg;}
        get display()               {
            return `${this.shippingCompany}: ${this.trackingNumber}`;
        }
    }

    class shipment {
        get trackingInfo() {
            return this._trackingInformation.display;
        }
        get trackingInformation()    {return this._trackingInformation;}
        set trackingInformation(aTrackingInformation) {
            this._trackingInformation = aTrackingInformation;
        }
    }
    ```
    ```js
    // putting a delegating method into the shipment, and adjusting the client to call that.
    class Shipment {
        set shippingCompany(arg) {this._trackingInformation.shippingCompany = arg;}
    }

    // client old
    aShipment.trackingInformation.shippingCompany = request.vendor;

    // client new
    aShipment.shippingCompany = request.vendor;
    ```
    ```js
    // applying Inline Function to the display method
    class Shipment {
        get trackingInfo() {
            return `${this.shippingCompany}: ${this.trackingNumber}`;
        }
    }
    ```
    ```js
    // move the shipping company field
    class Shipment {
        get shippingCompany()    {return this._shippingCompany;}
        set shippingCompany(arg) {this._shippingCompany = arg;}
    }
    ```
    ```js
    class Shipment {
        get trackingInfo() {
            return `${this.shippingCompany}: ${this.trackingNumber}`;
        }
        get shippingCompany()    {return this._shippingCompany;}
        set shippingCompany(arg) {this._shippingCompany = arg;}
        get trackingNumber()    {return this._trackingNumber;}
        set trackingNumber(arg) {this._trackingNumber = arg;}
    }
    ```

## Hide Delegate

![](../Images/Refactor/7-hide-delegate.jpg)

* Motivation

    One of the keys—if not the key—to good modular design is encapsulation. Encapsulation means that modules need to know less about other parts of the system. Then, when things change, fewer modules need to be told about the change—which makes the change easier to make.

    When we are first taught about object orientation, we are told that encapsulation means hiding our fields. As we become more sophisticated, we realize there is more that we can encapsulate.

* Mechanics
    * For each method on the delegate, create a simple delegating method on the server.
    * Adjust the client to call the server. Test after each change.
    * If no client needs to access the delegate anymore, remove the server’s accessor for the delegate.
    * Test.

* Example
    ```js
    class Person {
        constructor(name) {
            this._name = name;
        }
        get name() {return this._name;}
        get department()    {return this._department;}
        set department(arg) {this._department = arg;}
    }

    class Departmen {
        get chargeCode()    {return this._chargeCode;}
        set chargeCode(arg) {this._chargeCode = arg;}
        get manager()    {return this._manager;}
        set manager(arg) {this._manager = arg;}
    }

    // client
    manager = aPerson.department.manager;
    ```
    ```js
    // creating a simple delegating method on person for department
    class Person {
        get manager() {return this._department.manager;}
    }

    // client
    manager = aPerson.manager;
    ```

## Remove Middle Man

![](../Images/Refactor/7-remove-middle-man.jpg)

* Motivation

    Every time the client wants to use a new feature of the delegate, I have to add a simple delegating method to the server. After adding features for a while, I get irritated with all this forwarding. The server class is just a middle man (`Middle Man`), and perhaps it’s time for the client to call the delegate directly.

* Mechanics
    * Create a getter for the delegate.
    * For each client use of a delegating method, replace the call to the delegating method by chaining through the accessor. Test after each replacement.
        * If all calls to a delegating method are replaced, you can delete the delegating method.
        * With automated refactorings, you can use `Encapsulate Variable` on the delegate field and then `Inline Function` on all the methods that use it.

* Example
    ```js
    class Person {
        get manager() {return this._department.manager;}
    }

    class Department {
        get manager() {return this._manager;}
    }

    // client
    manager = aPerson.manager;
    ```
    ```js
    class Person {
        get department() {return this._department;}
        get manager() {return this.department.manager;}
    }

    // client
    manager = aPerson.department.manager;
    ```
    There is no absolute reason why I should either hide a delegate or remove a middle man—particular circumstances suggest which approach to take, and reasonable people can differ on what works best.

## Substitute Algorithm

![](../Images/Refactor/7-substitute-algorithm.jpg)

* Motivation

    Sometimes, when I want to change the algorithm to work slightly differently, it’s easier to start by replacing it with something that would make my change more straightforward to make.

    When I have to take this step, I have to be sure I’ve decomposed the method as much as I can. Replacing a large, complex algorithm is very difficult; only by making it simple can I make the substitution tractable.

* Mechanics
    * Arrange the code to be replaced so that it fills a complete function.
    * Prepare tests using this function only, to capture its behavior.
    * Prepare your alternative algorithm.
    * Run static checks.
    * Run tests to compare the output of the old algorithm to the new one. If they are the same, you’re done. Otherwise, use the old algorithm for comparison in testing and debugging.

# 8 Moving Features

I use `Move Function` to move functions between classes and other modules. Fields can move too, with `Move Field`.

I also move individual statements around. I use `Move Statements into Function` and `Move Statements to Callers` to move them in or out of functions, as well as `Slide Statements` to move them within a function. Sometimes, I can take some statements that match an existing function and use `Replace Inline Code with Function Call` to remove the duplication.

Two refactorings I often do with loops are `Split Loop`, to ensure a loop does only one thing, and `Replace Loop with Pipeline` to get rid of a loop entirely.

And then there’s the favorite refactoring of many a fine programmer: `Remove Dead Code`. Nothing is as satisfying as applying the digital flamethrower to superfluous statements.

## Move Function

![](../Images/Refactor/8-moe-function.jpg)

* Motivation

    The heart of a good software design is its modularity—which is my ability to make most modifications to a program while only having to understand a small part of it.

    One of the most straightforward reasons to move a function is when it references elements in other contexts more than the one it currently resides in. Moving it together with those elements often improves encapsulation, allowing other parts of the software to be less dependent on the details of this module.

    I may move a function because of where its callers live, or where I need to call it from in my next enhancement. A function defined as a helper inside another function may have value on its own, so it’s worth moving it to somewhere more accessible. A method on a class may be easier for me to use if shifted to another.

* Mechanics
    * Examine all the program elements used by the chosen function in its current context. Consider whether they should move too.
        * If I find a called function that should also move, I usually move it first. That way, moving a clusters of functions begins with the one that has the least dependency on the others in the group.
        * If a high-level function is the only caller of subfunctions, then you can inline those functions into the high-level method, move, and reextract at the destination.
    * Check if the chosen function is a polymorphic method.
        * If I’m in an object-oriented language, I have to take account of super- and subclass declarations.
    * Copy the function to the target context. Adjust it to fit in its new home.
        * If the body uses elements in the source context, I need to either pass those elements as parameters or pass a reference to that source context.
        * Moving a function often means I need to come up with a different name that works better in the new context.
    * Perform static analysis.
    * Figure out how to reference the target function from the source context.
    * Turn the source function into a delegating function.
    * Test.
    * Consider `Inline Function` on the source function.
        * The source function can stay indefinitely as a delegating function. But if its callers can just as easily reach the target directly, then it’s better to remove the middle man.

* Example: Moving a Nested Function to Top Level
    ```js
    function trackSummary(points) {
        const totalTime = calculateTime();
        const totalDistance = calculateDistance();
        const pace = totalTime / 60 /  totalDistance ;
        return {
            time: totalTime,
            distance: totalDistance,
            pace: pace
        };

        function calculateDistance() {
            let result = 0;
            for (let i = 1; i < points.length; i++) {
            result += distance(points[i-1], points[i]);
            }
            return result;
        }

        function distance(p1,p2) { ... }
        function radians(degrees) { ... }
        function calculateTime() { ... }
    }
    ```
    I’d like to move calculateDistance to the top level so I can calculate distances for tracks without all the other parts of the summary.
    ```js
    // move calculateDistance to the top level
    function top_calculateDistance() {
        let result = 0;
        for (let i = 1; i < points.length; i++) {
            result += distance(points[i-1], points[i]);
        }
        return result;
    }

    function trackSummary(points) {
        const totalTime = calculateTime();
        const totalDistance = calculateDistance();
        const pace = totalTime / 60 /  totalDistance ;
        return {
            time: totalTime,
            distance: totalDistance,
            pace: pace
        };

        function calculateDistance() {
            let result = 0;
            for (let i = 1; i < points.length; i++) {
            result += distance(points[i-1], points[i]);
            }
            return result;
        }
        ...

        function distance(p1,p2) { ... }
        function radians(degrees) { ... }
        function calculateTime() { ... }

    }
    ```
    ```js
    // move dependencies
    function top_calculateDistance(points) {
        let result = 0;
        for (let i = 1; i < points.length; i++) {
            result += distance(points[i-1], points[i]);
        }
        return result;

        function distance(p1,p2) { ... }
        function radians(degrees) { ... }
    }

    function trackSummary(points) {
        const totalTime = calculateTime();
        const totalDistance = calculateDistance();
        const pace = totalTime / 60 /  totalDistance ;
        return {
            time: totalTime,
            distance: totalDistance,
            pace: pace
        };

        function calculateDistance() {
            return top_calculateDistance(points);
        }
    }
    ```
    ```js
    // Since the functions for distance and radians don’t depend on anything inside totalDistance,
    // I prefer to move them to top level too, putting all four functions at the top level.
    function trackSummary(points) { ... }
    function totalDistance(points) { ... }
    function distance(p1,p2) { ... }
    function radians(degrees) { ... }
    ```

* Example: Moving between Classes
    ```js
    class Account {
        get bankCharge() {
            let result = 4.5;
            if (this._daysOverdrawn > 0)
                result += this.overdraftCharge;
            return result;
        }

        get overdraftCharge() {
            if (this.type.isPremium) {
                const baseCharge = 10;
                if (this.daysOverdrawn <= 7)
                    return baseCharge;
                else
                    return baseCharge + (this.daysOverdrawn - 7) * 0.85;
            }
            else
                return this.daysOverdrawn * 1.75;
        }
    }
    ```
    Coming up are changes that lead to different types of account having different algorithms for determining the charge. Thus it seems natural to move overdraftCharge to the account type class.
    ```js
    class AccountType {
        overdraftCharge(daysOverdrawn) {
            if (this.isPremium) {
                const baseCharge = 10;
                if (daysOverdrawn <= 7)
                    return baseCharge;
                else
                    return baseCharge + (daysOverdrawn - 7) * 0.85;
            }
            else
                return daysOverdrawn * 1.75;
        }
    }

    class Account {
        get bankCharge() {
            let result = 4.5;
            if (this._daysOverdrawn > 0)
                result += this.overdraftCharge;
            return result;
        }

        get overdraftCharge() {
            return this.type.overdraftCharge(this.daysOverdrawn);
        }
    }
    ```
    ```js
    // ineline function: overdraftCharge
    class Account {
        get bankCharge() {
            let result = 4.5;
            if (this._daysOverdrawn > 0)
                result += this.type.overdraftCharge(this.daysOverdrawn);
            return result;
        }
    }
    ```
    ```js
    // modify parameters
    // if there’s a lot of data from the account to pass, I might prefer to pass the account itself.
    class Account {
        get bankCharge() {
        let result = 4.5;
            if (this._daysOverdrawn > 0)
                result += this.type.overdraftCharge(this);
            return result;
        }
    }

    class AccountType {
        overdraftCharge(account) {
            if (this.isPremium) {
                const baseCharge = 10;
                if (account.daysOverdrawn <= 7)
                    return baseCharge;
                else
                    return baseCharge + (account.daysOverdrawn - 7) * 0.85;
            }
            else
                return account.daysOverdrawn * 1.75;
        }
    }
    ```

## Move Field

![](../Images/Refactor/8-move-field.jpg)

* Motivation

    * Programming involves writing a lot of code that implements behavior—but the strength of a program is really founded on its data structures. If I have a good set of data structures that match the problem, then my behavior code is simple and straightforward. But poor data structures lead to lots of code whose job is merely dealing with the poor data.

    * As soon as I realize that a data structure isn’t right, it’s vital to change it. If I leave my data structures with their blemishes, those blemishes will confuse my thinking and complicate my code far into the future.

    * Pieces of data that are always passed to functions together are usually best put in a single record in order to clarify their relationship.

    * Change is also a factor:
        1. If a change in one record causes a field in another record to change too, that’s a sign of a field in the wrong place.
        2. If I have to update the same field in multiple structures, that’s a sign that it should move to another place where it only needs to be updated once.

* Mechanics
    1. Ensure the source field is encapsulated.
    2. Test.
    3. Create a field (and accessors) in the target.
    4. Run static checks.
    5. Ensure there is a reference from the source object to the target object.
        * An existing field or method may give you the target. If not, see if you can easily create a method that will do so. Failing that, you may need to create a new field in the source object that can store the target. This may be a permanent change, but you can also do it temporarily until you have done enough refactoring in the broader context.
    6. Adjust accessors to use the target field.
        * If the target is shared between source objects, consider first updating the setter to modify both target and source fields, followed by `Introduce Assertion` to detect inconsistent updates. Once you determine all is well, finish changing the accessors to use the target field.
    7. Test.
    8. Remove the source field.
    9. Test.

* Example
    ```js
    class Customer {
        constructor(name, discountRate) {
            this._name = name;
            this._discountRate = discountRate;
            this._contract = new CustomerContract(dateToday());
        }
        get discountRate() {return this._discountRate;}
        becomePreferred() {
            this._discountRate += 0.03;
            // other nice things
        }
        applyDiscount(amount) {
            return amount.subtract(amount.multiply(this._discountRate));
        }
    }

    class CustomerContract {
        constructor(startDate) {
            this._startDate = startDate;
        }
    }
    ```
    I want to move the discount rate field from the customer to the customer contract.
    ```js
    // 1. Encapsulate Variable: discountRate, add _setDiscountRate function
    class Customer {
        constructor(name, discountRate) {
            this._name = name;
            this._setDiscountRate(discountRate);
            this._contract = new CustomerContract(dateToday());
        }

        get discountRate() {return this._discountRate;}

        _setDiscountRate(aNumber) {this._discountRate = aNumber;}

        becomePreferred() {
            this._setDiscountRate(this.discountRate + 0.03);
            // other nice things
        }

        applyDiscount(amount) {
            return amount.subtract(amount.multiply(this.discountRate));
        }
    }
    ```
    ```js
    // 3. add a field and accessors to the customer contract
    class CustomerContract {
        constructor(startDate, discountRate) {
            this._startDate = startDate;
            this._discountRate = discountRate;
        }
        get discountRate()    {return this._discountRate;}
        set discountRate(arg) {this._discountRate = arg;}
    }
    ```
    ```js
    // 5. Ensure there is a reference from the source object to the target object
    // Customer already has discountRate() to get the target object this._contract.discountRate
    ```
    ```js
    // 6. Adjust accessors to use the target field
    class Customer {
        get discountRate() {return this._contract.discountRate;}
        _setDiscountRate(aNumber) {this._contract.discountRate = aNumber;}
    }
    ```

* Example: Moving to a Shared Object
    ```js
    class Account {
        constructor(number, type, interestRate) {
            this._number = number;
            this._type = type;
            this._interestRate = interestRate;
        }
        get interestRate() {return this._interestRate;}
    }

    class AccountType {
        constructor(nameString) {
            this._name = nameString;
        }
    }
    ```
    I want to change things so that an account’s interest rate is determined from its account type.
    ```js
    // 1. the filed is already encapsulated
    // 3. Create a field (and accessors) in the target.
    class AccountType {
        constructor(nameString, interestRate) {
            this._name = nameString;
            this._interestRate = interestRate;
        }
        get interestRate() {return this._interestRate;}
    }
    ```
    ```js
    // 6. Adjust accessors to use the target field
    class Account {
        constructor(number, type) {
            this._number = number;
            this._type = type;
        }
        get interestRate() {return this._type.interestRate;}
    }
    ```

## Move Statements into Function

![](../Images/Refactor/8-move-statements-into-function.jpg)

* Motivation

    Removing duplication is one of the best rules of thumb of healthy code. If I see the same code executed every time I call a particular function, I look to combine that repeating code into the function itself. That way, any future modifications to the repeating code can be done in one place and used by all the callers. Should the code vary in the future, I can easily move it (or some of it) out again with `Move Statements to Callers`.

    If they don’t make sense as part of the called function, but still should be called with it, I’ll simply use `Extract Function` on the statements and the called function.

* Mechanics
    1. If the repetitive code isn’t adjacent to the call of the target function, use `Slide Statements` to get it adjacent.
    2. If the target function is only called by the source function, just cut the code from the source, paste it into the target, test, and ignore the rest of these mechanics.
    3. If you have more callers, use `Extract Function` on one of the call sites to extract both the call to the target function and the statements you wish to move into it. Give it a name that’s transient, but easy to grep.
    4. Convert every other call to use the new function. Test after each conversion.
    5. When all the original calls use the new function, use `Inline Function` to inline the original function completely into the new function, removing the original function.
    6. `Rename Function` to change the name of the new function to the same name as the original function. Or to a better name, if there is one.

* Example
    ```js
    function renderPerson(outStream, person) {
        const result = [];
        result.push(`<p>${person.name}</p>`);
        result.push(renderPhoto(person.photo));
        result.push(`<p>title: ${person.photo.title}</p>`);
        result.push(emitPhotoData(person.photo));
        return result.join("\n");
    }

    function photoDiv(p) {
        return [
            "<div>",
            `<p>title: ${p.title}</p>`,
            emitPhotoData(p),
            "</div>",
        ].join("\n");
    }

    function emitPhotoData(aPhoto) {
        const result = [];
        result.push(`<p>location: ${aPhoto.location}</p>`);
        result.push(`<p>date: ${aPhoto.date.toDateString()}</p>`);
        return result.join("\n");
    }
    ```
    This code shows two calls to emitPhotoData. I’d like to remove this duplication by moving the title printing into emitPhotoData.
    ```js
    // 3. Extract Function on one of the callers.
    function photoDiv(p) {
        return [
            "<div>",
            zznew(p),
            "</div>",
        ].join("\n");
    }

    function zznew(p) {
        return [
            `<p>title: ${p.title}</p>`,
            emitPhotoData(p),
        ].join("\n");
    }
    ```
    ```js
    // 4. Convert every other call to use the new function
    function renderPerson(outStream, person) {
        const result = [];
        result.push(`<p>${person.name}</p>`);
        result.push(renderPhoto(person.photo));
        result.push(zznew(person.photo));
        return result.join("\n");
    }
    ```
    ```js
    // 5. use Inline Function on emitPhotoData
    function zznew(p) {
        return [
            `<p>title: ${p.title}</p>`,
            `<p>location: ${p.location}</p>`,
            `<p>date: ${p.date.toDateString()}</p>`,
        ].join("\n");
    }
    ```
    ```js
    // 6. finish with Rename Function
    function renderPerson(outStream, person) {
        const result = [];
        result.push(`<p>${person.name}</p>`);
        result.push(renderPhoto(person.photo));
        result.push(emitPhotoData(person.photo));
        return result.join("\n");
    }

    function photoDiv(aPhoto) {
        return [
            "<div>",
            emitPhotoData(aPhoto),
            "</div>",
        ].join("\n");
    }

    function emitPhotoData(aPhoto) {
        return [
            `<p>title: ${aPhoto.title}</p>`,
            `<p>location: ${aPhoto.location}</p>`,
            `<p>date: ${aPhoto.date.toDateString()}</p>`,
        ].join("\n");
    }
    ```
## Move Statements to Callers

![](../Images/Refactor/8-move-statements-to-callers.jpg)

* Motivation

    Functions are the basic building block of the abstractions we build as programmers. And, as with any abstraction, we don’t always get the boundaries right. As a code base changes its capabilities—as most useful software does—we often find our abstraction boundaries shift. For functions, that means that what might once have been a cohesive, atomic unit of behavior becomes a mix of two or more different things.

    One trigger for this is when common behavior used in several places needs to vary in some of its calls. Now, we need to move the varying behavior out of the function to its callers.

* Mechanics
    1. In simple circumstances, where you have only one or two callers and a simple function to call from, just cut the first line from the called function and paste (and perhaps fit) it into the callers. Test and you’re done.
    2. Otherwise, apply `Extract Function` to all the statements that you don’t wish to move; give it a temporary but easily searchable name.
        * If the function is a method that is overridden by subclasses, do the extraction on all of them so that the remaining method is identical in all classes. Then remove the subclass methods.
    3. Use `Inline Function` on the original function.
    4. Apply `Change Function Declaration` on the extracted function to rename it to the original name. Or to a better name, if you can think of one.

* Example
    ```js
    function renderPerson(outStream, person) {
        outStream.write(`<p>${person.name}</p>\n`);
        renderPhoto(outStream, person.photo);
        emitPhotoData(outStream, person.photo);
    }

    function listRecentPhotos(outStream, photos) {
        photos
            .filter(p => p.date > recentDateCutoff())
            .forEach(p => {
                outStream.write("<div>\n");
                emitPhotoData(outStream, p);
                outStream.write("</div>\n");
            });
    }

    function emitPhotoData(outStream, photo) {
        outStream.write(`<p>title: ${photo.title}</p>\n`);
        outStream.write(`<p>date: ${photo.date.toDateString()}</p>\n`);
        outStream.write(`<p>location: ${photo.location}</p>\n`);
    }
    ```
    I need to modify the software so that listRecentPhotos renders the location information differently while renderPerson stays the same. To make this change easier, I’ll use Move Statements to Callers on the final line.
    ```js
    // 2. `Extract Function` on the code that will remain in emitPhotoData.
    function emitPhotoData(outStream, photo) {
        zztmp(outStream, photo);
        outStream.write(`<p>location: ${photo.location}</p>\n`);
    }

    function zztmp(outStream, photo) {
        outStream.write(`<p>title: ${photo.title}</p>\n`);
        outStream.write(`<p>date: ${photo.date.toDateString()}</p>\n`);
    }
    ```
    ```js
    // 3.1 Inline Function
    function renderPerson(outStream, person) {
        outStream.write(`<p>${person.name}</p>\n`);
        renderPhoto(outStream, person.photo);
        zztmp(outStream, person.photo);
        outStream.write(`<p>location: ${person.photo.location}</p>\n`);
    }

    function listRecentPhotos(outStream, photos) {
        photos
            .filter(p => p.date > recentDateCutoff())
            .forEach(p => {
                outStream.write("<div>\n");
                zztmp(outStream, p);
                outStream.write(`<p>location: ${p.location}</p>\n`);
                outStream.write("</div>\n");
            });
    }

    // 3.2 delete the outer emitPhotoData function, completing Inline Function
    ```
    ```js
    // 4. rename zztmp back to the original emitPhotoData name
    function renderPerson(outStream, person) {
        outStream.write(`<p>${person.name}</p>\n`);
        renderPhoto(outStream, person.photo);
        emitPhotoData(outStream, person.photo);
        outStream.write(`<p>location: ${person.photo.location}</p>\n`);
    }

    function listRecentPhotos(outStream, photos) {
        photos
            .filter(p => p.date > recentDateCutoff())
            .forEach(p => {
                outStream.write("<div>\n");
                emitPhotoData(outStream, p);
                outStream.write(`<p>location: ${p.location}</p>\n`);
                outStream.write("</div>\n");
            });
    }

    function emitPhotoData(outStream, photo) {
        outStream.write(`<p>title: ${photo.title}</p>\n`);
        outStream.write(`<p>date: ${photo.date.toDateString()}</p>\n`);
    }
    ```

## Replace Inline Code with Function Call

![](../Images/Refactor/8-replace-inline-code-with-function-call.jpg)

* Motivation

    Functions allow me to package up bits of behavior. This is useful for understanding—a named function can explain the purpose of the code rather than its mechanics. It’s also valuable to remove duplication

* Mechanics
    * Replace the inline code with a call to the existing function.
    * Test.

## Slide Statements

![](../Images/Refactor/8-slide-statements.jpg)

* Motivation

    Code is easier to understand when things that are related to each other appear together. If several lines of code access the same data structure, it’s best for them to be together rather than intermingled with code accessing other data structures.

    Usually, I move related code together as a preparatory step for another refactoring, often an `Extract Function`.

    In this case, I’m also helped by the fact that the code I’m moving over doesn’t have side effects either. Indeed, I can freely rearrange code that lacks side effects to my heart’s content, which is one of the reasons why wise programmers prefer to use side-effect-free code as much as possible.

* Mechanics
    1. Identify the target position to move the fragment to. Examine statements between source and target to see if there is interference for the candidate fragment. Abandon action if there is any interference.
        * A fragment cannot slide backwards earlier than any element it references is declared.
        * A fragment cannot slide forwards beyond any element that references it.
        * A fragment cannot slide over any statement that modifies an element it references.
        * A fragment that modifies an element cannot slide over any other element that references the modified element.
    2. Cut the fragment from the source and paste into the target position.
    3. Test.

## Split Loop

![](../Images/Refactor/8-split-loop.jpg)

* Motivation

    You often see loops that are doing two different things at once just because they can do that with one pass through a loop. But if you’re doing two different things in the same loop, then whenever you need to modify the loop you have to understand both things. By splitting the loop, you ensure you only need to understand the behavior you need to modify.

    Many programmers are uncomfortable with this refactoring, as it forces you to execute the loop twice. My reminder, as usual, is to separate refactoring from optimization (`Refactoring and Performance`).

* Mechanics
    1. Copy the loop.
    2. Identify and eliminate duplicate side effects.
    3. Test. When done, consider `Extract Function` on each loop.

* Example
    ```js
    //  calculates the total salary and youngest age
    let youngest = people[0] ? people[0].age : Infinity;
    let totalSalary = 0;
    for (const p of people) {
        if (p.age < youngest)
            youngest = p.age;
        totalSalary += p.salary;
    }

    return `youngestAge: ${youngest}, totalSalary: ${totalSalary}`;
    ```
    ```js
    // 1. Copy the loop.
    let youngest = people[0] ? people[0].age : Infinity;
    let totalSalary = 0;
    for (const p of people) {
        if (p.age < youngest)
            youngest = p.age;
        totalSalary += p.salary;
    }

    for (const p of people) {
        if (p.age < youngest)
            youngest = p.age;
        totalSalary += p.salary;
    }

    return `youngestAge: ${youngest}, totalSalary: ${totalSalary}`;
    ```
    ```js
    // 2. Identify and eliminate duplicate side effects.
    let youngest = people[0] ? people[0].age : Infinity;
    let totalSalary = 0;

    for (const p of people) {
        totalSalary += p.salary;
    }

    for (const p of people) {
        if (p.age < youngest)
            youngest = p.age;
    }

    return `youngestAge: ${youngest}, totalSalary: ${totalSalary}`;
    ```
    The point of Split Loop isn’t what it does on its own but what it sets up for the next move—and I’m usually looking to extract the loops into their own functions.
    ```js
    // 3.1 `Slide Statements` to reorganize the code a bit first
    let totalSalary = 0;
    for (const p of people) {
        totalSalary += p.salary;
    }

    let youngest = people[0] ? people[0].age : Infinity;
    for (const p of people) {
        if (p.age < youngest)
            youngest = p.age;
    }

    return `youngestAge: ${youngest}, totalSalary: ${totalSalary}`;
    ```
    ```js
    // 3.2 Extract Function
    return `youngestAge: ${youngestAge()}, totalSalary: ${totalSalary()}`;

    function totalSalary() {
    let totalSalary = 0;
        for (const p of people) {
            totalSalary += p.salary;
        }
        return totalSalary;
    }

    function youngestAge() {
        let youngest = people[0] ? people[0].age : Infinity;
        for (const p of people) {
            if (p.age < youngest)
                youngest = p.age;
        }
        return youngest;
    }
    ```
    ```js
    // 4. `Replace Loop with Pipeline` for the total salary,
    //  `Substitute Algorithm` for the youngest age.
    return `youngestAge: ${youngestAge()}, totalSalary: ${totalSalary()}`;

    function totalSalary() {
        return people.reduce((total,p) => total + p.salary, 0);
    }

    function youngestAge() {
        return Math.min(...people.map(p => p.age));
    }
    ```

## Replace Loop with Pipeline

![](../Images/Refactor/8-replace-loop-with-pipeline.jpg)


## Remove Dead Code

![](../Images/Refactor/8-remove-dead-code.jpg)

* Motivation

    When we put code into production, even on people’s devices, we aren’t charged by weight. A few unused lines of code don’t slow down our systems nor take up significant memory; indeed, decent compilers will instinctively remove them. But unused code is still a significant burden when trying to understand how the software works. It doesn’t carry any warning signs telling programmers that they can ignore this function as it’s never called any more, so they still have to spend time understanding what it’s doing and why changing it doesn’t seem to alter the output as they expected.

    I don’t worry that I may need it sometime in the future; should that happen, I have my version control system so I can always dig it out again.

    Commenting out dead code was once a common habit. This was useful in the days before version control systems were widely used, or when they were inconvenient.

# 9 Organizing Data

A value that’s used for different purposes is a breeding ground for confusion and bugs—so, when I see one, I use `Split Variable` to separate the usages. As with any program element, getting a variable’s name right is tricky and important, so `Rename Variable` is often my friend. But sometimes the best thing I can do with a variable is to get rid of it completely—with `Replace Derived Variable with Query`.

I often find problems in a code base due to a confusion between references and values, so I use `Change Reference to Value` and `Change Value to Reference` to change between these styles.

## Split Variable

* Motivation

    * Variables have various uses.
        * Some of these uses naturally lead to the variable being assigned to several times.
        * Loop variables change for each run of a loop.
        * Collecting variables store a value that is built up during the method.
        * Many other variables are used to hold the result of a long-winded bit of code for easy reference later.

    * These kinds of variables should be set only once. If they are set more than once, it is a sign that they have more than one responsibility within the method. Any variable with more than one responsibility should be replaced with multiple variables, one for each responsibility. Using a variable for two different things is very confusing for the reader.

*  Mechanics
    1. Change the name of the variable at its declaration and first assignment.
        * If the later assignments are of the form i = i + something, that is a collecting variable, so don’t split it. A collecting variable is often used for calculating sums, string concatenation, writing to a stream, or adding to a collection.
    2. If possible, declare the new variable as immutable.
    3. Change all references of the variable up to its second assignment.
    4. Test.
    5. Repeat in stages, at each stage renaming the variable at the declaration and changing references until the next assignment, until you reach the final assignment.

* Example
    ```js
    function distanceTravelled (scenario, time) {
        let result;
        let acc = scenario.primaryForce / scenario.mass;
        let primaryTime = Math.min(time, scenario.delay);
        result = 0.5 * acc * primaryTime * primaryTime;
        let secondaryTime = time - scenario.delay;
        if (secondaryTime > 0) {
            let primaryVelocity = acc * scenario.delay;
            acc = (scenario.primaryForce + scenario.secondaryForce) / scenario.mass;
            result += primaryVelocity * secondaryTime + 0.5 * acc * secondaryTime * secondaryTime;
        }
        return result;
    }
    ```
    acc has two responsibilities: one to hold the initial acceleration from the first force and another later to hold the acceleration from both forces. I want to split this variable.
    ```js
    // 1 & 2. changing the name of the variable and declaring the new name as const
    function distanceTravelled (scenario, time) {
        let result;
        const primaryAcceleration = scenario.primaryForce / scenario.mass;
        let primaryTime = Math.min(time, scenario.delay);
        result = 0.5 * primaryAcceleration * primaryTime * primaryTime;
        let secondaryTime = time - scenario.delay;
        if (secondaryTime > 0) {
            let primaryVelocity = primaryAcceleration * scenario.delay;
            let acc = (scenario.primaryForce + scenario.secondaryForce) / scenario.mass;
            result += primaryVelocity * secondaryTime + 0.5 * acc * secondaryTime * secondaryTime;
        }
        return result;
    }
    ```
    ```js
    // continue on the second assignment of the variable
    function distanceTravelled (scenario, time) {
        let result;
        const primaryAcceleration = scenario.primaryForce / scenario.mass;
        let primaryTime = Math.min(time, scenario.delay);
        result = 0.5 * primaryAcceleration * primaryTime * primaryTime;

        let secondaryTime = time - scenario.delay;
        if (secondaryTime > 0) {
            let primaryVelocity = primaryAcceleration * scenario.delay;
            const secondaryAcceleration = (scenario.primaryForce + scenario.secondaryForce) / scenario.mass;
            result += primaryVelocity * secondaryTime + 0.5 * secondaryAcceleration * secondaryTime * secondaryTime;
        }
        return result;
    }
    ```

* Example: Assigning to an Input Parameter
    ```js
    function discount (inputValue, quantity) {
        if (inputValue > 50) inputValue = inputValue - 2;
        if (quantity > 100) inputValue = inputValue - 1;
        return inputValue;
    }
    ```
    Here inputValue is used both to supply an input to the function and to hold the result for the caller.
    ```js
    function discount (inputValue, quantity) {
        let result = inputValue;
        if (inputValue > 50) result = result - 2;
        if (quantity > 100) result = result - 1;
        return result;
    }
    ```

## Rename Field

![](../Images/Refactor/9-split-variable.jpg)

* Motivation

    Names are important, and field names in record structures can be especially important when those record structures are widely used across a program.

    Data structures are the key to understanding what’s going on.

* Mechanics
    1. If the record has limited scope, rename all accesses to the field and test; no need to do the rest of the mechanics.
    2. If the record isn’t already encapsulated, apply `Encapsulate Record`.
    3. Rename the private field inside the object, adjust internal methods to fit.
    4. Test.
    5. If the constructor uses the name, apply `Change Function Declaration` to rename it.
    6. Apply `Rename Function` to the accessors.


* Example: Renaming a Field
    ```js
    const organization = {name: "Acme Gooseberries", country: "GB"};
    ```
    I want to change “name” to “title”.
    ```js
    // 2. `Encapsulate Record`
    class Organization {
        constructor(data) {
            this._name = data.name;
            this._country = data.country;
        }
        get name()    {return this._name;}
        set name(aString) {this._name = aString;}
        get country()    {return this._country;}
        set country(aCountryCode) {this._country = aCountryCode;}
    }

    const organization = new Organization({name: "Acme Gooseberries", country: "GB"});
    ```
    ```js
    // 3.1 defining a separate field `_title` and adjusting the constructor and accessors to use it.
    class Organization {
        constructor(data) {
            this._title = data.name;
            this._country = data.country;
        }
        get name()    {return this._title;}
        set name(aString) {this._title = aString;}
        get country()    {return this._country;}
        set country(aCountryCode) {this._country = aCountryCode;}
    }
    ```
    ```js
    // 3.2. add support for using “title” in the constructor
    class Organization {
        constructor(data) {
            this._title = (data.title !== undefined) ? data.title : data.name;
            this._country = data.country;
        }
        get name()    {return this._title;}
        set name(aString) {this._title = aString;}
        get country()    {return this._country;}
        set country(aCountryCode) {this._country = aCountryCode;}
    }
    ```
    ```js
    // change all constructor callers one-by-one to use the new name
    const organization = new Organization({title: "Acme Gooseberries", country: "GB"});
    ```
    ```js
    // 6. Rename Function name to title()
    class Organization {
        constructor(data) {
            this._title = data.title;
            this._country = data.country;
        }
        get title()    {return this._title;}
        set title(aString) {this._title = aString;}
        get country()    {return this._country;}
        set country(aCountryCode) {this._country = aCountryCode;}
    }
    ```

## Replace Derived Variable with Query

![](../Images/Refactor/9-replace-derived-variable-with-query.jpg)


* Motivation

    One of the biggest sources of problems in software is mutable data. Data changes can often couple together parts of code in awkward ways, with changes in one part leading to knock-on effects that are hard to spot.

    One way I can make a big impact is by removing any variables that I could just as easily calculate. A calculation often makes it clearer what the meaning of the data is, and it is protected from being corrupted when you fail to update the variable as the source data changes.

* Mechanics
    1. Identify all points of update for the variable. If necessary, use `Split Variable` to separate each point of update.
    2. Create a function that calculates the value of the variable.
    3. Use `Introduce Assertion` to assert that the variable and the calculation give the same result whenever the variable is used.
        * If necessary, use `Encapsulate Variable` to provide a home for the assertion.
    4. Test.
    5. Replace any reader of the variable with a call to the new function.
    6. Test.
    7. Apply `Remove Dead Code` to the declaration and updates to the variable.

* Example
    ```js
    class ProductionPlan {
        get production() {return this._production;}

        applyAdjustment(anAdjustment) {
            this._adjustments.push(anAdjustment);
            this._production += anAdjustment.amount;
        }
    }
    ```
    ```js
    // 3. Insert Assertion to insure the update
    get production() {
        assert(this._production === this.calculatedProduction);
        return this._production;
    }

    get calculatedProduction() {
        return this._adjustments.reduce((sum, a) => sum + a.amount, 0);
    }
    ```
    ```js
    // inline function
    get production() {
        return this._adjustments.reduce((sum, a) => sum + a.amount, 0);
    }

    applyAdjustment(anAdjustment) {
        this._adjustments.push(anAdjustment);
    }
    ```

* Example: More Than One Source
    ```js
    class ProductionPlan {
        constructor (production) {
            this._production = production;
            this._adjustments = [];
        }

        get production() {return this._production;}

        applyAdjustment(anAdjustment) {
            this._adjustments.push(anAdjustment);
            this._production += anAdjustment.amount;
        }
    }
    ```
    ```js
    // split variable
    class ProductionPlan {
        constructor (production) {
            this._initialProduction = production;
            this._productionAccumulator = 0;
            this._adjustments = [];
        }

        get production() {
            return this._initialProduction + this._productionAccumulator;
        }
    }
    ```
    ```js
    // insert assertion
    class ProductionPlan {
        get production() {
            assert(this._productionAccumulator === this.calculatedProductionAccumulator);
            return this._initialProduction + this._productionAccumulator;
        }

        get calculatedProductionAccumulator() {
            return this._adjustments.reduce((sum, a) => sum + a.amount, 0);
        }

        applyAdjustment(anAdjustment) {
            this._adjustments.push(anAdjustment);
            this._productionAccumulator += anAdjustment.amount;
        }
    }
    }
    ```

## Change Reference to Value

![](../Images/Refactor/9-change-reference-to-value.jpg)

* Motivation

    When I nest an object, or data structure, within another I can treat the inner object as a reference or as a value. The difference is most obviously visible in how I handle updates of the inner object’s properties. If I treat it as a reference, I’ll update the inner object’s property keeping the same inner object. If I treat it as a value, I will replace the entire inner object with a new one that has the desired property.

    If I treat a field as a value, I can change the class of the inner object to make it a Value Object [mf-vo]. Value objects are generally easier to reason about, particularly because they are immutable. In general, immutable data structures are easier to deal with.

* Mechanics
    1. Check that the candidate class is immutable or can become immutable.
    2. For each setter, apply `Remove Setting Method`.
    3. Provide a value-based equality method that uses the fields of the value object.
    4. Most language environments provide an overridable equality function for this purpose. Usually you must override a hashcode generator method as well.

* Example
    ```js
    class Person {
        constructor() {
            this._telephoneNumber = new TelephoneNumber();
        }

        get officeAreaCode()    {return this._telephoneNumber.areaCode;}
        set officeAreaCode(arg) {this._telephoneNumber.areaCode = arg;}
        get officeNumber()    {return this._telephoneNumber.number;}
        set officeNumber(arg) {this._telephoneNumber.number = arg;}
    }

    class TelephoneNumber {
        get areaCode()    {return this._areaCode;}
        set areaCode(arg) {this._areaCode = arg;}

        get number()    {return this._number;}
        set number(arg) {this._number = arg;}
    }
    ```
    ```js
    // 2. Remove Setting Method
    // add the two fields to the constructor and enhance the constructor to call the setters
    class TelephoneNumber {
        constructor(areaCode, number) {
            this._areaCode = areaCode;
            this._number = number;
        }
    }

    class Person {
        get officeAreaCode()    {return this._telephoneNumber.areaCode;}
        set officeAreaCode(arg) {
            this._telephoneNumber = new TelephoneNumber(arg, this.officeNumber);
        }

        get officeNumber()    {return this._telephoneNumber.number;}
        set officeNumber(arg) {
            this._telephoneNumber = new TelephoneNumber(this.officeAreaCode, arg);
        }
    }
    ```

## Change Value to Reference

![](../Images/Refactor/9-change-value-to-reference.jpg)

* Motivation

    The biggest difficulty in having physical copies of the same logical data occurs when I need to update the shared data. I then have to find all the copies and update them all. If I miss one, I’ll get a troubling inconsistency in my data. In this case, it’s often worthwhile to change the copied data into a single reference

# 10 Simplifying Conditional Logic

I regularly apply `Decompose Conditional` to complicated conditionals, and I use `Consolidate Conditional Expression` to make logical combinations clearer. I use `Replace Nested Conditional with Guard Clauses` to clarify cases where I want to run some pre-checks before my main processing. If I see several conditions using the same switching logic, it’s a good time to pull `Replace Conditional with Polymorphism` out the box.

A lot of conditionals are used to handle special cases, such as nulls; if that logic is mostly the same, then `Introduce Special Case`(often referred to as `Introduce Null Object`) can remove a lot of duplicate code. And, although I like to remove conditions a lot, if I want to communicate (and check) a program’s state, I find `Introduce Assertion` a worthwhile addition.


## Decompose Conditional

![](../Images/Refactor/10-decompose-conditional.jpg)

* Motivation

    One of the most common sources of complexity in a program is complex conditional logic.

    The problem usually lies in the fact that the code, both in the condition checks and in the actions, tells me what happens but can easily obscure why it happens.

    As with any large block of code, I can make my intention clearer by decomposing it and replacing each chunk of code with a function call named after the intention of that chunk.

* Mechanics
    * Apply `Extract Function` on the condition and each leg of the conditional.

* Example
    ```js
    // Suppose I’m calculating the charge for something that has separate rates for winter and summer:
    if (!aDate.isBefore(plan.summerStart) && !aDate.isAfter(plan.summerEnd))
        charge = quantity * plan.summerRate;
    else
        charge = quantity * plan.regularRate + plan.regularServiceCharge;
    ```
    ```js
    // extract functions
    if (isSummer())
        charge = quantity * plan.summerRate;
    else
        charge = quantity * plan.regularRate + plan.regularServiceCharge;

    function isSummer() {
        return !aDate.isBefore(plan.summerStart) && !aDate.isAfter(plan.summerEnd);
    }
    ```
    ```js
    // continue extract function
    if (isSummer())
        charge = doSummerCharge();
    else
        charge = doRegularCharge();

    function isSummer() {
        return !aDate.isBefore(plan.summerStart) && !aDate.isAfter(plan.summerEnd);
    }

    function doSummerCharge() {
        return quantity * plan.summerRate;
    }

    function doRegularCharge() {
        return quantity * plan.regularRate + plan.regularServiceCharge;
    }
    ```
    ```js
    // reformat the conditional using the ternary operator
    charge = isSummer() ? doSummerCharge() : doRegularCharge();

    function isSummer() {
        return !aDate.isBefore(plan.summerStart) && !aDate.isAfter(plan.summerEnd);
    }

    function doSummerCharge() {
        return quantity * plan.summerRate;
    }

    function doRegularCharge() {
        return quantity * plan.regularRate + plan.regularServiceCharge;
    }
    ```

## Consolidate Conditional Expression

![](../Images/Refactor/10-consolidate-conditional-expression.jpg)

* Motivation

    Sometimes, I run into a series of conditional checks where each check is different yet the resulting action is the same. When I see this, I use and and or operators to consolidate them into a single conditional check with a single result.
    Consolidating the conditional code is important for two reasons. 1. it makes it clearer by showing that I’m really making a single check that combines other checks. 2. it often sets me up for `Extract Function`.

* Mechanics
    * Ensure that none of the conditionals have any side effects.
        * If any do, use `Separate Query from Modifier` on them first.
    * Take two of the conditional statements and combine their conditions using a logical operator.
        * Sequences combine with or, nested if statements combine with and.
    * Test.
    * Repeat combining conditionals until they are all in a single condition.
    * Consider using `Extract Function` on the resulting condition.

* Example
    ```js
    function disabilityAmount(anEmployee) {
        if (anEmployee.seniority < 2) return 0;
        if (anEmployee.monthsDisabled > 12) return 0;
        if (anEmployee.isPartTime) return 0;
        // compute the disability amount
    }
    ```
    ```js
    function disabilityAmount(anEmployee) {
        if (isNotEligableForDisability())
            return 0;
        // compute the disability amount

        function isNotEligableForDisability() {
            return ((anEmployee.seniority < 2)
                || (anEmployee.monthsDisabled > 12)
                || (anEmployee.isPartTime));
        }
    }
    ```
* Example: Using ands
    ```js
    if (anEmployee.onVacation)
        if (anEmployee.seniority > 10)
            return 1;
    return 0.5;
    ```
    ```js
    if ((anEmployee.onVacation) && (anEmployee.seniority > 10))
        return 1;
    return 0.5;
    ```

## Replace Nested Conditional with Guard Clauses

![](../Images/Refactor/10-replace-nested-conditional-with-guard-clauses.jpg)

* Motivation

    Conditional expressions come in two styles. In the first style, both legs of the conditional are part of normal behavior, while in the second style, one leg is normal and the other indicates an unusual condition.

    If both are part of normal behavior, I use a condition with an if and an else leg. If the condition is an unusual condition, I check the condition and return if it’s true. This kind of check is often called a **guard clause**.

    The key point of Replace Nested Conditional with Guard Clauses is emphasis. If I’m using an if-then-else construct, I’m giving equal weight to the if leg and the else leg. This communicates to the reader that the legs are equally likely and important. Instead, the guard clause says, “This isn’t the core to this function, and if it happens, do something and get out.”

* Mechanics
    * Select outermost condition that needs to be replaced, and change it into a guard clause.
    * Test.
    * Repeat as needed.
    * If all the guard clauses return the same result, use `Consolidate Conditional Expression`.

* Example
    ```js
    function payAmount(employee) {
        let result;
        if (employee.isSeparated) {
            result = {amount: 0, reasonCode: "SEP"};
        } else {
            if (employee.isRetired) {
                result = {amount: 0, reasonCode: "RET"};
            } else {
                // logic to compute amount
                lorem.ipsum(dolor.sitAmet);
                consectetur(adipiscing).elit();
                sed.do.eiusmod = tempor.incididunt.ut(labore) && dolore(magna.aliqua);
                ut.enim.ad(minim.veniam);
                result = someFinalComputation();
            }
        }
        return result;
    }
    ```
    ```js
    function payAmount(employee) {
        if (employee.isSeparated)
            return {amount: 0, reasonCode: "SEP"};
        if (employee.isRetired)
            return {amount: 0, reasonCode: "RET"};

        // logic to compute amount
        lorem.ipsum(dolor.sitAmet);
        consectetur(adipiscing).elit();
        sed.do.eiusmod = tempor.incididunt.ut(labore) && dolore(magna.aliqua);
        ut.enim.ad(minim.veniam);
        return someFinalComputation();
    }
    ```

* Example: Reversing the Conditions
    ```js
    function adjustedCapital(anInstrument) {
        let result = 0;
        if (anInstrument.capital > 0) {
            if (anInstrument.interestRate > 0 && anInstrument.duration > 0) {
                result = (anInstrument.income / anInstrument.duration) * anInstrument.adjustmentFactor;
            }
        }
        return result;
    }
    ```
    ```js
    function adjustedCapital(anInstrument) {
        if (anInstrument.capital <= 0
            || anInstrument.interestRate <= 0
            || anInstrument.duration <= 0)
        {
            return 0;
        }
        return (anInstrument.income / anInstrument.duration) * anInstrument.adjustmentFactor;
    }
    ```

## Replace Conditional with Polymorphism

![](../Images/Refactor/10-replace-conditional-with-polymorphism.jpg)

* Motivation

    Complex conditional logic is one of the hardest things to reason about in programming, so I always look for ways to add structure to conditional logic.

    A common case for this is where I can form a set of types, each handling the conditional logic differently.

    Another situation is where I can think of the logic as a base case with variants. The base case may be the most common or most straightforward. I can put this logic into a superclass, I then put each variant case into a subclass, which I express with code that emphasizes its difference from the base case.

    Most of my conditional logic uses basic conditional statements—if/else and switch/case. But when I see complex conditional logic that can be improved as discussed above, I find polymorphism a powerful tool.

* Mechanics
    * If classes do not exist for polymorphic behavior, create them together with a factory function to return the correct instance.
    * Use the factory function in calling code.
    * Move the conditional function to the superclass.
        * If the conditional logic is not a self-contained function, use `Extract Function` to make it so.
    * Pick one of the subclasses. Create a subclass method that overrides the conditional statement method. Copy the body of that leg of the conditional statement into the subclass method and adjust it to fit.
    * Repeat for each leg of the conditional.
    * Leave a default case for the superclass method. Or, if superclass should be abstract, declare that method as abstract or throw an error to show it should be the responsibility of a subclass.

* Example
    ```js
    function plumages(birds) {
        return new Map(birds.map(b => [b.name, plumage(b)]));
    }

    function speeds(birds) {
        return new Map(birds.map(b => [b.name, airSpeedVelocity(b)]));
    }

    function plumage(bird) {
        switch (bird.type) {
        case 'EuropeanSwallow':
            return "average";
        case 'AfricanSwallow':
            return (bird.numberOfCoconuts > 2) ? "tired" : "average";
        case 'NorwegianBlueParrot':
            return (bird.voltage > 100) ? "scorched" : "beautiful";
        default:
            return "unknown";
        }
    }

    function airSpeedVelocity(bird) {
        switch (bird.type) {
        case 'EuropeanSwallow':
            return 35;
        case 'AfricanSwallow':
            return 40 - 2 * bird.numberOfCoconuts;
        case 'NorwegianBlueParrot':
            return (bird.isNailed) ? 0 : 10 + bird.voltage / 10;
        default:
            return null;
        }
    }
    ```
    ```js
    //  Combine Functions into Class
    function plumage(bird) {
        return new Bird(bird).plumage;
    }

    function airSpeedVelocity(bird) {
        return new Bird(bird).airSpeedVelocity;
    }

    class Bird {
        constructor(birdObject) {
            Object.assign(this, birdObject);
        }
        get plumage() {
            switch (this.type) {
            case 'EuropeanSwallow':
                return "average";
            case 'AfricanSwallow':
                return (this.numberOfCoconuts > 2) ? "tired" : "average";
            case 'NorwegianBlueParrot':
                return (this.voltage > 100) ? "scorched" : "beautiful";
            default:
                return "unknown";
            }
        }
        get airSpeedVelocity() {
            switch (this.type) {
            case 'EuropeanSwallow':
                return 35;
            case 'AfricanSwallow':
                return 40 - 2 * this.numberOfCoconuts;
            case 'NorwegianBlueParrot':
                return (this.isNailed) ? 0 : 10 + this.voltage / 10;
            default:
                return null;
            }
        }
    }
    ```
    ```js
    // add subclasses for each kind of bird, together with a factory function
    function plumages(birds) {
        return new Map(birds
            .map(b => createBird(b))
            .map(bird => [bird.name, bird.plumage]));
    }
    function speeds(birds) {
        return new Map(birds
            .map(b => createBird(b))
            .map(bird => [bird.name, bird.airSpeedVelocity]));
    }

    function createBird(bird) {
        Map<Bird> birdMap = {
            {'EuropeanSwallow', new EuropeanSwallow(bird)},
            {'AfricanSwallow', new AfricanSwallow(bird)},
            {'NorweigianBlueParrot', new NorwegianBlueParrot(bird)}
        };

        return birdMap[bird.type];
    }

    class Bird {
        get plumage() {
            return "unknow";
        }

        get airSpeedVelocity() {
            return null;
        }
    }

    class EuropeanSwallow extends Bird {
        get plumage() {
            return "average";
        }

        get airSpeedVelocity() {
            return 35;
        }
    }

    class AfricanSwallow extends Bird {
        get plumage() {
            return (this.numberOfCoconuts > 2) ? "tired" : "average";
        }

        get airSpeedVelocity() {
            return 40 - 2 * this.numberOfCoconuts;
        }
    }

    class NorwegianBlueParrot extends Bird {
        get plumage() {
            return (this.voltage > 100) ? "scorched" : "beautiful";
        }

        get airSpeedVelocity() {
            return (this.isNailed) ? 0 : 10 + this.voltage / 10;
        }
    }
    ```
## Introduce Special Case

* Motivation

    A common case of duplicated code is when many users of a data structure check a specific value, and then most of them do the same thing.
    A good mechanism for this is the Special Case pattern where I create a special-case element that captures all the common behavior. This allows me to replace most of the special-case checks with simple calls.

    If I need more behavior than simple values, I can create a special object with methods for all the common behavior. The special-case object can be returned by an encapsulating class, or inserted into a data structure with a transform.

* Mechanics
    1. Add a special-case check property to the subject, returning false.
    2. Create a special-case object with only the special-case check property, returning true.
    3. Apply `Extract Function` to the special-case comparison code. Ensure that all clients use the new function instead of directly comparing it.
    4. Introduce the new special-case subject into the code, either by returning it from a function call or by applying a transform function.
    5. Change the body of the special-case comparison function so that it uses the special-case check property.
    6. Test.
    7. Use `Combine Functions into Class` or `Combine Functions into Transform` to move all of the common special-case behavior into the new element.
        * Since the special-case class usually returns fixed values to simple requests, these may be handled by making the special case a literal record.
    8. Use `Inline Function` on the special-case comparison function for the places where it’s still needed.

* Example
    ```js
    class Site {
        get customer() {return this._customer;}
    }

    class Customer {
        get name()           {...}
        get billingPlan()    {...}
        set billingPlan(arg) {...}
        get paymentHistory() {...}
    }

    // client 1
    const aCustomer = site.customer;
    let customerName = (aCustomer === "unknown") ? "occupant" : aCustomer.name;

    // client 2
    const plan = (aCustomer === "unknown")
        ? registry.billingPlans.basic
        : aCustomer.billingPlan;

    // client 3
    if (aCustomer !== "unknown")
        aCustomer.billingPlan = newPlan;

    // client 4
    const weeksDelinquent = (aCustomer === "unknown")
        ? 0
        : aCustomer.paymentHistory.weeksDelinquentInLastYear;
    ```
    ```js
    // 1. Add a special-case check property to the subject, returning false.
    class Customer {
        get isUnknown() {return false;}
    }
    ```
    ```js
    // 2. Create a special-case object with only the special-case check property, returning true.
    class UnknownCustomer {
        get isUnknown() {return true;}
    }
    ```
    ```js
    // 3. Apply `Extract Function` to the special-case comparison code
    function isUnknown(arg) {
        if (!((arg instanceof Customer) || (arg === "unknown")))
            throw new Error(`investigate bad value: <${arg}>`);
        return (arg === "unknown");
    }
    ```
    ```js
    // 4. Introduce the new special-case subject into the code
    class Site {
        get customer() {
            return (this._customer === "unknown") ? new UnknownCustomer() : this._customer;
        }
    }
    ```
    ```js
    // 5. Change the body of the special-case comparison function so that it uses the special-case check property.
    function isUnknown(arg) {
        if (!(arg instanceof Customer || arg instanceof UnknownCustomer))
            throw new Error(`investigate bad value: <${arg}>`);
        return arg.isUnknown;
    }
    ```
    ```js
    // 7. `Combine Functions into Class` to take each client’s special-case
    class class UnknownCustomer {
        get name() {return "occupant";}
        get billingPlan()    {return registry.billingPlans.basic;}
        set billingPlan(arg) { /* ignore */ }
        get paymentHistory() {return new NullPaymentHistory();}
    }

    class NullPaymentHistory {
        get weeksDelinquentInLastYear() {return 0;}
    }

    // client 1
    const aCustomer = site.customer;
    const customerName = aCustomer.name;

    // client 2
    const plan = (isUnknown(aCustomer))
        ? registry.billingPlans.basic
        : aCustomer.billingPlan;

    // client 3
    if (!isUnknown(aCustomer))
        aCustomer.billingPlan = newPlan;

    ```
    ```js
    // 8. Use `Inline Function` on the special-case comparison function for the places where it’s still needed.
    // I may have 23 clients that use “occupant” for the name of an unknown customer,
    // but there’s always one that needs something different - "unknown occupant".
    const name = aCustomer.isUnknown ? "unknown occupant" : aCustomer.name;
    ```

## Introduce Assertion

![](../Images/Refactor/10-introduce-assertion.jpg)


* Motivation

    Often, sections of code work only if certain conditions are true.

    I find assertions to be a valuable form of communication—they tell the reader something about the assumed state of the program at this point of execution. I also find them handy for debugging, and their communication value means I’m inclined to leave them in once I’ve fixed the error I’m chasing.

* Mechanics

    When you see that a condition is assumed to be true, add an assertion to state it.

# 11 Refactoring APIs

Modules and their functions are the building blocks of our software. APIs are the joints that we use to plug them together. Making these APIs easy to understand and use is important but also difficult: I need to refactor them as I learn how to improve them.

If I see them combined, I use `Separate Query from Modifier` to tease them apart. I can unify functions that only vary due to a value with `Parameterize Function`. Some parameters, however, are really just a signal of an entirely different behavior and are best excised with `Remove Flag Argument`.

Data structures are often unpacked unnecessarily when passed between functions; I prefer to keep them together with `Preserve Whole Object`. Decisions on what should be passed as a parameter, and what can be resolved by the called function, are ones I often need to revisit with `Replace Parameter with Query` and `Replace Query with Parameter`.

A class is a common form of module. I prefer my objects to be as immutable as possible, so I use `Remove Setting Method` whenever I can. Often, when a caller asks for a new object, I need more flexibility than a simple constructor gives, which I can get by using `Replace Constructor with Factory Function`.

The last two refactorings address the difficulty of breaking down a particularly complex function that passes a lot of data around. I can turn that function into an object with `Replace Function with Command`, which makes it easier to use `Extract Function` on the function’s body. If I later simplify the function and no longer need it as a command object, I turn it back into a function with `Replace Command with Function`.

## Separate Query from Modifier

![](../Images/Refactor/11-separate-query-from-modifier.jpg)

* Motivation

    It is a good idea to clearly signal the difference between functions with side effects and those without. A good rule to follow is that any function that returns a value should not have observable side effects—the command-query separation.

    Note that I use the **phrase observable side effects**. A common optimization is to cache the value of a query in a field so that repeated calls go quicker. Although this changes the state of the object with the cache, the change is not observable. Any sequence of queries will always return the same results for each query.

* Mechanics
    1. Copy the function, name it as a query.
        * Look into the function to see what is returned. If the query is used to populate a variable, the variable’s name should provide a good clue.
    2. Remove any side effects from the new query function.
    3. Run static checks.
    4. Find each call of the original method. If that call uses the return value, replace the original call with a call to the query and insert a call to the original method below it. Test after each change.
    5. Remove return values from original.
    6. Test.

* Example
    ```js
    function alertForMiscreant (people) {
        for (const p of people) {
            if (p === "Don") {
                setOffAlarms();
                return "Don";
            }
            if (p === "John") {
                setOffAlarms();
                return "John";
            }
        }
        return "";
    }
    ```
    ```js
    // 1. copy the function, name it after the query aspect of the function.
    function findMiscreant (people) {
        for (const p of people) {
            if (p === "Don") {
                setOffAlarms();
                return "Don";
            }
            if (p === "John") {
                setOffAlarms();
                return "John";
            }
        }
        return "";
    }
    ```
    ```js
    // 2. remove the side effects from this new query.
    function findMiscreant (people) {
        for (const p of people) {
            if (p === "Don") {
                return "Don";
            }
            if (p === "John") {
                return "John";
            }
        }
        return "";
    }
    ```
    ```js
    // 4. Find each call of the original method
    const found = alertForMiscreant(people);

    // changed to
    const found = findMiscreant(people);
    alertForMiscreant(people);
    ```
    ```js
    // 5. Remove return values from original.
    function alertForMiscreant (people) {
        for (const p of people) {
            if (p === "Don") {
                setOffAlarms();
                return;
            }
            if (p === "John") {
                setOffAlarms();
                return;
            }
        }
        return;
    }
    ```
    Now I have a lot of duplication between the original modifier and the new query, so I can use `Substitute Algorithm` so that the modifier uses the query.
    ```js
    function alertForMiscreant (people) {
        if (findMiscreant(people) !== "")
            setOffAlarms();
    }
    ```

## Parameterize Function

![](../Images/Refactor/11-parameterize-function.jpg)

* Motivation

    If I see two functions that carry out very similar logic with different literal values, I can remove the duplication by using a single function with parameters for the different values.

* Mechanics
    1. Select one of the similar methods.
    2. Use `Change Function Declaration` to add any literals that need to turn into parameters.
    3. For each caller of the function, add the literal value.
    4. Test.
    5. Change the body of the function to use the new parameters. Test after each change.
    6. For each similar function, replace the call with a call to the parameterized function. Test after each one.

* Example
    ```js
    function tenPercentRaise(aPerson) {
        aPerson.salary = aPerson.salary.multiply(1.1);
    }
    function fivePercentRaise(aPerson) {
        aPerson.salary = aPerson.salary.multiply(1.05);
    }
    ```
    ```js
    function raise(aPerson, factor) {
        aPerson.salary = aPerson.salary.multiply(1 + factor);
    }
    ```
    ```js
    function baseCharge(usage) {
        if (usage < 0) return usd(0);
        const amount =
            bottomBand(usage) * 0.03
            + middleBand(usage) * 0.05
            + topBand(usage) * 0.07;
        return usd(amount);
    }

    function bottomBand(usage) {
        return Math.min(usage, 100);
    }

    function middleBand(usage) {
        return usage > 100 ? Math.min(usage, 200) - 100 : 0;
    }

    function topBand(usage) {
        return usage > 200 ? usage - 200 : 0;
    }
    ```
    ```js
    function baseCharge(usage) {
        if (usage < 0) return usd(0);
        const amount =
            bottomBand(usage) * 0.03
            + withinBand(usage, 100, 200) * 0.05
            + topBand(usage) * 0.07;
        return usd(amount);
    }

    function withinBand(usage, bottom, top) {
        return usage > bottom ? Math.min(usage, 200) - bottom : 0;
    }
    ```
    ```js
    function baseCharge(usage) {
        if (usage < 0) return usd(0);
        const amount =
            withinBand(usage, 0, 100) * 0.03
            + withinBand(usage, 100, 200) * 0.05
            + withinBand(usage, 200, Infinity) * 0.07;
        return usd(amount);
    }
    ```

## Remove Flag Argument

![](../Images/Refactor/11-remove-flag-argument.jpg)

* Motivation

    I dislike flag arguments because they complicate the process of understanding what function calls are available and how to call them. My first route into an API is usually the list of available functions, and flag arguments hide the differences in the function calls that are available. Once I select a function, I have to figure out what values are available for the flag arguments. Boolean flags are even worse since they don’t convey their meaning to the reader—in a function call, I can’t figure out what true means. It’s clearer to provide an explicit function for the task I want to do.

    Removing flag arguments doesn’t just make the code clearer—it also helps my tooling. Code analysis tools can now more easily see the difference between calling the premium logic and calling regular logic.

    Flag arguments can have a place if there’s more than one of them in the function, since otherwise I would need explicit functions for every combination of their values. But that’s also a signal of a function doing too much, and I should look for a way to create simpler functions that I can compose for this logic.

* Mechanics
    1. Create an explicit function for each value of the parameter.
        * If the main function has a clear dispatch conditional, use `Decompose Conditional` to create the explicit functions. Otherwise, create wrapping functions.
    2. For each caller that uses a literal value for the parameter, replace it with a call to the explicit function.

* Example
    ```js
    function deliveryDate(anOrder, isRush) {
        if (isRush) {
            let deliveryTime;
            if (["MA", "CT"]     .includes(anOrder.deliveryState)) deliveryTime = 1;
            else if (["NY", "NH"].includes(anOrder.deliveryState)) deliveryTime = 2;
            else deliveryTime = 3;
            return anOrder.placedOn.plusDays(1 + deliveryTime);
        }
        else {
            let deliveryTime;
            if (["MA", "CT", "NY"].includes(anOrder.deliveryState)) deliveryTime = 2;
            else if (["ME", "NH"] .includes(anOrder.deliveryState)) deliveryTime = 3;
            else deliveryTime = 4;
            return anOrder.placedOn.plusDays(2 + deliveryTime);
        }
    }
    ```
    ```js
    // 1. Decompose Conditional
    function deliveryDate(anOrder, isRush) {
        if (isRush) return rushDeliveryDate(anOrder);
        else        return regularDeliveryDate(anOrder);
    }
    function rushDeliveryDate(anOrder) {
        let deliveryTime;
        if (["MA", "CT"]     .includes(anOrder.deliveryState)) deliveryTime = 1;
        else if (["NY", "NH"].includes(anOrder.deliveryState)) deliveryTime = 2;
        else deliveryTime = 3;
        return anOrder.placedOn.plusDays(1 + deliveryTime);
    }
    function regularDeliveryDate(anOrder) {
        let deliveryTime;
        if (["MA", "CT", "NY"].includes(anOrder.deliveryState)) deliveryTime = 2;
        else if (["ME", "NH"] .includes(anOrder.deliveryState)) deliveryTime = 3;
        else deliveryTime = 4;
        return anOrder.placedOn.plusDays(2 + deliveryTime);
    }
    ```

* Example
    ```js
    function deliveryDate(anOrder, isRush) {
        let result;
        let deliveryTime;
        if (anOrder.deliveryState === "MA" || anOrder.deliveryState === "CT")
            deliveryTime = isRush? 1 : 2;
        else if (anOrder.deliveryState === "NY" || anOrder.deliveryState === "NH") {
            deliveryTime = 2;
            if (anOrder.deliveryState === "NH" && !isRush)
            deliveryTime = 3;
        }
        else if (isRush)
            deliveryTime = 3;
        else if (anOrder.deliveryState === "ME")
            deliveryTime = 3;
        else
            deliveryTime = 4;
        result = anOrder.placedOn.plusDays(2 + deliveryTime);
        if (isRush) result = result.minusDays(1);
        return result;
    }
    ```
    In this case, teasing out isRush into a top-level dispatch conditional is likely more work than I fancy. So instead, I can layer functions over the deliveryDate:
    ```js
    function rushDeliveryDate   (anOrder) {return deliveryDate(anOrder, true);}
    function regularDeliveryDate(anOrder) {return deliveryDate(anOrder, false);}
    ```

## Preserve Whole Object

![](../Images/Refactor/11-preserve-whole-object.jpg)

* Motivation

    If I see code that derives a couple of values from a record and then passes these values into a function, I like to replace those values with the whole record itself, letting the function body derive the values it needs.

    1. Passing the whole record handles change better should the called function need more data from the whole in the future—that change would not require me to alter the parameter list.
    2. It also reduces the size of the parameter list, which usually makes the function call easier to understand.
    3. If many functions are called with the parts, they often duplicate the logic that manipulates these parts—logic that can often be moved to the whole.

    The main reason I wouldn’t do this is if I don’t want the called function to have a dependency on the whole—which typically occurs when they are in different modules.

* Mechanics
    1. Create an empty function with the desired parameters.
    2. Give the function an easily searchable name so it can be replaced at the end.
    3. Fill the body of the new function with a call to the old function, mapping from the new parameters to the old ones.
    4. Run static checks.
    5. Adjust each caller to use the new function, testing after each change.
    6. This may mean that some code that derives the parameter isn’t needed, so can fall to `Remove Dead Code`.
    7. Once all original callers have been changed, use `Inline Function` on the original function.
    8. Change the name of the new function and all its callers.

* Example
    ```js
    const low = aRoom.daysTempRange.low;
    const high = aRoom.daysTempRange.high;
    if (!aPlan.withinRange(low, high))
        alerts.push("room temperature went outside range");

    class HeatingPlane {
        withinRange(bottom, top) {
            return (bottom >= this._temperatureRange.low) && (top <= this._temperatureRange.high);
        }
    }
    ```
    ```js
    // 1. create a new function with desired parameters
    // 2. naming the new function
    class HeatingPlan {
        xxNEWwithinRange(aNumberRange) {
        }

        withinRange(bottom, top) {
            return (bottom >= this._temperatureRange.low) && (top <= this._temperatureRange.high);
        }
    }
    ```
    ```js
    // 3. fill the body of the new function, mapping new and old parameters
    class HeatingPlan {
        xxNEWwithinRange(aNumberRange) {
            return this.withinRange(aNumberRange.low, aNumberRange.high);
        }

        withinRange(bottom, top) {
            return (bottom >= this._temperatureRange.low) && (top <= this._temperatureRange.high);
        }
    }
    ```
    ```js
    // 5. replace the callers wiht new function
    // 6. remove dead code of the caller
    // 7. inline new function
    class HeatingPlan {
        xxNEWwithinRange(aNumberRange) {
            return (aNumberRange.low >= this._temperatureRange.low) &&
                (aNumberRange.high <= this._temperatureRange.high);
        }

        withinRange(bottom, top) {
            return (bottom >= this._temperatureRange.low) && (top <= this._temperatureRange.high);
        }
    }
    ```
    ```js
    // 8. change the new function name, and remove the dead code
    class HeatingPlan {
        withinRange(aNumberRange) {
            return (aNumberRange.low >= this._temperatureRange.low) &&
                (aNumberRange.high <= this._temperatureRange.high);
        }
    }
    ```

* Example: A Variation to Create the New Function
    ```js
    const low = aRoom.daysTempRange.low;
    const high = aRoom.daysTempRange.high;
    if (!aPlan.withinRange(low, high))
        alerts.push("room temperature went outside range");
    ```
    I want to rearrange the code so I can create the new function by using `Extract Function` on some existing code. The caller code isn’t quite there yet, but I can get there by using `Extract Variable` a few times.
    ```js
    // disentangle the call to the old function from the conditional.
    const low = aRoom.daysTempRange.low;
    const high = aRoom.daysTempRange.high;
    const isWithinRange = aPlan.withinRange(low, high);
    if (!isWithinRange)
        alerts.push("room temperature went outside range");
    ```
    ```js
    // extract input parameters
    const tempRange = aRoom.daysTempRange;
    const low = tempRange.low;
    const high = tempRange.high;
    const isWithinRange = aPlan.withinRange(low, high);
    if (!isWithinRange)
        alerts.push("room temperature went outside range");
    ```
    ```js
    // extract function
    const tempRange = aRoom.daysTempRange;
    const isWithinRange = xxNEWwithinRange(aPlan, tempRange);
    if (!isWithinRange)
        alerts.push("room temperature went outside range");

    function xxNEWwithinRange(aPlan, tempRange) {
        const low = tempRange.low;
        const high = tempRange.high;
        const isWithinRange = aPlan.withinRange(low, high);
        return isWithinRange;
    }
    ```
    ```js
    // move function
    const tempRange = aRoom.daysTempRange;
    const isWithinRange = aPlan.xxNEWwithinRange(tempRange);
    if (!isWithinRange)
        alerts.push("room temperature went outside range");
    ```
    ```js
    class HeatingPlan {
        xxNEWwithinRange(tempRange) {
            const low = tempRange.low;
            const high = tempRange.high;
            const isWithinRange = this.withinRange(low, high);
            return isWithinRange;
        }
    }
    ```

## Replace Parameter with Query

![](../Images/Refactor/11-replace-parameter-with-query.jpg)

* Motivation

    The parameter list to a function should summarize the points of variability of that function, indicating the primary ways in which that function may behave differently. As with any statement in code, it’s good to avoid any duplication, and it’s easier to understand if the parameter list is short.

    By removing the parameter, I’m shifting the responsibility for determining the parameter value. When the parameter is present, determining its value is the caller’s responsibility; otherwise, that responsibility shifts to the function body.

    The most common reason to avoid Replace Parameter with Query is if removing the parameter adds an unwanted dependency to the function body—forcing it to access a program element that I’d rather it remained ignorant of.

    One thing to watch out for is if the function I’m looking at has referential transparency—that is, if I can be sure that it will behave the same way whenever it’s called with the same parameter values. Such functions are much easier to reason about and test, and I don’t want to alter them to lose that property. So I wouldn’t replace a parameter with an access to a mutable global variable.

* Mechanics
    1. If necessary, use `Extract Function` on the calculation of the parameter.
    2. Replace references to the parameter in the function body with references to the expression that yields the parameter. Test after each change.
    3. Use `Change Function Declaration` to remove the parameter.

* Example
    ```js
    get finalPrice() {
        const basePrice = this.quantity * this.itemPrice;
        let discountLevel;
        if (this.quantity > 100) discountLevel = 2;
        else discountLevel = 1;
        return this.discountedPrice(basePrice, discountLevel);
    }

    discountedPrice(basePrice, discountLevel) {
        switch (discountLevel) {
            case 1: return basePrice * 0.95;
            case 2: return basePrice * 0.9;
        }
    }
    ```
    ```js
    get finalPrice() {
        const basePrice = this.quantity * this.itemPrice;
        return this.discountedPrice(basePrice, this.discountLevel);
    }

    get discountLevel() {
        return (this.quantity > 100) ? 2 : 1;
    }
    ```
    ```js
    get finalPrice() {
        const basePrice = this.quantity * this.itemPrice;
        return this.discountedPrice(basePrice);
    }

    discountedPrice(basePrice) {
        switch (this.discountLevel) {
            case 1: return basePrice * 0.95;
            case 2: return basePrice * 0.9;
        }
    }
    ```

## Replace Query with Parameter

![](../Images/Refactor/11-replace-query-with-parameter.jpg)

* Motivation
    When looking through a function’s body, I sometimes see references to something in the function’s scope that I’m not happy with. This might be a reference to a global variable, or to an element in the same module that I intend to move away. To resolve this, I need to replace the internal reference with a parameter, shifting the responsibility of resolving the reference to the caller of the function.

    Most of these cases are due to my wish to alter the dependency relationships in the code—to make the target function no longer dependent on the element I want to parameterize.

    It’s easier to reason about a function that will always give the same result when called with same parameter values—this is called **referential transparency**. If a function accesses some element in its scope that isn’t referentially transparent, then the containing function also lacks referential transparency.

    But Replace Query with Parameter isn’t just a bag of benefits. By moving a query to a parameter, I force my caller to figure out how to provide this value. This complicates life for callers of the functions, and my usual bias is to design interfaces that make life easier for their consumers. In the end, it boils down to allocation of responsibility around the program, and that’s a decision that’s neither easy nor immutable—which is why this refactoring (and its inverse) is one that I need to be very familiar with.

* Mechanics
    1. Use `Extract Variable` on the query code to separate it from the rest of the function body.
    2. Apply `Extract Function` to the body code that isn’t the call to the query.
    3. Give the new function an easily searchable name, for later renaming.
    4. Use `Inline Variable` to get rid of the variable you just created.
    5. Apply `Inline Function` to the original function.
    6. Rename the new function to that of the original.

* Example
    ```js
    class HeatingPlan {
        get targetTemperature() {
            if      (thermostat.selectedTemperature >  this._max) return this._max;
            else if (thermostat.selectedTemperature <  this._min) return this._min;
            else return thermostat.selectedTemperature;
        }
    }

    // client
    if      (thePlan.targetTemperature > thermostat.currentTemperature) setToHeat();
    else if (thePlan.targetTemperature < thermostat.currentTemperature) setToCool();
    else setOff();
    ```
    I might be more concerned about how the targetTemperature function has a dependency on a global thermostat object. I can break this dependency by moving it to a parameter.
    ```js
    // 1. Extract Variable
    get targetTemperature() {
        const selectedTemperature = thermostat.selectedTemperature;
        if      (selectedTemperature >  this._max) return this._max;
        else if (selectedTemperature <  this._min) return this._min;
        else return selectedTemperature;
    }
    ```
    ```js
    // 2. Extract Function
    get targetTemperature() {
        const selectedTemperature = thermostat.selectedTemperature;
        return this.xxNEWtargetTemperature(selectedTemperature);
    }

    xxNEWtargetTemperature(selectedTemperature) {
        if      (selectedTemperature >  this._max) return this._max;
        else if (selectedTemperature <  this._min) return this._min;
        else return selectedTemperature;
    }
    ```
    ```js
    // 4. Inline Variable
    get targetTemperature() {
        return this.xxNEWtargetTemperature(thermostat.selectedTemperature);
    }
    ```
    ```js
    // 5. Inline Function
    if (thePlan.xxNEWtargetTemperature(thermostat.selectedTemperature) > thermostat.currentTemperature)
        setToHeat();
    else if (thePlan.xxNEWtargetTemperature(thermostat.selectedTemperature) < thermostat.currentTemperature)
        setToCool();
    else
        setOff();
    ```
    ```js
    // 6. Rename Function
    if (thePlan.targetTemperature(thermostat.selectedTemperature) > thermostat.currentTemperature)
        setToHeat();
    else if (thePlan.targetTemperature(thermostat.selectedTemperature) < thermostat.currentTemperature)
        setToCool();
    else
        setOff();
    ```
    ```js
    class HeatingPlan {
        targetTemperature(selectedTemperature) {
            if      (selectedTemperature >  this._max) return this._max;
            else if (selectedTemperature <  this._min) return this._min;
            else return selectedTemperature;
        }
    }
    ```


## Remove Setting Method

![](../Images/Refactor/11-remove-setting-method.jpg)

* Motivation

    Providing a setting method indicates that a field may be changed. If I don’t want that field to change once the object is created, I don’t provide a setting method.

    * There’s a couple of common cases where this comes up.
        1. One is where people always use accessor methods to manipulate a field, even within constructors.
        2. Object is created by clients using creation script rather than by a simple constructor call.

* Mechanics
    1. If the value that’s being set isn’t provided to the constructor, use `Change Function Declaration` to add it. Add a call to the setting method within the constructor.
        * If you wish to remove several setting methods, add all their values to the constructor at once. This simplifies the later steps.
    2. Remove each call of a setting method outside of the constructor, using the new constructor value instead. Test after each one.
        * If you can’t replace the call to the setter by creating a new object (because you are updating a shared reference object), abandon the refactoring.
    3. Use `Inline Function` on the setting method. Make the field immutable if possible.
    4. Test.

## Replace Constructor with Factory Function

![](../Images/Refactor/11-replace-constructor-with-factory-function.jpg)

* Motivation
    * Constructors often come with awkward limitations that aren’t there for more general functions:
        1. A Java constructor must return an instance of the class it was called with, which means I can’t replace it with a subclass or proxy depending on the environment or parameters.
        2. Constructor naming is fixed, which makes it impossible for me to use a name that is clearer than the default
        3. Constructors often require a special operator to invoke (“new” in many languages) which makes them difficult to use in contexts that expect normal functions.
    * A factory function suffers from no such limitations. It will likely call the constructor as part of its implementation, but I can freely substitute something else.

* Mechanics
    1. Create a factory function, its body being a call to the constructor.
    2. Replace each call to the constructor with a call to the factory function.
    3. Test after each change.
    4. Limit the constructor’s visibility as much as possible.

* Example
    ```js
    class Employee {
        constructor (name, typeCode) {
            this._name = name;
            this._typeCode = typeCode;
        }
        get name() {return this._name;}
        get type() {
            return Employee.legalTypeCodes[this._typeCode];
        }
        static get legalTypeCodes() {
            return {"E": "Engineer", "M": "Manager", "S": "Salesman"};
        }
    }
    ```
    ```js
    // 1. create factory methods
    function createEmployee(name, typeCode) {
        return new Employee(name, typeCode);
    }
    ```
## Replace Function with Command

![](../Images/Refactor/11-replace-function-with-command.jpg)

* Motivation

    A command offers a greater flexibility for the control and expression of a function than the plain function mechanism. Commands can have complimentary operations, such as undo. I can provide methods to build up their parameters to support a richer lifecycle. I can build in customizations using inheritance and hooks.

    We must not forget that this flexibility, as ever, comes at a price paid in complexity. So, given the choice between a first-class function and a command, I’ll pick the function 95% of the time. I only use a command when I specifically need a facility that simpler approaches can’t provide.

    There are still times when a command is the right tool for the job. One of these cases is breaking up a complex function so I can better understand and modify it.

* Mechanics
    1. Create an empty class for the function. Name it based on the function.
    2. Use `Move Function` to move the function to the empty class.
        * Keep the original function as a forwarding function until at least the end of the refactoring.
        * Follow any convention the language has for naming commands. If there is no convention, choose a generic name for the command’s execute function, such as “execute” or “call”.
    3. Consider making a field for each argument, and move these arguments to the constructor.

* Example
    ```js
    function score(candidate, medicalExam, scoringGuide) {
        let result = 0;
        let healthLevel = 0;
        let highMedicalRiskFlag = false;

        if (medicalExam.isSmoker) {
            healthLevel += 10;
            highMedicalRiskFlag = true;
        }
        let certificationGrade = "regular";
        if (scoringGuide.stateWithLowCertification(candidate.originState)) {
            certificationGrade = "low";
            result -= 5;
        }
        // lots more code like this
        result -= Math.max(healthLevel - 5, 0);
        return result;
    }
    ```
    ```js
    // 1. create a calss
    // 2. Move Function
    function score(candidate, medicalExam, scoringGuide) {
        return new Scorer().execute(candidate, medicalExam, scoringGuide);
    }

    class Scorer {
        execute (candidate, medicalExam, scoringGuide) {
            let result = 0;
            let healthLevel = 0;
            let highMedicalRiskFlag = false;

            if (medicalExam.isSmoker) {
                healthLevel += 10;
                highMedicalRiskFlag = true;
            }
            let certificationGrade = "regular";
            if (scoringGuide.stateWithLowCertification(candidate.originState)) {
                certificationGrade = "low";
                result -= 5;
            }
            // lots more code like this
            result -= Math.max(healthLevel - 5, 0);
            return result;
        }
    }
    ```
    ```js
    // move paramter to constructor
    class Scorer {
        constructor(candidate, medicalExam, scoringGuide){
            this._candidate = candidate;
            this._medicalExam = medicalExam;
            this._scoringGuide = scoringGuide;

            this._result = 0;
            this._healthLevel = 0;
            this._highMedicalRiskFlag = false;
        }

        execute () {
            if (this._medicalExam.isSmoker) {
                this._healthLevel += 10;
                this._highMedicalRiskFlag = true;
            }
            this._certificationGrade = "regular";
            if (this._scoringGuide.stateWithLowCertification(this._candidate.originState)) {
                this._certificationGrade = "low";
                this._result -= 5;
            }
            // lots more code like this
            this._result -= Math.max(this._healthLevel - 5, 0);
            return this._result;
        }
    }
    ```
    ```js
    // Extract Function: scoreSmoking()
    // move all the function’s state to the command object
    execute () {
        this._result = 0;
        this._healthLevel = 0;
        this._highMedicalRiskFlag = false;

        this.scoreSmoking();
        this._certificationGrade = "regular";
        if (this._scoringGuide.stateWithLowCertification(this._candidate.originState)) {
            this._certificationGrade = "low";
            this._result -= 5;
        }
        // lots more code like this
        this._result -= Math.max(this._healthLevel - 5, 0);
        return this._result;
    }

    scoreSmoking() {
        if (this._medicalExam.isSmoker) {
            this._healthLevel += 10;
            this._highMedicalRiskFlag = true;
        }
    }


## Replace Command with Function

![](../Images/Refactor/11-replace-command-with-function.jpg)

* Motivation

    Command objects provide a powerful mechanism for handling complex computations. They can easily be broken down into separate methods sharing common state through the fields; they can be invoked via different methods for different effects; they can have their data built up in stages. But that power comes at a cost. Most of the time, I just want to invoke a function and have it do its thing. If that’s the case, and the function isn’t too complex, then a command object is more trouble than its worth and should be turned into a regular function.
* Mechanics
    1. Apply `Extract Function` to the creation of the command and the call to the command’s execution method.
        * This creates the new function that will replace the command in due course.
    2. For each method called by the command’s execution method, apply `Inline Function`.
        * If the supporting function returns a value, use `Extract Variable` on the call first and then `Inline Function`.
    3. Use `Change Function Declaration` to put all the parameters of the constructor into the command’s execution method instead.
    4. For each field, alter the references in the command’s execution method to use the parameter instead. Test after each change.
    5. Inline the constructor call and command’s execution method call into the caller (which is the replacement function).
    6. Test.
    7. Apply `Remove Dead Code` to the command class.

* Example
    ```js
    class ChargeCalculator {
        constructor (customer, usage, provider){
            this._customer = customer;
            this._usage = usage;
            this._provider = provider;
        }
        get baseCharge() {
            return this._customer.baseRate * this._usage;
        }
        get charge() {
            return this.baseCharge + this._provider.connectionCharge;
        }
    }
    ```
    ```js
    // 1. Extract Function
    function charge(customer, usage, provider) {
        return new ChargeCalculator(customer, usage, provider).charge;
    }
    ```
    ```js
    // 2.1 Inline Variable: baseCharge
    class ChargeCalculator {
        get baseCharge() {
            return this._customer.baseRate * this._usage;
        }
        get charge() {
            const baseCharge = this.baseCharge;
            return baseCharge + this._provider.connectionCharge;
        }
    }

    // 2.2 Inline Function
    class ChargeCalculator {
        get charge() {
            const baseCharge = this._customer.baseRate * this._usage;
            return baseCharge + this._provider.connectionCharge;
        }
    }
    ```
    ```js
    // 3. Change Funciton Declaration: move parameters from contructor to function
    class ChargeCalculator {
        constructor() {

        }

        function charge(customer, usage, provider) {
            const baseCharge = customer.baseRate * usage;
            return baseCharge + provider.connectionCharge;
        }
    }
    ```
    ```js
    // 4. Inline Function
    function charge(customer, usage, provider) {
        const baseCharge = customer.baseRate * usage;
        return baseCharge + provider.connectionCharge;
    }
    ```

# 12 Dealing with Inheritance

## Pull Up Method

* Exmaple
    ```js
    ```
    ```js
    ```
    ```js
    ```
    ```js
    ```
    ```js
    ```
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
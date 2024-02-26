# 1 Building Abstractions with Procedures

## 1.1 Elements of Programming
### 1.1.1 Expressions
### 1.1.2 NamingandtheEnvironment
### 1.1.3 EvaluatingCombinations
### 1.1.4 CompoundProcedures
### 1.1.5 Substitution Model for Procedure Application
### 1.1.6 Conditional Expressions and Predicates
### 1.1.7 Example: Square Roots by Newton’s Method
### 1.1.8 Procedures as Black-Box Abstractions
## 1.2 Procedures and the Processes y Generate
### 1.2.1 Linear Recursion and Iteration
### 1.2.2 TreeRecursion
### 1.2.3 OrdersofGrowth
### 1.2.4 Exponentiation
### 1.2.5 GreatestCommonDivisors
### 1.2.6 Example: Testing for Primality
## 1.3 Formulating Abstractions withHigher-OrderProcedures
### 1.3.1 ProceduresasArguments
### 1.3.2 Constructing Procedures Using lambda
### 1.3.3 ProceduresasGeneralMethods
### 1.3.4 ProceduresasReturnedValues

# 2 Building Abstractions with Data
## 2.1 IntroductiontoDataAbstraction
### 2.1.1 Example: Arithmetic Operations forRationalNumbers
### 2.1.2 AbstractionBarriers
### 2.1.3 WhatIsMeantbyData?
### 2.1.4 Extended Exercise: Interval Arithmetic
## 2.2 Hierarchical Data and the Closure Property
### 2.2.1 RepresentingSequences
### 2.2.2 HierarchicalStructures
### 2.2.3 Sequences as Conventional Interfaces
### 2.2.4 Example:APictureLanguage
## 2.3 SymbolicData
### 2.3.1 Quotation
### 2.3.2 Example: Symbolic Differentiation
### 2.3.3 Example:RepresentingSets
### 2.3.4 Example: Huffman Encoding Trees
## 2.4 Multiple Representations for Abstract Data
### 2.4.1 Representations for Complex Numbers
### 2.4.2 Taggeddata
### 2.4.3 Data-Directed Programming and Additivity
## 2.5 SystemswithGenericOperations
### 2.5.1 Generic Arithmetic Operations
### 2.5.2 Combining Data of Different Types
### 2.5.3 Example:SymbolicAlgebra

# 3 Modularity, Objects, and State
## 3.1 AssignmentandLocalState
### 3.1.1 LocalStateVariables
### 3.1.2 Benefits of Introducing Assignment
### 3.1.3 Costs of Introducing Assignment
## 3.2 EnvironmentModelofEvaluation
### 3.2.1 RulesforEvaluation
### 3.2.2 ApplyingSimpleProcedures
### 3.2.3 Frames as the Repository of Local State
### 3.2.4 InternalDefinitions
## 3.3 ModelingwithMutableData
### 3.3.1 MutableListStructure
### 3.3.2 Representing Queues
### 3.3.3 RepresentingTables
### 3.3.4 ASimulatorforDigitalCircuits
### 3.3.5 PropagationofConstraints
## 3.4 Concurrency:TimeIsoftheEssenc3
### 3.4.1 Nature of Time in Concurrent Systems
### 3.4.2 Mechanisms for Controlling Concurrency
## 3.5 Streams
### 3.5.1 StreamsAreDelayedLists
### 3.5.2 InfiniteStreams
### 3.5.3 Exploiting the Stream Paradigm
### 3.5.4 Streams and Delayed Evaluation
### 3.5.5 Modularity of Functional Programs andModularityofObjects

# 4 Metalinguistic Abstraction
## 4.1 MetacircularEvaluator
### 4.1.1 CoreoftheEvaluator
### 4.1.2 RepresentingExpressions
### 4.1.3 EvaluatorDataStructures
### 4.1.4 Running the Evaluator as a Program
### 4.1.5 DataasPrograms
### 4.1.6 InternalDefinitions
### 4.1.7 Separating Syntactic Analysis from Execution
## 4.2 Variations on a Scheme — Lazy Evaluation
### 4.2.1 Normal Order and Applicative Order
### 4.2.2 An Interpreter with Lazy Evaluation
### 4.2.3 StreamsasLazyLists
## 4.3 Variations on a Scheme — Nondeterministic Computing
### 4.3.1 AmbandSearch
### 4.3.2 Examples of Nondeterministic Programs
### 4.3.3 Implementing the amb Evaluator
## 4.4 LogicProgramming
### 4.4.1 Deductive Information Retrieval

### 4.4.2 How the Query System Works
### 4.4.3 Is Logic Programming Mathematical Logic?
### 4.4.4 Implementing the Query System
#### 4.4.4.1 Driver Loop and Instantiation
#### 4.4.4.2 Evaluator
#### 4.4.4.3 Finding Assertions by Pattern Matching
#### 4.4.4.4 RulesandUnification
#### 4.4.4.5 Maintaining the Data Base
#### 4.4.4.6 StreamOperations
#### 4.4.4.7 Query Syntax Procedures
#### 4.4.4.8 FramesandBindings

# 5 Computing with Register Machines
## 5.1 DesigningRegisterMachines
### 5.1.1 A Language for Describing Register Machines
### 5.1.2 AbstractioninMachineDesign
### 5.1.3 Subroutines
### 5.1.4 Using a Stack to Implement Recursion
### 5.1.5 InstructionSummary
## 5.2 ARegister-MachineSimulator
### 5.2.1 MachineModel
### 5.2.2 Assembler
### 5.2.3 Generating Execution Procedures forInstructions
### 5.2.4 Monitoring Machine Performance
## 5.3 Storage Allocation and Garbage Collection
### 5.3.1 MemoryasVectors
### 5.3.2 Maintaining the Illusion of Infinite Memory

## 5.4 Explicit-ControlEvaluator
### 5.4.1 Core of the Explicit-Control Evaluator
### 5.4.2 Sequence Evaluation and Tail Recursion
### 5.4.3 Conditionals, Assignments, and Definitions
### 5.4.4 RunningtheEvaluator
## 5.5 Compilation

# Algorithm = Logic + Control

* An algorithm can be regarded as consisting of a logic component, which specifies the knowledge to be used in solving problems, and a control component, which determines the problem-solving strategies by means of which that knowledge is used. The logic component determines the meaning of the algorithm whereas the control component only affects its effkiency.

* The relationship of equivalence between algorithms, be- cause they have the same logic, is the basis for using logical analysis to improve the efficiency of an algorithm by retaining its logic but improving the way it is used.

* The logic component ex-presses the knowledge which can be used in solving problems and the control component determines the way in which that knowledge can be used.

* A typical Horn clause problem has the form of
    * a set of clauses which defines a problem domain and
    * a theorem which consists of
        * hypotheses repre- sented by assertions A1<--- ..... An<--- and
        * a conclusion which is negated and represented by a denial <---B1..... Bn.

# The Relationship Between Logic and Control

* Dif- ferent control strategies for the same logical representa- tion generate different behaviors. However, information about a problem-domain can be represented in logic in different ways. Alternative representations can have a more significant effect on the efficiency of an algorithm than alternative control strategies for the same represen- tation.

* The logic component defines the problem- domain-specific part of an algorithm. It not only deter- mines the meaning of the algorithm but also influences the way the algorithm behaves. The control component specifies the problem-solving strategy. It affects the be- havior of the algorithm without affecting its meaning.

* In general, the higher the level of the programming language and the less advanced the level of the program- mer, the more the system needs to assume responsibility for efficiency and to exercise control over the use of the information which it is given.

* Greater efficiency can often be achieved when the programmer is able to communicate control information to the computer.

# Data Structures

* In a well-structured program it is desirable to separate data structures from the procedures which interrogate and manipulate them. Such separation means that the representation of data structures can be altered without altering the higher level procedures.

* Comparing the original data-structure-free definition with the new data-structure-dependent one, we notice another advantage of data-structure-independence: the fact that, with well-chosen names for the interfacing procedures, data-structure-independent programs are virtually self-documenting.

# Top-Down Execution of Procedure Calls

* The arguments for separating logic and control are like the ones for separating procedures and data struc- tures. When procedures are separated from data struc- tures, it is possible to distinguish (in the procedures) what functions the data structures fulfill from the manner in which the data structures fulfill them. When logic is separated from control, it is possible to distinguish (in the logic) what the program does from how the program does it (in the control). In both cases it is more obvious what the program does, and therefore it is easier to determine whether it correctly does what it is intended to do.

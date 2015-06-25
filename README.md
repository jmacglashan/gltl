# gltl

## Compiling and running
Compiling is performed using ant, or alternatively, you can import the code into your favorite IDE (I prefer IntelliJ). If you do not have ant installed, you can get it from http://ant.apache.org/bindownload.cgi or you can install it with a command-line package manger like apt-get (linux) or mac port (mac).

Assuming you have ant installed, to compile and run from the command line, first cd into the directory. Then compile with:

`ant`

Then you can run the simple demo with

`java -cp lib/*:build gltl.demo.SimpleDemo`

This will run with formula 4 by default. If you want to try a different formula add the formula number as a command line argument; for example

`java -cp lib/*:build gltl.demo.SimpleDemo 2`

## About the code
The compiler DomainGenerator code is 
`src/gltl/compiler/GLTLCompiler.java`.
The compiler can take any arbitrary BURALP domain as input for the environment domain (see SimpleDemo to see it in action). Currently, the compiler does not actually compile a GLTL formula and instead uses Min's hard coded task transition dynamics for formula's specified by the numebr 1-4. However, the GLTLCompiler class is designed around the notion that it will be extended so that it takes as input an arbitrary GLTL formula as well as a mapping from symbols in the formula to environment state propositional functions that can be evaluated and then will generate the TASK MDP accordingly. Primarily, this will require rewriting the method `getTaskTransitions(State s, State nextEnvState)`, which should return a distribution over the next task MDP state values (and again, is currently hardcoded for formulas 1-4). I imagine that the `generateRewardFunction()` and `generateTerminalFunction()` methods will also need to be defined so that they are arbitrary to any GLTL formula that is input.

The previous example code that Min wrote is also included in this repo. It is in the file `src/gltl/prototype/EnvironmentMDP.java`.

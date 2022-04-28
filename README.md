# Multi-Agent-Path-Finding

# Problem Statement

In a factory setting, usage of robots to automate tasks like pick up and delivery are quite prevalent in recent times. In such scenarios, the collision of the robots is undesirable and might lead to higher maintenance costs. Furthermore, in the given problem statement, the robot has to “pick up” a task/item from the pick up (source) location, after starting in its initial start position and then eventually after “dropping” the task/item to a drop location (destination), the robot has to finally move to its end position. There are obstacles as well where the robot cannot move and the source and destination locations can accommodate multiple robots simultaneously.

In this project I focused on implementing a single- path solver, namely space-time A*, and parts of two MAPF solvers, namely prioritized planning, Conflict-Based Search (CBS).



# Used Algorithms

## CBS

As explained by the creators of this algorithm - "CBS (Conflict Based Search) is a two-level algorithm that does not convert the problem into the single ‘joint agent’ model. 

## A-star
In order to find the shortest path between an initial and Final state.



# Requirements

Python3 and above


# Installation & Usage

* Running the code

python3 [--instance <path>] [--solver <solver>] [--batch]

* --batch will run all instances available in /instances
  
* The following are options for --solver

* CBS the original CBS algorithm



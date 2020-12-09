# Experimental robotic assignment 1

## Introduction

This assignment target is to build an ROS architecture to implement a robot, simulating a pet, that
interact with a human and moves in a discrete 2D environment.  
The pet has three states representing behaviours, that are simulated as a finite state machine with 3 states:  
* Play 
* Sleep 
* Normal  
  
These states determine the way the robot act inside the grid, whether moving randomly as in normal, going to targets determined by the user or simply sleeping in the home position.  

The robot will change between states randomly, eccept for play, which is received by the user.  

## Software Architecture

The software architecture consists in a pipeline, starting from the command generation and arriving to the pet simulation in turtlesim.  
Here we show the architecture image:

\image html Components_diagram.PNG
<center>Component Diagram</center>

The main components are:

* Random command generator(Command_giver.py)- send a string representing the concatenation of one or more commands of the form: 'play'(to start the play state),'point to x y'(to simulate the pointing gesture to x y),'go to x y'(to simulate the voice command to x y)  
Pet Interpreter(Pet_logic.py)- to interpret the string commands and translate them to a command list
* Pet behaviours(Pet_behaviours.py)- that simulate behaviours as a finite state, in the already mentioned states  
* Turtle simulation- that represents the position of the robot in the map  
* User- may or may not be present and provides the same type of messages that the Command_giver provides, adding also symbolical location such as "home" and "owner"  

Starting from the Command_giver, it is a publisher, with String type message, that transmit a series of 1 to 5 commands as the one discussed with a conjunction of an "and".

Talking about the Pet interpreter, this component subscribes to the command generator topic and provides a service with share the value of the command string composed by integers and devided by the '|' character.

Now, we can discuss the finite state machine. This, can be described by the following image:

\image html Finite_state_machines.PNG
<center>Finite state machine diagram</center>
While the 'Sleep' and 'Normal' state are quite simple in nature (containing just an infinite loop to sleep and to roam respectively), the 'Play' state is quite more complex in nature, having the following structure:
\image html Play_behaviour_flowchart.PNG
<center>Play behaviour flowchart</center>

And finally the simulator node, which was not implemented by the outhors, but is the GUI that demonstrate the position of the robot during the pet activity.

## Installation and running procedure

* Download the package from the github repository
* Set the package in the src folder of the catkin workspace
* With the shell, get into the folder and run 
 ```sh
	chmod +x launcher.sh
 ```
* Write 
```sh
	./launcher.sh
 ```
* You can look at the blue-background screen to obtain the graphical representation of the robot location, while, on the shell, the state transition and the command from the command menager are displayed
* To write a command:
```sh
	rostopic pub /commander std_msgs/String "data: ''" 
 ```
where, in place of '', you can put any commands as presented before


## Working assumptions

The working assumptions will be discussed as the following list:
* The robot, simulating a pet, interact with a human and moves in a discrete 2D environment.
* Both the robot targets and its positions belongs exclusively to the map(11 by 11 grid)representing the 2D environment.
* The robot has 3 main states:
	- Play
	- Normal
	- Sleep
* The robot receive forms in strings with possible form:
	- "play"	
	- "go to x1 y1" (equivalent to voice command)
	- "point to x1 y1" (equivalent to pointing commands)
	- combination of commands with conjuctions of "and":
		- All the command after the play are executed
		- if a "play" is not in first place, only commands after the "play" command are executed
* The robot can receive any command while executing one in Play state but the ones given are neither executed nor stored.
* The robot can receive any command while in sleep state but the ones given are neither executed nor stored.
* Sleep preempt any other state when it starts.
* From Sleep you can only transition to Normal.
* The only command that can be received in Normal is "play".
* Two predifined positions inside the map are "Owner" and "Home", which cannot be changed during the execution, and can be used instead of coordinates in giving commands.

## System features and limitations

Starting from the limitations:
* The system is not scalable in the number of type of commands
* It is not scalable in the number of symbolic locations
* It is not really scalable in the number of states
* Does not provide a complete graphical interface, as the grid is not visible
* The simulation is not modifiable as it was out-sourced
* Does not distinguish between the pointing action and the vocal command

Going on to the feature:
* Understand both integer and symbolic location, provided they are of the predefined nature
* Can show the location of the robot in the map, provide, via shell, the state transition and the commands generated by the command generator
* Can take any number of commands, even if the execution cannot be stopped if not by the random intervention by Pet_logic.py


## Possible technical improvements

There are many possible technical improvements to this architecture:
* Modify the simulation component to make it more scalable
* Modify the interpreter to broaden the symbolic targets( to do it, the method is to setup a search in the parameters server to extract the coordinates related to the string)
* Create a more comprehensive propositional logic, adding an 'or' conjunction to make the system more intelligent
* Add other states to the state machine, which implies also a modification of the Pet_logic.py
* Make distinction between the pointing action and the vocal command

## Author and contacts
Matteo Palmas: matteo.palmas7gmail.com

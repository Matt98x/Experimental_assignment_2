# Experimental robotic assignment 1

<a href="http://htmlpreview.github.io/?https://github.com/Matt98x/Experimental_assignment1/blob/main/pet_package/html/index.html" title="Documentation">Documentation</a>

## Introduction

This assignment target is to build an ROS architecture to implement a robot, simulating a pet, that
interact with a human and moves in a discrete 2D environment.  
The pet has three states representing behaviours, that are simulated as a finite state machine with 3 states:  
* Play 
* Sleep 
* Normal  
  
These states determine the way the robot act inside the grid, whether moving randomly as in normal, going to targets determined by the position of a ball guided by the human or simply sleeping in the home position.  

The robot will change between states randomly, eccept for play, which is received by the user, by imposing a positive height with respect to the ground.  

## Software Architecture, State Machine and communications

### Architecture
The software architecture consists of two main areas: the world and the pet, programmatically represented by the two packages that compose this implementation.  
The first handle how the world is organized and works externally to the pet. This means that the commander, the Pet-logic(now simply interpretable as Logic) and the movement of the ball can ba all thought to be component of this macro-architecture.  
Going to the Pet, this can be considered as connected to the world over just two aspect:  
* The Perception: that handles how the pet perceives the world and in particular the ball
* The Logic: which is simply a remaining part from the first assignment, in which this node controlled both the command interpretation and the control of the robot state.  
Here we show the architecture image:  
<p align="center">
  <img src="https://github.com/Matt98x/Experimental_assignment1/blob/main/Images/Components_diagram.PNG?raw=true "Title"">
</p>
<p align="center">
  Component Diagram
</p>

Going in depth of the components, we have:  
* Random command generator(Command_giver.py)- randomically send a string representing the concatenation of one or more commands of the form: 'play'(to start the play state),'point to x y'(to simulate the pointing gesture to x y),'go to x y'(to simulate the voice command to x y) and 'hide'(to stop the play state without entering the sleep state)
* Interpreter(Pet_logic.py)- to interpret the string commands and translate them to movement of the ball, moreover, it handles the switch from play and normal to sleep and from sleep to normal
* Pet behaviours(Pet_behaviours.py)- that simulate behaviours as a finite state, in the already mentioned states
* Perception(robot_following.py): which is the node that handles the camera inputs(target identification and research) and the hardware control for these tasks(control of the neck joint(target tracking) and body(target search) ) 
* User- may or may not be present and provides the same type of messages that the Command_giver provides, adding also symbolical location such as "home" and "owner", moreover can query or set the state of the robot and interact with the parameters.  

### State Machine
Now, we can discuss the finite state machine. This, can be described by the following image:

<p align="center">
  <img src="https://github.com/Matt98x/Experimental_assignment1/blob/main/Images/Finite_state_machines.PNG?raw=true "Title"">
</p>
<p align="center">
  Finite state machine diagram
</p>
The Normal state is the simplest in nature of the three states, it simply consist of a loop of setting random destinations inside the grid without other interventions while the targets are not achieved.  

On the other hand, the sleep consist in setting the target to 'home' (set in the parameter server), and, when the position is achieved, just wait ignoring all signals exept for the change of state.  

While the 'Sleep' and 'Normal' state are quite simple in nature, the 'Play' state is quite more complex in nature.  

Of course, having to consider the position of the ball without having it, we have to construct a control with the few notions we have: the relative radius of the ball and the angle of the neck with respect to the principal axis of the chassis(that is the principal axis of the robot).  

With this, the algorith is to first minimize the angular offset of the neck w.r.t. the body rotating the body itself and then set a linear velocity while you receive the two data from the Perception node.  

Obviously, while this process is happening, the state is checked and change if the change conditions are satisfied.  

Although this is the part of the state explicitely coded in the finite state machine, an additional part is present inside the Perception node and is related to how the target is handled and searched.  

When the target is achieved, the robot proceeds to what has been defined as "swing routine", in which it moves the neck 45° to the left and to the right before centering back again to the target.  

When, eventually, the robot loose sight of the ball (always inside the perception code), the robot align the camera to the body and start spinning in the same direction of the last velocity of the body(right if the velocity was to be zero), to try and find it back again, if it cannot manage it, it will switch to the normal state.  

### Messages and parameters



## Packages and file list

As already said, the implementation is based on two packages: exp_assignment2 and pet_2.  
The first handle the simulation of the environment and the movements of the elements in it. In particular, it contains the world, robot and ball description, with the additional control parameters and related topics.  
The script present in this package are just 2:  
* go_to_point_action.py: action server to handle the movement of the robot to a specified target
* go_to_point_ball.py: action server to move the ball to a specified target
Going over to the pet_2 package, this handles the pet from perception to behaviours, plus the ball movement:  
* Command_giver.py: randomically generate command for the ball to follow
* Pet_logic: receives the commands from the command_giver and convert them to movements of the ball, apart from changing the state from play and normal to sleep and sleep to normal.
* Pet_behaviours.py: is the implementation of the finite state machine, or at least, the entirety of the sleep and normal phase and the control part of the play state
* robot_following.py: implement the perception part of the robot(in particular the vision), and handles the control of the neck joint, the swing routine and the switch from normal to play and vice versa

This were the scripts which are at the core of this implementation, but, at the side, there are other scripts.  
Starting from exp_assignment2:
* gazebo_world.launch: inside the "launch" folder: handles the definition of the simulation environment and of the elements inside it, and the controller for them.
* The urdf folder contains the xacro and gazebo description of the ball and robot
* the config folder containing the motor_config.yaml, with the controller description
* Planning.action is the message of the action server
* Finally the world folder contains the world description  

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

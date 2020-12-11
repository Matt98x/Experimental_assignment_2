# Experimental robotic assignment 2

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
  <img src="https://github.com/Matt98x/Experimental_assignment_2/blob/main/Images/Component_diagrams.PNG?raw=true "Title"">
</p>
<p align="center">
  Component Diagram
</p>

Going in depth of the components, we have:  
* Random command generator(Command_giver.py): randomically send a string representing the concatenation of one or more commands of the form: 'play'(to start the play state),'point to x y'(to simulate the pointing gesture to x y),'go to x y'(to simulate the voice command to x y) and 'hide'(to stop the play state without entering the sleep state)
* Interpreter(Pet_logic.py): to interpret the string commands and translate them to movement of the ball, moreover, it handles the switch from play and normal to sleep and from sleep to normal
* Pet behaviours(Pet_behaviours.py)- that simulate behaviours as a finite state, in the already mentioned states
* Perception(robot_following.py): which is the node that handles the camera inputs(target identification and research) and the hardware control for these tasks(control of the neck joint(target tracking) and body(target search) ) 
* Actor: may or may not be present and provides the same type of messages that the Command_giver provides, adding also symbolical location such as "home" and "owner", moreover can query or set the state of the robot and interact with the parameters.
* Ball: representation of the logic controlling the ball
* Robot Control: representation of the low-level control of the ball
* Gazebo: although not a logic part of the component diagram, it represent the way ball, robot control and perception are fundamentally part of the simulation and communicate with it

Here we can see how these last three elements communicates with the Gazebo simulation environment:  
<p align="center">
  <img src="https://github.com/Matt98x/Experimental_assignment1/blob/main/Images/Components_diagram.PNG?raw=true "Title"">
</p>
<p align="center">
  Component Diagram
</p>

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

With this, the algorithm is to first minimize the angular offset of the neck w.r.t. the body rotating the body itself and then set a linear velocity while you receive the two data from the Perception node.  

Obviously, while this process is happening, the state is checked and change if the change conditions are satisfied.  

Although this is the part of the state explicitely coded in the finite state machine, an additional part is present inside the Perception node and is related to how the target is handled and searched.  

When the target is achieved, the robot proceeds to what has been defined as "swing routine", in which it moves the neck 45° to the left and to the right before centering back again to the target.  

When, eventually, the robot loose sight of the ball (always inside the perception code), the robot align the camera to the body and start spinning in the same direction of the last velocity of the body(right if the velocity was to be zero), to try and find it back again, if it cannot manage it, it will switch to the normal state.  

### Messages and parameters

The main parameters used for this implementation are:  
* ball/ball_description: the gazebo description of the ball
* /home x&y: coordinates of the home, location where the pet sleeps
* /human_description: gazebo description of the human model
* /robot/camera1: set of parameters of the camera sensor
* /robot/joint1_position_controller: parameters for the neck joint
* /robot/joint_state_controller: parameters for the robot
* /robot/robot_description: gazebo robot description
* /state: state of the pet(play, normal, sleep)
* /gazebo: gazebo anvironment parameters  

Regarding the messages, they will be listed as  
* std_msgs.String: used by the commander to the logic, in order to send the command for the ball
* std_msgs.Float64: used by the Perception and Behavious to control the neck joint
* geometry_msgs:Pose2D: used by Perception to send the radius and camera angle for the control in the play behaviour 
* geometry_msgs:Twist: used by Behaviours and Following to Gazebo in order to control the twist of the robot
* sensor_msgs:JointStates: Used to read the values of the robot states
* sensor_msgs:Image: Message with the image camera information
* exp_assignment.PlanningAction: message of the action server, it is used by the Pet_logic and Behaviour to use the action server of the ball and robot respectively

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

### Installation

* Download the package from the github repository
* Set the package in the src folder of the catkin workspace
* Go to the main folder of the catkin workspace and launch
```sh
	.catkin_make
 ```
### Running

* After the compiler has completed its task we can use the same shell to launch the simulation environment.
```sh
	roslauch exp_assignment2 gazebo_world.launch
 ```
* On a differrent shell we can launch the pet control
```sh
	roslauch pet_2 launcher.launch
 ```
* Now the implementation is up and running
* One can observe the world and ball behaviour in the first shell, on the other there shell there is the robot states, and whether the target is achieved

### User commands

While there is the Command_giver to generate random commands, a human user can interface with the implementation, giving command using the following command.  
* To write a command:
```sh
	rostopic pub /commander std_msgs/String "data: ''" 
 ```
where, in place of '', you can put any commands as presented before.  
* Moreover, one can set the state(play(2),normal(1) and sleep(0)), writing:
```sh
	rosparam /state state_code 
 ```
state_code is to be substitute with one of the integer code stated


## Working assumptions

The working assumptions will be discussed as the following list:
* The robot, simulating a pet, interact with a human with a ball moving in the environment and moves in a 2D surface in a simulation environment.
* Both the robot targets and its positions belongs exclusively to the map(16 by 16 grid with center in (0,0))representing the 2D environment.
* The robot has 3 main states:
	- Play
	- Normal
	- Sleep
* The logic receive forms in strings with possible form:
	- "play"
	- "hide"	
	- "go to x1 y1" (equivalent to voice command)
	- "point to x1 y1" (equivalent to pointing commands)
* Once logic receives the message it applies it to the ball
* if the command is "play", the ball is positioned above ground, if "hide", it is positioned below the ground  .
* The robot activates the play mode only when the robot perceives the ball.
* When the ball is percieved the robot tries to get to it
* When achieved the "swing routine" is initialized, for which the neck is first angled to pi/4 and then too -pi/4 and then back to center
* If the ball is not in sight anymore, the robot procede to a full rotation to find if the ball is still in the environment, if not, it switch to the normal state..
* Two predifined positions inside the map are "Owner" and "Home", which cannot be changed during the execution, and can be used instead of coordinates in giving commands.

## System features and limitations

Starting from the limitations:
* The system is not scalable in the number of individually controllable robots, but if all robots have the same state, it is scalable, even if collision between robots are not handled
* It is not scalable in the number of symbolic locations
* It is not really scalable in the number of states
* Does not distinguish between the pointing action and the vocal command, since the ball act
* The robot control is not too responsive, since the control can't have a too high gain to avoid instability and the robot toppling over.
* The movement speed is high but make a trade-off for instability
* There are some problems with the target reaching when it is on the side of the robot especially when really close to the chassis(It cannot handle small curvature radii and the robot tends to run in circles around the target)
* Underline jittering in the motors, observable when control is not yet active

Going on to the features:
* The robot is controlled in almost its entirety by the pet package, which means a high scalability and modularity
* We have the robot perspective in a separate window 
* Can show the location of the robot in the map
* The robot can check the state without being stuck in an action server loop


## Possible technical improvements

There are many possible technical improvements to this architecture:  
* Modify the simulation component to make it more scalable, introducing the state change from and to sleep inside the pet_package
* Improve the control, of both the camera and the robot chassis in such a way to perform both linear and angular velocities, and in a way that the robot do not topple over
* Handle the situation when the ball is close to the side of the robot, in order to have it centered inside the robot perspective
* Add a way to avoid collisions with other obstacles, with the possible introduction of a proximity sensor of some sort
* Add multiple robots to the simulation  

## Author and contacts
Matteo Palmas: matteo.palmas7gmail.com

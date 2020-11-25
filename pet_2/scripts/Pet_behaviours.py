#!/usr/bin/env python

## @file Pet_behaviours.py
# @brief Pet state machine
#
# Details: This component simulate the behaviour state machine
#

## Libraries declaration
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import *
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from robot_pose_ekf.srv import GetStatus
from turtlesim.msg import Color
import math
import sys
import random
import smach
import smach_ros
import time


## Variable definition
## global x position
x=0 
## global y position
y=0 
## global theta position
theta=0
## command received 
comm=0
## x target 
xt=0 
## y target
yt=0 
## delay timer
timer=0.5  
## background color publisher
pub=0
## background resets
resets=0



## go_to_target: function to go to a target specified by the coordinates (xtar,ytar)
def go_to_target(xtar,ytar):
	
	global x,y
	## Control loop
	#to avoid zero division check and to control from the current position
	if xtar==x:
		nx=0
	else:
		nx=(xtar-x)/abs(xtar-x)

	if ytar==y:
		ny=0
	else:
		ny=(ytar-y)/abs(ytar-y)
 	## teleport to the nex position of the grid
	teleportabs(x+nx,y+ny,math.atan2(ny,nx))
	time.sleep(timer)
			
## roam: function to simulate the roaming behaviour of normal
def roam():
	global x,y
	## Command decision
	dx=random.randint(-1,1)
	dy=random.randint(-1,1)
	while not (x+dx<11 and x+dx>=1):
		dx=random.randint(-1,1)
	while not (y+dy<11 and y+dy>=1):
		dy=random.randint(-1,1)
	## Application of the command
	time.sleep(timer)
	#rospy.loginfo(str(dx)+" "+str(dy))
	teleportabs(x+dx,y+dy,math.atan2(dy,dx))

## simcallback: callback for the simulated pet position
def simcallback(data):
    global x,y
    x=data.x
    y=data.y
    theta=data.theta
    rospy.set_param('position',{'x':x,'y':y,'theta':theta})



## Sleep: class that describes the Sleep state
class Sleep(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
	

    def execute(self, userdata):
	global x,y
	## Check if in normal or not
	rospy.set_param('in_course',0)
	while True:
		## while not at home
		xtar=rospy.get_param('home/x')
		ytar=rospy.get_param('home/y')
		while not((xtar-x==0) and (ytar-y==0)):
			go_to_target(xtar,ytar)
			## if sleep state activated
			state=rospy.get_param('state')
			if not state==3:
				return 'outcome1'
		## change state if not sleep
		state=rospy.get_param('state')
		if not state==3:
			return 'outcome1'

## Normal: class that describes the Normal state
class Normal(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):
	global x,y
	## Check if in normal or not
	rospy.set_param('in_course',0)
	while True:
		## roam
		roam()
		## check the state
		state=rospy.get_param('state')
  		if state==2:
			return 'outcome1'
		elif state==3:
			return 'outcome2'

## Play: class that describes the Play state
class Play(smach.State):
	
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])

    def execute(self, userdata):

	while True:
		## if the command is received
		## Extract the command
		strings=""
		strings=str(strings)
		## Find if the command is empty
		while not strings=="":
			## if command received
			rospy.set_param('in_course',1)
			temp1=strings.split("|")# devide each command
			strings=""
			for i in range(len(temp1)):
				## parse each command in x and y
				temp2=temp1[i].split(" ")
				if not(len(temp2)<2):  
					xtar=int(temp2[0])
					ytar=int(temp2[1])
					#go to target
					while not((xtar==x) and (ytar==y)):
						go_to_target(xtar,ytar)
						#if sleep state activates
						state=rospy.get_param('state')
						if state==3:
							return 'outcome2'
				
				## go owner
				xtar=rospy.get_param('owner/x')
				ytar=rospy.get_param('owner/y')
				while not((xtar==x) and (ytar==y)):
					go_to_target(xtar,ytar)
					#if sleep state activates
					state=rospy.get_param('state')
					if state==3:
						return 'outcome2'
			## Check if in normal or not
			rospy.set_param('in_course',0)
		## Return to the owner when all commands are executed
		xtar=rospy.get_param('owner/x')
		ytar=rospy.get_param('owner/y')
		while not ((xtar==x) and (ytar==y)):
			go_to_target(xtar,ytar)
			# check the state
			state=rospy.get_param('state')
			if state==1:
				return 'outcome1'
			if state==3:
				return 'outcome2'
		## check the state
		state=rospy.get_param('state')
		if state==1:
			return 'outcome1'


## Main function declaration

if __name__ == '__main__':
	## Init the ros node
	rospy.init_node("state_machine")
	
	# Service to change color
	rospy.wait_for_service('/turtle1/set_pen')
	# Service to set the position
	rospy.wait_for_service('/turtle1/teleport_absolute')
	# Service to clear the path
	#rospy.wait_for_service('/clear')
	rospy.wait_for_service('/turtle1/teleport_absolute')

	# Declaration of the subscriber
	rospy.Subscriber("/turtle1/pose",Pose,simcallback)
	
	# Setting up the scene
	
	teleportabs = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
	teleportabs(5,5,0)
	resets=rospy.ServiceProxy('/clear',Empty)
	
	resets()

	## Create a SMACH state machine
        sm_s_a = smach.StateMachine(outcomes=['outcome4'])

        ## Open the container
        with sm_s_a:

	    ## Add states to the container
	    smach.StateMachine.add('NORMAL', Normal(), 
				          transitions={'outcome1':'PLAY', 
				                       'outcome2':'SLEEP'})
            ## Add states to the container
            smach.StateMachine.add('SLEEP', Sleep(), 
					transitions={'outcome1':'NORMAL'})

	    ## Add states to the container
	    smach.StateMachine.add('PLAY', Play(), 
				          transitions={'outcome1':'NORMAL', 
				                       'outcome2':'SLEEP'})
	    
            ## Execute SMACH plan
            outcome = sm_s_a.execute()

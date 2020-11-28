#!/usr/bin/env python

## @file Pet_logic.py
# @brief Pet logic
#
# Details: This component is the one that receives command, convert the command in a more compliant format and
# change state
#

##! Libraries declaration
import rospy
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from turtlesim.srv import *
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Empty
from robot_pose_ekf.srv import GetStatus
import math
import sys
import random
import time
import actionlib
import actionlib.msg
import exp_assignment2.msg

## Variable declaration
## Command string
command=[] 
## variable if a message is received
msg_received=1 
# action service client definition
client = actionlib.SimpleActionClient('/reaching_goal', exp_assignment2.msg.PlanningAction)
## coordinates component of the ball
x_comp=0
y_comp=0
z_comp=-1

## Function to assign the ball position via the action server
def setPosition(x,y,z):
	global client
	pose=Pose() # initialization of the pose message
	pose.position.x=x #assign the x component
	pose.position.y=y #assign the y component
	pose.position.z=z # the z component will be switched from over to under and vice versa
	msg=PoseStamped() # create the correct format
	msg.pose=pose # initialize a goal object with the correct format
	goal=exp_assignment2.msg.PlanningGoal(target_pose=msg) # initialize a goal object with the correct format
	client.send_goal(goal) # Sends the goal to the action server.
	client.wait_for_result() # wait for the result to come 


## comCallback: The callback that receive the command from the commander and convert them to a compliant format
def comCallback(msg):
	global msg_received,command,x_comp,y_comp,z_comp,client
	msg_received=rospy.get_param('in_course')
	state=rospy.get_param('state')
	# If it is not processing any other message
	# If a message was received parse it
	if not msg_received:
		temp_string="";
		# separate all commands with "and" delimeter
		tlist=str(msg.data).split(" and ")
		# for every command, parse it
		rospy.loginfo(tlist)
		for i in range(len(tlist)):
			# split for every space
			temp=tlist[i].split(" ")
			if temp[0]=='play' and state==1 and len(temp)<3: #Play state
				# Take the ball above the surface
				z_comp=1
				setPosition(x_comp,y_comp,z_comp)
			elif temp[0]=='hide' and not state==3 and len(temp)<3: #Play state
				# Take the ball above the surface
				z_comp=-1
				setPosition(x_comp,y_comp,z_comp) 
					
			else: # position command
				
				state=rospy.get_param('state')
				if not temp[0]=='play':
					if temp[3]=="home" or temp[3]=="owner": #topological location
						x_comp=int(rospy.get_param(str(temp[2])+str("/")+str("x")))
						y_comp=int(rospy.get_param(str(temp[2])+str("/")+str("y")))
					else: # geometrical location
						x_comp=int(temp[2]) # the x component is the third component in the string 
						y_comp=int(temp[3]) # the y component is the fourth component in the string
					# Move the ball in the plane mantaining constant the z_component
					setPosition(x_comp,y_comp,z_comp) 
							



## Main function declaration
if __name__ == '__main__':
	global client,x_comp,y_comp,z_comp

	## Init the ros node
	rospy.init_node("pet_logic")
	client.wait_for_server()
	# Declaration of the subscriber
	rospy.Subscriber("commander",String,comCallback)
	# Loop rate
	rate = rospy.Rate(5) # 10hz
	# Set the position of the ball below the surface
	z_comp=1
	setPosition(x_comp,y_comp,z_comp)
	# Loop to change state randomly
	while not rospy.is_shutdown():
		value=random.randrange(15,45,5) #timer in which to change state (from 15 s to 1 and 1/2 min)
		time.sleep(value)
		#get the current state
		state=rospy.get_param('state')
		# choose a random value to decide to change state
		value=random.randrange(1,15,1)
		if state==1 or state==2: # if it is in normal or play choose to change to sleep
			
			if value<3:
				z_comp=-1 # temporary
				setPosition(x_comp,y_comp,z_comp)
				rospy.set_param('state',3)
				
		if state==3: # if in sleep choose to change in normal
			
			if value<3:
				rospy.set_param('state',1)

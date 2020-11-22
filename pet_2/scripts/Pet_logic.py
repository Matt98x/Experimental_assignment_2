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
# action service client
client = actionlib.SimpleActionClient('/reaching_goal', exp_assignment2.msg.PlanningAction)
# Waits until the action server has started up and started
# listening for goals.
#client.wait_for_server()



## comCallback: The callback that receive the command from the commander and convert them to a compliant format
def comCallback(msg):
	global msg_received,command
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
					rospy.set_param('state',2)
					pose=Pose()
					pose.position.x=0
					pose.position.y=0
					pose.position.z=2
					msg=PoseStamped()
					msg.pose=pose
					goal=exp_assignment2.msg.PlanningGoal(target_pose=msg)
					# Sends the goal to the action server.
		    			client.send_goal(goal)
					client.wait_for_result()
					rospy.loginfo('Coordinates :'+str(pose.position.x)+" "+str(pose.position.y))
			else: # position command
				
				state=rospy.get_param('state')
				if state==2 and not temp[0]=='play':
					if temp[3]=="home" or temp[3]=="owner": #topological location
						x=(rospy.get_param(str(temp[2])+str("/")+str("x")))
						y=(rospy.get_param(str(temp[2])+str("/")+str("y")))
					else: # geometrical location
						x=temp[2] # the x component is the third component in the string 
						y=temp[3] # the y component is the fourth component in the string
					rospy.loginfo('Coordinates :'+str(x)+" "+str(y))
					pose=Pose() #create a new pose
					pose.position.x=int(x) #assign the x component
					pose.position.y=int(y) #assign the y component
					pose.position.z=0 # the z component is always zero for semplicity
					msg=PoseStamped() # create the correct format
					msg.pose=pose # assign the pose to it
					goal=exp_assignment2.msg.PlanningGoal(target_pose=msg) # initialize a goal object with the correct format
					# Sends the goal to the action server.
	    				client.send_goal(goal)
					client.wait_for_result()
							



## Main function declaration
if __name__ == '__main__':


	## Init the ros nodek
	rospy.init_node("pet_logic")
	client.wait_for_server()
	# Declaration of the subscriber
	rospy.Subscriber("commander",String,comCallback)
	# Declaration of the service
	#s = rospy.Service('commandsrv', GetStatus, srvCallback)
	rate = rospy.Rate(5) # 10hz
	
	# Loop to change state randomly
	while not rospy.is_shutdown():
		value=random.randrange(15,45,5) #timer in which to change state (from 15 s to 1 and 1/2 min)
		time.sleep(value)
		#get the current state
		state=rospy.get_param('state')
		# choose a random value to decide to change state
		value=random.randrange(1,10,1)
		if state==1: # if it is in normale choose to change to sleep
			if value<3:
				rospy.set_param('state',3)
		if state==2: # if in play choose between normal and sleep
			value=random.randrange(1,10,1)
			if value<3:
				rospy.set_param('state',3)
			if value>3 and value<7:
				rospy.set_param('state',1)
		if state==3: # if in sleep choose to change in normal
			if value<3:
				rospy.set_param('state',1)

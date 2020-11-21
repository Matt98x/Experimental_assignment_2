#!/usr/bin/env python

## @file state_outfit_simulator_node.py
# @brief Test component of the pet state machine output to the turtlesim environment
#
# Details: This component simulate the command from the pet state machine and relays them to the turtlesim
# environment reading the current position of the turtlesim pet
#

## Libraries declaration
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import *
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_srvs.srv import Empty
import math
import sys
import random

# Variables declaration

## global x position
x=0 
## global y position
y=0 


## simcallback: pose response from turtlesim
def simcallback(data):
    global x,y
    x=data.x
    y=data.y

## Main function declaration
if __name__ == '__main__':

	## Init the ros node
	rospy.init_node("state_out_sim")
	# Declaration of the subscriber
	rospy.Subscriber("/turtle1/pose",Pose,simcallback)
	# Declaration of the service
	rospy.wait_for_service('/turtle1/teleport_absolute')
	rospy.wait_for_service('/clear')
	## teleportation function
	teleportabs = rospy.ServiceProxy('/turtle1/teleport_absolute', TeleportAbsolute)
	resets=rospy.ServiceProxy('/clear',Empty)
	rate = rospy.Rate(5) # 10hz
	teleportabs(5,5,0)
	resets()

	# Main loop
	while not rospy.is_shutdown():
		# Command decision
		dx=random.randint(-1,1)
		dy=random.randint(-1,1)
		while not (x+dx<11 and x+dx>=1):
			dx=random.randint(-1,1)
		while not (y+dy<11 and y+dy>=1):
			dy=random.randint(-1,1)
		# Application of the command
		rospy.loginfo(str(dx)+" "+str(dy))
		teleportabs(x+dx,y+dy,math.atan2(dy,dx))
		#teleport(1,0)		
		rate.sleep()
	
	

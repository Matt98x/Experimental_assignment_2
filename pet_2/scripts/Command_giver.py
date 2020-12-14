#!/usr/bin/env python

## @file Command_giver.py
# @brief Component that gives command either randomly generated or given by a user, switching between the two
#
# Details: This component is the one that inputs commands inside the pet_simulation, either it randomly generates random string command or receives them from the user
#

## Libraries declaration
import rospy
from std_msgs.msg import String
import math
import sys
import random
import time

## Variable declaration


## Main function declaration
if __name__ == '__main__':
	## Init the ros node
	rospy.init_node("Command_giver")
	# Declaration of the subscriber
	pub=rospy.Publisher("commander",String,queue_size=10)
	cmd_gen=0 # to decide whether it is randomically generated or user defined
	#Main loop	
	while True:
		value=random.randrange(30,45,5) #timer in which to change state (from 15 s to 1 and 1/2 min)
		time.sleep(value)
		strings=""
		leng=1
		## Concatenate a lis t of commands
		for i in range(leng):
			value=random.randrange(1,10,1)
			if value<3:
				strings+="play"	
			elif value>=3 and value<4:
				strings+="hide"
			else:
				## choose a target inside the map
				xtar=random.randrange(-8,8,1)
				ytar=random.randrange(-8,8,1)
				## chose whether to point or tell to go
				value=random.randrange(1,3,1)
				if value==1:
					strings+="go to "+str(xtar)+" "+str(ytar)
				else:
					strings+="point to "+str(xtar)+" "+str(ytar)
			if i<leng-1:
				strings+=" and "
		pub.publish(strings)
			

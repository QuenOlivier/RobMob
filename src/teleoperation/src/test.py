#!/usr/bin/env python

import rospy
import roslib
import numpy
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3

## \file template_class.py
# \brief A template script to help you get started
# \details It is prepared to use code from the sawyer_robot_code package. The comments are made in accordance to the Doxygen rules so that you simply have to replace this description by the one of your one making. Let this file in this folder to allow him to be imported like any Python library.

## \class template
# Class used to show how a class works with ROS callbacks
class TurtleTeleop:
	
	def __init__(self):
		self._axes=[0,0,0,0,0,0,0,0]
		self._buttons=[0,0,0,0,0,0,0,0,0,0,0]
		self._turn=0
		rospy.Rate(1)	
		self._joy_sub=rospy.Subscriber("joy", Joy, self.getJoy)	
		self._turtle_pub=rospy.Publisher("turtle1/cmd_vel",Twist,queue_size=1)
	
	##Fetch the data.
	#
	# \details Extract the information from the ROS message and set it in a usable format (e.g float instead of str).
	def getJoy(self,data):
		self._axes=data.axes
		self._buttons=data.buttons
		
		

		###########################################
		# Write your own code here
		###########################################

## \brief This is your main function
# \details This is where you will put your argument parseer lines and main code
def main():
	"""<Your file name>

	Short description of what this script does once launched in a terminal.

	To use this script, type in a terminal :
		rosrun sawyer_robot_code <your_script_name>
	"""

	print "Initializing ROS node...\n\n"
	rospy.init_node("Main",anonymous=True)
	print "Doing stuff...\n"

	test=TurtleTeleop()

	while(1):
		norm=math.sqrt(test._axes[1]*test._axes[1]+test._axes[0]*test._axes[0])
		if(test._axes[1]!=0):
			angle=numpy.arctan(test._axes[0]/test._axes[1])
		else:
			angle=0
		linear=Vector3(test._axes[1],0,0)
		angular=Vector3(0,0,angle)
		msg=Twist(linear,angular)
		test._turtle_pub.publish(msg)



## \brief these lines are simply made to check if this script was called in a terminal or imported 
if __name__ == '__main__':
	
	main()
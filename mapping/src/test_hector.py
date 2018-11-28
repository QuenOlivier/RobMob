#!/usr/bin/env python

import rospy
from PIL import Image
import cv2
import roslib
import numpy
import math
import argparse
from sensor_msgs.msg import Joy
from nav_msgs.msg import MapMetaData,OccupancyGrid
from geometry_msgs.msg import Pose,Twist, Vector3

## \file template_class.py
# \brief A template script to help you get started
# \details It is prepared to use code from the sawyer_robot_code package. The comments are made in accordance to the Doxygen rules so that you simply have to replace this description by the one of your one making. Let this file in this folder to allow him to be imported like any Python library.

## \class template
# Class used to show how a class works with ROS callbacks
class HectorMap:
	
	def __init__(self):
		self._meta=MapMetaData()
		self._map = numpy.zeros((1,1))
		self._rate=10
		self._turn=0
		rospy.Rate(1)	
		self._joy_sub=rospy.Subscriber("map", OccupancyGrid, self.getMap)
	
	##Fetch the data.
	#
	# \details Extract the information from the ROS message and set it in a usable format (e.g float instead of str).
	def getMap(self,data):
		if(self._turn%self._rate==0):
			self._meta=data.info
			self._map=numpy.zeros((self._meta.width,self._meta.height))
			for i in range(self._meta.height):
				for j in range(self._meta.width):
					self._map[j,i]=data.data[j+i*self._meta.width]
		self._turn=self._turn+1
		
		

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

	test=HectorMap()

	while(1):
		cv2.imshow('map hector',test._map)



## \brief these lines are simply made to check if this script was called in a terminal or imported 
if __name__ == '__main__':
	
	main()

#!/usr/bin/env python

import rospy
from PIL import Image
import cv2
import roslib
import numpy as np
import math
import argparse
from sensor_msgs.msg import Joy
from nav_msgs.srv import GetMap
from geometry_msgs.msg import Pose,Twist, Vector3

## \file template_class.py
# \brief A template script to help you get started
# \details It is prepared to use code from the sawyer_robot_code package. The comments are made in accordance to the Doxygen rules so that you simply have to replace this description by the one of your one making. Let this file in this folder to allow him to be imported like any Python library.

## \class template
# Class used to show how a class works with ROS callbacks
class HectorMap:
	
	def __init__(self):
		self._height=0
		self._width=0
		self._map=np.zeros((1,1), np.uint8)
		self._resolution=0
		self._origin=Pose()
	
	##Fetch the data.
	#
	# \details Extract the information from the ROS message and set it in a usable format (e.g float instead of str).
	def updateMap(self):
		rospy.wait_for_service('/dynamic_map')
		try:
			getData = rospy.ServiceProxy('/dynamic_map', GetMap)
			data = getData().map
			self._height=data.info.height
			self._width=data.info.width
			self._resolution=data.info.resolution
			self._origin=data.info.origin
			print("Data : ")
			print( self._width )
			print(self._height)
			#New test
			print(len(data.data))
			self._map = np.zeros((self._height,self._width),np.uint8)
			i=self._height-1
			j=0
			current=0
			while i>=0:
				while j<=self._width-1:
					value=data.data[current]
					if value == -1:
						self._map[i,j]=0
					if value != -1:
						self._map[i,j]=255
					current=current+1
					j=j+1
				i=i-1
				j=0
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		

		
   	
	
## \brief This is your main function
# \details This is where you will put your argument parseer lines and main code
def main():
	"""<Your file name>

	Short description of what this script does once launched in a terminal.

	To use this script, type in a terminal :
		rosrun sawyer_robot_code <your_script_name>
	"""

	print "Doing stuff...\n"

	test=HectorMap()
	test.updateMap()
	while(1):
		cv2.namedWindow('map hector', 0)
	    	# refresh the image on the screen
		cv2.imshow('map hector',test._map)
		cv2.waitKey(3)
	



## \brief these lines are simply made to check if this script was called in a terminal or imported 
if __name__ == '__main__':
	
	main()

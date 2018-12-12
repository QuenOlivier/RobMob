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

## \class template
# Class used to show how a class works with ROS callbacks
class Map:
	
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

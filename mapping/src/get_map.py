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
from util import map

## \file template_class.py
# \brief A template script to help you get started
# \details It is prepared to use code from the sawyer_robot_code package. The comments are made in accordance to the Doxygen rules so that you simply have to replace this description by the one of your one making. Let this file in this folder to allow him to be imported like any Python library.	
   	
	
## \brief This is your main function
# \details This is where you will put your argument parseer lines and main code
def main():
	"""<Your file name>

	Short description of what this script does once launched in a terminal.

	To use this script, type in a terminal :
		rosrun sawyer_robot_code <your_script_name>
	"""

	print "Doing stuff...\n"

	test=map.Map()
	test.updateMap()
	while(1):
		cv2.namedWindow('map hector', 0)
	    	# refresh the image on the screen
		cv2.imshow('map hector',test._map)
		cv2.waitKey(3)
	



## \brief these lines are simply made to check if this script was called in a terminal or imported 
if __name__ == '__main__':
	
	main()

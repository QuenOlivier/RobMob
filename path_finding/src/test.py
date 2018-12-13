#!/usr/bin/env python

from PIL import Image
import numpy
import cv2
import rospy
from util import map
from geometry_msgs.msg import PoseStamped, Pose, Point

class PrintMap:
	
	def __init__(self):
		self._map=map.Map()
		self._flag=False
		self._goal=Point()
		self._sub=rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback)
		

	def callback(self, data):
		self.getMap()
    		self._goal=data.pose.position
		self._flag=True
		self.printData()
		

	def getMap(self):
		self._map.updateMap()

	def printData(self):
		print("PrintMap data :\n\nFlag : ")
		print(self._flag)
		print("\nGoal : ")
		print(self._goal)
		print("\n")
	

def main():
	test=PrintMap() 
	cv2.namedWindow('map hector', 0)
	# refresh the image on the screen
	while(1):
		img=test._map._map
		cv2.imshow('map hector',img)
		cv2.circle(img,(int(test._goal.x),int(test._goal.y)),2,(0,0,255),-1)
		cv2.waitKey(3)
	

	# spin() simply keeps python from exiting until this node is stopped
    	rospy.spin()
	

if __name__ == '__main__':
    try:
	print("\n[Initializing node...]\n")
	rospy.init_node('Goal',anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass

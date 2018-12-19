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
		#Calcul position de l'objectif
    		self._goal=data.pose.position
		self._goal.x=self._goal.x-self._map._origin.position.x
		self._goal.y=self._goal.y-self._map._origin.position.y
		#Reset de l'origin
		self._map._origin.position.x=self._map._origin.position.x/self._map._resolution
		self._map._origin.position.y=self._map._origin.position.y/self._map._resolution

		self._flag=True
		self.printMapData()
		self.printData()

		print(self._goal.y/self._map._resolution)
		

	def getMap(self):
		self._map.updateMap()

	def printData(self):
		print("PrintMap data :\n\nFlag : ")
		print(self._flag)
		print("\nGoal : ")
		print(self._goal)
		print("\nOrigin : ")
		print(self._map._origin.position)
		print("\n\n")

	def printMapData(self):
		print("Map data :\n\nHeight :")
		print(self._map._height)
		print("\nWidth : ")
		print(self._map._width)
		print("\nResolution : ")
		print(self._map._resolution)
		print("\n\n")
		
		
	

def main():
	test=PrintMap() 
	cv2.namedWindow('map hector', 0)
	# refresh the image on the screen
	while(1):
		if(test._flag):
			img=cv2.cvtColor(test._map._map, cv2.COLOR_GRAY2RGB)
			#Draw goal
			x=int(test._goal.x/test._map._resolution)
			y=int(test._map._height-test._goal.y/test._map._resolution)
			cv2.circle(img,(x,y),5,(0,0,255),-1)

			#Draw origin
			
			#cv2.circle(img,(x,y),5,(0,255,0),-1)

			cv2.imshow('map hector',img)
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

#!/usr/bin/env python

from PIL import Image
import numpy
import cv2
import rospy
from util import map
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Odometry
import path_finding

class PrintMap:

	def __init__(self):
		self._map=map.Map()
		self._flagMap=False
		self._flagRob=False
		self._goal=Point()
		self._robot=Point()
		self._sub1=rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callbackMap)
		self._sub2=rospy.Subscriber("/odom", Odometry, self.callbackRob)

	def callbackMap(self, data):
		self.getMap()
		#Calcul position de l'objectif

		x=data.pose.position.x-self._map._origin.position.x
		x=x/self._map._resolution

		y=data.pose.position.y-self._map._origin.position.y
		y=self._map._height-y/self._map._resolution

		self._goal=Point(int(x),int(y),0)

		self._flagMap=True
		#self.printMapData()
		#self.printData()

		#print(self._goal.y/self._map._resolution)

	def callbackRob(self, data):
		if(self._flagMap):
			#Calcul position du robot
			x=data.pose.pose.position.x-self._map._origin.position.x
			x=x/self._map._resolution

			y=data.pose.pose.position.y-self._map._origin.position.y
			y=self._map._height-y/self._map._resolution

			self._robot=Point(int(x),int(y),0)
			self._flagRob=True

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

#Convert a list of openCV points to RealSpace points related to a map
def CVtoRealspaceList(liste,refMap):
	height=refMap._map._height
	width=refMap._map._width
	originX=refMap._map._origin.position.x
	originY=refMap._map._origin.position.y
	res=refMap._map._resolution

	for i in range(0,len(liste)):
		liste[i].x=liste[i].x*res + originX
		liste[i].y=(height-liste[i].y)*res + originY


def main():
	test=PrintMap()

	# refresh the image on the screen
	running=True
	path=None
	while(running):
		if(test._flagMap and test._flagRob):
			img=cv2.cvtColor(test._map._map, cv2.COLOR_GRAY2RGB)
			cv2.imwrite('map.png',img)


			#Draw goal
			cv2.circle(img,(test._goal.x,test._goal.y),5,(0,0,255),-1)

			#Draw robot

			cv2.circle(img,(test._robot.x,test._robot.y),5,(0,255,0),-1)


			print("Starting RTT algorithm")
			path=path_finding.main(test._robot,test._goal)
			#path_finding.test(test._robot,test._goal)
			print("Ended path finding\n")
			cv2.namedWindow('map hector', 0)
			for i in range(1, len(path)):
				cv2.line(img,(path[i-1].x,path[i-1].y),(path[i].x,path[i].y),(0,0,255),2)
			cv2.imshow('map hector',img)
			cv2.waitKey(0)
			running=False



	# spin() simply keeps python from exiting until this node is stopped
	CVtoRealspaceList(path,test)
	return path


if __name__ == '__main__':
    try:
	print("\n[Initializing node...]\n")
	rospy.init_node('Goal',anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

from PIL import Image
import numpy
import cv2
import rospy
from util import map
from geometry_msgs.msg import PoseStamped, Pose

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

def main():
	test=map.Map()
	rospy.Subscriber("chatter", PoseStamped, callback)

    	# spin() simply keeps python from exiting until this node is stopped
    	rospy.spin()
	

if __name__ == '__main__':
    try:
	print("\n[Initializing node...]\n")
	rospy.init_node('Goal',anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass

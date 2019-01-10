import rospy
import time
import roslib
import random
import copy
import math
import numpy as np
from util import map
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist, Vector3
from nav_msgs.msg import Odometry
import retrieve_map
from tf.transformations import euler_from_quaternion

class Minilab:
    def __init__(self):
		self._path=None
		self._state=Vector3()
		self._pub=rospy.Publisher("/cmd_vel",Twist,queue_size=1)
		self._sub=rospy.Subscriber("/odom", Odometry, self.getRob)

    def setPath(self,path):
        self._path=path

    def getRob(self, data):
		#Calcul position du robot
        x=data.pose.pose.position.x
        y=data.pose.pose.position.y
        quaternion=data.pose.pose.orientation
        explicit_quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        euler= euler_from_quaternion(explicit_quat)
        theta=euler[2]
        self._state=Vector3(x,y,theta)

    def setSpeed(self,linear,angular):
        lin=Vector3(linear,0,0)
        ang=Vector3(0,0,angular)
        msg=Twist(lin,ang)
        self._pub.publish(msg)

def main():
    rob=Minilab()
    path=retrieve_map.main()
    del path[0]
    rob.setPath(path)

    #Parametre de suivi
    l1=0.05
    k1=0.05
    k2=0.01

    kp=0.1
    ka=0.2
    kb=0.01
    prec=0.3

    reachedAng=False

    while(path != None):
        print("On entre dans la boucle")
        theta=rob._state.z
        x=rob._state.x
        y=rob._state.y
        xp=x+l1*math.cos(theta)
        yp=y+l1*math.sin(theta)

        e1=x-rob._path[0].x
        e2=y-rob._path[0].y
        rho = math.sqrt(e1*e1 + e2*e2)
        if( rho < 0.6):
            kp=kp*2
        if( rho < prec):
            print("Distance faible")
            reachedAng=False
            kp=0.1
            if(len(path)==2):
                print("Plus qu'une etape")
                prec=0.05
                del path[0]
            elif(len(path)==1):
                print("Fin de parcours")
                path=None
                rob.setSpeed(0,0)
            else:
                del path[0]
                print "Il reste ",len(path)," etapes"
        else:
            #commande polaire
            beta=math.atan2(e2,e1)+math.pi
            if beta > math.pi:
                beta = -2*math.pi+beta
            alpha = beta - theta
            if(reachedAng):
                ang=0
            elif(alpha<0.01 and alpha>-0.01):
                ang=0
                reachedAng=True
            else:
                ang = ka * alpha

            lin = kp * rho
            if lin > 3:
                lin=3

            print "Angle robot:",theta," ,angle beta:",beta," ,angle alpha:",alpha
            print "Consigne angulaire:",ang
            rob.setSpeed(lin,ang)
        time.sleep(0.1)


if __name__ == '__main__':
    try:
        print("\n[Initializing node...]\n")
        rospy.init_node('Path_following',anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass

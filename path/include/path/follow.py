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
    ka=0.5
    kp=0.2

    #Parametrage de filtrage passe haut
    k2=0.1

    angPrec=0

    precDist=0.3
    seuilMinVit=0.2
    seuilMaxVit=1
    precAngle=0.05
    seuilMinAngle=0.65
    seuilMaxAngle=1

    reachedAng=False
    flagLast=False
    i=20
    rho=5

    while(path != None):

        while(rho>precDist):
            theta=rob._state.z
            x=rob._state.x
            y=rob._state.y

            e1=rob._path[0].x-x
            e2=rob._path[0].y-y
            rho = math.sqrt(e1*e1 + e2*e2)
            #commande polaire
            beta=math.atan2(e2,e1)
            alpha = beta - theta
            ang = ka * alpha
            if(reachedAng):
                ang=0
            elif(abs(alpha)<precAngle):
                ang=0
                reachedAng=True
            elif(abs(alpha)>2*precAngle):
                reachedAng=False
            elif(alpha>seuilMaxAngle):
                ang=2*ang
            elif(alpha<seuilMaxAngle):
                ang= 2*ang
            ang=ang+k2*angPrec
            angPrec=ang

            lin=kp*rho
            if(lin>seuilMaxVit):
                lin=seuilMaxVit
            elif(lin<seuilMinVit):
                lin=seuilMinVit
            elif(abs(alpha)>seuilMinAngle):
                lin=0.1

            if i==20:
                print "Angle robot:",theta,", angle beta:",beta
                print "Distance restante:",rho,", erreur angulaire alpha:",alpha,"\n"
                i=0
            i=i+1
            rob.setSpeed(lin,ang)
            time.sleep(0.1)
        print("Distance faible")
        reachedAng=False
        rho=5
        if(len(path)==2):
            print("Plus qu'une etape\n")
            precDist=0.05
            precAngle=0.01
            seuilMaxVit=0.5
            seuilMinVit=0.1
            seuilMinAngle=seuilMinAngle/2
            del path[0]
        elif(len(path)==1):
            print("Fin de parcours\n")
            path=None
            rho=0
            rob.setSpeed(0,0)
        else:
            del path[0]
            print "Il reste ",len(path)," etapes\n"



if __name__ == '__main__':
    try:
        print("\n[Initializing node...]\n")
        rospy.init_node('Path_following',anonymous=True)
        main()
    except rospy.ROSInterruptException:
        pass

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

def correctAngle(prec,angle):
    betaBis=angle

    if abs(prec-angle)>2:
        print("Hello corrector")
        print "Current angle :",angle,", prev beta :",prec
        if abs(prec)<2*math.pi :
            if(angle<0):
                betaBis=2*math.pi+angle
            else:
                betaBis=angle-2*math.pi
    return betaBis

def main():
    rob=Minilab()
    path=retrieve_map.main()
    del path[0]
    rob.setPath(path)

    #Parametre de suivi
    ka=0.5
    kp=0.2

    precisionDist=0.3
    seuilMinVit=0.2
    seuilMaxVit=1
    precisionAngle=0.05
    seuilMinAngle=0.65
    seuilMaxAngle=1
    rho=5
    lin=0
    ang=0
    angPrec=0

    #Parametrage de filtrage passe haut
    k2=0.1

    reachedAng=False
    flagLast=False
    i=20

    e1=rob._path[0].x-rob._state.x
    e2=rob._path[0].y-rob._state.y
    beta=math.atan2(e2,e1)
    alpha=beta - rob._state.z

    while(path != None):

        while(rho>precisionDist):
            theta=rob._state.z
            x=rob._state.x
            y=rob._state.y

            e1=rob._path[0].x-x
            e2=rob._path[0].y-y
            rho = math.sqrt(e1*e1 + e2*e2)
            #commande polaire

            beta=math.atan2(e2,e1)

            alpha = correctAngle(alpha,beta - theta)

            ang = ka * alpha
            if(reachedAng):
                ang=0
            elif(abs(alpha)<precisionAngle):
                ang=0
                reachedAng=True
            elif(abs(alpha)>2*precisionAngle):
                reachedAng=False
            elif(alpha>seuilMaxAngle):
                ang= 2 * ang
            elif(alpha<seuilMaxAngle):
                ang= 2 * ang

            ang= ang + k2*angPrec
            angPrec=ang

            if(ang>1.5):
                ang=1.5

            lin=kp*rho
            #if(abs(alpha)>math.pi):
            #    lin=0
            if(abs(alpha)>seuilMinAngle):
                lin=0.1
            elif(lin>seuilMaxVit):
                lin=seuilMaxVit
            elif(lin<seuilMinVit):
                lin=seuilMinVit



            if i==20:
                print "Angle robot:",theta,", angle beta:",beta
                print "Distance restante:",rho,", erreur angulaire alpha:",alpha,"\n"
                print "Commande : lin :",lin,", ang:",ang,"\n"
                i=0
            i=i+1
            rob.setSpeed(lin,ang)
            time.sleep(0.1)
        print("Point atteint")
        reachedAng=False
        rho=5
        if(len(path)==2):
            print("Plus qu'une etape\n")
            precisionDist=0.05
            precisionAngle=0.01
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

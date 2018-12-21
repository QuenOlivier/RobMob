#!/usr/bin/python
# -*- coding: utf-8 -*-

#On a une image binaire
#Tirage d'une position aleatoire dans cette image
#Verification que la position soit libre
#Trouver le point le plus proche pnear (listprox[0])
#Creer un point np avec une distance dq moins grande que depuis pnear
#Verifier que le trajet entre pnear et np est libre
#Sinon prendre le second point le plus proche comme nouveau pnear, etc
#Ajouter np a la liste des points de l'arbre, avec pnear comme predecesseur
#Verifier que le trajet entre np et goal est libre
#Si oui, ajouter goal a l'arbre avec np comme predecesseur
#Sinon, refaire une boucle

#Une fois l'arbre fait, pour chaque point, verifier si les trajet avec les points precedents sont libres
#Supprimer tout les points dont le trajet est inutile
#Creer une trajectoire avec les points restants
#Lisser la trajectoire

#Fonctions :
#Tirage aleatoire de point avec verif qu'il est libre
#Copie d'une liste avec tri par ordre de proximite
#Creation d'un point a une distance dq d'un point pinit en direction d'un point pgoal
#Verification que la distance entre les points pinit et pgoal est libre

import random
import copy
import math
import cv2
import numpy as np

class point:
    def __init__(self, xp, yp, ind, pre):
        self.x = xp
        self.y = yp
        self.indice = ind 	#Position dans la liste des points
        self.preced = pre 	#Indice du point precedent dans la liste des points


#Cree un nouveau point dans une case libre
def randompoint(xmax, ymax, img):
	x = random.randint(0, xmax-1)
	y = random.randint(0, ymax-1)
	#while(img[y, x] != 255):
	#	x = random.randint(0, xmax-1)
	#	y = random.randint(0, ymax-1)
	return point(x, y, 0, 0)

def dist(pos, goal):
	px = pos.x
	py = pos.y
	gx = goal.x
	gy = goal.y
	return math.sqrt((gx - px)**2 + (gy - py)**2)

def listeproximite(listpoints, goal):
	listcop = copy.copy(listpoints)		#Si bug, essayer la deepcopy, mais copy devrait sufire
	listres = []

	while(len(listcop) > 0):
		distmin = dist(listcop[0], goal)
		ind = 0
		for i in range(len(listcop)):
			if dist(listcop[i], goal) < distmin :
				distmin = dist(listcop[i], goal)
				ind = i

		listres.append(listcop[ind])
		del listcop[ind]

	return listres

def pathfree(pinit, goal, img):
	#Puisque le deplacement sera petit, on peut verifier la totalite du rectangle dont la diagonale est la
	#trajectoire que l'on souhaite verifier. Ainsi si la ligne est libre mais que l'on se rapproche
	#dangeureusement du mur, nous n'emprinterons pas ce chemin

	#print(pinit.x)
	#print(pinit.y)
	#print(goal.x)
	#print(goal.y)
    if(img[goal.y,goal.x]==0):
        return False
	for i in range(min(pinit.x, goal.x),max(pinit.x, goal.x) +1):
		for j in range(min(pinit.y, goal.y),max(pinit.y, goal.y) +1):
			#print(img[j, i])
			if img[j, i] == 0 :
				return False
	return True

def dqpoint(listprox, goal, dq, img):
    free = False
    xnp = 0
    ynp = 0
    indpre = 0
    indice_test = 0
    while(free == False):
        x = listprox[indice_test].x
        y = listprox[indice_test].y
        norm=dist(listprox[indice_test], goal)
        if(norm != 0):
            if(norm<dq):
                xnp = goal.x
                ynp = goal.y
            else:
                xnp = int(x + (goal.x - x)*(dq/norm))
                ynp = int(y + (goal.y - y)*(dq/norm))
            free = pathfree(listprox[indice_test], point(xnp, ynp, 0, 0), img)
            indpre = listprox[indice_test].indice

        else:
            free = False


        if(not free):
            indice_test = indice_test +1
            if(len(listprox)==indice_test):
                return point(0,0,0,None)

	return point(xnp, ynp, 0, indpre)

def printList(liste):
	for i in range(len(liste)):
		print("x :")
		print(liste[i].x)
		print("y :")
		print(liste[i].y)
		print("ind :")
		print(liste[i].indice)
		print("ind precede :")
		print(liste[i].preced)
		print("")

def main(rob,objective):
    pas = 5 #la longueur des pas du RRT

    #Recuperation de la map
    kernel = np.ones((5,5),np.uint8)
    map=cv2.imread("map.png",0)
    img = cv2.erode(map,kernel,iterations = 1)
    yimg, ximg = img.shape[:2]
    print("Dimension =",yimg,",",ximg)
    imgaff = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    goal = point(int(objective.x),int(objective.y),0,0)

    robot=point(int(rob.x),int(rob.y),0,None)

    ListePoints = []	#Liste des points du RRT
    ListePoints.append(robot)	#Ajout de la pos du Robot comme premier point

	#Draw goal
    cv2.circle(imgaff,(goal.x,goal.y),5,(0,0,255),-1)
	#Draw robot
    cv2.circle(imgaff,(robot.x,robot.y),5,(0,255,0),-1)

	#Jusque la OK

    while((ListePoints[-1].x != goal.x) & (ListePoints[-1].y != goal.y)):
        randpoint = randompoint(ximg, yimg, img)					#Lance aleatoire du RRT
        #cv2.circle(imgaff,(randpoint.x,randpoint.y),2,(255,0,0),-1)
        listprox = listeproximite(ListePoints, randpoint)			#Tri par ordre de proximite des points de l'arbre

        		#printList(listprox)

        pointTemp=dqpoint(listprox, randpoint, pas, img)
        value=img[pointTemp.y,pointTemp.x]
        if(pointTemp.preced==None or value==0):
            print("ERREUR PAS DE SOLUTION\n")
        else:
            ListePoints.append(pointTemp)	#Calcul du nouveau point de l'arbre et ajout a la liste
            ListePoints[-1].indice = len(ListePoints)-1					#Changement de l'indice du nouveau point pour le faire correspondre a sa position

        	#Trace sur l'image a afficher d'une petite ligne pour relier le nouveau point du RRT a son point parent
            cv2.line(imgaff,(ListePoints[-1].x,ListePoints[-1].y),(ListePoints[ListePoints[-1].preced].x,ListePoints[ListePoints[-1].preced].y),(0,0,0),1)

        		#Test pour voir si le dernier point a une vue directe sur l'objectif, si oui, on fini le RRT
        if(pathfree(ListePoints[-1], goal, img) == True):
            ListePoints.append(goal)						#Ajout de l'objectif a l'arbre en position finale
            ListePoints[-1].preced = len(ListePoints)-2		#Changement du parent de l'objectif pour le dernier point calcule
            ListePoints[-1].indice = len(ListePoints)-1		#Changement de l'indice de l'objectif pour correspondre a la liste
            cv2.line(imgaff,(ListePoints[-1].x,ListePoints[-1].y),(ListePoints[ListePoints[-1].preced].x,ListePoints[ListePoints[-1].preced].y),(0,0,0),1)

        cv2.imshow('map_RRT',imgaff)
        cv2.waitKey(5)

    return ListePoints

def test(rob,objective):
    pas = 5 #la longueur des pas du RRT

    #Recuperation de la map
    kernel = np.ones((5,5),np.uint8)
    map=cv2.imread("map.png",0)
    img = cv2.erode(map,kernel,iterations = 1)
    yimg, ximg = img.shape[:2]
    print("Dimension =",yimg,",",ximg)
    imgaff = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

    goal = point(int(objective.x),int(objective.y),0,0)

    robot=point(int(rob.x),int(rob.y),0,None)

    ListePoints = []	#Liste des points du RRT
    ListePoints.append(robot)	#Ajout de la pos du Robot comme premier point

	#Draw goal
    cv2.circle(imgaff,(goal.x,goal.y),5,(0,0,255),-1)
	#Draw robot
    cv2.circle(imgaff,(robot.x,robot.y),5,(0,255,0),-1)

	randpoint = point(int((robot.x+goal.x)/2),int((robot.y+goal.y)/2),0,0)
    #cv2.circle(imgaff,(randpoint.x,randpoint.y),2,(255,0,0),-1)
    listprox = listeproximite(ListePoints, randpoint)			#Tri par ordre de proximite des points de l'arbre

    pointTemp=dqpoint(listprox, randpoint, pas, img)
    print(pointTemp.x," ",pointTemp.y)
    value=img[pointTemp.y,pointTemp.x]
    if(pointTemp.preced==None or value==0):
        print("ERREUR PAS DE SOLUTION\n")
    else:
        ListePoints.append(pointTemp)	#Calcul du nouveau point de l'arbre et ajout a la liste
        ListePoints[-1].indice = len(ListePoints)-1					#Changement de l'indice du nouveau point pour le faire correspondre a sa position

        #Trace sur l'image a afficher d'une petite ligne pour relier le nouveau point du RRT a son point parent
        cv2.line(imgaff,(ListePoints[-1].x,ListePoints[-1].y),(ListePoints[ListePoints[-1].preced].x,ListePoints[ListePoints[-1].preced].y),(0,0,0),1)

        		#Test pour voir si le dernier point a une vue directe sur l'objectif, si oui, on fini le RRT
    if(pathfree(ListePoints[-1], goal, img) == True):
        ListePoints.append(goal)						#Ajout de l'objectif a l'arbre en position finale
        ListePoints[-1].preced = len(ListePoints)-2		#Changement du parent de l'objectif pour le dernier point calcule
        ListePoints[-1].indice = len(ListePoints)-1		#Changement de l'indice de l'objectif pour correspondre a la liste
        cv2.line(imgaff,(ListePoints[-1].x,ListePoints[-1].y),(ListePoints[ListePoints[-1].preced].x,ListePoints[ListePoints[-1].preced].y),(0,0,0),1)

    cv2.imshow('map_RRT',imgaff)
    cv2.waitKey(0)

    return ListePoints


if __name__ == '__main__':
    main()
#POUR LA SUITE :
#Definir l'image, la pos du robot et de l'objectif
#Afficher dans l'image le chemin trouve
#Simplifier le chemin
#Lisser la trajectoire

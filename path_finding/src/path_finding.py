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
    listrand = []
    listres = []
    nb = 0;

    while((len(listcop) > 0) and (nb < 20)): #On prend les 20 derniers points de la liste de points
        listrand.append(listcop[-1])
        del listcop[-1]
        nb = nb + 1

    nb = 0

    while((len(listcop) > 0) and (nb < 30)): #Puis on prend 30 points alleatoirement dans la liste de points
        ind = random.randint(0, len(listcop)-1)
        listrand.append(listcop[ind])
        del listcop[ind]
        nb = nb + 1


    while(len(listrand) > 0):  #On trie par ordre de proximite nos 50 points selectionnes
        distmin = dist(listrand[0], goal)
        ind = 0
        for i in range(len(listrand)):
            if dist(listrand[i], goal) < distmin :
                distmin = dist(listrand[i], goal)
                ind = i

        listres.append(listrand[ind])
        del listrand[ind]

    return listres

def listeproximite_old(listpoints, goal):
    listcop = copy.copy(listpoints)     #Si bug, essayer la deepcopy, mais copy devrait sufire
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
    #if(img[goal.y,goal.x]==0):
    #    return False
	for i in range(min(pinit.x, goal.x),max(pinit.x, goal.x) +1):
		for j in range(min(pinit.y, goal.y),max(pinit.y, goal.y) +1):
			#print(img[j, i])
			if img[j, i] == 0 :
				return False
	return True

def dqpoint(listprox, goal, dq, img):
    #cv2.imshow('temp',img)
    #cv2.waitKey(500)
    free = False
    xnp = 0
    ynp = 0
    indpre = 0
    indice_test = 0
    np=point(0,0,0,0)
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
            img2=img
            np=point(xnp, ynp, 0, 0)
            free = pathfree(listprox[indice_test], np , img2)
            indpre = listprox[indice_test].indice

        else:
            free = False


        if(not free):
            indice_test = indice_test +1
            if(len(listprox)==indice_test):
                return point(0,0,0,None)

    np=point(xnp, ynp, 0, indpre)
    #print(pathfree(np, listprox[indice_test], img) )
    #print(free)
    return np

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

#Tested, work fine
def retrieve_list(listinit):
    listres = []
    listres.append(listinit[-1])
    indice=listinit[-1].preced
    i = 0
    while(indice != None):
        listres.append(listinit[indice])
        indice=listinit[indice].preced
        i=i+1

    listres=invers_list(listres)
    return listres

#Tested, works fine
def invers_list(listinit):
    listres = []

    for i in range(len(listinit)):
        listres.append(listinit[len(listinit)-1 - i])

    return listres

def reduce_list(list_path, img):
    #Regarde chaque point si son parent est accessible depuis le point actuel pour reduire le nombre de point de la liste et ne garder que quelques points cles
    listres = []

    listres.append(list_path[0])
    indice=0

    while(indice <= len(list_path)-2):
        j=len(list_path)-1

        while((not pathfree(listres[-1], list_path[j], img)) and j > indice):
            j=j-1

        listres.append(list_path[j])
        indice=j


    return listres


def main(rob,objective):
    pas = 15 #la longueur des pas du RRT

    #Recuperation de la map
    kernel = np.ones((10,10),np.uint8)
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

    while(dist(goal,ListePoints[-1]) > 0.5):
        randpoint = randompoint(ximg, yimg, img)					#Lance aleatoire du RRT
        #cv2.circle(imgaff,(randpoint.x,randpoint.y),2,(255,0,0),-1)
        listprox = listeproximite(ListePoints, randpoint)			#Tri par ordre de proximite des points de l'arbre
        #listprox = listeproximite_old(ListePoints, randpoint)           #Tri par ordre de proximite des points de l'arbre

        		#printList(listprox)

        pointTemp=dqpoint(listprox, randpoint, pas, img)
        value=img[pointTemp.y,pointTemp.x]

        if(pointTemp.preced==None):
            print("ERREUR PAS DE SOLUTION\n")
        else:
            ListePoints.append(pointTemp)	#Calcul du nouveau point de l'arbre et ajout a la liste
            ListePoints[-1].indice = len(ListePoints)-1					#Changement de l'indice du nouveau point pour le faire correspondre a sa position

        	#Trace sur l'image a afficher d'une petite ligne pour relier le nouveau point du RRT a son point parent
            cv2.line(imgaff,(ListePoints[-1].x,ListePoints[-1].y),(ListePoints[ListePoints[-1].preced].x,ListePoints[ListePoints[-1].preced].y),(0,0,0),1)

        		#Test pour voir si le dernier point a une vue directe sur l'objectif, si oui, on fini le RRT
        if(pathfree(ListePoints[-1], goal, img) == True):
            goal.preced=len(ListePoints)-1
            goal.indice=len(ListePoints)
            ListePoints.append(goal)						#Ajout de l'objectif a l'arbre en position finale
            cv2.line(imgaff,(ListePoints[-1].x,ListePoints[-1].y),(ListePoints[ListePoints[-1].preced].x,ListePoints[ListePoints[-1].preced].y),(0,0,0),1)

        cv2.imshow('map_RRT',imgaff)
        cv2.waitKey(5)


    print('Fin du path finding..........\n')
    path= []
    path=retrieve_list(ListePoints)

    for i in range(0,len(path)-1):
        cv2.line(imgaff,(path[i].x,path[i].y),(path[i+1].x,path[i+1].y),(0,0,255),1)
        cv2.imshow('map_RRT',imgaff)
        cv2.waitKey(200)
    cv2.imshow('map_RRT',imgaff)
    cv2.waitKey(500)


    print('Reduction du nombre de point en cours...................\n')
    # Debut de la partie reduction du nombre de points
    ListKeyPoints = reduce_list(path, img)

    for i in range(1, len(ListKeyPoints)):
        cv2.line(imgaff,(ListKeyPoints[i-1].x,ListKeyPoints[i-1].y),(ListKeyPoints[i].x,ListKeyPoints[i].y),(255,0,0),2)
        cv2.imshow('map_RRT',imgaff)
        cv2.waitKey(200)
    cv2.imshow('map_RRT',imgaff)
    cv2.waitKey(1000)

    return ListKeyPoints
    #Fin de la partie reduction du nombre de points

    #return ListePoints     #Precedent return


if __name__ == '__main__':
    main()
#POUR LA SUITE :
#Definir l'image, la pos du robot et de l'objectif
#Afficher dans l'image le chemin trouve
#Simplifier le chemin
#Lisser la trajectoire

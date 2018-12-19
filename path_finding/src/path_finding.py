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

class point:
    def __init__(self, xp, yp, ind, pre):
        self.x = xp			
        self.y = yp
        self.indice = ind 	#Position dans la liste des points
        self.preced = pre 	#Indice du point precedent dans la liste des points


#Cree un nouveau point dans une case libre
def randompoint(xmax, ymax, img):
	x = randomchoice(range(xmax))
	y = randomchoice(range(ymax))
	while(img[x][y] != 0):
		x = randomchoice(range(xmax))
		y = randomchoice(range(ymax))
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
	free = True
	for i in range(pinit.x, goal.x):
		for j in range(pinit.y, goal.y):
			if img[i][j] != 0 :
				free = False
	return free

def dqpoint(listprox, goal, dq, img):
	free = False
	xnp = 0
	ynp = 0
	indpre = 0
	while(free == False):
		xnp = listprox[0].x + math.ceil((goal.x - listprox[0].x)*(dq/dist(listprox[0], goal)))
		ynp = listprox[0].y + math.ceil((goal.y - listprox[0].y)*(dq/dist(listprox[0], goal)))

		free = pathfree(listprox[0], point(xnp, ynp, 0, 0), img)

		indpre = listprox[0].indice
		del listprox[0]
	
	return point(xnp, ynp, 0, indpre)


def main():
	img = #TODO

	ximg = #TODO	#taille en x de l'image
	yimg = #TODO	#taille en y de l'image

	imgaff = copy.copy(img)		#Creation d'une image pour l'affichage
	#Conversion des donnes de cet image en couleure RGB
	for i in range ximg:
		for j in range yimg:
			if(imgaff(i, j) == 0):
				imgaff(i,j) = (255,255,255)
			else:
				imgaff(i,j) = (0,0,0)
	cv2.imshow('map',imgaff)

	xrobot = #TODO
	yrobot = #TODO

	goal = point()#TODO

	ListePoints = []	#Liste des points du RRT
	ListePoints.append(point(xrobot, yrobot, len(ListePoints), None))	#Ajout de la pos du Robot comme premier point

	while((ListePoints[-1].x != goal.x) & (ListePoints[-1].y != goal.y)):
		cv2.imshow('map_RRT',imgaff)
		randpoint = randompoint(ximg, yimg, img)					#Lance aleatoire du RRT
		listprox = listeproximite(ListePoints, randpoint)			#Tri par ordre de proximite des points de l'arbre
		ListePoints.append(dqpoint(listprox, randpoint, 5, img))	#Calcul du nouveau point de l'arbre et ajout a la liste
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

#POUR LA SUITE :
#Definir l'image, la pos du robot et de l'objectif
#Afficher dans l'image le chemin trouve
#Simplifier le chemin
#Lisser la trajectoire
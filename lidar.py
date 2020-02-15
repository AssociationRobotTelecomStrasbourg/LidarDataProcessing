import numpy as np
import matplotlib.pyplot as plt
import time as tmps

from data import data

def appartenance(groupes, point) :
    if len(groupes) == 0 :
        groupes.append([point])
        return 

    for i in range (0,len(groupes)) :
        for j in range (0, len(groupes[i])) :
            if ((point[0] - groupes[i][j][0])**2 + (point[1] - groupes[i][j][1])**2 <= 2000) :
                groupes[i].append(point)
                return

    groupes.append([point])
    return
    
def barycentre(groupe) :
    x_bary = 0
    y_bary = 0
    for i in range (0,len(groupe[0])) :
        x_bary += groupe[0][i]
        y_bary += groupe[1][i]
    x_bary /= len(groupe[0])
    y_bary /= len(groupe[0])
    return ([x_bary,y_bary])

def insert_bary(bary,obstacles_reserve) :
    if len(obstacles_reserve) == 0 :
        obstacles_reserve.append([bary])
        return 

    for i in range (0,len(obstacles_reserve)) :
        for j in range (0, len(obstacles_reserve[i])) :
            if ((bary[0] - obstacles_reserve[i][j][0])**2 + (bary[1] - obstacles_reserve[i][j][1])**2 <= 5000) :
                obstacles_reserve[i].append(bary)
                return

    obstacles_reserve.append([bary])
    return



# CONSTANTES
nb_angles = 360
xr = 1000
yr = 500
x_goal = 2000
y_goal = 1600
speed = 3 #pas encore utilisé
dist_secu = 75
pos_r = np.array([0,0])
teta = 0
time = 10
vitesses = []


obstacles_reserve = []

#Pour chaque tour de Lidar
for rotation in range (0, 7) : 
    start = tmps.time()
    x_o = []
    y_o = []
    groupes = []
    angle = 0
    
    # On stock dans le point détecté par le lidar à cet angle si il est sur le terrain et suffisament éloignés du robot
    for i in range (0,360):
        if (data[rotation][angle][1]) > 130 : # si le point détecté est à plus de 13 cm du robot
            pos_r = np.array([xr,yr])
            teta = np.pi/3
            angle_rad = angle * 2 * np.pi / 360 + teta
            mat_rot = np.array([np.cos(angle_rad),np.sin(angle_rad)])
            pos_car = data[rotation][angle][1]*mat_rot+pos_r
            if pos_car[0] > 0 and pos_car[0] < 3000 and pos_car[1] > 0 and pos_car[1] < 2000 : # si le point se trouve sur le terrain
                x_o.append(pos_car[0])
                y_o.append(pos_car[1])
        angle +=1
    
    # Si le point appartient à un groupe de points, on le stock dans ce groupe, sinon on créé un nouveau groupe
    for i in range (0,len(x_o)):
        appartenance(groupes, [x_o[i],y_o[i]])
    
    # On calcul le barycentre de chaque groupe de plus de 10 points (on matérialise l'objet ennemi par un point), que l'on insère dans obstacles_reserve 
    for i, gr in enumerate(groupes):
        if len(gr)>=10 :
            ls_p = [[],[]]
            for p in gr:
                ls_p[0].append(p[0])
                ls_p[1].append(p[1])
            bary = barycentre(ls_p)
            insert_bary(bary,obstacles_reserve)

    trajectoire = [[],[]]
    for pt_x in np.linspace(xr,x_goal,10) :
        trajectoire[0].append(pt_x)

    for pt_y in np.linspace(yr,y_goal,10) :
        trajectoire[1].append(pt_y)


    for obstacle in obstacles_reserve :
        if len(obstacle) >= 3:
            prediction = [[],[]]
            
            vitesse2 = np.sqrt((obstacle[len(obstacle)-1][0]-obstacle[len(obstacle)-2][0])**2+(obstacle[len(obstacle)-1][1]-obstacle[len(obstacle)-2][1])**2)/ time
            vitesse_x2 = (obstacle[len(obstacle)-1][0]-obstacle[len(obstacle)-2][0])/ time
            vitesse_y2 = (obstacle[len(obstacle)-1][1]-obstacle[len(obstacle)-2][1])/ time

            vitesse1 = np.sqrt((obstacle[len(obstacle)-2][0]-obstacle[len(obstacle)-3][0])**2+(obstacle[len(obstacle)-2][1]-obstacle[len(obstacle)-3][1])**2)/ time
            vitesse_x1 = (obstacle[len(obstacle)-2][0]-obstacle[len(obstacle)-3][0])/ time
            vitesse_y1 = (obstacle[len(obstacle)-2][1]-obstacle[len(obstacle)-3][1])/ time

            acceleration = vitesse2-vitesse1/ time
            acceleration_x = vitesse_x2-vitesse_x1/ time
            acceleration_y = vitesse_y2-vitesse_y1/ time
            
            for t in range (0,30,4) :
                prediction[0].append(0.5*acceleration_x * t**2 + vitesse_x2 * t + obstacle[len(obstacle)-1][0])
                prediction[1].append(0.5*acceleration_y * t**2 + vitesse_y2 * t + obstacle[len(obstacle)-1][1])
                circle = plt.Circle((prediction[0][len(prediction[0])-1], prediction[1][len(prediction[0])-1]), dist_secu, color='r',fill=False)
                fig = plt.gcf()
                ax = fig.gca()
                ax.add_artist(circle)

            plt.plot(prediction[0],prediction[1],'r')
            plt.scatter(prediction[0],prediction[1],color = 'r')

            for i in range(0, min(len(trajectoire[0]),len(prediction[0]))) :
                if ((np.sqrt((prediction[0][i] - trajectoire[0][i])**2+(prediction[1][i] - trajectoire[1][i])**2))<=2*dist_secu) :
                    print ("BOOM au",i+1, "ème point")
                    break


    
    end = tmps.time()
    print("Delay =", end-start)
    for i, gr in enumerate(groupes):
        if len(gr)>=10 :
            ls_p = [[],[]]
            for p in gr:
                ls_p[0].append(p[0])
                ls_p[1].append(p[1])
            plt.scatter(*ls_p,marker = "x")
            bary = barycentre(ls_p)
            plt.plot(bary[0],bary[1],'ro')
            circle = plt.Circle((bary[0], bary[1]), dist_secu, color='r',fill=False)
            fig = plt.gcf()
            ax = fig.gca()
            ax.add_artist(circle)
    plt.plot(xr,yr,"k", marker = "D")   
    plt.plot(x_goal,y_goal,"k", marker = "D")
    plt.plot([xr,x_goal], [yr,y_goal], "k")
    
    
    for i in range (0,len(trajectoire[0])) :
        circle = plt.Circle((trajectoire[0][i], trajectoire[1][i]), dist_secu, color='k',fill=False)
        fig = plt.gcf()
        ax = fig.gca()
        ax.add_artist(circle)    
 
    plt.scatter(np.linspace(xr,x_goal,10), np.linspace(yr,y_goal,10), color = 'k')

    
    plt.plot([0,0,3000,3000,0],[0,2000,2000,0,0],"b")
    plt.axis([-100, 3100, -100, 2100])
    plt.show()



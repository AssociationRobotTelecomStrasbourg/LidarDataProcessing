import numpy as np
import matplotlib.pyplot as plt
import time as tmps

def appartenance(groupes, point) :
    if len(groupes) == 0 :
        groupes.append([point])
        return 

    for i in range (0,len(groupes)) :
        for j in range (0, len(groupes[i])) :
            if ((point[0] - groupes[i][j][0])**2 + (point[1] - groupes[i][j][1])**2 <= 900) :
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



# CONSTANTES
nb_angles = 360
xr = 1000
yr = 500
x_goal = 2000
y_goal = 1600
angle = 0
speed = 3 #pas encore utilisé
dist_secu = 75
pos_r = np.array([0,0])
teta = 0
time = 10
obstacles = []
vitesses = []
obstacles_reserve = [[1117,961],[1167,991],[1227,1021]]
for pt in obstacles_reserve :
    if len(obstacles) == 2 :
        obstacles.pop(0)
    obstacles.append(pt)
    if len(obstacles) == 2 :
        vitesse = np.sqrt((obstacles[len(obstacles)-1][0]-obstacles[len(obstacles)-2][0])**2+(obstacles[len(obstacles)-1][1]-obstacles[len(obstacles)-2][1])**2)/ time
        vitesse_x = (obstacles[len(obstacles)-1][0]-obstacles[len(obstacles)-2][0])/ time
        vitesse_y = (obstacles[len(obstacles)-1][1]-obstacles[len(obstacles)-2][1])/ time
        if len(vitesses) == 2 :
            vitesses.pop(0)
        vitesses.append([vitesse,vitesse_x,vitesse_y])
        if len(vitesses) == 2 :
            acceleration = (vitesses[len(vitesses)-1][0]-vitesses[len(vitesses)-2][0])/ time
            acceleration_x = (vitesses[len(vitesses)-1][1]-vitesses[len(vitesses)-2][1])/ time
            acceleration_y = (vitesses[len(vitesses)-1][2]-vitesses[len(vitesses)-2][2])/ time
            print("----------------")
            print("Vitesse : ", vitesse, "cm.s-1")
            print("Vitesse selon x : ", vitesse_x, "cm.s-1")
            print("Vitesse selon y : ", vitesse_y, "cm.s-1")
            print ("Accélération : ", acceleration, "cm.s-2")
            print ("Accélération selon x : ", acceleration_x, "cm.s-2")
            print ("Accélération selon y : ", acceleration_y, "cm.s-2")
plt.plot(1117,961,'bo')
plt.plot(1167,991,'bo')
plt.plot(1227,1021,'go')
obstacles_x = []
obstacles_y = []
for obs in obstacles :
    obstacles_x.append(obs[0])
    obstacles_y.append(obs[1])
# plt.plot(obstacles_x,obstacles_y)
prediction = [[],[]]

for t in range (0,100,10) :
    prediction[0].append(0.5*acceleration_x * t**2 + vitesses[len(vitesses)-1][1] * t + obstacles[len(obstacles)-1][0])
    prediction[1].append(0.5*acceleration_y * t**2 + vitesses[len(vitesses)-1][2] * t + obstacles[len(obstacles)-1][1])
    circle = plt.Circle((prediction[0][len(prediction[0])-1], prediction[1][len(prediction[0])-1]), dist_secu, color='r',fill=False)
    fig = plt.gcf()
    ax = fig.gca()
    ax.add_artist(circle)

plt.plot(prediction[0],prediction[1],'r')
plt.scatter(prediction[0],prediction[1],color = 'r')


for essay in range (0, 1) :
    start = tmps.time()
    dist = np.random.rand(nb_angles)*1000
    x_o = []
    y_o = []
    groupes = []
    
    for i in range (0,360):
        angle = angle % 360
        if (dist[angle]) > 130 :
            pos_r = np.array([xr,yr])
            teta = np.pi/3
            angle_rad = angle * 2 * np.pi / 360 + teta
            mat_rot = np.array([np.cos(angle_rad),np.sin(angle_rad)])
            pos_car = dist[angle]*mat_rot+pos_r
            if pos_car[0] > 0 and pos_car[0] < 3000 and pos_car[1] > 0 and pos_car[1] < 2000 :
                x_o.append(pos_car[0])
                y_o.append(pos_car[1])

        angle +=1
    for i in range (0,len(x_o)):
        appartenance(groupes, [x_o[i],y_o[i]])

    trajectoire = [[],[]]
    for pt_x in np.linspace(xr,x_goal,10) :
        trajectoire[0].append(pt_x)

    for pt_y in np.linspace(yr,y_goal,10) :
        trajectoire[1].append(pt_y)

    for i in range(0, min(len(trajectoire[0]),len(prediction[0]))) :
        if ((np.sqrt((prediction[0][i] - trajectoire[0][i])**2+(prediction[1][i] - trajectoire[1][i])**2))<=2*dist_secu) :
            print ("BOOM au",i+1, "ème point")
            break
    end = tmps.time()
    print("Delay =", end-start)
    for i, gr in enumerate(groupes):
        if len(gr)>=4 :
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


# angle_goal = arctan(delta_y/delta_x)
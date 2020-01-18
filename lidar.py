import numpy as np
import matplotlib.pyplot as plt

import time
start = time.time()

# CONSTANTES
nb_angles = 360
xr = 1000
yr = 500
angle = 0
pos_r = np.array([[0],[0]])
teta = 0
dist = np.random.rand(nb_angles)*1000
print(dist)

for i in range (0,3600):
    angle = angle % 360
    pos_r = np.array([[xr],[yr]])
    teta = np.pi/3
    angle_rad = angle * 2 * np.pi / 360 + teta
    mat_rot = np.array([[np.cos(angle_rad)],[np.sin(angle_rad)]])
    pos_car = dist[angle]*mat_rot+pos_r
    print (pos_car)
    angle +=1




# x = np.linspace(0,2,nb_angles)
# y = np.linspace(0,3,nb_angles)
# plt.plot(x,y)
# plt.show()
end = time.time()
print("Delay =", end-start)
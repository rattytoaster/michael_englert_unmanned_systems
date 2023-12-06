
import numpy as np
import math as m
import matplotlib.pyplot as plt



import RRT
import Djikstra_Astar
import Djikstras
import tqdm



Obstacle_x = [2, 2, 2, 2, 0, 1, 2, 3, 4, 5, 5, 5, 5, 5, 8, 9, 10, 11, 12, 13, 8, 8, 8,
8, 8, 8, 8, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13, 14, 15, 2, 2, 2, 2, 2, 2, 5, 5, 5, 5, 5,
5, 5, 6, 7, 8, 9, 10, 11, 12, 12, 12, 12, 12]

Obstacle_y = [2, 3, 4, 5, 5, 5, 5, 5, 5, 5, 2, 3, 4, 5, 2, 2, 2, 2, 2, 2, 3, 4, 5, 6, 7,
8, 9, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 8, 9, 10, 11, 12, 13, 9, 10, 11, 12, 13,
14, 15, 12, 12, 12, 12, 12, 12, 8, 9, 10, 11, 12]

length = len(Obstacle_x)
obstacles = []

for i in range(0,length-1):
    temp_x = Obstacle_x[i]
    temp_y = Obstacle_y[i]

    temp = [temp_x,temp_y]

    obstacles.append(temp)
    continue



min_x = 0
max_x = 15
min_y = 0
max_y = 15
obs_radius = 0.5
start_x = 1
start_y = 1
goal_x = 7
goal_y = 13
gs = 0.5
robot_radius = 0.4
dl = 0.5    


path_x = []
path_y = []
path = [path_x, path_y]

for i in tqdm.tqdm(range(0, 100)):


    path = Djikstra_Astar.Astar(min_x, max_x, min_y,max_y,obstacles,obs_radius,start_x,start_y,goal_x,goal_y,gs,robot_radius)

    continue

fig, ax = plt.subplots() # note we must use plt.subplots, not plt.subplot
ax.set_xlim(min_x-1, max_x+1)
ax.set_ylim(min_y-1, max_y+1)    
for obs in obstacles:
    obs_plot = plt.Circle((obs[0], obs[1]), obs_radius, color='red')
    ax.add_patch(obs_plot)

start_plot = plt.Circle((start_x, start_y), obs_radius, color='blue')
ax.add_patch(start_plot)

goal_plot = plt.Circle((goal_x, goal_y), obs_radius, color='green')
ax.add_patch(goal_plot)

ax.plot(path[0],path[1])
plt.show()






    





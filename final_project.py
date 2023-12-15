

import numpy as np
import matplotlib.pyplot as plt
import final_project_demo


start_x =-2
start_y =-5

min_x = -5
max_x = 3

min_y = -8
max_y = 0

goal_x  = 0.25
goal_y = -4.75

gs = 0.25
iter = []
#iter.append(0)
iter.append(0)
iteriter=0
obstacle_positions_init =  []
obstacle_positions = []
#obstacle_positions = [] # store obstacle classes
obstacle_radius = 0.2
closing = np.sqrt((start_x-goal_x)**2 + (start_y - goal_y)**2)


path_traversed = []
"""
while closing >= 0.25:
    if iter == 20:
        obstacle_positions = []
        iter[1] = iter[1] + 1
        iter = 0
    obstacle_positions, start, path_traversed_temp = evader_anh_final.tb_nav_final(start_x,start_y,min_x,max_x,min_y,max_y,goal_x,goal_y,gs,obstacle_positions,obstacle_radius, iter)
    
    

    
    start_x = int(start[0])
    start_y = int(start[1])
    closing = np.sqrt((start_x-goal_x)**2 + (start_y - goal_y)**2)
    #print("start", start)
    #print("obstacles", obstacle_positions)
    iter[0] = iter[0] + 1
"""


obstacle_positions = []
#iter[1] = iter[1] + 1
iter = 0
obstacle_positions, start, path_traversed_temp, init_wp_list = final_project_demo.tb_nav_final(
    start_x,start_y,min_x,max_x,min_y,max_y,goal_x,goal_y,gs,obstacle_positions,obstacle_radius, iter)




start_x = int(start[0])
start_y = int(start[1])
closing = np.sqrt((start_x-goal_x)**2 + (start_y - goal_y)**2)
#print("start", start)
#print("obstacles", obstacle_positions)
#iter[0] = iter[0] + 1

"""


fig = plt.figure()
ax1 = fig.add_subplot(111)

start_x =0
start_y =0

min_x = -2
max_x = 15

min_y = -2
max_y = 15

goal_x  = 4
goal_y = 4

gs = 1.0
iter = []
iter.append(0)
iter.append(0)
iteriter=0
obstacle_positions_init =  []
obstacle_positions = []
#obstacle_positions = [] # store obstacle classes
obstacle_radius = 0.5
closing = np.sqrt((start_x-goal_x)**2 + (start_y - goal_y)**2)


#path_traversed = []






obstacle_positions, start, path_traversed_temp, init_wp_list = final_project_demo.tb_nav_final(start_x,start_y,min_x,max_x,min_y,max_y,goal_x,goal_y,gs,obstacle_positions,obstacle_radius, iter)

init_wp_list_x = []
init_wp_list_y = []
for init_wp in init_wp_list:
    init_wp_list_x.append(init_wp[0])
    init_wp_list_y.append(init_wp[1])


path_traversed_x = []
path_traversed_y = []
for wp in path_traversed_temp:
    path_traversed_x.append(wp[0])
    path_traversed_y.append(wp[1])

ax1.plot(init_wp_list_x,init_wp_list_y, label = 'init path')
ax1.plot(path_traversed_x,path_traversed_y, label = 'actual path')

for obs in obstacle_positions:
    ax1.scatter(obs[0],obs[1], s=30, color = 'red')
#print("path traversed", path_traversed_temp)
ax1.legend()
plt.show()
    
#start_x = int(start[0])
#start_y = int(start[1])
closing = np.sqrt((start_x-goal_x)**2 + (start_y - goal_y)**2)
#print("start", start)
#print("obstacles", obstacle_positions)
iter[0] = iter[0] + 1 
"""

fig = plt.figure()
ax1 = fig.add_subplot(111)


init_wp_list_x = []
init_wp_list_y = []
for init_wp in init_wp_list:
    init_wp_list_x.append(init_wp[0])
    init_wp_list_y.append(init_wp[1])


path_traversed_x = []
path_traversed_y = []
for wp in path_traversed_temp:
    path_traversed_x.append(wp[0])
    path_traversed_y.append(wp[1])

ax1.plot(init_wp_list_x,init_wp_list_y, label = 'init path')
ax1.plot(path_traversed_x,path_traversed_y, label = 'actual path')
print("obs list len", len(obstacle_positions))
for obs in obstacle_positions:
    ax1.scatter(obs[0],obs[1], s=10, color = 'red')
#print("path traversed", path_traversed_temp)
ax1.legend()
plt.show()
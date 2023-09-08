

import numpy as np
import matplotlib.pyplot as plt

class Node():
    def __init__(self, x_pos:float, y_pos:float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        
        
class Obstacle():
    def __init__(self, x_pos:float, y_pos:float, radius:float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius
                
    
def is_not_valid(obst_list:list,x_min:int, y_min:int, x_max:int, y_max:int,
                 node_curr, robot_radius:float=0.0):

    
    # check if near obstacle or inside
    for obs in obst_list:
        dist_from = np.sqrt((node_curr.x_pos - obs.x_pos)**2 +
                            (node_curr.y_pos - obs.y_pos)**2)
                
        if dist_from < robot_radius:
            return True
    
    if x_min > node_curr.x_pos:
        return True
    if x_max < node_curr.x_pos:
        return True
    if y_min > node_curr.y_pos:
        return True
    if y_max < node_curr.y_pos:
        return True
    
    print("the position is valid")

    return False

if __name__=='__main__':
    
    obstacle_positions =  [(1,1), (4,4), (3,4), (5,0), (5,1), (0,7), (1,7), 
                           (2,7), (3,7)]
    
    obstacle_list = [] # store obstacle classes
    obstacle_radius = 0.25
    
    # Loop through position of obstacles
    for obs_pos in obstacle_positions:
        # print("obstacle_positions", obs_pos)
        #store obstacle information into obstacle list
        obstacle = Obstacle(obs_pos[0], obs_pos[1], obstacle_radius)
        obstacle_list.append(obstacle)
        
    agent_node = Node(1.5,1.5)
    agent_radius = 0.1
    

        
x_min = 0
x_max = 10

y_min = 0
y_max = 10

is_not_valid(obstacle_list, x_min, y_min, x_max, y_max, agent_node, obstacle_radius)


fig, ax = plt.subplots() # note we must use plt.subplots, not plt.subplot
ax.set_xlim(x_min, x_max)
ax.set_ylim(x_min, x_max)    
for obs in obstacle_list:
    obs_plot = plt.Circle((obs.x_pos, obs.y_pos), obs.radius, color='blue')
    ax.add_patch(obs_plot)
    
agent_plot = plt.Circle((agent_node.x_pos, agent_node.y_pos), agent_radius, color='red')
ax.add_patch(agent_plot)
plt.show()

        

        
        
    

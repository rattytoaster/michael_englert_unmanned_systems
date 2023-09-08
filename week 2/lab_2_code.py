# -*- coding: utf-8 -*-
"""
Created on Thu Aug 31 10:42:02 2023

@author: michael englert
"""
import numpy as np
import matplotlib.pyplot as plt






class Node():
    def __init__(self, x_pos:float, y_pos:float, cost:float, 
                 parent_index:int) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.cost = cost
        self.parent_index = parent_index
        
        
class Obstacle():
    def __init__(self, x_pos:float, y_pos:float, radius:float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius
        
        
def compute_distance(x_1:float, y_1:float, x_2:float, y_2:float) -> float:
    
        distance = np.sqrt((x_1-x_2)**2 + (y_1-y_2)**2)
    
        return distance
                
    
def is_not_valid(obst_list:list,x_min:int, y_min:int, x_max:int, y_max:int,
                 x_pos, y_pos, robot_radius:float):


    
    # check if near obstacle or inside
    for obs in obst_list:
        dist_from = np.sqrt((x_pos - obs.x_pos)**2 +
                            (y_pos - obs.y_pos)**2)
                
        if dist_from < robot_radius:
            print("hit an object")
            return True
        
    
    if x_min > x_pos:
        print("the position is not valid")
        return True
    
    if x_max < x_pos:
        print("the position is not valid")
        return True
    
    if y_min > y_pos:
        print("the position is not valid")
        return True
    
    if y_max < y_pos:
        print("the position is not valid")
        return True
    
    print("the position is valid")

    return False

def node_indexer(node_curr, x_min:float, x_max:float, y_min, y_max, gs:float):
    
            index = ((node_curr.x_pos - x_min)/gs) + \
                (((node_curr.y_pos - y_min)/gs)* ( ((x_max + gs) - x_min)/gs))
            
            return index
            
                  
                 
            
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




start_x = 0
start_y = 0
goal_x = 9
goal_y = 9

x_min = 0
x_max = 10
y_min = 0
y_max = 10

gs = 0.5
robot_radius = 0.1



    
start = Node(start_x, start_y,0, -1)
goal = Node(goal_x, goal_y, 0, 0)
    
    
#create unvisited nodes dict
unvisited_nodes = {}
visited_nodes = {} 

    

node_curr = start
index_curr = int(node_indexer(node_curr, x_min, x_max, y_min, y_max, gs))
unvisited_nodes[index_curr] = start

while [node_curr.x_pos, node_curr.y_pos] != [goal.x_pos, goal.y_pos]:
    index_curr = min(unvisited_nodes, key=lambda x:unvisited_nodes[x].cost)
    node_curr = unvisited_nodes[index_curr]
    
    visited_nodes[index_curr] = node_curr
    
    del unvisited_nodes[index_curr]
    
    for i in range(-1,2):
        for j in range(-1,2):
                
                
            cost_curr = compute_distance(node_curr.x_pos, node_curr.y_pos, node_curr.x_pos + i*gs, node_curr.y_pos + j*gs)
            
            node_temp = Node(node_curr.x_pos + i*gs, node_curr.y_pos + j*gs, cost_curr, index_curr)
            
            index_temp = node_indexer(node_temp, x_min, x_max, y_min, y_max, gs)
            
            print("the index is", index_temp)
            
            node_temp = Node(node_curr.x_pos + i*gs, node_curr.y_pos + j*gs, 
                             compute_distance(node_curr.x_pos,
                             node_curr.y_pos, node_temp.x_pos, 
                             node_temp.y_pos), index_temp)
                
            print(node_temp.x_pos, node_temp.y_pos)
            

            
            if [i,j] != [0,0]:

                if is_not_valid(obstacle_list, x_min, y_min, x_max, y_max, node_temp.x_pos, node_temp.y_pos, robot_radius) == False:

                    if index_temp in visited_nodes  and visited_nodes(index_temp).cost > node_temp.cost:

                        visited_nodes(index_temp) == node_temp

                    print('1')                        

if index_temp in unvisited_nodes:
# compare new_cost to unvisited cost:
# if new_cost < unvisited cost:
    if node_temp.cost < unvisited_nodes[index_temp].cost:
        #update the cost value
        unvisited_nodes[index_temp].cost = node_temp.cost
        #update the parent index
        unvisited_nodes[index_temp].parent_index = index_temp

    # continue to next move since it already exists
    continue  
                        


                            
    
    
    
    





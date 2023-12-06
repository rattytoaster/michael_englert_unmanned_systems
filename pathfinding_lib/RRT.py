import numpy as np 
import math as m
import matplotlib.pyplot as plt
import random




class RRT():
    def __init__(self, min_x: float, max_x: float, min_y:float, max_y:float, 
        obstacle_positions:list, obs_radius:float, start_x:float, start_y:float, goal_x:float, 
        goal_y:float, gs:float, robot_radius:float):
        self.min_x = min_x
        self.min_y = min_y
        self.max_x = max_x
        self.max_y = max_y
        self.obstacle_positions = obstacle_positions
        self.obs_radius = obs_radius
        self.start_x = start_x
        self.start_y = start_y
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.gs = gs
        self.robot_radius = robot_radius


        

    
    
    
    
    


def rrt(min_x,max_x,min_y,max_y,obstacle_positions,obs_radius,start_x,start_y,goal_x,goal_y,gs,robot_radius,dl):

    class Node():
        def __init__(self, 
                    x:float ,
                    y:float,
                    cost:float, 
                    parent_idx:int) -> None:
            self.x = x
            self.y = y 
            self.cost = cost
            self.parent_idx = int(parent_idx)
        
    class Obstacle():
        def __init__(self, x_pos:float, y_pos:float, radius:float) -> None:
            self.x_pos = x_pos
            self.y_pos = y_pos
            self.radius = radius
            
        def is_inside(self,curr_x:float, curr_y:float, 
                    robot_radius:float=0) -> bool:
            """
            Compute euclidean distance from current location -> obstacle locatin
            Store this value in a variable: dist_from 
            If dist_from > obstacle size:
                    return false
            Otherwise return true
            """
            dist_from = np.sqrt((curr_x - self.x_pos)**2 + (curr_y - self.y_pos)**2)
            
            if dist_from > self.radius + robot_radius:
                return False
            
            return True

    def compute_index(min_x:int, max_x:int, min_y:int, max_y:int,
                    gs:float, curr_x:int, curr_y:int) -> float:

        index = ((curr_x - min_x)/gs) + ((curr_y - min_y)/gs) * ((max_x+gs - min_x)/gs)
        
        return index

    def get_all_moves(current_x:float, current_y:float, gs:float) -> list:
        """
        Inputs: 
            - current x and current y 
            - gridspace/step_size
            
        How should we do this:
            Hint a nested for loop helps and arange helps
        
        Returns a list of moves:
            - [[m1], [m2], [m3]..., [mn]]
        
        """
        
        move_list = []
        gs_x_bounds = np.arange(-gs, gs+gs, gs)
        gs_y_bounds = np.arange(-gs, gs+gs, gs)
        
        for dx in gs_x_bounds:
            for dy in gs_y_bounds:
                x_next = current_x + dx
                y_next = current_y + dy
                
                if [x_next, y_next] == [current_x, current_y]:
                    continue
                
                move = [x_next, y_next]
                move_list.append(move)
                
        return move_list
                
    def is_not_valid(obst_list:list, 
                    x_min:int, 
                    y_min:int, 
                    x_max:int, 
                    y_max:int,
                    x_curr:float, 
                    y_curr:float, 
                    agent_radius:float=0.0):
        """
        - True if not valid
        - False if valid
        
        - Check if inside/near obstacles -> done 
        - Check if outside the boundary:
            - Check x pos:
                - Compare to x min and xmax 
                    - If x_min > x_pos: 
                        Return True
                    - If x_max < x_pos:
                        Return True
                - Do the same for y position
        """
        
        # check if near obstacle or inside
        for obs in obst_list:
            if obs.is_inside(x_curr, y_curr,agent_radius):

                return True
        
        if x_min + agent_radius> x_curr:
            return True
        if x_max - agent_radius < x_curr:
            return True
        if y_min  + agent_radius > y_curr:
            return True
        if y_max - agent_radius < y_curr:
            return True

        return False


        
    def compute_distance(x_1:float, y_1:float, x_2:float, y_2:float) -> float:
        
            distance = np.sqrt((x_1-x_2)**2 + (y_1-y_2)**2)
        
            return distance
        
        #initialize obstacle list
    obstacle_list = [] # store obstacle classes

        # Loop through position of obstacles to make a list




    for obs_pos in obstacle_positions:
            obstacle = Obstacle(obs_pos[0], obs_pos[1], obs_radius)
            obstacle_list.append(obstacle)



    #initialize tree
    tree = {}

    #initialize tree w/ start 
    index_curr = compute_index(min_x, max_x, min_y, max_y, gs, start_x, start_y)
    node_curr = Node(start_x, start_y, 0, -1)

    tree[index_curr] = node_curr



    while (tree[index_curr].x, tree[index_curr].y) != (goal_x, goal_y):
    #for i in range(1, 100):
        
        #generating random point
        rand_x = random.uniform(min_x, max_x)
        rand_y = random.uniform(min_y, max_y)
        

        
        #plt.scatter(rand_x, rand_y, c="black")
        
        #finding closest node in tree to random point
        dist_min = compute_distance(tree[index_curr].x, tree[index_curr].y, rand_x, rand_y)      
        for index in tree:
            

            
            dist_curr = compute_distance(tree[index].x, tree[index].y, rand_x, rand_y)
            

            
            if dist_min >= dist_curr:

                dist_min = dist_curr
                index_close = index
                
        #calculating angle from closest node to random point
        theta = np.arctan2((rand_x - tree[index_close].x), (rand_y - tree[index_close].y))
        

        
        #jump in direction of thet by dl, rounding to nearest gs
        
        x_new = int((1/gs)*dl*np.sin(theta))*gs + tree[index_close].x
        y_new = int((1/gs)*dl*np.cos(theta))*gs + tree[index_close].y
        

        
        #making sure new node doesnt already exist, is valid
        if is_not_valid(obstacle_list, min_x, min_y, max_x, max_y, x_new, y_new) == False :
            rand_idx = compute_index(min_x, max_x, min_y, max_y, gs, x_new, y_new)
            
            if rand_idx in tree:
                continue
            else:
                index_curr = rand_idx
                node_curr = Node(x_new, y_new, 0, index_close)
                
                tree[index_curr] = node_curr
                

            

        
        
        

        
        
    #plotting path from start to goal
    path_x = []
    path_y = []
            
    path_x.append(node_curr.x)
    path_y.append(node_curr.y)

    while node_curr.parent_idx != -1:
        






        previous_node = node_curr
        temp_idx = previous_node.parent_idx
        node_curr = tree[temp_idx]
        
        path_x.append(node_curr.x)
        path_y.append(node_curr.y)

    path = [path_x, path_y]
    return path
        
                

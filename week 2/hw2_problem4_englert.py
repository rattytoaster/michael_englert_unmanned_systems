# -*- coding: utf-8 -*-
"""
Created on Fri Sep  8 08:01:09 2023

@author: michael englert
"""






import numpy as np 
import math as m
import matplotlib.pyplot as plt















def djikstras(min_x, max_x, min_y, max_y, 
              obstacle_positions, obs_radius, start_x, start_y, goal_x, 
              goal_y, gs, robot_radius):
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
                #print("You're dead at ", obs.x_pos, obs.y_pos)
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
    
    
    #initialize obstacle list
    obstacle_list = [] # store obstacle classes
    
    # Loop through position of obstacles
    for obs_pos in obstacle_positions:
        obstacle = Obstacle(obs_pos[0], obs_pos[1], obs_radius)
        obstacle_list.append(obstacle)
        




    # - Make two bins/dictionaries:
    unvisited = {}
    visited = {}


    # - Initialize current_node with the following parameters:
        # - position = start position
        # - cost = 0
        # - parent_index = -1
    current_node = Node(start_x, start_y, 0, int(-1))


    # - initialize current_index by utilizing compute_index() function 
    # based on the current position, which is the start 
    current_idx = int(compute_index(min_x, max_x, min_y, max_y,
                    gs, start_x, start_y))


    # - insert current_node into unvisited dictionary, 
    # use the current_index as the key
    unvisited[current_idx] = current_node

    # - While current node position is not equal to goal location:

    #While current node position is not equal to goal location:
    while [current_node.x, current_node.y] != [goal_x, goal_y]:

        # set current_index to min value from unvisited 
        current_idx = min(unvisited, key=lambda x:unvisited[x].cost)    

        # - set current_node to unvisited[current_index]
        current_node = unvisited[current_idx]

        # - put current_node into visited dictionary
        visited[current_idx] = current_node
        
        # - delete current_index from unvisited 
        del unvisited[current_idx] 
        
        if [current_node.x, current_node.y] == [goal_x, goal_y]:
            #print("YAY you found it =)")
            
            wp_node = current_node
            wp_list = []
            wp_list.append([wp_node.x, wp_node.y])
            
            while wp_node.parent_idx != -1:
                next_idx = wp_node.parent_idx
                wp_node  = visited[next_idx]            
                wp_list.append([wp_node.x, wp_node.y])
            break
        
        # Begin search by doing th following:
        # - Use get_all_moves() to get all moves based on current position
        all_moves = get_all_moves(current_node.x, current_node.y, gs)

        #initialize a filtered_move list 
        filtered_moves = []


        # - With all moves check if move is_not_valid by using is_not_valid() function
        #     - This function should check if:
        #         - Inside obstacle
        #         - Outside boundary 
        #         - Not on top of ourselves(already done by get_all moves)
        for move in all_moves:
            if (is_not_valid(obstacle_list, min_x, min_y, max_x, max_y, 
                            move[0], move[1], robot_radius) == True):
                continue
            else:
                #print("good move", move[0], move[1])
                #If move is valid we append to filtered 
                filtered_moves.append(move)
                
        #  - loop through all filtered moves:
        for move in filtered_moves:
            # based on this current filtered move:
            
            # calculate the filtered/new index
            new_index = int(compute_index(min_x, max_x, min_y, max_y,
                    gs, move[0], move[1]))
            
            # calculate the filtered/new cost
            # from + to new_node
            new_cost = current_node.cost + m.dist(move, [current_node.x, current_node.y])

            #check if new index is in visited
            if new_index in visited:
                continue

            # - if inside unvisited:        
            if new_index in unvisited:
                # compare new_cost to unvisited cost:
                # if new_cost < unvisited cost:
                if new_cost < unvisited[new_index].cost:
                    #update the cost value
                    unvisited[new_index].cost = new_cost
                    #update the parent index
                    unvisited[new_index].parent_idx = current_idx
                    
                # continue to next move since it already exists
                continue    
            
            # - this last condition means that we have a node so 
            #     make a new node and append to unvisited
            new_node = Node(move[0], move[1], new_cost, current_idx)
            unvisited[new_index] = new_node 


    fig, ax = plt.subplots() # note we must use plt.subplots, not plt.subplot
    ax.set_xlim(min_x-1, max_x+1)
    ax.set_ylim(min_y-1, max_y+1)    
    for obs in obstacle_list:
        obs_plot = plt.Circle((obs.x_pos, obs.y_pos), obs.radius, color='red')
        ax.add_patch(obs_plot)
            
            
            
    path_x = []
    path_y = []
            
    path_x.append(current_node.x)
    path_y.append(current_node.y)

    while current_node.parent_idx != -1:
        
        #print(current_node.parent_idx)





        previous_node = current_node
        temp_idx = previous_node.parent_idx
        current_node = visited[temp_idx]
        
        path_x.append(current_node.x)
        path_y.append(current_node.y)



    path = plt.plot(path_x,path_y)

    start_plot = plt.Circle((start_x, start_y), obs.radius, color='blue')
    ax.add_patch(start_plot)
    goal_plot = plt.Circle((goal_x, goal_y), obs.radius, color='green')
    ax.add_patch(goal_plot)

    plt.show()
    
    
    
    
    

# initialize some parameters
start_x = 1
start_y = 4

min_x = 0
max_x = 10

min_y = 0
max_y = 10

goal_x = 8
goal_y = 9

gs = 0.5
robot_radius = 0.5

obstacle_positions =  [(1,1), (4,4), (3,4), (5,0), (5,1), (0,7), (1,7), (2,7)]
obstacle_list = [] # store obstacle classes
obs_radius = 0.5
    
djikstras(min_x, max_x, min_y, max_y, 
              obstacle_positions, obs_radius, start_x, start_y, goal_x, 
              goal_y, gs, robot_radius)
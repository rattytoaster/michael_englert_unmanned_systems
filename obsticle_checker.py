# -*- coding: utf-8 -*-
"""
Created on Wed Aug 30 08:15:47 2023

@author: michael englert
"""
import numpy as np
import matplotlib.pyplot as plt


class Obsticle():
    def __init__(self, x_pos:float, y_pos:float, radius:float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius
        
    def is_inside(self, curr_x:float, curr_y:float, robot_radius:float):
            

    
            dist_from = np.sqrt((curr_x - self.x_pos)**2 + (curr_y - self.y_pos)**2)
    
            if dist_from + robot_radius > self.radius:
        
                return False
    
            return True
        
if __name__=='__main__':
    obsticle_positions = [(1,1), (4,4), (3,4), (5,0)]
    
    
    obsticle_list = []
    
    obsticle_radius = 0.25
    # loop through position of obsticles
    for obs_pos in obsticle_positions:
        print("obsticle_positions", obs_pos)
        #store obsticle position into obsticle list
        obsticle = Obsticle(obs_pos[0], obs_pos[1], obsticle_radius)
        obsticle_list.append(obsticle)
        
agent_x = 1
agent_y = 1.2
agent_radius = 0.5

obs_x = 1
obs_y = 1

for obs in obsticle_list:
    print("this obsticles position is", obs.x_pos, obs.y_pos)
    
    if obs.is_inside(agent_x, agent_y, agent_radius):
            print("youre inside the obsticle at", obs.x_pos, obs.y_pos)
            
    break

fig, ax = plt.subplots()
ax.set_xlim(0,10)
ax.set_ylim(0,10)
for obs in obsticle_list:
    obs_plot = plt.Circle((obs.x_pos, obs.y_pos), obs.radius, color='blue')
    ax.add_patch(obs_plot)
                           
                           
                           
# plot circle takes in a tuple of coordinates and radius
obsticle_plot = plt.Circle((obs_x, obs_y), color='blue')
fig, ax = plt.subplots()
ax.scatter(agent_x, agent_y, c='r')

ax.add_patch(obsticle_plot)



ax.set_xlim(0,10)
ax.set_ylim(0,10)
agent_plot = plt.Circle(agent_x, agent_y)



plt.show()


"""
    writing function tips
        -is_position_valid()
        -what are our inputs?
            -agent position
            -agent radius
            -obsticles
            -obsticle radius
            -x limits
            -y limits
        -what are out outputs?
            -true/false
            -true if not valid
            -false if valid
            
        -what will happen in blackbox/function?
            -check if inside/near any obsticles -> done
            -check if outside boundary
                -check x pos:
                    -compare to xmin and xmax
                        -if xmin>x_pos
                            return true
                        if xmax<x_pos
                            return true
                        -if ymin>y_pos
                            return true
                        if ymax<y_pos
                            return true
        
        
        
"""

def is_not_valid(obst_list:list, x_min:int, x_max:int, y_min:int, y_max:int, 
                 x_curr:float, y_curr:float, agent_radius:float):
    
    for obs in obst_list:
        print("this is the obsticles position: ")
    
    if obs.is_inside(x_curr, y_curr, agent_radius):
       print("youre dead at ", obs.x_pos, obs.y_pos)
       return True
   
    if x_min > x_curr:
        return True
    if x_max < x_curr:
        return True
    if y_min > y_curr:
        return True
    if y_max < y_curr:
        return True
    
        
        
    
    """
    -check if outside boundary
        -check x pos:
            -compare to xmin and xmax
                -if xmin>x_pos
                    return true
                if xmax<x_pos
                    return true
                -if ymin>y_pos
                    return true
                if ymax<y_pos
                    return true
    """
        
        
        
        
        
        
        
        
        
        
        
        
        
        
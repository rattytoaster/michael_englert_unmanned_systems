


"""
hw2 problem 1

    writing function tips
        -the way you think of writing a function is:
                -what are my inputs?
                -what are my outputs?
            
                -what will happen inside the blackbox/function?
    for function is_inside_obsticle():
        -what are our inputs?
            -obsticle locations [x,y]
            -obsticle size: radius
            -current location [x,y]
            
        -what are our outputs?
            -true/false
            -if inside: return true
            -if outside: return false 
            
        -what will happen inside the blackbox
            -compute euclidian distance between the current location to the obsticle location
            -store this value in a variable: dist_from
            -if dist_from > obsticle_size
                return false
            -otherwise return true
    
    
            
"""

import numpy as np

import matplotlib.pyplot as plt



def is_inside_obsticle(obs_x:float, obs_y:float, obs_radius:float, 
                       curr_x:float, curr_y:float) -> bool:
    """
            -compute euclidian distance between the current location to the obsticle location
            -store this value in a variable: dist_from
            -if dist_from > obsticle_size
                return false
            -otherwise return true
    """
    
    dist_from = np.sqrt((curr_x - obs_x)**2 + (curr_y - obs_y)**2)
    
    if dist_from > obs_radius:
        
        return False
    
    return True

obs_x = 5
obs_y = 5
obs_radius = 4

curr_x = 1
curr_y = 1

agent_x = 11
agent_y = 11

x_min = 0
x_max = 20

y_min = 0
y_max = 20

# plot circle takes in a tuple of coordinates and radius
obsticle_plot = plt.Circle((obs_x, obs_y), color='blue')
fig, ax = plt.subplots()
ax.scatter(agent_x, agent_y, c='r')

ax.add_patch(obsticle_plot)



ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, y_max)
agent_plot = plt.Circle(agent_x, agent_y)



plt.show()

if is_inside_obsticle(obs_x, obs_y, obs_radius, agent_x, agent_y):
    print("youre dead")
else:
    print("youre ok")


"""
making the obsticle method better
"""

class Obsticle():
    def __init__(self, x_pos:float, y_pos:float, radius:float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        self.radius = radius
        
    def is_inside(self, curr_x:float, curr_y:float):
            

    
            dist_from = np.sqrt((curr_x - self.x_pos)**2 + (curr_y - self.y_pos)**2)
    
            if dist_from > obs_radius:
        
                return False
    
            return True

obsticle_object = Obsticle(obs_x, obs_y, obs_radius)
                           





some_obs = Obsticle(obs_x, obs_y, obs_radius)
    
if some_obs.is_inside(agent_x, agent_y):
        print("youre dead")
else:
        print("youre ok")



print("this obsticles position is", obsticle_object.x_pos, obsticle_object.y_pos)
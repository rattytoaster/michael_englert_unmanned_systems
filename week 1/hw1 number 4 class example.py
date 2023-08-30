#hw1 problem 4




#import stuff here
import matplotlib.pyplot as plt
import numpy as np


#classes
def compute_index(min_x:int, max_x:int, min_y:int, max_y:int, 
                   gs:int, x_curr:int, y_curr:int) ->int:
    index = ((x_curr - min_x)/gs) + \
         (((y_curr - min_y)/gs)* ( ((max_x + gs) - min_x)/gs))
         
    return index




min_x = 0
max_x = 10
gs = 0.5

min_y = 0
max_y = 10

plt.axis([min_x,max_x+gs,min_y,max_y+gs])

for i in range(int((min_y/gs)), int((max_y/gs)+1)):
    for j in range(int(min_x/gs), int((max_x/gs)+1)):
        index = compute_index(min_x, max_x, min_y, max_y, gs, j, i)*gs
        plt.text(j*gs, i*gs, str(int(index)), color="red", fontsize=8)




import numpy as np
import matplotlib.pyplot as plt

class Node():
    def __init__(self, x_pos:float, y_pos:float) -> None:
        self.x_pos = x_pos
        self.y_pos = y_pos
        


def compute_distance(x_1:float, y_1:float, x_2:float, y_2:float) -> float:
    
        distance = np.sqrt((x_1-x_2)**2 + (y_1-y_2)**2)
    
        return distance
    
node_1 = Node(2,1)
node_2 = Node(3,2)



node_dist = compute_distance((node_1.x_pos), (node_1.y_pos), (node_2.x_pos), (node_2.y_pos))

print('the nodes are ', node_dist, 'apart')
    



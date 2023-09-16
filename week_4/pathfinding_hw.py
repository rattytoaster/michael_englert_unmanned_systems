from pathfinding_lib.Node import Node

from pathfinding_lib.Obstacle import Obstacle

from pathfinding_lib.Djikstras import Djikstras

node = Node(0,0,-1,0)

some_obs = Obstacle(2, 2, 5)

#check if is_inside works
print(some_obs.is_inside(1,1))

djikstras = Djikstras()
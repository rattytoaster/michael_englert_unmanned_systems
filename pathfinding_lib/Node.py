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
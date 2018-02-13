import networkx as nx 


class Pathfinder: 

    graph = None  # Graph representation of the map


    # Assign a graph - map - to the pathfinder
    def populate(self, newGraph):
        self.graph = newGraph


    # Find a path 
    def findPath(self, origin, target):
        return nx.astar_path(self.G,origin, target, heuristic)

    # Can swap out heuristic functions
    def heuristic(a,b):
        return euclidean(a,b)


    # Euclidean distance.
    # Accurate, but a bit expensive because of the sqrt()
    def euclidean(a, b):
        dx = abs(a.x - b.x)
        dy = abs(a.y - b.y)
        return D * sqrt(dx * dx + dy * dy)
  
    # Manhattan distance
    def manhattan(a, b):
        return abs(a.x - b.x) + abs(a.y - b.y)

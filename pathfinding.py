import networkx as nx 
import matplotlib.pyplot as plt
from math import sqrt


# Heuristic parameter - might be worth a second look:
# http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#manhattan-distance
D = 1


class Pathfinder: 

    G = None  # Graph representation of the map
    verts = None # Vertices indexed by their ID

    # Assign a graph - map - to the pathfinder
    def populate(self, newGraph, newVerts):
        self.G = newGraph
        self.verts = newVerts

    # Find a path 
    def findPath(self, origin, target):
        return nx.astar_path(self.G,origin, target, heuristic=self.heuristic)

    # Can swap out heuristic functions
    def heuristic(x,a,b):
        return euclidean(a,b)


def readFile(fn):
    verts = dict()
    G = nx.Graph()

    # Read file
    lines  = open(fn).readlines()
    numVertices =int(lines[0])

    # Add vertices
    for i in range (1, numVertices+1):
        (x, y) = lines[i].replace("\n","").split(',')
        vertex = (i-1,int(x),int(y))
        G.add_node(vertex)
        verts[i-1] = vertex
        print(vertex)

    # Add edges
    for i in range(0, numVertices):
        for j, c in enumerate(lines[i+numVertices+1].split(',')):
            if(c =='1'):
                # There is an edge from i to j
                G.add_edge(verts[i], verts[j], weight=euclidean(verts[i], verts[j]))

    return (G, verts)
        

## Helper functions

# Euclidean distance.
# Accurate, but a bit expensive because of the sqrt()
def euclidean(a, b):
    dx = abs(a[1] - b[1])
    dy = abs(a[2] - b[2])
    return D * sqrt(dx * dx + dy * dy)

# Manhattan distance
def manhattan(a, b):
    return abs(a.x - b.x) + abs(a.y - b.y)


# Debug visualizer
def visualize(fn):
    # Create pf
    pf = Pathfinder()

    # This assignment is important!
    (graph, verts) = readFile(fn)
    pf.populate(graph, verts)
    
    # Visualize
    plt.figure()
    nx.draw(graph, with_labels=True)
    plt.show()

    print(pf.findPath(verts[1], verts[15]))


# Debug - visualize current graph
visualize("graph.txt")
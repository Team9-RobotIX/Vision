import networkx as nx
import numpy as np
from math import sqrt
import analysis
import requests

class Planner:

    def __init__(self, robot, camera):
        self.targets = []
        self.pathDetector = analysis.PathDetector(robot, camera)
        self.robotDetector = self.pathDetector.robotDetector

    def createGraph(self):
        graph = nx.Graph()
        verts = []
        for i,vertex in enumerate(self.vertices):
            vert = (i,vertex[0],vertex[1])
            graph.add_node(vert)
            verts.append(vert)
        for i in range(len(self.vertices)):
            for j in range(len(self.vertices)):
                if self.matrix[i,j] == 1:
                    graph.add_edge(verts[i], verts[j], length=self.euclidean(verts[i], verts[j]))
        for i in range(len(verts)):
            print(i, verts[i])
        print('----------')
        for targ in self.targets:
            print(targ.center)
        print('----------')
        return graph

    def euclidean(self, a, b):
        D = 1
        dx = abs(a[1] - b[1])
        dy = abs(a[2] - b[2])
        return D * sqrt(dx * dx + dy * dy)

    def getPaths(self):
        print(39)
        self.pathDetector.recreateGraph()
        self.vertices = self.pathDetector.vertices
        self.matrix = self.pathDetector.matrix
        self.targets = self.pathDetector.targets
        self.graph = self.createGraph()
        self.nonTargets = len(self.vertices) - len(self.targets)

    def plan(self, targetName):
        print(47, targetName)
        self.getPaths()
        startVert = self.vertices[0]
        start = (0,startVert[0],startVert[1])
        end = None
        for i in range(len(self.targets)):
            if self.targets[i].name == targetName:
                end = self.nonTargets + i
                break
        end = (end, self.vertices[end][0], self.vertices[end][1])
        points = nx.astar_path(self.graph, start, end, heuristic=self.euclidean, weight='length')
        print(points)
        return points


    def getFrame(self):
        return self.pathDetector.getFrame()

    def nearest(self):
        (cx,cy) = self.robotDetector.center()
        center = (0,cx,cy)
        minIndex = -1
        minDistance = 10000000000
        for i, vertex in enumerate(self.vertices):
            vert = (0,vertex[0],vertex[1])
            dist = self.euclidean(center,vertex)

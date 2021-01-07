import multiprocessing
import time
import random
import numpy as np

class Node:
    def __init__(self, id, degree):
        self.id = id,
        self.isColored = False
        self.degree = degree
        self.neighbours = {}
        self.available_colors = list(range(1, degree +1))

class Graph:
    def __init__ (self, G):
        self.G = G
        self.generate_nodes(self.calc_degrees())

    def generate_nodes(self, degrees):
        nodes = list([i for i in range(len(self.G[0]))])
        self.V = [Node(i, degrees[i]) for i in nodes]

    def calc_degrees(self):
        return multiprocessing.Pool().map(self.calc_degree, range(len(self.G[0])))

    def calc_degree(self, vertex):
        return sum(self.G[vertex])


def initialize_graph(G):
    graph = Graph(G)
    return graph

## Color graph function
## Preveri če so vozlišča obarvana
## Obarvaj vozlišče
## pošli sporočilo
## prejmi sporočilo
## Če imam najmanjšo izbrano barvo jo izberem
## ponovi klic

def randomised_coloring(graph):
    print("drek")

    #Check if every node in Graph is colored
    isColored = all(v != 0 for v in graph.C)
    if isColored:
        return True

# Main
def main():
    start = time.time()
    G = [[0, 1, 1,0,0],
    [1, 0, 0,1,1], 
    [1, 0, 0,1,1], 
    [0, 1, 1,0,1], 
    [0, 1, 1,1,0]]

    graph = initialize_graph(G)
    #print(graph.P)
    #print(graph.N)

    end = time.time()
    print('Algorithm time: ', (end - start))


if __name__ == "__main__":
    main()
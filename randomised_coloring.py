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
        self.generate_nodes(G)

    def generate_nodes(self,G):
        jobs = []

        q = multiprocessing.Queue()
        for i in range(len(G)):
            p = multiprocessing.Process(target=self.calc_degree, args=(G, i,q))
            jobs.append(p)
            p.start()

        for proc in jobs:
            proc.join()

        graph_nodes = [0]*len(G)
        while q.empty() is False:
            node =  q.get()
            node_id = node.id[0]
            graph_nodes[node_id] = node

        self.V = graph_nodes

    def calc_degree(self,graph,id,q):
        degree = sum(graph[id])
        node = Node(id,degree)
        q.put(node)

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
    print(graph.V[0].available_colors)
    #print(graph.P)
    #print(graph.N)

    end = time.time()
    print('Algorithm time: ', (end - start))


if __name__ == "__main__":
    main()
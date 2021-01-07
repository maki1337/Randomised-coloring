import multiprocessing
from multiprocessing import Pipe, Lock
import multiprocessing.pool
import time
import random
import numpy as np

class NoDaemonProcess(multiprocessing.Process):
    @property
    def daemon(self):
        return False

    @daemon.setter
    def daemon(self, value):
        pass
class NoDaemonContext(type(multiprocessing.get_context())):
    Process = NoDaemonProcess
class NestablePool(multiprocessing.pool.Pool):
    def __init__(self, *args, **kwargs):
        kwargs['context'] = NoDaemonContext()
        super(NestablePool, self).__init__(*args, **kwargs)

class Node:
    def __init__(self, id, degree):
        self.id = id,
        self.isColored = False
        self.degree = degree
        self.neighbours = {}
        self.chosen_color = 0
        self.available_colors = list(range(1, degree +1))
        self.isProcessed = False

    def connect(self, node):
        send_conn, recv_conn = multiprocessing.Pipe()
        if node.set_neighbour(self.id[0], recv_conn):
            self.set_neighbour(node.id[0], send_conn)

    def set_neighbour(self, neighbour_id, pipe):
        if not neighbour_id in self.neighbours:
            self.neighbours[neighbour_id] = pipe
            return True
        return False

    def inform_neighbours(self, msg):
        packages = [(key, msg) for key in self.neighbours.keys()]
        multiprocessing.Pool().map(self.inform_neighbour, packages)

    def inform_neighbour(self, payload):
        return self.neighbours.get(payload[0]).send((self.id[0], payload[1]))

    def check_for_messages(self):
        return multiprocessing.Pool().map(self.check_neighbour_message, self.neighbours.keys())

    def check_neighbour_message(self, neighbour_id):
        return self.neighbours.get(neighbour_id).recv()

    def delete_neighbours(self, neighbours_to_del):
        for neighbour in neighbours_to_del: 
            self.delete_neighbour(neighbour)

    def delete_neighbour(self, neighbour_to_del):
        if neighbour_to_del[1][1] == True:
            self.unset_neighbour(neighbour_to_del[0])

    def unset_neighbour(self, neighbour_id):
        self.neighbours.pop(neighbour_id)

    def delete_colors(self, color_to_del):
        for color in color_to_del: 
            self.delete_color(color)

    def delete_color(self, color):
        if color[1][1] == True:
            self.unset_color(color[1][0])

    def unset_color(self, color):
        if color in self.available_colors:
            self.available_colors.remove(color)

class Graph:
    def __init__ (self, G):
        self.G = G
        self.generate_nodes(G)
        self.create_communication_network()

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

        self.N = graph_nodes

    def calc_degree(self,graph,id,q):
        degree = sum(graph[id])
        node = Node(id,degree)
        q.put(node)
    
    def create_communication_network(self):
        for i in range(len(self.G[0])):
            self.connect_node(self.N[i])

    def connect_node(self, node):
        # dobimo sosede
        neighbour_indexes = np.nonzero(self.G[node.id[0]])[0]
        # pošlemo index sosedov da jih doda v neighbour array in doda pipe communication
        for i in range(len(neighbour_indexes)):
             node.connect(self.N[neighbour_indexes[i]])


def initialize_graph(G):
    graph = Graph(G)
    return graph

def choose_color(node):
    if(node.isProcessed):
        return node

    # Izbere naključno barvo iz array in jo zapiše
    random_color = random.choice(node.available_colors)
    node.chosen_color = random_color

    # pošlji sporočilom sosedom katero barvo si izbral
    node.inform_neighbours(random_color)

    return node

def recv_color(node):
    if(node.isProcessed):
        return node

    # Dobimo barve od sosedov
    neighbour_colors = [msg for msg in node.check_for_messages() if msg[1] <= node.chosen_color]
    canColor = False

    #print(str(neighbour_colors)+" | "+ str(node.id[0]))
    if len(neighbour_colors)==0:
        canColor = True
    #print("Can color: "+str(canColor)+" | "+str(node.id[0]))
    if canColor:
        node.isColored = True
    else:
        node.chosen_color = 0

    return node

def check_if_colored(node):
    if(node.isProcessed):
        return node

    print("node id: "+ str(node.id[0])+" can color?:"+str(node.isColored))
    # Če sem obarvan obvesti sosede
    if node.isColored:
        node.inform_neighbours((node.chosen_color, True))
        node.isProcessed = True
    else:
        node.inform_neighbours((node.chosen_color, False))

    return node

def delete_connection(node):
    if(node.isProcessed):
        return node

    # Pridobim informacije, če je sosed že obarvam
    nodes = [msg for msg in node.check_for_messages() if msg[1][1] == True]

    if len(nodes) != 0: 
        node.delete_colors(nodes)
        node.delete_neighbours(nodes)

    return node

def can_color(neighbour_colors, chosen_color):
    for color in neighbour_colors:
        if color[1] <= chosen_color:
            return True
    return True

def randomised_coloring(graph):
    pool = NestablePool()
    print("Choosing color")
    graph.N = pool.map(choose_color, graph.N)
    print("Colors have been chosen")
    print("Receiving color update")
    graph.N = pool.map(recv_color, graph.N)
    print("check if colored")
    graph.N = pool.map(check_if_colored, graph.N)
    print("Deleting connections")
    graph.N = pool.map(delete_connection, graph.N)
    if all(n.isProcessed == True for n in graph.N):
        print("yes")
        return graph.N
    else:
        print("not")
        return randomised_coloring(graph)



# Main
def main():
    start = time.time()
    G = [[0, 1, 1,0,0],
    [1, 0, 0,1,1], 
    [1, 0, 0,1,1], 
    [0, 1, 1,0,1], 
    [0, 1, 1,1,0]]

    graph = initialize_graph(G)
    print("Randomised algorith starting ...")
    colored_graph = randomised_coloring(graph)

    for x in colored_graph:
        print(x.id)
        print(x.chosen_color)

    end = time.time()
    print('Algorithm time: ', (end - start))


if __name__ == "__main__":
    main()
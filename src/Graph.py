import time

class Graph:
    def __init__(self):
        self.nodes = []
        self.node_keys = {}
        self.edges = {}
        self.edge_keys = {}
        self.starts = []

        self.count = 0
        self.start_time = 0
        self.search_limit = 30
        self.print_exceeded = True

        self.path = None
        self.max_paths = 20
        self.path_found = False

        self.path_check = None
        self.min_weight = 10000

    def print_graph_data(self):
        print(str(len(self.nodes))+" nodes")
        #print([node.name for node in self.nodes])
        edges = [str(start.name)+' --'+str(int(self.edges[start][end]))+'--> '+str(end.name) for start in self.edges for end in self.edges[start]]
        print(str(len(edges))+" edges")
        #print(edges)

    def add_node(self, node):
        if node in self.nodes:
            raise ValueError("Duplicate node")
        else:
            self.nodes.append(node)
            self.node_keys[node.data] = node
            self.edges[node] = {}
            self.edge_keys[node] = []
            self.min_weight = 1000000

    def add_edge(self, edge):
        if edge.start not in self.nodes:
            raise ValueError("Node not in graph " + str(edge.start.data))
        if edge.end not in self.nodes:
            raise ValueError("Node not in graph " + str(edge.end.data))
        elif edge.end not in self.edges[edge.start]:
            self.edges[edge.start][edge.end] = edge.weight
            self.min_weight = max(1000000, edge.weight*100)
            self.edge_keys[edge.start].append(edge.end)
            self.edge_keys[edge.start] = sorted(self.edge_keys[edge.start], key=lambda x:self.edges[edge.start][x], reverse=True)

    def remove_edge(self, start, end):
        if start not in self.nodes:
            raise ValueError("Node not in graph " + str(start.data))
        if end not in self.nodes:
            raise ValueError("Node not in graph " + str(end.data))
        if self.edge_keys.get(start)==None or end not in self.edge_keys[start]:
            raise ValueError("Edge not in graph " + str(start.data) + ", " + str(end.data))
        else:
            self.edges[start].pop(end)
            self.edge_keys[start].remove(end)

    def get_node(self, data):
        return self.node_keys.get(data)
        #for node in self.nodes:
        #    if node.data == data: return node

    def get_shortest_hamiltonian_path(self):
        paths = self.get_all_hamiltonian_paths(True)
        if len(paths) == 0:
            print("Unable to find a Hamiltonian path in graph")
            return [[]]
        paths = sorted(paths, key=lambda path: path[1])
        print(str(len(paths))+" hamiltonian paths found.")
        weights = ""
        for path in paths:
            weights = weights + str(round(path[1], 2))
            if path!=paths[-1]: weights = weights + ", "
        print("Hamiltonian path weights: "+weights)
        return paths[0]

    def get_all_hamiltonian_paths(self, shortest=False):
        self.start_time = time.time()
        self.print_exceeded = True
        paths = []
        for start, weight in self.starts:
            if start not in self.nodes:
                raise ValueError("Node not in graph")
            else:
                self.get_hamiltonian_paths(([start], weight), paths, shortest)
        
        return paths

    def get_hamiltonian_paths(self, path, paths, shortest=False):
        if (time.time() - self.start_time > self.search_limit) or len(paths)>=self.max_paths:
            if self.print_exceeded:
                print("Exceeded search limit")
                self.print_exceeded = False
            return paths

        if all([node in path[0] for node in self.nodes]):
            if path[1] < self.min_weight: self.min_weight = path[1]
            paths.append(path)
            return paths

        for end in self.edge_keys[path[0][-1]]:
            weight = path[1]+self.edges[path[0][-1]][end]
            # check that we have not already been to this graph node
            if end not in path[0]:
                # if we are looking for the shortest hamiltonian path, check
                # that the weight of the current search is not greater than
                # the least weighted path we have already traversed
                if not shortest or (weight) <= self.min_weight:
                    # if Graph has a function for checking the path to make sure
                    # the traversal is "legal"
                    if not self.path_check or (self.path_check and self.path_check(end, path[0], self)):
                        self.get_hamiltonian_paths((path[0]+[end], weight), paths, shortest)

        return paths

    def check_for_path(self, start, end):
        self.path = None
        self.path_found = False
        self.start_time = time.time()
        self.get_path_to(end, [start])
        return self.path_found, self.path

    def get_path_to(self, end, path):
        if not self.path_found:
            if time.time()-self.start_time > self.search_limit:
                raise ValueError("Exceeded search limit")

            for node in self.edges[path[-1]].keys():
                if node not in path:
                    if node == end:
                        self.path_found = True
                        self.path = path + [node]
                        return
                    else:
                        self.get_path_to(end, path + [node])


class Graph_Node:
    def __init__(self, data):
        self.data = data
        self.name = None
        self.start = False


class Graph_Edge:
    def __init__(self, start, end, weight):
        self.end = end
        self.start = start
        self.weight = weight

class NFA:
    def __init__(self):
        self.nodes = []
        self.start = None
        self.final = []
        self.alphabet = []
        self.transitions = {}

    def add_node(self, node):
        if node in self.nodes:
            raise ValueError("Duplicate node")
        else:
            self.nodes.append(node)
            self.transitions[node]={}

    def set_start(self, node):
        if node in self.nodes:
            self.start = node
        else:
            raise ValueError("Node not in nodes")

    def set_final(self, node):
        if node in self.nodes:
            self.final.append(node)
        else:
            raise ValueError("Node not in nodes")

    def add_to_alphabet(self, elem):
        if elem in self.alphabet:
            raise ValueError("Duplicate element")
        else:
            self.alphabet.append(elem)

    def add_transition(self, start, elem, end):
        if elem not in self.alphabet or start not in self.nodes or end not in self.nodes:
            raise ValueError("Cannot add transition")
        else:
            self.transitions[start][elem] = end

    def transition(self, start, elem):
        if elem not in self.alphabet or start not in self.nodes:
            raise ValueError("Illegal transition")
        else:
            return self.transitions[start][elem]
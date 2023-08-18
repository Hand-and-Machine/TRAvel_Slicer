class Graph:
    def __init__(self):
        self.nodes = []
        self.edges = {}
        self.starts = []

        self.count = 0
        self.count_limit = 1000

        self.path = None
        self.path_found = False

        self.path_check = None
        self.min_weight = 1000000

    def print_graph_data(self):
        print([node.name for node in self.nodes])
        print([str(start.name)+' --'+str(int(self.edges[start][end]))+'--> '+str(end.name) for start in self.edges for end in self.edges[start]])

    def add_node(self, node):
        if node in self.nodes:
            raise ValueError("Duplicate node")
        else:
            self.nodes.append(node)
            self.edges[node] = {}
            self.min_weight = 1000000

    def add_edge(self, edge):
        if edge.start not in self.nodes:
            raise ValueError("Node not in graph " + str(edge.start.data))
        if edge.end not in self.nodes:
            raise ValueError("Node not in graph " + str(edge.end.data))
        elif edge.end not in self.edges[edge.start]:
            self.edges[edge.start][edge.end] = edge.weight
            self.min_weight = max(1000000, edge.weight*100)

    def get_node(self, data):
        for node in self.nodes:
            if node.data == data: return node

    def get_shortest_hamiltonian_path(self):
        self.count = 0
        paths = self.get_all_hamiltonian_paths(True)
        if len(paths) == 0:
            raise ValueError("Unable to find a hamiltonian path in graph")
        return sorted(paths, key=lambda path: path[1])[0]

    def get_all_hamiltonian_paths(self, shortest=False):
        paths = []
        for start in self.starts:
            if start not in self.nodes:
                raise ValueError("Node not in graph")
            else:
                self.get_hamiltonian_paths(([start], 0), paths, shortest)
        
        return paths

    def get_hamiltonian_paths(self, path, paths, shortest=False):
        self.count = self.count + 1
        if self.count > self.count_limit:
            raise ValueError("Exceeded search limit")

        if all([node in path[0] for node in self.nodes]):
            if path[1] < self.min_weight: self.min_weight = path[1]
            paths.append(path)
            return paths

        for end in self.edges[path[0][-1]].keys():
            weight = path[1]+self.edges[path[0][-1]][end]
            # check that we have not already been to this graph node
            if end not in path[0]:
                # if we are looking for the shortest hamiltonian path, check
                # that the weight of the current search is not greater than
                # the least weighted path we have already traversed
                if not shortest or (weight) <= self.min_weight:
                    # if Graph has a function for checking the path to make sure
                    # the traversal is "legal"
                    if not self.path_check or (self.path_check and self.path_check(end, path[0])):
                        self.get_hamiltonian_paths((path[0]+[end], weight), paths)

        return paths

    def check_for_path(self, start, end):
        self.count = 0
        self.path = None
        self.path_found = False
        self.get_path_to(end, [start])
        return self.path_found, self.path

    def get_path_to(self, end, path):
        if not self.path_found:
            self.count = self.count + 1
            if self.count > self.count_limit:
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


class Graph_Edge:
    def __init__(self, start, end, weight):
        self.end = end
        self.start = start
        self.weight = weight
class Graph:
    def __init__(self):
        self.nodes = []
        self.edges = {}
        self.starts = []

        self.count = 0
        self.count_limit = 100

    def add_node(self, node):
        if node in self.nodes:
            raise ValueError("Duplicate node")
        else:
            self.nodes.append(node)
            self.edges[node] = {}

    def add_edge(self, edge):
        if edge.start not in self.nodes:
            raise ValueError("Node not in graph " + str(edge.start.data))
        if edge.end not in self.nodes:
            raise ValueError("Node not in graph " + str(edge.end.data))
        elif edge.end not in self.edges[edge.start]:
            self.edges[edge.start][edge.end] = edge.weight

    def get_node(self, data):
        for node in self.nodes:
            if node.data == data: return node

    def get_shortest_hamiltonian_path(self):
        self.count = 0
        paths = self.get_all_hamiltonian_paths()
        return sorted(paths, key=lambda path: path[1])[0]

    def get_all_hamiltonian_paths(self):
        paths = []
        for start in self.starts:
            if start not in self.nodes:
                raise ValueError("Node not in graph")
            else:
                self.get_hamiltonian_paths(([start], 0), paths)
        
        return paths

    def get_hamiltonian_paths(self, path, paths):
        self.count = self.count + 1
        if self.count > self.count_limit:
            raise ValueError("Exceeded search limit")

        if all([node in path[0] for node in self.nodes]):
            paths.append(path)
            return paths

        for end in self.edges[path[0][-1]].keys():
            if end not in path[0]:
                self.get_hamiltonian_paths((path[0]+[end], path[1]+self.edges[path[0][-1]][end]), paths)

        return paths


class Graph_Node:
    def __init__(self, data):
        self.data = data


class Graph_Edge:
    def __init__(self, start, end, weight):
        self.end = end
        self.start = start
        self.weight = weight
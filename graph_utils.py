class Graph:
    def __init__(self):
        self.nodes = []
        self.edges = {}
        self.starts = []

    def add_node(self, node):
        if node in self.nodes:
            raise ValueError("Duplicate node")
        else:
            self.nodes.append(node)
            self.edges[node] = []

    def add_edge(self, edge):
        if edge.start not in self.nodes:
            raise ValueError("Node not in graph " + str(edge.start.data))
        if edge.end not in self.nodes:
            raise ValueError("Node not in graph " + str(edge.end.data))
        elif edge.end not in self.edges[edge.start]:
            self.edges[edge.start].append(edge.end)

    def get_node(self, data):
        for node in self.nodes:
            if node.data == data: return node

    def get_shortest_hamiltonian_path(self):
        paths = self.get_all_hamiltonian_paths()
        return paths.sorted(key=lambda path: path[1])[-1]

    def get_all_hamiltonian_paths(self):
        paths = []
        for start in self.starts:
            if start not in self.nodes:
                raise ValueError("Node not in graph")
            else:
                self.get_hamiltonian_paths(([start], 0), paths)
        
        return paths

    def get_hamiltonian_paths(self, path, paths):
        print(path)
        if all([node in path[0] for node in self.nodes]):
            paths.append(path)
            return paths
        
        for edge in self.edges[path[0][-1]]:
            if edge.end not in path[0]:
                self.get_hamiltonian_paths((path[0]+[edge.end], path[1]+edge.weight), paths)

        return paths


class Graph_Node:
    def __init__(self, data):
        self.data = data


class Graph_Edge:
    def __init__(self, start, end):
        self.end = end
        self.start = start
        self.weight = 0
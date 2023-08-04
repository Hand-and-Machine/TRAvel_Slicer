class Node:
    def __init__(self, data):
        self.data = data
        self.type = None
        self.depth = None
        self.height = None
        self.parents = []
        self.children = []
        self.sub_nodes = []
        self.needs_support = None

    def dfs(self, list=[]):
        list.append(self.data)
        for child in self.children:
            child.dfs(list)
        return list

    def bfs(self):
        queue = [self]

        list = []
        while len(queue) != 0:
            node = queue.pop(0)
            list.append(node)
            for child in node.children:
                queue.append(child)
        
        return list

    def nodes_at_depth(self, depth=0, list=[]):
        print(depth)
        nodes = self.get_all_nodes()
        print(nodes)

    def get_all_nodes(self, list=[]):
        if self not in list: list.append(self)
        for child in self.children:
            child.get_all_nodes(list)
        for parent in self.parents:
            if parent not in list:
                parent.get_all_nodes_above(list)
        return list

    def get_all_nodes_below(self, list=[]):
        if self not in list: list.append(self)
        for child in self.children:
            child.get_all_nodes_below(list)
        return list

    def get_all_nodes_above(self, list=[]):
        if self not in list: list.append(self)
        for parent in self.parents:
            parent.get_all_nodes_above(list)
        return list

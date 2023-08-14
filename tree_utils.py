class Node:
    def __init__(self, data):
        self.data = data
        self.type = None
        self.depth = None
        self.height = None
        self.overlap = []
        self.parents = []
        self.children = []
        self.sub_nodes = []
        self.start_point = None
        self.needs_support = None

    def dfs(self, list=[]):
        if self not in list: list.append(self)
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

    def get_all_nodes(self, list):
        if self not in list: list.append(self)
        for child in self.children:
            child.get_all_nodes(list)
        for parent in self.parents:
            if parent not in list:
                parent.get_all_ancestors(list)
        return list

    def get_all_descendants(self, list):
        if self not in list: list.append(self)
        for child in self.children:
            child.get_all_descendants(list)
        return list

    def get_all_ancestors(self, list):
        if self not in list: list.append(self)
        for parent in self.parents:
            parent.get_all_ancestors(list)
        return list

class Node:
    def __init__(self, data):
        self.data = data
        self.name = None
        self.type = None
        self.depth = None
        self.height = None
        self.parent = None
        self.overlap = []
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
        if self.parent is not None and self.parent not in list:
            self.parent.get_all_ancestors(list)
        return list

    def get_all_descendants(self, list):
        if self not in list: list.append(self)
        for child in self.children:
            child.get_all_descendants(list)
        return list

    def get_all_ancestors(self, list):
        if self not in list: list.append(self)
        if self.parent is not None:
            self.parent.get_all_ancestors(list)
        return list

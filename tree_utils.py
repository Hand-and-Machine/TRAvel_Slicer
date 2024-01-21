class Node:
    def __init__(self, data):
        self.data = data
        self.box = None
        self.name = None
        self.type = 1
        self.depth = 0
        self.height = 0
        self.parent = None
        self.is_wall = False
        self.overlap = []
        self.reverse = False
        self.children = []
        self.sub_nodes = []
        self.connection = {}
        self.start_point = None
        self.needs_support = None
        self.fermat_spiral = None

    def add_child(self, data):
        new_node = Node(data)
        self.children.append(new_node)
        if len(self.children) > 1: self.type = 2
        else: self.type = 1
        new_node.parent = self
        new_node.depth = self.depth + 1
        return new_node

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

    def get_node(self, data):
        if self.data == data: return self
        else:
            for node in self.get_all_nodes([]):
                if node.data == data:
                    return node

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

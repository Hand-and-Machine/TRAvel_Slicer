import math
import Rhino
import rhinoscriptsyntax as rs
import extruder_turtle  
import turtle_utilities as tu
from extruder_turtle import *

import tree_utils
from tree_utils import *

import geometry_utils
from geometry_utils import *

def build_vertical_tree(t, shape):
    layers = int(math.floor(get_shape_height(shape) / t.get_layer_height())) + 1
    print("Number of Layers", layers)
    root = Node('root')

    previous_nodes = [root]
    for l in range(layers):
        z = l*t.get_layer_height()
        plane = get_plane(z)
        curves = rs.AddSrfContourCrvs(shape, plane)
        new_nodes = []
        for curve in curves:
            node = Node(curve)
            node.depth = l
            node.height = l
            new_nodes.append(node)
            if root in previous_nodes:
                node.parents.append(root)
                root.children.append(node)
            else:
                for prev_n in previous_nodes:
                    if xy_bbox_overlap(prev_n.data, curve):
                        node.parents.append(prev_n)
                        prev_n.children.append(node)
            if len(node.parents) == 0: node.needs_support = True
            else: node.needs_support = False

        previous_nodes = new_nodes

    return segment_tree_by_height(t, root)

def segment_tree_by_height(t, tree):
    #nozzle_height = t.get_nozzle_height()
    #nozzle_width = t.get_nozzle_max_width()
    nozzle_height = 30 #height of nozzle in mm
    limit = int(math.floor(nozzle_height / t.get_layer_height()))
    super_root = Node('root')
    super_root.depth = 0
    super_root.height = 0
    idx = 0
    for child in tree.children:
        group_by_height(child, super_root, limit, idx)
        idx = idx + 1
    return super_root


def group_by_height(node, super_node, height, idx=0):
    s_node = super_node
    if node.depth // height == super_node.height:
        super_node.sub_nodes.append(node)
    elif node.depth // height > super_node.height:
        new_super = Node(str(super_node.data)+'_'+str(idx))
        new_super.parent = super_node
        new_super.depth = super_node.depth + 1
        new_super.height = node.depth // height
        new_super.sub_nodes.append(node)

        super_node.children.append(new_super)

        s_node = new_super
    elif node.depth // height < super_node.height:
        print("Error, node should not be below current super_node")

    idx = 0
    if len(node.children) > 1:
        for child in node.children:
            new_new_super = Node(str(s_node.data)+'_'+str(idx))
            new_new_super.parent = s_node
            new_new_super.depth = s_node.depth + 1
            new_new_super.height = node.depth // height
            s_node.children.append(new_new_super)
            group_by_height(child, new_new_super, height, 0)
            idx = idx + 1
    else:
        for child in node.children:
            group_by_height(child, s_node, height, idx)
            idx = idx + 1


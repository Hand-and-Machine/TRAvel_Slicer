import math
import time
import Rhino
import rhinoscriptsyntax as rs
import extruder_turtle  
import turtle_utilities as tu
from extruder_turtle import *

import Node
from Node import *

import Graph
from Graph import *

import contour_utils
from contour_utils import *

import geometry_utils
from geometry_utils import *


# Outer-Travel Reduction code
def outer_travel_reduction(t, shape, curves, initial_offset=0.5, debug=False):
    vert_start_time = time.time()

    global nozzle_width
    nozzle_width = float(t.get_nozzle_width())
    global nozzle_height
    nozzle_height = float(t.get_nozzle_height())

    global xy_plane_crvs
    xy_plane_crvs = {}
    global path_overlap
    path_overlap = {}
    global overlap_above
    overlap_above = {}
    global overlap
    overlap = {}

    total_height = len(curves)*t.get_layer_height()
    init_tree = build_height_dependence_tree(t, shape, curves, initial_offset=initial_offset, debug=debug)
    vert_tree = segment_tree_by_height(t, init_tree, total_height, debug=debug)
    if len(vert_tree.get_all_nodes([])) > len(init_tree.get_all_nodes([])):
        raise ValueError("There should not be more super nodes than nodes")

    all_nodes = vert_tree.get_all_nodes([])

    # compute bounding boxes and box containing sub_nodes for each node
    # this is used to avoid collisions between previously printed nodes during travel
    for node in all_nodes:
        try:
            flat = [data for sub in node.sub_nodes for data in sub.data]
            flat = [f for ff in flat for f in ff]
            bb = rs.BoundingBox(flat)
            try:
                box = rs.AddBox(bb)
                node.box = box
            except:
                # this may be a single layer
                try:
                    z = None
                    minX = None
                    maxX = None
                    minY = None
                    maxY = None
                    for pnt in bb:
                        if minX == None or pnt.X < minX:
                            minX = pnt.X
                        if maxX == None or pnt.X > maxX:
                            maxX = pnt.X
                        if minY == None or pnt.Y < minY:
                            minY = pnt.Y
                        if maxY == None or pnt.Y > maxY:
                            maxY = pnt.Y
                        if z == None:
                            z = pnt.Z
                    bb = [
                        rs.CreatePoint(minX, minY, z),
                        rs.CreatePoint(maxX, minY, z),
                        rs.CreatePoint(maxX, maxY, z),
                        rs.CreatePoint(minX, maxY, z),
                        rs.CreatePoint(minX, minY, z+t.get_layer_height()),
                        rs.CreatePoint(maxX, minY, z+t.get_layer_height()),
                        rs.CreatePoint(maxX, maxY, z+t.get_layer_height()),
                        rs.CreatePoint(minX, maxY, z+t.get_layer_height())]
                    box = rs.AddBox(bb)
                    node.box = box
                except:
                    print("Unable to create box from bounding box: ", bb)
        except:
            print("Could not get bounding box: ", [rs.ObjectType(f) for f in flat])

    if debug: print("Size of grouped height tree: "+str(len(all_nodes)))

    st_time = time.time()

    # edges are used for debugging and visualization rather than further calculations
    edges = []

    node_path = []
    for h in range(int(math.floor(total_height / nozzle_height))+1):
        nodes_at_height = [node for node in all_nodes if node.height == h]
        if debug: print("Nodes at nozzle height "+str(h)+": "+str(len(nodes_at_height)))
        if len(nodes_at_height) == 0: break

        # create a graph for this height chunk
        height_graph = Graph()
        min_height = 1000000000
        # add nodes to graph for every super node within the height chunk
        for node in nodes_at_height:
            graph_node = Graph_Node(node)
            graph_node.name = node.name
            height_graph.add_node(graph_node)
            # add start nodes to the graph
            if node.min_sub_height <= min_height:
                min_height = node.min_sub_height

        prev_node = None
        if len(node_path)>0: prev_node = node_path[-1]
        for node in height_graph.nodes:
            if node.data.min_sub_height == min_height:
                weight = 0
                if prev_node!=None:
                    flat = [f for ff in node.data.sub_nodes[0].data for f in ff]
                    if len(flat) > 1:
                        closest_pnt_end = rs.PointClosestObject(prev_node.data.sub_nodes[-1].center_point, [f for ff in node.data.sub_nodes[0].data for f in ff])[1]
                    elif len(flat) == 1:
                        closest_pnt_end = rs.EvaluateCurve(flat[0], rs.CurveClosestPoint(flat[0], prev_node.data.sub_nodes[-1].center_point))
                    else:
                        print("That shouldn't be possible")
                    weight = rs.Distance(prev_node.data.sub_nodes[-1].center_point, closest_pnt_end)

                height_graph.starts.append((node, weight))
                node.start = True

        # add edges to graph
        for graph_node in height_graph.nodes:
            # edges related to height dependency
            node1 = graph_node.data
            for child in node1.children:
                if child in nodes_at_height and child!=graph_node.data:
                    height_graph.add_edge(Graph_Edge(graph_node, height_graph.get_node(child), 0))

                    closest_pnt_end = rs.PointClosestObject(node1.sub_nodes[-1].center_point, [f for ff in child.sub_nodes[0].data for f in ff])[1]
                    edges.append(rs.AddCurve([node1.sub_nodes[-1].center_point, closest_pnt_end]))

            # edges related to travel between nodes
            direct_relations = node1.get_all_descendants([]) + node1.get_all_ancestors([])
            siblings_and_counsins = [n for n in nodes_at_height if n not in direct_relations]
            for node2 in siblings_and_counsins:
                if node2!=node1:
                    if (node1.min_sub_height == node2.sub_nodes[-1].height+1):
                        print("node 2 is directly below node 1")
                        if is_sub_node_overlapping_above(node1.sub_nodes[0], node2.sub_nodes[-1], 0):
                            print("technically a parent")
                            height_graph.add_edge(Graph_Edge(graph_node, height_graph.get_node(node2), 0))
                    elif check_path(height_graph.get_node(node2), [graph_node]):
                        # only add edge if node1 does not overlap node2
                        # compute travel between curves, where weight is set as distance between center of start and end curves within node
                        closest_pnt_end = rs.PointClosestObject(node1.sub_nodes[-1].center_point, [f for ff in node2.sub_nodes[0].data for f in ff])[1]
                        weight = rs.Distance(node1.sub_nodes[-1].center_point, closest_pnt_end)
                        height_graph.add_edge(Graph_Edge(graph_node, height_graph.get_node(node2), weight))
                        if weight!=0:
                            edge = rs.AddCurve([node1.sub_nodes[-1].center_point, closest_pnt_end])
                            edges.append(edge)

        height_graph.print_graph_data()
        height_graph.path_check = check_path

        start_time = time.time()
        path_section = height_graph.get_shortest_hamiltonian_path()[0]
        node_path = node_path + path_section
        if debug:
            print("Hamiltonian Path Search Time: "+str(round(time.time() - start_time, 3))+" seconds")
            print('')

    path = []
    for node in node_path:
        for sub in node.data.sub_nodes:
            path.append(sub.data)

    if debug:
        print("Graph construction and all hamiltonian paths search time: "+str(round(time.time() - st_time, 3))+" seconds")
        print("Total Outer Travel Reduction time: "+str(round(time.time() - vert_start_time, 3))+" seconds")

    return vert_tree, node_path, path, edges


def build_height_dependence_tree(t, shape, all_curves, initial_offset=0.5, debug=False):
    start_time = time.time()

    print('Number of layers: '+str(len(all_curves)))

    root = Node('root')
    root.name = 'root'

    time1 = 0

    #extrude_width = float(t.get_extrude_width())
    #initial_offset = extrude_width*initial_offset
    #gap = extrude_width*0.2

    center_point = rs.CreatePoint(0, 0, 0)
    previous_nodes = [root]
    for l in range(len(all_curves)):
        st_1 = time.time()
        initial_curves = all_curves[l]
        curve_groups = get_curves(shape, rs.CurveStartPoint(initial_curves[0]).Z, initial_curves=initial_curves)
        #curve_groups = connect_curve_groups(curve_groups, gap, initial_offset=initial_offset)
        #curve_groups = [[crv] for crv in curve_groups]
        time1 = time1 + time.time()-st_1

        outer_curves = []
        for crvs in curve_groups:
            # curve grouping function returns outermost
            # contour as first curve in curve_group list
            outer_curve = crvs[0]
            center_point = rs.PointAdd(center_point, get_area_center(outer_curve))
            outer_curves.append(outer_curve)
        center_point = rs.CreatePoint(center_point.X/(len(curve_groups) + 1), center_point.Y/(len(curve_groups) + 1), l*t.get_layer_height())

        # combine separated curves if they are within nozzle_width/2 of one another
        idx_groups = {c:[c] for c in range(len(outer_curves))}
        for c1 in range(len(outer_curves)):
            for c2 in range(c1+1, len(outer_curves)):
                if dynamic_curve_overlap_check(curve_groups[c1], curve_groups[c2], nozzle_width):
                    idx_groups[c1].append(c2)

        # iterate through idx_groups
        connection_found = True
        while connection_found:
            connection_found = False
            for c1 in idx_groups.keys():
                if idx_groups.get(c1):
                    for c2 in idx_groups[c1]:
                        if idx_groups.get(c2):
                            if c2 != c1:
                                connection_found = True
                                for crv3 in idx_groups[c2]:
                                    if crv3 not in idx_groups[c1]:
                                        idx_groups[c1].append(crv3)
                                idx_groups.pop(c2)

        groups = []
        outer_curve_groups = []
        for i in idx_groups:
            idxs = idx_groups[i]
            groups.append([curve_groups[c] for c in idxs])
            outer_curve_groups.append([outer_curves[c] for c in idxs])

        new_nodes = []
        for c in range(len(groups)):
            node = Node(groups[c])
            node.name = str(l) + str(c)
            node.depth = l
            node.height = l
            node.center_point = center_point
            new_nodes.append(node)
            if root in previous_nodes:
                node.parent = root
                root.children.append(node)
            else:
                for prev_n in previous_nodes:
                    if not node.parent and is_sub_node_overlapping_above(node, prev_n, 0):
                        node.parent = prev_n
                        prev_n.children.append(node)

            if node.parent == None:
                print('No parent found')
                node.needs_support = True
            else: node.needs_support = False

        previous_nodes = new_nodes

    if debug:
        print('')
        print("Time to get curves: "+str(round(time1, 3))+" seconds")
        print('')
        print("Initial treeing time: "+str(round(time.time() - start_time, 3))+" seconds")
        print("Initial height tree size: "+str(len(root.get_all_nodes([]))))

    return root

def segment_tree_by_height(t, tree, total_height, offset=0.0, debug=False):
    limit = nozzle_height / t.get_layer_height()
    super_root = Node('root')
    super_root.name = 'root'
    super_root.depth = 0
    super_root.height = 0
    idx = 0

    start_time = time.time()
    for child in tree.children:
        group_by_height(child, super_root, limit, offset, idx=idx)
        idx = idx + 1
    if debug: print("Grouping tree by nozzle height: "+str(round(time.time() - start_time, 3))+" seconds")

    s_t = time.time()
    divide_by_overlap(super_root, total_height, offset)
    if debug: print("Dividing super tree by overlap: "+str(round(time.time() - s_t, 3))+" seconds")
    return super_root


def group_by_height(node, super_node, height, offset, idx=0):
    s_node = super_node
    if node.height // height == super_node.height:
        super_node.add_sub_node(node)
    elif node.height // height > super_node.height:
        if len(super_node.sub_nodes) > 0:
            new_super = super_node.add_child(str(super_node.data)+'_'+str(idx))
            new_super.name = new_super.data
            new_super.height = node.height // height
            new_super.add_sub_node(node)

            s_node = new_super
        else:
            super_node.add_sub_node(node)
            super_node.height = node.height // height
    elif node.height // height < super_node.height:
        raise ValueError("Error, node should not be below current super_node")

    idx = 0
    if len(node.children) > 1:
        for child in node.children:
            new_new_super = s_node.add_child(str(s_node.data)+'_'+str(idx))
            new_new_super.name = new_new_super.data
            new_new_super.height = node.height // height
            group_by_height(child, new_new_super, height, 0)
            idx = idx + 1
    elif len(node.children) == 1:
        group_by_height(node.children[0], s_node, height, idx)
        idx = idx + 1


def divide_by_overlap(super_root, total_height, offset):
    height = int(math.floor(total_height / nozzle_height)) + 1
    nodes = super_root.get_all_nodes([])
    for h in range(height):
        nodes_at_height = [node for node in nodes if node.height == h]
        subdivide_by_overlap(nodes_at_height)


def subdivide_by_overlap(nodes):
    # create a DFA (all DFAs are NFAs) with:
    # States = Above (A), Below (B), Neither (N),
    # Both (X), starting state (S) (also split state)
    # Final States = { S }
    # Alphabet = { a, b, n, x } (above, below, neither, both)
    # Transitions:
    # d(S, a) = A, d(S, b) = B, d(S, n) = N, d(S, x) = X
    # d(A, a) = A, d(A, n) = A, d(A, b) = S, d(A, x) = S
    # d(B, b) = B, d(B, n) = B, d(B, a) = S, d(B, x) = S
    # d(X, x) = X, d(X, n) = X, d(X, a) = S, d(X, b) = S
    # d(N, n) = N, d(N, a) = A, d(N, b) = B, d(N, x) = S
    dfa = NFA()
    S = Graph_Node('S')
    A = Graph_Node('A')
    B = Graph_Node('B')
    N = Graph_Node('N')
    X = Graph_Node('X')

    dfa.add_node(S)
    dfa.add_node(A)
    dfa.add_node(B)
    dfa.add_node(N)
    dfa.add_node(X)

    dfa.set_start(S)
    dfa.set_final(S)

    dfa.add_to_alphabet('a')
    dfa.add_to_alphabet('b')
    dfa.add_to_alphabet('n')
    dfa.add_to_alphabet('x')

    dfa.transitions[S] = {'a': A, 'b': B, 'n': N, 'x': X}
    dfa.transitions[A] = {'a': A, 'b': S, 'n': A, 'x': S}
    dfa.transitions[B] = {'a': S, 'b': B, 'n': B, 'x': S}
    dfa.transitions[N] = {'a': A, 'b': B, 'n': N, 'x': S}
    dfa.transitions[X] = {'a': S, 'b': S, 'n': S, 'x': X}

    splits = {node:[] for node in nodes}
    for node1 in nodes:
        # check each sub-layer within the node to see if it overlaps with other
        # nodes' layers, if those nodes are siblings or cousins of the node
        for node2 in nodes:
            other_nodes = node1.get_all_ancestors([]) + node1.get_all_descendants([])

            current = dfa.start

            # if node2 within height chunk is a sibling or cousin
            if node1 != node2 and node2 not in other_nodes:
                elem = ''

                for s1 in node1.sub_nodes:
                    above = False
                    below = False
                    for s2 in node2.sub_nodes:
                        if layer_overlap_check(s1, s2, nozzle_width):
                            if s1.height > s2.height:
                                above = True
                            if s1.height < s2.height:
                                below = True

                    if above and not below: elem = 'a'
                    if not above and below: elem = 'b'
                    if above and below: elem = 'x'
                    if not above and not below: elem = 'n'

                    # transition in the dfa according element value
                    current = dfa.transition(current, elem)
                    if current in dfa.final:
                        splits[node1].append(s1.height-1)

    for node in splits:
        for split in splits[node]:
            split_super_node_at_height(node, split)


def split_super_node_at_height(node, height):
    subs1 = [s for s in node.sub_nodes if s.height > height]
    subs2 = [s for s in node.sub_nodes if s.height <= height]
    if len(subs1) > 0 and len(subs2) > 0:
        split_node = Node(node.data+'_split_'+str(height))
        split_node.name = split_node.data
        split_node.depth = node.depth + 1
        split_node.height = node.height
        split_node.children = [child for child in node.children]
        split_node.parent = node
        split_node.sub_nodes = subs1
        split_node.min_sub_height = subs1[0].height
        split_node.max_sub_height = subs1[-1].height

        node.children = [split_node]
        node.sub_nodes = subs2
        node.min_sub_height = subs2[0].height
        node.max_sub_height = subs2[-1].height

        descendants = split_node.get_all_descendants([])
        for d in descendants:
            d.depth = d.depth + 1
        #print("Split node "+str(node.name)+" at height "+str(height)+": from "+str(subs2.min_sub_height)+" to "+ str(subs1.max_sub_height))
    else:
        print("Split called on node "+str(node.name)+" that divides poorly at height "+str(height)+", "+str(len(node.sub_nodes))+": from "+str(node.min_sub_height)+" to "+ str(node.max_sub_height))


def union_curves(curves):
    if curves!=None:
        if len(curves) > 1:
            try:
                return rs.CurveBooleanUnion([curve for curve in curves if rs.IsCurve(curve) and rs.IsCurveClosed(curve)])
            except Exception as err:
                for curve in curves:
                    print(curve, rs.ObjectType(curve))
                raise ValueError(err)
        elif len(curves) == 1:
            return [curve for curve in curves if curve!=None and rs.IsCurve(curve) and rs.IsCurveClosed(curve)]
        else:
            raise ValueError("Called union_curves with no curves")


def union_curves_on_xy_plane(curves):
    if curves!=None:
        if len(curves) > 1:
            try:
                return rs.CurveBooleanUnion([rs.CopyObject(curve, [0, 0, -rs.CurveStartPoint(curve).Z]) for curve in curves if rs.IsCurve(curve) and rs.IsCurveClosed(curve)])
            except Exception as err:
                for curve in curves:
                    print(curve, rs.ObjectType(curve))
                raise ValueError(err)
        elif len(curves) == 1:
            return [rs.CopyObject(curve, [0, 0, -rs.CurveStartPoint(curve).Z]) for curve in curves if curve!=None and rs.IsCurve(curve) and rs.IsCurveClosed(curve)]
        else:
            raise ValueError("Called union_curves_on_xy_plane with no curves")


def layer_overlap_check(sub1, sub2, width):
    if xy_plane_crvs.get(sub1) == None:
        xy_plane_crvs[sub1] = []
        for crvs in sub1.data:
            xy_plane_crvs[sub1].append([rs.CopyObject(curve, [0, 0, -rs.CurveStartPoint(curve).Z]) for curve in crvs if curve!=None and rs.IsCurve(curve) and rs.IsCurveClosed(curve)])
    if xy_plane_crvs.get(sub2) == None:
        xy_plane_crvs[sub2] = []
        for crvs in sub2.data:
            xy_plane_crvs[sub2].append([rs.CopyObject(curve, [0, 0, -rs.CurveStartPoint(curve).Z]) for curve in crvs if curve!=None and rs.IsCurve(curve) and rs.IsCurveClosed(curve)])

    for curves1 in xy_plane_crvs[sub1]:
        for curves2 in xy_plane_crvs[sub2]:
            if dynamic_curve_overlap_check(curves1, curves2, width): return True
    return False


def dynamic_curve_overlap_check(curves1, curves2, width):
    if overlap.get(curves1[0]) == None or overlap[curves1[0]].get(curves2[0]) == None or overlap[curves1[0]][curves2[0]].get(width) == None:
        if overlap.get(curves1[0]) == None: overlap[curves1[0]] = {}
        if overlap[curves1[0]].get(curves2[0]) == None: overlap[curves1[0]][curves2[0]] = {}
        overlap[curves1[0]][curves2[0]][width] = curve_overlap_check(curves1, curves2, width)

        if overlap.get(curves2[0]) == None: overlap[curves2[0]] = {}
        if overlap[curves2[0]].get(curves1[0]) == None: overlap[curves2[0]][curves1[0]] = {}
        overlap[curves2[0]][curves1[0]][width] = overlap[curves1[0]][curves2[0]][width]

    return overlap[curves1[0]][curves2[0]][width]


def curve_overlap_check(curves1, curves2, width):
    outer1 = curves1[0]
    inners1 = curves1[1:]

    outer2 = curves2[0]
    inners2 = curves2[1:]
    intersection = rs.PlanarClosedCurveContainment(outer1, outer2, tolerance=width/2)
    if intersection == 1:
        # curve 1 intersects curve 2
        return True
    elif intersection > 1:
        if intersection == 2:
            # outer curve 1 is in outer curve 2
            outer = outer1
            inners = inners2
        if intersection == 3:
            # outer curve 2 is in outer curve 1
            outer = outer2
            inners = inners1

        for inner in inners:
            if xy_bbox_overlap(outer, inner):
                intersection = rs.PlanarClosedCurveContainment(outer, inner, tolerance=width/2)
                if intersection == 1 or intersection==3:
                    return True
                elif intersection == 2:
                    # outer curve is inside an inner curve
                    return False
        return True
    return False


# check that no node in path overlaps next_node
def check_path(next_node, path, graph=None):
    # first check that the node preceding next_node has been printed
    if graph and not next_node.start and graph.get_node(next_node.data.parent) not in path:
        return False

    for node in path:
        if path_overlap.get(next_node) == None or path_overlap[next_node].get(node) == None:
            if path_overlap.get(next_node) == None:
                path_overlap[next_node] = {}
            path_overlap[next_node][node] = check_layers_for_overlap(node.data.sub_nodes, next_node.data.sub_nodes)

        if path_overlap[next_node][node]==True:
            return False
    return True


# if any node in sub_nodes1 overlaps sub_nodes2, return true
def check_layers_for_overlap(sub_nodes1, sub_nodes2):
    # check if sub_nodes1 are all above or on par with sub_nodes2
    if sub_nodes1[-1].height >= sub_nodes2[0].height:
        for sub1 in sub_nodes1:
            for sub2 in sub_nodes2:
                if is_sub_node_overlapping_above(sub1, sub2, nozzle_width):
                    return True
    return False


# is sub1 overlapping sub2?
def is_sub_node_overlapping_above(sub1, sub2, width):
    if overlap_above.get(sub1) == None or overlap_above[sub1].get(sub2) == None or overlap_above[sub1][sub2].get(width) == None:
        if overlap_above.get(sub1) == None:
            overlap_above[sub1] = {}
        if overlap_above[sub1].get(sub2) == None:
            overlap_above[sub1][sub2] = {}
        overlap_above[sub1][sub2][width] = (sub1.height > sub2.height) and layer_overlap_check(sub1, sub2, width)
    return overlap_above[sub1][sub2][width]

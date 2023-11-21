import math
import time
import Rhino
import rhinoscriptsyntax as rs
import extruder_turtle  
import turtle_utilities as tu
from extruder_turtle import *

import tree_utils
from tree_utils import *

import graph_utils
from graph_utils import *

import geometry_utils
from geometry_utils import *

def test_graph():
    test = Graph()
    a = Graph_Node("A")
    b = Graph_Node("B")
    c = Graph_Node("C")
    d = Graph_Node("D")
    test.add_node(a)
    test.add_node(b)
    test.add_node(c)
    test.add_node(d)

    test.add_edge(Graph_Edge(a, b, 3))
    test.add_edge(Graph_Edge(b, a, 3))

    test.add_edge(Graph_Edge(a, c, 5))
    test.add_edge(Graph_Edge(c, a, 5))
    
    test.add_edge(Graph_Edge(b, c, 4))
    test.add_edge(Graph_Edge(c, b, 4))

    test.add_edge(Graph_Edge(c, d, 2))
    test.add_edge(Graph_Edge(d, c, 2))

    path_found, path = test.check_for_path(a, d)
    print(path_found)
    if path: print([node.data for node in path])


def best_vertical_path(t, shape):
    global overlap
    overlap = {}
    vert_tree, center_points = build_vertical_tree(t, shape)
    all_nodes = vert_tree.get_all_nodes([])

    print("Size of grouped height tree: "+str(len(all_nodes)))

    st_time = time.time()

    path = []
    edges = []
    boundingBoxes = []
    nozzle_width = 8 # maximum width of nozzle
    nozzle_height = 30 # height of nozzle in mm
    height = get_shape_height(shape)
    for h in range(int(math.floor(height / nozzle_height))+1):
        nodes_at_height = [node for node in all_nodes if node.height == h]

        # create a graph for this height chunk
        height_graph = Graph()
        min_height = 1000000000
        # add nodes to graph for every super node within the height chunk
        for node in nodes_at_height:
            graph_node = Graph_Node(node)
            graph_node.name = node.name
            height_graph.add_node(graph_node)
            # add start nodes to the graph
            if node.sub_nodes[0].height <= min_height:
                min_height = node.sub_nodes[0].height

        for node in nodes_at_height:
            if node.sub_nodes[0].height == min_height:
                height_graph.starts.append(graph_node)

        print("starts", [n.name for n in height_graph.starts])

        # add edges to graph
        # edges related to height dependency
        for graph_node in height_graph.nodes:
            node1 = graph_node.data
            for child in node1.children:
                if child in nodes_at_height:
                    height_graph.add_edge(Graph_Edge(graph_node, height_graph.get_node(child), 0))

                    arrow = rs.AddCurve([graph_node.data.sub_nodes[-1].start_point, child.sub_nodes[0].start_point])
                    rs.CurveArrows(arrow, 2)
                    edges.append(arrow)

            # edges related to travel between nodes
            direct_relations = node1.get_all_descendants([]) + node1.get_all_ancestors([])
            siblings_and_counsins = [n for n in nodes_at_height if n not in direct_relations]
            for node2 in siblings_and_counsins:
                # do not add edge if there is overlap
                if not is_overlapping(node1, node2, nozzle_width):
                    # compute travel between curves, where weight is set as
                    # distance between center of start and end curves within node
                    weight = rs.Distance(node1.sub_nodes[-1].start_point, node2.sub_nodes[0].start_point)
                    height_graph.add_edge(Graph_Edge(graph_node, height_graph.get_node(node2), weight))

                    arrow = rs.AddCurve([graph_node.data.sub_nodes[-1].start_point, node2.sub_nodes[0].start_point])
                    edges.append(arrow)

        num_edges = 0
        for n in height_graph.edges:
            num_edges = num_edges + len(height_graph.edges[n].keys())
        #height_graph.print_graph_data()
        height_graph.path_check = check_path

        bbs = [rs.BoundingBox([data for sub in node.data.sub_nodes for data in sub.data]) for node in height_graph.nodes]
        for bb in bbs:
            try: 
                box = rs.AddBox(bb)
                boundingBoxes.append(box)
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
                    points = [
                        rs.AddPoint(minX, minY, z),
                        rs.AddPoint(minX, maxY, z),
                        rs.AddPoint(maxX, maxY, z),
                        rs.AddPoint(maxX, minY, z)]
                    srf = rs.AddSrfPt(points)
                    boundingBoxes.append(srf)
                except:
                    print("Unable to create box from bounding box: ", bb)

        path_section = height_graph.get_shortest_hamiltonian_path()[0]
        print(path_section)
        path = path + path_section

    print("Graph construction time: "+str(time.time() - st_time))

    return vert_tree, path, center_points, boundingBoxes, edges


def build_vertical_tree(t, shape):
    start_time = time.time()

    nozzle_width = 8
    layers = int(math.floor(get_shape_height(shape) / t.get_layer_height()))
    root = Node('root')
    root.name = 'root'

    center_point = rs.CreatePoint(0, 0, 0)
    center_points = []
    previous_nodes = [root]
    for l in range(layers + 1):
        curve_groups = get_curves(shape, l*t.get_layer_height())

        outer_curves = []
        for crvs in curve_groups:
            outer_curve = sorted(crvs, key=lambda x: get_area(x), reverse=True)[0]
            center_point = rs.PointAdd(center_point, get_area_center(outer_curve))
            outer_curves.append(outer_curve)
        center_point = rs.CreatePoint(center_point.X/(len(curve_groups) + 1), center_point.Y/(len(curve_groups) + 1), l*t.get_layer_height())
        center_points.append(center_point)

        # combine separated curves if their bounding boxes overlap by the nozzle_width
        idx_groups = {c:[c] for c in range(len(outer_curves))}
        for c1 in range(len(outer_curves)):
            for c2 in range(c1+1, len(outer_curves)):
                curves1 = union_curves_on_xy_plane([outer_curves[c1]])
                curves2 = union_curves_on_xy_plane([outer_curves[c2]])
                overlap_found = False
                for curve1 in curves1:
                    if overlap_found: break
                    for curve2 in curves2:
                        if rs.PlanarClosedCurveContainment(curve1, curve2, tolerance=nozzle_width/2) > 0:
                            print("Overlap between curves at layer: "+str(l))
                            idx_groups[c1].append(c2)
                            overlap_found = True
                            break


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
            new_nodes.append(node)
            if root in previous_nodes:
                node.parent = root
                root.children.append(node)
            else:
                parent_found = False
                for prev_n in previous_nodes:
                    if parent_found: break
                    curves1 = union_curves_on_xy_plane([crv for g in prev_n.data for crv in g])
                    curves2 = union_curves_on_xy_plane([crv for g in groups[c] for crv in g])
                    for curve1 in curves1:
                        if node in prev_n.children: break
                        for curve2 in curves2:
                            if (curve1 and rs.IsCurve(curve1) and rs.IsCurveClosed(curve1)
                                and curve2 and rs.IsCurve(curve2) and rs.IsCurveClosed(curve2)
                                and rs.PlanarClosedCurveContainment(curve1, curve2, tolerance=nozzle_width/2) > 0):
                                node.parent = prev_n
                                prev_n.children.append(node)
                                parent_found = True
                                break
            if node.parent == None: node.needs_support = True
            else: node.needs_support = False

            # may need to address picking the first outer_curve
            # in the group of outer_curves, if more than one
            pnts = rs.DivideCurve(outer_curve_groups[c][0], 100)
            node.start_point = pnts[closest_point(center_point, pnts)[0]]

        previous_nodes = new_nodes

    print("Initial treeing time: "+str(time.time() - start_time)+" seconds")
    print("Initial height tree size: "+str(len(root.get_all_nodes([]))))

    super_root = segment_tree_by_height(t, root, get_shape_height(shape))
    if len(super_root.get_all_nodes([])) > len(root.get_all_nodes([])):
        raise ValueError("There should not be more super nodes than nodes")

    return super_root, center_points

def segment_tree_by_height(t, tree, total_height):
    #nozzle_height = t.get_nozzle_height()
    #nozzle_width = t.get_nozzle_max_width()
    nozzle_height = 30 #height of nozzle in mm
    limit = int(math.floor(nozzle_height / t.get_layer_height()))
    super_root = Node('root')
    super_root.name = 'root'
    super_root.depth = 0
    super_root.height = 0
    idx = 0

    start_time = time.time()
    for child in tree.children:
        group_by_height(child, super_root, limit, idx=idx)
        idx = idx + 1
    print("Grouping tree by nozzle height: "+str(time.time() - start_time)+" seconds")

    s_t = time.time()
    divide_by_overlap(super_root, total_height)
    print("Dividing super tree by overlap: "+str(time.time() - s_t)+" seconds")
    return super_root


def group_by_height(node, super_node, height, idx=0):
    s_node = super_node
    if node.height // height == super_node.height:
        super_node.sub_nodes.append(node)
    elif node.height // height > super_node.height:
        new_super = Node(str(super_node.data)+'_'+str(idx))
        new_super.name = new_super.data
        new_super.parent = super_node
        new_super.depth = super_node.depth + 1
        new_super.height = node.depth // height
        new_super.sub_nodes.append(node)

        super_node.children.append(new_super)

        s_node = new_super
    elif node.height // height < super_node.height:
        raise ValueError("Error, node should not be below current super_node")

    idx = 0
    if len(node.children) > 1:
        for child in node.children:
            new_new_super = Node(str(s_node.data)+'_'+str(idx))
            new_new_super.name = new_new_super.data
            new_new_super.parent = s_node
            new_new_super.depth = s_node.depth + 1
            new_new_super.height = node.height // height
            s_node.children.append(new_new_super)
            group_by_height(child, new_new_super, height, 0)
            idx = idx + 1
    elif len(node.children) == 1:
        group_by_height(node.children[0], s_node, height, idx)
        idx = idx + 1


def divide_by_overlap(super_root, total_height):
    #nozzle_height = t.get_nozzle_height()
    #nozzle_width = t.get_nozzle_max_width()
    nozzle_width = 8
    nozzle_height = 30 #height of nozzle in mm
    height = int(math.floor(total_height / nozzle_height)) + 1
    nodes = super_root.get_all_nodes([])
    for h in range(height):
        nodes_at_height = [node for node in nodes if node.height == h]
        subdivide_by_overlap(nodes_at_height, nozzle_width)


def subdivide_by_overlap(nodes, width):
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
    for n1 in range(len(nodes)):
        # check each sub-layer within the node to see if it overlaps with other
        # nodes' layers, if those nodes are siblings or cousins of the node
        node1 = nodes[n1]
        for n2 in range(len(nodes)):
            node2 = nodes[n2]
            other_nodes = node1.get_all_ancestors([]) + node1.get_all_descendants([])

            current = dfa.start

            # if node2 within height chunk is a sibling or cousin
            if node1 != node2 and node2 not in other_nodes:
                elem = ''

                for s1 in node1.sub_nodes:
                    above = False
                    below = False
                    for s2 in node2.sub_nodes:
                        if curve_overlap_check(union_curves_on_xy_plane(s1.data), union_curves_on_xy_plane(s2.data), width):
                            if s1.height > s2.height:
                                above = True
                            if s1.height < s2.height:
                                below = True

                    if above and not below: elem = 'a'
                    if not above and below: elem = 'b'
                    if above and below: elem = 'n'
                    if not above and not below: elem = 'x'

                    # transition in the dfa according element value
                    current = dfa.transition(current, elem)
                    if current in dfa.final:
                        splits[node1].append(s1.height-1)
                        #print("Split!", splits)

    for node in splits:
        for split in splits[node]:
            split_super_node_at_height(node, split)


def split_super_node_at_height(node, height):
    subs1 = [n for n in node.sub_nodes if n.height <= height]
    subs2 = [n for n in node.sub_nodes if n.height > height]
    if len(subs1) > 0 and len(subs2) > 0:
        split_node = Node(node.data+'_split_'+str(height))
        split_node.name = split_node.data
        split_node.depth = node.depth
        split_node.height = node.height
        split_node.children.append(node)
        split_node.parent = node.parent
        split_node.parent.children.append(split_node)
        split_node.parent.children.remove(node)
        split_node.sub_nodes = subs1

        node.depth = node.depth + 1
        node.parent = split_node
        node.sub_nodes = subs2

        descendants = node.get_all_descendants([])
        for d in descendants:
            d.depth = d.depth + 1


def union_curves_on_xy_plane(curves):
    if len(curves) > 1:
        return rs.CurveBooleanUnion([rs.CopyObject(curve, rs.AddPoint(0, 0, -rs.CurveStartPoint(curve).Z)) for curve in curves if rs.IsCurve(curve) and rs.IsCurveClosed(curve)])
    elif len(curves) == 1:
        return [rs.CopyObject(curve, rs.AddPoint(0, 0, -rs.CurveStartPoint(curve).Z)) for curve in curves if rs.IsCurve(curve) and rs.IsCurveClosed(curve)]
    else:
        raise ValueError("Called union_curves_on_xy_plane with no curves")


def is_overlapping(node1, node2, width=0):
    curves1 = union_curves_on_xy_plane([curve for sub1 in node1.sub_nodes for group in sub1.data for curve in group])
    curves2 = union_curves_on_xy_plane([curve for sub2 in node2.sub_nodes for group in sub2.data for curve in group])
    return curve_overlap_check(curves1, curves2, width)


def curve_overlap_check(curves1, curves2, width=0):
    for curve1 in curves1:
        for curve2 in curves2:
            # curve1 intersects curve2, curve1 is in curve2, or curve2 is in curve1
            if rs.PlanarClosedCurveContainment(curve1, curve2, tolerance=width/2) > 0:
                return True

            #if width > 0 and xy_bbox_overlap(curve1, curve2, width):
                # check if they are within width of each other
            #    points1 = rs.DivideCurve(curve1, get_num_points(curve1, 1.0))
            #    points2 = rs.DivideCurve(curve2, get_num_points(curve2, 1.0))

            #    for pnt1 in points1:
            #        for pnt2 in points2:
            #            if rs.Distance(pnt1, pnt2) < width/2:
            #                return True
    return False


def check_path(next_node, path):
    nozzle_width = 8 #max diameter of the nozzle in mm
    for sub in next_node.data.sub_nodes:
        for node in path:
            for sub2 in node.data.sub_nodes:
                if sub.height > sub2.height and curve_overlap_check(sub.data, sub2.data, nozzle_width):
                    return False
    return True
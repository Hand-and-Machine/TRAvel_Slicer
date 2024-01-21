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

nozzle_width = 8
nozzle_height = 30

# My vertical path finding code
def best_vertical_path(t, shape):
    vert_start_time = time.time()

    global nozzle_width
    nozzle_width = float(t.get_nozzle_width())
    global nozzle_height
    nozzle_height = float(t.get_nozzle_height())

    global overlap_super
    overlap_super = {}
    global overlap
    overlap = {}

    global xy_overlap
    xy_overlap = {}
    global sub_crvs
    sub_crvs = {}

    # center_points is a visualization variable for debugging,
    # it isn't used further in the code for calculations
    init_tree, center_points = build_vertical_tree(t, shape)
    vert_tree = segment_tree_by_height(t, init_tree, get_shape_height(shape))
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
                    points = [
                        rs.AddPoint(minX, minY, z),
                        rs.AddPoint(minX, maxY, z),
                        rs.AddPoint(maxX, maxY, z),
                        rs.AddPoint(maxX, minY, z),
                        rs.AddPoint(minX, minY, z+t.get_layer_height()),
                        rs.AddPoint(minX, maxY, z+t.get_layer_height()),
                        rs.AddPoint(maxX, maxY, z+t.get_layer_height()),
                        rs.AddPoint(maxX, minY, z+t.get_layer_height())]
                    box = rs.Box(points)
                    node.box = box
                except:
                    print("Unable to create box from bounding box: ", bb)
        except:
            print("Could not get bounding box: ", [rs.ObjectType(f) for f in flat])

    print("Size of grouped height tree: "+str(len(all_nodes)))

    st_time = time.time()

    # edges are used for debugging and visualization rather than further calculations
    edges = []

    path = []
    height = get_shape_height(shape)
    for h in range(int(math.floor(height / nozzle_height))+1):
        nodes_at_height = [node for node in all_nodes if node.height == h]
        print("Nodes at nozzle height "+str(h)+": "+str(len(nodes_at_height)))
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
            if node.sub_nodes[0].height <= min_height:
                min_height = node.sub_nodes[0].height

        prev_node = None
        if len(path)>0: prev_node = path[-1]
        for node in height_graph.nodes:
            if node.data.sub_nodes[0].height == min_height:
                weight = 0
                if prev_node!=None:
                    weight = rs.Distance(prev_node.data.sub_nodes[-1].start_point, node.data.sub_nodes[0].start_point)
                height_graph.starts.append((node, weight))
                node.start = True

        #print("starts", [(n[0].name, round(n[1], 2)) for n in height_graph.starts])

        # add edges to graph
        for graph_node in height_graph.nodes:
            # edges related to height dependency
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
                # do not add edge if node2 overlaps node1
                if check_path(height_graph.get_node(node2), [graph_node]):
                    # compute travel between curves, where weight is set as
                    # distance between center of start and end curves within node
                    weight = rs.Distance(node1.sub_nodes[-1].start_point, node2.sub_nodes[0].start_point)
                    height_graph.add_edge(Graph_Edge(graph_node, height_graph.get_node(node2), weight))

                    arrow = rs.AddCurve([graph_node.data.sub_nodes[-1].start_point, node2.sub_nodes[0].start_point])
                    edges.append(arrow)

        #height_graph.print_graph_data()
        height_graph.path_check = check_path

        start_time = time.time()
        path_section = height_graph.get_shortest_hamiltonian_path()[0]
        path = path + path_section
        print("Hamiltonian Path Search Time: "+str(round(time.time() - start_time, 3))+" seconds")

    print("Graph construction and all hamiltonian paths search time: "+str(round(time.time() - st_time, 3))+" seconds")
    print("Total Vertical Path search time: "+str(round(time.time() - vert_start_time, 3))+" seconds")
    return vert_tree, path, center_points, edges


def build_vertical_tree(t, shape):
    start_time = time.time()

    layers = int(math.floor(get_shape_height(shape) / t.get_layer_height()))
    root = Node('root')
    root.name = 'root'

    print('Number of layers:'+str(layers))

    center_point = rs.CreatePoint(0, 0, 0)
    center_points = []
    previous_nodes = [root]
    for l in range(layers + 1):
        curve_groups = get_curves(shape, l*t.get_layer_height())

        outer_curves = []
        for crvs in curve_groups:
            outer_curve = union_curves(crvs)[0]
            center_point = rs.PointAdd(center_point, get_area_center(outer_curve))
            outer_curves.append(outer_curve)
        center_point = rs.CreatePoint(center_point.X/(len(curve_groups) + 1), center_point.Y/(len(curve_groups) + 1), l*t.get_layer_height())
        center_points.append(center_point)

        # combine separated curves if they are within nozzle_width/2 of one another
        idx_groups = {c:[c] for c in range(len(outer_curves))}
        for c1 in range(len(outer_curves)):
            for c2 in range(c1+1, len(outer_curves)):
                if rs.PlanarClosedCurveContainment(outer_curves[c1], outer_curves[c2], tolerance=nozzle_width*5/8) > 0:
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
            new_nodes.append(node)
            if root in previous_nodes:
                node.parent = root
                root.children.append(node)
            else:
                for prev_n in previous_nodes:
                    if not node.parent and is_overlapping(node, prev_n, width=0):
                        node.parent = prev_n
                        prev_n.children.append(node)

            if node.parent == None:
                print('No parent found')
                node.needs_support = True
            else: node.needs_support = False

            # may need to address picking the first outer_curve
            # in the group of outer_curves, if more than one
            pnts = rs.DivideCurve(outer_curve_groups[c][0], 100)
            node.start_point = pnts[closest_point(center_point, pnts)[0]]

        previous_nodes = new_nodes

    print("Initial treeing time: "+str(round(time.time() - start_time, 3))+" seconds")
    print("Initial height tree size: "+str(len(root.get_all_nodes([]))))

    return root, center_points

def segment_tree_by_height(t, tree, total_height):
    limit = nozzle_height / t.get_layer_height()
    super_root = Node('root')
    super_root.name = 'root'
    super_root.depth = 0
    super_root.height = 0
    idx = 0

    start_time = time.time()
    for child in tree.children:
        group_by_height(child, super_root, limit, idx=idx)
        idx = idx + 1
    print("Grouping tree by nozzle height: "+str(round(time.time() - start_time, 3))+" seconds")

    s_t = time.time()
    divide_by_overlap(super_root, total_height)
    print("Dividing super tree by overlap: "+str(round(time.time() - s_t, 3))+" seconds")
    return super_root


def group_by_height(node, super_node, height, idx=0):
    s_node = super_node
    if node.height // height == super_node.height:
        super_node.sub_nodes.append(node)
    elif node.height // height > super_node.height:
        if len(super_node.sub_nodes) > 0:
            new_super = Node(str(super_node.data)+'_'+str(idx))
            new_super.name = new_super.data
            new_super.parent = super_node
            new_super.depth = super_node.depth + 1
            new_super.height = node.height // height
            new_super.sub_nodes.append(node)

            super_node.children.append(new_super)

            s_node = new_super
        else:
            super_node.sub_nodes.append(node)
            super_node.height = node.height // height
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
                        if is_overlapping(s1, s2, xy=True):
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
    else: print("Split called on node that divides poorly at height "+str(height))


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
                return rs.CurveBooleanUnion([rs.CopyObject(curve, rs.AddPoint(0, 0, -rs.CurveStartPoint(curve).Z)) for curve in curves if rs.IsCurve(curve) and rs.IsCurveClosed(curve)])
            except Exception as err:
                for curve in curves:
                    print(curve, rs.ObjectType(curve))
                raise ValueError(err)
        elif len(curves) == 1:
            return [rs.CopyObject(curve, rs.AddPoint(0, 0, -rs.CurveStartPoint(curve).Z)) for curve in curves if curve!=None and rs.IsCurve(curve) and rs.IsCurveClosed(curve)]
        else:
            raise ValueError("Called union_curves_on_xy_plane with no curves")


def curve_overlap_check(curves1, curves2, width=0):
    for curve1 in curves1:
        for curve2 in curves2:
            # curve1 intersects curve2, curve1 is in curve2, or curve2 is in curve1
            if rs.PlanarClosedCurveContainment(curve1, curve2, tolerance=width/2) > 0:
                return True
    return False


# check that no node in path overlaps next_node
def check_path(next_node, path, graph=None):
    # first check that the node preceding next_node has been printed
    if graph and not next_node.start and graph.get_node(next_node.data.parent) not in path:
        return False

    for node in path:
        if overlap_super.get(next_node) == None or overlap_super.get(next_node).get(node) == None:
            if overlap_super.get(next_node) == None: overlap_super[next_node] = {}
            overlap_super.get(next_node)[node] = check_layers_for_overlap(node.data.sub_nodes, next_node.data.sub_nodes)

        if overlap_super.get(next_node)[node]:
            return False
    return True


# if any node in sub_nodes1 overlaps sub_nodes2, return true
def check_layers_for_overlap(sub_nodes1, sub_nodes2):
    # check if sub_nodes1 are all above or on par with sub_nodes2
    if sub_nodes1[-1].height >= sub_nodes2[0].height:
        for sub1 in sub_nodes1:
            for sub2 in sub_nodes2:
                if is_overlapping(sub1, sub2):
                    return True
    return False


# is sub1 overlapping sub2?
def is_overlapping(sub1, sub2, xy=False, width=nozzle_width):
    if ((xy and (xy_overlap.get(sub1) == None or xy_overlap[sub1].get(sub2) == None)) 
                or (overlap.get(sub1) == None or overlap[sub1].get(sub2) == None)):
        if sub_crvs.get(sub1) == None:
            sub_crvs[sub1] = union_curves_on_xy_plane([crv for g in sub1.data for crv in g])
        if sub_crvs.get(sub2) == None:
            sub_crvs[sub2] = union_curves_on_xy_plane([crv for g in sub2.data for crv in g])

        if xy_overlap.get(sub1) == None: xy_overlap[sub1] = {}
        if xy_overlap.get(sub2) == None: xy_overlap[sub2] = {}

        if (xy or sub1.height > sub2.height) and xy_overlap[sub1].get(sub2) == None:
            xy_overlap[sub1][sub2] = curve_overlap_check(sub_crvs[sub1], sub_crvs[sub2], width=width)
            xy_overlap[sub2][sub1] = xy_overlap[sub1][sub2]

        if overlap.get(sub1) == None: overlap[sub1] = {}

        overlap.get(sub1)[sub2] = sub1.height > sub2.height and xy_overlap[sub1][sub2]

    if xy and xy_overlap[sub1][sub2]:
        return True
    if not xy and overlap[sub1][sub2]:
        return True
    return False

import math

import Grid
from Grid import *

import Graph
from Graph import *

import Node
from Node import *

import geometry_utils
from geometry_utils import *

def get_contours(curve, offset, walls=3, wall_mode=False, separate_wall=True):
    all_contours_time = time.time()

    first_contours = [curve]

    root = Node("root")
    root.depth = -1
    isocontours = []

    if not separate_wall:
        root.data = curve

    if first_contours != None:
        for crv in first_contours:
            node = root.add_child(crv)
            node.is_wall = True
            isocontours = isocontours + [crv]
            new_curves = get_isocontours(crv, offset, node, walls=walls, wall_mode=wall_mode)
            if new_curves:
                for crv in new_curves:
                    size = get_size(crv)
                    if size>offset:
                        isocontours.append(crv)

    contour_time = time.time()-all_contours_time

    return root, isocontours, contour_time


def get_isocontours(curve, offset, parent, wall_mode=False, walls=3):
    new_curves = get_isocontour(curve, offset)
    if not new_curves:
        return []
    else:
        curves = [] + new_curves
        new_depth = parent.depth+1
        if (not wall_mode or (wall_mode and new_depth < walls)):
            for c in new_curves:
                node = parent.add_child(c)
                new_new_curves = get_isocontours(c, offset, node, wall_mode=wall_mode, walls=walls)
                for nc in new_new_curves:
                    curves.append(nc)
        return curves


def get_isocontour(curve, offset, reverse=False, fine_precision=False):
    if not curve:
        print("Error: get_isocontours not called with a curve", curve)
        return None
    elif not rs.IsCurveClosed(curve):
        if rs.IsCurveClosable(curve, offset*1.5):
            curve = rs.CloseCurve(curve, offset*1.5)
        else:
            print("Error: get_isocontours called with an unclosable curve: ", curve)
            return None

    # get points and tangent vectors
    if fine_precision: dist = get_segment_distance(offset/3)
    else: dist = get_segment_distance(offset)
    equi_pnts = [pnt for pnt in rs.DivideCurveEquidistant(curve, dist, True)]
    if len(equi_pnts) < 4:
        return None

    points = [(equi_pnts[p], rs.VectorSubtract(equi_pnts[(p+1) % len(equi_pnts)], equi_pnts[(p-1) % len(equi_pnts)])) for p in range(len(equi_pnts))]
    points = points + [(pnt, rs.CurveTangent(curve, rs.CurveClosestPoint(curve, pnt))) for pnt in get_corners(curve)]
    points = sorted(points, key=lambda x: rs.CurveClosestPoint(curve, x[0]))

    grid = Grid([pnt[0] for pnt in points], offset*1.5)

    # determine each new p' at distance offset away from p
    new_points_exist = False
    discarded_points_exist = False

    orientation = rs.ClosedCurveOrientation(curve)

    new_points = [None]*len(points)
    discarded_points = [None]*len(points)

    # get "up" vector
    up = rs.CreateVector(0, 0, 1)
    for i in range(len(points)):
        tangent = points[i][1]

        if round(tangent.X, 3)==0 and round(tangent.Y, 3)==0 and round(tangent.Z, 3)==0:
            tangent = rs.CurveTangent(curve, rs.CurveClosestPoint(curve, points[i][0]))
        if round(tangent.X, 3)==0 and round(tangent.Y, 3)==0 and round(tangent.Z, 3)==0:
            tangent = rs.VectorSubtract(points[(i+1) % len(equi_pnts)][0], points[(i-1) % len(equi_pnts)][0])

        if round(tangent.X, 3)==0 and round(tangent.Y, 3)==0 and round(tangent.Z, 3)==0:
            print("Unable to create tangent", tangent)

        # get vector orthogonal to tangent vector
        if orientation == 1:
            if reverse: ortho = rs.VectorCrossProduct(tangent, up)
            else: ortho = rs.VectorCrossProduct(up, tangent)
        elif orientation == -1:
            if reverse: ortho = rs.VectorCrossProduct(up, tangent)
            else: ortho = rs.VectorCrossProduct(tangent, up)
        # normalize and scale orthogonal vector
        ortho = rs.VectorScale(ortho, offset/rs.VectorLength(ortho))

        # compute new point
        new_point = rs.VectorAdd(points[i][0], ortho)

        # make sure point is actually inside curve
        include = True
        # check that distance from all neighboring points is >= offset
        neighbor_points = grid.get_neighbors(new_point)

        for npoint in neighbor_points:
            if not npoint == points[i][0] and rs.Distance(npoint, new_point) < offset:
                include = False
                break
        if include:
            new_points[i] = new_point
            new_points_exist = True
        else:
            discarded_points[i] = new_point
            discarded_points_exist = True

    # if there are new points
    if new_points_exist:
        # if any points have been discarded
        if discarded_points_exist:
            # get list of lists of all sequential indices
            init_sequences = [[]]
            start_index = next((index for index, value in enumerate(new_points) if value != None and new_points[index-1] == None), -1)
            if start_index !=- 1:
                indices = range(start_index, len(new_points)) + range(0, start_index)
                for i in indices:
                    next_i = (i+1)%len(new_points)
                    if new_points[i] != None:
                        init_sequences[-1].append(i)
                    elif new_points[next_i] != None and next_i != start_index:
                        init_sequences.append([])

            # verify that broken pieces of curve are inside or outside of parent contour
            sequences = []
            for seq in init_sequences:
                if len(seq) > 1 and ((reverse and rs.CurveCurveIntersection(rs.AddCurve([new_points[idx] for idx in seq]), curve)==None and rs.PointInPlanarClosedCurve(new_points[seq[0]], curve)==0) or (not reverse and rs.PlanarClosedCurveContainment(rs.AddCurve([new_points[idx] for idx in seq+seq[-2:0:-1]+[seq[0]]]), curve)==2)):
                    sequences.append(seq)
                elif len(seq) == 1 and ((reverse and rs.PointInPlanarClosedCurve(new_points[seq[0]], curve)==0) or (not reverse and rs.PointInPlanarClosedCurve(new_points[seq[0]], curve)==1)):
                    sequences.append(seq)

            if len(sequences) == 0:
                return None

            # get start and end points of all sequences
            start = [seq[0] for seq in sequences]
            end = [seq[-1] for seq in sequences]

            starts_grid = Grid([new_points[s] for s in start], offset*1.5)
            all_starts = [new_points[k] for k in start]
            # get connections between sequences
            # prime connections dictionary with None index and large initial minimum distance
            connections = {j: (None, 100000) for j in end}
            # find shortest connection provided the line does not intersect the outer curve
            for j in end:
                neighbors = starts_grid.get_neighbors(new_points[j])
                if len(neighbors)<=3: neighbors = starts_grid.get_neighbors(new_points[j], 3)
                if len(neighbors)<=3: neighbors = all_starts
                for n_pnt in neighbors:
                    if n_pnt != new_points[j]:
                        dist = rs.Distance(new_points[j], n_pnt)
                        if dist < connections[j][1]:
                            connections[j] = (new_points.index(n_pnt), dist)

            # get rid of distances now that connections have been made
            connections = {j: connections[j][0] for j in connections}

            # get sequence grouping indices of connections
            nodes = [[i] for i in range(len(sequences))]

            # check through groupings until no more connections are found between sequences
            connections_found = True
            while connections_found:
                connections_found = False
                for node in nodes:
                    idx = nodes.index(node)
                    seq = sequences[node[-1]]
                    connect = connections[seq[-1]]
                    for node2 in nodes:
                        seq2 = sequences[node2[0]]
                        idx2 = nodes.index(node2)
                        if idx != idx2 and connect in seq2:
                            nodes[idx] = nodes[idx] + nodes[idx2]
                            nodes.remove(node2)
                            connections_found = True
                            break
                    if connections_found: break

            # construct curves from groupings of indices
            curves = []
            for node in nodes:
                curves.append([])
                for idx in node:
                    curves[-1] = curves[-1] + [new_points[i] for i in sequences[idx]]

            # make sure curve is closed; add start point to the end
            curves = [c+[c[0]] for c in curves]

            # Transform point lists into curves
            curves = [rs.AddCurve(c) for c in curves if len(c) > 2]
            curves = [crv for crv in curves if ((reverse and rs.CurveCurveIntersection(crv, curve)==None) or (not reverse and rs.PlanarClosedCurveContainment(crv, curve)==2))]
            return curves
        else:
            curves = [rs.AddCurve(new_points + [new_points[0]])]
            curves = [crv for crv in curves if ((reverse and rs.CurveCurveIntersection(crv, curve)==None) or (not reverse and rs.PlanarClosedCurveContainment(crv, curve)==2))]
            return curves
    else:
        return None


def trim_curve(curve, offset, start_pnt):
    start_param = rs.CurveClosestPoint(curve, start_pnt)
    start = rs.EvaluateCurve(curve, start_param)
    split_circ = rs.AddCircle(start, offset)
    intersection = rs.CurveCurveIntersection(curve, split_circ)

    if intersection==None:
        return curve

    param = intersection[0][5]
    for i in range(len(intersection)):
        inter = intersection[i]
        if inter[0] == 1:
            trim_crv = rs.TrimCurve(curve, [start_param, inter[5]], delete_input=False)
            crv_length = rs.CurveLength(trim_crv)
            if crv_length<offset*2:
                param = inter[5]
    
    return rs.TrimCurve(curve, [param, start_param], delete_input=False)


def connect_curve_groups(curve_groups, gap, initial_offset=0.0):
    connected_curves = []
    for crvs in curve_groups:
        outer = []
        if initial_offset > 0:
            outer = get_isocontour(crvs[0], initial_offset)
            if outer == None or len(outer) == 0:
                # retry with higher precision
                print("Retrying with higher precision.")
                outer = get_isocontour(crvs[0], initial_offset, fine_precision=True)
        else:
            outer = [crvs[0]]

        holes = []
        for c in range(1, len(crvs)):
            if initial_offset > 0:
                iso = get_isocontour(crvs[c], initial_offset, reverse=True)
                holes = holes + iso
            else:
                holes.append(crvs[c])

        inside = []
        for h in range(len(holes)):
            for o in range(len(outer)):
                if rs.PlanarClosedCurveContainment(holes[h], outer[o])==2:
                    inside.append(True)
                    break
                inside.append(False)

        if all(inside) or len(holes)==0:
            # All expanded hole curves are inside the contracted outer curve(s).
            connected_curves.append(connect_curves(outer+holes, gap))
        else:
            # Expanded hole curves are not all inside the contracted outer curve(s).
            print("Expanded hole curves are not all inside the contracted outer curve(s).")
            inside2 = []
            for h in range(1, len(crvs)):
                for o in range(len(outer)):
                    if rs.PlanarClosedCurveContainment(crvs[h], outer[o])==2:
                        inside2.append(True)
                        break
                    inside2.append(False)

            if all(inside2):
                # All original hole curves are inside the contracted outer curve(s).
                connected_curves.append(connect_curves(outer+crvs[1:], gap))
            else:
                # Original hole curves are not all inside the contracted outer curve(s).
                connected_curves.append(connect_curves(crvs, gap))
    length1 = 0
    for crv in crvs:
        if crv is not None and rs.IsCurve(crv):
            length1 = length1 + rs.CurveLength(crv)
        else:
            print("Not a curve", crv)
    return connected_curves


def connect_curves(curves, offset):
    if len(curves) == 1:
        return curves[0]

    # construct a fully connected graph where each curve
    # represents a single node in the graph, and edges
    # are the minimum distance between each curve
    graph = Graph()
    for c in range(len(curves)):
        node = Graph_Node(curves[c])
        node.name = 'c' + str(c)
        graph.add_node(node)

    # find closest points between all curves and add edges
    closest = {}
    for c1 in range(len(curves)):
        curve1 = curves[c1]
        for c2 in range(c1+1, len(curves)):
            curve2 = curves[c2]
            id, pnt1, pnt2 = rs.CurveClosestObject(curve1, curve2)
            weight = rs.Distance(pnt1, pnt2)

            node1 = graph.get_node(curve1)
            node2 = graph.get_node(curve2)
            graph.add_edge(Graph_Edge(node1, node2, weight))
            graph.add_edge(Graph_Edge(node2, node1, weight))

            if closest.get(node1) == None: closest[node1] = {}
            if closest.get(node2) == None: closest[node2] = {}
            closest[node1][node2] = (pnt2, pnt1, weight)

    # trim graph by ordering edges from most weighted to least
    ordered_edges = get_edge_tuples(graph)

    # remove edges, starting with most weighted and determine
    # if nodes are still reachable from the start node
    start_node = graph.get_node(curves[0])
    for edge in ordered_edges:
        node1 = edge[0]
        node2 = edge[1]
        weight = edge[2]
        graph.remove_edge(node1, node2)
        graph.remove_edge(node2, node1)

        for node in graph.nodes:
            if node != start_node:
                if not graph.check_for_path(start_node, node)[0]:
                    # add edge back if unable to reach all nodes
                    # in graph from the start node (outermost curve)
                    graph.add_edge(Graph_Edge(node1, node2, weight))
                    graph.add_edge(Graph_Edge(node2, node1, weight))
                    break

    # split curves at shortest connection points to other curves
    final_curve = None
    min_edges = get_edge_tuples(graph)
    for edge in min_edges:
        node1 = edge[0]
        node2 = edge[1]
        split_point1 = get_connect_point(closest, node1, node2)[0]
        split_curves1, split_ends1 = split_curve(node1.data, split_point1, tolerance=offset)
        split_point2 = get_connect_point(closest, node2, node1)[0]
        split_curves2, split_ends2 = split_curve(node2.data, split_point2, tolerance=offset)

        pnt1_1 = split_ends1[0]
        pnt1_2 = split_ends1[1]
        pnt2_1 = split_ends2[0]
        pnt2_2 = split_ends2[1]

        all_curves = split_curves1 + split_curves2

        crv1 = rs.AddCurve([pnt1_1, pnt2_1])
        crv2 = rs.AddCurve([pnt1_2, pnt2_2])
        intersect = rs.PlanarCurveCollision(crv1, crv2)

        if not intersect:
            all_curves.append(crv1)
            all_curves.append(crv2)
        else:
            all_curves.append(rs.AddCurve([pnt1_1, pnt2_2]))
            all_curves.append(rs.AddCurve([pnt1_2, pnt2_1]))
        
        final_curve = rs.JoinCurves(all_curves)[0]
        node1.data = final_curve
        node2.data = final_curve

    #length1 = 0
    #print(curves)
    #for crv in curves:
        #print("Object", crv)
        #print("Object type", rs.ObjectType(rs.coerceguid(crv)))
        #length1 = length1 + rs.CurveLength(crv)

    #print("Difference in length between connected curves: "+str(rs.CurveLength(final_curve)-length1))

    return final_curve


def get_connect_point(closest, node1, node2):
    split_point = None
    connection = closest[node1].get(node2)
    if connection:
        split_point = connection[0]
    else:
        connection = closest[node2].get(node1)
        split_point = connection[1]

    weight = connection[2]
    return split_point, weight


def get_edge_tuples(graph):
    ordered_edges = []
    for node1 in graph.edges:
        for node2 in graph.edges[node1]:
            weight = graph.edges[node1][node2]
            if (node1, node2, weight) not in ordered_edges and (node2, node1, weight) not in ordered_edges:
                ordered_edges.append((node1, node2, weight))

    return sorted(ordered_edges, key=lambda x: x[2], reverse=True)


def split_curve_at(curve, points, tolerance=0):
    curves = [curve]
    split_ends = []

    for point in points:
        for crv in curves:
            closest_pnt = rs.EvaluateCurve(crv, rs.CurveClosestPoint(crv, point))
            if rs.Distance(closest_pnt, point) < tolerance:
                curves.remove(crv)
                split_curves, ends = split_curve(crv, point, tolerance)
                split_ends = split_ends + ends
                curves = curves + split_curves
                break
    
    return curves, split_ends


def split_curve(curve, split_point, tolerance):
    split_circ = rs.AddCircle(split_point, tolerance/2)
    intersections = rs.CurveCurveIntersection(curve, split_circ)

    split_curves = [curve]
    if intersections!=None and len(intersections)>1:
        split_curves = rs.TrimCurve(curve, [intersections[1][5], intersections[0][5]])
        if type(split_curves) != list:
            split_curves = [split_curves]
    else:
        print("Error: Unable to split curve at point.")
    
    return split_curves, [intersections[0][1], intersections[1][1]]


def get_corners(curve, resolution=None):
    corners = []
    if rs.IsPolyline(curve):
        poly = curve
    elif resolution!=None and resolution>0:
        poly = rs.ConvertCurveToPolyline(curve, min_edge_length=resolution/4)
    else: poly = rs.ConvertCurveToPolyline(curve)

    for pnt in rs.PolylineVertices(poly):
        curv = rs.CurveCurvature(curve, rs.CurveClosestPoint(curve, pnt))
        if curv!=None and curv[3]<0.5:
            corners.append(pnt)

    return corners
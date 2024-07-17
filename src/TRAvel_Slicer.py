import math
import time
import Rhino
import rhinoscriptsyntax as rs
import extruder_turtle  
import turtle_utilities as tu
from extruder_turtle import *

import outer_travel_utils
from outer_travel_utils import *

import geometry_utils
from geometry_utils import *

import contour_utils
from contour_utils import *

max_z = 0

def draw_points(t, points, start_idx=0, bboxes=[], move_up=True, spiral_seam=False, outer_wall=False):
    global max_z

    travel = []
    if len(points) > 1:
        pos = t.get_position()
        speed = float(t.get_speed())
        layer_height = float(t.get_layer_height())
        nozzle_width = float(t.get_nozzle_width())
        extrude_width = float(t.get_extrude_width())

        # retract if travel is greater than 2mm
        short_dist = 1
        retract_dist = 6.5
        retract_min_dist_requirement = 2
        if rs.Distance(pos, points[start_idx]) > retract_min_dist_requirement:
            t.pen_up()

        if t.get_printer()=='ender':
            t.pen_up()
            t.set_speed(speed*3/2)

            if rs.Distance(pos, points[start_idx]) > retract_min_dist_requirement:
                t.extrude(-retract_dist)
            elif rs.Distance(pos, points[start_idx]) > max(extrude_width*3, 2*layer_height):
                t.extrude(-short_dist)

            t.set_speed(9000)

        if move_up and rs.Distance(pos, points[start_idx]) > max(nozzle_width, 2*layer_height):
            z_lift = 2*layer_height
            higher_z = max(pos.Z, points[start_idx].Z)+z_lift
            # go up layer_height*2, go to start position of next start + layer_height*2
            points1 = [rs.CreatePoint(pos.X, pos.Y, pos.Z+z_lift), rs.CreatePoint(points[start_idx].X, points[start_idx].Y, points[start_idx].Z+z_lift)]
            # go up to higher z between current and next position, move parallel to x-y plane to next start point
            points2 = [rs.CreatePoint(pos.X, pos.Y, higher_z), rs.CreatePoint(points[start_idx].X, points[start_idx].Y, higher_z), points[start_idx]]
            # go up to maximum height, move parallel to x-y plane
            points3 = [rs.CreatePoint(pos.X, pos.Y, max_z+z_lift), rs.CreatePoint(points[start_idx].X, points[start_idx].Y, max_z+z_lift), points[start_idx]]

            if check_path_intersection(t, points1, bboxes):
                travel_points = points1
            elif check_path_intersection(t, points2, bboxes):
                travel_points = points2
            else:
                travel_points = points3
            
            for t_pnt in travel_points:
                t.set_position(t_pnt.X, t_pnt.Y, t_pnt.Z)

            travel.append(rs.AddPolyline([pos]+travel_points))
        else:
            t.set_position(points[start_idx].X, points[start_idx].Y, points[start_idx].Z)
            travel.append(rs.AddPolyline([pos, points[start_idx]]))

        if t.get_printer()=='ender':
            t.set_speed(speed*3/2)
            if rs.Distance(pos, points[start_idx]) > retract_min_dist_requirement:
                t.extrude(retract_dist)
            elif rs.Distance(pos, points[start_idx]) > max(extrude_width*3, 2*layer_height):
                t.extrude(short_dist)

            box = rs.BoundingBox(points)
            side = get_longest_side(box)
            if side<5.0:
                t.set_speed(float(speed*0.5))
            elif side<=20.0:
                t.set_speed(float(speed*(0.05*side)))
            else:
                t.set_speed(speed)

        t.pen_down()

        indices = range(start_idx, len(points)) + range(0, start_idx)
        for p in indices:
            t.set_position(points[p].X, points[p].Y, points[p].Z)
            if points[p].Z > max_z: max_z = points[p].Z

        if spiral_seam: t.set_position(points[start_idx].X, points[start_idx].Y, points[start_idx].Z)

        t.set_speed(speed)

    return travel


def get_transition_movements(t, node_path, start_point):
    boxes = []
    seam_points = []
    transitions = []

    for s in range(len(node_path)):
        if node_path[s].data.height!=node_path[s-1].data.height:
            # only compare to boxes within nozzle height chunk
            boxes = []
        if node_path[s].data.box!=None: boxes.append(node_path[s].data.box)

        for node in node_path[s].data.sub_nodes:
            curves = node.data
            for crvs in curves:
                dist = 100000000000
                s_pnt = None
                for crv in crvs:
                    pnt = rs.EvaluateCurve(crv, rs.CurveClosestPoint(crv, start_point))
                    d = rs.Distance(start_point, pnt)
                    if d < dist:
                        dist = d
                        s_pnt = pnt
                start_point = s_pnt
                seam_points.append(start_point)

    return seam_points, transitions


def check_path_intersection(t, path, boxes):
    nozzle_width = t.get_nozzle_width()
    nozzle_height = t.get_nozzle_height()

    # create a geometry that represents the nozzle along the path
    direct = rs.CreatePoint(path[1].X, path[1].Y, 0.0) - rs.CreatePoint(path[0].X, path[0].Y, 0.0)

    vecCW = rs.VectorScale(rs.VectorUnitize(rs.VectorRotate(direct, -90, [0,0,1])), nozzle_width/2)
    vecCCW = rs.VectorScale(rs.VectorUnitize(rs.VectorRotate(direct, 90, [0,0,1])), nozzle_width/2)

    vol_crv = rs.AddPolyline([path[0]+vecCCW, path[1]+vecCCW, path[1]+vecCW, path[0]+vecCW, path[0]+vecCCW])
    path_vol = rs.ExtrudeCurveStraight(vol_crv, (0, 0, 0), (0, 0, nozzle_height*2))
    rs.CapPlanarHoles(path_vol)

    intersect1 = []
    for b in (range(len(boxes)-2, -1, -1)):
        int1 = rs.IntersectBreps(path_vol, boxes[b])
        if int1 != None:
            intersect1.append(int1)
            break
    if len(intersect1) == 0:
        return True
    return False


def get_corner(outer_curve, inner_curve, offset):
    prec = 100
    outer_points = rs.DivideCurve(outer_curve, prec)
    center = get_area_center(inner_curve)

    closest = {"point": rs.CurveStartPoint(outer_curve), "distance": 1000000}

    for p in range(len(outer_points)):
        pnt = outer_points[p]
        prev_pnt = outer_points[(p-1) % len(outer_points)]
        next_pnt = outer_points[(p+1) % len(outer_points)]
        v1 = rs.VectorUnitize(rs.VectorSubtract(prev_pnt, pnt))
        v2 = rs.VectorUnitize(rs.VectorSubtract(next_pnt, pnt))
        dot = rs.VectorDotProduct(v1, v2)
        add = rs.VectorAdd(v1, v2)

        if not (add.X == 0 and add.Y == 0 and add.Z == 0):
            # acos has a domain of [-1, 1], sometimes rhinoscript
            # returns close to 1 due to a precision error
            angle = math.acos(min(max(dot, -1), 1)) * (180 / math.pi)

            if angle < 160 and angle > 20:
                dist = rs.Distance(pnt, center)
                # check that the corner isn't "inverted"
                inside = rs.PointInPlanarClosedCurve(rs.VectorAdd(pnt, rs.VectorScale(rs.VectorUnitize(add), offset/4)), outer_curve)
                if dist < closest["distance"] and inside:
                    closest["distance"] = dist
                    closest["point"] = pnt

    return closest["point"]


def spiral_contours(t, isocontours, start_index=0):
    if len(isocontours) == 0:
        print("Error: no isocontours passed in")
        return
    # a single spirallable region
    # connect each isocontour with the one succeeding it
    spiral = []
    offset = float(t.get_extrude_width())
    num_pnts = get_num_points(isocontours[0], offset)
    points = rs.DivideCurve(isocontours[0], num_pnts)

    # spiral region
    start_point = None
    spiral_contour_indices = []
    for i in range(len(isocontours)):
        start_point = points[start_index]

        # get break point on isocontour for connecting spiral
        marching_order = range(start_index, -1, -1) + range(len(points)-1, start_index, -1)
        closest = {"point": None, "length":1000000, "distance": 1000000}

        for j in marching_order:
            if j != start_index:
                dist = abs(0.75*offset - rs.Distance(start_point, points[j]))
                if dist < closest["distance"]:
                    indices1 = []
                    indices2 = []
                    if j > start_index:
                        indices1 = range(j, len(points)) + range(0, start_index+1)
                        indices2 = range(j, start_index-1, -1)
                    elif j < start_index:
                        indices1 = range(j, start_index+1)
                        indices2 = range(j, -1, -1) + range(len(points)-1, start_index-1, -1)

                    length1 = len(indices1)
                    length2 = len(indices2)
                    if length2 > length1 and length2 < closest["length"]:
                        closest["distance"] = dist
                        closest["length"] = length2
                        closest["point"] = j
        break_index = closest["point"]
        if break_index == None:
            print("Unable to find break_index on isocontour "+str(i+1)+" out of "+str(len(isocontours)))
            break
        break_point = points[break_index]

        # append the points from the isocontour from start_point
        # to break_point to the spiral
        indices = []
        if start_index > break_index:
            indices = range(start_index, len(points))+range(0, break_index+1)
        elif start_index < break_index:
            indices =  range(start_index, break_index+1)
        else:
            print("Error: start and break index should not be the same")

        spiral = spiral + [points[j] for j in indices]
        spiral_contour_indices.append(len(spiral)-1)

        # find closest point in next contour to the break point
        # if we are not at the centermost contour
        if i < len(isocontours) - 1:
            next_points = rs.DivideCurve(isocontours[i+1], get_num_points(isocontours[i+1], offset))
            closest = {"point": None, "distance": 1000000}
            for j in range(len(next_points)):
                dist = rs.Distance(break_point, next_points[j])
                if dist < closest["distance"]:
                    closest["distance"] = dist
                    closest["point"] = j

            # set next start index and next points
            start_index = closest["point"]
            points = next_points

    # return spiral and corresponding contour index
    return spiral, spiral_contour_indices


def fermat_spiral(contours, start_pnt, offset):
    isocontours = [crv for crv in contours]

    offset = float(offset)*0.8

    start_param = rs.CurveClosestPoint(isocontours[0], start_pnt)
    start = rs.EvaluateCurve(isocontours[0], start_param)
    split_circ = rs.AddCircle(start, offset)
    intersection = rs.CurveCurveIntersection(isocontours[0], split_circ)

    l = intersection[0][5]
    l_pnt = intersection[0][1]
    for i in range(len(intersection)):
        inter = intersection[i]
        if inter[0] == 1:
            trim_crv = rs.TrimCurve(isocontours[0], [start_param, inter[5]], delete_input=False)
            crv_length = rs.CurveLength(trim_crv)
            if crv_length<offset*2:
                l = inter[5]
                l_pnt = inter[1]

    trims = [[l, start_param]]
    joining_curves = []
    for i in range(len(isocontours)-1):
        split_circ = rs.AddCircle(l_pnt, offset)
        intersection = rs.CurveCurveIntersection(isocontours[i], split_circ)

        ll = intersection[0][5]
        ll_pnt = intersection[0][1]
        for inter in intersection:
            trim_crv = rs.TrimCurve(isocontours[i], [l, inter[5]], delete_input=False)
            crv_length = rs.CurveLength(trim_crv)
            if crv_length<offset*2 and rs.Distance(l_pnt, inter[1])>offset/2:
                ll = inter[5]
                ll_pnt = inter[1]

        trims[i] = [ll, start_param]

        # find connecting points on next contour
        up = rs.VectorSubtract(rs.CreatePoint(l_pnt.X, l_pnt.Y, l_pnt.Z+1.0), l_pnt)
        l_prime = rs.CurveClosestPoint(isocontours[i+1], l_pnt+rs.VectorCrossProduct(up, rs.VectorSubtract(l_pnt, start)))
        l_prime_pnt = rs.EvaluateCurve(isocontours[i+1], l_prime)

        ll_prime = rs.CurveClosestPoint(isocontours[i+1], ll_pnt+rs.VectorSubtract(l_prime_pnt, l_pnt))
        ll_prime_pnt = rs.EvaluateCurve(isocontours[i+1], ll_prime)

        joining_curves.append(rs.AddCurve([l_pnt, l_prime_pnt]))
        joining_curves.append(rs.AddCurve([ll_pnt, ll_prime_pnt]))

        start = l_prime_pnt
        start_param = l_prime

        l_pnt = ll_prime_pnt
        l = ll_prime

        trims.append([l, start_param])

    for i in range(len(isocontours)):
        if trims[i][0] < 0.0000001: trims[i][0] = 0.0
        if trims[i][1] < 0.0000001: trims[i][1] = 0.0
        isocontours[i] = rs.TrimCurve(isocontours[i], trims[i], delete_input=False)

    spiraled_curve = rs.JoinCurves(isocontours+joining_curves, tolerance=offset/2.)
    if len(spiraled_curve) > 1:
        print(rs.CurveStartPoint(spiraled_curve[0]).Z)
        for sp_crv in spiraled_curve:
            print(rs.CurveLength(sp_crv))
        spiraled_curve = [sp_crv for sp_crv in spiraled_curve if rs.CurveLength(sp_crv)>offset]
        print(spiraled_curve)

    try:
        return rs.DivideCurve(spiraled_curve, get_num_points(spiraled_curve, offset))
    except:
        print("Error dividing curve: ", spiraled_curve)
        crv = sorted(spiraled_curve, key=lambda x: rs.CurveLength(x))[-1]
        return rs.DivideCurve(crv, get_num_points(crv, offset))


def segment_tree(root):
    region_root = Node("0")
    fill_region(region_root, root, 0)
    return region_root


def fill_region(region_node, node, idx):
    if(len(node.children) == 0):
        # if the number of children is zero, append to curves
        region_node.sub_nodes.append(node.data)
    elif len(node.children) == 1:
        # if the number of children is one, proceed to the next node
        # in the tree and add curve to the region
        region_node.sub_nodes.append(node.data)
        fill_region(region_node, node.children[0], 0)
    elif len(node.children) > 1:
        # if the number of children is greater than one,
        # we have found a split and need to add a new node
        # add new region node and curve to the new region
        new_node = region_node
        if len(region_node.sub_nodes) > 0:
            new_node = region_node.add_child(region_node.data+"_"+str(idx))

        new_node.sub_nodes.append(node.data)

        idx = 0
        for child in node.children:
            if len(child.children) > 1:
                fill_region(new_node, child, idx)
                idx = idx + 1
            else:
                # add new region node
                new_new_node = new_node.add_child(new_node.data+"_"+str(idx))
                fill_region(new_new_node, child, 0)
                idx = idx + 1


def connect_spiralled_nodes(root, offset):
    find_connections(root, offset)
    all_nodes = root.get_all_nodes([])
    all_nodes = {node.data: node for node in all_nodes}

    path = connect_path(root, offset, all_nodes, 0, [])
    spiral = []
    for p in range(len(path)):
        node = all_nodes[path[p][0]]
        indices = get_marching_indices(node, path[p][1], path[p][2], path[p][3])
        if indices != None:
            spiral = spiral + [node.fermat_spiral[idx] for idx in indices]

    return spiral


def connect_path(node, offset, all_nodes, start_idx, spiral):
    final_idx = len(node.fermat_spiral)-1
    if node.parent:
        final_idx = next(idx for idx in node.connection[node.parent.data] if idx != start_idx)

    marching_order, reverse = get_marching_order(node, start_idx, final_idx)

    if not node.reverse:
        node.reverse = reverse

    if len(node.children)>0:
        everything_sorted = sorted([(k, n) for n in node.connection for k in node.connection[n].keys()], key=lambda x: marching_order.index(x[0]))
        sorted_children = []
        for c in everything_sorted:
            include = True
            for child in sorted_children:
                if child[1] == c[1]:
                    include = False
                    break
            if include and c[0] != start_idx: sorted_children.append(c)

        for c in sorted_children:
            if len(c[1]) > len(node.data):
                child = all_nodes[c[1]]
                end_idx = c[0]

                # add path from node to child index to spiral
                spiral.append((node.data, start_idx, end_idx, reverse))

                # recursively call connect_path on child
                child_start_idx = node.connection[child.data][end_idx]
                spiral = connect_path(child, offset, all_nodes, child_start_idx, spiral)

                # find next start index for node
                other_idx = next(idx for idx in child.connection[node.data] if idx != child_start_idx)
                start_idx = child.connection[node.data][other_idx]

    # close off spiral with remaining portion of node's fermat spiral
    spiral.append((node.data, start_idx, final_idx, reverse))

    return spiral


def get_marching_order(node, start, end):
    points = node.fermat_spiral

    other_idx = len(node.fermat_spiral)-1
    if node.parent:
        other_idx = next(idx for idx in node.connection[node.parent.data] if idx != start)

    marching_order = []
    reverse_marching_order = []
    if start > other_idx:
        marching_order = range(start, len(points)) + range(0, other_idx+1)
        reverse_marching_order = range(start, other_idx-1, -1)
    elif start < other_idx:
        marching_order = range(start, other_idx+1)
        reverse_marching_order = range(start, -1, -1) + range(len(points)-1, other_idx-1, -1)
    else:
        print("get_marching_order error: start and end indices should not be the same")

    if len(marching_order) < len(reverse_marching_order):
        return get_marching_indices(node, start, end, True), True
    else:
        return get_marching_indices(node, start, end, False), False


def get_marching_indices(node, start, end, reverse):
    points = node.fermat_spiral

    if reverse:
        if start > end:
            return range(start, end-1, -1)
        elif start < end:
            return range(start, -1, -1) + range(len(points)-1, end-1, -1)
        else:
            print("get_marching_indices error: start and end indices should not be the same")
    else:
        if start > end:
            return range(start, len(points)) + range(0, end+1)
        elif start < end:
            return range(start, end+1)
        else:
            print("get_marching_indices error: start and end indices should not be the same")


def find_connections(node, offset):
    for child in node.children:
        find_connections(child, offset)

    parent = node.parent
    if parent!=None and parent.fermat_spiral!=None:
        connect_node_to_parent(node, parent, offset)


def connect_node_to_parent(node, parent, offset):
    #offset = float(t.get_extrude_width())

    node_start, node_end = get_connection_indices(node, offset)

    points = node.fermat_spiral
    start_pnt = points[node_start]
    end_pnt = points[node_end]

    if node_start == node_end or rs.Distance(start_pnt, end_pnt) == 0:
        print("Error: could not find suitable connection indices", node_start, node_end)
        return

    direction = 90
    vec = rs.VectorSubtract(end_pnt, start_pnt)
    vec = rs.VectorScale(rs.VectorUnitize(rs.VectorRotate(vec, direction, [0, 0, 1])), offset)
    pnt1 = rs.VectorAdd(start_pnt, vec)
    pnt2 = rs.VectorAdd(end_pnt, vec)

    if (rs.PointInPlanarClosedCurve(pnt1, node.sub_nodes[0]) or rs.PointInPlanarClosedCurve(pnt2, node.sub_nodes[0])):
        # collision occurred, switching directions
        vec = rs.VectorSubtract(end_pnt, start_pnt)
        vec = rs.VectorScale(rs.VectorUnitize(rs.VectorRotate(vec, -direction, [0, 0, 1])), offset)
        pnt1 = rs.VectorAdd(start_pnt, vec)
        pnt2 = rs.VectorAdd(end_pnt, vec)

    # search parent node points for points
    # closest to computed intersection
    closest = {"start": {"point": None, "distance": 1000000}, "end": {"point": None, "distance": 1000000}}
    points = parent.fermat_spiral
    for p in range(len(points)):
        dist = rs.Distance(points[p], pnt1)
        if dist < closest["start"]["distance"]:
            closest["start"]["distance"] = dist
            closest["start"]["point"] = p

    for p in range(len(points)):
        dist1 = rs.Distance(points[p], pnt2)
        dist2 = rs.Distance(points[p], pnt1) - offset*0.75
        if (p != closest["start"]["point"]
            and abs(dist2) <= offset*0.75
            and dist1 < closest["end"]["distance"]):
            closest["end"]["distance"] = dist1
            closest["end"]["point"] = p

    parent_start = closest["start"]["point"]
    parent_end = closest["end"]["point"]

    if parent_start == None or parent_end == None:
        print("Did not find suitable connection to outer contour", node_start, node_end, parent_start, parent_end)
        return

    parent.connection[node.data] = { parent_start: node_start, parent_end: node_end }
    node.connection[parent.data] = { node_start: parent_start, node_end: parent_end }


def get_connection_indices(node, offset):
    #offset = float(t.get_extrude_width())

    start_index = 0
    end_index = len(node.fermat_spiral) - 1

    if node.type == 2 or len(node.sub_nodes) == 1:
        points = node.fermat_spiral
        # verify that we haven't already tried to connect to a node
        # at those indices, otherwise move along curve to a new spot
        available_indices = range(len(points))
        connections = node.connection
        if connections:
            connection = [connections[n].keys() for n in connections]
            for c in connection:
                connect_indices = get_shortest_indices(c[0], c[1], points)
                available_indices = [x for x in available_indices if not x in connect_indices]

            # set start index
            start_index = available_indices[0]

        start_pnt = points[start_index]
        # find end index
        closest = {"point": None, "distance": 1000000}
        for i in available_indices:
            dist = rs.Distance(points[i], start_pnt) - 0.75*offset
            if abs(dist) < closest["distance"]:
                closest["distance"] = abs(dist)
                closest["point"] = i

        end_index = closest["point"]
    
    return start_index, end_index


def fill_curve_with_fermat_spiral(t, curve, bboxes=[], move_up=True, start_pnt=None, wall_mode=False, walls=3, wall_first=False, spiral_seam=False):
    extrude_width = float(t.get_extrude_width())

    # slice the shape
    # Generate isocontours
    root, isocontours, contour_time = get_contours(curve, extrude_width, walls=walls, wall_mode=wall_mode)

    st_time = time.time()

    final_spiral = []
    outer_travel_paths = []
    inner_travel_paths = []

    start_point = start_pnt

    # Spiralling Regions
    # if generating the first isocontour resulted in multiple
    # regions, we have to handle them separately
    for node in root.children:
        # there may be multiple inner regions within the outermost wall due to initial_offset
        inner_regions = []
        for child in node.children:
            region_tree = segment_tree(child)
            all_nodes = region_tree.get_all_nodes([])
            for n in all_nodes:
                start_point = rs.EvaluateCurve(n.sub_nodes[0], rs.CurveClosestPoint(n.sub_nodes[0], start_pnt))
                if len(n.sub_nodes) > 1:
                    num_pnts = get_num_points(n.sub_nodes[0], extrude_width)
                    if n.type == 1:
                        if n.parent:
                            #start_point = get_corner(n.sub_nodes[0], n.sub_nodes[-1], extrude_width)
                            start_point = rs.CurveClosestObject(n.sub_nodes[0], n.sub_nodes[-1])[1]
                        n.fermat_spiral = fermat_spiral(n.sub_nodes, start_point, extrude_width)
                    elif n.type == 2:
                        n.fermat_spiral = rs.DivideCurve(n.sub_nodes[0], num_pnts)
                elif len(n.sub_nodes) == 1:
                    num_pnts = get_num_points(n.sub_nodes[0], extrude_width)
                    #n.fermat_spiral = rs.DivideCurve(trim_curve(n.sub_nodes[0], extrude_width*0.25, start_point), num_pnts)
                    n.fermat_spiral = rs.DivideCurve(n.sub_nodes[0], num_pnts)
                else:
                    print("Error: node with no curves in it at all", n)

            if len(all_nodes) > 1:
                inner_regions.append(connect_spiralled_nodes(region_tree, extrude_width))
            elif len(all_nodes) == 1:
                inner_regions.append(all_nodes[0].fermat_spiral)

        amt = 1.0
        if t.get_printer() == 'ender': amt = 3.0

        start_point = rs.EvaluateCurve(curve, rs.CurveClosestPoint(curve, start_pnt))
        outer_wall = trim_curve(node.data, extrude_width*0.5, start_point)

        outer_points = rs.PolylineVertices(rs.ConvertCurveToPolyline(outer_wall, min_edge_length=t.get_resolution()))

        if wall_first:
            outer_travel_paths = outer_travel_paths + draw_points(t, outer_points, 0, bboxes=bboxes, move_up=move_up, spiral_seam=spiral_seam)
            final_spiral = final_spiral + outer_points
        for region in inner_regions:
            region_curve = rs.AddCurve(region)
            region_points = rs.PolylineVertices(rs.ConvertCurveToPolyline(region_curve, min_edge_length=t.get_resolution()))
            if region_points==None: region_points = region
            travel_paths = draw_points(t, region_points, 0, bboxes=bboxes, move_up=move_up, spiral_seam=False)
            if wall_first:
                inner_travel_paths = inner_travel_paths + travel_paths
            else:
                outer_travel_paths = outer_travel_paths + travel_paths
            final_spiral = final_spiral + region_points
        if not wall_first:
            travel_paths = draw_points(t, outer_points, 0, bboxes=bboxes, move_up=move_up, spiral_seam=spiral_seam)
            if len(inner_regions) > 0:
                inner_travel_paths = inner_travel_paths + travel_paths
            else:
                outer_travel_paths = outer_travel_paths + travel_paths
            final_spiral = final_spiral + outer_points

    fermat_time = time.time() - st_time

    return outer_travel_paths, inner_travel_paths, start_point, contour_time, fermat_time


def fill_curve_with_spiral(t, curve, start_pnt=None, wall_first=False):
    extrude_width = float(t.get_extrude_width())

    # slice the shape
    # Generate isocontours
    root, isocontours, contour_time = get_contours(curve, extrude_width)

    travel_paths = []

    for node in root.children:
        outer_wall = node.data
        outer_points = rs.DivideCurve(outer_wall, int(rs.CurveLength(outer_wall)/t.get_resolution()))
        if outer_points == None:
            outer_points = rs.DivideCurve(outer_wall, get_num_points(outer_wall, extrude_width))

        if wall_first:
            start_idx = 0
            if start_pnt: start_idx, d = closest_point(start_pnt, outer_points)
            travel_paths = travel_paths + draw_points(t, outer_points, start_idx)

        for child in node.children:
            region_tree = segment_tree(child)
            all_nodes = region_tree.get_all_nodes([])

            # Spiral Regions
            for node in all_nodes:
                if len(node.sub_nodes) > 0:
                    num_pnts = get_num_points(node.sub_nodes[0], float(t.get_extrude_width()))
                    if node.type == 1:
                        start_idx = 0
                        if start_pnt:
                            start_idx, d = closest_point(start_pnt, rs.DivideCurve(node.sub_nodes[0], num_pnts))
                        spiral, indices = spiral_contours(t, node.sub_nodes, start_idx)
                        travel_paths = travel_paths + draw_points(t, spiral, start_idx=start_idx)
                    elif node.type == 2:
                        points = rs.DivideCurve(node.sub_nodes[0], num_pnts)
                        travel_paths = travel_paths + draw_points(t, points, start_idx=start_idx)
        
        if not wall_first:
            start_idx = 0
            if start_pnt: start_idx, d = closest_point(start_pnt, outer_points)
            travel_paths = travel_paths + draw_points(t, outer_points, start_idx)

    return travel_paths


def fill_curve_with_contours(t, curve, start_pnt=None, wall_mode=False, walls=3):
    extrude_width = float(t.get_extrude_width())

    # slice the shape
    # Generate isocontours
    root, isocontours, contour_time = get_contours(curve, extrude_width, walls=walls, wall_mode=wall_mode)

    isocontours = [rs.DivideCurve(i, int(rs.CurveLength(i)/t.get_resolution())) for i in isocontours]

    travel_paths = []
    start_idx = 0
    for i in range(len(isocontours)):
        points = isocontours[i]
        start_idx, d = closest_point(start_pnt, points)
        if start_idx == None: start_idx = 0

        travel_paths = travel_paths + [rs.AddCurve([t.get_position(), points[start_idx]])]
        t.pen_up()
        t.set_position(points[start_idx].X, points[start_idx].Y, t.get_position().Z)
        t.set_position(points[start_idx].X, points[start_idx].Y, points[start_idx].Z)
        t.pen_down()

        for p in (range(start_idx+1, len(points)) + range(0, start_idx)):
            t.set_position(points[p].X, points[p].Y, points[p].Z)
        t.set_position(points[start_idx].X, points[start_idx].Y, points[start_idx].Z)

    return travel_paths


def slice_fermat_fill(t, shape, start_pnt=None, start=0, end=None, wall_mode=False, walls=3, fill_bottom=False, bottom_layers=3, initial_offset=0.5):
    travel_paths = []
    layers = int(math.floor(get_shape_height(shape) / t.get_layer_height())) + 1

    print("Number of layers: "+str(layers-1))

    if end is None: end = layers

    extrude_width = float(t.get_extrude_width())
    gap = 0.2*extrude_width

    for l in range(start, min(layers, end)):
        get_group_curves_start = time.time()
        curve_groups = get_curves(shape, l*t.get_layer_height())
        curves = connect_curve_groups(curve_groups, gap, initial_offset=initial_offset*extrude_width)
        print("Time to get and group curves: "+str(round(time.time()-get_group_curves_start, 3))+" seconds")

        for crv in curves:
            if start_pnt == None: start_pnt = t.get_position()
            if not wall_mode or (wall_mode and fill_bottom and l<bottom_layers):
                travel_paths = travel_paths + fill_curve_with_fermat_spiral(t, crv, start_pnt=start_pnt)[0]
            else:
                travel_paths = travel_paths + fill_curve_with_fermat_spiral(t, crv, start_pnt=start_pnt, wall_mode=wall_mode, walls=walls)[0]

    return travel_paths


def slice_spiral_fill(t, shape, start_pnt=None, start=0, end=None, wall_mode=False, walls=3, fill_bottom=False, bottom_layers=3, initial_offset=0.5):
    travel_paths = []
    layers = int(math.floor(get_shape_height(shape) / t.get_layer_height())) + 1

    print("Number of layers: "+str(layers-1))

    if end is None: end = layers

    extrude_width = float(t.get_extrude_width())
    gap = 0.2*extrude_width

    for l in range(start, min(layers, end)):
        curve_groups = get_curves(shape, l*t.get_layer_height())
        curves = connect_curve_groups(curve_groups, gap, initial_offset=initial_offset*extrude_width)

        for crv in curves:
            if start_pnt == None: start_pnt = t.get_position()
            if not wall_mode or (wall_mode and fill_bottom and l<bottom_layers):
                travel_paths = travel_paths + fill_curve_with_spiral(t, crv, start_pnt=start_pnt)
            else:
                travel_paths = travel_paths + fill_curve_with_spiral(t, crv, start_pnt=start_pnt, wall_mode=wall_mode, walls=walls)

    return travel_paths


def slice_contour_fill(t, shape, start=0, end=None, wall_mode=False, walls=3, fill_bottom=False, bottom_layers=3, initial_offset=0.5):
    travel_paths = []
    layers = int(math.floor(get_shape_height(shape) / t.get_layer_height())) + 1

    print("Number of layers: "+str(layers-1))

    if end is None: end = layers

    extrude_width = float(t.get_extrude_width())
    gap = 0.2*extrude_width

    for l in range(start, min(layers, end)):
        curve_groups = get_curves(shape, l*t.get_layer_height())
        curves = connect_curve_groups(curve_groups, gap, initial_offset=initial_offset*extrude_width)

        for crv in curves:
            if not wall_mode or (wall_mode and fill_bottom and l<bottom_layers):
                travel_paths = travel_paths + fill_curve_with_contours(t, crv, start_pnt=t.get_position())
            else:
                travel_paths = travel_paths + fill_curve_with_contours(t, crv, start_pnt=t.get_position(), wall_mode=wall_mode, walls=walls)

    return travel_paths


def TRAvel_Slice(t, shape, all_curves, wall_mode=False, walls=3, fill_bottom=False, bottom_layers=3, initial_offset=0.5, spiral_seam=False, debug=False):
    overall_start_time = time.time()

    inner_travel_paths = []
    outer_travel_paths = []
    #tree, node_path, path, edges = best_vertical_path(t, shape, all_curves, initial_offset=initial_offset)
    #node_path, path = outer_travel_reduction(t, shape, all_curves, initial_offset=initial_offset)
    tree, node_path, path, edges = outer_travel_reduction(t, shape, all_curves, initial_offset=initial_offset, debug=debug)

    contour_time = 0
    fermat_time = 0

    start_point = t.get_position()
    extrude_width = t.get_extrude_width()
    gap = 0.2*extrude_width
    boxes = []
    move_up = False
    for s in range(len(node_path)):
        if node_path[s].data.height!=node_path[s-1].data.height:
            # only compare to boxes within nozzle height chunk
            boxes = []
        if node_path[s].data.box!=None: boxes.append(node_path[s].data.box)

        for node in node_path[s].data.sub_nodes:
            start_point = t.get_position()
            curves = connect_curve_groups(node.data, gap, initial_offset=initial_offset)
            for curve in curves:
                if not wall_mode or (wall_mode and fill_bottom and node.height<bottom_layers):
                    outer_travel, inner_travel, start_point, c_time, f_time = fill_curve_with_fermat_spiral(t, curve, bboxes=boxes, move_up=move_up, start_pnt=start_point, spiral_seam=spiral_seam)
                    outer_travel_paths = outer_travel_paths + outer_travel
                    inner_travel_paths = inner_travel_paths + inner_travel
                    contour_time = contour_time + c_time
                    fermat_time = fermat_time + f_time
                else:
                    outer_travel, inner_travel, start_point, c_time, f_time = fill_curve_with_fermat_spiral(t, curve, bboxes=boxes, move_up=move_up, start_pnt=start_point, wall_mode=wall_mode, walls=walls, spiral_seam=spiral_seam)
                    outer_travel_paths = outer_travel_paths + outer_travel
                    inner_travel_paths = inner_travel_paths + inner_travel
                    contour_time = contour_time + c_time
                    fermat_time = fermat_time + f_time
            move_up = False
        move_up = True

    if debug:
        print("Contour generation time: "+str(round(contour_time, 3))+" seconds")
        print("Fermat Spiraling time: "+str(round(fermat_time, 3))+" seconds")
        print("Full path generation: "+str(round(time.time()-overall_start_time, 3))+" seconds")

    return outer_travel_paths, inner_travel_paths, tree, node_path, path, edges

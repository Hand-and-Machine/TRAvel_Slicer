import math
import tree_utils
from tree_utils import *
import geometry_utils
from geometry_utils import *

def get_contours(t, curve, walls=3, wall_mode=False, initial_offset=0.5):
    all_contours_time = time.time()

    print('')

    offset = float(t.get_extrude_width())
    if initial_offset > 0:
        first_contours = get_isocontour(curve, offset*initial_offset)
    else:
        first_contours = [curve]

    root = Node("root")
    root.depth = -1
    isocontours = []
    if first_contours != None:
        for crv in first_contours:
            node = root.add_child(crv)
            node.is_wall = True
            isocontours = isocontours + [crv]
            new_curves = get_isocontours(t, crv, node, walls=walls, wall_mode=wall_mode)
            if new_curves:
                isocontours = isocontours + [crv for crv in new_curves if get_size(crv) > 1.5*offset]

    print("Time to get "+str(len(isocontours))+" contours for layer: "+str(round(time.time()-all_contours_time, 3))+" seconds")

    return root, isocontours


def get_isocontours(t, curve, parent, wall_mode=False, walls=3):
    new_curves = get_isocontour(curve, float(t.get_extrude_width()))
    if not new_curves:
        return []
    else:
        curves = [] + new_curves
        new_depth = parent.depth+1
        if (not wall_mode or (wall_mode and new_depth < walls)):
            for c in new_curves:
                node = parent.add_child(c)
                new_new_curves = get_isocontours(t, c, node, wall_mode=wall_mode, walls=walls)
                for nc in new_new_curves:
                    curves.append(nc)
        return curves


def get_isocontour(curve, offset):
    if not curve:
        print("Error: get_isocontours not called with a curve", curve)
        return None
    elif not rs.IsCurveClosed(curve):
        if rs.IsCurveClosable(curve, offset*1.5):
            curve = rs.CloseCurve(curve, offset*1.5)
        else:
            print("Error: get_isocontours called with an unclosable curve: ", curve)
            return None

    num_pnts = get_num_points(curve, offset)

    if num_pnts <= 5:
        return None

    points = rs.DivideCurve(curve, num_pnts)
    grid = Grid(points, offset/2)

    # determine each new p' at distance offset away from p
    new_points_exist = False
    discarded_points_exist = False

    if len(points) % 2 == 0: short_pnts = len(points)/2
    else: short_pnts = int(math.floor(len(points)/2)) + 1

    orientation = rs.ClosedCurveOrientation(curve)

    new_points = [None]*short_pnts
    discarded_points = [None]*short_pnts
    for i in range(0, len(points), 2):
        prev_i = (i-1) % len(points)
        next_i = (i+1) % len(points)

        # get tangent vector
        tangent = rs.VectorSubtract(points[next_i], points[prev_i])
        # get "up" vector
        up = rs.VectorSubtract(rs.CreatePoint(points[i].X, points[i].Y, points[i].Z+1.0), points[i])
        # get vector orthogonal to tangent vector
        if orientation == 1:
            ortho = rs.VectorCrossProduct(up, tangent)
        elif orientation == -1:
            ortho = rs.VectorCrossProduct(tangent, up)
        # normalize and scale orthogonal vector
        ortho = rs.VectorScale(ortho, offset/rs.VectorLength(ortho))
        # compute new point
        new_point = rs.VectorAdd(points[i], ortho)

        # make sure point is actually inside curve
        include = True
        # check that distance from all neighboring points is >= offset
        neighbor_points = grid.get_neighbors(new_point)

        for npoint in neighbor_points:
            if not npoint == points[i] and rs.Distance(npoint, new_point) < offset:
                include = False
                break
        if include:
            new_points[i/2] = new_point
            new_points_exist = True
        else:
            discarded_points[i/2] = new_point
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

            # verify that broken pieces of curve are within parent contour
            sequences = []
            for seq in init_sequences:
                if len(seq) > 1 and rs.PlanarClosedCurveContainment(rs.AddCurve([new_points[idx] for idx in seq+seq[-2:0:-1]+[seq[0]]]), curve)==2:
                    sequences.append(seq)
                #elif len(seq) == 1 and rs.PointInPlanarClosedCurve(new_points[seq[0]], curve)==1:
                    #sequences.append(seq)

            if len(sequences) == 0:
                return None

            # get start and end points of all sequences
            start = [seq[0] for seq in sequences]
            end = [seq[-1] for seq in sequences]

            inner_grid = Grid([new_points[s] for s in start], offset/2)

            # get connections between sequences
            # prime connections dictionary with None index and large initial minimum distance
            connections = {j: (None, 100000) for j in end}
            # find shortest connection provided the line does not intersect the outer curve
            for j in end:
                neighbors = inner_grid.get_neighbors(new_points[j])
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
                            nodes.pop(idx2)
                            connections_found = True

            # construct curves from groupings of indices
            curves = []
            for node in nodes:
                curves.append([])
                for idx in node:
                    curves[-1] = curves[-1] + [new_points[i] for i in sequences[idx]]

            # make sure curve is closed; add start point to the end
            curves = [c+[c[0]] for c in curves]

            # Transform point lists into curves
            curves = [rs.AddCurve(c) for c in curves if len(c) > 5]
            return curves
        else:
            return [rs.AddCurve(new_points + [new_points[0]])]
    else:
        return None
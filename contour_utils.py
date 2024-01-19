import tree_utils
from tree_utils import *
import geometry_utils
from geometry_utils import *

def get_contours(t, curve, walls=3, wall_mode=False, initial_offset=0.5):
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
            new_curves = get_isocontours(t, crv, node, wall_mode=wall_mode, walls=walls)
            if new_curves:
                isocontours = isocontours + [crv for crv in new_curves if get_size(crv) > 1.5*offset]
   # else:
    #    node = root.add_child(curve)
    #    isocontours = [curve]
    
    return root, isocontours


def get_isocontours(t, curve, parent, wall_mode=False, walls=3, recursion=0):
    if recursion > 30:
        print("Recursion exceeded limit")
        return []
    new_curves = get_isocontour(curve, float(t.get_extrude_width()))
    if not new_curves:
        return []
    else:
        curves = [] + new_curves
        new_depth = parent.depth+1
        if (not wall_mode or (wall_mode and new_depth < walls)):
            for c in new_curves:
                node = parent.add_child(c)
                new_new_curves = get_isocontours(t, c, node, wall_mode=wall_mode, walls=walls, recursion=recursion+1)
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

    winding_order, direction = get_winding_order(curve, points, offset)

    if winding_order == None:
        print("Error: winding order could not return a direction")
        return None

    # determine each new p' at distance offset away from p
    new_points = [None]*len(points)
    discarded_points = [None]*len(points)
    for i in range(len(points)):
        prev_i = (i-1) % len(points)
        next_i = (i+1) % len(points)

        # get tangent vector
        tangent = rs.VectorSubtract(points[next_i], points[prev_i])
        # get vector orthogonal to tangent vector
        ortho = rs.VectorRotate(tangent, direction, [0, 0, 1])
        # normalize and scale orthogonal vector
        ortho = rs.VectorScale(ortho, offset/rs.VectorLength(ortho))
        # compute new point
        new_point = rs.VectorAdd(points[i], ortho)

        # make sure point is actually inside curve
        include = True
        if not rs.PointInPlanarClosedCurve(new_point, curve):
            include = False
        else:
            # check that distance from all neighboring points is >= offset
            neighbor_points = grid.get_neighbors(new_point)

            for point in neighbor_points:
                if not point == points[i] and rs.Distance(point, new_point) < offset:
                    include = False
                    break
        if include:
            new_points[i] = new_point
        else:
            discarded_points[i] = new_point

    # if there are new points
    if not all(x is None for x in new_points):
        # if any points have been discarded
        if not all(x is None for x in discarded_points):
            # get list of lists of all sequential indices
            sequences = [[]]
            start_index = next((index for index, value in enumerate(new_points) if value != None and new_points[index-1] == None), -1)
            if start_index !=- 1:
                indices = range(start_index, len(new_points)) + range(0, start_index)
                for i in indices:
                    next_i = (i+1)%len(new_points)
                    if new_points[i] != None:
                        sequences[-1].append(i)
                    elif new_points[next_i] != None and next_i != start_index:
                        sequences.append([])

            # remove sequences that consist of a single point
            #for seq in sequences:
                #if len(seq) <= 1:
                    #sequences.remove(seq)

            # get start and end points of all sequences
            start = [seq[0] for seq in sequences]
            end = [seq[-1] for seq in sequences]

            # get connections between sequences
            # prime connections dictionary with None
            # index and large initial minimum distance
            connections = {j: (None, 100000) for j in end}
            for i in start:
                for j in end:
                    if i!=j:
                        dist = rs.Distance(new_points[i], new_points[j])
                        if dist < connections[j][1]:
                            connections[j] = (i, dist)

            # get rid of distances now that
            # connections have been made
            connections = {j: connections[j][0] for j in connections}

            # get sequence grouping indices of connections
            nodes = [[i] for i in range(len(sequences))]

            # check through groupings until no more
            # connections are found between sequences
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

            # make sure curve is closed;
            # add start point to the end
            curves = [c+[c[0]] for c in curves]

            # Transform point lists into curves
            curves = [rs.AddCurve(c) for c in curves if len(c) > 5]

            return curves #, new_points, discarded_points
        else:
            return [rs.AddCurve(new_points + [new_points[0]])] #, new_points, discarded_points
    else:
        return None
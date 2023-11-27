import geometry_utils
from geometry_utils import *

def get_contours(t, curve, walls=3, wall_mode=False, initial_offset=0.5):
    first_contours = get_isocontour(curve, float(t.get_extrude_width())*initial_offset)
    first_curve = first_contours[0]
    if len(first_contours) > 0:
        first_curve = sorted(first_contours, key=lambda x: get_area(x), reverse=True)[0]

    root = {"guid": first_curve, "depth": 0, "children":[]}
    isocontours = [] + [first_curve]
    new_curves = get_isocontours(t, first_curve, root, wall_mode=wall_mode, walls=walls)
    if new_curves:
        isocontours = isocontours + new_curves
    
    return root, isocontours


def get_isocontours(t, curve, parent, recursion=0, wall_mode=False, walls=3):
    if recursion > 30:
        print("Recursion exceeded limit")
        return []
    new_curves = get_isocontour(curve, float(t.get_extrude_width()))
    if not new_curves:
        return []
    else:
        curves = [] + new_curves
        new_depth = parent["depth"]+1
        if (not wall_mode or (wall_mode and new_depth < walls)):
            for c in new_curves:
                node = {"guid":c, "depth":new_depth, "parent":parent, "children":[]}
                parent["children"].append(node)
                new_new_curves = get_isocontours(t, c, node, recursion=recursion+1, wall_mode=wall_mode, walls=walls)
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
        #print("Precision too low or curve too small")
        return None

    points = rs.DivideCurve(curve, num_pnts)
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
            # check that distance from all points is >= offset
            idx_range = range(max(i-10, 0), len(points), 4) + range(0, max(i-10, 0), 4)
            for j in idx_range:
                if not i == j and rs.Distance(points[j], new_point) < offset:
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
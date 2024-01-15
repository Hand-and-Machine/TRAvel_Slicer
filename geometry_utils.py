import Rhino
import rhinoscriptsyntax as rs

def get_area(curve):
    try:
        return rs.Area(curve)
    except:
        return 0

def get_area_center(curve):
    try:
        return rs.CurveAreaCentroid(curve)[0]
    except:
        return rs.AddPoint(0, 0, 0)

def get_size(shape):
    # bounding box of shape
    bb = rs.BoundingBox(shape)
    size = rs.Distance(bb[0], bb[2])
    return size


def get_shape_height(shape):
    # bounding box of shape
    bb = rs.BoundingBox(shape)
    height = rs.Distance(bb[0], bb[4])
    return height


def xy_bbox_overlap(crv1, crv2, width=0):
    w = width/2
    # do bounding boxes of shape overlap in XY plane
    bb1 = rs.BoundingBox(crv1)
    minX1 = bb1[0].X - w
    minY1 = bb1[0].Y - w
    maxX1 = bb1[2].X + w
    maxY1 = bb1[2].Y + w
    bb2 = rs.BoundingBox(crv2)
    minX2 = bb2[0].X
    minY2 = bb2[0].Y
    maxX2 = bb2[2].X
    maxY2 = bb2[2].Y
    return minX1 < maxX2 and maxX1 > minX2 and minY1 < maxY2 and maxY1 > minY2


def get_plane(z):
    return rs.PlaneFromNormal([0, 0, z], [0, 0, 1])


def get_surface(curve, z):
    size = get_size(curve)*2

    points = []
    points.append(rs.CreatePoint(-size,-size,z))
    points.append(rs.CreatePoint(-size,size,z))
    points.append(rs.CreatePoint(size,size,z))
    points.append(rs.CreatePoint(size,-size,z))
    surface = rs.AddSrfPt(points)
    return surface


def get_num_points(curve, offset):
    return max(int(rs.CurveLength(curve)/(float(offset)/4)), 4)


def get_winding_order(curve, points, offset):
    # get winding order, CW or CCW
    winding_order = None

    for i in range(4):
        index = i*(len(points)/4)

        tangent = rs.VectorSubtract(points[index+1], points[index-1])
        pnt_cw = rs.VectorAdd(points[index], rs.VectorScale(rs.VectorUnitize(rs.VectorRotate(tangent, -90, [0, 0, 1])), offset/10))
        pnt_ccw = rs.VectorAdd(points[index], rs.VectorScale(rs.VectorUnitize(rs.VectorRotate(tangent, 90, [0, 0, 1])), offset/10))

        direction = None
        if rs.PointInPlanarClosedCurve(pnt_cw, curve):
            winding_order = "CW"
            direction = -90
            break
        elif rs.PointInPlanarClosedCurve(pnt_ccw, curve):
            winding_order = "CCW"
            direction = 90
            break

    return winding_order, direction


def closest_point(point, points):
    closest = {"point": None, "distance": 1000000}
    for p in range(len(points)):
        dist = rs.Distance(point, points[p])
        if dist < closest['distance']:
            closest['distance'] = dist
            closest['point'] = p
    
    return closest['point'], closest['distance']


def get_shortest_indices(start, end, points):
    indices = []

    indices1 = []
    indices2 = []

    if start > end:
        indices1 = range(start, len(points)) + range(0, end+1)
        indices2 = range(end, start-1, -1)
    elif start < end:
        indices1 = range(start, end+1)
        indices2 = range(start, 0, -1) + range(len(points)-1, end-1, -1)

    points1 = [points[x] for x in indices1]
    points2 = [points[x] for x in indices2]

    curve_length_1 = 0
    if len(points1) > 0: curve_length_1 = rs.CurveLength(rs.AddCurve(points1))
    curve_length_2 = 0
    if len(points2) > 0: curve_length_2 = rs.CurveLength(rs.AddCurve(points2))

    if curve_length_1 < curve_length_2: indices = indices1
    else: indices = indices2

    return indices


def get_curves(shape, z, retry=0):
    plane = get_plane(float(z))
    initial_curves = rs.AddSrfContourCrvs(shape, plane)
    if len(initial_curves) > 1: initial_curves = rs.JoinCurves(initial_curves)

    curves = []
    for curve in initial_curves:
        if curve is not None:
            if not rs.IsCurve(curve):
                print("Slice: contour is not curve", curve)
            elif not rs.IsCurveClosed(curve):
                if rs.IsCurveClosable(curve):
                    curves.append(rs.CloseCurve(curve))
                else:
                    print("Slice: curve is not closed and is not closable at height "+str(z))
            else:
                curves.append(curve)

    if initial_curves > 0 and len(curves) == 0 and retry==0:
        print("Slicing shape at height "+str(z)+" was unsuccessful. Retrying.")
        return get_curves(shape, float(z)-0.01, retry+1)
    elif initial_curves > 0 and len(curves) == 0 and retry==1:
        print("Slicing shape at height "+str(z)+" was unsuccessful. Retrying.")
        return get_curves(shape, float(z)+0.02, retry+1)

    try:
        curve_groups = get_curve_groupings(curves)
        if retry>0: print("Success after retry at height "+str(z))
        return curve_groups
    except Exception as err:
        print("Could not group curves at height "+str(z), err)
        return [curves]


def get_curve_groupings(curves):
    # find curve groupings from intersection of shape with plane
    # curves can represent the inside of a surface or potentially
    # a nested curve within another set of curves defining a surface
    all_points = [rs.DivideCurve(curve, 100) for curve in curves]
    inside = {c:{c2:all([rs.PointInPlanarClosedCurve(p, curves[c2]) for p in all_points[c]]) for c2 in range(len(curves)) if c2 != c} for c in range(len(curves))}
    outer_curves = [c for c in range(len(curves)) if not any([inside[c][k] for k in inside[c]])]
    inner_curves = [c for c in range(len(curves)) if c not in outer_curves]

    iteration = 0
    curve_groupings = {c:[] for c in outer_curves}
    while len(inner_curves) > 0:
        if iteration > 30: break
        iteration = iteration+1
        next_group = []
        for c in outer_curves:
            for c2 in inner_curves:
                if inside[c2][c] and all([inside[c2][c3] == inside[c][c3] for c3 in inside[c2] if c3 != c]):
                    curve_groupings[c].append(c2)
                    next_group.append(c2)
                    inner_curves.remove(c2)

        outer_curves = []
        for c in next_group:
            for c2 in inner_curves:
                if inside[c2][c] and all([inside[c2][c3] == inside[c][c3] for c3 in inside[c2] if c3 != c]):
                    outer_curves.append(c2)
                    curve_groupings[c2] = []
                    inner_curves.remove(c2)

    groups = [[c]+curve_groupings[c] for c in curve_groupings]
    curves = [[curves[c] for c in g] for g in groups]
    return curves

def split_curve_at(curve, points, tolerance=0):
    curves = [curve]
    split_ends = []

    for point in points:
        for crv in curves:
            if closest_point(point, rs.DivideCurve(crv, int(rs.CurveLength(curve)/(tolerance/4))))[1] < tolerance:
                curves.remove(crv)
                split_curves, split_ends = split_curve(crv, point, tolerance)
                curves = curves + split_curves
                break
    
    return curves, split_ends


def split_curve(curve, split_point, tolerance):
    points = rs.DivideCurve(curve, get_num_points(curve, tolerance))

    # find closest point first
    closest_idx, dist = closest_point(split_point, points)

    # collect all points on curve that are a distance of <= tolerance/2
    # from the point closest to the split point, then verify that the
    # length of the curve between those two indices
    new_points = [None]*len(points)
    remove_idxs = []
    for p in range(len(points)):
        new_points[p] = points[p]
        if p == closest_idx:
            remove_idxs.append(p)
            new_points[p] = None
        elif p!=closest_idx and rs.Distance(points[p], points[closest_idx]) < tolerance/2:
            indices1 = []
            indices2 = []
            if p > closest_idx:
                indices1 = range(p, len(points)) + range(0, closest_idx+1)
                indices2 = range(closest_idx, p-1, -1)
            elif p < closest_idx:
                indices1 = range(p, closest_idx+1)
                indices2 = range(p, 0, -1) + range(len(points)-1, closest_idx-1, -1)

            points1 = [points[x] for x in indices1]
            points2 = [points[x] for x in indices2]

            curve_length_1 = 0
            if len(points1) > 0: curve_length_1 = rs.CurveLength(rs.AddCurve(points1))
            curve_length_2 = 0
            if len(points2) > 0: curve_length_2 = rs.CurveLength(rs.AddCurve(points2))

            if curve_length_1 < tolerance / 2 or curve_length_2 < tolerance / 2:
                remove_idxs.append(p)
                new_points[p] = None

    sequences = [[]]
    start_index = 0
    if rs.IsCurveClosed(curve):
        start_index = next((index for index, value in enumerate(new_points) if value != None and new_points[index-1] == None), -1)
    if start_index !=- 1:
        indices = range(start_index, len(new_points)) + range(0, start_index)
        for i in indices:
            next_i = (i+1)%len(new_points)
            if new_points[i] != None:
                sequences[-1].append(i)
            elif new_points[next_i] != None and next_i != start_index:
                sequences.append([])

    return [rs.AddCurve([points[p] for p in sequence]) for sequence in sequences if len(sequence) > 1], [points[sequence[0]] for sequence in sequences if len(sequences)>0] + [points[sequence[-1]] for sequence in sequences if len(sequences)>0]

class Grid:
    def __init__(self, points, width):
        bbox = rs.BoundingBox(points)
        self.minX = bbox[0].X
        self.minY = bbox[0].Y
        self.width = float(width)

        self.max_x_idx = int((bbox[2].X - self.minX) // self.width)
        self.max_y_idx = int((bbox[2].Y - self.minY) // self.width)

        self.grid = [[[] for _ in range(self.max_y_idx+1)] for _ in range(self.max_x_idx+1)]

        for point in points:
            x_idx = int((point.X - self.minX) // self.width)
            y_idx = int((point.Y - self.minY) // self.width)

            self.grid[x_idx][y_idx].append(point)

    def get_neighbors(self, point):
        x_idx = int((point.X - self.minX) // self.width)
        y_idx = int((point.Y - self.minY) // self.width)

        points = []

        for x in range(max(0, x_idx-1), min(self.max_x_idx, x_idx+2)):
            for y in range(max(0, y_idx-1), min(self.max_y_idx, y_idx+2)):
                points = points + self.grid[x][y]
        
        return points

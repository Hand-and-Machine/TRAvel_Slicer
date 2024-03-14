import Rhino
import rhinoscriptsyntax as rs

import time

def get_area(curve):
    try:
        areaMass = Rhino.Geometry.AreaMassProperties.Compute(curve)
        area = areaMass.Area
        return area, True
    except:
        try:
            area = rs.CurveAreaCentroid(rs.coercecurve(curve))
            return area[1], True
        except Exception as err:
            print(err)
            length = rs.CurveLength(curve)
            print("Could not get area of curve. Curve is closed: "+str(curve.IsClosed)+". Curve length: "+str(length), curve)
            return 0, False


def get_area_center(curve):
    try:
        areaMass = curve.AreaMassProperties.Compute()
        center = areaMass.Centroid
        return center
    except:
        try:
            return rs.CurveAreaCentroid(rs.coercecurve(curve))[0]
        except:
            try:
                bbox = rs.BoundingBox(curve)
                center = bbox[0] + 0.5*(bbox[2]-bbox[0])
                return center
            except Exception as err:
                print(err)
                length = rs.CurveLength(curve)
                print("Could not get centroid of curve. Curve is closed: "+str(curve.IsClosed)+". Curve length: "+str(length), curve)
                return rs.CreatePoint(0, 0, rs.CurveStartPoint(curve).Z)


def get_size(shape):
    # bounding box of shape
    bb = rs.BoundingBox(shape)
    size = rs.Distance(bb[0], bb[2])
    return size


def get_shape_height(shape, xy_plane=True):
    # bounding box of shape
    bb = rs.BoundingBox(shape)
    if xy_plane:
        return rs.Distance([bb[0].X, bb[0].Y, 0], bb[4])
    else:
        return rs.Distance(bb[0], bb[4])


def get_longest_side(bb):
    size = max(rs.Distance(bb[0], bb[1]), rs.Distance(bb[1], bb[2]))
    return size


def xy_bbox_overlap(crv1, crv2, width=0):
    w = float(width)/2
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


def get_num_points(curve, tolerance):
    # we justify a coefficient of 1/2, the points
    # overlap by half the extrusion width
    k = 0.5
    return max(int(rs.CurveLength(curve)/(float(tolerance)*k)), 4)


def closest_point(point, points):
    idx = None
    min_dist = 1000000
    for p in range(len(points)):
        dist = rs.Distance(point, points[p])
        if dist < min_dist:
            min_dist = dist
            idx = p
    
    return idx, min_dist


def get_shortest_indices(start, end, points):
    indices = []

    indices1 = []
    indices2 = []

    if start > end:
        indices1 = range(start, len(points)) + range(0, end+1)
        indices2 = range(start, end-1, -1)
    elif start < end:
        indices1 = range(start, end+1)
        indices2 = range(start, -1, -1) + range(len(points)-1, end-1, -1)

    points1 = [points[x] for x in indices1]
    points2 = [points[x] for x in indices2]

    curve_length_1 = 0
    if len(points1) > 0: curve_length_1 = rs.CurveLength(rs.AddCurve(points1))
    curve_length_2 = 0
    if len(points2) > 0: curve_length_2 = rs.CurveLength(rs.AddCurve(points2))

    if curve_length_1 < curve_length_2: indices = indices1
    else: indices = indices2

    return indices


def get_curves(shape, z, retry=0, initial_curves=None):
    if initial_curves==None:
        if z == 0:
            z = 0.1
        plane = get_plane(float(z))
        initial_curves = rs.AddSrfContourCrvs(shape, plane)

    curves = repair_curves(initial_curves)

    if initial_curves > 0 and len(curves) == 0 and retry==0:
        print("Slicing shape at height "+str(z)+" was unsuccessful. Retrying at "+str(round(float(z)+0.01, 3))+".")
        return get_curves(shape, float(z)+0.01, retry=retry+1, initial_curves=None)
    elif initial_curves > 0 and len(curves) == 0 and retry==1:
        print("Slicing shape at height "+str(z)+" was unsuccessful. Retrying at "+str(round(float(z)-0.02, 3))+".")
        return get_curves(shape, float(z)-0.02, retry=retry+1, initial_curves=None)

    try:
        curve_groups = get_curve_groupings(curves)
        if retry>0: print("Success after retry at height "+str(z))
        return curve_groups
    except Exception as err:
        print("Could not group curves at height "+str(z), err)
        return [curves]


def repair_curves(initial_curves):
    if len(initial_curves) > 1:
        initial_curves = rs.JoinCurves(initial_curves)

    curves = []
    for curve in initial_curves:
        if curve is not None:
            if not rs.IsCurve(curve):
                print("Slice: contour is not curve", curve)
            elif not rs.IsCurveClosed(curve):
                if rs.IsCurveClosable(curve):
                    curves.append(rs.CloseCurve(curve))
                else:
                    print("Slice: curve is not closed and is not closable at height "+str(rs.CurveStartPoint(curve).Z))
            else:
                curves.append(curve)

    return curves


def get_curve_groupings(curves):
    # find curve groupings from intersection of shape with plane
    # curves can represent the inside of a surface or potentially
    # a nested curve within another set of curves defining a surface
    inside = {c:{c2:rs.PlanarClosedCurveContainment(curves[c], curves[c2])==2 for c2 in range(len(curves)) if c2 != c} for c in range(len(curves))}
    outer_curves = [c for c in range(len(curves)) if not any([inside[c][k] for k in inside[c]])]
    inner_curves = [c for c in range(len(curves)) if c not in outer_curves]

    iteration = 0
    curve_groupings = {c:[] for c in outer_curves}
    while len(inner_curves) > 0:
        if iteration > 30: break
        iteration = iteration+1
        next_group = []
        inner_copy = [c for c in inner_curves]
        for c in outer_curves:
            for c2 in inner_copy:
                if inside[c2][c] and all([inside[c2][c3] == inside[c][c3] for c3 in inside[c2] if c3 != c]):
                    curve_groupings[c].append(c2)
                    next_group.append(c2)
                    inner_curves.remove(c2)

        outer_curves = []
        inner_copy = [c for c in inner_curves]
        for c in next_group:
            for c2 in inner_copy:
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
        x_idx = max(min(int((point.X - self.minX) // self.width), self.max_x_idx), 0)
        y_idx = max(min(int((point.Y - self.minY) // self.width), self.max_y_idx), 0)

        # start with center
        points = self.grid[x_idx][y_idx]

        # retrieve eight neighbors
        for x in range(max(0, x_idx-1), min(self.max_x_idx+1, x_idx+2)):
            for y in range(max(0, y_idx-1), min(self.max_y_idx+1, y_idx+2)):
                if not (x==x_idx and y==y_idx):
                    points = points + self.grid[x][y]
        
        return points

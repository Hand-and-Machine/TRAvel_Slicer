import Rhino
import rhinoscriptsyntax as rs

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
    # do bounding boxes of shape overlap in XY plane
    bb1 = rs.BoundingBox(crv1)
    minX1 = bb1[0].X
    minY1 = bb1[0].Y
    maxX1 = bb1[2].X
    maxY1 = bb1[2].Y
    bb2 = rs.BoundingBox(crv2)
    minX2 = bb2[0].X
    minY2 = bb2[0].Y
    maxX2 = bb2[2].X
    maxY2 = bb2[2].Y
    return minX1-width/2 < maxX2+width/2 and maxX1+width/2 > minX2-width/2 and minY1-width/2 < maxY2+width/2 and maxY1+width/2 > minY2-width/2


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
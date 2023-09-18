import Rhino
import System.Drawing

def getBitMap(crvs):
    bb = Rhino.Geometry.BoundingBox.Unset
    for crv in crvs:
        bb.Union(crv.GetBoundingBox(True))

    solid_hatch_index = 9
    hatches = Rhino.Geometry.Hatch.Create(crvs, solid_hatch_index, rotationRadians=0, scale=1.0)
    for h in hatches:
        Rhino.RhinoDoc.ActiveDoc.Objects.Add(h) # bake the hatches

    RhinoDocument = Rhino.RhinoDoc.ActiveDoc
    view = RhinoDocument.Views.Find("Top", False)
    vp = view.ActiveViewport
    width, height = bb.Diagonal.X, bb.Diagonal.Y
    size = System.Drawing.Size(width, height)

    Rhino.RhinoApp.RunScript("_SetZoomExtentsBorder _ParallelView=1 _Enter", True)

    vp.ZoomBoundingBox(bb)
    capture = view.CaptureToBitmap(size)
    print(capture)
    return capture
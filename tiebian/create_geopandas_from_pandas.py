from shapely.geometry import MultiPoint
from shapely.wkt import dumps, loads
points = MultiPoint([(0.952782, 0.449282),
                     (0.704062, 0.208803), (0.953877, 0.964079),
                     (0.598159, 0.0764813), (0.840743, 0.0159086),
                     (0.442819, 0.00150517), (0.83682, 0.260941),
                     (0.518703, 0.346917), (0.0222098, 0.910489),
                     (0.375886, 0.116402), (0.898596, 0.577918),
                     (0.429001, 0.494606), (0.199571, 0.0736017),
                     (0.303095, 0.928467), (0.5383, 0.443554),
                     (0.910235, 0.328052), (0.525293, 0.888946),
                     (0.30683, 0.493123), (0.0344695, 0.950913),
                     (0.715335, 0.656555), (0.768716, 0.947667),
                     (0.059516, 0.423081), (0.627099, 0.679968),
                     (0.265189, 0.522203), (0.312344, 0.770625),
                     (0.522695, 0.0243094), (0.408626, 0.984248),
                     (0.892932, 0.0452719), (0.573771, 0.791526),
                     (0.567889, 0.621321)])
ext = loads(points.convex_hull.wkt)
print(ext.exterior.coords[:])

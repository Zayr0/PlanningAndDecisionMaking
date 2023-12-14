import numpy as np


class Corridor:
    def __init__(self):
        # ellipse parameters
        self.ellipsoids = []


class Ellipse:
    def __init__(self, center):
        self.d = center  # center


def find_ellipse(edge_start, edge_end):
    #find center
    # spawn circle at center of edge
    # scale to spheroid, such that it is collision free (ie take nearest obstacle point to be the radius)
    # stretch 3rd axis
    return


#main loop
graph = 0
corridor = Corridor()
for edge in graph:
    ellipse = find_ellipse(edge[0], edge[1])
    corridor.ellipsoids.append(ellipse)
#calculate a convex polygon per ellispoid
# take union of all the polygons
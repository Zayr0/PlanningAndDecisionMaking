import math


def dist3D(self, pos1, pos2):
    x = pos1[0] - pos2[0]
    y = pos1[1] - pos2[1]
    z = pos1[2] - pos2[2]
    return math.sqrt(x ** 2 + y ** 2 + z ** 2)
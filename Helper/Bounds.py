import pybullet as p
import numpy as np

class Bounds:

    #size is a 3x2 vector or array
    def __init__(self, size, center=[0, 0, 0]):
        self.center = center
        self.size = size

        self.xMin = size[0][0]
        self.xMax = size[0][1]

        self.yMin = size[1][0]
        self.yMax = size[1][1]

        self.zMin = size[2][0]
        self.zMax = size[2][1]

    #function that draws the box shaped bounds
    def drawBounds(self):
        sizeMin = np.asarray([self.xMin, self.yMin, self.zMin]) + np.asarray(self.center)
        sizeMax = np.asarray([self.xMax, self.yMax, self.zMax]) + np.asarray(self.center)
        # print(sizeMin, sizeMax)
        f = [sizeMin[0], sizeMin[1], sizeMin[2]]
        t = [sizeMax[0], sizeMin[1], sizeMin[2]]
        p.addUserDebugLine(f, t, [1, 0, 0])
        f = [sizeMin[0], sizeMin[1], sizeMin[2]]
        t = [sizeMin[0], sizeMax[1], sizeMin[2]]
        p.addUserDebugLine(f, t, [0, 1, 0])
        f = [sizeMin[0], sizeMin[1], sizeMin[2]]
        t = [sizeMin[0], sizeMin[1], sizeMax[2]]
        p.addUserDebugLine(f, t, [0, 0, 1])

        f = [sizeMin[0], sizeMin[1], sizeMax[2]]
        t = [sizeMin[0], sizeMax[1], sizeMax[2]]
        p.addUserDebugLine(f, t, [1, 1, 1])

        f = [sizeMin[0], sizeMin[1], sizeMax[2]]
        t = [sizeMax[0], sizeMin[1], sizeMax[2]]
        p.addUserDebugLine(f, t, [1, 1, 1])

        f = [sizeMax[0], sizeMin[1], sizeMin[2]]
        t = [sizeMax[0], sizeMin[1], sizeMax[2]]
        p.addUserDebugLine(f, t, [1, 1, 1])

        f = [sizeMax[0], sizeMin[1], sizeMin[2]]
        t = [sizeMax[0], sizeMax[1], sizeMin[2]]
        p.addUserDebugLine(f, t, [1, 1, 1])

        f = [sizeMax[0], sizeMax[1], sizeMin[2]]
        t = [sizeMin[0], sizeMax[1], sizeMin[2]]
        p.addUserDebugLine(f, t, [1, 1, 1])

        f = [sizeMin[0], sizeMax[1], sizeMin[2]]
        t = [sizeMin[0], sizeMax[1], sizeMax[2]]
        p.addUserDebugLine(f, t, [1, 1, 1])

        f = [sizeMax[0], sizeMax[1], sizeMax[2]]
        t = [sizeMin[0], sizeMax[1], sizeMax[2]]
        p.addUserDebugLine(f, t, [1.0, 0.5, 0.5])
        f = [sizeMax[0], sizeMax[1], sizeMax[2]]
        t = [sizeMax[0], sizeMin[1], sizeMax[2]]
        p.addUserDebugLine(f, t, [1, 1, 1])
        f = [sizeMax[0], sizeMax[1], sizeMax[2]]
        t = [sizeMax[0], sizeMax[1], sizeMin[2]]
        p.addUserDebugLine(f, t, [1, 1, 1])




import math
import pybullet as p
import numpy as np
from Helper.Rotations import Rx, Ry, Rz


class Lidar:
    def __init__(self, mode, numRays):
        self.mode = mode
        self.pos = None

        self.rayHitColor = [1, 0, 0]
        self.rayMissColor = [0, 1, 0]
        self.showRays = False

        self.rayLen = 10.
        self.clearLen = 0.5
        self.clearFrac = self.clearLen/self.rayLen

        self.numRays = numRays
        self.turnFraction = 0.61803398875
        self.power = 1.0


    def scan(self, position, orn):
        if (self.mode == '3D'):
            results = self.scan3D(position, orn)
        elif (self.mode == '2D'):
            results = self.scan2D(position)

        return results

    def scan2D(self, position):
        rayTo = []
        for i in range(self.numRays):
            rayTo.append([
                self.rayLen * math.sin(2. * math.pi * float(i) / self.numRays),
                self.rayLen * math.cos(2. * math.pi * float(i) / self.numRays),
                1])

        rayFrom = [position for i in range(len(rayTo))]
        samplePoints = p.rayTestBatch(rayFrom, rayTo)
        return samplePoints


    def scan3D(self, position, orn):
        results = []
        rayIds = []

        p.removeAllUserDebugItems()
        for i in range(self.numRays):
            t = float(pow(i / self.numRays, self.power))
            theta = math.acos(1 - 2 * t)
            psi = 2 * math.pi * self.turnFraction * i

            R = Rz(orn[2]) * Ry(orn[1]) * Rx(orn[0])

            vec = R * np.array([[self.rayLen * math.sin(theta) * math.cos(psi)],
                                    [self.rayLen * math.sin(theta) * math.sin(psi)],
                                    [self.rayLen * math.cos(theta)]])
            [x, y, z] = np.split(vec, 3)

            nx = ((1 - self.clearFrac) * position[0] + self.clearFrac * x)
            ny = ((1 - self.clearFrac) * position[1] + self.clearFrac * y)
            nz = ((1 - self.clearFrac) * position[2] + self.clearFrac * z)

            nx = position[0]
            ny = position[1]
            nz = position[2]

            result = p.rayTest([nx, ny, nz], [x, y, z])

            results.append(result)

            hitObjectUid = result[0][0]
            if self.showRays:
                rayIds.append(p.addUserDebugLine([nx, ny, nz], [x, y, z], self.rayMissColor))
                if hitObjectUid < 0:
                    p.addUserDebugLine([nx, ny, nz], [x, y, z], self.rayMissColor, replaceItemUniqueId=rayIds[i])
                else:
                    # print('hit object', hitObjectUid, 'with ray', i)
                    p.addUserDebugLine([nx, ny, nz], result[0][3], self.rayHitColor, replaceItemUniqueId=rayIds[i])

        return results


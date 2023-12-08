import pybullet as p
import numpy as np
from scipy.optimize import linprog

#This class makes
class Sampler:

    def __init__(self):
        self.drawSamples = True
        self.printSamples = True

        self.sampleHitColor = [1, 0, 0]
        self.sampleMissColor = [0, 1, 0]
        self.sampleDrawSize = 5

    def sampleSpace(self, numSamples, bounds, obList):
        hitList = np.array([])
        missList = np.array([])

        for i in range(numSamples):
            x = np.random.uniform(bounds.xMin, bounds.xMax)
            y = np.random.uniform(bounds.yMin, bounds.yMax)
            z = np.random.uniform(bounds.zMin, bounds.zMax)

            for obId in obList:
                verts = self.findAllVerts(obId)

                samplePoint = [x, y, z]
                isInside = self.isInside(verts, samplePoint)

                if isInside:
                    np.append(hitList, samplePoint)
                    if self.printSamples:
                        print('Point ', samplePoint, 'is inside object', obId)
                    if self.drawSamples:
                        p.addUserDebugPoints([samplePoint], [self.sampleHitColor], self.sampleDrawSize)
                else:
                    np.append(missList, samplePoint)
                    if self.drawSamples:
                        p.addUserDebugPoints([samplePoint], [self.sampleMissColor], self.sampleDrawSize)
        return hitList, missList


    #This function checks if all given point x is a convex linear combination of points specified.
    #If it is this means the point x is within the convex hull of the points.
    #Reference: https://stackoverflow.com/questions/16750618/whats-an-efficient-way-to-find-if-a-point-lies-in-the-convex-hull-of-a-point-cl#:~:text=For%20each%20of%20the%20edges,it%20lies%20outside%20the%20polygon.
    def isInside(self, points, x):
        n_points = len(points)
        n_dim = len(x)
        c = np.zeros(n_points)
        A = np.r_[points.T,np.ones((1,n_points))]
        b = np.r_[x, np.ones(1)]
        lp = linprog(c, A_eq=A, b_eq=b)
        return lp.success

    #This function finds all vertices of an object with ID obId
    def findAllVerts(self, obId):
        v, i = p.getMeshData(obId, -1)
        pos, orn = p.getBasePositionAndOrientation(obId)
        verts = np.asarray(i)
        all_verts = verts + np.tile(np.asarray(pos), (verts.shape[0], 1))
        return all_verts
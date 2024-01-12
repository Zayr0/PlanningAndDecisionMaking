import cvxpy as cp
import numpy as np
import pybullet as p
from Sphere import Sphere


class VelocityObstacles:

    def __init__(self, tau, dt):
        self.epsilon = 1
        self.tau = tau
        self.dt = dt
        self.numSamples = 100


    def calcVO(self, currentObstacle, movingObstacles):
        rows, cols = (len(movingObstacles), int((self.tau - self.epsilon)/self.dt))
        Spheres = [[None]*cols]*rows
        for mo in range(len(movingObstacles)):
            for i in range(0, cols):
                t = self.epsilon + i * self.dt
                pos = (movingObstacles[mo].pos - currentObstacle.pos)/t + movingObstacles[mo].vel
                radius = (movingObstacles[mo].r + currentObstacle.r)/t
                Spheres[mo][i] = Sphere(pos, radius)
        return Spheres

    def detInputBySampling(self, currentObject, movingObjects):
        # sample
        # reject samples in VO_{i|j}
        # determine cost J(u) for u
        # keep low cost u
        Spheres = self.calcVO(currentObject, movingObjects)

        lowestCost = np.inf
        bestSample = None
        for i in range(self.numSamples):
            sample = np.random.normal(currentObject.desiredVelocity())
            goodSample = self.checkSample(sample, Spheres)
            if goodSample:
                cost = self.calcCost(currentObject.desiredVelocity(), sample)

                if cost < lowestCost:
                    lowestCost = cost
                    bestSample = sample
        return bestSample

    def checkSample(self, s, Spheres):
        x = len(Spheres)
        y = len(Spheres[0])

        for i in range(x):
            for j in range(y):
                dx = s[0] - Spheres[i][j].pos[0]
                dy = s[1] - Spheres[i][j].pos[1]
                dz = s[2] - Spheres[i][j].pos[2]

                dist = np.sqrt(dx**2 + dy**2 + dz**2)
                if dist < Spheres[i][j].r:
                    return False
        return True

    def calcCost(self, prefU, sampleU):
        return np.linalg.norm(prefU - sampleU)


    def detInputByMinimization(self, prefU, movingObjects):
        Wu = 1 * np.eye(prefU.size[0])
        u = cp.Variable((prefU.size))

        uWu = 0.
        constraints = []

        uWu += cp.quad_form(u - prefU, Wu)
        for mo in movingObjects:
            VO = self.calcVO(mo);
            constraints += [u != VO]


        problem = cp.Problem(cp.Minimize(uWu), constraints)
        problem.solve(solver=cp.OSQP)

        return 0#newU
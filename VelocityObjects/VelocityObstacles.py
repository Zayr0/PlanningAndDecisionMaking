import cvxpy as cp
import numpy as np
import pybullet as p
from VelocityObjects.Sphere import Sphere
from math import sqrt
import matplotlib.pyplot as plt


class VelocityObstacles:

    def __init__(self, tau, dt):
        self.epsilon = 1
        self.tau = tau
        self.dt = dt
        self.numSamples = 100


    def calcCircles(self, radius1, radius2, pos1, pos2, vel):
        P1 = (pos2-pos1) + vel
        P2 = (pos2-pos1)/self.tau + vel

        R1 = (radius1+radius2)
        R2 = R1/self.tau
        return P1, P2, R1, R2

    def calcLinConstraint2(self, Cx, Cy, r, Px, Py):
        dx, dy = Px - Cx, Py - Cy
        dxr, dyr = -dy, dx
        d = sqrt(dx ** 2 + dy ** 2)
        if d >= r:
            rho = r / d
            ad = rho ** 2
            bd = rho * sqrt(1 - rho ** 2)
            T1x = Cx + ad * dx + bd * dxr
            T1y = Cy + ad * dy + bd * dyr
            T2x = Cx + ad * dx - bd * dxr
            T2y = Cy + ad * dy - bd * dyr

            return (np.array([T1x, T1y]), np.array([T2x, T2y]), True)
        else:
            return (np.array([0, 0]), np.array([0, 0]), False)

    # https://math.stackexchange.com/questions/543496/how-to-find-the-equation-of-a-line-tangent-to-a-circle-that-passes-through-a-g
    def calcLinConstraint(self, Cx, Cy, r, Px, Py):
        dx, dy = Px - Cx, Py - Cy
        dxr, dyr = -dy, dx
        d = sqrt(dx ** 2 + dy ** 2)
        if d >= r:
            rho = r / d
            ad = rho ** 2
            bd = rho * sqrt(1 - rho ** 2)
            T1x = Cx + ad * dx + bd * dxr
            T1y = Cy + ad * dy + bd * dyr
            T2x = Cx + ad * dx - bd * dxr
            T2y = Cy + ad * dy - bd * dyr

            # print('The tangent points:')
            # print('\tT1≡(%g,%g),  T2≡(%g,%g).' % (T1x, T1y, T2x, T2y))
            if (d / r - 1) < 1E-8:
                print('P is on the circumference')
            else:
                #y = ax+b
                a1 = -(Py - T1y) / (T1x - Px)
                b1 = -(T1y * Px - T1x * Py) / (T1x - Px)
                a2 = -(Py - T2y) / (T2x - Px)
                b2 = -(T2y * Px - T2x * Py) / (T2x - Px)


                # print('The equations of the lines P-T1 and P-T2:')
                # print('\t%+g·y%+g·x%+g = 0' % (T1x - Px, Py - T1y, T1y * Px - T1x * Py))
                # print('\t%+g·y%+g·x%+g = 0' % (T2x - Px, Py - T2y, T2y * Px - T2x * Py))
                return (a1, b1, a2, b2)
        else:
            #print('''Point P≡(%g,%g) is inside the circle with centre C≡(%g,%g) and radius r=%g. No tangent is possible...''' % (Px, Py, Cx, Cy, r))
            print('tangent caculation failed')
            return (0,0,0,0)


    def detInputByMinimization2D(self, drone_pos, obstacles, plotConstraints=True):
        radius = 1.0

        for ob in obstacles:
            p1, p2, r1, r2 = self.calcCircles(radius, ob.r, drone_pos, ob.p, ob.v)
            point1, point2, succes = self.calcLinConstraint2(p1[0], p1[1], r1, ob.v[0], ob.v[1])


            velpos = [ob.v[0] + ob.p[0], ob.v[1] + ob.p[1], ob.v[2] + ob.p[2]]

            pC = [1, 1, 0]

            if ob.pointID == 0:
                ob.pointID = p.addUserDebugPoints([point1, point2, velpos], [pC, pC, pC], pointSize=2)
            else:
                p.addUserDebugPoints([point1, point2, velpos], [pC, pC, pC], pointSize=2, replaceItemUniqueId=ob.pointID)


            # if ob.lineIDs[0] == 0:
            #     ob.lineIDs[0] = p.addUserDebugLine(ob.p, velpos, lineColorRGB=[1, 1, 0])
            # else:
            #     p.addUserDebugLine(ob.p, velpos, lineColorRGB=[1, 1, 0], replaceItemUniqueId=ob.lineIDs[0])
            #
            # if succes:
            #     if ob.lineIDs[1] == 0:
            #         ob.lineIDs[1] = p.addUserDebugLine([point1[0], point1[1], ob.p[2]], velpos, lineColorRGB=[1, 1, 0])
            #     else:
            #         p.addUserDebugLine([point1[0], point1[1], ob.p[2]], velpos, lineColorRGB=[1, 1, 0], replaceItemUniqueId=ob.lineIDs[1])
            #
            #     if ob.lineIDs[2] == 0:
            #         ob.lineIDs[2] = p.addUserDebugLine([point1[0], point1[1], ob.p[2]], velpos, lineColorRGB=[1, 1, 0])
            #     else:
            #         p.addUserDebugLine([point1[0], point1[1], ob.p[2]], velpos, lineColorRGB=[1, 1, 0], replaceItemUniqueId=ob.lineIDs[2])

        return


    def calcVO3D(self, currentObstacle, movingObstacles):
        rows, cols = (len(movingObstacles), int((self.tau - self.epsilon)/self.dt))
        Spheres = [[None]*cols]*rows
        for mo in range(len(movingObstacles)):
            for i in range(0, cols):
                t = self.epsilon + i * self.dt
                pos = (movingObstacles[mo].pos - currentObstacle.pos)/t + movingObstacles[mo].vel
                radius = (movingObstacles[mo].r + currentObstacle.r)/t
                Spheres[mo][i] = Sphere(pos, radius)
        return Spheres

    def detInputBySampling3D(self, currentObject, movingObjects):
        # sample
        # reject samples in VO_{i|j}
        # determine cost J(u) for u
        # keep low cost u
        Spheres = self.calcVO3D(currentObject, movingObjects)

        lowestCost = np.inf
        bestSample = None
        for i in range(self.numSamples):
            sample = np.random.normal(currentObject.desiredVelocity())
            goodSample = self.checkSample3D(sample, Spheres)
            if goodSample:
                cost = self.calcCost(currentObject.desiredVelocity(), sample)

                if cost < lowestCost:
                    lowestCost = cost
                    bestSample = sample
        return bestSample

    def checkSample3D(self, s, Spheres):
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


    def detInputByMinimization3D(self, prefU, movingObjects):
        Wu = 1 * np.eye(prefU.size[0])
        u = cp.Variable((prefU.size))

        uWu = 0.
        constraints = []

        uWu += cp.quad_form(u - prefU, Wu)
        for mo in movingObjects:
            VO = self.calcVO3D(mo);
            constraints += [u != VO]


        problem = cp.Problem(cp.Minimize(uWu), constraints)
        problem.solve(solver=cp.OSQP)

        return 0#newU
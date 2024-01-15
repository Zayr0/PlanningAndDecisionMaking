import cvxpy as cp
import numpy as np
import pybullet as p
from LocalPlanner.VelocityObstacles.Sphere import Sphere
from math import sqrt
from mip import xsum, maximize, BINARY
from gekko import GEKKO

class VelocityObstacles:

    def __init__(self, tau, dt):
        self.epsilon = 1
        self.tau = tau
        self.dt = dt
        self.numSamples = 100
        self.sampleID = 0

        self.drawDroneGizmosIDs = [0, 0]


    def calcCircles(self, ob, drone_pos, radius):
        epsilon = 0.001
        P1 = (ob.p-drone_pos)/epsilon + ob.v
        P2 = (ob.p-drone_pos)/self.tau + ob.v

        R1 = (ob.r+radius)/epsilon
        R2 = (ob.r+radius)/self.tau
        return P1, P2, R1, R2

    def calcLinConstraint2(self, Cx, Cy, r, Px, Py, velPos):
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

            a1 = -(velPos[1] - T1y) / (T1x - velPos[0])
            b1 = -(T1y * velPos[0] - T1x * velPos[1]) / (T1x - velPos[0])
            a2 = -(velPos[1] - T2y) / (T2x - velPos[0])
            b2 = -(T2y * velPos[0] - T2x * velPos[1]) / (T2x - velPos[0])

            return (np.array([T1x, T1y]), np.array([T2x, T2y]), [a1, b1, a2, b2], True)
        else:
            return (np.array([0, 0]), np.array([0, 0]), [0, 0, 0, 0], False)

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


    def detInputBySampling2D(self, desVel, drone_pos, obstacles, envBounds, plotConstraints=True, numSamples=1000):
        radius = 1.0

        vx = np.random.uniform(envBounds.xMin, envBounds.xMax, numSamples)
        vy = np.random.uniform(envBounds.yMin, envBounds.yMax, numSamples)
        V = np.vstack((vx, vy)).T


        for ob in obstacles:
            velpos = np.array(ob.p) + np.array(ob.v)
            p1, p2, r1, r2 = self.calcCircles(ob, drone_pos, radius)
            point1, point2, [a1, b1, a2, b2], succes = self.calcLinConstraint2(p1[0], p1[1], r1, ob.v[0], ob.v[1], velpos)

            acceptedV = []
            if succes:
                for i, po in enumerate(V):
                    # if ( (a1 * p2[0] + b1 - p2[1]) >= 0) or ((a2 * p2[0] + b2 - p2[1]) <= 0):
                    if ((a1*po[0] + b1 - po[1]) >= 0) or ((a2*po[0] + b2 - po[1]) <= 0):
                        acceptedV.append([po[0], po[1], velpos[2]])
            V = np.asarray(acceptedV)

        newVel = np.min(V - desVel*np.ones(V.shape))

        if plotConstraints:
            if self.drawDroneGizmosIDs[0] == 0:
                self.drawDroneGizmosIDs[0] = p.addUserDebugLine(drone_pos, drone_pos + desVel, lineColorRGB=[1, 1, 0])
            else:
                p.addUserDebugLine(drone_pos, drone_pos + desVel, lineColorRGB=[1, 0, 0],
                                   replaceItemUniqueId=self.drawDroneGizmosIDs[0])

            if self.drawDroneGizmosIDs[1] == 0:
                self.drawDroneGizmosIDs[1] = p.addUserDebugLine(drone_pos, drone_pos + newVel, lineColorRGB=[1, 1, 0])
            else:
                p.addUserDebugLine(drone_pos, drone_pos + newVel, lineColorRGB=[1, 1, 0],
                                   replaceItemUniqueId=self.drawDroneGizmosIDs[1])

            extVal = 1
            point1 = np.hstack((extVal * (point1 - velpos[1:2]), ob.p[2]))
            point2 = np.hstack((extVal * (point2 - velpos[1:2]), ob.p[2]))

            pC = [0, 0, 1]

            if ob.pointID == 0:
                ob.pointID = p.addUserDebugPoints([point1, point2, velpos], [pC, pC, pC], pointSize=2)
            else:
                p.addUserDebugPoints([point1, point2, velpos], [pC, pC, pC], pointSize=2,
                                     replaceItemUniqueId=ob.pointID)

            if succes:
                if ob.lineIDs[1] == 0:
                    ob.lineIDs[1] = p.addUserDebugLine(point1, velpos, lineColorRGB=[1, 1, 0])
                else:
                    p.addUserDebugLine(point1, velpos, lineColorRGB=[1, 1, 0], replaceItemUniqueId=ob.lineIDs[1])

                if ob.lineIDs[2] == 0:
                    ob.lineIDs[2] = p.addUserDebugLine(point2, velpos, lineColorRGB=[1, 1, 0])
                else:
                    p.addUserDebugLine(point2, velpos, lineColorRGB=[1, 1, 0], replaceItemUniqueId=ob.lineIDs[2])


            colors = np.tile(np.array([0, 1, 0]), (len(V), 1))
            if len(V) > 0:
                if self.sampleID == 0:
                    self.sampleID = p.addUserDebugPoints(V, colors, pointSize=2)
                else:
                    p.addUserDebugPoints(V, colors, pointSize=2, replaceItemUniqueId=self.sampleID)
        return newVel

    def detInputByMinimization2D(self, desVel, drone_pos, obstacles, plotConstraints=True):
        radius = 1.0

        for ob in obstacles:
            p1, p2, r1, r2 = self.calcCircles(ob, drone_pos, radius)
            point1, point2, [a1, b1, a2, b2], succes = self.calcLinConstraint2(p1[0], p1[1], r1, ob.v[0], ob.v[1])

            velpos = np.array(ob.p) + np.array(ob.v)

            #A_ineq, b_ineq = self.generateConstraints2D(velpos[1:2], point1, point2)

            if plotConstraints:
                extVal = 1
                point1 = np.hstack((extVal * (point1-velpos[1:2]), ob.p[2]))
                point2 = np.hstack((extVal * (point2-velpos[1:2]), ob.p[2]))

                pC = [0, 0, 1]

                if ob.pointID == 0:
                    ob.pointID = p.addUserDebugPoints([point1, point2, velpos], [pC, pC, pC], pointSize=2)
                else:
                    p.addUserDebugPoints([point1, point2, velpos], [pC, pC, pC], pointSize=2, replaceItemUniqueId=ob.pointID)


                if succes:
                    if ob.lineIDs[1] == 0:
                        ob.lineIDs[1] = p.addUserDebugLine(point1, velpos, lineColorRGB=[1, 1, 0])
                    else:
                        p.addUserDebugLine(point1, velpos, lineColorRGB=[1, 1, 0], replaceItemUniqueId=ob.lineIDs[1])

                    if ob.lineIDs[2] == 0:
                        ob.lineIDs[2] = p.addUserDebugLine(point2, velpos, lineColorRGB=[1, 1, 0])
                    else:
                        p.addUserDebugLine(point2, velpos, lineColorRGB=[1, 1, 0], replaceItemUniqueId=ob.lineIDs[2])

            if succes:
                nObs = len(obstacles)

                vxref = desVel[0]*np.ones((nObs, 1))
                vyref = desVel[1] * np.ones((nObs, 1))

                P = np.eye(nObs)

                m = GEKKO()

                vx = [m.add_var() for i in range(nObs)]
                vy = [m.add_var() for i in range(nObs)]
                d = m.add_var(var_type=BINARY)

                m.objective = maximize(-xsum((vx[i]-vxref[i])^2 - (vy[i]-vyref[i])^2) for i in range(nObs))
                m += d * (vy - a1 * vx - b1) >= 0
                m += (1-d) * (vy - a2 * vx - b2) <= 0

                m.optimize()
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
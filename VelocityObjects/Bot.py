import numpy as np
import pybullet as p
from Helper.Bounds import Bounds

class Bot:

    def __init__(self, ID, bounds, start, radius=0.5, color=[0, 0, 1, 0.8]):
        self.ID = ID
        self.maxVel = 2.0

        self.start = start
        self.goal = self.randomPos(bounds)

        self.pos = self.start
        self.desVel = self.desiredVelocity()
        self.vel = self.desVel
        self.r = radius

        self.lineID = ID + 101
        self.pointID = ID + 201

        self.color = color
        self.collision = False

    def drawStartGoal(self):
        p.addUserDebugPoints([self.start], [[1, 0, 0]], 5)
        p.addUserDebugPoints([self.goal], [[0, 1, 0]], 5)
        p.addUserDebugLine(self.start, self.goal, [0, 0, 1])

    def randomPos(self, bounds):
        x = np.random.uniform(bounds.xMin, bounds.xMax)
        y = np.random.uniform(bounds.yMin, bounds.yMax)
        z = np.random.uniform(bounds.zMin, bounds.zMax)

        return np.array([x, y, z])

    def randomVel(self):
        return self.maxVel * (np.random.random((1, 3))[0] - 0.5)

    def desiredVelocity(self):
        return self.maxVel * (self.goal - self.pos)

    def drawVel(self, Vel, color):
        #self.pointID = p.addUserDebugPoints([self.pos + desVel], [[0, 0, 1]], 5, replaceItemUniqueId=self.pointID)
        self.lineID = p.addUserDebugLine(self.pos, self.pos + Vel, color, replaceItemUniqueId=self.lineID)

    def update(self, bots, dt, vel=None):
        self.calculateCollision(bots)
        self.desVel = self.desiredVelocity()
        self.drawVel(self.desVel, [0, 1, 0])

        if vel is None:
            self.vel = self.desVel
        else:
            self.vel = vel
            self.drawVel(vel, [1, 0, 0])
        self.pos = self.pos + self.vel * dt
        p.resetBasePositionAndOrientation(self.ID, self.pos, p.getQuaternionFromEuler([0, 0, 0]))

    def calculateCollision(self, bots):
        for b in bots:
            if np.linalg.norm(b.pos - self.pos) < (b.r + self.r):
                print('Collision between: ', self.ID, 'and ', b.ID)
                self.collision = True
                p.changeVisualShape(self.ID, -1, rgbaColor=[1, 0, 0, 1])
                p.changeVisualShape(b.ID, -1, rgbaColor=[1, 0, 0, 1])
            else:
                self.collision = False
                p.changeVisualShape(self.ID, -1, rgbaColor=self.color)


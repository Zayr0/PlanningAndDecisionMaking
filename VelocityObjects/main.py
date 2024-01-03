import numpy as np
import pybullet as p
import time
from Bot import Bot
from VelocityObjects.VelocityObstacles import VelocityObstacles
from Helper.Bounds import Bounds
import matplotlib.pyplot as plt
from environment.world import Environment
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setRealTimeSimulation(1)
p.setGravity(0, 0, 0)

bLen = 3.0
bSize = np.array([[-bLen, bLen], [-bLen, bLen], [-bLen, bLen]])
bounds = Bounds(bSize)

alpha = 0.8

robotStart = np.array([0, 0, 3])
robotID = p.loadURDF("VisualSphere.urdf", robotStart,  useMaximalCoordinates=False)
robot = Bot(robotID, bounds, robotStart, color=[0, 1, 0, 1])
robot.drawStartGoal()


numBots = 10
bots = []

for i in range(1, numBots + 1):
    x = np.random.uniform(bounds.xMin, bounds.xMax)
    y = np.random.uniform(bounds.yMin, bounds.yMax)
    z = np.random.uniform(bounds.zMin, bounds.zMax)

    botID = p.loadURDF("VisualSphere.urdf", [x, y, z],  useMaximalCoordinates=False)
    bots.append(Bot(i, bounds, np.array([x, y, z])))

    bots[botID - 1].drawStartGoal()

# env = Environment()
bounds.drawBounds()

dt = 1.0/240.

VO = VelocityObstacles(3, 1/10)

while (p.isConnected()):
    newVel = VO.detInputBySampling(robot, bots)
    robot.update(bots, dt, newVel)

    bots.append(robot)

    for b in bots:
        bots.remove(b)

        if True:
            newVel = VO.detInputBySampling(b, bots)
            b.update(bots, dt, newVel)
        else:
            b.update(bots, dt)
        bots.append(b)

    bots.remove(robot)
    time.sleep(dt)

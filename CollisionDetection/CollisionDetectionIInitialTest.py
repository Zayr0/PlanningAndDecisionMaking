import os
import numpy as np
import time
import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#droneId should be 0 and plane should be 1
droneId = p.loadURDF("r2d2.urdf", [0, 0, 3],  useMaximalCoordinates=False)
cubeId = p.loadURDF("cube.urdf", [0, 0, 3],  useMaximalCoordinates=False)
groundId = p.loadURDF("plane.urdf", useMaximalCoordinates=False)



collisionFilterGroup = 0
collisionFilterMask = 0
p.setCollisionFilterGroupMask(droneId, -1, collisionFilterGroup, collisionFilterMask)

enableCollision = 1
p.setCollisionFilterPair(groundId, droneId, -1, -1, enableCollision)

p.setRealTimeSimulation(1)
p.setGravity(0, 0, -10)


while (p.isConnected()):
  time.sleep(1. / 240.)
  points = p.getContactPoints(groundId, droneId, -1, -1)
  print(points)

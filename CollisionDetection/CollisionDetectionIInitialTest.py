import os
import numpy as np
import time
import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#droneId should be 0 and plane should be 1
droneId = p.loadURDF("sphere2.urdf", [0, 0, 3],  useMaximalCoordinates=False)
cubeId = p.loadURDF("cube.urdf", [0, 0, 2],  useMaximalCoordinates=False)
groundId = p.loadURDF("plane.urdf", useMaximalCoordinates=False)



collisionFilterGroup = 0
collisionFilterMask = 0
p.setCollisionFilterGroupMask(droneId, -1, collisionFilterGroup, collisionFilterMask)
p.setCollisionFilterGroupMask(cubeId, -1, collisionFilterGroup, collisionFilterMask)
p.setCollisionFilterGroupMask(groundId, -1, collisionFilterGroup, collisionFilterMask)

enableCollision = 1
p.setCollisionFilterPair(groundId, droneId, -1, -1, enableCollision)
p.setCollisionFilterPair(droneId, cubeId, -1, -1, enableCollision)
p.setCollisionFilterPair(groundId, cubeId, -1, -1, enableCollision)


p.setRealTimeSimulation(1)
p.setGravity(0, 0, -10)



while (p.isConnected()):
  time.sleep(1. / 240.)
  contactPoints = p.getContactPoints(groundId, droneId, -1, -1)
  contactPoints2 = p.getContactPoints(droneId, cubeId, -1, -1)
  print(contactPoints, contactPoints2)

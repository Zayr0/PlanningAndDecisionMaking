import math
import os
import numpy as np
import time
from Lidar import Lidar
import pybullet as p
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

#droneId should be 0 and plane should be 1 based on initialization order
droneId = p.loadURDF("r2d2.urdf", [0, 0, 3],  useMaximalCoordinates=False)
groundId = p.loadURDF("plane.urdf", useMaximalCoordinates=False)

p.setRealTimeSimulation(1)

lidar = Lidar('3D', 10)
lidar.showRays = True

while (p.isConnected()):
  time.sleep(1. / 240.)
  pos, orn = p.getBasePositionAndOrientation(droneId)
  results = lidar.scan(pos, p.getEulerFromQuaternion(orn))







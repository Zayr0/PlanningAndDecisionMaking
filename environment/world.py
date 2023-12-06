import pybullet as p
import pybullet_data
import time
import numpy as np


def build_world():
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    planeId = p.loadURDF("plane.urdf")
    startPos = [0, 0, 3]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])

    sphere1 = p.loadURDF("sphere2.urdf", [1, 0, 0.5], startOrientation)
    sphere2 = p.loadURDF("sphere_small.urdf", [0, 1, 1], startOrientation)
    sphere3 = p.loadURDF("sphere_small.urdf", [0, 0, 0.5], startOrientation)
    obstacles = [sphere1, sphere2, sphere3]

    return obstacles

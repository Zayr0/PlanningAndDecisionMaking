import pybullet as p
from environment.world import build_world
import time
import numpy as np

# connect
p.connect(p.GUI)
p.setGravity(0, 0, 0)

# build the environment by loading obstacle .urdfs and obtaining their IDs
obstacles = build_world()

# load the drone
drone = p.loadURDF("aliengo/aliengo.urdf", [1, 1, 1], p.getQuaternionFromEuler([0, 0, 0]))


for i in range(1000):
    new_location = [2*np.cos(i/10), 2*np.sin(i/10), 3*np.cos(i/100)**2] # this would be calculated with our state dynamics
    p.resetBasePositionAndOrientation(drone, new_location, p.getQuaternionFromEuler([0, 0, 0]))
    p.stepSimulation()
    time.sleep(0.02)

# disconnect
p.disconnect()
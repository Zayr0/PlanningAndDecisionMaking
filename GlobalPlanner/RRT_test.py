import pybullet as p
from Environment.Environment import Environment
from Modelling.drone_dynamics import Quadrotor
from Modelling.trajectory_generation import *
import time
import numpy as np
from RRT import RRT

np.random.seed(10)

# connect
p.connect(p.GUI)
p.setGravity(0, 0, 0)
N = 1000  # number of simulation steps

# build the Environment by loading obstacle .urdfs and obtaining their IDs
env = Environment()

# load the drone and specify its dynamics
droneID = p.loadURDF("sphere2.urdf", [1, 1, 1], p.getQuaternionFromEuler([0, 0, 0]))
drone = Quadrotor()

maxIter = 200
node_sets = np.zeros((maxIter, 4))  # contain x y z and parent
goal = [5, 5, 0.5]
start = list(p.getBasePositionAndOrientation(droneID)[0])
node_sets[0] = np.array(start + [-1])

p.removeBody(droneID)
rrt = RRT(x_range=(-5, 5), y_range=(-5, 5), z_range=(0, 1), expandDis=1.0, goalSampleRate=10, maxIter=maxIter, droneID=droneID)

# path, path_distance = rrt.rrt_planning(start, goal)
# path, path_distance = rrt.rrt_star_planning(start, goal)
path, path_distance = rrt.informed_rrt_star_planning(start, goal)

# disconnect
p.disconnect()
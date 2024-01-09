import pybullet as p
from Environment.Environment import Environment
from Modelling.drone_dynamics import Quadrotor
from Modelling.trajectory_generation import *
import time
import numpy as np
from RRT import RRT

np.random.seed(0)

# connect
p.connect(p.GUI)
p.setGravity(0, 0, 0)
N = 1000  # number of simulation steps

# build the Environment by loading obstacle .urdfs and obtaining their IDs
env = Environment()

# load the drone and specify its dynamics
droneID = p.loadURDF("sphere2.urdf", [1, 1, 1], p.getQuaternionFromEuler([0, 0, 0]))
drone = Quadrotor()
x_bag, u_bag = drone.get_ss_bag_vectors(N)  # arrays to bag the historical data of the states and inputs
x_ref = test_traj_wp(N)

x0 = np.array([1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0])
u0 = np.array([0, 0, 0, 0])
x_bag[:, 0] = x0
u_bag[:, 0] = u0


maxIter = 200
node_sets = np.zeros((maxIter, 4))  # contain x y z and parent
goal = [5, -5, 0.5]
start = list(p.getBasePositionAndOrientation(droneID)[0])
node_sets[0] = np.array(start + [-1])

rrt = RRT(x_range=(-5, 5), y_range=(-5, 5), z_range=(0, 1), expandDis=1.0, goalSampleRate=10, maxIter=maxIter, droneID=droneID)
path, path_distance = rrt.rrt_planning(start, goal)


for i in range(maxIter):

    time.sleep(0.02)

# disconnect
p.disconnect()
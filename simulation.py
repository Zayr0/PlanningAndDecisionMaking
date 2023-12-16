import pybullet as p
#from environment.world import build_world
from environment.world import Environment
from Modelling.drone_dynamics import Quadrotor
from Modelling.trajectory_generation import *
import time
import numpy as np

# connect
p.connect(p.GUI)
p.setGravity(0, 0, -10)
N = 1000  # number of simulation steps

# build the environment by loading obstacle .urdfs and obtaining their IDs
env = Environment()

# load the drone and specify its dynamics
droneID = p.loadURDF("sphere2.urdf", [1, 1, 1], p.getQuaternionFromEuler([0, 0, 0]))
drone = Quadrotor()
x_bag, u_bag = drone.get_ss_bag_vectors(N)  # arrays to bag the historical data of the states and inputs
x_ref = test_traj_square(N)

x0 = np.array([1, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0])
u0 = np.array([0, 0, 0, 0])
x_bag[:, 0] = x0
u_bag[:, 0] = u0

# drone_pos = x_bag[:3, 0]
# drone_att = x_bag[3:6, 0] * np.array([-1, 1, 1])
# drone_att_quaternion = p.getQuaternionFromEuler(drone_att)
# p.resetBasePositionAndOrientation(droneID, drone_pos, drone_att_quaternion)
for k in range(N - 1):
# while True:
    drone_pos = x_bag[:3, k]
    drone_att = x_bag[3:6, k] * np.array([-1, 1, 1])
    drone_att_quaternion = p.getQuaternionFromEuler(drone_att)

    p.resetBasePositionAndOrientation(droneID, drone_pos, drone_att_quaternion)
    p.stepSimulation()

    x_bag[:, k+1] = drone.step(x_bag[:, k], x_ref[:, k])
    time.sleep(0.02)

# disconnect
p.disconnect()

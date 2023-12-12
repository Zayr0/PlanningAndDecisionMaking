import pybullet as p
#from environment.world import build_world
from environment.world import Environment
from Modelling.drone_dynamics import Quadrotor
from Modelling.trajectory_generation import *
import time
import numpy as np
from RRT.RRT import RRT
import matplotlib.pyplot as plt


# connect
p.connect(p.GUI)
p.setGravity(0, 0, -10)
N = 300  # number of simulation steps

# build the environment by loading obstacle .urdfs and obtaining their IDs
env = Environment()
start = [0, -10, 5]


# load the drone and specify its dynamics
droneID = p.loadURDF("sphere2.urdf", start, p.getQuaternionFromEuler([0, 0, 0]))
drone = Quadrotor()

maxIter = 200
node_sets = np.zeros((maxIter, 4))  # contain x y z and parent
goal = [3, 10, 2]
start = list(p.getBasePositionAndOrientation(droneID)[0])
node_sets[0] = np.array(start + [-1])

rrt = RRT(x_range=(-env.width/2, env.width/2), y_range=(-env.depth/2, env.depth/2), z_range=(0, env.height), expandDis=1.0, goalSampleRate=10, maxIter=maxIter, droneID=droneID)
path, path_distance = rrt.rrt_planning(start, goal)
print(path)
x_bag, u_bag = drone.get_ss_bag_vectors(N)  # arrays to bag the historical data of the states and inputs
x_ref = test_traj_wps(N, np.array(path))


x0 = np.array([0, -10, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0])
u0 = np.array([0, 0, 0, 0])
x_bag[:, 0] = x0
u_bag[:, 0] = u0

pov = True
for k in range(N - 1):
    drone_pos = x_bag[:3, k]
    drone_att = x_bag[3:6, k] * np.array([-1, 1, 1])
    drone_att_quaternion = p.getQuaternionFromEuler(drone_att)

    p.resetBasePositionAndOrientation(droneID, drone_pos, drone_att_quaternion)
    p.stepSimulation()

    x_bag[:, k+1] = drone.step(x_bag[:, k], x_ref[:, k])
    time.sleep(0.02)

    if pov:
        camera_target_position = x_bag[:3, k]
        camera_distance = 3
        camera_yaw = x_bag[4, k]
        camera_pitch = x_bag[3,k]
        p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                                     cameraYaw=camera_yaw,
                                     cameraPitch=camera_pitch,
                                     cameraTargetPosition=camera_target_position)

# disconnect
p.disconnect()

import pybullet as p
#from Environment.world import build_world
from Environment.Environment import Environment
from Environment.Obstacle import Obstacle
from Modelling.drone_dynamics import Quadrotor
from Modelling.trajectory_generation import *
import time
import numpy as np
from Planning.RRT import RRT
import matplotlib.pyplot as plt


# connect
p.connect(p.GUI)
p.setGravity(0, 0, 0)
N = 300  # number of simulation steps

# build the Environment by loading obstacle .urdfs and obtaining their IDs
env = Environment(numObstacles=100, type="Dynamic", basePosition=[0, 0, 0])
start = [0, -10, 5]


# load the drone and specify its dynamics
droneID = p.loadURDF("Environment/VisualSphere.urdf", start, p.getQuaternionFromEuler([0, 0, 0]))
p.changeVisualShape(droneID, -1, rgbaColor=[0, 1, 0, 1])
drone = Quadrotor()

collisionFilterGroup = 0
collisionFilterMask = 0
enableCollision = 1

p.setCollisionFilterGroupMask(droneID, -1, collisionFilterGroup, collisionFilterMask)

for ob in env.obstacles:
    p.setCollisionFilterGroupMask(ob.ID, -1, collisionFilterGroup, collisionFilterMask)
    p.setCollisionFilterPair(droneID, ob.ID, -1, -1, enableCollision)

maxIter = 200
node_sets = np.zeros((maxIter, 4))  # contain x y z and parent
goal = [3, 10, 2]
start = list(p.getBasePositionAndOrientation(droneID)[0])
node_sets[0] = np.array(start + [-1])

rrt = RRT(x_range=(-env.width/2, env.width/2), y_range=(-env.depth/2, env.depth/2), z_range=(0, env.height), expandDis=1.0, goalSampleRate=10, maxIter=maxIter, droneID=droneID)
path, path_distance = rrt.rrt_planning(start, goal)
print('path:', path)
x_bag, u_bag = drone.get_ss_bag_vectors(N)  # arrays to bag the historical data of the states and inputs
x_ref = test_traj_wps(N, np.array(path))


x0 = np.array([0, -10, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0])
u0 = np.array([0, 0, 0, 0])
x_bag[:, 0] = x0
u_bag[:, 0] = u0

dt = 0.02

pov = False
for k in range(N - 1):
    drone_pos = x_bag[:3, k]
    drone_att = x_bag[3:6, k] * np.array([-1, 1, 1])
    drone_att_quaternion = p.getQuaternionFromEuler(drone_att)

    p.resetBasePositionAndOrientation(droneID, drone_pos, drone_att_quaternion)
    p.stepSimulation()

    x_bag[:, k+1] = drone.step(x_bag[:, k], x_ref[:, k])

    time.sleep(dt)

    for ob in env.obstacles:
        ob.update(dt)
        contactPoints = p.getContactPoints(droneID, ob.ID, -1, -1)
        if len(contactPoints) > 0:
            print('Collision at: ', contactPoints, 'with object:', ob.ID)
            p.changeVisualShape(ob.ID, -1, rgbaColor=[1, 0, 0, 0.9])

    if pov:
        camera_target_position = x_bag[:3, k]
        camera_distance = 10
        camera_yaw = x_bag[4, k]
        camera_pitch = x_bag[3,k]
        p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                                     cameraYaw=camera_yaw,
                                     cameraPitch=camera_pitch,
                                     cameraTargetPosition=camera_target_position)

# disconnect
p.disconnect()

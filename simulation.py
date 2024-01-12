import pybullet as p
from Environment.Environment import Environment
from Environment.Obstacle import Obstacle
from Modelling.drone_dynamics import Quadrotor
from Modelling.trajectory_generation import *
import time
from Helper.Bounds import Bounds
import numpy as np
from Planning.RRT import RRT
from Planning.SafeFlightPolytope import get_sfp
from Helper.HullMaths import *
import matplotlib.pyplot as plt
import pybullet_data


#Setup pybullet simulation variables

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
p.setRealTimeSimulation(1)
p.setGravity(0, 0, 0)
p.resetDebugVisualizerCamera(cameraDistance=20,
                                     cameraYaw=90,
                                     cameraPitch=-30,
                                     cameraTargetPosition=[0, 0, 0])
dt = 0.02
pov = False
sfp = False



# Initialize obstacle environments

staticBounds = Bounds([[-5, 5], [-5, 5], [0, 10]], center=[0, -6, 0])
staticEnv = Environment(type="Static", bounds=staticBounds)

dynamicBounds = Bounds([[-5, 5], [-5, 5], [1, 10]], center=[0, 6, 0])
dynamicEnv = Environment(numObstacles=90, type="Dynamic", bounds=dynamicBounds)


# Define start and goal for the drone

start = [0, -20, 5]
goal = [0, 0, 5]



# Load drone body and specify the dynamics of movement

droneID = p.loadURDF("Environment/VisualSphere.urdf", start, p.getQuaternionFromEuler([0, 0, 0]))
p.changeVisualShape(droneID, -1, rgbaColor=[0, 1, 0, 1])
drone = Quadrotor()


# Setup collisions for moving obstacles

collisionFilterGroup = 0
collisionFilterMask = 0
enableCollision = 1
p.setCollisionFilterGroupMask(droneID, -1, collisionFilterGroup, collisionFilterMask)

for ob in dynamicEnv.obstacles:
    p.setCollisionFilterGroupMask(ob.ID, -1, collisionFilterGroup, collisionFilterMask)
    p.setCollisionFilterPair(droneID, ob.ID, -1, -1, enableCollision)



# Setup for RRT - Generate global path

maxIter = 200
node_sets = np.zeros((maxIter, 4))  # contain x y z and parent
start = list(p.getBasePositionAndOrientation(droneID)[0])
node_sets[0] = np.array(start + [-1])

rrt = RRT(bounds=staticEnv.Bounds, expandDis=1.0, goalSampleRate=10, maxIter=maxIter, droneID=droneID)
path, path_distance = rrt.rrt_planning(start, goal)


# Setup for drone dynamics

N = 300
x_bag, u_bag = drone.get_ss_bag_vectors(N)  # arrays to bag the historical data of the states and inputs
x_ref = test_traj_wps(N, np.array(path))


x0 = np.array([0, -10, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0])
u0 = np.array([0, 0, 0, 0])
x_bag[:, 0] = x0
u_bag[:, 0] = u0


# Start of simulation loop

for k in range(N - 1):
    drone_pos = x_bag[:3, k]
    drone_att = x_bag[3:6, k] * np.array([-1, 1, 1])
    drone_att_quaternion = p.getQuaternionFromEuler(drone_att)

    if sfp and k%10 == 0:
        A_ineq, b_ineq, vertices = get_sfp(drone_pos, env, polytope_vertices=True)
        print(vertices)
        #p.addUserDebugPoints([entry.tolist() for entry in vertices], [[1, 1, 1] for _ in vertices], 30)
        sfp_id = draw_polytope2(vertices)
        #time.sleep(0.5)
        #p.removeBody(sfp_id)

    p.resetBasePositionAndOrientation(droneID, drone_pos, drone_att_quaternion)
    p.stepSimulation()

    x_bag[:, k+1] = drone.step(x_bag[:, k], x_ref[:, k])

    time.sleep(dt)

    for ob in dynamicEnv.obstacles:
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

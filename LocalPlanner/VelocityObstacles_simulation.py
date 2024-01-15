from Environment.Environment import Environment
from Modelling.drone_dynamics import Quadrotor
from GlobalPlanner.SafeFlightPolytope import *
from Helper.HullMaths import *
import pybullet_data
from Sampling.Sampler import Sampler
from VelocityObstacles.VelocityObstacles import VelocityObstacles


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



# Initialize obstacle environments
size = 5
dynamicBounds = Bounds([[-size, size], [-size, size], [0, 0]], center=[0, 0, 5])
dynamicEnv = Environment(numObstacles=1, type="Dynamic2D", bounds=dynamicBounds)


# Define start and goal for the drone

start = [0, -6, 5]
goal = [0, 6, 5]

p.addUserDebugLine(start, goal, lineColorRGB=[1, 1, 0], lineWidth=2)



# Load drone body and specify the dynamics of movement

droneID = p.loadURDF("VelocityObstacles/VisualSphere.urdf", start, p.getQuaternionFromEuler([0, 0, 0]))
p.changeVisualShape(droneID, -1, rgbaColor=[0, 1, 0, 1])
drone = Quadrotor()
droneRadius = 1.0


# Setup collisions for moving obstacles

collisionFilterGroup = 0
collisionFilterMask = 0
enableCollision = 1
p.setCollisionFilterGroupMask(droneID, -1, collisionFilterGroup, collisionFilterMask)

for ob in dynamicEnv.obstacles:
    p.setCollisionFilterGroupMask(ob.ID, -1, collisionFilterGroup, collisionFilterMask)
    p.setCollisionFilterPair(droneID, ob.ID, -1, -1, enableCollision)

N = 100000
x_bag, u_bag = drone.get_ss_bag_vectors(N)  # arrays to bag the historical data of the states and inputs
x_ref = np.vstack((np.linspace(np.array(start), np.array(goal), N, axis=0).T, np.zeros((9, N))))

x0 = np.array([start[0], start[1], start[2], 0, 0, 0, 0, 0, 0, 0, 0, 0])
u0 = np.array([0, 0, 0, 0])
x_bag[:, 0] = x0
u_bag[:, 0] = u0

tau = 10.0

sampler = Sampler()
VO = VelocityObstacles(tau, dt)


# Start of simulation loop
desVel = np.zeros((3, N))

for k in range(N - 1):
    drone_pos = x_bag[:3, k]
    drone_att = x_bag[3:6, k] * np.array([-1, 1, 1])
    drone_att_quaternion = p.getQuaternionFromEuler(drone_att)
    #p.resetBasePositionAndOrientation(droneID, drone_pos, drone_att_quaternion)


    p.stepSimulation()
    x_bag[:, k+1] = drone.step(x_bag[:, k], x_ref[:, k])
    desVel[:, k] = x_bag[6:9, k+1]

    pos, orn = p.getBasePositionAndOrientation(droneID)
    VO.detInputBySampling2D(desVel[:, k], pos, dynamicEnv.obstacles, dynamicEnv.Bounds)


    time.sleep(dt)

    for ob in dynamicEnv.obstacles:
        #ob.update2D(dt)
        ob.drawVel()
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




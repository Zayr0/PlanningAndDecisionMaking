import pybullet as p
from Environment.Environment import Environment
from Environment.Obstacle import Obstacle
from Modelling.drone_dynamics import Quadrotor
from Modelling.trajectory_generation import *
import time
from Helper.Bounds import Bounds
import numpy as np
from Planning.RRT import RRT
from Planning.SafeFlightPolytope import *
from Helper.HullMaths import *
import matplotlib.pyplot as plt
import pybullet_data
from Sampling.Sampler import Sampler


def run(auto_testing=False, settings=None):
    """results variables"""
    rslt_time_stamp_1 = None
    rslt_time_stamp_2 = None
    rslt_gp_path_length = None
    rslt_dynamic_success = None
    rslt_static_success = None
    results = {}
    pov = False
    sfp = True
    env_static = True
    static_active = False
    env_bugtrap = True
    env_dynamic = True
    dynamic_active = True
    planner = "rrt"
    # env_static = settings["env_static"]
    # static_active = settings["env_static"]
    # env_dynamic = settings["env_dynamic"]
    # dynamic_active = settings["env_dynamic"]
    dynamic_controller = "LQR"
    planner = "rrt"
    #Setup pybullet simulation variables

    if auto_testing:
        p.connect(p.DIRECT)
    else:
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
    # Define start and goal for the drone
    start = [0, -13, 5]
    goal = [0, 0, 5]
    # Load drone body and specify the dynamics of movement
    droneID = p.loadURDF("Environment/VisualSphere.urdf", start, p.getQuaternionFromEuler([0, 0, 0]))
    p.changeVisualShape(droneID, -1, rgbaColor=[0, 1, 0, 1])
    drone = Quadrotor()
    droneRadius = 1.0

    # Initialize obstacle environments
    if env_static:
        staticBounds = Bounds([[-5, 5], [-5, 5], [-5, 5]], center=[0, -6, 5])
        staticEnv = Environment(type="Static", bounds=staticBounds)


    if env_dynamic:
        dynamicBounds = Bounds([[-5, 5], [-5, 5], [4, 6]], center=[0, 6, 0])
        dynamicEnv = Environment(numObstacles=3, type="Dynamic", bounds=dynamicBounds)

        # Setup collisions for moving obstacles
        collisionFilterGroup = 0
        collisionFilterMask = 0
        enableCollision = 1
        p.setCollisionFilterGroupMask(droneID, -1, collisionFilterGroup, collisionFilterMask)

        for ob in dynamicEnv.obstacles:
            p.setCollisionFilterGroupMask(ob.ID, -1, collisionFilterGroup, collisionFilterMask)
            p.setCollisionFilterPair(droneID, ob.ID, -1, -1, enableCollision)

    # Start of static simulation loop
    if static_active:
        # Setup for RRT - Generate global path
        maxIter = 200
        rslt_time_stamp_1 = time.perf_counter()
        rrt = RRT(bounds=staticEnv.Bounds, expandDis=1.0, goalSampleRate=10, maxIter=maxIter, droneID=droneID)
        if planner=="rrt":
            path, path_distance = rrt.rrt_planning(start, goal)
        elif planner == "rrt_star":
            path, path_distance = rrt.rrt_star_planning(start, goal)
        elif planner == "rrt_star_informed":
            path, path_distance = rrt.informed_rrt_star_planning(start, goal)
        rslt_time_stamp_2 = time.perf_counter()
        rslt_gp_path_length = path_distance
        # Setup for drone dynamics
        N = 300
        x_bag, u_bag = drone.get_ss_bag_vectors(N)  # arrays to bag the historical data of the states and inputs
        x_ref = min_snap(N, np.array(path))  # test_traj_wps(N, np.array(path)) #
        colors = [[1, 1, 0] for _ in range(x_ref.shape[1])]
        p.addUserDebugPoints([x_ref[:3, i] for i in range(x_ref.shape[1])], colors, pointSize=3)

        x0 = np.array([start[0], start[1], start[2], 0, 0, 0, 0, 0, 0, 0, 0, 0])
        u0 = np.array([0, 0, 0, 0])
        x_bag[:, 0] = x0
        u_bag[:, 0] = u0

        info_dict = {"deltaB": None}  # dictionary for further information required by MPC

        for k in range(N - 1):
            drone_pos = x_bag[:3, k]
            drone_att = x_bag[3:6, k] * np.array([-1, 1, 1])
            drone_att_quaternion = p.getQuaternionFromEuler(drone_att)
            p.resetBasePositionAndOrientation(droneID, drone_pos, drone_att_quaternion)

            prox_radius = 10.0

            if sfp and k%20==0:# and (np.linalg.norm(p_r - np.asarray(drone_pos)) <  droneRadius):
                #A_ineq, b_ineq, vertices = get_sfp(drone_pos, staticEnv, polytope_vertices=True)
                A_ineq, b_ineq, vertices = get_sfp(drone_pos, staticEnv, polytope_vertices=True, proximity_radius=prox_radius)
                info_dict["A"] = A_ineq
                info_dict["b"] = b_ineq
                sfp_id = draw_polytope2(vertices)


            p.stepSimulation()
            x_bag[:, k+1] = drone.step(x_bag[:, k], x_ref[:, k], cont_type="MPC", info_dict=info_dict)
            time.sleep(dt)

            if pov:
                camera_target_position = x_bag[:3, k]
                camera_distance = 10
                camera_yaw = x_bag[4, k]
                camera_pitch = x_bag[3,k]
                p.resetDebugVisualizerCamera(cameraDistance=camera_distance,
                                             cameraYaw=camera_yaw,
                                             cameraPitch=camera_pitch,
                                             cameraTargetPosition=camera_target_position)

    if dynamic_active:
        sampler = Sampler()
        #define new start and goal
        start = [0, 0, 5]
        goal = [0, 13, 5]

        # # Setup for RRT - Generate global path
        # maxIter = 200
        # rrt = RRT(bounds=dynamicEnv.Bounds, expandDis=1.0, goalSampleRate=10, maxIter=maxIter, droneID=droneID)
        # path, path_distance = rrt.rrt_planning(start, goal)

        # Setup for drone dynamics
        N = 300
        x_bag, u_bag = drone.get_ss_bag_vectors(N)  # arrays to bag the historical data of the states and inputs
        x_ref = linear_traj(N, [start, goal])
        #x_ref = min_snap(N, np.array(path))  # test_traj_wps(N, np.array(path)) #
        colors = [[1, 1, 0] for _ in range(x_ref.shape[1])]
        p.addUserDebugPoints([x_ref[:3, i] for i in range(x_ref.shape[1])], colors, pointSize=3)

        x0 = np.array([start[0], start[1], start[2], 0, 0, 0, 0, 0, 0, 0, 0, 0])
        u0 = np.array([0, 0, 0, 0])
        x_bag[:, 0] = x0
        u_bag[:, 0] = u0

        info_dict = {}  # dictionary for further information required by MPC
        sfp_id = None
        rslt_time_stamp_3 = time.perf_counter()
        for k in range(N - 1):
            drone_pos = x_bag[:3, k]
            drone_att = x_bag[3:6, k] * np.array([-1, 1, 1])
            drone_att_quaternion = p.getQuaternionFromEuler(drone_att)
            p.resetBasePositionAndOrientation(droneID, drone_pos, drone_att_quaternion)

            prox_radius = 100.0

            # if sfp and k%1==0:# and (np.linalg.norm(p_r - np.asarray(drone_pos)) <  droneRadius):
            #     #A_ineq, b_ineq, vertices = get_sfp(drone_pos, staticEnv, polytope_vertices=True)
            #     A_ineq, b_ineq, vertices = get_sfp(drone_pos, dynamicEnv, polytope_vertices=True, proximity_radius=prox_radius)
            #     info_dict["A"] = A_ineq
            #     info_dict["b"] = b_ineq
            #     if sfp_id:
            #         p.removeBody(sfp_id)
            #     sfp_id = draw_polytope2(vertices)
            #
            #     A_ineq, b_ineq = get_sfp(drone_pos, dynamicEnv, polytope_vertices=False, proximity_radius=prox_radius)
            #     deltaB = calculateDeltaB(A_ineq, dynamicEnv.obstacles, dt)
            #     info_dict["deltaB"] = deltaB

            p.stepSimulation()
            go_allowed = True
            for obs in dynamicEnv.obstacles:
                if obs.min_dist(drone_pos, get_closest_point=False) <= 1:
                    go_allowed = False
            if go_allowed:
                x_bag[:, k + 1] = drone.step(x_bag[:, k], x_ref[:,-1], cont_type=dynamic_controller, info_dict=info_dict)
            else:
                dummy_ref = x_ref[:,-1].copy()
                dummy_ref[:3] = drone_pos
                x_bag[:, k + 1] = drone.step(x_bag[:, k], dummy_ref, cont_type=dynamic_controller, info_dict=info_dict)

            if np.linalg.norm(x_bag[:3, k + 1] - np.array(goal)) <= 1:
                print("Goal was reached successfully")
                if auto_testing:
                    p.disconnect()
                    rslt_time_stamp_4 = time.perf_counter()
                    results = {}
                    results["rslt_dynamic_success"] = 1
                    results["rslt_dynamic_time"] = rslt_time_stamp_4 - rslt_time_stamp_3
                    return results

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

    # result_gp_solver_time = rslt_time_stamp_2 - rslt_time_stamp_1
    # results["result_gp_solver_time"] = result_gp_solver_time
    # result_gp_path_length = rslt_gp_path_length
    # results["result_gp_path_length"] = result_gp_path_length


    return results
#
# settings = {}
# settings["planner"] = "rrt"
# settings["env_static"] = True
# settings["env_dynamic"] = False
# settings["dynamic_controller"] ="MPC"
#run(auto_testing=False)
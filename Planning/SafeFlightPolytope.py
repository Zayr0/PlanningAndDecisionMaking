import numpy as np
from pypoman import compute_polytope_vertices
from Helper.Bounds import Bounds
import pybullet as p
import cvxpy as cp


def get_sfp(drone_pos, env, polytope_vertices=False, proximity_radius=None):
    root = drone_pos
    closest_points = []
    A_ineq = np.array([[0, 0, -1],
                       [0, 0, 1],
                       [0, -1, 0],
                       [0, 1, 0],
                       [-1, 0, 0],
                       [1, 0, 0]])
    slack = 3
    b_ineq = np.array([[-env.Bounds.zMin - env.Bounds.center[2] + slack],
                       [env.Bounds.zMax + env.Bounds.center[2] + slack],
                       [- env.Bounds.yMin - env.Bounds.center[1] + slack],
                       [env.Bounds.yMax + env.Bounds.center[1] + slack],
                       [-env.Bounds.xMin - env.Bounds.center[0] + slack],
                       [env.Bounds.xMax + env.Bounds.center[0] + slack]])
    obs_near = []
    if proximity_radius != None:
        if env.type == "Static":
            obs_near = [obs for obs in env.obstacles if np.linalg.norm(obs.center - drone_pos)<=proximity_radius]
        elif env.type == "Dynamic":
            obs_near = [obs for obs in env.obstacles if np.linalg.norm(obs.p - drone_pos) <= proximity_radius]
    else:
        obs_near = env.obstacles
    for obs in obs_near:
        zi = obs.min_dist(root, get_closest_point=True)
        vector = zi - root
        A = vector
        b = np.dot(vector, zi)
        A_ineq = np.vstack((A_ineq, A))
        b_ineq = np.vstack((b_ineq, b))
    if polytope_vertices:
        vertices = compute_polytope_vertices(A_ineq, b_ineq)
        return A_ineq, b_ineq, vertices
    else:
        return A_ineq, b_ineq




#This function is suppose to draw the distance from the drone to the next RRT node.
#Its also suppose to calculate the last point within the object proximity scan radius,
# before recalculation is needed.
def recalculation_point(rrt_path, drone_pos, droneRadius, k, N, vertices, prox_radius, pos_last_calc):
    n_links = np.array(rrt_path).shape[0]
    nextPathPoint = np.asarray(rrt_path[int(n_links * k / N) + 1])
    dist2NextNode = nextPathPoint - np.asarray(drone_pos)
    p.addUserDebugLine(drone_pos, np.asarray(drone_pos) + dist2NextNode, [1, 0, 0], 4, replaceItemUniqueId=1)

    bool = False
    # if len(vertices) > 0:
        #This checks if the next goal point is in the polytope
        # bool = sampler.isInside(np.asarray(vertices), nextPathPoint)

    # Distance to last point in proximity radius
    recalPoint = pos_last_calc + dist2NextNode * (prox_radius-droneRadius)/prox_radius

    dist2recalculate = np.linalg.norm(recalPoint - drone_pos)


#This function is suppose to calculate the last point in the
# convex safe flight polytope before recalculation is needed.
def recalculation_point2(drone_pos, drone_radius, A_ineq, B_ineq):
    zi = cp.Variable(3)
    zj = cp.Variable(3)
    epsilon = 0.1

    cost = cp.quad_form(zi - zj - drone_radius - epsilon, np.eye(3))
    constraints = []
    constraints += [(A_ineq @ zi).reshape((6,1)) <= B_ineq]
    constraints += [zj == drone_pos]

    problem = cp.Problem(cp.Minimize(cost), constraints)
    result = problem.solve(solver=cp.OSQP, verbose=True)

    return zi.value


def calculateDeltaB(A_ineq, obstacles, dt):
    tNormal = -A_ineq

    deltaB = []
    for i, ob in enumerate(obstacles):
        deltaB.append(dt * np.dot(ob.v, A_ineq[i, :]))

    deltaB = np.vstack([np.zeros((6, 1)), np.array(deltaB).reshape((-1,1))])
    return deltaB
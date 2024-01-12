import numpy as np
from pypoman import compute_polytope_vertices
from Helper.Bounds import Bounds



def get_sfp(drone_pos, env, polytope_vertices=False):
    root = drone_pos
    closest_points = []
    A_ineq = np.array([[0, 0, -1],
                       [0, 0, 1],
                       [0, -1, 0],
                       [0, 1, 0],
                       [-1, 0, 0],
                       [1, 0, 0]])
    slack = 1
    b_ineq = np.array([[env.Bounds.zMin + env.Bounds.center[2]-slack], [env.Bounds.zMax + env.Bounds.center[2] + slack], [env.Bounds.xMax + env.Bounds.center[0] + slack], [env.Bounds.xMax + env.Bounds.center[0] + slack], [env.Bounds.yMax + env.Bounds.center[1] + slack], [env.Bounds.yMax + env.Bounds.center[1] + slack]])

    for obs in env.obstacles:
        zi = obs.min_dist(root, get_closest_point=True)
        vector = zi - root
        A = vector
        b = np.dot(vector, zi)
        A_ineq = np.vstack((A_ineq, A))
        b_ineq = np.vstack((b_ineq, b))
        print("Closest point", zi)
    if polytope_vertices:
        vertices = compute_polytope_vertices(A_ineq, b_ineq)
        return A_ineq, b_ineq, vertices
    else:
        return A_ineq, b_ineq


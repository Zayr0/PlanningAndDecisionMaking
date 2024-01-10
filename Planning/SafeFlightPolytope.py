import numpy as np
from pypoman import compute_polytope_vertices


def get_sfp(drone_pos, env, polytope_vertices=False):
    root = drone_pos
    closest_points = []
    A_ineq = np.empty((0, 3))
    b_ineq = np.empty((0, 1))
    for obs in env.obstacles:
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


import cvxpy as cp
import numpy as np


class Obstacle:
    def __init__(self, p_id):
        self.p_id = p_id
        self.center = 0
        self.vertices = []
        self.constr_mat_A = 0
        self.constr_mat_b = 0

    def min_dist(self, zj_val, get_closest_point=False):
        """this formulates the QP optimization to obtain the shortest distance between
        the polytope i of this obstacle and a point j"""
        zi = cp.Variable(3)
        zj = cp.Variable(3)  # this could also be put as a constant, but in case we use a bounding box for the drone later, this must be a decision variable too

        cost = cp.quad_form(zi - zj, np.eye(3))
        constraints = []
        constraints += [self.constr_mat_A @ zi <= self.constr_mat_b]
        constraints += [zj == zj_val]

        problem = cp.Problem(cp.Minimize(cost), constraints)
        result = problem.solve(solver=cp.OSQP, verbose=False)
        if get_closest_point:
            return zi.value
        return float(result)